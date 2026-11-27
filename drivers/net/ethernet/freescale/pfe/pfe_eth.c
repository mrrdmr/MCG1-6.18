// SPDX-License-Identifier: GPL-2.0
/*
 * PFE Stage P7-P8: net_device/MDIO/PHY bring-up (P7) and a working
 * traffic path (P8) for GEM0. Ported from the 3.2.26 vendor tree's
 * pfe_ctrl/pfe_eth.c (kmodules/mspd-c2k/pfe/ in symops/MCG1-3.2.26,
 * 2846 lines) -- almost none of that file's size is ported here; see
 * pfe_eth.h's banner for what was structurally dropped.
 *
 * Stage P7 confirmed on real hardware: PHY address 0 (Broadcom
 * BCM54612E), MDIO bus and PHY connect both working (see
 * Documentation/arm/ls1024a-wdmycloud.rst). Getting there needed one
 * fix not obvious from the vendor source: each GEM has its own
 * "extphy" reference clock (LS1024A_CLK_EXTPHY0/1/2), separate from
 * the pfe node's shared "gemtx", that this port never requested --
 * without it, the kernel's own "disabling unused clocks" late-boot
 * step gates it off a couple seconds in, and every MDIO transaction
 * after that silently returns stale data instead of erroring (see
 * pfe_eth_probe_gem0()).
 *
 * Stage P8 adds the actual Tx/Rx path:
 *
 *  - pfe_eth_send_packet() (.ndo_start_xmit) submits via
 *    hif_lib_xmit_pkt_sg() (Stage P8, pfe_hif_lib.c) with scatter-gather
 *    support, BQL accounting, and stop/wake flow control
 *    (PFE_TX_STOP_THRES / PFE_TX_WAKE_THRES).
 *  - TSO/TSO6 is segmented by the Linux 6.18 network stack prior to
 *    ndo_start_xmit() -- the firmware's own hardware TSO submission
 *    path (struct hif_tso_hdr) isn't wired up yet.
 *  - pfe_eth_rx_drain() consumes what hif_lib_client_register() (Stage
 *    P7) made this GEM reachable for, called directly from
 *    pfe_eth_event_handler() -- itself called synchronously from
 *    hif_lib_indicate_client() (Stage P5), which already runs inside
 *    the shared HIF-level NAPI poll (pfe_hif.c's struct pfe_hif.napi).
 *    No second NAPI layer needed: unlike the vendor's one-NAPI-triple-
 *    per-GEM design, this port centralized Rx polling at the HIF level
 *    back in Stage P5 (a single physical Rx DMA ring shared by every
 *    client, demultiplexed by client id), and the drain runs to
 *    completion inline, in the same softirq context the outer HIF poll
 *    is already in -- there's nothing to schedule.
 *  - No multi-descriptor (jumbo/LRO) Rx reassembly: PFE_PKT_SIZE (1544)
 *    covers any standard MTU + headers in one HIF descriptor, so this
 *    should never actually come up; pfe_eth_rx_drain() drops
 *    defensively rather than mishandle it if it ever does.
 */

#include <linux/bitops.h>
#include <linux/brcmphy.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/ethtool.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mii.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "pfe_eth.h"
#include "pfe_hw_lib.h"
#include "pfe_mod.h"

#define PFE_GEM0_ID		0
#define HIF_GEMAC_TMUQ_BASE	6

struct pfe_mdio_priv {
	void __iomem *EMAC_baseaddr;
	int mdc_div;
};

static struct pfe_mdio_priv pfe_mdio_priv;
static struct mii_bus *pfe_mii_bus;
static struct net_device *pfe_gem0_netdev;

static int pfe_mdio_wait_idle(void __iomem *emac_base)
{
	u32 val;

	return readl_poll_timeout(emac_base + EMAC_NETWORK_STATUS,
				  val, val & EMAC_PHY_IDLE,
				  10, EMAC_MDIO_TIMEOUT * 10);
}

static int pfe_eth_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct pfe_mdio_priv *mpriv = bus->priv;
	u32 write_data;

	if (pfe_mdio_wait_idle(mpriv->EMAC_baseaddr))
		return -ETIMEDOUT;

	write_data = 0x60020000 | ((mii_id & 0x1F) << 23) | ((regnum & 0x1F) << 18);
	writel(write_data, mpriv->EMAC_baseaddr + EMAC_PHY_MANAGEMENT);

	if (pfe_mdio_wait_idle(mpriv->EMAC_baseaddr))
		return -ETIMEDOUT;

	return readl(mpriv->EMAC_baseaddr + EMAC_PHY_MANAGEMENT) & 0xFFFF;
}

static int pfe_eth_mdio_write(struct mii_bus *bus, int mii_id, int regnum, u16 value)
{
	struct pfe_mdio_priv *mpriv = bus->priv;
	u32 write_data;

	if (pfe_mdio_wait_idle(mpriv->EMAC_baseaddr))
		return -ETIMEDOUT;

	write_data = 0x50020000 | ((mii_id & 0x1F) << 23) | ((regnum & 0x1F) << 18) | value;
	writel(write_data, mpriv->EMAC_baseaddr + EMAC_PHY_MANAGEMENT);

	return pfe_mdio_wait_idle(mpriv->EMAC_baseaddr);
}

static int pfe_eth_mdio_reset(struct mii_bus *bus)
{
	struct pfe_mdio_priv *mpriv = bus->priv;

	gemac_set_mdc_div(mpriv->EMAC_baseaddr, mpriv->mdc_div);

	writel(readl(mpriv->EMAC_baseaddr + EMAC_NETWORK_CONTROL) | EMAC_MDIO_EN,
	       mpriv->EMAC_baseaddr + EMAC_NETWORK_CONTROL);

	return pfe_mdio_wait_idle(mpriv->EMAC_baseaddr);
}

static int pfe_eth_mdio_init(struct pfe *pfe, struct device_node *np)
{
	struct mii_bus *bus;
	int rc, addr;

	pfe_mdio_priv.EMAC_baseaddr = EMAC1_BASE_ADDR;
	pfe_mdio_priv.mdc_div = 96;

	bus = mdiobus_alloc();
	if (!bus)
		return -ENOMEM;

	bus->name = "ls1024a-pfe-mdio";
	bus->read = pfe_eth_mdio_read;
	bus->write = pfe_eth_mdio_write;
	bus->reset = pfe_eth_mdio_reset;
	bus->priv = &pfe_mdio_priv;
	bus->phy_mask = 0;
	bus->parent = pfe->dev;
	snprintf(bus->id, MII_BUS_ID_SIZE, "ls1024a-pfe-mdio");

	rc = of_mdiobus_register(bus, np);
	if (rc) {
		dev_err(pfe->dev, "of_mdiobus_register() failed: %d\n", rc);
		mdiobus_free(bus);
		return rc;
	}

	for (addr = 0; addr < PHY_MAX_ADDR; addr++) {
		int id1 = mdiobus_read(bus, addr, MII_PHYSID1);
		int id2 = mdiobus_read(bus, addr, MII_PHYSID2);

		if (id1 < 0 || id2 < 0)
			continue;
		if ((id1 == 0xffff && id2 == 0xffff) || (id1 == 0 && id2 == 0))
			continue;

		dev_info(pfe->dev, "mdio: PHY responding at address %d (id %04x:%04x)\n",
			 addr, id1, id2);
	}

	pfe_mii_bus = bus;

	return 0;
}

static void pfe_eth_mdio_exit(void)
{
	if (!pfe_mii_bus)
		return;

	mdiobus_unregister(pfe_mii_bus);
	mdiobus_free(pfe_mii_bus);
	pfe_mii_bus = NULL;
}

static void pfe_gemac_init(struct pfe_eth_priv_s *priv)
{
	GEMAC_CFG cfg;

	switch (priv->phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		cfg.mode = RGMII;
		break;
	case PHY_INTERFACE_MODE_GMII:
		cfg.mode = GMII;
		break;
	case PHY_INTERFACE_MODE_RMII:
		cfg.mode = RMII;
		break;
	case PHY_INTERFACE_MODE_SGMII:
		cfg.mode = SGMII;
		break;
	case PHY_INTERFACE_MODE_MII:
	default:
		cfg.mode = MII;
		break;
	}

	/* Software defaults until adjust_link() corrects them post-autoneg. */
	cfg.speed = SPEED_1000M;
	cfg.duplex = DUPLEX_FULL;

	gemac_set_config(priv->EMAC_baseaddr, &cfg);
	gemac_allow_broadcast(priv->EMAC_baseaddr);
	gemac_disable_unicast(priv->EMAC_baseaddr);
	gemac_disable_multicast(priv->EMAC_baseaddr);
	gemac_disable_fcs_rx(priv->EMAC_baseaddr);
	gemac_enable_1536_rx(priv->EMAC_baseaddr);
	gemac_set_bus_width(priv->EMAC_baseaddr, 64);

	if (priv->dev->features & NETIF_F_RXCSUM)
		gemac_enable_rx_checksum_offload(priv->EMAC_baseaddr);
	else
		gemac_disable_rx_checksum_offload(priv->EMAC_baseaddr);
}

static void pfe_eth_adjust_link(struct net_device *dev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(dev);
	struct phy_device *phydev = priv->phydev;
	unsigned long flags;
	int new_state = 0;

	spin_lock_irqsave(&priv->lock, flags);

	if (phydev->link) {
		if (phydev->duplex != priv->oldduplex) {
			gemac_set_duplex(priv->EMAC_baseaddr, phydev->duplex);
			priv->oldduplex = phydev->duplex;
			new_state = 1;
		}

		if (phydev->speed != priv->oldspeed) {
			MAC_SPEED mac_speed;

			switch (phydev->speed) {
			case SPEED_10:
				mac_speed = SPEED_10M;
				break;
			case SPEED_100:
				mac_speed = SPEED_100M;
				break;
			case SPEED_1000:
			default:
				mac_speed = SPEED_1000M;
				break;
			}
			gemac_set_speed(priv->EMAC_baseaddr, mac_speed);
			priv->oldspeed = phydev->speed;
			new_state = 1;
		}

		if (!priv->oldlink) {
			priv->oldlink = 1;
			new_state = 1;
		}
	} else if (priv->oldlink) {
		priv->oldlink = 0;
		priv->oldspeed = 0;
		priv->oldduplex = -1;
		new_state = 1;
	}

	if (new_state)
		phy_print_status(phydev);

	spin_unlock_irqrestore(&priv->lock, flags);
}

#define PFE_PHY_SHD_RGMII_MODE_VENDOR_VAL	0x08c

/*
 * Restores this PHY's LED-adjacent shadow registers to their vendor-
 * matching values. Must run after pfe_eth_start()/phy_start(), because
 * phy_start() triggers resume -> config_init() which otherwise overwrites
 * LEDS1 and ECR.IM.
 */
static void pfe_phy_restore_led_mode(struct phy_device *phydev)
{
	u16 leds1 = BCM54XX_SHD_LEDS1_LED1(BCM_LED_SRC_LINKSPD1) |
		    BCM54XX_SHD_LEDS1_LED3(BCM_LED_SRC_LINKSPD2);

	phy_write(phydev, MII_BCM54XX_SHD,
		  MII_BCM54XX_SHD_WRITE | MII_BCM54XX_SHD_VAL(BCM54XX_SHD_LEDS1) |
		  MII_BCM54XX_SHD_DATA(leds1));

	phy_write(phydev, MII_BCM54XX_ECR, phy_read(phydev, MII_BCM54XX_ECR) & ~MII_BCM54XX_ECR_IM);

	phy_write(phydev, MII_BCM54XX_SHD,
		  MII_BCM54XX_SHD_WRITE | MII_BCM54XX_SHD_VAL(BCM54XX_SHD_RGMII_MODE) |
		  MII_BCM54XX_SHD_DATA(PFE_PHY_SHD_RGMII_MODE_VENDOR_VAL));
}

static int pfe_phy_init(struct net_device *dev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(dev);
	struct device_node *phy_np;
	struct phy_device *phydev;

	priv->oldlink = 0;
	priv->oldspeed = 0;
	priv->oldduplex = -1;

	phy_np = of_parse_phandle(priv->of_node, "phy-handle", 0);
	if (!phy_np) {
		netdev_err(dev, "phy-handle not found\n");
		return -ENODEV;
	}

	phydev = of_phy_connect(dev, phy_np, pfe_eth_adjust_link,
				 PHY_BRCM_RX_REFCLK_UNUSED, priv->phy_mode);
	of_node_put(phy_np);
	if (!phydev) {
		netdev_err(dev, "of_phy_connect() failed\n");
		return -ENODEV;
	}

	priv->phydev = phydev;
	phy_attached_info(phydev);

	return 0;
}

static void pfe_phy_exit(struct net_device *dev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(dev);

	phy_disconnect(priv->phydev);
	priv->phydev = NULL;
}

static void pfe_eth_start(struct pfe_eth_priv_s *priv)
{
	if (priv->phydev)
		phy_start(priv->phydev);

	hif_lib_tmu_queue_start(&priv->client, priv->low_tmuQ);
	hif_lib_tmu_queue_start(&priv->client, priv->high_tmuQ);

	gpi_enable(priv->GPI_baseaddr);
	gemac_enable(priv->EMAC_baseaddr);
}

static void pfe_eth_stop(struct pfe_eth_priv_s *priv)
{
	gemac_disable(priv->EMAC_baseaddr);
	gpi_disable(priv->GPI_baseaddr);

	hif_lib_tmu_queue_stop(&priv->client, priv->low_tmuQ);
	hif_lib_tmu_queue_stop(&priv->client, priv->high_tmuQ);

	if (priv->phydev)
		phy_stop(priv->phydev);
}

#define PFE_ETH_TXQ		0
#define PFE_ETH_TX_FREE_MAX	16
#define PFE_TX_MIN_HEADLEN	54U
#define PFE_TX_STOP_THRES	64U
#define PFE_TX_WAKE_THRES	128U

/*
 * Reap completed TX skbs and update BQL byte/packet accounting.
 * Must be called with priv->tx_clean_lock held.
 */
static void __pfe_eth_reap_txq(struct pfe_eth_priv_s *priv, int qno, int count)
{
	struct net_device *dev = priv->dev;
	int reap_max = max_t(int, count, PFE_ETH_TX_FREE_MAX);
	unsigned int completed_pkts = 0;
	unsigned int completed_bytes = 0;
	struct sk_buff *skb;
	unsigned int flags;

	lockdep_assert_held(&priv->tx_clean_lock);

	while (count-- > 0) {
		skb = hif_lib_tx_get_next_complete(&priv->client, qno, &flags, reap_max);
		if (!skb)
			break;

		if (flags & HIF_DATA_VALID) {
			completed_pkts++;
			completed_bytes += skb->len;
			dev_consume_skb_any(skb);
		}
	}

	if (completed_pkts)
		netdev_completed_queue(dev, completed_pkts, completed_bytes);
}

/*
 * Wake a stopped netdev queue once available TX slots reach tx_wake_thres.
 * Must be called with priv->tx_clean_lock held.
 */
static void __pfe_eth_maybe_wake_txq(struct pfe_eth_priv_s *priv, int qno)
{
	struct net_device *dev = priv->dev;

	lockdep_assert_held(&priv->tx_clean_lock);

	if (unlikely(!priv->closing && netif_running(dev) && netif_queue_stopped(dev)) &&
	    hif_lib_tx_avail(&priv->client, qno) >= priv->tx_wake_thres) {
		priv->tx_wake_thres = PFE_TX_WAKE_THRES;
		pfe_hif_release_tx_irq(&priv->pfe->hif, priv->client.id);
		priv->sw_stats.tx_queue_wakes++;
		netif_wake_queue(dev);
	}
}

static void __pfe_eth_flush_txq(struct pfe_eth_priv_s *priv, int qno, int count)
{
	__pfe_eth_reap_txq(priv, qno, count);
	__pfe_eth_maybe_wake_txq(priv, qno);
}

static void pfe_eth_flush_txq(struct pfe_eth_priv_s *priv, int qno, int count)
{
	spin_lock_bh(&priv->tx_clean_lock);
	__pfe_eth_flush_txq(priv, qno, count);
	spin_unlock_bh(&priv->tx_clean_lock);
}

/*
 * Drain received frames from the client RX queue into skbs built over
 * page-fragment buffers (build_skb), rejecting multi-descriptor frames and
 * out-of-range lengths/offsets before GRO delivery.
 */
static void pfe_eth_rx_drain(struct pfe_eth_priv_s *priv, unsigned int qno)
{
	struct net_device *dev = priv->dev;
	unsigned int rx_ctrl, desc_ctrl;
	void *buf_addr, *priv_data;
	int length, offset;
	struct sk_buff *skb;

	BUILD_BUG_ON(PFE_PKT_HEADROOM + PFE_PKT_SIZE +
		     SKB_DATA_ALIGN(sizeof(struct skb_shared_info)) > PFE_BUF_SIZE);

	for (;;) {
		buf_addr = hif_lib_receive_pkt(&priv->client, qno, &length, &offset,
						&rx_ctrl, &desc_ctrl, &priv_data);
		if (!buf_addr)
			break;

		if (!(desc_ctrl & CL_DESC_FIRST) || !(desc_ctrl & CL_DESC_LAST)) {
			net_warn_ratelimited("%s: dropping unsupported multi-descriptor packet (ctrl=%#x)\n",
					      dev->name, desc_ctrl);
			pfe_hif_spare_buf_put(buf_addr + PFE_PKT_HEADROOM);
			dev->stats.rx_dropped++;
			continue;
		}

		if (unlikely(length < ETH_HLEN || offset < 0 ||
			     offset + length > SKB_WITH_OVERHEAD(PFE_BUF_SIZE))) {
			net_warn_ratelimited("%s: dropping invalid rx length=%d offset=%d\n",
					      dev->name, length, offset);
			pfe_hif_spare_buf_put(buf_addr + PFE_PKT_HEADROOM);
			dev->stats.rx_dropped++;
			continue;
		}

		skb = build_skb(buf_addr, PFE_BUF_SIZE);
		if (!skb) {
			pfe_hif_spare_buf_put(buf_addr + PFE_PKT_HEADROOM);
			dev->stats.rx_dropped++;
			continue;
		}

		skb_reserve(skb, offset);
		skb_put(skb, length);

		skb->dev = dev;
		skb->protocol = eth_type_trans(skb, dev);
		if ((dev->features & NETIF_F_RXCSUM) && (rx_ctrl & HIF_CTRL_RX_CHECKSUMMED))
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		else
			skb_checksum_none_assert(skb);

		dev->stats.rx_packets++;
		dev->stats.rx_bytes += length;

		napi_gro_receive(&priv->pfe->hif.napi, skb);
	}
}

static int pfe_eth_event_handler(void *data, int event, int qno)
{
	struct pfe_eth_priv_s *priv = data;

	switch (event) {
	case EVENT_RX_PKT_IND:
		pfe_eth_rx_drain(priv, qno);
		hif_lib_event_handler_start(&priv->client, EVENT_RX_PKT_IND, qno);
		break;
	case EVENT_TXDONE_IND:
		pfe_eth_flush_txq(priv, PFE_ETH_TXQ, EMAC_TXQ_DEPTH);
		hif_lib_event_handler_start(&priv->client, EVENT_TXDONE_IND, qno);
		break;
	default:
		break;
	}

	return 0;
}

static void __pfe_eth_stop_txq_locked(struct pfe_eth_priv_s *priv, unsigned int wake_thres)
{
	struct net_device *dev = priv->dev;

	lockdep_assert_held(&priv->tx_clean_lock);

	if (unlikely(priv->closing || !netif_running(dev)))
		return;

	priv->tx_wake_thres = wake_thres;
	if (!netif_queue_stopped(dev)) {
		netif_stop_queue(dev);
		priv->sw_stats.tx_queue_stops++;
	}
	pfe_hif_request_tx_irq(&priv->pfe->hif, priv->client.id);
	__pfe_eth_reap_txq(priv, PFE_ETH_TXQ, EMAC_TXQ_DEPTH);
	__pfe_eth_maybe_wake_txq(priv, PFE_ETH_TXQ);
}

static void __pfe_eth_tx_post_xmit(struct pfe_eth_priv_s *priv, int reap_count,
				   unsigned int stop_thres, bool force_stop)
{
	unsigned int wake_thres = max_t(unsigned int, PFE_TX_WAKE_THRES, stop_thres);

	lockdep_assert_held(&priv->tx_clean_lock);

	__pfe_eth_reap_txq(priv, PFE_ETH_TXQ, reap_count);
	if (force_stop || unlikely(hif_lib_tx_avail(&priv->client, PFE_ETH_TXQ) < stop_thres))
		__pfe_eth_stop_txq_locked(priv, wake_thres);
	else
		__pfe_eth_maybe_wake_txq(priv, PFE_ETH_TXQ);
}

#define PHY_PORT_STRUCT_SIZE	16
#define PHY_PORT_ITF_INDEX_OFF	5
#define PHY_PORT_MAC_ADDR_OFF	8

static void pfe_eth_class_register_port(struct pfe_eth_priv_s *priv)
{
	unsigned long base = priv->pfe->class_phy_port_dmem + PFE_GEM0_ID * PHY_PORT_STRUCT_SIZE;
	int id;

	for (id = CLASS0_ID; id <= CLASS_MAX_ID; id++) {
		pe_dmem_write(id, PFE_GEM0_ID, base + PHY_PORT_ITF_INDEX_OFF, 1);
		pe_dmem_memcpy_to32(id, base + PHY_PORT_MAC_ADDR_OFF, priv->dev->dev_addr, ETH_ALEN);
	}
}

static int pfe_eth_open(struct net_device *dev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(dev);
	struct hif_client_s *client = &priv->client;
	MAC_ADDR addr;
	int rc;

	memset(client, 0, sizeof(*client));
	client->id = PFE_CL_GEM0;
	client->tx_qn = EMAC_TXQ_CNT;
	client->rx_qn = EMAC_RXQ_CNT;
	client->tx_qsize = EMAC_TXQ_DEPTH;
	client->rx_qsize = EMAC_RXQ_DEPTH;
	client->priv = priv;
	client->pfe = priv->pfe;
	client->event_handler = pfe_eth_event_handler;

	spin_lock_bh(&priv->tx_clean_lock);
	priv->closing = false;
	priv->tx_wake_thres = PFE_TX_WAKE_THRES;
	spin_unlock_bh(&priv->tx_clean_lock);

	rc = hif_lib_client_register(client);
	if (rc) {
		netdev_err(dev, "hif_lib_client_register(%d) failed: %d\n", client->id, rc);
		goto err_register;
	}

	pfe_gemac_init(priv);

	if (!is_valid_ether_addr(dev->dev_addr)) {
		netdev_err(dev, "invalid MAC address\n");
		rc = -EADDRNOTAVAIL;
		goto err_addr;
	}

	addr.bottom = get_unaligned_le32(dev->dev_addr);
	addr.top = get_unaligned_le16(dev->dev_addr + 4);
	gemac_set_laddrN(priv->EMAC_baseaddr, &addr, 1);

	pfe_eth_class_register_port(priv);

	if (of_property_present(priv->of_node, "phy-handle")) {
		rc = pfe_phy_init(dev);
		if (rc) {
			netdev_err(dev, "pfe_phy_init() failed: %d\n", rc);
			goto err_addr;
		}
	} else {
		netif_carrier_off(dev);
		netdev_info(dev, "no phy-handle in DT yet -- link will stay down\n");
	}

	pfe_eth_start(priv);

	if (priv->phydev)
		pfe_phy_restore_led_mode(priv->phydev);

	netdev_reset_queue(dev);
	netif_start_queue(dev);

	return 0;

err_addr:
	hif_lib_client_unregister(client);
err_register:
	spin_lock_bh(&priv->tx_clean_lock);
	priv->closing = true;
	spin_unlock_bh(&priv->tx_clean_lock);
	return rc;
}

/*
 * Stop TX and PHY/MAC, poll pending TX completions for up to ~10 ms,
 * and orphan any descriptors still in flight before unregistering the client.
 */
static int pfe_eth_close(struct net_device *dev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(dev);
	struct hif_client_tx_queue *q = &priv->client.tx_q[PFE_ETH_TXQ];
	int retries = 100;

	BUILD_BUG_ON(PFE_TX_STOP_THRES < PFE_TX_MAX_SG_DESCS);
	BUILD_BUG_ON(PFE_TX_WAKE_THRES < PFE_TX_STOP_THRES);

	spin_lock_bh(&priv->tx_clean_lock);
	priv->closing = true;
	spin_unlock_bh(&priv->tx_clean_lock);

	netif_tx_disable(dev);

	pfe_eth_stop(priv);

	if (priv->phydev)
		pfe_phy_exit(dev);

	while (READ_ONCE(q->tx_pending) && retries-- > 0) {
		pfe_eth_flush_txq(priv, PFE_ETH_TXQ, EMAC_TXQ_DEPTH);
		if (!READ_ONCE(q->tx_pending))
			break;
		usleep_range(100, 200);
	}

	if (unlikely(READ_ONCE(q->tx_pending))) {
		netdev_warn(dev, "closing with %u pending TX descriptors\n",
			    READ_ONCE(q->tx_pending));
		spin_lock_bh(&priv->tx_clean_lock);
		hif_tx_lock(&priv->pfe->hif);
		pfe_hif_tx_orphan_client(&priv->pfe->hif, priv->client.id, q);
		hif_tx_unlock(&priv->pfe->hif);
		spin_unlock_bh(&priv->tx_clean_lock);
	}

	netdev_reset_queue(dev);
	pfe_hif_release_tx_irq(&priv->pfe->hif, priv->client.id);

	hif_lib_client_unregister(&priv->client);

	return 0;
}

static void pfe_eth_unmap_frag_segs(struct device *dma_dev,
				    const struct pfe_tx_sg_seg *segs,
				    unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		if (segs[i].dma)
			dma_unmap_page(dma_dev, segs[i].dma, segs[i].len,
				       DMA_TO_DEVICE);
	}
}

/*
 * Transmit an skb using scatter-gather segments (splitting fragments at
 * PFE_HIF_MAX_BD_LEN and linearizing if descriptor count would exceed
 * PFE_TX_MAX_SG_DESCS or frag_list is present); large GSO skbs are
 * segmented by the Linux 6.18 stack before reaching ndo_start_xmit().
 */
static netdev_tx_t pfe_eth_send_packet(struct sk_buff *skb, struct net_device *dev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(dev);
	struct device *dma_dev = priv->pfe->hif.dev;
	struct pfe_tx_sg_seg frag_segs[PFE_TX_MAX_SG_DESCS - 1];
	unsigned int nr_frag_segs = 0;
	unsigned int pkt_len = skb->len;
	u32 ctrl = 0;
	int rc;

	if (skb->ip_summed == CHECKSUM_PARTIAL)
		ctrl = HIF_CTRL_TX_CHECKSUM;

	if (skb_is_nonlinear(skb)) {
		bool need_linearize = skb_has_frag_list(skb);
		unsigned int nr_frags = skb_shinfo(skb)->nr_frags;
		unsigned int est_descs = 1;
		unsigned int i;

		for (i = 0; !need_linearize && i < nr_frags; i++) {
			unsigned int flen = skb_frag_size(&skb_shinfo(skb)->frags[i]);

			if (!flen)
				continue;
			est_descs += DIV_ROUND_UP(flen, PFE_HIF_MAX_BD_LEN);
			if (est_descs > PFE_TX_MAX_SG_DESCS)
				need_linearize = true;
		}

		if (need_linearize) {
			priv->sw_stats.tx_sg_linearized++;
			if (skb_linearize(skb)) {
				dev_kfree_skb_any(skb);
				dev->stats.tx_dropped++;
				return NETDEV_TX_OK;
			}
		}
	}

	if (skb_headlen(skb) < min_t(unsigned int, pkt_len, PFE_TX_MIN_HEADLEN) &&
	    !pskb_may_pull(skb, min_t(unsigned int, pkt_len, PFE_TX_MIN_HEADLEN))) {
		dev_kfree_skb_any(skb);
		dev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	if (unlikely(skb_cow_head(skb, sizeof(struct hif_hdr)) < 0)) {
		dev_kfree_skb_any(skb);
		dev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	if (unlikely(sizeof(struct hif_hdr) + skb_headlen(skb) > PFE_HIF_MAX_BD_LEN)) {
		dev_kfree_skb_any(skb);
		dev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	if (skb_is_nonlinear(skb)) {
		unsigned int nr_frags = skb_shinfo(skb)->nr_frags;
		unsigned int i;

		for (i = 0; i < nr_frags; i++) {
			const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
			unsigned int flen = skb_frag_size(frag);
			unsigned int off = 0;

			while (flen > 0) {
				unsigned int chunk = min_t(unsigned int, flen,
							   PFE_HIF_MAX_BD_LEN);

				if (unlikely(nr_frag_segs >= ARRAY_SIZE(frag_segs))) {
					pfe_eth_unmap_frag_segs(dma_dev, frag_segs,
								nr_frag_segs);
					dev_kfree_skb_any(skb);
					dev->stats.tx_dropped++;
					return NETDEV_TX_OK;
				}

				frag_segs[nr_frag_segs].dma =
					skb_frag_dma_map(dma_dev, frag, off,
							 chunk, DMA_TO_DEVICE);
				if (unlikely(dma_mapping_error(dma_dev,
							       frag_segs[nr_frag_segs].dma))) {
					pfe_eth_unmap_frag_segs(dma_dev, frag_segs,
								nr_frag_segs);
					dev_kfree_skb_any(skb);
					dev->stats.tx_dropped++;
					return NETDEV_TX_OK;
				}

				frag_segs[nr_frag_segs].len = chunk;
				frag_segs[nr_frag_segs].map_type = PFE_TX_MAP_PAGE;
				nr_frag_segs++;
				off += chunk;
				flen -= chunk;
			}
		}
	}

	rc = hif_lib_xmit_pkt_sg(&priv->client, PFE_ETH_TXQ,
				 skb->data, skb_headlen(skb),
				 frag_segs, nr_frag_segs, ctrl, skb,
				 dev, pkt_len);
	if (unlikely(rc)) {
		pfe_eth_unmap_frag_segs(dma_dev, frag_segs, nr_frag_segs);
		if (rc == -EBUSY) {
			priv->sw_stats.tx_busy_retries++;
			spin_lock_bh(&priv->tx_clean_lock);
			__pfe_eth_tx_post_xmit(priv, EMAC_TXQ_DEPTH,
						PFE_TX_STOP_THRES, true);
			spin_unlock_bh(&priv->tx_clean_lock);
			return NETDEV_TX_BUSY;
		}
		dev_kfree_skb_any(skb);
		dev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += pkt_len;
	if (nr_frag_segs) {
		priv->sw_stats.tx_sg_pkts++;
		priv->sw_stats.tx_sg_frags += nr_frag_segs;
	}

	spin_lock_bh(&priv->tx_clean_lock);
	__pfe_eth_tx_post_xmit(priv, max_t(int, PFE_ETH_TX_FREE_MAX, (int)(1 + nr_frag_segs)),
			       PFE_TX_STOP_THRES, false);
	spin_unlock_bh(&priv->tx_clean_lock);

	return NETDEV_TX_OK;
}

static void pfe_eth_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct pfe_eth_priv_s *priv = netdev_priv(dev);
	struct pfe_hif *hif = &priv->pfe->hif;
	struct hif_client_tx_queue *q = &priv->client.tx_q[PFE_ETH_TXQ];
	struct hif_tx_queue *hq = &hif->client[priv->client.id].tx_q[PFE_ETH_TXQ];
	u32 ttc = READ_ONCE(hif->Txtoclean);

	priv->sw_stats.tx_timeouts++;
	netdev_warn(dev, "TX timeout (q=%u pending=%u r=%u w=%u ack=%u | hif avail=%u send=%u clean=%u bd_ctrl=%#x)\n",
		    txqueue, q->tx_pending, q->read_idx, q->write_idx, hq->ack_idx,
		    READ_ONCE(hif->TxAvail), READ_ONCE(hif->Txtosend), ttc,
		    READ_ONCE(hif->TxBase[ttc].ctrl));

	pfe_eth_flush_txq(priv, PFE_ETH_TXQ, EMAC_TXQ_DEPTH);
	hif_tx_dma_start();
}

static int pfe_eth_get_hash(const u8 *addr)
{
	u8 temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8;

	temp1 = addr[0] & 0x3F;
	temp2 = ((addr[0] & 0xC0) >> 6) | ((addr[1] & 0x0F) << 2);
	temp3 = ((addr[1] & 0xF0) >> 4) | ((addr[2] & 0x03) << 4);
	temp4 = (addr[2] & 0xFC) >> 2;
	temp5 = addr[3] & 0x3F;
	temp6 = ((addr[3] & 0xC0) >> 6) | ((addr[4] & 0x0F) << 2);
	temp7 = ((addr[4] & 0xF0) >> 4) | ((addr[5] & 0x03) << 4);
	temp8 = (addr[5] & 0xFC) >> 2;

	return temp1 ^ temp2 ^ temp3 ^ temp4 ^ temp5 ^ temp6 ^ temp7 ^ temp8;
}

static void pfe_eth_set_multi(struct net_device *dev)
{
	struct pfe_eth_priv_s *priv = netdev_priv(dev);
	struct netdev_hw_addr *ha;
	MAC_ADDR hash_addr;
	MAC_ADDR spec_addr;
	MAC_ADDR zero_addr = { .bottom = 0, .top = 0 };
	int programmed_uc = 0;
	int uc_count = 0;
	int result, slot;

	if (dev->flags & IFF_PROMISC)
		gemac_enable_copy_all(priv->EMAC_baseaddr);
	else
		gemac_disable_copy_all(priv->EMAC_baseaddr);

	if (dev->flags & IFF_BROADCAST)
		gemac_allow_broadcast(priv->EMAC_baseaddr);
	else
		gemac_no_broadcast(priv->EMAC_baseaddr);

	if (dev->flags & IFF_ALLMULTI) {
		hash_addr.bottom = 0xFFFFFFFF;
		hash_addr.top = 0xFFFFFFFF;
		gemac_set_hash(priv->EMAC_baseaddr, &hash_addr);
		gemac_enable_multicast(priv->EMAC_baseaddr);

		netdev_for_each_uc_addr(ha, dev) {
			if (programmed_uc >= EMAC_SPEC_ADDR_MAX - 1)
				break;
			spec_addr.bottom = get_unaligned_le32(ha->addr);
			spec_addr.top = get_unaligned_le16(ha->addr + 4);
			gemac_set_laddrN(priv->EMAC_baseaddr, &spec_addr, programmed_uc + 2);
			programmed_uc++;
		}
	} else if (netdev_mc_count(dev) || netdev_uc_count(dev)) {
		hash_addr.bottom = 0;
		hash_addr.top = 0;

		netdev_for_each_mc_addr(ha, dev) {
			result = pfe_eth_get_hash(ha->addr);
			if (result < 0 || result >= EMAC_HASH_REG_BITS)
				continue;
			if (result < 32)
				hash_addr.bottom |= BIT(result);
			else
				hash_addr.top |= BIT(result - 32);
		}

		uc_count = -1;
		netdev_for_each_uc_addr(ha, dev) {
			if (++uc_count < EMAC_SPEC_ADDR_MAX - 1) {
				spec_addr.bottom = get_unaligned_le32(ha->addr);
				spec_addr.top = get_unaligned_le16(ha->addr + 4);
				gemac_set_laddrN(priv->EMAC_baseaddr, &spec_addr, uc_count + 2);
				programmed_uc++;
			} else {
				result = pfe_eth_get_hash(ha->addr);
				if (result < 0 || result >= EMAC_HASH_REG_BITS)
					continue;
				if (result < 32)
					hash_addr.bottom |= BIT(result);
				else
					hash_addr.top |= BIT(result - 32);
			}
		}

		gemac_set_hash(priv->EMAC_baseaddr, &hash_addr);
		if (netdev_mc_count(dev))
			gemac_enable_multicast(priv->EMAC_baseaddr);
		else
			gemac_disable_multicast(priv->EMAC_baseaddr);
	} else {
		gemac_disable_multicast(priv->EMAC_baseaddr);
	}

	for (slot = programmed_uc + 2; slot <= EMAC_SPEC_ADDR_MAX; slot++)
		gemac_set_laddrN(priv->EMAC_baseaddr, &zero_addr, slot);

	if (netdev_uc_count(dev) > EMAC_SPEC_ADDR_MAX - 1)
		gemac_enable_unicast(priv->EMAC_baseaddr);
	else
		gemac_disable_unicast(priv->EMAC_baseaddr);
}

static int pfe_eth_set_features(struct net_device *dev, netdev_features_t features)
{
	struct pfe_eth_priv_s *priv = netdev_priv(dev);

	if (features & NETIF_F_RXCSUM)
		gemac_enable_rx_checksum_offload(priv->EMAC_baseaddr);
	else
		gemac_disable_rx_checksum_offload(priv->EMAC_baseaddr);

	return 0;
}

static int pfe_eth_set_mac_address(struct net_device *dev, void *p)
{
	struct pfe_eth_priv_s *priv = netdev_priv(dev);
	MAC_ADDR addr;
	int rc;

	rc = eth_mac_addr(dev, p);
	if (rc)
		return rc;

	if (netif_running(dev)) {
		addr.bottom = get_unaligned_le32(dev->dev_addr);
		addr.top = get_unaligned_le16(dev->dev_addr + 4);
		gemac_set_laddrN(priv->EMAC_baseaddr, &addr, 1);
		pfe_eth_class_register_port(priv);
	}

	return 0;
}

static const struct net_device_ops pfe_netdev_ops = {
	.ndo_open = pfe_eth_open,
	.ndo_set_features = pfe_eth_set_features,
	.ndo_stop = pfe_eth_close,
	.ndo_start_xmit = pfe_eth_send_packet,
	.ndo_tx_timeout = pfe_eth_tx_timeout,
	.ndo_set_mac_address = pfe_eth_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_rx_mode = pfe_eth_set_multi,
	.ndo_eth_ioctl = phy_do_ioctl,
};

static const char pfe_ethtool_stat_names[][ETH_GSTRING_LEN] = {
	"tx_sg_pkts",
	"tx_sg_frags",
	"tx_sg_linearized",
	"tx_queue_stops",
	"tx_queue_wakes",
	"tx_busy_retries",
	"tx_timeouts",
	"tx_ring_avail",
	"hif_tx_avail",
	"hif_tx_irq_count",
	"hif_napi_sched",
	"hif_napi_poll",
	"hif_napi_packets",
	"hif_napi_descs",
	"hif_napi_full_budget",
	"hif_napi_client_full",
};

static int pfe_ethtool_get_sset_count(struct net_device *dev, int sset)
{
	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	return ARRAY_SIZE(pfe_ethtool_stat_names);
}

static void pfe_ethtool_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	if (stringset == ETH_SS_STATS)
		memcpy(data, pfe_ethtool_stat_names, sizeof(pfe_ethtool_stat_names));
}

static void pfe_ethtool_get_stats(struct net_device *dev,
				  struct ethtool_stats *stats, u64 *data)
{
	struct pfe_eth_priv_s *priv = netdev_priv(dev);
	struct pfe_hif *hif = &priv->pfe->hif;
	int i = 0;

	data[i++] = priv->sw_stats.tx_sg_pkts;
	data[i++] = priv->sw_stats.tx_sg_frags;
	data[i++] = priv->sw_stats.tx_sg_linearized;
	data[i++] = priv->sw_stats.tx_queue_stops;
	data[i++] = priv->sw_stats.tx_queue_wakes;
	data[i++] = priv->sw_stats.tx_busy_retries;
	data[i++] = priv->sw_stats.tx_timeouts;
	data[i++] = hif_lib_tx_avail(&priv->client, PFE_ETH_TXQ);
	data[i++] = READ_ONCE(hif->TxAvail);
	data[i++] = READ_ONCE(hif->tx_irq_count);
	data[i++] = hif->napi_counters[NAPI_SCHED_COUNT];
	data[i++] = hif->napi_counters[NAPI_POLL_COUNT];
	data[i++] = hif->napi_counters[NAPI_PACKET_COUNT];
	data[i++] = hif->napi_counters[NAPI_DESC_COUNT];
	data[i++] = hif->napi_counters[NAPI_FULL_BUDGET_COUNT];
	data[i++] = hif->napi_counters[NAPI_CLIENT_FULL_COUNT];
}

static const struct ethtool_ops pfe_ethtool_ops = {
	.get_link = ethtool_op_get_link,
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
	.get_sset_count = pfe_ethtool_get_sset_count,
	.get_strings = pfe_ethtool_get_strings,
	.get_ethtool_stats = pfe_ethtool_get_stats,
};

static void pfe_eth_extphy_clk_release(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
	clk_put(clk);
}

static char pfe_boot_mac_param[18];
static u8 pfe_boot_mac_addr[ETH_ALEN];
static bool pfe_boot_mac_addr_valid;

static void pfe_parse_mac_param(void)
{
	char buf[sizeof(pfe_boot_mac_param)];
	char *p = buf, *tok;

	if (!pfe_boot_mac_param[0])
		return;

	strscpy(buf, pfe_boot_mac_param, sizeof(buf));
	tok = strsep(&p, ",");
	if (tok && *tok && mac_pton(tok, pfe_boot_mac_addr))
		pfe_boot_mac_addr_valid = true;
}

module_param_string(mac_addr, pfe_boot_mac_param, sizeof(pfe_boot_mac_param), 0444);
MODULE_PARM_DESC(mac_addr, "Initial MAC address for GEM0");

static int pfe_eth_probe_gem0(struct pfe *pfe, struct device_node *np)
{
	struct pfe_eth_priv_s *priv;
	struct net_device *dev;
	struct clk *extphy_clk;
	int rc;

	if (pfe_gem0_netdev) {
		dev_err(pfe->dev, "%pOF: gem0 already registered\n", np);
		return -EEXIST;
	}

	extphy_clk = of_clk_get_by_name(np, "extphy");
	if (IS_ERR(extphy_clk)) {
		dev_warn(pfe->dev, "%pOF: no extphy clock (rc=%ld) -- MDIO may stop working after boot\n",
			 np, PTR_ERR(extphy_clk));
	} else {
		rc = clk_prepare_enable(extphy_clk);
		if (rc) {
			dev_warn(pfe->dev, "%pOF: failed to enable extphy clock: %d\n", np, rc);
			clk_put(extphy_clk);
		} else {
			rc = devm_add_action_or_reset(pfe->dev, pfe_eth_extphy_clk_release, extphy_clk);
			if (rc)
				return rc;
		}
	}

	dev = alloc_etherdev(sizeof(*priv));
	if (!dev)
		return -ENOMEM;

	SET_NETDEV_DEV(dev, pfe->dev);
	dev->netdev_ops = &pfe_netdev_ops;
	dev->ethtool_ops = &pfe_ethtool_ops;
	dev->watchdog_timeo = 5 * HZ;
	dev->min_mtu = ETH_MIN_MTU;
	dev->max_mtu = ETH_DATA_LEN;
	dev->needed_headroom = sizeof(struct hif_hdr);

	dev->hw_features = NETIF_F_RXCSUM | NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
			    NETIF_F_SG;
	dev->features |= dev->hw_features | NETIF_F_GRO;

	priv = netdev_priv(dev);
	priv->pfe = pfe;
	priv->dev = dev;
	priv->id = PFE_GEM0_ID;
	priv->of_node = of_node_get(np);
	priv->mii_bus = pfe_mii_bus;
	priv->closing = true;
	spin_lock_init(&priv->lock);
	spin_lock_init(&priv->tx_clean_lock);

	priv->EMAC_baseaddr = EMAC1_BASE_ADDR;
	priv->GPI_baseaddr = EGPI1_BASE_ADDR;
	priv->low_tmuQ = HIF_GEMAC_TMUQ_BASE;
	priv->high_tmuQ = priv->low_tmuQ + 1;

	if (of_get_phy_mode(np, &priv->phy_mode))
		priv->phy_mode = PHY_INTERFACE_MODE_MII;

	pfe_parse_mac_param();
	if (of_get_ethdev_address(np, dev)) {
		if (pfe_boot_mac_addr_valid)
			eth_hw_addr_set(dev, pfe_boot_mac_addr);
		else
			eth_hw_addr_random(dev);
	}

	rc = register_netdev(dev);
	if (rc) {
		dev_err(pfe->dev, "register_netdev() failed for gem0: %d\n", rc);
		of_node_put(priv->of_node);
		free_netdev(dev);
		return rc;
	}

	dev_info(pfe->dev, "gem0 registered as %s (phy-mode %s)\n",
		 dev->name, phy_modes(priv->phy_mode));

	pfe_gem0_netdev = dev;

	return 0;
}

int pfe_eth_init(struct pfe *pfe)
{
	struct device_node *pfe_np = pfe->dev->of_node;
	struct device_node *mdio_np;
	struct device_node *np;
	int rc;

	pfe->clk_gem_tx = devm_clk_get_enabled(pfe->dev, "gemtx");
	if (IS_ERR(pfe->clk_gem_tx))
		return dev_err_probe(pfe->dev, PTR_ERR(pfe->clk_gem_tx),
				      "failed to get/enable gemtx clock\n");

	mdio_np = of_get_child_by_name(pfe_np, "mdio");
	if (!mdio_np) {
		dev_info(pfe->dev, "no mdio child node -- skipping ethernet bring-up\n");
		return 0;
	}

	rc = pfe_eth_mdio_init(pfe, mdio_np);
	of_node_put(mdio_np);
	if (rc)
		return rc;

	for_each_available_child_of_node(pfe_np, np) {
		u32 id;

		if (!of_device_is_compatible(np, "fsl,ls1024a-pfe-gem"))
			continue;

		if (of_property_read_u32(np, "reg", &id) || id != PFE_GEM0_ID)
			continue;

		rc = pfe_eth_probe_gem0(pfe, np);
		if (rc) {
			dev_err(pfe->dev, "failed to probe %pOF: %d\n", np, rc);
			of_node_put(np);
			pfe_eth_mdio_exit();
			return rc;
		}
		of_node_put(np);
		break;
	}

	return 0;
}

void pfe_eth_exit(struct pfe *pfe)
{
	struct net_device *dev = pfe_gem0_netdev;

	if (dev) {
		struct pfe_eth_priv_s *priv = netdev_priv(dev);
		struct device_node *of_node = priv->of_node;

		unregister_netdev(dev);
		free_netdev(dev);
		of_node_put(of_node);
		pfe_gem0_netdev = NULL;
	}

	pfe_eth_mdio_exit();
}
