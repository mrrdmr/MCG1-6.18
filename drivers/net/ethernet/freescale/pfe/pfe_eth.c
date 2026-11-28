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

/* TMU queue numbers this GEM's traffic is scheduled on: TMU queues 0-5
 * are reserved for other PFE-internal traffic classes in the vendor
 * design; each GEM gets a low/high priority queue pair starting at 6.
 */
#define PFE_GEM0_ID		0
#define HIF_GEMAC_TMUQ_BASE	6

/*
 * This SoC has exactly one PFE, and the MDIO management registers are
 * physically wired through GEM0/EMAC1's own register block regardless
 * of which GEM's PHY is being addressed (matches the vendor board file,
 * which only ever defines MDIO bus 0). Kept as file-static state for
 * the same reason g_pfe is (see pfe_mod.h) -- there's only ever one of
 * these on real hardware, and threading a pointer to it through every
 * per-GEM probe call for no benefit isn't worth it.
 */
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
	/*
	 * 96 is the only real data point found for this divisor: the GPL
	 * source drop's arch/arm/mach-comcerto/board-c2kevm.c has this
	 * exact value hand-edited into comcerto_mdio_pdata[0], alongside a
	 * comment identifying a Broadcom PHY on this bus (see the P7
	 * research writeup in the commit log) -- unconfirmed for this
	 * specific board like the PHY address itself, but a real divisor
	 * setting a wide margin under any PHY's max MDC frequency either
	 * way, so it's a safe default regardless.
	 */
	pfe_mdio_priv.mdc_div = 96;

	bus = mdiobus_alloc();
	if (!bus)
		return -ENOMEM;

	bus->name = "ls1024a-pfe-mdio";
	bus->read = pfe_eth_mdio_read;
	bus->write = pfe_eth_mdio_write;
	bus->reset = pfe_eth_mdio_reset;
	bus->priv = &pfe_mdio_priv;
	bus->phy_mask = 0; /* scan every address (0..31) */
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

	/* GEM verifies IPv4/TCP/UDP checksums; the class PE reports the
	 * result as HIF_CTRL_RX_CHECKSUMMED in the per-packet client_ctrl
	 * (see pfe_eth_rx_drain()).
	 */
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

/*
 * Restores this PHY's LED-adjacent shadow registers to their vendor-
 * matching values -- found by diffing raw register reads against the
 * 3.2.26 vendor kernel on the same board. Two prior fixes already
 * cover other real differences the same way: PHY_BRCM_RX_REFCLK_UNUSED
 * (see pfe_phy_init() below) stops bcm54612e_config_init() from
 * diverting LED4 to a CLK125 clock output, and this function's own ECR
 * write undoes that same config_init()'s global interrupt/event mask
 * (0x1000 here vs 0x0 on the vendor kernel, which never touches it).
 *
 * LEDS1 (shadow 0x0d, LED1/LED3 -- the "Link" LED): the generic
 * Broadcom config_init() shared by every non-SFP chip this driver
 * matches (drivers/net/phy/broadcom.c, right after the switch on
 * phy_id) unconditionally reprograms this pin pair from its hardware-
 * strap default (BCM_LED_SRC_LINKSPD1/LINKSPD2, 0x10 -- matching the
 * vendor kernel exactly, which never touches this register at all)
 * into BCM_LED_SRC_MULTICOLOR1 (0xaa) plus a Multicolor expansion-
 * register LINK_ACT encoding -- its own comment there says this is
 * meant for a single physical bi-color LED sharing both pins. This
 * board instead has two separate single-color LEDs wired to LED1 and
 * LED3 (confirmed: the vendor kernel's LED is lit *steady*, matching a
 * plain link-speed indicator, not a blink-on-activity one), so forcing
 * the bi-color pairing breaks that. Restored to the vendor-matching
 * encoding here -- though on its own this wasn't sufficient to light
 * it; see the RGMII_MODE write below for what actually was.
 *
 * RGMII Mode Selector (shadow 0x0b) -- not one of mainline's named LED
 * registers, but this is the exact register the vendor's WD-specific
 * bcm54610_config_init() hack (shared by BCM54610/BCM54612 in that
 * tree) writes as a side effect of what its own comment describes as
 * "disable half-duplex" (the real half-duplex-disable part is two
 * separate writes to MII regs 4/9; this shadow-0x0b write is a third,
 * distinct action the same function performs that this port never
 * carried over -- dismissed early on as "not LED-related" based on the
 * register's generic name alone, without checking the actual value).
 * A full blind register sweep (every shadow reg 0x00-0x1f, every
 * expansion reg on the three pages any BCM54xx code in this driver
 * selects) diffed against the same sweep on the vendor kernel turned up
 * this as the one meaningful difference among them: this port reads
 * 0x009 here (hardware default, untouched) vs the vendor kernel's
 * 0x08c -- which exactly matches the DATA field the WD hack's own raw
 * MDIO writes encode (phy_write(0x1C, 0xAC8C) -- VAL=0x0b, DATA=0x08c).
 * BCM54XX_SHD_RGMII_MODE is a real, documented-elsewhere-in-mainline
 * register (bcm_phy_write_shadow(..., BCM54XX_SHD_RGMII_MODE, 0) is
 * used for BCM50610/BCM50610M under PHY_BRCM_CLEAR_RGMII_MODE), so this
 * genuinely configures something -- unlike TOP_MISC_LED_CTL, which a
 * previous round found reads back unchanged no matter what's written
 * (likely mostly reserved bits for this chip), this one is confirmed
 * writable and vendor-nonzero.
 *
 * Caller must run this after pfe_eth_start()/phy_start(), not right
 * after connecting -- see the comment at its one call site in
 * pfe_eth_open() for why (phy_start() unconditionally resumes the PHY,
 * which re-runs config_init and undoes this otherwise).
 */
#define PFE_PHY_SHD_RGMII_MODE_VENDOR_VAL	0x08c

static void pfe_phy_restore_led_mode(struct phy_device *phydev)
{
	/*
	 * LEDS1 (LED1/LED3, the "Link" LED): restored to the vendor-
	 * matching hardware-default encoding (LINKSPD1/LINKSPD2 -- real
	 * link-speed indication, not a blunt "always on"). Confirmed on
	 * real hardware (putty.log.213) that the LED actually responds now
	 * that RGMII_MODE (below) is also fixed -- earlier rounds tried
	 * this same LEDS1 value (and a forced-all-on variant) without
	 * RGMII_MODE fixed and got nothing, so RGMII_MODE was the real
	 * missing piece, not LEDS1's content.
	 */
	u16 leds1 = BCM54XX_SHD_LEDS1_LED1(BCM_LED_SRC_LINKSPD1) |
		    BCM54XX_SHD_LEDS1_LED3(BCM_LED_SRC_LINKSPD2);
	int ecr;

	phy_write(phydev, MII_BCM54XX_SHD,
		  MII_BCM54XX_SHD_WRITE | MII_BCM54XX_SHD_VAL(BCM54XX_SHD_LEDS1) |
		  MII_BCM54XX_SHD_DATA(leds1));

	/*
	 * ECR.IM (global interrupt/event mask): confirmed real difference
	 * vs. the vendor kernel (0x1000 here vs 0x0 there, neither kernel
	 * ever using a real hardware IRQ line for this PHY either way --
	 * see git log for the full writeup). Clear it to match; harmless.
	 */
	ecr = phy_read(phydev, MII_BCM54XX_ECR);
	if (ecr < 0)
		netdev_warn(phydev->attached_dev, "failed to read PHY ECR: %d\n", ecr);
	else
		phy_write(phydev, MII_BCM54XX_ECR, ecr & ~MII_BCM54XX_ECR_IM);

	/* LEDS2 (LED0/LED2, "ACT"): deliberately left untouched -- this one
	 * already works correctly on hardware defaults with no software
	 * help on either kernel, so there's nothing to restore here.
	 */

	/*
	 * RGMII Mode Selector (shadow 0x0b) -- the actual fix. Confirmed on
	 * real hardware (putty.log.213): with 0x08c written, the "Link" LED
	 * responds for the first time.
	 */
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

	/*
	 * Real hardware regression found post-Stage-P8: only one of this
	 * port's two LEDs showed activity, vs. both on the 3.2.26 vendor
	 * kernel on the same board/cable/switch. Traced to
	 * bcm54612e_config_init() (drivers/net/phy/broadcom.c) -- mainline-
	 * only code that didn't exist in the vendor's 2012-era kernel --
	 * which by default repurposes this PHY's LED4 pin as a CLK125
	 * reference-clock output rather than leaving it as an LED, unless
	 * told PHY_BRCM_RX_REFCLK_UNUSED. Nothing on this board consumes a
	 * PHY-sourced 125 MHz reference clock (RGMII carries its own TXC/
	 * RXC), so that repurposing silently darkens an LED that's supposed
	 * to just show activity. of_phy_get_and_connect() has no way to set
	 * phydev->dev_flags before phy_connect_direct() runs config_init(),
	 * so this uses of_phy_connect() instead, which does
	 * ("phy->dev_flags |= flags;" before connecting, drivers/net/mdio/
	 * of_mdio.c) -- priv->phy_mode already comes from of_get_phy_mode()
	 * in pfe_eth_probe_gem0(), same as of_phy_get_and_connect() would
	 * look up internally.
	 */
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

/*
 * TX queue index this port always submits to. The client registers
 * EMAC_TXQ_CNT (16) SW queues (matching the vendor's per-packet
 * priority classification, pfe_eth_get_queuenum() in the vendor
 * driver), but this port doesn't implement any QoS classification --
 * everything goes on queue 0, the same "unclassified default" queue
 * traffic lands on in the vendor driver's own default configuration
 * (priv->default_priority, never set to anything else there either).
 */
#define PFE_ETH_TXQ		0

/*
 * Per-call cap on how many completed Tx descriptors hif_tx_done_process()
 * (called inside hif_lib_tx_get_next_complete(), Stage P5) will process
 * in one go -- distinct from pfe_hif.c's own private TX_FREE_MAX_COUNT
 * (64), which bounds the same call from the HIF ring's own housekeeping
 * path instead. 16 matches the vendor's TX_FREE_MAX_COUNT for this
 * client-level flush.
 */
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

	/*
	 * Skip netdev_completed_queue() if BQL was already reset during
	 * pfe_eth_close() (marked by tx_wake_thres == 0).
	 */
	if (completed_pkts && likely(priv->tx_wake_thres))
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

		/*
		 * hif_lib_receive_pkt() hands back the true base of the
		 * page-fragment buffer (what build_skb() needs);
		 * pfe_hif_spare_buf_put() takes the same "usable"
		 * (base + headroom) pointer pfe_hif_spare_buf_get() returns,
		 * so every free below adds the headroom back.
		 */
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

		/* Zero-copy: wrap the whole fragment, the stack frees it
		 * (skb_free_frag()) when done. offset already skips the
		 * headroom + HIF header, and PFE_BUF_SIZE leaves room for
		 * skb_shared_info after the packet area.
		 */
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

		dev_sw_netstats_rx_add(dev, length);

		/* Runs inside the HIF NAPI poll (pfe_hif_rx_poll()), so GRO can
		 * coalesce consecutive TCP segments before they hit the stack;
		 * napi_complete_done() in the poll flushes whatever is held.
		 */
		napi_gro_receive(&priv->pfe->hif.napi, skb);
	}
}

static int pfe_eth_event_handler(void *data, int event, int qno)
{
	struct pfe_eth_priv_s *priv = data;

	switch (event) {
	case EVENT_RX_PKT_IND:
		pfe_eth_rx_drain(priv, qno);
		/* Edge-triggered -- re-arm so the next arrival indicates
		 * again (see hif_lib_event_handler_start()'s own comment).
		 */
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

/*
 * Layout of the CLASS PE firmware's phy_port[] array (vendor tree's
 * pfe/pfe/c2000/common/types.h -- "struct itf { void *phys; u8 type;
 * u8 index; }" padded to 8 bytes, then "struct physical_port { struct
 * itf itf; u8 mac_addr[6]; u8 id; }" padded to 16 -- confirmed via
 * readelf -sW against our own firmware/ls1024a-pfe/class_c2000.elf:
 * phy_port @ 0x1610, size 80 = 5 * 16 (MAX_PHY_PORTS = GEM_PORTS(3) +
 * MAX_WIFI_VAPS(2) in that build)).
 */
#define PHY_PORT_STRUCT_SIZE	16
#define PHY_PORT_ITF_INDEX_OFF	5
#define PHY_PORT_MAC_ADDR_OFF	8

/*
 * Without this, the CLASS PE firmware never learns this port's MAC
 * address or interface index -- confirmed on real hardware to be
 * required for the firmware to forward any received frame to the host
 * at all: TX worked fine without it, but RX stayed at 0 packets
 * forever, even after minutes of real broadcast/DHCP traffic on the
 * wire. This mirrors two of the DMEM writes the vendor host driver's
 * pfe_ctrl_set_eth_state()/CMD_TX_ENABLE made as a side effect of an
 * FCI command this port doesn't implement (see pfe_ctrl.h's banner for
 * why the FCI apparatus itself isn't ported) -- done directly here
 * instead of routing through that dropped command-handler machinery.
 * Vendor's own onif/bridge registration (add_onif()/
 * bridge_interface_register(), also part of that same FCI init path)
 * is genuinely bridging-specific (forwarding *between* GEM0/1/2) and
 * isn't needed for this port's "receive my own traffic, deliver it to
 * my own HIF client" case.
 */
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

	/*
	 * gemac_set_laddrN() takes a MAC_ADDR {u32 bottom; u32 top;} pair.
	 * The vendor driver gets there via (MAC_ADDR *)dev->dev_addr, which
	 * reads 2 bytes past the end of the 6-byte dev_addr[] array -- build
	 * the pair explicitly instead of copying that over-read.
	 */
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

	/*
	 * Must run after pfe_eth_start(), not right after connecting in
	 * pfe_phy_init() -- confirmed on real hardware (putty.log.203/204/
	 * 205) that the write lands fine at connect() time but is gone
	 * again by the time link-up is reported a few ms later. Root cause:
	 * pfe_eth_start()'s phy_start() unconditionally calls
	 * __phy_resume() (drivers/net/phy/phy.c) regardless of whether the
	 * PHY was ever actually suspended, which for this driver calls
	 * bcm54xx_resume() -> ends by re-running bcm54xx_config_init() --
	 * the exact function whose LED1/LED3 multicolor override and ECR
	 * interrupt masking this fix exists to undo. of_phy_connect()
	 * itself already goes through the same config_init/resume dance
	 * once internally (phy_attach_direct() calls phy_init_hw() then
	 * unconditionally phy_resume()), so the value written in
	 * pfe_phy_init() never had a chance regardless -- this is the
	 * first point after phy_start() where it can actually stick.
	 */
	if (priv->phydev)
		pfe_phy_restore_led_mode(priv->phydev);

	spin_lock_bh(&priv->tx_clean_lock);
	priv->tx_wake_thres = PFE_TX_WAKE_THRES;
	netdev_reset_queue(dev);
	spin_unlock_bh(&priv->tx_clean_lock);
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

	spin_lock_bh(&priv->tx_clean_lock);
	if (unlikely(READ_ONCE(q->tx_pending))) {
		netdev_warn(dev, "closing with %u pending TX descriptors\n",
			    READ_ONCE(q->tx_pending));
		hif_tx_lock(&priv->pfe->hif);
		pfe_hif_tx_orphan_client(&priv->pfe->hif, priv->client.id, q);
		hif_tx_unlock(&priv->pfe->hif);
	}
	/*
	 * Mark BQL reset (tx_wake_thres = 0) before unlocking so any late
	 * TX completion arriving before hif_lib_client_unregister() frees
	 * its skb without calling netdev_completed_queue() on a reset queue.
	 */
	priv->tx_wake_thres = 0;
	netdev_reset_queue(dev);
	spin_unlock_bh(&priv->tx_clean_lock);

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
	unsigned int req_descs = 1;
	u32 ctrl = 0;
	int rc;

	/* The class PE computes the IPv4 header and TCP/UDP checksums when
	 * HIF_CTRL_TX_CHECKSUM is set in the HIF header (vendor
	 * pfe_eth_send_packet() does the same for CHECKSUM_PARTIAL).
	 */
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

		if (!need_linearize)
			req_descs = est_descs;

		/*
		 * Check ring space before modifying skb or mapping DMA pages
		 * so NETDEV_TX_BUSY returns an unmodified skb.
		 */
		if (unlikely(hif_lib_tx_avail(&priv->client, PFE_ETH_TXQ) < req_descs)) {
			priv->sw_stats.tx_busy_retries++;
			spin_lock_bh(&priv->tx_clean_lock);
			__pfe_eth_tx_post_xmit(priv, EMAC_TXQ_DEPTH,
						max_t(unsigned int, PFE_TX_STOP_THRES, req_descs),
						true);
			spin_unlock_bh(&priv->tx_clean_lock);
			return NETDEV_TX_BUSY;
		}

		if (need_linearize) {
			priv->sw_stats.tx_sg_linearized++;
			if (skb_linearize(skb)) {
				dev_kfree_skb_any(skb);
				dev->stats.tx_dropped++;
				return NETDEV_TX_OK;
			}
		}
	} else if (unlikely(hif_lib_tx_avail(&priv->client, PFE_ETH_TXQ) < req_descs)) {
		priv->sw_stats.tx_busy_retries++;
		spin_lock_bh(&priv->tx_clean_lock);
		__pfe_eth_tx_post_xmit(priv, EMAC_TXQ_DEPTH,
					PFE_TX_STOP_THRES, true);
		spin_unlock_bh(&priv->tx_clean_lock);
		return NETDEV_TX_BUSY;
	}

	if (skb_headlen(skb) < min_t(unsigned int, pkt_len, PFE_TX_MIN_HEADLEN) &&
	    !pskb_may_pull(skb, min_t(unsigned int, pkt_len, PFE_TX_MIN_HEADLEN))) {
		dev_kfree_skb_any(skb);
		dev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	/* Ensure writable headroom for struct hif_hdr prepended in place. */
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

	/*
	 * Do not acquire hif->tx_lock here: hif_lib_xmit_pkt_sg() takes
	 * hif->tx_lock internally (spin_lock_bh() is non-reentrant, and
	 * lock order is tx_clean_lock -> hif->tx_lock). BQL accounting is
	 * also performed inside hif_lib_xmit_pkt_sg() under hif->tx_lock.
	 */
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


	dev_sw_netstats_tx_add(dev, 1, pkt_len);
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

/*
 * 6-bit CRC-free hash over a MAC address, ported unchanged from the
 * vendor's pfe_eth_get_hash(): XOR-folds each 6-bit group of the 48-bit
 * address together. The result selects one bit of the GEMAC's 64-bit
 * hash register (gemac_set_hash()) -- this is the hardware's multicast
 * (and overflow-unicast) address filter, not a general-purpose hash.
 */
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

/*
 * .ndo_set_rx_mode -- ported from the vendor's pfe_eth_set_multi(). Without
 * this, the GEMAC's multicast filter stays in the disabled state
 * pfe_gemac_init() leaves it in at probe time and never gets enabled, no
 * matter what multicast groups the stack joins. IPv6 depends on this far
 * more than IPv4 does: Neighbor Discovery and Router Advertisement are
 * multicast-only (solicited-node ff02::1:ffXX:XXXX / MAC 33:33:ffXX:XXXX,
 * all-nodes ff02::1), so without a working hash filter, neighbor/router
 * resolution over IPv6 depends entirely on stale cache entries or timing
 * luck instead of actually working -- indistinguishable from "IPv6 is
 * flaky" until traced back here.
 */
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
		/* Hash-match every multicast address. */
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
	/* SIOCGMIIPHY/SIOCGMIIREG/SIOCSMIIREG -- phy_do_ioctl() dispatches
	 * to phy_mii_ioctl() when a phydev is attached. Wired up for raw
	 * MII register access (mii-tool and similar), not otherwise used
	 * by this driver.
	 */
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

/*
 * Without this, `ethtool eth0` (link/speed/duplex/autoneg query) fails
 * outright with "No data available" instead of reporting phydev's real
 * state -- there's no core-kernel fallback for the settings ioctl the
 * way there is for driver-info (which is why `ethtool -i eth0` worked
 * even before this was added). phy_ethtool_get/set_link_ksettings() are
 * the standard phylib-backed helpers used by most mainline drivers with
 * an attached PHY; .get_link falls back to netif_carrier_ok() when
 * there's no phydev.
 */
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

/*
 * This board's barebox has no per-unit DT (its DTB is compiled into this
 * kernel image, not read from flash/OTP), so the real, per-unit hardware
 * MAC address(es) reach Linux the same way they did under the 3.2.26
 * vendor kernel's legacy ATAG boot path: a "mac_addr=" kernel command
 * line parameter (comma-separated, up to one per GEM), set by barebox's
 * boot script. The vendor kernel consumed this via its own __setup()
 * handler in arch/arm/mach-comcerto/comcerto-2000.c
 * (mac_addr_setup()/mac_addr_init()); this is the DT-based port's
 * equivalent, feeding pfe_eth_probe_gem0() below instead of platform
 * data. Confirmed present on this board's real boot log: "commandline:
 * ... mac_addr=00:90:A9:D0:AA:4F ...", and confirmed NOT consumed
 * anywhere before this -- the kernel logs it under "Unknown kernel
 * command line parameters" and passes it through to userspace, meaning
 * GEM0 fell back to eth_hw_addr_random() below. That's not just
 * cosmetic: a random MAC on this netdev never matches the CLASS PE
 * firmware's actual traffic (see pfe_eth_class_register_port()'s
 * comment -- the firmware needs to know this port's real MAC to
 * forward anything to the host at all), so this alone can fully
 * account for real inbound traffic never reaching the host on a board
 * that boots this way, independent of anything else.
 *
 * Support both bare "mac_addr=" on the kernel command line (via __setup()
 * when built-in) and "<module>.mac_addr=" via module_param_string().
 */
static u8 pfe_boot_mac_addr[ETH_ALEN];
static bool pfe_boot_mac_addr_valid;

static void pfe_parse_mac_str(const char *str)
{
	char buf[18];
	char *p = buf, *tok;

	if (!str || !*str)
		return;

	strscpy(buf, str, sizeof(buf));
	tok = strsep(&p, ",");
	if (tok && *tok && mac_pton(tok, pfe_boot_mac_addr))
		pfe_boot_mac_addr_valid = true;
}

#ifndef MODULE
static int __init pfe_mac_addr_setup(char *str)
{
	pfe_parse_mac_str(str);
	return 1;
}
__setup("mac_addr=", pfe_mac_addr_setup);
#endif

static char pfe_boot_mac_param[18];
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

	/*
	 * Each GEM has its own "extphy" reference clock line
	 * (LS1024A_CLK_EXTPHY0/1/2 in clk-ls1024a.c) distinct from the pfe
	 * node's shared "gemtx" -- confirmed on real hardware to be required
	 * for this GEM's PHY management (MDIO) to keep working at all. Without
	 * it, MDIO reads/writes still complete (the bus protocol handshake
	 * itself doesn't need it) but every read past the first ~1-2s of
	 * boot silently returns the last successfully-read value instead of
	 * the register actually requested, regardless of which PHY/register
	 * is addressed -- traced to the kernel's own "clk: Disabling unused
	 * clocks" late boot step physically gating this line off, since
	 * nothing had ever requested it before. gem0 isn't its own struct
	 * device (no of_platform_populate() for pfe's child nodes), so
	 * of_clk_get_by_name() rather than devm_clk_get() -- cleanup is tied
	 * to the parent pfe device's lifetime via devm_add_action_or_reset()
	 * instead, since nothing currently unbinds a single GEM's node
	 * independently of the whole pfe device.
	 */
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
	/* hif_lib_xmit_pkt_sg() writes struct hif_hdr in place just before
	 * skb->data -- ask the stack to reserve room for it up front so
	 * pfe_eth_send_packet()'s skb_cow_head() fallback only has to
	 * reallocate for an skb that came without headroom.
	 */
	dev->watchdog_timeo = 5 * HZ;
	dev->min_mtu = ETH_MIN_MTU;
	dev->max_mtu = ETH_DATA_LEN;
	dev->needed_headroom = sizeof(struct hif_hdr);
	dev->pcpu_stat_type = NETDEV_PCPU_STAT_TSTATS;

	/* Rx/Tx checksum offload is done by the GEMAC + class PE (same
	 * firmware the vendor driver uses, which advertises the same bits);
	 * GRO is pure software on top of the HIF NAPI poll.
	 *
	 * Hardware TSO (struct hif_tso_hdr, HIF_CTRL_TX_TSO/_TSO6/_TSO_END in
	 * pfe_hif.h) isn't wired up yet because its exact field semantics
	 * (byte order and units of ip_off/ip_id/ip_len/tcp_off/tcp_seq)
	 * aren't confirmed, and guessing wrong risks silent TCP stream
	 * corruption; large GSO/TSO frames are segmented by the Linux 6.18
	 * network stack before reaching ndo_start_xmit().
	 */
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

	if (pfe_boot_mac_param[0])
		pfe_parse_mac_str(pfe_boot_mac_param);
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

	/*
	 * Enabled for the platform device's whole lifetime, not scoped to
	 * .ndo_open/.ndo_stop like the vendor driver's own clk_enable()/
	 * clk_disable() calls in pfe_eth_open()/pfe_eth_close() -- a plain
	 * devm_clk_get() here left the clock unprepared/disabled between
	 * probe (~1.2s) and the first .ndo_open() (~10s, whenever ifup
	 * actually runs), and the kernel's own late_initcall "clk: Disabling
	 * unused clocks" (~2.2s) physically gates off exactly this kind of
	 * still-unused clock in that window. Confirmed on real hardware:
	 * every MDIO transaction after that gate-then-open cycle returned
	 * the same stuck stale value regardless of which register was
	 * requested, as if the GEMAC's MDIO management sub-block never
	 * recovered from losing (and later regaining) its clock mid-boot.
	 */
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
