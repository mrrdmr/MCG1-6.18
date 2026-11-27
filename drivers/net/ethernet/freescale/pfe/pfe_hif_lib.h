/* SPDX-License-Identifier: GPL-2.0 */
/*
 * HIF client-facing shared-memory layout and helpers -- ported from the
 * 3.2.26 vendor tree's pfe_ctrl/pfe_hif_lib.h (kmodules/mspd-c2k/pfe/ in
 * symops/MCG1-3.2.26).
 *
 * Stage P5 added what pfe_hif.c's init/exit/ISR/NAPI path itself needs
 * (the shared-memory rx buffer pool, and the client event indication
 * used by the Rx path). Stage P7 added the client-registration API
 * (hif_lib_client_register()/unregister()) that pfe_eth.c calls from
 * its .ndo_open/.ndo_stop. Stage P8 adds the SG TX submission path
 * (hif_lib_xmit_pkt_sg(), hif_lib_tx_get_next_complete()) and
 * hif_lib_event_handler_start() (the Rx re-arm half of the
 * hif_lib_indicate_client() edge-triggered wakeup below). Still out of
 * scope: TSO and TX credit/QoS accounting -- neither is needed for
 * plain, non-jumbo Ethernet traffic at the pace this baseline targets
 * (DHCP+ping), and dropping them avoids reintroducing the FCI-adjacent
 * machinery this whole port has deliberately left out since Stage P1.
 */
#ifndef _PFE_HIF_LIB_H_
#define _PFE_HIF_LIB_H_

#include <linux/dma-mapping.h>
#include <linux/netdevice.h>

#include "pfe_hif.h"

enum {
	REQUEST_CL_REGISTER = 0,
	REQUEST_CL_UNREGISTER,
	HIF_REQUEST_MAX
};

enum {
	EVENT_HIGH_RX_WM = 0,
	EVENT_RX_PKT_IND,
	EVENT_TXDONE_IND,
	HIF_EVENT_MAX
};

struct hif_client_rx_queue {
	struct rx_queue_desc *base;
	u32 size;
	u32 read_idx;
	u32 write_idx;
};

struct hif_client_tx_queue {
	struct tx_queue_desc *base;
	u32 size;
	u32 read_idx;
	u32 write_idx;
	u32 tx_pending;
	unsigned long jiffies_last_packet;
};

struct hif_client_s {
	int id;
	int tx_qn;
	int rx_qn;
	void *rx_qbase;
	void *tx_qbase;
	int tx_qsize;
	int rx_qsize;
	struct hif_client_tx_queue tx_q[HIF_CLIENT_QUEUES_MAX];
	struct hif_client_rx_queue rx_q[HIF_CLIENT_QUEUES_MAX];
	int (*event_handler)(void *priv, int event, int data);
	unsigned long queue_mask[HIF_EVENT_MAX];
	struct pfe *pfe;
	void *priv;
};

/* Client-specific shared memory: number of Rx/Tx queues, base addresses
 * and queue sizes.
 */
struct hif_client_shm {
	volatile u32 ctrl; /* 0-7: number of Rx queues, 8-15: number of Tx queues */
	volatile u32 rx_qbase;
	volatile u32 rx_qsize;
	volatile u32 tx_qbase;
	volatile u32 tx_qsize;
};

#define CLIENT_CTRL_RX_Q_CNT_OFST	0
#define CLIENT_CTRL_TX_Q_CNT_OFST	8
#define CLIENT_CTRL_RX_Q_CNT(ctrl)	(((ctrl) >> CLIENT_CTRL_RX_Q_CNT_OFST) & 0xFF)
#define CLIENT_CTRL_TX_Q_CNT(ctrl)	(((ctrl) >> CLIENT_CTRL_TX_Q_CNT_OFST) & 0xFF)

/* Shared memory used to communicate between the HIF driver and host/
 * client drivers. rx_buf_pool/rx_buf_pool_cnt must be filled in (by
 * pfe_hif_shm_init()) before the HIF driver starts; rx_buf_pool_cnt
 * must be >= HIF_RX_DESC_NT.
 */
struct hif_shm {
	volatile u32 rx_buf_pool_cnt;
	volatile void *rx_buf_pool[HIF_RX_DESC_NT];
	volatile u32 gClient_status;
	volatile u32 hif_qfull;
	volatile u32 hif_qresume;
	struct hif_client_shm client[HIF_CLIENTS_MAX];
};

#define CL_DESC_OWN		(1 << 31) /* Ownership: set = HIF driver owns it */
#define CL_DESC_LAST		(1 << 30)
#define CL_DESC_FIRST		(1 << 29)
#define CL_DESC_BUF_LEN(x)	((x) & 0xFFFF)
#define CL_DESC_FLAGS(x)	(((x) & 0xF) << 16)
#define CL_DESC_GET_FLAGS(x)	(((x) >> 16) & 0xF)

struct rx_queue_desc {
	void *data;
	u32 ctrl;
	u32 client_ctrl;
};

struct tx_queue_desc {
	void *data;
	u32 ctrl;
};

/*
 * "ip_header = 64 + 6(hif_header) + 14 (MAC Header)" lands 4-byte
 * aligned, which HIF Rx needs for good performance (vendor comment).
 */
#define PFE_PKT_HEADER_SZ	sizeof(struct hif_hdr)
#define PFE_BUF_SIZE		2048	/* headroom + pkt size + skb shared info */
#define PFE_PKT_HEADROOM	128
#define PFE_PKT_SIZE		1544	/* maximum ethernet packet size */

extern unsigned int pfe_pkt_size;
extern unsigned int pfe_pkt_headroom;

#define PFE_BUF_ALIGN		128U

/* Allocate a 2048-byte page fragment and return the packet data pointer
 * past PFE_PKT_HEADROOM, using the first headroom slot for RX DMA state.
 */
static inline void *pfe_hif_buf_alloc(struct device *dev)
{
	void *base;

	(void)dev;

	/* Second argument to netdev_alloc_frag_align() is the alignment
	 * itself, which must be a power of two -- it is converted to the
	 * internal mask by the helper.
	 */
	base = netdev_alloc_frag_align(PFE_BUF_SIZE, PFE_BUF_ALIGN);
	if (!base)
		return NULL;

	*(dma_addr_t *)base = 0;

	return base + PFE_PKT_HEADROOM;
}

/* Release an RX buffer back to the page-fragment allocator. */
static inline void pfe_hif_buf_free(struct device *dev, void *pkt)
{
	(void)dev;

	if (likely(pkt))
		skb_free_frag(pkt - PFE_PKT_HEADROOM);
}

/* Map the packet area for the HIF to write into; store the bus address in
 * the headroom slot so unmap/reuse helpers do not require a separate copy.
 */
static inline dma_addr_t pfe_hif_buf_map(struct device *dev, void *pkt)
{
	dma_addr_t dma = dma_map_single(dev, pkt, PFE_PKT_SIZE, DMA_FROM_DEVICE);

	if (dma_mapping_error(dev, dma))
		return 0;

	*(dma_addr_t *)(pkt - PFE_PKT_HEADROOM) = dma;

	return dma;
}

/* Bus address of the current RX mapping (0 if unmapped). */
static inline dma_addr_t pfe_hif_buf_dma(void *pkt)
{
	return *(dma_addr_t *)(pkt - PFE_PKT_HEADROOM);
}

/* Unmap the active RX DMA mapping and clear the stored headroom address. */
static inline void pfe_hif_buf_unmap(struct device *dev, void *pkt)
{
	dma_addr_t *slot = pkt - PFE_PKT_HEADROOM;

	if (*slot) {
		dma_unmap_single(dev, *slot, PFE_PKT_SIZE, DMA_FROM_DEVICE);
		*slot = 0;
	}
}

void *pfe_hif_spare_buf_get(void);
void pfe_hif_spare_buf_put(void *buf);

int pfe_hif_lib_init(struct pfe *pfe);
void pfe_hif_lib_exit(struct pfe *pfe);
void hif_lib_indicate_client(int cl_id, int event, int data);

int hif_lib_client_register(struct hif_client_s *client);
int hif_lib_client_unregister(struct hif_client_s *client);
int hif_lib_tmu_queue_start(struct hif_client_s *client, int qno);
int hif_lib_tmu_queue_stop(struct hif_client_s *client, int qno);

#define PFE_TX_MAX_SG_DESCS	(MAX_SKB_FRAGS + 1)
#define PFE_HIF_MAX_BD_LEN	0x3FFFU

unsigned int hif_lib_tx_avail(struct hif_client_s *client, unsigned int qno);
int hif_lib_xmit_pkt_sg(struct hif_client_s *client, unsigned int qno,
			void *head_data, unsigned int head_len,
			const struct pfe_tx_sg_seg *frag_segs,
			unsigned int nr_frags, u32 client_ctrl,
			void *client_data, struct net_device *dev,
			unsigned int pkt_len);

void *hif_lib_tx_get_next_complete(struct hif_client_s *client, int qno,
				    unsigned int *flags, int count);

void *hif_lib_receive_pkt(struct hif_client_s *client, int qno, int *len, int *ofst,
			   unsigned int *rx_ctrl, unsigned int *desc_ctrl, void **priv_data);

int hif_lib_event_handler_start(struct hif_client_s *client, int event, int qno);

#endif /* _PFE_HIF_LIB_H_ */
