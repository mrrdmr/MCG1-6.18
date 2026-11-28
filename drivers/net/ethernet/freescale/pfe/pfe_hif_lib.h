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
 * scope: hardware TSO and multi-queue TX credit/QoS accounting --
 * dropping them avoids reintroducing the FCI-adjacent machinery this
 * whole port has deliberately left out since Stage P1.
 */
#ifndef _PFE_HIF_LIB_H_
#define _PFE_HIF_LIB_H_

#include <linux/bitops.h>
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

#define CL_DESC_OWN		BIT(31) /* Ownership: set = HIF driver owns it */
#define CL_DESC_LAST		BIT(30)
#define CL_DESC_FIRST		BIT(29)
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

/*
 * Zero-copy Rx: buffers are ordinary cached page fragments
 * (netdev_alloc_frag_align(), 128-byte aligned so the metadata word at
 * base[0] and the DMA region at base + PFE_PKT_HEADROOM never share a
 * cache line) and go through the streaming DMA API: pfe_hif_buf_map()
 * when posted to the HIF ring, pfe_hif_buf_unmap() when the descriptor
 * completes, before the CPU touches the data. Once received, the whole
 * fragment is handed to the stack with build_skb() (pfe_eth.c) and freed
 * by skb_free_frag() on the way out -- the same lifecycle every mainline
 * page-fragment Rx path uses.
 *
 * The dma_addr_t of the current mapping is stashed in the buffer's own
 * headroom so it travels alongside the plain "void *" pointer.
 */
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

	*(dma_addr_t *)base = DMA_MAPPING_ERROR;

	return base + PFE_PKT_HEADROOM;
}

/* Bus address of the current RX mapping (DMA_MAPPING_ERROR if unmapped). */
static inline dma_addr_t pfe_hif_buf_dma(void *pkt)
{
	return *(dma_addr_t *)(pkt - PFE_PKT_HEADROOM);
}

/* Release an RX buffer back to the page-fragment allocator. */
static inline void pfe_hif_buf_free(struct device *dev, void *pkt)
{
	(void)dev;

	if (likely(pkt)) {
		WARN_ON_ONCE(pfe_hif_buf_dma(pkt) != DMA_MAPPING_ERROR);
		skb_free_frag(pkt - PFE_PKT_HEADROOM);
	}
}

/* Map the packet area for the HIF to write into; store the bus address in
 * the headroom slot so unmap/reuse helpers do not require a separate copy.
 */
static inline dma_addr_t pfe_hif_buf_map(struct device *dev, void *pkt)
{
	dma_addr_t dma = dma_map_single(dev, pkt, PFE_PKT_SIZE, DMA_FROM_DEVICE);

	if (dma_mapping_error(dev, dma)) {
		*(dma_addr_t *)(pkt - PFE_PKT_HEADROOM) = DMA_MAPPING_ERROR;
		return 0;
	}

	*(dma_addr_t *)(pkt - PFE_PKT_HEADROOM) = dma;

	return dma;
}

/* Unmap the active RX DMA mapping and clear the stored headroom address. */
static inline void pfe_hif_buf_unmap(struct device *dev, void *pkt)
{
	dma_addr_t *slot = pkt - PFE_PKT_HEADROOM;

	if (*slot != DMA_MAPPING_ERROR) {
		dma_unmap_single(dev, *slot, PFE_PKT_SIZE, DMA_FROM_DEVICE);
		*slot = DMA_MAPPING_ERROR;
	}
}

/*
 * Pool of HIF_RX_DESC_NT spare buffers pre-allocated via GFP_KERNEL in
 * pfe_hif_lib_init() and recycled via plain array push/pop, keeping page-
 * fragment allocator calls off the hot RX path in the steady state (with
 * pfe_hif_buf_alloc() fallback in client_put_rxpacket() if the spare
 * pool ever drains). Sized at HIF_RX_DESC_NT because the drain runs
 * synchronously inside the HIF NAPI poll, so at most a small number of
 * buffers are in flight between HIF dequeue and build_skb().
 */
void *pfe_hif_spare_buf_get(void);
void pfe_hif_spare_buf_put(void *buf);

int pfe_hif_lib_init(struct pfe *pfe);
void pfe_hif_lib_exit(struct pfe *pfe);
void hif_lib_indicate_client(int cl_id, int event, int data);

/*
 * Client registration API (Stage P7) -- lets a net_device driver
 * (pfe_eth.c) hand the HIF driver its Rx/Tx shared-memory queues and
 * become reachable via hif_lib_indicate_client() above.
 */
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

/*
 * Dequeues one received packet's buffer (Stage P8). *ofst is the byte
 * offset from the start of that buffer where the actual payload begins
 * (past the hif_hdr, and past an optional firmware-prepended private
 * header whose size is packed into the top byte of *rx_ctrl -- this
 * port never sets that up on the Tx side, so it should always read 0
 * for anything this driver itself sends, but a packet ingested straight
 * off the wire goes through the same CLASS PE code path regardless, so
 * the offset math is kept general rather than assumed away). *priv_data
 * points at that private header if present, NULL otherwise -- unused by
 * this port (no consumer for it), kept only because getting *ofst right
 * depends on computing it.
 */
void *hif_lib_receive_pkt(struct hif_client_s *client, int qno, int *len, int *ofst,
			   unsigned int *rx_ctrl, unsigned int *desc_ctrl, void **priv_data);

/*
 * Rx re-arm (Stage P8) -- call after draining a client's Rx queue for
 * the given (event, qno) pair in response to hif_lib_indicate_client()
 * calling this client's event_handler. hif_lib_indicate_client() is
 * edge-triggered (queue_mask[] latches once until cleared here), so
 * skipping this call would mean this queue never gets indicated again.
 * Also re-checks for a packet that arrived during the drain (a race
 * between "queue looked empty" and "mask cleared") and re-indicates
 * immediately if so, rather than risking a missed wakeup.
 */
int hif_lib_event_handler_start(struct hif_client_s *client, int event, int qno);

#endif /* _PFE_HIF_LIB_H_ */
