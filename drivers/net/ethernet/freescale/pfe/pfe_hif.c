// SPDX-License-Identifier: GPL-2.0
/*
 * HIF (Host Interface) DMA descriptor ring + ISR -- ported from the
 * 3.2.26 vendor tree's pfe_ctrl/pfe_hif.c (kmodules/mspd-c2k/pfe/ in
 * symops/MCG1-3.2.26). This is the highest-risk file in the whole PFE
 * port: a fully custom host<->firmware DMA/descriptor protocol with no
 * mainline equivalent to crib from.
 *
 * Dropped, not deferred, relative to the vendor file:
 *  - CONFIG_PLATFORM_PCI and CONFIG_PLATFORM_EMULATION branches: this
 *    board is neither.
 *  - page_mode/LRO buffer allocation: a performance feature, not needed
 *    for basic connectivity (see pfe_hif_lib.c). pfe_pkt_size/
 *    pfe_pkt_headroom are fixed at their non-LRO values.
 *  - The __memcpy8()/__memcpy12() hand-written ARM ldm/stm helpers:
 *    replaced with plain memcpy() of the same fixed sizes. The
 *    descriptor fields are already `volatile`, and the ring itself is
 *    dma_alloc_coherent() memory, so there's no correctness reason for
 *    hand-rolled assembly here -- it bought a minor, unmeasured
 *    micro-optimization on 2012-era compilers at the cost of being
 *    silently wrong if the struct layouts it assumes ever changed.
 *  - outer_inv_range() on the descriptor ring + the napi_first_batch
 *    flag that gated it: this was an explicit PL310 outer-cache
 *    invalidate on the *descriptor* memory specifically. dma_alloc_
 *    coherent() memory is coherent by definition -- if this SoC's
 *    descriptor ring genuinely needed manual cache maintenance on top
 *    of that, dma_alloc_coherent() would be the wrong allocator to
 *    begin with. Dropped; flagged here in case real-hardware Rx
 *    corruption ever points back at it.
 *
 * Rx packet buffers are allocated as 2048-byte page fragments via
 * netdev_alloc_frag_align() (pfe_hif_buf_alloc()/_free()/_map()/
 * _unmap() in pfe_hif_lib.h) and mapped with streaming DMA
 * (dma_map_single(..., DMA_FROM_DEVICE)). Storing DMA metadata in the
 * first slot of PFE_PKT_HEADROOM keeps it on a separate cache line
 * from the packet payload and allows zero-copy build_skb() delivery in
 * pfe_eth.c.
 */

#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/lockdep.h>
#include <linux/log2.h>
#include <linux/skbuff.h>
#include <linux/string.h>

#include "pfe_mod.h"
#include "pfe_hif.h"
#include "pfe_hif_lib.h"
#include "pfe_hw_lib.h"

#ifndef HIF_TXPKT_INT
#define HIF_TXPKT_INT		(1 << 4)
#endif
#define HIF_INT_MASK		(HIF_INT | HIF_RXPKT_INT)
#define TX_FREE_MAX_COUNT	64

#define inc_hif_rxidx(idxname)	idxname = (idxname + 1) & (hif->RxRingSize - 1)
#define inc_hif_txidx(idxname)	idxname = (idxname + 1) & (hif->TxRingSize - 1)

static int pfe_hif_alloc_descr(struct pfe_hif *hif)
{
	void *addr;
	dma_addr_t dma_addr;

	addr = dma_alloc_coherent(hif->dev,
				   HIF_RX_DESC_NT * sizeof(struct hif_desc) +
				   HIF_TX_DESC_NT * sizeof(struct hif_desc),
				   &dma_addr, GFP_KERNEL);
	if (!addr) {
		dev_err(hif->dev, "%s: could not allocate buffer descriptors\n", __func__);
		return -ENOMEM;
	}

	hif->descr_baseaddr_p = dma_addr;
	hif->descr_baseaddr_v = addr;
	hif->RxRingSize = HIF_RX_DESC_NT;
	hif->TxRingSize = HIF_TX_DESC_NT;

	return 0;
}

static void pfe_hif_free_descr(struct pfe_hif *hif)
{
	if (hif->descr_baseaddr_v) {
		dma_free_coherent(hif->dev,
				   hif->RxRingSize * sizeof(struct hif_desc) +
				   hif->TxRingSize * sizeof(struct hif_desc),
				   hif->descr_baseaddr_v, hif->descr_baseaddr_p);
		hif->descr_baseaddr_v = NULL;
		hif->descr_baseaddr_p = 0;
	}
}

static void pfe_hif_unmap_tx_desc(struct pfe_hif *hif, struct hif_desc_sw *desc_sw)
{
	if (desc_sw->map_type == PFE_TX_MAP_NONE)
		return;

	if (desc_sw->map_type == PFE_TX_MAP_PAGE)
		dma_unmap_page(hif->dev, desc_sw->data, desc_sw->len, DMA_TO_DEVICE);
	else if (desc_sw->map_type == PFE_TX_MAP_SINGLE)
		dma_unmap_single(hif->dev, desc_sw->data, desc_sw->len, DMA_TO_DEVICE);

	desc_sw->data = DMA_MAPPING_ERROR;
	desc_sw->len = 0;
	desc_sw->map_type = PFE_TX_MAP_NONE;
}

static void pfe_hif_reset_tx_desc_sw(struct hif_desc_sw *desc_sw)
{
	desc_sw->data = DMA_MAPPING_ERROR;
	desc_sw->skb = NULL;
	desc_sw->len = 0;
	desc_sw->q_idx = 0;
	desc_sw->client_id = HIF_CLIENTS_MAX;
	desc_sw->q_no = 0;
	desc_sw->chain_id = 0;
	desc_sw->map_type = PFE_TX_MAP_NONE;
}

static void pfe_hif_release_buffers(struct pfe_hif *hif)
{
	struct hif_desc *desc = hif->RxBase;
	int i;

	if (desc) {
		for (i = 0; i < hif->RxRingSize; i++) {
			if (hif->rx_buf_addr[i]) {
				pfe_hif_buf_unmap(hif->dev, hif->rx_buf_addr[i]);
				if (i < hif->shm->rx_buf_pool_cnt && !hif->shm->rx_buf_pool[i]) {
					hif->shm->rx_buf_pool[i] = hif->rx_buf_addr[i];
				} else {
					dev_err(hif->dev, "%s: buffer pool already full\n", __func__);
					pfe_hif_buf_free(hif->dev, hif->rx_buf_addr[i]);
				}
				hif->rx_buf_addr[i] = NULL;
			}

			desc->data = 0;
			desc->status = 0;
			desc->ctrl = 0;
			desc++;
		}
	}

	if (hif->TxBase) {
		desc = hif->TxBase;
		for (i = 0; i < hif->TxRingSize; i++) {
			struct hif_desc_sw *desc_sw = &hif->tx_sw_queue[i];
			struct sk_buff *skb = desc_sw->skb;

			desc_sw->skb = NULL;
			pfe_hif_unmap_tx_desc(hif, desc_sw);
			pfe_hif_reset_tx_desc_sw(desc_sw);
			if (unlikely(skb))
				dev_kfree_skb_any(skb);
			desc->data = 0;
			desc->status = 0;
			desc->ctrl = 0;
			desc++;
		}
	}
}

/*
 * Initializes the HIF Rx/Tx ring descriptors and fills the Rx ring with
 * buffers from the shared-memory pool.
 */
static int pfe_hif_init_buffers(struct pfe_hif *hif)
{
	struct hif_desc *desc;
	dma_addr_t first_desc_p, tx_base_p;
	int i;

	if (hif->shm->rx_buf_pool_cnt < hif->RxRingSize)
		return -ENOMEM;

	hif->RxBase = hif->descr_baseaddr_v;
	memset(hif->RxBase, 0, hif->RxRingSize * sizeof(struct hif_desc));

	desc = hif->RxBase;
	first_desc_p = hif->descr_baseaddr_p;

	for (i = 0; i < hif->RxRingSize; i++) {
		u32 next_idx = (i + 1) & (hif->RxRingSize - 1);
		void *buf = (void *)hif->shm->rx_buf_pool[i];

		if (!pfe_hif_buf_map(hif->dev, buf)) {
			while (i-- > 0) {
				pfe_hif_buf_unmap(hif->dev, hif->rx_buf_addr[i]);
				hif->shm->rx_buf_pool[i] = hif->rx_buf_addr[i];
				hif->rx_buf_addr[i] = NULL;
			}
			return -ENOMEM;
		}
		hif->rx_buf_addr[i] = buf;
		hif->shm->rx_buf_pool[i] = NULL;

		desc->data = lower_32_bits(pfe_hif_buf_dma(buf));
		desc->status = 0;
		desc->next = lower_32_bits(first_desc_p + next_idx * sizeof(struct hif_desc));
		wmb();
		desc->ctrl = BD_CTRL_PKT_INT_EN | BD_CTRL_LIFM | BD_CTRL_DIR |
			     BD_CTRL_DESC_EN | BD_BUF_LEN(pfe_pkt_size);
		desc++;
	}

	hif->RxtocleanIndex = 0;
	hif->rx_drop_pkt = false;

	writel(lower_32_bits(first_desc_p), HIF_RX_BDP_ADDR);

	hif->TxBase = hif->RxBase + hif->RxRingSize;
	tx_base_p = first_desc_p + hif->RxRingSize * sizeof(struct hif_desc);
	memset(hif->TxBase, 0, hif->TxRingSize * sizeof(struct hif_desc));
	memset(hif->tx_sw_queue, 0, sizeof(hif->tx_sw_queue));
	for (i = 0; i < hif->TxRingSize; i++)
		pfe_hif_reset_tx_desc_sw(&hif->tx_sw_queue[i]);

	desc = hif->TxBase;
	for (i = 0; i < hif->TxRingSize; i++) {
		u32 next_idx = (i + 1) & (hif->TxRingSize - 1);

		desc->next = lower_32_bits(tx_base_p + next_idx * sizeof(struct hif_desc));
		desc++;
	}

	hif->TxAvail = hif->TxRingSize;
	hif->Txtosend = 0;
	hif->Txtoclean = 0;
	hif->tx_chain_seq = 1;

	writel(lower_32_bits(tx_base_p), HIF_TX_BDP_ADDR);

	return 0;
}

static void pfe_hif_mark_client_orphaned(struct pfe_hif *hif, unsigned int client_id)
{
	unsigned int active = hif->TxRingSize - hif->TxAvail;
	unsigned int i;

	lockdep_assert_held(&hif->tx_lock);

	for (i = 0; i < active; i++) {
		unsigned int cur = (hif->Txtoclean + i) & (hif->TxRingSize - 1);

		if (hif->tx_sw_queue[cur].client_id == client_id)
			hif->tx_sw_queue[cur].client_id = HIF_CLIENTS_MAX;
	}
}

/*
 * Prove that an entire orphaned SG packet chain starting at start_q_idx with
 * length chain_len is present in the active HIF TX window with a unique
 * non-zero chain_id and that every descriptor in the chain has completed HW
 * DMA (!(ctrl & BD_CTRL_DESC_EN)) with BD_CTRL_LIFM set only on the final
 * descriptor.
 */
static bool pfe_hif_orphan_chain_done(struct pfe_hif *hif,
				      const struct hif_client_tx_queue *q,
				      unsigned int start_q_idx,
				      unsigned int chain_len)
{
	unsigned int active = hif->TxRingSize - hif->TxAvail;
	unsigned int mask = q->size - 1;
	unsigned int i;

	if (!chain_len || chain_len > PFE_TX_MAX_SG_DESCS || chain_len > active)
		return false;

	for (i = 0; i + chain_len <= active; i++) {
		unsigned int hw_idx = (hif->Txtoclean + i) & (hif->TxRingSize - 1);
		struct hif_desc_sw *sw = &hif->tx_sw_queue[hw_idx];
		u64 target_chain_id;
		unsigned int q_no, step;
		bool valid_chain = true;

		if (sw->client_id != HIF_CLIENTS_MAX ||
		    !sw->chain_id ||
		    sw->q_idx != start_q_idx)
			continue;

		target_chain_id = sw->chain_id;
		q_no = sw->q_no;

		for (step = 0; step < chain_len; step++) {
			unsigned int c_hw_idx = (hw_idx + step) & (hif->TxRingSize - 1);
			struct hif_desc_sw *c_sw = &hif->tx_sw_queue[c_hw_idx];
			struct hif_desc *c_hw = hif->TxBase + c_hw_idx;
			u32 c_ctrl = READ_ONCE(c_hw->ctrl);
			unsigned int exp_q_idx = (start_q_idx + step) & mask;
			bool is_last = (step + 1 == chain_len);

			if (c_sw->client_id != HIF_CLIENTS_MAX ||
			    c_sw->chain_id != target_chain_id ||
			    c_sw->q_no != q_no ||
			    c_sw->q_idx != exp_q_idx ||
			    (c_ctrl & BD_CTRL_DESC_EN) ||
			    (bool)(c_ctrl & BD_CTRL_LIFM) != is_last) {
				valid_chain = false;
				break;
			}
		}

		if (valid_chain)
			return true;
	}

	return false;
}

/*
 * Validate every descriptor in the full SG chain from q->read_idx through
 * the HIF_LAST_BUFFER descriptor and prove that every descriptor in that
 * chain has completed before advancing ack_idx, read_idx, or tx_pending.
 */
static void pfe_hif_orphan_sync_client_queue(struct pfe_hif *hif,
					     struct hif_client_tx_queue *q)
{
	unsigned int mask, chain_len = 0, step;
	bool found_last = false;

	if (!q || !q->base || !q->size || !is_power_of_2(q->size) || !q->tx_pending)
		return;

	if (q->ack_idx != q->read_idx)
		return;

	mask = q->size - 1;

	for (step = 0; step < q->tx_pending && step < PFE_TX_MAX_SG_DESCS; step++) {
		unsigned int q_idx = (q->read_idx + step) & mask;
		struct tx_queue_desc *q_desc = q->base + q_idx;
		u32 q_ctrl = READ_ONCE(q_desc->ctrl);
		u32 desc_flags = CL_DESC_GET_FLAGS(q_ctrl);

		if (step == 0 && !(desc_flags & HIF_FIRST_BUFFER))
			return;
		if (step > 0 && (desc_flags & HIF_FIRST_BUFFER))
			return;

		if (desc_flags & HIF_LAST_BUFFER) {
			if (!(desc_flags & HIF_DATA_VALID))
				return;
			chain_len = step + 1;
			found_last = true;
			break;
		}

		if (desc_flags & HIF_DATA_VALID)
			return;
	}

	if (!found_last || !chain_len)
		return;

	if (!pfe_hif_orphan_chain_done(hif, q, q->read_idx, chain_len))
		return;

	for (step = 0; step < chain_len; step++) {
		unsigned int q_idx = (q->read_idx + step) & mask;
		struct tx_queue_desc *q_desc = q->base + q_idx;

		q_desc->data = NULL;
		smp_wmb();
		WRITE_ONCE(q_desc->ctrl, 0);
	}

	q->ack_idx = (q->read_idx + chain_len) & mask;
	q->read_idx = (q->read_idx + chain_len) & mask;
	q->tx_pending -= chain_len;
}

static bool pfe_hif_has_orphaned_tx(struct pfe_hif *hif, int ttc, int tx_avl)
{
	int active = hif->TxRingSize - tx_avl;
	int i;

	for (i = 0; i < active; i++) {
		unsigned int cur = (ttc + i) & (hif->TxRingSize - 1);

		if (hif->tx_sw_queue[cur].client_id == HIF_CLIENTS_MAX)
			return true;
	}

	return false;
}

/*
 * Drain contiguous completed orphaned descriptors from Txtoclean: unmaps
 * DMA, releases skb once, resets desc_sw to a clean reusable state, and
 * advances Txtoclean / TxAvail together. Synchronizes any complete orphaned
 * client queue chain afterward and schedules NAPI if orphaned descriptors
 * remain in flight. Returns the number of drained descriptors.
 */
static unsigned int pfe_hif_drain_orphaned_tx(struct pfe_hif *hif,
					      struct hif_client_tx_queue *q)
{
	int ttc, tx_avl;
	unsigned int drained = 0;

	lockdep_assert_held(&hif->tx_lock);

	ttc = hif->Txtoclean;
	tx_avl = hif->TxAvail;

	while (tx_avl < hif->TxRingSize) {
		struct hif_desc_sw *desc_sw = &hif->tx_sw_queue[ttc];
		struct hif_desc *desc = hif->TxBase + ttc;
		struct sk_buff *skb;

		if (desc_sw->client_id != HIF_CLIENTS_MAX)
			break;

		if (READ_ONCE(desc->ctrl) & BD_CTRL_DESC_EN)
			break;

		dma_rmb();

		skb = desc_sw->skb;
		desc_sw->skb = NULL;
		if (unlikely(skb))
			dev_kfree_skb_any(skb);

		pfe_hif_unmap_tx_desc(hif, desc_sw);
		pfe_hif_reset_tx_desc_sw(desc_sw);
		desc->data = 0;
		desc->status = 0;
		WRITE_ONCE(desc->ctrl, 0);

		inc_hif_txidx(ttc);
		tx_avl++;
		drained++;
	}

	hif->Txtoclean = ttc;
	hif->TxAvail = tx_avl;

	pfe_hif_orphan_sync_client_queue(hif, q);

	if (pfe_hif_has_orphaned_tx(hif, ttc, tx_avl) && hif->dummy_dev)
		napi_schedule(&hif->napi);

	return drained;
}

static int pfe_hif_client_register(struct pfe_hif *hif, u32 client_id,
				    struct hif_client_shm *client_shm)
{
	struct hif_client *client = &hif->client[client_id];
	struct rx_queue_desc *rx_qbase;
	struct tx_queue_desc *tx_qbase;
	u32 i, cnt;

	spin_lock_bh(&hif->lock);
	spin_lock_bh(&hif->tx_lock);

	if (hif->shm->gClient_status & (1 << client_id)) {
		dev_err(hif->dev, "%s: client %d already registered\n", __func__, client_id);
		spin_unlock_bh(&hif->tx_lock);
		spin_unlock_bh(&hif->lock);
		return -EINVAL;
	}

	memset(client, 0, sizeof(*client));

	cnt = min_t(u32, CLIENT_CTRL_RX_Q_CNT(client_shm->ctrl), HIF_CLIENT_QUEUES_MAX);
	client->rx_qn = cnt;
	rx_qbase = (struct rx_queue_desc *)(uintptr_t)client_shm->rx_qbase;
	for (i = 0; i < cnt; i++) {
		client->rx_q[i].base = rx_qbase + i * client_shm->rx_qsize;
		client->rx_q[i].size = client_shm->rx_qsize;
		client->rx_q[i].write_idx = 0;
	}

	cnt = min_t(u32, CLIENT_CTRL_TX_Q_CNT(client_shm->ctrl), HIF_CLIENT_QUEUES_MAX);
	client->tx_qn = cnt;
	tx_qbase = (struct tx_queue_desc *)(uintptr_t)client_shm->tx_qbase;
	for (i = 0; i < cnt; i++) {
		client->tx_q[i].base = tx_qbase + i * client_shm->tx_qsize;
		client->tx_q[i].size = client_shm->tx_qsize;
		client->tx_q[i].ack_idx = 0;
	}

	hif->shm->gClient_status |= (1 << client_id);

	spin_unlock_bh(&hif->tx_lock);
	spin_unlock_bh(&hif->lock);

	return 0;
}

static void pfe_hif_client_unregister(struct pfe_hif *hif, u32 client_id)
{
	spin_lock_bh(&hif->lock);
	spin_lock_bh(&hif->tx_lock);

	if (!(hif->shm->gClient_status & (1 << client_id))) {
		dev_err(hif->dev, "%s: client %d not registered\n", __func__, client_id);
	} else {
		hif->shm->gClient_status &= ~(1 << client_id);
		memset(&hif->client[client_id], 0, sizeof(hif->client[client_id]));
		pfe_hif_mark_client_orphaned(hif, client_id);
		pfe_hif_drain_orphaned_tx(hif, NULL);
	}

	spin_unlock_bh(&hif->tx_lock);
	spin_unlock_bh(&hif->lock);

	pfe_hif_release_tx_irq(hif, client_id);
}

/*
 * Detach remaining TX descriptors and skbs from a closing client queue using
 * CL_DESC_OWN as the single owner flag:
 *  - Completed orphaned SG chains are drained and synchronized first.
 *  - For remaining pending client descriptors, if !(q_ctrl & CL_DESC_OWN) the
 *    client ring owns q_desc->data so it is freed once here; if CL_DESC_OWN
 *    is set, desc_sw owns the skb and will release it upon completion.
 */
void pfe_hif_tx_orphan_client(struct pfe_hif *hif, unsigned int client_id,
			      struct hif_client_tx_queue *q)
{
	unsigned int mask;

	lockdep_assert_held(&hif->tx_lock);

	if (unlikely(!q || !q->base || !q->size || !is_power_of_2(q->size)))
		return;

	mask = q->size - 1;
	pfe_hif_mark_client_orphaned(hif, client_id);
	pfe_hif_drain_orphaned_tx(hif, q);

	while (q->tx_pending) {
		struct tx_queue_desc *q_desc = q->base + q->read_idx;
		u32 q_ctrl = READ_ONCE(q_desc->ctrl);
		struct sk_buff *client_skb = q_desc->data;

		if (q_ctrl & CL_DESC_OWN)
			break;

		q_desc->data = NULL;
		WRITE_ONCE(q_desc->ctrl, 0);
		if (q->ack_idx == q->read_idx)
			q->ack_idx = (q->ack_idx + 1) & mask;
		q->read_idx = (q->read_idx + 1) & mask;
		q->tx_pending--;

		if (client_skb)
			dev_kfree_skb_any(client_skb);
	}
}

/*
 * Puts the Rx packet in the given client's Rx queue, swapping in a
 * freshly allocated buffer and returning the one that came out (which
 * the caller then re-posts into the HIF descriptor). Sets *queue_full
 * when the client's Rx queue has no free slot; returns NULL on either
 * queue-full or allocation/DMA-mapping failure.
 */
static void *client_put_rxpacket(struct pfe_hif *hif, void *pkt, u32 len, u32 flags,
				  u32 client_ctrl, bool *queue_full)
{
	struct hif_rx_queue *queue = &hif->client[hif->client_id].rx_q[hif->qno];
	struct rx_queue_desc *desc = queue->base + queue->write_idx;
	void *free_pkt = NULL;

	*queue_full = false;

	if (!(READ_ONCE(desc->ctrl) & CL_DESC_OWN)) {
		*queue_full = true;
		return NULL;
	}

	free_pkt = pfe_hif_spare_buf_get();
	if (!free_pkt)
		free_pkt = pfe_hif_buf_alloc(hif->dev, GFP_ATOMIC);
	if (!free_pkt)
		return NULL;

	if (!pfe_hif_buf_map(hif->dev, free_pkt)) {
		pfe_hif_spare_buf_put(free_pkt);
		return NULL;
	}

	desc->data = pkt;
	desc->client_ctrl = client_ctrl;
	smp_wmb();
	WRITE_ONCE(desc->ctrl, CL_DESC_BUF_LEN(len) | flags);
	queue->write_idx = (queue->write_idx + 1) & (queue->size - 1);

	return free_pkt;
}

static bool pfe_hif_rx_arm_desc(struct pfe_hif *hif, int rtc, void *buf)
{
	struct hif_desc *desc = hif->RxBase + rtc;

	if (!pfe_hif_buf_is_mapped(buf) && !pfe_hif_buf_map(hif->dev, buf)) {
		void *alt_buf = pfe_hif_spare_buf_get();

		if (!alt_buf || !pfe_hif_buf_map(hif->dev, alt_buf)) {
			if (alt_buf)
				pfe_hif_spare_buf_put(alt_buf);
			hif->rx_buf_addr[rtc] = buf;
			desc->data = 0;
			wmb();
			desc->ctrl = 0;
			return false;
		}

		pfe_hif_spare_buf_put(buf);
		buf = alt_buf;
	}

	hif->rx_buf_addr[rtc] = buf;
	desc->data = lower_32_bits(pfe_hif_buf_dma(buf));
	wmb();
	desc->ctrl = BD_CTRL_PKT_INT_EN | BD_CTRL_LIFM | BD_CTRL_DIR |
		     BD_CTRL_DESC_EN | BD_BUF_LEN(pfe_pkt_size);

	return true;
}

/*
 * Dequeues HIF Rx descriptors and hands completed packets to the owning
 * client's Rx queue. NAPI poll function.
 */
static int pfe_hif_rx_process(struct pfe_hif *hif, int budget, bool *remap_stalled_out)
{
	struct hif_hdr pkt_hdr;
	void *pkt_hdr_ptr;
	void *free_buf;
	int rtc, len, rx_processed = 0;
	bool retried_full = false;
	bool remap_stalled = false;
	struct hif_desc *desc;
	u32 ctrl;
	int flags;

	*remap_stalled_out = false;

	spin_lock_bh(&hif->lock);

	rtc = hif->RxtocleanIndex;

	while (rx_processed < budget) {
		bool queue_full;

		desc = hif->RxBase + rtc;
		pkt_hdr_ptr = hif->rx_buf_addr[rtc];

		/*
		 * Retry arming a descriptor whose buffer could not be mapped on
		 * a previous poll before inspecting BD_CTRL_DESC_EN.
		 */
		if (unlikely(!pfe_hif_buf_is_mapped(pkt_hdr_ptr))) {
			if (!pfe_hif_rx_arm_desc(hif, rtc, pkt_hdr_ptr)) {
				remap_stalled = true;
				break;
			}
			inc_hif_rxidx(rtc);
			continue;
		}

		ctrl = READ_ONCE(desc->ctrl);
		if (ctrl & BD_CTRL_DESC_EN) {
			writel(HIF_INT_MASK, HIF_INT_SRC);
			ctrl = READ_ONCE(desc->ctrl);
			if (ctrl & BD_CTRL_DESC_EN)
				break;
		}

		/* Ensure DMA writes to packet buffer complete before reading it. */
		dma_rmb();

		hif->napi_counters[NAPI_DESC_COUNT]++;

		len = BD_BUF_LEN(ctrl);

		pfe_hif_buf_unmap(hif->dev, pkt_hdr_ptr);

		/* Track the last HIF header received, for multi-buffer packets. */
		if (!hif->started) {
			hif->started = 1;
			memcpy(&pkt_hdr, pkt_hdr_ptr, sizeof(pkt_hdr));
			hif->qno = pkt_hdr.qNo;
			hif->client_id = pkt_hdr.client_id;
			hif->client_ctrl = (pkt_hdr.client_ctrl1 << 16) | pkt_hdr.client_ctrl;
			flags = CL_DESC_FIRST;
		} else {
			flags = 0;
		}

		if (ctrl & BD_CTRL_LIFM)
			flags |= CL_DESC_LAST;

		if (hif->rx_drop_pkt) {
			if (ctrl & BD_CTRL_LIFM) {
				hif->rx_drop_pkt = false;
				hif->started = 0;
			}
			free_buf = pkt_hdr_ptr;
			goto pkt_drop;
		}

		if (hif->client_id >= HIF_CLIENTS_MAX ||
		    !(hif->shm->gClient_status & (1 << hif->client_id))) {
			dev_err(hif->dev, "%s: packet with invalid client id %d qNo %d\n",
				__func__, hif->client_id, hif->qno);
			if (ctrl & BD_CTRL_LIFM)
				hif->started = 0;
			else
				hif->rx_drop_pkt = true;
			free_buf = pkt_hdr_ptr;
			goto pkt_drop;
		}

		if (hif->client[hif->client_id].rx_qn <= hif->qno) {
			dev_info(hif->dev, "%s: packet with invalid queue: %d\n",
				 __func__, hif->qno);
			hif->qno = 0;
		}

		free_buf = client_put_rxpacket(hif, pkt_hdr_ptr, len, flags,
					       hif->client_ctrl, &queue_full);
		if (unlikely(!free_buf)) {
			if (queue_full && !retried_full) {
				unsigned int ind_client = hif->client_id;
				unsigned int ind_qno = hif->qno;

				if (flags & CL_DESC_FIRST)
					hif->started = 0;

				retried_full = true;
				hif->napi_counters[NAPI_CLIENT_FULL_COUNT]++;
				hif->RxtocleanIndex = rtc;
				spin_unlock_bh(&hif->lock);

				hif_lib_indicate_client(ind_client, EVENT_RX_PKT_IND, ind_qno);

				spin_lock_bh(&hif->lock);
				rtc = hif->RxtocleanIndex;
				continue;
			}

			/*
			 * Either client Rx queue is still full after an event
			 * indication or buffer allocation/DMA mapping failed:
			 * recycle the current buffer back to the HIF Rx ring so
			 * HIF_RXPKT_INT cannot storm.
			 */
			if (flags & CL_DESC_FIRST)
				hif->started = 0;
			if (!(ctrl & BD_CTRL_LIFM))
				hif->rx_drop_pkt = true;
			free_buf = pkt_hdr_ptr;
			goto pkt_drop;
		}

		retried_full = false;

pkt_drop:
		if (unlikely(!pfe_hif_rx_arm_desc(hif, rtc, free_buf))) {
			WARN_ON_ONCE(1);
			if (ctrl & BD_CTRL_LIFM) {
				hif->rx_drop_pkt = false;
				hif->started = 0;
			}
			remap_stalled = true;
			break;
		}

		inc_hif_rxidx(rtc);

		if (ctrl & BD_CTRL_LIFM) {
			unsigned int ind_client = hif->client_id;
			unsigned int ind_qno = hif->qno;

			if (!(hif->client_ctrl & HIF_CTRL_RX_CONTINUED)) {
				rx_processed++;
				hif->napi_counters[NAPI_PACKET_COUNT]++;
			}
			hif->started = 0;
			hif->RxtocleanIndex = rtc;

			spin_unlock_bh(&hif->lock);
			hif_lib_indicate_client(ind_client, EVENT_RX_PKT_IND, ind_qno);
			spin_lock_bh(&hif->lock);
		}
	}

	hif->RxtocleanIndex = rtc;
	spin_unlock_bh(&hif->lock);

	*remap_stalled_out = remap_stalled;
	if (!remap_stalled)
		hif_rx_dma_start();

	return remap_stalled ? budget : rx_processed;
}

/*
 * Validate client TX descriptor position and ownership before transferring
 * skb ownership from HIF (desc_sw->skb, CL_DESC_OWN set) to the client ring
 * (desc->data, CL_DESC_OWN clear). On validation failure, leaves client
 * descriptor state untouched and returns an error so the caller can release
 * the skb cleanly and advance ring reclamation.
 */
static int client_ack_txpacket(struct pfe_hif *hif, unsigned int client_id,
			       unsigned int q_no, unsigned int q_idx,
			       struct sk_buff *skb)
{
	struct hif_tx_queue *queue;
	struct tx_queue_desc *desc;
	u32 q_ctrl, desc_flags;

	if (unlikely(client_id >= HIF_CLIENTS_MAX ||
		     !(hif->shm->gClient_status & (1 << client_id)) ||
		     q_no >= hif->client[client_id].tx_qn)) {
		dev_err_ratelimited(hif->dev,
				    "%s: invalid client %u or q %u\n",
				    __func__, client_id, q_no);
		return -EINVAL;
	}

	queue = &hif->client[client_id].tx_q[q_no];
	if (unlikely(!queue->base || !queue->size ||
		     !is_power_of_2(queue->size) || q_idx >= queue->size)) {
		dev_err_ratelimited(hif->dev,
				    "%s: invalid queue state for client %u q %u idx %u\n",
				    __func__, client_id, q_no, q_idx);
		return -EINVAL;
	}

	if (unlikely(queue->ack_idx != q_idx)) {
		dev_err_ratelimited(hif->dev,
				    "%s: stale/out-of-order q_idx %u (expected ack_idx %u) for client %u q %u\n",
				    __func__, q_idx, queue->ack_idx, client_id, q_no);
		return -ERANGE;
	}

	desc = queue->base + q_idx;
	q_ctrl = READ_ONCE(desc->ctrl);

	if (unlikely(!(q_ctrl & CL_DESC_OWN))) {
		dev_err_ratelimited(hif->dev,
				    "%s: client %u q %u idx %u not owned by HIF (ctrl=%#x)\n",
				    __func__, client_id, q_no, q_idx, q_ctrl);
		return -EINVAL;
	}

	desc_flags = CL_DESC_GET_FLAGS(q_ctrl);
	if (skb) {
		if (unlikely((desc_flags & (HIF_LAST_BUFFER | HIF_DATA_VALID)) !=
			     (HIF_LAST_BUFFER | HIF_DATA_VALID) || desc->data)) {
			dev_err_ratelimited(hif->dev,
					    "%s: client %u q %u idx %u final desc mismatch (ctrl=%#x data=%p)\n",
					    __func__, client_id, q_no, q_idx, q_ctrl, desc->data);
			return -EILSEQ;
		}
	} else {
		if (unlikely((desc_flags & (HIF_LAST_BUFFER | HIF_DATA_VALID)) ||
			     desc->data)) {
			dev_err_ratelimited(hif->dev,
					    "%s: client %u q %u idx %u non-final desc mismatch (ctrl=%#x data=%p)\n",
					    __func__, client_id, q_no, q_idx, q_ctrl, desc->data);
			return -EILSEQ;
		}
	}

	desc->data = skb;
	smp_wmb();
	WRITE_ONCE(desc->ctrl, q_ctrl & ~CL_DESC_OWN);
	queue->ack_idx = (q_idx + 1) & (queue->size - 1);

	return 0;
}

void __hif_tx_done_process(struct pfe_hif *hif, int count)
{
	struct hif_desc *desc;
	struct hif_desc_sw *desc_sw;
	int ttc, tx_avl;

	ttc = hif->Txtoclean;
	tx_avl = hif->TxAvail;

	while (tx_avl < hif->TxRingSize && count--) {
		struct sk_buff *skb;
		bool acked = false;

		desc = hif->TxBase + ttc;
		if (READ_ONCE(desc->ctrl) & BD_CTRL_DESC_EN)
			break;

		dma_rmb();

		desc_sw = &hif->tx_sw_queue[ttc];
		skb = desc_sw->skb;

		if (desc_sw->client_id < HIF_CLIENTS_MAX) {
			int ack_err = client_ack_txpacket(hif, desc_sw->client_id,
							desc_sw->q_no, desc_sw->q_idx,
							skb);
			if (likely(!ack_err)) {
				desc_sw->skb = NULL;
				acked = true;
			}
		}

		if (!acked) {
			desc_sw->skb = NULL;
			if (unlikely(skb))
				dev_kfree_skb_any(skb);
		}

		pfe_hif_unmap_tx_desc(hif, desc_sw);
		pfe_hif_reset_tx_desc_sw(desc_sw);

		inc_hif_txidx(ttc);
		tx_avl++;
	}

	hif->Txtoclean = ttc;
	hif->TxAvail = tx_avl;
}

/*
 * Program a scatter-gather descriptor chain into the HIF TX ring and start
 * TX DMA; opportunistically reclaim completed descriptors if ring room drops
 * below half.
 */
int __hif_xmit_pkt_sg(struct pfe_hif *hif, unsigned int client_id, unsigned int q_no,
		      const struct pfe_tx_sg_seg *segs, unsigned int nr_descs)
{
	struct hif_desc *first_desc;
	u32 first_ctrl = 0;
	u64 chain_id;
	unsigned int start_idx, i;

	if (unlikely(!nr_descs || nr_descs > hif->TxRingSize))
		return -EINVAL;

	if (unlikely(hif->TxAvail < nr_descs)) {
		__hif_tx_done_process(hif, TX_FREE_MAX_COUNT);
		if (hif->TxAvail < nr_descs)
			return -EBUSY;
	}

	chain_id = hif->tx_chain_seq++;
	if (unlikely(!chain_id))
		chain_id = hif->tx_chain_seq++;

	start_idx = hif->Txtosend;
	first_desc = hif->TxBase + start_idx;

	for (i = 0; i < nr_descs; i++) {
		unsigned int cur = (start_idx + i) & (hif->TxRingSize - 1);
		struct hif_desc *desc = hif->TxBase + cur;
		struct hif_desc_sw *desc_sw = &hif->tx_sw_queue[cur];
		bool is_last = (i == nr_descs - 1);
		u32 ctrl = BD_BUF_LEN(segs[i].len);

		desc_sw->data = segs[i].dma;
		desc_sw->skb = is_last ? segs[i].skb : NULL;
		desc_sw->len = segs[i].len;
		desc_sw->q_idx = segs[i].q_idx;
		desc_sw->client_id = client_id;
		desc_sw->q_no = q_no;
		desc_sw->chain_id = chain_id;
		desc_sw->map_type = segs[i].map_type;

		desc->data = lower_32_bits(segs[i].dma);
		desc->status = 0;

		if (is_last) {
			if (client_id != PFE_CL_VWD)
				ctrl |= BD_CTRL_LIFM | BD_CTRL_BRFETCH_DISABLE |
					BD_CTRL_RTFETCH_DISABLE | BD_CTRL_PARSE_DISABLE |
					BD_CTRL_PKT_INT_EN | BD_CTRL_DESC_EN;
			else
				ctrl |= BD_CTRL_LIFM | BD_CTRL_PKT_INT_EN | BD_CTRL_DESC_EN;
		} else {
			ctrl |= BD_CTRL_DESC_EN;
		}

		if (i == 0)
			first_ctrl = ctrl;
		else
			desc->ctrl = ctrl;
	}

	wmb();
	first_desc->ctrl = first_ctrl;

	hif->Txtosend = (start_idx + nr_descs) & (hif->TxRingSize - 1);
	hif->TxAvail -= nr_descs;

	hif_tx_dma_start();

	if (hif->TxAvail < (hif->TxRingSize >> 1))
		__hif_tx_done_process(hif, TX_FREE_MAX_COUNT);

	return 0;
}

/*
 * TX completion interrupts (HIF_TXPKT_INT) are enabled per client only
 * when a stopped TX queue needs wake-up notification, avoiding constant
 * TX IRQ overhead during normal transmission.
 */
void pfe_hif_request_tx_irq(struct pfe_hif *hif, unsigned int client_id)
{
	unsigned long flags;

	if (unlikely(client_id >= HIF_CLIENTS_MAX))
		return;

	spin_lock_irqsave(&hif->irq_lock, flags);
	__set_bit(client_id, &hif->tx_irq_clients);
	if (!napi_is_scheduled(&hif->napi))
		writel_relaxed(HIF_INT_MASK | HIF_TXPKT_INT, HIF_INT_ENABLE);
	spin_unlock_irqrestore(&hif->irq_lock, flags);
}

void pfe_hif_release_tx_irq(struct pfe_hif *hif, unsigned int client_id)
{
	unsigned long flags;

	if (unlikely(client_id >= HIF_CLIENTS_MAX))
		return;

	spin_lock_irqsave(&hif->irq_lock, flags);
	__clear_bit(client_id, &hif->tx_irq_clients);
	if (!napi_is_scheduled(&hif->napi)) {
		u32 mask = HIF_INT_MASK;

		if (hif->tx_irq_clients)
			mask |= HIF_TXPKT_INT;
		writel_relaxed(mask, HIF_INT_ENABLE);
	}
	spin_unlock_irqrestore(&hif->irq_lock, flags);
}

static irqreturn_t hif_isr(int irq, void *dev_id)
{
	struct pfe_hif *hif = dev_id;
	int int_status;

	int_status = readl_relaxed(HIF_INT_SRC);

	if (!(int_status & HIF_INT))
		return IRQ_NONE;

	int_status &= ~HIF_INT;

	if (int_status & (HIF_RXPKT_INT | HIF_TXPKT_INT)) {
		spin_lock(&hif->irq_lock);
		if (int_status & HIF_TXPKT_INT) {
			hif->tx_irq_count++;
			writel_relaxed(HIF_TXPKT_INT, HIF_INT_SRC);
		}
		int_status &= ~(HIF_RXPKT_INT | HIF_TXPKT_INT);

		/* Disable interrupts; the NAPI poll re-enables them. */
		writel_relaxed(0, HIF_INT_ENABLE);
		hif->napi_counters[NAPI_SCHED_COUNT]++;
		napi_schedule_irqoff(&hif->napi);
		spin_unlock(&hif->irq_lock);
	}

	if (int_status) {
		dev_info(hif->dev, "%s: unhandled interrupt: %d\n", __func__, int_status);
		writel(int_status, HIF_INT_SRC);
	}

	return IRQ_HANDLED;
}

void hif_process_client_req(struct pfe_hif *hif, int req, int data1, int data2)
{
	unsigned int client_id = data1;

	if (client_id >= HIF_CLIENTS_MAX) {
		dev_err(hif->dev, "%s: client id %d out of bounds\n", __func__, client_id);
		return;
	}

	switch (req) {
	case REQUEST_CL_REGISTER:
		pfe_hif_client_register(hif, client_id,
					 (struct hif_client_shm *)&hif->shm->client[client_id]);
		break;
	case REQUEST_CL_UNREGISTER:
		pfe_hif_client_unregister(hif, client_id);
		break;
	default:
		dev_err(hif->dev, "%s: unsupported request %d\n", __func__, req);
		break;
	}
}

static void pfe_hif_tx_poll_notify(struct pfe_hif *hif)
{
	unsigned long clients = READ_ONCE(hif->tx_irq_clients);
	unsigned int client_id;

	if (!clients)
		return;

	hif_tx_done_process(hif, TX_FREE_MAX_COUNT);

	for_each_set_bit(client_id, &clients, HIF_CLIENTS_MAX) {
		if (hif->shm->gClient_status & (1 << client_id))
			hif_lib_indicate_client(client_id, EVENT_TXDONE_IND, 0);
	}
}

static int pfe_hif_rx_poll(struct napi_struct *napi, int budget)
{
	struct pfe_hif *hif = container_of(napi, struct pfe_hif, napi);
	bool remap_stalled = false;
	int work_done;

	hif->napi_counters[NAPI_POLL_COUNT]++;

	work_done = pfe_hif_rx_process(hif, budget, &remap_stalled);
	pfe_hif_tx_poll_notify(hif);

	if (remap_stalled) {
		hif->napi_counters[NAPI_REMAP_STALLED_COUNT]++;
	} else if (work_done < budget) {
		unsigned long flags;

		spin_lock_irqsave(&hif->irq_lock, flags);
		if (napi_complete_done(napi, work_done)) {
			u32 mask = HIF_INT_MASK;

			if (hif->tx_irq_clients)
				mask |= HIF_TXPKT_INT;
			writel_relaxed(mask, HIF_INT_ENABLE);
		}
		spin_unlock_irqrestore(&hif->irq_lock, flags);
	} else {
		hif->napi_counters[NAPI_FULL_BUDGET_COUNT]++;
	}

	return work_done;
}

int pfe_hif_init(struct pfe *pfe)
{
	struct pfe_hif *hif = &pfe->hif;
	int err;

	hif->dev = pfe->dev;
	hif->irq = pfe->hif_irq;

	spin_lock_init(&hif->tx_lock);
	spin_lock_init(&hif->lock);
	spin_lock_init(&hif->irq_lock);
	hif->tx_irq_clients = 0;

	err = pfe_hif_alloc_descr(hif);
	if (err)
		return err;

	err = pfe_hif_init_buffers(hif);
	if (err) {
		dev_err(hif->dev, "%s: could not initialize buffer descriptors\n", __func__);
		goto err_descr;
	}

	/*
	 * HIF's NAPI instance is shared at the HIF level -- needs its own
	 * carrier net_device (alloc_netdev_dummy() in Linux 6.18).
	 */
	hif->dummy_dev = alloc_netdev_dummy(0);
	if (!hif->dummy_dev) {
		err = -ENOMEM;
		goto err_buffers;
	}

	netif_napi_add(hif->dummy_dev, &hif->napi, pfe_hif_rx_poll);
	napi_enable(&hif->napi);

	hif_init();
	hif_rx_enable();
	hif_tx_enable();

	writel(HIF_INT_MASK, HIF_INT_ENABLE);

	gpi_enable(HGPI_BASE_ADDR);

	err = devm_request_irq(hif->dev, hif->irq, hif_isr, 0, "pfe_hif", hif);
	if (err) {
		dev_err(hif->dev, "%s: failed to request hif IRQ = %d\n", __func__, err);
		goto err_irq;
	}

	return 0;

err_irq:
	writel(0, HIF_INT_ENABLE);
	gpi_disable(HGPI_BASE_ADDR);
	hif_rx_disable();
	hif_tx_disable();
	napi_disable(&hif->napi);
	netif_napi_del(&hif->napi);
	free_netdev(hif->dummy_dev);
err_buffers:
	pfe_hif_release_buffers(hif);
err_descr:
	pfe_hif_free_descr(hif);
	return err;
}

void pfe_hif_exit(struct pfe *pfe)
{
	struct pfe_hif *hif = &pfe->hif;
	unsigned long flags;

	hif->shm->gClient_status = 0;

	spin_lock_irqsave(&hif->irq_lock, flags);
	writel(0, HIF_INT_ENABLE);
	writel(readl(HIF_INT_SRC), HIF_INT_SRC);
	spin_unlock_irqrestore(&hif->irq_lock, flags);

	devm_free_irq(hif->dev, hif->irq, hif);

	gpi_disable(HGPI_BASE_ADDR);
	hif_rx_disable();
	hif_tx_disable();

	napi_disable(&hif->napi);

	spin_lock_irqsave(&hif->irq_lock, flags);
	writel(0, HIF_INT_ENABLE);
	writel(readl(HIF_INT_SRC), HIF_INT_SRC);
	spin_unlock_irqrestore(&hif->irq_lock, flags);

	netif_napi_del(&hif->napi);
	free_netdev(hif->dummy_dev);

	pfe_hif_release_buffers(hif);
	pfe_hif_free_descr(hif);
}