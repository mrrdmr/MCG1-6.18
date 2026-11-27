// SPDX-License-Identifier: GPL-2.0
/*
 * HIF client-facing shared-memory setup and event indication -- ported
 * from the 3.2.26 vendor tree's pfe_ctrl/pfe_hif_lib.c (kmodules/
 * mspd-c2k/pfe/ in symops/MCG1-3.2.26).
 *
 * Stage P5 added the Rx buffer pool and event indication; Stage P7
 * added client registration; Stage P8 adds the TX submission path and
 * the Rx dequeue/re-arm pair (hif_lib_receive_pkt()/
 * hif_lib_event_handler_start()) -- see pfe_hif_lib.h for what's still
 * out of scope (TSO, TX credit/QoS).
 *
 * page_mode/LRO support is dropped, not deferred: it's a performance
 * feature (larger multi-page receive buffers instead of fixed 2048-byte
 * page fragments allocated via netdev_alloc_frag_align()) irrelevant to
 * reaching basic connectivity, and carrying its conditional paths
 * through pfe_hif.c/pfe_hif_lib.c would just be more surface area to
 * get wrong for no benefit at this stage. pfe_pkt_size/pfe_pkt_headroom
 * are fixed at their non-LRO values.
 */

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/slab.h>

#include "pfe_mod.h"
#include "pfe_hif.h"
#include "pfe_hif_lib.h"

unsigned int pfe_pkt_size = PFE_PKT_SIZE;
unsigned int pfe_pkt_headroom = PFE_PKT_HEADROOM;

/* HIF shared memory -- single instance, matches this SoC only ever
 * having one PFE.
 */
static struct hif_shm ghif_shm;

/* Spare Rx buffer pool (HIF_RX_DESC_NT entries) keeps allocator calls off
 * the hot RX path, with a GFP_ATOMIC fallback when empty.
 */
static void *ghif_spare_buf[HIF_RX_DESC_NT];
static int ghif_spare_buf_count;
static DEFINE_SPINLOCK(ghif_spare_buf_lock);

static int pfe_hif_spare_pool_init(struct device *dev)
{
	int i;

	for (i = 0; i < HIF_RX_DESC_NT; i++) {
		void *buf = pfe_hif_buf_alloc(dev, GFP_KERNEL);

		if (!buf) {
			while (i-- > 0)
				pfe_hif_buf_free(dev, ghif_spare_buf[i]);
			ghif_spare_buf_count = 0;
			return -ENOMEM;
		}

		ghif_spare_buf[i] = buf;
	}

	ghif_spare_buf_count = HIF_RX_DESC_NT;

	return 0;
}

static void pfe_hif_spare_pool_exit(struct device *dev)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&ghif_spare_buf_lock, flags);

	for (i = 0; i < ghif_spare_buf_count; i++)
		pfe_hif_buf_free(dev, ghif_spare_buf[i]);
	ghif_spare_buf_count = 0;

	spin_unlock_irqrestore(&ghif_spare_buf_lock, flags);
}

void *pfe_hif_spare_buf_get(void)
{
	unsigned long flags;
	void *buf = NULL;

	spin_lock_irqsave(&ghif_spare_buf_lock, flags);

	if (ghif_spare_buf_count > 0)
		buf = ghif_spare_buf[--ghif_spare_buf_count];

	spin_unlock_irqrestore(&ghif_spare_buf_lock, flags);

	return buf;
}

void pfe_hif_spare_buf_put(void *buf)
{
	unsigned long flags;

	if (unlikely(!buf))
		return;

	spin_lock_irqsave(&ghif_spare_buf_lock, flags);

	if (ghif_spare_buf_count < HIF_RX_DESC_NT)
		ghif_spare_buf[ghif_spare_buf_count++] = buf;
	else
		pfe_hif_buf_free(NULL, buf);

	spin_unlock_irqrestore(&ghif_spare_buf_lock, flags);
}

static void pfe_hif_shm_clean(struct device *dev, struct hif_shm *hif_shm)
{
	int i;
	void *pkt;

	for (i = 0; i < hif_shm->rx_buf_pool_cnt; i++) {
		pkt = (void *)hif_shm->rx_buf_pool[i];
		if (pkt) {
			hif_shm->rx_buf_pool[i] = NULL;
			pfe_hif_buf_free(dev, pkt);
		}
	}
}

/*
 * Allocate the Rx buffer pool the HIF Rx ring is filled from. Must run
 * before pfe_hif_init() (pfe_hif_init_buffers() checks rx_buf_pool_cnt).
 */
static int pfe_hif_shm_init(struct device *dev, struct hif_shm *hif_shm)
{
	void *pkt;
	int i;

	memset(hif_shm, 0, sizeof(*hif_shm));
	hif_shm->rx_buf_pool_cnt = HIF_RX_DESC_NT;

	for (i = 0; i < hif_shm->rx_buf_pool_cnt; i++) {
		pkt = pfe_hif_buf_alloc(dev, GFP_KERNEL);
		if (!pkt)
			goto err;

		hif_shm->rx_buf_pool[i] = pkt;
	}

	return 0;

err:
	pr_err("%s: low memory\n", __func__);
	pfe_hif_shm_clean(dev, hif_shm);
	return -ENOMEM;
}

void hif_lib_indicate_client(int client_id, int event_type, int qno)
{
	struct hif_client_s *client;

	if (unlikely(client_id < 0 || client_id >= HIF_CLIENTS_MAX ||
		     event_type < 0 || event_type >= HIF_EVENT_MAX ||
		     qno < 0 || qno >= HIF_CLIENT_QUEUES_MAX))
		return;

	client = READ_ONCE(g_pfe->hif_client[client_id]);
	if (!client)
		return;

	if (!test_and_set_bit(qno, &client->queue_mask[event_type]))
		client->event_handler(client->priv, event_type, qno);
}

static int hif_lib_event_dummy(void *priv, int event_type, int qno)
{
	return 0;
}

static int hif_lib_client_init_rx_buffers(struct hif_client_s *client, int q_size)
{
	struct hif_client_rx_queue *queue;
	struct rx_queue_desc *desc;
	int qno, i;

	client->rx_qbase = kcalloc(client->rx_qn * q_size, sizeof(struct rx_queue_desc), GFP_KERNEL);
	if (!client->rx_qbase)
		return -ENOMEM;

	for (qno = 0; qno < client->rx_qn; qno++) {
		queue = &client->rx_q[qno];
		queue->base = (struct rx_queue_desc *)client->rx_qbase + qno * q_size;
		queue->size = q_size;
		queue->read_idx = 0;
		queue->write_idx = 0;
	}

	for (qno = 0; qno < client->rx_qn; qno++) {
		queue = &client->rx_q[qno];
		desc = queue->base;

		for (i = 0; i < queue->size; i++, desc++)
			desc->ctrl = CL_DESC_BUF_LEN(pfe_pkt_size) | CL_DESC_OWN;
	}

	return 0;
}

static void hif_lib_client_release_rx_buffers(struct hif_client_s *client)
{
	struct rx_queue_desc *desc;
	int qno, i;
	void *buf;

	if (!client->rx_qbase)
		return;

	for (qno = 0; qno < client->rx_qn; qno++) {
		desc = client->rx_q[qno].base;

		for (i = 0; i < client->rx_q[qno].size; i++, desc++) {
			buf = desc->data;
			if (buf)
				pfe_hif_spare_buf_put(buf);
		}
	}

	kfree(client->rx_qbase);
	client->rx_qbase = NULL;
}

static int hif_lib_client_init_tx_buffers(struct hif_client_s *client, int q_size)
{
	struct hif_client_tx_queue *queue;
	int qno;

	client->tx_qbase = kcalloc(client->tx_qn * q_size, sizeof(struct tx_queue_desc), GFP_KERNEL);
	if (!client->tx_qbase)
		return -ENOMEM;

	for (qno = 0; qno < client->tx_qn; qno++) {
		queue = &client->tx_q[qno];
		queue->base = (struct tx_queue_desc *)client->tx_qbase + qno * q_size;
		queue->size = q_size;
		queue->read_idx = 0;
		queue->write_idx = 0;
		queue->tx_pending = 0;
	}

	return 0;
}

static void hif_lib_client_release_tx_buffers(struct hif_client_s *client)
{
	int qno;

	if (!client->tx_qbase)
		return;

	for (qno = 0; qno < client->tx_qn; qno++) {
		if (client->tx_q[qno].tx_pending)
			pr_err("%s: client %d queue %d has pending tx packets\n",
			       __func__, client->id, qno);
	}

	kfree(client->tx_qbase);
	client->tx_qbase = NULL;
}

int hif_lib_client_register(struct hif_client_s *client)
{
	struct hif_client_shm *client_shm;
	struct hif_shm *hif_shm;
	int rc;

	if (!client || !client->pfe || client->id < 0 ||
	    client->id >= HIF_CLIENTS_MAX || g_pfe->hif_client[client->id])
		return -EINVAL;

	hif_shm = client->pfe->hif.shm;

	rc = hif_lib_client_init_rx_buffers(client, client->rx_qsize);
	if (rc)
		return rc;

	rc = hif_lib_client_init_tx_buffers(client, client->tx_qsize);
	if (rc) {
		hif_lib_client_release_rx_buffers(client);
		return rc;
	}

	if (!client->event_handler)
		client->event_handler = hif_lib_event_dummy;

	client_shm = &hif_shm->client[client->id];
	client_shm->rx_qbase = (u32)(uintptr_t)client->rx_qbase;
	client_shm->rx_qsize = client->rx_qsize;
	client_shm->tx_qbase = (u32)(uintptr_t)client->tx_qbase;
	client_shm->tx_qsize = client->tx_qsize;
	client_shm->ctrl = (client->tx_qn << CLIENT_CTRL_TX_Q_CNT_OFST) |
			   (client->rx_qn << CLIENT_CTRL_RX_Q_CNT_OFST);

	memset(client->queue_mask, 0, sizeof(client->queue_mask));

	WRITE_ONCE(g_pfe->hif_client[client->id], client);

	hif_process_client_req(&client->pfe->hif, REQUEST_CL_REGISTER, client->id, 0);

	return 0;
}

int hif_lib_client_unregister(struct hif_client_s *client)
{
	struct pfe *pfe;

	if (!client || !client->pfe || client->id < 0 || client->id >= HIF_CLIENTS_MAX)
		return -EINVAL;

	pfe = client->pfe;

	hif_process_client_req(&pfe->hif, REQUEST_CL_UNREGISTER, client->id, 0);

	WRITE_ONCE(g_pfe->hif_client[client->id], NULL);
	napi_synchronize(&pfe->hif.napi);

	hif_lib_client_release_tx_buffers(client);
	hif_lib_client_release_rx_buffers(client);

	return 0;
}

int hif_lib_tmu_queue_start(struct hif_client_s *client, int qno)
{
	return 0;
}

int hif_lib_tmu_queue_stop(struct hif_client_s *client, int qno)
{
	return 0;
}

static inline void hif_hdr_write(struct hif_hdr *pkt_hdr, unsigned int client_id,
				  unsigned int qno, u32 client_ctrl)
{
	pkt_hdr->client_id = client_id;
	pkt_hdr->qNo = qno;
	pkt_hdr->client_ctrl = client_ctrl & 0xFFFF;
	pkt_hdr->client_ctrl1 = client_ctrl >> 16;
}

unsigned int hif_lib_tx_avail(struct hif_client_s *client, unsigned int qno)
{
	struct hif_client_tx_queue *queue;
	struct pfe_hif *hif;
	unsigned int sw_avail, hw_avail;

	if (unlikely(!client || !client->pfe || qno >= client->tx_qn))
		return 0;

	queue = &client->tx_q[qno];
	if (unlikely(!queue->base))
		return 0;

	hif = &client->pfe->hif;

	spin_lock_bh(&hif->tx_lock);
	sw_avail = (queue->tx_pending < queue->size) ?
		   (queue->size - queue->tx_pending) : 0;
	hw_avail = hif->TxAvail;
	spin_unlock_bh(&hif->tx_lock);

	return min(sw_avail, hw_avail);
}

/*
 * Prepend the HIF header onto head_data, DMA-map the linear head, and
 * enqueue the head plus pre-mapped scatter-gather segments into the client
 * and HIF TX rings under hif->tx_lock with BQL accounting.
 */
int hif_lib_xmit_pkt_sg(struct hif_client_s *client, unsigned int qno,
			void *head_data, unsigned int head_len,
			const struct pfe_tx_sg_seg *frag_segs,
			unsigned int nr_frags, u32 client_ctrl,
			void *client_data, struct net_device *dev,
			unsigned int pkt_len)
{
	struct hif_client_tx_queue *queue;
	struct pfe_hif *hif;
	struct pfe_tx_sg_seg segs[PFE_TX_MAX_SG_DESCS];
	unsigned int nr_descs = 1 + nr_frags;
	unsigned int i, widx;
	int rc;

	if (unlikely(!client || !client->pfe || qno >= client->tx_qn))
		return -EINVAL;

	queue = &client->tx_q[qno];
	if (unlikely(!queue->base))
		return -EINVAL;

	hif = &client->pfe->hif;

	if (unlikely(nr_descs > PFE_TX_MAX_SG_DESCS || nr_descs > queue->size))
		return -EINVAL;

	head_data -= sizeof(struct hif_hdr);
	head_len += sizeof(struct hif_hdr);

	if (unlikely(head_len > PFE_HIF_MAX_BD_LEN))
		return -EINVAL;

	hif_hdr_write(head_data, client->id, qno, client_ctrl);

	segs[0].dma = dma_map_single(hif->dev, head_data, head_len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(hif->dev, segs[0].dma)))
		return -ENOMEM;
	segs[0].len = head_len;
	segs[0].map_type = PFE_TX_MAP_SINGLE;

	for (i = 0; i < nr_frags; i++)
		segs[i + 1] = frag_segs[i];

	spin_lock_bh(&hif->tx_lock);

	if (unlikely(queue->tx_pending + nr_descs > queue->size)) {
		spin_unlock_bh(&hif->tx_lock);
		dma_unmap_single(hif->dev, segs[0].dma, segs[0].len, DMA_TO_DEVICE);
		return -EBUSY;
	}

	widx = queue->write_idx;
	for (i = 0; i < nr_descs; i++) {
		unsigned int cur = (widx + i) & (queue->size - 1);
		struct tx_queue_desc *desc = queue->base + cur;
		unsigned int desc_flags = 0;

		if (i == 0)
			desc_flags |= HIF_FIRST_BUFFER;
		if (i == nr_descs - 1)
			desc_flags |= HIF_LAST_BUFFER | HIF_DATA_VALID;

		desc->data = client_data;
		WRITE_ONCE(desc->ctrl, CL_DESC_OWN | CL_DESC_FLAGS(desc_flags));
	}

	if (dev)
		netdev_sent_queue(dev, pkt_len);

	rc = __hif_xmit_pkt_sg(hif, client->id, qno, segs, nr_descs);
	if (unlikely(rc)) {
		if (dev)
			netdev_completed_queue(dev, 1, pkt_len);
		for (i = 0; i < nr_descs; i++) {
			unsigned int cur = (widx + i) & (queue->size - 1);

			WRITE_ONCE(queue->base[cur].ctrl, 0);
			queue->base[cur].data = NULL;
		}
		spin_unlock_bh(&hif->tx_lock);
		dma_unmap_single(hif->dev, segs[0].dma, segs[0].len, DMA_TO_DEVICE);
		return rc;
	}

	queue->write_idx = (widx + nr_descs) & (queue->size - 1);
	queue->tx_pending += nr_descs;
	queue->jiffies_last_packet = jiffies;

	spin_unlock_bh(&hif->tx_lock);

	return 0;
}

void *hif_lib_tx_get_next_complete(struct hif_client_s *client, int qno,
				    unsigned int *flags, int count)
{
	struct hif_client_tx_queue *queue;
	struct tx_queue_desc *desc;
	struct pfe_hif *hif;
	u32 q_ctrl;
	void *data;

	if (unlikely(!client || !client->pfe || qno < 0 || qno >= client->tx_qn))
		return NULL;

	queue = &client->tx_q[qno];
	if (unlikely(!queue->base))
		return NULL;

	desc = queue->base + queue->read_idx;
	hif = &client->pfe->hif;

	spin_lock_bh(&hif->tx_lock);

	if (!queue->tx_pending) {
		spin_unlock_bh(&hif->tx_lock);
		return NULL;
	}

	q_ctrl = READ_ONCE(desc->ctrl);
	if (q_ctrl & CL_DESC_OWN) {
		__hif_tx_done_process(hif, count);

		q_ctrl = READ_ONCE(desc->ctrl);
		if (q_ctrl & CL_DESC_OWN) {
			spin_unlock_bh(&hif->tx_lock);
			return NULL;
		}
	}

	data = desc->data;
	desc->data = NULL;
	queue->read_idx = (queue->read_idx + 1) & (queue->size - 1);
	queue->tx_pending--;
	*flags = CL_DESC_GET_FLAGS(q_ctrl);

	spin_unlock_bh(&hif->tx_lock);

	return data;
}

void *hif_lib_receive_pkt(struct hif_client_s *client, int qno, int *len, int *ofst,
			   unsigned int *rx_ctrl, unsigned int *desc_ctrl, void **priv_data)
{
	struct hif_client_rx_queue *queue;
	struct rx_queue_desc *desc;
	void *pkt = NULL;
	u32 ctrl;

	if (unlikely(!client || qno < 0 || qno >= client->rx_qn))
		return NULL;

	queue = &client->rx_q[qno];
	if (unlikely(!queue->base))
		return NULL;

	desc = queue->base + queue->read_idx;

	ctrl = READ_ONCE(desc->ctrl);
	if (ctrl & CL_DESC_OWN)
		return NULL;

	/* Pairs with smp_wmb() in client_put_rxpacket(). */
	smp_rmb();

	if (unlikely(!desc->data))
		return NULL;

	pkt = desc->data - pfe_pkt_headroom;

	*rx_ctrl = desc->client_ctrl;
	*desc_ctrl = ctrl;

	if (ctrl & CL_DESC_FIRST) {
		u16 size = *rx_ctrl >> 24;

		if (size) {
			*len = CL_DESC_BUF_LEN(ctrl) - PFE_PKT_HEADER_SZ - size;
			*ofst = pfe_pkt_headroom + PFE_PKT_HEADER_SZ + size;
			*priv_data = desc->data + PFE_PKT_HEADER_SZ;
		} else {
			*len = CL_DESC_BUF_LEN(ctrl) - PFE_PKT_HEADER_SZ;
			*ofst = pfe_pkt_headroom + PFE_PKT_HEADER_SZ;
			*priv_data = NULL;
		}
	} else {
		*len = CL_DESC_BUF_LEN(ctrl);
		*ofst = pfe_pkt_headroom;
	}

	desc->data = NULL;
	smp_wmb();

	WRITE_ONCE(desc->ctrl, CL_DESC_BUF_LEN(pfe_pkt_size) | CL_DESC_OWN);
	queue->read_idx = (queue->read_idx + 1) & (queue->size - 1);

	return pkt;
}

int hif_lib_event_handler_start(struct hif_client_s *client, int event, int qno)
{
	if (!client || event < 0 || event >= HIF_EVENT_MAX ||
	    qno < 0 || qno >= HIF_CLIENT_QUEUES_MAX)
		return -1;

	test_and_clear_bit(qno, &client->queue_mask[event]);

	switch (event) {
	case EVENT_RX_PKT_IND: {
		struct hif_client_rx_queue *queue;
		struct rx_queue_desc *desc;

		if (qno >= client->rx_qn)
			return -1;

		queue = &client->rx_q[qno];
		if (unlikely(!queue->base))
			return -1;

		desc = queue->base + queue->read_idx;

		if (!(READ_ONCE(desc->ctrl) & CL_DESC_OWN))
			hif_lib_indicate_client(client->id, EVENT_RX_PKT_IND, qno);
		break;
	}
	default:
		break;
	}

	return 0;
}

int pfe_hif_lib_init(struct pfe *pfe)
{
	int rc;

	pfe->hif.shm = &ghif_shm;
	rc = pfe_hif_shm_init(pfe->dev, pfe->hif.shm);
	if (rc)
		return rc;

	rc = pfe_hif_spare_pool_init(pfe->dev);
	if (rc) {
		pfe_hif_shm_clean(pfe->dev, pfe->hif.shm);
		return rc;
	}

	dev_info(pfe->dev, "pfe_hif_lib_init: pkt size %u, rx buffers %u\n",
		 pfe_pkt_size, HIF_RX_DESC_NT);

	return 0;
}

void pfe_hif_lib_exit(struct pfe *pfe)
{
	pfe_hif_spare_pool_exit(pfe->dev);
	pfe_hif_shm_clean(pfe->dev, pfe->hif.shm);
}