static int hif_lib_client_init_rx_buffers(struct hif_client_s *client, int q_size)
{
	struct hif_client_rx_queue *queue;
	struct rx_queue_desc *desc;
	int qno, i;

	client->rx_qbase = kvcalloc(client->rx_qn * q_size, sizeof(struct rx_queue_desc),
				    GFP_KERNEL);
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

	kvfree(client->rx_qbase);
	client->rx_qbase = NULL;
}

static int hif_lib_client_init_tx_buffers(struct hif_client_s *client, int q_size)
{
	struct hif_client_tx_queue *queue;
	int qno;

	client->tx_qbase = kvcalloc(client->tx_qn * q_size, sizeof(struct tx_queue_desc),
				    GFP_KERNEL);
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

	kvfree(client->tx_qbase);
	client->tx_qbase = NULL;
}
