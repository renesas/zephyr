/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcarsoc_eth_tsnes

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_tsnes, CONFIG_ETHERNET_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/cache.h>

#include "eth_renesas_rcarsoc_tsnes_priv.h"

#define ETH_TSNES_INIT_DESC_RING(n)                                                                \
	static struct eth_tsnes_ext_desc_with_ts                                                   \
		eth_tsnes_##n##_rx_desc_ring[DT_INST_PROP(n, rx_queues) *                          \
					     DT_INST_PROP(n, rx_queue_size)] ETH_RAM_SECTION       \
		__aligned(CONFIG_ETH_RENESAS_RCARSOC_TSNES_DESCRIPTOR_CACHE_ALIGN);                \
	static struct eth_tsnes_ext_desc                                                           \
		eth_tsnes_##n##_tx_desc_ring[DT_INST_PROP(n, tx_queues) *                          \
					     DT_INST_PROP(n, tx_queue_size)] ETH_RAM_SECTION       \
		__aligned(CONFIG_ETH_RENESAS_RCARSOC_TSNES_DESCRIPTOR_CACHE_ALIGN);

#define ETH_TSNES_INIT_BUF_NODES(n)                                                                \
	static struct eth_tsnes_buf_mgmt                                                           \
		eth_tsnes_##n##_rx_buf_nodes[DT_INST_PROP(n, rx_queues) *                          \
					     DT_INST_PROP(n, rx_queue_size)] ETH_RAM_SECTION       \
		__aligned(CONFIG_ETH_RENESAS_RCARSOC_TSNES_DESCRIPTOR_CACHE_ALIGN);                \
	static struct eth_tsnes_buf_mgmt                                                           \
		eth_tsnes_##n##_tx_buf_nodes[DT_INST_PROP(n, tx_queues) *                          \
					     DT_INST_PROP(n, tx_queue_size)] ETH_RAM_SECTION       \
		__aligned(CONFIG_ETH_RENESAS_RCARSOC_TSNES_DESCRIPTOR_CACHE_ALIGN);

#define ETH_TSNES_INIT_BUF_POOL(n)                                                                 \
	static uint8_t eth_tsnes_##n##_buf_pool                                                    \
		[CONFIG_ETH_RENESAS_RCARSOC_TSNES_BUFFER_POOL_SIZE] ETH_RAM_SECTION                \
		__aligned(CONFIG_ETH_RENESAS_RCARSOC_TSNES_DESCRIPTOR_CACHE_ALIGN);

#define ETH_TSNES_RX_DESC_RING_PTR(idx, n)                                                         \
	&eth_tsnes_##n##_rx_desc_ring[(idx) * DT_INST_PROP(n, rx_queue_size)]
#define ETH_TSNES_TX_DESC_RING_PTR(idx, n)                                                         \
	&eth_tsnes_##n##_tx_desc_ring[(idx) * DT_INST_PROP(n, tx_queue_size)]
#define ETH_TSNES_RX_BUF_NODES_PTR(idx, n)                                                         \
	&eth_tsnes_##n##_rx_buf_nodes[(idx) * DT_INST_PROP(n, rx_queue_size)]
#define ETH_TSNES_TX_BUF_NODES_PTR(idx, n)                                                         \
	&eth_tsnes_##n##_tx_buf_nodes[(idx) * DT_INST_PROP(n, tx_queue_size)]

#define ETH_TSNES_INIT_DESC_CHAIN(n)                                                               \
	static struct eth_tsnes_ext_desc eth_tsnes_##n##_init_desc_chain[17] ETH_RAM_SECTION       \
		__aligned(CONFIG_ETH_RENESAS_RCARSOC_TSNES_DESCRIPTOR_CACHE_ALIGN);
#define ETH_TSNES_INIT_DUMMY_BUF(n)                                                                \
	static uint8_t eth_tsnes_##n##_init_dummy_buf[2048] ETH_RAM_SECTION                        \
		__aligned(CONFIG_ETH_RENESAS_RCARSOC_TSNES_DESCRIPTOR_CACHE_ALIGN);

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
#define ETH_TSNES_INIT_TIMESTAMP_DESC_RING(n)                                                      \
	static struct eth_tsnes_basic_desc_with_ts                                                 \
		eth_tsnes_##n##_ts_desc_ring[ETH_MAX_TS_DESC_NUM + 1] ETH_RAM_SECTION              \
		__aligned(CONFIG_ETH_RENESAS_RCARSOC_TSNES_DESCRIPTOR_CACHE_ALIGN);

#define ETH_TSNES_INIT_TS_PKT_NODES(n)                                                             \
	static struct eth_tsnes_ts_pkt_mgmt                                                        \
		eth_tsnes_##n##_ts_pkt_nodes[ETH_MAX_TS_DESC_NUM] ETH_RAM_SECTION                  \
		__aligned(CONFIG_ETH_RENESAS_RCARSOC_TSNES_DESCRIPTOR_CACHE_ALIGN);
#else
#define ETH_TSNES_INIT_TIMESTAMP_DESC_RING(n)
#define ETH_TSNES_INIT_TS_PKT_NODES(n)
#endif

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_ENABLE_DMA_CACHE_OPS
#define TSNES_CACHE_LINE_SIZE 64

static inline void tsnes_dcache_invalidate(uintptr_t addr, size_t size)
{
	sys_cache_data_flush_and_invd_range((void *)addr, size);
}

static inline void tsnes_dcache_flush(uintptr_t addr, size_t size)
{
	sys_cache_data_flush_range((void *)addr, size);
}

static inline size_t tsnes_cache_align_size(size_t size)
{
	return (size + TSNES_CACHE_LINE_SIZE - 1) & ~(TSNES_CACHE_LINE_SIZE - 1);
}
#else
#define tsnes_dcache_invalidate(addr, size)                                                        \
	do {                                                                                       \
	} while (0)
#define tsnes_dcache_flush(addr, size)                                                             \
	do {                                                                                       \
	} while (0)
#define tsnes_cache_align_size(size) (size)
#endif

struct eth_tsnes_rx_dma_meta {
	struct eth_tsnes_queue *queue;
	uint32_t desc_idx;
};

static void eth_tsnes_rx_dma_destroy(struct net_buf *buf)
{
	struct eth_tsnes_rx_dma_meta *meta = net_buf_user_data(buf);
	struct eth_tsnes_ext_desc_with_ts *ring;
	struct eth_tsnes_ext_desc_with_ts *desc;
	k_spinlock_key_t key;

	if (meta && meta->queue) {
		key = k_spin_lock(&meta->queue->lock);
		ring = (struct eth_tsnes_ext_desc_with_ts *)meta->queue->desc_ring;
		desc = &ring[meta->desc_idx];

		desc->info_ds = MIN(meta->queue->buf_nodes[meta->desc_idx].buf_size, 0xFFF);
		desc->die_dt = ETH_DESC_FEMPTY | ETH_DESC_DIE;
		desc->info = 0;
		desc->info1[0] = 0;
		desc->info1[1] = 0;
		desc->tsns = 0;
		desc->tss = 0;

		tsnes_dcache_flush((uintptr_t)desc, sizeof(*desc));
		k_spin_unlock(&meta->queue->lock, key);
	}

	net_buf_destroy(buf);
}

NET_BUF_POOL_FIXED_DEFINE(eth_tsnes_rx_dma_pool, 256, sizeof(struct eth_tsnes_rx_dma_meta),
			  sizeof(struct eth_tsnes_rx_dma_meta), eth_tsnes_rx_dma_destroy);

static int eth_tsnes_set_mac_loopback(const struct eth_tsnes_config *cfg, bool enable)
{
	volatile eth_tsnes_rmac_reg_t *rmac = &cfg->tsnes_base->stRMACSys.stRMAC;

	if (enable) {
		rmac->MLBC = 0x00000001UL;
		rmac->MRAFC |= 0x01010101UL; /* Allow DA=SA frames (anti-spoofing bypass) */
		LOG_INF("MAC Loopback enabled");
	} else {
		rmac->MLBC = 0x00000000UL;
		rmac->MRAFC &= ~0x01010101UL;
		LOG_INF("MAC Loopback disabled");
	}

	return 0;
}

static int eth_tsnes_alloc_buffer_pool(const struct device *dev)
{
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;

	data->max_frame_size = CONFIG_ETH_RENESAS_RCARSOC_TSNES_MAX_FRAME_SIZE;
	data->buf_pool_size = CONFIG_ETH_RENESAS_RCARSOC_TSNES_BUFFER_POOL_SIZE;
	data->buf_pool = cfg->buf_pool;

	memset(data->buf_pool, 0, data->buf_pool_size);
	data->buf_pool_used = 0;

	return 0;
}

static uint8_t *eth_tsnes_alloc_buffer(struct eth_tsnes_data *data, uint32_t size)
{
	uint8_t *buf = NULL;
	uint32_t aligned_size;

	if (!data->buf_pool) {
		return NULL;
	}

	aligned_size = tsnes_cache_align_size(size);

	if (data->buf_pool_used + aligned_size > data->buf_pool_size) {
		return NULL;
	}

	buf = &data->buf_pool[data->buf_pool_used];
	data->buf_pool_used += aligned_size;

	return buf;
}

static int eth_tsnes_alloc_queue_descriptors(const struct device *dev,
					     struct eth_tsnes_queue *queue, uint32_t count,
					     size_t desc_size, void *desc_ring,
					     struct eth_tsnes_buf_mgmt *buf_nodes)
{
	queue->desc_ring = desc_ring;
	queue->buf_nodes = buf_nodes;
	queue->desc_count = count;
	queue->head_idx = 0;
	queue->tail_idx = 0;

	memset(queue->desc_ring, 0, count * desc_size);
	if (buf_nodes != NULL) {
		memset(queue->buf_nodes, 0, count * sizeof(struct eth_tsnes_buf_mgmt));
	}

	return 0;
}

static int eth_tsnes_alloc_queues(const struct device *dev)
{
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;
	int i, ret;

	data->num_rx_queues = cfg->num_rx_queues;
	data->num_tx_queues = cfg->num_tx_queues;

	for (i = 0; i < data->num_rx_queues; i++) {
		ret = eth_tsnes_alloc_queue_descriptors(
			dev, &data->rx_queues[i], cfg->rx_queue_size,
			sizeof(struct eth_tsnes_ext_desc_with_ts), cfg->rx_desc_rings[i],
			cfg->rx_buf_nodes[i]);
		if (ret) {
			return ret;
		}
	}

	for (i = 0; i < data->num_tx_queues; i++) {
		ret = eth_tsnes_alloc_queue_descriptors(
			dev, &data->tx_queues[i], cfg->tx_queue_size,
			sizeof(struct eth_tsnes_ext_desc), cfg->tx_desc_rings[i],
			cfg->tx_buf_nodes[i]);
		if (ret) {
			return ret;
		}
	}

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
	ret = eth_tsnes_alloc_queue_descriptors(dev, &data->ts_queues[0], ETH_MAX_TS_DESC_NUM + 1,
						sizeof(struct eth_tsnes_basic_desc_with_ts),
						cfg->ts_desc_rings[0], NULL);
	if (ret) {
		return ret;
	}
#endif

	return 0;
}

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
static inline bool eth_tsnes_should_capture_tx_ts(struct net_pkt *pkt)
{
	return (net_ntohs(NET_ETH_HDR(pkt)->type) == NET_ETH_PTYPE_PTP) ||
	       net_pkt_is_tx_timestamping(pkt);
}
#endif

static int eth_tsnes_mhd_mode_setting(volatile eth_tsnes_mhd_reg_t *mhd, uint32_t mode)
{
	mhd->OCR = mode;

	if (!WAIT_FOR((mhd->OSR & (1UL << mode)) != 0, 1000, k_busy_wait(1))) {
		LOG_ERR("Failed to switch to MHD Operation Mode");
		return -ETIMEDOUT;
	}
	return 0;
}

static int eth_tsnes_wait_ram_reset(volatile eth_tsnes_axibmi_reg_t *axi)
{
	if (!WAIT_FOR(axi->RR == (ETH_TSNES_RATRR_RESET | ETH_TSNES_TATRR_RESET), 10000,
		      k_busy_wait(1))) {
		return -ETIMEDOUT;
	}
	return 0;
}

static int eth_tsnes_tx_desc_learning(volatile eth_tsnes_axibmi_reg_t *axi, uint32_t queue_idx,
				      uint32_t desc_addr)
{
	bool ok;

	axi->TATLS0 = ETH_TSNES_TATLS0_CONFIG | (queue_idx << 24);
	axi->TATLS1 = desc_addr;
	axi->TATLR = ETH_TSNES_TATLR_LEARNING;

	ok = WAIT_FOR(!(axi->TATLR & ETH_TSNES_TATLR_LEARNING), 1000, k_busy_wait(1));

	if (!ok || (axi->TATLR & ETH_TSNES_TATLR_FAILED)) {
		return -EIO;
	}
	return 0;
}

static int eth_tsnes_rx_desc_learning(volatile eth_tsnes_axibmi_reg_t *axi, uint32_t queue_idx,
				      uint32_t desc_addr)
{
	bool ok;

	axi->RATLS0 = ETH_TSNES_RATLS0_CONFIG | (queue_idx << 24);
	axi->RATLS1 = desc_addr;
	axi->RATLR = ETH_TSNES_RATLR_LEARNING;

	ok = WAIT_FOR(!(axi->RATLR & ETH_TSNES_RATLR_LEARNING), 1000, k_busy_wait(1));

	if (!ok || (axi->RATLR & ETH_TSNES_RATLR_FAILED)) {
		return -EIO;
	}
	return 0;
}

static int eth_tsnes_setup_tx_descriptors(const struct device *dev)
{
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;
	volatile eth_tsnes_axibmi_reg_t *axi = &cfg->tsnes_base->stAXIBMI;
	int i, j;

	for (i = 0; i < data->num_tx_queues; i++) {
		struct eth_tsnes_queue *queue = &data->tx_queues[i];
		struct eth_tsnes_ext_desc *ring = (struct eth_tsnes_ext_desc *)queue->desc_ring;
		struct eth_tsnes_ext_desc *desc;
		uint8_t *buf;

		for (j = 0; j < queue->desc_count; j++) {
			desc = &ring[j];
			buf = eth_tsnes_alloc_buffer(data, data->max_frame_size);
			if (!buf) {
				return -ENOMEM;
			}
			desc->die_dt = ETH_DESC_FEMPTY;
			desc->info_ds = 0;
			desc->info = 0;
			desc->dptr = (uint32_t)(uintptr_t)buf;
			desc->info1[0] = 0;
			desc->info1[1] = 0;

			queue->buf_nodes[j].buf_addr = (uintptr_t)buf;
			queue->buf_nodes[j].buf_size = data->max_frame_size;
		}
		/* Link last to first */
		ring[queue->desc_count - 1].die_dt = ETH_DESC_LINK;
		ring[queue->desc_count - 1].info_ds = 0;
		ring[queue->desc_count - 1].info = 0;
		ring[queue->desc_count - 1].dptr = (uint32_t)(uintptr_t)&ring[0];
		ring[queue->desc_count - 1].info1[0] = 0;
		ring[queue->desc_count - 1].info1[1] = 0;

		tsnes_dcache_flush((uintptr_t)ring,
				   queue->desc_count * sizeof(struct eth_tsnes_ext_desc));

		if (eth_tsnes_tx_desc_learning(axi, i, (uint32_t)(uintptr_t)&ring[0])) {
			LOG_ERR("TX learning failed for queue %d", i);
			return -EIO;
		}
	}
	return 0;
}

static int eth_tsnes_setup_rx_descriptors(const struct device *dev)
{
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;
	volatile eth_tsnes_axibmi_reg_t *axi = &cfg->tsnes_base->stAXIBMI;
	int i, j;

	for (i = 0; i < data->num_rx_queues; i++) {
		struct eth_tsnes_queue *queue = &data->rx_queues[i];
		struct eth_tsnes_ext_desc_with_ts *ring =
			(struct eth_tsnes_ext_desc_with_ts *)queue->desc_ring;
		struct eth_tsnes_ext_desc_with_ts *desc;
		uint8_t *buf;

		for (j = 0; j < queue->desc_count; j++) {
			desc = &ring[j];
			buf = eth_tsnes_alloc_buffer(data, data->max_frame_size);
			if (!buf) {
				return -ENOMEM;
			}

			desc->die_dt = ETH_DESC_FEMPTY | ETH_DESC_DIE;
			desc->info_ds = MIN(data->max_frame_size, 0xFFF);
			desc->info = 0;
			desc->dptr = (uint32_t)(uintptr_t)buf;
			desc->info1[0] = 0;
			desc->info1[1] = 0;
			desc->tsns = 0;
			desc->tss = 0;

			queue->buf_nodes[j].buf_addr = (uintptr_t)buf;
			queue->buf_nodes[j].buf_size = data->max_frame_size;
		}

		/* Link last to first */
		ring[queue->desc_count - 1].die_dt = ETH_DESC_LINK;
		ring[queue->desc_count - 1].info_ds = 0;
		ring[queue->desc_count - 1].info = 0;
		ring[queue->desc_count - 1].dptr = (uint32_t)(uintptr_t)&ring[0];
		ring[queue->desc_count - 1].info1[0] = 0;
		ring[queue->desc_count - 1].info1[1] = 0;
		ring[queue->desc_count - 1].tsns = 0;
		ring[queue->desc_count - 1].tss = 0;

		tsnes_dcache_flush((uintptr_t)ring,
				   queue->desc_count * sizeof(struct eth_tsnes_ext_desc_with_ts));

		if (eth_tsnes_rx_desc_learning(axi, i, (uint32_t)(uintptr_t)&ring[0])) {
			LOG_ERR("RX learning failed for queue %d", i);
			return -EIO;
		}
	}
	return 0;
}

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
static int eth_tsnes_setup_ts_descriptors(const struct device *dev)
{
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;
	volatile eth_tsnes_axibmi_reg_t *axi = &cfg->tsnes_base->stAXIBMI;
	int i;

	struct eth_tsnes_queue *queue = &data->ts_queues[0];
	struct eth_tsnes_basic_desc_with_ts *ring =
		(struct eth_tsnes_basic_desc_with_ts *)queue->desc_ring;
	struct eth_tsnes_basic_desc_with_ts *desc;

	for (i = 0; i < queue->desc_count; i++) {
		desc = &ring[i];

		desc->die_dt = ETH_DESC_FEMPTY_ND | ETH_DESC_DIE;
		desc->info_ds = 0;
		desc->info = 0;
		desc->dptr = 0;
		desc->tsns = 0;
		desc->tss = 0;
	}

	/* Link last to first */
	ring[queue->desc_count - 1].die_dt = ETH_DESC_LINK;
	ring[queue->desc_count - 1].info_ds = 0;
	ring[queue->desc_count - 1].info = 0;
	ring[queue->desc_count - 1].dptr = (uint32_t)(uintptr_t)&ring[0];
	ring[queue->desc_count - 1].tsns = 0;
	ring[queue->desc_count - 1].tss = 0;

	tsnes_dcache_flush((uintptr_t)ring,
			   queue->desc_count * sizeof(struct eth_tsnes_basic_desc_with_ts));

	axi->stTSD[0].TSSs = ETH_TSNES_TS_DESC_CHAIN_VALID;
	axi->stTSD[0].TSAs = (uint32_t)(uintptr_t)&ring[0];

	return 0;
}
#endif

static int eth_tsnes_axibmi_init(const struct device *dev)
{
	const struct eth_tsnes_config *cfg = dev->config;
	volatile eth_tsnes_axibmi_reg_t *axi = &cfg->tsnes_base->stAXIBMI;
	volatile eth_tsnes_mhd_reg_t *mhd = &cfg->tsnes_base->stMHD;
	int ret;

	/* AXIBMI Init requires MHD in CONFIG mode to perform RAM reset */
	ret = eth_tsnes_mhd_mode_setting(mhd, ETH_TSNES_MHD_DISABLE_MODE);
	if (ret) {
		return ret;
	}

	ret = eth_tsnes_mhd_mode_setting(mhd, ETH_TSNES_MHD_CONFIG_MODE);
	if (ret) {
		return ret;
	}

	/* Reset RAMs */
	axi->RR = ETH_TSNES_RATRR_RESET | ETH_TSNES_TATRR_RESET;
	ret = eth_tsnes_wait_ram_reset(axi);
	if (ret) {
		LOG_ERR("tsnes reset ram failed!, ret: %d", ret);
		return ret;
	}

	/* Configure AXIBMI */
	axi->AXIWC = ETH_TSNES_AXIWC_CONFIG;
	axi->AXIRC = ETH_TSNES_AXIRC_CONFIG;
	axi->TDPC[0] = ETH_TSNES_TDPC_CONFIG;
	axi->TFT = ETH_TSNES_TFT_CONFIG;

	ret = eth_tsnes_setup_tx_descriptors(dev);
	if (ret) {
		return ret;
	}

	ret = eth_tsnes_setup_rx_descriptors(dev);
	if (ret) {
		return ret;
	}

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
	ret = eth_tsnes_setup_ts_descriptors(dev);
	if (ret) {
		return ret;
	}
#endif

	axi->DIE = ETH_TSNES_DIE_CONFIG;
	axi->TSDIE = ETH_TSNES_TSDIE_CONFIG;
	axi->EIE0 = ETH_TSNES_EIE0_CONFIG;

	/* Descriptor Data Interrupt Enable */
	axi->stTXDI[0].TDIEi = ETH_TSNES_TDIE0_CONFIG;
	axi->stRXDI[0].RDIEi = ETH_TSNES_RDIE0_CONFIG;

	return 0;
}

static void eth_tsnes_mhd_init(const struct device *dev)
{
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;
	volatile eth_tsnes_axibmi_reg_t *axi = &cfg->tsnes_base->stAXIBMI;
	volatile eth_tsnes_mhd_reg_t *mhd = &cfg->tsnes_base->stMHD;
	uint32_t i;

	/* --- MHD Initialization --- */
	mhd->TCIE = ETH_TSNES_TCIE_CONFIG;
	mhd->TIE2 = ETH_TSNES_TIE2_CONFIG;
	mhd->RIE = ETH_TSNES_RIE_CONFIG;

	for (i = 0; i < data->num_tx_queues; i++) {
		mhd->TMS[i] = data->max_frame_size;
	}

	for (i = 0; i < 4; i++) {
		mhd->TFS[i] = ETH_TSNES_TFS_CONFIG;
	}

	mhd->TGC1 = ETH_TSNES_TGC1_CONFIG;
	mhd->TGC2 = axi->TFT;

	mhd->TCR1 = ETH_TSNES_TCR1_CONFIG;
	mhd->TCR2 = ETH_TSNES_TCR2_CONFIG;
	mhd->TCR3 = ETH_TSNES_TCR3_CONFIG;
	mhd->TCR4 = ETH_TSNES_TCR4_CONFIG;

	mhd->RGC = ETH_TSNES_RGC_CONFIG;
	mhd->RDFCR = ETH_TSNES_RDFCR_CONFIG;
	mhd->RCFCR = ETH_TSNES_RCFCR_CONFIG;

	/* TAS/CBS config */
	mhd->TPTPC = 0;    /* FSP ETH_TSNES_GPTP_TIMER_DOMAIN is 0 */
	mhd->TTML = 0x480; /* FSP ETH_TSNES_TRANSMISSION_MIN_LATENCY */
	mhd->TTJ = 0x48;   /* FSP ETH_TSNES_TRANSMISSION_JITTER */
}

static void eth_tsnes_rmac_init(const struct device *dev)
{
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;
	volatile eth_tsnes_rmacs_reg_t *rmacs = &cfg->tsnes_base->stRMACSys;
	volatile eth_tsnes_rmac_reg_t *rmac = &rmacs->stRMAC;

	/* --- RMAC Initialization --- */
	rmac->MRMAC0 = (cfg->mac_addr[0] << 8) | cfg->mac_addr[1];
	rmac->MRMAC1 = (cfg->mac_addr[2] << 24) | (cfg->mac_addr[3] << 16) |
		       (cfg->mac_addr[4] << 8) | cfg->mac_addr[5];

	rmac->MLBC = 0;
	rmac->MTFFC = 0;

	/* Automatic timestamp */
	rmac->MRGC = 0;

	rmac->MRFSCE = data->max_frame_size;
	rmac->MRFSCP = data->max_frame_size;

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
	rmac->MTRC = 0x00020000UL | (cfg->ptp_clock_channel << 28);
#else
	rmac->MTRC = 0x03020000UL;
#endif

	rmac->MRSCE = 0x003F003FUL; /* FSP ETH_TSNES_MRSCE_CONFIG */
	rmac->MRSCP = 0x003F003FUL; /* FSP ETH_TSNES_MRSCP_CONFIG */

	rmac->MRAFC = 0x07870787UL;

#if defined(CONFIG_ETH_RENESAS_RCARSOC_TSNES_CONFIG_LOOPBACK)
	eth_tsnes_set_mac_loopback(cfg, true);
#else
	eth_tsnes_set_mac_loopback(cfg, false);
#endif

	/* PHY Interface configuration (Default to SGMII/1GB for this SOC) */
	rmac->MPIC = ETH_TSNES_MPIC_CONFIG;
}

static int eth_tsnes_tx_ram_init(const struct device *dev)
{
	const struct eth_tsnes_config *cfg = dev->config;
	volatile eth_tsnes_axibmi_reg_t *axi = &cfg->tsnes_base->stAXIBMI;
	volatile eth_tsnes_mhd_reg_t *mhd = &cfg->tsnes_base->stMHD;
	volatile eth_tsnes_rmacs_reg_t *rmacs = &cfg->tsnes_base->stRMACSys;
	struct eth_tsnes_ext_desc *desc_chain = cfg->init_desc_chain;
	uint8_t *dummy_buf = cfg->init_dummy_buf;
	int ret, i, j;

	if (!desc_chain || !dummy_buf) {
		return -ENOMEM;
	}

	memset(desc_chain, 0, 17 * sizeof(struct eth_tsnes_ext_desc));
	memset(dummy_buf, 0, 2048);

	ret = eth_tsnes_mhd_mode_setting(mhd, ETH_TSNES_MHD_DISABLE_MODE);
	if (ret) {
		goto cleanup;
	}

	ret = eth_tsnes_mhd_mode_setting(mhd, ETH_TSNES_MHD_CONFIG_MODE);
	if (ret) {
		goto cleanup;
	}

	/* Configure AXIBMI for dummy transmission */
	axi->AXIWC = ETH_TSNES_AXIWC_CONFIG;
	axi->AXIRC = ETH_TSNES_AXIRC_CONFIG;
	axi->TDPC[0] = ETH_TSNES_TDPC_CONFIG;
	axi->TFT = ETH_TSNES_TFT_CONFIG;

	mhd->TFS[0] = ETH_TSNES_INIT_TFS0_CONFIG; /* ETH_TSNES_INIT_TFS0_CONFIG */
	mhd->TFS[1] = 0;
	mhd->TFS[2] = 0;
	mhd->TFS[3] = 0;
	mhd->TGC1 = ETH_TSNES_TGC1_CONFIG;
	mhd->TGC2 = 0;
	mhd->TMS[0] = ETH_TSNES_INIT_TMS_CONFIG; /* ETH_TSNES_INIT_TMS_CONFIG */

	rmacs->stRMAC.MCC &= ~0x00000002UL; /* Clear TXE mask */

	/* Setup descriptors */
	struct eth_tsnes_ext_desc *desc = desc_chain;

	for (i = 0; i < 2; i++) {
		desc->die_dt = ETH_DESC_FSTART;
		desc->info_ds = (2048 & 0xFFF);
		desc->info = 0;
		desc->dptr = (uint32_t)(uintptr_t)dummy_buf;
		desc->info1[0] = ETH_TSNES_INIT_FRAME_LENGTH;
		desc->info1[1] = 0;
		desc++;

		for (j = 0; j < 6; j++) {
			desc->die_dt = ETH_DESC_FMID;
			desc->info_ds = (2048 & 0xFFF);
			desc->info = 0;
			desc->dptr = (uint32_t)(uintptr_t)dummy_buf;
			desc->info1[0] = ETH_TSNES_INIT_FRAME_LENGTH;
			desc->info1[1] = 0;
			desc++;
		}

		desc->die_dt = ETH_DESC_FEND;
		desc->info_ds = (2048 & 0xFFF);
		desc->info = 0;
		desc->dptr = (uint32_t)(uintptr_t)dummy_buf;
		desc->info1[0] = ETH_TSNES_INIT_FRAME_LENGTH;
		desc->info1[1] = 0;
		desc++;
	}

	/* EOS descriptor */
	desc->die_dt = ETH_DESC_EOS | ETH_DESC_DIE;
	desc->info_ds = 0;
	desc->info = 0;
	desc->dptr = (uint32_t)(uintptr_t)desc_chain;
	desc->info1[0] = 0;
	desc->info1[1] = 0;

	tsnes_dcache_flush((uintptr_t)desc_chain, 17 * sizeof(struct eth_tsnes_ext_desc));

	ret = eth_tsnes_tx_desc_learning(axi, 0, (uint32_t)(uintptr_t)desc_chain);
	if (ret) {
		goto cleanup;
	}

	ret = eth_tsnes_mhd_mode_setting(mhd, ETH_TSNES_MHD_DISABLE_MODE);
	if (ret) {

		goto cleanup;
	}

	ret = eth_tsnes_mhd_mode_setting(mhd, ETH_TSNES_MHD_OPERATION_MODE);
	if (ret) {
		goto cleanup;
	}

	/* Trigger transmission */
	axi->TRCR[0] = 1; /* ETH_TSNES_TRCR0_TRANSMIT_ENABLE */

	if (!WAIT_FOR(axi->stTXDI[0].TDISi & 1, 10000, k_busy_wait(1))) {
		ret = -ETIMEDOUT;
		LOG_ERR("failed to Trigger transmission, axi->stTXDI[0].TDISi: %d",
			axi->stTXDI[0].TDISi);
		goto cleanup;
	}

	axi->stTXDI[0].TDISi = 1; /* Clear interrupt */

	/* Software Reset MHD */
	ret = eth_tsnes_mhd_mode_setting(mhd, ETH_TSNES_MHD_DISABLE_MODE);
	if (ret) {
		goto cleanup;
	}

	mhd->SWR = 1; /* ETH_TSNES_SOFTWARE_RESET_ENABLE */

	if (!WAIT_FOR(mhd->GIS & ETH_TSNES_GIS_SWRCF_MASK, 1000, k_busy_wait(1))) {
		ret = -ETIMEDOUT;
		goto cleanup;
	}

	mhd->GIS = ETH_TSNES_GIS_SWRCF_MASK; /* Clear flag */

	LOG_INF("RAM init successful");

	return 0;

cleanup:
	return ret;
}

static int eth_tsnes_hw_init_tsnes(const struct device *dev)
{
	int ret;

	/* Prime TX RAM to prevent issues on 16KB frames */
	ret = eth_tsnes_tx_ram_init(dev);
	if (ret) {
		LOG_ERR("Tx RAM init failed, ret: %d", ret);
		return ret;
	}

	ret = eth_tsnes_axibmi_init(dev);
	if (ret) {
		LOG_ERR("failed to init AXIBMI");
		return ret;
	}

	eth_tsnes_mhd_init(dev);

	eth_tsnes_rmac_init(dev);

	LOG_INF("TSNES hardware modules initialized");
	return 0;
}

static void eth_tsnes_process_tx_completions(const struct device *dev)
{
	struct eth_tsnes_data *data = dev->data;
	int i;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->lock);

	for (i = 0; i < data->num_tx_queues; i++) {
		struct eth_tsnes_queue *queue = &data->tx_queues[i];
		struct eth_tsnes_ext_desc *ring = (struct eth_tsnes_ext_desc *)queue->desc_ring;

		while (queue->head_idx != queue->tail_idx) {
			struct eth_tsnes_ext_desc *desc = &ring[queue->head_idx];

			tsnes_dcache_invalidate((uintptr_t)desc, sizeof(*desc));

			uint8_t dt = desc->die_dt & ETH_DESC_DT_MASK;

			if (dt != ETH_DESC_FEMPTY && dt != ETH_DESC_FEMPTY_ND) {
				break;
			}

			desc->die_dt = ETH_DESC_EEMPTY;
			desc->info_ds = 0;

			queue->head_idx = (queue->head_idx + 1) % (queue->desc_count - 1);
		}

		ring[queue->desc_count - 1].die_dt = ETH_DESC_LINK;
	}

	k_spin_unlock(&data->lock, key);
}

static void eth_tsnes_ctrl_data_irq(const struct device *dev, bool enable)
{
	const struct eth_tsnes_config *cfg = dev->config;
	volatile eth_tsnes_axibmi_reg_t *axi = &cfg->tsnes_base->stAXIBMI;

	if (enable) {
		axi->stTXDI[0].TDIEi = ETH_TSNES_TDIE0_CONFIG;
		axi->stRXDI[0].RDIEi = ETH_TSNES_RDIE0_CONFIG;
		axi->stRXDFEI[0].RFEIEi = ETH_TSNES_RFEIE0_CONFIG;
	} else {
		axi->stTXDI[0].TDIDi = ETH_TSNES_TDIE0_CONFIG;
		axi->stRXDI[0].RDIDi = ETH_TSNES_RDIE0_CONFIG;
		axi->stRXDFEI[0].RFEIDi = ETH_TSNES_RFEIE0_CONFIG;
	}

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
	if (enable) {
		axi->TSDIE = ETH_TSNES_TSDIE_CONFIG;
	} else {
		axi->TSDID = ETH_TSNES_TSDIE_CONFIG;
	}
#endif
}

static void eth_tsnes_process_rx_full(const struct device *dev, uint32_t rxfull_stat)
{
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;
	volatile eth_tsnes_axibmi_reg_t *axi = &cfg->tsnes_base->stAXIBMI;
	uint32_t i;

	for (i = 0; i < data->num_rx_queues; i++) {
		if (!ETH_TSN_GET_RX_FULL_INT_STAT(i, rxfull_stat, 0)) {
			continue;
		}

		data->rx_queues[i].stats.rx_errors++;
		data->rx_queues[i].rxfull_pending = true;
		LOG_WRN("RX queue %u descriptor ring reported full (RFEISi); "
			"will restore once drained",
			i);
	}

	/* Clear the RX descriptor full interrupt status bits just handled */
	axi->stRXDFEI[0].RFEISi = rxfull_stat;
}

/* True once every real (non-LINK) descriptor in the queue is back to FEMPTY. */
static bool eth_tsnes_rx_queue_drained(struct eth_tsnes_queue *queue)
{
	struct eth_tsnes_ext_desc_with_ts *ring =
		(struct eth_tsnes_ext_desc_with_ts *)queue->desc_ring;
	uint32_t i;
	k_spinlock_key_t key;

	key = k_spin_lock(&queue->lock);
	for (i = 0; i < queue->desc_count - 1; i++) {
		uint8_t dt = ring[i].die_dt & ETH_DESC_DT_MASK;

		if (dt != ETH_DESC_FEMPTY && dt != ETH_DESC_FEMPTY_ND) {
			LOG_WRN("RX queue not drained yet: desc %u die_dt=0x%02x info_ds=0x%x", i,
				ring[i].die_dt, ring[i].info_ds);
			k_spin_unlock(&queue->lock, key);
			return false;
		}
	}
	k_spin_unlock(&queue->lock, key);

	return true;
}

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
static void eth_tsnes_process_tx_ts_read(const struct device *dev)
{
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;
	struct eth_tsnes_queue *queue = &data->ts_queues[0];
	struct eth_tsnes_basic_desc_with_ts *ring = queue->desc_ring;
	k_spinlock_key_t key;

	for (uint32_t i = 0; i < queue->desc_count - 1; i++) {
		struct eth_tsnes_basic_desc_with_ts *desc;
		struct net_pkt *pkt;
		uint8_t tsun;

		desc = &ring[queue->head_idx];

		key = k_spin_lock(&queue->lock);
		tsnes_dcache_invalidate((uintptr_t)desc, sizeof(*desc));

		if ((desc->die_dt & ETH_DESC_DT_MASK) != ETH_DESC_FSINGLE) {
			k_spin_unlock(&queue->lock, key);
			break;
		}

		tsun = desc->info;

		k_spin_unlock(&queue->lock, key);

		key = k_spin_lock(&data->lock);
		pkt = cfg->ts_pkt_nodes[0][tsun].pkt;
		if (pkt) {
			pkt->timestamp.nanosecond = desc->tsns;
			pkt->timestamp.second = desc->tss;

			net_if_add_tx_timestamp(pkt);
			net_pkt_unref(pkt);

			cfg->ts_pkt_nodes[0][tsun].pkt = NULL;
		}

		k_spin_unlock(&data->lock, key);

		key = k_spin_lock(&queue->lock);
		desc->die_dt = ETH_DESC_FEMPTY_ND | ETH_DESC_DIE;
		desc->info_ds = 0;
		desc->info = 0;
		desc->dptr = 0;
		desc->tsns = 0;
		desc->tss = 0;
		tsnes_dcache_flush((uintptr_t)desc, sizeof(*desc));
		k_spin_unlock(&queue->lock, key);

		queue->head_idx = (queue->head_idx + 1) % (queue->desc_count - 1);
	}

	/* Change the last desc to LINK because HW changed it to LEMPTY after process */
	ring[queue->desc_count - 1].die_dt = ETH_DESC_LINK;
	tsnes_dcache_flush((uintptr_t)&ring[queue->desc_count - 1], sizeof(*ring));
}
#endif

static void eth_tsnes_process_work_handler(struct k_work *work)
{
	struct eth_tsnes_data *data = CONTAINER_OF(work, struct eth_tsnes_data, process_work);
	const struct device *dev = data->dev;
	uint32_t status;
	uint32_t rxfull_status;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	status = data->pending_irq_status;
	data->pending_irq_status = 0;
	rxfull_status = data->pending_rxfull_status;
	data->pending_rxfull_status = 0;
	k_spin_unlock(&data->lock, key);

	if (status & 1) {
		eth_tsnes_process_tx_completions(dev);
	}

	if (status & 2) {
		k_sem_give(&data->rx_sem);
	}

	if (status & 4) {
		LOG_DBG("eth_tsnes_process_rx_full, rxfull_status=%d", rxfull_status);
		eth_tsnes_process_rx_full(dev, rxfull_status);
	}

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
	if (status & 8) {
		eth_tsnes_process_tx_ts_read(dev);
	}
#endif

	eth_tsnes_ctrl_data_irq(dev, true);
}

static void eth_tsnes_isr(const struct device *dev)
{
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;
	volatile eth_tsnes_axibmi_reg_t *axi = &cfg->tsnes_base->stAXIBMI;
	uint32_t txi_stat = axi->stTXDI[0].TDISi;
	uint32_t rxi_stat = axi->stRXDI[0].RDISi;
	uint32_t rxfull_stat = axi->stRXDFEI[0].RFEISi;
	uint32_t tsi_stat = axi->TSDIS;

	if (txi_stat || rxi_stat || rxfull_stat || (tsi_stat != 0U)) {
		axi->stTXDI[0].TDISi = txi_stat;
		axi->stRXDI[0].RDISi = rxi_stat;
		axi->stRXDFEI[0].RFEISi = rxfull_stat;
		axi->TSDIS = tsi_stat;

		eth_tsnes_ctrl_data_irq(dev, false);

		data->pending_irq_status |=
			(txi_stat ? 1 : 0) | ((rxi_stat ? 1 : 0) << 1) | (rxfull_stat ? 4 : 0);
#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
		data->pending_irq_status |= (tsi_stat ? 8 : 0);
#endif
		data->pending_rxfull_status |= rxfull_stat;

		k_work_submit(&data->process_work);
	}

	if (!txi_stat && !rxi_stat && !rxfull_stat && !tsi_stat) {
		LOG_INF("error signal interrupt!");
	}
}

static int eth_tsnes_process_rx_descriptor(const struct device *dev, struct eth_tsnes_queue *queue,
					   uint32_t desc_idx)
{
	struct eth_tsnes_data *data = dev->data;
	struct eth_tsnes_ext_desc_with_ts *ring =
		(struct eth_tsnes_ext_desc_with_ts *)queue->desc_ring;
	struct eth_tsnes_ext_desc_with_ts *desc = &ring[desc_idx];
	struct net_pkt *pkt;
	struct net_buf *buf;
	struct eth_tsnes_rx_dma_meta *meta;
	uint16_t pkt_len;
	int ret = 0;
	k_spinlock_key_t key;

	key = k_spin_lock(&queue->lock);
	tsnes_dcache_invalidate((uintptr_t)desc, sizeof(*desc));

	/* Check if HW transferred ownership to software */
	if ((desc->die_dt & ETH_DESC_DT_MASK) != ETH_DESC_FSINGLE) {
		k_spin_unlock(&queue->lock, key);
		return -EAGAIN; /* Hardware still owns it */
	}

	if (desc->info_ds & ETH_DESC_RX_EI) {
		queue->stats.rx_errors++;
		LOG_DBG("RX desc %u: hardware reported error, info_ds=0x%x", desc_idx,
			desc->info_ds);
		ret = -EIO;
		goto rx_done;
	}

	pkt_len = desc->info_ds & 0xFFF;
	if (pkt_len < ETH_HEADER_SIZE || pkt_len > data->max_frame_size) {
		queue->stats.rx_errors++;
		LOG_DBG("RX desc %u: invalid length %u, max %u", desc_idx, pkt_len,
			data->max_frame_size);
		ret = -EINVAL;
		goto rx_done;
	}

	k_spin_unlock(&queue->lock, key);

	tsnes_dcache_invalidate((uintptr_t)desc->dptr, tsnes_cache_align_size(pkt_len));

	LOG_DBG("RX desc %u: len=%u", desc_idx, pkt_len);
	LOG_HEXDUMP_DBG((uint8_t *)(uintptr_t)desc->dptr, pkt_len, "RX packet data");

	pkt = net_pkt_rx_alloc_on_iface(data->iface, K_NO_WAIT);
	if (!pkt) {
		queue->stats.rx_dropped++;
		LOG_DBG("RX desc %u: net_pkt alloc failed, len %u", desc_idx, pkt_len);
		ret = -ENOMEM;
		goto rx_done;
	}

	buf = net_buf_alloc_with_data(&eth_tsnes_rx_dma_pool, (void *)(uintptr_t)desc->dptr,
				      pkt_len, K_NO_WAIT);
	if (!buf) {
		net_pkt_unref(pkt);
		queue->stats.rx_dropped++;
		LOG_DBG("RX desc %u: net_buf alloc failed, len %u", desc_idx, pkt_len);
		ret = -ENOMEM;
		goto rx_done;
	}

	meta = net_buf_user_data(buf);
	meta->queue = queue;
	meta->desc_idx = desc_idx;

	net_pkt_append_buffer(pkt, buf);

	queue->stats.rx_packets++;
	queue->stats.rx_bytes += pkt_len;

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
	pkt->timestamp.nanosecond = desc->tsns;
	pkt->timestamp.second = desc->tss;
#endif

	net_recv_data(data->iface, pkt);

rx_done:
	if (ret < 0) {
		key = k_spin_lock(&queue->lock);
		desc->info_ds = MIN(data->max_frame_size, 0xFFF);
		desc->die_dt = ETH_DESC_FEMPTY | ETH_DESC_DIE;
		desc->info = 0;
		desc->info1[0] = 0;
		desc->info1[1] = 0;
		desc->tsns = 0;
		desc->tss = 0;
		tsnes_dcache_flush((uintptr_t)desc, sizeof(*desc));
		k_spin_unlock(&queue->lock, key);
	}

	return ret;
}

static void eth_tsnes_rx_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = (const struct device *)arg1;
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;
	volatile eth_tsnes_axibmi_reg_t *axi = &cfg->tsnes_base->stAXIBMI;
	uint32_t q_idx, desc_idx;
	uint32_t polled_rxfull_stat;
	int res = 0;
	int i;

	while (1) {
		res = k_sem_take(&data->rx_sem, K_MSEC(1));
		ARG_UNUSED(res);

		/* Poll fallback for a lost RFEISi edge */
		polled_rxfull_stat = axi->stRXDFEI[0].RFEISi;

		if (polled_rxfull_stat) {
			eth_tsnes_process_rx_full(dev, polled_rxfull_stat);
		}

		for (q_idx = 0; q_idx < data->num_rx_queues; q_idx++) {
			struct eth_tsnes_queue *queue = &data->rx_queues[q_idx];
			struct eth_tsnes_ext_desc_with_ts *ring =
				(struct eth_tsnes_ext_desc_with_ts *)queue->desc_ring;

			for (i = 0; i < queue->desc_count; i++) {
				desc_idx = queue->head_idx;
				res = eth_tsnes_process_rx_descriptor(dev, queue, desc_idx);
				if (res == -EAGAIN) {
					break;
				}

				queue->head_idx = (queue->head_idx + 1) % (queue->desc_count - 1);
			}

			if (queue->rxfull_pending) {
				LOG_INF("RX queue %u: this tick consumed %d descriptor(s), "
					"head_idx now %u",
					q_idx, i, queue->head_idx);
			}

			{
				k_spinlock_key_t lock_key = k_spin_lock(&queue->lock);

				ring[queue->desc_count - 1].die_dt = ETH_DESC_LINK;
				k_spin_unlock(&queue->lock, lock_key);
			}

			if (queue->rxfull_pending) {
				if (eth_tsnes_rx_queue_drained(queue)) {
					if (eth_tsnes_rx_desc_learning(
						    axi, q_idx,
						    (uint32_t)(uintptr_t)queue->desc_ring) == 0) {
						queue->rxfull_pending = false;
						LOG_INF("RX queue %u restored after descriptor "
							"full",
							q_idx);
					} else {
						LOG_ERR("RX queue %u restore (address table "
							"learning) failed, will retry",
							q_idx);
					}
				}
			}
		}
	}
}

static int eth_tsnes_tx(const struct device *dev, struct net_pkt *pkt)
{
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;
	struct eth_tsnes_queue *tx_queue = &data->tx_queues[0];
	struct eth_tsnes_ext_desc *ring = (struct eth_tsnes_ext_desc *)tx_queue->desc_ring;
	struct eth_tsnes_ext_desc *desc;
	size_t pkt_len = net_pkt_get_len(pkt);
	uint8_t die_dt = ETH_DESC_FSINGLE | ETH_DESC_DIE;
	uint16_t info_ds = pkt_len & 0xFFF;
	uint8_t info = 0;
	uint8_t *buf;
	uint32_t desc_idx;
	uint32_t next_tail_idx;
	k_spinlock_key_t key;
	volatile eth_tsnes_axibmi_reg_t *axi = &cfg->tsnes_base->stAXIBMI;
#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
	bool is_ptp_tx = eth_tsnes_should_capture_tx_ts(pkt);
#endif

	if (!data->is_started) {
		LOG_ERR("data->is_started");
		return -ENETDOWN;
	}
	if (pkt_len > data->max_frame_size || pkt_len < ETH_HEADER_SIZE) {
		LOG_ERR("pkt_len > data->max_frame_size || pkt_len < ETH_HEADER_SIZE");
		return -EMSGSIZE;
	}
	if (pkt_len > 0xFFF) {
		/* info_ds is a 12-bit field; this driver sends every frame as a single
		 * ETH_DESC_FSINGLE descriptor, so frames above this size cannot be
		 * represented without fragmentation, which is not implemented.
		 */
		LOG_ERR("pkt_len %u exceeds single-descriptor limit (4095)", pkt_len);
		return -EMSGSIZE;
	}

	key = k_spin_lock(&data->lock);

	next_tail_idx = (tx_queue->tail_idx + 1) % (tx_queue->desc_count - 1);

	if (next_tail_idx == tx_queue->head_idx) {
		k_spin_unlock(&data->lock, key);
		LOG_DBG("TX queue full");
		return -ENOBUFS;
	}

	desc_idx = tx_queue->tail_idx;
	desc = &ring[desc_idx];
	buf = (uint8_t *)tx_queue->buf_nodes[desc_idx].buf_addr;

	if (!buf) {
		k_spin_unlock(&data->lock, key);
		LOG_DBG("buffer null");
		return -ENOBUFS;
	}

	if (net_pkt_read(pkt, buf, pkt_len)) {
		k_spin_unlock(&data->lock, key);
		LOG_ERR("packet read failed");
		return -EIO;
	}

	tsnes_dcache_flush((uintptr_t)buf, tsnes_cache_align_size(pkt_len));

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
	if (is_ptp_tx) {
		struct eth_tsnes_ts_pkt_mgmt *ts_node = &cfg->ts_pkt_nodes[0][data->unique_index];

		/* No timestamp came back for this TSUN one full TSUN wrap ago. */
		if (ts_node->pkt != NULL) {
			LOG_WRN("TX timestamp %u lost", data->unique_index);
			net_pkt_unref(ts_node->pkt);
		}

		ts_node->pkt = net_pkt_ref(pkt);
		info_ds |= ETH_DESC_TX_CAPTURE;
		info = data->unique_index;
		die_dt |= cfg->ptp_clock_channel & 0x1U;
		data->unique_index = (data->unique_index + 1) % ETH_MAX_TS_DESC_NUM;
	}
#endif

	desc->info = info;
	desc->info_ds = info_ds;
	desc->dptr = (uint32_t)(uintptr_t)buf;
	desc->info1[0] = pkt_len; /* Tx Frame Length (TFL) */
	desc->info1[1] = 0;
	tsnes_dcache_flush((uintptr_t)desc, sizeof(*desc));

	/* Hand over to HW: DT=0 would be a descriptor configuration error */
	desc->die_dt = die_dt;
	tsnes_dcache_flush((uintptr_t)desc, sizeof(*desc));

	/* Trigger transmission on Queue 0 */
	axi->TRCR[0] |= (1 << 0);

	tx_queue->stats.tx_packets++;
	tx_queue->stats.tx_bytes += pkt_len;

	tx_queue->tail_idx = next_tail_idx;

	k_spin_unlock(&data->lock, key);

	return 0;
}

static void eth_tsnes_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;

	data->iface = iface;
	net_if_set_link_addr(iface, (uint8_t *)cfg->mac_addr, ETH_TSNES_MAC_ADDR_LEN,
			     NET_LINK_ETHERNET);
	ethernet_init(iface);

	net_if_carrier_off(iface);
#if defined(CONFIG_ETH_RENESAS_RCARSOC_TSNES_CONFIG_LOOPBACK)
	net_eth_carrier_on(iface);
#endif
}

static enum ethernet_hw_caps eth_tsnes_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE | ETHERNET_LINK_1000BASE |
#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
	       ETHERNET_PTP |
#endif
	       ETHERNET_LINK_2500BASE;
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *eth_tsnes_stats(const struct device *dev)
{
	struct eth_tsnes_data *data = dev->data;

	data->stats.pkts.rx = 0;
	data->stats.errors.rx = 0;
	data->stats.bytes.received = 0;
	data->stats.error_details.rx_no_buffer_count = 0;

	for (int i = 0; i < data->num_rx_queues; i++) {
		data->stats.pkts.rx += data->rx_queues[i].stats.rx_packets;
		data->stats.errors.rx += data->rx_queues[i].stats.rx_errors;
		data->stats.bytes.received += data->rx_queues[i].stats.rx_bytes;
		data->stats.error_details.rx_no_buffer_count += data->rx_queues[i].stats.rx_dropped;
	}

	data->stats.pkts.tx = 0;
	data->stats.bytes.sent = 0;

	for (int i = 0; i < data->num_tx_queues; i++) {
		data->stats.pkts.tx += data->tx_queues[i].stats.tx_packets;
		data->stats.bytes.sent += data->tx_queues[i].stats.tx_bytes;
	}

	return &data->stats;
}
#endif

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
static const struct device *eth_tsnes_get_ptp_clock(const struct device *dev)
{
	const struct eth_tsnes_config *cfg = dev->config;

	return cfg->ptp_clock;
}
#endif

static const struct ethernet_api eth_tsnes_api = {
	.iface_api.init = eth_tsnes_iface_init,
	.get_capabilities = eth_tsnes_caps,
	.send = eth_tsnes_tx,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = eth_tsnes_stats,
#endif
#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
	.get_ptp_clock = eth_tsnes_get_ptp_clock,
#endif
};

static int eth_tsnes_init(const struct device *dev)
{
	struct eth_tsnes_data *data = dev->data;
	const struct eth_tsnes_config *cfg = dev->config;
	int ret;

	ret = clock_control_on(cfg->clock_dev, RCAR_CLOCK_SUBSYS(cfg->mod_clk));
	if (ret < 0) {
		LOG_ERR("failed to enable TSNES clock (err %d)", ret);
		return ret;
	}

	if (data->initialized) {
		return 0;
	}

	k_sem_init(&data->rx_sem, 0, cfg->rx_queue_size);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("failed to configure pins");
		return ret;
	}

	data->dev = dev;
	data->link_up = false;
	data->is_started = false;

	ret = eth_tsnes_alloc_buffer_pool(dev);
	if (ret) {
		return ret;
	}

	ret = eth_tsnes_alloc_queues(dev);
	if (ret) {
		return ret;
	}

	ret = eth_tsnes_hw_init_tsnes(dev);
	if (ret) {
		return ret;
	}

	/* Switch MHD Mode to Operation after descriptors are setup */
	ret = eth_tsnes_mhd_mode_setting(&cfg->tsnes_base->stMHD, ETH_TSNES_MHD_DISABLE_MODE);
	if (ret) {
		return ret;
	}
	ret = eth_tsnes_mhd_mode_setting(&cfg->tsnes_base->stMHD, ETH_TSNES_MHD_OPERATION_MODE);
	if (ret) {
		return ret;
	}

	k_work_init(&data->process_work, eth_tsnes_process_work_handler);

	k_thread_create(
		&data->rx_thread, data->rx_stack, CONFIG_ETH_RENESAS_RCARSOC_TSNES_RX_THREAD_STACK,
		eth_tsnes_rx_thread, (void *)dev, NULL, NULL,
		K_PRIO_COOP(CONFIG_ETH_RENESAS_RCARSOC_TSNES_RX_THREAD_PRIORITY), 0, K_NO_WAIT);

	cfg->config_irq(dev);

	data->initialized = true;
	data->is_started = true;
	data->link_up = true;

	return 0;
}

#define ETH_TSNES_INIT_DESC(n)                                                                     \
	.rx_desc_rings = {LISTIFY(DT_INST_PROP(n, rx_queues),                                      \
				ETH_TSNES_RX_DESC_RING_PTR, (), n)},     \
			  .tx_desc_rings = {LISTIFY(DT_INST_PROP(n, tx_queues),                    \
				ETH_TSNES_TX_DESC_RING_PTR, (), n)}

#define ETH_TSNES_INIT_BUF_NODE(n)                                                                 \
	.rx_buf_nodes = {LISTIFY(DT_INST_PROP(n, rx_queues),                                       \
				  ETH_TSNES_RX_BUF_NODES_PTR, (), n)},      \
			 .tx_buf_nodes = {LISTIFY(DT_INST_PROP(n, tx_queues),                      \
				  ETH_TSNES_TX_BUF_NODES_PTR, (), n)}

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES_TIMESTAMP_CAPTURE
#define RENESAS_RCAR_PTP_DEFINE(n)                                                                 \
	.ts_desc_rings = {&eth_tsnes_##n##_ts_desc_ring[0]},                                       \
	.ts_pkt_nodes = {&eth_tsnes_##n##_ts_pkt_nodes[0]},                                        \
	.ptp_clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, ptp_clock)),                                 \
	.ptp_clock_channel = DT_REG_ADDR(DT_INST_PHANDLE(n, ptp_clock)),
#else
#define RENESAS_RCAR_PTP_DEFINE(n)
#define PTP_CONFIG_BUILD_ASSERT(n)
#endif

#define ETH_TSNES_MOD_CLK(n) RCAR_DT_INST_CLOCKS_CELL_BY_NAME(n, module)
#define ETH_TSNES_INIT(n)                                                                          \
	BUILD_ASSERT(DT_INST_PROP(n, rx_queues) >= 1 &&                                            \
			     DT_INST_PROP(n, rx_queues) <= ETH_NUM_RX_QUEUES,                      \
		     "rx-queues out of range");                                                    \
	BUILD_ASSERT(DT_INST_PROP(n, tx_queues) >= 1 &&                                            \
			     DT_INST_PROP(n, tx_queues) <= ETH_NUM_TX_QUEUES,                      \
		     "tx-queues out of range");                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static void eth_tsnes_config_irq_##n(const struct device *dev)                             \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, tdis, irq),                                     \
			    DT_INST_IRQ_BY_NAME(n, tdis, priority), eth_tsnes_isr,                 \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(n, tdis, irq));                                     \
	}                                                                                          \
	ETH_TSNES_INIT_DESC_RING(n);                                                               \
	ETH_TSNES_INIT_TIMESTAMP_DESC_RING(n);                                                     \
	ETH_TSNES_INIT_BUF_NODES(n);                                                               \
	ETH_TSNES_INIT_TS_PKT_NODES(n);                                                            \
	ETH_TSNES_INIT_BUF_POOL(n);                                                                \
	ETH_TSNES_INIT_DESC_CHAIN(n);                                                              \
	ETH_TSNES_INIT_DUMMY_BUF(n);                                                               \
                                                                                                   \
	static const struct eth_tsnes_config eth_tsnes_config_##n = {                              \
		.tsnes_base = (volatile eth_tsnes_reg_t *)DT_INST_REG_ADDR_BY_NAME(n, tsnes),      \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, module)),                \
		.mod_clk = ETH_TSNES_MOD_CLK(n),                                                   \
		.config_irq = eth_tsnes_config_irq_##n,                                            \
		.mac_addr = DT_INST_PROP_OR(n, local_mac_address, {0}),                            \
		.num_rx_queues = DT_INST_PROP(n, rx_queues),                                       \
		.num_tx_queues = DT_INST_PROP(n, tx_queues),                                       \
		.rx_queue_size = DT_INST_PROP(n, rx_queue_size),                                   \
		.tx_queue_size = DT_INST_PROP(n, tx_queue_size),                                   \
		ETH_TSNES_INIT_DESC(n),                                                            \
		ETH_TSNES_INIT_BUF_NODE(n),                                                        \
		.buf_pool = eth_tsnes_##n##_buf_pool,                                              \
		.init_desc_chain = eth_tsnes_##n##_init_desc_chain,                                \
		.init_dummy_buf = eth_tsnes_##n##_init_dummy_buf,                                  \
		.channel = DT_INST_PROP(n, channel),                                               \
		RENESAS_RCAR_PTP_DEFINE(n)};                                                       \
                                                                                                   \
	static struct eth_tsnes_data eth_tsnes_data_##n;                                           \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, eth_tsnes_init, NULL, &eth_tsnes_data_##n,                \
				      &eth_tsnes_config_##n, CONFIG_ETH_INIT_PRIORITY,             \
				      &eth_tsnes_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_TSNES_INIT)

#undef ETH_TSNES_MOD_CLK
