/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_ethernet_etnd

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <ethernet/eth_stats.h>

#include "r_ether_tsn.h"

LOG_MODULE_REGISTER(eth_renesas_etnd, CONFIG_ETHERNET_LOG_LEVEL);

extern void r_ether_tsn_data_isr(void);

#define ETH_ETND_TX_QUEUE_NUM(n)         DT_INST_PROP(n, txq_num)
#define ETH_ETND_RX_QUEUE_NUM(n)         DT_INST_PROP(n, rxq_num)
#define ETH_ETND_RX_BUFFER_NUM(n, i)     DT_INST_PROP(n, rxb##i##_num)
#define ETH_ETND_RX_MAX_FRAME_SIZE(n, i) DT_INST_PROP(n, rxq##i##_max_frame_size)
#define ETH_ETND_TX_BUFFER_NUM(n, i)     DT_INST_PROP(n, txb##i##_num)
#define ETH_ETND_TX_MAX_FRAME_SIZE(n, i) DT_INST_PROP(n, txq##i##_max_frame_size)

#define ETH_ETND_ALIGN4(x) (((x) + 3U) & ~3U)

/* Check the input TX queue index is active or not */
#define ETH_ETND_TXQ_ACTIVE(n, i) ((i) < ETH_ETND_TX_QUEUE_NUM(n))

/* heap_size + (tx_buf[i] + cyclic_descr) * descr_tx_size + max_tx_frame_size[i] * tx_buf[i] */
#define ETH_ETND_TXQ_RAM(n, i)                                                                     \
	(ETH_ETND_TXQ_ACTIVE(n, i)                                                                 \
		 ? (16U + (ETH_ETND_TX_BUFFER_NUM(n, i) + 1U) * 16U +                              \
		    ETH_ETND_TX_MAX_FRAME_SIZE(n, i) * ETH_ETND_TX_BUFFER_NUM(n, i))               \
		 : 0U)

/* Check the input RX queue index is active or not */
#define ETH_ETND_RXQ_ACTIVE(n, i) ((i) < ETH_ETND_RX_QUEUE_NUM(n))

/*
 * heap_size + (rx_buf[i] + cyclic_descr) * descr_rx_size
 * + align4(rx_max_frame_size[i]+ descr_offset + size_uint32 + heap_size) * rx_buf[i]
 */
#define ETH_ETND_RXQ_RAM(n, i)                                                                     \
	(ETH_ETND_RXQ_ACTIVE(n, i)                                                                 \
		 ? (16U + (ETH_ETND_RX_BUFFER_NUM(n, i) + 1U) * 24U +                              \
		    ETH_ETND_ALIGN4(ETH_ETND_RX_MAX_FRAME_SIZE(n, i) + 2U + 4U + 16U) *            \
			    ETH_ETND_RX_BUFFER_NUM(n, i))                                          \
		 : 0U)

/*
 * ETND internal buffer/descriptor RAM pool size (ether_tsn_ram_size).
 * (Constants: size_uint32=4, size_2KB=2048, cyclic_descr=1, heap_size=16,
 * init_descr=16, descr_offset=2, tx_pointer=4, descr_tx_size=16,
 * descr_rx_size=24, descr_current=2)
 *   RAM calculation formula:
 *   ram = ((size_uint32 * tx_queue) + heap_size) * tx_pointer
 *       + (size_uint32 * rx_queue) + heap_size
 *       + descr_current * ((size_uint32 * rx_queue) + heap_size)
 *       + heap_size + size_2KB
 *       + (init_descr + cyclic_descr + heap_size) * descr_tx_size
 *       + sum(heap_size + (tx_buf[i] + cyclic_descr) * descr_tx_size)
 *       + sum(heap_size + (rx_buf[i] + cyclic_descr) * descr_rx_size)
 *       + sum(max_tx_frame_size[i] * tx_buf[i])
 *       + sum(align4(max_rx_framesize + descr_offset + size_uint32 + heap_size) * rx_buf[i])
 */
#define ETH_ETND_RAM_SIZE(n)                                                                       \
	((((4U * ETH_ETND_TX_QUEUE_NUM(n)) + 16U) * 4U) + (4U * ETH_ETND_RX_QUEUE_NUM(n) + 16U) +  \
	 2U * (4U * ETH_ETND_RX_QUEUE_NUM(n) + 16U) + (16U + 2048U) + ((16U + 1U + 16U) * 16U) +   \
	 ETH_ETND_TXQ_RAM(n, 0) + ETH_ETND_RXQ_RAM(n, 0))

#define ETH_ETND_LOOPBACK_CFG(n)                                                                   \
	(DT_INST_PROP(n, loopback) ? ETHER_TSN_LOOPBACK_ON : ETHER_TSN_LOOPBACK_OFF)

#define ETH_ETND_HW_TX_QUEUES 8

/* TSNES.OCR operating modes */
#define ETH_ETND_DISABLE_MODE    0U
#define ETH_ETND_CONFIG_MODE     1U
#define ETH_ETND_MODE_TIMEOUT_US 1000U

/* ether_tsn_tx_queue_type_t entry for used TX queue. */
#define ETH_ETND_TXQ_ENTRY(n, i)                                                                   \
	{                                                                                          \
		.queue_bufs = (uint32_t)ETH_ETND_TX_BUFFER_NUM(n, i),                              \
		.queue_id = i##U,                                                                  \
		.frame_type = ETHER_TSN_EXPRESS_FRAME,                                             \
		.max_frame_size = ETH_ETND_TX_MAX_FRAME_SIZE(n, i),                                \
		.cbs_config =                                                                      \
			{                                                                          \
				.ce = ETHER_TSN_OPTION_DISABLE,                                    \
				.civ = 0U,                                                         \
				.cul = 0U,                                                         \
			},                                                                         \
	}

/*
 * ether_tsn_tx_queue_type_t entry for unused TX queue.
 * No buffers, default max frame size so its TMS register still holds a valid value
 */
#define ETH_ETND_TXQ_UNUSED_ENTRY(n, i)                                                            \
	{                                                                                          \
		.queue_bufs = 0U,                                                                  \
		.queue_id = i##U,                                                                  \
		.frame_type = ETHER_TSN_EXPRESS_FRAME,                                             \
		.max_frame_size = 1518U,                                                           \
		.cbs_config =                                                                      \
			{                                                                          \
				.ce = ETHER_TSN_OPTION_DISABLE,                                    \
				.civ = 0U,                                                         \
				.cul = 0U,                                                         \
			},                                                                         \
	}

#define ETH_ETND_PHY_IF(n) ETHER_TSN_MII_RMII

/* Multicast address filter table from devicetree, 6 bytes per entry */
#define ETH_ETND_ADDR_FILTER_LEN(n) DT_INST_PROP_LEN_OR(n, zephyr_mac_address_filter, 0)
#define ETH_ETND_ADDR_FILTER_NUM(n) (ETH_ETND_ADDR_FILTER_LEN(n) / NET_ETH_ADDR_LEN)
#define ETH_ETND_ADDR_FILTER_MAX    32
#define ETH_ETND_ADDR_FILTER_TABLE(n)                                                              \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, zephyr_mac_address_filter),                    \
		    (eth_etnd_addr_filter_##n), (NULL))

#define ETH_ETND_ADDR_FILTER_DEFINE(n)                                                             \
	static uint8_t eth_etnd_addr_filter_##n[] = DT_INST_PROP(n, zephyr_mac_address_filter);

struct eth_etnd_data {
	ether_tsn_instance_ctrl_t fsp_ctrl; /* MUST be first */

	struct net_if *iface;
	uint8_t mac_addr[6];

	K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_ETH_RENESAS_RH850_ETND_RX_THREAD_STACK_SIZE);
	struct k_thread rx_thread;
	struct k_sem rx_sem;
	struct k_sem tx_sem;
	struct k_mutex tx_lock;
	/* Zero-copy: a TX timed out and the hardware may still own tx_frame_buf */
	bool tx_busy;

	const struct device *phy_dev;
	bool link_up;
	enum phy_link_speed link_speed;

	uint8_t rx_frame_buf[NET_ETH_MAX_FRAME_SIZE];
	uint8_t tx_frame_buf[NET_ETH_MAX_FRAME_SIZE];

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif
};

struct eth_etnd_config {
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config)(void);
	const ether_cfg_t *p_fsp_cfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
};

static int eth_etnd_set_mode(struct eth_etnd_data *data, uint32_t mode)
{
	data->fsp_ctrl.p_reg_tsnes->OCR = mode;

	if (!WAIT_FOR((data->fsp_ctrl.p_reg_tsnes->OSR & BIT(mode)) != 0U, ETH_ETND_MODE_TIMEOUT_US,
		      k_busy_wait(1))) {
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * MPIC.LSC is set once at R_ETHER_TSN_Open(); update it to the negotiated speed.
 * MPIC is written in CONFIG mode, as at open. R_ETHER_TSN_LinkProcess() then
 * moves the MAC to OPERATION mode. ETND supports full duplex only.
 */
static int eth_etnd_set_link_speed(struct eth_etnd_data *data, enum phy_link_speed speed)
{
	uint32_t lsc;
	uint32_t mpic;
	int ret;

	switch (speed) {
	case LINK_FULL_10BASE:
		lsc = ETHER_TSN_SPEED_10MBPS;
		break;
	case LINK_FULL_100BASE:
		lsc = ETHER_TSN_SPEED_100MBPS;
		break;
	default:
		return -ENOTSUP;
	}

	ret = eth_etnd_set_mode(data, ETH_ETND_DISABLE_MODE);
	if (ret != 0) {
		return ret;
	}

	ret = eth_etnd_set_mode(data, ETH_ETND_CONFIG_MODE);
	if (ret != 0) {
		return ret;
	}

	mpic = data->fsp_ctrl.p_reg_rmaca->MPIC;
	mpic &= ~R_RMACA0_MPIC_LSC_Msk;
	mpic |= (lsc << R_RMACA0_MPIC_LSC_Pos) & R_RMACA0_MPIC_LSC_Msk;
	data->fsp_ctrl.p_reg_rmaca->MPIC = mpic;

	return 0;
}

static void eth_etnd_phy_link_callback(const struct device *phy_dev, struct phy_link_state *state,
				       void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct eth_etnd_data *data = dev->data;
	fsp_err_t fsp_err;
	int ret;

	ARG_UNUSED(phy_dev);

	if (state->is_up == data->link_up && state->speed == data->link_speed) {
		return;
	}

	data->link_up = state->is_up;
	data->link_speed = state->speed;

	if (data->link_up) {
		LOG_INF("Link UP");

		ret = eth_etnd_set_link_speed(data, data->link_speed);
		if (ret != 0) {
			LOG_ERR("Failed to set link speed 0x%x: %d", data->link_speed, ret);
			data->link_up = false;
			return;
		}

		data->fsp_ctrl.link_change = ETHER_TSN_LINK_CHANGE_LINK_UP;
		data->fsp_ctrl.previous_link_status = ETHER_TSN_PREVIOUS_LINK_STATUS_UP;
		fsp_err = R_ETHER_TSN_LinkProcess(&data->fsp_ctrl);
		if (fsp_err != FSP_SUCCESS) {
			LOG_ERR("R_ETHER_TSN_LinkProcess failed: %d", fsp_err);
			data->link_up = false;
			return;
		}

		net_if_carrier_on(data->iface);
	} else {
		LOG_INF("Link DOWN");
		data->fsp_ctrl.link_change = ETHER_TSN_LINK_CHANGE_LINK_DOWN;
		data->fsp_ctrl.previous_link_status = ETHER_TSN_PREVIOUS_LINK_STATUS_DOWN;
		(void)R_ETHER_TSN_LinkProcess(&data->fsp_ctrl);

		net_if_carrier_off(data->iface);
	}
}

static void eth_etnd_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_etnd_data *data = dev->data;
	int ret;

	data->iface = iface;

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr), NET_LINK_ETHERNET);

	ethernet_init(iface);
	net_if_carrier_off(iface);

	if (!device_is_ready(data->phy_dev)) {
		LOG_WRN("PHY device not ready - link detection disabled");
		return;
	}

	ret = phy_link_callback_set(data->phy_dev, eth_etnd_phy_link_callback, (void *)dev);
	if (ret != 0) {
		LOG_WRN("phy_link_callback_set failed: %d", ret);
	}
}

static void eth_etnd_data_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	r_ether_tsn_data_isr();
}

static int eth_etnd_send(const struct device *dev, struct net_pkt *pkt)
{
	struct eth_etnd_data *data = dev->data;
	uint16_t len = net_pkt_get_len(pkt);
	uint8_t *tx_buf = data->tx_frame_buf;
	fsp_err_t fsp_err;
	int ret = 0;

	if (!data->link_up) {
		return -ENETDOWN;
	}

	if (len > sizeof(data->tx_frame_buf)) {
		return -EMSGSIZE;
	}

	/*
	 * .send() can be called from several threads at once, and tx_frame_buf
	 * and tx_sem are shared: serialize the whole write + wait.
	 */
	k_mutex_lock(&data->tx_lock, K_FOREVER);

	/*
	 * In zero-copy mode the TX descriptor points at tx_frame_buf: after a
	 * timeout, wait for the pending frame to complete before overwriting it.
	 */
	if (data->tx_busy) {
		if (k_sem_take(&data->tx_sem, K_MSEC(CONFIG_PHY_MONITOR_PERIOD)) != 0) {
			ret = -EBUSY;
			goto unlock;
		}
		data->tx_busy = false;
	}

	if (net_pkt_read(pkt, tx_buf, len)) {
		ret = -EIO;
		goto unlock;
	}

	if (len < NET_ETH_MINIMAL_FRAME_SIZE) {
		memset(tx_buf + len, 0, NET_ETH_MINIMAL_FRAME_SIZE - len);
		len = NET_ETH_MINIMAL_FRAME_SIZE;
	}

	k_sem_reset(&data->tx_sem);

	fsp_err = R_ETHER_TSN_Write(&data->fsp_ctrl, tx_buf, len);
	if (fsp_err != FSP_SUCCESS) {
		switch (fsp_err) {
		case FSP_ERR_ETHER_ERROR_LINK:
			ret = -ENETDOWN;
			break;
		case FSP_ERR_IN_USE:
			ret = -ENOBUFS;
			break;
		case FSP_ERR_INVALID_SIZE:
			ret = -EMSGSIZE;
			break;
		default:
			LOG_ERR("TX: R_ETHER_TSN_Write error %d", fsp_err);
			ret = -EIO;
			break;
		}
		goto unlock;
	}

	ret = k_sem_take(&data->tx_sem, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
	if (ret != 0 && IS_ENABLED(CONFIG_ETH_RENESAS_RH850_ETND_USE_ZEROCOPY)) {
		data->tx_busy = true;
	}

unlock:
	k_mutex_unlock(&data->tx_lock);

	return ret;
}

static enum ethernet_hw_caps eth_etnd_get_capabilities(const struct device *dev,
						       struct net_if *iface)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(iface);
	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE;
}

static const struct device *eth_etnd_get_phy(const struct device *dev, struct net_if *iface)
{
	ARG_UNUSED(iface);
	return ((const struct eth_etnd_data *)dev->data)->phy_dev;
}

#ifdef CONFIG_NET_STATISTICS_ETHERNET
static struct net_stats_eth *eth_etnd_get_stats(const struct device *dev, struct net_if *iface)
{
	ARG_UNUSED(iface);
	return &((struct eth_etnd_data *)dev->data)->stats;
}
#endif

static const struct ethernet_api eth_etnd_api = {
	.iface_api.init = eth_etnd_iface_init,
	.send = eth_etnd_send,
	.get_capabilities = eth_etnd_get_capabilities,
	.get_phy = eth_etnd_get_phy,
#ifdef CONFIG_NET_STATISTICS_ETHERNET
	.get_stats = eth_etnd_get_stats,
#endif
};

/* Largest data area of one net_buf (no fixed limit with variable-size buffers) */
#if defined(CONFIG_NET_BUF_FIXED_DATA_SIZE)
#define ETH_ETND_RX_BUF_MAX_LEN CONFIG_NET_BUF_DATA_SIZE
#else
#define ETH_ETND_RX_BUF_MAX_LEN SIZE_MAX
#endif

/* Allocate an RX net_pkt whose data has the same 4-byte alignment as the HAL frame */
static struct net_pkt *eth_etnd_rx_pkt_alloc(struct eth_etnd_data *data, const uint8_t *frame,
					     uint32_t len)
{
	struct net_pkt *pkt;
	struct net_buf *buf;
	size_t offset;

	/* Frame and alignment bytes do not fit in one net_buf: plain allocation */
	if ((len + 3U) > ETH_ETND_RX_BUF_MAX_LEN) {
		return net_pkt_rx_alloc_with_buffer(data->iface, len, AF_UNSPEC, 0, K_NO_WAIT);
	}

	/* net_pkt without data buffer */
	pkt = net_pkt_rx_alloc_on_iface(data->iface, K_NO_WAIT);
	if (pkt == NULL) {
		return NULL;
	}

	/* Data buffer with room for the alignment bytes (not shrunk to the MTU) */
	buf = net_pkt_get_reserve_rx_data(len + 3U, K_NO_WAIT);
	if (buf == NULL) {
		net_pkt_unref(pkt);
		return NULL;
	}

	/* HAL frame starts at 4n + 2: shift buf->data to the same offset modulo 4 */
	offset = ((uintptr_t)frame - (uintptr_t)buf->data) & 3U;
	net_buf_reserve(buf, offset);

	/* Attach buf to pkt, also sets the write cursor */
	net_pkt_append_buffer(pkt, buf);

	return pkt;
}

/*
 * Read one frame from the RX ring. Returns false when the ring is empty;
 * *out is NULL if the frame was dropped.
 */
static bool eth_etnd_receive(const struct device *dev, struct net_pkt **out)
{
	struct eth_etnd_data *data = dev->data;
	struct net_pkt *pkt = NULL;
	uint32_t frame_len = 0U;
	fsp_err_t fsp_err;

#if defined(CONFIG_ETH_RENESAS_RH850_ETND_USE_ZEROCOPY)
	uint8_t *buf_ptr = NULL;

	fsp_err = R_ETHER_TSN_Read((ether_ctrl_t *)&data->fsp_ctrl, (void *)&buf_ptr, &frame_len);
#else
	uint8_t *buf_ptr = data->rx_frame_buf;

	/* In non-zero-copy mode *length_bytes is the capacity of the user buffer */
	frame_len = sizeof(data->rx_frame_buf);
	fsp_err = R_ETHER_TSN_Read((ether_ctrl_t *)&data->fsp_ctrl, buf_ptr, &frame_len);
#endif

	if (fsp_err != FSP_SUCCESS && fsp_err != FSP_ERR_ETHER_ERROR_NO_DATA) {
		LOG_ERR("RX: R_ETHER_TSN_Read error %d", fsp_err);
		goto release;
	}

	if (frame_len == 0U) {
		goto release;
	}

	/*
	 * Never block here: the HW ring only holds a few frames, so waiting for
	 * a free net_pkt lets it overflow and drops a whole burst instead of one.
	 */
	pkt = eth_etnd_rx_pkt_alloc(data, buf_ptr, frame_len);
	if (pkt == NULL) {
		LOG_ERR_RATELIMIT("RX: alloc failed, dropping %u B frame", frame_len);
		goto release;
	}

	if (net_pkt_write(pkt, buf_ptr, frame_len)) {
		LOG_ERR("RX: net_pkt_write failed");
		net_pkt_unref(pkt);
		pkt = NULL;
	}

release:
#if defined(CONFIG_ETH_RENESAS_RH850_ETND_USE_ZEROCOPY)
	(void)R_ETHER_TSN_BufferRelease((ether_ctrl_t *)&data->fsp_ctrl);
#endif
	if (pkt == NULL && frame_len != 0U) {
		eth_stats_update_errors_rx(data->iface);
	}

	*out = pkt;
	return (fsp_err == FSP_SUCCESS) && (frame_len != 0U);
}

static void eth_etnd_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct eth_etnd_data *data = dev->data;
	struct net_pkt *pkt;
	int res;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		res = k_sem_take(&data->rx_sem, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
		if (res != 0) {
			continue;
		}

		k_sem_reset(&data->rx_sem);

		while (eth_etnd_receive(dev, &pkt)) {
			if (pkt == NULL) {
				continue;
			}

			if (net_recv_data(net_pkt_iface(pkt), pkt) < 0) {
				net_pkt_unref(pkt);
			}
		}
	}
}

static void eth_etnd_fsp_callback(ether_callback_args_t *p_args)
{
	const struct device *dev = (const struct device *)p_args->p_context;
	struct eth_etnd_data *data = dev->data;

	switch (p_args->event) {
	case ETHER_EVENT_RX_INTERRUPT:
		k_sem_give(&data->rx_sem);
		break;

	case ETHER_EVENT_TX_COMPLETE:
		k_sem_give(&data->tx_sem);
		break;

	default:
		break;
	}
}

static int eth_etnd_init(const struct device *dev)
{
	struct eth_etnd_data *data = dev->data;
	const struct eth_etnd_config *cfg = dev->config;
	fsp_err_t fsp_err;
	int ret;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("Ethernet TSN clock device is not ready");
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if ((ret < 0) && (ret != -EALREADY)) {
		LOG_ERR("Ethernet TSN module clock enable failed (err %d)", ret);
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("pinctrl_apply_state failed: %d", ret);
		return ret;
	}

	fsp_err = R_ETHER_TSN_Open(&data->fsp_ctrl, cfg->p_fsp_cfg);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ETHER_TSN_Open failed: %d", fsp_err);
		return -EIO;
	}

	k_sem_init(&data->rx_sem, 0, K_SEM_MAX_LIMIT);
	k_sem_init(&data->tx_sem, 0, 8);
	k_mutex_init(&data->tx_lock);

	fsp_err = R_ETHER_TSN_CallbackSet(&data->fsp_ctrl, eth_etnd_fsp_callback, (void *)dev,
					  NULL);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ETHER_TSN_CallbackSet failed: %d", fsp_err);
		(void)R_ETHER_TSN_Close((ether_ctrl_t *)&data->fsp_ctrl);
		return -EIO;
	}

	cfg->irq_config();

	k_thread_create(&data->rx_thread, data->rx_stack,
			CONFIG_ETH_RENESAS_RH850_ETND_RX_THREAD_STACK_SIZE, eth_etnd_rx_thread,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ETH_RENESAS_RH850_ETND_RX_THREAD_PRIORITY), 0,
			K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "etnd_rx");

	LOG_INF("ETND 10/100 TSN driver initialized");
	return 0;
}

#define ETH_ETND_DEFINE_STATIC(n)                                                                  \
	BUILD_ASSERT(ETH_ETND_TX_QUEUE_NUM(n) == 1,                                                \
		     "txq-num > 1 is not supported in this version");                              \
	BUILD_ASSERT(ETH_ETND_RX_QUEUE_NUM(n) == 1,                                                \
		     "rxq-num > 1 needs multi-RX-queue support not yet implemented");              \
	BUILD_ASSERT(IS_ENABLED(CONFIG_ETH_RENESAS_RH850_ETND_USE_ZEROCOPY) ||                     \
		     ETH_ETND_RX_MAX_FRAME_SIZE(n, 0) <= NET_ETH_MAX_FRAME_SIZE,                   \
		     "rxq0-max-frame-size > NET_ETH_MAX_FRAME_SIZE needs zero-copy");              \
	BUILD_ASSERT(ETH_ETND_ADDR_FILTER_LEN(n) % NET_ETH_ADDR_LEN == 0,                          \
		     "zephyr,mac-address-filter must be a multiple of 6 bytes");                   \
	BUILD_ASSERT(ETH_ETND_ADDR_FILTER_NUM(n) <= ETH_ETND_ADDR_FILTER_MAX,                      \
		     "zephyr,mac-address-filter supports at most 32 entries");                     \
                                                                                                   \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(n, zephyr_mac_address_filter),                            \
		   (ETH_ETND_ADDR_FILTER_DEFINE(n)))                                               \
                                                                                                   \
	static uint8_t g_ether_tsn_mem_pool_##n[ETH_ETND_RAM_SIZE(n)] BSP_ALIGN_VARIABLE(4);       \
                                                                                                   \
	static const ether_tsn_tx_queue_type_t eth_etnd_tx_q_##n[ETH_ETND_HW_TX_QUEUES] = {        \
		[0] = ETH_ETND_TXQ_ENTRY(n, 0),        /* used */                                  \
		[1] = ETH_ETND_TXQ_UNUSED_ENTRY(n, 1), /* unused */                                \
		[2] = ETH_ETND_TXQ_UNUSED_ENTRY(n, 2), /* unused */                                \
		[3] = ETH_ETND_TXQ_UNUSED_ENTRY(n, 3), /* unused */                                \
		[4] = ETH_ETND_TXQ_UNUSED_ENTRY(n, 4), /* unused */                                \
		[5] = ETH_ETND_TXQ_UNUSED_ENTRY(n, 5), /* unused */                                \
		[6] = ETH_ETND_TXQ_UNUSED_ENTRY(n, 6), /* unused */                                \
		[7] = ETH_ETND_TXQ_UNUSED_ENTRY(n, 7), /* unused */                                \
	};                                                                                         \
                                                                                                   \
	static const ether_tsn_extended_cfg_t eth_etnd_ext_cfg_##n = {                             \
		.ether_tsn_transmission_fragment_size = ETHER_TSN_TRANSMISSION_FRAGMENT_SIZE_64,   \
		.ether_tsn_queue_contanins_traffic = ETHER_TSN_E_MAC,                              \
		.ether_tsn_cfg_type_tx_isr = ETHER_TSN_CTRL_ENABLE_TX_INTERRUPT,                   \
		.ether_tsn_cfg_type_rx_isr = ETHER_TSN_CTRL_ENABLE_RX_INTERRUPT,                   \
		.ether_tsn_ctl_mode = ETHER_TSN_CTRL_INTERRUPT_MODE,                               \
		.ether_tsn_filter_action_type = ETHER_TSN_ADD_TO_FILTER,                           \
		.p_mac_address_filter = ETH_ETND_ADDR_FILTER_TABLE(n),                             \
		.number_of_addr_filter = (uint8_t)ETH_ETND_ADDR_FILTER_NUM(n),                     \
		.header_data_custom = ETHER_TSN_HEADER_CUSTOM,                                     \
		.pause_frame = ETHER_TSN_PAUSE_FRAME_DISABLE,                                      \
		.pause_frame_retransmission_time = 0U,                                             \
		.pause_time = 1U,                                                                  \
		.pause_time_zero = ETHER_TSN_PAUSE_FRAME_ZERO_DISABLE,                             \
		.ether_loopback_config = ETH_ETND_LOOPBACK_CFG(n),                                 \
		.ether_network_speed = ETHER_TSN_SPEED_100MBPS,                                    \
		.ether_phy_interface = ETH_ETND_PHY_IF(n),                                         \
		.ether_broadcast = ETHER_TSN_BROADCAST_ENABLE,                                     \
		.ether_unicast = ETHER_TSN_UNICAST_ENABLE,                                         \
		.ether_preemption_link_verification =                                              \
			ETHER_TSN_PREEMPTION_LINK_VERIFICATION_DISABLE,                            \
		.p_mac_address_target = NULL,                                                      \
		.ether_type = 0U,                                                                  \
		.ether_tsn_ram_size = ETH_ETND_RAM_SIZE(n),                                        \
		.p_mem_pool_buffer_table = g_ether_tsn_mem_pool_##n,                               \
		.gptp_irq = FSP_INVALID_VECTOR, /* gPTP not supported */                           \
		.interrupt_priority = BSP_IRQ_DISABLED,                                            \
		.queue_config =                                                                    \
			{                                                                          \
				.p_tx_queue_config = eth_etnd_tx_q_##n,                            \
				.number_of_tx_queue = (uint8_t)ETH_ETND_TX_QUEUE_NUM(n),           \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	static struct eth_etnd_data eth_etnd_data_##n = {                                          \
		.mac_addr = DT_INST_PROP(n, local_mac_address),                                    \
		.phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, phy_handle)),                          \
		.link_speed = LINK_FULL_100BASE,                                                   \
	};                                                                                         \
                                                                                                   \
	static const ether_cfg_t eth_etnd_fsp_cfg_##n = {                                          \
		.channel = DT_INST_PROP_OR(n, channel, 0),                                         \
		.zerocopy = IS_ENABLED(CONFIG_ETH_RENESAS_RH850_ETND_USE_ZEROCOPY)               \
				    ? ETHER_ZEROCOPY_ENABLE                                        \
				    : ETHER_ZEROCOPY_DISABLE,                                      \
		.multicast = ETHER_MULTICAST_ENABLE,                                               \
		.promiscuous = ETHER_PROMISCUOUS_DISABLE,                                          \
		.flow_control = ETHER_FLOW_CONTROL_DISABLE,                                        \
		.p_mac_address = eth_etnd_data_##n.mac_addr,                                       \
		.num_rx_descriptors = (uint8_t)ETH_ETND_RX_BUFFER_NUM(n, 0),                       \
		.ether_buffer_size = ETH_ETND_RX_MAX_FRAME_SIZE(n, 0),                             \
		.irq = DT_INST_IRQ_BY_NAME(n, data, irq),                                          \
		.interrupt_priority = DT_INST_IRQ_BY_NAME(n, data, priority),                      \
		.p_callback = NULL,                                                                \
		.p_ether_phy_instance = NULL, /* PHY handled via Zephyr MDIO + phy-handle */       \
		.p_context = NULL,                                                                 \
		.p_extend = &eth_etnd_ext_cfg_##n,                                                 \
	};                                                                                         \
                                                                                                   \
	static void eth_etnd_irq_config_##n(void)                                                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, data, irq),                                     \
			    DT_INST_IRQ_BY_NAME(n, data, priority), eth_etnd_data_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(n, data, irq));                                     \
	}                                                                                          \
                                                                                                   \
	static const struct eth_etnd_config eth_etnd_config_##n = {                                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.irq_config = eth_etnd_irq_config_##n,                                             \
		.p_fsp_cfg = &eth_etnd_fsp_cfg_##n,                                                \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)(uintptr_t)DT_INST_CLOCKS_CELL(n, clk_id), \
	};

#define ETH_ETND_DEVICE_DEFINE(n)                                                                  \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	ETH_ETND_DEFINE_STATIC(n)                                                                  \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, eth_etnd_init, NULL, &eth_etnd_data_##n,                  \
				      &eth_etnd_config_##n, CONFIG_ETH_INIT_PRIORITY,              \
				      &eth_etnd_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_ETND_DEVICE_DEFINE)
