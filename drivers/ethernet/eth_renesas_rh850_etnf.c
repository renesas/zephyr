/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_ethernet_etnf

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <ethernet/eth_stats.h>

#include "r_ether_t1s.h"

LOG_MODULE_REGISTER(eth_renesas_etnf, CONFIG_ETHERNET_LOG_LEVEL);

#define ETH_ETNF_FRAME_BUF_SIZE 1536

/* Largest frame R_ETHER_T1S_Read() can return: ETH_RX_BUF_LENGTH in r_ether_t1s, without FCS */
#define ETH_ETNF_RX_FRAME_MAX 1518U

#define ETH_ETNF_MDIO_IF(n) ((ether_t1s_mdio_interface_type_t)DT_INST_ENUM_IDX(n, mdio_interface))

#define ETH_ETNF_PMA_TYPE(n)                                                                       \
	(ETH_ETNF_PMA_FILTERED(n) ? ETHER_T1S_PMA_FILTERED : ETHER_T1S_PMA_UNFILTERED)

#define ETH_ETNF_LOOPBACK_CFG(n)                                                                   \
	(ETH_ETNF_LOOPBACK(n) ? ETHER_T1S_LOOPBACK_ON : ETHER_T1S_LOOPBACK_OFF)

#define ETH_ETNF_IRQ_NUM(n)  DT_INST_IRQN(n)
#define ETH_ETNF_IRQ_PRIO(n) DT_INST_IRQ(n, priority)

#define ETH_ETNF_TXQ_NUM(n) DT_INST_PROP(n, txq_num)
#define ETH_ETNF_RXQ_NUM(n) DT_INST_PROP(n, rxq_num)

#define ETH_ETNF_TXB_NUM(n, i) DT_INST_PROP(n, txb##i##_num)
#define ETH_ETNF_RXB_NUM(n, i) DT_INST_PROP(n, rxb##i##_num)

/* Check the input TX queue index is active or not */
#define ETH_ETNF_TXQ_ACTIVE(n, i) ((i) < ETH_ETNF_TXQ_NUM(n))

/*
 * heap_size + (tx_buf[i] + cyclic_descr) * normal_descr_size
 * + (heap_size + max_buffer_size + tx_buf_handler_size + header_size) * tx_buf[i]
 */
#define ETH_ETNF_TXQ_RAM(n, i)                                                                     \
	(ETH_ETNF_TXQ_ACTIVE(n, i) ? (16U + (ETH_ETNF_TXB_NUM(n, i) + 1U) * 8U +                   \
				      (16U + 1518U + 40U + 14U) * ETH_ETNF_TXB_NUM(n, i))          \
				   : 0U)

/* Check the input RX queue index is active or not */
#define ETH_ETNF_RXQ_ACTIVE(n, i) ((i) < ETH_ETNF_RXQ_NUM(n))

/*
 * heap_size + (rx_buf[i] + cyclic_descr) * special_descr_size
 * + (heap_size + max_buffer_size + size_uint32 + descr_offset) * rx_buf[i]
 */
#define ETH_ETNF_RXQ_RAM(n, i)                                                                     \
	(ETH_ETNF_RXQ_ACTIVE(n, i) ? (16U + (ETH_ETNF_RXB_NUM(n, i) + 1U) * 20U +                  \
				      (16U + 1518U + 4U + 2U) * ETH_ETNF_RXB_NUM(n, i))            \
				   : 0U)

/*
 * ETNF internal buffer/descriptor RAM pool size (ether_t1s_ram_size).
 * (Constants: heap_size=16, max_tx_queue=4, max_rx_queue=18, normal_descr_size=8,
 * special_descr_size=20, cyclic_descr=1, max_buffer_size=1518, tx_buf_handler_size=40,
 * header_size=14, size_uint32=4, descr_offset=2)
 *   RAM calculation formula:
 *   ram = heap_size + (max_tx_queue + max_rx_queue) * normal_descr_size
 *       + sum(heap_size + (tx_buf[i] + cyclic_descr) * normal_descr_size)
 *       + sum(heap_size + (rx_buf[i] + cyclic_descr) * special_descr_size)
 *       + sum((heap_size + max_buffer_size + tx_buf_handler_size + header_size) * tx_buf[i])
 *       + sum((heap_size + max_buffer_size + size_uint32 + descr_offset) * rx_buf[i])
 */
#define ETH_ETNF_RAM_SIZE(n)                                                                       \
	((16U + (4U + 18U) * 8U) + ETH_ETNF_TXQ_RAM(n, 0) + ETH_ETNF_RXQ_RAM(n, 0))

/* ether_t1s_tx_queue_type_etnf entry for used TX queue. */
#define ETH_ETNF_TXQ_ENTRY(n, i)                                                                   \
	{                                                                                          \
		.eth_tx_queue_bufs = (uint16_t)ETH_ETNF_TXB_NUM(n, i),                             \
		.eth_tx_queue_id = i##U,                                                           \
	}

/* ether_t1s_rx_queue_type_etnf entry for used RX queue. */
#define ETH_ETNF_RXQ_ENTRY(n, i)                                                                   \
	{                                                                                          \
		.eth_rx_queue_bufs = (uint16_t)ETH_ETNF_RXB_NUM(n, i),                             \
		.eth_rx_queue_id = i##U,                                                           \
	}

#define ETH_ETNF_LOOPBACK(n)     DT_INST_PROP(n, loopback)
#define ETH_ETNF_PMA_FILTERED(n) DT_INST_PROP(n, pma_filtered)

extern void r_ether_t1s_isr();

struct eth_etnf_data {
	ether_t1s_instance_ctrl_t fsp_ctrl; /* MUST be first */

	struct net_if *iface;
	uint8_t mac_addr[6];
	K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_ETH_RENESAS_RH850_ETNF_RX_THREAD_STACK_SIZE);
	struct k_thread rx_thread;
	struct k_sem rx_sem;
	struct k_sem tx_sem;
	struct k_mutex tx_lock;
	bool tx_busy;

	const struct device *phy_dev;
	bool link_up;
	uint8_t rx_frame_buf[ETH_ETNF_RX_FRAME_MAX];
	uint8_t tx_frame_buf[NET_ETH_MAX_FRAME_SIZE];
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif
};

struct eth_etnf_config {
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config)(void);

	const ether_cfg_t *p_fsp_cfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
};

static void eth_etnf_phy_link_callback(const struct device *phy_dev, struct phy_link_state *state,
				       void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct eth_etnf_data *data = dev->data;
	fsp_err_t fsp_err;

	ARG_UNUSED(phy_dev);

	if (state->is_up != data->link_up) {
		data->link_up = state->is_up;

		if (data->link_up) {
			LOG_INF("Link UP — 10BASE-T1S half-duplex 10 Mbps");
			data->fsp_ctrl.link_change = ETHER_T1S_LINK_CHANGE_LINK_UP;
			data->fsp_ctrl.previous_link_status = ETHER_T1S_PREVIOUS_LINK_STATUS_UP;

			fsp_err = R_ETHER_T1S_LinkProcess(&data->fsp_ctrl);
			if (fsp_err != FSP_SUCCESS) {
				LOG_ERR("R_ETHER_T1S_LinkProcess failed: %d", fsp_err);
				data->link_up = false;
				return;
			}

			net_if_carrier_on(data->iface);
		} else {
			LOG_INF("Link DOWN");
			data->fsp_ctrl.link_change = ETHER_T1S_LINK_CHANGE_LINK_DOWN;
			data->fsp_ctrl.previous_link_status = ETHER_T1S_PREVIOUS_LINK_STATUS_DOWN;
			(void)R_ETHER_T1S_LinkProcess(&data->fsp_ctrl);

			net_if_carrier_off(data->iface);
		}
	}
}

static void eth_etnf_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_etnf_data *data = dev->data;
	int ret;
	data->iface = iface;

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr), NET_LINK_ETHERNET);

	ethernet_init(iface);
	net_if_carrier_off(iface);

	if (!device_is_ready(data->phy_dev)) {
		LOG_WRN("PHY device not ready — link detection disabled");
	} else {
		ret = phy_link_callback_set(data->phy_dev, eth_etnf_phy_link_callback, (void *)dev);
		if (ret != 0) {
			LOG_WRN("phy_link_callback_set failed: %d", ret);
		}
	}

	LOG_DBG("net_if bound, MAC %02x:%02x:%02x:%02x:%02x:%02x", data->mac_addr[0],
		data->mac_addr[1], data->mac_addr[2], data->mac_addr[3], data->mac_addr[4],
		data->mac_addr[5]);
}

void eth_etnf_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	r_ether_t1s_isr();
}

static int eth_etnf_send(const struct device *dev, struct net_pkt *pkt)
{
	struct eth_etnf_data *data = dev->data;
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

	fsp_err = R_ETHER_T1S_Write(&data->fsp_ctrl, tx_buf, len);
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
			LOG_ERR("TX: R_ETHER_T1S_Write error %d", fsp_err);
			ret = -EIO;
			break;
		}
		goto unlock;
	}

	ret = k_sem_take(&data->tx_sem, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
	if (ret != 0 && IS_ENABLED(CONFIG_ETH_RENESAS_RH850_ETNF_USE_ZEROCOPY)) {
		data->tx_busy = true;
	}

unlock:
	k_mutex_unlock(&data->tx_lock);

	return ret;
}

static enum ethernet_hw_caps eth_etnf_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_10BASE;
}

static const struct device *eth_etnf_get_phy(const struct device *dev)
{
	return ((const struct eth_etnf_data *)dev->data)->phy_dev;
}

#ifdef CONFIG_NET_STATISTICS_ETHERNET
static struct net_stats_eth *eth_etnf_get_stats(const struct device *dev)
{
	struct eth_etnf_data *data = dev->data;

	return &data->stats;
}
#endif

static const struct ethernet_api eth_etnf_api = {
	.iface_api.init = eth_etnf_iface_init,
	.send = eth_etnf_send,
	.get_capabilities = eth_etnf_get_capabilities,
	.get_phy = eth_etnf_get_phy,
#ifdef CONFIG_NET_STATISTICS_ETHERNET
	.get_stats = eth_etnf_get_stats,
#endif
};

/*
 * Read one frame from the RX ring. Returns false when the ring is empty;
 * *out is NULL if the frame was dropped.
 */
static bool eth_etnf_receive(const struct device *dev, struct net_pkt **out)
{
	struct eth_etnf_data *data = dev->data;
	struct net_pkt *pkt = NULL;
	uint32_t frame_len = 0U;
	fsp_err_t fsp_err;

#if defined(CONFIG_ETH_RENESAS_RH850_ETNF_USE_ZEROCOPY)
	uint8_t *buf_ptr = NULL;

	fsp_err = R_ETHER_T1S_Read((ether_ctrl_t *)&data->fsp_ctrl, (void *)&buf_ptr, &frame_len);
#else
	/*
	 * Non-zerocopy: static buffer is safe here because the RX thread
	 * is the only consumer and it runs sequentially.
	 */
	uint8_t *buf_ptr = data->rx_frame_buf;

	/* In non-zero-copy mode *length_bytes is the capacity of the user buffer */
	frame_len = sizeof(data->rx_frame_buf);
	fsp_err = R_ETHER_T1S_Read((ether_ctrl_t *)&data->fsp_ctrl, buf_ptr, &frame_len);
#endif

	if (fsp_err != FSP_SUCCESS && fsp_err != FSP_ERR_ETHER_ERROR_NO_DATA) {
		LOG_ERR("RX: R_ETHER_T1S_Read error %d", fsp_err);
		goto release;
	}

	/*
	 * The HAL has already consumed this ring entry (UFCD decremented), even
	 * when the descriptor held no usable frame and frame_len is 0, so keep
	 * draining in that case.
	 */
	if (frame_len == 0U) {
		goto release;
	}

	/*
	 * Never block here: the HW ring only holds a few frames, so waiting for
	 * a free net_pkt lets it overflow and drops a whole burst instead of one.
	 */
	pkt = net_pkt_rx_alloc_with_buffer(data->iface, frame_len, AF_UNSPEC, 0, K_NO_WAIT);
	if (pkt == NULL) {
		LOG_ERR("RX: alloc failed, dropping %u B frame", frame_len);
		goto release;
	}

	if (net_pkt_write(pkt, buf_ptr, frame_len) != 0) {
		LOG_ERR("RX: net_pkt_write failed");
		net_pkt_unref(pkt);
		pkt = NULL;
	}

release:
#if defined(CONFIG_ETH_RENESAS_RH850_ETNF_USE_ZEROCOPY)
	(void)R_ETHER_T1S_BufferRelease((ether_ctrl_t *)&data->fsp_ctrl);
#endif
	if (pkt == NULL && frame_len != 0U) {
		eth_stats_update_errors_rx(data->iface);
	}

	*out = pkt;
	return fsp_err == FSP_SUCCESS;
}

static void eth_etnf_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct eth_etnf_data *data = dev->data;
	struct net_pkt *pkt;
	int res;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		res = k_sem_take(&data->rx_sem, K_FOREVER);
		if (res != 0) {
			continue;
		}

		k_sem_reset(&data->rx_sem);

		while (eth_etnf_receive(dev, &pkt)) {
			if (pkt == NULL) {
				continue;
			}

			if (net_recv_data(net_pkt_iface(pkt), pkt) < 0) {
				net_pkt_unref(pkt);
			}
		}
	}
}

void eth_etnf_fsp_callback(ether_callback_args_t *p_args)
{
	struct device *dev = (struct device *)p_args->p_context;
	struct eth_etnf_data *data = dev->data;

	switch (p_args->event) {
	case ETHER_EVENT_RX_INTERRUPT:
		/* Wake RX thread — k_sem_give is ISR-safe */
		k_sem_give(&data->rx_sem);
		break;

	case ETHER_EVENT_TX_COMPLETE:
		k_sem_give(&data->tx_sem);
		break;

	default:
		break;
	}
}

static int eth_etnf_init(const struct device *dev)
{
	struct eth_etnf_data *data = dev->data;
	const struct eth_etnf_config *cfg = dev->config;
	fsp_err_t fsp_err;
	int ret;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("Ethernet T1S clock device is not ready");
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if ((ret < 0) && (ret != -EALREADY)) {
		LOG_ERR("Ethernet T1S module clock enable failed (err %d)", ret);
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("pinctrl_apply_state failed: %d", ret);
		return ret;
	}

	k_sem_init(&data->rx_sem, 0, K_SEM_MAX_LIMIT);
	k_sem_init(&data->tx_sem, 0, 8);
	k_mutex_init(&data->tx_lock);

	fsp_err = R_ETHER_T1S_Open(&data->fsp_ctrl, cfg->p_fsp_cfg);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ETHER_T1S_Open failed: %d", fsp_err);
		return -EIO;
	}

	fsp_err = R_ETHER_T1S_CallbackSet(&data->fsp_ctrl, eth_etnf_fsp_callback, dev, NULL);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ETHER_T1S_CallbackSet failed: %d", fsp_err);
		(void)R_ETHER_T1S_Close((ether_ctrl_t *)&data->fsp_ctrl);
		return -EIO;
	}

	cfg->irq_config();

	k_thread_create(&data->rx_thread, data->rx_stack,
			CONFIG_ETH_RENESAS_RH850_ETNF_RX_THREAD_STACK_SIZE, eth_etnf_rx_thread,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ETH_RENESAS_RH850_ETNF_RX_THREAD_PRIORITY), 0,
			K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "etnf_rx");

	LOG_INF("ETNF0 10BASE-T1S driver initialized");
	return 0;
}

#define ETH_ETNF_Q_TOPO_INIT(n)                                                                    \
	{                                                                                          \
		.p_tx_queue_config = &eth_etnf_tx_q_##n[0],                                        \
		.p_rx_queue_config = &eth_etnf_rx_q_##n[0],                                        \
		.number_of_tx_queue = (uint8_t)ETH_ETNF_TXQ_NUM(n),                                \
		.number_of_rx_queue = (uint8_t)ETH_ETNF_RXQ_NUM(n),                                \
	}

#define ETH_ETNF_DEFINE_STATIC(n)                                                                  \
	BUILD_ASSERT(ETH_ETNF_TXQ_NUM(n) == 1, "txq-num > 1 is not supported in this version");    \
	BUILD_ASSERT(ETH_ETNF_RXQ_NUM(n) == 1, "rxq-num > 1 is not supported in this version");    \
                                                                                                   \
	static uint8_t g_ether_t1s_mem_pool_buffer_##n[ETH_ETNF_RAM_SIZE(n)]                       \
		BSP_ALIGN_VARIABLE(4);                                                             \
	static const ether_t1s_tx_queue_type_etnf eth_etnf_tx_q_##n[] = {                          \
		ETH_ETNF_TXQ_ENTRY(n, 0),                                                          \
	};                                                                                         \
                                                                                                   \
	static const ether_t1s_rx_queue_type_etnf eth_etnf_rx_q_##n[] = {                          \
		ETH_ETNF_RXQ_ENTRY(n, 0),                                                          \
	};                                                                                         \
                                                                                                   \
	static const ether_t1s_etnf_config_type g_ether_t1s_hw_unit_configuration_etnf_##n = {     \
		.queue_config = ETH_ETNF_Q_TOPO_INIT(n),                                           \
		.rx_config =                                                                       \
			{                                                                          \
				.encf = ETHER_T1S_DISABLE,                                         \
				.esf = ETHER_T1S_AVBNMQUE0,                                        \
				.ets0 = ETHER_T1S_DISABLE,                                         \
				.ets2 = ETHER_T1S_DISABLE,                                         \
				.rfcl = 0x1800U,                                                   \
			},                                                                         \
		.tx_config = ETHER_T1S_AVBDEF,                                                     \
		.ecbs = ETHER_T1S_DISABLE,                                                         \
	};                                                                                         \
                                                                                                   \
	static const ether_t1s_extended_cfg_t eth_etnf_ext_cfg_##n = {                             \
		.eth_mac_addr = DT_INST_PROP(n, local_mac_address),                                \
		.ether_t1s_cfg_type_tx_isr = ETHER_T1S_CTRL_INTERRUPT_MODE,                        \
		.ether_t1s_cfg_type_rx_isr = ETHER_T1S_CTRL_INTERRUPT_MODE,                        \
		.ether_t1s_ctl_mode = ETHER_T1S_CTRL_INTERRUPT_MODE,                               \
		.ether_t1s_filter_action_type = ETHER_T1S_ADD_TO_FILTER,                           \
		.p_mac_address_filter = NULL,                                                      \
		.header_data_custom = ETHER_T1S_HEADER_CUSTOM,                                     \
		.p_hw_unit_config = &g_ether_t1s_hw_unit_configuration_etnf_##n,                   \
		.ether_loopback_config = ETH_ETNF_LOOPBACK_CFG(n),                                 \
		.ether_network_speed = ETHER_T1S_SPEED_10MBPS,                                     \
		.ether_phy_interface = ETHER_T1S_MODE,                                             \
		.p_mac_address_target = NULL,                                                      \
		.number_of_addr_filter = 1U,                                                       \
		.ether_type = 0x003E,                                                              \
		.pma_type = ETH_ETNF_PMA_TYPE(n),                                                  \
		.mdio_interface_type = ETH_ETNF_MDIO_IF(n),                                        \
		.mdio_slave_address = DT_PROP(DT_INST_CHILD(n, phy), phy_lsi_addr),                \
		.ether_t1s_ram_size = ETH_ETNF_RAM_SIZE(n),                                        \
		.p_mem_pool_buffer_table = g_ether_t1s_mem_pool_buffer_##n,                        \
		.other_irq = FSP_INVALID_VECTOR, /* management IRQ, timestamp only */              \
		.interrupt_priority = BSP_IRQ_DISABLED,                                            \
		.time_stamp_sel = ETHER_T1S_TIME_STAMP_MII,                                        \
	};                                                                                         \
                                                                                                   \
	/* Runtime data — MAC address populated from DT */                                         \
	static struct eth_etnf_data eth_etnf_data_##n = {                                          \
		.mac_addr = DT_INST_PROP(n, local_mac_address),                                    \
		.phy_dev = DEVICE_DT_GET(DT_INST_CHILD(n, phy)),                                   \
	};                                                                                         \
                                                                                                   \
	static const ether_cfg_t eth_etnf_fsp_cfg_##n = {                                          \
		.channel = n,                                                                      \
		.zerocopy = IS_ENABLED(CONFIG_ETH_RENESAS_RH850_ETNF_USE_ZEROCOPY)               \
				    ? ETHER_ZEROCOPY_ENABLE                                        \
				    : ETHER_ZEROCOPY_DISABLE,                                      \
		.p_mac_address = eth_etnf_data_##n.mac_addr,                                       \
		.irq = ETH_ETNF_IRQ_NUM(n),                                                        \
		.interrupt_priority = ETH_ETNF_IRQ_PRIO(n),                                        \
		.p_callback = NULL,                                                                \
		.p_context = NULL,                                                                 \
		.p_extend = (const ether_t1s_extended_cfg_t *)&eth_etnf_ext_cfg_##n,               \
	};                                                                                         \
                                                                                                   \
	/* IRQ connect stub */                                                                     \
	static void eth_etnf_irq_config_##n(void)                                                  \
	{                                                                                          \
		IRQ_CONNECT(ETH_ETNF_IRQ_NUM(n), ETH_ETNF_IRQ_PRIO(n), eth_etnf_isr,               \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(ETH_ETNF_IRQ_NUM(n));                                                   \
	}                                                                                          \
                                                                                                   \
	/* Zephyr driver config */                                                                 \
	static const struct eth_etnf_config eth_etnf_config_##n = {                                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.irq_config = eth_etnf_irq_config_##n,                                             \
		.p_fsp_cfg = &eth_etnf_fsp_cfg_##n,                                                \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)(uintptr_t)DT_INST_CLOCKS_CELL(n, clk_id), \
	};

#define ETH_ETNF_DEVICE_DEFINE(n)                                                                  \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	ETH_ETNF_DEFINE_STATIC(n)                                                                  \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, eth_etnf_init, NULL, /* pm_device */                      \
				      &eth_etnf_data_##n, &eth_etnf_config_##n,                    \
				      CONFIG_ETH_INIT_PRIORITY, &eth_etnf_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_ETNF_DEVICE_DEFINE)
