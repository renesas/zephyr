/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_rmac

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rx_rmac, CONFIG_ETHERNET_LOG_LEVEL);

#include <soc.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>
#include <ethernet/eth_stats.h>
#include <zephyr/drivers/pinctrl.h>
#include "eth.h"

#if defined(CONFIG_RENESAS_RX_GRP_INTC_FSP)
#include <zephyr/drivers/interrupt_controller/intc_renesas_rx_grp_int_fsp.h>
#endif /* CONFIG_RENESAS_RX_GRP_INTC_FSP */
#include <zephyr/drivers/ethernet/eth_renesas_rx_eswm.h>
#include "rp_rmac.h"

BUILD_ASSERT((CONFIG_ETH_INIT_PRIORITY < CONFIG_MDIO_INIT_PRIORITY),
	     "Ethernet driver must be initialized before MDIO driver in the Ethernet RX system");

BUILD_ASSERT(
	(LAYER3_SWITCH_CFG_AVAILABLE_QUEUE_NUM < LAYER3_SWITCH_CFG_MAX_QUEUE_NUM),
	"LAYER3_SWITCH_CFG_AVAILABLE_QUEUE_NUM must be less than LAYER3_SWITCH_CFG_MAX_QUEUE_NUM");

/* Additional configurations to use with hal_renesas */
#define ETHER_DEFAULT        NULL
#define ETHER_PADDING_OFFSET 0

extern void rmac_init_buffers(rmac_instance_ctrl_t *const p_instance_ctrl);
extern void rmac_configure_reception_filter(rmac_instance_ctrl_t const *const p_instance_ctrl);
extern void rmac_init_descriptors(rmac_instance_ctrl_t *const p_instance_ctrl);
extern void r_rmac_disable_reception(rmac_instance_ctrl_t *p_instance_ctrl);

typedef enum {
	RX_MII_MODE = 0U,   /*!< MII mode for data interface. */
	RX_RMII_MODE = 1U,  /*!< RMII mode for data interface. */
	RX_GMII_MODE = 2U,  /*!< GMII mode for data interface. */
	RX_RGMII_MODE = 3U, /*!< RGMII mode for data interface. */
	RX_XGMII_MODE = 4U, /*!< XGMII mode for data interface. */
} RX_MiiType;

struct renesas_rx_eth_data {
	struct net_if *iface;
	uint8_t mac[6];
	bool link_is_up;
	enum phy_link_speed link_speed;
	uint8_t tx_buf[NET_ETH_MAX_FRAME_SIZE];
	uint8_t rx_buf[NET_ETH_MAX_FRAME_SIZE];

	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ETH_RENESAS_RX_RMAC_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem rx_sem;
	ether_cfg_t *p_cfg;
	rmac_instance_ctrl_t ctrl;
};

struct renesas_rx_eth_config {
	uint8_t channel;
	/* pinctrl configs */
	const struct pinctrl_dev_config *pcfg;
	/* Use a random MAC address generated when the driver is initialized */
	bool random_mac_address;
	const struct device *eswm_dev;
	const struct device *phy_dev;
	RX_MiiType mii;
	/* Group interrupt controller for the RMAC PHY interrupt (GWDI) */
	const struct device *gwdi_ctrl;
	/* Group interrupt number for the RMAC PHY interrupt (GWDI) */
	uint8_t gwdi_num;
	void *const regs;
};

static enum ethernet_hw_caps renesas_rx_eth_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE | ETHERNET_LINK_1000BASE;
}

void renesas_rx_eth_callback(ether_callback_args_t *p_args)
{
	struct device *dev = (struct device *)p_args->p_context;
	struct renesas_rx_eth_data *data = dev->data;

	switch (p_args->event) {
	case ETHER_EVENT_RX_COMPLETE:
		__fallthrough;
	case ETHER_EVENT_RX_MESSAGE_LOST:
		k_sem_give(&data->rx_sem);
		break;
	default:
		break;
	}
}

static void r_eswm_set_link_speed_configuration(const struct device *dev, enum phy_link_speed speed)
{
	const struct renesas_rx_eth_config *cfg = dev->config;
	R_RMAC0_Type *rmac_reg = (R_RMAC0_Type *)cfg->regs;
	uint8_t mpic_lsc = 0;

	/* Configure ESWM as MII, RMII, or RGMII. */
	switch (speed) {
	case LINK_HALF_10BASE:
		__fallthrough;
	case LINK_FULL_10BASE: {
		mpic_lsc = 0;
		break;
	}

	case LINK_HALF_100BASE:
		__fallthrough;
	case LINK_FULL_100BASE: {
		mpic_lsc = 1;
		break;
	}

	case LINK_HALF_1000BASE:
		__fallthrough;
	case LINK_FULL_1000BASE: {
		mpic_lsc = 2;
		break;
	}

	default: {
		LOG_ERR("invalid link speed!");
		break;
	}
	}

	r_layer3_switch_update_etha_operation_mode(cfg->channel, LAYER3_SWITCH_AGENT_MODE_DISABLE);
	r_layer3_switch_update_etha_operation_mode(cfg->channel, LAYER3_SWITCH_AGENT_MODE_CONFIG);

	rmac_reg->MPIC_b.LSC = mpic_lsc;

	r_layer3_switch_update_etha_operation_mode(cfg->channel, LAYER3_SWITCH_AGENT_MODE_DISABLE);
	r_layer3_switch_update_etha_operation_mode(cfg->channel,
						   LAYER3_SWITCH_AGENT_MODE_OPERATION);
}

static const struct device *renesas_rx_eth_get_phy(const struct device *dev)
{
	const struct renesas_rx_eth_config *config = dev->config;

	return config->phy_dev;
}

static void phy_link_state_changed(const struct device *pdev, struct phy_link_state *state,
				   void *user_data)
{
	ARG_UNUSED(pdev);
	const struct device *dev = (struct device *)user_data;
	struct renesas_rx_eth_data *data = dev->data;
	const struct renesas_rx_eth_config *cfg = dev->config;
	R_RMAC0_Type *rmac_reg = (R_RMAC0_Type *)cfg->regs;
	const rmac_extended_cfg_t *p_extend = (rmac_extended_cfg_t *)data->ctrl.p_cfg->p_extend;
	fsp_err_t err;
	bool is_up;
	enum phy_link_speed speed;

	is_up = state->is_up;
	speed = state->speed;
	if (is_up != data->link_is_up) {
		data->link_is_up = is_up;
		if (is_up) {
			if (speed != data->link_speed) {
				data->link_speed = speed;
				r_eswm_set_link_speed_configuration(dev, speed);
			}

			data->ctrl.link_establish_status = ETHER_LINK_ESTABLISH_STATUS_UP;

			LOG_DBG("cfg->channel: %d", cfg->channel);
			rmac_init_buffers(&data->ctrl);

			/* Initialize receive and transmit descriptors */
			rmac_init_descriptors(&data->ctrl);

			/* Configure reception filters. */
			rmac_configure_reception_filter(&data->ctrl);

			err = rpmac_do_link(&data->ctrl,
					    LAYER3_SWITCH_MAGIC_PACKET_DETECTION_DISABLE);

			if (err != 0 && err != FSP_ERR_IN_USE) {
				LOG_ERR("rmac do link failed! error: %d", err);
			}

			LOG_DBG("phy link changed, state->is_up:%d, RMAC.MPIC %d \n", state->is_up,
				rmac_reg->MPIC);

			rx_grp_intc_callback_set(cfg->gwdi_ctrl, cfg->gwdi_num,
						 layer3_switch_gwdi_isr,
						 (void *)p_extend->p_ether_switch->p_ctrl);

			/* Enable Group interrupt.*/
			rx_grp_intc_enable(cfg->gwdi_ctrl, cfg->gwdi_num);

			k_thread_resume(&data->thread);
			net_eth_carrier_on(data->iface);

		} else {
			/* Disable reception. */
			k_thread_suspend(&data->thread);
			data->ctrl.link_establish_status = ETHER_LINK_ESTABLISH_STATUS_DOWN;
			r_rmac_disable_reception(&data->ctrl);
			net_eth_carrier_off(data->iface);
		}
	}
}

static void renesas_rx_eth_initialize(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct renesas_rx_eth_data *data = dev->data;
	const struct renesas_rx_eth_config *cfg = dev->config;

	net_if_set_link_addr(iface, data->mac, sizeof(data->mac), NET_LINK_ETHERNET);

	data->iface = iface;

	ethernet_init(iface);

	if (device_is_ready(cfg->phy_dev)) {
		phy_link_callback_set(cfg->phy_dev, &phy_link_state_changed, (void *)dev);
	} else {
		LOG_ERR("PHY device not ready");
	}

	/* Do not start the interface until PHY link is up */
	net_if_carrier_off(iface);
}

static int renesas_rx_eth_tx(const struct device *dev, struct net_pkt *pkt)
{
	fsp_err_t err = FSP_SUCCESS;
	struct renesas_rx_eth_data *data = dev->data;
	uint16_t len = net_pkt_get_len(pkt);
	int ret = 0;

	if (net_pkt_read(pkt, data->tx_buf, len)) {
		goto error;
	}

	/* Check if packet length is less than minimum Ethernet frame size */
	if (len < NET_ETH_MINIMAL_FRAME_SIZE) {
		/* Add padding to meet the minimum frame size */
		memset(data->tx_buf + len, 0, NET_ETH_MINIMAL_FRAME_SIZE - len);
		len = NET_ETH_MINIMAL_FRAME_SIZE;
	}

	err = R_RMAC_Write(&data->ctrl, data->tx_buf, len);

	if (err != FSP_SUCCESS) {
		LOG_DBG("RMAC write fail, ERR: %d", err);
		ret = (err == FSP_ERR_ETHER_ERROR_TRANSMIT_BUFFER_FULL) ? -ENOBUFS : -EIO;
		goto error;
	}
	return ret;

error:
	LOG_ERR("Writing to FIFO failed");
	return ret;
}

static const struct ethernet_api api_funcs = {
	.iface_api.init = renesas_rx_eth_initialize,
	.get_capabilities = renesas_rx_eth_get_capabilities,
	.send = renesas_rx_eth_tx,
	.get_phy = renesas_rx_eth_get_phy,
};

static struct net_pkt *renesas_rx_eth_rx(const struct device *dev)
{
	fsp_err_t err = FSP_SUCCESS;
	struct renesas_rx_eth_data *data;
	struct net_pkt *pkt = NULL;
	uint32_t len = 0;

	__ASSERT_NO_MSG(dev != NULL);
	data = dev->data;
	__ASSERT_NO_MSG(data != NULL);

	err = R_RMAC_Read(&data->ctrl, data->rx_buf, &len);
	if ((err != FSP_SUCCESS) && (err != FSP_ERR_ETHER_ERROR_NO_DATA)) {
		LOG_ERR("Failed to read packets");
		goto out;
	}

	pkt = net_pkt_rx_alloc_with_buffer(data->iface, len, AF_UNSPEC, 0, K_MSEC(100));
	if (!pkt) {
		LOG_ERR("Failed to obtain RX buffer");
		goto out;
	}

	if (net_pkt_write(pkt, data->rx_buf, len)) {
		LOG_ERR("Failed to append RX buffer to context buffer");
		net_pkt_unref(pkt);
		pkt = NULL;
		goto out;
	}

out:
	if (pkt == NULL) {
		eth_stats_update_errors_rx(data->iface);
	}

	return pkt;
}

static void renesas_rx_eth_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct net_if *iface;
	int res;
	struct net_pkt *pkt = NULL;
	struct renesas_rx_eth_data *data = dev->data;

	while (true) {
		res = k_sem_take(&data->rx_sem, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
		if (res == 0) {
			pkt = renesas_rx_eth_rx(dev);

			if (pkt != NULL) {
				iface = net_pkt_iface(pkt);
				res = net_recv_data(iface, pkt);
				if (res < 0) {
					net_pkt_unref(pkt);
				}
			}
		}
	}
}

static int renesas_rx_eth_init(const struct device *dev)
{
	struct renesas_rx_eth_data *data = dev->data;
	const struct renesas_rx_eth_config *config = dev->config;
	const struct device *eswm = config->eswm_dev;
	struct renesas_rx_eswm_data *eswm_data = eswm->data;
	uint8_t ret = 0;
	uint8_t fsp_err = 0;

	data->link_is_up = false;
	data->link_speed = -1;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret != 0) {
		return ret;
	}

	if (!device_is_ready(eswm)) {
		LOG_ERR("eswm device is not init before!");
		return -EIO;
	}

	rmac_extended_cfg_t *rmac_extend_cfg = (rmac_extended_cfg_t *)data->p_cfg->p_extend;

	if (eswm_data->ether_switch == NULL) {
		LOG_ERR("ether_switch is NULL in eswm_data");
		return -EINVAL;
	}

	rmac_extend_cfg->p_ether_switch = eswm_data->ether_switch;

	/* Generate or fetch MAC address */
	if (config->random_mac_address == true) {
		gen_random_mac(data->mac, 0x74, 0x90, 0x50);
	}

	memcpy((uint8_t *)data->p_cfg->p_mac_address, data->mac, sizeof(data->mac));

	fsp_err = R_RMAC_Open(&data->ctrl, data->p_cfg);

	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_RMAC_Open fail: %d", fsp_err);
		return -EIO;
	}

	fsp_err = R_RMAC_CallbackSet(&data->ctrl, renesas_rx_eth_callback, (void *)dev, NULL);

	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_LAYER3_SWITCH_CallbackSet fail");
		R_RMAC_Close(&data->ctrl);
		return -EIO;
	}

	k_thread_create(&data->thread, data->thread_stack,
			CONFIG_ETH_RENESAS_RX_RMAC_THREAD_STACK_SIZE, renesas_rx_eth_thread,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ETH_RENESAS_RX_RMAC_THREAD_PRIORITY), 0, K_NO_WAIT);

	/* start suspended and resume once we have link */
	k_thread_suspend(&data->thread);
	k_thread_name_set(&data->thread, "mac-rx-thread");

	return 0;
}

#ifdef CONFIG_RENESAS_RX_GRP_INTC_FSP
#define ETHER_RX_RMAC_GRP_INTC_CONFIG_INIT(index)                                                  \
	.gwdi_ctrl = DEVICE_DT_GET(DT_INST_IRQ_INTC_BY_NAME(index, gwdi)),                         \
	.gwdi_num = DT_INST_IRQ_BY_NAME(index, gwdi, irq),
#else
#define ETHER_RX_RMAC_GRP_INTC_CONFIG_INIT(index)
#endif

#define ETH_RENESAS_RX_RMAC_TX_QUEUE_NUM(n) DT_INST_PROP(n, tx_queue_num)
#define ETH_RENESAS_RX_RMAC_RX_QUEUE_NUM(n) DT_INST_PROP(n, rx_queue_num)
#define ETH_RENESAS_RX_RMAC_NUM_TX_BUF(n)   DT_INST_PROP(n, tx_buf_num)
#define ETH_RENESAS_RX_RMAC_NUM_RX_BUF(n)   DT_INST_PROP(n, rx_buf_num)

#define ETH_RENESAS_RX_RMAC_TX_QUEUE_LENGTH(n)                                                     \
	(ETH_RENESAS_RX_RMAC_NUM_TX_BUF(n) / ETH_RENESAS_RX_RMAC_TX_QUEUE_NUM(n)) + 1
#define ETH_RENESAS_RX_RMAC_RX_QUEUE_LENGTH(n)                                                     \
	(ETH_RENESAS_RX_RMAC_NUM_RX_BUF(n) / ETH_RENESAS_RX_RMAC_RX_QUEUE_NUM(n)) + 1

#define DECLARE_ETHER_RX_BUFFER_WRAP(idx, n)                                                       \
	uint8_t g_ether##n##_ether_rx_buffer##idx[CONFIG_ETH_RENESAS_RX_RMAC_BUF_SIZE];

#define DECLARE_ETHER_TX_BUFFER_WRAP(idx, n)                                                       \
	uint8_t g_ether##n##_ether_tx_buffer##idx[CONFIG_ETH_RENESAS_RX_RMAC_BUF_SIZE];

#define DECLARE_ETHER_RX_BUFFER_PTR_WRAP(idx, n) (uint8_t *)&g_ether##n##_ether_rx_buffer##idx[0]
#define DECLARE_ETHER_TX_BUFFER_PTR_WRAP(idx, n) (uint8_t *)&g_ether##n##_ether_tx_buffer##idx[0]

/* Descriptor array per queue */
#define DECLARE_ETHER_TX_DESCRIPTOR_WRAP(idx, n)                                                   \
	layer3_switch_descriptor_t                                                                 \
		g_ether##n##_tx_descriptor_array##idx[ETH_RENESAS_RX_RMAC_TX_QUEUE_LENGTH(n)];

#define DECLARE_ETHER_RX_DESCRIPTOR_WRAP(idx, n)                                                   \
	layer3_switch_descriptor_t                                                                 \
		g_ether##n##_rx_descriptor_array##idx[ETH_RENESAS_RX_RMAC_RX_QUEUE_LENGTH(n)];

/* TX queue wrapper */
#define DECLARE_TX_QUEUE_WRAP(idx, n)                                                              \
	{                                                                                          \
		.queue_cfg = {                                                                     \
			.array_length = ETH_RENESAS_RX_RMAC_TX_QUEUE_LENGTH(n),                    \
			.p_descriptor_array = g_ether##n##_tx_descriptor_array##idx,               \
			.ports = (1 << DT_INST_PROP(n, channel)),                                  \
			.type = LAYER3_SWITCH_QUEUE_TYPE_TX,                                       \
			.write_back_mode = LAYER3_SWITCH_WRITE_BACK_MODE_FULL,                     \
			.descriptor_format = LAYER3_SWITCH_DISCRIPTOR_FORMTAT_EXTENDED,            \
		}                                                                                  \
	}

/* RX queue wrapper */
#define DECLARE_RX_QUEUE_WRAP(idx, n)                                                              \
	{                                                                                          \
		.queue_cfg = {                                                                     \
			.array_length = ETH_RENESAS_RX_RMAC_RX_QUEUE_LENGTH(n),                    \
			.p_descriptor_array = g_ether##n##_rx_descriptor_array##idx,               \
			.ports = (1 << DT_INST_PROP(n, channel)),                                  \
			.type = LAYER3_SWITCH_QUEUE_TYPE_RX,                                       \
			.write_back_mode = LAYER3_SWITCH_WRITE_BACK_MODE_FULL,                     \
			.descriptor_format = LAYER3_SWITCH_DISCRIPTOR_FORMTAT_EXTENDED,            \
		}                                                                                  \
	}

#define DECLARE_ETHER_BUFFERS(n)                                                                   \
	LISTIFY(ETH_RENESAS_RX_RMAC_NUM_RX_BUF(n), DECLARE_ETHER_RX_BUFFER_WRAP, (;), n)                                                                                    \
	LISTIFY(ETH_RENESAS_RX_RMAC_NUM_TX_BUF(n), DECLARE_ETHER_TX_BUFFER_WRAP, (;), n)                                                                                    \
	uint8_t *pp_g_ether##n##_ether_buffers[ETH_RENESAS_RX_RMAC_NUM_RX_BUF(n) +                 \
					       ETH_RENESAS_RX_RMAC_NUM_TX_BUF(n)] = {              \
		LISTIFY(ETH_RENESAS_RX_RMAC_NUM_RX_BUF(n), DECLARE_ETHER_RX_BUFFER_PTR_WRAP,   \
			(, ), n),                                \
			LISTIFY(ETH_RENESAS_RX_RMAC_NUM_TX_BUF(n), DECLARE_ETHER_TX_BUFFER_PTR_WRAP,   \
			(, ), n)}

#define DECLARE_ETHER_DESCRIPTOR(n)                                                                \
	LISTIFY(ETH_RENESAS_RX_RMAC_RX_QUEUE_NUM(n), DECLARE_ETHER_RX_DESCRIPTOR_WRAP, (;), n)                                                                                    \
	LISTIFY(ETH_RENESAS_RX_RMAC_TX_QUEUE_NUM(n), DECLARE_ETHER_TX_DESCRIPTOR_WRAP, (;), n)

#define ETHER_RX_CONFIG(n)                                                                         \
	/* Descriptor arrays */                                                                    \
	DECLARE_ETHER_DESCRIPTOR(n);                                                               \
	/* TX queue list */                                                                        \
	rmac_queue_info_t g_ether##n##_tx_queue_list[ETH_RENESAS_RX_RMAC_TX_QUEUE_NUM(n)] = {      \
		LISTIFY(ETH_RENESAS_RX_RMAC_TX_QUEUE_NUM(n), DECLARE_TX_QUEUE_WRAP, (, ), n)};         \
	/* RX queue list */                                                                        \
	rmac_queue_info_t g_ether##n##_rx_queue_list[ETH_RENESAS_RX_RMAC_RX_QUEUE_NUM(n)] = {      \
		LISTIFY(ETH_RENESAS_RX_RMAC_RX_QUEUE_NUM(n), DECLARE_RX_QUEUE_WRAP, (, ), n)};         \
	DECLARE_ETHER_BUFFERS(n);                                                                  \
	uint8_t g_ether##n##_mac_address[6] = DT_INST_PROP_OR(n, local_mac_address, {0});          \
	rmac_extended_cfg_t g_ether##n##_extended_cfg = {                                          \
		.tx_queue_num = ETH_RENESAS_RX_RMAC_TX_QUEUE_NUM(n),                               \
		.rx_queue_num = ETH_RENESAS_RX_RMAC_RX_QUEUE_NUM(n),                               \
		.p_tx_queue_list = g_ether##n##_tx_queue_list,                                     \
		.p_rx_queue_list = g_ether##n##_rx_queue_list,                                     \
	};                                                                                         \
	ether_cfg_t g_ether##n##_cfg = {                                                           \
		.channel = DT_INST_PROP(n, channel),                                               \
		.zerocopy = ETHER_ZEROCOPY_DISABLE,                                                \
		.multicast = ETHER_MULTICAST_ENABLE,                                               \
		.promiscuous = ETHER_PROMISCUOUS_DISABLE,                                          \
		.flow_control = ETHER_FLOW_CONTROL_DISABLE,                                        \
		.padding = ETHER_PADDING_DISABLE,                                                  \
		.padding_offset = ETHER_PADDING_OFFSET,                                            \
		.broadcast_filter = CONFIG_ETH_RENESAS_RX_RMAC_BROADCAST_FILTER,                   \
		.p_mac_address = g_ether##n##_mac_address,                                         \
		.num_tx_descriptors = ETH_RENESAS_RX_RMAC_NUM_TX_BUF(n),                           \
		.num_rx_descriptors = ETH_RENESAS_RX_RMAC_NUM_RX_BUF(n),                           \
		.pp_ether_buffers = pp_g_ether##n##_ether_buffers,                                 \
		.ether_buffer_size = CONFIG_ETH_RENESAS_RX_RMAC_BUF_SIZE,                          \
		.p_callback = ETHER_DEFAULT,                                                       \
		.p_context = ETHER_DEFAULT,                                                        \
		.p_extend = &g_ether##n##_extended_cfg,                                            \
	};

#define ETHER_RX_INIT(n)                                                                           \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	ETHER_RX_CONFIG(n)                                                                         \
	static struct renesas_rx_eth_data eth_##n##_data = {                                       \
		.rx_sem = Z_SEM_INITIALIZER(eth_##n##_data.rx_sem, 0, UINT8_MAX),                  \
		.mac = DT_INST_PROP_OR(n, local_mac_address, {0}),                                 \
		.p_cfg = &g_ether##n##_cfg,                                                        \
	};                                                                                         \
	static struct renesas_rx_eth_config eth_##n##_config = {                                   \
		.channel = DT_INST_PROP(n, channel),                                               \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.random_mac_address = DT_INST_PROP(n, zephyr_random_mac_address),                  \
		.mii = DT_INST_ENUM_IDX(n, phy_connection_type),                                   \
		.eswm_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                      \
		.phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, phy_handle)),                          \
		.regs = (R_RMAC0_Type *)DT_INST_REG_ADDR(n),                                       \
		ETHER_RX_RMAC_GRP_INTC_CONFIG_INIT(n)};                                            \
	static int renesas_rx_eth_init##n(const struct device *dev)                                \
	{                                                                                          \
		int err = renesas_rx_eth_init(dev);                                                \
		if (err != 0) {                                                                    \
			return err;                                                                \
		}                                                                                  \
		return 0;                                                                          \
	}                                                                                          \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, renesas_rx_eth_init##n, NULL, &eth_##n##_data,            \
				      &eth_##n##_config, CONFIG_ETH_INIT_PRIORITY, &api_funcs,     \
				      NET_ETH_MTU /*MTU*/);

DT_INST_FOREACH_STATUS_OKAY(ETHER_RX_INIT);
