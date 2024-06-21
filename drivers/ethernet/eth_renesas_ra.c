/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_ra_ethernet

/* Renesas RA ethernet driver */

#define LOG_MODULE_NAME eth_ra_ethernet
#define LOG_LEVEL       CONFIG_ETHERNET_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <soc.h>
#include <zephyr/device.h>
#include <errno.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/irq.h>
#include "ethernet/eth_stats.h"
#include <instances/r_ether.h>
#include <instances/r_ether_phy.h>
#include <zephyr/drivers/pinctrl.h>
#include <instances/r_dtc.h>

void ether_eint_isr(void);
void renesas_ra_eth_callback(ether_callback_args_t *p_args);

uint8_t g_ether0_mac_address[6] = DT_INST_PROP(0, local_mac_address);
struct renesas_ra_eth_context {
	struct net_if *iface;
	uint8_t mac[6];

	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ETH_RA_RX_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem rx_sem;
	bool link_up;
	ether_instance_ctrl_t ctrl;
	/** pinctrl configs */
	const struct pinctrl_dev_config *pcfg;
};

struct renesas_ra_eth_config {
	const ether_cfg_t *p_cfg;
};

#define ETHER_EXAMPLE_ETHER_ISR_EE_FR_MASK (1UL << 18)

__aligned(32) uint8_t g_ether0_ether_buffer0[1536];
__aligned(32) uint8_t g_ether0_ether_buffer1[1536];
__aligned(32) uint8_t g_ether0_ether_buffer2[1536];
__aligned(32) uint8_t g_ether0_ether_buffer3[1536];
__aligned(32) uint8_t g_ether0_ether_buffer4[1536];
__aligned(32) uint8_t g_ether0_ether_buffer5[1536];
__aligned(32) uint8_t g_ether0_ether_buffer6[1536];
__aligned(32) uint8_t g_ether0_ether_buffer7[1536];

uint8_t *pp_g_ether0_ether_buffers[8] = {
	(uint8_t *)&g_ether0_ether_buffer0[0], (uint8_t *)&g_ether0_ether_buffer1[0],
	(uint8_t *)&g_ether0_ether_buffer2[0], (uint8_t *)&g_ether0_ether_buffer3[0],
	(uint8_t *)&g_ether0_ether_buffer4[0], (uint8_t *)&g_ether0_ether_buffer5[0],
	(uint8_t *)&g_ether0_ether_buffer6[0], (uint8_t *)&g_ether0_ether_buffer7[0],
};

__aligned(16) ether_instance_descriptor_t g_ether0_tx_descriptors[4];
__aligned(16) ether_instance_descriptor_t g_ether0_rx_descriptors[4];

const ether_extended_cfg_t g_ether0_extended_cfg_t = {
	.p_rx_descriptors = g_ether0_rx_descriptors,
	.p_tx_descriptors = g_ether0_tx_descriptors,
};

const ether_phy_extended_cfg_t g_ether_phy0_extended_cfg = {
	.p_target_init = NULL, .p_target_link_partner_ability_get = NULL};

const ether_phy_cfg_t g_ether_phy0_cfg = {
	.channel = 0,
	.phy_lsi_address = 5,
	.phy_reset_wait_time = 0x00020000,
	.mii_bit_access_wait_time = 8,
	.phy_lsi_type = ETHER_PHY_LSI_TYPE_KIT_COMPONENT,
	.flow_control = ETHER_PHY_FLOW_CONTROL_DISABLE,
	.mii_type = ETHER_PHY_MII_TYPE_RMII,
	.p_context = NULL,
	.p_extend = &g_ether_phy0_extended_cfg,
};

ether_phy_instance_ctrl_t g_ether_phy0_ctrl;

const ether_phy_instance_t g_ether_phy0 = {.p_ctrl = &g_ether_phy0_ctrl,
					   .p_cfg = &g_ether_phy0_cfg,
					   .p_api = &g_ether_phy_on_ether_phy};

const ether_cfg_t g_ether0_cfg = {
	.channel = 0,
	.zerocopy = ETHER_ZEROCOPY_DISABLE,
	.multicast = ETHER_MULTICAST_ENABLE,
	.promiscuous = ETHER_PROMISCUOUS_DISABLE,
	.flow_control = ETHER_FLOW_CONTROL_DISABLE,
	.padding = ETHER_PADDING_DISABLE,
	.padding_offset = 1,
	.broadcast_filter = 0,
	.p_mac_address = g_ether0_mac_address,
	.num_tx_descriptors = 4,
	.num_rx_descriptors = 4,
	.pp_ether_buffers = pp_g_ether0_ether_buffers,
	.ether_buffer_size = 1536,
	.irq = DT_INST_IRQN(0),
	.interrupt_priority = DT_INST_IRQ(0, priority),
	.p_callback = NULL,
	.p_ether_phy_instance = &g_ether_phy0,
	.p_context = NULL,
	.p_extend = &g_ether0_extended_cfg_t,
};

static struct renesas_ra_eth_config eth_0_config = {
	.p_cfg = &g_ether0_cfg,
};

/* Driver functions */
static enum ethernet_hw_caps renesas_ra_eth_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T;
}

void renesas_ra_eth_callback(ether_callback_args_t *p_args)
{
	struct device *dev = (struct device *)p_args->p_context;
	struct renesas_ra_eth_context *ctx = dev->data;

	if (ETHER_EXAMPLE_ETHER_ISR_EE_FR_MASK ==
	    (p_args->status_eesr & ETHER_EXAMPLE_ETHER_ISR_EE_FR_MASK)) {
		k_sem_give(&ctx->rx_sem);
	}
}

static void renesas_ra_eth_initialize(struct net_if *iface)
{
	fsp_err_t err = FSP_SUCCESS;
	const struct device *dev = net_if_get_device(iface);
	struct renesas_ra_eth_context *ctx = dev->data;
	const struct renesas_ra_eth_config *cfg = dev->config;

	LOG_DBG("eth_initialize");

	net_if_set_link_addr(iface, ctx->mac, sizeof(ctx->mac), NET_LINK_ETHERNET);

	if (ctx->iface == NULL) {
		ctx->iface = iface;
	}

	ethernet_init(iface);

	err = R_ETHER_Open(&ctx->ctrl, cfg->p_cfg);

	if (err != FSP_SUCCESS) {
		LOG_ERR("Failed to init ether");
	}

	err = R_ETHER_CallbackSet(&ctx->ctrl, renesas_ra_eth_callback, dev, NULL);

	if (err != FSP_SUCCESS) {
		LOG_ERR("Failed to init ether");
	}
}

static int renesas_ra_eth_tx(const struct device *dev, struct net_pkt *pkt)
{
	fsp_err_t err = FSP_SUCCESS;
	struct renesas_ra_eth_context *ctx = dev->data;
	uint16_t len = net_pkt_get_len(pkt);
	static uint8_t tx_buf[NET_ETH_MAX_FRAME_SIZE] __aligned(4);

	if (net_pkt_read(pkt, tx_buf, len)) {
		goto error;
	}

	/* Check if packet length is less than minimum Ethernet frame size */
	if (len < NET_ETH_MINIMAL_FRAME_SIZE) {
		/* Add padding to meet the minimum frame size */
		memset(tx_buf + len, 0, NET_ETH_MINIMAL_FRAME_SIZE - len);
		len = NET_ETH_MINIMAL_FRAME_SIZE;
	}

	if (ctx->link_up == true) {
		err = R_ETHER_Write(&ctx->ctrl, tx_buf, len);
		if (err != FSP_SUCCESS) {
			goto error;
		}
	} else {
		return -1;
	}

	return 0;

error:
	LOG_ERR("Writing to FIFO failed");
	return -1;
}

static const struct ethernet_api api_funcs = {
	.iface_api.init = renesas_ra_eth_initialize,
	.get_capabilities = renesas_ra_eth_get_capabilities,
	.send = renesas_ra_eth_tx,
};

static void renesas_ra_eth_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	ether_eint_isr();
}

static struct net_pkt *renesas_ra_eth_rx(const struct device *dev)
{
	fsp_err_t err = FSP_SUCCESS;
	struct renesas_ra_eth_context *ctx;
	struct net_pkt *pkt = NULL;
	uint32_t len = 0;
	static uint8_t rx_buf[NET_ETH_MAX_FRAME_SIZE] __aligned(4);

	__ASSERT_NO_MSG(dev != NULL);
	ctx = dev->data;
	__ASSERT_NO_MSG(ctx != NULL);

	err = R_ETHER_Read(&ctx->ctrl, rx_buf, &len);
	if ((err != FSP_SUCCESS) && (err != FSP_ERR_ETHER_ERROR_NO_DATA)) {
		LOG_ERR("Failed to read packets");
		goto out;
	}

	pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, len, AF_UNSPEC, 0, K_MSEC(100));
	if (!pkt) {
		LOG_ERR("Failed to obtain RX buffer");
		goto out;
	}

	if (net_pkt_write(pkt, rx_buf, len)) {
		LOG_ERR("Failed to append RX buffer to context buffer");
		net_pkt_unref(pkt);
		pkt = NULL;
		goto out;
	}

out:
	if (!pkt) {
		eth_stats_update_errors_rx(ctx->iface);
	}

	return pkt;
}

static void renesas_ra_eth_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct net_if *iface;
	int res;
	struct net_pkt *pkt = NULL;
	struct renesas_ra_eth_context *ctx = dev->data;

	while (true) {
		res = k_sem_take(&ctx->rx_sem, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
		if (res == 0) {
			/* semaphore taken, update link status and receive packets */
			if (ctx->link_up != true) {
				res = R_ETHER_LinkProcess(&ctx->ctrl);
				if (res == FSP_SUCCESS) {
					ctx->link_up = true;
					net_eth_carrier_on(ctx->iface);
				}
			}

			pkt = renesas_ra_eth_rx(dev);

			if (pkt != NULL) {
				iface = net_pkt_iface(pkt);
				res = net_recv_data(iface, pkt);
				if (res < 0) {
					net_pkt_unref(pkt);
				}
			}
		} else if (res == -EAGAIN) {
			res = R_ETHER_LinkProcess(&ctx->ctrl);
			if (res == FSP_SUCCESS) {
				if (ctx->ctrl.link_establish_status ==
				    ETHER_LINK_ESTABLISH_STATUS_UP) {
					ctx->link_up = true;
					net_eth_carrier_on(ctx->iface);
				} else {
					ctx->link_up = false;
					net_eth_carrier_off(ctx->iface);
				}
			}
		}
	}
}

#define ELC_EVENT_EDMAC_EINT(channel) ELC_EVENT_EDMAC##channel##_EINT

/* Bindings to the platform */
int renesas_ra_eth_init(const struct device *dev)
{
	struct renesas_ra_eth_context *ctx = dev->data;
	int ret;

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(ctx->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	R_ICU->IELSR[DT_INST_IRQN(0)] = ELC_EVENT_EDMAC_EINT(0);

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), renesas_ra_eth_isr,
		    DEVICE_DT_INST_GET(0), 0);

	k_thread_create(&ctx->thread, ctx->thread_stack, CONFIG_ETH_RA_RX_THREAD_STACK_SIZE,
			renesas_ra_eth_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ETH_RA_RX_THREAD_PRIORITY), 0, K_NO_WAIT);

	irq_enable(DT_INST_IRQN(0));

	return 0;
}

#define ETHER_RA_INIT(idx)                                                                         \
	PINCTRL_DT_INST_DEFINE(0);                                                                 \
	static struct renesas_ra_eth_context eth_0_context = {                                     \
		.mac = DT_INST_PROP(0, local_mac_address),                                         \
		.rx_sem = Z_SEM_INITIALIZER(eth_0_context.rx_sem, 0, UINT_MAX),                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),                                         \
	};                                                                                         \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(0, renesas_ra_eth_init, NULL, &eth_0_context, &eth_0_config, \
				      CONFIG_ETH_INIT_PRIORITY, &api_funcs, NET_ETH_MTU /*MTU*/);

DT_INST_FOREACH_STATUS_OKAY(ETHER_RA_INIT);
