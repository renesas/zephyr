/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_eswm
/* Renesas RX ethernet driver */

#define LOG_MODULE_NAME eth_rx_eswm
#define LOG_LEVEL       CONFIG_ETHERNET_LOG_LEVEL

#define DEV_NODE DT_INST(0, DT_DRV_COMPAT)

#include <soc.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <ethernet/eth_stats.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/net/phy.h>

#include <zephyr/drivers/ethernet/eth_renesas_rx_eswm.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static int renesas_rx_eswm_init(const struct device *dev)
{
	const struct renesas_rx_eswm_config *config = dev->config;
	struct renesas_rx_eswm_data *data = dev->data;
	int ret;
	int fsp_err;

	/* TODO: once Renesas reset drivers are created, use that to reset.
	 * Until then, make sure reset is done in platform init.
	 */

	/* Enable ESW clock */
	ret = clock_control_on(config->eswclk_dev, (clock_control_subsys_t)&config->eswclk_subsys);
	if (ret) {
		LOG_ERR("Failed to enable ESW clock (err %d)", ret);
		return ret;
	}

	/* Enable ESW PHY clock */
	ret = clock_control_on(config->eswphyclk_dev,
			       (clock_control_subsys_t)&config->eswphyclk_subsys);
	if (ret) {
		LOG_ERR("Failed to enable ESW PHY clock (err %d)", ret);
		/* Roll back ESW clock if PHY clock failed */
		clock_control_off(config->eswclk_dev,
				  (clock_control_subsys_t)&config->eswclk_subsys);
		return ret;
	}

	uint32_t iclk_dev_freq;
	uint32_t pclk_dev_freq;
	uint32_t eswclk_dev_freq;
	uint32_t eswphyclk_dev_freq;

	clock_control_get_rate(config->iclk_dev, NULL, &iclk_dev_freq);
	clock_control_get_rate(config->pclk_dev, NULL, &pclk_dev_freq);
	clock_control_get_rate(config->eswphyclk_dev, NULL, &eswphyclk_dev_freq);
	clock_control_get_rate(config->eswclk_dev, NULL, &eswclk_dev_freq);

	/* Clock restrictions for eswm on HM */
	if ((iclk_dev_freq * 1.5 < eswclk_dev_freq) || (eswclk_dev_freq <= pclk_dev_freq) ||
	    (iclk_dev_freq <= pclk_dev_freq)) {
		LOG_ERR("ESWM clock invalid");
		return -EIO;
	}

	data->ether_switch->p_cfg = data->fsp_cfg;
	data->ether_switch->p_ctrl = data->fsp_ctrl;
	data->ether_switch->p_api = &g_ether_switch_on_layer3_switch,

	fsp_err = R_LAYER3_SWITCH_Open(data->fsp_ctrl, data->fsp_cfg);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("ESWM open failed, err=%d", fsp_err);
		return -EIO;
	}



	LOG_DBG("eswclk_dev_freq: %d, eswphyclk_dev_freq: %d ", eswclk_dev_freq,
		eswphyclk_dev_freq);

	return 0;
}

layer3_switch_extended_cfg_t ether_switch_extended_cfg = {
	.fowarding_target_port_masks =
		{
			(LAYER3_SWITCH_PORT_BITMASK_PORT2 | 0U),
			(LAYER3_SWITCH_PORT_BITMASK_PORT2 | 0U),
		},
};
ether_switch_cfg_t ether_switch_cfg = {
	.channel = 0,
	.p_callback = NULL,
	.p_context = NULL,
	.p_extend = &ether_switch_extended_cfg,
};

layer3_switch_instance_ctrl_t ether_switch_ctrl;
ether_switch_instance_t ether_switch_inst;

static struct renesas_rx_eswm_data renesas_rx_eswm_context = {
	.fsp_cfg = &ether_switch_cfg,
	.fsp_ctrl = &ether_switch_ctrl,
	.ether_switch = &ether_switch_inst,
};

static struct renesas_rx_eswm_config renesas_rx_eswm_cfg = {
	.iclk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR_BY_NAME(DEV_NODE, iclk)),
	.pclk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR_BY_NAME(DEV_NODE, pclk)),
	.eswclk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR_BY_NAME(DEV_NODE, esw)),
	.eswphyclk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR_BY_NAME(DEV_NODE, eswphy)),
	.eswclk_subsys =
		{
			.mstp = DT_CLOCKS_CELL_BY_NAME(DEV_NODE, esw, mstp),
			.stop_bit = DT_CLOCKS_CELL_BY_NAME(DEV_NODE, esw, stop_bit),
		},
	.eswphyclk_subsys =
		{
			.mstp = DT_CLOCKS_CELL_BY_NAME(DEV_NODE, eswphy, mstp),
			.stop_bit = DT_CLOCKS_CELL_BY_NAME(DEV_NODE, eswphy, stop_bit),
		},
};

/* Init the module before any enet device inits so priority 0 */
DEVICE_DT_DEFINE(DEV_NODE, renesas_rx_eswm_init, NULL, &renesas_rx_eswm_context, &renesas_rx_eswm_cfg, POST_KERNEL, CONFIG_ETH_INIT_PRIORITY,
		 NULL);
