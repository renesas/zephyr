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

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(LOG_MODULE_NAME);

struct renesas_rx_eswm_config {
	const struct device *eswclk_dev;                     /* eswclk clock */
	const struct device *eswphyclk_dev;                  /* eswphyclk clock */
	struct clock_control_rx_subsys_cfg eswclk_subsys;    /* eswclk clock subsys */
	struct clock_control_rx_subsys_cfg eswphyclk_subsys; /* eswphyclk clock subsys */
	/* pinctrl configs */
	const struct pinctrl_dev_config *pcfg;
};

static int renesas_rx_eswm_init(const struct device *dev)
{
	const struct renesas_rx_eswm_config *config = dev->config;
	int ret;

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
	uint32_t eswclk_dev_freq;
	uint32_t eswphyclk_dev_freq;
	ret = clock_control_get_rate(config->eswphyclk_dev, NULL, &eswphyclk_dev_freq);
	if (ret) {
		return ret;
	}

	ret = clock_control_get_rate(config->eswclk_dev, NULL, &eswclk_dev_freq);
	if (ret) {
		return ret;
	}

	LOG_DBG("eswclk_dev_freq: %d, eswphyclk_dev_freq: %d ", eswclk_dev_freq,
		eswphyclk_dev_freq);

	return 0;
}
PINCTRL_DT_DEFINE(DEV_NODE);
static const struct renesas_rx_eswm_config renesas_rx_eswm_cfg = {
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
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DEV_NODE),
};

/* Init the module before any enet device inits so priority 0 */
DEVICE_DT_DEFINE(DEV_NODE, renesas_rx_eswm_init, NULL, NULL, &renesas_rx_eswm_cfg, POST_KERNEL, 0,
		 NULL);
