/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_etnd_mdio

#include <stdint.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/mdio.h>

#include "r_ether_phy.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_renesas_rh850_etnd, CONFIG_MDIO_LOG_LEVEL);

struct mdio_renesas_rh850_etnd_config {
	const struct pinctrl_dev_config *pincfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
};

struct mdio_renesas_rh850_etnd_data {
	struct k_mutex rw_mutex;
	ether_phy_cfg_t ether_phy_cfg;
	ether_phy_instance_ctrl_t ether_phy_ctrl;
};

static int mdio_renesas_rh850_etnd_read(const struct device *dev, uint8_t prtad, uint8_t regad,
					  uint16_t *data)
{
	struct mdio_renesas_rh850_etnd_data *dev_data = dev->data;
	ether_phy_extended_cfg_t *ext_cfg =
		(ether_phy_extended_cfg_t *)dev_data->ether_phy_cfg.p_extend;
	uint32_t reg_val;
	fsp_err_t err;

	k_mutex_lock(&dev_data->rw_mutex, K_FOREVER);

	((ether_phy_lsi_cfg_t *)ext_cfg->p_phy_lsi_cfg_list)->address = prtad;

	err = R_ETHER_PHY_Read(&dev_data->ether_phy_ctrl, regad, &reg_val);

	k_mutex_unlock(&dev_data->rw_mutex);

	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	*data = (uint16_t)reg_val;

	return 0;
}

static int mdio_renesas_rh850_etnd_write(const struct device *dev, uint8_t prtad, uint8_t regad,
					   uint16_t data)
{
	struct mdio_renesas_rh850_etnd_data *dev_data = dev->data;
	ether_phy_extended_cfg_t *ext_cfg =
		(ether_phy_extended_cfg_t *)dev_data->ether_phy_cfg.p_extend;
	fsp_err_t err;

	k_mutex_lock(&dev_data->rw_mutex, K_FOREVER);

	((ether_phy_lsi_cfg_t *)ext_cfg->p_phy_lsi_cfg_list)->address = prtad;

	err = R_ETHER_PHY_Write(&dev_data->ether_phy_ctrl, regad, data);

	k_mutex_unlock(&dev_data->rw_mutex);

	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	return 0;
}

static int mdio_renesas_rh850_etnd_initialize(const struct device *dev)
{
	struct mdio_renesas_rh850_etnd_data *data = dev->data;
	const struct mdio_renesas_rh850_etnd_config *cfg = dev->config;
	fsp_err_t fsp_err;
	int err;

	k_mutex_init(&data->rw_mutex);

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("Ethernet TSN clock device is not ready");
		return -ENODEV;
	}

	err = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if ((err < 0) && (err != -EALREADY)) {
		LOG_ERR("Ethernet TSN module clock enable failed (err %d)", err);
		return err;
	}

	err = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

	/*
	 * MDIO and the ETND MAC share one ETND module: clock_control_on() above
	 * already supplies it, so MDIO works before the MAC is opened.
	 * RMACA.MPIC (MDC timing) is only written by the MAC, in R_ETHER_TSN_Open().
	 * Until then MDIO runs with the MPIC reset value.
	 */
	fsp_err = R_ETHER_PHY_Open(&data->ether_phy_ctrl, &data->ether_phy_cfg);

	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ETHER_PHY_Open failed: %d", fsp_err);
		return -EIO;
	}

	return 0;
}

static DEVICE_API(mdio, mdio_renesas_rh850_etnd_api) = {
	.read = mdio_renesas_rh850_etnd_read,
	.write = mdio_renesas_rh850_etnd_write,
};

#define MDIO_RENESAS_RH850_ETND_DEFINE(n)                                                        \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static ether_phy_lsi_cfg_t mdio_renesas_rh850_etnd_lsi_cfg_##n = {                       \
		.type = ETHER_PHY_LSI_TYPE_DEFAULT,                                                \
	};                                                                                         \
                                                                                                   \
	static const ether_phy_extended_cfg_t mdio_renesas_rh850_etnd_ext_cfg_##n = {            \
		.p_target_init = NULL,                                                             \
		.p_target_link_partner_ability_get = NULL,                                         \
		.p_phy_lsi_cfg_list = &mdio_renesas_rh850_etnd_lsi_cfg_##n,                      \
		.eth_reg = ETHER_PHY_ETH_REG_TSN,                                                  \
	};                                                                                         \
                                                                                                   \
	static struct mdio_renesas_rh850_etnd_data mdio_renesas_rh850_etnd_data_##n = {        \
		.ether_phy_cfg =                                                                   \
			{                                                                          \
				.channel = DT_INST_PROP_OR(n, channel, 0),                         \
				.phy_reset_wait_time = 0x00020000,                                 \
				.mii_bit_access_wait_time = 8,                                     \
				.phy_lsi_type = ETHER_PHY_LSI_TYPE_DEFAULT,                        \
				.flow_control = ETHER_PHY_FLOW_CONTROL_DISABLE,                    \
				.p_extend = &mdio_renesas_rh850_etnd_ext_cfg_##n,                \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	static const struct mdio_renesas_rh850_etnd_config                                       \
		mdio_renesas_rh850_etnd_config_##n = {                                           \
			.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                               \
			.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                        \
			.clock_subsys =                                                            \
				(clock_control_subsys_t)(uintptr_t)DT_INST_CLOCKS_CELL(n, clk_id), \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &mdio_renesas_rh850_etnd_initialize, NULL,                      \
			      &mdio_renesas_rh850_etnd_data_##n,                                 \
			      &mdio_renesas_rh850_etnd_config_##n, POST_KERNEL,                  \
			      CONFIG_MDIO_INIT_PRIORITY, &mdio_renesas_rh850_etnd_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_RENESAS_RH850_ETND_DEFINE)
