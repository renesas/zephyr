/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/mdio.h>
#include <zephyr/drivers/ethernet/eth_renesas_rx_eswm.h>
#include "r_rmac_phy.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(renesas_rx_rmac_mdio, CONFIG_MDIO_LOG_LEVEL);

#define DT_DRV_COMPAT renesas_rx_rmac_mdio

struct renesas_rx_mdio_config {
	const struct pinctrl_dev_config *pincfg;
	const struct device *ethphyclk_dev;                  /* ethphyclk clock */
	struct clock_control_rx_subsys_cfg ethphyclk_subsys; /* ethphyclk clock subsys */
	R_RMAC0_Type *const regs;
	uint8_t channel;
	const struct gpio_dt_spec reset_gpio;
};

struct renesas_rx_mdio_data {
	struct k_mutex rw_mutex;
	struct st_ether_phy_cfg ether_phy_cfg;
	struct st_rmac_phy_instance_ctrl ether_phy_ctrl;
};

extern void r_rmac_phy_set_mii_type_configuration(rmac_phy_instance_ctrl_t *p_instance_ctrl,
						  uint8_t port);

static int renesas_rx_mdio_read(const struct device *dev, uint8_t prtad, uint8_t regad,
				uint16_t *data)
{
	struct renesas_rx_mdio_data *dev_data = dev->data;
	uint32_t read;
	fsp_err_t err;

	k_mutex_lock(&dev_data->rw_mutex, K_FOREVER);

	R_RMAC_PHY_ChipSelect(&dev_data->ether_phy_ctrl, prtad);

	err = R_RMAC_PHY_Read(&dev_data->ether_phy_ctrl, regad, &read);

	k_mutex_unlock(&dev_data->rw_mutex);

	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	*data = read & UINT16_MAX;

	return 0;
}

static int renesas_rx_mdio_write(const struct device *dev, uint8_t prtad, uint8_t regad,
				 uint16_t data)
{
	struct renesas_rx_mdio_data *dev_data = dev->data;
	fsp_err_t err;

	k_mutex_lock(&dev_data->rw_mutex, K_FOREVER);

	R_RMAC_PHY_ChipSelect(&dev_data->ether_phy_ctrl, prtad);

	err = R_RMAC_PHY_Write(&dev_data->ether_phy_ctrl, regad, data);

	if (err) {
		LOG_ERR("R_RMAC_PHY_Write failed!");
	}

	k_mutex_unlock(&dev_data->rw_mutex);

	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	return 0;
}

static int renesas_rx_mdio_initialize(const struct device *dev)
{
	struct renesas_rx_mdio_data *data = dev->data;
	const struct renesas_rx_mdio_config *cfg = dev->config;
	rmac_phy_instance_ctrl_t *p_instance_ctrl =
		(rmac_phy_instance_ctrl_t *)&data->ether_phy_ctrl;
	int err;
	fsp_err_t fsp_err;
	uint32_t clock_dev_freq;

	err = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

	/* Enable ETHPHY clock */
	err = clock_control_on(cfg->ethphyclk_dev, (clock_control_subsys_t)&cfg->ethphyclk_subsys);
	if (err) {
		LOG_ERR("Failed to enable ETHPHY clock (err %d)", err);
		return err;
	}

	err = clock_control_get_rate(cfg->ethphyclk_dev, NULL, &clock_dev_freq);
	if (err) {
		return err;
	}

	LOG_DBG("ETHPHY clock freq: %d", clock_dev_freq);

	/* Set ETHA to CONFIG mode */
	r_layer3_switch_update_etha_operation_mode(cfg->channel, LAYER3_SWITCH_AGENT_MODE_DISABLE);
	r_layer3_switch_update_etha_operation_mode(cfg->channel, LAYER3_SWITCH_AGENT_MODE_CONFIG);

	fsp_err = R_RMAC_PHY_Open(p_instance_ctrl, &data->ether_phy_cfg);

	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("Failed to init mdio driver - R_RMAC_PHY_Open fail, fsp_err: %d", fsp_err);
		return -EIO;
	}

	R_RMAC_PHY_ChipSelect(p_instance_ctrl, cfg->channel);

	r_rmac_phy_set_mii_type_configuration(p_instance_ctrl, cfg->channel);

	k_mutex_init(&data->rw_mutex);

	if (!cfg->reset_gpio.port) {
		LOG_WRN("missing reset port definition");
		return -EINVAL;
	}

	/* configure the reset pin */
	err = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		return err;
	}

	for (uint32_t i = 0; i < 2; i++) {
		/* Start reset */
		err = gpio_pin_set_dt(&cfg->reset_gpio, 1);
		if (err < 0) {
			LOG_WRN("failed to set reset gpio");
			return -EINVAL;
		}

		/* Wait as specified by datasheet */
		k_sleep(K_MSEC(200));

		/* Reset over */
		gpio_pin_set_dt(&cfg->reset_gpio, 0);

		/* After de-asserting reset, must wait before using the config interface */
		k_sleep(K_MSEC(200));
	}

	LOG_DBG("Init MDIO success!");

	return 0;
}

static DEVICE_API(mdio, renesas_rx_mdio_api) = {
	.read = renesas_rx_mdio_read,
	.write = renesas_rx_mdio_write,
};

#define DECLARE_ETHER_PHY_LSI_WRAP(child_node_id)                                                  \
	static const ether_phy_lsi_cfg_t g_phy_lsi_##child_node_id = {                             \
		.address = DT_REG_ADDR(child_node_id),                                             \
		.type = ETHER_PHY_LSI_TYPE_CUSTOM,                                                 \
	};

#define DECLARE_ETHER_PHY_LSI_PTR_WRAP(child_node_id) &g_phy_lsi_##child_node_id

#define RENESAS_RX_MDIO_INSTANCE_DEFINE(n)                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	DT_INST_FOREACH_CHILD(n, DECLARE_ETHER_PHY_LSI_WRAP);                                      \
	static const rmac_phy_extended_cfg_t g_rmac_phy##n##_extended_cfg = {                      \
		.p_target_init = NULL,                                                             \
		.p_target_link_partner_ability_get = NULL,                                         \
		.frame_format = RMAC_PHY_FRAME_FORMAT_MDIO,                                        \
		.mdc_clock_rate = 2500000U,                                                        \
		.mdio_hold_time = 7,                                                               \
		.mdio_capture_time = 0U,                                                           \
		.p_phy_lsi_cfg_list = {DT_INST_FOREACH_CHILD_SEP(                                  \
			n, DECLARE_ETHER_PHY_LSI_PTR_WRAP, (, ))},                                 \
		.default_phy_lsi_cfg_index = 0,                                                    \
	};                                                                                         \
	static struct renesas_rx_mdio_data renesas_rx_mdio##n##_data = {                           \
		.ether_phy_cfg =                                                                   \
			{                                                                          \
				.channel = DT_INST_PROP(n, channel),                               \
				.phy_reset_wait_time = 0x00020000U,                                \
				.mii_bit_access_wait_time = 8U,                                    \
				.mii_type = ETHER_PHY_MII_TYPE_GMII,                               \
				.flow_control = ETHER_PHY_FLOW_CONTROL_DISABLE,                    \
				.p_extend = &g_rmac_phy##n##_extended_cfg,                         \
			},                                                                         \
	};                                                                                         \
	static const struct renesas_rx_mdio_config renesas_rx_mdio##n##_cfg = {                    \
		.channel = DT_INST_PROP(n, channel),                                               \
		.regs = (R_RMAC0_Type *)DT_REG_ADDR(DT_INST_PARENT(n)),                            \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.ethphyclk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                            \
		.ethphyclk_subsys =                                                                \
			{                                                                          \
				.mstp = DT_INST_CLOCKS_CELL(n, mstp),                              \
				.stop_bit = DT_INST_CLOCKS_CELL(n, stop_bit),                      \
			},                                                                         \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                       \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, &renesas_rx_mdio_initialize, NULL, &renesas_rx_mdio##n##_data,    \
			      &renesas_rx_mdio##n##_cfg, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,   \
			      &renesas_rx_mdio_api);

DT_INST_FOREACH_STATUS_OKAY(RENESAS_RX_MDIO_INSTANCE_DEFINE)
