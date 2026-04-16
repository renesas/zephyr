/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_ct25205_phy

#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>
#include "r_ether_phy.h"
#include "r_ether_phy_api.h"

#define LOG_MODULE_NAME phy_renesas_ct25205
#define LOG_LEVEL       CONFIG_PHY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define ETH_PHY_CT25205_REG_PLCACTRL0 (0x3FCA01U)
#define ETH_PHY_CT25205_REG_PLCACTRL1 (0x3FCA02U)
#define ETH_PHY_CT25205_REG_PLCAST    (0x3FCA03U)
#define ETH_PHY_CT25205_REG_PLCAIDVER (0x3FCA00U)
#define ETH_PHY_CT25205_REG_PLCATOTMR (0x3FCA04U)
#define ETH_PHY_CT25205_REG_PLCABURST (0x3FCA05U)

struct renesas_ct25205_config {
	const ether_phy_cfg_t *p_cfg;
};

struct renesas_ct25205_data {
	const struct device *dev;
	ether_phy_instance_ctrl_t *p_ctrl;
	struct phy_link_state state;
	phy_callback_t cb;
	void *cb_data;
	struct k_work_delayable phy_monitor_work;
};

static int phy_renesas_ct25205_get_link(const struct device *dev, struct phy_link_state *state)
{
	struct renesas_ct25205_data *data = dev->data;
	fsp_err_t fsp_err = FSP_SUCCESS;

	fsp_err = R_ETHER_PHY_LinkStatusGet(data->p_ctrl);

	if (FSP_SUCCESS != fsp_err) {
		state->is_up = false;
	} else {
		state->is_up = true;
	}

	state->speed = LINK_HALF_10BASE;

	return 0;
}

static int phy_renesas_ct25205_link_cb_set(const struct device *dev, phy_callback_t cb,
					   void *user_data)
{
	struct renesas_ct25205_data *data = dev->data;
	struct phy_link_state state;

	data->cb = cb;
	data->cb_data = user_data;

	if (data->cb) {
		state.is_up = data->state.is_up;
		state.speed = LINK_HALF_10BASE;
		data->cb(dev, &state, data->cb_data);
	}

	return 0;
}

static void phy_monitor_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct renesas_ct25205_data *const data =
		CONTAINER_OF(dwork, struct renesas_ct25205_data, phy_monitor_work);
	const struct device *dev = data->dev;

	if (!data->cb) {
		k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
		return;
	}

	phy_renesas_ct25205_get_link(dev, &data->state);

	data->cb(dev, &data->state, data->cb_data);

	/* Submit delayed work */
	k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
}

static int phy_renesas_ct25205_set_plca_cfg(const struct device *dev, struct phy_plca_cfg *plca_cfg)
{
	struct renesas_ct25205_data *data = dev->data;
	uint32_t reg_val;

	reg_val = (plca_cfg->node_count << 8U) | (plca_cfg->node_id);
	R_ETHER_PHY_Write(data->p_ctrl, ETH_PHY_CT25205_REG_PLCACTRL1, reg_val);

	R_ETHER_PHY_Write(data->p_ctrl, ETH_PHY_CT25205_REG_PLCATOTMR, plca_cfg->to_timer);

	reg_val = (plca_cfg->burst_count << 8U) | (plca_cfg->burst_timer);
	R_ETHER_PHY_Write(data->p_ctrl, ETH_PHY_CT25205_REG_PLCABURST, reg_val);

	R_ETHER_PHY_Read(data->p_ctrl, ETH_PHY_CT25205_REG_PLCACTRL0, &reg_val);
	reg_val = reg_val | (plca_cfg->enable << 15U);
	R_ETHER_PHY_Write(data->p_ctrl, ETH_PHY_CT25205_REG_PLCACTRL0, reg_val);

	return 0;
}

static int phy_renesas_ct25205_get_plca_cfg(const struct device *dev, struct phy_plca_cfg *plca_cfg)
{
	struct renesas_ct25205_data *data = dev->data;
	uint32_t reg_val;

	R_ETHER_PHY_Read(data->p_ctrl, ETH_PHY_CT25205_REG_PLCAIDVER, &reg_val);
	plca_cfg->version = (uint8_t)reg_val;

	R_ETHER_PHY_Read(data->p_ctrl, ETH_PHY_CT25205_REG_PLCACTRL0, &reg_val);
	plca_cfg->enable = (bool)(reg_val >> 15U);

	R_ETHER_PHY_Read(data->p_ctrl, ETH_PHY_CT25205_REG_PLCACTRL1, &reg_val);
	plca_cfg->node_id = (uint8_t)reg_val;

	R_ETHER_PHY_Read(data->p_ctrl, ETH_PHY_CT25205_REG_PLCACTRL1, &reg_val);
	plca_cfg->node_count = (uint8_t)(reg_val >> 8U);

	R_ETHER_PHY_Read(data->p_ctrl, ETH_PHY_CT25205_REG_PLCABURST, &reg_val);
	plca_cfg->burst_count = (uint8_t)(reg_val >> 8U);

	R_ETHER_PHY_Read(data->p_ctrl, ETH_PHY_CT25205_REG_PLCABURST, &reg_val);
	plca_cfg->burst_timer = (uint8_t)reg_val;

	R_ETHER_PHY_Read(data->p_ctrl, ETH_PHY_CT25205_REG_PLCATOTMR, &reg_val);
	plca_cfg->to_timer = (uint8_t)reg_val;

	return 0;
}

static int phy_renesas_ct25205_get_plca_sts(const struct device *dev, bool *plca_sts)
{
	struct renesas_ct25205_data *data = dev->data;
	uint32_t reg_val;

	R_ETHER_PHY_Read(data->p_ctrl, ETH_PHY_CT25205_REG_PLCAST, &reg_val);

	*plca_sts = reg_val;

	return 0;
}

static int phy_renesas_ct25205_init(const struct device *dev)
{
	const struct renesas_ct25205_config *config = dev->config;
	struct renesas_ct25205_data *data = dev->data;
	data->dev = dev;
	int err = 0;
	fsp_err_t fsp_err = FSP_SUCCESS;

	fsp_err = R_ETHER_PHY_Open(data->p_ctrl, config->p_cfg);

	if (FSP_SUCCESS != fsp_err) {
		LOG_ERR("PHY CT25205 Failed to Open");
		err = -EIO;
		goto end;
	}

	fsp_err = R_ETHER_PHY_ChipInit(data->p_ctrl, config->p_cfg);

	if (FSP_SUCCESS != fsp_err) {
		LOG_ERR("PHY CT25205 Failed to Initialize");
		err = -EIO;
	}

	k_work_init_delayable(&data->phy_monitor_work, phy_monitor_work_handler);
	phy_monitor_work_handler(&data->phy_monitor_work.work);
	LOG_INF("PHY CT25205 Initialized");

end:
	return err;
}

static DEVICE_API(ethphy, renesas_ct25205_phy_api) = {
	.get_link = phy_renesas_ct25205_get_link,
	.link_cb_set = phy_renesas_ct25205_link_cb_set,
	.set_plca_cfg = phy_renesas_ct25205_set_plca_cfg,
	.get_plca_cfg = phy_renesas_ct25205_get_plca_cfg,
	.get_plca_sts = phy_renesas_ct25205_get_plca_sts,
};

#define RENESAS_CT25205_PHY_CONFIG(idx)                                                            \
	ether_phy_instance_ctrl_t g_ether_phy##idx##_ctrl;                                         \
                                                                                                   \
	const ether_phy_lsi_cfg_t g_ether_phy##idx##_phy_lsi_cfg_list = {                          \
		.address = DT_INST_PROP(idx, phy_lsi_addr),                                        \
		.type = ETHER_PHY_LSI_TYPE_CT25205_RTL,                                            \
	};                                                                                         \
                                                                                                   \
	const ether_phy_extended_cfg_t g_ether_phy##idx##_extended_cfg = {                         \
		.p_target_init = NULL,                                                             \
		.p_target_link_partner_ability_get = NULL,                                         \
		.p_phy_lsi_cfg_list = &g_ether_phy##idx##_phy_lsi_cfg_list,                        \
		.eth_reg = ETHER_PHY_ETH_REG_T1S,                                                  \
	};                                                                                         \
                                                                                                   \
	const ether_phy_cfg_t g_ether_phy##idx##_cfg = {                                           \
		.channel = idx,                                                                    \
		.phy_lsi_address = DT_INST_PROP(idx, phy_lsi_addr),                                \
		.phy_reset_wait_time = 0x00020000,                                                 \
		.mii_bit_access_wait_time = 8,                                                     \
		.phy_lsi_type = ETHER_PHY_LSI_TYPE_CT25205_RTL,                                    \
		.flow_control = ETHER_PHY_FLOW_CONTROL_DISABLE,                                    \
		.p_context = NULL,                                                                 \
		.p_extend = &g_ether_phy##idx##_extended_cfg,                                      \
	};

#define RENESAS_CT25205_PHY_INIT_DRIVER(idx)                                                       \
	RENESAS_CT25205_PHY_CONFIG(idx)                                                            \
                                                                                                   \
	static const struct renesas_ct25205_config renesas_ct25205_##idx##_config = {              \
		.p_cfg = &g_ether_phy##idx##_cfg,                                                  \
	};                                                                                         \
                                                                                                   \
	static struct renesas_ct25205_data renesas_ct25205_##idx##_data = {                        \
		.p_ctrl = &g_ether_phy##idx##_ctrl,                                                \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, &phy_renesas_ct25205_init, NULL, &renesas_ct25205_##idx##_data, \
			      &renesas_ct25205_##idx##_config, POST_KERNEL,                        \
			      CONFIG_PHY_RENESAS_CT25205_INIT_PRIORITY, &renesas_ct25205_phy_api);

DT_INST_FOREACH_STATUS_OKAY(RENESAS_CT25205_PHY_INIT_DRIVER);
