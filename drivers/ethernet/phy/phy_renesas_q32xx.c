/*
 * Copyright 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_q32xx_phy

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>
#include <zephyr/sys/util.h>
#include "rp_phy_q32xx.h"

#define LOG_MODULE_NAME phy_renesas_q32xx
#define LOG_LEVEL       CONFIG_PHY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

struct renesas_q32xx_config {
	const struct device *mdio_dev;
	const uint8_t phy_addr;
	const struct rp_phy_q32xx_init_cfg init_cfg;
};

struct renesas_q32xx_data {
	const struct device *dev;
	struct phy_link_state state;
	phy_callback_t cb;
	void *cb_data;
	struct k_work_delayable phy_monitor_work;
};

/**
 * @brief Convert PHY line rate to Zephyr link speed
 */
static int phy_renesas_q32xx_line_rate_to_zephyr_speed(enum rp_phy_q32xx_line_rate line_rate,
						       enum phy_link_speed *speed)
{
	switch (line_rate) {
	case RP_PHY_Q32XX_LINE_RATE_2P5G:
		*speed = LINK_FULL_2500BASE;
		return 0;
	case RP_PHY_Q32XX_LINE_RATE_5G:
		*speed = LINK_FULL_5000BASE;
		return 0;
	case RP_PHY_Q32XX_LINE_RATE_NONE:
		*speed = 0;
		return 0;
	default:
		*speed = 0;
		return -EINVAL;
	}
}

/**
 * @brief Convert Zephyr link speed to PHY line rate
 */
static int phy_renesas_q32xx_zephyr_speed_to_line_rate(enum phy_link_speed speed,
						       enum rp_phy_q32xx_line_rate *line_rate)
{
	switch (speed) {
	case LINK_FULL_2500BASE:
		*line_rate = RP_PHY_Q32XX_LINE_RATE_2P5G;
		return 0;
	case LINK_FULL_5000BASE:
		*line_rate = RP_PHY_Q32XX_LINE_RATE_5G;
		return 0;
	default:
		*line_rate = RP_PHY_Q32XX_LINE_RATE_NONE;
		return -ENOTSUP;
	}
}

/**
 * @brief Get link state
 */
static int phy_renesas_q32xx_get_link(const struct device *dev, struct phy_link_state *state)
{
	const struct renesas_q32xx_config *config = dev->config;
	struct rp_phy_q32xx_link_state q32xx_state;
	int err;

	state->is_up = false;
	state->speed = 0;

	err = rp_phy_q32xx_get_link_state(config->mdio_dev, config->phy_addr, &q32xx_state);

	if (err < 0) {
		LOG_ERR("Failed to get link status: %d", err);
		return err;
	}

	state->is_up = q32xx_state.is_up;
	if (q32xx_state.is_up) {
		err = phy_renesas_q32xx_line_rate_to_zephyr_speed(q32xx_state.line_rate,
								  &state->speed);
		if (err < 0) {
			return err;
		}
	}

	return 0;
}

/**
 * @brief Configure PHY link speed. This will also restart the line link to apply the new speed
 * setting.
 */
static int phy_renesas_q32xx_cfg_link(const struct device *dev, enum phy_link_speed speeds,
				      enum phy_cfg_link_flag flags)
{
	const struct renesas_q32xx_config *config = dev->config;
	enum rp_phy_q32xx_line_rate line_rate;

	if ((flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) == 0U) {
		LOG_ERR("Auto-negotiation is not supported, PHY_FLAG_AUTO_NEGOTIATION_DISABLED "
			"must be set");
		return -ENOTSUP;
	}

	if (phy_renesas_q32xx_zephyr_speed_to_line_rate(speeds, &line_rate) < 0) {
		LOG_ERR("Unsupported fixed link speed: 0x%x", speeds);
		return -ENOTSUP;
	}

	return rp_phy_q32xx_set_line_rate(config->mdio_dev, config->phy_addr, line_rate);
}

/**
 * @brief Set callback to be invoked when link state changes. Driver has to invoke callback
 * once after setting it, even if link state has not changed.
 */
static int phy_renesas_q32xx_link_cb_set(const struct device *dev, phy_callback_t cb,
					 void *user_data)
{
	struct renesas_q32xx_data *data = dev->data;
	int err;

	data->cb = cb;
	data->cb_data = user_data;

	/* Immediately get current status and invoke the callback to notify the caller */
	if (data->cb) {
		err = phy_renesas_q32xx_get_link(dev, &data->state);
		if ((err < 0) && (err != -ENOTSUP)) {
			LOG_WRN("Initial callback link read failed: %d", err);
		}

		data->cb(dev, &data->state, data->cb_data);
	}

	return 0;
}

static void phy_monitor_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct renesas_q32xx_data *const data =
		CONTAINER_OF(dwork, struct renesas_q32xx_data, phy_monitor_work);
	const struct device *dev = data->dev;
	struct phy_link_state state = {};
	int err;

	/* If there is no callback set, continue monitoring */
	if (!data->cb) {
		k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
		return;
	}

	err = phy_renesas_q32xx_get_link(dev, &state);

	/* If the link state has changed, update link state and invoke callback */
	if (err == 0 && !util_eq(&state, sizeof(state), &data->state, sizeof(data->state))) {
		memcpy(&data->state, &state, sizeof(struct phy_link_state));
		data->cb(dev, &data->state, data->cb_data);
	}

	/* Continue monitoring */
	k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
}

/**
 * @brief Initializes the phy and starts the link monitor
 */
static int phy_renesas_q32xx_init(const struct device *dev)
{
	const struct renesas_q32xx_config *config = dev->config;
	struct renesas_q32xx_data *data = dev->data;
	int err;

	if (!device_is_ready(config->mdio_dev)) {
		LOG_ERR("MDIO bus device is not ready");
		return -ENODEV;
	}

	data->state.is_up = false;
	data->state.speed = 0;

	err = rp_phy_q32xx_init(config->mdio_dev, config->phy_addr, &config->init_cfg);
	if (err < 0) {
		LOG_ERR("Failed to initialize PHY Q32XX: %d", err);
		return err;
	}

	k_work_init_delayable(&data->phy_monitor_work, phy_monitor_work_handler);
	phy_monitor_work_handler(&data->phy_monitor_work.work);

	LOG_INF("PHY Q32XX initialized");

	return 0;
}

static DEVICE_API(ethphy, renesas_q32xx_phy_api) = {
	.cfg_link = phy_renesas_q32xx_cfg_link,
	.get_link = phy_renesas_q32xx_get_link,
	.link_cb_set = phy_renesas_q32xx_link_cb_set,
};

/**
 * ************************* DRIVER REGISTER SECTION ***************************
 */

#define RENESAS_Q32XX_CFG_T1_RATE_FLAG(idx)                                                        \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, renesas_line_rate), (RP_PHY_Q32XX_CFG_T1_RATE), (0))

#define RENESAS_Q32XX_CFG_MASTER_SLAVE_FLAG(idx)                                                   \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, renesas_master_slave),                             \
		    (RP_PHY_Q32XX_CFG_MASTER_SLAVE), (0))

#define RENESAS_Q32XX_CFG_SERDES_SPEED_FLAG(idx)                                                   \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, renesas_serdes_speed),                              \
		    (RP_PHY_Q32XX_CFG_SERDES_SPEED), (0))

#define RENESAS_Q32XX_INIT_CFG_FLAGS(idx)                                                          \
	(RENESAS_Q32XX_CFG_T1_RATE_FLAG(idx) | RENESAS_Q32XX_CFG_MASTER_SLAVE_FLAG(idx) |          \
	 RENESAS_Q32XX_CFG_SERDES_SPEED_FLAG(idx))

#define RENESAS_Q32XX_T1_RATE(idx)                                                                 \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, renesas_line_rate),                                 \
		    (DT_INST_ENUM_IDX(idx, renesas_line_rate) + 1),                                \
		    (RP_PHY_Q32XX_LINE_RATE_NONE))

#define RENESAS_Q32XX_SERDES_SPEED(idx)                                                            \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(idx, renesas_serdes_speed),                              \
		    (DT_INST_ENUM_IDX(idx, renesas_serdes_speed) + 1),                             \
		    (RP_PHY_Q32XX_SERDES_SPEED_DISABLED))

#define RENESAS_Q32XX_OPERATION_MODE(idx)                                                          \
	DT_INST_ENUM_IDX_OR(idx, renesas_master_slave, RP_PHY_Q32XX_OP_SLAVE)

#define RENESAS_Q32XX_PHY_INIT_DRIVER(idx)                                                         \
                                                                                                   \
	static const struct renesas_q32xx_config renesas_q32xx_##idx##_config = {                  \
		.mdio_dev = DEVICE_DT_GET(DT_INST_BUS(idx)),                                       \
		.phy_addr = DT_INST_REG_ADDR(idx),                                                 \
		.init_cfg =                                                                        \
			{                                                                          \
				.flags = RENESAS_Q32XX_INIT_CFG_FLAGS(idx),                        \
				.t1_rate = RENESAS_Q32XX_T1_RATE(idx),                             \
				.serdes_speed = RENESAS_Q32XX_SERDES_SPEED(idx),                   \
				.operation_mode = RENESAS_Q32XX_OPERATION_MODE(idx),               \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	static struct renesas_q32xx_data renesas_q32xx_##idx##_data = {                            \
		.dev = DEVICE_DT_INST_GET(idx),                                                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, &phy_renesas_q32xx_init, NULL, &renesas_q32xx_##idx##_data,     \
			      &renesas_q32xx_##idx##_config, POST_KERNEL,                          \
			      CONFIG_PHY_INIT_PRIORITY, &renesas_q32xx_phy_api);

DT_INST_FOREACH_STATUS_OKAY(RENESAS_Q32XX_PHY_INIT_DRIVER);
