/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_lvd

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/renesas_lvd/renesas_rx_lvd.h>
#include <zephyr/drivers/pinctrl.h>

LOG_MODULE_REGISTER(renesas_rx_lvd, CONFIG_SOC_LOG_LEVEL);

#define LVD0_NODE   DT_NODELABEL(lvd0)
#define LVD1_NODE   DT_NODELABEL(lvd1)
#define LVD_NO_FUNC ((void (*)(void *))NULL)

/*
 * The extern functions below are implemented in the r_lvd_rx_hw.c source file.
 * For more information, please refer to r_lvd_rx_hw.c in HAL Renesas
 */
extern void lvd_ch1_isr(void);
extern void lvd_ch2_isr(void);

struct rx_lvd_data {
	void (*callback)(void *);
	void (*user_callback)(void *user_data);
	void *user_data;
};

struct rx_lvd_config {
	lvd_channel_t channel;
	lvd_config_t lvd_config;
	uint8_t vdet_target;
	uint8_t lvd_action;
	bool lvd_support_cmpa;
};

static int renesas_rx_pin_set_cmpa(const struct device *dev)
{
	const struct rx_lvd_config *config = dev->config;
	const struct pinctrl_dev_config *pcfg;

	if (&config->channel == 0) {
		if (DT_NODE_HAS_PROP(LVD0_NODE, pinctrl_0)) {
			PINCTRL_DT_DEFINE(LVD0_NODE);
			pcfg = PINCTRL_DT_DEV_CONFIG_GET(LVD0_NODE);
		} else {
			LOG_ERR("No pinctrl-0 property found in the device tree");
			return -EINVAL;
		}
	} else {
		if (DT_NODE_HAS_PROP(LVD1_NODE, pinctrl_0)) {
			PINCTRL_DT_DEFINE(LVD1_NODE);
			pcfg = PINCTRL_DT_DEV_CONFIG_GET(LVD1_NODE);
		} else {
			LOG_ERR("No pinctrl_0 property found in the device tree");
			return -EINVAL;
		}
	}

	/* In the case of monitoring the CMPA2 pin, set the CMPA2 pin. */
	/* This only applicable to channel 1 with the LVDb driver */
	int ret = pinctrl_apply_state(pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to apply pinctrl state: %d\n", ret);
		return -EINVAL;
	}

	return 0;
}

int renesas_rx_lvd_get_status(const struct device *dev, lvd_status_position_t *status_position,
			      lvd_status_cross_t *status_cross)
{
	const struct rx_lvd_config *config = dev->config;
	int ret;

	ret = R_LVD_GetStatus(config->channel, status_position, status_cross);
	if (ret != 0) {
		return -EINVAL;
	}

	return ret;
}

int renesas_rx_lvd_clear_status(const struct device *dev)
{
	const struct rx_lvd_config *config = dev->config;

	int ret = R_LVD_ClearStatus(config->channel);
	if (ret != 0) {
		return -EINVAL;
	}

	return ret;
}

int renesas_rx_lvd_register_callback(const struct device *dev, void (*callback)(void *),
				     void *user_data)
{
	struct rx_lvd_data *data = dev->data;
	data->user_callback = callback;
	data->user_data = user_data;

	return 0;
}

#define LVD_IRQ_CONNECT()                                                                          \
	do {                                                                                       \
		IF_ENABLED(DT_NODE_HAS_STATUS_OKAY(LVD0_NODE), ( \
            IRQ_CONNECT(DT_IRQN(LVD0_NODE),               \
                        DT_IRQ(LVD0_NODE, priority),      \
                        lvd_ch1_isr,                 \
                        DEVICE_DT_GET(LVD0_NODE),         \
                        0);                               \
            irq_enable(DT_IRQN(LVD0_NODE));               \
        ))                                 \
		IF_ENABLED(DT_NODE_HAS_STATUS_OKAY(LVD1_NODE), ( \
            IRQ_CONNECT(DT_IRQN(LVD1_NODE),               \
                        DT_IRQ(LVD1_NODE, priority),      \
                        lvd_ch2_isr,                 \
                        DEVICE_DT_GET(LVD1_NODE),         \
                        0);                               \
            irq_enable(DT_IRQN(LVD1_NODE));               \
        ))                                 \
	} while (0)

static int renesas_rx_lvd_init(const struct device *dev)
{
	lvd_err_t ret;

	LVD_IRQ_CONNECT();

	const struct rx_lvd_config *config = dev->config;
	const struct rx_lvd_data *data = dev->data;

	if ((config->lvd_action == 0) || (config->lvd_action == 3)) {
		/* In reset or no action when LVD is detected, no need for the callback */
		ret = R_LVD_Open(config->channel, &config->lvd_config, LVD_NO_FUNC);
		if (ret != 0) {
			LOG_ERR("Failed to initialize LVD channel %d", config->channel);
			return -EIO;
		}
	} else {
		/* Opens with callback function */
		ret = R_LVD_Open(config->channel, &config->lvd_config, data->callback);
		if (ret != 0) {
			LOG_ERR("Failed to initialize LVD channel %d", config->channel);
			return -EIO;
		}
	}

	/* Set the CMPA2 pin if the target is CMPA2 */
	/* NOTE: For the RX130 series, CMPA2 is only used on channel 2. */
	if ((config->lvd_support_cmpa) && (config->vdet_target == 1)) {
		return renesas_rx_pin_set_cmpa2(dev);
	}

	return 0;
}

#define RENESAS_RX_LVD_INIT(index)                                                                 \
                                                                                                   \
	static const struct rx_lvd_config rx_lvd_config_##index = {                                \
		.channel = DT_INST_PROP(index, channel),                                           \
		.lvd_config =                                                                      \
			{                                                                          \
				.trigger = DT_INST_PROP(index, lvd_trigger),                       \
			},                                                                         \
		.lvd_action = DT_INST_PROP(index, lvd_action),                                     \
		.vdet_target = DT_INST_PROP(index, vdet_target),                                   \
		.lvd_support_cmpa = DT_INST_PROP(index, renesas_lvd_external_target),             \
	};                                                                                         \
                                                                                                   \
	void rx_lvd_callback_##index(void *args)                                                   \
	{                                                                                          \
		const struct device *dev = DEVICE_DT_GET(DT_INST(index, renesas_rx_lvd));          \
		struct rx_lvd_data *data = dev->data;                                              \
                                                                                                   \
		/* Call the user's callback function*/                                             \
		if (data->user_callback) {                                                         \
			data->user_callback(data->user_data);                                      \
		}                                                                                  \
	};                                                                                         \
                                                                                                   \
	static struct rx_lvd_data rx_lvd_data_##index = {                                          \
		.callback = rx_lvd_callback_##index,                                               \
	};                                                                                         \
                                                                                                   \
	static int renesas_rx_lvd_init_##index(const struct device *dev)                           \
	{                                                                                          \
		int err = renesas_rx_lvd_init(dev);                                                \
		if (err != 0) {                                                                    \
			return err;                                                                \
		}                                                                                  \
		return 0;                                                                          \
	}                                                                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, renesas_rx_lvd_init_##index, NULL, &rx_lvd_data_##index,      \
			      &rx_lvd_config_##index, PRE_KERNEL_1,                                \
			      CONFIG_RENESAS_RX_LVD_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(RENESAS_RX_LVD_INIT)
