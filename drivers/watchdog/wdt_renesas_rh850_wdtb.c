/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_wdtb

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/watchdog.h>
#include "r_wdt.h"
#include <math.h>

LOG_MODULE_REGISTER(wdt_renesas_rh850, CONFIG_WDT_LOG_LEVEL);

struct wdt_rh850_config {
	const wdt_api_t *fsp_api;
	IRQn_Type irq;
};

struct wdt_rh850_data {
	struct wdt_timeout_cfg timeout;
	wdt_instance_ctrl_t *fsp_ctrl;
	wdt_cfg_t *fsp_cfg;
	struct k_mutex inst_lock;
	uint32_t clock_rate;
	atomic_t device_state;
};

#define WDT_RH850_ATOMIC_ENABLE      (0)
#define WDT_RH850_ATOMIC_TIMEOUT_SET (1)

#define SELECT_BY_WS                         WDT_WINDOW_OPEN_FUNCTION_MODE_SELECT_BY_WS
#define SET_BY_WORST                         WDT_WINDOW_OPEN_FUNCTION_MODE_SET_BY_WOST

#define WDT_RH850_TIMEOUT_UNSUPPORT        0xFF

#define WDT_COUNTER_MAX                      0xFFFF

/* Lookup table for WDT period raw cycle */
static const float timeout_period_lut[] = {[WDT_OVERFLOW_WDTBTCKI_2_9] = 512,
					   [WDT_OVERFLOW_WDTBTCKI_2_10] = 1024,
					   [WDT_OVERFLOW_WDTBTCKI_2_11] = 2048,
					   [WDT_OVERFLOW_WDTBTCKI_2_12] = 4096,
					   [WDT_OVERFLOW_WDTBTCKI_2_13] = 8192,
					   [WDT_OVERFLOW_WDTBTCKI_2_14] = 16384,
					   [WDT_OVERFLOW_WDTBTCKI_2_15] = 32768,
					   [WDT_OVERFLOW_WDTBTCKI_2_16] = 65536,};

/* Lookup table for the period over which the window is open */
static const int window_open_period[] = {[0] = WDT_PERIOD_25,
				       [1] = WDT_PERIOD_50,
				       [2] = WDT_PERIOD_75,
				       [3] = WDT_PERIOD_100};

void wdt_isr(void);

static inline void wdt_rh850_inst_lock(const struct device *dev)
{
	struct wdt_rh850_data *data = dev->data;

	k_mutex_lock(&data->inst_lock, K_FOREVER);
}

static inline void wdt_rh850_inst_unlock(const struct device *dev)
{
	struct wdt_rh850_data *data = dev->data;

	k_mutex_unlock(&data->inst_lock);
}

static int wdt_rh850_timeout_calculate(const struct device *dev,
					const struct wdt_timeout_cfg *config)
{
	struct wdt_rh850_data *data = dev->data;
	wdt_extended_cfg_t *fsp_cfg_extend = (wdt_extended_cfg_t *)data->fsp_cfg->p_extend;

	if (data->clock_rate == 0U) {
		LOG_ERR("invalid clock rate");
		return -EINVAL;
	}

	if (atomic_test_bit(&data->device_state, WDT_RH850_ATOMIC_TIMEOUT_SET)) {
		if (config->window.min != data->timeout.window.min ||
		    config->window.max != data->timeout.window.max ||
		    config->flags != data->timeout.flags) {
			LOG_ERR("wdt support only one timeout setting value");
			return -EINVAL;
		}

		data->timeout.callback = config->callback;
		return 0;
	}

	unsigned int window_open_idx;
	unsigned int best_timeout = WDT_OVERFLOW_WDTBTCKI_2_9;
	unsigned int best_period_ms = UINT_MAX;
	unsigned int min_delta = UINT_MAX;
	unsigned int open_ms;

	for (unsigned int timeout = WDT_OVERFLOW_WDTBTCKI_2_9;
		timeout < ARRAY_SIZE(timeout_period_lut);
		timeout++) {
		if (timeout_period_lut[timeout] == WDT_RH850_TIMEOUT_UNSUPPORT) {
			continue;
		}
		unsigned int period_ms =
			(unsigned int)(1000U * timeout_period_lut[timeout] / data->clock_rate);
		unsigned int delta = period_ms > config->window.max
					    ? period_ms - config->window.max
					    : config->window.max - period_ms;

		if (delta < min_delta) {
			min_delta = delta;
			best_timeout = timeout;
			best_period_ms = period_ms;
		}
	}

	/* Calculate maximum period time out */
	unsigned int max_period_ms =
		(unsigned int)(1000U * timeout_period_lut[(ARRAY_SIZE(timeout_period_lut)-1)] /
								data->clock_rate);

	if ((min_delta == UINT_MAX) || (config->window.max > max_period_ms)) {
		LOG_ERR("wdt timeout out of range");
		return -EINVAL;
	}

	if (config->window.min > best_period_ms) {
		LOG_ERR("window_min invalid");
		return -EINVAL;
	}

	if (SELECT_BY_WS == (uint8_t) fsp_cfg_extend->window_open_mode_select) {
		open_ms = best_period_ms - config->window.min;
		if (open_ms == 0) {
			LOG_ERR("window open period invalid");
			return -EINVAL;
		} else if (open_ms <= best_period_ms / 4) {
			window_open_idx = 0; /* 25% */
		} else if (open_ms <= best_period_ms / 2) {
			window_open_idx = 1; /* 50% */
		} else if (open_ms <= (best_period_ms * 3) / 4) {
			window_open_idx = 2; /* 75% */
		} else {
			window_open_idx = 3; /* 100% */
		}

		fsp_cfg_extend->window_open =
			(wdt_window_open_t)window_open_period[window_open_idx];
		LOG_INF("actual window min = %d%%", (100 - (window_open_idx+1) * 25));
	} else {
		fsp_cfg_extend->window_open_start =
		(uint16_t) (((uint64_t)config->window.min * WDT_COUNTER_MAX) / best_period_ms);
		fsp_cfg_extend->window_open_interrupt_output_timimg = (WDT_COUNTER_MAX * 3/4);

		LOG_INF("actual window min = %d%%",
				(fsp_cfg_extend->window_open_start*100/WDT_COUNTER_MAX));
	}

	fsp_cfg_extend->overflow_time = (wdt_overflow_time_t)best_timeout;

	LOG_INF("actual window max = %d%%", 100);

	data->timeout = *config;

	return 0;
}

static int wdt_rh850_setup(const struct device *dev, uint8_t options)
{
	const struct wdt_rh850_config *cfg = dev->config;
	struct wdt_rh850_data *data = dev->data;
	int ret = 0;

	if ((options & WDT_OPT_PAUSE_IN_SLEEP) != 0) {
		LOG_ERR("wdt pause in sleep mode not supported");
		return -ENOTSUP;
	}

	wdt_rh850_inst_lock(dev);

	if (atomic_test_bit(&data->device_state, WDT_RH850_ATOMIC_ENABLE)) {
		LOG_ERR("wdt has been already setup");
		ret = -EBUSY;
		goto end;
	}

	if (!atomic_test_bit(&data->device_state, WDT_RH850_ATOMIC_TIMEOUT_SET)) {
		LOG_ERR("wdt timeout should be installed before");
		ret = -EFAULT;
		goto end;
	}

	if (cfg->fsp_api->open(data->fsp_ctrl, data->fsp_cfg) != FSP_SUCCESS) {
		LOG_ERR("wdt setup failed!");
		ret = -EIO;
		goto end;
	}

	if (cfg->fsp_api->refresh(data->fsp_ctrl) != FSP_SUCCESS) {
		LOG_ERR("wdt start failed!");
		ret = -EIO;
		goto end;
	}

	atomic_set_bit(&data->device_state, WDT_RH850_ATOMIC_ENABLE);

end:
	wdt_rh850_inst_unlock(dev);

	return ret;
}

static int wdt_rh850_disable(const struct device *dev)
{
	struct wdt_rh850_data *data = dev->data;

	if (!atomic_test_bit(&data->device_state, WDT_RH850_ATOMIC_ENABLE)) {
		LOG_ERR("wdt has not been enabled yet");
		return -EFAULT;
	}

	LOG_ERR("watchdog cannot be stopped once started unless SOC gets a reset");
	return -EPERM;
}

static int wdt_rh850_install_timeout(const struct device *dev,
						const struct wdt_timeout_cfg *config)
{
	struct wdt_rh850_data *data = dev->data;
	int ret = 0;

	if (config->window.min > config->window.max || config->window.max == 0) {
		return -EINVAL;
	}

	if (config->callback == NULL && (config->flags & WDT_FLAG_RESET_MASK) == 0) {
		LOG_ERR("no timeout response was chosen");
		return -EINVAL;
	}

	wdt_rh850_inst_lock(dev);

	if (atomic_test_bit(&data->device_state, WDT_RH850_ATOMIC_ENABLE)) {
		LOG_ERR("cannot change timeout settings after wdt setup");
		ret = -EBUSY;
		goto end;
	}

	ret = wdt_rh850_timeout_calculate(dev, config);
	if (ret < 0) {
		goto end;
	}

	atomic_set_bit(&data->device_state, WDT_RH850_ATOMIC_TIMEOUT_SET);

	LOG_INF("wdt timeout was set successfully");

end:
	wdt_rh850_inst_unlock(dev);

	return ret;
}

static int wdt_rh850_feed(const struct device *dev, int channel_id)
{
	const struct wdt_rh850_config *cfg = dev->config;
	struct wdt_rh850_data *data = dev->data;

	if (!atomic_test_bit(&data->device_state, WDT_RH850_ATOMIC_ENABLE)) {
		LOG_ERR("WDT has not been enabled yet!");
		return -EINVAL;
	}

	if (channel_id != 0) {
		LOG_ERR("Incorrect channel_id!");
		return -EINVAL;
	}

	if (cfg->fsp_api->refresh(data->fsp_ctrl) != FSP_SUCCESS) {
		LOG_ERR("Fail to refresh watchdog!");
		return -EIO;
	}

	return 0;
}

void wdt_rh850_callback(wdt_callback_args_t *p_args)
{
	const struct device *dev = p_args->p_context;
	struct wdt_rh850_data *data = dev->data;
	wdt_callback_t callback = data->timeout.callback;

	if (callback != NULL) {
		callback(dev, 0);
	}
}

static void wdt_rh850_isr_adapter(const struct device *dev)
{
	ARG_UNUSED(dev);

	wdt_isr();
}

static DEVICE_API(wdt, wdt_rh850_api) = {
	.setup = wdt_rh850_setup,
	.disable = wdt_rh850_disable,
	.install_timeout = wdt_rh850_install_timeout,
	.feed = wdt_rh850_feed,
};

#define WDT_RH850_INIT(inst)                                                        \
	static wdt_extended_cfg_t g_wdt_extend_cfg_##inst = {                           \
		.unit = DT_INST_PROP(inst, unit),                                           \
		.isr_request = WDT_INTERRUPT_ENABLED,                                       \
		.ipl = DT_IRQ(DT_DRV_INST(inst), priority),                                 \
		.wdt_irq = DT_IRQ(DT_DRV_INST(inst), irq),                                  \
		.window_open_mode_select = DT_INST_ENUM_IDX(inst, window_open_mode),        \
		.activation_code = WDT_ACTIVATION_CODE_FIX,                                 \
	};                                                                              \
	static wdt_cfg_t g_wdt_##inst##_cfg = {                                         \
		.reset_control = DT_INST_ENUM_IDX(inst, error_mode),                        \
		.p_callback = wdt_rh850_callback,                                           \
		.p_context = (void *)DEVICE_DT_INST_GET(inst),                              \
		.p_extend = &g_wdt_extend_cfg_##inst,                                       \
	};                                                                              \
                                                                                    \
	static wdt_instance_ctrl_t g_wdt##inst##_ctrl;                                  \
                                                                                    \
	static struct wdt_rh850_data wdt_rh850_data_##inst = {                          \
		.fsp_ctrl = &g_wdt##inst##_ctrl,                                            \
		.fsp_cfg = &g_wdt_##inst##_cfg,                                             \
		.clock_rate = DT_INST_PROP(inst, clock_freq),                               \
		.device_state = ATOMIC_INIT(0),                                             \
	};                                                                              \
                                                                                    \
	static struct wdt_rh850_config wdt_rh850_config_##inst = {                      \
		.fsp_api = &g_wdt_on_wdt,                                                   \
		.irq = DT_INST_IRQN(inst),                                                  \
	};                                                                              \
                                                                                    \
	static int wdt_rh850_init_##inst(const struct device *dev)                      \
	{                                                                               \
		struct wdt_rh850_data *data = dev->data;                                    \
                                                                                    \
		k_mutex_init(&data->inst_lock);                                             \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), wdt_rh850_isr_adapter,\
			    DEVICE_DT_INST_GET(inst), 0);                                       \
		irq_enable(DT_INST_IRQN(inst));                                             \
                                                                                    \
		return 0;                                                                   \
	}                                                                               \
                                                                                    \
	DEVICE_DT_INST_DEFINE(inst, &wdt_rh850_init_##inst, NULL,                       \
			&wdt_rh850_data_##inst, &wdt_rh850_config_##inst, POST_KERNEL,          \
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_rh850_api);

DT_INST_FOREACH_STATUS_OKAY(WDT_RH850_INIT)
