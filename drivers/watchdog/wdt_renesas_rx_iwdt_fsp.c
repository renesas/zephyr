/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Independent Watchdog (IWDT) Driver for Renesas RX.
 */

#define DT_DRV_COMPAT renesas_rx_iwdt_fsp

#include <limits.h>
#include <string.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>

#include "r_iwdt.h"

LOG_MODULE_REGISTER(renesas_rx_iwdt_fsp, CONFIG_WDT_LOG_LEVEL);

extern void iwdt_iwuni_isr(void);

#define WDT_RX_IWDTA_ATOMIC_ENABLE      (0)
#define WDT_RX_IWDTA_ATOMIC_TIMEOUT_SET (1)

/* IWDT can only reset the SoC or leave it running */
#define WDT_RENESAS_RX_IWDTA_SUPPORTED_FLAGS (WDT_FLAG_RESET_NONE | WDT_FLAG_RESET_SOC)

#define WDT_WINDOW_INVALID (-1)

struct wdt_rx_iwdta_config {
	const struct device *clock_dev;
#if defined(CONFIG_WDT_RENESAS_RX_IWDT_IRQ)
	void (*irq_configure)(void);
#endif /* CONFIG_WDT_RENESAS_RX_IWDT_IRQ */
};

struct wdt_rx_iwdta_data {
	struct wdt_timeout_cfg timeout;
	iwdt_instance_ctrl_t iwdt_ctrl;
	wdt_cfg_t wdt_cfg;
#if defined(CONFIG_WDT_RENESAS_RX_IWDT_IRQ)
	iwdt_extended_cfg_t iwdt_extend;
#endif /* CONFIG_WDT_RENESAS_RX_IWDT_IRQ */
	struct k_mutex inst_lock;
	atomic_t device_state;
};

/*
 * IWDTa only has 4 timeout periods defined by wdt_timeout_t (128/512/1024/2048
 * IWDTCLK cycles).
 */
static const uint32_t iwdt_timeout_period_lut[] = {
	[WDT_TIMEOUT_128] = 128U,
	[WDT_TIMEOUT_512] = 512U,
	[WDT_TIMEOUT_1024] = 1024U,
	[WDT_TIMEOUT_2048] = 2048U,
};

static const wdt_timeout_t iwdt_timeout_values[] = {
	WDT_TIMEOUT_128,
	WDT_TIMEOUT_512,
	WDT_TIMEOUT_1024,
	WDT_TIMEOUT_2048,
};

/*
 * IWDTa only has 6 clock division ratios defined by wdt_clock_division_t
 */
static const wdt_clock_division_t iwdt_clock_div_values[] = {
	WDT_CLOCK_DIVISION_1,  WDT_CLOCK_DIVISION_16,  WDT_CLOCK_DIVISION_32,
	WDT_CLOCK_DIVISION_64, WDT_CLOCK_DIVISION_128, WDT_CLOCK_DIVISION_256,
};

static const uint32_t iwdt_clock_div_ratio_lut[] = {
	1U, 16U, 32U, 64U, 128U, 256U,
};

/* Lookup table for the window start position setting. */
static const int window_start_lut[] = {
	[0] = WDT_WINDOW_START_100, [1] = WDT_WINDOW_START_75, [2] = WDT_WINDOW_START_50,
	[3] = WDT_WINDOW_START_25,  [4] = WDT_WINDOW_INVALID,
};

/* Lookup table for the window end position setting. */
static const int window_end_lut[] = {
	[0] = WDT_WINDOW_INVALID, [1] = WDT_WINDOW_END_75, [2] = WDT_WINDOW_END_50,
	[3] = WDT_WINDOW_END_25,  [4] = WDT_WINDOW_END_0,
};

static inline void wdt_rx_iwdta_lock(const struct device *dev)
{
	struct wdt_rx_iwdta_data *data = dev->data;

	k_mutex_lock(&data->inst_lock, K_FOREVER);
}

static inline void wdt_rx_iwdta_unlock(const struct device *dev)
{
	struct wdt_rx_iwdta_data *data = dev->data;

	k_mutex_unlock(&data->inst_lock);
}

static int wdt_rx_iwdta_timeout_calculate(const struct device *dev,
					  const struct wdt_timeout_cfg *config)
{
	struct wdt_rx_iwdta_data *data = dev->data;
	const struct wdt_rx_iwdta_config *cfg = dev->config;
	wdt_cfg_t *p_cfg = &data->wdt_cfg;
	unsigned int window_start_idx;
	unsigned int window_end_idx;
	unsigned int best_divisor_idx = 0;
	unsigned int best_timeout_idx = 0;
	unsigned int best_period_ms = 0;
	unsigned int min_delta = UINT_MAX;
	uint32_t clock_rate;
	int ret;

	if (atomic_test_bit(&data->device_state, WDT_RX_IWDTA_ATOMIC_TIMEOUT_SET)) {
		if (config->window.min != data->timeout.window.min ||
		    config->window.max != data->timeout.window.max ||
		    config->flags != data->timeout.flags) {
			LOG_ERR("wdt supports only one timeout setting value");
			return -EINVAL;
		}

		data->timeout.callback = config->callback;
		return 0;
	}

	ret = clock_control_get_rate(cfg->clock_dev, NULL, &clock_rate);
	if (ret != 0) {
		return ret;
	}

	for (unsigned int d = 0; d < ARRAY_SIZE(iwdt_clock_div_ratio_lut); d++) {
		for (unsigned int t = 0; t < ARRAY_SIZE(iwdt_timeout_period_lut); t++) {
			unsigned int period_ms = (1000U * iwdt_clock_div_ratio_lut[d] *
						   iwdt_timeout_period_lut[t]) / clock_rate;
			unsigned int delta = period_ms > config->window.max
						     ? period_ms - config->window.max
						     : config->window.max - period_ms;

			if (delta < min_delta) {
				min_delta = delta;
				best_divisor_idx = d;
				best_timeout_idx = t;
				best_period_ms = period_ms;
			}
		}
	}

	if (min_delta == UINT_MAX || best_period_ms == 0) {
		LOG_ERR("wdt timeout out of range");
		return -EINVAL;
	}

	window_start_idx = (config->window.min * 4 + best_period_ms - 1) / best_period_ms;
	window_end_idx = (config->window.max * 4 + best_period_ms - 1) / best_period_ms;

	if (window_start_idx >= ARRAY_SIZE(window_start_lut) ||
	    window_end_idx >= ARRAY_SIZE(window_end_lut) ||
	    window_start_lut[window_start_idx] == WDT_WINDOW_INVALID ||
	    window_end_lut[window_end_idx] == WDT_WINDOW_INVALID) {
		LOG_ERR("this wdt timeout window is not supported");
		return -ENOTSUP;
	}

	p_cfg->clock_division = iwdt_clock_div_values[best_divisor_idx];
	p_cfg->timeout = iwdt_timeout_values[best_timeout_idx];
	p_cfg->window_start = (wdt_window_start_t)window_start_lut[window_start_idx];
	p_cfg->window_end = (wdt_window_end_t)window_end_lut[window_end_idx];

	unsigned int window_min_centi_ms = window_start_idx * best_period_ms * 25U;
	unsigned int window_max_centi_ms = window_end_idx * best_period_ms * 25U;

	LOG_INF("actual window min = %u.%02u ms", window_min_centi_ms / 100U,
		window_min_centi_ms % 100U);
	LOG_INF("actual window max = %u.%02u ms", window_max_centi_ms / 100U,
		window_max_centi_ms % 100U);

	memcpy(&data->timeout, config, sizeof(struct wdt_timeout_cfg));

	return 0;
}

#if defined(CONFIG_WDT_RENESAS_RX_IWDT_IRQ)
static void wdt_rx_iwdta_callback_adapter(wdt_callback_args_t *p_args)
{
	const struct device *dev = p_args->p_context;
	struct wdt_rx_iwdta_data *data = dev->data;
	wdt_callback_t callback = data->timeout.callback;

	if (callback != NULL) {
		callback(dev, 0);
	}
}
#endif /* CONFIG_WDT_RENESAS_RX_IWDT_IRQ */

static int wdt_rx_iwdta_setup(const struct device *dev, uint8_t options)
{
	struct wdt_rx_iwdta_data *data = dev->data;
#if defined(CONFIG_WDT_RENESAS_RX_IWDT_IRQ)
	const struct wdt_rx_iwdta_config *cfg = dev->config;
#endif /* CONFIG_WDT_RENESAS_RX_IWDT_IRQ */
	int ret = 0;
	fsp_err_t err;

	if ((options & WDT_OPT_PAUSE_HALTED_BY_DBG) != 0) {
		LOG_ERR("wdt pause when halted by debugger is not supported");
		return -ENOTSUP;
	}

	wdt_rx_iwdta_lock(dev);

	if (atomic_test_bit(&data->device_state, WDT_RX_IWDTA_ATOMIC_ENABLE)) {
		LOG_ERR("wdt has already been setup");
		ret = -EBUSY;
		goto end;
	}

	if (!atomic_test_bit(&data->device_state, WDT_RX_IWDTA_ATOMIC_TIMEOUT_SET)) {
		LOG_ERR("wdt timeout should be installed before setup");
		ret = -EFAULT;
		goto end;
	}

	data->wdt_cfg.stop_control = (options & WDT_OPT_PAUSE_IN_SLEEP) != 0
					     ? WDT_STOP_CONTROL_ENABLE
					     : WDT_STOP_CONTROL_DISABLE;

#if defined(CONFIG_WDT_RENESAS_RX_IWDT_IRQ)
	if (cfg->irq_configure != NULL) {
		cfg->irq_configure();
	}
#endif /* CONFIG_WDT_RENESAS_RX_IWDT_IRQ */

	err = R_IWDT_Open(&data->iwdt_ctrl, &data->wdt_cfg);
	if (err != FSP_SUCCESS) {
		LOG_ERR("wdt setup failed");
		ret = -EIO;
		goto end;
	}

	err = R_IWDT_Refresh(&data->iwdt_ctrl);
	if (err != FSP_SUCCESS) {
		LOG_ERR("wdt start failed");
		ret = -EIO;
		goto end;
	}

	atomic_set_bit(&data->device_state, WDT_RX_IWDTA_ATOMIC_ENABLE);

end:
	wdt_rx_iwdta_unlock(dev);

	return ret;
}

static int wdt_rx_iwdta_disable(const struct device *dev)
{
	struct wdt_rx_iwdta_data *data = dev->data;

	if (!atomic_test_bit(&data->device_state, WDT_RX_IWDTA_ATOMIC_ENABLE)) {
		LOG_ERR("wdt has not been enabled yet");
		return -EFAULT;
	}

	LOG_ERR("Independent Watchdog cannot be stopped once started");
	return -EPERM;
}

static int wdt_rx_iwdta_install_timeout(const struct device *dev,
					const struct wdt_timeout_cfg *config)
{
	struct wdt_rx_iwdta_data *data = dev->data;
	wdt_cfg_t *p_config = &data->wdt_cfg;
	int ret = 0;

	if (config->window.min > config->window.max || config->window.max == 0) {
		return -EINVAL;
	}

	if ((config->flags & ~WDT_RENESAS_RX_IWDTA_SUPPORTED_FLAGS) != 0) {
		return -ENOTSUP;
	}

	if (config->callback == NULL && (config->flags & WDT_FLAG_RESET_MASK) == 0) {
		LOG_ERR("no timeout response was chosen");
		return -EINVAL;
	}

	if (config->callback != NULL && (config->flags & WDT_FLAG_RESET_MASK) != 0) {
		LOG_ERR("WDT_FLAG_RESET_NONE should be chosen in case of interrupt response");
		return -ENOTSUP;
	}

	wdt_rx_iwdta_lock(dev);

	if (atomic_test_bit(&data->device_state, WDT_RX_IWDTA_ATOMIC_ENABLE)) {
		LOG_ERR("cannot change timeout settings after wdt setup");
		ret = -EBUSY;
		goto end;
	}

#if !defined(CONFIG_WDT_RENESAS_RX_IWDT_IRQ)
	if (config->callback != NULL) {
		LOG_ERR("interrupt response is not supported by this IWDT configuration");
		ret = -ENOTSUP;
		goto end;
	}

	p_config->reset_control = WDT_RESET_CONTROL_RESET;
#else
	p_config->reset_control = (config->flags & WDT_FLAG_RESET_MASK) != 0
					  ? WDT_RESET_CONTROL_RESET
					  : WDT_RESET_CONTROL_NMI;
#endif /* !CONFIG_WDT_RENESAS_RX_IWDT_IRQ */

	ret = wdt_rx_iwdta_timeout_calculate(dev, config);
	if (ret < 0) {
		goto end;
	}

	atomic_set_bit(&data->device_state, WDT_RX_IWDTA_ATOMIC_TIMEOUT_SET);

end:
	wdt_rx_iwdta_unlock(dev);

	return ret;
}

static int wdt_rx_iwdta_feed(const struct device *dev, int channel_id)
{
	struct wdt_rx_iwdta_data *data = dev->data;
	fsp_err_t err;

	if (!atomic_test_bit(&data->device_state, WDT_RX_IWDTA_ATOMIC_ENABLE) || channel_id != 0) {
		return -EINVAL;
	}

	err = R_IWDT_Refresh(&data->iwdt_ctrl);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	return 0;
}

static DEVICE_API(wdt, wdt_rx_iwdt_fsp_api) = {
	.setup = wdt_rx_iwdta_setup,
	.disable = wdt_rx_iwdta_disable,
	.install_timeout = wdt_rx_iwdta_install_timeout,
	.feed = wdt_rx_iwdta_feed,
};

#if defined(CONFIG_WDT_RENESAS_RX_IWDT_IRQ)
#define WDT_RENESAS_RX_IWDTA_IRQ_CONFIGURE_DEFINE(id)                                              \
	static void wdt_rx_iwdta_isr##id(const void *arg)                                          \
	{                                                                                          \
		ARG_UNUSED(arg);                                                                   \
		BSP_ICU_IRQ_HANDLER(DT_INST_IRQ_BY_NAME(id, iwuni, irq), iwdt_iwuni_isr);          \
	}                                                                                          \
	static void wdt_rx_iwdta_irq_configure##id(void)                                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(id, iwuni, irq),                                   \
			    DT_INST_IRQ_BY_NAME(id, iwuni, priority), wdt_rx_iwdta_isr##id, NULL,  \
			    0);                                                                    \
		irq_enable(DT_INST_IRQ_BY_NAME(id, iwuni, irq));                                   \
	}
#define WDT_RENESAS_RX_IWDTA_IRQ_CONFIGURE_INIT(id) .irq_configure = wdt_rx_iwdta_irq_configure##id,
#define WDT_RENESAS_RX_IWDTA_EXTEND_INIT(id)                                                       \
	.iwdt_extend = {.irq = (IRQn_Type)DT_INST_IRQ_BY_NAME(id, iwuni, irq),                     \
			.ipl = DT_INST_IRQ_BY_NAME(id, iwuni, priority)},
#define WDT_RENESAS_RX_IWDTA_CALLBACK     wdt_rx_iwdta_callback_adapter
#define WDT_RENESAS_RX_IWDTA_P_EXTEND(id) (void const *)&wdt_rx_iwdta_data##id.iwdt_extend
#else
#define WDT_RENESAS_RX_IWDTA_IRQ_CONFIGURE_DEFINE(id)
#define WDT_RENESAS_RX_IWDTA_IRQ_CONFIGURE_INIT(id)
#define WDT_RENESAS_RX_IWDTA_EXTEND_INIT(id)
#define WDT_RENESAS_RX_IWDTA_CALLBACK     NULL
#define WDT_RENESAS_RX_IWDTA_P_EXTEND(id) NULL
#endif /* CONFIG_WDT_RENESAS_RX_IWDT_IRQ */

#define WDT_RENESAS_RX_IWDT_FSP_DEFINE(id)                                                         \
	WDT_RENESAS_RX_IWDTA_IRQ_CONFIGURE_DEFINE(id)                                              \
                                                                                                   \
	static const struct wdt_rx_iwdta_config wdt_rx_iwdta_cfg##id = {                           \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(id)),                               \
		WDT_RENESAS_RX_IWDTA_IRQ_CONFIGURE_INIT(id)};                                      \
                                                                                                   \
	static struct wdt_rx_iwdta_data wdt_rx_iwdta_data##id = {                                  \
		.device_state = ATOMIC_INIT(0),                                                    \
		.wdt_cfg = {.stop_control = WDT_STOP_CONTROL_DISABLE,                              \
			    .reset_control = WDT_RESET_CONTROL_RESET,                              \
			    .p_callback = WDT_RENESAS_RX_IWDTA_CALLBACK,                           \
			    .p_context = (void *)DEVICE_DT_GET(DT_DRV_INST(id)),                   \
			    .p_extend = WDT_RENESAS_RX_IWDTA_P_EXTEND(id)},                        \
		WDT_RENESAS_RX_IWDTA_EXTEND_INIT(id).inst_lock =                                   \
			Z_MUTEX_INITIALIZER(wdt_rx_iwdta_data##id.inst_lock),                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(id, NULL, NULL, &wdt_rx_iwdta_data##id, &wdt_rx_iwdta_cfg##id,       \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                     \
			      &wdt_rx_iwdt_fsp_api);

DT_INST_FOREACH_STATUS_OKAY(WDT_RENESAS_RX_IWDT_FSP_DEFINE)
