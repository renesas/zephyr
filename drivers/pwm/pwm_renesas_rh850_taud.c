/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_taud_pwm

#include <r_taud.h>
#include "r_taud_cfg.h"
#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pwm/pwm_utils.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util_macro.h>

LOG_MODULE_REGISTER(rh850_pwm, CONFIG_PWM_LOG_LEVEL);

extern void taud_int_isr(void);
extern const timer_api_t g_timer_on_taud;

/* ================================================================== */
/* HELPERS & CONSTANTS                                                */
/* ================================================================== */

#define TAUD_MAX_NUM_CHANNELS 16

/* ================================================================== */
/* UNIFIED CHANNEL STRUCTURES                                         */
/* ================================================================== */

#ifdef CONFIG_PWM_CAPTURE
/* Child channel in "independent" mode, used as a capture input */
struct pwm_rh850_capture_data {
	const struct device *dev;
	taud_instance_ctrl_t ctrl;
	timer_cfg_t cfg;
	taud_extended_cfg_t ext_cfg;

	pwm_capture_callback_handler_t callback;
	void *user_data;
	bool is_open;
	bool is_busy;
	bool is_pulse_capture;
	bool continuous;
	bool skip_next;
};
#endif /* CONFIG_PWM_CAPTURE */

/* Channel configuration */
struct pwm_rh850_channel_config {
	const timer_api_t *fsp_api;
	uint32_t unit;
	uint32_t channel;

	/* Clock configuration */
	uint32_t clock_source;
	uint32_t clock_divider;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;

	/* Pin control configuration */
	const struct pinctrl_dev_config *pinctrl_dev;

	/* FSP synchronized channel list */
	const timer_instance_t **synchronized_instances;

	/* Noise filter: DNFACTL value shared by the unit, DNFAEN enable bit per capture channel */
	uint8_t noise_filter_prescaler;
	uint16_t noise_filter_mask;
	uintptr_t dnfa_addr;

#ifdef CONFIG_PWM_CAPTURE
	/* Indexed by TAUD channel, NULL for channels without capture */
	struct pwm_rh850_capture_data *const *captures;
#endif /* CONFIG_PWM_CAPTURE */
};

/* Channel runtime data */
struct pwm_rh850_channel_data {
	timer_ctrl_t *ctrl;
	timer_cfg_t *cfg;
	taud_extended_cfg_t *ext_cfg;

	uint32_t period_counts;
	uint8_t num_synchronized;
	bool primary_running;

#ifdef CONFIG_PWM_EVENT
	sys_slist_t event_callbacks;
#endif /* CONFIG_PWM_EVENT */
};

/* ================================================================== */
/* CHANNEL MANAGEMENT FUNCTIONS                                       */
/* ================================================================== */

/* Channel interrupt handler (shared for all modes). */
static void pwm_rh850_channel_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	taud_int_isr();
}

/* Enable the input noise filter of the channels selected in devicetree */
/* Configure digital noise filter (DNFA) for PWM capture input pins. */
static int pwm_rh850_configure_noise_filter(const struct device *dev)
{
	const struct pwm_rh850_channel_config *cfg = dev->config;

	/* No channels have noise filter enabled; nothing to configure */
	if (cfg->noise_filter_mask == 0U) {
		return 0;
	}

	/* Access DNFA registers: control and enable for this TAUD unit */
	/* Write prescaler to DNFACTLn (shared by all channels in unit) */
	sys_write8(cfg->noise_filter_prescaler, cfg->dnfa_addr);

	/* Set enable bits in DNFAENn for each capture channel with filter enabled */
	sys_write16(sys_read16(cfg->dnfa_addr + 4U) | cfg->noise_filter_mask, cfg->dnfa_addr + 4U);

	LOG_DBG("%s: DNFA config: prescaler=%u, enable_mask=0x%04x.", dev->name,
		cfg->noise_filter_prescaler, cfg->noise_filter_mask);

	return 0;
}

/* Initialize a primary PWM channel and register synchronized children */
static int pwm_rh850_channel_init(const struct device *dev)
{
	const struct pwm_rh850_channel_config *cfg = dev->config;
	struct pwm_rh850_channel_data *data = dev->data;
	fsp_err_t err;
	int ret;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("%s: clock device is not ready.", dev->name);
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret < 0) {
		LOG_ERR("%s: clock enable failed: %d.", dev->name, ret);
		return ret;
	}

	/* Validate channel number range */
	if (cfg->channel >= TAUD_MAX_NUM_CHANNELS) {
		LOG_ERR("%s: invalid channel %u (max=%u).", dev->name, cfg->channel,
			TAUD_MAX_NUM_CHANNELS - 1);
		return -EINVAL;
	}

	/* Apply clock settings from devicetree to all extended configs (primary, synchronized,
	 * capture). Must be done BEFORE opening the primary, since R_TAUD_Open() opens all
	 * synchronized and each writes the shared TAUDnTPS prescaler from its own extended config.
	 */
	data->ext_cfg->operation_clk = (taud_clock_t)cfg->clock_source;
	data->ext_cfg->clk_div = (taud_div_t)cfg->clock_divider;

	for (uint8_t i = 0U; i < TAUD_MAX_NUM_SLAVE_CHANNELS; i++) {
		if (cfg->synchronized_instances[i] != NULL) {
			taud_extended_cfg_t *sync_ext_cfg =
				(taud_extended_cfg_t *)cfg->synchronized_instances[i]
					->p_cfg->p_extend;
			sync_ext_cfg->operation_clk = (taud_clock_t)cfg->clock_source;
			sync_ext_cfg->clk_div = (taud_div_t)cfg->clock_divider;
		}
	}

#ifdef CONFIG_PWM_CAPTURE
	if (cfg->captures != NULL) {
		for (uint8_t i = 0U; i < TAUD_MAX_NUM_CHANNELS; i++) {
			if (cfg->captures[i] != NULL) {
				cfg->captures[i]->ext_cfg.operation_clk =
					(taud_clock_t)cfg->clock_source;
				cfg->captures[i]->ext_cfg.clk_div = (taud_div_t)cfg->clock_divider;
			}
		}
	}
#endif /* CONFIG_PWM_CAPTURE */

	/* Apply pin control configuration (ignored if pinctrl-0 not defined) */
	ret = pinctrl_apply_state(cfg->pinctrl_dev, PINCTRL_STATE_DEFAULT);
	if (ret < 0 && ret != -ENOENT) {
		LOG_ERR("%s: pinctrl failed: %d.", dev->name, ret);
		return ret;
	}

	/* Configure noise filter BEFORE opening FSP channel */
	ret = pwm_rh850_configure_noise_filter(dev);
	if (ret < 0) {
		LOG_ERR("%s: noise filter configuration failed: %d", dev->name, ret);
		return ret;
	}

	/* Open the primary channel */
	err = cfg->fsp_api->open(data->ctrl, data->cfg);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: channel open failed: %d.", dev->name, err);
		return -EIO;
	}

	/* R_TAUD_Open() on the primary already opened every synchronized channel */
	for (uint8_t i = 0U; i < TAUD_MAX_NUM_SLAVE_CHANNELS; i++) {
		if (cfg->synchronized_instances[i] != NULL) {
			data->num_synchronized++;
		}
	}

	LOG_DBG("%s: init OK (unit=%u, ch=%u, clock=%u, div=%u, synchronized=%u, mask=0x%04x).",
		dev->name, cfg->unit, cfg->channel, cfg->clock_source, cfg->clock_divider,
		data->num_synchronized, ((taud_instance_ctrl_t *)data->ctrl)->slave_channels_mask);

	return 0;
}

/* Configure PWM period (primary) and duty cycle (synchronized) */
static int pwm_rh850_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
				uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_rh850_channel_config *cfg = dev->config;
	struct pwm_rh850_channel_data *data = dev->data;
	taud_instance_ctrl_t *const ctrl = data->ctrl;
	fsp_err_t err;

	if (channel >= TAUD_MAX_NUM_CHANNELS) {
		LOG_ERR("Invalid channel: %u (max=%u).", channel, TAUD_MAX_NUM_CHANNELS - 1);
		return -EINVAL;
	}

	/* Validate period before any hardware changes */
	if (period_cycles == 0U) {
		LOG_ERR("%s: period_cycles is zero.", dev->name);
		return -EINVAL;
	}

	/* TAUD counters are 16-bit: max value is 65535. At 65536 a 100% pulse wraps to 0%. */
	if (period_cycles > 65535U) {
		LOG_ERR("%s: period_cycles %u exceeds 16-bit maximum (65535).", dev->name,
			period_cycles);
		return -EINVAL;
	}

	/* Find and validate the synchronized channel before any hardware changes */
	const timer_instance_t *sync_inst = NULL;

	for (uint8_t i = 0U; i < TAUD_MAX_NUM_SLAVE_CHANNELS; i++) {
		if (cfg->synchronized_instances[i] != NULL) {
			timer_cfg_t *sync_cfg = cfg->synchronized_instances[i]->p_cfg;

			if (sync_cfg->channel == channel) {
				sync_inst = cfg->synchronized_instances[i];
				break;
			}
		}
	}

	if (sync_inst == NULL) {
		LOG_ERR("Synchronized channel %u not found (registered=%u).", channel,
			data->num_synchronized);
		return -EINVAL;
	}

	/* Clamp pulse width to period */
	if (pulse_cycles > period_cycles) {
		pulse_cycles = period_cycles;
	}

	/* Update period in primary channel if changed (shared by all synchronized) */
	if (period_cycles != data->period_counts) {
		err = cfg->fsp_api->periodSet(data->ctrl, period_cycles);
		if (err != FSP_SUCCESS) {
			if (err == FSP_ERR_INVALID_HW_CONDITION) {
				return -EBUSY;
			}
			LOG_ERR("%s: PeriodSet failed: %d.", dev->name, err);
			return -EIO;
		}
	}

	/* Set duty cycle using synchronized channel's control structure with its channel number */
	taud_instance_ctrl_t *sync_ctrl = (taud_instance_ctrl_t *)sync_inst->p_ctrl;

	err = cfg->fsp_api->dutyCycleSet(sync_ctrl, pulse_cycles, channel);
	if (err != FSP_SUCCESS) {
		if (err == FSP_ERR_INVALID_HW_CONDITION) {
			return -EBUSY;
		}
		LOG_ERR("%s: DutyCycleSet failed for channel %u: %d.", dev->name, channel, err);
		return -EIO;
	}

	/* Set output polarity for the synchronized channel (after FSP operations succeed) */
	if ((flags & PWM_POLARITY_INVERTED) != 0U) {
		ctrl->p_reg->TAUDnTOL |= (uint16_t)BIT(channel);
	} else {
		ctrl->p_reg->TAUDnTOL &= (uint16_t)~BIT(channel);
	}

	/* Update period cache only after both FSP operations succeed */
	if (period_cycles != data->period_counts) {
		data->period_counts = period_cycles;
	}

	/* Start the primary channel only on first call (after configuring period/duty) */
	if (!data->primary_running) {
		err = cfg->fsp_api->start(data->ctrl);
		if (err != FSP_SUCCESS) {
			LOG_ERR("%s: start failed: %d.", dev->name, err);
			return -EIO;
		}
		data->primary_running = true;
	}

	return 0;
}

/* Get PWM clock frequency */
static int pwm_rh850_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					uint64_t *cycles)
{
	const struct pwm_rh850_channel_config *cfg = dev->config;
	struct pwm_rh850_channel_data *data = dev->data;
	timer_info_t info;
	fsp_err_t err;

	ARG_UNUSED(channel);

	err = cfg->fsp_api->infoGet(data->ctrl, &info);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	*cycles = (uint64_t)info.clock_frequency;

	return 0;
}

#ifdef CONFIG_PWM_CAPTURE
static struct pwm_rh850_capture_data *pwm_rh850_get_capture(const struct device *dev,
							    uint32_t channel)
{
	const struct pwm_rh850_channel_config *cfg = dev->config;

	if (channel >= TAUD_MAX_NUM_CHANNELS) {
		return NULL;
	}

	return cfg->captures[channel];
}

static int pwm_rh850_configure_capture(const struct device *dev, uint32_t channel,
				       pwm_flags_t flags, pwm_capture_callback_handler_t cb,
				       void *user_data)
{
	const struct pwm_rh850_channel_config *cfg = dev->config;
	struct pwm_rh850_capture_data *capture = pwm_rh850_get_capture(dev, channel);
	taud_extended_cfg_t *ext_cfg;
	bool inverted = (flags & PWM_POLARITY_INVERTED) != 0U;
	fsp_err_t err;

	if (capture == NULL) {
		LOG_ERR("%s: channel %u is not an independent (capture) channel.", dev->name,
			channel);
		return -EINVAL;
	}

	if ((flags & PWM_CAPTURE_TYPE_MASK) == 0U) {
		LOG_ERR("%s: no PWM capture type specified.", dev->name);
		return -EINVAL;
	}

	if ((flags & PWM_CAPTURE_TYPE_MASK) == PWM_CAPTURE_TYPE_BOTH) {
		LOG_ERR("%s: cannot capture both period and pulse width.", dev->name);
		return -ENOTSUP;
	}

	if (capture->is_busy) {
		LOG_ERR("%s: capture already active on channel %u.", dev->name, channel);
		return -EBUSY;
	}

	ext_cfg = &capture->ext_cfg;

	if ((flags & PWM_CAPTURE_TYPE_PERIOD) != 0U) {
		capture->is_pulse_capture = false;
		ext_cfg->taud_function = TAUD_FUNCTION_INPUT_PULSE_INTERVAL_MEASUREMENT;
		ext_cfg->operating_mode = TAUD_MODE_CAPTURE_MODE;
		ext_cfg->trigger_type = TAUD_TRIGGER_VALID_EDGE;
		ext_cfg->edge_type = inverted ? TAUD_INPUT_EDGE_FALLING : TAUD_INPUT_EDGE_RISING;
	} else {
		capture->is_pulse_capture = true;
		ext_cfg->taud_function = TAUD_FUNCTION_INPUT_SIGNAL_WIDTH_MEASUREMENT;
		ext_cfg->operating_mode = TAUD_MODE_CAPTURE_ONE_COUNT_MODE;
		ext_cfg->trigger_type = TAUD_TRIGGER_START_STOP_VALID_EDGE;
		ext_cfg->edge_type = inverted ? TAUD_INPUT_EDGE_BOTH_MEASURE_LOW
					      : TAUD_INPUT_EDGE_BOTH_MEASURE_HIGH;
	}

	/* The function and edge are applied by open(); InputEdgeSet() cannot change them */
	if (capture->is_open) {
		err = cfg->fsp_api->close(&capture->ctrl);
		if (err != FSP_SUCCESS) {
			LOG_ERR("%s: capture close failed: %d.", dev->name, err);
			return -EIO;
		}
		capture->is_open = false;
	}

	err = cfg->fsp_api->open(&capture->ctrl, &capture->cfg);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: capture open failed on channel %u: %d.", dev->name, channel, err);
		return -EIO;
	}
	capture->is_open = true;

	capture->callback = cb;
	capture->user_data = user_data;
	capture->continuous = (flags & PWM_CAPTURE_MODE_CONTINUOUS) != 0U;

	irq_enable(capture->cfg.cycle_end_irq);

	return 0;
}

static int pwm_rh850_enable_capture(const struct device *dev, uint32_t channel)
{
	const struct pwm_rh850_channel_config *cfg = dev->config;
	struct pwm_rh850_capture_data *capture = pwm_rh850_get_capture(dev, channel);
	fsp_err_t err;

	if (capture == NULL) {
		LOG_ERR("%s: channel %u is not an independent (capture) channel.", dev->name,
			channel);
		return -EINVAL;
	}

	if (!capture->is_open || (capture->callback == NULL)) {
		LOG_ERR("%s: capture not configured on channel %u.", dev->name, channel);
		return -EINVAL;
	}

	if (capture->is_busy) {
		LOG_ERR("%s: capture already active on channel %u.", dev->name, channel);
		return -EBUSY;
	}

	/* The first period capture spans from the software start to the first edge */
	capture->skip_next = !capture->is_pulse_capture;
	capture->is_busy = true;

	err = cfg->fsp_api->start(&capture->ctrl);
	if (err != FSP_SUCCESS) {
		capture->is_busy = false;
		LOG_ERR("%s: capture start failed on channel %u: %d.", dev->name, channel, err);
		return -EIO;
	}

	return 0;
}

static int pwm_rh850_disable_capture(const struct device *dev, uint32_t channel)
{
	const struct pwm_rh850_channel_config *cfg = dev->config;
	struct pwm_rh850_capture_data *capture = pwm_rh850_get_capture(dev, channel);
	fsp_err_t err;

	if (capture == NULL) {
		LOG_ERR("%s: channel %u is not an independent (capture) channel.", dev->name,
			channel);
		return -EINVAL;
	}

	if (!capture->is_open) {
		LOG_ERR("%s: capture not configured on channel %u.", dev->name, channel);
		return -EINVAL;
	}

	err = cfg->fsp_api->stop(&capture->ctrl);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: capture stop failed on channel %u: %d.", dev->name, channel, err);
		return -EIO;
	}

	capture->is_busy = false;

	return 0;
}

static void pwm_rh850_capture_isr_cb(timer_callback_args_t *p_args)
{
	struct pwm_rh850_capture_data *capture = (struct pwm_rh850_capture_data *)p_args->p_context;
	uint32_t channel = capture->cfg.channel;
	uint32_t cycles = (uint32_t)p_args->capture;
	int callback_status = 0;

	if ((p_args->event != TIMER_EVENT_INPUT_CAPTURE) || !capture->is_busy) {
		return;
	}

	if (capture->skip_next) {
		capture->skip_next = false;
		return;
	}

	/* Stop before the callback so that it may re-arm the capture */
	if (!capture->continuous) {
		int ret = pwm_rh850_disable_capture(capture->dev, channel);

		if (ret != 0) {
			LOG_ERR("%s: stop failed during capture completion on ch%u: %d.",
				capture->dev->name, channel, ret);
			callback_status = ret;
			capture->is_busy = false;
		}
	}

	if (capture->is_pulse_capture) {
		capture->callback(capture->dev, channel, 0U, cycles, callback_status,
				  capture->user_data);
	} else {
		capture->callback(capture->dev, channel, cycles, 0U, callback_status,
				  capture->user_data);
	}
}
#endif /* CONFIG_PWM_CAPTURE */

#ifdef CONFIG_PWM_EVENT
/* Primary cycle end is the period end of every synchronized channel */
static void pwm_rh850_primary_isr_cb(timer_callback_args_t *p_args)
{
	const struct device *dev = p_args->p_context;
	const struct pwm_rh850_channel_config *cfg = dev->config;
	struct pwm_rh850_channel_data *data = dev->data;

	if (p_args->event != TIMER_EVENT_CYCLE_END) {
		return;
	}

	for (uint8_t i = 0U; i < TAUD_MAX_NUM_SLAVE_CHANNELS; i++) {
		if (cfg->synchronized_instances[i] != NULL) {
			pwm_fire_event_callbacks(&data->event_callbacks, dev,
						 cfg->synchronized_instances[i]->p_cfg->channel,
						 PWM_EVENT_TYPE_PERIOD);
		}
	}
}

static int pwm_rh850_manage_event_callback(const struct device *dev,
					   struct pwm_event_callback *callback, bool set)
{
	struct pwm_rh850_channel_data *data = dev->data;

	/* Only register callbacks that include PWM_EVENT_TYPE_PERIOD, which is the
	 * only event this driver can generate.
	 */
	if (set && (callback->event_mask & PWM_EVENT_TYPE_PERIOD) == 0) {
		LOG_ERR("%s: callback event_mask 0x%x does not include PWM_EVENT_TYPE_PERIOD.",
			dev->name, callback->event_mask);
		return -EINVAL;
	}

	return pwm_manage_event_callback(&data->event_callbacks, callback, set);
}
#endif /* CONFIG_PWM_EVENT */

/* PWM API mapping. */
static DEVICE_API(pwm, pwm_rh850_driver_api) = {
	.set_cycles = pwm_rh850_set_cycles,
	.get_cycles_per_sec = pwm_rh850_get_cycles_per_sec,
#ifdef CONFIG_PWM_CAPTURE
	.configure_capture = pwm_rh850_configure_capture,
	.enable_capture = pwm_rh850_enable_capture,
	.disable_capture = pwm_rh850_disable_capture,
#endif /* CONFIG_PWM_CAPTURE */
#ifdef CONFIG_PWM_EVENT
	.manage_event_callback = pwm_rh850_manage_event_callback,
#endif /* CONFIG_PWM_EVENT */
};

#ifdef CONFIG_PWM_EVENT
#define PWM_RH850_IRQ_CONFIG(n)                                                                    \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), pwm_rh850_channel_isr,      \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	} while (0)
#else
#define PWM_RH850_IRQ_CONFIG(n)
#endif /* CONFIG_PWM_EVENT */

/* Unique per-child symbol, keyed by the child node's devicetree ordinal */
#define PWM_RH850_CHILD_SYM(prefix, child) _CONCAT(prefix, DT_DEP_ORD(child))

/* channel-mode enum index: 0 = "synchronized", 1 = "independent" */
#define PWM_RH850_IS_INDEPENDENT(child) DT_ENUM_IDX(child, channel_mode)

/* A synchronized channel follows the nearest primary below it,
 * so no other primary may sit in between
 */
#define PWM_RH850_NO_PRIMARY_BETWEEN(other, child, n)                                              \
	BUILD_ASSERT(                                                                              \
		(DT_PROP(other, unit) != DT_INST_PROP(n, unit)) ||                                 \
			(DT_PROP(other, channel) <= DT_INST_PROP(n, channel)) ||                   \
			(DT_PROP(other, channel) > DT_REG_ADDR(child)),                            \
		"TAUD synchronized channel is separated from its primary by another primary");

/* "synchronized" mode: synchronized with the primary, PWM output (reg = TAUD channel) */
#define PWM_RH850_SYNCHRONIZED_DEFINE(child, n)                                                    \
	BUILD_ASSERT(DT_REG_ADDR(child) > DT_INST_PROP(n, channel),                                \
		     "TAUD synchronized channel must be above its primary channel");               \
	DT_FOREACH_STATUS_OKAY_VARGS(DT_DRV_COMPAT, PWM_RH850_NO_PRIMARY_BETWEEN, child, n)        \
	static taud_instance_ctrl_t PWM_RH850_CHILD_SYM(pwm_rh850_sync_ctrl_, child);              \
	static taud_extended_cfg_t PWM_RH850_CHILD_SYM(pwm_rh850_sync_ext_cfg_, child) = {         \
		.taud_function = TAUD_FUNCTION_PWM_OUTPUT,                                         \
		.trigger_type = TAUD_TRIGGER_MASTER_INT,                                           \
		.operating_mode = TAUD_MODE_ONE_COUNT_MODE,                                        \
		.output_enable = TAUD_OUTPUT_ENABLE,                                               \
		.output_mode = TAUD_OUTPUT_MODE_SYNCHRONOUS,                                       \
		.channel_type = TAUD_CHANNEL_TYPE_SLAVE,                                           \
		.simultaneous_rewrite = TAUD_SIMULTANEOUS_REWRITE_ENABLE,                          \
		.taud_unit = DT_INST_PROP(n, unit),                                                \
		.mode_config = TAUD_MODE_SPECIFIC_START_INT_GENERATED,                             \
	};                                                                                         \
	static timer_cfg_t PWM_RH850_CHILD_SYM(pwm_rh850_sync_cfg_, child) = {                     \
		.channel = DT_REG_ADDR(child),                                                     \
		.cycle_end_ipl = 0,                                                                \
		.cycle_end_irq = -1,                                                               \
		.p_extend = &PWM_RH850_CHILD_SYM(pwm_rh850_sync_ext_cfg_, child),                  \
	};                                                                                         \
	static const timer_instance_t PWM_RH850_CHILD_SYM(pwm_rh850_sync_inst_, child) = {         \
		.p_ctrl = &PWM_RH850_CHILD_SYM(pwm_rh850_sync_ctrl_, child),                       \
		.p_cfg = &PWM_RH850_CHILD_SYM(pwm_rh850_sync_cfg_, child),                         \
		.p_api = &g_timer_on_taud,                                                         \
	};

#ifdef CONFIG_PWM_CAPTURE
/* "independent" mode: own input trigger, PWM capture input (reg = TAUD channel) */
#define PWM_RH850_INDEPENDENT_DEFINE(child, n)                                                     \
	static struct pwm_rh850_capture_data PWM_RH850_CHILD_SYM(pwm_rh850_capture_, child) = {    \
		.dev = DEVICE_DT_INST_GET(n),                                                      \
		.cfg =                                                                             \
			{                                                                          \
				.channel = DT_REG_ADDR(child),                                     \
				.cycle_end_ipl = DT_IRQ(child, priority),                          \
				.cycle_end_irq = DT_IRQN(child),                                   \
				.p_callback = pwm_rh850_capture_isr_cb,                            \
				.p_context = &PWM_RH850_CHILD_SYM(pwm_rh850_capture_, child),      \
				.p_extend =                                                        \
					&PWM_RH850_CHILD_SYM(pwm_rh850_capture_, child).ext_cfg,   \
			},                                                                         \
		.ext_cfg =                                                                         \
			{                                                                          \
				.channel_type = TAUD_CHANNEL_TYPE_INDEPENDENT,                     \
				.taud_unit = DT_INST_PROP(n, unit),                                \
				.output_enable = TAUD_OUTPUT_DISABLE,                              \
				.overflow_timing = TAUD_OVERFLOW_AUTO_CLEAR,                       \
				.mode_config = TAUD_MODE_SPECIFIC_START_INT_NOT_GENERATED,         \
			},                                                                         \
	};

#define PWM_RH850_CAPTURE_ENTRY(child)                                                             \
	COND_CODE_1(PWM_RH850_IS_INDEPENDENT(child),                                               \
		    ([DT_REG_ADDR(child)] = &PWM_RH850_CHILD_SYM(pwm_rh850_capture_, child),), ())

#define PWM_RH850_CAPTURE_IRQ_CONNECT(child)                                                       \
	COND_CODE_1(PWM_RH850_IS_INDEPENDENT(child),                                               \
		    (IRQ_CONNECT(DT_IRQN(child), DT_IRQ(child, priority), pwm_rh850_channel_isr,   \
				 NULL, 0);),                                                       \
		    ())

/* The extra trailing entry keeps the initializer non-empty when there is no independent child */
#define PWM_RH850_CAPTURE_TABLE_DEFINE(n)                                                          \
	static struct pwm_rh850_capture_data *const                                                \
		pwm_rh850_captures_##n[TAUD_MAX_NUM_CHANNELS + 1] = {                              \
			DT_INST_FOREACH_CHILD_STATUS_OKAY(                                         \
				n, PWM_RH850_CAPTURE_ENTRY)[TAUD_MAX_NUM_CHANNELS] = NULL,         \
	};

#define PWM_RH850_CAPTURE_CFG(n) .captures = pwm_rh850_captures_##n,
#define PWM_RH850_CAPTURE_IRQ_CONFIG(n)                                                            \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(n, PWM_RH850_CAPTURE_IRQ_CONNECT)
#else
#define PWM_RH850_INDEPENDENT_DEFINE(child, n)
#define PWM_RH850_CAPTURE_TABLE_DEFINE(n)
#define PWM_RH850_CAPTURE_CFG(n)
#define PWM_RH850_CAPTURE_IRQ_CONFIG(n)
#endif /* CONFIG_PWM_CAPTURE */

#ifdef CONFIG_PWM_EVENT
#define PWM_RH850_EVENT_CB_CFG(n)                                                                  \
	.p_callback = pwm_rh850_primary_isr_cb, .p_context = (void *)DEVICE_DT_INST_GET(n),
#define PWM_RH850_PRIMARY_IRQ_CFG(n)                                                               \
	.cycle_end_ipl = DT_INST_IRQ(n, priority), .cycle_end_irq = DT_INST_IRQN(n),
#else
#define PWM_RH850_EVENT_CB_CFG(n)
#define PWM_RH850_PRIMARY_IRQ_CFG(n) .cycle_end_ipl = 0, .cycle_end_irq = -1,
#endif /* CONFIG_PWM_EVENT */

#define PWM_RH850_CHILD_DEFINE(child, n)                                                           \
	BUILD_ASSERT(DT_REG_ADDR(child) < TAUD_MAX_NUM_CHANNELS, "TAUD channel out of range");     \
	BUILD_ASSERT(DT_REG_ADDR(child) != DT_INST_PROP(n, channel),                               \
		     "TAUD child channel must differ from primary channel");                       \
	COND_CODE_1(PWM_RH850_IS_INDEPENDENT(child), (PWM_RH850_INDEPENDENT_DEFINE(child, n)),     \
		    (PWM_RH850_SYNCHRONIZED_DEFINE(child, n)))

#define PWM_RH850_SYNCHRONIZED_ENTRY(child)                                                        \
	COND_CODE_1(PWM_RH850_IS_INDEPENDENT(child), (),                                           \
		    (&PWM_RH850_CHILD_SYM(pwm_rh850_sync_inst_, child),))

/* Accumulate DNFA enable bits (one per channel) from child nodes with noise-filter-enable=true.
 * BIT(channel_reg) sets the corresponding bit in DNFAENn register.
 */
#define PWM_RH850_NF_MASK(child)                                                                   \
	COND_CODE_1(PWM_RH850_IS_INDEPENDENT(child),                                             \
		    (COND_CODE_1(DT_PROP(child, noise_filter_enable),                         \
				 (BIT(DT_REG_ADDR(child)) |), ())), ())

#define PWM_RH850_NF_DNFA_ASSERT(n)                                                                \
	BUILD_ASSERT((DT_INST_FOREACH_CHILD_STATUS_OKAY(n, PWM_RH850_NF_MASK) 0U) == 0U ||         \
			     DT_INST_REG_ADDR_BY_NAME_OR(n, dnfa, 0) != 0U,                        \
		     "noise-filter-enable requires a dnfa register block");

#define PWM_RH850_SHARED_CONFIG_MATCH(other, n)                                                    \
	BUILD_ASSERT(                                                                              \
		(DT_PROP(other, unit) != DT_INST_PROP(n, unit)) ||                                 \
			(DT_PROP(other, noise_filter_prescaler) ==                                 \
			 DT_INST_PROP(n, noise_filter_prescaler)),                                 \
		"TAUD primary nodes in the same unit must use the same noise-filter-prescaler");   \
	BUILD_ASSERT((DT_PROP(other, unit) != DT_INST_PROP(n, unit)) ||                            \
			     (DT_PROP(other, clock_divider) == DT_INST_PROP(n, clock_divider)),    \
		     "TAUD primary nodes in the same unit must use the same clock-divider");

#define PWM_RH850_CHANNEL_INIT(n)                                                                  \
	BUILD_ASSERT(DT_INST_PROP(n, unit) == 0, "TAUD unit 0 is the only supported unit");        \
	BUILD_ASSERT(DT_INST_PROP(n, channel) < TAUD_MAX_NUM_CHANNELS,                             \
		     "TAUD primary channel out of range");                                         \
	BUILD_ASSERT((DT_INST_PROP(n, channel) % 2) == 0, "TAUD primary channel must be even");    \
	BUILD_ASSERT(DT_INST_CHILD_NUM_STATUS_OKAY(n) <= TAUD_MAX_NUM_SLAVE_CHANNELS,              \
		     "Too many TAUD child channels");                                              \
	DT_FOREACH_STATUS_OKAY_VARGS(DT_DRV_COMPAT, PWM_RH850_SHARED_CONFIG_MATCH, n)              \
	PWM_RH850_NF_DNFA_ASSERT(n)                                                                \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(n, PWM_RH850_CHILD_DEFINE, n)                      \
	PWM_RH850_CAPTURE_TABLE_DEFINE(n)                                                          \
	static taud_instance_ctrl_t g_ctrl_##n;                                                    \
	static taud_extended_cfg_t g_ext_cfg_##n = {                                               \
		.taud_function = TAUD_FUNCTION_PWM_OUTPUT,                                         \
		.trigger_type = TAUD_TRIGGER_SOFTWARE,                                             \
		.operating_mode = TAUD_MODE_INTERVAL_TIMER_MODE,                                   \
		.output_enable = TAUD_OUTPUT_ENABLE,                                               \
		.output_mode = TAUD_OUTPUT_MODE_INDEPENDENT,                                       \
		.channel_type = TAUD_CHANNEL_TYPE_MASTER,                                          \
		.simultaneous_rewrite = TAUD_SIMULTANEOUS_REWRITE_ENABLE,                          \
		.taud_unit = DT_INST_PROP(n, unit),                                                \
		.mode_config = TAUD_MODE_SPECIFIC_START_INT_GENERATED,                             \
		.p_slave_channel_instances = {DT_INST_FOREACH_CHILD_STATUS_OKAY(                   \
			n, PWM_RH850_SYNCHRONIZED_ENTRY)},                                         \
	};                                                                                         \
	static timer_cfg_t g_cfg_##n = {                                                           \
		.channel = DT_INST_PROP(n, channel),                                               \
		PWM_RH850_PRIMARY_IRQ_CFG(n) PWM_RH850_EVENT_CB_CFG(n).p_extend = &g_ext_cfg_##n,  \
	};                                                                                         \
	static const timer_instance_t g_inst_##n = {                                               \
		.p_ctrl = &g_ctrl_##n,                                                             \
		.p_cfg = &g_cfg_##n,                                                               \
		.p_api = &g_timer_on_taud,                                                         \
	};                                                                                         \
	static const struct pwm_rh850_channel_config pwm_rh850_channel_cfg_##n = {                 \
		.fsp_api = &g_timer_on_taud,                                                       \
		.unit = DT_INST_PROP(n, unit),                                                     \
		.channel = DT_INST_PROP(n, channel),                                               \
		.clock_source = DT_INST_PROP_OR(n, clock_source, 0),                               \
		.clock_divider = DT_INST_PROP_OR(n, clock_divider, 0),                             \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)(uintptr_t)DT_INST_CLOCKS_CELL(n, clk_id), \
		.pinctrl_dev = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                  \
		.synchronized_instances = g_ext_cfg_##n.p_slave_channel_instances,                 \
		.noise_filter_prescaler = DT_INST_PROP(n, noise_filter_prescaler),                 \
		.noise_filter_mask = (DT_INST_FOREACH_CHILD_STATUS_OKAY(n, PWM_RH850_NF_MASK) 0U), \
		.dnfa_addr = DT_INST_REG_ADDR_BY_NAME_OR(n, dnfa, 0),                              \
		PWM_RH850_CAPTURE_CFG(n)};                                                         \
	static struct pwm_rh850_channel_data pwm_rh850_channel_data_##n = {                        \
		.ctrl = &g_ctrl_##n,                                                               \
		.cfg = &g_cfg_##n,                                                                 \
		.ext_cfg = &g_ext_cfg_##n,                                                         \
		.period_counts = 0,                                                                \
		.num_synchronized = 0,                                                             \
		.primary_running = false,                                                          \
	};                                                                                         \
	static int pwm_rh850_channel_init_##n(const struct device *dev)                            \
	{                                                                                          \
		int ret = pwm_rh850_channel_init(dev);                                             \
		if (ret < 0) {                                                                     \
			return ret;                                                                \
		}                                                                                  \
		PWM_RH850_IRQ_CONFIG(n);                                                           \
		PWM_RH850_CAPTURE_IRQ_CONFIG(n)                                                    \
		return 0;                                                                          \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(n, pwm_rh850_channel_init_##n, NULL, &pwm_rh850_channel_data_##n,    \
			      &pwm_rh850_channel_cfg_##n, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,   \
			      &pwm_rh850_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_RH850_CHANNEL_INIT)
