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

};

/* Channel runtime data */
struct pwm_rh850_channel_data {
	timer_ctrl_t *ctrl;
	timer_cfg_t *cfg;
	taud_extended_cfg_t *ext_cfg;

	uint32_t period_counts;
	uint8_t num_synchronized;
	bool primary_running;

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
			taud_extended_cfg_t *sync_ext_cfg = (taud_extended_cfg_t *)
				cfg->synchronized_instances[i]->p_cfg->p_extend;
			sync_ext_cfg->operation_clk = (taud_clock_t)cfg->clock_source;
			sync_ext_cfg->clk_div = (taud_div_t)cfg->clock_divider;
		}
	}


	/* Apply pin control configuration (ignored if pinctrl-0 not defined) */
	ret = pinctrl_apply_state(cfg->pinctrl_dev, PINCTRL_STATE_DEFAULT);
	if (ret < 0 && ret != -ENOENT) {
		LOG_ERR("%s: pinctrl failed: %d.", dev->name, ret);
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


/* PWM API mapping. */
static DEVICE_API(pwm, pwm_rh850_driver_api) = {
	.set_cycles = pwm_rh850_set_cycles,
	.get_cycles_per_sec = pwm_rh850_get_cycles_per_sec,
};

#define PWM_RH850_IRQ_CONFIG(n)                                                                    \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), pwm_rh850_channel_isr,      \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	} while (0)

/* Unique per-child symbol, keyed by the child node's devicetree ordinal */
#define PWM_RH850_CHILD_SYM(prefix, child) _CONCAT(prefix, DT_DEP_ORD(child))

/* channel-mode enum index: 0 = "synchronized", 1 = "independent" */
#define PWM_RH850_IS_INDEPENDENT(child) DT_ENUM_IDX(child, channel_mode)

/* A synchronized channel follows the nearest primary below it,
 * so no other primary may sit in between
 */
#define PWM_RH850_NO_PRIMARY_BETWEEN(other, child, n)                                              \
	BUILD_ASSERT((DT_PROP(other, unit) != DT_INST_PROP(n, unit)) ||                            \
		     (DT_PROP(other, channel) <= DT_INST_PROP(n, channel)) ||                      \
		     (DT_PROP(other, channel) > DT_REG_ADDR(child)),                               \
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

#define PWM_RH850_INDEPENDENT_DEFINE(child, n)
#define PWM_RH850_CAPTURE_TABLE_DEFINE(n)
#define PWM_RH850_CAPTURE_CFG(n)
#define PWM_RH850_CAPTURE_IRQ_CONFIG(n)
#define PWM_RH850_EVENT_CB_CFG(n)
#define PWM_RH850_PRIMARY_IRQ_CFG(n) .cycle_end_ipl = 0, .cycle_end_irq = -1,

#define PWM_RH850_CHILD_DEFINE(child, n)                                                           \
	BUILD_ASSERT(DT_REG_ADDR(child) < TAUD_MAX_NUM_CHANNELS, "TAUD channel out of range");     \
	BUILD_ASSERT(DT_REG_ADDR(child) != DT_INST_PROP(n, channel),                               \
		     "TAUD child channel must differ from primary channel");                       \
	COND_CODE_1(PWM_RH850_IS_INDEPENDENT(child), (PWM_RH850_INDEPENDENT_DEFINE(child, n)),     \
		    (PWM_RH850_SYNCHRONIZED_DEFINE(child, n)))

#define PWM_RH850_SYNCHRONIZED_ENTRY(child)                                                        \
	COND_CODE_1(PWM_RH850_IS_INDEPENDENT(child), (),                                           \
		    (&PWM_RH850_CHILD_SYM(pwm_rh850_sync_inst_, child),))

#define PWM_RH850_CHANNEL_INIT(n)                                                                  \
	BUILD_ASSERT(DT_INST_PROP(n, channel) < TAUD_MAX_NUM_CHANNELS,                             \
		     "TAUD primary channel out of range");                                         \
	BUILD_ASSERT((DT_INST_PROP(n, channel) % 2) == 0, "TAUD primary channel must be even");    \
	BUILD_ASSERT(DT_INST_CHILD_NUM_STATUS_OKAY(n) <= TAUD_MAX_NUM_SLAVE_CHANNELS,              \
		     "Too many TAUD child channels");                                              \
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
		.p_slave_channel_instances = {                                                     \
			DT_INST_FOREACH_CHILD_STATUS_OKAY(n, PWM_RH850_SYNCHRONIZED_ENTRY)         \
		},                                                                                 \
	};                                                                                         \
	static timer_cfg_t g_cfg_##n = {                                                           \
		.channel = DT_INST_PROP(n, channel),                                               \
		PWM_RH850_PRIMARY_IRQ_CFG(n)                                                       \
		PWM_RH850_EVENT_CB_CFG(n)                                                          \
		.p_extend = &g_ext_cfg_##n,                                                        \
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
		PWM_RH850_CAPTURE_CFG(n)                                                           \
	};                                                                                         \
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
	DEVICE_DT_INST_DEFINE(n, pwm_rh850_channel_init_##n, NULL,                                 \
			      &pwm_rh850_channel_data_##n, &pwm_rh850_channel_cfg_##n,             \
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY, &pwm_rh850_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_RH850_CHANNEL_INIT)
