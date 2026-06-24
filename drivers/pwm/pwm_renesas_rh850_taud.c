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
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util_macro.h>

LOG_MODULE_REGISTER(rh850_pwm, CONFIG_PWM_LOG_LEVEL);

extern void taud_int_isr(void);
extern const timer_api_t g_timer_on_taud;

/* ================================================================== */
/* HELPERS                                                            */
/* ================================================================== */

#define PWM_RH850_IS_MASTER(n) DT_INST_ENUM_HAS_VALUE(n, channel_type, master)
#define PWM_RH850_IS_SLAVE(n)  DT_INST_ENUM_HAS_VALUE(n, channel_type, slave)

/* ================================================================== */
/* MASTER PART                                                        */
/* ================================================================== */

/* Configuration for a PWM Master Channel. */
struct pwm_rh850_master_config {
	const timer_api_t *fsp_api;
	uint32_t unit;
	uint32_t channel;
	uint32_t irq;
	uint32_t irq_prio;
};

/* Data for a PWM Master Channel. */
struct pwm_rh850_master_data {
	timer_ctrl_t *ctrl;
	timer_cfg_t *cfg;
	taud_extended_cfg_t *ext_cfg;
	uint8_t num_slaves;
};

/* Handle the Master Channel interrupt. */
static void pwm_rh850_master_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	taud_int_isr();
}

static int pwm_rh850_master_register_slave(const struct device *master_dev,
					   const timer_instance_t *slave_inst)
{
	struct pwm_rh850_master_data *master_data = master_dev->data;

	/* Ensure this Master Channel does not exceed its supported Slave Channel limit.
	 * For example, Master Channel 0 supports up to 15 Slave Channels.
	 */
	if (master_data->num_slaves >= TAUD_MAX_NUM_SLAVE_CHANNELS) {
		LOG_ERR("slave channel limit reached (max=%u).", TAUD_MAX_NUM_SLAVE_CHANNELS);
		return -ENOMEM;
	}

	master_data->ext_cfg->p_slave_channel_instances[master_data->num_slaves] = slave_inst;
	master_data->num_slaves++;

	/* Log the successful registration of the Slave Channel(s) to the Master Channel. */
	LOG_DBG("master %s: registered slave %u.", master_dev->name, master_data->num_slaves);

	return 0;
}

/* Initializes the PWM Master Channel. */
static int pwm_rh850_master_init(const struct device *dev)
{
	const struct pwm_rh850_master_config *master_cfg = dev->config;
	struct pwm_rh850_master_data *master_data = dev->data;
	fsp_err_t err;

	/* The Master Channel must use an even-numbered channel. */
	if ((master_cfg->channel % 2) != 0) {
		LOG_ERR("%s: master channel must be even (ch=%u).", dev->name, master_cfg->channel);
		return -EINVAL;
	}

	/* Open the Master Channel. This also opens all Slave Channels linked to this Master. */
	err = master_cfg->fsp_api->open(master_data->ctrl, master_data->cfg);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: master open failed: %d.", dev->name, err);
		return -EIO;
	}

	/* Log the successful initialization of the Master Channel. */
	LOG_INF("%s: master init OK (unit=%u , ch=%u , slaves=%u).", dev->name, master_cfg->unit,
		master_cfg->channel, master_data->num_slaves);

	return 0;
}

/* ================================================================== */
/* SLAVE PART                                                         */
/* ================================================================== */

/* Configuration for a PWM Slave Channel. */
struct pwm_rh850_slave_config {
	const struct pinctrl_dev_config *pinctrl_dev;
	const timer_api_t *fsp_api;
	const struct device *master_dev;
	const timer_instance_t *slave_inst;
	uint32_t unit;
	uint32_t channel;
};

/* Data for a PWM Slave Channel. */
struct pwm_rh850_slave_data {
	timer_ctrl_t *ctrl;
	timer_cfg_t *cfg;
};

/* Handle the Slave Channel interrupt. */
static void pwm_rh850_slave_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	taud_int_isr();
}

/* Configure the PWM period and duty cycle for a Slave Channel.
 *
 * The period is configured through the Master Channel because
 * all Slave Channels assigned to the same Master share the same period.
 *
 * The duty cycle is configured for this Slave Channel only.
 */
static int pwm_rh850_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
				uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_rh850_slave_config *slave_config = dev->config;
	struct pwm_rh850_slave_data *slave_data = dev->data;
	struct pwm_rh850_master_config *master_config = slave_config->master_dev->config;
	struct pwm_rh850_master_data *master_data = slave_config->master_dev->data;
	taud_instance_ctrl_t *const slave_ctrl = slave_data->ctrl;

	fsp_err_t err;

	if (channel >= TAUD_MAX_NUM_SLAVE_CHANNELS) {
		LOG_INF("Fail to set channel: %d.", channel);
		return -EINVAL;
	}

	/* Set output polarity for this channel in TAUDnTOL register */
	slave_ctrl->p_reg->TAUDnTOL |= (flags << channel);

	/* Reject zero period because it is not a valid PWM period. */
	if (period_cycles == 0U) {
		LOG_ERR("%s: period_cycles is zero, invalid input.", dev->name);
		return -EINVAL;
	}

	/* Clamp the pulse width to the period to avoid an invalid duty cycle. */
	if (pulse_cycles > period_cycles) {
		pulse_cycles = period_cycles;
	}

	/* Update the shared period only when it is different from the current Master period. */
	if (period_cycles != master_data->cfg->period_counts) {

		/* Notify that changing the Master period affects all registered Slave Channels. */
		if (master_data->num_slaves > 1) {
			LOG_INF("%s: changing the period affects for all %u slaves.", dev->name,
				master_data->num_slaves);
		}

		/* Set the PWM period through the Master Channel. */
		err = slave_config->fsp_api->periodSet(master_data->ctrl, period_cycles);
		if (err == FSP_ERR_INVALID_HW_CONDITION) {
			LOG_ERR("%s: The simultaneous rewrite status had not yet completed.: %d.",
				dev->name, err);
			return -EBUSY;
		} else if (err != FSP_SUCCESS) {
			LOG_ERR("%s: PeriodSet failed: %d.", dev->name, err);
			return -EIO;
		}

		/* Keep the cached Master period in sync with the updated hardware setting. */
		master_data->cfg->period_counts = period_cycles;
	}

	/* Set the PWM duty cycle for this Slave Channel. */
	err = slave_config->fsp_api->dutyCycleSet(slave_data->ctrl, pulse_cycles, 0);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: DutyCycleSet failed: %d.", dev->name, err);
		return -EIO;
	}

	/* Start the Master Channel.
	 * This also starts the PWM output for its Slave Channels.
	 */
	err = master_config->fsp_api->start(master_data->ctrl);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: master start failed: %d.", dev->name, err);
		return -EIO;
	}

	return 0;
}

/* Get the PWM clock frequency used by the Master Channel. */
static int pwm_rh850_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					uint64_t *cycles)
{
	const struct pwm_rh850_slave_config *slave_config = dev->config;
	struct pwm_rh850_master_data *master_data = slave_config->master_dev->data;
	timer_info_t info;
	fsp_err_t err;

	/* The channel is not used because the clock source is shared by the Master Channel. */
	ARG_UNUSED(channel);

	/* Get timer information from the Master Channel. */
	err = slave_config->fsp_api->infoGet(master_data->ctrl, &info);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	/* Return the timer clock frequency as cycles per second. */
	*cycles = (uint64_t)info.clock_frequency;

	return 0;
}

/* PWM API mapping. */
static DEVICE_API(pwm, pwm_rh850_driver_api) = {
	.set_cycles = pwm_rh850_set_cycles,
	.get_cycles_per_sec = pwm_rh850_get_cycles_per_sec,
};

/* Initializes the PWM Slave Channel(s). */
static int pwm_rh850_slave_init(const struct device *dev)
{
	const struct pwm_rh850_slave_config *slave_config = dev->config;
	const struct pwm_rh850_master_config *master_config = slave_config->master_dev->config;
	int ret;

	/* Port pin configuration for PWM Output (Only for Slave Channel). */
	ret = pinctrl_apply_state(slave_config->pinctrl_dev, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("%s: pinctrl failed: %d.", dev->name, ret);
		return ret;
	}

	/* The TAUD Unit of the Slave Channel must be the same as the Master Channel. */
	if (slave_config->unit != master_config->unit) {
		LOG_ERR("%s: unit mismatch (slave=%u , master=%u).", dev->name, slave_config->unit,
			master_config->unit);
		return -EINVAL;
	}

	/* The Slave Channel number must be greater than or equal to the Master Channel number. */
	if (slave_config->channel < master_config->channel) {
		LOG_ERR("%s: slave channel %u must be > master channel %u.", dev->name,
			slave_config->channel, master_config->channel);
		return -EINVAL;
	}

	/* Register this Slave Channel to its configured Master Channel. */
	ret = pwm_rh850_master_register_slave(slave_config->master_dev, slave_config->slave_inst);
	if (ret < 0) {
		LOG_ERR("%s: register with master failed: %d.", dev->name, ret);
		return ret;
	}

	/* Log the successful initialization of the Slave Channel. */
	LOG_INF("%s: slave init OK (unit=%u , ch=%u , master=%s).", dev->name, slave_config->unit,
		slave_config->channel, slave_config->master_dev->name);

	return 0;
}

#define PWM_RH850_MASTER_IRQ_CONFIG(n)                                                             \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), pwm_rh850_master_isr,       \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	} while (0)

#define PWM_RH850_MASTER_INIT(n)                                                                   \
	static taud_instance_ctrl_t g_master_ctrl_##n;                                             \
	static taud_extended_cfg_t g_master_ext_cfg_##n = {                                        \
		.taud_function = TAUD_FUNCTION_PWM_OUTPUT,                                         \
		.trigger_type = TAUD_TRIGGER_SOFTWARE,                                             \
		.operating_mode = TAUD_MODE_INTERVAL_TIMER_MODE,                                   \
		.simultaneous_rewrite = TAUD_SIMULTANEOUS_REWRITE_ENABLE,                          \
		.channel_type = TAUD_CHANNEL_TYPE_MASTER,                                          \
		.taud_unit = DT_INST_PROP(n, unit),                                                \
		.mode_config = TAUD_MODE_SPECIFIC_START_INT_GENERATED,                             \
	};                                                                                         \
	static timer_cfg_t g_master_cfg_##n = {                                                    \
		.channel = DT_INST_PROP(n, channel),                                               \
		.cycle_end_ipl = DT_INST_IRQ(n, priority),                                         \
		.cycle_end_irq = DT_INST_IRQN(n),                                                  \
		.p_extend = &g_master_ext_cfg_##n,                                                 \
	};                                                                                         \
	static const struct pwm_rh850_master_config pwm_rh850_master_cfg_##n = {                   \
		.fsp_api = &g_timer_on_taud,                                                       \
		.unit = DT_INST_PROP(n, unit),                                                     \
		.channel = DT_INST_PROP(n, channel),                                               \
	};                                                                                         \
	static struct pwm_rh850_master_data pwm_rh850_master_data_##n = {                          \
		.ctrl = &g_master_ctrl_##n,                                                        \
		.cfg = &g_master_cfg_##n,                                                          \
		.ext_cfg = &g_master_ext_cfg_##n,                                                  \
		.num_slaves = 0,                                                                   \
	};                                                                                         \
	static int pwm_rh850_master_init_##n(const struct device *dev)                             \
	{                                                                                          \
		int ret = pwm_rh850_master_init(dev);                                              \
		if (ret < 0) {                                                                     \
			return ret;                                                                \
		}                                                                                  \
		PWM_RH850_MASTER_IRQ_CONFIG(n);                                                    \
		return 0;                                                                          \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(n, pwm_rh850_master_init_##n, NULL, &pwm_rh850_master_data_##n,      \
			      &pwm_rh850_master_cfg_##n, POST_KERNEL,                              \
			      CONFIG_PWM_RH850_MASTER_INIT_PRIORITY, NULL);

#define PWM_RH850_SLAVE_IRQ_CONFIG(n)                                                              \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), pwm_rh850_slave_isr,        \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	} while (0)

#define PWM_RH850_SLAVE_INIT(n)                                                                    \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static taud_instance_ctrl_t g_slave_ctrl_##n;                                              \
	static taud_extended_cfg_t g_slave_ext_cfg_##n = {                                         \
		.taud_function = TAUD_FUNCTION_PWM_OUTPUT,                                         \
		.trigger_type = TAUD_TRIGGER_MASTER_INT,                                           \
		.operating_mode = TAUD_MODE_ONE_COUNT_MODE,                                        \
		.output_enable = TAUD_OUTPUT_ENABLE,                                               \
		.channel_type = TAUD_CHANNEL_TYPE_SLAVE,                                           \
		.output_mode = TAUD_OUTPUT_MODE_SYNCHRONOUS,                                       \
		.simultaneous_rewrite = TAUD_SIMULTANEOUS_REWRITE_ENABLE,                          \
		.taud_unit = DT_INST_PROP(n, unit),                                                \
		.mode_config = TAUD_MODE_SPECIFIC_START_INT_GENERATED,                             \
	};                                                                                         \
	static timer_cfg_t g_slave_cfg_##n = {                                                     \
		.channel = DT_INST_PROP(n, channel),                                               \
		.cycle_end_ipl = DT_INST_IRQ(n, priority),                                         \
		.cycle_end_irq = DT_INST_IRQN(n),                                                  \
		.p_extend = &g_slave_ext_cfg_##n,                                                  \
	};                                                                                         \
	static const timer_instance_t g_slave_inst_##n = {                                         \
		.p_ctrl = &g_slave_ctrl_##n,                                                       \
		.p_cfg = &g_slave_cfg_##n,                                                         \
		.p_api = &g_timer_on_taud,                                                         \
	};                                                                                         \
	static const struct pwm_rh850_slave_config pwm_rh850_slave_config_##n = {                  \
		.pinctrl_dev = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                  \
		.fsp_api = &g_timer_on_taud,                                                       \
		.master_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, master)),                           \
		.slave_inst = &g_slave_inst_##n,                                                   \
		.unit = DT_INST_PROP(n, unit),                                                     \
		.channel = DT_INST_PROP(n, channel),                                               \
	};                                                                                         \
	static struct pwm_rh850_slave_data pwm_rh850_slave_data_##n = {                            \
		.ctrl = &g_slave_ctrl_##n,                                                         \
		.cfg = &g_slave_cfg_##n,                                                           \
	};                                                                                         \
	static int pwm_rh850_slave_init_##n(const struct device *dev)                              \
	{                                                                                          \
		int ret = pwm_rh850_slave_init(dev);                                               \
		if (ret < 0) {                                                                     \
			return ret;                                                                \
		}                                                                                  \
		PWM_RH850_SLAVE_IRQ_CONFIG(n);                                                     \
		return 0;                                                                          \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(n, pwm_rh850_slave_init_##n, NULL, &pwm_rh850_slave_data_##n,        \
			      &pwm_rh850_slave_config_##n, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,  \
			      &pwm_rh850_driver_api);

#define PWM_RH850_INIT(n)                                                                          \
	COND_CODE_1(PWM_RH850_IS_MASTER(n),                                                    \
		(PWM_RH850_MASTER_INIT(n)),                                                        \
		(PWM_RH850_SLAVE_INIT(n)))

DT_INST_FOREACH_STATUS_OKAY(PWM_RH850_INIT)
