/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_taud3p_pwm3p

#include <r_taud.h>
#include <r_taud_three_phase.h>
#include <r_tapa.h>

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pwm3p.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(rh850_pwm3p, CONFIG_PWM3P_LOG_LEVEL);

/* Interrupt handler of the HAL timer driver, shared by all timer interrupts. */
extern void taud_int_isr(void);

/*
 * Timer channels used by one PWM3P instance. The 13 channels of a timer unit have a fixed
 * role, so they are not configurable from devicetree.
 */
#define PWM3P_MASTER_PWM_CHANNEL 2U

/* Order: [U-high, U-low, V-high, V-low, W-high, W-low] */
static const uint8_t pwm3p_slave_pwm_channels[6] = {4, 5, 6, 7, 8, 9};

/* Order: [U, V, W] */
static const uint8_t pwm3p_master_oneshot_channels[3] = {10, 12, 14};
static const uint8_t pwm3p_slave_oneshot_channels[3] = {11, 13, 15};

struct pwm_rh850_taud3p_config {
	const struct pinctrl_dev_config *pincfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
#ifdef CONFIG_RENESAS_RH850_PIC
	const struct device *pic_dev;
#endif
	const three_phase_api_t *taud3p_api;
	uint32_t taud_three_phase_unit;
	uint32_t operation_clk;
	uint32_t clk_div;
	uint32_t cycle_end_irq;
	uint8_t cycle_end_ipl;
	bool has_cycle_end_irq;
};

struct pwm_rh850_taud3p_data {
	/* Timer channels (13 in total) grouped by the HAL three-phase driver. */
	taud_three_phase_instance_ctrl_t taud3p_ctrl;
	three_phase_cfg_t taud3p_cfg;
	taud_three_phase_extended_cfg_t taud3p_ext_cfg;

	taud_instance_ctrl_t master_pwm_ctrl;
	timer_cfg_t master_pwm_cfg;
	taud_extended_cfg_t master_pwm_ext_cfg;
	timer_instance_t master_pwm_inst;

	taud_instance_ctrl_t slave_pwm_ctrl[6];
	timer_cfg_t slave_pwm_cfg[6];
	taud_extended_cfg_t slave_pwm_ext_cfg[6];
	timer_instance_t slave_pwm_inst[6];

	taud_instance_ctrl_t master_oneshot_ctrl[3];
	timer_cfg_t master_oneshot_cfg[3];
	taud_extended_cfg_t master_oneshot_ext_cfg[3];
	timer_instance_t master_oneshot_inst[3];

	taud_instance_ctrl_t slave_oneshot_ctrl[3];
	timer_cfg_t slave_oneshot_cfg[3];
	taud_extended_cfg_t slave_oneshot_ext_cfg[3];
	timer_instance_t slave_oneshot_inst[3];

	/* Auxiliary unit required by the HAL three-phase driver. It is configured with all of
	 * its functions disabled.
	 */
	tapa_instance_ctrl_t tapa_ctrl;
	tapa_cfg_t tapa_cfg;
	tapa_extended_cfg_t tapa_ext_cfg;
	tapa_instance_t tapa_inst;

	/* Cached state, used to rescale the duty cycles when the period changes. */
	uint32_t period_counts;
	uint32_t deadtime_counts;
	uint32_t duty_u;
	uint32_t duty_v;
	uint32_t duty_w;
	bool output_enabled;

	/* State of pwm3p_set_callback(). The callback runs from the per-period interrupt and is
	 * only used when config->has_cycle_end_irq is true.
	 */
	pwm3p_callback_t app_callback;
	void *app_callback_user_data;
	timer_callback_args_t callback_memory;
};

/* Settings shared by all timer channels of one PWM3P instance. */
static void pwm3p_taud_ext_cfg_common(taud_extended_cfg_t *ext,
				       const struct pwm_rh850_taud3p_config *config,
				       uint32_t deadtime_counts)
{
	*ext = (taud_extended_cfg_t){
		.count_clk = TAUD_COUNT_CLOCK_OPERATION_CLOCK,
		.operation_clk = (taud_clock_t)config->operation_clk,
		.clk_div = (taud_div_t)config->clk_div,
		.baudrate = 1,
		.taud_unit = (taud_unit_source_t)config->taud_three_phase_unit,
		.slave_ordinal_number = 0,
		.deadtime_value = deadtime_counts,
		.width_counts = 0,
		.delay_counts = 1,
		.divider_value = 2,
		.overflow_timing = TAUD_OVERFLOW_UNUSED,
		.real_time_output = TAUD_REAL_TIME_OUTPUT_DISABLE,
		.real_time_output_trigger = TAUD_REAL_TIME_OUTPUT_TRIGGER_UNUSED,
		.real_time_output_level = TAUD_REAL_TIME_OUTPUT_LEVEL_UNUSED,
		.modulation = TAUD_MODULATION_DISABLE,
		.simultaneous_rewrite = TAUD_SIMULTANEOUS_REWRITE_ENABLE,
		.simultaneous_rewrite_channel = TAUD_SIMULTANEOUS_REWRITE_CHANNEL_MASTER,
	};
}

/* Master channel of the triangle PWM, it generates the PWM period. */
static void pwm3p_channel_master_pwm(taud_extended_cfg_t *ext)
{
	ext->taud_function = TAUD_FUNCTION_TRIANGLE_PWM_OUTPUT_DEADTIME;
	ext->trigger_type = TAUD_TRIGGER_SOFTWARE;
	ext->operating_mode = TAUD_MODE_INTERVAL_TIMER_MODE;
	ext->output_enable = TAUD_OUTPUT_ENABLE;
	ext->output_mode = TAUD_OUTPUT_MODE_INDEPENDENT;
	ext->output_operating = TAUD_OUTPUT_OPERATING_MODE_1;
	ext->deadtime_operation = TAUD_DEADTIME_OPERATION_DISABLED;
	ext->channel_type = TAUD_CHANNEL_TYPE_MASTER;
	ext->mode_config = TAUD_MODE_SPECIFIC_START_INT_GENERATED;
	ext->edge_type = TAUD_INPUT_EDGE_UNUSED;
	ext->output_logic = TAUD_OUTPUT_LOGIC_DISABLED;
	ext->add_deadtime_phase = TAUD_ADD_DEADTIME_UNUSED;
	ext->simultaneous_rewrite_trigger = TAUD_SIMULTANEOUS_REWRITE_TRIGGER_SLAVE_PEAK;
}

/* Slave channel of the triangle PWM for a high-side output (U-high/V-high/W-high). */
static void pwm3p_channel_slave_pwm_even(taud_extended_cfg_t *ext)
{
	ext->taud_function = TAUD_FUNCTION_TRIANGLE_PWM_OUTPUT_DEADTIME;
	ext->trigger_type = TAUD_TRIGGER_UP_DOWN_OUTPUT;
	ext->operating_mode = TAUD_MODE_COUNT_UP_DOWN_MODE;
	ext->output_mode = TAUD_OUTPUT_MODE_SYNCHRONOUS;
	ext->output_enable = TAUD_OUTPUT_ENABLE;
	ext->output_operating = TAUD_OUTPUT_OPERATING_MODE_2;
	ext->deadtime_operation = TAUD_DEADTIME_OPERATION_ENABLED;
	ext->channel_type = TAUD_CHANNEL_TYPE_SLAVE_EVEN;
	ext->mode_config = TAUD_MODE_SPECIFIC_START_INT_NOT_GENERATED;
	ext->edge_type = TAUD_INPUT_EDGE_UNUSED;
	ext->output_logic = TAUD_OUTPUT_LOGIC_POSITIVE;
	ext->add_deadtime_phase = TAUD_ADD_DEADTIME_NORMAL_PHASE;
	ext->simultaneous_rewrite_trigger = TAUD_SIMULTANEOUS_REWRITE_TRIGGER_SLAVE_PEAK;
}

/* Slave channel of the triangle PWM for a low-side output (U-low/V-low/W-low). */
static void pwm3p_channel_slave_pwm_odd(taud_extended_cfg_t *ext)
{
	ext->taud_function = TAUD_FUNCTION_TRIANGLE_PWM_OUTPUT_DEADTIME;
	ext->trigger_type = TAUD_TRIGGER_DEAD_TIME_OUTPUT;
	ext->operating_mode = TAUD_MODE_ONE_COUNT_MODE;
	ext->output_enable = TAUD_OUTPUT_ENABLE;
	ext->output_mode = TAUD_OUTPUT_MODE_SYNCHRONOUS;
	ext->output_operating = TAUD_OUTPUT_OPERATING_MODE_2;
	ext->deadtime_operation = TAUD_DEADTIME_OPERATION_ENABLED;
	ext->channel_type = TAUD_CHANNEL_TYPE_SLAVE_ODD;
	ext->mode_config = TAUD_MODE_SPECIFIC_START_TRIGGER_ENABLE;
	ext->edge_type = TAUD_INPUT_EDGE_UNUSED;
	ext->output_logic = TAUD_OUTPUT_LOGIC_POSITIVE;
	ext->add_deadtime_phase = TAUD_ADD_DEADTIME_REVERSE_PHASE;
	ext->simultaneous_rewrite_trigger = TAUD_SIMULTANEOUS_REWRITE_TRIGGER_SLAVE_PEAK;
}

/* Master channel of the deadtime generator, one per phase (U/V/W). */
static void pwm3p_channel_master_oneshot(taud_extended_cfg_t *ext)
{
	ext->taud_function = TAUD_FUNCTION_ONE_SHOT_PULSE_OUTPUT;
	ext->trigger_type = TAUD_TRIGGER_VALID_EDGE;
	ext->operating_mode = TAUD_MODE_ONE_COUNT_MODE;
	ext->output_enable = TAUD_OUTPUT_DISABLE;
	ext->channel_type = TAUD_CHANNEL_TYPE_MASTER;
	ext->mode_config = TAUD_MODE_SPECIFIC_START_TRIGGER_DISABLE;
	ext->edge_type = TAUD_INPUT_EDGE_BOTH_MEASURE_LOW;
	ext->output_logic = TAUD_OUTPUT_LOGIC_DISABLED;
	ext->add_deadtime_phase = TAUD_ADD_DEADTIME_UNUSED;
	ext->simultaneous_rewrite_trigger = TAUD_SIMULTANEOUS_REWRITE_TRIGGER_UNUSED;
}

/* Slave channel of the deadtime generator, one per phase (U/V/W). */
static void pwm3p_channel_slave_oneshot(taud_extended_cfg_t *ext)
{
	ext->taud_function = TAUD_FUNCTION_ONE_SHOT_PULSE_OUTPUT;
	ext->trigger_type = TAUD_TRIGGER_MASTER_INT;
	ext->operating_mode = TAUD_MODE_PULSE_ONECOUNT_MODE;
	ext->output_enable = TAUD_OUTPUT_ENABLE;
	ext->output_operating = TAUD_OUTPUT_OPERATING_MODE_2;
	ext->output_mode = TAUD_OUTPUT_MODE_INDEPENDENT;
	ext->deadtime_operation = TAUD_DEADTIME_OPERATION_DISABLED;
	ext->channel_type = TAUD_CHANNEL_TYPE_SLAVE;
	ext->mode_config = TAUD_MODE_SPECIFIC_START_TRIGGER_DISABLE;
	ext->edge_type = TAUD_INPUT_EDGE_UNUSED;
	ext->output_logic = TAUD_OUTPUT_LOGIC_NEGATIVE;
	ext->add_deadtime_phase = TAUD_ADD_DEADTIME_UNUSED;
	ext->simultaneous_rewrite_trigger = TAUD_SIMULTANEOUS_REWRITE_TRIGGER_UNUSED;
}

/* Adapts the Zephyr interrupt handler signature to the HAL interrupt handler. */
static void pwm3p_rh850_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	taud_int_isr();
}

/* Forwards the end-of-period event to the callback registered with pwm3p_set_callback().
 * The context of the event is the device that registered the callback.
 */
static void pwm3p_taud_master_callback_shim(timer_callback_args_t *p_args)
{
	const struct device *dev = (const struct device *)p_args->p_context;
	struct pwm_rh850_taud3p_data *data = dev->data;

	if (p_args->event != TIMER_EVENT_CYCLE_END) {
		return;
	}

	if (data->app_callback != NULL) {
		data->app_callback(dev, data->app_callback_user_data);
	}
}

static void pwm3p_timer_cfg_init(timer_cfg_t *cfg, uint32_t channel, uint32_t period_counts,
				  uint32_t duty_cycle_counts, const taud_extended_cfg_t *ext)
{
	*cfg = (timer_cfg_t){
		.period_counts = period_counts,
		.duty_cycle_counts = duty_cycle_counts,
		.channel = (uint8_t)channel,
		.cycle_end_irq = FSP_INVALID_VECTOR,
		.cycle_end_ipl = BSP_IRQ_DISABLED,
		.p_callback = NULL,
		.p_context = NULL,
		.p_extend = ext,
	};
}

static void pwm_rh850_taud3p_tapa_build(const struct device *dev)
{
	const struct pwm_rh850_taud3p_config *config = dev->config;
	struct pwm_rh850_taud3p_data *data = dev->data;

	data->tapa_ext_cfg = (tapa_extended_cfg_t){
		.async_hiz_control_func = TAPA_FUNC_ASYNCHRONOUS_HIZ_CONTROL_DISABLE,
		.ad_conversion_trigger_func = TAPA_FUNC_AD_CONVERSION_TRIGGER_DISABLE,
		.clear_condition = TAPA_CLEAR_CONDITION_REGARDLESS_THASHIN_SIGNAL_LEVEL,
		.thasin_input_edge = TAPA_INPUT_EDGE_NOT_DETECT,
		.ad_conversion_trigger_0 = TAPA_INTERRUPT_TRIGGER_FALLING,
		.ad_conversion_trigger_1 = TAPA_INTERRUPT_TRIGGER_FALLING,
		.tapa_unit = (tapa_unit_t)config->taud_three_phase_unit,
	};
	data->tapa_cfg = (tapa_cfg_t){
		.p_extend = &data->tapa_ext_cfg,
		.p_callback = NULL,
		.p_context = NULL,
		.valley_irq = FSP_INVALID_VECTOR,
		.valley_ipl = BSP_IRQ_DISABLED,
		.peak_irq = FSP_INVALID_VECTOR,
		.peak_ipl = BSP_IRQ_DISABLED,
	};
	data->tapa_inst = (tapa_instance_t){
		.p_ctrl = (tapa_ctrl_t *)&data->tapa_ctrl,
		.p_cfg = &data->tapa_cfg,
		.p_api = &g_tapa_on_tapa,
	};
}

static int pwm_rh850_taud3p_taud_open(const struct device *dev)
{
	const struct pwm_rh850_taud3p_config *config = dev->config;
	struct pwm_rh850_taud3p_data *data = dev->data;
	fsp_err_t err;
	int i;

	/* Master channel of the triangle PWM. */
	pwm3p_taud_ext_cfg_common(&data->master_pwm_ext_cfg, config, data->deadtime_counts);
	pwm3p_channel_master_pwm(&data->master_pwm_ext_cfg);
	for (i = 0; i < 6; i++) {
		data->master_pwm_ext_cfg.p_slave_channel_instances[i] = &data->slave_pwm_inst[i];
	}
	pwm3p_timer_cfg_init(&data->master_pwm_cfg, PWM3P_MASTER_PWM_CHANNEL,
			     data->period_counts, data->period_counts / 2U,
			     &data->master_pwm_ext_cfg);
	if (config->has_cycle_end_irq) {
		data->master_pwm_cfg.cycle_end_irq = config->cycle_end_irq;
		data->master_pwm_cfg.cycle_end_ipl = config->cycle_end_ipl;
	}
	data->master_pwm_inst = (timer_instance_t){
		.p_ctrl = (timer_ctrl_t *)&data->master_pwm_ctrl,
		.p_cfg = &data->master_pwm_cfg,
		.p_api = &g_timer_on_taud,
	};

	/* Slave channels of the triangle PWM: even index = high side, odd index = low side. */
	for (i = 0; i < 6; i++) {
		pwm3p_taud_ext_cfg_common(&data->slave_pwm_ext_cfg[i], config,
					   data->deadtime_counts);
		if ((i % 2) == 0) {
			pwm3p_channel_slave_pwm_even(&data->slave_pwm_ext_cfg[i]);
		} else {
			pwm3p_channel_slave_pwm_odd(&data->slave_pwm_ext_cfg[i]);
		}
		pwm3p_timer_cfg_init(&data->slave_pwm_cfg[i], pwm3p_slave_pwm_channels[i],
				     data->period_counts, data->period_counts / 2U,
				     &data->slave_pwm_ext_cfg[i]);
		data->slave_pwm_inst[i] = (timer_instance_t){
			.p_ctrl = (timer_ctrl_t *)&data->slave_pwm_ctrl[i],
			.p_cfg = &data->slave_pwm_cfg[i],
			.p_api = &g_timer_on_taud,
		};
	}

	/* Deadtime generator channel pairs, one per phase. */
	for (i = 0; i < 3; i++) {
		pwm3p_taud_ext_cfg_common(&data->slave_oneshot_ext_cfg[i], config,
					   data->deadtime_counts);
		pwm3p_channel_slave_oneshot(&data->slave_oneshot_ext_cfg[i]);
		pwm3p_timer_cfg_init(&data->slave_oneshot_cfg[i],
				     pwm3p_slave_oneshot_channels[i], data->period_counts, 0,
				     &data->slave_oneshot_ext_cfg[i]);
		data->slave_oneshot_inst[i] = (timer_instance_t){
			.p_ctrl = (timer_ctrl_t *)&data->slave_oneshot_ctrl[i],
			.p_cfg = &data->slave_oneshot_cfg[i],
			.p_api = &g_timer_on_taud,
		};

		pwm3p_taud_ext_cfg_common(&data->master_oneshot_ext_cfg[i], config,
					   data->deadtime_counts);
		pwm3p_channel_master_oneshot(&data->master_oneshot_ext_cfg[i]);
		data->master_oneshot_ext_cfg[i].p_slave_channel_instances[0] =
			&data->slave_oneshot_inst[i];
		pwm3p_timer_cfg_init(&data->master_oneshot_cfg[i],
				     pwm3p_master_oneshot_channels[i], data->period_counts, 0,
				     &data->master_oneshot_ext_cfg[i]);
		data->master_oneshot_inst[i] = (timer_instance_t){
			.p_ctrl = (timer_ctrl_t *)&data->master_oneshot_ctrl[i],
			.p_cfg = &data->master_oneshot_cfg[i],
			.p_api = &g_timer_on_taud,
		};
	}

	pwm_rh850_taud3p_tapa_build(dev);

	const taud_three_phase_extended_cfg_t taud3p_ext_cfg = {
		.taud_unit = (uint8_t)config->taud_three_phase_unit,
		.duty_cycle = {data->duty_u, data->duty_v, data->duty_w},
		.p_tapa_taud = &data->tapa_inst,
		.p_taud_master_pwm = &data->master_pwm_inst,
		.p_taud_slave_pwm = {
			&data->slave_pwm_inst[0], &data->slave_pwm_inst[1],
			&data->slave_pwm_inst[2], &data->slave_pwm_inst[3],
			&data->slave_pwm_inst[4], &data->slave_pwm_inst[5],
		},
		.p_taud_master_oneshot = {
			&data->master_oneshot_inst[0], &data->master_oneshot_inst[1],
			&data->master_oneshot_inst[2],
		},
		.p_taud_slave_oneshot = {
			&data->slave_oneshot_inst[0], &data->slave_oneshot_inst[1],
			&data->slave_oneshot_inst[2],
		},
	};
	/* The type has read-only members, so it cannot be assigned directly. */
	memcpy(&data->taud3p_ext_cfg, &taud3p_ext_cfg, sizeof(taud3p_ext_cfg));
	data->taud3p_cfg = (three_phase_cfg_t){
		.p_extend = &data->taud3p_ext_cfg,
	};

	err = config->taud3p_api->open(&data->taud3p_ctrl, &data->taud3p_cfg);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: failed to open the timer driver: %d", dev->name, err);
		return -EIO;
	}

	return 0;
}

static int pwm_rh850_taud3p_init(const struct device *dev)
{
	const struct pwm_rh850_taud3p_config *config = dev->config;
	struct pwm_rh850_taud3p_data *data = dev->data;
	int ret;

#ifdef CONFIG_RENESAS_RH850_PIC
	if (!device_is_ready(config->pic_dev)) {
		LOG_ERR("%s: PIC device %s is not ready", dev->name, config->pic_dev->name);
		return -ENODEV;
	}
#endif

	if (data->deadtime_counts > (data->period_counts / 2U)) {
		LOG_ERR("%s: deadtime-counts %u exceeds period-counts/2 (%u)", dev->name,
			data->deadtime_counts, data->period_counts / 2U);
		return -EINVAL;
	}

	/* The TAUD unit is in module standby until its clock is enabled, its registers cannot be
	 * accessed before that.
	 */
	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("%s: clock device %s is not ready", dev->name, config->clock_dev->name);
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret < 0) {
		LOG_ERR("%s: module clock enable failed: %d", dev->name, ret);
		return ret;
	}

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("%s: pinctrl failed: %d", dev->name, ret);
		return ret;
	}

	ret = pwm_rh850_taud3p_taud_open(dev);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("%s: initialized (unit=%u, period=%u, deadtime=%u)", dev->name,
		config->taud_three_phase_unit, data->period_counts, data->deadtime_counts);

	return 0;
}

/* The HAL reports an invalid hardware condition while the previous update is still pending.
 * It is mapped to -EBUSY, the caller can retry after the next PWM period.
 */
static int pwm3p_fsp_err_to_errno(fsp_err_t err)
{
	switch (err) {
	case FSP_ERR_INVALID_HW_CONDITION:
		return -EBUSY;
	case FSP_ERR_INVALID_ARGUMENT:
		return -EINVAL;
	default:
		return -EIO;
	}
}

static uint32_t pwm3p_max_duty(uint32_t period, uint32_t deadtime)
{
	deadtime = MAX(deadtime, 1U);

	return (period > deadtime) ? period - deadtime : 0U;
}

static bool pwm3p_duty_is_valid(uint32_t period, uint32_t deadtime, uint32_t duty)
{
	return (duty <= pwm3p_max_duty(period, deadtime)) || (duty == period);
}

/* Keeps the duty ratio across a period change. */
static uint32_t pwm3p_duty_rescale(uint32_t duty, uint32_t old_period, uint32_t new_period,
				   uint32_t deadtime)
{
	uint32_t scaled;

	if (duty >= old_period) {
		return new_period;
	}

	scaled = (uint32_t)(((uint64_t)duty * new_period) / old_period);

	return MIN(scaled, pwm3p_max_duty(new_period, deadtime));
}

/* Writes the duty cycles of the three phases. The cached duty cycles are not modified. */
static int pwm3p_duty_apply(const struct device *dev, uint32_t duty_u, uint32_t duty_v,
			    uint32_t duty_w)
{
	struct pwm_rh850_taud3p_data *data = dev->data;
	timer_ctrl_t *master = (timer_ctrl_t *)&data->master_pwm_ctrl;
	fsp_err_t err;

	/* The duty cycle of a phase is set through its high-side channel, in raw counts. */
	err = g_timer_on_taud.dutyCycleSet(master, duty_u, pwm3p_slave_pwm_channels[0]);
	if (err == FSP_SUCCESS) {
		err = g_timer_on_taud.dutyCycleSet(master, duty_v, pwm3p_slave_pwm_channels[2]);
	}
	if (err == FSP_SUCCESS) {
		err = g_timer_on_taud.dutyCycleSet(master, duty_w, pwm3p_slave_pwm_channels[4]);
	}

	if (err != FSP_SUCCESS) {
		LOG_DBG("%s: duty cycle update failed: %d", dev->name, err);
		return pwm3p_fsp_err_to_errno(err);
	}

	return 0;
}

static int pwm3p_rh850_set_period_cycles(const struct device *dev, uint32_t period_cycles)
{
	struct pwm_rh850_taud3p_data *data = dev->data;
	uint32_t old_period = data->period_counts;
	fsp_err_t err;
	int ret;

	/* The period must be at least 2 counts. */
	if (period_cycles < 2U) {
		LOG_ERR("%s: period_cycles %u is too small (must be >= 2)", dev->name,
			period_cycles);
		return -EINVAL;
	}

	if (period_cycles > TAUD_PERIOD_COUNT_MAX_CDR_X2) {
		LOG_ERR("%s: period_cycles %u is too large (max %u)", dev->name, period_cycles,
			(unsigned int)TAUD_PERIOD_COUNT_MAX_CDR_X2);
		return -EINVAL;
	}

	/* The counter counts up and down, so an odd period would lose one count. */
	if ((period_cycles & 1U) != 0U) {
		LOG_ERR("%s: period_cycles %u must be even", dev->name, period_cycles);
		return -EINVAL;
	}

	/* The deadtime comes from devicetree and cannot be changed at runtime, the new period
	 * must still be able to hold it.
	 */
	if (data->deadtime_counts > (period_cycles / 2U)) {
		LOG_ERR("%s: period_cycles %u would violate deadtime-counts %u (max period/2 = %u)",
			dev->name, period_cycles, data->deadtime_counts, period_cycles / 2U);
		return -EINVAL;
	}

	/* Nothing to do if the period does not change. */
	if (period_cycles == old_period) {
		return 0;
	}

	err = g_timer_on_taud.periodSet((timer_ctrl_t *)&data->master_pwm_ctrl, period_cycles);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: period update failed: %d", dev->name, err);
		return pwm3p_fsp_err_to_errno(err);
	}

	data->master_pwm_cfg.period_counts = period_cycles;
	data->period_counts = period_cycles;

	/* The duty cycles are counts relative to the period, so they are rescaled to keep the
	 * same ratio. The cache is updated first so that it holds the intended state even if
	 * the write below fails.
	 */
	data->duty_u = pwm3p_duty_rescale(data->duty_u, old_period, period_cycles,
					  data->deadtime_counts);
	data->duty_v = pwm3p_duty_rescale(data->duty_v, old_period, period_cycles,
					  data->deadtime_counts);
	data->duty_w = pwm3p_duty_rescale(data->duty_w, old_period, period_cycles,
					  data->deadtime_counts);

	ret = pwm3p_duty_apply(dev, data->duty_u, data->duty_v, data->duty_w);
	if (ret < 0) {
		LOG_ERR("%s: period changed to %u but duty re-application failed (%d), call "
			"pwm3p_set_duty_cycles() again", dev->name, period_cycles, ret);
		return ret;
	}

	return 0;
}

static int pwm3p_rh850_get_period_cycles(const struct device *dev, uint32_t *period_cycles)
{
	struct pwm_rh850_taud3p_data *data = dev->data;

	*period_cycles = data->period_counts;

	return 0;
}

static int pwm3p_rh850_set_duty_cycles(const struct device *dev, uint32_t duty_u, uint32_t duty_v,
					uint32_t duty_w)
{
	struct pwm_rh850_taud3p_data *data = dev->data;
	int ret;

	if (!pwm3p_duty_is_valid(data->period_counts, data->deadtime_counts, duty_u) ||
	    !pwm3p_duty_is_valid(data->period_counts, data->deadtime_counts, duty_v) ||
	    !pwm3p_duty_is_valid(data->period_counts, data->deadtime_counts, duty_w)) {
		return -EINVAL;
	}

	ret = pwm3p_duty_apply(dev, duty_u, duty_v, duty_w);
	if (ret < 0) {
		return ret;
	}

	data->duty_u = duty_u;
	data->duty_v = duty_v;
	data->duty_w = duty_w;

	return 0;
}

static int pwm3p_rh850_enable_output(const struct device *dev, bool enable)
{
	const struct pwm_rh850_taud3p_config *config = dev->config;
	struct pwm_rh850_taud3p_data *data = dev->data;
	fsp_err_t err;

	if (enable) {
		err = config->taud3p_api->start(&data->taud3p_ctrl);
	} else {
		err = config->taud3p_api->stop(&data->taud3p_ctrl);
	}

	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: failed to %s the output: %d", dev->name, enable ? "enable" : "disable",
			err);
		return -EIO;
	}

	data->output_enabled = enable;

	return 0;
}

static int pwm3p_rh850_reset(const struct device *dev)
{
	const struct pwm_rh850_taud3p_config *config = dev->config;
	struct pwm_rh850_taud3p_data *data = dev->data;
	fsp_err_t err;

	if (!data->output_enabled) {
		LOG_ERR("%s: cannot reset counters while output is disabled", dev->name);
		return -EAGAIN;
	}

	err = config->taud3p_api->reset(&data->taud3p_ctrl);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: reset failed: %d", dev->name, err);
		return -EIO;
	}

	return 0;
}

static int pwm3p_rh850_get_cycles_per_sec(const struct device *dev, uint64_t *cycles)
{
	struct pwm_rh850_taud3p_data *data = dev->data;
	timer_info_t info;
	fsp_err_t err;

	err = g_timer_on_taud.infoGet((timer_ctrl_t *)&data->master_pwm_ctrl, &info);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: failed to get the clock rate: %d", dev->name, err);
		return -EIO;
	}

	*cycles = (uint64_t)info.clock_frequency;

	return 0;
}

static int pwm3p_rh850_set_callback(const struct device *dev, pwm3p_callback_t callback,
				     void *user_data)
{
	const struct pwm_rh850_taud3p_config *config = dev->config;
	struct pwm_rh850_taud3p_data *data = dev->data;
	fsp_err_t err;

	if (!config->has_cycle_end_irq) {
		LOG_ERR("%s: no 'interrupts' property, per-period callback is not supported",
			dev->name);
		return -ENOTSUP;
	}

	if (callback == NULL) {
		/* Stop the interrupt first so the callback is not called while it is cleared. */
		irq_disable(config->cycle_end_irq);
		data->app_callback = NULL;
		data->app_callback_user_data = NULL;

		return 0;
	}

	data->app_callback = callback;
	data->app_callback_user_data = user_data;

	/* The HAL driver always calls the shim, which forwards to the application callback. */
	err = config->taud3p_api->callbackSet(&data->taud3p_ctrl, pwm3p_taud_master_callback_shim,
					       (void *)dev, &data->callback_memory);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: failed to register the callback: %d", dev->name, err);
		data->app_callback = NULL;
		data->app_callback_user_data = NULL;

		return -EIO;
	}

	/* The interrupt is only enabled while a callback is registered. */
	irq_enable(config->cycle_end_irq);

	return 0;
}

static DEVICE_API(pwm3p, pwm3p_rh850_driver_api) = {
	.set_period_cycles = pwm3p_rh850_set_period_cycles,
	.get_period_cycles = pwm3p_rh850_get_period_cycles,
	.set_duty_cycles = pwm3p_rh850_set_duty_cycles,
	.enable_output = pwm3p_rh850_enable_output,
	.get_cycles_per_sec = pwm3p_rh850_get_cycles_per_sec,
	.reset = pwm3p_rh850_reset,
	.set_callback = pwm3p_rh850_set_callback,
};

/* Only compiled in when DT_INST_IRQ_HAS_IDX(n, 0) is true (see IF_ENABLED() below).
 * The HAL driver enables the interrupt when it is opened, but the interrupt is only needed
 * while an application callback is registered, so it is disabled again here and enabled by
 * pwm3p_set_callback().
 */
#define PWM3P_RH850_CYCLE_END_IRQ_CONFIG(n)                                                        \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), pwm3p_rh850_isr,            \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_disable(DT_INST_IRQN(n));                                                      \
	} while (0)

/* The PIC checks are only built when the PIC driver is enabled. */
#ifdef CONFIG_RENESAS_RH850_PIC
#define PWM3P_RH850_VALIDATE_PIC(n)                                                                \
	BUILD_ASSERT(DT_INST_NODE_HAS_PROP(n, pic),                                                \
		     "the 'pic' property is required when CONFIG_RENESAS_RH850_PIC is enabled");   \
	BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(DT_INST_PHANDLE(n, pic)),                             \
		     "the node referenced by the 'pic' property must be enabled");                 \
	BUILD_ASSERT(CONFIG_RENESAS_PIC_INIT_PRIORITY <= CONFIG_PWM3P_INIT_PRIORITY,               \
		     "the PIC must be initialized before PWM3P");
#else
#define PWM3P_RH850_VALIDATE_PIC(n)
#endif

/* Build-time validation of the devicetree values. */
#define PWM3P_RH850_VALIDATE(n)                                                                    \
	PWM3P_RH850_VALIDATE_PIC(n)                                                                \
	BUILD_ASSERT(DT_INST_PROP(n, taud_three_phase_unit) <                                      \
			     BSP_FEATURE_TAUD_THREE_PHASE_NUMBER_OF_UNIT,                          \
		     "taud-three-phase-unit is out of range for this MCU");                        \
	BUILD_ASSERT(DT_INST_PROP(n, period_counts) >= 2U &&                                       \
			     DT_INST_PROP(n, period_counts) <= TAUD_PERIOD_COUNT_MAX_CDR_X2,       \
		     "period-counts must be in 2..131072");                                        \
	BUILD_ASSERT((DT_INST_PROP(n, period_counts) % 2U) == 0U,                                  \
		     "period-counts must be even (the counter counts up and down)");               \
	BUILD_ASSERT(DT_INST_PROP(n, deadtime_counts) <= 0xFFFFU,                                  \
		     "deadtime-counts must be in 0..65535");                                       \
	BUILD_ASSERT(DT_INST_PROP(n, deadtime_counts) <= DT_INST_PROP(n, period_counts) / 2U,      \
		     "deadtime-counts must not exceed period-counts / 2")

#define PWM3P_RH850_INIT(n)                                                                        \
	PWM3P_RH850_VALIDATE(n);                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct pwm_rh850_taud3p_config pwm3p_rh850_cfg_##n = {                        \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)(uintptr_t)                                \
				DT_INST_CLOCKS_CELL(n, clk_id),                                    \
		IF_ENABLED(CONFIG_RENESAS_RH850_PIC,                                               \
			   (.pic_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, pic)),))                   \
		.taud3p_api = &g_taud_three_phase_on_taud_three_phase,                             \
		.taud_three_phase_unit = DT_INST_PROP(n, taud_three_phase_unit),                   \
		.operation_clk = DT_INST_PROP(n, operation_clk),                                   \
		.clk_div = DT_INST_PROP(n, clk_div),                                               \
		.has_cycle_end_irq = DT_INST_IRQ_HAS_IDX(n, 0),                                    \
		.cycle_end_irq = COND_CODE_1(DT_INST_IRQ_HAS_IDX(n, 0), (DT_INST_IRQN(n)),         \
					      (FSP_INVALID_VECTOR)),                               \
		.cycle_end_ipl = COND_CODE_1(DT_INST_IRQ_HAS_IDX(n, 0),                            \
					      (DT_INST_IRQ(n, priority)), (BSP_IRQ_DISABLED)),     \
	};                                                                                         \
	static struct pwm_rh850_taud3p_data pwm3p_rh850_data_##n = {                               \
		.period_counts = DT_INST_PROP(n, period_counts),                                   \
		.deadtime_counts = DT_INST_PROP(n, deadtime_counts),                               \
		.duty_u = DT_INST_PROP(n, period_counts) / 2U,                                     \
		.duty_v = DT_INST_PROP(n, period_counts) / 2U,                                     \
		.duty_w = DT_INST_PROP(n, period_counts) / 2U,                                     \
	};                                                                                         \
	static int pwm3p_rh850_init_##n(const struct device *dev)                                  \
	{                                                                                          \
		int ret = pwm_rh850_taud3p_init(dev);                                              \
		if (ret < 0) {                                                                     \
			return ret;                                                                \
		}                                                                                  \
		IF_ENABLED(DT_INST_IRQ_HAS_IDX(n, 0), (PWM3P_RH850_CYCLE_END_IRQ_CONFIG(n);))      \
		return 0;                                                                          \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(n, pwm3p_rh850_init_##n, NULL, &pwm3p_rh850_data_##n,                \
			      &pwm3p_rh850_cfg_##n, POST_KERNEL, CONFIG_PWM3P_INIT_PRIORITY,       \
			      &pwm3p_rh850_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM3P_RH850_INIT)
