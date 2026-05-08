/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_ostm_counter

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <r_ostm.h>

#define RH850_OSTM_TOP_VALUE UINT32_MAX
#define counter_rh850_ostm_clear_pending(irq)              R_BSP_IrqStatusClear(irq)
#define counter_rh850_ostm_set_pending(irq_reg)            WRITE_BIT(*(irq_reg), 12, 1)
#define counter_rh850_ostm_is_pending                      R_FSP_CurrentIrqGet()

void ostm_int_isr(void);
static int ostm_period_set(timer_ctrl_t * const p_ctrl, uint32_t period_counts);

struct counter_rh850_ostm_config {
	struct counter_config_info config_info;
	const timer_api_t *fsp_api;
};

struct counter_rh850_ostm_data {
	timer_cfg_t *fsp_cfg;
	ostm_instance_ctrl_t *fsp_ctrl;
	/* Top callback function */
	counter_top_callback_t top_cb;
	/* Alarm callback function */
	counter_alarm_callback_t alarm_cb;
	void *user_data;
	uint32_t clk_freq;
	struct k_spinlock lock;
	uint32_t guard_period;
	uint32_t top_val;
	bool is_started;
	bool is_periodic;

	volatile uint16_t *const ovf_trigger_reg;
};

static int counter_rh850_ostm_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_rh850_ostm_config *cfg = dev->config;
	struct counter_rh850_ostm_data *data = dev->data;
	timer_status_t timer_status;
	fsp_err_t err;

	err = cfg->fsp_api->statusGet(data->fsp_ctrl, &timer_status);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}
	*ticks = (uint32_t)timer_status.counter;

	return 0;
}

static void counter_rh850_ostm_irq_handler(timer_callback_args_t *p_args)
{
	const struct device *dev = p_args->p_context;
	struct counter_rh850_ostm_data *data = dev->data;
	counter_alarm_callback_t alarm_callback = data->alarm_cb;

	if (alarm_callback) {
		uint32_t now;

		if (counter_rh850_ostm_get_value(dev, &now) != 0) {
			return;
		}
		data->alarm_cb = NULL;
		alarm_callback(dev, 0, now, data->user_data);
	} else if (data->top_cb) {
		data->top_cb(dev, data->user_data);
	}
}

static int counter_rh850_ostm_init(const struct device *dev)
{
	struct counter_rh850_ostm_data *data = dev->data;
	const struct counter_rh850_ostm_config *cfg = dev->config;
	int err;

	data->top_val = data->fsp_cfg->period_counts;

	err = cfg->fsp_api->open(data->fsp_ctrl, data->fsp_cfg);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	data->is_periodic = false;

	return err;
}

static int counter_rh850_ostm_switch_timer_mode(const struct device *dev)
{
	const struct counter_rh850_ostm_config *cfg = dev->config;
	struct counter_rh850_ostm_data *data = dev->data;
	ostm_extended_cfg_t *fsp_cfg_extend = (ostm_extended_cfg_t *)data->fsp_cfg->p_extend;
	int err;

	if (data->is_periodic) {
		fsp_cfg_extend->ostm_mode = OSTM_TIMER_MODE_INTERVAL;
	} else {
		fsp_cfg_extend->ostm_mode = OSTM_TIMER_MODE_FREERUN;
	}

	err = cfg->fsp_api->close(data->fsp_ctrl);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	err = cfg->fsp_api->open(data->fsp_ctrl, data->fsp_cfg);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	err = cfg->fsp_api->start(data->fsp_ctrl);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	return err;
}

static int counter_rh850_ostm_start(const struct device *dev)
{
	struct counter_rh850_ostm_data *data = dev->data;
	k_spinlock_key_t key;
	int err;

	key = k_spin_lock(&data->lock);

	if (data->is_started) {
		k_spin_unlock(&data->lock, key);
		return -EALREADY;
	}

	if (data->is_periodic) {
		data->fsp_cfg->period_counts = data->top_val;
	}

	err = counter_rh850_ostm_switch_timer_mode(dev);
	if (err != FSP_SUCCESS) {
		k_spin_unlock(&data->lock, key);
		return -EIO;
	}

	counter_rh850_ostm_clear_pending(data->fsp_cfg->cycle_end_irq);
	data->is_started = true;
	if (data->top_cb) {
		irq_enable(data->fsp_cfg->cycle_end_irq);
	}

	k_spin_unlock(&data->lock, key);

	return err;
}

static int counter_rh850_ostm_stop(const struct device *dev)
{
	const struct counter_rh850_ostm_config *cfg = dev->config;
	struct counter_rh850_ostm_data *data = dev->data;
	k_spinlock_key_t key;
	int err;

	key = k_spin_lock(&data->lock);

	if (!data->is_started) {
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	/* Stop timer */
	err = cfg->fsp_api->stop(data->fsp_ctrl);
	if (err != FSP_SUCCESS) {
		k_spin_unlock(&data->lock, key);
		return -EIO;
	}

	/* Disable irq */
	irq_disable(data->fsp_cfg->cycle_end_irq);
	counter_rh850_ostm_clear_pending(data->fsp_cfg->cycle_end_irq);

	data->top_cb = NULL;
	data->alarm_cb = NULL;
	data->user_data = NULL;

	data->is_started = false;

	k_spin_unlock(&data->lock, key);

	return err;
}

static int counter_rh850_ostm_set_alarm(const struct device *dev, uint8_t chan,
				    const struct counter_alarm_cfg *alarm_cfg)
{
	const struct counter_rh850_ostm_config *cfg = dev->config;
	struct counter_rh850_ostm_data *data = dev->data;

	bool absolute;
	uint32_t val;
	k_spinlock_key_t key;
	bool irq_on_late;
	uint32_t max_rel_val;
	uint32_t now, diff;
	uint32_t read_again;
	int err;

	if (chan != 0) {
		return -EINVAL;
	}

	if (!alarm_cfg) {
		return -EINVAL;
	}

	absolute = alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE;
	val = alarm_cfg->ticks;

	/* Alarm callback is mandatory */
	if (!alarm_cfg->callback) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	if (!data->is_started) {
		k_spin_unlock(&data->lock, key);
		return -EINVAL;
	}

	/* Alarm_cb need equal NULL before */
	if (data->alarm_cb) {
		k_spin_unlock(&data->lock, key);
		return -EBUSY;
	}

	/* Timer is currently in interval mode */
	if (data->is_periodic) {
		/* Return error because val exceeded the limit set alarm */
		if (val > data->fsp_cfg->period_counts) {
			k_spin_unlock(&data->lock, key);
			return -EINVAL;
		}

		/* Restore free running mode */
		irq_disable(data->fsp_cfg->cycle_end_irq);
		data->top_cb = NULL;
		data->top_val = RH850_OSTM_TOP_VALUE;
		data->is_periodic = false;
		data->fsp_cfg->period_counts = data->top_val;

		err = counter_rh850_ostm_switch_timer_mode(dev);
		if (err != FSP_SUCCESS) {
			k_spin_unlock(&data->lock, key);
			return -EIO;
		}
	}

	err = counter_rh850_ostm_get_value(dev, &now);
	if (err != 0) {
		k_spin_unlock(&data->lock, key);
		return -EIO;
	}

	data->alarm_cb = alarm_cfg->callback;
	data->user_data = alarm_cfg->user_data;

	if (absolute) {
		max_rel_val = RH850_OSTM_TOP_VALUE - data->guard_period;
		irq_on_late = alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE;
	} else {
		/* If relative value is smaller than half of the counter range
		 * it is assumed that there is a risk of setting value too late
		 * and late detection algorithm must be applied. When late
		 * setting is detected, interrupt shall be triggered for
		 * immediate expiration of the timer. Detection is performed
		 * by limiting relative distance between CC and counter.
		 *
		 * Note that half of counter range is an arbitrary value.
		 */
		irq_on_late = val < (RH850_OSTM_TOP_VALUE / 2U);
		/* Limit max to detect short relative being set too late. */
		max_rel_val = irq_on_late ? RH850_OSTM_TOP_VALUE / 2U : RH850_OSTM_TOP_VALUE;
		val = (now + val) & RH850_OSTM_TOP_VALUE;
	}

	/* Set new period */
	data->fsp_cfg->period_counts = val;
	err = ostm_period_set(data->fsp_ctrl, data->fsp_cfg->period_counts);
	if (err != FSP_SUCCESS) {
		k_spin_unlock(&data->lock, key);
		return -EIO;
	}

	err = counter_rh850_ostm_get_value(dev, &read_again);
	if (err != 0) {
		k_spin_unlock(&data->lock, key);
		return -EIO;
	}

	if (val >= read_again) {
		diff = (val - read_again);
	} else {
		diff = val + (RH850_OSTM_TOP_VALUE - read_again);
	}

	if (diff > max_rel_val) {
		if (absolute) {
			err = -ETIME;
		}

		/* Interrupt is triggered always for relative alarm and
		 * for absolute depending on the flag.
		 */
		if (irq_on_late) {
			irq_enable(data->fsp_cfg->cycle_end_irq);
			counter_rh850_ostm_set_pending(data->ovf_trigger_reg);
		} else {
			data->alarm_cb = NULL;
		}
	} else {
		if (diff == 0) {
			/* RELOAD value could be set just in time for interrupt
			 * trigger or too late. In any case time is interrupt
			 * should be triggered. No need to enable interrupt
			 * on TIMER just make sure interrupt is pending.
			 */
			irq_enable(data->fsp_cfg->cycle_end_irq);
			counter_rh850_ostm_set_pending(data->ovf_trigger_reg);
		} else {
			counter_rh850_ostm_clear_pending(data->fsp_cfg->cycle_end_irq);
			irq_enable(data->fsp_cfg->cycle_end_irq);
		}
	}

	k_spin_unlock(&data->lock, key);

	return err;
}

static int counter_rh850_ostm_cancel_alarm(const struct device *dev, uint8_t chan)
{
	struct counter_rh850_ostm_data *data = dev->data;
	k_spinlock_key_t key;

	ARG_UNUSED(chan);

	key = k_spin_lock(&data->lock);

	if (!data->is_started) {
		k_spin_unlock(&data->lock, key);
		return -EINVAL;
	}

	if (!data->alarm_cb) {
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	irq_disable(data->fsp_cfg->cycle_end_irq);
	counter_rh850_ostm_clear_pending(data->fsp_cfg->cycle_end_irq);
	data->alarm_cb = NULL;
	data->user_data = NULL;

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int counter_rh850_ostm_set_top_value(const struct device *dev,
					const struct counter_top_cfg *top_cfg)
{
	const struct counter_rh850_ostm_config *cfg = dev->config;
	struct counter_rh850_ostm_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t cur_tick;
	bool reset;
	int err = 0;

	if (!top_cfg) {
		return -EINVAL;
	}

	/* -EBUSY if any alarm is active */
	if (data->alarm_cb) {
		return -EBUSY;
	}

	key = k_spin_lock(&data->lock);

	if (!data->is_periodic && top_cfg->ticks == RH850_OSTM_TOP_VALUE) {
		goto exit_unlock;
	}

	if (top_cfg->ticks == RH850_OSTM_TOP_VALUE) {
		/* Restore free running mode */
		irq_disable(data->fsp_cfg->cycle_end_irq);
		counter_rh850_ostm_clear_pending(data->fsp_cfg->cycle_end_irq);
		data->top_cb = NULL;
		data->user_data = NULL;
		data->top_val = RH850_OSTM_TOP_VALUE;
		data->is_periodic = false;

		if (data->is_started) {
			err = counter_rh850_ostm_switch_timer_mode(dev);
			if (err != FSP_SUCCESS) {
				k_spin_unlock(&data->lock, key);
				return -EIO;
			}
			counter_rh850_ostm_clear_pending(data->fsp_cfg->cycle_end_irq);
		}
		goto exit_unlock;
	}

	data->top_cb = top_cfg->callback;
	data->user_data = top_cfg->user_data;
	data->top_val = top_cfg->ticks;

	if (!data->is_started) {
		data->is_periodic = true;
		goto exit_unlock;
	}

	if (!data->is_periodic) {
		/* Switch to interval mode first time, restart timer */
		err = cfg->fsp_api->stop(data->fsp_ctrl);
		if (err != FSP_SUCCESS) {
			k_spin_unlock(&data->lock, key);
			return -EIO;
		}

		irq_disable(data->fsp_cfg->cycle_end_irq);
		data->is_periodic = true;
		data->fsp_cfg->period_counts = data->top_val;

		err = counter_rh850_ostm_switch_timer_mode(dev);
		if (err != FSP_SUCCESS) {
			k_spin_unlock(&data->lock, key);
			return -EIO;
		}

		if (data->top_cb) {
			counter_rh850_ostm_clear_pending(data->fsp_cfg->cycle_end_irq);
			irq_enable(data->fsp_cfg->cycle_end_irq);
		}
		goto exit_unlock;
	}

	if (!data->top_cb) {
		/* New top cfg is without callback - stop IRQs */
		irq_disable(data->fsp_cfg->cycle_end_irq);
		counter_rh850_ostm_clear_pending(data->fsp_cfg->cycle_end_irq);
	}
	/* Timer already in interval mode - only change top value */
	data->fsp_cfg->period_counts = data->top_val;
	err = cfg->fsp_api->periodSet(data->fsp_ctrl, data->fsp_cfg->period_counts);
	if (err != FSP_SUCCESS) {
		k_spin_unlock(&data->lock, key);
		return -EIO;
	}

	/* Check if counter reset is required */
	reset = false;
	if (top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET) {
		/* Don't reset counter */
		err = counter_rh850_ostm_get_value(dev, &cur_tick);
		if (err != 0) {
			k_spin_unlock(&data->lock, key);
			return -EIO;
		}

		if (cur_tick >= data->top_val) {
			err = -ETIME;
			if (top_cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) {
				/* Reset counter if current is late */
				reset = true;
			}
		}
	} else {
		reset = true;
	}

	if (reset) {
		err = cfg->fsp_api->reset(data->fsp_ctrl);
		if (err != FSP_SUCCESS) {
			k_spin_unlock(&data->lock, key);
			return -EIO;
		}
	}

exit_unlock:
	k_spin_unlock(&data->lock, key);

	return err;
}

static uint32_t counter_rh850_ostm_get_pending_int(const struct device *dev)
{
	struct counter_rh850_ostm_data *data = dev->data;

	return (uint32_t) counter_rh850_ostm_is_pending;
}

static uint32_t counter_rh850_ostm_get_top_value(const struct device *dev)
{
	struct counter_rh850_ostm_data *data = dev->data;
	const struct counter_rh850_ostm_config *cfg = dev->config;
	uint32_t top_val = RH850_OSTM_TOP_VALUE;

	if (data->is_periodic) {
		timer_info_t info;
		int err;

		err = cfg->fsp_api->infoGet(data->fsp_ctrl, &info);
		if (err != FSP_SUCCESS) {
			return 0;
		}
		top_val = info.period_counts;
	}

	return top_val;
}
static uint32_t counter_rh850_ostm_get_guard_period(const struct device *dev, uint32_t flags)
{
	struct counter_rh850_ostm_data *data = dev->data;

	ARG_UNUSED(flags);

	return data->guard_period;
}

static int counter_rh850_ostm_set_guard_period(const struct device *dev,
						uint32_t guard, uint32_t flags)
{
	struct counter_rh850_ostm_data *data = dev->data;

	ARG_UNUSED(flags);
	if (counter_rh850_ostm_get_top_value(dev) < guard) {
		return -EINVAL;
	}

	data->guard_period = guard;

	return 0;
}

static uint32_t counter_rh850_ostm_get_freq(const struct device *dev)
{
	struct counter_rh850_ostm_data *data = dev->data;
	const struct counter_rh850_ostm_config *cfg = dev->config;
	timer_info_t info;
	int err;

	err = cfg->fsp_api->infoGet(data->fsp_ctrl, &info);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	return info.clock_frequency;
}

static int ostm_period_set(timer_ctrl_t * const p_ctrl, uint32_t period_counts)
{
	ostm_instance_ctrl_t *p_instance_ctrl = (ostm_instance_ctrl_t *) p_ctrl;
	R_OSTMn_Type *reg = p_instance_ctrl->p_reg;

	p_instance_ctrl->period = period_counts;
	reg->OSTMnCMP = period_counts;

	return 0;
}

static DEVICE_API(counter, counter_rh850_ostm_driver_api) = {
	.start = counter_rh850_ostm_start,
	.stop = counter_rh850_ostm_stop,
	.get_value = counter_rh850_ostm_get_value,
	.set_alarm = counter_rh850_ostm_set_alarm,
	.cancel_alarm = counter_rh850_ostm_cancel_alarm,
	.set_top_value = counter_rh850_ostm_set_top_value,
	.get_pending_int = counter_rh850_ostm_get_pending_int,
	.get_top_value = counter_rh850_ostm_get_top_value,
	.get_guard_period = counter_rh850_ostm_get_guard_period,
	.set_guard_period = counter_rh850_ostm_set_guard_period,
	.get_freq = counter_rh850_ostm_get_freq,
};

void counter_rh850_ostm_ovf_isr(const struct device *dev)
{
	struct counter_rh850_ostm_data *data = dev->data;

	ostm_int_isr();
}


#define RH850_OSTM(idx) DT_INST_PARENT(idx)

#define RH850_OSTM_GET_IRQ_FLAGS(idx, irq_name) 0

#define COUNTER_RH850_OSTM_INIT(inst)                                                      \
	static ostm_instance_ctrl_t g_timer##inst##_ctrl;                                      \
	static ostm_extended_cfg_t g_timer##inst##_extend = {                                  \
		.generate_interrupt_when_starts = OSTM_GIWS_TYPE_DISABLED,                         \
		.ostm_mode = OSTM_TIMER_MODE_FREERUN,                                \
		.counter_load_at_start = OSTM_COUNTER_START_RELOAD_ENABLE,       \
		.output_setting = OSTM_OUTPUT_SOFTWARE_CONTROL_MODE,                \
	};                                                                                     \
	static timer_cfg_t g_timer##inst##_cfg = {                                             \
		.mode = TIMER_MODE_PERIODIC,                                                       \
		.period_counts = (uint32_t)RH850_OSTM_TOP_VALUE,                                   \
		.channel = DT_PROP(RH850_OSTM(inst), channel),                                     \
		.p_callback = counter_rh850_ostm_irq_handler,                                      \
		.p_context = DEVICE_DT_GET(DT_DRV_INST(inst)),                                     \
		.p_extend = &g_timer##inst##_extend,                                               \
		.cycle_end_ipl = DT_IRQ_BY_NAME(RH850_OSTM(inst), overflow, priority),             \
		.cycle_end_irq = DT_IRQ_BY_NAME(RH850_OSTM(inst), overflow, irq),                  \
	};                                                                                     \
	static const struct counter_rh850_ostm_config counter_rh850_ostm_config_##inst = {     \
		.config_info =                                                                     \
			{                                                                     \
				.max_top_value = RH850_OSTM_TOP_VALUE,                            \
				.flags = COUNTER_CONFIG_INFO_COUNT_UP,                            \
				.channels = 1,                                                    \
			},                                                                    \
		.fsp_api = &g_timer_on_ostm,                                              \
	};                                                                            \
	static struct counter_rh850_ostm_data counter_rh850_ostm_data_##inst = {      \
		.fsp_cfg = &g_timer##inst##_cfg,         \
		.fsp_ctrl = (timer_ctrl_t *)&g_timer##inst##_ctrl,                        \
		.ovf_trigger_reg = (uint16_t volatile *) (DT_REG_ADDR_BY_NAME(	          \
		DT_IRQ_INTC(DT_INST_PARENT(inst)), EIC)				                   \
		+ 2*DT_IRQ_BY_NAME(DT_INST_PARENT(inst), overflow, irq)),};               \
	static int counter_rh850_ostm_init_##inst(const struct device *dev)           \
	{                                                                             \
		IRQ_CONNECT(DT_IRQ_BY_NAME(RH850_OSTM(inst), overflow, irq),              \
			    DT_IRQ_BY_NAME(RH850_OSTM(inst), overflow, priority),             \
			    counter_rh850_ostm_ovf_isr, DEVICE_DT_INST_GET(inst),             \
			    RH850_OSTM_GET_IRQ_FLAGS(inst, overflow));                        \
		return counter_rh850_ostm_init(dev);                                      \
	}                                                                             \
	DEVICE_DT_INST_DEFINE(inst, counter_rh850_ostm_init_##inst, NULL,             \
				  &counter_rh850_ostm_data_##inst,                                \
			      &counter_rh850_ostm_config_##inst, PRE_KERNEL_1,                \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_rh850_ostm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_RH850_OSTM_INIT)
