/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_tmr_counter

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/pinctrl.h>

#include <r_tmr.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(renesas_rx_tmr_counter, CONFIG_COUNTER_LOG_LEVEL);

struct counter_renesas_rx_tmr_config {
	struct counter_config_info info;
	void (*irq_config_func)(void);
};

struct counter_renesas_rx_tmr_data {
	timer_cfg_t tmr_cfg;
	tmr_instance_ctrl_t tmr_ctrl;
	tmr_extended_cfg_t tmr_extend_cfg;
	uint32_t guard_period;
	counter_alarm_callback_t alarm_cb;
	void *alarm_data;
	counter_top_callback_t top_cb;
	void *top_data;
	struct k_spinlock lock;
};

static void counter_renesas_rx_tmr_cmp_isr(const struct device *dev);

static inline int renesas_rx_tmr_alarm_set_value(const struct device *dev, uint32_t val)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;

	data->tmr_ctrl.p_reg->TCORB = (uint16_t)val;

	return 0;
}

static inline k_spinlock_key_t counter_renesas_rx_tmr_lock(const struct device *dev)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;

	return k_spin_lock(&data->lock);
}

static inline void counter_renesas_rx_tmr_unlock(const struct device *dev, k_spinlock_key_t key)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;

	k_spin_unlock(&data->lock, key);
}

static int counter_renesas_rx_tmr_start(const struct device *dev)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;
	fsp_err_t err;

	err = R_TMR_Start(&data->tmr_ctrl);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Counter start failed");
		return -EIO;
	}

	return 0;
}

static int counter_renesas_rx_tmr_stop(const struct device *dev)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;
	fsp_err_t err;

	err = R_TMR_Stop(&data->tmr_ctrl);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Counter stop failed");
		return -EIO;
	}

	return 0;
}

static int counter_renesas_rx_tmr_get_value(const struct device *dev, uint32_t *ticks)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;
	timer_status_t status;
	fsp_err_t err;

	err = R_TMR_StatusGet(&data->tmr_ctrl, &status);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	*ticks = status.counter;

	return 0;
}

static uint32_t counter_renesas_rx_tmr_get_top_value(const struct device *dev)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;

	return (uint32_t)data->tmr_ctrl.p_reg->TCORA;
}

static int counter_renesas_rx_tmr_set_top_value(const struct device *dev,
						const struct counter_top_cfg *cfg)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t now;
	bool reset = false;
	fsp_err_t err;
	int ret = 0;

	if (cfg->ticks == 0 || cfg->ticks > UINT16_MAX) {
		return -EINVAL;
	}

	if (data->alarm_cb != NULL) {
		return -EBUSY;
	}

	key = counter_renesas_rx_tmr_lock(dev);

	data->top_cb = cfg->callback;
	data->top_data = cfg->user_data;

	/* Write new TOP directly to TCORA. */
	data->tmr_ctrl.p_reg->TCORA = (uint16_t)(cfg->ticks);

	/* Enable CMIA interrupt if top callback is provided */
	if (cfg->callback != NULL) {
		data->tmr_ctrl.p_reg->TCR_b[data->tmr_ctrl.index].CMIEA = 1U;
	} else {
		data->tmr_ctrl.p_reg->TCR_b[data->tmr_ctrl.index].CMIEA = 0U;
	}

	if (!(cfg->flags & COUNTER_TOP_CFG_DONT_RESET)) {
		reset = true;
	} else if ((cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) != 0) {
		ret = counter_renesas_rx_tmr_get_value(dev, &now);
		if (ret != 0) {
			goto out;
		}

		if (now >= cfg->ticks) {
			reset = true;
			ret = -ETIME;
		}
	} else {
		;
	}

	if (reset) {
		err = R_TMR_Reset(&data->tmr_ctrl);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Counter reset failed");
			ret = -EIO;
			goto out;
		}
	}
out:
	counter_renesas_rx_tmr_unlock(dev, key);
	return ret;
}

static uint32_t ticks_add(uint32_t val1, uint32_t val2, uint32_t top)
{
	uint32_t to_top;

	if (likely(IS_BIT_MASK(top))) {
		return (val1 + val2) & top;
	}

	to_top = top - val1;

	return (val2 <= to_top) ? val1 + val2 : val2 - to_top;
}

static inline uint32_t ticks_sub(uint32_t val, uint32_t old, uint32_t top)
{
	if (likely(IS_BIT_MASK(top))) {
		return (val - old) & top;
	}

	/* if top is not 2^n-1 */
	return (val >= old) ? (val - old) : (val + top + 1 - old);
}

static int renesas_rx_tmr_abs_alarm_set(const struct device *dev, uint32_t val, uint32_t top,
					bool irq_on_late)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;
	uint32_t max_val;
	uint32_t read_again;
	uint32_t diff;
	int ret;

	ret = renesas_rx_tmr_alarm_set_value(dev, val);
	if (ret != 0) {
		LOG_ERR("Alarm: set value failed, %d", ret);
		return ret;
	}

	ret = counter_renesas_rx_tmr_get_value(dev, &read_again);
	if (ret != 0) {
		LOG_ERR("Alarm: get value failed, %d", ret);
		return ret;
	}

	max_val = top - data->guard_period;
	diff = ticks_sub(val, read_again, top);

	if (diff > max_val || diff == 0) {
		data->alarm_cb = NULL;
		ret = -ETIME;
	}

	return ret;
}

static int renesas_rx_tmr_rel_alarm_set(const struct device *dev, uint32_t val, uint32_t top,
					bool irq_on_late)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;
	uint32_t max_rel_val = irq_on_late ? (top / 2) : top;
	uint32_t diff;
	uint32_t now;
	int ret;

	ret = counter_renesas_rx_tmr_get_value(dev, &now);
	if (ret != 0) {
		return ret;
	}

	val = ticks_add(now, val, top);

	ret = renesas_rx_tmr_alarm_set_value(dev, val);
	if (ret != 0) {
		return ret;
	}

	ret = counter_renesas_rx_tmr_get_value(dev, &now);
	if (ret != 0) {
		return ret;
	}

	diff = ticks_sub(val, now, top);

	if (diff > max_rel_val || diff == 0) {
		if (irq_on_late) {
			data->alarm_cb = NULL;
			data->alarm_data = NULL;
			ret = -ENOTSUP;
		} else {
			data->alarm_cb = NULL;
		}
	}

	return ret;
}

static int counter_renesas_rx_tmr_set_alarm(const struct device *dev, uint8_t chan,
					    const struct counter_alarm_cfg *alarm_cfg)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;
	const uint32_t top = counter_renesas_rx_tmr_get_top_value(dev);
	const bool absolute = (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) != 0;
	const bool irq_on_late =
		absolute ? (alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE) != 0
			 : alarm_cfg->ticks < (top / 2);
	k_spinlock_key_t key;
	int ret = 0;

	if (chan != 0) {
		LOG_ERR("Channel invalid");
		ret = -EINVAL;
		goto out;
	}

	if (alarm_cfg->ticks > top) {
		LOG_ERR("Alarm value exceeds the timer's maximum count");
		ret = -EINVAL;
		goto out;
	}

	if (data->alarm_cb != NULL) {
		ret = -EBUSY;
		goto out;
	}

	if (absolute && ((alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE) != 0)) {
		ret = -ENOTSUP;
		goto out;
	}

	key = counter_renesas_rx_tmr_lock(dev);

	data->alarm_cb = alarm_cfg->callback;
	data->alarm_data = alarm_cfg->user_data;

	if (absolute) {
		ret = renesas_rx_tmr_abs_alarm_set(dev, alarm_cfg->ticks, top, irq_on_late);
	} else {
		ret = renesas_rx_tmr_rel_alarm_set(dev, alarm_cfg->ticks, top, irq_on_late);
	}

	if (ret != 0) {
		data->tmr_ctrl.p_reg->TCR_b[data->tmr_ctrl.index].CMIEB = 0U;
	} else {
		data->tmr_ctrl.p_reg->TCR_b[data->tmr_ctrl.index].CMIEB = 1U;
	}

	counter_renesas_rx_tmr_unlock(dev, key);
out:
	return ret;
}

static int counter_renesas_rx_tmr_cancel_alarm(const struct device *dev, uint8_t chan)
{
	k_spinlock_key_t key = counter_renesas_rx_tmr_lock(dev);
	struct counter_renesas_rx_tmr_data *data = dev->data;

	if (data->tmr_cfg.cycle_end_irq == BSP_IRQ_DISABLED) {
		counter_renesas_rx_tmr_unlock(dev, key);
		return -ENOTSUP;
	}

	data->tmr_ctrl.p_reg->TCR_b[data->tmr_ctrl.index].CMIEB = 0U;

	data->alarm_cb = NULL;
	data->alarm_data = NULL;

	counter_renesas_rx_tmr_unlock(dev, key);
	return 0;
}

static uint32_t counter_renesas_rx_tmr_get_guard_period(const struct device *dev, uint32_t flags)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;

	return data->guard_period;
}

static int counter_renesas_rx_tmr_set_guard_period(const struct device *dev, uint32_t guard,
						   uint32_t flags)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;
	k_spinlock_key_t key = counter_renesas_rx_tmr_lock(dev);
	int ret = 0;
	uint32_t top = counter_renesas_rx_tmr_get_top_value(dev);

	if (top < guard) {
		ret = -EINVAL;
		goto out;
	}

	data->guard_period = guard;
out:
	counter_renesas_rx_tmr_unlock(dev, key);
	return ret;
}

static uint32_t counter_renesas_rx_tmr_get_pending_int(const struct device *dev)
{
	return 0;
}

static uint32_t counter_renesas_rx_tmr_get_freq(const struct device *dev)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;

	uint32_t clock_freq_hz = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKB);

	clock_freq_hz >>= data->tmr_cfg.source_div;

	return (clock_freq_hz);
}

static int counter_renesas_rx_tmr_init(const struct device *dev)
{
	const struct counter_renesas_rx_tmr_config *cfg = dev->config;
	struct counter_renesas_rx_tmr_data *data = dev->data;
	fsp_err_t err;

	err = R_TMR_Open(&data->tmr_ctrl, &data->tmr_cfg);
	if (err != FSP_SUCCESS) {
		LOG_ERR("initialization: open failed with code: %d", err);
		return -EIO;
	}

	data->tmr_ctrl.p_reg->TCR_b[data->tmr_ctrl.index].CMIEA = 0U;

	cfg->irq_config_func();

	return 0;
}

static void counter_renesas_rx_tmr_cmp_isr(const struct device *dev)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;
	counter_alarm_callback_t cb = data->alarm_cb;
	void *usr_data = data->alarm_data;
	uint32_t now;

	data->tmr_ctrl.p_reg->TCR_b[data->tmr_ctrl.index].CMIEB = 0U;

	if (cb != NULL) {
		data->alarm_cb = NULL;
		data->alarm_data = NULL;

		if (counter_renesas_rx_tmr_get_value(dev, &now) != 0) {
			LOG_ERR("Error in counter alarm");
			return;
		}

		cb(dev, 0, now, usr_data);
	}
}

static void counter_renesas_rx_tmr_top_isr(const struct device *dev)
{
	struct counter_renesas_rx_tmr_data *data = dev->data;
	counter_top_callback_t cb = data->top_cb;

	if (cb != NULL) {
		cb(dev, data->top_data);
	}
}

static DEVICE_API(counter, tmr_renesas_rx_driver_api) = {
	.start = counter_renesas_rx_tmr_start,
	.stop = counter_renesas_rx_tmr_stop,
	.get_value = counter_renesas_rx_tmr_get_value,
	.set_alarm = counter_renesas_rx_tmr_set_alarm,
	.cancel_alarm = counter_renesas_rx_tmr_cancel_alarm,
	.set_top_value = counter_renesas_rx_tmr_set_top_value,
	.get_pending_int = counter_renesas_rx_tmr_get_pending_int,
	.get_top_value = counter_renesas_rx_tmr_get_top_value,
	.get_freq = counter_renesas_rx_tmr_get_freq,
	.get_guard_period = counter_renesas_rx_tmr_get_guard_period,
	.set_guard_period = counter_renesas_rx_tmr_set_guard_period,
};

#define TIMER(idx) DT_INST_PARENT(idx)

#define ICU_EVENT_TMR_CMIA(channel) CONCAT(ICU_EVENT_TMR, channel, _COMPARE_A)
#define ICU_EVENT_TMR_CMIB(channel) CONCAT(ICU_EVENT_TMR, channel, _COMPARE_B)

#define COUNTER_TMR_IRQ_INIT(index)                                                                \
	do {                                                                                       \
                                                                                                   \
		const int cmia_irq = DT_IRQ_BY_NAME(TIMER(index), cmia, irq);                      \
		if (128 <= cmia_irq && cmia_irq < 144) {                                           \
			R_ICU->SLIXR[cmia_irq - 128].SLIXR =                                       \
				ICU_EVENT_TMR_CMIA(DT_PROP(TIMER(index), channel));                \
		} else if (144 <= cmia_irq) {                                                      \
			R_ICU->SLIR[cmia_irq - 144].SLIR =                                         \
				ICU_EVENT_TMR_CMIA(DT_PROP(TIMER(index), channel));                \
		}                                                                                  \
		IRQ_CONNECT(cmia_irq, DT_IRQ_BY_NAME(TIMER(index), cmia, priority),                \
			    counter_renesas_rx_tmr_top_isr, DEVICE_DT_INST_GET(index), 0);         \
		irq_enable(cmia_irq);                                                              \
                                                                                                   \
		const int cmib_irq = DT_IRQ_BY_NAME(TIMER(index), cmib, irq);                      \
		if (128 <= cmib_irq && cmib_irq < 144) {                                           \
			R_ICU->SLIXR[cmib_irq - 128].SLIXR =                                       \
				ICU_EVENT_TMR_CMIB(DT_PROP(TIMER(index), channel));                \
		} else if (144 <= cmib_irq) {                                                      \
			R_ICU->SLIR[cmib_irq - 144].SLIR =                                         \
				ICU_EVENT_TMR_CMIB(DT_PROP(TIMER(index), channel));                \
		}                                                                                  \
		IRQ_CONNECT(cmib_irq, DT_IRQ_BY_NAME(TIMER(index), cmib, priority),                \
			    counter_renesas_rx_tmr_cmp_isr, DEVICE_DT_INST_GET(index), 0);         \
		irq_enable(cmib_irq);                                                              \
	} while (0)

#define COUNTER_TMR_DEVICE_INIT(inst)                                                              \
	static void counter_renesas_rx_tmr##inst##_irq_config_func(void)                           \
	{                                                                                          \
		COUNTER_TMR_IRQ_INIT(inst);                                                        \
	}                                                                                          \
                                                                                                   \
	struct counter_renesas_rx_tmr_config counter_renesas_rx_tmr_config##inst = {               \
		.info.max_top_value = UINT16_MAX,                                                  \
		.info.flags = COUNTER_CONFIG_INFO_COUNT_UP,                                        \
		.info.channels = 1,                                                                \
		.irq_config_func = counter_renesas_rx_tmr##inst##_irq_config_func,                 \
	};                                                                                         \
                                                                                                   \
	static struct counter_renesas_rx_tmr_data counter_renesas_rx_tmr_data##inst = {            \
		.tmr_cfg =                                                                         \
			{                                                                          \
				.mode = TIMER_MODE_PERIODIC,                                       \
				.period_counts = (UINT16_MAX + 1),                                 \
				.source_div = DT_PROP(TIMER(inst), renesas_prescaler),             \
				.channel = DT_PROP(TIMER(inst), channel),                          \
				.cycle_end_irq = DT_IRQ_BY_NAME(TIMER(inst), cmia, irq),           \
				.cycle_end_ipl = DT_IRQ_BY_NAME(TIMER(inst), cmia, priority),      \
				.p_extend = &counter_renesas_rx_tmr_data##inst.tmr_extend_cfg,     \
			},                                                                         \
		.tmr_extend_cfg =                                                                  \
			{                                                                          \
				.counter_size = TMR_COUNTER_SIZE_16_BIT,                           \
				.adc_request_enable = TMR_ADC_TRIGGER_DISABLE,                     \
				.output_compare_ipl = BSP_IRQ_DISABLED,                            \
				.output_compare_irq = FSP_INVALID_VECTOR,                          \
			},                                                                         \
		.guard_period = 0,                                                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, counter_renesas_rx_tmr_init, NULL,                             \
			      &counter_renesas_rx_tmr_data##inst,                                  \
			      &counter_renesas_rx_tmr_config##inst, POST_KERNEL,                   \
			      CONFIG_COUNTER_INIT_PRIORITY, &tmr_renesas_rx_driver_api);

DT_INST_FOREACH_STATUS_OKAY(COUNTER_TMR_DEVICE_INIT)
