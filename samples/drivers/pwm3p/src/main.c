/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/pwm3p.h>
#include <zephyr/sys/printk.h>

#define PWM3P_NODE DT_NODELABEL(pwm3p0)

/* Duty sweeps between 25% and 75% of the period, 5% per step, once per second. */
#define DUTY_MIN_PERCENT  25
#define DUTY_MAX_PERCENT  75
#define DUTY_STEP_PERCENT 5

static atomic_t cycle_count;

static void pwm3p_cycle_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	atomic_inc(&cycle_count);
}

int main(void)
{
	const struct device *pwm3p = DEVICE_DT_GET(PWM3P_NODE);
	uint32_t period_cycles;
	uint64_t cycles_per_sec;
	int duty_percent = DUTY_MIN_PERCENT;
	int step = DUTY_STEP_PERCENT;
	int ret;

	if (!device_is_ready(pwm3p)) {
		printk("PWM3P device is not ready\n");
		return 0;
	}

	ret = pwm3p_get_period_cycles(pwm3p, &period_cycles);
	if (ret < 0) {
		printk("pwm3p_get_period_cycles failed: %d\n", ret);
		return 0;
	}

	ret = pwm3p_get_cycles_per_sec(pwm3p, &cycles_per_sec);
	if (ret < 0) {
		printk("pwm3p_get_cycles_per_sec failed: %d\n", ret);
		return 0;
	}

	printk("PWM3P ready: period=%u counts, counter clock=%llu Hz (PWM freq=%llu Hz)\n",
	       period_cycles, cycles_per_sec, (uint64_t)cycles_per_sec / period_cycles);

	/* Optional: a device may not expose a cycle-end interrupt (no 'interrupts' property
	 * in its devicetree node), in which case this simply reports -ENOTSUP.
	 */
	ret = pwm3p_set_callback(pwm3p, pwm3p_cycle_callback, NULL);
	if (ret == 0) {
		printk("Cycle-end callback registered\n");
	} else if (ret == -ENOTSUP) {
		printk("Cycle-end callback not supported on this instance\n");
	} else {
		printk("pwm3p_set_callback failed: %d\n", ret);
	}

	ret = pwm3p_set_duty_cycles(pwm3p, period_cycles / 2U, period_cycles / 2U,
				     period_cycles / 2U);
	if (ret < 0) {
		printk("pwm3p_set_duty_cycles failed: %d\n", ret);
		return 0;
	}

	ret = pwm3p_enable_output(pwm3p, true);
	if (ret < 0) {
		printk("pwm3p_enable_output failed: %d\n", ret);
		return 0;
	}

	printk("PWM3P output enabled, sweeping duty %d%%..%d%%\n", DUTY_MIN_PERCENT,
	       DUTY_MAX_PERCENT);

	while (1) {
		uint32_t duty_cycles = (uint32_t)(((uint64_t)period_cycles * duty_percent) / 100U);

		ret = pwm3p_set_duty_cycles(pwm3p, duty_cycles, duty_cycles, duty_cycles);
		if (ret < 0) {
			printk("pwm3p_set_duty_cycles failed: %d\n", ret);
		}

		printk("duty=%d%% (%u counts), cycle_count=%ld\n", duty_percent, duty_cycles,
		       (long)atomic_get(&cycle_count));

		if (duty_percent == DUTY_MAX_PERCENT) {
			step = -DUTY_STEP_PERCENT;
		} else if (duty_percent == DUTY_MIN_PERCENT) {
			step = DUTY_STEP_PERCENT;
		}
		duty_percent += step;

		k_sleep(K_SECONDS(1));
	}

	return 0;
}
