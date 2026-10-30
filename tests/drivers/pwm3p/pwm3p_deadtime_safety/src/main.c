/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This is a manual hardware bring-up test: whether the deadtime
 * between a phase's high-side and low-side output ever reaches 0 can only be observed with a
 * logic analyzer/oscilloscope on the real 6 physical outputs - it is not something software
 * running on the MCU itself can measure. This ztest suite only automates the software-visible
 * half of the test (driving the API through the exact sequence the hardware check needs,
 * and confirming every call the sequence depends on succeeds); a human must capture the
 * waveform and confirm the deadtime gap. See README.rst for the manual procedure.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm3p.h>
#include <zephyr/ztest.h>

#define PWM3P_NODE DT_NODELABEL(pwm3p0)

#define DUTY_LOW_PERCENT  5U
#define DUTY_HIGH_PERCENT 95U

static const struct device *const pwm3p = DEVICE_DT_GET(PWM3P_NODE);

static void *pwm3p_deadtime_setup(void)
{
	int ret;

	zassert_true(device_is_ready(pwm3p), "PWM3P device is not ready");

	ret = pwm3p_enable_output(pwm3p, true);
	zassert_equal(ret, 0, "pwm3p_enable_output(true) failed: %d", ret);

	return NULL;
}

static void pwm3p_deadtime_teardown(void *f)
{
	ARG_UNUSED(f);

	(void)pwm3p_enable_output(pwm3p, false);
}

ZTEST_SUITE(pwm3p_deadtime_safety, NULL, pwm3p_deadtime_setup, NULL, NULL,
	    pwm3p_deadtime_teardown);

/*
 * Jumps duty from 5% to 95% (and back) in a single pwm3p_set_duty_cycles() call - the
 * worst case for deadtime insertion, since the high-side and low-side compare points move
 * almost all the way across the period in one update. Manual step: capture phase U's
 * high-side and low-side pins on a logic analyzer while this test runs and confirm the gap
 * between high-side falling edge and low-side rising edge (and vice versa) is always >=
 * the configured 'deadtime-counts', on every transition this test prints.
 */
ZTEST(pwm3p_deadtime_safety, test_large_sudden_duty_jump_is_accepted)
{
	uint32_t period_cycles;
	uint32_t duty_low;
	uint32_t duty_high;
	int ret;

	ret = pwm3p_get_period_cycles(pwm3p, &period_cycles);
	zassert_equal(ret, 0, "pwm3p_get_period_cycles failed: %d", ret);

	duty_low = (period_cycles * DUTY_LOW_PERCENT) / 100U;
	duty_high = (period_cycles * DUTY_HIGH_PERCENT) / 100U;

	/* The driver rejects duty in (period - deadtime, period), so the "high" step is capped at
	 * the largest duty that still leaves room for the configured deadtime.
	 */
	duty_high = MIN(duty_high, period_cycles - MAX(DT_PROP(PWM3P_NODE, deadtime_counts), 1U));

	for (int i = 0; i < 20; i++) {
		printk("DEADTIME_SAFETY: period=%u transition -> duty=%u (%u%%)\n", period_cycles,
		       duty_low, DUTY_LOW_PERCENT);
		ret = pwm3p_set_duty_cycles(pwm3p, duty_low, duty_low, duty_low);
		zassert_equal(ret, 0, "pwm3p_set_duty_cycles(low) failed on iteration %d: %d", i,
			      ret);
		k_msleep(50);

		printk("DEADTIME_SAFETY: period=%u transition -> duty=%u (%u%%)\n", period_cycles,
		       duty_high, DUTY_HIGH_PERCENT);
		ret = pwm3p_set_duty_cycles(pwm3p, duty_high, duty_high, duty_high);
		zassert_equal(ret, 0, "pwm3p_set_duty_cycles(high) failed on iteration %d: %d", i,
			      ret);
		k_msleep(50);
	}

	printk("DEADTIME_SAFETY: sequence complete, inspect captured waveform now\n");
}
