/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This is a manual hardware bring-up test: whether the actual
 * physical duty cycle of each phase (U/V/W) matches the value commanded through
 * pwm3p_set_duty_cycles() can only be confirmed with a logic analyzer/oscilloscope on the real
 * 6 physical outputs - it is not something software running on the MCU itself can measure.
 * This ztest suite only automates the software-visible half of the test (driving each phase to
 * a distinct, known duty cycle and confirming every API call succeeds); a human must capture
 * the waveform and confirm the measured duty of each phase. See README.rst for the manual
 * procedure. This is a different concern from tests/drivers/pwm/pwm3p_deadtime_safety, which
 * checks the high-side/low-side edge *gap* (deadtime), not the commanded duty *level*.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm3p.h>
#include <zephyr/ztest.h>

#define PWM3P_NODE DT_NODELABEL(pwm3p0)

static const struct device *const pwm3p = DEVICE_DT_GET(PWM3P_NODE);

static void *pwm3p_duty_cycles_setup(void)
{
	int ret;

	zassert_true(device_is_ready(pwm3p), "PWM3P device is not ready");

	ret = pwm3p_enable_output(pwm3p, true);
	zassert_equal(ret, 0, "pwm3p_enable_output(true) failed: %d", ret);

	return NULL;
}

static void pwm3p_duty_cycles_teardown(void *f)
{
	ARG_UNUSED(f);

	(void)pwm3p_enable_output(pwm3p, false);
}

ZTEST_SUITE(pwm3p_duty_cycles, NULL, pwm3p_duty_cycles_setup, NULL, NULL,
	    pwm3p_duty_cycles_teardown);

/*
 * Commands 3 distinct, easy-to-read-on-a-scope duty percentages for U/V/W (10%/50%/90%) and
 * holds them long enough for a human to capture and measure each phase. Manual step: measure
 * the high-side pulse width of each phase as a percentage of the period and confirm it matches
 * the printed target within measurement tolerance.
 */
ZTEST(pwm3p_duty_cycles, test_distinct_duty_per_phase_is_accepted)
{
	uint32_t period_cycles;
	uint32_t duty_u;
	uint32_t duty_v;
	uint32_t duty_w;
	int ret;

	ret = pwm3p_get_period_cycles(pwm3p, &period_cycles);
	zassert_equal(ret, 0, "pwm3p_get_period_cycles failed: %d", ret);

	duty_u = (period_cycles * 10U) / 100U;
	duty_v = (period_cycles * 50U) / 100U;
	duty_w = (period_cycles * 90U) / 100U;

	printk("DUTY_CYCLES: period=%u commanding U=%u(10%%) V=%u(50%%) W=%u(90%%)\n",
	       period_cycles, duty_u, duty_v, duty_w);

	ret = pwm3p_set_duty_cycles(pwm3p, duty_u, duty_v, duty_w);
	zassert_equal(ret, 0, "pwm3p_set_duty_cycles(10%%, 50%%, 90%%) failed: %d", ret);

	k_msleep(2000);

	printk("DUTY_CYCLES: hold complete, measure now\n");
}

/*
 * Sweeps all 3 phases together through a range of duty percentages, holding each for long
 * enough to measure. Manual step: at each printed target, confirm the measured duty of every
 * phase tracks the commanded value (not just phase U).
 */
ZTEST(pwm3p_duty_cycles, test_duty_sweep_matches_commanded_percentage)
{
	static const uint32_t percentages[] = {0U, 25U, 50U, 75U, 100U};
	uint32_t period_cycles;
	int ret;

	ret = pwm3p_get_period_cycles(pwm3p, &period_cycles);
	zassert_equal(ret, 0, "pwm3p_get_period_cycles failed: %d", ret);

	for (size_t i = 0; i < ARRAY_SIZE(percentages); i++) {
		uint32_t duty_cycles = (period_cycles * percentages[i]) / 100U;

		printk("DUTY_CYCLES: period=%u commanding U=V=W=%u(%u%%)\n", period_cycles,
		       duty_cycles, percentages[i]);

		ret = pwm3p_set_duty_cycles(pwm3p, duty_cycles, duty_cycles, duty_cycles);
		zassert_equal(ret, 0, "pwm3p_set_duty_cycles(%u%%) failed on step %zu: %d",
			      percentages[i], i, ret);

		k_msleep(1000);
	}

	printk("DUTY_CYCLES: sweep complete, inspect captured waveform now\n");
}
