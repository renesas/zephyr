/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm3p.h>
#include <zephyr/ztest.h>

#define PWM3P_NODE DT_NODELABEL(taud3p0)
#define PWM3P_REWRITE_SETTLE_MS 5

static const struct device *const pwm3p = DEVICE_DT_GET(PWM3P_NODE);

static uint32_t original_period_cycles;

static void pwm3p_wait_rewrite_done(void)
{
	zassert_equal(pwm3p_enable_output(pwm3p, true), 0, "pwm3p_enable_output(true) failed");
	k_msleep(PWM3P_REWRITE_SETTLE_MS);
}

static void *pwm3p_api_setup(void)
{
	int ret;

	zassert_true(device_is_ready(pwm3p), "PWM3P device is not ready");

	ret = pwm3p_get_period_cycles(pwm3p, &original_period_cycles);
	zassert_equal(ret, 0, "pwm3p_get_period_cycles failed: %d", ret);

	return NULL;
}

static void pwm3p_api_after(void *f)
{
	ARG_UNUSED(f);

	/* Leave the device in a known, disabled state, unregister any callback, and restore
	 * the original period so test order does not matter.
	 */
	(void)pwm3p_set_callback(pwm3p, NULL, NULL);
	(void)pwm3p_enable_output(pwm3p, true);
	k_msleep(PWM3P_REWRITE_SETTLE_MS);
	(void)pwm3p_set_period_cycles(pwm3p, original_period_cycles);
	k_msleep(PWM3P_REWRITE_SETTLE_MS);
	(void)pwm3p_enable_output(pwm3p, false);
}

ZTEST_SUITE(pwm3p_api, NULL, pwm3p_api_setup, NULL, pwm3p_api_after, NULL);

ZTEST(pwm3p_api, test_get_period_cycles_matches_devicetree)
{
	uint32_t period_cycles;
	int ret;

	ret = pwm3p_get_period_cycles(pwm3p, &period_cycles);

	zassert_equal(ret, 0, "pwm3p_get_period_cycles failed: %d", ret);
	zassert_equal(period_cycles, DT_PROP(PWM3P_NODE, period_counts),
		      "period_cycles %u does not match devicetree period-counts %u",
		      period_cycles, DT_PROP(PWM3P_NODE, period_counts));
}

ZTEST(pwm3p_api, test_set_period_cycles_valid)
{
	uint32_t period_cycles;
	int ret;

	ret = pwm3p_set_period_cycles(pwm3p, 1000);
	zassert_equal(ret, 0, "pwm3p_set_period_cycles(1000) failed: %d", ret);

	ret = pwm3p_get_period_cycles(pwm3p, &period_cycles);
	zassert_equal(ret, 0, "pwm3p_get_period_cycles failed: %d", ret);
	zassert_equal(period_cycles, 1000, "period_cycles %u != 1000", period_cycles);
}

ZTEST(pwm3p_api, test_set_period_cycles_rejects_zero)
{
	int ret = pwm3p_set_period_cycles(pwm3p, 0);

	zassert_equal(ret, -EINVAL, "pwm3p_set_period_cycles(0) returned %d, expected -EINVAL",
		      ret);
}

ZTEST(pwm3p_api, test_set_period_cycles_rejects_one)
{
	int ret = pwm3p_set_period_cycles(pwm3p, 1);

	zassert_equal(ret, -EINVAL, "pwm3p_set_period_cycles(1) returned %d, expected -EINVAL",
		      ret);
}

ZTEST(pwm3p_api, test_set_period_cycles_rejects_odd)
{
	int ret = pwm3p_set_period_cycles(pwm3p, 1001);

	zassert_equal(ret, -EINVAL, "pwm3p_set_period_cycles(1001) returned %d, expected -EINVAL",
		      ret);
}

ZTEST(pwm3p_api, test_set_period_cycles_rejects_too_large)
{
	int ret = pwm3p_set_period_cycles(pwm3p, 0x20002U);

	zassert_equal(ret, -EINVAL,
		      "pwm3p_set_period_cycles(0x20002) returned %d, expected -EINVAL", ret);
}

ZTEST(pwm3p_api, test_set_duty_cycles_valid)
{
	uint32_t period_cycles;
	int ret;

	ret = pwm3p_get_period_cycles(pwm3p, &period_cycles);
	zassert_equal(ret, 0, "pwm3p_get_period_cycles failed: %d", ret);

	ret = pwm3p_set_duty_cycles(pwm3p, period_cycles / 4U, period_cycles / 2U,
				     (3U * period_cycles) / 4U);
	zassert_equal(ret, 0, "pwm3p_set_duty_cycles with mixed valid duties failed: %d", ret);
	pwm3p_wait_rewrite_done();

	ret = pwm3p_set_duty_cycles(pwm3p, period_cycles, period_cycles, period_cycles);
	zassert_equal(ret, 0, "pwm3p_set_duty_cycles(period, period, period) failed: %d", ret);
	pwm3p_wait_rewrite_done();

	ret = pwm3p_set_duty_cycles(pwm3p, 0, 0, 0);
	zassert_equal(ret, 0, "pwm3p_set_duty_cycles(0, 0, 0) failed: %d", ret);
}

ZTEST(pwm3p_api, test_set_duty_cycles_rejects_duty_above_period)
{
	uint32_t period_cycles;
	int ret;

	ret = pwm3p_get_period_cycles(pwm3p, &period_cycles);
	zassert_equal(ret, 0, "pwm3p_get_period_cycles failed: %d", ret);

	ret = pwm3p_set_duty_cycles(pwm3p, period_cycles + 1U, period_cycles / 2U,
				     period_cycles / 2U);
	zassert_equal(ret, -EINVAL, "duty_u > period: returned %d, expected -EINVAL", ret);

	/* Inside the deadtime guard band (period - deadtime, period): not allowed. */
	ret = pwm3p_set_duty_cycles(pwm3p, period_cycles - 1U, period_cycles / 2U,
				     period_cycles / 2U);
	zassert_equal(ret, -EINVAL, "duty_u in deadtime guard band: returned %d, expected -EINVAL",
		      ret);

	ret = pwm3p_set_duty_cycles(pwm3p, period_cycles / 2U, period_cycles + 1U,
				     period_cycles / 2U);
	zassert_equal(ret, -EINVAL, "duty_v > period: returned %d, expected -EINVAL", ret);

	ret = pwm3p_set_duty_cycles(pwm3p, period_cycles / 2U, period_cycles / 2U,
				     period_cycles + 1U);
	zassert_equal(ret, -EINVAL, "duty_w > period: returned %d, expected -EINVAL", ret);
}

ZTEST(pwm3p_api, test_enable_disable_output)
{
	int ret;

	ret = pwm3p_enable_output(pwm3p, true);
	zassert_equal(ret, 0, "pwm3p_enable_output(true) failed: %d", ret);

	ret = pwm3p_enable_output(pwm3p, false);
	zassert_equal(ret, 0, "pwm3p_enable_output(false) failed: %d", ret);
}

ZTEST(pwm3p_api, test_get_cycles_per_sec_nonzero)
{
	uint64_t cycles;
	int ret;

	ret = pwm3p_get_cycles_per_sec(pwm3p, &cycles);

	zassert_equal(ret, 0, "pwm3p_get_cycles_per_sec failed: %d", ret);
	zassert_true(cycles > 0, "cycles_per_sec must be > 0");
}

ZTEST(pwm3p_api, test_reset_requires_output_enabled)
{
	int ret;

	ret = pwm3p_enable_output(pwm3p, false);
	zassert_equal(ret, 0, "pwm3p_enable_output(false) failed: %d", ret);

	ret = pwm3p_reset(pwm3p);
	zassert_equal(ret, -EAGAIN, "pwm3p_reset while disabled returned %d, expected -EAGAIN",
		      ret);

	ret = pwm3p_enable_output(pwm3p, true);
	zassert_equal(ret, 0, "pwm3p_enable_output(true) failed: %d", ret);

	ret = pwm3p_reset(pwm3p);
	zassert_equal(ret, 0, "pwm3p_reset while enabled failed: %d", ret);
}

static atomic_t callback_fire_count;
static void *callback_last_user_data;

static void test_cycle_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);

	callback_last_user_data = user_data;
	atomic_inc(&callback_fire_count);
}

ZTEST(pwm3p_api, test_set_callback_register_and_unregister)
{
	int ret;

	ret = pwm3p_set_callback(pwm3p, test_cycle_callback, NULL);
	zassert_equal(ret, 0, "pwm3p_set_callback(callback) failed: %d", ret);

	ret = pwm3p_set_callback(pwm3p, NULL, NULL);
	zassert_equal(ret, 0, "pwm3p_set_callback(NULL) to unregister failed: %d", ret);
}

ZTEST(pwm3p_api, test_set_callback_passes_user_data)
{
	int marker = 0x3c00;
	int ret;

	atomic_clear(&callback_fire_count);
	callback_last_user_data = NULL;

	ret = pwm3p_set_callback(pwm3p, test_cycle_callback, &marker);
	zassert_equal(ret, 0, "pwm3p_set_callback failed: %d", ret);

	ret = pwm3p_enable_output(pwm3p, true);
	zassert_equal(ret, 0, "pwm3p_enable_output(true) failed: %d", ret);

	k_msleep(20);

	ret = pwm3p_enable_output(pwm3p, false);
	zassert_equal(ret, 0, "pwm3p_enable_output(false) failed: %d", ret);

	zassert_true(atomic_get(&callback_fire_count) > 0,
		     "callback never fired, cannot verify user_data was forwarded");
	zassert_equal_ptr(callback_last_user_data, &marker,
			   "callback received user_data %p, expected %p",
			   callback_last_user_data, &marker);
}

/*
 * This instance's devicetree node has an 'interrupts' property, so the callback is expected to
 * actually fire once per PWM period while the output is enabled - not just return 0 from
 * set_callback(). Uses a loose tolerance (4x in either direction) since this is a sanity check
 * that the ISR fires at roughly the right rate, not a precision timing test.
 */
ZTEST(pwm3p_api, test_callback_fires_once_per_period_while_enabled)
{
	uint32_t period_cycles;
	uint64_t cycles_per_sec;
	uint32_t sleep_ms = 50U;
	uint32_t period_us;
	uint32_t expected_count;
	int count;
	int ret;

	ret = pwm3p_get_period_cycles(pwm3p, &period_cycles);
	zassert_equal(ret, 0, "pwm3p_get_period_cycles failed: %d", ret);

	ret = pwm3p_get_cycles_per_sec(pwm3p, &cycles_per_sec);
	zassert_equal(ret, 0, "pwm3p_get_cycles_per_sec failed: %d", ret);

	period_us = (uint32_t)(((uint64_t)period_cycles * 1000000ULL) / cycles_per_sec);
	zassert_true(period_us > 0, "computed PWM period is 0us, cannot size the test sleep");
	expected_count = (sleep_ms * 1000U) / period_us;
	zassert_true(expected_count > 0,
		     "sleep_ms %u too short for a %u us period to fire even once", sleep_ms,
		     period_us);

	atomic_clear(&callback_fire_count);

	ret = pwm3p_set_callback(pwm3p, test_cycle_callback, NULL);
	zassert_equal(ret, 0, "pwm3p_set_callback failed: %d", ret);

	ret = pwm3p_enable_output(pwm3p, true);
	zassert_equal(ret, 0, "pwm3p_enable_output(true) failed: %d", ret);

	k_msleep(sleep_ms);

	ret = pwm3p_enable_output(pwm3p, false);
	zassert_equal(ret, 0, "pwm3p_enable_output(false) failed: %d", ret);

	count = atomic_get(&callback_fire_count);
	zassert_true(count > 0, "callback never fired after %u ms with output enabled",
		     sleep_ms);
	zassert_true(count >= (int)(expected_count / 4U) && count <= (int)(expected_count * 4U),
		     "callback fired %d times, expected roughly %u (period=%u us, slept %u ms)",
		     count, expected_count, period_us, sleep_ms);
}

ZTEST(pwm3p_api, test_callback_stops_firing_after_unregister)
{
	int ret;
	int count_at_unregister;
	int count_after_wait;

	atomic_clear(&callback_fire_count);

	ret = pwm3p_set_callback(pwm3p, test_cycle_callback, NULL);
	zassert_equal(ret, 0, "pwm3p_set_callback failed: %d", ret);

	ret = pwm3p_enable_output(pwm3p, true);
	zassert_equal(ret, 0, "pwm3p_enable_output(true) failed: %d", ret);

	k_msleep(20);

	ret = pwm3p_set_callback(pwm3p, NULL, NULL);
	zassert_equal(ret, 0, "pwm3p_set_callback(NULL) failed: %d", ret);

	count_at_unregister = atomic_get(&callback_fire_count);
	zassert_true(count_at_unregister > 0, "callback never fired before unregistering");

	k_msleep(20);

	count_after_wait = atomic_get(&callback_fire_count);
	zassert_equal(count_after_wait, count_at_unregister,
		      "callback kept firing (%d -> %d) after being unregistered",
		      count_at_unregister, count_after_wait);
}
