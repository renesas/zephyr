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
#define PIC_NODE   DT_NODELABEL(pic0)

static const struct device *const pwm3p = DEVICE_DT_GET(PWM3P_NODE);

/*
 * The PWM3P driver does not configure the PIC: the outputs are routed by the separate PIC device
 * (renesas,rh850-pic) referenced by the "pic" property of the PWM3P node. The overlay of the
 * test enables both nodes. The PIC is initialized before the PWM3P device because the PWM3P
 * node depends on it.
 */
BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(PIC_NODE),
	     "pic0 must be enabled by the overlay, otherwise PWM3P has no output");
BUILD_ASSERT(DT_SAME_NODE(DT_PHANDLE(PWM3P_NODE, pic), PIC_NODE),
	     "taud3p0 must reference pic0");

static void pwm3p_sync_after(void *f)
{
	ARG_UNUSED(f);

	(void)pwm3p_enable_output(pwm3p, false);
}

ZTEST_SUITE(pwm3p_taud3p_sync, NULL, NULL, NULL, pwm3p_sync_after, NULL);

/* The timer channels of the PWM3P device are all configured by its init function, which runs
 * before main() and this test. device_is_ready() reports true only if the whole init sequence
 * succeeded, so this is the end-to-end check of the device initialization. The PIC is checked by
 * test_pic_is_ready below.
 */
ZTEST(pwm3p_taud3p_sync, test_all_channels_opened_successfully)
{
	zassert_true(device_is_ready(pwm3p),
		     "PWM3P device not ready: TAUD open sequence failed during init");
}

/* The PIC is a separate device that routes the PWM3P signals. Its init function configures
 * the routing, and device_is_ready() is true only if that succeeded.
 */
ZTEST(pwm3p_taud3p_sync, test_pic_is_ready)
{
	zassert_true(device_is_ready(DEVICE_DT_GET(PIC_NODE)),
		     "PIC device not ready: its init failed, PWM3P has no output");
}

/* pwm3p_enable_output() starts and stops all the PWM channels together. This does not prove
 * that no physical pin is left toggling (that needs a logic analyzer, see
 * tests/drivers/pwm/pwm3p_deadtime_safety), but it verifies the driver-level contract: a single
 * pwm3p_enable_output(false) call must fully succeed.
 */
ZTEST(pwm3p_taud3p_sync, test_enable_then_disable_reports_success)
{
	int ret;

	ret = pwm3p_enable_output(pwm3p, true);
	zassert_equal(ret, 0, "pwm3p_enable_output(true) failed: %d", ret);

	ret = pwm3p_enable_output(pwm3p, false);
	zassert_equal(ret, 0, "pwm3p_enable_output(false) failed: %d", ret);
}

/* pwm3p_reset() is only allowed while the outputs are enabled. Checking -EAGAIN right after a
 * disable is an indirect but driver-visible proof that the disable changed the run state of
 * the device and not only a flag of the driver.
 */
ZTEST(pwm3p_taud3p_sync, test_disable_actually_stops_hardware)
{
	int ret;

	ret = pwm3p_enable_output(pwm3p, true);
	zassert_equal(ret, 0, "pwm3p_enable_output(true) failed: %d", ret);

	ret = pwm3p_enable_output(pwm3p, false);
	zassert_equal(ret, 0, "pwm3p_enable_output(false) failed: %d", ret);

	ret = pwm3p_reset(pwm3p);
	zassert_equal(ret, -EAGAIN,
		      "pwm3p_reset() succeeded (%d) after disable - hardware may not have "
		      "actually stopped",
		      ret);
}

/* Repeated enable/disable cycles must stay consistent. This guards against a channel being
 * left active, which only shows after the device has been toggled more than once.
 */
ZTEST(pwm3p_taud3p_sync, test_repeated_enable_disable_cycles_stay_consistent)
{
	for (int i = 0; i < 10; i++) {
		int ret = pwm3p_enable_output(pwm3p, true);

		zassert_equal(ret, 0, "cycle %d: pwm3p_enable_output(true) failed: %d", i, ret);

		ret = pwm3p_enable_output(pwm3p, false);
		zassert_equal(ret, 0, "cycle %d: pwm3p_enable_output(false) failed: %d", i, ret);
	}
}

/* The duty cycles must be settable right after the outputs are enabled. An error in the
 * initialization sequence would most likely show here, as a failure of one of the phase
 * updates.
 */
ZTEST(pwm3p_taud3p_sync, test_set_duty_cycles_after_enable)
{
	uint32_t period_cycles;
	int ret;

	ret = pwm3p_get_period_cycles(pwm3p, &period_cycles);
	zassert_equal(ret, 0, "pwm3p_get_period_cycles failed: %d", ret);

	ret = pwm3p_enable_output(pwm3p, true);
	zassert_equal(ret, 0, "pwm3p_enable_output(true) failed: %d", ret);

	ret = pwm3p_set_duty_cycles(pwm3p, period_cycles / 4U, period_cycles / 2U,
				     (3U * period_cycles) / 4U);
	zassert_equal(ret, 0, "pwm3p_set_duty_cycles while running failed: %d", ret);
}
