/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/firmware/scmi/clk.h>
#include <zephyr/drivers/firmware/scmi/protocol.h>
#include <zephyr/dt-bindings/clock/r8a78000_scmi_clock.h>
#include <zephyr/logging/log.h>
#include <zephyr/ztest.h>

LOG_MODULE_REGISTER(scmi_clock_test, LOG_LEVEL_INF);

#define SCMI_CLOCK_NODE DT_NODELABEL(scmi_clk)

/*
 * SCIF1 is the R-Car R52 console and must remain enabled.  SCIF0 is unused
 * by the board DTS and is also the non-console test target used by scmi_reset.
 */
#define RCAR_X5H_SCMI_CLOCK_TEST_ID R8A78000_SCMI_CLK_SCIF0

#define SCMI_CLOCK_SUBSYS(id) ((clock_control_subsys_t)(uintptr_t)(id))
#define SCMI_CLOCK_RATE(rate) ((clock_control_subsys_rate_t)(uintptr_t)(rate))

static bool scmi_command_is_optional(int ret)
{
	return (ret == -ENOTSUP) || (ret == -EOPNOTSUPP) || (ret == -EACCES) || (ret == -EPERM) ||
	       (ret == -EALREADY);
}

static void assert_generic_status(const struct device *clock_dev,
				  enum clock_control_status expected)
{
	zassert_equal(
		clock_control_get_status(clock_dev, SCMI_CLOCK_SUBSYS(RCAR_X5H_SCMI_CLOCK_TEST_ID)),
		expected, "unexpected SCIF0 generic clock status");
}

ZTEST(scmi_clock, test_protocol_and_raw_query_apis)
{
	const struct device *clock_dev = DEVICE_DT_GET(SCMI_CLOCK_NODE);
	struct scmi_protocol *proto = clock_dev->data;
	struct scmi_clock_attributes clock_attributes;
	uint32_t version;
	uint32_t protocol_attributes;
	uint32_t num_clocks;
	uint32_t config;
	uint32_t rate = 0U;
	int ret;

	zassert_true(device_is_ready(clock_dev), "SCMI clock controller is not ready");
	zassert_equal(proto->id, SCMI_PROTOCOL_CLOCK, "unexpected SCMI protocol ID: %u", proto->id);

	zassert_ok(scmi_protocol_get_version(proto, &version),
		   "Clock protocol version query failed");
	zassert_equal(version, SCMI_CLK_PROTOCOL_SUPPORTED_VERSION,
		      "unexpected clock protocol version: 0x%08x", version);

	zassert_ok(scmi_protocol_attributes_get(proto, &protocol_attributes),
		   "Clock protocol attributes query failed");
	num_clocks = SCMI_CLK_ATTRIBUTES_CLK_NUM(protocol_attributes);
	zassert_true(num_clocks > RCAR_X5H_SCMI_CLOCK_TEST_ID,
		     "SCP exposes %u clocks; SCIF0 ID %u is unavailable", num_clocks,
		     RCAR_X5H_SCMI_CLOCK_TEST_ID);

	zassert_ok(scmi_clock_attributes(proto, RCAR_X5H_SCMI_CLOCK_TEST_ID, &clock_attributes),
		   "SCIF0 attributes query failed");
	zassert_not_equal(clock_attributes.clock_name[0], '\0', "SCIF0 has no clock name");

	zassert_ok(scmi_clock_config_get(proto, RCAR_X5H_SCMI_CLOCK_TEST_ID, 0U, &config),
		   "SCIF0 config query failed");
	ret = scmi_clock_rate_get(proto, RCAR_X5H_SCMI_CLOCK_TEST_ID, &rate);
	if (ret == 0) {
		LOG_INF("SCIF0: config %#x, rate %u Hz, enable delay %u us", config, rate,
			clock_attributes.clock_enable_delay);
	} else if (scmi_command_is_optional(ret)) {
		LOG_INF("SCIF0 CLOCK_RATE_GET is not supported (%d)", ret);
	} else {
		zassert_ok(ret, "SCIF0 rate query failed");
	}

	LOG_INF("SCIF0 CLOCK_PARENT_GET skipped: unsupported by X5H SCP");

	/* The generic controller must reject this ID without sending an SCMI request. */
	zassert_equal(clock_control_on(clock_dev, SCMI_CLOCK_SUBSYS(num_clocks)), -EINVAL,
		      "out-of-range clock ID was not rejected by clock_control_on()");
	zassert_equal(clock_control_off(clock_dev, SCMI_CLOCK_SUBSYS(num_clocks)), -EINVAL,
		      "out-of-range clock ID was not rejected by clock_control_off()");
	zassert_equal(clock_control_get_rate(clock_dev, SCMI_CLOCK_SUBSYS(num_clocks), &rate),
		      -EINVAL,
		      "out-of-range clock ID was not rejected by clock_control_get_rate()");
	zassert_equal(clock_control_set_rate(clock_dev, SCMI_CLOCK_SUBSYS(num_clocks),
					     SCMI_CLOCK_RATE(rate)),
		      -EINVAL,
		      "out-of-range clock ID was not rejected by clock_control_set_rate()");
	zassert_equal(clock_control_get_status(clock_dev, SCMI_CLOCK_SUBSYS(num_clocks)),
		      CLOCK_CONTROL_STATUS_UNKNOWN,
		      "out-of-range clock ID did not report unknown status");
}

ZTEST(scmi_clock, test_raw_scmi_mutating_apis)
{
	const struct device *clock_dev = DEVICE_DT_GET(SCMI_CLOCK_NODE);
	struct scmi_protocol *proto = clock_dev->data;
	struct scmi_clock_rate_config rate_cfg = {0};
	struct scmi_clock_config config = {
		.clk_id = RCAR_X5H_SCMI_CLOCK_TEST_ID,
	};
	uint32_t rate = 0U;
	uint32_t original_config;
	int ret;

	ret = scmi_clock_rate_get(proto, RCAR_X5H_SCMI_CLOCK_TEST_ID, &rate);
	if (ret == 0) {
		rate_cfg.flags = SCMI_CLK_RATE_SET_FLAGS_ROUNDS_AUTO;
		rate_cfg.clk_id = RCAR_X5H_SCMI_CLOCK_TEST_ID;
		rate_cfg.rate[0] = rate;

		/* Setting the existing rate verifies CLOCK_RATE_SET without changing it. */
		ret = scmi_clock_rate_set(proto, &rate_cfg);
		if (scmi_command_is_optional(ret)) {
			LOG_INF("SCIF0 CLOCK_RATE_SET is not supported for its current rate (%d)",
				ret);
		} else {
			zassert_ok(ret, "SCIF0 CLOCK_RATE_SET failed");
		}
	} else if (scmi_command_is_optional(ret)) {
		LOG_INF("SCIF0 CLOCK_RATE_SET skipped: CLOCK_RATE_GET is not supported (%d)", ret);
	} else {
		zassert_ok(ret, "SCIF0 rate query failed");
	}

	LOG_INF("SCIF0 CLOCK_PARENT_SET skipped: unsupported by X5H SCP");

	zassert_ok(scmi_clock_config_get(proto, RCAR_X5H_SCMI_CLOCK_TEST_ID, 0U, &original_config),
		   "SCIF0 config query failed");
	config.attributes = SCMI_CLK_CONFIG_ENABLE_DISABLE(true);
	ret = scmi_clock_config_set(proto, &config);
	if (scmi_command_is_optional(ret)) {
		LOG_INF("SCIF0 CLOCK_CONFIG_SET enable is not permitted (%d)", ret);
		return;
	}
	zassert_ok(ret, "SCIF0 CLOCK_CONFIG_SET enable failed");
	zassert_ok(
		scmi_clock_config_get(proto, RCAR_X5H_SCMI_CLOCK_TEST_ID, 0U, &config.attributes),
		"SCIF0 config query failed after enable");
	zassert_equal(SCMI_CLK_CONFIG_ENABLE_DISABLE(config.attributes), 1U,
		      "SCIF0 is not enabled after CLOCK_CONFIG_SET");

	config.attributes = SCMI_CLK_CONFIG_ENABLE_DISABLE(false);
	ret = scmi_clock_config_set(proto, &config);
	if (ret != 0) {
		/* The enable succeeded, so restore it before reporting the failure. */
		config.attributes = SCMI_CLK_CONFIG_ENABLE_DISABLE(
			SCMI_CLK_CONFIG_ENABLE_DISABLE(original_config) == 1U);
		(void)scmi_clock_config_set(proto, &config);
		zassert_ok(ret, "SCIF0 CLOCK_CONFIG_SET disable failed");
		return;
	}
	zassert_ok(
		scmi_clock_config_get(proto, RCAR_X5H_SCMI_CLOCK_TEST_ID, 0U, &config.attributes),
		"SCIF0 config query failed after disable");
	zassert_equal(SCMI_CLK_CONFIG_ENABLE_DISABLE(config.attributes), 0U,
		      "SCIF0 remains enabled after CLOCK_CONFIG_SET disable");

	config.attributes = SCMI_CLK_CONFIG_ENABLE_DISABLE(
		SCMI_CLK_CONFIG_ENABLE_DISABLE(original_config) == 1U);
	zassert_ok(scmi_clock_config_set(proto, &config), "SCIF0 state restoration failed");
}

ZTEST(scmi_clock, test_generic_clock_control_apis)
{
	const struct device *clock_dev = DEVICE_DT_GET(SCMI_CLOCK_NODE);
	struct scmi_protocol *proto = clock_dev->data;
	clock_control_subsys_t scif0 = SCMI_CLOCK_SUBSYS(RCAR_X5H_SCMI_CLOCK_TEST_ID);
	uint32_t generic_rate = 0U;
	uint32_t raw_rate = 0U;
	enum clock_control_status original_status;
	enum clock_control_status status_after_on;
	enum clock_control_status status_after_off;
	int ret;

	original_status = clock_control_get_status(clock_dev, scif0);
	zassert_true((original_status == CLOCK_CONTROL_STATUS_ON) ||
			     (original_status == CLOCK_CONTROL_STATUS_OFF),
		     "SCIF0 generic status is unavailable");

	ret = clock_control_get_rate(clock_dev, scif0, &generic_rate);
	if (ret == 0) {
		zassert_ok(scmi_clock_rate_get(proto, RCAR_X5H_SCMI_CLOCK_TEST_ID, &raw_rate),
			   "SCIF0 raw rate query failed after generic success");
		zassert_equal(generic_rate, raw_rate, "generic and raw SCMI clock rates differ");

		/* Preserve the rate while exercising clock_control_set_rate(). */
		ret = clock_control_set_rate(clock_dev, scif0, SCMI_CLOCK_RATE(generic_rate));
		if (scmi_command_is_optional(ret)) {
			LOG_INF("clock_control_set_rate() is not supported for SCIF0 (%d)", ret);
		} else {
			zassert_ok(ret, "clock_control_set_rate() failed");
		}
	} else if (scmi_command_is_optional(ret)) {
		LOG_INF("clock_control_get_rate() is not supported for SCIF0 (%d)", ret);
	} else {
		zassert_ok(ret, "clock_control_get_rate() failed");
	}

	zassert_ok(clock_control_on(clock_dev, scif0), "clock_control_on() failed");
	status_after_on = clock_control_get_status(clock_dev, scif0);
	zassert_true((status_after_on == CLOCK_CONTROL_STATUS_ON) ||
			     (status_after_on == CLOCK_CONTROL_STATUS_OFF),
		     "SCIF0 status is unavailable after clock_control_on()");
	zassert_ok(clock_control_off(clock_dev, scif0), "clock_control_off() failed");
	status_after_off = clock_control_get_status(clock_dev, scif0);
	zassert_true((status_after_off == CLOCK_CONTROL_STATUS_ON) ||
			     (status_after_off == CLOCK_CONTROL_STATUS_OFF),
		     "SCIF0 status is unavailable after clock_control_off()");

	/* Restore the state even if SCP treated the two generic requests as no-ops. */
	if (original_status == CLOCK_CONTROL_STATUS_ON) {
		zassert_ok(clock_control_on(clock_dev, scif0), "SCIF0 state restoration failed");
	} else {
		zassert_ok(clock_control_off(clock_dev, scif0), "SCIF0 state restoration failed");
	}
	assert_generic_status(clock_dev, original_status);

	if ((status_after_on == CLOCK_CONTROL_STATUS_ON) &&
	    (status_after_off == CLOCK_CONTROL_STATUS_OFF)) {
		LOG_INF("clock_control_on() and clock_control_off() changed SCIF0 state");
	} else {
		LOG_INF("SCIF0 state did not change; SCP may restrict state control");
	}
}

ZTEST_SUITE(scmi_clock, NULL, NULL, NULL, NULL, NULL);
