/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/firmware/scmi/power.h>
#include <zephyr/drivers/firmware/scmi/system.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

struct scmi_power_domain_test_case {
	const char *path;
	const struct device *dev;
	uint32_t id;
};

#define SCMI_POWER_DOMAIN_TEST_CASE(node_id)                                                       \
	{DT_NODE_PATH(node_id), DEVICE_DT_GET(node_id), DT_REG_ADDR(node_id)},

/*
 * This covers every enabled SCMI PD node supplied by the R-Car X5H DTS.
 * The state test below only reads state, so it does not turn off domains used
 * by other peripherals.
 */
static const struct scmi_power_domain_test_case power_domains[] = {
	DT_FOREACH_CHILD_STATUS_OKAY(DT_PATH(power_domains), SCMI_POWER_DOMAIN_TEST_CASE)};

ZTEST(scmi_power_system, test_system_power_protocol_queries)
{
	uint32_t version;
	uint32_t attributes;

	zassert_ok(scmi_system_protocol_version(&version),
		   "System Power Protocol version query failed");
	zassert_not_equal(version, 0U, "System Power Protocol version is zero");

	zassert_ok(scmi_system_protocol_attributes(&attributes),
		   "System Power Protocol attributes query failed");
}

ZTEST(scmi_power_system, test_power_domain_states)
{
	for (size_t i = 0; i < ARRAY_SIZE(power_domains); i++) {
		const struct scmi_power_domain_test_case *test_case = &power_domains[i];
		enum pm_device_state pm_state;
		uint32_t scmi_state;

		zassert_true(device_is_ready(test_case->dev), "%s is not ready", test_case->path);
		zassert_ok(pm_device_state_get(test_case->dev, &pm_state), "%s has no PM state",
			   test_case->path);
		zassert_equal(pm_state, PM_DEVICE_STATE_ACTIVE, "%s is not active",
			      test_case->path);

		zassert_ok(scmi_power_state_get(test_case->id, &scmi_state),
			   "SCMI Power State Get failed for %s (ID %u)", test_case->path,
			   test_case->id);
		zassert_equal(scmi_state, SCMI_POWER_STATE_GENERIC_ON, "%s (ID %u) is not ON",
			      test_case->path, test_case->id);
	}
}

/*
 * VCN is currently an isolated power domain in the board DTS: it has no
 * parent power-domain and no enabled consumer. It is therefore suitable for
 * exercising a real PM transition without disrupting Ethernet or MP-PHY.
 */
ZTEST(scmi_power_system, test_vcn_power_domain_pm_resume)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(vcn_pd));
	enum pm_device_state pm_state;
	uint32_t scmi_state;

	zassert_true(device_is_ready(dev), "VCN power domain is not ready");
	zassert_ok(pm_device_state_get(dev, &pm_state), "VCN power domain has no PM state");
	zassert_equal(pm_state, PM_DEVICE_STATE_ACTIVE, "VCN power domain is not initially active");

	/* Suspend first so RESUME exercises the SCMI adapter callback. */
	zassert_ok(pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND),
		   "VCN power-domain suspend failed");
	zassert_ok(pm_device_state_get(dev, &pm_state),
		   "VCN power-domain state query failed after suspend");
	zassert_equal(pm_state, PM_DEVICE_STATE_SUSPENDED,
		      "VCN power domain did not become suspended");

	zassert_ok(pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME),
		   "VCN power-domain resume failed");
	zassert_ok(pm_device_state_get(dev, &pm_state),
		   "VCN power-domain state query failed after resume");
	zassert_equal(pm_state, PM_DEVICE_STATE_ACTIVE, "VCN power domain did not become active");

	zassert_ok(scmi_power_state_get(DT_REG_ADDR(DT_NODELABEL(vcn_pd)), &scmi_state),
		   "SCMI VCN power-state query failed");
	zassert_equal(scmi_state, SCMI_POWER_STATE_GENERIC_ON,
		      "SCMI VCN power domain is not ON after resume");
}

ZTEST_SUITE(scmi_power_system, NULL, NULL, NULL, NULL, NULL);
