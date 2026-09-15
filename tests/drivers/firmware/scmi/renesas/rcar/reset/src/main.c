/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/firmware/scmi/renesas/reset.h>
#include <zephyr/drivers/firmware/scmi/protocol.h>
#include <zephyr/drivers/firmware/scmi/reset.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/ztest.h>

#define SCMI_RESET_NODE DT_NODELABEL(scmi_reset)

/* SCP_RESET_DOMAIN_ID_SCIF0 in the R-Car X5H V1 SCP firmware. */
#define RCAR_X5H_SCMI_RESET_SCIF0 205U

ZTEST(scmi_reset, test_protocol_and_controller)
{
	const struct device *reset_dev = DEVICE_DT_GET(SCMI_RESET_NODE);
	struct scmi_protocol *proto = reset_dev->data;
	struct scmi_reset_domain_attr domain_attr;
	uint32_t version;
	uint32_t attributes;
	uint32_t num_domains;

	/* reset_scmi_init() has already queried PROTOCOL_ATTRIBUTES from SCP. */
	zassert_true(device_is_ready(reset_dev), "SCMI reset controller is not ready");
	zassert_equal(proto->id, SCMI_PROTOCOL_RESET_DOMAIN, "unexpected SCMI protocol ID: %u",
		      proto->id);

	zassert_ok(scmi_protocol_get_version(proto, &version),
		   "Reset protocol version query failed");
	zassert_equal(version, SCMI_RESET_PROTOCOL_SUPPORTED_VERSION,
		      "unexpected reset protocol version: 0x%08x", version);

	zassert_ok(scmi_protocol_attributes_get(proto, &attributes),
		   "Reset protocol attributes query failed");
	num_domains = SCMI_RESET_ATTR_GET_NUM_DOMAINS(attributes);
	zassert_not_equal(num_domains, 0U, "SCP reports no reset domains");

	/* Domain 0 is VIPN_FCPCS0 in the X5H SCP reset-domain table. */
	zassert_ok(scmi_reset_domain_get_attr(proto, 0U, &domain_attr),
		   "Reset domain 0 attributes query failed");
	zassert_not_equal(domain_attr.name[0], '\0', "Reset domain 0 has no name");

	/*
	 * Exercise the standard Zephyr reset-controller API without resetting
	 * hardware.  The driver rejects this out-of-range ID before it sends an
	 * SCMI RESET request.
	 */
	zassert_equal(reset_line_toggle(reset_dev, num_domains), -EINVAL,
		      "out-of-range reset ID was not rejected");
}

/*
 * SCIF1 is the R-Car R52 console and must not be reset by this test. SCIF0 is
 * unused by the board DTS, so it is the safe reset-domain request target.
 *
 * The reset operations below intentionally use the generic Zephyr Reset API.
 * The Renesas vendor status helper is used only as an observation mechanism;
 * the standard SCMI reset adapter does not implement reset_status().
 */
ZTEST(scmi_reset, test_scif0_reset_generic_api)
{
	const struct device *reset_dev = DEVICE_DT_GET(SCMI_RESET_NODE);
	struct scmi_protocol *proto = reset_dev->data;
	struct scmi_reset_domain_attr domain_attr;
	bool reset_asserted;

	zassert_true(device_is_ready(reset_dev), "SCMI reset controller is not ready");
	zassert_ok(scmi_reset_domain_get_attr(proto, RCAR_X5H_SCMI_RESET_SCIF0, &domain_attr),
		   "SCIF0 reset domain attributes query failed");
	zassert_ok(reset_line_assert(reset_dev, RCAR_X5H_SCMI_RESET_SCIF0),
		   "SCIF0 reset assert failed");
	zassert_ok(scmi_renesas_reset_status_get(RCAR_X5H_SCMI_RESET_SCIF0, &reset_asserted),
		   "SCIF0 reset status query failed");
	zassert_true(reset_asserted, "SCIF0 reset was not asserted");

	zassert_ok(reset_line_deassert(reset_dev, RCAR_X5H_SCMI_RESET_SCIF0),
		   "SCIF0 reset deassert failed");
	zassert_ok(scmi_renesas_reset_status_get(RCAR_X5H_SCMI_RESET_SCIF0, &reset_asserted),
		   "SCIF0 reset status query failed");
	zassert_false(reset_asserted, "SCIF0 reset remains asserted after deassert");

	zassert_ok(reset_line_toggle(reset_dev, RCAR_X5H_SCMI_RESET_SCIF0),
		   "SCIF0 reset toggle failed");
	zassert_ok(scmi_renesas_reset_status_get(RCAR_X5H_SCMI_RESET_SCIF0, &reset_asserted),
		   "SCIF0 reset status query failed");
	zassert_false(reset_asserted, "SCIF0 reset remains asserted after toggle");
}

ZTEST_SUITE(scmi_reset, NULL, NULL, NULL, NULL, NULL);
