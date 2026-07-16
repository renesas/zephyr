/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/usb/class/hid.h>

#define HID_TEST_MAX_VALUES 8u

struct hid_report_suite_fixture {
	struct hid_report report;
};

struct hid_test_values {
	size_t num_values;
	uint32_t values[HID_TEST_MAX_VALUES];
};

static void *suite_setup(void)
{
	static struct hid_report_suite_fixture fixture = {};
	return &fixture;
}

static void suite_before(void *fixture)
{
	memset(fixture, 0, sizeof(struct hid_report_suite_fixture));
}

ZTEST_SUITE(hid_report_suite, NULL, suite_setup, suite_before, NULL, NULL);

static int field_inspector(struct hid_report_field const *field, uint8_t report_id,
			   uint8_t const *data, size_t bit_index, void *user_data)
{
	size_t value_index = bit_index / 8;
	size_t value_bit_shift = bit_index % 8;
	int32_t i32_value = 0;
	uint32_t u32_value = 0;
	struct hid_test_values *test_values = user_data;

	/* Keyboard input */
	if (hid_report_match_usage_page(field, HID_USAGE_GEN_KEYBOARD)) {
		/* Keyboard array, each element is a button press */
		if (HID_REPORT_DATA_IS_ARRAY(field->flags) && field->size == 8u) {
			for (size_t key_index = value_index; key_index < field->count;
			     key_index++) {
				if (data[key_index] != 0u) {
					test_values->values[test_values->num_values] =
						data[key_index];
					test_values->num_values++;
				}
			}

		}
		/* Keyboard variable, mostly for modifiers */
		else if (HID_REPORT_DATA_IS_VARIABLE(field->flags) && field->size == 1u) {
			for (size_t key_position = 0u; key_position < field->count;
			     key_position++) {
				if (data[value_index] & (1u << key_position)) {
					test_values->values[test_values->num_values] =
						(field->usage_minimum & 0xFFFF) + key_position;
					test_values->num_values++;
				}
			}
		}
	}
	/* Mouse button input */
	if (hid_report_get_usage_id_u32(field, &data[value_index], value_bit_shift,
					HID_USAGE_ID(HID_USAGE_GEN_BUTTON, 1u), &u32_value) == 0) {
		test_values->values[test_values->num_values] = u32_value;
		test_values->num_values++;
	}
	/* Mouse X movement */
	if (hid_report_get_usage_id_i32(
		    field, &data[value_index], value_bit_shift,
		    HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_X),
		    &i32_value) == 0) {
		test_values->values[test_values->num_values] = i32_value;
		test_values->num_values++;
	}
	/* Mouse Y movement */
	if (hid_report_get_usage_id_i32(
		    field, &data[value_index], value_bit_shift,
		    HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_Y),
		    &i32_value) == 0) {
		test_values->values[test_values->num_values] = i32_value;
		test_values->num_values++;
	}
	/* Mouse wheel */
	if (hid_report_get_usage_id_i32(
		    field, &data[value_index], value_bit_shift,
		    HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_WHEEL),
		    &i32_value) == 0) {
		test_values->values[test_values->num_values] = i32_value;
		test_values->num_values++;
	}

	return 0;
}

ZTEST_F(hid_report_suite, test_hid_mouse1_report)
{
	struct hid_report *report = &fixture->report;
	struct hid_test_values test_values = {};
	size_t collection_index = 0;
	size_t report_index = 0;
	size_t field_index = 0;

	/* Complex mouse */
	uint8_t report_data[] = {
		0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x85, 0x02, 0x09, 0x01, 0xa1, 0x00, 0x05, 0x09,
		0x19, 0x01, 0x29, 0x10, 0x15, 0x00, 0x25, 0x01, 0x95, 0x10, 0x75, 0x01, 0x81, 0x02,
		0x05, 0x01, 0x16, 0x01, 0xf8, 0x26, 0xff, 0x07, 0x75, 0x0c, 0x95, 0x02, 0x09, 0x30,
		0x09, 0x31, 0x81, 0x06, 0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x01, 0x09, 0x38,
		0x81, 0x06, 0x05, 0x0c, 0x0a, 0x38, 0x02, 0x95, 0x01, 0x81, 0x06, 0xc0, 0xc0, 0x05,
		0x0c, 0x09, 0x01, 0xa1, 0x01, 0x85, 0x03, 0x75, 0x10, 0x95, 0x02, 0x15, 0x01, 0x26,
		0x8c, 0x02, 0x19, 0x01, 0x2a, 0x8c, 0x02, 0x81, 0x00, 0xc0, 0x05, 0x01, 0x09, 0x80,
		0xa1, 0x01, 0x85, 0x04, 0x75, 0x02, 0x95, 0x01, 0x15, 0x01, 0x25, 0x03, 0x09, 0x82,
		0x09, 0x81, 0x09, 0x83, 0x81, 0x60, 0x75, 0x06, 0x81, 0x03, 0xc0, 0x06, 0x00, 0xff,
		0x09, 0x01, 0xa1, 0x01, 0x85, 0x10, 0x75, 0x08, 0x95, 0x06, 0x15, 0x00, 0x26, 0xff,
		0x00, 0x09, 0x01, 0x81, 0x00, 0x09, 0x01, 0x91, 0x00, 0xc0, 0x06, 0x00, 0xff, 0x09,
		0x02, 0xa1, 0x01, 0x85, 0x11, 0x75, 0x08, 0x95, 0x13, 0x15, 0x00, 0x26, 0xff, 0x00,
		0x09, 0x02, 0x81, 0x00, 0x09, 0x02, 0x91, 0x00, 0xc0,
	};

	int result = hid_report_parse(report, sizeof(report_data), report_data);
	zassert_equal(0, result, "Could not parse report descriptor");

	/* Collections */
	zassert_equal(6u, report->num_collections,
		      "Wrong number of collections in report descriptor");
	collection_index = 0u;

	/* First collection, application */
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_MOUSE),
		      report->collections[collection_index].usage, "Wrong collection usage ID");
	zassert_equal(HID_COLLECTION_APPLICATION, report->collections[collection_index].type,
		      "Wrong collection type");
	zassert_equal(0u, report->collections[collection_index].start,
		      "Wrong collection starting point");
	zassert_equal(4u, report->collections[collection_index].end, "Wrong collection end point");
	collection_index++;

	/* Second collection, physical */
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_POINTER),
		      report->collections[collection_index].usage, "Wrong collection usage ID");
	zassert_equal(HID_COLLECTION_PHYSICAL, report->collections[collection_index].type,
		      "Wrong collection type");
	zassert_equal(0u, report->collections[collection_index].start,
		      "Wrong collection starting point");
	zassert_equal(4u, report->collections[collection_index].end, "Wrong collection end point");
	collection_index++;

	/* Third collection, application */
	zassert_equal(HID_USAGE_ID(HID_USAGE_CONSUMER, HID_USAGE_CONSUMER_CONTROL),
		      report->collections[collection_index].usage, "Wrong collection usage ID");
	zassert_equal(HID_COLLECTION_APPLICATION, report->collections[collection_index].type,
		      "Wrong collection type");
	zassert_equal(4u, report->collections[collection_index].start,
		      "Wrong collection starting point");
	zassert_equal(5u, report->collections[collection_index].end, "Wrong collection end point");
	collection_index++;

	/* Fourth collection, application */
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_SYSTEM_CONTROL),
		      report->collections[collection_index].usage, "Wrong collection usage ID");
	zassert_equal(HID_COLLECTION_APPLICATION, report->collections[collection_index].type,
		      "Wrong collection type");
	zassert_equal(5u, report->collections[collection_index].start,
		      "Wrong collection starting point");
	zassert_equal(7u, report->collections[collection_index].end, "Wrong collection end point");
	collection_index++;

	/* Fifth collection, application */
	zassert_equal(HID_USAGE_ID(0xFF00u, 1u), report->collections[collection_index].usage,
		      "Wrong collection usage ID");
	zassert_equal(HID_COLLECTION_APPLICATION, report->collections[collection_index].type,
		      "Wrong collection type");
	zassert_equal(7u, report->collections[collection_index].start,
		      "Wrong collection starting point");
	zassert_equal(9u, report->collections[collection_index].end, "Wrong collection end point");
	collection_index++;

	/* Sixth collection, application */
	zassert_equal(HID_USAGE_ID(0xFF00u, 2u), report->collections[collection_index].usage,
		      "Wrong collection usage ID");
	zassert_equal(HID_COLLECTION_APPLICATION, report->collections[collection_index].type,
		      "Wrong collection type");
	zassert_equal(9u, report->collections[collection_index].start,
		      "Wrong collection starting point");
	zassert_equal(11u, report->collections[collection_index].end, "Wrong collection end point");
	collection_index++;

	/* Reports */
	zassert_equal(5u, report->num_reports, "Wrong number of report variants");
	report_index = 0u;

	zassert_equal(2u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(0u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(3u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(4u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(4u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(5u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(16u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(7u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(17u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(9u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	/* Fields and report parsing */
	zassert_equal(11u, report->num_fields, "Wrong number of fields");
	field_index = 0u;

	/* First field, buttons */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly constant");
	zassert_equal(16u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(1u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_BUTTON, 0x1),
		      report->fields[field_index].usage_minimum, "Wrong field usage ID");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_BUTTON, 0x10),
		      report->fields[field_index].usage_maximum, "Wrong field usage ID");
	zassert_equal(0, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(1, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	/* Second field, X/Y */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly constant");
	zassert_equal(2u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(12u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_X),
		      report->fields[field_index].usages[0u], "Wrong field usage ID");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_Y),
		      report->fields[field_index].usages[1u], "Wrong field usage ID");
	zassert_equal(-2047, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(2047, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	/* Third field, wheel */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly constant");
	zassert_equal(1u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(8u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_WHEEL),
		      report->fields[field_index].usages[0u], "Wrong field usage ID");
	zassert_equal(-127, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(127, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	/* Fourth field, AC pan */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly constant");
	zassert_equal(1u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(8u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_CONSUMER, HID_USAGE_CONSUMER_AC_PAN),
		      report->fields[field_index].usages[0u], "Wrong field usage ID");
	zassert_equal(-127, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(127, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	uint8_t const data[] = {0x02u, 0x01u, 0x00u, 0xFEu, 0x1Fu, 0x00u, 0x10u, 0x00u};
	hid_report_input_iterate(report, sizeof(data), data, field_inspector, &test_values);

	zassert_equal(4u, test_values.num_values, "Wrong number of values");
	/* First button  */
	zassert_equal(1, test_values.values[0u], "Wrong value for first button");
	/* X relative movement */
	zassert_equal(-2, test_values.values[1u], "Wrong value for X movement");
	/* Y relative movement */
	zassert_equal(1, test_values.values[2u], "Wrong value for Y movement");
	/* Scroll */
	zassert_equal(16, test_values.values[3u], "Wrong scroll value");
}

ZTEST_F(hid_report_suite, test_hid_mouse2_report)
{
	struct hid_report *report = &fixture->report;
	struct hid_test_values test_values = {};
	size_t collection_index = 0u;
	size_t field_index = 0u;

	uint8_t const report_data[] = {
		0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01, 0xa1, 0x00, 0x05, 0x09, 0x15,
		0x00, 0x25, 0x01, 0x19, 0x01, 0x29, 0x05, 0x75, 0x01, 0x95, 0x05, 0x81, 0x02,
		0x95, 0x03, 0x81, 0x01, 0x05, 0x01, 0x16, 0x01, 0x80, 0x26, 0xff, 0x7f, 0x09,
		0x30, 0x09, 0x31, 0x75, 0x10, 0x95, 0x02, 0x81, 0x06, 0x15, 0x81, 0x25, 0x7f,
		0x09, 0x38, 0x75, 0x08, 0x95, 0x01, 0x81, 0x06, 0x05, 0x0c, 0x0a, 0x38, 0x02,
		0x95, 0x01, 0x81, 0x06, 0x06, 0x00, 0xff, 0x09, 0x20, 0x15, 0x00, 0x26, 0xff,
		0x00, 0x95, 0x40, 0xb1, 0x02, 0xc0, 0xc0,
	};

	int result = hid_report_parse(report, sizeof(report_data), report_data);
	zassert_equal(0, result, "Could not parse the report descriptor");

	/* Collections */
	zassert_equal(2u, report->num_collections,
		      "Wrong number of collections in report descriptor");
	collection_index = 0u;

	/* First collection, application */
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_MOUSE),
		      report->collections[collection_index].usage, "Wrong collection usage ID");
	zassert_equal(HID_COLLECTION_APPLICATION, report->collections[collection_index].type,
		      "Wrong collection type");
	zassert_equal(0u, report->collections[collection_index].start,
		      "Wrong collection starting point");
	zassert_equal(6u, report->collections[collection_index].end, "Wrong collection end point");
	collection_index++;

	/* Second collection, physical */
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_POINTER),
		      report->collections[collection_index].usage, "Wrong collection usage ID");
	zassert_equal(HID_COLLECTION_PHYSICAL, report->collections[collection_index].type,
		      "Wrong collection type");
	zassert_equal(0u, report->collections[collection_index].start,
		      "Wrong collection starting point");
	zassert_equal(6u, report->collections[collection_index].end, "Wrong collection end point");
	collection_index++;

	/* Reports */
	zassert_equal(0u, report->num_reports, "Wrong number of report variants");

	/* Fields and report parsing */
	zassert_equal(6u, report->num_fields, "Wrong number of fields");
	field_index = 0u;

	/* First field, buttons */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly an array");
	zassert_equal(5u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(1u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_BUTTON, 0x1),
		      report->fields[field_index].usage_minimum, "Wrong field usage ID");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_BUTTON, 0x5),
		      report->fields[field_index].usage_maximum, "Wrong field usage ID");
	zassert_equal(0, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(1, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	/* Second field, padding */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_equal(3u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(1u, report->fields[field_index].size, "Wrong field report size");
	zassert_true(HID_REPORT_DATA_IS_CONSTANT(report->fields[field_index].flags),
		     "Field is unexpectedly mutable");
	field_index++;

	/* Third field, X/Y */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly constant");
	zassert_equal(2u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(16u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_X),
		      report->fields[field_index].usages[0u], "Wrong field usage ID");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_Y),
		      report->fields[field_index].usages[1u], "Wrong field usage ID");
	zassert_equal(-32767, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(32767, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	/* Fourth field, wheel */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly constant");
	zassert_equal(1u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(8u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_WHEEL),
		      report->fields[field_index].usages[0u], "Wrong field usage ID");
	zassert_equal(-127, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(127, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	/* Fifth field, AC pan */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly constant");
	zassert_equal(1u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(8u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_CONSUMER, HID_USAGE_CONSUMER_AC_PAN),
		      report->fields[field_index].usages[0u], "Wrong field usage ID");
	zassert_equal(-127, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(127, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	/* Sixth field, vendor specific feature */
	zassert_equal(HID_REPORT_FIELD_TYPE_FEATURE, report->fields[field_index].type,
		      "Wrong field type");
	zassert_equal(64u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(8u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(0xFF00, 0x20), report->fields[field_index].usages[0u],
		      "Wrong field usage ID");
	zassert_equal(0, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(255, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	{
		/* Scroll forward by 1 */
		uint8_t const data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};

		memset(&test_values, 0, sizeof(test_values));
		hid_report_input_iterate(report, sizeof(data), data, field_inspector, &test_values);

		zassert_equal(4u, test_values.num_values, "Wrong number of values");
		zassert_equal(1u, test_values.values[3], "Wrong scroll value");
	}

	{
		/* Scroll backwards by 1 */
		uint8_t const data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00};

		memset(&test_values, 0, sizeof(test_values));
		hid_report_input_iterate(report, sizeof(data), data, field_inspector, &test_values);

		zassert_equal(4u, test_values.num_values, "Wrong number of values");
		zassert_equal(-1u, test_values.values[3], "Wrong scroll value");
	}
}

ZTEST_F(hid_report_suite, test_hid_mouse3_report)
{
	struct hid_report *report = &fixture->report;
	struct hid_test_values test_values = {};
	size_t collection_index = 0;
	size_t report_index = 0;
	size_t field_index = 0;

	uint8_t const report_data[] = {
		0x06, 0xb5, 0xff, 0x09, 0x01, 0xa1, 0x01, 0x85, 0xb5, 0x09, 0x02, 0x15, 0x00, 0x26,
		0xff, 0x00, 0x75, 0x08, 0x95, 0x07, 0x81, 0x02, 0x09, 0x02, 0x15, 0x00, 0x26, 0xff,
		0x00, 0x75, 0x08, 0x95, 0x07, 0x91, 0x02, 0xc0, 0x05, 0x01, 0x09, 0x02, 0xa1, 0x01,
		0x85, 0x02, 0x09, 0x01, 0xa1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x08, 0x15, 0x00,
		0x25, 0x01, 0x95, 0x08, 0x75, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31,
		0x16, 0x01, 0xf8, 0x26, 0xff, 0x07, 0x75, 0x0c, 0x95, 0x02, 0x81, 0x06, 0x09, 0x38,
		0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x01, 0x81, 0x06, 0x05, 0x0c, 0x0a, 0x38,
		0x02, 0x95, 0x01, 0x81, 0x06, 0xc0, 0xc0,
	};

	int result = hid_report_parse(report, sizeof(report_data), report_data);
	zassert_equal(0, result);

	/* Collections */
	zassert_equal(3u, report->num_collections,
		      "Wrong number of collections in report descriptor");
	collection_index = 0u;

	/* First collection, application, vendor defined */
	zassert_equal(HID_USAGE_ID(0xFFB5, HID_USAGE_GEN_DESKTOP_POINTER),
		      report->collections[collection_index].usage, "Wrong collection usage ID");
	zassert_equal(HID_COLLECTION_APPLICATION, report->collections[collection_index].type,
		      "Wrong collection type");
	zassert_equal(0u, report->collections[collection_index].start,
		      "Wrong collection starting point");
	zassert_equal(2u, report->collections[collection_index].end, "Wrong collection end point");
	collection_index++;

	/* Second collection, application */
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_MOUSE),
		      report->collections[collection_index].usage, "Wrong collection usage ID");
	zassert_equal(HID_COLLECTION_APPLICATION, report->collections[collection_index].type,
		      "Wrong collection type");
	zassert_equal(2u, report->collections[collection_index].start,
		      "Wrong collection starting point");
	zassert_equal(6u, report->collections[collection_index].end, "Wrong collection end point");
	collection_index++;

	/* Third collection, physical */
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_POINTER),
		      report->collections[collection_index].usage, "Wrong collection usage ID");
	zassert_equal(HID_COLLECTION_PHYSICAL, report->collections[collection_index].type,
		      "Wrong collection type");
	zassert_equal(2u, report->collections[collection_index].start,
		      "Wrong collection starting point");
	zassert_equal(6u, report->collections[collection_index].end, "Wrong collection end point");
	collection_index++;

	/* Reports */
	zassert_equal(2u, report->num_reports, "Wrong number of report variants");
	report_index = 0u;

	zassert_equal(0xB5u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(0u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(2u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(2u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	/* Fields and report parsing */
	zassert_equal(6u, report->num_fields, "Wrong number of fields");
	field_index = 0u;

	/* First field, vendor specific */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly constant");
	zassert_equal(7u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(8u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(0xFFB5, 0x2), report->fields[field_index].usages[0],
		      "Wrong field usage ID");
	zassert_equal(0, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(255, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	/* Second field, vendor specific */
	zassert_equal(HID_REPORT_FIELD_TYPE_OUTPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly constant");
	zassert_equal(7u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(8u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(0xFFB5, 0x02), report->fields[field_index].usages[0u],
		      "Wrong field usage ID");
	zassert_equal(0, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(255, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	/* Third field, buttons */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly constant");
	zassert_equal(8u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(1u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_BUTTON, 0x01),
		      report->fields[field_index].usage_minimum, "Wrong field usage ID");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_BUTTON, 0x08),
		      report->fields[field_index].usage_maximum, "Wrong field usage ID");
	zassert_equal(0, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(1, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	/* Fourth field, X/Y */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly constant");
	zassert_equal(2u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(12u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_X),
		      report->fields[field_index].usages[0u], "Wrong field usage ID");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_Y),
		      report->fields[field_index].usages[1u], "Wrong field usage ID");
	zassert_equal(-2047, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(2047, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	/* Fifth field, wheel */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly constant");
	zassert_equal(1u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(8u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_WHEEL),
		      report->fields[field_index].usages[0u], "Wrong field usage ID");
	zassert_equal(-127, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(127, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	/* Sixth field, AC pan */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly constant");
	zassert_equal(1u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(8u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_CONSUMER, HID_USAGE_CONSUMER_AC_PAN),
		      report->fields[field_index].usages[0u], "Wrong field usage ID");
	zassert_equal(-127, report->fields[field_index].logical_minimum,
		      "Wrong field logical minimum");
	zassert_equal(127, report->fields[field_index].logical_maximum,
		      "Wrong field logical maximum");
	field_index++;

	{
		memset(&test_values, 0, sizeof(test_values));
		uint8_t const data[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
		hid_report_input_iterate(report, sizeof(data), data, field_inspector, &test_values);

		zassert_equal(4u, test_values.num_values);
		zassert_equal(1u, test_values.values[3]);
	}

	{
		memset(&test_values, 0, sizeof(test_values));
		uint8_t const data[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00};
		hid_report_input_iterate(report, sizeof(data), data, field_inspector, &test_values);

		zassert_equal(4u, test_values.num_values);
		zassert_equal(-1u, test_values.values[3]);
	}
}

ZTEST_F(hid_report_suite, test_hid_gamepad_parsing)
{
	struct hid_report *report = &fixture->report;
	size_t collection_index = 0;
	size_t report_index = 0;

	uint8_t const report_data[] = {
		0x05, 0x01, 0x09, 0x05, 0xa1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32,
		0x09, 0x35, 0x09, 0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95,
		0x06, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x20, 0x95, 0x01, 0x81, 0x02, 0x05, 0x01,
		0x09, 0x39, 0x15, 0x00, 0x25, 0x07, 0x35, 0x00, 0x46, 0x3b, 0x01, 0x65, 0x14, 0x75,
		0x04, 0x95, 0x01, 0x81, 0x42, 0x65, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0f, 0x15,
		0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x0f, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x21,
		0x95, 0x0d, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x22, 0x15, 0x00, 0x26, 0xff, 0x00,
		0x75, 0x08, 0x95, 0x34, 0x81, 0x02, 0x85, 0x02, 0x09, 0x23, 0x95, 0x2f, 0x91, 0x02,
		0x85, 0x05, 0x09, 0x33, 0x95, 0x28, 0xb1, 0x02, 0x85, 0x08, 0x09, 0x34, 0x95, 0x2f,
		0xb1, 0x02, 0x85, 0x09, 0x09, 0x24, 0x95, 0x13, 0xb1, 0x02, 0x85, 0x0a, 0x09, 0x25,
		0x95, 0x1a, 0xb1, 0x02, 0x85, 0x0b, 0x09, 0x41, 0x95, 0x29, 0xb1, 0x02, 0x85, 0x0c,
		0x09, 0x42, 0x95, 0x29, 0xb1, 0x02, 0x85, 0x20, 0x09, 0x26, 0x95, 0x3f, 0xb1, 0x02,
		0x85, 0x21, 0x09, 0x27, 0x95, 0x04, 0xb1, 0x02, 0x85, 0x22, 0x09, 0x40, 0x95, 0x3f,
		0xb1, 0x02, 0x85, 0x80, 0x09, 0x28, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0x81, 0x09, 0x29,
		0x95, 0x3f, 0xb1, 0x02, 0x85, 0x82, 0x09, 0x2a, 0x95, 0x09, 0xb1, 0x02, 0x85, 0x83,
		0x09, 0x2b, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0x84, 0x09, 0x2c, 0x95, 0x3f, 0xb1, 0x02,
		0x85, 0x85, 0x09, 0x2d, 0x95, 0x02, 0xb1, 0x02, 0x85, 0xa0, 0x09, 0x2e, 0x95, 0x01,
		0xb1, 0x02, 0x85, 0xe0, 0x09, 0x2f, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf0, 0x09, 0x30,
		0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf1, 0x09, 0x31, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf2,
		0x09, 0x32, 0x95, 0x0f, 0xb1, 0x02, 0x85, 0xf4, 0x09, 0x35, 0x95, 0x3f, 0xb1, 0x02,
		0x85, 0xf5, 0x09, 0x36, 0x95, 0x03, 0xb1, 0x02, 0xc0,
	};

	int result = hid_report_parse(report, sizeof(report_data), report_data);
	zassert_equal(0, result);

	/* Collections */
	zassert_equal(1u, report->num_collections,
		      "Wrong number of collections in report descriptor");
	collection_index = 0u;

	/* First collection, application, vendor defined */
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_GAMEPAD),
		      report->collections[collection_index].usage, "Wrong collection usage ID");
	zassert_equal(HID_COLLECTION_APPLICATION, report->collections[collection_index].type,
		      "Wrong collection type");
	zassert_equal(0u, report->collections[collection_index].start,
		      "Wrong collection starting point");
	zassert_equal(29u, report->collections[collection_index].end, "Wrong collection end point");
	collection_index++;

	/* Reports */
	zassert_equal(24u, report->num_reports, "Wrong number of report variants");
	report_index = 0u;

	zassert_equal(0x01u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(0u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x02u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(6u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x05u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(7u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x08u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(8u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x09u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(9u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x0Au, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(10u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x0Bu, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(11u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x0Cu, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(12u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x20u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(13u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x21u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(14u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x22u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(15u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x80u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(16u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x81u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(17u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x82u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(18u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x83u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(19u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x84u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(20u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0x85u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(21u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0xA0u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(22u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0xE0u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(23u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0xF0u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(24u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0xF1u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(25u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0xF2u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(26u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0xF4u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(27u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;

	zassert_equal(0xF5u, report->reports[report_index].id, "Wrong report ID");
	zassert_equal(28u, report->reports[report_index].start, "Wrong report starting point");
	report_index++;
}

ZTEST_F(hid_report_suite, test_hid_keyboard_report)
{
	struct hid_report *report = &fixture->report;
	struct hid_test_values test_values = {};
	size_t collection_index = 0;
	size_t field_index = 0;

	/* Keyboard */
	uint8_t report_data[] = {
		0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x95, 0x08, 0x75, 0x01, 0x15, 0x00,
		0x25, 0x01, 0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7, 0x81, 0x02, 0x81, 0x03,
		0x95, 0x05, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02, 0x95, 0x01,
		0x75, 0x03, 0x91, 0x01, 0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x26, 0xff,
		0x00, 0x05, 0x07, 0x19, 0x00, 0x2a, 0xff, 0x00, 0x81, 0x00, 0xc0,
	};

	int result = hid_report_parse(report, sizeof(report_data), report_data);
	zassert_equal(0, result, "Could not parse report descriptor");

	/* Collections */
	zassert_equal(1u, report->num_collections,
		      "Wrong number of collections in report descriptor");
	collection_index = 0u;

	/* First collection, application */
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP, HID_USAGE_GEN_DESKTOP_KEYBOARD),
		      report->collections[collection_index].usage, "Wrong collection usage ID");
	zassert_equal(HID_COLLECTION_APPLICATION, report->collections[collection_index].type,
		      "Wrong collection type");
	zassert_equal(0u, report->collections[collection_index].start,
		      "Wrong collection starting point");
	zassert_equal(5u, report->collections[collection_index].end, "Wrong collection end point");
	collection_index++;

	/* Reports */
	zassert_equal(0u, report->num_reports, "There should be no report variants");

	/* Fields and report parsing */
	zassert_equal(5u, report->num_fields, "Wrong number of fields");
	field_index = 0u;

	/* First field, modifiers */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly an array");
	zassert_equal(8u, report->fields[field_index].count, "Wrong field report size");
	zassert_equal(1u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP_KEYPAD,
				   HID_USAGE_GEN_DESKTOP_KEYBOARD_LEFT_CTRL),
		      report->fields[field_index].usage_minimum, "Wrong field usage ID");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP_KEYPAD,
				   HID_USAGE_GEN_DESKTOP_KEYBOARD_RIGHT_GUI),
		      report->fields[field_index].usage_maximum, "Wrong field usage ID");
	field_index++;

	/* Second field, padding */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_VARIABLE(report->fields[field_index].flags),
		     "Field is unexpectedly an array");
	zassert_equal(8u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(1u, report->fields[field_index].size, "Wrong field report size");
	zassert_true(HID_REPORT_DATA_IS_CONSTANT(report->fields[field_index].flags),
		     "Field is unexpectedly mutable");
	field_index++;

	/* Third field, LEDs */
	zassert_equal(HID_REPORT_FIELD_TYPE_OUTPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_equal(5u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(1u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_LEDS, 1),
		      report->fields[field_index].usage_minimum, "Wrong field usage ID");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_LEDS, 5),
		      report->fields[field_index].usage_maximum, "Wrong field usage ID");
	field_index++;

	/* Fourth field, padding */
	zassert_equal(HID_REPORT_FIELD_TYPE_OUTPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_equal(1u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(3u, report->fields[field_index].size, "Wrong field report size");
	zassert_true(HID_REPORT_DATA_IS_CONSTANT(report->fields[field_index].flags),
		     "Field is unexpectedly mutable");
	field_index++;

	/* Fifth field, keys */
	zassert_equal(HID_REPORT_FIELD_TYPE_INPUT, report->fields[field_index].type,
		      "Wrong field type");
	zassert_true(HID_REPORT_DATA_IS_ARRAY(report->fields[field_index].flags),
		     "Field is unexpectedly a variable");
	zassert_equal(6u, report->fields[field_index].count, "Wrong field report count");
	zassert_equal(8u, report->fields[field_index].size, "Wrong field report size");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP_KEYPAD, 0u),
		      report->fields[field_index].usage_minimum, "Wrong field usage ID");
	zassert_equal(HID_USAGE_ID(HID_USAGE_GEN_DESKTOP_KEYPAD, 255u),
		      report->fields[field_index].usage_maximum, "Wrong field usage ID");
	field_index++;

	uint8_t const data[] = {0x01u, 0x00u, 0x1Bu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};
	hid_report_input_iterate(report, sizeof(data), data, field_inspector, &test_values);

	zassert_equal(2u, test_values.num_values);
	/* First button  */
	zassert_equal(HID_USAGE_GEN_DESKTOP_KEYBOARD_LEFT_CTRL, test_values.values[0u]);
	/* Second button */
	zassert_equal(0x1Bu, test_values.values[1u]);
}
