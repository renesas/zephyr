/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <zephyr/usb/class/hid.h>

/* Wether the item is long or short, given its first byte */
#define ITEM_IS_LONG(First)  ((First) == 0xFE)
#define ITEM_IS_SHORT(First) !ITEM_IS_LONG(First)
/* Get the tag field of the item's first byte */
#define ITEM_TAG(Tag)        (((Tag) & 0xF0) >> 4)
/* Get the type field of the item's first byte */
#define ITEM_TYPE(Tag)       (((Tag) & 0x0C) >> 2)
/* Get the size field of the item's first byte */
#define ITEM_SIZE(Tag)       ((Tag) & 0x03)

enum item_type {
	ITEM_TYPE_MAIN = 0,
	ITEM_TYPE_GLOBAL = 1,
	ITEM_TYPE_LOCAL = 2,
};

/* The size of short items can be of either 0, 1, 2 or 4 bytes, skipping 3. This enumeration
 * reflects that */
enum item_data_size {
	ITEM_DATA_SIZE_0 = 0,
	ITEM_DATA_SIZE_1 = 1,
	ITEM_DATA_SIZE_2 = 2,
	ITEM_DATA_SIZE_4 = 3,
};

/* Possible main items and associated tag value */
enum main_item_tag {
	MAIN_ITEM_TAG_INPUT = 0x8,
	MAIN_ITEM_TAG_OUTPUT = 0x9,
	MAIN_ITEM_TAG_COLLECTION = 0xA,
	MAIN_ITEM_TAG_FEATURE = 0xB,
	MAIN_ITEM_TAG_COLLECTION_END = 0xC,
};

/* Possible global items and associated tag value */
enum global_item_tag {
	GLOBAL_ITEM_TAG_USAGE_PAGE = 0x0,
	GLOBAL_ITEM_TAG_LOGICAL_MINIMUM = 0x1,
	GLOBAL_ITEM_TAG_LOGICAL_MAXIMUM = 0x2,
	GLOBAL_ITEM_TAG_PHYSICAL_MINIMUM = 0x3,
	GLOBAL_ITEM_TAG_PHYSICAL_MAXIMUM = 0x4,
	GLOBAL_ITEM_TAG_UNIT_EXPONENT = 0x5,
	GLOBAL_ITEM_TAG_UNIT = 0x6,
	GLOBAL_ITEM_TAG_REPORT_SIZE = 0x7,
	GLOBAL_ITEM_TAG_REPORT_ID = 0x8,
	GLOBAL_ITEM_TAG_REPORT_COUNT = 0x9,
	GLOBAL_ITEM_TAG_PUSH = 0xA,
	GLOBAL_ITEM_TAG_POP = 0xB,
};

/* Possible local items and associated tag value */
enum local_item_tag {
	LOCAL_ITEM_TAG_USAGE = 0x0,
	LOCAL_ITEM_TAG_USAGE_MINIMUM = 0x1,
	LOCAL_ITEM_TAG_USAGE_MAXIMUM = 0x2,
	LOCAL_ITEM_TAG_DESIGNATOR_INDEX = 0x3,
	LOCAL_ITEM_TAG_DESIGNATOR_MINIMUM = 0x4,
	LOCAL_ITEM_TAG_DESIGNATOR_MAXIMUM = 0x5,
	LOCAL_ITEM_TAG_STRING_INDEX = 0x7,
	LOCAL_ITEM_TAG_STRING_MINIMUM = 0x8,
	LOCAL_ITEM_TAG_STRING_MAXIMUM = 0x9,
	LOCAL_ITEM_TAG_DELIMITER = 0xA,
};

/* Global state during report descriptor parsing */
struct global_state {
	uint32_t usage_page;

	/* Minimum reportable value */
	int32_t logical_minimum;
	/* Maximum reportable value */
	int32_t logical_maximum;

	/* Minimum value in units */
	int32_t physical_minimum;
	/* Maximum value in units */
	int32_t physical_maximum;
	/* Value of this unit exponent in base 10 */
	uint32_t unit_exponent;
	/* Unit values */
	uint32_t unit;

	/* Size of this field */
	uint32_t report_size;
	/* Number of instances of this field */
	uint32_t report_count;
};

struct context {
	/* A stack of indexes pointing to the report's collection list */
	size_t collection_stack_index;
	size_t collection_stack[CONFIG_USBH_HID_REPORT_MAX_COLLECTIONS];

	/* Local state, reset at each main item, to be used during parsing */
	struct {
		/* Usage range */
		uint32_t usage_minimum;
		uint32_t usage_maximum;

		/* Index of the physical body part related to this field */
		uint32_t designator_index;
		/* Starting designator  */
		uint32_t designator_minimum;
		/* End designator  */
		uint32_t designator_maximum;

		/* Index of a string descriptor describing this field */
		uint32_t string_index;
		/* Starting string index */
		uint32_t string_minimum;
		/* End string index */
		uint32_t string_maximum;

		/* Pseudo-stack of usage IDs */
		size_t num_usages;
		uint32_t usages[CONFIG_USBH_HID_REPORT_MAX_USAGES];
	} local_state;

	/* Global state stack used during parsing. The stack can stores copies of the global state,
	 * to be pushed and popped from and to `global_state` */
	size_t state_stack_index;
	struct global_state state_stack[CONFIG_USBH_HID_REPORT_MAX_STATE_STACK];
	struct global_state global_state;

	/* Pointer to the output report structure */
	struct hid_report *report;

	/* Prasing cursor (i.e. byte index for the data array) */
	size_t cursor;
	/* Length of the report descriptor data */
	size_t data_length;
	/* Raw report descriptor */
	uint8_t const *data;
};

/**
 * @brief Checks that the item currently under cursor can fit in the remaining data
 *
 * @param context Parsing context
 *
 * @retval A positive integer represents the length of the item
 * @retval -EINVAL if there is not enough data for the item
 */
static int check_size(struct context *context);

/**
 * @brief Reads a usage page under the cursor position into the context
 *
 * @param context Parsing context
 *
 * @retval 0 if successful
 * @retval -EINVAL if the usage page is invalid
 */
static int read_usage_page(struct context *context);

/**
 * @brief Reads a usage ID under the cursor
 *
 * @param context Parsing context
 * @param usage Output pointer for the usage ID
 *
 * @retval 0 if successful
 * @retval -EINVAL if the usage ID is invalid
 */
static int read_usage_id(struct context *context, uint32_t *usage);

/**
 * @brief Reads up to a 32-bit unsigned integer from under the cursor
 *
 * @param context Parsing context
 * @param value Output pointer for the value
 *
 * @retval 0 if successful
 * @retval -EINVAL if the value is invalid
 */
static int read_u32(struct context *context, uint32_t *value);

/**
 * @brief Reads up to a 32-bit signed integer from under the cursor
 *
 * @param context Parsing context
 * @param value Output pointer for the value
 *
 * @retval 0 if successful
 * @retval -EINVAL if the value is invalid
 */
static int read_i32(struct context *context, int32_t *value);

/**
 * @brief Pops a usage ID from the list in the context
 *
 * @details The last ID in the list is always kept on, only cleared when moving to a new main item.
 *
 * @param context Parsing context
 * @param usage Output pointer for the usage ID
 *
 * @retval 0 if successful
 * @retval -EINVAL if the value is invalid
 */
static int pop_usage(struct context *context, uint32_t *usage);

/**
 * @brief Parse a global item. Does not advance the cursor.
 *
 * @param context Parsing context
 *
 * @retval 0 if successful
 * @retval -EINVAL if the value is invalid
 * @retval -ENOMEM if the statically allocated resources are not sufficient
 */
static int parse_global_item(struct context *context);

/**
 * @brief Parse a local item. Does not advance the cursor.
 *
 * @param context Parsing context
 *
 * @retval 0 if successful
 * @retval -EINVAL if the value is invalid
 * @retval -ENOMEM if the statically allocated resources are not sufficient
 */
static int parse_local_item(struct context *context);

/**
 * @brief Parse a main item. Does not advance the cursor.
 *
 * @param context Parsing context
 *
 * @retval 0 if successful
 * @retval -EINVAL if the value is invalid
 * @retval -ENOMEM if the statically allocated resources are not sufficient
 */
static int parse_main_item(struct context *context);

/**
 * @brief Parse a field. Does not advance the cursor.
 *
 * @details Most of the data for the field is actually taken from local and global states.
 *
 * @param context Parsing context
 * @param type Input, output or feature
 *
 * @retval 0 if successful
 * @retval -EINVAL if the value is invalid
 * @retval -ENOMEM if the statically allocated resources are not sufficient
 */
static int parse_field(struct context *context, enum hid_report_field_type type);

static int get_report_boundaries(struct hid_report const *report, uint8_t report_id,
				 size_t *start_index, size_t *end_index);

int hid_report_parse(struct hid_report *report, size_t data_length, uint8_t const data[data_length])
{
	int result = 0;
	/* Initialize the context */
	struct context context = {
		.report = report,
		.data_length = data_length,
		.data = data,
	};

	while (context.cursor < data_length) {
		/* Long item: reserved for future use, skip */
		if (ITEM_IS_LONG(data[context.cursor])) {
			uint8_t data_size = 0;
			/* Wouldn't fit in the remaining data */
			if (context.cursor + 3 > data_length) {
				return -EINVAL;
			}

			data_size = data[context.cursor + 1];
			context.cursor += 3 + data_size;
		}
		/* Short item, part of the specification */
		else {
			int item_length = check_size(&context);
			if (item_length < 0) {
				/* Doesn't fit in the remaining data */
				return item_length;
			}

			/* Switch over the type of the item currently under examination */
			switch (ITEM_TYPE(data[context.cursor])) {
			case ITEM_TYPE_GLOBAL: {
				result = parse_global_item(&context);
				break;
			}
			case ITEM_TYPE_LOCAL: {
				result = parse_local_item(&context);
				break;
			}
			case ITEM_TYPE_MAIN: {
				result = parse_main_item(&context);
				break;
			}
				/* Unknown item type, probably corrupted data */
			default: {
				return -EINVAL;
			}
			}

			if (result < 0) {
				return result;
			}

			/* Advance the cursor */
			context.cursor += item_length;
		}
	}

	return 0;
}

uint16_t hid_report_field_get_usage_id_by_index(struct hid_report_field const *field,
						size_t field_index)
{
	uint16_t result = 0;

	if (
		/* Usage ID is specified as range */
		field->usage_minimum != 0 && field->usage_maximum != 0 &&
		/* The range is valid */
		field->usage_maximum > field->usage_minimum) {

		/* if the index fits in it use it */
		if (field_index < field->usage_maximum - field->usage_minimum) {
			/* Remove the usage page */
			result = (uint16_t)(field->usage_minimum & 0xFFFF) + field_index;
		}
		/* Otherwise it defaults to the last */
		else {
			/* Remove the usage page */
			result = (uint16_t)(field->usage_maximum & 0xFFFF);
		}
	}
	/* Othersise look for it in the usages list */
	else {
		result = field->usages[0];

		/* If there are not enough usage IDs it defaults to the last */
		for (size_t usage_index = 0;
		     usage_index < CONFIG_USBH_HID_REPORT_MAX_USAGES && usage_index < field_index &&
		     field->usages[usage_index] != 0;
		     usage_index++) {
			/* Remove the usage page */
			result = (uint16_t)(field->usages[usage_index] & 0xFFFF);
		}
	}

	return result;
}

bool hid_report_field_contains_usage_id(struct hid_report_field const *field, uint32_t usage_id,
					size_t *field_index)
{
	bool result = false;

	/* Usage range */
	if (field->usage_minimum != 0 && field->usage_maximum != 0) {
		result = (usage_id >= field->usage_minimum) && (usage_id <= field->usage_maximum);

		if (result && field_index != NULL) {
			*field_index = usage_id - field->usage_minimum;
		}
	}
	/* Search the usage list */
	else {
		for (size_t usage_index = 0; usage_index < CONFIG_USBH_HID_REPORT_MAX_USAGES &&
					     field->usages[usage_index] != 0;
		     usage_index++) {
			if (field->usages[usage_index] == usage_id) {
				result = true;

				if (field_index != NULL) {
					*field_index = usage_index;
				}
				break;
			}
		}
	}

	return result;
}

int hid_report_get_usage_id_u32(struct hid_report_field const *field, uint8_t const *data,
				size_t bit_start, uint32_t usage_id, uint32_t *value)
{
	size_t index = 0;

	if (field == NULL || data == NULL || value == NULL) {
		return -EINVAL;
	}

	if (field->size == 0 || field->size > 32) {
		/* Invalid field size */
		return -ENOTSUP;
	}

	if (HID_REPORT_DATA_IS_ARRAY(field->flags)) {
		/* Search is supported only for data items */
		return -ENOTSUP;
	}

	if (!hid_report_field_contains_usage_id(field, usage_id, &index)) {
		/* No such usage ID */
		return -ENOENT;
	}

	*value = 0;
	bit_start += index * field->size;
	for (size_t bit_index = 0; bit_index < field->size; bit_index++) {
		size_t bit_field_position = bit_start + bit_index;
		uint32_t byte_index = bit_field_position / 8;
		uint32_t bit_shift = bit_field_position % 8;

		if (data[byte_index] & (1u << bit_shift)) {
			*value |= 1u << bit_index;
		}
	}

	return 0;
}

int hid_report_get_usage_id_i32(struct hid_report_field const *field, uint8_t const *data,
				size_t bit_start, uint32_t usage_id, int32_t *value)
{
	/* First, fetch the u32 value */
	uint32_t unsigned_value = 0;
	uint32_t max_field_value = 0;

	/* Null size, the following code would not make sense */
	if (field->size == 0) {
		*value = 0;
		return 0;
	}

	int result = hid_report_get_usage_id_u32(field, data, bit_start, usage_id, &unsigned_value);
	if (result < 0) {
		return result;
	}

	/* If the value would be negative (most significant bit set) convert between the arbitrarily
	 * sized two's complement to the i32 */
	if ((unsigned_value & (1 << (field->size - 1))) > 0) {
		max_field_value = (1 << field->size) - 1;
		*value = (int32_t)(((int64_t)unsigned_value - (int64_t)max_field_value) - 1ll);
	}
	/* The value was positive */
	else {
		*value = (int32_t)unsigned_value;
	}

	return 0;
}

bool hid_report_match_usage_page(struct hid_report_field const *field, uint16_t usage_page)
{
	bool result = false;

	/* Generic usage page, always true */
	if (usage_page == 0) {
		result = true;
	}
	/* Usage range */
	else if (field->usage_minimum != 0 && field->usage_maximum != 0) {
		result = ((field->usage_minimum >> 16) == usage_page) &&
			 (field->usage_maximum >> 16 == usage_page);
	}
	/* Search usage list */
	else {
		for (size_t usage_index = 0;
		     field->usages[usage_index] != 0 &&
		     usage_index < sizeof(field->usages) / sizeof(field->usages[0]);
		     usage_index++) {
			if ((field->usages[usage_index] >> 16) == usage_page) {
				result = true;
				break;
			}
		}
	}

	return result;
}

int hid_report_get_input_size(struct hid_report const *report, uint8_t report_id)
{
	/* The report descriptors have bit precision */
	size_t bitsize = 0;
	/* Limits of the report */
	size_t start_index = 0;
	size_t end_index = 0;
	int result = 0;

	if (report == NULL) {
		return -EINVAL;
	}

	result = get_report_boundaries(report, report_id, &start_index, &end_index);
	if (result != 0) {
		return result;
	}

	for (size_t field_index = start_index; field_index < end_index; field_index++) {
		struct hid_report_field const *field = &report->fields[field_index];

		/* We are only interested in input reports */
		if (field->type != HID_REPORT_FIELD_TYPE_INPUT) {
			continue;
		}

		bitsize += (field->size * field->count);
	}

	/* Pad to byte size */
	if ((bitsize % 8) != 0) {
		bitsize += 8 - (bitsize % 8);
	}

	if (report->num_reports > 1) {
		/* If multiple variants are present the report ID is prepended to the report, adding
		 * one byte to the total length */
		bitsize += 8;
	}

	/* The length in bytes */
	return bitsize / 8;
}

int hid_report_input_iterate(struct hid_report const *report, size_t data_length,
			     uint8_t const data[data_length], hid_report_cb_t callback,
			     void *user_data)
{
	size_t data_bit_position = 0;
	int result = 0;
	uint8_t report_id = 0;
	size_t start_index = 0;
	size_t end_index = 0;

	if (report == NULL || data == NULL || callback == NULL) {
		return -EINVAL;
	}

	/* If multiple variants are specified expect the report ID in the first byte */
	if (report->num_reports > 1) {
		report_id = data[0];
		data_bit_position = 8;
	}

	result = get_report_boundaries(report, report_id, &start_index, &end_index);
	if (result != 0) {
		return result;
	}

	for (size_t field_index = start_index; field_index < end_index; field_index++) {
		struct hid_report_field const *field = &report->fields[field_index];
		size_t field_bit_length = field->size * field->count;

		if (data_bit_position + field_bit_length > data_length * 8) {
			return -EINVAL;
		}

		/* Stop only on input fields */
		if (field->type != HID_REPORT_FIELD_TYPE_INPUT) {
			continue;
		}

		/* Invoke the user provided callback on each non-constant field */
		if (HID_REPORT_DATA_IS_DATA(field->flags)) {
			result = callback(field, report_id, data, data_bit_position, user_data);
			if (result != 0) {
				return result;
			}
		}

		data_bit_position += field->size * field->count;
	}

	return 0;
}

/**
 * @brief Print space indentation
 */
static inline void print_indentation(unsigned int indentation)
{
	printf("%*c", indentation, ' ');
}

void hid_report_print(struct hid_report const *report)
{
	unsigned int indentation = 0;
	char const *const field_names[] = {"Input", "Output", "Feature"};

	indentation++;
	for (size_t field_index = 0; field_index < report->num_fields; field_index++) {
		struct hid_report_field const *const field = &report->fields[field_index];

		/* Check if a collection starts on this field, in which case it should be printed */
		for (size_t collection_index = 0; collection_index < report->num_collections;
		     collection_index++) {
			struct hid_report_collection const *const collection =
				&report->collections[collection_index];
			if (collection->start == field_index) {
				print_indentation(indentation);
				printf("Collection 0x%02X - 0x%04X\n", collection->type,
				       collection->usage);
				indentation++;
			}
		}

		/* Check if a report variant starts on this field, in which case it should be
		 * printed */
		for (size_t report_index = 0; report_index < report->num_reports; report_index++) {
			struct hid_report_variant const *const variant =
				&report->reports[report_index];
			if (variant->start == field_index) {
				print_indentation(indentation);
				printf("Report ID 0x%02X\n", variant->id);
			}
		}

		/* Print the field */
		print_indentation(indentation);
		if (field->type > sizeof(field_names) / sizeof(field_names[0])) {
			printf("Field %zu: %i\n", field_index, field->type);
		} else {
			printf("%s %zu:\n", field_names[field->type], field_index);
		}
		indentation++;

		/* Print its usages */
		print_indentation(indentation);
		printf("Usages: ");
		if (field->usage_minimum != 0 || field->usage_maximum != 0) {
			printf("0x%04X - 0x%04X", field->usage_minimum, field->usage_maximum);
		} else {
			for (size_t usage_index = 0;
			     usage_index < sizeof(field->usages) / sizeof(field->usages[0]) &&
			     field->usages[usage_index] != 0;
			     usage_index++) {
				printf("0x%04X, ", field->usages[usage_index]);
			}
		}
		printf("\n");

		/* Print its flags */
		print_indentation(indentation);
		printf("Flags: %s,%s,%s,%s,%s,%s,%s,%s,%s\n",
		       HID_REPORT_DATA_IS_CONSTANT(field->flags) ? "Const" : "Data",
		       HID_REPORT_DATA_IS_ARRAY(field->flags) ? "Array" : "Var",
		       HID_REPORT_DATA_IS_RELATIVE(field->flags) ? "Rel" : "Abs",
		       HID_REPORT_DATA_IS_WRAP(field->flags) ? "Wrap" : "No Wrap",
		       HID_REPORT_DATA_IS_NON_LINEAR(field->flags) ? "Non Linear" : "Linear",
		       HID_REPORT_DATA_HAS_NO_PREFERRED(field->flags) ? "No Preferred"
								      : "Preferred State",
		       HID_REPORT_DATA_HAS_NULL_STATE(field->flags) ? "Null State"
								    : "No Null Position",
		       HID_REPORT_DATA_IS_VOLATILE(field->flags) ? "Volatile" : "Non Volatile",
		       HID_REPORT_DATA_IS_BUFFERED_BYTES(field->flags) ? "Buffered Bytes"
								       : "Bit Field");

		/* Print its range and layout */
		print_indentation(indentation);
		printf("Logical range: %i - %i\n", field->logical_minimum, field->logical_maximum);

		print_indentation(indentation);
		printf("Physical range: %i - %i (%i, %i)\n", field->physical_minimum,
		       field->physical_maximum, field->unit, field->unit_exponent);

		print_indentation(indentation);
		printf("Report size & count: %i, %i\n", field->size, field->count);

		/* If a collection ends here we should reduce indentation */
		for (size_t collection_index = 0; collection_index < report->num_collections;
		     collection_index++) {
			struct hid_report_collection const *const collection =
				&report->collections[collection_index];
			if (collection->end == field_index) {
				indentation--;
			}
		}
		indentation--;
	}
	indentation--;

	printf("Done\n");
}

static int parse_main_item(struct context *context)
{
	enum item_data_size item_size = ITEM_SIZE(context->data[context->cursor]);
	struct hid_report *report = context->report;
	int result = 0;

	switch (ITEM_TAG(context->data[context->cursor])) {
	case MAIN_ITEM_TAG_INPUT: {
		result = parse_field(context, HID_REPORT_FIELD_TYPE_INPUT);
		if (result < 0) {
			return result;
		}
		break;
	}
	case MAIN_ITEM_TAG_OUTPUT: {
		result = parse_field(context, HID_REPORT_FIELD_TYPE_OUTPUT);
		if (result < 0) {
			return result;
		}
		break;
	}
	case MAIN_ITEM_TAG_FEATURE: {
		result = parse_field(context, HID_REPORT_FIELD_TYPE_FEATURE);
		if (result < 0) {
			return result;
		}
		break;
	}
	case MAIN_ITEM_TAG_COLLECTION: {
		uint32_t usage = 0;
		size_t collection_index = report->num_collections;
		struct hid_report_collection *collection = NULL;

		if (report->num_collections >=
		    sizeof(report->collections) / sizeof(report->collections[0])) {
			/* Too many collections */
			return -ENOMEM;
		}

		if (context->collection_stack_index >=
		    sizeof(context->collection_stack) / sizeof(context->collection_stack[0])) {
			/* Too many collections in the stack */
			return -ENOMEM;
		}

		/* Invalid collection with no type */
		if (item_size == 0) {
			return -EINVAL;
		}

		/* Get the usage from the local state */
		result = pop_usage(context, &usage);
		if (result < 0) {
			return result;
		}

		/* Initialize the collection */
		collection = &report->collections[collection_index];
		report->num_collections++;

		context->collection_stack[context->collection_stack_index] = collection_index;

		collection->type = context->data[context->cursor + 1];
		/* The collection is delimited by the current data item */
		collection->start = report->num_fields;
		collection->usage = usage;
		context->collection_stack_index++;
		break;
	}
	case MAIN_ITEM_TAG_COLLECTION_END: {
		if (context->collection_stack_index == 0) {
			/* Mismatched collection end */
			return -EINVAL;
		}

		/* Pop the collection from the stack */
		context->collection_stack_index--;
		/* Add the end to its fields */
		report->collections[context->collection_stack[context->collection_stack_index]]
			.end = report->num_fields;
		break;
	}
	default: {
		result = -EINVAL;
	}
	}

	/* Each time a new main item is found clear the local state */
	context->local_state.usage_minimum = 0;
	context->local_state.usage_maximum = 0;
	context->local_state.num_usages = 0;
	context->local_state.designator_index = 0;
	context->local_state.designator_minimum = 0;
	context->local_state.designator_maximum = 0;
	context->local_state.string_index = 0;
	context->local_state.string_minimum = 0;
	context->local_state.string_maximum = 0;

	return 0;
}

static int parse_global_item(struct context *context)
{
	int result = 0;

	switch (ITEM_TAG(context->data[context->cursor])) {
	case GLOBAL_ITEM_TAG_USAGE_PAGE: {
		result = read_usage_page(context);
		break;
	}
	case GLOBAL_ITEM_TAG_PHYSICAL_MINIMUM: {
		result = read_i32(context, &context->global_state.physical_minimum);
		break;
	}
	case GLOBAL_ITEM_TAG_PHYSICAL_MAXIMUM: {
		result = read_i32(context, &context->global_state.physical_maximum);
		break;
	}
	case GLOBAL_ITEM_TAG_UNIT_EXPONENT: {
		result = read_u32(context, &context->global_state.unit_exponent);
		break;
	}
	case GLOBAL_ITEM_TAG_UNIT: {
		result = read_u32(context, &context->global_state.unit);
		break;
	}
	case GLOBAL_ITEM_TAG_LOGICAL_MINIMUM: {
		result = read_i32(context, &context->global_state.logical_minimum);
		break;
	}
	case GLOBAL_ITEM_TAG_LOGICAL_MAXIMUM: {
		result = read_i32(context, &context->global_state.logical_maximum);
		break;
	}
	case GLOBAL_ITEM_TAG_REPORT_COUNT: {
		result = read_u32(context, &context->global_state.report_count);
		break;
	}
	case GLOBAL_ITEM_TAG_REPORT_SIZE: {
		result = read_u32(context, &context->global_state.report_size);
		break;
	}
	case GLOBAL_ITEM_TAG_REPORT_ID: {
		struct hid_report *report = context->report;
		struct hid_report_variant *variant = NULL;
		uint32_t report_id = 0;

		if (report->num_reports >= sizeof(report->reports) / sizeof(report->reports[0])) {
			/* Too many report variants */
			result = -ENOMEM;
			break;
		}

		/* Read the report ID */
		read_u32(context, &report_id);

		/* A report id of 0 is reserved */
		if (report_id == 0) {
			result = -EINVAL;
			break;
		}

		/* The first report ID must appear before any field */
		if (report->num_fields > 0 && report->num_reports == 0) {
			result = -EINVAL;
			break;
		}

		/* Initialize the report variant */
		variant = &report->reports[report->num_reports];
		variant->id = (uint8_t)report_id;
		variant->start = report->num_fields;
		report->num_reports++;

		break;
	}
	case GLOBAL_ITEM_TAG_PUSH: {
		if (context->state_stack_index >=
		    sizeof(context->state_stack) / sizeof(context->state_stack[0])) {
			/* Too many states */
			result = -ENOMEM;
			break;
		}

		/* Push a copy of the global item table on the stack */
		context->state_stack[context->state_stack_index] = context->global_state;
		context->state_stack_index++;
		break;
	}
	case GLOBAL_ITEM_TAG_POP: {
		if (context->state_stack_index == 0) {
			/* Mismatched pop operation */
			result = -EINVAL;
			break;
		}

		/* Pop the global state from the stack */
		context->state_stack_index--;
		context->global_state = context->state_stack[context->state_stack_index];
		break;
	}
	default: {
		result = -EINVAL;
		break;
	}
	}

	return result;
}

static int parse_local_item(struct context *context)
{
	int result = 0;

	switch (ITEM_TAG(context->data[context->cursor])) {
	case LOCAL_ITEM_TAG_USAGE: {
		/* Populate next usage in the local list*/
		uint32_t *usage = NULL;

		if (context->local_state.num_usages >=
		    sizeof(context->local_state.usages) / sizeof(context->local_state.usages[0])) {
			/* Too many usages */
			return -ENOMEM;
		}

		usage = &context->local_state.usages[context->local_state.num_usages];
		result = read_usage_id(context, usage);
		context->local_state.num_usages++;
		break;
	}
	case LOCAL_ITEM_TAG_USAGE_MINIMUM: {
		result = read_usage_id(context, &context->local_state.usage_minimum);
		break;
	}
	case LOCAL_ITEM_TAG_USAGE_MAXIMUM: {
		result = read_usage_id(context, &context->local_state.usage_maximum);
		break;
	}
	case LOCAL_ITEM_TAG_DESIGNATOR_INDEX: {
		result = read_u32(context, &context->local_state.designator_index);
		break;
	}
	case LOCAL_ITEM_TAG_DESIGNATOR_MINIMUM: {
		result = read_u32(context, &context->local_state.designator_minimum);
		break;
	}
	case LOCAL_ITEM_TAG_DESIGNATOR_MAXIMUM: {
		result = read_u32(context, &context->local_state.designator_maximum);
		break;
	}
	case LOCAL_ITEM_TAG_STRING_INDEX: {
		result = read_u32(context, &context->local_state.string_index);
		break;
	}
	case LOCAL_ITEM_TAG_STRING_MINIMUM: {
		result = read_u32(context, &context->local_state.string_minimum);
		break;
	}
	case LOCAL_ITEM_TAG_STRING_MAXIMUM: {
		result = read_u32(context, &context->local_state.string_maximum);
		break;
	}
	case LOCAL_ITEM_TAG_DELIMITER: {
		/* Ignore */
		break;
	}
	default: {
		result = -EINVAL;
	}
	}

	return result;
}

static int read_usage_page(struct context *context)
{
	struct global_state *global_state = &context->global_state;
	enum item_data_size item_size = ITEM_SIZE(context->data[context->cursor]);

	switch (item_size) {
	case ITEM_DATA_SIZE_0: {
		/* We assume a zero sized usage page represents a value of 0 */
		global_state->usage_page = 0;
		break;
	}

	case ITEM_DATA_SIZE_1: {
		/* If just one byte it's considered part of the upper halfword */
		global_state->usage_page = context->data[context->cursor + 1] << 16;
		break;
	}

	case ITEM_DATA_SIZE_2: {
		/* Size 2 means the upper two bytes */
		global_state->usage_page = (context->data[context->cursor + 1] << 16) |
					   (context->data[context->cursor + 2] << 24);
		break;
	}

	case ITEM_DATA_SIZE_4: {
		global_state->usage_page = context->data[context->cursor + 1] |
					   (context->data[context->cursor + 2] << 8) |
					   (context->data[context->cursor + 3] << 16) |
					   (context->data[context->cursor + 4] << 24);
		break;
	}

	default: {
		return -EINVAL;
	}
	}

	return 0;
}

static int read_usage_id(struct context *context, uint32_t *usage)
{
	struct global_state *global_state = &context->global_state;
	enum item_data_size item_size = ITEM_SIZE(context->data[context->cursor]);

	switch (item_size) {
	case ITEM_DATA_SIZE_0: {
		/* A zero sized usage id is invalid */
		return -EINVAL;
	}

	case ITEM_DATA_SIZE_1: {
		/* If just one byte it's considered an offset ot the current usage page */
		*usage = global_state->usage_page | context->data[context->cursor + 1];
		break;
	}

	case ITEM_DATA_SIZE_2: {
		/* If two bytes long it's considered an offset ot the current usage page */
		*usage = global_state->usage_page | context->data[context->cursor + 1] |
			 (context->data[context->cursor + 2] << 8);
		break;
	}

	case ITEM_DATA_SIZE_4: {
		/* Full usage, no need to rely on the usage page */
		*usage = context->data[context->cursor + 1] |
			 (context->data[context->cursor + 2] << 8) |
			 (context->data[context->cursor + 3] << 16) |
			 (context->data[context->cursor + 4] << 24);
		break;
	}

	default: {
		return -EINVAL;
	}
	}

	return 0;
}

static int read_i32(struct context *context, int32_t *value)
{
	enum item_data_size item_size = ITEM_SIZE(context->data[context->cursor]);

	switch (item_size) {
	case ITEM_DATA_SIZE_0: {
		*value = 0;
		break;
	}

	case ITEM_DATA_SIZE_1: {
		*value = (int8_t)context->data[context->cursor + 1];
		break;
	}

	case ITEM_DATA_SIZE_2: {
		*value = (int16_t)((context->data[context->cursor + 1] |
				    (context->data[context->cursor + 2] << 8)));
		break;
	}

	case ITEM_DATA_SIZE_4: {
		*value = (int32_t)((context->data[context->cursor + 1] |
				    (context->data[context->cursor + 2] << 8) |
				    (context->data[context->cursor + 3] << 16) |
				    (context->data[context->cursor + 4] << 24)));
		break;
	}

	default: {
		return -EINVAL;
	}
	}

	return 0;
}

static int read_u32(struct context *context, uint32_t *value)
{
	enum item_data_size item_size = ITEM_SIZE(context->data[context->cursor]);

	switch (item_size) {
	case ITEM_DATA_SIZE_0: {
		*value = 0;
		break;
	}

	case ITEM_DATA_SIZE_1: {
		*value = context->data[context->cursor + 1];
		break;
	}

	case ITEM_DATA_SIZE_2: {
		*value = (context->data[context->cursor + 1] |
			  (context->data[context->cursor + 2] << 8));
		break;
	}

	case ITEM_DATA_SIZE_4: {
		*value = (context->data[context->cursor + 1] |
			  (context->data[context->cursor + 2] << 8) |
			  (context->data[context->cursor + 3] << 16) |
			  (context->data[context->cursor + 4] << 24));
		break;
	}

	default: {
		return -EINVAL;
	}
	}

	return 0;
}

static int check_size(struct context *context)
{
	/* `item_size` is guaranteed to be 0-3 */
	enum item_data_size item_size = ITEM_SIZE(context->data[context->cursor]);
	/* Value to actual size conversion */
	size_t sizes[] = {1, 2, 3, 5};

	return sizes[item_size];
}

static int pop_usage(struct context *context, uint32_t *usage)
{
	if (context->local_state.num_usages == 0) {
		/* No usage to pop; it's not correct but we allow it */
		*usage = 0;
		return 0;
	}

	*usage = context->local_state.usages[context->local_state.num_usages - 1];

	if (context->local_state.num_usages > 1) {
		context->local_state.num_usages--;
	}

	return 0;
}

static int parse_field(struct context *context, enum hid_report_field_type type)
{
	struct hid_report *report = context->report;
	struct global_state *global_state = &context->global_state;
	struct hid_report_field *field = NULL;

	if (report->num_fields >= sizeof(report->fields) / sizeof(report->fields[0])) {
		/* Too many fields */
		return -ENOMEM;
	}

	field = &report->fields[report->num_fields];
	report->num_fields++;

	field->type = type;

	/* Copy usages to the data item */
	for (size_t usage_index = 0; usage_index < context->local_state.num_usages; usage_index++) {
		field->usages[usage_index] = context->local_state.usages[usage_index];
	}
	field->usage_minimum = context->local_state.usage_minimum;
	field->usage_maximum = context->local_state.usage_maximum;

	/* Store the remaining global state */
	field->logical_minimum = global_state->logical_minimum;
	field->logical_maximum = global_state->logical_maximum;
	field->physical_minimum = global_state->physical_minimum;
	field->physical_maximum = global_state->physical_maximum;
	field->count = global_state->report_count;
	field->size = global_state->report_size;
	field->unit = global_state->unit;
	field->unit_exponent = global_state->unit_exponent;
	read_u32(context, &field->flags);

	return 0;
}

static int get_report_boundaries(struct hid_report const *report, uint8_t report_id,
				 size_t *start_index, size_t *end_index)
{
	if (report == NULL || start_index == NULL || end_index == NULL) {
		return -EINVAL;
	}

	/* Multiple report IDs, find the one specified by `report_id` */
	if (report->num_reports > 1) {
		bool found = false;

		for (size_t report_index = 0; report_index < report->num_reports; report_index++) {
			struct hid_report_variant const *variant = &report->reports[report_index];

			if (variant->id == report_id) {
				/* Start from the beginning of the report variant */
				*start_index = variant->start;

				/* If it's not the last report variant, place the end at the start
				 * of the next one */
				if (report_index + 1 < report->num_reports) {
					*end_index = report->reports[report_index + 1].start;
				} else {
					*end_index = report->num_fields;
				}
				found = true;
				break;
			}
		}

		if (!found) {
			return -EINVAL;
		}
	}
	/* Only one report, default boundaries */
	else {
		*start_index = 0;
		*end_index = report->num_fields;
	}

	return 0;
}
