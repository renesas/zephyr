/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_adcj

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

#include "r_adcj.h"

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE
#include "adc_context.h"

LOG_MODULE_REGISTER(adc_renesas_rh850_adcj, CONFIG_ADC_LOG_LEVEL);

#define ADC_RENESAS_ADCJ_INVALID_VC            0xffU
#define ADC_RENESAS_ADCJ_MAX_ZEPHYR_CHANNELS   32U
#define ADCJ_WAIT_TIME_MAX 0x3FFFU

#define ADCJ_DATA_FORMAT_12_bit_signed_fixed_point \
	ADCJ_DATA_FORMAT_12_BIT_SIGNED_FIXED_POINT
#define ADCJ_DATA_FORMAT_12_bit_signed_integer \
	ADCJ_DATA_FORMAT_12_BIT_SIGNED_INTEGER
#define ADCJ_DATA_FORMAT_12_bit_unsigned_fixed_point \
	ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_FIXED_POINT
#define ADCJ_DATA_FORMAT_12_bit_unsigned_right_justified \
	ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_INTEGER_RIGHT_JUSTIFIED
#define ADCJ_DATA_FORMAT_12_bit_unsigned_left_justified \
	ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_INTEGER_LEFT_JUSTIFIED

#define ADCJ_RESOLUTION_12_bit_signed_fixed_point 12U
#define ADCJ_RESOLUTION_12_bit_signed_integer 12U
#define ADCJ_RESOLUTION_12_bit_unsigned_fixed_point 12U
#define ADCJ_RESOLUTION_12_bit_unsigned_right_justified 12U
#define ADCJ_RESOLUTION_12_bit_unsigned_left_justified 12U

#define ADCJ_FSP_RESOLUTION_12_bit_signed_fixed_point ADC_RESOLUTION_12_BIT
#define ADCJ_FSP_RESOLUTION_12_bit_signed_integer ADC_RESOLUTION_12_BIT
#define ADCJ_FSP_RESOLUTION_12_bit_unsigned_fixed_point ADC_RESOLUTION_12_BIT
#define ADCJ_FSP_RESOLUTION_12_bit_unsigned_right_justified ADC_RESOLUTION_12_BIT
#define ADCJ_FSP_RESOLUTION_12_bit_unsigned_left_justified ADC_RESOLUTION_12_BIT

#define ADCJ_ADDITION_COUNT_2    0U
#define ADCJ_ADDITION_COUNT_4    1U
#define ADCJ_DIRECT_MPX_MIN_WAIT_CYCLES 40U

enum adc_adcj_data_format {
	ADC_ADCJ_DATA_FORMAT_12_BIT_SIGNED_FIXED = 0U,
	ADC_ADCJ_DATA_FORMAT_12_BIT_SIGNED_INTEGER = 1U,
	ADC_ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_FIXED = 2U,
	ADC_ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_RIGHT = 4U,
	ADC_ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_LEFT = 5U,
};

/* FSP ADCJ has 64 virtual channel data registers. */
#define ADC_RENESAS_ADCJ_MAX_VIRTUAL_CHANNELS ADCJ_NUMBER_OF_VIRTUAL_CHANNEL

/*
 * ADCJ VCR.GCTRL is 6 bits wide. The valid selector range is common for
 * normal and hold-value conversions; package-specific pin availability is
 * described by board DTS instead of being hard-coded in the driver.
 */
#define ADC_RENESAS_ADCJ_MAX_PHYSICAL_CHANNELS 64U

#define ADCJ_CONVERSION_TYPE_NORMAL      0U
#define ADCJ_CONVERSION_TYPE_TRACK_HOLD  1U

#define ADCJ_TRACK_HOLD_MASK_TH0 BIT(0)
#define ADCJ_TRACK_HOLD_MASK_TH1 BIT(1)
#define ADCJ_TRACK_HOLD_MASK_TH2 BIT(2)
#define ADCJ_TRACK_HOLD_MASK_TH3 BIT(3)
#define ADCJ_TRACK_HOLD_MASK_VALID \
	(ADCJ_TRACK_HOLD_MASK_TH0 | ADCJ_TRACK_HOLD_MASK_TH1 | \
	 ADCJ_TRACK_HOLD_MASK_TH2 | ADCJ_TRACK_HOLD_MASK_TH3)

#define ADCJ_TH_ENABLE_FROM_MASK(idx, th) \
	(((ADCJ_TRACK_HOLD_MASK(idx) & BIT(th)) != 0U) ? \
	 ADC_TH_UNIT_CFG_ENABLE : ADC_TH_UNIT_CFG_DISABLE)

#define ADCJ_WAIT_TIME(idx, entry)                                      \
	COND_CODE_1(                                                        \
		DT_INST_PROP_HAS_IDX(idx, renesas_wait_time_cycles, entry),     \
		(DT_INST_PROP_BY_IDX(idx, renesas_wait_time_cycles, entry)),    \
		(0U)                                                            \
	)

#define ADCJ_CHILD_WAIT_TABLE(child) \
	DT_PROP_OR(child, renesas_wait_table, 0)

#define ADCJ_CHILD_MPX_VALUE(child) \
	DT_PROP_OR(child, renesas_mpx_value, 0)

#define ADCJ_CHILD_IS_MPX_NORMAL(child) \
	DT_ENUM_HAS_VALUE(child, renesas_conversion_type, mpx_normal)

#define ADCJ_CHILD_IS_MPX_ADDITION(child) \
	DT_ENUM_HAS_VALUE(child, renesas_conversion_type, mpx_addition)

#define ADCJ_CHILD_IS_MPX(child) \
	(ADCJ_CHILD_IS_MPX_NORMAL(child) || \
	ADCJ_CHILD_IS_MPX_ADDITION(child))

#define ADCJ_CHILD_IS_HOLD(child) \
	DT_ENUM_HAS_VALUE(child, renesas_conversion_type, hold)

#define ADCJ_CHILD_IS_AD_CORE_DIAG(child) \
	DT_ENUM_HAS_VALUE(child, renesas_conversion_type, \
		ad_core_self_diagnostic)

#define ADCJ_CHILD_IS_ADVAL(child) \
	DT_ENUM_HAS_VALUE(child, renesas_conversion_type, \
		ad_conversion_path_diagnostic)

#define ADCJ_CHILD_OR_HAS_WAIT(child) \
	|| (ADCJ_CHILD_WAIT_TABLE(child) != 0U)

#define ADCJ_CHILD_OR_IS_HOLD(child) \
	|| ADCJ_CHILD_IS_HOLD(child)

#define ADCJ_INSTANCE_HAS_WAIT(idx) \
	(false DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, \
		ADCJ_CHILD_OR_HAS_WAIT))

#define ADCJ_INSTANCE_HAS_HOLD(idx) \
	(false DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, \
		ADCJ_CHILD_OR_IS_HOLD))

#define ADCJ_CHILD_MPX_USES_ENABLED_TH_CHANNEL(child) \
	((ADCJ_CHILD_PHYSICAL_CHANNEL(child) < 4U) && \
	((ADCJ_CHILD_PARENT_TRACK_HOLD_MASK(child) & \
	BIT(ADCJ_CHILD_PHYSICAL_CHANNEL(child) & 0x1fU)) != 0U))

#define ADCJ_CHILD_OR_IS_MPX(child) \
	|| ADCJ_CHILD_IS_MPX(child)

#define ADCJ_INSTANCE_HAS_MPX(idx) \
	(false DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, \
		ADCJ_CHILD_OR_IS_MPX))

#define ADCJ_NODE_WAIT_VALUE(node_id, entry) \
	COND_CODE_1( \
		DT_PROP_HAS_IDX(node_id, renesas_wait_time_cycles, entry), \
		(DT_PROP_BY_IDX(node_id, renesas_wait_time_cycles, entry)), \
		(0U))

#define ADCJ_NODE_WAIT_VALUE_0(node_id) 0U
#define ADCJ_NODE_WAIT_VALUE_1(node_id) \
	ADCJ_NODE_WAIT_VALUE(node_id, 0)
#define ADCJ_NODE_WAIT_VALUE_2(node_id) \
	ADCJ_NODE_WAIT_VALUE(node_id, 1)
#define ADCJ_NODE_WAIT_VALUE_3(node_id) \
	ADCJ_NODE_WAIT_VALUE(node_id, 2)
#define ADCJ_NODE_WAIT_VALUE_4(node_id) \
	ADCJ_NODE_WAIT_VALUE(node_id, 3)
#define ADCJ_NODE_WAIT_VALUE_5(node_id) \
	ADCJ_NODE_WAIT_VALUE(node_id, 4)
#define ADCJ_NODE_WAIT_VALUE_6(node_id) \
	ADCJ_NODE_WAIT_VALUE(node_id, 5)
#define ADCJ_NODE_WAIT_VALUE_7(node_id) \
	ADCJ_NODE_WAIT_VALUE(node_id, 6)
#define ADCJ_NODE_WAIT_VALUE_8(node_id) \
	ADCJ_NODE_WAIT_VALUE(node_id, 7)

#define ADCJ_CHILD_SELECTED_WAIT_VALUE(child) \
	UTIL_CAT(ADCJ_NODE_WAIT_VALUE_, \
		ADCJ_CHILD_WAIT_TABLE(child))(DT_PARENT(child))

#define ADCJ_NODE_WAIT_TABLE_COUNT(node_id) \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, renesas_wait_time_cycles), \
		(DT_PROP_LEN(node_id, renesas_wait_time_cycles)), \
		(0U))

#define ADCJ_INSTANCE_USES_TH(idx) \
	(ADCJ_TRACK_HOLD_MASK(idx) != 0U)

#define ADCJ_TRIGGER_IS_SOFTWARE_STARTED(idx) \
	(ADCJ_TRIGGER_IS_SOFTWARE(idx) || ADCJ_TRIGGER_IS_TH(idx))

#define ADCJ_CHILD_OR_HOLD_MASK(child) \
	| (ADCJ_CHILD_IS_HOLD(child) ? \
	BIT(ADCJ_CHILD_PHYSICAL_CHANNEL(child) & 0x1fU) : 0U)

#define ADCJ_HOLD_CHANNEL_MASK(idx) \
	(0U DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, \
		ADCJ_CHILD_OR_HOLD_MASK))

#define ADCJ_CHILD_INPUT_HAS_TH_ENABLED(child) \
	((ADCJ_CHILD_PHYSICAL_CHANNEL(child) < 4U) && \
	((ADCJ_CHILD_PARENT_TRACK_HOLD_MASK(child) & \
	BIT(ADCJ_CHILD_PHYSICAL_CHANNEL(child) & 0x1fU)) != 0U))

#define ADCJ_CHILD_OR_VC_MASK(child) \
	| BIT64(ADCJ_CHILD_VC(child) & 0x3fU)

#define ADCJ_CONFIGURED_VC_MASK(idx) \
	(0ULL DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, \
		ADCJ_CHILD_OR_VC_MASK))

#define ADCJ_CHILD_ADD_COUNT(child) + 1U

#define ADCJ_CHANNEL_CHILD_COUNT(idx) \
	(0U DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADCJ_CHILD_ADD_COUNT))

#define ADCJ_CHILD_ADD_HOLD_COUNT(child) \
	+ (ADCJ_CHILD_IS_HOLD(child) ? 1U : 0U)

#define ADCJ_HOLD_CHANNEL_COUNT(idx) \
	(0U DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, \
		ADCJ_CHILD_ADD_HOLD_COUNT))

#define ADCJ_CHILD_USES_PHYSICAL_INPUT(child) \
	(!ADCJ_CHILD_IS_HOLD(child) && \
	!ADCJ_CHILD_IS_AD_CORE_DIAG(child) && \
	!ADCJ_CHILD_IS_ADVAL(child))

#define ADCJ_CHILD_PHYSICAL_CHANNEL_MASK(child) \
	((ADCJ_CHILD_PHYSICAL_CHANNEL(child) < 32U) ? \
	DT_PROP_BY_IDX(DT_PARENT(child), renesas_physical_channel_mask, 0) : \
	DT_PROP_BY_IDX(DT_PARENT(child), renesas_physical_channel_mask, 1))

#define ADCJ_CHILD_PHYSICAL_CHANNEL_BIT(child) \
	BIT(ADCJ_CHILD_PHYSICAL_CHANNEL(child) & 0x1fU)

#define ADCJ_CHILD_PHYSICAL_CHANNEL_IS_VALID(child) \
	((ADCJ_CHILD_PHYSICAL_CHANNEL_MASK(child) & \
	ADCJ_CHILD_PHYSICAL_CHANNEL_BIT(child)) != 0U)

#define ADCJ_CHILD_IS_PIN_DIAG(child) \
	DT_ENUM_HAS_VALUE(child, renesas_conversion_type, \
		pin_level_self_diagnostic)

#define ADCJ_CHILD_IS_IO_DIAGNOSTIC(child) \
	((ADCJ_CHILD_CONVERSION_TYPE(child) >= 7U) && \
	(ADCJ_CHILD_CONVERSION_TYPE(child) <= 13U))

#define ADCJ_CHILD_IS_VMON_INPUT(child) \
	((DT_PROP(DT_PARENT(child), renesas_unit) == 0U) && \
	(ADCJ_CHILD_PHYSICAL_CHANNEL(child) >= 20U) && \
	(ADCJ_CHILD_PHYSICAL_CHANNEL(child) <= 23U))

#define ADCJ_CHILD_IS_LOW_ACCURACY_INPUT(child) \
	(((DT_PROP(DT_PARENT(child), renesas_unit) == 0U) && \
	(ADCJ_CHILD_PHYSICAL_CHANNEL(child) >= 24U) && \
	(ADCJ_CHILD_PHYSICAL_CHANNEL(child) <= 33U)) || \
	((DT_PROP(DT_PARENT(child), renesas_unit) == 1U) && \
	(ADCJ_CHILD_PHYSICAL_CHANNEL(child) >= 20U) && \
	(ADCJ_CHILD_PHYSICAL_CHANNEL(child) <= 33U)) || \
	((DT_PROP(DT_PARENT(child), renesas_unit) == 2U) && \
	(ADCJ_CHILD_PHYSICAL_CHANNEL(child) >= 20U) && \
	(ADCJ_CHILD_PHYSICAL_CHANNEL(child) <= 29U)))

#define ADCJ_CHILD_OR_CHANNEL_MASK(child) \
	| BIT(DT_REG_ADDR(child) & 0x1fU)

#define ADCJ_CONFIGURED_CHANNEL_MASK(idx) \
	(0U DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, \
		ADCJ_CHILD_OR_CHANNEL_MASK))

#define ADCJ_LIMIT_TABLE_ENTRY(idx, n) \
	[n] = { \
		.vcullmtbr = \
		((uint32_t)DT_INST_PROP_BY_IDX( \
			idx, renesas_upper_limit_values, n) << 16) | \
		(uint32_t)DT_INST_PROP_BY_IDX( \
			idx, renesas_lower_limit_values, n), \
	}

#define ADCJ_LIMIT_VALUE_VALIDATE(node_id, prop, index)                 \
	BUILD_ASSERT(                                                  \
		(uint64_t)DT_PROP_BY_IDX(node_id, prop, index) <=         \
		0xFFFFULL,                                      \
		"ADCJ limit threshold must fit in 16 bits");

#define ADCJ_CHILD_LIMIT_IRQ_MASK(child) \
	| (DT_PROP_OR(child, renesas_limit_interrupt, 0) ? \
	BIT64(ADCJ_CHILD_VC(child)) : 0ULL)

#define ADCJ_LIMIT_IRQ_MASK(idx) \
	(0ULL DT_INST_FOREACH_CHILD_STATUS_OKAY( \
		idx, ADCJ_CHILD_LIMIT_IRQ_MASK))

extern void adcj_sg0_scan_end_isr(void);
extern void adcj_sg1_scan_end_isr(void);
extern void adcj_sg2_scan_end_isr(void);
extern void adcj_sg3_scan_end_isr(void);
extern void adcj_sg4_scan_end_isr(void);
extern void adcj_error_isr(void);

struct adc_adcj_config {
	uint32_t channel_available_mask;
	uint8_t scan_group;
	uint8_t resolution;
	uint8_t data_format;
	uint8_t track_hold_mask;
	bool software_trigger;
	bool uses_th;
	adc_group_mask_t scan_group_mask;

	const adc_cfg_t *fsp_cfg;
	const adcj_sg_cfg_t *fsp_sg_cfg;
	const uint8_t *channel_to_vc;

	void (*irq_config_func)(const struct device *dev);
};

struct adc_adcj_data {
	struct adc_context ctx;
	const struct device *dev;

	adcj_instance_ctrl_t fsp_ctrl;
	adc_callback_args_t callback_memory;

	uint16_t *buffer;
	uint32_t channels;
	uint32_t configured_channels;
	bool active;
	int status;
};

static int adc_adcj_fsp_err_to_errno(fsp_err_t err)
{
	switch (err) {
	case FSP_SUCCESS:
		return 0;
	case FSP_ERR_UNSUPPORTED:
		return -ENOTSUP;
	case FSP_ERR_ALREADY_OPEN:
		return -EALREADY;
	case FSP_ERR_NOT_OPEN:
		return -EIO;
	case FSP_ERR_IN_USE:
		return -EBUSY;
	case FSP_ERR_TIMEOUT:
		return -ETIMEDOUT;
	case FSP_ERR_ASSERTION:
	default:
		return -EINVAL;
	}
}

static void adc_adcj_error_dispatch_isr(const struct device *dev)
{
	ARG_UNUSED(dev);

	adcj_error_isr();
}

static int adc_adcj_channel_setup(const struct device *dev,
				  const struct adc_channel_cfg *channel_cfg)
{
	struct adc_adcj_data *data = dev->data;
	const struct adc_adcj_config *config = dev->config;
	uint8_t channel_id = channel_cfg->channel_id;

	if (channel_id >= ADC_RENESAS_ADCJ_MAX_ZEPHYR_CHANNELS) {
		LOG_ERR("Unsupported channel id %u", channel_id);
		return -EINVAL;
	}

	if ((config->channel_available_mask & BIT(channel_id)) == 0U) {
		LOG_ERR("Channel %u is not available", channel_id);
		return -ENOTSUP;
	}

	if (config->channel_to_vc[channel_id] >= ADC_RENESAS_ADCJ_MAX_VIRTUAL_CHANNELS) {
		LOG_ERR("Channel %u is not mapped to a valid ADCJ virtual channel", channel_id);
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -ENOTSUP;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Unsupported channel gain %d", channel_cfg->gain);
		return -ENOTSUP;
	}

	/*
	 * ADCJ reference is board/SoC analog reference dependent. The driver accepts
	 * VDD and internal references for compatibility with Zephyr ADC samples.
	 */
	if ((channel_cfg->reference != ADC_REF_VDD_1) &&
	    (channel_cfg->reference != ADC_REF_INTERNAL)) {
		LOG_ERR("Unsupported channel reference %d", channel_cfg->reference);
		return -ENOTSUP;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Unsupported acquisition time");
		return -ENOTSUP;
	}

	data->configured_channels |= BIT(channel_id);

	return 0;
}

static int adc_adcj_check_buffer_size(const struct adc_sequence *sequence)
{
	size_t needed = POPCOUNT(sequence->channels) * sizeof(uint16_t);

	if (sequence->options != NULL) {
		needed *= (1U + sequence->options->extra_samplings);
	}

	return (sequence->buffer_size < needed) ? -ENOMEM : 0;
}

static int adc_adcj_validate_sequence(const struct device *dev,
				      const struct adc_sequence *sequence)
{
	const struct adc_adcj_config *config = dev->config;
	struct adc_adcj_data *data = dev->data;
	int err;

	if (sequence == NULL) {
		return -EINVAL;
	}

	if (sequence->channels == 0U) {
		LOG_ERR("No ADC channel selected");
		return -EINVAL;
	}

	if ((sequence->channels & ~config->channel_available_mask) != 0U) {
		LOG_ERR("Requested unsupported channels 0x%08x", sequence->channels);
		return -ENOTSUP;
	}

	if ((sequence->channels & ~data->configured_channels) != 0U) {
		LOG_ERR("Requested unconfigured channels 0x%08x", sequence->channels);
		return -EINVAL;
	}

	if ((sequence->resolution != 12U)) {
		LOG_ERR("Unsupported resolution %u", sequence->resolution);
		return -ENOTSUP;
	}

	if (sequence->resolution != config->resolution) {
		LOG_ERR("Requested resolution %u does not match ADCJ FSP data format resolution %u",
			sequence->resolution, config->resolution);
		return -ENOTSUP;
	}

	/*
	 * ADCJ does not support calibration. Zephyr requires ADC drivers
	 * without calibration support to ignore sequence->calibrate.
	 */

	if (sequence->oversampling != 0U) {
		LOG_ERR("Oversampling is not supported");
		return -ENOTSUP;
	}

	if (sequence->buffer == NULL) {
		return -EINVAL;
	}

	err = adc_adcj_check_buffer_size(sequence);
	if (err != 0) {
		LOG_ERR("ADC buffer is too small");
		return err;
	}

	return 0;
}

static int adc_adcj_raw_to_count_from_metadata(
			enum adc_adcj_data_format data_format,
			uint8_t conversion_type,
			uint8_t addition_count,
			bool wiring_break_addition,
			uint16_t raw,
			uint16_t *sample)
{
	uint8_t conversion_count = 1U;
	uint8_t extra_bits = 0U;
	uint8_t result_bits = 12U;
	uint8_t shift;
	uint32_t mask;
	uint16_t sum;
	bool addition;

	if (sample == NULL) {
		return -EINVAL;
	}

	addition =
		(conversion_type == ADCJ_VC_CONV_TYPE_ADDITION_MODE) ||
		(conversion_type == ADCJ_VC_CONV_TYPE_MPX_ADDITION_MODE);

	if ((conversion_type >=
		ADCJ_VC_CONV_TYPE_WIRING_BREAK_DETECT_MODE_1) &&
		(conversion_type <=
		ADCJ_VC_CONV_TYPE_SELF_DIAG_WIRING_BREAK_DETECT_MODE_2_PULL_UP) &&
		wiring_break_addition) {
		addition = true;
	}

	if (addition) {
		conversion_count =
		(addition_count == ADCJ_ADDITION_COUNT_FOUR) ? 4U : 2U;

		extra_bits = (conversion_count == 4U) ? 2U : 1U;
	}

	switch (data_format) {
	case ADC_ADCJ_DATA_FORMAT_12_BIT_SIGNED_FIXED:
		shift = 3U - extra_bits;
		break;

	case ADC_ADCJ_DATA_FORMAT_12_BIT_SIGNED_INTEGER:
	case ADC_ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_RIGHT:
		shift = 0U;
		break;

	case ADC_ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_FIXED:
	case ADC_ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_LEFT:
		shift = 4U - extra_bits;
		break;

	default:
		return -ENOTSUP;
	}

	result_bits += extra_bits;
	mask = (1UL << result_bits) - 1UL;

	/* Decode the accumulation, then normalize to the ADC resolution. */
	sum = (uint16_t)(((uint32_t)raw >> shift) & mask);
	*sample = (uint16_t)(sum / conversion_count);

	return 0;
}

static int adc_adcj_raw_to_count(const struct device *dev,
			uint8_t channel,
			uint16_t raw,
			uint16_t *sample)
{
	const struct adc_adcj_config *config = dev->config;
	const adcj_extended_cfg_t *extend;
	const adcj_sg_generic_cfg_t *sg_cfg;
	enum adc_adcj_data_format data_format;
	adcj_vc_conv_type_t conv_type;
	uint8_t addition_count;
	uint8_t vc;
	bool wiring_break_addition;

	if ((sample == NULL) ||
		(channel >= ADC_RENESAS_ADCJ_MAX_ZEPHYR_CHANNELS) ||
		(config->scan_group >= ADCJ_NUMBER_OF_SG)) {
		return -EINVAL;
	}

	extend = config->fsp_cfg->p_extend;
	sg_cfg = config->fsp_sg_cfg->p_sg_cfg[config->scan_group];

	if ((extend == NULL) || (sg_cfg == NULL) ||
		(sg_cfg->p_vc_cfg == NULL)) {
		return -EINVAL;
	}

	vc = config->channel_to_vc[channel];

	if ((vc >= ADC_RENESAS_ADCJ_MAX_VIRTUAL_CHANNELS) ||
		(vc < sg_cfg->start_vc_ptr) ||
		(vc > sg_cfg->end_vc_ptr)) {
		return -EINVAL;
	}

	conv_type =
		sg_cfg->p_vc_cfg[vc - sg_cfg->start_vc_ptr].vcr_b.conv_type;

	data_format = (enum adc_adcj_data_format)
		extend->data_format_addition_count.adcr2_b.data_format;

	addition_count =
		extend->data_format_addition_count.adcr2_b.addition_count;

	wiring_break_addition =
		(extend->wiring_break_detect_cfg.odcr_b.addition_mode ==
		ADCJ_WIRING_BREAK_ADDITION_MODE_ENABLE);

	return adc_adcj_raw_to_count_from_metadata(
		data_format, conv_type, addition_count,
		wiring_break_addition, raw, sample);
}

static int adc_adcj_read_channels(const struct device *dev)
{
	const struct adc_adcj_config *config = dev->config;
	struct adc_adcj_data *data = dev->data;
	uint32_t channels = data->channels;
	uint16_t *buffer = data->buffer;

	for (uint8_t ch = 0U; channels != 0U; ch++, channels >>= 1U) {
		uint8_t vc;
		uint16_t raw;
		fsp_err_t fsp_err;
		int err;

		if ((channels & 1U) == 0U) {
			continue;
		}

		vc = config->channel_to_vc[ch];

		if (vc >= ADC_RENESAS_ADCJ_MAX_VIRTUAL_CHANNELS) {
			return -EINVAL;
		}

		fsp_err = R_ADCJ_Read(&data->fsp_ctrl,
				(adc_channel_t)vc, &raw);
		if (fsp_err != FSP_SUCCESS) {
			return adc_adcj_fsp_err_to_errno(fsp_err);
		}

		err = adc_adcj_raw_to_count(dev, ch, raw, buffer);
		if (err != 0) {
			return err;
		}

		buffer++;
	}

	return 0;
}

static void adc_adcj_fsp_callback(adc_callback_args_t *args)
{
	const struct device *dev = args->p_context;
	const struct adc_adcj_config *config = dev->config;
	struct adc_adcj_data *data = dev->data;
	int err;

	if (!data->active) {
		return;
	}

	if (args->event == ADC_EVENT_MPX_CONVERSION) {
		return;
	}

	if ((args->event == ADC_EVENT_ID_ERROR) ||
	    (args->event == ADC_EVENT_OVERWRITE_ERROR) ||
	    (args->event == ADC_EVENT_TRIGGER_OVERLAP_ERROR) ||
	    (args->event == ADC_EVENT_UPPER_LOWER_LIMIT_ERROR)) {
		data->status = -EIO;
		data->active = false;
		adc_context_complete(&data->ctx, -EIO);
		return;
	}

	if (args->event != (ADC_EVENT_SCAN_COMPLETE_GROUP_0 + config->scan_group)) {
		return;
	}

	if (R_ADCJ_ScanGroupStop(&data->fsp_ctrl, config->scan_group_mask) != FSP_SUCCESS) {
		data->status = -EIO;
		data->active = false;
		adc_context_complete(&data->ctx, -EIO);
		return;
	}

	err = adc_adcj_read_channels(dev);
	if (err != 0) {
		data->status = err;
		data->active = false;
		adc_context_complete(&data->ctx, err);
		return;
	}

	data->status = 0;
	adc_context_on_sampling_done(&data->ctx, dev);
}

static int adc_adcj_start_read(const struct device *dev,
			       const struct adc_sequence *sequence)
{
	struct adc_adcj_data *data = dev->data;
	int err;

	err = adc_adcj_validate_sequence(dev, sequence);
	if (err != 0) {
		return err;
	}

	data->buffer = sequence->buffer;
	data->channels = sequence->channels;
	data->status = 0;
	data->active = false;

	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_adcj_read_common(const struct device *dev,
		const struct adc_sequence *sequence,
		bool asynchronous,
		struct k_poll_signal *signal)
{
	struct adc_adcj_data *data = dev->data;
	int err;

	if (sequence == NULL) {
		return -EINVAL;
	}

	adc_context_lock(&data->ctx, asynchronous, signal);
	err = adc_adcj_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_adcj_read_async(const struct device *dev,
		const struct adc_sequence *sequence,
		struct k_poll_signal *async)
{
	return adc_adcj_read_common(dev, sequence, true, async);
}
#endif

static int adc_adcj_read(const struct device *dev,
		const struct adc_sequence *sequence)
{
	const struct adc_adcj_config *config = dev->config;

	if (sequence == NULL) {
		return -EINVAL;
	}

	if (!config->software_trigger) {
		return -ENOTSUP;
	}

	return adc_adcj_read_common(dev, sequence, false, NULL);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_adcj_data *data = CONTAINER_OF(ctx, struct adc_adcj_data, ctx);
	const struct device *dev = data->dev;
	const struct adc_adcj_config *config = dev->config;
	fsp_err_t fsp_err;

	data->channels = ctx->sequence.channels;
	data->active = true;

	if (config->uses_th) {
		fsp_err = R_ADCJ_THEnable(&data->fsp_ctrl);
		if (fsp_err != FSP_SUCCESS) {
			int err = adc_adcj_fsp_err_to_errno(fsp_err);

			data->status = err;
			data->active = false;
			adc_context_complete(ctx, err);
			return;
		}
	}

	fsp_err = R_ADCJ_ScanGroupStart(&data->fsp_ctrl, config->scan_group_mask);
	if (fsp_err != FSP_SUCCESS) {
		int err = adc_adcj_fsp_err_to_errno(fsp_err);

		data->status = err;
		data->active = false;
		adc_context_complete(ctx, err);
	}
}

static void adc_context_on_complete(struct adc_context *ctx, int status)
{
	struct adc_adcj_data *data = CONTAINER_OF(ctx, struct adc_adcj_data, ctx);
	const struct adc_adcj_config *config = data->dev->config;
	fsp_err_t fsp_err;

	ARG_UNUSED(status);

	adc_context_disable_timer(ctx);
	data->active = false;

	fsp_err = R_ADCJ_ScanGroupStop(&data->fsp_ctrl,
			config->scan_group_mask);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("ADCJ cleanup failed: %d", fsp_err);
	}
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_adcj_data *data = CONTAINER_OF(ctx, struct adc_adcj_data, ctx);

	if (!repeat_sampling) {
		data->buffer += POPCOUNT(ctx->sequence.channels);
	}
}

static void adc_adcj_scanend_dispatch_isr(const struct device *dev)
{
	const struct adc_adcj_config *config = dev->config;

	switch (config->scan_group) {
	case 0U:
		adcj_sg0_scan_end_isr();
		break;
	case 1U:
		adcj_sg1_scan_end_isr();
		break;
	case 2U:
		adcj_sg2_scan_end_isr();
		break;
	case 3U:
		adcj_sg3_scan_end_isr();
		break;
	case 4U:
		adcj_sg4_scan_end_isr();
		break;
	default:
		break;
	}
}

static int adc_adcj_init(const struct device *dev)
{
	const struct adc_adcj_config *config = dev->config;
	struct adc_adcj_data *data = dev->data;
	fsp_err_t fsp_err;
	int ret;

	data->dev = dev;
	data->active = false;
	data->status = 0;

	fsp_err = R_ADCJ_Open(&data->fsp_ctrl, config->fsp_cfg);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADCJ_Open failed: %d", fsp_err);
		return adc_adcj_fsp_err_to_errno(fsp_err);
	}

	fsp_err = R_ADCJ_CallbackSet(&data->fsp_ctrl, adc_adcj_fsp_callback,
				       (void *)dev, &data->callback_memory);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADCJ_CallbackSet failed: %d", fsp_err);
		(void)R_ADCJ_Close(&data->fsp_ctrl);
		return adc_adcj_fsp_err_to_errno(fsp_err);
	}

	fsp_err = R_ADCJ_ScanCfg(&data->fsp_ctrl, config->fsp_sg_cfg);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADCJ_ScanCfg failed: %d", fsp_err);
		(void)R_ADCJ_Close(&data->fsp_ctrl);
		return adc_adcj_fsp_err_to_errno(fsp_err);
	}

	if (config->irq_config_func != NULL) {
		config->irq_config_func(dev);
	}

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define ADCJ_GROUP_MASK_0 ADC_GROUP_MASK_0
#define ADCJ_GROUP_MASK_1 ADC_GROUP_MASK_1
#define ADCJ_GROUP_MASK_2 ADC_GROUP_MASK_2
#define ADCJ_GROUP_MASK_3 ADC_GROUP_MASK_3
#define ADCJ_GROUP_MASK_4 ADC_GROUP_MASK_4
#define ADCJ_GROUP_MASK_FROM_GROUP(idx) \
	UTIL_CAT(ADCJ_GROUP_MASK_, DT_INST_PROP(idx, renesas_scan_group))

#define ADCJ_ADDITION_COUNT(idx) \
	UTIL_CAT(ADCJ_ADDITION_COUNT_, DT_INST_PROP(idx, renesas_addition_count))
#define ADCJ_DATA_FORMAT(idx) \
	UTIL_CAT(ADCJ_DATA_FORMAT_, DT_INST_STRING_TOKEN(idx, renesas_data_format))
#define ADCJ_RESOLUTION(idx) \
	UTIL_CAT(ADCJ_RESOLUTION_, DT_INST_STRING_TOKEN(idx, renesas_data_format))
#define ADCJ_NODE_RESOLUTION(node_id) \
	UTIL_CAT(ADCJ_RESOLUTION_, DT_STRING_TOKEN(node_id, renesas_data_format))
#define ADCJ_FSP_RESOLUTION(idx) \
	UTIL_CAT(ADCJ_FSP_RESOLUTION_, DT_INST_STRING_TOKEN(idx, renesas_data_format))

#define ADCJ_TRIGGER_METHOD_software ADCJ_SG_TRIGGER_METHOD_SW
#define ADCJ_TRIGGER_METHOD_hardware ADCJ_SG_TRIGGER_METHOD_HW
#define ADCJ_TRIGGER_METHOD_ad_timer ADCJ_SG_TRIGGER_METHOD_AD_TIMER
#define ADCJ_TRIGGER_METHOD_th_a ADCJ_SG_TRIGGER_METHOD_THA
#define ADCJ_TRIGGER_METHOD(idx) \
	UTIL_CAT(ADCJ_TRIGGER_METHOD_, DT_INST_STRING_TOKEN(idx, renesas_trigger_method))
#define ADCJ_TRIGGER_IS_SOFTWARE(idx) \
	DT_INST_ENUM_HAS_VALUE(idx, renesas_trigger_method, software)
#define ADCJ_TRIGGER_IS_AD_TIMER(idx) \
	DT_INST_ENUM_HAS_VALUE(idx, renesas_trigger_method, ad_timer)
#define ADCJ_TRIGGER_IS_TH(idx) \
	DT_INST_ENUM_HAS_VALUE(idx, renesas_trigger_method, th_a)
#define ADCJ0_TRIGGER_SOURCE_disabled ADCJ_ADCJ0_TRIGGER_SRC_DISABLE
#define ADCJ0_TRIGGER_SOURCE_pic2 ADCJ_ADCJ0_TRIGGER_SRC_PIC2
#define ADCJ0_TRIGGER_SOURCE(idx) \
	UTIL_CAT(ADCJ0_TRIGGER_SOURCE_, \
		 DT_INST_STRING_TOKEN(idx, renesas_adcj0_trigger_source))
#define ADCJ_AD_TIMER_TRIGGER_software ADCJ_AD_TIMER_TRIGGER_METHOD_SW
#define ADCJ_AD_TIMER_TRIGGER_hardware ADCJ_AD_TIMER_TRIGGER_METHOD_HW
#define ADCJ_AD_TIMER_TRIGGER(idx) \
	UTIL_CAT(ADCJ_AD_TIMER_TRIGGER_, \
		 DT_INST_STRING_TOKEN(idx, renesas_ad_timer_trigger_method))

#define ADCJ_CHANNEL_TO_VC_INVALID_INIT \
	[0] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[1] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[2] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[3] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[4] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[5] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[6] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[7] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[8] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[9] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[10] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[11] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[12] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[13] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[14] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[15] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[16] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[17] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[18] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[19] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[20] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[21] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[22] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[23] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[24] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[25] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[26] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[27] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[28] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[29] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[30] = ADC_RENESAS_ADCJ_INVALID_VC, \
	[31] = ADC_RENESAS_ADCJ_INVALID_VC

#define ADCJ_CHILD_PHYSICAL_CHANNEL(child) \
	DT_PROP_OR(child, renesas_physical_channel, DT_REG_ADDR(child))
#define ADCJ_CHILD_CONVERSION_TYPE(child) \
	DT_ENUM_IDX_OR(child, renesas_conversion_type, 0)
#define ADCJ_CHILD_PARENT_TRACK_HOLD_MASK(child) \
	DT_PROP_OR(DT_PARENT(child), renesas_track_hold_mask, 0)
#define ADCJ_CHILD_CONVERSION_TYPE_VALUE(child) ADCJ_CHILD_CONVERSION_TYPE(child)
#define ADCJ_VC_CONV_TYPE_0 ADCJ_VC_CONV_TYPE_NORMAL
#define ADCJ_VC_CONV_TYPE_1 ADCJ_VC_CONV_TYPE_HOLD_VALUE
#define ADCJ_VC_CONV_TYPE_2 ADCJ_VC_CONV_TYPE_EXTENDED_SAMPLING_TIME
#define ADCJ_VC_CONV_TYPE_3 ADCJ_VC_CONV_TYPE_AD_CORE_SELF_DIAG
#define ADCJ_VC_CONV_TYPE_4 ADCJ_VC_CONV_TYPE_ADDITION_MODE
#define ADCJ_VC_CONV_TYPE_5 ADCJ_VC_CONV_TYPE_MPX_NORMAL
#define ADCJ_VC_CONV_TYPE_6 ADCJ_VC_CONV_TYPE_MPX_ADDITION_MODE
#define ADCJ_VC_CONV_TYPE_7 ADCJ_VC_CONV_TYPE_PIN_LEVEL_SELF_DIAGNOSTIC
#define ADCJ_VC_CONV_TYPE_8 ADCJ_VC_CONV_TYPE_WIRING_BREAK_DETECT_MODE_1
#define ADCJ_VC_CONV_TYPE_9 ADCJ_VC_CONV_TYPE_WIRING_BREAK_DETECT_MODE_2_PULL_DOWN
#define ADCJ_VC_CONV_TYPE_10 ADCJ_VC_CONV_TYPE_WIRING_BREAK_DETECT_MODE_2_PULL_UP
#define ADCJ_VC_CONV_TYPE_11 ADCJ_VC_CONV_TYPE_SELF_DIAG_WIRING_BREAK_DETECT_MODE_1
#define ADCJ_VC_CONV_TYPE_12 ADCJ_VC_CONV_TYPE_SELF_DIAG_WIRING_BREAK_DETECT_MODE_2_PULL_DOWN
#define ADCJ_VC_CONV_TYPE_13 ADCJ_VC_CONV_TYPE_SELF_DIAG_WIRING_BREAK_DETECT_MODE_2_PULL_UP
#define ADCJ_VC_CONV_TYPE_14 ADCJ_VC_CONV_TYPE_AD_CONV_PATH_DIAG
#define ADCJ_CHILD_VC_CONV_TYPE(child) \
	UTIL_CAT(ADCJ_VC_CONV_TYPE_, ADCJ_CHILD_CONVERSION_TYPE(child))
#define ADCJ_CHILD_GENERAL_CTRL(child) \
	((ADCJ_CHILD_CONVERSION_TYPE(child) == 3U) ? \
	 DT_PROP_OR(child, renesas_self_diagnostic_level, 0) : \
	 ((ADCJ_CHILD_CONVERSION_TYPE(child) == 14U) ? \
	  DT_PROP_OR(child, renesas_adval_data, 0) : \
	  ADCJ_CHILD_PHYSICAL_CHANNEL(child)))
#define ADCJ_START_VC(idx) \
	DT_INST_PROP_OR(idx, renesas_start_virtual_channel, 0)
#define ADCJ_END_VC(idx) \
	DT_INST_PROP_OR(idx, renesas_end_virtual_channel, 0)
#define ADCJ_TRACK_HOLD_MASK(idx) \
	DT_INST_PROP_OR(idx, renesas_track_hold_mask, 0)
#define ADCJ_CHILD_VC(child) \
	DT_PROP_OR(child, renesas_virtual_channel, DT_REG_ADDR(child))

#define ADCJ_CHILD_START_VC(child) \
	DT_PROP(DT_PARENT(child), renesas_start_virtual_channel)

#define ADCJ_CHILD_END_VC(child) \
	DT_PROP(DT_PARENT(child), renesas_end_virtual_channel)

#define ADCJ_CHILD_VALIDATE(child) \
	BUILD_ASSERT(DT_REG_ADDR(child) < \
		ADC_RENESAS_ADCJ_MAX_ZEPHYR_CHANNELS, \
		"ADCJ Zephyr channel ID must be less than 32"); \
	BUILD_ASSERT(ADCJ_CHILD_VC(child) < \
		ADC_RENESAS_ADCJ_MAX_VIRTUAL_CHANNELS, \
		"ADCJ virtual channel is out of range"); \
	BUILD_ASSERT(ADCJ_CHILD_VC(child) >= ADCJ_CHILD_START_VC(child), \
		"ADCJ virtual channel is below scan group start"); \
	BUILD_ASSERT(ADCJ_CHILD_VC(child) <= ADCJ_CHILD_END_VC(child), \
		"ADCJ virtual channel is above scan group end"); \
	BUILD_ASSERT(ADCJ_CHILD_PHYSICAL_CHANNEL(child) < \
		ADC_RENESAS_ADCJ_MAX_PHYSICAL_CHANNELS, \
		"ADCJ physical channel selector is out of range"); \
	BUILD_ASSERT(DT_PROP_OR(child, renesas_mpx_value, 0) <= 7U, \
		"ADCJ MPX value must be in range 0..7"); \
	BUILD_ASSERT(DT_PROP_OR(child, renesas_wait_table, 0) <= 8U, \
		"ADCJ wait table selector must be in range 0..8"); \
	BUILD_ASSERT(DT_PROP_OR(child, renesas_upper_lower_limit_table, 0) <= 8U, \
		"ADCJ upper/lower limit selector must be in range 0..8"); \
	BUILD_ASSERT((ADCJ_CHILD_CONVERSION_TYPE_VALUE(child) != ADCJ_CONVERSION_TYPE_TRACK_HOLD) \
		|| (ADCJ_CHILD_PHYSICAL_CHANNEL(child) < 4U), \
		"ADCJ T&H channel must be T&H0..T&H3"); \
	BUILD_ASSERT((ADCJ_CHILD_CONVERSION_TYPE_VALUE(child) != \
		ADCJ_CONVERSION_TYPE_TRACK_HOLD) || \
		((ADCJ_CHILD_PARENT_TRACK_HOLD_MASK(child) & \
		BIT(ADCJ_CHILD_PHYSICAL_CHANNEL(child) & 0x1fU)) != 0U), \
		"ADCJ T&H channel must be enabled by renesas,track-hold-mask"); \
	BUILD_ASSERT(DT_PROP(child, zephyr_resolution) == \
		ADCJ_NODE_RESOLUTION(DT_PARENT(child)), \
		"ADCJ channel resolution must match data format"); \
	BUILD_ASSERT(!ADCJ_CHILD_IS_MPX(child) || \
		(ADCJ_CHILD_WAIT_TABLE(child) != 0U), \
		"ADCJ MPX conversion requires a wait table"); \
	BUILD_ASSERT(!ADCJ_CHILD_IS_MPX(child) || \
		(ADCJ_CHILD_SELECTED_WAIT_VALUE(child) >= \
		ADCJ_DIRECT_MPX_MIN_WAIT_CYCLES), \
		"ADCJ direct MPX requires at least 40 CLKAD wait cycles"); \
	BUILD_ASSERT(ADCJ_CHILD_IS_MPX(child) || \
		(ADCJ_CHILD_MPX_VALUE(child) == 0U), \
		"ADCJ MPX value must be zero for non-MPX conversion"); \
	BUILD_ASSERT(!(ADCJ_CHILD_IS_HOLD(child) || \
		ADCJ_CHILD_IS_AD_CORE_DIAG(child) || \
		ADCJ_CHILD_IS_ADVAL(child)) || \
		(ADCJ_CHILD_WAIT_TABLE(child) == 0U), \
		"ADCJ hold, AD-core diagnostic, and ADVAL conversions cannot use wait"); \
	BUILD_ASSERT(!ADCJ_CHILD_IS_MPX(child) || \
		!ADCJ_CHILD_MPX_USES_ENABLED_TH_CHANNEL(child), \
		"ADCJ MPX cannot use a T&H-enabled physical channel"); \
	BUILD_ASSERT(ADCJ_CHILD_IS_HOLD(child) || \
		ADCJ_CHILD_IS_AD_CORE_DIAG(child) || \
		ADCJ_CHILD_IS_ADVAL(child) || \
		!ADCJ_CHILD_INPUT_HAS_TH_ENABLED(child), \
		"ADCJ T&H-enabled inputs require hold-value conversion"); \
	BUILD_ASSERT(!ADCJ_CHILD_USES_PHYSICAL_INPUT(child) || \
		ADCJ_CHILD_PHYSICAL_CHANNEL_IS_VALID(child), \
		"ADCJ physical channel is unavailable on this instance"); \
	BUILD_ASSERT(!ADCJ_CHILD_IS_IO_DIAGNOSTIC(child) || \
		!ADCJ_CHILD_IS_VMON_INPUT(child), \
		"ADCJ pin-level and wiring-break diagnosis require an external input"); \
	BUILD_ASSERT(!(ADCJ_CHILD_IS_PIN_DIAG(child) && \
		ADCJ_CHILD_IS_LOW_ACCURACY_INPUT(child)) || \
		(DT_PROP(DT_PARENT(child), \
			renesas_pin_self_diagnostic_level) <= 1U), \
		"ADCJ low-accuracy pin diagnosis supports only TDLV 0 or 1"); \
	BUILD_ASSERT(                                                         \
		!DT_PROP_OR(child, renesas_limit_interrupt, 0) ||                     \
		(DT_PROP_OR(child, renesas_upper_lower_limit_table, 0) != 0U),  \
		"ADCJ limit interrupt requires a non-zero limit table");

#define ADCJ_CHILD_CHANNEL_TO_VC_INIT(child) \
	[DT_REG_ADDR(child)] = ADCJ_CHILD_VC(child),
#define ADCJ_CHILD_VC_CFG_INIT(child) \
	[ADCJ_CHILD_VC(child)] = { \
		.vcr_b.general_ctrl = ADCJ_CHILD_GENERAL_CTRL(child), \
		.vcr_b.channel_end_int = ADCJ_VC_END_INT_DISABLE, \
		.vcr_b.mpx_value = DT_PROP_OR(child, renesas_mpx_value, 0), \
		.vcr_b.conv_type = ADCJ_CHILD_VC_CONV_TYPE(child), \
		.vcr_b.wait_time_sel = DT_PROP_OR(child, renesas_wait_table, 0), \
		.vcr_b.ul_limit_sel = DT_PROP_OR(child, renesas_upper_lower_limit_table, 0), \
	},

#define ADCJ_IRQ_SG0(idx) DT_INST_IRQ_BY_NAME(idx, sg0, irq)
#define ADCJ_IRQ_SG1(idx) DT_INST_IRQ_BY_NAME(idx, sg1, irq)
#define ADCJ_IRQ_SG2(idx) DT_INST_IRQ_BY_NAME(idx, sg2, irq)
#define ADCJ_IRQ_SG3(idx) DT_INST_IRQ_BY_NAME(idx, sg3, irq)
#define ADCJ_IRQ_SG4(idx) DT_INST_IRQ_BY_NAME(idx, sg4, irq)
#define ADCJ_IRQ_PRIO_SG0(idx) DT_INST_IRQ_BY_NAME(idx, sg0, priority)
#define ADCJ_IRQ_PRIO_SG1(idx) DT_INST_IRQ_BY_NAME(idx, sg1, priority)
#define ADCJ_IRQ_PRIO_SG2(idx) DT_INST_IRQ_BY_NAME(idx, sg2, priority)
#define ADCJ_IRQ_PRIO_SG3(idx) DT_INST_IRQ_BY_NAME(idx, sg3, priority)
#define ADCJ_IRQ_PRIO_SG4(idx) DT_INST_IRQ_BY_NAME(idx, sg4, priority)
#define ADCJ_IRQ_FLAGS(idx) 0
#define ADCJ_IRQ_NAME_0 sg0
#define ADCJ_IRQ_NAME_1 sg1
#define ADCJ_IRQ_NAME_2 sg2
#define ADCJ_IRQ_NAME_3 sg3
#define ADCJ_IRQ_NAME_4 sg4
#define ADCJ_SELECTED_IRQ_NAME(idx) \
	UTIL_CAT(ADCJ_IRQ_NAME_, DT_INST_PROP(idx, renesas_scan_group))
#define ADCJ_SELECTED_IRQ(idx) \
	DT_INST_IRQ_BY_NAME(idx, ADCJ_SELECTED_IRQ_NAME(idx), irq)
#define ADCJ_SELECTED_IRQ_PRIO(idx) \
	DT_INST_IRQ_BY_NAME(idx, ADCJ_SELECTED_IRQ_NAME(idx), priority)

#define ADCJ_SCAN_GROUP_IS(idx, group) \
	IS_EQ(DT_INST_PROP(idx, renesas_scan_group), group)

#define ADCJ_SCAN_END_IRQ(idx, group) \
	COND_CODE_1(ADCJ_SCAN_GROUP_IS(idx, group), \
		(UTIL_CAT(ADCJ_IRQ_SG, group)(idx)), \
		(FSP_INVALID_VECTOR))

#define ADCJ_SCAN_END_IPL(idx, group) \
	COND_CODE_1(ADCJ_SCAN_GROUP_IS(idx, group), \
		(UTIL_CAT(ADCJ_IRQ_PRIO_SG, group)(idx)), \
		(BSP_IRQ_DISABLED))

#define ADCJ_HAS_ERROR_IRQ(idx) \
	DT_INST_IRQ_HAS_NAME(idx, error)

#define ADCJ_ERROR_IRQ(idx) \
	COND_CODE_1(ADCJ_HAS_ERROR_IRQ(idx), \
		(DT_INST_IRQ_BY_NAME(idx, error, irq)), \
		(FSP_INVALID_VECTOR))

#define ADCJ_ERROR_IPL(idx) \
	COND_CODE_1(ADCJ_HAS_ERROR_IRQ(idx), \
		(DT_INST_IRQ_BY_NAME(idx, error, priority)), \
		(BSP_IRQ_DISABLED))

#define ADCJ_CONNECT_ERROR_IRQ(idx) \
	COND_CODE_1(ADCJ_HAS_ERROR_IRQ(idx), \
		(IRQ_CONNECT(DT_INST_IRQ_BY_NAME(idx, error, irq), \
			DT_INST_IRQ_BY_NAME(idx, error, priority), \
			adc_adcj_error_dispatch_isr, \
			DEVICE_DT_INST_GET(idx), 0); \
		irq_enable(DT_INST_IRQ_BY_NAME(idx, error, irq));), \
		())

#define ADCJ_DEVICE_INIT(idx) \
	BUILD_ASSERT(ADCJ_START_VC(idx) <= ADCJ_END_VC(idx), \
		"ADCJ start virtual channel must not exceed end virtual channel"); \
	BUILD_ASSERT((ADCJ_TRACK_HOLD_MASK(idx) & ~ADCJ_TRACK_HOLD_MASK_VALID) == 0U, \
		"ADCJ track-hold mask can only enable T&H0..T&H3"); \
	BUILD_ASSERT(!ADCJ_TRIGGER_IS_AD_TIMER(idx) || \
		(DT_INST_PROP(idx, renesas_scan_group) >= 3U), \
		"ADCJ A/D timer is supported only by SG3 and SG4"); \
	BUILD_ASSERT(!ADCJ_TRIGGER_IS_TH(idx) || \
		(DT_INST_PROP(idx, renesas_scan_group) != 0U), \
		"ADCJ T&H trigger is unsupported by SG0"); \
	BUILD_ASSERT(DT_INST_PROP(idx, renesas_ad_timer_initial_phase) <= 0x1fffffU, \
		"ADCJ A/D timer initial phase exceeds the 21-bit field"); \
	BUILD_ASSERT(DT_INST_PROP(idx, renesas_ad_timer_cycle) <= 0x1fffffU, \
		"ADCJ A/D timer cycle exceeds the 21-bit field"); \
	BUILD_ASSERT(ADCJ_WAIT_TIME(idx, 0) <= ADCJ_WAIT_TIME_MAX, \
		"ADCJ WAITTR0 exceeds the 14-bit field"); \
	BUILD_ASSERT(ADCJ_WAIT_TIME(idx, 1) <= ADCJ_WAIT_TIME_MAX, \
		"ADCJ WAITTR1 exceeds the 14-bit field"); \
	BUILD_ASSERT(ADCJ_WAIT_TIME(idx, 2) <= ADCJ_WAIT_TIME_MAX, \
		"ADCJ WAITTR2 exceeds the 14-bit field"); \
	BUILD_ASSERT(ADCJ_WAIT_TIME(idx, 3) <= ADCJ_WAIT_TIME_MAX, \
		"ADCJ WAITTR3 exceeds the 14-bit field"); \
	BUILD_ASSERT(ADCJ_WAIT_TIME(idx, 4) <= ADCJ_WAIT_TIME_MAX, \
		"ADCJ WAITTR4 exceeds the 14-bit field"); \
	BUILD_ASSERT(ADCJ_WAIT_TIME(idx, 5) <= ADCJ_WAIT_TIME_MAX, \
		"ADCJ WAITTR5 exceeds the 14-bit field"); \
	BUILD_ASSERT(ADCJ_WAIT_TIME(idx, 6) <= ADCJ_WAIT_TIME_MAX, \
		"ADCJ WAITTR6 exceeds the 14-bit field"); \
	BUILD_ASSERT(ADCJ_WAIT_TIME(idx, 7) <= ADCJ_WAIT_TIME_MAX, \
		"ADCJ WAITTR7 exceeds the 14-bit field"); \
	BUILD_ASSERT(!(ADCJ_INSTANCE_HAS_HOLD(idx) && \
		ADCJ_INSTANCE_HAS_WAIT(idx)), \
		"ADCJ wait is prohibited in a scan group containing hold conversion"); \
	BUILD_ASSERT(ADCJ_NODE_WAIT_TABLE_COUNT(DT_DRV_INST(idx)) <= 8U, \
		"ADCJ supports at most 8 wait table entries"); \
	BUILD_ASSERT(!ADCJ_INSTANCE_USES_TH(idx) || \
		(DT_INST_PROP(idx, renesas_unit) < 2U), \
		"ADCJ T&H is supported only on ADCJ0 and ADCJ1"); \
	BUILD_ASSERT(!ADCJ_INSTANCE_USES_TH(idx) || \
		ADCJ_TRIGGER_IS_TH(idx), \
		"ADCJ T&H requires trigger-method th-a"); \
	BUILD_ASSERT(!ADCJ_TRIGGER_IS_TH(idx) || \
		ADCJ_INSTANCE_USES_TH(idx), \
		"ADCJ th-a requires a non-zero track-hold-mask"); \
	BUILD_ASSERT(!ADCJ_INSTANCE_USES_TH(idx) || \
		ADCJ_INSTANCE_HAS_HOLD(idx), \
		"ADCJ T&H requires at least one hold-value channel"); \
	BUILD_ASSERT(!ADCJ_INSTANCE_USES_TH(idx) || \
		(DT_INST_PROP(idx, renesas_scan_group) != 0U), \
		"ADCJ T&H cannot use SG0"); \
	BUILD_ASSERT(ADCJ_HOLD_CHANNEL_MASK(idx) == \
		ADCJ_TRACK_HOLD_MASK(idx), \
		"ADCJ enabled T&H circuits must match hold-value channels"); \
	BUILD_ASSERT(ADCJ_START_VC(idx) < \
		ADC_RENESAS_ADCJ_MAX_VIRTUAL_CHANNELS, \
		"ADCJ start VC is out of range"); \
	BUILD_ASSERT(ADCJ_END_VC(idx) < \
		ADC_RENESAS_ADCJ_MAX_VIRTUAL_CHANNELS, \
		"ADCJ end VC is out of range"); \
	BUILD_ASSERT(ADCJ_CONFIGURED_VC_MASK(idx) == \
		GENMASK64(ADCJ_END_VC(idx) & 0x3fU, \
		ADCJ_START_VC(idx) & 0x3fU), \
		"ADCJ must configure every VC in the scan range"); \
	BUILD_ASSERT(ADCJ_CHANNEL_CHILD_COUNT(idx) == \
		(ADCJ_END_VC(idx) - ADCJ_START_VC(idx) + 1U), \
		"ADCJ VC mappings must be unique"); \
	BUILD_ASSERT(ADCJ_HOLD_CHANNEL_COUNT(idx) == \
		POPCOUNT(ADCJ_HOLD_CHANNEL_MASK(idx)), \
		"ADCJ each T&H circuit may be used only once per scan"); \
	BUILD_ASSERT(DT_INST_PROP_LEN(idx, renesas_physical_channel_mask) == 2U, \
		"ADCJ physical-channel-mask must contain two cells"); \
	BUILD_ASSERT(DT_INST_PROP(idx, channel_available_mask) == \
		ADCJ_CONFIGURED_CHANNEL_MASK(idx), \
		"ADCJ channel-available-mask must match configured children"); \
	BUILD_ASSERT(                                                        \
		DT_INST_PROP_LEN(idx, renesas_lower_limit_values) ==           \
		ADCJ_UPPER_LOWER_LIMIT_TABLE_ENTRIES,                  \
		"ADCJ requires exactly eight lower-limit entries");           \
	BUILD_ASSERT(                                                        \
		DT_INST_PROP_LEN(idx, renesas_upper_limit_values) ==           \
		ADCJ_UPPER_LOWER_LIMIT_TABLE_ENTRIES,                  \
		"ADCJ requires exactly eight upper-limit entries");           \
	DT_FOREACH_PROP_ELEM(DT_DRV_INST(idx), renesas_lower_limit_values,    \
			ADCJ_LIMIT_VALUE_VALIDATE)                      \
	DT_FOREACH_PROP_ELEM(DT_DRV_INST(idx), renesas_upper_limit_values,    \
			ADCJ_LIMIT_VALUE_VALIDATE)                      \
	BUILD_ASSERT(                                                         \
		((ADCJ_LIMIT_IRQ_MASK(idx) == 0ULL) &&                           \
		!DT_INST_PROP_OR(idx, renesas_trigger_overlap_interrupt, 0)) ||  \
		ADCJ_HAS_ERROR_IRQ(idx),                               \
		"ADCJ enabled error sources require an error IRQ");            \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADCJ_CHILD_VALIDATE) \
	static const uint8_t adc_adcj_channel_to_vc_##idx \
		[ADC_RENESAS_ADCJ_MAX_ZEPHYR_CHANNELS] = { \
		ADCJ_CHANNEL_TO_VC_INVALID_INIT, \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADCJ_CHILD_CHANNEL_TO_VC_INIT) \
	}; \
	static const adcj_vc_cfg_t adc_adcj_vc_cfg_##idx \
		[ADC_RENESAS_ADCJ_MAX_VIRTUAL_CHANNELS] = { \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADCJ_CHILD_VC_CFG_INIT) \
	}; \
	static const adcj_sg_generic_cfg_t adc_adcj_sg_generic_cfg_##idx = { \
		.adcj_trigger_src = ADCJ0_TRIGGER_SOURCE(idx), \
		.trigger_method = ADCJ_TRIGGER_METHOD(idx), \
		.sg_ctrl = { \
			.sgcr_b.end_int = ADCJ_SG_END_INT_ENABLE, \
			.sgcr_b.scan_mode = ADCJ_SG_SCAN_MODE_MULTICYLE, \
		}, \
		.scan_cycle = 0U, \
		.start_vc_ptr = ADCJ_START_VC(idx), \
		.end_vc_ptr = ADCJ_END_VC(idx), \
		.p_vc_cfg = &adc_adcj_vc_cfg_##idx[ADCJ_START_VC(idx)], \
	}; \
	static const adcj_sg_cfg_t adc_adcj_sg_cfg_##idx = { \
		.p_sg_cfg = { \
			[DT_INST_PROP(idx, renesas_scan_group)] = \
				&adc_adcj_sg_generic_cfg_##idx, \
		}, \
		.pwm_diag_cfg = ADCJ_PWM_DIAG_CFG_DISABLE, \
		.tsn0_measure_cfg = ADCJ_TSN0_MEASURE_CFG_DISABLE, \
	}; \
	static const adcj_extended_cfg_t adc_adcj_extend_cfg_##idx = { \
		.suspend_method = ADCJ_INSTANCE_USES_TH(idx) ? \
				ADCJ_SUSPEND_METHOD_ASYNCHRONOUS : \
				ADCJ_SUSPEND_METHOD_SYNCHRONOUS, \
		.data_format_addition_count = { \
			.adcr2_b.addition_count = ADCJ_ADDITION_COUNT(idx), \
			.adcr2_b.data_format = ADCJ_DATA_FORMAT(idx), \
		}, \
		.safety_cfg = { \
			.sftcr_b.id = ADCJ_ID_ERROR_INT_DISABLE, \
			.sftcr_b.parity = ADCJ_PARITY_ERROR_INT_DISABLE, \
			.sftcr_b.overwrite = ADCJ_OVERWRITE_ERROR_INT_DISABLE, \
			.sftcr_b.read_clear = ADCJ_CLEAR_AFTER_READ_DISABLE, \
			.sftcr_b.trigger_overlap = \
				DT_INST_PROP_OR(idx, renesas_trigger_overlap_interrupt, 0) ? \
				ADCJ_TRIGGER_OVERLAP_INT_ENABLE : \
				ADCJ_TRIGGER_OVERLAP_INT_DISABLE, \
		}, \
		.ul_limit_cfg = { \
			.ul_limit_tb = { \
				ADCJ_LIMIT_TABLE_ENTRY(idx, 0), \
				ADCJ_LIMIT_TABLE_ENTRY(idx, 1), \
				ADCJ_LIMIT_TABLE_ENTRY(idx, 2), \
				ADCJ_LIMIT_TABLE_ENTRY(idx, 3), \
				ADCJ_LIMIT_TABLE_ENTRY(idx, 4), \
				ADCJ_LIMIT_TABLE_ENTRY(idx, 5), \
				ADCJ_LIMIT_TABLE_ENTRY(idx, 6), \
				ADCJ_LIMIT_TABLE_ENTRY(idx, 7), \
			}, \
			.vc_ul_limit_int_0 = \
				(uint32_t)ADCJ_LIMIT_IRQ_MASK(idx), \
			.vc_ul_limit_int_1 = \
				(uint32_t)(ADCJ_LIMIT_IRQ_MASK(idx) >> 32), \
			.pwm_diag_ul_limit_int = ADCJ_PWM_DIAG_UL_LIMIT_INT_DISABLE, \
		}, \
		.sampling_cfg = { \
			.smpcr_b.period = ADCJ_SAMPLING_PERIOD_18_STATES, \
			.smpcr_b.period_sel = ADCJ_SAMPLING_PERIOD_SEL_DISABLE, \
			.smpcr_b.buffer_amp = ADCJ_BUFFER_AMP_ENABLE, \
			.smpcr_b.extended_period = ADCJ_EXTENDED_SAMPLING_PERIOD_60_STATES, \
		}, \
		.pin_lv_self_diag_cfg = { \
			.tdcr_b.voltage_level = \
				(adcj_pin_lv_self_diag_vol_t) \
				DT_INST_PROP(idx, renesas_pin_self_diagnostic_level), \
			.tdcr_b.th_path_self_diag = ADCJ_TH_PATH_SELF_DIAG_DISABLE, \
		}, \
		.wiring_break_detect_cfg = { \
			.odcr_b.pulse_width = \
				(adcj_wiring_break_detect_pulse_width_t) \
				(DT_INST_PROP(idx, renesas_wiring_break_pulse_width) + 3U), \
			.odcr_b.addition_mode = ADCJ_WIRING_BREAK_ADDITION_MODE_DISABLE, \
		}, \
		.mpx_cfg = { \
			.mask_control_format = 0U, \
			.spi_command_information = 0U, \
			.mpx_int = ADCJ_MPX_INT_DISABLE, \
		}, \
		.trigger_overlap_detect = \
			DT_INST_PROP_OR(idx, renesas_trigger_overlap_interrupt, 0) ? \
			ADCJ_TRIGGER_OVERLAP_DETECT_ENABLE : \
			ADCJ_TRIGGER_OVERLAP_DETECT_DISABLE, \
		.th_cfg = { \
			.group_select_cfg = { \
				.thgsr_b.th0_group_cfg = ADCJ_TH_GROUP_A, \
				.thgsr_b.th1_group_cfg = ADCJ_TH_GROUP_A, \
				.thgsr_b.th2_group_cfg = ADCJ_TH_GROUP_A, \
				.thgsr_b.th3_group_cfg = ADCJ_TH_GROUP_A, \
			}, \
			.th_enable_cfg = { \
				.ther_b.th0_enable = ADCJ_TH_ENABLE_FROM_MASK(idx, 0), \
				.ther_b.th1_enable = ADCJ_TH_ENABLE_FROM_MASK(idx, 1), \
				.ther_b.th2_enable = ADCJ_TH_ENABLE_FROM_MASK(idx, 2), \
				.ther_b.th3_enable = ADCJ_TH_ENABLE_FROM_MASK(idx, 3), \
			}, \
			.trigger_method[ADCJ_TH_GROUP_A] = ADCJ_TH_GROUP_TRIGGER_METHOD_SW, \
		}, \
		.ad_timer_cfg = { \
			.trigger_method[0] = ADCJ_AD_TIMER_TRIGGER(idx), \
			.trigger_method[1] = ADCJ_AD_TIMER_TRIGGER(idx), \
			.initial_phase[0] = DT_INST_PROP(idx, renesas_ad_timer_initial_phase), \
			.initial_phase[1] = DT_INST_PROP(idx, renesas_ad_timer_initial_phase), \
			.cycle[0] = DT_INST_PROP(idx, renesas_ad_timer_cycle), \
			.cycle[1] = DT_INST_PROP(idx, renesas_ad_timer_cycle), \
		}, \
		.wait_setting = { \
			.wait_setting = { \
				[0] = ADCJ_WAIT_TIME(idx, 0), \
				[1] = ADCJ_WAIT_TIME(idx, 1), \
				[2] = ADCJ_WAIT_TIME(idx, 2), \
				[3] = ADCJ_WAIT_TIME(idx, 3), \
				[4] = ADCJ_WAIT_TIME(idx, 4), \
				[5] = ADCJ_WAIT_TIME(idx, 5), \
				[6] = ADCJ_WAIT_TIME(idx, 6), \
				[7] = ADCJ_WAIT_TIME(idx, 7), \
			}, \
	}, \
		.int_cfg = { \
			.error_irq = ADCJ_ERROR_IRQ(idx), \
			.error_ipl = ADCJ_ERROR_IPL(idx), \
			.scan_group_0_end_irq = ADCJ_SCAN_END_IRQ(idx, 0), \
			.scan_group_0_end_ipl = ADCJ_SCAN_END_IPL(idx, 0), \
			.scan_group_1_end_irq = ADCJ_SCAN_END_IRQ(idx, 1), \
			.scan_group_1_end_ipl = ADCJ_SCAN_END_IPL(idx, 1), \
			.scan_group_2_end_irq = ADCJ_SCAN_END_IRQ(idx, 2), \
			.scan_group_2_end_ipl = ADCJ_SCAN_END_IPL(idx, 2), \
			.scan_group_3_end_irq = ADCJ_SCAN_END_IRQ(idx, 3), \
			.scan_group_3_end_ipl = ADCJ_SCAN_END_IPL(idx, 3), \
			.scan_group_4_end_irq = ADCJ_SCAN_END_IRQ(idx, 4), \
			.scan_group_4_end_ipl = ADCJ_SCAN_END_IPL(idx, 4), \
		}, \
	}; \
	static const adc_cfg_t adc_adcj_fsp_cfg_##idx = { \
		.unit = DT_INST_PROP(idx, renesas_unit), \
		.mode = ADC_MODE_SINGLE_SCAN, \
		.resolution = ADCJ_FSP_RESOLUTION(idx), \
		.alignment = ADC_ALIGNMENT_RIGHT, \
		.trigger = ADC_TRIGGER_SOFTWARE, \
		.scan_end_irq = FSP_INVALID_VECTOR, \
		.scan_end_b_irq = FSP_INVALID_VECTOR, \
		.scan_end_c_irq = FSP_INVALID_VECTOR, \
		.scan_end_ipl = BSP_IRQ_DISABLED, \
		.scan_end_b_ipl = BSP_IRQ_DISABLED, \
		.scan_end_c_ipl = BSP_IRQ_DISABLED, \
		.p_callback = NULL, \
		.p_context = NULL, \
		.p_extend = &adc_adcj_extend_cfg_##idx, \
	}; \
	static void adc_adcj_irq_config_##idx(const struct device *dev) \
	{ \
		ARG_UNUSED(dev); \
		IRQ_CONNECT(ADCJ_SELECTED_IRQ(idx), ADCJ_SELECTED_IRQ_PRIO(idx), \
			    adc_adcj_scanend_dispatch_isr, DEVICE_DT_INST_GET(idx), \
			    ADCJ_IRQ_FLAGS(idx)); \
		irq_enable(ADCJ_SELECTED_IRQ(idx)); \
		ADCJ_CONNECT_ERROR_IRQ(idx); \
	} \
	static const struct adc_adcj_config adc_adcj_config_##idx = { \
		.channel_available_mask = DT_INST_PROP(idx, channel_available_mask), \
		.scan_group = DT_INST_PROP(idx, renesas_scan_group), \
		.resolution = ADCJ_RESOLUTION(idx), \
		.data_format = ADCJ_DATA_FORMAT(idx), \
		.track_hold_mask = ADCJ_TRACK_HOLD_MASK(idx), \
		.software_trigger = ADCJ_TRIGGER_IS_SOFTWARE_STARTED(idx), \
		.uses_th = ADCJ_INSTANCE_USES_TH(idx), \
		.scan_group_mask = ADCJ_GROUP_MASK_FROM_GROUP(idx), \
		.fsp_cfg = &adc_adcj_fsp_cfg_##idx, \
		.fsp_sg_cfg = &adc_adcj_sg_cfg_##idx, \
		.channel_to_vc = adc_adcj_channel_to_vc_##idx, \
		.irq_config_func = adc_adcj_irq_config_##idx, \
	}; \
	static struct adc_adcj_data adc_adcj_data_##idx = { \
		ADC_CONTEXT_INIT_TIMER(adc_adcj_data_##idx, ctx), \
		ADC_CONTEXT_INIT_LOCK(adc_adcj_data_##idx, ctx), \
		ADC_CONTEXT_INIT_SYNC(adc_adcj_data_##idx, ctx), \
		.dev = DEVICE_DT_INST_GET(idx), \
	}; \
	static DEVICE_API(adc, adc_adcj_api_##idx) = { \
		.channel_setup = adc_adcj_channel_setup, \
		.read = adc_adcj_read, \
		.ref_internal = DT_INST_PROP_OR(idx, vref_mv, 0), \
		IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = adc_adcj_read_async,)) \
	}; \
	DEVICE_DT_INST_DEFINE(idx, adc_adcj_init, NULL, &adc_adcj_data_##idx, \
		&adc_adcj_config_##idx, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, \
		&adc_adcj_api_##idx);

DT_INST_FOREACH_STATUS_OKAY(ADCJ_DEVICE_INIT)
