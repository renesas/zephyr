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

/*
 * ADCJ ADCR2.DFMT encodings supported by the Zephyr driver.
 * Signed, fixed-point, and left-aligned formats are intentionally excluded.
 */
#define ADCJ_DATA_FORMAT_10_bit 3U
#define ADCJ_DATA_FORMAT_12_bit 4U

#define ADCJ_RESOLUTION_10_bit 10U
#define ADCJ_RESOLUTION_12_bit 12U

#define ADCJ_FSP_RESOLUTION_10_bit ADC_RESOLUTION_10_BIT
#define ADCJ_FSP_RESOLUTION_12_bit ADC_RESOLUTION_12_BIT

/* FSP ADCJ has 64 virtual channel data registers. */
#define ADC_RENESAS_ADCJ_MAX_VIRTUAL_CHANNELS ADCJ_NUMBER_OF_VIRTUAL_CHANNEL

/*
 * ADCJ VCR.GCTRL is 6 bits wide. The valid selector range is common for
 * normal and hold-value conversions; package-specific pin availability is
 * described by board DTS instead of being hard-coded in the driver.
 */
#define ADC_RENESAS_ADCJ_MAX_PHYSICAL_CHANNELS 64U

/*
 * ADCJ hold-value A/D conversion is encoded by CNVCLS = 8. Some FSP
 * versions do not expose a symbolic name for this value, so keep the driver
 * local constant instead of depending on generated headers.
 */
#define ADCJ_VC_CONV_TYPE_HOLD_VALUE 8U

#define ADCJ_CONVERSION_TYPE_NORMAL      0U
#define ADCJ_CONVERSION_TYPE_TRACK_HOLD  1U

#define ADCJ_TRACK_HOLD_MASK_TH0 BIT(0)
#define ADCJ_TRACK_HOLD_MASK_TH1 BIT(1)
#define ADCJ_TRACK_HOLD_MASK_TH2 BIT(2)
#define ADCJ_TRACK_HOLD_MASK_TH3 BIT(3)
#define ADCJ_TRACK_HOLD_MASK_VALID \
	(ADCJ_TRACK_HOLD_MASK_TH0 | ADCJ_TRACK_HOLD_MASK_TH1 | \
	 ADCJ_TRACK_HOLD_MASK_TH2 | ADCJ_TRACK_HOLD_MASK_TH3)

#define ADCJ_TH_ENABLE(idx, th) \
	(((ADCJ_TRACK_HOLD_MASK(idx) & BIT(th)) != 0U) ? \
	 ADC_TH_UNIT_CFG_ENABLE : ADC_TH_UNIT_CFG_DISABLE)

extern void adcj_sg0_scan_end_isr(void);
extern void adcj_sg1_scan_end_isr(void);
extern void adcj_sg2_scan_end_isr(void);
extern void adcj_sg3_scan_end_isr(void);
extern void adcj_sg4_scan_end_isr(void);

struct adc_adcj_config {
	uint32_t channel_available_mask;
	uint8_t scan_group;
	uint8_t resolution;
	uint8_t track_hold_mask;
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

	if ((sequence->resolution != 10U) && (sequence->resolution != 12U)) {
		LOG_ERR("Unsupported resolution %u", sequence->resolution);
		return -ENOTSUP;
	}

	if (sequence->resolution != config->resolution) {
		LOG_ERR("Requested resolution %u does not match ADCJ FSP data format resolution %u",
			sequence->resolution, config->resolution);
		return -ENOTSUP;
	}

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

static int adc_adcj_read_channels(const struct device *dev)
{
	const struct adc_adcj_config *config = dev->config;
	struct adc_adcj_data *data = dev->data;
	uint32_t channels = data->channels;
	uint16_t *buffer = data->buffer;
	adc_channel_t ch;

	for (ch = 0; channels != 0U; ch++, channels >>= 1) {
		if ((channels & 0x1U) == 0U) {
			continue;
		}

		uint8_t vc = config->channel_to_vc[ch];
		fsp_err_t fsp_err;

		if (vc >= ADC_RENESAS_ADCJ_MAX_VIRTUAL_CHANNELS) {
			return -EINVAL;
		}

		fsp_err = R_ADCJ_Read(&data->fsp_ctrl, (adc_channel_t)vc, buffer);
		if (fsp_err != FSP_SUCCESS) {
			return adc_adcj_fsp_err_to_errno(fsp_err);
		}

		buffer++;
	}

	return 0;
}

static void adc_adcj_fsp_callback(adc_callback_args_t *args)
{
	const struct device *dev = args->p_context;
	struct adc_adcj_data *data = dev->data;
	int err;

	if (!data->active) {
		return;
	}

	err = adc_adcj_read_channels(dev);
	if (err != 0) {
		adc_context_complete(&data->ctx, err);
		return;
	}

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

	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_adcj_read_async(const struct device *dev,
			       const struct adc_sequence *sequence,
			       struct k_poll_signal *async)
{
	struct adc_adcj_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, async != NULL, async);
	err = adc_adcj_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

static int adc_adcj_read(const struct device *dev,
			 const struct adc_sequence *sequence)
{
	return adc_adcj_read_async(dev, sequence, NULL);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_adcj_data *data = CONTAINER_OF(ctx, struct adc_adcj_data, ctx);
	const struct device *dev = data->dev;
	const struct adc_adcj_config *config = dev->config;
	fsp_err_t fsp_err;

	data->channels = ctx->sequence.channels;
	data->active = true;

	fsp_err = R_ADCJ_ScanGroupStart(&data->fsp_ctrl, config->scan_group_mask);
	if (fsp_err != FSP_SUCCESS) {
		int err = adc_adcj_fsp_err_to_errno(fsp_err);

		data->active = false;
		adc_context_complete(ctx, err);
	}
}

static void adc_context_on_complete(struct adc_context *ctx, int status)
{
	struct adc_adcj_data *data = CONTAINER_OF(ctx, struct adc_adcj_data, ctx);
	const struct device *dev = data->dev;
	const struct adc_adcj_config *config = dev->config;

	ARG_UNUSED(status);

	data->active = false;
	(void)R_ADCJ_ScanGroupStop(&data->fsp_ctrl, config->scan_group_mask);
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

	data->dev = dev;

	if (config->irq_config_func != NULL) {
		config->irq_config_func(dev);
	}

	fsp_err = R_ADCJ_Open(&data->fsp_ctrl, config->fsp_cfg);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADCJ_Open failed: %d", fsp_err);
		return adc_adcj_fsp_err_to_errno(fsp_err);
	}

	fsp_err = R_ADCJ_CallbackSet(&data->fsp_ctrl, adc_adcj_fsp_callback,
				       (void *)dev, &data->callback_memory);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADCJ_CallbackSet failed: %d", fsp_err);
		return adc_adcj_fsp_err_to_errno(fsp_err);
	}

	fsp_err = R_ADCJ_ScanCfg(&data->fsp_ctrl, config->fsp_sg_cfg);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADCJ_ScanCfg failed: %d", fsp_err);
		return adc_adcj_fsp_err_to_errno(fsp_err);
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

#define ADCJ_DATA_FORMAT(idx) \
	UTIL_CAT(ADCJ_DATA_FORMAT_, DT_INST_STRING_TOKEN(idx, renesas_data_format))
#define ADCJ_RESOLUTION(idx) \
	UTIL_CAT(ADCJ_RESOLUTION_, DT_INST_STRING_TOKEN(idx, renesas_data_format))
#define ADCJ_NODE_RESOLUTION(node_id) \
	UTIL_CAT(ADCJ_RESOLUTION_, DT_STRING_TOKEN(node_id, renesas_data_format))
#define ADCJ_FSP_RESOLUTION(idx) \
	UTIL_CAT(ADCJ_FSP_RESOLUTION_, DT_INST_STRING_TOKEN(idx, renesas_data_format))

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
	DT_STRING_TOKEN_OR(child, renesas_conversion_type, normal)
#define ADCJ_CHILD_PARENT_TRACK_HOLD_MASK(child) \
	DT_PROP_OR(DT_PARENT(child), renesas_track_hold_mask, 0)
#define ADCJ_CONVERSION_TYPE_normal ADCJ_CONVERSION_TYPE_NORMAL
#define ADCJ_CONVERSION_TYPE_track_hold ADCJ_CONVERSION_TYPE_TRACK_HOLD
#define ADCJ_CHILD_CONVERSION_TYPE_VALUE(child) \
	UTIL_CAT(ADCJ_CONVERSION_TYPE_, ADCJ_CHILD_CONVERSION_TYPE(child))
#define ADCJ_VC_CONV_TYPE_FROM_TYPE_normal ADCJ_VC_CONV_TYPE_NORMAL
#define ADCJ_VC_CONV_TYPE_FROM_TYPE_track_hold ADCJ_VC_CONV_TYPE_HOLD_VALUE
#define ADCJ_CHILD_VC_CONV_TYPE(child) \
	UTIL_CAT(ADCJ_VC_CONV_TYPE_FROM_TYPE_, ADCJ_CHILD_CONVERSION_TYPE(child))
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
	BUILD_ASSERT((ADCJ_CHILD_CONVERSION_TYPE_VALUE(child) == ADCJ_CONVERSION_TYPE_NORMAL) || \
		     (ADCJ_CHILD_PHYSICAL_CHANNEL(child) < 4U), \
		     "ADCJ T&H channel must be T&H0..T&H3"); \
	BUILD_ASSERT((ADCJ_CHILD_CONVERSION_TYPE_VALUE(child) == ADCJ_CONVERSION_TYPE_NORMAL) || \
		     ((ADCJ_CHILD_PARENT_TRACK_HOLD_MASK(child) & \
		       BIT(ADCJ_CHILD_PHYSICAL_CHANNEL(child))) != 0U), \
		     "ADCJ T&H channel must be enabled by renesas,track-hold-mask"); \
	BUILD_ASSERT(DT_PROP(child, zephyr_resolution) == \
		     ADCJ_NODE_RESOLUTION(DT_PARENT(child)), \
		     "ADCJ channel resolution must match data format");

#define ADCJ_CHILD_CHANNEL_TO_VC_INIT(child) \
	[DT_REG_ADDR(child)] = ADCJ_CHILD_VC(child),
#define ADCJ_CHILD_VC_CFG_INIT(child) \
	[ADCJ_CHILD_VC(child)] = { \
		.vcr_b.general_ctrl = ADCJ_CHILD_PHYSICAL_CHANNEL(child), \
		.vcr_b.channel_end_int = ADCJ_VC_END_INT_DISABLE, \
		.vcr_b.mpx_value = 0U, \
		.vcr_b.conv_type = ADCJ_CHILD_VC_CONV_TYPE(child), \
		.vcr_b.wait_time_sel = ADCJ_WAIT_TIME_TABLE_DISABLE, \
		.vcr_b.ul_limit_sel = ADCJ_UL_LIMIT_TABLE_DISABLE, \
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

#define ADCJ_DEVICE_INIT(idx) \
	BUILD_ASSERT(ADCJ_START_VC(idx) <= ADCJ_END_VC(idx), \
		     "ADCJ start virtual channel must not exceed end virtual channel"); \
	BUILD_ASSERT((ADCJ_TRACK_HOLD_MASK(idx) & ~ADCJ_TRACK_HOLD_MASK_VALID) == 0U, \
		     "ADCJ track-hold mask can only enable T&H0..T&H3"); \
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
		.adcj_trigger_src = ADCJ_ADCJ0_TRIGGER_SRC_DISABLE, \
		.trigger_method = ADCJ_SG_TRIGGER_METHOD_SW, \
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
		.suspend_method = ADCJ_SUSPEND_METHOD_SYNCHRONOUS, \
		.data_format_addition_count = { \
			.adcr2_b.data_format = ADCJ_DATA_FORMAT(idx), \
		}, \
		.safety_cfg = { \
			.sftcr_b.id = ADCJ_ID_ERROR_INT_DISABLE, \
			.sftcr_b.parity = ADCJ_PARITY_ERROR_INT_DISABLE, \
			.sftcr_b.overwrite = ADCJ_OVERWRITE_ERROR_INT_DISABLE, \
			.sftcr_b.read_clear = ADCJ_CLEAR_AFTER_READ_DISABLE, \
			.sftcr_b.trigger_overlap = ADCJ_TRIGGER_OVERLAP_INT_DISABLE, \
		}, \
		.ul_limit_cfg = { \
			.vc_ul_limit_int_0 = 0U, \
			.vc_ul_limit_int_1 = 0U, \
			.pwm_diag_ul_limit_int = ADCJ_PWM_DIAG_UL_LIMIT_INT_DISABLE, \
		}, \
		.sampling_cfg = { \
			.smpcr_b.period = ADCJ_SAMPLING_PERIOD_18_STATES, \
			.smpcr_b.period_sel = ADCJ_SAMPLING_PERIOD_SEL_DISABLE, \
			.smpcr_b.buffer_amp = ADCJ_BUFFER_AMP_ENABLE, \
			.smpcr_b.extended_period = ADCJ_EXTENDED_SAMPLING_PERIOD_60_STATES, \
		}, \
		.pin_lv_self_diag_cfg = { \
			.tdcr_b.voltage_level = ADCJ_PIN_LV_SELF_DIAG_VOL_EVEN_AVSS_ODD_AVCC, \
			.tdcr_b.th_path_self_diag = ADCJ_TH_PATH_SELF_DIAG_DISABLE, \
		}, \
		.wiring_break_detect_cfg = { \
			.odcr_b.pulse_width = ADCJ_WIRING_BREAK_DETECT_PULSE_WIDTH_1_STATE, \
			.odcr_b.addition_mode = ADCJ_WIRING_BREAK_ADDITION_MODE_DISABLE, \
		}, \
		.mpx_cfg = { \
			.mask_control_format = 0U, \
			.spi_command_information = 0U, \
			.mpx_int = ADCJ_MPX_INT_DISABLE, \
		}, \
		.trigger_overlap_detect = ADCJ_TRIGGER_OVERLAP_DETECT_DISABLE, \
		.th_cfg = { \
			.group_select_cfg = { \
				.thgsr_b.th0_group_cfg = ADCJ_TH_GROUP_A, \
				.thgsr_b.th1_group_cfg = ADCJ_TH_GROUP_A, \
				.thgsr_b.th2_group_cfg = ADCJ_TH_GROUP_A, \
				.thgsr_b.th3_group_cfg = ADCJ_TH_GROUP_A, \
			}, \
			.th_enable_cfg = { \
				.ther_b.th0_enable = ADCJ_TH_ENABLE(idx, 0), \
				.ther_b.th1_enable = ADCJ_TH_ENABLE(idx, 1), \
				.ther_b.th2_enable = ADCJ_TH_ENABLE(idx, 2), \
				.ther_b.th3_enable = ADCJ_TH_ENABLE(idx, 3), \
			}, \
			.trigger_method[ADCJ_TH_GROUP_A] = ADCJ_TH_GROUP_TRIGGER_METHOD_SW, \
			.trigger_method[ADCJ_TH_GROUP_B] = ADCJ_TH_GROUP_TRIGGER_METHOD_SW, \
		}, \
		.ad_timer_cfg = { \
			.trigger_method[0] = ADCJ_AD_TIMER_TRIGGER_METHOD_SW, \
			.trigger_method[1] = ADCJ_AD_TIMER_TRIGGER_METHOD_SW, \
			.initial_phase[0] = 0U, \
			.initial_phase[1] = 0U, \
			.cycle[0] = 0x1FFFFFU, \
			.cycle[1] = 0x1FFFFFU, \
		}, \
		.int_cfg = { \
			.error_irq = FSP_INVALID_VECTOR, \
			.error_ipl = BSP_IRQ_DISABLED, \
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
	} \
	static const struct adc_adcj_config adc_adcj_config_##idx = { \
		.channel_available_mask = DT_INST_PROP(idx, channel_available_mask), \
		.scan_group = DT_INST_PROP(idx, renesas_scan_group), \
		.resolution = ADCJ_RESOLUTION(idx), \
		.track_hold_mask = ADCJ_TRACK_HOLD_MASK(idx), \
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
