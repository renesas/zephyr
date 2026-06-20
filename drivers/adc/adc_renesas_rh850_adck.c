/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_adck

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

#include "r_adck.h"

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE
#include "adc_context.h"

LOG_MODULE_REGISTER(adc_renesas_rh850_adck, CONFIG_ADC_LOG_LEVEL);

#define ADC_RENESAS_ADCK_INVALID_VC           0xffU
#define ADC_RENESAS_ADCK_MAX_ZEPHYR_CHANNELS  32U
#define ADC_RENESAS_ADCK_MAX_PHYSICAL_CHANNELS 64U
#define ADC_RENESAS_ADCK_PHYSICAL_MASK_WORDS   2U

#ifndef ADCK_VC_CONV_TYPE_HOLD
#ifdef ADCK_VC_CONV_TYPE_HOLD_VALUE
#define ADCK_VC_CONV_TYPE_HOLD ADCK_VC_CONV_TYPE_HOLD_VALUE
#else
#define ADCK_VC_CONV_TYPE_HOLD 1U
#endif
#endif

#ifndef ADC_TH_UNIT_CFG_ENABLE
#define ADC_TH_UNIT_CFG_ENABLE 1U
#endif

/* FSP ADCK has 64 virtual channel data registers. */
#define ADC_RENESAS_ADCK_MAX_VIRTUAL_CHANNELS ADCK_NUMBER_OF_VIRTUAL_CHANNEL

extern void adck_sg0_scan_end_isr(void);
extern void adck_sg1_scan_end_isr(void);
extern void adck_sg2_scan_end_isr(void);
extern void adck_sg3_scan_end_isr(void);
extern void adck_sg4_scan_end_isr(void);

struct adc_adck_config {
	uint32_t channel_available_mask;
	uint8_t scan_group;
	uint8_t resolution;
	adc_group_mask_t scan_group_mask;

	const adc_cfg_t *fsp_cfg;
	const adck_sg_cfg_t *fsp_sg_cfg;
	const uint8_t *channel_to_vc;

	void (*irq_config_func)(const struct device *dev);
};

struct adc_adck_data {
	struct adc_context ctx;
	const struct device *dev;

	adck_instance_ctrl_t fsp_ctrl;
	adc_callback_args_t callback_memory;

	uint16_t *buffer;
	uint32_t channels;
	uint32_t configured_channels;
	bool active;
	int status;
};

static int adc_adck_fsp_err_to_errno(fsp_err_t err)
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

static int adc_adck_channel_setup(const struct device *dev,
				  const struct adc_channel_cfg *channel_cfg)
{
	struct adc_adck_data *data = dev->data;
	const struct adc_adck_config *config = dev->config;
	uint8_t channel_id = channel_cfg->channel_id;

	if (channel_id >= ADC_RENESAS_ADCK_MAX_ZEPHYR_CHANNELS) {
		LOG_ERR("Unsupported channel id %u", channel_id);
		return -EINVAL;
	}

	if ((config->channel_available_mask & BIT(channel_id)) == 0U) {
		LOG_ERR("Channel %u is not available", channel_id);
		return -ENOTSUP;
	}

	if (config->channel_to_vc[channel_id] >= ADC_RENESAS_ADCK_MAX_VIRTUAL_CHANNELS) {
		LOG_ERR("Channel %u is not mapped to a valid ADCK virtual channel", channel_id);
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
	 * ADCK reference is board/SoC analog reference dependent. The driver accepts
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

static int adc_adck_check_buffer_size(const struct adc_sequence *sequence)
{
	size_t needed = POPCOUNT(sequence->channels) * sizeof(uint16_t);

	if (sequence->options != NULL) {
		needed *= (1U + sequence->options->extra_samplings);
	}

	return (sequence->buffer_size < needed) ? -ENOMEM : 0;
}

static int adc_adck_validate_sequence(const struct device *dev,
				      const struct adc_sequence *sequence)
{
	const struct adc_adck_config *config = dev->config;
	struct adc_adck_data *data = dev->data;
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
		LOG_ERR("Requested resolution %u does not match ADCK FSP data format resolution %u",
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

	err = adc_adck_check_buffer_size(sequence);
	if (err != 0) {
		LOG_ERR("ADC buffer is too small");
		return err;
	}

	return 0;
}

static int adc_adck_read_channels(const struct device *dev)
{
	const struct adc_adck_config *config = dev->config;
	struct adc_adck_data *data = dev->data;
	uint32_t channels = data->channels;
	uint16_t *buffer = data->buffer;
	adc_channel_t ch;

	for (ch = 0; channels != 0U; ch++, channels >>= 1) {
		if ((channels & 0x1U) == 0U) {
			continue;
		}

		uint8_t vc = config->channel_to_vc[ch];
		fsp_err_t fsp_err;

		if (vc >= ADC_RENESAS_ADCK_MAX_VIRTUAL_CHANNELS) {
			return -EINVAL;
		}

		fsp_err = R_ADCK_Read(&data->fsp_ctrl, (adc_channel_t)vc, buffer);
		if (fsp_err != FSP_SUCCESS) {
			return adc_adck_fsp_err_to_errno(fsp_err);
		}

		buffer++;
	}

	return 0;
}

static void adc_adck_fsp_callback(adc_callback_args_t *args)
{
	const struct device *dev = args->p_context;
	struct adc_adck_data *data = dev->data;
	int err;

	if (!data->active) {
		return;
	}

	err = adc_adck_read_channels(dev);
	if (err != 0) {
		data->status = err;
		data->active = false;
		adc_context_complete(&data->ctx, err);
		return;
	}

	data->status = 0;
	adc_context_on_sampling_done(&data->ctx, dev);
}

static int adc_adck_start_read(const struct device *dev,
			       const struct adc_sequence *sequence)
{
	struct adc_adck_data *data = dev->data;
	int err;

	err = adc_adck_validate_sequence(dev, sequence);
	if (err != 0) {
		return err;
	}

	data->status = 0;
	data->active = false;
	data->buffer = sequence->buffer;
	data->channels = sequence->channels;

	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_adck_read_async(const struct device *dev,
			       const struct adc_sequence *sequence,
			       struct k_poll_signal *async)
{
	struct adc_adck_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, async != NULL, async);
	err = adc_adck_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

static int adc_adck_read(const struct device *dev,
			 const struct adc_sequence *sequence)
{
	return adc_adck_read_async(dev, sequence, NULL);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_adck_data *data = CONTAINER_OF(ctx, struct adc_adck_data, ctx);
	const struct device *dev = data->dev;
	const struct adc_adck_config *config = dev->config;
	fsp_err_t fsp_err;

	data->channels = ctx->sequence.channels;
	data->active = true;

	fsp_err = R_ADCK_ScanGroupStart(&data->fsp_ctrl, config->scan_group_mask);
	if (fsp_err != FSP_SUCCESS) {
		int err = adc_adck_fsp_err_to_errno(fsp_err);

		data->status = err;
		data->active = false;
		adc_context_complete(ctx, err);
	}
}

static void adc_context_on_complete(struct adc_context *ctx, int status)
{
	struct adc_adck_data *data = CONTAINER_OF(ctx, struct adc_adck_data, ctx);
	const struct device *dev = data->dev;
	const struct adc_adck_config *config = dev->config;

	ARG_UNUSED(status);

	data->active = false;
	(void)R_ADCK_ScanGroupStop(&data->fsp_ctrl, config->scan_group_mask);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_adck_data *data = CONTAINER_OF(ctx, struct adc_adck_data, ctx);

	if (!repeat_sampling) {
		data->buffer += POPCOUNT(ctx->sequence.channels);
	}
}

static void adc_adck_scanend_dispatch_isr(const struct device *dev)
{
	const struct adc_adck_config *config = dev->config;

	switch (config->scan_group) {
	case 0U:
		adck_sg0_scan_end_isr();
		break;
	case 1U:
		adck_sg1_scan_end_isr();
		break;
	case 2U:
		adck_sg2_scan_end_isr();
		break;
	case 3U:
		adck_sg3_scan_end_isr();
		break;
	case 4U:
		adck_sg4_scan_end_isr();
		break;
	default:
		break;
	}
}

static int adc_adck_init(const struct device *dev)
{
	const struct adc_adck_config *config = dev->config;
	struct adc_adck_data *data = dev->data;
	fsp_err_t fsp_err;
	int err;

	data->dev = dev;
	data->active = false;

	if (config->irq_config_func != NULL) {
		config->irq_config_func(dev);
	}

	fsp_err = R_ADCK_Open(&data->fsp_ctrl, config->fsp_cfg);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADCK_Open failed: %d", fsp_err);
		return adc_adck_fsp_err_to_errno(fsp_err);
	}

	fsp_err = R_ADCK_CallbackSet(&data->fsp_ctrl, adc_adck_fsp_callback,
				       (void *)dev, &data->callback_memory);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADCK_CallbackSet failed: %d", fsp_err);
		return adc_adck_fsp_err_to_errno(fsp_err);
	}

	fsp_err = R_ADCK_ScanCfg(&data->fsp_ctrl, config->fsp_sg_cfg);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADCK_ScanCfg failed: %d", fsp_err);
		return adc_adck_fsp_err_to_errno(fsp_err);
	}

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define ADCK_GROUP_MASK_0 ADC_GROUP_MASK_0
#define ADCK_GROUP_MASK_1 ADC_GROUP_MASK_1
#define ADCK_GROUP_MASK_2 ADC_GROUP_MASK_2
#define ADCK_GROUP_MASK_3 ADC_GROUP_MASK_3
#define ADCK_GROUP_MASK_4 ADC_GROUP_MASK_4
#define ADCK_GROUP_MASK_FROM_GROUP(idx) \
	UTIL_CAT(ADCK_GROUP_MASK_, DT_INST_PROP(idx, renesas_scan_group))

/* ADCR2.DFMT values defined by the RH850 ADCK FSP interface. */
enum adc_adck_data_format {
	ADC_ADCK_DATA_FORMAT_10_BIT_UNSIGNED = 3U,
	ADC_ADCK_DATA_FORMAT_12_BIT_UNSIGNED_RIGHT = 4U,
};

#define ADCK_DATA_FORMAT_10_bit ADC_ADCK_DATA_FORMAT_10_BIT_UNSIGNED
#define ADCK_DATA_FORMAT_12_bit ADC_ADCK_DATA_FORMAT_12_BIT_UNSIGNED_RIGHT
#define ADCK_RESOLUTION_10_bit 10U
#define ADCK_RESOLUTION_12_bit 12U
#define ADCK_FSP_RESOLUTION_10_bit ADC_RESOLUTION_10_BIT
#define ADCK_FSP_RESOLUTION_12_bit ADC_RESOLUTION_12_BIT
#define ADCK_DATA_FORMAT(idx) \
	UTIL_CAT(ADCK_DATA_FORMAT_, DT_INST_STRING_TOKEN(idx, renesas_data_format))
#define ADCK_RESOLUTION(idx) \
	UTIL_CAT(ADCK_RESOLUTION_, DT_INST_STRING_TOKEN(idx, renesas_data_format))
#define ADCK_FSP_RESOLUTION(idx) \
	UTIL_CAT(ADCK_FSP_RESOLUTION_, DT_INST_STRING_TOKEN(idx, renesas_data_format))

#define ADCK_CHANNEL_TO_VC_INVALID_INIT \
	[0] = ADC_RENESAS_ADCK_INVALID_VC, \
	[1] = ADC_RENESAS_ADCK_INVALID_VC, \
	[2] = ADC_RENESAS_ADCK_INVALID_VC, \
	[3] = ADC_RENESAS_ADCK_INVALID_VC, \
	[4] = ADC_RENESAS_ADCK_INVALID_VC, \
	[5] = ADC_RENESAS_ADCK_INVALID_VC, \
	[6] = ADC_RENESAS_ADCK_INVALID_VC, \
	[7] = ADC_RENESAS_ADCK_INVALID_VC, \
	[8] = ADC_RENESAS_ADCK_INVALID_VC, \
	[9] = ADC_RENESAS_ADCK_INVALID_VC, \
	[10] = ADC_RENESAS_ADCK_INVALID_VC, \
	[11] = ADC_RENESAS_ADCK_INVALID_VC, \
	[12] = ADC_RENESAS_ADCK_INVALID_VC, \
	[13] = ADC_RENESAS_ADCK_INVALID_VC, \
	[14] = ADC_RENESAS_ADCK_INVALID_VC, \
	[15] = ADC_RENESAS_ADCK_INVALID_VC, \
	[16] = ADC_RENESAS_ADCK_INVALID_VC, \
	[17] = ADC_RENESAS_ADCK_INVALID_VC, \
	[18] = ADC_RENESAS_ADCK_INVALID_VC, \
	[19] = ADC_RENESAS_ADCK_INVALID_VC, \
	[20] = ADC_RENESAS_ADCK_INVALID_VC, \
	[21] = ADC_RENESAS_ADCK_INVALID_VC, \
	[22] = ADC_RENESAS_ADCK_INVALID_VC, \
	[23] = ADC_RENESAS_ADCK_INVALID_VC, \
	[24] = ADC_RENESAS_ADCK_INVALID_VC, \
	[25] = ADC_RENESAS_ADCK_INVALID_VC, \
	[26] = ADC_RENESAS_ADCK_INVALID_VC, \
	[27] = ADC_RENESAS_ADCK_INVALID_VC, \
	[28] = ADC_RENESAS_ADCK_INVALID_VC, \
	[29] = ADC_RENESAS_ADCK_INVALID_VC, \
	[30] = ADC_RENESAS_ADCK_INVALID_VC, \
	[31] = ADC_RENESAS_ADCK_INVALID_VC

#define ADCK_CHILD_PHYSICAL_CHANNEL(child) \
	DT_PROP_OR(child, renesas_physical_channel, DT_REG_ADDR(child))

#define ADCK_START_VC(idx) \
	DT_INST_PROP_OR(idx, renesas_start_virtual_channel, 0)

#define ADCK_END_VC(idx) \
	DT_INST_PROP_OR(idx, renesas_end_virtual_channel, 0)

#define ADCK_CHILD_VC(child) \
	DT_PROP_OR(child, renesas_virtual_channel, DT_REG_ADDR(child))

/*
 * Keep the bit index valid for both mask words. Some compilers still parse
 * both branches of preprocessor condition helpers, which can otherwise form
 * BIT(channel - 32) for a channel below 32 and report an invalid shift.
 */
#define ADCK_CHILD_PHYSICAL_CHANNEL_MASK(child) \
	((ADCK_CHILD_PHYSICAL_CHANNEL(child) < 32U) ? \
	 DT_PROP_BY_IDX(DT_PARENT(child), renesas_physical_channel_mask, 0) : \
	 DT_PROP_BY_IDX(DT_PARENT(child), renesas_physical_channel_mask, 1))

#define ADCK_CHILD_PHYSICAL_CHANNEL_BIT(child) \
	BIT(ADCK_CHILD_PHYSICAL_CHANNEL(child) & 0x1fU)

#define ADCK_CHILD_PHYSICAL_CHANNEL_IS_VALID(child) \
	((ADCK_CHILD_PHYSICAL_CHANNEL_MASK(child) & \
	  ADCK_CHILD_PHYSICAL_CHANNEL_BIT(child)) != 0U)

#define ADCK_CHILD_CONVERSION_TYPE(child) \
	DT_ENUM_IDX_OR(child, renesas_conversion_type, 0)

#define ADCK_CHILD_CONVERSION_TYPE_NORMAL 0U
#define ADCK_CHILD_CONVERSION_TYPE_HOLD   1U

#define ADCK_CHILD_IS_HOLD(child) \
	(ADCK_CHILD_CONVERSION_TYPE(child) == ADCK_CHILD_CONVERSION_TYPE_HOLD)

#define ADCK_VC_CONV_TYPE_0 ADCK_VC_CONV_TYPE_NORMAL
#define ADCK_VC_CONV_TYPE_1 ADCK_VC_CONV_TYPE_HOLD
#define ADCK_CHILD_VC_CONV_TYPE(child) \
	UTIL_CAT(ADCK_VC_CONV_TYPE_, ADCK_CHILD_CONVERSION_TYPE(child))

#define ADCK_CHILD_TH_CHANNEL_MASK_OR(child) \
	| (ADCK_CHILD_IS_HOLD(child) ? BIT(ADCK_CHILD_PHYSICAL_CHANNEL(child)) : 0U)

#define ADCK_TH_CHILD_CHANNEL_MASK(idx) \
	(0U DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADCK_CHILD_TH_CHANNEL_MASK_OR))

#define ADCK_TH_CHANNEL_MASK(idx) \
	(DT_INST_PROP_OR(idx, renesas_th_channel_mask, 0) | ADCK_TH_CHILD_CHANNEL_MASK(idx))

#define ADCK_TH_ENABLE_FROM_MASK(idx, th) \
	(((ADCK_TH_CHANNEL_MASK(idx) & BIT(th)) != 0U) ? \
	 ADC_TH_UNIT_CFG_ENABLE : ADC_TH_UNIT_CFG_DISABLE)

#define ADCK_CHILD_START_VC(child) \
	DT_PROP(DT_PARENT(child), renesas_start_virtual_channel)

#define ADCK_CHILD_END_VC(child) \
	DT_PROP(DT_PARENT(child), renesas_end_virtual_channel)

#define ADCK_CHILD_VALIDATE(child) \
	BUILD_ASSERT(DT_REG_ADDR(child) < \
		     ADC_RENESAS_ADCK_MAX_ZEPHYR_CHANNELS, \
		     "ADCK Zephyr channel ID must be less than 32"); \
	BUILD_ASSERT(ADCK_CHILD_VC(child) < \
		     ADC_RENESAS_ADCK_MAX_VIRTUAL_CHANNELS, \
		     "ADCK virtual channel is out of range"); \
	BUILD_ASSERT(ADCK_CHILD_VC(child) >= ADCK_CHILD_START_VC(child), \
		     "ADCK virtual channel is below scan group start"); \
	BUILD_ASSERT(ADCK_CHILD_VC(child) <= ADCK_CHILD_END_VC(child), \
		     "ADCK virtual channel is above scan group end"); \
	BUILD_ASSERT(ADCK_CHILD_PHYSICAL_CHANNEL(child) < \
		     ADC_RENESAS_ADCK_MAX_PHYSICAL_CHANNELS, \
		     "ADCK physical channel selector must be less than 64"); \
	BUILD_ASSERT(!ADCK_CHILD_IS_HOLD(child) || \
		     (ADCK_CHILD_PHYSICAL_CHANNEL(child) <= 2U), \
		     "ADCK T&H channel must be 0, 1, or 2");

#define ADCK_CHILD_CHANNEL_TO_VC_INIT(child) \
	[DT_REG_ADDR(child)] = ADCK_CHILD_VC(child),

#define ADCK_CHILD_VC_CFG_INIT(child) \
	[ADCK_CHILD_VC(child)] = { \
		.vcr_b.general_ctrl = ADCK_CHILD_PHYSICAL_CHANNEL(child), \
		.vcr_b.channel_end_int = ADCK_VC_END_INT_DISABLE, \
		.vcr_b.mpx_value = 0U, \
		.vcr_b.conv_type = ADCK_CHILD_VC_CONV_TYPE(child), \
		.vcr_b.wait_time_sel = ADCK_WAIT_TIME_TABLE_DISABLE, \
		.vcr_b.ul_limit_sel = ADCK_UL_LIMIT_TABLE_DISABLE, \
	},

#define ADCK_IRQ_SG0(idx) DT_INST_IRQ_BY_NAME(idx, sg0, irq)
#define ADCK_IRQ_SG1(idx) DT_INST_IRQ_BY_NAME(idx, sg1, irq)
#define ADCK_IRQ_SG2(idx) DT_INST_IRQ_BY_NAME(idx, sg2, irq)
#define ADCK_IRQ_SG3(idx) DT_INST_IRQ_BY_NAME(idx, sg3, irq)
#define ADCK_IRQ_SG4(idx) DT_INST_IRQ_BY_NAME(idx, sg4, irq)

#define ADCK_IRQ_PRIO_SG0(idx) DT_INST_IRQ_BY_NAME(idx, sg0, priority)
#define ADCK_IRQ_PRIO_SG1(idx) DT_INST_IRQ_BY_NAME(idx, sg1, priority)
#define ADCK_IRQ_PRIO_SG2(idx) DT_INST_IRQ_BY_NAME(idx, sg2, priority)
#define ADCK_IRQ_PRIO_SG3(idx) DT_INST_IRQ_BY_NAME(idx, sg3, priority)
#define ADCK_IRQ_PRIO_SG4(idx) DT_INST_IRQ_BY_NAME(idx, sg4, priority)

#define ADCK_IRQ_FLAGS(idx) 0

#define ADCK_IRQ_NAME_0 sg0
#define ADCK_IRQ_NAME_1 sg1
#define ADCK_IRQ_NAME_2 sg2
#define ADCK_IRQ_NAME_3 sg3
#define ADCK_IRQ_NAME_4 sg4
#define ADCK_SELECTED_IRQ_NAME(idx) \
	UTIL_CAT(ADCK_IRQ_NAME_, DT_INST_PROP(idx, renesas_scan_group))
#define ADCK_SELECTED_IRQ(idx) \
	DT_INST_IRQ_BY_NAME(idx, ADCK_SELECTED_IRQ_NAME(idx), irq)
#define ADCK_SELECTED_IRQ_PRIO(idx) \
	DT_INST_IRQ_BY_NAME(idx, ADCK_SELECTED_IRQ_NAME(idx), priority)

#define ADCK_SCAN_GROUP_IS(idx, group) \
	IS_EQ(DT_INST_PROP(idx, renesas_scan_group), group)

#define ADCK_SCAN_END_IRQ(idx, group) \
	COND_CODE_1(ADCK_SCAN_GROUP_IS(idx, group), \
		    (UTIL_CAT(ADCK_IRQ_SG, group)(idx)), \
		    (FSP_INVALID_VECTOR))

#define ADCK_SCAN_END_IPL(idx, group) \
	COND_CODE_1(ADCK_SCAN_GROUP_IS(idx, group), \
		    (UTIL_CAT(ADCK_IRQ_PRIO_SG, group)(idx)), \
		    (BSP_IRQ_DISABLED))

#define ADCK_DEVICE_INIT(idx) \
	BUILD_ASSERT(ADCK_START_VC(idx) <= ADCK_END_VC(idx), \
		     "ADCK start virtual channel must not exceed end virtual channel"); \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADCK_CHILD_VALIDATE) \
	static const uint8_t adc_adck_channel_to_vc_##idx \
		[ADC_RENESAS_ADCK_MAX_ZEPHYR_CHANNELS] = { \
		ADCK_CHANNEL_TO_VC_INVALID_INIT, \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADCK_CHILD_CHANNEL_TO_VC_INIT) \
	}; \
	static const adck_vc_cfg_t adc_adck_vc_cfg_##idx \
		[ADC_RENESAS_ADCK_MAX_VIRTUAL_CHANNELS] = { \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADCK_CHILD_VC_CFG_INIT) \
	}; \
	static const adck_sg_generic_cfg_t adc_adck_sg_generic_cfg_##idx = { \
		.adck_trigger_src = ADCK_ADCK0_TRIGGER_SRC_DISABLE, \
		.trigger_method = ADCK_SG_TRIGGER_METHOD_SW, \
		.sg_ctrl = { \
			.sgcr_b.end_int = ADCK_SG_END_INT_ENABLE, \
			.sgcr_b.scan_mode = ADCK_SG_SCAN_MODE_MULTICYLE, \
		}, \
		.scan_cycle = 0U, \
		.start_vc_ptr = ADCK_START_VC(idx), \
		.end_vc_ptr = ADCK_END_VC(idx), \
		.p_vc_cfg = &adc_adck_vc_cfg_##idx[ADCK_START_VC(idx)], \
	}; \
	static const adck_sg_cfg_t adc_adck_sg_cfg_##idx = { \
		.p_sg_cfg = { \
			[DT_INST_PROP(idx, renesas_scan_group)] = &adc_adck_sg_generic_cfg_##idx, \
		}, \
		.p_sg_diag_cfg = NULL, \
		.pwm_diag_cfg = ADCK_PWM_DIAG_CFG_DISABLE, \
		.tsn0_measure_cfg = ADCK_TSN0_MEASURE_CFG_DISABLE, \
	}; \
	static const adck_extended_cfg_t adc_adck_extend_cfg_##idx = { \
		.suspend_method = ADCK_SUSPEND_METHOD_SYNCHRONOUS, \
		.data_format_addition_count = { \
			.adcr2_b.data_format = ADCK_DATA_FORMAT(idx), \
			.adcr2_b.addition_count = ADCK_ADDITION_COUNT_TWICE, \
		}, \
		.safety_cfg = { \
			.sftcr_b.id = ADCK_ID_ERROR_INT_DISABLE, \
			.sftcr_b.parity = ADCK_PARITY_ERROR_INT_DISABLE, \
			.sftcr_b.overwrite = ADCK_OVERWRITE_ERROR_INT_DISABLE, \
			.sftcr_b.read_clear = ADCK_CLEAR_AFTER_READ_DISABLE, \
			.sftcr_b.trigger_overlap = ADCK_TRIGGER_OVERLAP_DISABLE, \
		}, \
		.ul_limit_cfg = { \
			.vc_ul_limit_int_0 = 0U, \
			.vc_ul_limit_int_1 = 0U, \
			.pwm_diag_ul_limit_int = ADCK_PWM_DIAG_UL_LIMIT_INT_DISABLE, \
			.sg_diag_ul_limit_int = ADCK_SG_DIAG_UL_LIMIT_INT_DISABLE, \
		}, \
		.sampling_cfg = { \
			.smpcr_b.period = ADCK_SAMPLING_PERIOD_18_STATES, \
			.smpcr_b.period_sel = ADCK_SAMPLING_PERIOD_SEL_DISABLE, \
			.smpcr_b.buffer_amp = ADCK_BUFFER_AMP_ENABLE, \
			.smpcr_b.extended_period = ADCK_EXTENDED_SAMPLING_PERIOD_60_STATES, \
		}, \
		.pin_lv_self_diag_cfg = { \
			.tdcr_b.voltage_level = ADCK_PIN_LV_SELF_DIAG_VOL_EVEN_AVSS_ODD_AVCC, \
			.tdcr_b.th_path_self_diag = ADCK_TH_PATH_SELF_DIAG_DISABLE, \
		}, \
		.wiring_break_detect_cfg = { \
			.odcr_b.pulse_width = ADCK_WIRING_BREAK_DETECT_PULSE_WIDTH_1_STATE, \
			.odcr_b.addition_mode = ADCK_WIRING_BREAK_ADDITION_MODE_DISABLE, \
		}, \
		.mpx_cfg = { \
			.mask_control_format = 0U, \
			.spi_command_information = 0U, \
			.mpx_int = ADCK_MPX_INT_DISABLE, \
		}, \
		.trigger_overlap_detect = ADCK_TRIGGER_OVERLAP_DETECT_DISABLE, \
		.th_cfg = { \
			.group_select_cfg = { \
				.thgsr_b.th0_group_cfg = ADCK_TH_GROUP_A, \
				.thgsr_b.th1_group_cfg = ADCK_TH_GROUP_A, \
				.thgsr_b.th2_group_cfg = ADCK_TH_GROUP_A, \
				.thgsr_b.th3_group_cfg = ADCK_TH_GROUP_A, \
			}, \
			.th_enable_cfg = { \
				.ther_b.th0_enable = ADCK_TH_ENABLE_FROM_MASK(idx, 0), \
				.ther_b.th1_enable = ADCK_TH_ENABLE_FROM_MASK(idx, 1), \
				.ther_b.th2_enable = ADCK_TH_ENABLE_FROM_MASK(idx, 2), \
				.ther_b.th3_enable = ADCK_TH_ENABLE_FROM_MASK(idx, 3), \
			}, \
			.trigger_method[ADCK_TH_GROUP_A] = ADCK_TH_GROUP_TRIGGER_METHOD_SW, \
			.trigger_method[ADCK_TH_GROUP_B] = ADCK_TH_GROUP_TRIGGER_METHOD_SW, \
		}, \
		.ad_timer_cfg = { \
			.trigger_method[0] = ADCK_AD_TIMER_TRIGGER_METHOD_SW, \
			.trigger_method[1] = ADCK_AD_TIMER_TRIGGER_METHOD_SW, \
			.initial_phase[0] = 0U, \
			.initial_phase[1] = 0U, \
			.cycle[0] = 0x1FFFFFU, \
			.cycle[1] = 0x1FFFFFU, \
		}, \
		.int_cfg = { \
			.error_irq = FSP_INVALID_VECTOR, \
			.error_ipl = BSP_IRQ_DISABLED, \
			.scan_group_0_end_irq = ADCK_SCAN_END_IRQ(idx, 0), \
			.scan_group_0_end_ipl = ADCK_SCAN_END_IPL(idx, 0), \
			.scan_group_1_end_irq = ADCK_SCAN_END_IRQ(idx, 1), \
			.scan_group_1_end_ipl = ADCK_SCAN_END_IPL(idx, 1), \
			.scan_group_2_end_irq = ADCK_SCAN_END_IRQ(idx, 2), \
			.scan_group_2_end_ipl = ADCK_SCAN_END_IPL(idx, 2), \
			.scan_group_3_end_irq = ADCK_SCAN_END_IRQ(idx, 3), \
			.scan_group_3_end_ipl = ADCK_SCAN_END_IPL(idx, 3), \
			.scan_group_4_end_irq = ADCK_SCAN_END_IRQ(idx, 4), \
			.scan_group_4_end_ipl = ADCK_SCAN_END_IPL(idx, 4), \
			.sg_diag_end_irq = FSP_INVALID_VECTOR, \
			.sg_diag_end_ipl = BSP_IRQ_DISABLED, \
			.mpx_irq = FSP_INVALID_VECTOR, \
			.mpx_ipl = BSP_IRQ_DISABLED, \
		}, \
	}; \
	static const adc_cfg_t adc_adck_fsp_cfg_##idx = { \
		.unit = DT_INST_PROP(idx, renesas_unit), \
		.mode = ADC_MODE_SINGLE_SCAN, \
		.resolution = ADCK_FSP_RESOLUTION(idx), \
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
		.p_extend = &adc_adck_extend_cfg_##idx, \
	}; \
	static void adc_adck_irq_config_##idx(const struct device *dev) \
	{ \
		ARG_UNUSED(dev); \
		IRQ_CONNECT(ADCK_SELECTED_IRQ(idx), ADCK_SELECTED_IRQ_PRIO(idx), \
			    adc_adck_scanend_dispatch_isr, DEVICE_DT_INST_GET(idx), \
			    ADCK_IRQ_FLAGS(idx)); \
		irq_enable(ADCK_SELECTED_IRQ(idx)); \
	} \
	static const struct adc_adck_config adc_adck_config_##idx = { \
		.channel_available_mask = DT_INST_PROP(idx, channel_available_mask), \
		.scan_group = DT_INST_PROP(idx, renesas_scan_group), \
		.resolution = ADCK_RESOLUTION(idx), \
		.scan_group_mask = ADCK_GROUP_MASK_FROM_GROUP(idx), \
		.fsp_cfg = &adc_adck_fsp_cfg_##idx, \
		.fsp_sg_cfg = &adc_adck_sg_cfg_##idx, \
		.channel_to_vc = adc_adck_channel_to_vc_##idx, \
		.irq_config_func = adc_adck_irq_config_##idx, \
	}; \
	static struct adc_adck_data adc_adck_data_##idx = { \
		ADC_CONTEXT_INIT_TIMER(adc_adck_data_##idx, ctx), \
		ADC_CONTEXT_INIT_LOCK(adc_adck_data_##idx, ctx), \
		ADC_CONTEXT_INIT_SYNC(adc_adck_data_##idx, ctx), \
		.dev = DEVICE_DT_INST_GET(idx), \
	}; \
	static DEVICE_API(adc, adc_adck_api_##idx) = { \
		.channel_setup = adc_adck_channel_setup, \
		.read = adc_adck_read, \
		.ref_internal = DT_INST_PROP_OR(idx, vref_mv, 0), \
		IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = adc_adck_read_async,)) \
	}; \
	DEVICE_DT_INST_DEFINE(idx, adc_adck_init, NULL, &adc_adck_data_##idx, \
		&adc_adck_config_##idx, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, \
		&adc_adck_api_##idx);

DT_INST_FOREACH_STATUS_OKAY(ADCK_DEVICE_INIT)
