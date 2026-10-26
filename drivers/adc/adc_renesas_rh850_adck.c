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
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/renesas_rh850_clock_common.h>

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM_DMA)
#include <zephyr/drivers/dma.h>
#endif

#include "r_adck.h"

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE
#include "adc_context.h"

LOG_MODULE_REGISTER(adc_renesas_rh850_adck, CONFIG_ADC_LOG_LEVEL);

#define ADC_RENESAS_ADCK_INVALID_VC            UINT8_MAX

/* Channel masks in this driver use uint32_t. */
#define ADC_RENESAS_ADCK_MAX_ZEPHYR_CHANNELS   32U

/* RH850 ADCK physical input selectors span 0..63. */
#define ADC_RENESAS_ADCK_MAX_PHYSICAL_CHANNELS 64U
#define ADC_RENESAS_ADCK_PHYSICAL_MASK_WORDS (ADC_RENESAS_ADCK_MAX_PHYSICAL_CHANNELS / 32U)

/* One stream frame carries at most one sample for each Zephyr channel. */
#define ADC_RENESAS_ADCK_STREAM_MAX_CHANNELS ADC_RENESAS_ADCK_MAX_ZEPHYR_CHANNELS

/* FSP ADCK has 64 virtual channel data registers. */
#define ADC_RENESAS_ADCK_MAX_VIRTUAL_CHANNELS ADCK_NUMBER_OF_VIRTUAL_CHANNEL

#define ADC_ADCK_STREAM_FLAG_VALID      BIT(0)
#define ADC_ADCK_STREAM_FLAG_DMA        BIT(1)
#define ADC_ADCK_STREAM_FLAG_WIRING_BREAK_ADDITION BIT(2)

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

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM_DMA)

#define ADCK_DMA_INIT(idx) \
	.dma_dev = ADCK_DMA_DEV(idx), \
	.dma_channel = ADCK_DMA_CHANNEL(idx), \
	.dma_slot = ADCK_DMA_CONFIG(idx), \
	.dma_group = ADCK_DMA_GROUP(idx),

#else

#define ADCK_DMA_INIT(idx)

#endif

extern void adck_sg0_scan_end_isr(void);
extern void adck_sg1_scan_end_isr(void);
extern void adck_sg2_scan_end_isr(void);
extern void adck_sg3_scan_end_isr(void);
extern void adck_sg4_scan_end_isr(void);

enum adc_adck_request_mode {
	ADC_ADCK_REQUEST_IDLE = 0,
	ADC_ADCK_REQUEST_FINITE,
	ADC_ADCK_REQUEST_STREAM,
};

enum adc_adck_data_format {
	ADC_ADCK_DATA_FORMAT_12_BIT_SIGNED_FIXED = 0U,
	ADC_ADCK_DATA_FORMAT_12_BIT_SIGNED_INTEGER = 1U,
	ADC_ADCK_DATA_FORMAT_12_BIT_UNSIGNED_FIXED = 2U,
	ADC_ADCK_DATA_FORMAT_10_BIT_UNSIGNED_FIXED = 3U,
	ADC_ADCK_DATA_FORMAT_12_BIT_UNSIGNED_RIGHT = 4U,
	ADC_ADCK_DATA_FORMAT_12_BIT_UNSIGNED_LEFT = 5U,
};

struct adc_adck_clock {
	const struct device *dev;
	clock_control_subsys_t subsys;
};

struct adc_adck_stream_header {
	uint64_t timestamp_ns;
	uint32_t sequence_number;

	uint16_t payload_size;
	uint8_t num_channels;
	uint8_t data_format;

	uint8_t flags;
	uint8_t addition_count;
	uint16_t vref_mv;
};

struct adc_adck_stream_channel {
	uint8_t channel_id;
	uint8_t virtual_channel;
	uint8_t resolution;
	uint8_t conversion_type;
};

struct adc_adck_config {
	uint32_t channel_available_mask;
	uint8_t scan_group;
	uint8_t resolution;
	uint8_t data_format;
	uint16_t vref_mv;
	bool software_trigger;
	bool uses_th;
	adc_group_mask_t scan_group_mask;

	const adc_cfg_t *fsp_cfg;
	const adck_sg_cfg_t *fsp_sg_cfg;
	const uint8_t *channel_to_vc;
	const struct adc_adck_clock *clocks;
	size_t clock_count;
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

	enum adc_adck_request_mode request_mode;

	bool active;
	int status;

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM)

	struct rtio_iodev_sqe *stream_sqe;
	const struct adc_read_config *stream_config;
	uint8_t *stream_buffer;
	uint32_t stream_buffer_len;
	uint32_t stream_channels;
	uint32_t stream_frame_size;
	uint32_t stream_sequence_number;
#endif

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM_DMA)

	const struct device *dma_dev;
	uint32_t dma_channel;
	uint32_t dma_slot;
	uint32_t dma_group;
	struct dma_config dma_cfg;
	struct dma_block_config dma_block;
	adc_info_t adc_info;
	void *dma_src;
	uint32_t dma_length;

#endif
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

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM)

static void adc_adck_stream_reset(struct adc_adck_data *data)
{
	if (data == NULL) {
		return;
	}

	data->stream_sqe = NULL;
	data->stream_config = NULL;

	data->stream_buffer = NULL;
	data->stream_buffer_len = 0U;

	data->stream_channels = 0U;
	data->stream_frame_size = 0U;

	data->request_mode = ADC_ADCK_REQUEST_IDLE;
	data->active = false;
	data->status = 0;
}

static void adc_adck_stream_fail(const struct device *dev, int error)
{
	const struct adc_adck_config *config;
	struct adc_adck_data *data;
	struct rtio_iodev_sqe *sqe;

	if (dev == NULL) {
		return;
	}

	config = dev->config;
	data = dev->data;

	if ((config == NULL) || (data == NULL)) {
		return;
	}

	/*
	 * Preserve the current SQE before resetting the stream state.
	 * adc_adck_stream_reset() clears data->stream_sqe.
	 */
	sqe = data->stream_sqe;

	if (data->request_mode == ADC_ADCK_REQUEST_STREAM) {
		(void)R_ADCK_ScanGroupStop(&data->fsp_ctrl, config->scan_group_mask);
	}

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM_DMA)
	/*
	 * Stop DMA if the DMA device exists and is ready.
	 * The DMA channel might not have started yet, so its stop result
	 * is intentionally ignored.
	 */
	if ((data->dma_dev != NULL) &&
		device_is_ready(data->dma_dev)) {
		(void)dma_stop(
			data->dma_dev,
			data->dma_channel);
	}
#endif

	/*
	 * Clear all internal stream state before completing the SQE.
	 * This allows a new stream request to be submitted immediately
	 * after the error completion is delivered.
	 */
	adc_adck_stream_reset(data);

	if (sqe != NULL) {
		rtio_iodev_sqe_err(sqe, error);
	}
}

static struct adc_adck_stream_header *adc_adck_stream_header(uint8_t *buffer)
{
	return (struct adc_adck_stream_header *)buffer;
}

static struct adc_adck_stream_channel *adc_adck_stream_channels(uint8_t *buffer)
{
	return (struct adc_adck_stream_channel *)
		(buffer + sizeof(struct adc_adck_stream_header));
}

static uint8_t *adc_adck_stream_sample_payload(uint8_t *buffer,
							   uint8_t num_channels)
{
	return buffer +
		sizeof(struct adc_adck_stream_header) +
		   num_channels *
			sizeof(struct adc_adck_stream_channel);
}

static size_t adc_adck_stream_frame_size(uint32_t channels)
{
	size_t channel_count = POPCOUNT(channels);

	return sizeof(struct adc_adck_stream_header) +
		   channel_count *
			   sizeof(struct adc_adck_stream_channel) +
		   channel_count * sizeof(uint16_t);
}

static int adc_adck_stream_prepare_frame(const struct device *dev)
{
	const struct adc_adck_config *config;
	struct adc_adck_data *data;
	struct adc_adck_stream_header *header;
	struct adc_adck_stream_channel *channel_info;
	uint32_t pending_channels;
	size_t expected_frame_size;
	size_t payload_size;
	uint8_t channel_count;
	uint8_t sample_index = 0U;
	const adck_extended_cfg_t *extend;
	const adck_sg_generic_cfg_t *sg_cfg;

	if (dev == NULL) {
		return -EINVAL;
	}

	config = dev->config;
	data = dev->data;

	extend = config->fsp_cfg->p_extend;
	sg_cfg = config->fsp_sg_cfg->p_sg_cfg[config->scan_group];

	if ((extend == NULL) || (sg_cfg == NULL) || (sg_cfg->p_vc_cfg == NULL)
	|| (config == NULL) || (data == NULL)) {
		return -EINVAL;
	}

	/*
	 * A stream frame can only be prepared while processing a valid
	 * streaming request.
	 */
	if ((data->request_mode != ADC_ADCK_REQUEST_STREAM) ||
		(data->stream_sqe == NULL)) {
		return -EINVAL;
	}

	if (data->stream_channels == 0U) {
		return -EINVAL;
	}

	/*
	 * Ensure that the requested channels are available and have been
	 * configured through adc_channel_setup().
	 */
	if ((data->stream_channels &
		 ~config->channel_available_mask) != 0U) {
		return -ENOTSUP;
	}

	if ((data->stream_channels &
		 ~data->configured_channels) != 0U) {
		return -EINVAL;
	}

	channel_count = (uint8_t)POPCOUNT(data->stream_channels);

	if ((channel_count == 0U) ||
		(channel_count > ADC_RENESAS_ADCK_STREAM_MAX_CHANNELS)) {
		return -EINVAL;
	}

	payload_size =
		(size_t)channel_count * sizeof(uint16_t);

	if (payload_size > UINT16_MAX) {
		return -EOVERFLOW;
	}

	expected_frame_size =
		adc_adck_stream_frame_size(data->stream_channels);

	/*
	 * stream_frame_size should have been calculated when the SQE was
	 * submitted. Verify it again before writing to the RTIO buffer.
	 */
	if (data->stream_frame_size != expected_frame_size) {
		return -EINVAL;
	}

	if ((data->stream_buffer == NULL) ||
		(data->stream_buffer_len < expected_frame_size)) {
		return -ENOMEM;
	}

	/*
	 * Initialize the entire encoded frame. This is important because
	 * the RTIO mempool may return a previously used buffer.
	 */
	memset(data->stream_buffer, 0, expected_frame_size);

	header = adc_adck_stream_header(data->stream_buffer);
	channel_info =
		adc_adck_stream_channels(data->stream_buffer);

	/*
	 * Timestamp is finalized when the scan or DMA transfer completes.
	 * Leaving it at zero here avoids reporting the time at which the
	 * frame was merely prepared.
	 */
	header->timestamp_ns = 0U;
	header->sequence_number = data->stream_sequence_number;
	header->payload_size = (uint16_t)payload_size;
	header->num_channels = channel_count;
	header->data_format = (uint8_t)config->data_format;
	if (extend->wiring_break_detect_cfg.odcr_b.addition_mode ==
	ADCK_WIRING_BREAK_ADDITION_MODE_ENABLE) {
		header->flags |= ADC_ADCK_STREAM_FLAG_WIRING_BREAK_ADDITION;
	}
	header->addition_count = (uint8_t)extend->data_format_addition_count.adcr2_b.addition_count;
	header->vref_mv = config->vref_mv;

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM_DMA)
	header->flags |= ADC_ADCK_STREAM_FLAG_DMA;
#endif

	/*
	 * Channel descriptors and raw samples use the same ordering.
	 *
	 * For example:
	 *
	 * channel_info[0] <-> raw_sample[0]
	 * channel_info[1] <-> raw_sample[1]
	 */
	pending_channels = data->stream_channels;

	for (uint8_t channel = 0U;
		 pending_channels != 0U;
		 channel++, pending_channels >>= 1U) {
		uint8_t virtual_channel;

		if ((pending_channels & 1U) == 0U) {
			continue;
		}

		if (channel >= ADC_RENESAS_ADCK_MAX_ZEPHYR_CHANNELS) {
			return -EINVAL;
		}

		virtual_channel = config->channel_to_vc[channel];

		if (virtual_channel >=
			ADC_RENESAS_ADCK_MAX_VIRTUAL_CHANNELS) {
			return -EINVAL;
		}

		if (sample_index >= channel_count) {
			return -EOVERFLOW;
		}

		channel_info[sample_index].channel_id = channel;
		channel_info[sample_index].virtual_channel =
			virtual_channel;
		channel_info[sample_index].resolution =
			config->resolution;
		channel_info[sample_index].conversion_type =
		(uint8_t)sg_cfg->p_vc_cfg[virtual_channel - sg_cfg->start_vc_ptr].vcr_b.conv_type;
		sample_index++;
	}

	if (sample_index != channel_count) {
		return -EINVAL;
	}

	return 0;
}


static int adc_adck_validate_stream_config(
	const struct device *dev,
	const struct adc_read_config *read_cfg,
	uint32_t *channel_mask)
{
	const struct adc_adck_config *config = dev->config;
	struct adc_adck_data *data = dev->data;
	uint32_t channels = 0U;

	if ((read_cfg == NULL) || !read_cfg->is_streaming) {
		return -EINVAL;
	}

	if ((read_cfg->adc != dev) ||
		(read_cfg->adc_spec == NULL) ||
		(read_cfg->adc_spec_cnt == 0U) ||
		(read_cfg->adc_spec_cnt > ADC_RENESAS_ADCK_STREAM_MAX_CHANNELS)) {
		return -EINVAL;
	}

	for (size_t i = 0; i < read_cfg->adc_spec_cnt; i++) {
		const struct adc_dt_spec *spec = &read_cfg->adc_spec[i];

		if (spec->dev != dev) {
			return -EINVAL;
		}

		if (spec->channel_id >= ADC_RENESAS_ADCK_MAX_ZEPHYR_CHANNELS) {
			return -EINVAL;
		}

		if ((config->channel_available_mask &
			 BIT(spec->channel_id)) == 0U) {
			return -ENOTSUP;
		}

		if ((data->configured_channels &
			 BIT(spec->channel_id)) == 0U) {
			return -EINVAL;
		}

		if (config->channel_to_vc[spec->channel_id] >=
			ADC_RENESAS_ADCK_MAX_VIRTUAL_CHANNELS) {
			return -EINVAL;
		}

		if ((channels & BIT(spec->channel_id)) != 0U) {
			return -EINVAL;
		}

		channels |= BIT(spec->channel_id);
	}

	/*
	 * Initial implementation supports one completion per completed
	 * ADCK scan. DATA_READY means that one complete frame is available.
	 */
	for (size_t i = 0; i < read_cfg->trigger_cnt; i++) {
		const struct adc_stream_trigger *trigger =
			&read_cfg->triggers[i];

		switch (trigger->trigger) {
		case ADC_TRIG_DATA_READY:
		case ADC_TRIG_FIFO_WATERMARK:
		case ADC_TRIG_FIFO_FULL:
			break;
		default:
			return -ENOTSUP;
		}

		if (trigger->opt != ADC_STREAM_DATA_INCLUDE) {
			return -ENOTSUP;
		}

	}

	*channel_mask = channels;
	return 0;
}

static int adc_adck_stream_acquire_buffer(struct adc_adck_data *data)
{
	uint8_t *buffer;
	uint32_t buffer_len;
	int err;

	err = rtio_sqe_rx_buf(data->stream_sqe,
				  data->stream_frame_size,
				  data->stream_frame_size,
				  &buffer,
				  &buffer_len);
	if (err != 0) {
		return err;
	}

	if ((buffer == NULL) ||
		(buffer_len < data->stream_frame_size)) {
		return -ENOMEM;
	}

	data->stream_buffer = buffer;
	data->stream_buffer_len = buffer_len;

	return 0;
}

static int adc_adck_stream_read_frame(const struct device *dev)
{
	const struct adc_adck_config *config = dev->config;
	struct adc_adck_data *data = dev->data;
	struct adc_adck_stream_header *header;
	uint8_t *sample_payload;
	uint32_t pending_channels;
	uint8_t sample_index = 0U;

	if ((data->stream_buffer == NULL) ||
		(data->stream_buffer_len < data->stream_frame_size)) {
		return -ENOMEM;
	}

	header = adc_adck_stream_header(data->stream_buffer);

	if ((header->num_channels == 0U) ||
	(header->num_channels > ADC_RENESAS_ADCK_STREAM_MAX_CHANNELS)) {
		return -EINVAL;
	}

	if (header->payload_size !=
		((uint16_t)header->num_channels *
		 sizeof(uint16_t))) {
		return -EINVAL;
	}

	sample_payload =
		adc_adck_stream_sample_payload(
			data->stream_buffer,
			header->num_channels);

	pending_channels = data->stream_channels;

	for (uint8_t channel = 0U;
		 pending_channels != 0U;
		 channel++, pending_channels >>= 1U) {
		uint8_t virtual_channel;
		uint16_t raw;
		fsp_err_t fsp_err;

		if ((pending_channels & 1U) == 0U) {
			continue;
		}

		if (sample_index >= header->num_channels) {
			return -EOVERFLOW;
		}

		virtual_channel =
			config->channel_to_vc[channel];

		if (virtual_channel >=
			ADC_RENESAS_ADCK_MAX_VIRTUAL_CHANNELS) {
			return -EINVAL;
		}

		fsp_err = R_ADCK_Read(
			&data->fsp_ctrl,
			(adc_channel_t)virtual_channel,
			&raw);

		if (fsp_err != FSP_SUCCESS) {
			return adc_adck_fsp_err_to_errno(
				fsp_err);
		}

		/*
		 * Preserve the native 16-bit ADCK representation
		 * selected by ADCR2.DFMT.
		 *
		 * memcpy() uses native-endian representation, which
		 * is also compatible with direct DMA transfers.
		 */
		memcpy(sample_payload +
				   sample_index * sizeof(raw),
			   &raw,
			   sizeof(raw));

		sample_index++;
	}

	if (sample_index != header->num_channels) {
		return -EINVAL;
	}

	header->timestamp_ns =
		k_ticks_to_ns_floor64(k_uptime_ticks());

	header->flags |= ADC_ADCK_STREAM_FLAG_VALID;

	return 0;
}

#endif

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

static int adc_adck_validate_sequence(const struct device *dev, const struct adc_sequence *sequence)
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

	/*
	 * ADCK does not support calibration. Zephyr requires ADC drivers
	 * without calibration support to ignore sequence->calibrate.
	 */

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

		fsp_err = R_ADCK_Read(&data->fsp_ctrl, (adc_channel_t)vc, data);
		if (fsp_err != FSP_SUCCESS) {
			return adc_adck_fsp_err_to_errno(fsp_err);
		}

		buffer++;
	}

	return 0;
}
#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM_DMA)

static void adc_adck_dma_callback(const struct device *dma_dev, void *user_data,
	uint32_t channel, int status)
{
	struct adc_adck_data *data = user_data;
	struct adc_adck_stream_header *header;
	struct rtio_iodev_sqe *completed_sqe;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (data == NULL) {
		return;
	}

	if (status < 0) {
		adc_adck_stream_fail(data->dev, status);
		return;
	}

	if ((data->request_mode != ADC_ADCK_REQUEST_STREAM) || !data->active ||
	(data->stream_sqe == NULL) || (data->stream_buffer == NULL)) {
		return;
	}

	header = adc_adck_stream_header(data->stream_buffer);
	header->timestamp_ns = k_ticks_to_ns_floor64(k_uptime_ticks());
	header->flags |= ADC_ADCK_STREAM_FLAG_VALID;
	data->stream_sequence_number++;
	completed_sqe = data->stream_sqe;

	/*
	 * Clear per-shot state before completing the shot.
	 * The stream session itself remains active.
	 */
	data->stream_sqe = NULL;
	data->stream_buffer = NULL;
	data->stream_buffer_len = 0U;

	rtio_iodev_sqe_ok(completed_sqe, 0);
}
#endif

static void adc_adck_fsp_callback(adc_callback_args_t *args)
{
	const struct device *dev = args->p_context;
	const struct adc_adck_config *config = dev->config;
	struct adc_adck_data *data = dev->data;
	int err;

	if (!data->active) {
		return;
	}

	if (args->event == ADC_EVENT_MPX_CONVERSION) {
		return;
	}

	if ((args->event == ADC_EVENT_ID_ERROR) || (args->event == ADC_EVENT_OVERWRITE_ERROR) ||
		(args->event == ADC_EVENT_TRIGGER_OVERLAP_ERROR) ||
		(args->event == ADC_EVENT_UPPER_LOWER_LIMIT_ERROR)) {
		data->status = -EIO;
		data->active = false;
#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM)
		if (data->request_mode == ADC_ADCK_REQUEST_STREAM) {
			adc_adck_stream_fail(dev, -EIO);
			return;
		}
#endif
		adc_context_complete(&data->ctx, -EIO);
		return;
	}

	if (args->event != (ADC_EVENT_SCAN_COMPLETE_GROUP_0 + config->scan_group)) {
		return;
	}

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM)
	if (data->request_mode == ADC_ADCK_REQUEST_STREAM) {
#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM_DMA)
		return;
#else
		struct rtio_iodev_sqe *completed_sqe;

		err = adc_adck_stream_read_frame(dev);
		if (err != 0) {
			adc_adck_stream_fail(dev, err);
			return;
		}

		completed_sqe = data->stream_sqe;

		data->stream_sequence_number++;
		data->stream_sqe = NULL;
		data->stream_buffer = NULL;
		data->stream_buffer_len = 0U;

		rtio_iodev_sqe_ok(completed_sqe, 0);
		return;
#endif
	}
#endif
	err = adc_adck_read_channels(dev);
	if (err != 0) {
		data->status = err;
		data->active = false;
		adc_context_complete(&data->ctx, err);
		return;
	}

	/*
	 * A sequence with extra samplings or ADC_ACTION_REPEAT starts the next
	 * scan from adc_context_on_sampling_done().  Stop the completed hardware
	 * scan first so that the following R_ADCK_ScanGroupStart() does not return
	 * FSP_ERR_IN_USE.
	 */
	if (R_ADCK_ScanGroupStop(&data->fsp_ctrl, config->scan_group_mask) != FSP_SUCCESS) {
		data->status = -EIO;
		data->active = false;
		adc_context_complete(&data->ctx, -EIO);
		return;
	}

	data->status = 0;
	adc_context_on_sampling_done(&data->ctx, dev);
}

static int adc_adck_start_read(const struct device *dev, const struct adc_sequence *sequence)
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

static int adc_adck_read_async(const struct device *dev, const struct adc_sequence *sequence,
			       struct k_poll_signal *async)
{
	struct adc_adck_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, async != NULL, async);
	err = adc_adck_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

static int adc_adck_read(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct adc_adck_config *config = dev->config;

	if (!config->software_trigger) {
		return -ENOTSUP;
	}

	return adc_adck_read_async(dev, sequence, NULL);
}

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM)

static void adc_adck_submit(const struct device *dev,
				struct rtio_iodev_sqe *iodev_sqe)
{
	fsp_err_t fsp_err;
	const struct adc_read_config *read_cfg;
	uint32_t channels;
	int err;

	if ((dev == NULL) || (iodev_sqe == NULL)) {
		return;
	}

	struct adc_adck_data *data = dev->data;

	if (data->request_mode == ADC_ADCK_REQUEST_IDLE) {
		const struct adc_adck_config *config = dev->config;

		if ((iodev_sqe->sqe.iodev == NULL) ||
			(iodev_sqe->sqe.iodev->data == NULL)) {
			rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
			return;
		}
		read_cfg = iodev_sqe->sqe.iodev->data;

		err = adc_adck_validate_stream_config(dev, read_cfg, &channels);

		if (err != 0) {
			rtio_iodev_sqe_err(iodev_sqe, err);
			return;
		}

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM_DMA)
		if (data->dma_length != POPCOUNT(channels)) {
			rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
			return;
		}
#endif

		data->stream_sqe = iodev_sqe;
		data->stream_config = read_cfg;
		data->stream_channels = channels;
		data->stream_frame_size =
			adc_adck_stream_frame_size(channels);
		data->status = 0;
		data->request_mode = ADC_ADCK_REQUEST_STREAM;
		data->active = true;

		/* Acquire next RTIO frame buffer */
		err = adc_adck_stream_acquire_buffer(data);

		if (err != 0) {
			adc_adck_stream_fail(dev, err);
			return;
		}

		err = adc_adck_stream_prepare_frame(dev);

		if (err != 0) {
			adc_adck_stream_fail(dev, err);
			return;
		}

		uint8_t *sample_payload;
		struct adc_adck_stream_header *header;

		header = adc_adck_stream_header(data->stream_buffer);
		sample_payload = adc_adck_stream_sample_payload(data->stream_buffer,
			 header->num_channels);

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM_DMA)
		data->dma_cfg.source_data_size = 2U;
		data->dma_cfg.dest_data_size = 2U;

		data->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;

		data->dma_cfg.block_count = 1U;
		data->dma_cfg.head_block = &data->dma_block;

		data->dma_cfg.dma_callback =
			adc_adck_dma_callback;

		data->dma_cfg.user_data = data;

		data->dma_block.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;

		data->dma_block.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

		data->dma_cfg.dma_slot = data->dma_slot;

		data->dma_cfg.linked_channel = data->dma_group;

		data->dma_block.source_address =
			(uint32_t)(uintptr_t)data->dma_src;

		data->dma_block.dest_address =
				(uint32_t)(uintptr_t)sample_payload;

		data->dma_block.block_size =
				data->dma_length *
				sizeof(uint16_t);

		err = dma_config(
				data->dma_dev,
				data->dma_channel,
				&data->dma_cfg);

		if (err != 0) {
			adc_adck_stream_fail(dev, err);
			return;
		}

		err = dma_start(
			data->dma_dev,
			data->dma_channel);

		if (err != 0) {
			adc_adck_stream_fail(dev, err);
			return;
		}
#endif
		/* Start ADCK only once */

		fsp_err = R_ADCK_ScanGroupStart(
				&data->fsp_ctrl,
				config->scan_group_mask);

		if (fsp_err != FSP_SUCCESS) {
			adc_adck_stream_fail(
				dev,
				adc_adck_fsp_err_to_errno(fsp_err));
			return;
		}

	} else if (data->request_mode == ADC_ADCK_REQUEST_STREAM) {
		if ((iodev_sqe->sqe.iodev == NULL) ||
			(iodev_sqe->sqe.iodev->data == NULL)) {
			rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
			return;
		}
		read_cfg = iodev_sqe->sqe.iodev->data;

		if (read_cfg != data->stream_config) {
			rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
			return;
		}

		if (data->stream_sqe != NULL) {
			rtio_iodev_sqe_err(iodev_sqe, -EBUSY);
			return;
		}

		data->stream_sqe = iodev_sqe;

		/* Acquire next RTIO frame buffer */
		err = adc_adck_stream_acquire_buffer(data);

		if (err != 0) {
			adc_adck_stream_fail(dev, err);
			return;
		}

		err = adc_adck_stream_prepare_frame(dev);

		if (err != 0) {
			adc_adck_stream_fail(dev, err);
			return;
		}

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM_DMA)

		uint8_t *sample_payload;
		struct adc_adck_stream_header *header;

		header = adc_adck_stream_header(data->stream_buffer);
		sample_payload = adc_adck_stream_sample_payload(data->stream_buffer,
			header->num_channels);

		err = dma_reload(data->dma_dev, data->dma_channel, (uintptr_t)data->dma_src,
			(uintptr_t)sample_payload, data->dma_length * sizeof(uint16_t));
		if (err != 0) {
			adc_adck_stream_fail(dev, err);
			return;
		}
#endif
	} else {
		rtio_iodev_sqe_err(iodev_sqe, -EBUSY);
		return;
	}

}

static const struct adc_adck_stream_header *
adc_adck_decoder_header(const uint8_t *buffer)
{
	return (const struct adc_adck_stream_header *)buffer;
}

static const struct adc_adck_stream_channel *
adc_adck_decoder_channels(const uint8_t *buffer)
{
	return (const struct adc_adck_stream_channel *)
		(buffer + sizeof(struct adc_adck_stream_header));
}

static const uint8_t *
adc_adck_decoder_sample_payload(const uint8_t *buffer,
				uint8_t num_channels)
{
	return buffer +
		   sizeof(struct adc_adck_stream_header) +
		   ((size_t)num_channels *
		sizeof(struct adc_adck_stream_channel));
}

static int adc_adck_decoder_find_channel(const uint8_t *buffer,
					uint32_t channel,
					uint8_t *sample_index)
{
	const struct adc_adck_stream_header *header;
	const struct adc_adck_stream_channel *channels;

	if ((buffer == NULL) || (sample_index == NULL)) {
		return -EINVAL;
	}

	header = adc_adck_decoder_header(buffer);
	channels = adc_adck_decoder_channels(buffer);

	for (uint8_t i = 0U; i < header->num_channels; i++) {
		if (channels[i].channel_id == channel) {
			*sample_index = i;
			return 0;
		}
	}

	return -ENODATA;
}

/*
 * Validate the metadata stored in an ADCK encoded stream frame.
 *
 * The decoder API does not receive the total buffer length, so this
 * function can validate the metadata fields but cannot independently
 * prove that the backing buffer is large enough.
 */
static int adc_adck_decoder_validate_frame(const uint8_t *buffer)
{
	const struct adc_adck_stream_header *header;
	uint16_t expected_payload_size;

	if (buffer == NULL) {
		return -EINVAL;
	}

	header = adc_adck_decoder_header(buffer);

	if (header->vref_mv == 0U) {
		return -EINVAL;
	}

	if ((header->flags & ADC_ADCK_STREAM_FLAG_VALID) == 0U) {
		return -ENODATA;
	}

	if ((header->num_channels == 0U) ||
		(header->num_channels >
		 ADC_RENESAS_ADCK_STREAM_MAX_CHANNELS)) {
		return -EINVAL;
	}

	expected_payload_size =
		(uint16_t)header->num_channels *
		sizeof(uint16_t);

	if (header->payload_size != expected_payload_size) {
		return -EINVAL;
	}

	if (header->data_format >
		ADC_ADCK_DATA_FORMAT_12_BIT_UNSIGNED_LEFT) {
		return -ENOTSUP;
	}

	return 0;
}

/*
 * Read one raw 16-bit ADCK result from the encoded payload.
 *
 * memcpy() is used instead of a uint16_t pointer dereference because
 * the payload might not be naturally aligned for a 16-bit access.
 *
 * The CPU-backed stream producer also stores the sample with memcpy(),
 * while the DMA path writes the native 16-bit representation directly.
 */
static int adc_adck_decoder_get_raw_sample(const uint8_t *buffer,
					   uint8_t sample_index,
					   uint16_t *raw)
{
	const struct adc_adck_stream_header *header;
	const uint8_t *sample_payload;
	int err;

	if (raw == NULL) {
		return -EINVAL;
	}

	err = adc_adck_decoder_validate_frame(buffer);
	if (err != 0) {
		return err;
	}

	header = adc_adck_decoder_header(buffer);

	if (sample_index >= header->num_channels) {
		return -EINVAL;
	}

	sample_payload =
		adc_adck_decoder_sample_payload(
			buffer,
			header->num_channels);

	memcpy(raw,
		   sample_payload +
			   ((size_t)sample_index * sizeof(*raw)),
		   sizeof(*raw));

	return 0;
}

/*
 * Convert a native ADCK raw result into a q31_t voltage representation.
 *
 * The resulting value and shift follow the same voltage-oriented
 * convention used by the STM32 streaming decoder.
 */
static int adc_adck_decoder_raw_to_voltage_q31(
	uint16_t count,
	uint8_t resolution,
	uint16_t vref_mv,
	q31_t *value,
	uint8_t *shift)
{
	uint32_t max_code;
	uint8_t q_shift;
	uint64_t num;
	uint64_t den;

	if ((value == NULL) || (shift == NULL) || (vref_mv == 0U)) {
		return -EINVAL;
	}
	if ((resolution == 0U) || (resolution > 16U)) {
		return -EINVAL;
	}

	max_code = BIT(resolution) - 1U;
	q_shift = find_msb_set(vref_mv) - 1U;
	num = (uint64_t)count * (uint64_t)vref_mv * (1ULL << (31U - q_shift));
	den = 1000ULL * (uint64_t)max_code;
	*value = (q31_t)(num / den);
	*shift = q_shift;
	return 0;
}

static int adc_adck_decoder_get_frame_count(const uint8_t *buffer,
	uint32_t channel, uint16_t *frame_count)
{
	int err;
	const struct adc_adck_stream_header *header;
	const struct adc_adck_stream_channel *channels;
	bool found = false;

	if ((frame_count == NULL) || (buffer == NULL)) {
		return -EINVAL;
	}

	if (channel >= ADC_RENESAS_ADCK_MAX_ZEPHYR_CHANNELS) {
		return -ENOTSUP;
	}

	err = adc_adck_decoder_validate_frame(buffer);
	if (err != 0) {
		return err;
	}

	header = adc_adck_decoder_header(buffer);
	channels = adc_adck_decoder_channels(buffer);

	for (uint8_t i = 0U; i < header->num_channels; i++) {
		if (channels[i].channel_id == channel) {
			found = true;
			break;
		}
	}

	if (!found) {
		return -ENODATA;
	}

	/* This ADC does not have FIFO so one frame contains one sample per channel */
	*frame_count = 1U;

	return 0;
}

static int adc_adck_decoder_get_size_info(struct adc_dt_spec adc_spec,
					  uint32_t channel,
					  size_t *base_size,
					  size_t *frame_size)
{
	if ((base_size == NULL) || (frame_size == NULL)) {
		return -EINVAL;
	}

	if (adc_spec.dev == NULL) {
		return -EINVAL;
	}

	if (channel >= ADC_RENESAS_ADCK_MAX_ZEPHYR_CHANNELS) {
		return -ENOTSUP;
	}

	/*
	 * The first decoded frame requires the adc_data base object.
	 * Each additional decoded frame requires one adc_sample_data.
	 *
	 * The current encoded ADCK buffer contains only one frame, but
	 * these values remain compatible with the generic decoder API.
	 */
	*base_size = sizeof(struct adc_data);
	*frame_size = sizeof(struct adc_sample_data);

	return 0;
}

static int adc_adck_decoder_decode(const uint8_t *buffer,
				   uint32_t channel,
				   uint32_t *fit,
				   uint16_t max_count,
				   void *data_out)
{
	const struct adc_adck_stream_header *header;
	const struct adc_adck_stream_channel *channels;
	struct adc_data *output;
	uint16_t raw;
	q31_t value;
	uint8_t shift;
	int err;
	uint8_t sample_index;
	uint16_t count;
	bool wiring_break_addition;

	if ((buffer == NULL) ||
		(fit == NULL) ||
		(data_out == NULL)) {
		return -EINVAL;
	}

	/*
	 * The caller requested no samples.
	 */
	if (max_count == 0U) {
		return 0;
	}

	/*
	 * The current ADCK frame contains one scan only.
	 *
	 * fit == 0 means the frame has not been decoded yet.
	 * fit != 0 means there are no additional frames in this buffer.
	 */
	if (*fit != 0U) {
		return 0;
	}

	err = adc_adck_decoder_validate_frame(buffer);
	if (err != 0) {
		return err;
	}

	err = adc_adck_decoder_find_channel(buffer, channel, &sample_index);
	if (err != 0) {
		return err;
	}

	err = adc_adck_decoder_get_raw_sample(
		buffer,
		sample_index,
		&raw);
	if (err != 0) {
		return err;
	}

	header = adc_adck_decoder_header(buffer);
	channels = adc_adck_decoder_channels(buffer);

	/*
	 * Validate that the resolution stored in the channel descriptor
	 * agrees with the selected ADCK format.
	 */
	switch ((enum adc_adck_data_format)header->data_format) {
	case ADC_ADCK_DATA_FORMAT_10_BIT_UNSIGNED_FIXED:
		if (channels[sample_index].resolution != 10U) {
			return -EINVAL;
		}
		break;

	case ADC_ADCK_DATA_FORMAT_12_BIT_SIGNED_FIXED:
	case ADC_ADCK_DATA_FORMAT_12_BIT_SIGNED_INTEGER:
	case ADC_ADCK_DATA_FORMAT_12_BIT_UNSIGNED_FIXED:
	case ADC_ADCK_DATA_FORMAT_12_BIT_UNSIGNED_RIGHT:
	case ADC_ADCK_DATA_FORMAT_12_BIT_UNSIGNED_LEFT:
		if (channels[sample_index].resolution != 12U) {
			return -EINVAL;
		}
		break;

	default:
		return -ENOTSUP;
	}

	wiring_break_addition =
	(header->flags & ADC_ADCK_STREAM_FLAG_WIRING_BREAK_ADDITION) != 0U;

	err = adc_adck_raw_to_count_from_metadata(
		(enum adc_adck_data_format)header->data_format,
		channels[sample_index].conversion_type,
		header->addition_count,
		wiring_break_addition,
		raw,
		&count);
	if (err != 0) {
		return err;
	}

	err = adc_adck_decoder_raw_to_voltage_q31(
		count, channels[sample_index].resolution, header->vref_mv, &value, &shift);
	if (err != 0) {
		return err;
	}

	output = (struct adc_data *)data_out;

	/*
	 * Clear the output before populating it. This prevents stale
	 * reading data from a previous decoder call.
	 */
	memset(output, 0, sizeof(*output));

	output->header.base_timestamp_ns =
		header->timestamp_ns;

	output->header.reading_count = 1U;

	output->shift = shift;

	output->readings[0].timestamp_delta = 0U;
	output->readings[0].value = value;

	/*
	 * Mark the single frame in this buffer as consumed.
	 */
	*fit = 1U;

	/*
	 * Return the number of frames decoded.
	 */
	return 1;
}

static bool adc_adck_decoder_has_trigger(const uint8_t *buffer,
					 enum adc_trigger_type trigger)
{
	if (adc_adck_decoder_validate_frame(buffer) != 0) {
		return false;
	}

	/*
	 * One encoded frame is generated after one completed ADCK scan.
	 * The initial streaming implementation therefore exposes only
	 * DATA_READY semantics.
	 */

	switch (trigger) {
	case ADC_TRIG_DATA_READY:
	case ADC_TRIG_FIFO_WATERMARK:
	case ADC_TRIG_FIFO_FULL:
		return true;
	default:
		return false;
	}
}

ADC_DECODER_API_DT_DEFINE() = {
	.get_frame_count = adc_adck_decoder_get_frame_count,
	.get_size_info = adc_adck_decoder_get_size_info,
	.decode = adc_adck_decoder_decode,
	.has_trigger = adc_adck_decoder_has_trigger,
};

static int adc_adck_get_decoder(
	const struct device *dev,
	const struct adc_decoder_api **api)
{
	ARG_UNUSED(dev);

	if (api == NULL) {
		return -EINVAL;
	}

	*api = &ADC_DECODER_NAME();

	return 0;
}

#endif

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_adck_data *data = CONTAINER_OF(ctx, struct adc_adck_data, ctx);
	const struct device *dev = data->dev;
	const struct adc_adck_config *config = dev->config;
	fsp_err_t fsp_err;

	if (data->request_mode != ADC_ADCK_REQUEST_IDLE &&
		data->request_mode != ADC_ADCK_REQUEST_FINITE) {
		data->status = -EBUSY;
		adc_context_complete(ctx, -EBUSY);
		return;
	}

	data->request_mode = ADC_ADCK_REQUEST_FINITE;
	data->channels = ctx->sequence.channels;
	data->active = true;

	if (config->uses_th) {
		fsp_err = R_ADCK_THEnable(&data->fsp_ctrl);
		if ((fsp_err != FSP_SUCCESS) && (fsp_err != FSP_ERR_IN_USE)) {
			int err = adc_adck_fsp_err_to_errno(fsp_err);

			data->status = err;
			data->active = false;
			adc_context_complete(ctx, err);
			return;
		}
	}

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
	data->request_mode = ADC_ADCK_REQUEST_IDLE;
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
	data->status = 0;
	data->request_mode = ADC_ADCK_REQUEST_IDLE;
#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM)
	data->stream_sqe = NULL;
	data->stream_config = NULL;
	data->stream_buffer = NULL;
	data->stream_buffer_len = 0U;
	data->stream_channels = 0U;
	data->stream_frame_size = 0U;
	data->stream_sequence_number = 0U;
#endif

	for (size_t i = 0U; i < config->clock_count; i++) {
		const struct device *clk_dev = config->clocks[i].dev;
		clock_control_subsys_t clk_subsys = config->clocks[i].subsys;

		if (!device_is_ready(clk_dev)) {
			LOG_ERR("ADCK clock device %zu is not ready", i);
			return -ENODEV;
		}

		err = clock_control_on(clk_dev, clk_subsys);
		if (err < 0) {
			LOG_ERR("ADCK clock %zu enable failed: %d", i, err);
			return err;
		}
	}

	fsp_err = R_ADCK_Open(&data->fsp_ctrl, config->fsp_cfg);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADCK_Open failed: %d", fsp_err);
		return adc_adck_fsp_err_to_errno(fsp_err);
	}

	fsp_err = R_ADCK_CallbackSet(&data->fsp_ctrl, adc_adck_fsp_callback, (void *)dev,
				     &data->callback_memory);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADCK_CallbackSet failed: %d", fsp_err);
		err = adc_adck_fsp_err_to_errno(fsp_err);
		goto close;
	}

	fsp_err = R_ADCK_ScanCfg(&data->fsp_ctrl, config->fsp_sg_cfg);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADCK_ScanCfg failed: %d", fsp_err);
		err = adc_adck_fsp_err_to_errno(fsp_err);
		goto close;
	}

	if (config->irq_config_func != NULL) {
		config->irq_config_func(dev);
	}

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM_DMA)

	fsp_err = R_ADCK_InfoGet(
			&data->fsp_ctrl,
			&data->adc_info);

	if (fsp_err != FSP_SUCCESS) {
		err = adc_adck_fsp_err_to_errno(fsp_err);
		return err;
	}

	data->dma_src =
		(void *)data->adc_info.p_address[
			config->scan_group];

	data->dma_length =
		data->adc_info.length[
			config->scan_group];

#endif

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;

close:
	(void)R_ADCK_Close(&data->fsp_ctrl);
	return err;
}

#define ADCK_GROUP_MASK_0 ADC_GROUP_MASK_0
#define ADCK_GROUP_MASK_1 ADC_GROUP_MASK_1
#define ADCK_GROUP_MASK_2 ADC_GROUP_MASK_2
#define ADCK_GROUP_MASK_3 ADC_GROUP_MASK_3
#define ADCK_GROUP_MASK_4 ADC_GROUP_MASK_4

#define ADCK_GROUP_MASK_FROM_GROUP(idx)                                                            \
	UTIL_CAT(ADCK_GROUP_MASK_, DT_INST_PROP(idx, renesas_scan_group))

#if defined(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM_DMA)

#define ADCK_DMA_DEV(idx)                                                                          \
	DEVICE_DT_GET(DT_DMAS_CTLR(DT_DRV_INST(idx)))

#define ADCK_DMA_CHANNEL(idx)                                                                      \
	DT_DMAS_CELL_BY_IDX(DT_DRV_INST(idx), 0, channel)

#define ADCK_DMA_CONFIG(idx)                                                                       \
	DT_DMAS_CELL_BY_IDX(DT_DRV_INST(idx), 0, config)

#define ADCK_DMA_GROUP(idx)                                                                       \
	DT_DMAS_CELL_BY_IDX(DT_DRV_INST(idx), 0, group)

#endif
#define ADCK_SCAN_MODE_multicycle  ADCK_SG_SCAN_MODE_MULTICYLE
#define ADCK_SCAN_MODE_continuous  ADCK_SG_SCAN_MODE_CONTINUOUS
#define ADCK_DATA_FORMAT_12_bit_signed_fixed_point	ADC_ADCK_DATA_FORMAT_12_BIT_SIGNED_FIXED
#define ADCK_DATA_FORMAT_12_bit_signed_integer		ADC_ADCK_DATA_FORMAT_12_BIT_SIGNED_INTEGER
#define ADCK_DATA_FORMAT_12_bit_unsigned_fixed_point	 ADC_ADCK_DATA_FORMAT_12_BIT_UNSIGNED_FIXED
#define ADCK_DATA_FORMAT_10_bit_unsigned_fixed_point	 ADC_ADCK_DATA_FORMAT_10_BIT_UNSIGNED_FIXED
#define ADCK_DATA_FORMAT_12_bit_unsigned_right_justified ADC_ADCK_DATA_FORMAT_12_BIT_UNSIGNED_RIGHT
#define ADCK_DATA_FORMAT_12_bit_unsigned_left_justified  ADC_ADCK_DATA_FORMAT_12_BIT_UNSIGNED_LEFT
#define ADCK_RESOLUTION_12_bit_signed_fixed_point		12U
#define ADCK_RESOLUTION_12_bit_signed_integer			12U
#define ADCK_RESOLUTION_12_bit_unsigned_fixed_point	  12U
#define ADCK_RESOLUTION_10_bit_unsigned_fixed_point	  10U
#define ADCK_RESOLUTION_12_bit_unsigned_right_justified  12U
#define ADCK_RESOLUTION_12_bit_unsigned_left_justified   12U
#define ADCK_FSP_RESOLUTION_12_bit_signed_fixed_point	ADC_RESOLUTION_12_BIT
#define ADCK_FSP_RESOLUTION_12_bit_signed_integer		ADC_RESOLUTION_12_BIT
#define ADCK_FSP_RESOLUTION_12_bit_unsigned_fixed_point  ADC_RESOLUTION_12_BIT
#define ADCK_FSP_RESOLUTION_10_bit_unsigned_fixed_point  ADC_RESOLUTION_10_BIT
#define ADCK_FSP_RESOLUTION_12_bit_unsigned_right_justified ADC_RESOLUTION_12_BIT
#define ADCK_FSP_RESOLUTION_12_bit_unsigned_left_justified  ADC_RESOLUTION_12_BIT
#define ADCK_DATA_FORMAT(idx)                                                                      \
	UTIL_CAT(ADCK_DATA_FORMAT_, DT_INST_STRING_TOKEN(idx, renesas_data_format))
#define ADCK_RESOLUTION(idx)                                                                       \
	UTIL_CAT(ADCK_RESOLUTION_, DT_INST_STRING_TOKEN(idx, renesas_data_format))
#define ADCK_FSP_RESOLUTION(idx)                                                                   \
	UTIL_CAT(ADCK_FSP_RESOLUTION_, DT_INST_STRING_TOKEN(idx, renesas_data_format))
#define ADCK_SCAN_MODE(idx)                                              \
	UTIL_CAT(ADCK_SCAN_MODE_, DT_INST_STRING_TOKEN(idx, renesas_scan_mode))

#define ADCK_TRIGGER_METHOD_software ADCK_SG_TRIGGER_METHOD_SW
#define ADCK_TRIGGER_METHOD_hardware ADCK_SG_TRIGGER_METHOD_HW
#define ADCK_TRIGGER_METHOD_ad_timer ADCK_SG_TRIGGER_METHOD_AD_TIMER
#define ADCK_TRIGGER_METHOD_th_a     ADCK_SG_TRIGGER_METHOD_THA
#define ADCK_TRIGGER_METHOD_th_b     ADCK_SG_TRIGGER_METHOD_THB
#define ADCK_TRIGGER_METHOD(idx)                                                                   \
	UTIL_CAT(ADCK_TRIGGER_METHOD_, DT_INST_STRING_TOKEN(idx, renesas_trigger_method))

#define ADCK_TRIGGER_IS_SOFTWARE(idx) DT_INST_ENUM_HAS_VALUE(idx, renesas_trigger_method, software)

#define ADCK_TRIGGER_IS_AD_TIMER(idx) DT_INST_ENUM_HAS_VALUE(idx, renesas_trigger_method, ad_timer)

#define ADCK_TRIGGER_IS_TH(idx)                                                                    \
	(DT_INST_ENUM_HAS_VALUE(idx, renesas_trigger_method, th_a) ||                              \
	 DT_INST_ENUM_HAS_VALUE(idx, renesas_trigger_method, th_b))

#define ADCK_TRIGGER_SOURCE_disabled ADCK_ADCK0_TRIGGER_SRC_DISABLE
#define ADCK_TRIGGER_SOURCE_pic2     ADCK_ADCK0_TRIGGER_SRC_PIC2
#define ADCK_TRIGGER_SOURCE(idx)                                                                   \
	UTIL_CAT(ADCK_TRIGGER_SOURCE_, DT_INST_STRING_TOKEN(idx, renesas_trigger_source))

#define ADCK_AD_TIMER_TRIGGER_software ADCK_AD_TIMER_TRIGGER_METHOD_SW
#define ADCK_AD_TIMER_TRIGGER_hardware ADCK_AD_TIMER_TRIGGER_METHOD_HW
#define ADCK_AD_TIMER_TRIGGER(idx)                                                                 \
	UTIL_CAT(ADCK_AD_TIMER_TRIGGER_, DT_INST_STRING_TOKEN(idx, renesas_ad_timer_trigger_method))

#define ADCK_CHANNEL_TO_VC_INVALID_INIT                                                            \
	[0] = ADC_RENESAS_ADCK_INVALID_VC, [1] = ADC_RENESAS_ADCK_INVALID_VC,                      \
	[2] = ADC_RENESAS_ADCK_INVALID_VC, [3] = ADC_RENESAS_ADCK_INVALID_VC,                      \
	[4] = ADC_RENESAS_ADCK_INVALID_VC, [5] = ADC_RENESAS_ADCK_INVALID_VC,                      \
	[6] = ADC_RENESAS_ADCK_INVALID_VC, [7] = ADC_RENESAS_ADCK_INVALID_VC,                      \
	[8] = ADC_RENESAS_ADCK_INVALID_VC, [9] = ADC_RENESAS_ADCK_INVALID_VC,                      \
	[10] = ADC_RENESAS_ADCK_INVALID_VC, [11] = ADC_RENESAS_ADCK_INVALID_VC,                    \
	[12] = ADC_RENESAS_ADCK_INVALID_VC, [13] = ADC_RENESAS_ADCK_INVALID_VC,                    \
	[14] = ADC_RENESAS_ADCK_INVALID_VC, [15] = ADC_RENESAS_ADCK_INVALID_VC,                    \
	[16] = ADC_RENESAS_ADCK_INVALID_VC, [17] = ADC_RENESAS_ADCK_INVALID_VC,                    \
	[18] = ADC_RENESAS_ADCK_INVALID_VC, [19] = ADC_RENESAS_ADCK_INVALID_VC,                    \
	[20] = ADC_RENESAS_ADCK_INVALID_VC, [21] = ADC_RENESAS_ADCK_INVALID_VC,                    \
	[22] = ADC_RENESAS_ADCK_INVALID_VC, [23] = ADC_RENESAS_ADCK_INVALID_VC,                    \
	[24] = ADC_RENESAS_ADCK_INVALID_VC, [25] = ADC_RENESAS_ADCK_INVALID_VC,                    \
	[26] = ADC_RENESAS_ADCK_INVALID_VC, [27] = ADC_RENESAS_ADCK_INVALID_VC,                    \
	[28] = ADC_RENESAS_ADCK_INVALID_VC, [29] = ADC_RENESAS_ADCK_INVALID_VC,                    \
	[30] = ADC_RENESAS_ADCK_INVALID_VC, [31] = ADC_RENESAS_ADCK_INVALID_VC

#define ADCK_CHILD_PHYSICAL_CHANNEL(child)                                                         \
	DT_PROP_OR(child, renesas_physical_channel, DT_REG_ADDR(child))

#define ADCK_START_VC(idx) DT_INST_PROP_OR(idx, renesas_start_virtual_channel, 0)

#define ADCK_END_VC(idx) DT_INST_PROP_OR(idx, renesas_end_virtual_channel, 0)

#define ADCK_CHILD_VC(child) DT_PROP_OR(child, renesas_virtual_channel, DT_REG_ADDR(child))

/*
 * Keep the bit index valid for both mask words. Some compilers still parse
 * both branches of preprocessor condition helpers, which can otherwise form
 * BIT(channel - 32) for a channel below 32 and report an invalid shift.
 */
#define ADCK_CHILD_PHYSICAL_CHANNEL_MASK(child)                                                    \
	((ADCK_CHILD_PHYSICAL_CHANNEL(child) < 32U)                                                \
		 ? DT_PROP_BY_IDX(DT_PARENT(child), renesas_physical_channel_mask, 0)              \
		 : DT_PROP_BY_IDX(DT_PARENT(child), renesas_physical_channel_mask, 1))

#define ADCK_CHILD_PHYSICAL_CHANNEL_BIT(child) BIT(ADCK_CHILD_PHYSICAL_CHANNEL(child) & 0x1fU)

#define ADCK_CHILD_PHYSICAL_CHANNEL_IS_VALID(child)                                                \
	((ADCK_CHILD_PHYSICAL_CHANNEL_MASK(child) & ADCK_CHILD_PHYSICAL_CHANNEL_BIT(child)) != 0U)

#define ADCK_CHILD_CONVERSION_TYPE(child) DT_ENUM_IDX_OR(child, renesas_conversion_type, 0)

#define ADCK_CHILD_CONVERSION_TYPE_NORMAL 0U
#define ADCK_CHILD_CONVERSION_TYPE_HOLD   1U

#define ADCK_CHILD_IS_HOLD(child)                                                                  \
	(ADCK_CHILD_CONVERSION_TYPE(child) == ADCK_CHILD_CONVERSION_TYPE_HOLD)

#define ADCK_CHILD_USES_PHYSICAL_INPUT(child)                                                      \
	(!ADCK_CHILD_IS_HOLD(child) && (ADCK_CHILD_CONVERSION_TYPE(child) != 3U) &&                \
	 (ADCK_CHILD_CONVERSION_TYPE(child) != 7U) && (ADCK_CHILD_CONVERSION_TYPE(child) != 14U))

#define ADCK_VC_CONV_TYPE_0  ADCK_VC_CONV_TYPE_NORMAL
#define ADCK_VC_CONV_TYPE_1  ADCK_VC_CONV_TYPE_HOLD
#define ADCK_VC_CONV_TYPE_2  ADCK_VC_CONV_TYPE_EXTENDED_SAMPLING_TIME
#define ADCK_VC_CONV_TYPE_3  ADCK_VC_CONV_TYPE_AD_CORE_SELF_DIAG
#define ADCK_VC_CONV_TYPE_4  ADCK_VC_CONV_TYPE_ADDITION_MODE
#define ADCK_VC_CONV_TYPE_5  ADCK_VC_CONV_TYPE_MPX_NORMAL
#define ADCK_VC_CONV_TYPE_6  ADCK_VC_CONV_TYPE_MPX_ADDITION_MODE
#define ADCK_VC_CONV_TYPE_7  ADCK_VC_CONV_TYPE_PIN_LEVEL_SELF_DIAGNOSTIC
#define ADCK_VC_CONV_TYPE_8  ADCK_VC_CONV_TYPE_WIRING_BREAK_DETECT_MODE_1
#define ADCK_VC_CONV_TYPE_9  ADCK_VC_CONV_TYPE_WIRING_BREAK_DETECT_MODE_2_PULL_DOWN
#define ADCK_VC_CONV_TYPE_10 ADCK_VC_CONV_TYPE_WIRING_BREAK_DETECT_MODE_2_PULL_UP
#define ADCK_VC_CONV_TYPE_11 ADCK_VC_CONV_TYPE_SELF_DIAG_WIRING_BREAK_DETECT_MODE_1
#define ADCK_VC_CONV_TYPE_12 ADCK_VC_CONV_TYPE_SELF_DIAG_WIRING_BREAK_DETECT_MODE_2_PULL_DOWN
#define ADCK_VC_CONV_TYPE_13 ADCK_VC_CONV_TYPE_SELF_DIAG_WIRING_BREAK_DETECT_MODE_2_PULL_UP
#define ADCK_VC_CONV_TYPE_14 ADCK_VC_CONV_TYPE_AD_CONV_PATH_DIAG
#define ADCK_CHILD_VC_CONV_TYPE(child)                                                             \
	UTIL_CAT(ADCK_VC_CONV_TYPE_, ADCK_CHILD_CONVERSION_TYPE(child))

#define ADCK_CHILD_GENERAL_CTRL(child)                                                             \
	((ADCK_CHILD_CONVERSION_TYPE(child) == 3U)                                                 \
		 ? DT_PROP_OR(child, renesas_self_diagnostic_level, 0)                             \
		 : ((ADCK_CHILD_CONVERSION_TYPE(child) == 14U)                                     \
			    ? DT_PROP_OR(child, renesas_adval_data, 0)                             \
			    : ADCK_CHILD_PHYSICAL_CHANNEL(child)))

#define ADCK_CHILD_TH_CHANNEL_MASK_OR(child)                                                       \
	| (ADCK_CHILD_IS_HOLD(child) ? BIT(ADCK_CHILD_PHYSICAL_CHANNEL(child)) : 0U)

#define ADCK_TH_CHILD_CHANNEL_MASK(idx)                                                            \
	(0U DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADCK_CHILD_TH_CHANNEL_MASK_OR))

#define ADCK_TH_CHANNEL_MASK(idx)                                                                  \
	(DT_INST_PROP_OR(idx, renesas_th_channel_mask, 0) | ADCK_TH_CHILD_CHANNEL_MASK(idx))

#define ADCK_TH_ENABLE_FROM_MASK(idx, th)                                                          \
	(((ADCK_TH_CHANNEL_MASK(idx) & BIT(th)) != 0U) ? ADC_TH_UNIT_CFG_ENABLE                    \
						       : ADC_TH_UNIT_CFG_DISABLE)

#define ADCK_CHILD_START_VC(child) DT_PROP(DT_PARENT(child), renesas_start_virtual_channel)

#define ADCK_CHILD_END_VC(child) DT_PROP(DT_PARENT(child), renesas_end_virtual_channel)

#define ADCK_CHILD_VALIDATE(child)                                                                 \
	BUILD_ASSERT(DT_REG_ADDR(child) < ADC_RENESAS_ADCK_MAX_ZEPHYR_CHANNELS,                    \
		     "ADCK Zephyr channel ID must be less than 32");                               \
	BUILD_ASSERT(ADCK_CHILD_VC(child) < ADC_RENESAS_ADCK_MAX_VIRTUAL_CHANNELS,                 \
		     "ADCK virtual channel is out of range");                                      \
	BUILD_ASSERT(ADCK_CHILD_VC(child) >= ADCK_CHILD_START_VC(child),                           \
		     "ADCK virtual channel is below scan group start");                            \
	BUILD_ASSERT(ADCK_CHILD_VC(child) <= ADCK_CHILD_END_VC(child),                             \
		     "ADCK virtual channel is above scan group end");                              \
	BUILD_ASSERT(ADCK_CHILD_PHYSICAL_CHANNEL(child) < ADC_RENESAS_ADCK_MAX_PHYSICAL_CHANNELS,  \
		     "ADCK physical channel selector must be less than 64");                       \
	BUILD_ASSERT(!ADCK_CHILD_USES_PHYSICAL_INPUT(child) ||                                     \
			     ADCK_CHILD_PHYSICAL_CHANNEL_IS_VALID(child),                          \
		     "ADCK physical channel is unavailable on this instance");                     \
	BUILD_ASSERT(!ADCK_CHILD_IS_HOLD(child) ||                                                 \
			     (ADCK_CHILD_PHYSICAL_CHANNEL(child) < ADCK_NUMBER_OF_TH_UNIT),        \
		     "ADCK T&H channel is out of range");

#define ADCK_CHILD_CHANNEL_TO_VC_INIT(child) [DT_REG_ADDR(child)] = ADCK_CHILD_VC(child),

#define ADCK_CHILD_VC_CFG_INIT(child)                                                              \
	[ADCK_CHILD_VC(child)] = {                                                                 \
		.vcr_b.general_ctrl = ADCK_CHILD_GENERAL_CTRL(child),                              \
		.vcr_b.channel_end_int = ADCK_VC_END_INT_DISABLE,                                  \
		.vcr_b.mpx_value = DT_PROP_OR(child, renesas_mpx_value, 0),                        \
		.vcr_b.conv_type = ADCK_CHILD_VC_CONV_TYPE(child),                                 \
		.vcr_b.wait_time_sel = DT_PROP_OR(child, renesas_wait_table, 0),                   \
		.vcr_b.ul_limit_sel = DT_PROP_OR(child, renesas_upper_lower_limit_table, 0),       \
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

#define ADCK_IRQ_NAME_0             sg0
#define ADCK_IRQ_NAME_1             sg1
#define ADCK_IRQ_NAME_2             sg2
#define ADCK_IRQ_NAME_3             sg3
#define ADCK_IRQ_NAME_4             sg4
#define ADCK_SELECTED_IRQ_NAME(idx) UTIL_CAT(ADCK_IRQ_NAME_, DT_INST_PROP(idx, renesas_scan_group))
#define ADCK_SELECTED_IRQ(idx)      DT_INST_IRQ_BY_NAME(idx, ADCK_SELECTED_IRQ_NAME(idx), irq)
#define ADCK_SELECTED_IRQ_PRIO(idx) DT_INST_IRQ_BY_NAME(idx, ADCK_SELECTED_IRQ_NAME(idx), priority)

#define ADCK_SCAN_GROUP_IS(idx, group) IS_EQ(DT_INST_PROP(idx, renesas_scan_group), group)

#define ADCK_SCAN_END_IRQ(idx, group)                                                              \
	COND_CODE_1(ADCK_SCAN_GROUP_IS(idx, group),                                                \
			(UTIL_CAT(ADCK_IRQ_SG, group)(idx)),                                       \
			(FSP_INVALID_VECTOR))

#define ADCK_SCAN_END_IPL(idx, group)                                                              \
	COND_CODE_1(ADCK_SCAN_GROUP_IS(idx, group), \
			(UTIL_CAT(ADCK_IRQ_PRIO_SG, group)(idx)), \
			(BSP_IRQ_DISABLED))

#define ADCK_CLOCK_INIT(i, idx)								\
	{										\
		.dev = DEVICE_DT_GET(DT_CLOCKS_CTLR_BY_IDX(DT_DRV_INST(idx), i)),	\
		.subsys = (clock_control_subsys_t)(uintptr_t)			\
			DT_CLOCKS_CELL_BY_IDX(DT_DRV_INST(idx), i, clk_id),	\
	}

#define ADCK_DEVICE_INIT(idx)                                                                      \
	BUILD_ASSERT(!IS_ENABLED(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM) ||                          \
		!DT_INST_ENUM_HAS_VALUE(idx, renesas_scan_mode, multicycle),                       \
			"ADCK streaming does not support multicycle scan mode");                   \
	BUILD_ASSERT(ADCK_START_VC(idx) <= ADCK_END_VC(idx),                                       \
			 "ADCK start virtual channel must not exceed end virtual channel");        \
	BUILD_ASSERT(!ADCK_TRIGGER_IS_AD_TIMER(idx) ||                                             \
				 (DT_INST_PROP(idx, renesas_scan_group) >= 3U),                    \
			 "ADCK A/D timer is supported only by SG3 and SG4");                       \
	BUILD_ASSERT(!ADCK_TRIGGER_IS_TH(idx) || (DT_INST_PROP(idx, renesas_scan_group) != 0U),    \
			 "ADCK T&H trigger is unsupported by SG0");                                \
	BUILD_ASSERT(!ADCK_TRIGGER_IS_AD_TIMER(idx) ||                                             \
				 (DT_INST_PROP(idx, renesas_ad_timer_cycle) >                      \
				  DT_INST_PROP(idx, renesas_ad_timer_initial_phase)),              \
			 "ADCK A/D timer cycle must exceed its initial phase");                    \
	BUILD_ASSERT(IS_ENABLED(CONFIG_ADC_ASYNC) || ADCK_TRIGGER_IS_SOFTWARE(idx) ||              \
				IS_ENABLED(CONFIG_ADC_STREAM),		\
			 "Hardware-triggered ADCK requires CONFIG_ADC_ASYNC");                     \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADCK_CHILD_VALIDATE)                                \
	static const uint8_t adc_adck_channel_to_vc_##idx[ADC_RENESAS_ADCK_MAX_ZEPHYR_CHANNELS] =  \
		{ADCK_CHANNEL_TO_VC_INVALID_INIT,                                                  \
		 DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADCK_CHILD_CHANNEL_TO_VC_INIT)};           \
	static const adck_vc_cfg_t adc_adck_vc_cfg_##idx[ADC_RENESAS_ADCK_MAX_VIRTUAL_CHANNELS] =  \
		{DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADCK_CHILD_VC_CFG_INIT)};                  \
	static const adck_sg_generic_cfg_t adc_adck_sg_generic_cfg_##idx = {                       \
		.adck_trigger_src = ADCK_TRIGGER_SOURCE(idx),                                      \
		.trigger_method = ADCK_TRIGGER_METHOD(idx),                                        \
		.sg_ctrl =                                                                         \
			{									\
				.sgcr_b.end_int = ADCK_SG_END_INT_ENABLE,			\
				.sgcr_b.scan_mode = ADCK_SCAN_MODE(idx),			\
			},									\
		.scan_cycle = 0U,								\
		.start_vc_ptr = ADCK_START_VC(idx),						\
		.end_vc_ptr = ADCK_END_VC(idx),							\
		.p_vc_cfg = &adc_adck_vc_cfg_##idx[ADCK_START_VC(idx)],				\
	};											\
	static const adck_sg_cfg_t adc_adck_sg_cfg_##idx = {					\
		.p_sg_cfg =									\
			{									\
				[DT_INST_PROP(idx, renesas_scan_group)] =			\
					&adc_adck_sg_generic_cfg_##idx,				\
			},									\
		.p_sg_diag_cfg = NULL,								\
		.pwm_diag_cfg = ADCK_PWM_DIAG_CFG_DISABLE,					\
		.tsn0_measure_cfg = ADCK_TSN0_MEASURE_CFG_DISABLE,				\
	};											\
	static const adck_extended_cfg_t adc_adck_extend_cfg_##idx = {				\
		.suspend_method = ADCK_SUSPEND_METHOD_SYNCHRONOUS,				\
		.data_format_addition_count =							\
			{									\
				.adcr2_b.data_format = ADCK_DATA_FORMAT(idx),			\
				.adcr2_b.addition_count = ADCK_ADDITION_COUNT_TWICE,		\
			},									\
		.safety_cfg =									\
			{									\
				.sftcr_b.id = ADCK_ID_ERROR_INT_DISABLE,			\
				.sftcr_b.parity = ADCK_PARITY_ERROR_INT_DISABLE,		\
				.sftcr_b.overwrite = ADCK_OVERWRITE_ERROR_INT_DISABLE,		\
				.sftcr_b.read_clear = ADCK_CLEAR_AFTER_READ_DISABLE,		\
				.sftcr_b.trigger_overlap = ADCK_TRIGGER_OVERLAP_DISABLE,	\
			},									\
		.ul_limit_cfg =									\
			{									\
				.vc_ul_limit_int_0 = 0U,					\
				.vc_ul_limit_int_1 = 0U,					\
				.pwm_diag_ul_limit_int = ADCK_PWM_DIAG_UL_LIMIT_INT_DISABLE,	\
				.sg_diag_ul_limit_int = ADCK_SG_DIAG_UL_LIMIT_INT_DISABLE,	\
			},									\
		.sampling_cfg =									\
			{									\
				.smpcr_b.period = ADCK_SAMPLING_PERIOD_18_STATES,		\
				.smpcr_b.period_sel = ADCK_SAMPLING_PERIOD_SEL_DISABLE,		\
				.smpcr_b.buffer_amp = ADCK_BUFFER_AMP_ENABLE,			\
				.smpcr_b.extended_period =					\
					ADCK_EXTENDED_SAMPLING_PERIOD_60_STATES,		\
			},									\
		.pin_lv_self_diag_cfg =								\
			{									\
				.tdcr_b.voltage_level =						\
					ADCK_PIN_LV_SELF_DIAG_VOL_EVEN_AVSS_ODD_AVCC,		\
				.tdcr_b.th_path_self_diag = ADCK_TH_PATH_SELF_DIAG_DISABLE,	\
			},									\
		.wiring_break_detect_cfg =							\
			{									\
				.odcr_b.pulse_width =						\
					ADCK_WIRING_BREAK_DETECT_PULSE_WIDTH_1_STATE,		\
				.odcr_b.addition_mode = ADCK_WIRING_BREAK_ADDITION_MODE_DISABLE,\
			},									\
		.mpx_cfg =									\
			{									\
				.mask_control_format = 0U,					\
				.spi_command_information = 0U,					\
				.mpx_int = ADCK_MPX_INT_DISABLE,				\
			},									\
		.trigger_overlap_detect = ADCK_TRIGGER_OVERLAP_DETECT_DISABLE,			\
		.th_cfg =									\
			{									\
				.group_select_cfg =						\
					{							\
						.thgsr_b.th0_group_cfg = ADCK_TH_GROUP_A,	\
						.thgsr_b.th1_group_cfg = ADCK_TH_GROUP_A,	\
						.thgsr_b.th2_group_cfg = ADCK_TH_GROUP_A,	\
						.thgsr_b.th3_group_cfg = ADCK_TH_GROUP_A,	\
					},							\
				.th_enable_cfg =						\
					{							\
						.ther_b.th0_enable =				\
							ADCK_TH_ENABLE_FROM_MASK(idx, 0),	\
						.ther_b.th1_enable =				\
							ADCK_TH_ENABLE_FROM_MASK(idx, 1),	\
						.ther_b.th2_enable =				\
							ADCK_TH_ENABLE_FROM_MASK(idx, 2),	\
						.ther_b.th3_enable =				\
							ADCK_TH_ENABLE_FROM_MASK(idx, 3),	\
					},							\
				.trigger_method[ADCK_TH_GROUP_A] =				\
					ADCK_TH_GROUP_TRIGGER_METHOD_SW,			\
				.trigger_method[ADCK_TH_GROUP_B] =				\
					ADCK_TH_GROUP_TRIGGER_METHOD_SW,			\
			},									\
		.ad_timer_cfg =									\
			{									\
				.trigger_method[0] = ADCK_AD_TIMER_TRIGGER(idx),		\
				.trigger_method[1] = ADCK_AD_TIMER_TRIGGER(idx),		\
				.initial_phase[0] =						\
					DT_INST_PROP(idx, renesas_ad_timer_initial_phase),	\
				.initial_phase[1] =						\
					DT_INST_PROP(idx, renesas_ad_timer_initial_phase),	\
				.cycle[0] = DT_INST_PROP(idx, renesas_ad_timer_cycle),		\
				.cycle[1] = DT_INST_PROP(idx, renesas_ad_timer_cycle),		\
			},									\
		.int_cfg =									\
			{									\
				.error_irq = FSP_INVALID_VECTOR,				\
				.error_ipl = BSP_IRQ_DISABLED,					\
				.scan_group_0_end_irq = ADCK_SCAN_END_IRQ(idx, 0),		\
				.scan_group_0_end_ipl = ADCK_SCAN_END_IPL(idx, 0),		\
				.scan_group_1_end_irq = ADCK_SCAN_END_IRQ(idx, 1),		\
				.scan_group_1_end_ipl = ADCK_SCAN_END_IPL(idx, 1),		\
				.scan_group_2_end_irq = ADCK_SCAN_END_IRQ(idx, 2),		\
				.scan_group_2_end_ipl = ADCK_SCAN_END_IPL(idx, 2),		\
				.scan_group_3_end_irq = ADCK_SCAN_END_IRQ(idx, 3),		\
				.scan_group_3_end_ipl = ADCK_SCAN_END_IPL(idx, 3),		\
				.scan_group_4_end_irq = ADCK_SCAN_END_IRQ(idx, 4),		\
				.scan_group_4_end_ipl = ADCK_SCAN_END_IPL(idx, 4),		\
				.sg_diag_end_irq = FSP_INVALID_VECTOR,				\
				.sg_diag_end_ipl = BSP_IRQ_DISABLED,				\
				.mpx_irq = FSP_INVALID_VECTOR,			 \
				.mpx_ipl = BSP_IRQ_DISABLED,			   \
			},								\
	};											\
	static const adc_cfg_t adc_adck_fsp_cfg_##idx = {				  \
		.unit = DT_INST_PROP(idx, renesas_unit),				   \
		.mode = ADC_MODE_SINGLE_SCAN,							  \
		.resolution = ADCK_FSP_RESOLUTION(idx),					\
		.alignment = ADC_ALIGNMENT_RIGHT,						  \
		.trigger = ADC_TRIGGER_SOFTWARE,						   \
		.scan_end_irq = FSP_INVALID_VECTOR,						\
		.scan_end_b_irq = FSP_INVALID_VECTOR,					  \
		.scan_end_c_irq = FSP_INVALID_VECTOR,					  \
		.scan_end_ipl = BSP_IRQ_DISABLED,						  \
		.scan_end_b_ipl = BSP_IRQ_DISABLED,						\
		.scan_end_c_ipl = BSP_IRQ_DISABLED,						\
		.p_callback = NULL,							\
		.p_context = NULL,								\
		.p_extend = &adc_adck_extend_cfg_##idx,					\
	};											 \
	static void adc_adck_irq_config_##idx(const struct device *dev)			\
	{											  \
		ARG_UNUSED(dev);							\
		IRQ_CONNECT(ADCK_SELECTED_IRQ(idx), ADCK_SELECTED_IRQ_PRIO(idx),		   \
				adc_adck_scanend_dispatch_isr, DEVICE_DT_INST_GET(idx),		\
				ADCK_IRQ_FLAGS(idx));						  \
		irq_enable(ADCK_SELECTED_IRQ(idx));						\
	}											  \
	static const struct adc_adck_clock adc_adck_clocks_##idx[] = {	\
		LISTIFY(DT_INST_PROP_LEN(idx, clocks), ADCK_CLOCK_INIT, (,), idx)	\
	};	\
	static const struct adc_adck_config adc_adck_config_##idx = {				\
		.channel_available_mask = DT_INST_PROP(idx, channel_available_mask),		\
		.scan_group = DT_INST_PROP(idx, renesas_scan_group),				\
		.resolution = ADCK_RESOLUTION(idx),				\
		.data_format = ADCK_DATA_FORMAT(idx),						\
		.vref_mv = DT_INST_PROP_OR(idx, vref_mv, 0),					\
		.software_trigger = ADCK_TRIGGER_IS_SOFTWARE(idx),		 \
		.uses_th = ADCK_TH_CHANNEL_MASK(idx) != 0U,				\
		.scan_group_mask = ADCK_GROUP_MASK_FROM_GROUP(idx),		\
		.fsp_cfg = &adc_adck_fsp_cfg_##idx,						\
		.fsp_sg_cfg = &adc_adck_sg_cfg_##idx,					  \
		.channel_to_vc = adc_adck_channel_to_vc_##idx,			 \
		.clocks = adc_adck_clocks_##idx,	\
		.clock_count = ARRAY_SIZE(adc_adck_clocks_##idx),	\
		.irq_config_func = adc_adck_irq_config_##idx,			  \
	};											 \
	static struct adc_adck_data adc_adck_data_##idx = {				\
		ADC_CONTEXT_INIT_TIMER(adc_adck_data_##idx, ctx),		  \
		ADC_CONTEXT_INIT_LOCK(adc_adck_data_##idx, ctx),		   \
		ADC_CONTEXT_INIT_SYNC(adc_adck_data_##idx, ctx),		   \
		.dev = DEVICE_DT_INST_GET(idx),							\
		ADCK_DMA_INIT(idx) \
	};											 \
	static DEVICE_API(adc, adc_adck_api_##idx) = {						\
		.channel_setup = adc_adck_channel_setup,					\
		.read = adc_adck_read,								\
		.ref_internal = DT_INST_PROP_OR(idx, vref_mv, 0),				   \
		IF_ENABLED(CONFIG_ADC_ASYNC,							\
			(.read_async = adc_adck_read_async,))					\
		IF_ENABLED(CONFIG_ADC_RENESAS_RH850_ADCK_STREAM,			\
			(.submit = adc_adck_submit,						\
			.get_decoder = adc_adck_get_decoder,))				  \
	}; \
	DEVICE_DT_INST_DEFINE(idx, adc_adck_init, NULL, &adc_adck_data_##idx, \
				&adc_adck_config_##idx, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, \
				&adc_adck_api_##idx);

DT_INST_FOREACH_STATUS_OKAY(ADCK_DEVICE_INIT)
