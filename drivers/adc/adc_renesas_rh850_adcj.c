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
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/clock_control.h>

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA)
#include <zephyr/drivers/dma.h>
#endif

#include "r_adcj.h"

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE
#include "adc_context.h"

LOG_MODULE_REGISTER(adc_renesas_rh850_adcj, CONFIG_ADC_LOG_LEVEL);

#define ADC_RENESAS_ADCJ_INVALID_VC            UINT8_MAX

/* Channel masks in this driver use uint32_t. */
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

/* One stream frame carries at most one sample for each Zephyr channel. */
#define ADC_RENESAS_ADCJ_STREAM_MAX_CHANNELS ADC_RENESAS_ADCJ_MAX_ZEPHYR_CHANNELS

/* FSP ADCJ has 64 virtual channel data registers. */
#define ADC_RENESAS_ADCJ_MAX_VIRTUAL_CHANNELS ADCJ_NUMBER_OF_VIRTUAL_CHANNEL

#define ADC_ADCJ_STREAM_FLAG_VALID      BIT(0)
#define ADC_ADCJ_STREAM_FLAG_DMA        BIT(1)
#define ADC_ADCJ_STREAM_FLAG_WIRING_BREAK_ADDITION BIT(2)

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

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA)

#define ADCJ_DMA_INIT(idx) \
	.dma_dev = ADCJ_DMA_DEV(idx), \
	.dma_channel = ADCJ_DMA_CHANNEL(idx), \
	.dma_slot = ADCJ_DMA_CONFIG(idx), \
	.dma_group = ADCJ_DMA_GROUP(idx),

#else

#define ADCJ_DMA_INIT(idx)

#endif

extern void adcj_sg0_scan_end_isr(void);
extern void adcj_sg1_scan_end_isr(void);
extern void adcj_sg2_scan_end_isr(void);
extern void adcj_sg3_scan_end_isr(void);
extern void adcj_sg4_scan_end_isr(void);
extern void adcj_error_isr(void);

enum adc_adcj_request_mode {
	ADC_ADCJ_REQUEST_IDLE = 0,
	ADC_ADCJ_REQUEST_FINITE,
	ADC_ADCJ_REQUEST_STREAM,
	ADC_ADCJ_REQUEST_STREAM_STOPPING,
	ADC_ADCJ_REQUEST_FAULT,
};

struct adc_adcj_stream_header {
	uint64_t timestamp_ns;
	uint32_t sequence_number;

	uint16_t payload_size;
	uint8_t num_channels;
	uint8_t data_format;

	uint8_t flags;
	uint8_t addition_count;
	uint16_t vref_mv;
};

struct adc_adcj_stream_channel {
	uint8_t channel_id;
	uint8_t virtual_channel;
	uint8_t resolution;
	uint8_t conversion_type;
};
struct adc_adcj_config {
	uint32_t channel_available_mask;
	uint8_t scan_group;
	uint8_t resolution;
	uint8_t data_format;
	uint16_t vref_mv;
	uint8_t track_hold_mask;
	bool software_trigger;
	bool uses_th;
	adc_group_mask_t scan_group_mask;

	const adc_cfg_t *fsp_cfg;
	const adcj_sg_cfg_t *fsp_sg_cfg;
	const uint8_t *channel_to_vc;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
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

	enum adc_adcj_request_mode request_mode;

	bool active;
	int status;

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM)

	struct rtio_iodev_sqe *stream_sqe;
	const struct adc_read_config *stream_config;
	uint8_t *stream_buffer;
	uint32_t stream_buffer_len;
	uint32_t stream_channels;
	uint32_t stream_frame_size;
	uint32_t stream_sequence_number;
#endif

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA)

	const struct device *dma_dev;
	uint32_t dma_channel;
	uint32_t dma_slot;
	uint32_t dma_group;
	struct dma_config dma_cfg;
	struct dma_block_config dma_block;
	adc_info_t adc_info;
	void *dma_src;
	uint32_t dma_length;
	bool dma_active;

#endif
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

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM)

static void adc_adcj_stream_reset(struct adc_adcj_data *data)
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

	data->active = false;
	data->status = 0;
	data->request_mode = ADC_ADCJ_REQUEST_IDLE;
}

static void adc_adcj_stream_fail(const struct device *dev, int error)
{
	const struct adc_adcj_config *config;
	struct adc_adcj_data *data;
	struct rtio_iodev_sqe *sqe;
	fsp_err_t fsp_err;

	if (dev == NULL) {
		return;
	}

	config = dev->config;
	data = dev->data;

	if ((config == NULL) || (data == NULL)) {
		return;
	}

	/*
	 * Only a stream request or an already stopping stream request may
	 * enter this cleanup path.
	 */
	if ((data->request_mode != ADC_ADCJ_REQUEST_STREAM) &&
	    (data->request_mode != ADC_ADCJ_REQUEST_STREAM_STOPPING)) {
		return;
	}

	/*
	 * Preserve the first stream error. If DMA cannot be stopped, this
	 * error will be reported later from the DMA callback.
	 */
	if (data->status == 0) {
		data->status = error;
	}

	/*
	 * Do not accept new requests while stopping the hardware. Keep the
	 * SQE and RX buffer owned by the driver until DMA is quiescent.
	 */
	data->active = false;
	data->request_mode = ADC_ADCJ_REQUEST_STREAM_STOPPING;

	/*
	 * Stop ADCJ first so that it cannot generate another DMA request.
	 */
	fsp_err = R_ADCJ_ScanGroupStop(
		&data->fsp_ctrl,
		config->scan_group_mask);

	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("ADCJ stream cleanup failed: %d", fsp_err);

		/*
		 * ADCJ is not confirmed stopped. Keep the device faulted and
		 * do not accept another request.
		 */
		data->request_mode = ADC_ADCJ_REQUEST_FAULT;

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA)
		/*
		 * If DMA is still active, retain the SQE and buffer until its
		 * callback confirms that the transfer has ended.
		 */
		if (data->dma_active) {
			return;
		}
#endif

		/*
		 * DMA is not using the buffer, so the SQE can be completed,
		 * but the device remains faulted because ADCJ did not stop.
		 */
		sqe = data->stream_sqe;

		data->stream_sqe = NULL;
		data->stream_buffer = NULL;
		data->stream_buffer_len = 0U;

		if (sqe != NULL) {
			rtio_iodev_sqe_err(sqe, data->status);
		}

		return;
	}

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA)
	if (data->dma_active) {
		int dma_err;

		if ((data->dma_dev == NULL) ||
		    !device_is_ready(data->dma_dev)) {
			LOG_ERR("DMA device is not ready during cleanup");
			data->request_mode = ADC_ADCJ_REQUEST_FAULT;
			return;
		}

		dma_err = dma_stop(
			data->dma_dev,
			data->dma_channel);

		if (dma_err != 0) {
			LOG_ERR("DMA stop failed: %d", dma_err);

			/*
			 * Keep STREAM_STOPPING and retain stream_sqe and
			 * stream_buffer. The DMA callback will complete the
			 * SQE after the in-flight transfer has ended.
			 */
			return;
		}

		/*
		 * A successful dma_stop() confirms that the channel no longer
		 * owns the current RX buffer.
		 */
		data->dma_active = false;
	}
#endif

	/*
	 * ADCJ is stopped and DMA no longer owns the buffer. It is now safe
	 * to reset the stream state and return the buffer to RTIO.
	 */
	sqe = data->stream_sqe;
	error = data->status;

	adc_adcj_stream_reset(data);

	if (sqe != NULL) {
		rtio_iodev_sqe_err(sqe, error);
	}
}

static struct adc_adcj_stream_header *adc_adcj_stream_header(uint8_t *buffer)
{
	return (struct adc_adcj_stream_header *)buffer;
}

static struct adc_adcj_stream_channel *adc_adcj_stream_channels(uint8_t *buffer)
{
	return (struct adc_adcj_stream_channel *)
		(buffer + sizeof(struct adc_adcj_stream_header));
}

static uint8_t *adc_adcj_stream_sample_payload(uint8_t *buffer,
							   uint8_t num_channels)
{
	return buffer +
		sizeof(struct adc_adcj_stream_header) +
		   num_channels *
			sizeof(struct adc_adcj_stream_channel);
}

static size_t adc_adcj_stream_frame_size(uint32_t channels)
{
	size_t channel_count = POPCOUNT(channels);

	return sizeof(struct adc_adcj_stream_header) +
		   channel_count *
			   sizeof(struct adc_adcj_stream_channel) +
		   channel_count * sizeof(uint16_t);
}

static int adc_adcj_stream_prepare_frame(const struct device *dev)
{
	const struct adc_adcj_config *config;
	struct adc_adcj_data *data;
	struct adc_adcj_stream_header *header;
	struct adc_adcj_stream_channel *channel_info;
	size_t expected_frame_size;
	size_t payload_size;
	uint8_t channel_count;
	uint8_t sample_index = 0U;
	const adcj_extended_cfg_t *extend;
	const adcj_sg_generic_cfg_t *sg_cfg;

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
	if ((data->request_mode != ADC_ADCJ_REQUEST_STREAM) ||
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
		(channel_count > ADC_RENESAS_ADCJ_STREAM_MAX_CHANNELS)) {
		return -EINVAL;
	}

	payload_size =
		(size_t)channel_count * sizeof(uint16_t);

	if (payload_size > UINT16_MAX) {
		return -EOVERFLOW;
	}

	expected_frame_size =
		adc_adcj_stream_frame_size(data->stream_channels);

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

	header = adc_adcj_stream_header(data->stream_buffer);
	channel_info =
		adc_adcj_stream_channels(data->stream_buffer);

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
	ADCJ_WIRING_BREAK_ADDITION_MODE_ENABLE) {
		header->flags |= ADC_ADCJ_STREAM_FLAG_WIRING_BREAK_ADDITION;
	}
	header->addition_count = (uint8_t)extend->data_format_addition_count.adcr2_b.addition_count;
	header->vref_mv = config->vref_mv;

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA)
	header->flags |= ADC_ADCJ_STREAM_FLAG_DMA;
#endif

	/*
	 * Channel descriptors and raw samples use the same ordering.
	 *
	 * For example:
	 *
	 * channel_info[0] <-> raw_sample[0]
	 * channel_info[1] <-> raw_sample[1]
	 */

	for (uint8_t virtual_channel = sg_cfg->start_vc_ptr; virtual_channel <= sg_cfg->end_vc_ptr;
		 virtual_channel++) {
		uint8_t channel;

		if (sample_index >= channel_count) {
			return -EOVERFLOW;
		}

		for (channel = 0U;
		channel < ADC_RENESAS_ADCJ_MAX_ZEPHYR_CHANNELS;
		channel++) {
			if (((data->stream_channels & BIT(channel)) != 0U) &&
			(config->channel_to_vc[channel] == virtual_channel)) {
				break;
			}
		}

		if (channel == ADC_RENESAS_ADCJ_MAX_ZEPHYR_CHANNELS) {
			return -EINVAL;
		}

		channel_info[sample_index].channel_id = channel;
		channel_info[sample_index].virtual_channel = virtual_channel;
		channel_info[sample_index].resolution = config->resolution;
		channel_info[sample_index].conversion_type =
			 (uint8_t)sg_cfg->p_vc_cfg[virtual_channel - sg_cfg->start_vc_ptr]
			 .vcr_b.conv_type;

		sample_index++;
	}

	if (sample_index != channel_count) {
		return -EINVAL;
	}

	return 0;
}


static int adc_adcj_validate_stream_config(
	const struct device *dev,
	const struct adc_read_config *read_cfg,
	uint32_t *channel_mask)
{
	const struct adc_adcj_config *config = dev->config;
	struct adc_adcj_data *data = dev->data;
	uint32_t channels = 0U;

	if ((read_cfg == NULL) || !read_cfg->is_streaming) {
		return -EINVAL;
	}

	if ((read_cfg->adc != dev) ||
		(read_cfg->adc_spec == NULL) ||
		(read_cfg->adc_spec_cnt == 0U) ||
		(read_cfg->adc_spec_cnt > ADC_RENESAS_ADCJ_STREAM_MAX_CHANNELS)) {
		return -EINVAL;
	}

	for (size_t i = 0; i < read_cfg->adc_spec_cnt; i++) {
		const struct adc_dt_spec *spec = &read_cfg->adc_spec[i];

		if (spec->dev != dev) {
			return -EINVAL;
		}

		if (spec->channel_id >= ADC_RENESAS_ADCJ_MAX_ZEPHYR_CHANNELS) {
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
			ADC_RENESAS_ADCJ_MAX_VIRTUAL_CHANNELS) {
			return -EINVAL;
		}

		if ((channels & BIT(spec->channel_id)) != 0U) {
			return -EINVAL;
		}

		channels |= BIT(spec->channel_id);
	}

	/*
	 * Initial implementation supports one completion per completed
	 * ADCJ scan. DATA_READY means that one complete frame is available.
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

static int adc_adcj_stream_acquire_buffer(struct adc_adcj_data *data)
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

static int adc_adcj_stream_read_frame(const struct device *dev)
{
	const struct adc_adcj_config *config = dev->config;
	struct adc_adcj_data *data = dev->data;
	struct adc_adcj_stream_header *header;
	struct adc_adcj_stream_channel *channels;
	uint8_t *sample_payload;
	uint8_t sample_index;
	const adcj_sg_generic_cfg_t *sg_cfg;

	if ((config == NULL) || (data == NULL)) {
		return -EINVAL;
	}

	if ((data->stream_buffer == NULL) ||
		 (data->stream_buffer_len < data->stream_frame_size)) {
		return -ENOMEM;
	}

	sg_cfg = config->fsp_sg_cfg->p_sg_cfg[config->scan_group];

	if ((sg_cfg == NULL) || (sg_cfg->p_vc_cfg == NULL)) {
		return -EINVAL;
	}

	header = adc_adcj_stream_header(data->stream_buffer);

	if ((header->num_channels == 0U) ||
		 (header->num_channels >
		 ADC_RENESAS_ADCJ_STREAM_MAX_CHANNELS)) {
		return -EINVAL;
	}

	if (header->payload_size !=
		 ((uint16_t)header->num_channels *
		 sizeof(uint16_t))) {
		return -EINVAL;
	}

	channels = adc_adcj_stream_channels(data->stream_buffer);

	sample_payload =
		adc_adcj_stream_sample_payload(
			data->stream_buffer,
			header->num_channels);

	for (sample_index = 0U;
		 sample_index < header->num_channels;
		 sample_index++) {

		uint8_t virtual_channel;
		uint16_t raw;
		fsp_err_t fsp_err;

		virtual_channel =
			channels[sample_index].virtual_channel;

		if (virtual_channel >=
		    ADC_RENESAS_ADCJ_MAX_VIRTUAL_CHANNELS) {
			return -EINVAL;
		}

		if ((virtual_channel < sg_cfg->start_vc_ptr) ||
			 (virtual_channel > sg_cfg->end_vc_ptr)) {
			return -EINVAL;
		}

		fsp_err = R_ADCJ_Read(
			&data->fsp_ctrl,
			(adc_channel_t)virtual_channel,
			&raw);

		if (fsp_err != FSP_SUCCESS) {
			return adc_adcj_fsp_err_to_errno(
				fsp_err);
		}

		/*
		 * Store the sample at the same index as the
		 * corresponding channel descriptor.
		 *
		 * channel_info[i] <-> sample_payload[i]
		 */
		memcpy(
			sample_payload +
				((size_t)sample_index *
				 sizeof(raw)),
			&raw,
			sizeof(raw));
	}

	header->timestamp_ns =
		k_ticks_to_ns_floor64(k_uptime_ticks());

	header->flags |= ADC_ADCJ_STREAM_FLAG_VALID;

	return 0;
}

#endif
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
#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA)

static void adc_adcj_dma_callback(const struct device *dma_dev, void *user_data,
	uint32_t channel, int status)
{
	struct adc_adcj_data *data = user_data;
	struct adc_adcj_stream_header *header;
	struct rtio_iodev_sqe *completed_sqe;
	int completion_error;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (data == NULL) {
		return;
	}

	/*
	 * Confirm according to the RH850 DMA callback contract that reaching
	 * this callback means the transfer no longer accesses its destination.
	 */
	data->dma_active = false;

	if ((data->request_mode ==
		 ADC_ADCJ_REQUEST_STREAM_STOPPING) ||
		 (data->request_mode ==
		 ADC_ADCJ_REQUEST_FAULT)) {
		completed_sqe = data->stream_sqe;

		/*
		 * Prefer the original stream error. If none was stored, use
		 * the DMA callback error or a generic I/O error.
		 */
		if (data->status != 0) {
			completion_error = data->status;
		} else if (status < 0) {
			completion_error = status;
		} else {
			completion_error = -EIO;
		}

		/*
		 * The DMA transfer has finished, so it is now safe to release
		 * the destination buffer.
		 *
		 * Only return to IDLE if ADCJ was also confirmed stopped.
		 */
		if (data->request_mode ==
			 ADC_ADCJ_REQUEST_STREAM_STOPPING) {
			adc_adcj_stream_reset(data);
		} else {
			/*
			 * DMA is done but another hardware cleanup operation
			 * failed. Release the buffer while retaining FAULT.
			 */
			data->stream_sqe = NULL;
			data->stream_buffer = NULL;
			data->stream_buffer_len = 0U;
			data->active = false;
		}

		if (completed_sqe != NULL) {
			rtio_iodev_sqe_err(
				completed_sqe,
				completion_error);
		}

		return;
	}

	if (status < 0) {
		adc_adcj_stream_fail(data->dev, status);
		return;
	}

	if ((data->request_mode != ADC_ADCJ_REQUEST_STREAM) || !data->active ||
	(data->stream_sqe == NULL) || (data->stream_buffer == NULL)) {
		return;
	}

	header = adc_adcj_stream_header(data->stream_buffer);
	header->timestamp_ns = k_ticks_to_ns_floor64(k_uptime_ticks());
	header->flags |= ADC_ADCJ_STREAM_FLAG_VALID;
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
#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM)
		if (data->request_mode == ADC_ADCJ_REQUEST_STREAM) {
			adc_adcj_stream_fail(dev, -EIO);
			return;
		}
#endif
		adc_context_complete(&data->ctx, -EIO);
		return;
	}

	if (args->event != (ADC_EVENT_SCAN_COMPLETE_GROUP_0 + config->scan_group)) {
		return;
	}

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM)
	if (data->request_mode == ADC_ADCJ_REQUEST_STREAM) {
#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA)
		return;
#else
		struct rtio_iodev_sqe *completed_sqe;

		err = adc_adcj_stream_read_frame(dev);
		if (err != 0) {
			adc_adcj_stream_fail(dev, err);
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


	/*
	 * A sequence with extra samplings or ADC_ACTION_REPEAT starts the next
	 * scan from adc_context_on_sampling_done().  Stop the completed hardware
	 * scan first so that the following R_ADCJ_ScanGroupStart() does not return
	 * FSP_ERR_IN_USE.
	 */
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

	/*
	 * Reject the finite request before modifying shared state if the
	 * ADC is currently owned by a stream or another finite request.
	 */
	if (data->request_mode != ADC_ADCJ_REQUEST_IDLE) {
		return -EBUSY;
	}

	/*
	 * Claim ownership before updating finite-request state or starting
	 * the ADC context.
	 */
	data->request_mode = ADC_ADCJ_REQUEST_FINITE;

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
		struct k_poll_signal *sig)
{
	struct adc_adcj_data *data = dev->data;
	int err;

	if (sequence == NULL) {
		return -EINVAL;
	}

	adc_context_lock(&data->ctx, asynchronous, sig);
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

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM)

static void adc_adcj_submit(const struct device *dev,
				struct rtio_iodev_sqe *iodev_sqe)
{
	fsp_err_t fsp_err;
	const struct adc_read_config *read_cfg;
	uint32_t channels;
	int err;

	if ((dev == NULL) || (iodev_sqe == NULL)) {
		return;
	}

	struct adc_adcj_data *data = dev->data;

	if (data->request_mode == ADC_ADCJ_REQUEST_IDLE) {
		const struct adc_adcj_config *config = dev->config;

		if ((iodev_sqe->sqe.iodev == NULL) ||
			(iodev_sqe->sqe.iodev->data == NULL)) {
			rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
			return;
		}
		read_cfg = iodev_sqe->sqe.iodev->data;

		err = adc_adcj_validate_stream_config(dev, read_cfg, &channels);

		if (err != 0) {
			rtio_iodev_sqe_err(iodev_sqe, err);
			return;
		}

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA)
		if (data->dma_length != POPCOUNT(channels)) {
			rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
			return;
		}
#endif
		data->request_mode = ADC_ADCJ_REQUEST_STREAM;
		data->stream_sqe = iodev_sqe;
		data->stream_config = read_cfg;
		data->stream_channels = channels;
		data->stream_frame_size =
			adc_adcj_stream_frame_size(channels);
		data->status = 0;
		data->active = true;

		/* Acquire next RTIO frame buffer */
		err = adc_adcj_stream_acquire_buffer(data);

		if (err != 0) {
			adc_adcj_stream_fail(dev, err);
			return;
		}

		err = adc_adcj_stream_prepare_frame(dev);

		if (err != 0) {
			adc_adcj_stream_fail(dev, err);
			return;
		}

		uint8_t *sample_payload;
		struct adc_adcj_stream_header *header;

		header = adc_adcj_stream_header(data->stream_buffer);
		sample_payload = adc_adcj_stream_sample_payload(data->stream_buffer,
			 header->num_channels);

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA)
		data->dma_cfg.source_data_size = 2U;
		data->dma_cfg.dest_data_size = 2U;

		data->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;

		data->dma_cfg.block_count = 1U;
		data->dma_cfg.head_block = &data->dma_block;

		data->dma_cfg.dma_callback =
			adc_adcj_dma_callback;

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
			adc_adcj_stream_fail(dev, err);
			return;
		}

		err = dma_start(
			data->dma_dev,
			data->dma_channel);

		if (err != 0) {
			adc_adcj_stream_fail(dev, err);
			return;
		}
		data->dma_active = true;
#endif
		/* Start ADCJ only once */

		fsp_err = R_ADCJ_ScanGroupStart(
				&data->fsp_ctrl,
				config->scan_group_mask);

		if (fsp_err != FSP_SUCCESS) {
			adc_adcj_stream_fail(
				dev,
				adc_adcj_fsp_err_to_errno(fsp_err));
			return;
		}

	} else if (data->request_mode == ADC_ADCJ_REQUEST_STREAM) {
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
		err = adc_adcj_stream_acquire_buffer(data);

		if (err != 0) {
			adc_adcj_stream_fail(dev, err);
			return;
		}

		err = adc_adcj_stream_prepare_frame(dev);

		if (err != 0) {
			adc_adcj_stream_fail(dev, err);
			return;
		}

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA)

		uint8_t *sample_payload;
		struct adc_adcj_stream_header *header;

		header = adc_adcj_stream_header(data->stream_buffer);
		sample_payload = adc_adcj_stream_sample_payload(data->stream_buffer,
			header->num_channels);

		err = dma_reload(data->dma_dev, data->dma_channel, (uintptr_t)data->dma_src,
			(uintptr_t)sample_payload, data->dma_length * sizeof(uint16_t));
		if (err != 0) {
			adc_adcj_stream_fail(dev, err);
			return;
		}
		data->dma_active = true;
#endif
	} else {
		rtio_iodev_sqe_err(iodev_sqe, -EBUSY);
		return;
	}

}

static const struct adc_adcj_stream_header *
adc_adcj_decoder_header(const uint8_t *buffer)
{
	return (const struct adc_adcj_stream_header *)buffer;
}

static const struct adc_adcj_stream_channel *
adc_adcj_decoder_channels(const uint8_t *buffer)
{
	return (const struct adc_adcj_stream_channel *)
		(buffer + sizeof(struct adc_adcj_stream_header));
}

static const uint8_t *
adc_adcj_decoder_sample_payload(const uint8_t *buffer,
				uint8_t num_channels)
{
	return buffer +
		   sizeof(struct adc_adcj_stream_header) +
		   ((size_t)num_channels *
		sizeof(struct adc_adcj_stream_channel));
}

static int adc_adcj_decoder_find_channel(const uint8_t *buffer,
					uint32_t channel,
					uint8_t *sample_index)
{
	const struct adc_adcj_stream_header *header;
	const struct adc_adcj_stream_channel *channels;

	if ((buffer == NULL) || (sample_index == NULL)) {
		return -EINVAL;
	}

	header = adc_adcj_decoder_header(buffer);
	channels = adc_adcj_decoder_channels(buffer);

	for (uint8_t i = 0U; i < header->num_channels; i++) {
		if (channels[i].channel_id == channel) {
			*sample_index = i;
			return 0;
		}
	}

	return -ENODATA;
}

/*
 * Validate the metadata stored in an ADCJ encoded stream frame.
 *
 * The decoder API does not receive the total buffer length, so this
 * function can validate the metadata fields but cannot independently
 * prove that the backing buffer is large enough.
 */
static int adc_adcj_decoder_validate_frame(const uint8_t *buffer)
{
	const struct adc_adcj_stream_header *header;
	uint16_t expected_payload_size;

	if (buffer == NULL) {
		return -EINVAL;
	}

	header = adc_adcj_decoder_header(buffer);

	if (header->vref_mv == 0U) {
		return -EINVAL;
	}

	if ((header->flags & ADC_ADCJ_STREAM_FLAG_VALID) == 0U) {
		return -ENODATA;
	}

	if ((header->num_channels == 0U) ||
		(header->num_channels >
		 ADC_RENESAS_ADCJ_STREAM_MAX_CHANNELS)) {
		return -EINVAL;
	}

	expected_payload_size =
		(uint16_t)header->num_channels *
		sizeof(uint16_t);

	if (header->payload_size != expected_payload_size) {
		return -EINVAL;
	}

	switch ((enum adc_adcj_data_format)header->data_format) {
	case ADC_ADCJ_DATA_FORMAT_12_BIT_SIGNED_FIXED:
	case ADC_ADCJ_DATA_FORMAT_12_BIT_SIGNED_INTEGER:
	case ADC_ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_FIXED:
	case ADC_ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_RIGHT:
	case ADC_ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_LEFT:
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

/*
 * Read one raw 16-bit ADCJ result from the encoded payload.
 *
 * memcpy() is used instead of a uint16_t pointer dereference because
 * the payload might not be naturally aligned for a 16-bit access.
 *
 * The CPU-backed stream producer also stores the sample with memcpy(),
 * while the DMA path writes the native 16-bit representation directly.
 */
static int adc_adcj_decoder_get_raw_sample(const uint8_t *buffer,
					   uint8_t sample_index,
					   uint16_t *raw)
{
	const struct adc_adcj_stream_header *header;
	const uint8_t *sample_payload;
	int err;

	if (raw == NULL) {
		return -EINVAL;
	}

	err = adc_adcj_decoder_validate_frame(buffer);
	if (err != 0) {
		return err;
	}

	header = adc_adcj_decoder_header(buffer);

	if (sample_index >= header->num_channels) {
		return -EINVAL;
	}

	sample_payload =
		adc_adcj_decoder_sample_payload(
			buffer,
			header->num_channels);

	memcpy(raw,
		   sample_payload +
			   ((size_t)sample_index * sizeof(*raw)),
		   sizeof(*raw));

	return 0;
}

/*
 * Convert a native ADCJ raw result into a q31_t voltage representation.
 *
 * The resulting value and shift follow the same voltage-oriented
 * convention used by the STM32 streaming decoder.
 */
static int adc_adcj_decoder_raw_to_voltage_q31(
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

static int adc_adcj_decoder_get_frame_count(const uint8_t *buffer,
	uint32_t channel, uint16_t *frame_count)
{
	int err;
	const struct adc_adcj_stream_header *header;
	const struct adc_adcj_stream_channel *channels;
	bool found = false;

	if ((frame_count == NULL) || (buffer == NULL)) {
		return -EINVAL;
	}

	if (channel >= ADC_RENESAS_ADCJ_MAX_ZEPHYR_CHANNELS) {
		return -ENOTSUP;
	}

	err = adc_adcj_decoder_validate_frame(buffer);
	if (err != 0) {
		return err;
	}

	header = adc_adcj_decoder_header(buffer);
	channels = adc_adcj_decoder_channels(buffer);

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

static int adc_adcj_decoder_get_size_info(struct adc_dt_spec adc_spec,
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

	if (channel >= ADC_RENESAS_ADCJ_MAX_ZEPHYR_CHANNELS) {
		return -ENOTSUP;
	}

	/*
	 * The first decoded frame requires the adc_data base object.
	 * Each additional decoded frame requires one adc_sample_data.
	 *
	 * The current encoded ADCJ buffer contains only one frame, but
	 * these values remain compatible with the generic decoder API.
	 */
	*base_size = sizeof(struct adc_data);
	*frame_size = sizeof(struct adc_sample_data);

	return 0;
}

static int adc_adcj_decoder_decode(const uint8_t *buffer,
				   uint32_t channel,
				   uint32_t *fit,
				   uint16_t max_count,
				   void *data_out)
{
	const struct adc_adcj_stream_header *header;
	const struct adc_adcj_stream_channel *channels;
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
	 * The current ADCJ frame contains one scan only.
	 *
	 * fit == 0 means the frame has not been decoded yet.
	 * fit != 0 means there are no additional frames in this buffer.
	 */
	if (*fit != 0U) {
		return 0;
	}

	err = adc_adcj_decoder_validate_frame(buffer);
	if (err != 0) {
		return err;
	}

	err = adc_adcj_decoder_find_channel(buffer, channel, &sample_index);
	if (err != 0) {
		return err;
	}

	err = adc_adcj_decoder_get_raw_sample(
		buffer,
		sample_index,
		&raw);
	if (err != 0) {
		return err;
	}

	header = adc_adcj_decoder_header(buffer);
	channels = adc_adcj_decoder_channels(buffer);

	/*
	 * Validate that the resolution stored in the channel descriptor
	 * agrees with the selected ADCJ format.
	 */
	switch ((enum adc_adcj_data_format)header->data_format) {
	case ADC_ADCJ_DATA_FORMAT_12_BIT_SIGNED_FIXED:
	case ADC_ADCJ_DATA_FORMAT_12_BIT_SIGNED_INTEGER:
	case ADC_ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_FIXED:
	case ADC_ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_RIGHT:
	case ADC_ADCJ_DATA_FORMAT_12_BIT_UNSIGNED_LEFT:
		if (channels[sample_index].resolution != 12U) {
			return -EINVAL;
		}
		break;

	default:
		return -ENOTSUP;
	}

	wiring_break_addition =
	(header->flags & ADC_ADCJ_STREAM_FLAG_WIRING_BREAK_ADDITION) != 0U;

	err = adc_adcj_raw_to_count_from_metadata(
		(enum adc_adcj_data_format)header->data_format,
		channels[sample_index].conversion_type,
		header->addition_count,
		wiring_break_addition,
		raw,
		&count);
	if (err != 0) {
		return err;
	}

	err = adc_adcj_decoder_raw_to_voltage_q31(
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

static bool adc_adcj_decoder_has_trigger(const uint8_t *buffer,
					 enum adc_trigger_type trigger)
{
	if (adc_adcj_decoder_validate_frame(buffer) != 0) {
		return false;
	}

	/*
	 * One encoded frame is generated after one completed ADCJ scan.
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
	.get_frame_count = adc_adcj_decoder_get_frame_count,
	.get_size_info = adc_adcj_decoder_get_size_info,
	.decode = adc_adcj_decoder_decode,
	.has_trigger = adc_adcj_decoder_has_trigger,
};

static int adc_adcj_get_decoder(
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
	struct adc_adcj_data *data = CONTAINER_OF(ctx, struct adc_adcj_data, ctx);
	const struct device *dev = data->dev;
	const struct adc_adcj_config *config = dev->config;
	fsp_err_t fsp_err;

	/*
	 * Finite ownership must have been claimed by
	 * adc_adcj_start_read() before the ADC context was started.
	 */
	if (data->request_mode != ADC_ADCJ_REQUEST_FINITE) {
		data->status = -EBUSY;
		adc_context_complete(ctx, -EBUSY);
		return;
	}

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

	/*
	 * The ADC context may only release hardware owned by a finite
	 * request. Never stop hardware owned by an active stream.
	 */
	if (data->request_mode != ADC_ADCJ_REQUEST_FINITE) {
		return;
	}

	data->active = false;
	fsp_err = R_ADCJ_ScanGroupStop(&data->fsp_ctrl,
			config->scan_group_mask);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("ADCJ cleanup failed: %d", fsp_err);
	}
	data->request_mode = ADC_ADCJ_REQUEST_IDLE;
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
	int err;

	data->dev = dev;
	data->active = false;
	data->status = 0;
	data->request_mode = ADC_ADCJ_REQUEST_IDLE;
#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM)
	data->stream_sqe = NULL;
	data->stream_config = NULL;
	data->stream_buffer = NULL;
	data->stream_buffer_len = 0U;
	data->stream_channels = 0U;
	data->stream_frame_size = 0U;
	data->stream_sequence_number = 0U;
#endif

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("ADCJ clock device is not ready");
		return -ENODEV;
	}

	err = clock_control_on(config->clock_dev, config->clock_subsys);
	if (err < 0) {
		LOG_ERR("ADCJ module clock enable failed (err %d)", err);
		return err;
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

#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA)

	data->dma_active = false;

	fsp_err = R_ADCJ_InfoGet(
			&data->fsp_ctrl,
			&data->adc_info);

	if (fsp_err != FSP_SUCCESS) {
		return adc_adcj_fsp_err_to_errno(fsp_err);
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
}

#define ADCJ_SCAN_MODE_multicycle  ADCJ_SG_SCAN_MODE_MULTICYLE
#define ADCJ_SCAN_MODE_continuous  ADCJ_SG_SCAN_MODE_CONTINUOUS
#define ADCJ_GROUP_MASK_0 ADC_GROUP_MASK_0
#define ADCJ_GROUP_MASK_1 ADC_GROUP_MASK_1
#define ADCJ_GROUP_MASK_2 ADC_GROUP_MASK_2
#define ADCJ_GROUP_MASK_3 ADC_GROUP_MASK_3
#define ADCJ_GROUP_MASK_4 ADC_GROUP_MASK_4
#define ADCJ_GROUP_MASK_FROM_GROUP(idx) \
	UTIL_CAT(ADCJ_GROUP_MASK_, DT_INST_PROP(idx, renesas_scan_group))
#if defined(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA)

#define ADCJ_DMA_DEV(idx)                                                                          \
	DEVICE_DT_GET(DT_DMAS_CTLR(DT_DRV_INST(idx)))

#define ADCJ_DMA_CHANNEL(idx)                                                                      \
	DT_DMAS_CELL_BY_IDX(DT_DRV_INST(idx), 0, channel)

#define ADCJ_DMA_CONFIG(idx)                                                                       \
	DT_DMAS_CELL_BY_IDX(DT_DRV_INST(idx), 0, config)

#define ADCJ_DMA_GROUP(idx)                                                                       \
	DT_DMAS_CELL_BY_IDX(DT_DRV_INST(idx), 0, group)

#endif

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
#define ADCJ_SCAN_MODE(idx)                                              \
	UTIL_CAT(ADCJ_SCAN_MODE_, DT_INST_STRING_TOKEN(idx, renesas_scan_mode))

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
	IF_ENABLED(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM_DMA, \
		(BUILD_ASSERT( \
			DT_INST_NODE_HAS_PROP(idx, dmas), \
			"ADCJ DMA mode requires a dmas property");)) \
	BUILD_ASSERT(!IS_ENABLED(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM) || \
		!DT_INST_ENUM_HAS_VALUE(idx, renesas_scan_mode, multicycle), \
			"ADCJ streaming does not support multicycle scan mode"); \
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
			.sgcr_b.scan_mode = ADCJ_SCAN_MODE(idx), \
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
		.vref_mv = DT_INST_PROP_OR(idx, vref_mv, 0),					\
		.track_hold_mask = ADCJ_TRACK_HOLD_MASK(idx), \
		.software_trigger = ADCJ_TRIGGER_IS_SOFTWARE_STARTED(idx), \
		.uses_th = ADCJ_INSTANCE_USES_TH(idx), \
		.scan_group_mask = ADCJ_GROUP_MASK_FROM_GROUP(idx), \
		.fsp_cfg = &adc_adcj_fsp_cfg_##idx, \
		.fsp_sg_cfg = &adc_adcj_sg_cfg_##idx, \
		.channel_to_vc = adc_adcj_channel_to_vc_##idx, \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_DRV_INST(idx))),			\
		.clock_subsys = (clock_control_subsys_t)(uintptr_t)				\
		DT_CLOCKS_CELL(DT_DRV_INST(idx), clk_id),				\
		.irq_config_func = adc_adcj_irq_config_##idx, \
	}; \
	static struct adc_adcj_data adc_adcj_data_##idx = { \
		ADC_CONTEXT_INIT_TIMER(adc_adcj_data_##idx, ctx), \
		ADC_CONTEXT_INIT_LOCK(adc_adcj_data_##idx, ctx), \
		ADC_CONTEXT_INIT_SYNC(adc_adcj_data_##idx, ctx), \
		.dev = DEVICE_DT_INST_GET(idx), \
		ADCJ_DMA_INIT(idx) \
	}; \
	static DEVICE_API(adc, adc_adcj_api_##idx) = { \
		.channel_setup = adc_adcj_channel_setup, \
		.read = adc_adcj_read, \
		.ref_internal = DT_INST_PROP_OR(idx, vref_mv, 0), \
		IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = adc_adcj_read_async,)) \
		IF_ENABLED(CONFIG_ADC_RENESAS_RH850_ADCJ_STREAM,			\
			(.submit = adc_adcj_submit,						\
			.get_decoder = adc_adcj_get_decoder,))				  \
	}; \
	DEVICE_DT_INST_DEFINE(idx, adc_adcj_init, NULL, &adc_adcj_data_##idx, \
		&adc_adcj_config_##idx, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, \
		&adc_adcj_api_##idx);

DT_INST_FOREACH_STATUS_OKAY(ADCJ_DEVICE_INIT)
