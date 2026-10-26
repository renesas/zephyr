/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_sent

#include <soc.h>
#include <errno.h>
#include <string.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sent/sent.h>
#include <zephyr/dt-bindings/sent/sent.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <r_dmac.h>
#include <r_rsent_sent.h>
#include <r_transfer_api.h>

LOG_MODULE_REGISTER(sent_renesas_rh850, CONFIG_SENT_LOG_LEVEL);

#define RSENT_MAX_RX_FRAMES 64U

/*
 * Runtime states of one Zephyr RSENT device.
 *
 * Each Zephyr device represents one RSENT hardware instance.
 * Each RSENT instance currently exposes one hardware channel, channel 0.
 */
enum sent_renesas_channel_state {
	SENT_RENESAS_CHANNEL_STOPPED,
	SENT_RENESAS_CHANNEL_STARTING,
	SENT_RENESAS_CHANNEL_STARTED,
	SENT_RENESAS_CHANNEL_STOPPING,
};

/*
 * Context passed through the FSP callback interface.
 *
 * FSP only returns p_context in sent_callback_args_t, so the corresponding
 * Zephyr device is stored here and recovered in sent_renesas_fsp_callback().
 */
struct sent_renesas_callback_context {
	const struct device *dev;
};

/*
 * Static configuration for one Zephyr RSENT device.
 *
 * This structure contains the immutable configuration generated from
 * Devicetree. Each Zephyr device represents one RSENT hardware instance,
 * whose channel configuration is provided by its enabled channel child.
 */
struct sent_renesas_config {
	/* Pinctrl configuration for the RSENT input pin. */
	const struct pinctrl_dev_config *pincfg;

	/* Clock controller and subsystem for this RSENT instance. */
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;

	/* Requested SENT tick length from Devicetree, in microseconds. */
	uint32_t clock_tick_length_us;

	/* Mutable FSP extended configuration populated from the clock rate. */
	rsent_sent_extended_cfg_t *fsp_extended_cfg;

	/* RSENT channel configured by the enabled child node. */
	uint8_t channel;

	/* Number of data nibbles contained in each fast message. */
	uint8_t num_data_nibbles;

	/* Configured slow/serial message format. */
	sent_serial_message_format_t serial_message_format;

	/* Fast-message CRC configuration from Devicetree. */
	uint32_t fast_crc;
	/* Short-serial-message CRC configuration from Devicetree. */
	uint32_t short_serial_crc;

	/* Static FSP configuration used to open the RSENT channel. */
	const sent_cfg_t *fsp_cfg;

	/* Optional RX DMA controller and channel used for ownership tracking. */
	const struct device *dma_dev;
	uint8_t dma_channel;
};

/*
 * Runtime data for one Zephyr RSENT device.
 *
 * This structure contains the mutable state associated with one RSENT
 * hardware channel, including the FSP control block, callback configuration,
 * receive buffers, request state, statistics, and synchronization state.
 */
struct sent_renesas_data {
	/* Current Zephyr driver state for this channel. */
	enum sent_renesas_channel_state state;

	/* FSP runtime control block used by the R_RSENT_SENT_* APIs. */
	rsent_sent_instance_ctrl_t fsp_ctrl;

	/* True after the optional DMA channel has been claimed. */
	bool dma_channel_claimed;

	/*
	 * Context used by the FSP callback to recover the corresponding
	 * Zephyr device.
	 */
	struct sent_renesas_callback_context callback_context;

	/* Working memory used by FSP when invoking its callback. */
	sent_callback_args_t callback_memory;

	/* Application callback configuration for fast messages. */
	struct sent_rx_callback_config fast_callback;

	/* Application callback configuration for serial messages. */
	struct sent_rx_callback_config serial_callback;

	/* Whether a valid fast-message callback is registered. */
	bool fast_callback_registered;

	/* Whether a valid serial-message callback is registered. */
	bool serial_callback_registered;

	/* Whether an FSP fast-message read request is currently active. */
	bool fast_read_active;

	/* Whether an FSP slow-message read request is currently active. */
	bool serial_read_active;

	/* Number of fast messages requested in the active receive batch. */
	uint32_t fast_request_count;

	/* Number of serial messages requested in the active receive batch. */
	uint32_t serial_request_count;

	/* Internal FSP receive buffer for fast messages. */
	rsent_sent_fast_msg_t fast_messages[RSENT_MAX_RX_FRAMES];

	/* Internal FSP receive buffer for slow/serial messages. */
	rsent_sent_slow_msg_t slow_messages[RSENT_MAX_RX_FRAMES];

	/* Total number of fast frames delivered to the application. */
	uint32_t fast_frames_received;

	/* Total number of serial frames delivered to the application. */
	uint32_t serial_frames_received;

	/* Fast-channel CRC error counter. */
	uint32_t fast_crc_errors;

	/* Slow-channel CRC error counter. */
	uint32_t slow_crc_errors;

	/* Fast-channel message-lost error counter. */
	uint32_t fast_message_lost_errors;

	/* Slow-channel message-lost error counter. */
	uint32_t slow_message_lost_errors;

	/* Fast-channel nibble encoding error counter. */
	uint32_t fast_encoding_errors;

	/* Slow-channel encoding error counter. */
	uint32_t slow_encoding_errors;

	/* Fast-channel invalid nibble-count error counter. */
	uint32_t fast_nibble_count_errors;

	/* Calibration pulse length error counter. */
	uint32_t calibration_length_errors;

	/* Calibration pulse variation error counter. */
	uint32_t calibration_variation_errors;

	/* SPC no-response error counter. */
	uint32_t no_response_errors;

	/* Counter for unexpected or unclassified hardware events. */
	uint32_t hardware_errors;

	/*
	 * Protects runtime state and callback configuration shared between
	 * application context and FSP callback or interrupt context.
	 */
	struct k_spinlock lock;
};

/*
 * FSP interrupt handlers.
 *
 * These recover the FSP control block using R_FSP_CurrentIrqGet() and
 * R_FSP_IsrContextGet(). R_RSENT_SENT_Open() registers that context.
 */
BSP_INTERRUPT_ATTRIBUTE void rsent_sent_rxi_isr(void);
BSP_INTERRUPT_ATTRIBUTE void rsent_sent_eri_isr(void);

/*
 * Zephyr ISR adapters.
 *
 * Unlike the RLIN UART flag-based receive path, SENT must execute the FSP
 * ISR to read messages, acknowledge hardware status, and issue callbacks.
 */
static void sent_renesas_rxi_isr(const void *arg)
{
	ARG_UNUSED(arg);

	rsent_sent_rxi_isr();
}

static void sent_renesas_eri_isr(const void *arg)
{
	ARG_UNUSED(arg);

	rsent_sent_eri_isr();
}

/*
 * Mask both interrupt-controller lines.
 *
 * Peripheral interrupt sources remain managed by the FSP Start/Stop APIs.
 */
static void sent_renesas_irq_disable(const struct device *dev)
{
	const struct sent_renesas_config *config = dev->config;
	const sent_cfg_t *fsp_cfg = config->fsp_cfg;

	irq_disable((unsigned int)fsp_cfg->rxi_irq);
	irq_disable((unsigned int)fsp_cfg->eri_irq);

	if (fsp_cfg->p_transfer_rx != NULL) {
		const dmac_extended_cfg_t *dma_extend =
			(const dmac_extended_cfg_t *)
			fsp_cfg->p_transfer_rx->p_cfg->p_extend;

		irq_disable((unsigned int)dma_extend->irq);
	}
}

/*
 * Unmask interrupt-controller lines after the driver state and receive
 * requests are ready.
 *
 * FSP disables CPU RXI when a transfer instance owns reception, so do not
 * override that behavior.
 */
static void sent_renesas_irq_enable(const struct device *dev)
{
	const struct sent_renesas_config *config = dev->config;
	const sent_cfg_t *fsp_cfg = config->fsp_cfg;

	irq_enable((unsigned int)fsp_cfg->eri_irq);

	if (fsp_cfg->p_transfer_rx == NULL) {
		irq_enable((unsigned int)fsp_cfg->rxi_irq);
	} else {
		const dmac_extended_cfg_t *dma_extend =
			(const dmac_extended_cfg_t *)
			fsp_cfg->p_transfer_rx->p_cfg->p_extend;

		irq_enable((unsigned int)dma_extend->irq);
	}
}

/*
 * Validate that the requested channel belongs to the supplied device.
 *
 * This driver uses a one-device-per-channel model. Therefore, the channel
 * argument from the generic Zephyr SENT API must match the channel assigned
 * to this device through Devicetree.
 */
static int sent_renesas_channel_validate(const struct device *dev,
					 uint8_t channel)
{
	const struct sent_renesas_config *config = dev->config;

	if (channel != config->channel) {
		return -EINVAL;
	}

	return 0;
}

/*
 * Convert an FSP error code to the closest equivalent Zephyr errno.
 */
static int sent_renesas_error_convert(fsp_err_t error)
{
	switch (error) {
	case FSP_SUCCESS:
		return 0;

	case FSP_ERR_ASSERTION:
	case FSP_ERR_INVALID_ARGUMENT:
	case FSP_ERR_INVALID_SIZE:
		return -EINVAL;

	case FSP_ERR_ALREADY_OPEN:
		return -EALREADY;

	case FSP_ERR_NOT_OPEN:
		return -ENODEV;

	case FSP_ERR_IN_USE:
		return -EBUSY;

	case FSP_ERR_TIMEOUT:
		return -ETIMEDOUT;

	case FSP_ERR_UNSUPPORTED:
		return -ENOTSUP;

	default:
		return -EIO;
	}
}

/*
 * Validate an application receive callback configuration.
 *
 * A valid receive callback requires:
 *
 * - A callback function.
 * - A destination frame buffer.
 * - A non-zero maximum frame count.
 */
static bool sent_renesas_callback_config_valid(
	const struct sent_rx_callback_config *callback_config)
{
	return callback_config != NULL &&
				callback_config->callback != NULL &&
				callback_config->frame != NULL &&
				callback_config->max_num_frame > 0U;
}

/*
 * Determine the number of frames to request from FSP.
 *
 * The application-requested count is limited to the capacity of the
 * driver's internal receive buffer.
 */
static uint32_t sent_renesas_requested_frame_count(
	const struct sent_rx_callback_config *callback)
{
	return MIN(callback->max_num_frame,
				(uint32_t)RSENT_MAX_RX_FRAMES);
}

/*
 * Submit a fast-message receive request to FSP.
 *
 * The function prepares the internal receive buffer and requests up to
 * RSENT_MAX_RX_FRAMES fast messages. Completion is reported asynchronously
 * through sent_renesas_fsp_callback().
 *
 * Returning zero means that the request was successfully submitted, or that
 * no fast-message callback is registered. It does not mean that a frame has
 * already been received.
 */
static int sent_renesas_fast_read_arm(const struct device *dev)
{
	struct sent_renesas_data *data = dev->data;
	uint32_t message_count;
	fsp_err_t fsp_ret;

	/*
	 * No fast callback means that the application is not interested in
	 * fast messages. This is a valid configuration, not an error.
	 */
	if (!data->fast_callback_registered) {
		return 0;
	}

	/*
	 * Limit the FSP request to the capacity of the internal receive
	 * buffer.
	 */
	message_count =
		sent_renesas_requested_frame_count(&data->fast_callback);

	if (message_count == 0U) {
		return -EINVAL;
	}

	/*
	 * Remove data left by the previous receive batch. This also gives
	 * fields not populated by every FSP receive path a defined value.
	 */
	memset(data->fast_messages, 0, sizeof(data->fast_messages));

	/*
	 * Register the internal receive buffer and requested count with FSP.
	 * This call is asynchronous and returns before messages are received.
	 */
	fsp_ret = R_RSENT_SENT_FastMsgRead(
		(sent_ctrl_t *)&data->fsp_ctrl,
		data->fast_messages,
		message_count);

	if (fsp_ret != FSP_SUCCESS) {
		return sent_renesas_error_convert(fsp_ret);
	}

	/* Save information describing the currently active request. */
	data->fast_request_count = message_count;
	data->fast_read_active = true;

	return 0;
}

/*
 * Submit a slow/serial-message receive request to FSP.
 *
 * The request is submitted only when a serial callback is registered and
 * serial-message extraction is enabled for this channel. Completion is
 * reported asynchronously through sent_renesas_fsp_callback().
 */
static int sent_renesas_serial_read_arm(const struct device *dev)
{
	const struct sent_renesas_config *config = dev->config;
	struct sent_renesas_data *data = dev->data;
	uint32_t message_count;
	fsp_err_t fsp_ret;

	if (!data->serial_callback_registered) {
		return 0;
	}

	if (config->serial_message_format ==
		SENT_SERIAL_MESSAGE_FORMAT_NO_SERIAL_MESSAGE) {
		return -ENOTSUP;
	}

	message_count =
		sent_renesas_requested_frame_count(&data->serial_callback);

	if (message_count == 0U) {
		return -EINVAL;
	}

	memset(data->slow_messages, 0, sizeof(data->slow_messages));

	fsp_ret = R_RSENT_SENT_SlowMsgRead(
		(sent_ctrl_t *)&data->fsp_ctrl,
		data->slow_messages,
		message_count);

	if (fsp_ret != FSP_SUCCESS) {
		return sent_renesas_error_convert(fsp_ret);
	}

	data->serial_request_count = message_count;
	data->serial_read_active = true;

	return 0;
}

/*
 * Abort the currently active fast-message read request.
 *
 * This function is used when callbacks are replaced, reception is stopped,
 * or an error requires the receive path to be restarted.
 */
static void sent_renesas_fast_read_abort(
	struct sent_renesas_data *data)
{
	fsp_err_t fsp_ret;

	if (!data->fast_read_active) {
		return;
	}

	fsp_ret = R_RSENT_SENT_Abort(
		(sent_ctrl_t *)&data->fsp_ctrl,
		SENT_COMMUNICATION_FAST_MESSAGE_READ);

	/*
	 * FSP_ERR_SENT_ABORT_FAILED is acceptable because the request may
	 * already have completed before the abort was processed.
	 */
	if ((fsp_ret != FSP_SUCCESS) &&
		(fsp_ret != FSP_ERR_SENT_ABORT_FAILED)) {
		LOG_WRN("Failed to abort fast read: %d",
			(int)fsp_ret);
	}

	data->fast_read_active = false;
	data->fast_request_count = 0U;
}

/*
 * Abort the currently active slow/serial-message read request.
 */
static void sent_renesas_serial_read_abort(
	struct sent_renesas_data *data)
{
	fsp_err_t fsp_ret;

	if (!data->serial_read_active) {
		return;
	}

	fsp_ret = R_RSENT_SENT_Abort(
		(sent_ctrl_t *)&data->fsp_ctrl,
		SENT_COMMUNICATION_SLOW_MESSAGE_READ);

	if ((fsp_ret != FSP_SUCCESS) &&
		(fsp_ret != FSP_ERR_SENT_ABORT_FAILED)) {
		LOG_WRN("Failed to abort serial read: %d",
			(int)fsp_ret);
	}

	data->serial_read_active = false;
	data->serial_request_count = 0U;
}

/*
 * Register, replace, or unregister application receive callbacks.
 *
 * A non-NULL fast or serial callback configuration registers or replaces
 * the corresponding callback. A NULL configuration unregisters that
 * callback.
 *
 * Active FSP receive requests are aborted before callback configuration or
 * application frame buffers are replaced. If the channel is already
 * started, receive requests are submitted again using the updated callback
 * configurations.
 */
static int sent_renesas_register_callback(
	const struct device *dev,
	uint8_t channel,
	struct sent_rx_callback_configs callback_configs)
{
	const struct sent_renesas_config *config = dev->config;
	struct sent_renesas_data *data = dev->data;
	struct sent_rx_callback_config fast_callback;
	struct sent_rx_callback_config serial_callback;
	k_spinlock_key_t key;
	bool replace_fast;
	bool replace_serial;
	bool rearm_fast;
	bool rearm_serial;
	int ret;

	ret = sent_renesas_channel_validate(dev, channel);
	if (ret != 0) {
		return ret;
	}

	if ((callback_configs.fast != NULL) &&
		!sent_renesas_callback_config_valid(callback_configs.fast)) {
		return -EINVAL;
	}

	if ((callback_configs.serial != NULL) &&
		!sent_renesas_callback_config_valid(callback_configs.serial)) {
		return -EINVAL;
	}

	if ((callback_configs.serial != NULL) &&
		(config->serial_message_format ==
		 SENT_SERIAL_MESSAGE_FORMAT_NO_SERIAL_MESSAGE)) {
		return -ENOTSUP;
	}

	/*
	 * A non-NULL callback configuration means register or replace the
	 * corresponding callback. NULL means unregister it.
	 */
	replace_fast = callback_configs.fast != NULL;
	replace_serial = callback_configs.serial != NULL;

	/*
	 * Copy application callback configurations. The driver does not retain
	 * the configuration structure pointers supplied by the application.
	 */
	if (replace_fast) {
		fast_callback = *callback_configs.fast;
	}

	if (replace_serial) {
		serial_callback = *callback_configs.serial;
	}

	/*
	 * Abort requests that may still reference the previous application
	 * callback configuration or frame buffer.
	 */
	sent_renesas_fast_read_abort(data);
	sent_renesas_serial_read_abort(data);

	key = k_spin_lock(&data->lock);

	/* Register or replace the fast-message callback. */
	if (replace_fast) {
		data->fast_callback = fast_callback;
		data->fast_callback_registered = true;
	} else {
		/* NULL unregisters the current fast-message callback. */
		memset(&data->fast_callback, 0,
					sizeof(data->fast_callback));
		data->fast_callback_registered = false;
	}

	/* Register or replace the serial-message callback. */
	if (replace_serial) {
		data->serial_callback = serial_callback;
		data->serial_callback_registered = true;
	} else {
		/* NULL unregisters the current serial-message callback. */
		memset(&data->serial_callback, 0,
					sizeof(data->serial_callback));
		data->serial_callback_registered = false;
	}

	/*
	 * If reception is already running, a new FSP request must be
	 * submitted using the updated callback configuration.
	 */
	rearm_fast =
		data->state == SENT_RENESAS_CHANNEL_STARTED &&
		data->fast_callback_registered;

	rearm_serial =
		data->state == SENT_RENESAS_CHANNEL_STARTED &&
		data->serial_callback_registered;

	k_spin_unlock(&data->lock, key);

	if (rearm_fast) {
		ret = sent_renesas_fast_read_arm(dev);
		if (ret != 0) {
			return ret;
		}
	}

	if (rearm_serial) {
		ret = sent_renesas_serial_read_arm(dev);
		if (ret != 0) {
			sent_renesas_fast_read_abort(data);
			return ret;
		}
	}

	return 0;
}

/*
 * Start receiving SENT messages on the channel represented by dev.
 *
 * Receive buffers are submitted to FSP before the hardware is placed in
 * ACTIVE mode, ensuring that incoming messages always have a valid
 * destination buffer.
 */
static int sent_renesas_start_listening(const struct device *dev,
					uint8_t channel)
{
	struct sent_renesas_data *data = dev->data;
	k_spinlock_key_t key;
	fsp_err_t fsp_ret;
	int ret;

	ret = sent_renesas_channel_validate(dev, channel);
	if (ret != 0) {
		return ret;
	}

	key = k_spin_lock(&data->lock);

	if (data->state != SENT_RENESAS_CHANNEL_STOPPED) {
		k_spin_unlock(&data->lock, key);
		return -EALREADY;
	}

	/* Reserve the start transition for the current caller. */
	data->state = SENT_RENESAS_CHANNEL_STARTING;

	k_spin_unlock(&data->lock, key);

	/*
	 * Submit receive requests before placing RSENT in ACTIVE mode.
	 */
	ret = sent_renesas_fast_read_arm(dev);
	if (ret != 0) {
		goto start_failed;
	}

	ret = sent_renesas_serial_read_arm(dev);
	if (ret != 0) {
		sent_renesas_fast_read_abort(data);
		goto start_failed;
	}

	fsp_ret = R_RSENT_SENT_Start(
		(sent_ctrl_t *)&data->fsp_ctrl);

	if (fsp_ret != FSP_SUCCESS) {
		ret = sent_renesas_error_convert(fsp_ret);

		sent_renesas_fast_read_abort(data);
		sent_renesas_serial_read_abort(data);

		goto start_failed;
	}

	key = k_spin_lock(&data->lock);

	/*
	 * Start() has enabled peripheral interrupt sources, but the
	 * interrupt-controller lines are still masked.
	 *
	 * Publish STARTED before allowing completion callbacks; otherwise
	 * an early completion could see STARTING and fail to rearm.
	 */
	data->state = SENT_RENESAS_CHANNEL_STARTED;
	sent_renesas_irq_enable(dev);

	k_spin_unlock(&data->lock, key);
	return 0;

start_failed:
	key = k_spin_lock(&data->lock);

	sent_renesas_irq_disable(dev);
	data->state = SENT_RENESAS_CHANNEL_STOPPED;

	k_spin_unlock(&data->lock, key);
	return ret;
}

/*
 * Stop receiving messages on the channel represented by dev.
 *
 * The hardware is first placed in IDLE mode. Any active fast and serial
 * read requests are then aborted and the driver state is returned to
 * STOPPED.
 */
static int sent_renesas_stop_listening(const struct device *dev,
							uint8_t channel)
{
	struct sent_renesas_data *data = dev->data;
	k_spinlock_key_t key;
	fsp_err_t fsp_ret;
	int ret;

	ret = sent_renesas_channel_validate(dev, channel);
	if (ret != 0) {
		return ret;
	}

	key = k_spin_lock(&data->lock);

	if (data->state == SENT_RENESAS_CHANNEL_STOPPED) {
		k_spin_unlock(&data->lock, key);
		return -EALREADY;
	}

	if (data->state != SENT_RENESAS_CHANNEL_STARTED) {
		k_spin_unlock(&data->lock, key);
		return -EBUSY;
	}

	data->state = SENT_RENESAS_CHANNEL_STOPPING;

	/*
	 * Prevent RXI/ERI from racing with Stop() and receive-request
	 * cleanup on the IRQ-owning CPU.
	 */
	sent_renesas_irq_disable(dev);

	k_spin_unlock(&data->lock, key);
	fsp_ret = R_RSENT_SENT_Stop(
		(sent_ctrl_t *)&data->fsp_ctrl);

	if (fsp_ret != FSP_SUCCESS) {
		key = k_spin_lock(&data->lock);

		data->state = SENT_RENESAS_CHANNEL_STARTED;
		sent_renesas_irq_enable(dev);

		k_spin_unlock(&data->lock, key);

		return sent_renesas_error_convert(fsp_ret);
	}

	sent_renesas_fast_read_abort(data);
	sent_renesas_serial_read_abort(data);

	key = k_spin_lock(&data->lock);
	data->state = SENT_RENESAS_CHANNEL_STOPPED;
	k_spin_unlock(&data->lock, key);

	return 0;
}

/*
 * Convert one FSP fast-message representation to the generic Zephyr
 * struct sent_frame format.
 *
 * FSP stores the fast-message data nibbles in one packed integer. This
 * function separates the packed nibbles into the generic data_nibbles
 * array.
 */
static void sent_renesas_unpack_fast_frame(
	const struct sent_renesas_config *config,
	const rsent_sent_fast_msg_t *source,
	struct sent_frame *destination)
{
	uint8_t num_data_nibbles;

	memset(destination, 0, sizeof(*destination));

	destination->type = SENT_FAST_FRAME;

#if RSENT_SENT_CFG_FAST_MSG_TIMESTAMP
	destination->timestamp = source->timestamp;
#endif

	num_data_nibbles =
		MIN(config->num_data_nibbles,
			SENT_MAX_DATA_NIBBLES);

	for (uint8_t i = 0U; i < num_data_nibbles; i++) {
		uint8_t shift =
			(uint8_t)(
				(num_data_nibbles - 1U - i) * 4U);

		destination->fast.data_nibbles[i] =
			(uint8_t)(
				(source->data >> shift) & 0x0FU);
	}

	/*
	 * The current FSP fast-message structure does not expose the
	 * received CRC nibble through the generic Zephyr API.
	 */
	destination->crc = 0U;
}

/*
 * Convert one FSP slow-message representation to the generic Zephyr
 * struct sent_frame format.
 *
 * Returns true if the configured serial format is supported and the
 * destination frame was populated. Returns false for a disabled or
 * unsupported serial-message format.
 */
static bool sent_renesas_unpack_serial_frame(
	const struct sent_renesas_config *config,
	const rsent_sent_slow_msg_t *source,
	struct sent_frame *destination)
{
	memset(destination, 0, sizeof(*destination));

	switch (config->serial_message_format) {
	case SENT_SERIAL_MESSAGE_FORMAT_SHORT_SERIAL_MESSAGE:
		destination->type = SENT_SHORT_SERIAL_FRAME;
		break;

	case SENT_SERIAL_MESSAGE_FORMAT_ENHANCED_SERIAL_MESSAGE:
		if (source->cfg_bit == 0U) {
			destination->type =
				SENT_ENHANCED_SERIAL_FRAME_8_BIT_ID;
		} else {
			destination->type =
				SENT_ENHANCED_SERIAL_FRAME_4_BIT_ID;
		}
		break;

	default:
		return false;
	}

	destination->serial.id = source->msg_id;
	destination->serial.data = source->data;

#if RSENT_SENT_CFG_SLOW_MSG_TIMESTAMP
	destination->timestamp = source->timestamp;
#endif

	/*
	 * The current FSP slow-message structure does not expose the
	 * received CRC. Keep the generic CRC field at zero.
	 */
	destination->crc = 0U;

	return true;
}

/*
 * Handle completion of an FSP fast-message receive request.
 *
 * The received FSP messages are converted into the application-provided
 * Zephyr frame buffer. The application callback is then invoked, and a new
 * request is submitted if the device is still started and the callback has
 * not been replaced.
 *
 * This function may execute in FSP callback or interrupt context.
 */
static void sent_renesas_fast_complete(const struct device *dev)
{
	const struct sent_renesas_config *config = dev->config;
	struct sent_renesas_data *data = dev->data;
	struct sent_rx_callback_config callback;
	k_spinlock_key_t key;
	uint32_t count;
	bool rearm;

	key = k_spin_lock(&data->lock);

	data->fast_read_active = false;

	if (!data->fast_callback_registered) {
		data->fast_request_count = 0U;
		k_spin_unlock(&data->lock, key);
		return;
	}

	/*
	 * Copy the callback configuration while holding the lock. The local
	 * copy remains stable if the registered callback changes later.
	 */
	callback = data->fast_callback;

	count = MIN(data->fast_request_count,
			callback.max_num_frame);

	data->fast_request_count = 0U;

	k_spin_unlock(&data->lock, key);

	for (uint32_t i = 0U; i < count; i++) {
		sent_renesas_unpack_fast_frame(
			config,
			&data->fast_messages[i],
			&callback.frame[i]);
	}

	/*
	 * Invoke application code without holding the internal driver lock.
	 */
	if (count > 0U) {
		callback.callback(dev,
				  config->channel,
				  count,
				  callback.user_data);
	}

	key = k_spin_lock(&data->lock);

	/*
	 * Rearm only when the channel remains started and the application did
	 * not replace or unregister the callback while it was being invoked.
	 */
	rearm =
		data->state == SENT_RENESAS_CHANNEL_STARTED &&
		data->fast_callback_registered &&
		data->fast_callback.callback == callback.callback &&
		data->fast_callback.frame == callback.frame;

	data->fast_frames_received += count;

	k_spin_unlock(&data->lock, key);

	if (rearm) {
		int ret = sent_renesas_fast_read_arm(dev);

		if (ret != 0) {
			LOG_ERR("Failed to rearm fast RX channel %u: %d",
				config->channel, ret);
		}
	}
}

/*
 * Handle completion of an FSP slow-message receive request.
 *
 * Valid FSP slow messages are converted into Zephyr serial frames. The
 * application callback receives the number of successfully converted
 * frames. Reception is then rearmed when appropriate.
 *
 * This function may execute in FSP callback or interrupt context.
 */
static void sent_renesas_serial_complete(const struct device *dev)
{
	const struct sent_renesas_config *config = dev->config;
	struct sent_renesas_data *data = dev->data;
	struct sent_rx_callback_config callback;
	k_spinlock_key_t key;
	uint32_t converted_count = 0U;
	uint32_t count;
	bool rearm;

	key = k_spin_lock(&data->lock);

	data->serial_read_active = false;

	if (!data->serial_callback_registered) {
		data->serial_request_count = 0U;
		k_spin_unlock(&data->lock, key);
		return;
	}

	callback = data->serial_callback;

	count = MIN(data->serial_request_count,
			callback.max_num_frame);

	data->serial_request_count = 0U;

	k_spin_unlock(&data->lock, key);

	for (uint32_t i = 0U; i < count; i++) {
		if (sent_renesas_unpack_serial_frame(
				config,
				&data->slow_messages[i],
				&callback.frame[converted_count])) {
			converted_count++;
		}
	}

	if (converted_count > 0U) {
		callback.callback(dev,
				  config->channel,
				  converted_count,
				  callback.user_data);
	}

	key = k_spin_lock(&data->lock);

	rearm =
		data->state == SENT_RENESAS_CHANNEL_STARTED &&
		data->serial_callback_registered &&
		data->serial_callback.callback == callback.callback &&
		data->serial_callback.frame == callback.frame;

	data->serial_frames_received += converted_count;

	k_spin_unlock(&data->lock, key);

	if (rearm) {
		int ret = sent_renesas_serial_read_arm(dev);

		if (ret != 0) {
			LOG_ERR("Failed to rearm serial RX channel %u: %d",
				config->channel, ret);
		}
	}
}

/*
 * Handle a fast-channel receive error.
 *
 * The corresponding diagnostic counter is incremented, the current fast
 * read request is aborted, and reception is restarted when the channel
 * remains active.
 */
static void sent_renesas_fast_error_handle(
	const struct device *dev,
	sent_event_t event)
{
	struct sent_renesas_data *data = dev->data;

	switch (event) {
	case SENT_EVENT_FAST_CHANNEL_CRC_ERROR:
		data->fast_crc_errors++;
		break;

	case SENT_EVENT_FAST_CHANNEL_MESSAGE_LOST:
		data->fast_message_lost_errors++;
		break;

	case SENT_EVENT_FAST_CHANNEL_NIBBLE_ENCODING_ERROR:
		data->fast_encoding_errors++;
		break;

	case SENT_EVENT_FAST_CHANNEL_NIBBLE_COUNT_ERROR:
		data->fast_nibble_count_errors++;
		break;

	default:
		data->hardware_errors++;
		break;
	}

	sent_renesas_fast_read_abort(data);

	if ((data->state == SENT_RENESAS_CHANNEL_STARTED) &&
		data->fast_callback_registered) {
		int ret = sent_renesas_fast_read_arm(dev);

		if (ret != 0) {
			LOG_ERR("Failed to recover fast RX: %d", ret);
		}
	}
}

/*
 * Handle a slow-channel receive error and restart serial reception when
 * possible.
 */
static void sent_renesas_serial_error_handle(
	const struct device *dev,
	sent_event_t event)
{
	struct sent_renesas_data *data = dev->data;

	switch (event) {
	case SENT_EVENT_SLOW_CHANNEL_CRC_ERROR:
		data->slow_crc_errors++;
		break;

	case SENT_EVENT_SLOW_CHANNEL_MESSAGE_LOST:
		data->slow_message_lost_errors++;
		break;

	case SENT_EVENT_SLOW_CHANNEL_ENCODING_ERROR:
		data->slow_encoding_errors++;
		break;

	default:
		data->hardware_errors++;
		break;
	}

	sent_renesas_serial_read_abort(data);

	if ((data->state == SENT_RENESAS_CHANNEL_STARTED) &&
		data->serial_callback_registered) {
		int ret = sent_renesas_serial_read_arm(dev);

		if (ret != 0) {
			LOG_ERR("Failed to recover serial RX: %d", ret);
		}
	}
}

/*
 * Handle an error that affects the entire RSENT communication channel.
 *
 * Both fast and serial receive requests are aborted and restarted if the
 * channel remains active.
 */
static void sent_renesas_common_error_handle(
	const struct device *dev,
	sent_event_t event)
{
	struct sent_renesas_data *data = dev->data;

	switch (event) {
	case SENT_EVENT_CALIBRATION_PULSE_LENGTH_ERROR:
		data->calibration_length_errors++;
		break;

	case SENT_EVENT_CALIBRATION_PULSE_LENGTH_VARIATION_ERROR:
		data->calibration_variation_errors++;
		break;

	case SENT_EVENT_NO_RESPONSE_ERROR:
		data->no_response_errors++;
		break;

	default:
		data->hardware_errors++;
		break;
	}

	sent_renesas_fast_read_abort(data);
	sent_renesas_serial_read_abort(data);

	if (data->state != SENT_RENESAS_CHANNEL_STARTED) {
		return;
	}

	if (data->fast_callback_registered) {
		int ret = sent_renesas_fast_read_arm(dev);

		if (ret != 0) {
			LOG_ERR("Failed to recover fast RX: %d", ret);
		}
	}

	if (data->serial_callback_registered) {
		int ret = sent_renesas_serial_read_arm(dev);

		if (ret != 0) {
			LOG_ERR("Failed to recover serial RX: %d", ret);
		}
	}
}

/*
 * Receive asynchronous events from the Renesas FSP RSENT driver.
 *
 * This callback is registered through R_RSENT_SENT_CallbackSet(). It acts
 * as the bridge between FSP events and the generic Zephyr SENT driver.
 *
 * The callback can run from FSP interrupt context. It must therefore avoid
 * blocking operations and use interrupt-safe synchronization.
 */
static void sent_renesas_fsp_callback(sent_callback_args_t *args)
{
	struct sent_renesas_callback_context *context;
	const struct device *dev;

	if ((args == NULL) || (args->p_context == NULL)) {
		return;
	}

	context =
		(struct sent_renesas_callback_context *)args->p_context;
	dev = context->dev;

	switch (args->event) {
	case SENT_EVENT_FAST_CHANNEL_RX_COMPLETE:
		sent_renesas_fast_complete(dev);
		break;

	case SENT_EVENT_SLOW_CHANNEL_RX_COMPLETE:
		sent_renesas_serial_complete(dev);
		break;

	case SENT_EVENT_FAST_CHANNEL_CRC_ERROR:
	case SENT_EVENT_FAST_CHANNEL_MESSAGE_LOST:
	case SENT_EVENT_FAST_CHANNEL_NIBBLE_ENCODING_ERROR:
	case SENT_EVENT_FAST_CHANNEL_NIBBLE_COUNT_ERROR:
		sent_renesas_fast_error_handle(dev, args->event);
		break;

	case SENT_EVENT_SLOW_CHANNEL_CRC_ERROR:
	case SENT_EVENT_SLOW_CHANNEL_MESSAGE_LOST:
	case SENT_EVENT_SLOW_CHANNEL_ENCODING_ERROR:
		sent_renesas_serial_error_handle(dev, args->event);
		break;

	case SENT_EVENT_CALIBRATION_PULSE_LENGTH_ERROR:
	case SENT_EVENT_CALIBRATION_PULSE_LENGTH_VARIATION_ERROR:
	case SENT_EVENT_NO_RESPONSE_ERROR:
		sent_renesas_common_error_handle(dev, args->event);
		break;

	case SENT_EVENT_SPC_TIMEOUT:
		LOG_WRN("RSENT SPC timeout");
		break;

	default:
		LOG_WRN("Unhandled RSENT event: %u",
			(uint32_t)args->event);
		break;
	}
}

/*
 * Release a DMA channel claimed during initialization.
 */
static void sent_renesas_dma_release(const struct device *dev)
{
	const struct sent_renesas_config *config = dev->config;
	struct sent_renesas_data *data = dev->data;

	if (!data->dma_channel_claimed) {
		return;
	}

	dma_release_channel(config->dma_dev, config->dma_channel);
	data->dma_channel_claimed = false;
}

/*
 * Claim the DMA channel and construct the FSP transfer instance expected by
 * the RSENT FSP driver. Zephyr DMA APIs are used only for channel ownership.
 */
static int sent_renesas_dma_init(const struct device *dev)
{
	const struct sent_renesas_config *config = dev->config;
	struct sent_renesas_data *data = dev->data;
	uint32_t requested_channel = config->dma_channel;
	int channel;

	if (config->dma_dev == NULL) {
		return 0;
	}

	if (!device_is_ready(config->dma_dev)) {
		return -ENODEV;
	}

	channel = dma_request_channel(config->dma_dev, &requested_channel);
	if (channel < 0) {
		return channel;
	}

	/* The RH850 DMA filter currently returns the first available channel. */
	if ((uint32_t)channel != requested_channel) {
		dma_release_channel(config->dma_dev, (uint32_t)channel);
		return -EBUSY;
	}

	data->dma_channel_claimed = true;
	return 0;
}

#define SENT_RENESAS_SAMPLE_CLOCK_HZ 16000000U
#define SENT_RENESAS_TICK_MIN_US     1U
#define SENT_RENESAS_TICK_MAX_US     90U
#define SENT_RENESAS_TIMESTAMP_HZ    1000000U

static int sent_renesas_clock_init(const struct device *dev)
{
	const struct sent_renesas_config *config = dev->config;
	int ret;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("RSENT clock device is not ready");
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret < 0) {
		LOG_ERR("Failed to enable RSENT clock: %d", ret);
		return ret;
	}

	return 0;
}

static int sent_renesas_configure_clock(const struct device *dev)
{
	const struct sent_renesas_config *config = dev->config;
	rsent_sent_extended_cfg_t *extended_cfg = config->fsp_extended_cfg;
	uint64_t best_error_numerator = SENT_RENESAS_SAMPLE_CLOCK_HZ;
	uint32_t best_error_divisor = 1U;
	uint32_t best_multiplier = 1U;
	uint32_t best_divider = 1U;
	uint32_t rounded_clock_mhz;
	uint32_t tick_length_x10;
	uint32_t clock_rate;
	int ret;

	ret = clock_control_get_rate(config->clock_dev, config->clock_subsys,
				     &clock_rate);
	if (ret < 0) {
		LOG_ERR("Failed to get RSENT clock rate: %d", ret);
		return ret;
	}

	if (clock_rate == 0U) {
		LOG_ERR("Invalid RSENT clock rate");
		return -EINVAL;
	}

	if ((config->clock_tick_length_us < SENT_RENESAS_TICK_MIN_US) ||
	    (config->clock_tick_length_us > SENT_RENESAS_TICK_MAX_US)) {
		LOG_ERR("Invalid RSENT tick length: %u us",
			config->clock_tick_length_us);
		return -EINVAL;
	}

	/* Match the FSP configurator search for the closest 16 MHz sample clock. */
	for (uint32_t multiplier = 1U; multiplier <= 32U; multiplier++) {
		for (uint32_t divider = 1U; divider <= 128U; divider++) {
			uint64_t multiplied_clock = (uint64_t)clock_rate * multiplier;
			uint64_t target_clock =
				(uint64_t)SENT_RENESAS_SAMPLE_CLOCK_HZ * divider;
			uint64_t error_numerator = (multiplied_clock > target_clock)
				? multiplied_clock - target_clock
				: target_clock - multiplied_clock;

			if ((error_numerator * best_error_divisor) <
			    (best_error_numerator * divider)) {
				best_error_numerator = error_numerator;
				best_error_divisor = divider;
				best_multiplier = multiplier;
				best_divider = divider;
			}

			/*
			 * An exact 16 MHz sample clock was found.
			 * No better result is possible.
			 */
			if (error_numerator == 0U) {
				goto sample_clock_found;
			}
		}
	}

sample_clock_found:

	tick_length_x10 = config->clock_tick_length_us * 10U;
	rounded_clock_mhz =
		(clock_rate + (SENT_RENESAS_TIMESTAMP_HZ / 2U)) /
		SENT_RENESAS_TIMESTAMP_HZ;
	if ((rounded_clock_mhz == 0U) || (rounded_clock_mhz > 128U)) {
		LOG_ERR("RSENT clock rate cannot be represented by TTPV: %u Hz",
			clock_rate);
		return -EINVAL;
	}

	extended_cfg->com_clk_cfg.bit.scmv = best_multiplier - 1U;
	extended_cfg->com_clk_cfg.bit.scdv = best_divider - 1U;
	extended_cfg->com_clk_cfg.bit.tti = (tick_length_x10 / 10U) - 1U;
	extended_cfg->com_clk_cfg.bit.ttf = tick_length_x10 % 10U;
	extended_cfg->timestamp_cfg.bit.ttpv = rounded_clock_mhz - 1U;

	return 0;
}

/*
 * Validate CRC settings against RH850 RSENT hardware limitations.
 *
 * RH850 does not support including the status nibble in the fast-message CRC.
 * It also provides only one data-nibble CRC format bit shared by the fast and
 * short serial channels. Therefore, when both CRC checks are enabled, both
 * properties must select the same CRC implementation.
 */
static int sent_renesas_crc_config_validate(const struct device *dev)
{
	const struct sent_renesas_config *config = dev->config;
	bool fast_crc_enabled = config->fast_crc != FAST_CRC_DISABLE;
	bool short_crc_enabled = config->short_serial_crc != SHORT_CRC_DISABLE;

	if ((config->fast_crc & FAST_CRC_STATUS_INCLUDE) != 0U) {
		LOG_ERR("RH850 RSENT does not support FAST_CRC_STATUS_INCLUDE");
		return -ENOTSUP;
	}

	if (fast_crc_enabled && short_crc_enabled &&
	    (((config->fast_crc == FAST_CRC_RECOMMENDED_IMPLEMENTATION) &&
	      (config->short_serial_crc != SHORT_CRC_RECOMMENDED_IMPLEMENTATION)) ||
	     ((config->fast_crc == FAST_CRC_LEGACY_IMPLEMENTATION) &&
	      (config->short_serial_crc != SHORT_CRC_LEGACY_IMPLEMENTATION)))) {
		LOG_ERR("RH850 RSENT requires fast-crc and short-serial-crc "
			"to use the same CRC format when both are enabled");
		return -ENOTSUP;
	}

	return 0;
}

/*
 * Initialize one Zephyr RSENT device.
 *
 * This function applies the configured pins, initializes runtime state,
 * opens the corresponding FSP RSENT channel, and registers the internal
 * FSP callback used to receive completion and error events.
 */
static int sent_renesas_init(const struct device *dev)
{
	const struct sent_renesas_config *config = dev->config;
	struct sent_renesas_data *data = dev->data;
	unsigned int irq_key;
	fsp_err_t fsp_ret;
	int ret;

	ret = sent_renesas_crc_config_validate(dev);
	if (ret != 0) {
		return ret;
	}

	ret = sent_renesas_clock_init(dev);
	if (ret != 0) {
		return ret;
	}

	ret = sent_renesas_configure_clock(dev);
	if (ret != 0) {
		return ret;
	}

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Failed to apply RSENT channel %u pinctrl: %d",
			config->channel, ret);
		return ret;
	}

	memset(data, 0, sizeof(*data));
	data->state = SENT_RENESAS_CHANNEL_STOPPED;
	data->callback_context.dev = dev;

	ret = sent_renesas_dma_init(dev);
	if (ret != 0) {
		return ret;
	}

	/*
	 * Open registers the FSP ISR context and may unmask RXI/ERI.
	 *
	 * Prevent an interrupt from observing a partially initialized FSP
	 * control block. Mask both lines before restoring CPU interrupts.
	 */
	irq_key = irq_lock();

	fsp_ret = R_RSENT_SENT_Open(
		(sent_ctrl_t *)&data->fsp_ctrl, config->fsp_cfg);

	sent_renesas_irq_disable(dev);
	irq_unlock(irq_key);

	if (fsp_ret != FSP_SUCCESS) {
		LOG_ERR("Failed to open RSENT channel %u: %d",
			config->channel, (int)fsp_ret);
		sent_renesas_dma_release(dev);
		return sent_renesas_error_convert(fsp_ret);
	}

	/*
	 * RXI/ERI remain masked while callback configuration is installed.
	 */
	fsp_ret = R_RSENT_SENT_CallbackSet(
		(sent_ctrl_t *)&data->fsp_ctrl,
		sent_renesas_fsp_callback,
		&data->callback_context,
		&data->callback_memory);
	if (fsp_ret != FSP_SUCCESS) {
		LOG_ERR("Failed to set RSENT channel %u callback: %d",
			config->channel, (int)fsp_ret);

		(void)R_RSENT_SENT_Close(
			(sent_ctrl_t *)&data->fsp_ctrl);
		sent_renesas_irq_disable(dev);
		sent_renesas_dma_release(dev);

		return sent_renesas_error_convert(fsp_ret);
	}

	/*
	 * Leave RXI/ERI masked until start_listening() has armed receive
	 * buffers and published the STARTED state.
	 */
	return 0;
}

/*
 * Generic Zephyr SENT operations implemented by the Renesas RH850 RSENT
 * driver.
 */
static DEVICE_API(sent, sent_renesas_driver_api) = {
	.start_listening = sent_renesas_start_listening,
	.stop_listening = sent_renesas_stop_listening,
	.register_callback = sent_renesas_register_callback,
};

/*
 * Devicetree value mapping.
 */

#define SENT_RENESAS_SLOW_FORMAT_disabled \
	SENT_SERIAL_MESSAGE_FORMAT_NO_SERIAL_MESSAGE

#define SENT_RENESAS_SLOW_FORMAT_short_serial \
	SENT_SERIAL_MESSAGE_FORMAT_SHORT_SERIAL_MESSAGE

#define SENT_RENESAS_SLOW_FORMAT_enhanced_serial \
	SENT_SERIAL_MESSAGE_FORMAT_ENHANCED_SERIAL_MESSAGE

#define SENT_RENESAS_SLOW_FORMAT(node_id)				\
	UTIL_CAT(SENT_RENESAS_SLOW_FORMAT_,				\
		 DT_STRING_TOKEN(node_id, slow_message_format))

#define SENT_RENESAS_SUCCESSIVE_CALIB_PULSE_OPTION_1 1
#define SENT_RENESAS_SUCCESSIVE_CALIB_PULSE_OPTION_2 2

/*
 * SAE J2716 successive calibration pulse check method:
 *
 * Option 1 is the preferred method. The result of comparing the current
 * calibration pulse with the following calibration pulse is applied to the
 * preceding frame.
 *
 * Option 2 is the low-latency method. The current calibration pulse is
 * compared with the calibration pulse of the last valid preceding frame.
 */
#define SENT_RENESAS_FRAME_CHECK(node_id)                              \
	COND_CODE_1(                                                     \
		IS_EQ(DT_PROP(node_id, successive_calib_pulse_method),       \
		      SENT_RENESAS_SUCCESSIVE_CALIB_PULSE_OPTION_1), \
		(SENT_FRAME_CHECK_METHOD_NEXT_CALIBRATION_PULSE),          \
		(SENT_FRAME_CHECK_METHOD_PREVIOUS_CALIBRATION_PULSE))

#define SENT_RENESAS_MESSAGE_LENGTH_variable \
	SENT_PAUSE_PULSE_CFG_VARIABLE_MESSAGE_LENGTH

#define SENT_RENESAS_MESSAGE_LENGTH_fixed \
	SENT_PAUSE_PULSE_CFG_FIXED_MESSAGE_LENGTH

#define SENT_RENESAS_MESSAGE_LENGTH(node_id)                           \
	UTIL_CAT(SENT_RENESAS_MESSAGE_LENGTH_,                           \
		 DT_STRING_TOKEN(node_id, message_length_mode))

/*
 * pause-pulse-detect controls whether a pause pulse is present. When it is
 * enabled, message-length-mode selects variable- or fixed-length operation.
 */
#define SENT_RENESAS_PAUSE_PULSE_ENABLED(node_id)                          \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, message_length_mode),        \
			(SENT_RENESAS_MESSAGE_LENGTH(node_id)),                 \
			(SENT_PAUSE_PULSE_CFG_ABSENT))

#define SENT_RENESAS_PAUSE_PULSE(node_id)                                  \
	COND_CODE_1(DT_PROP(node_id, pause_pulse_detect),                  \
			(SENT_RENESAS_PAUSE_PULSE_ENABLED(node_id)),            \
			(SENT_PAUSE_PULSE_CFG_ABSENT))

#define SENT_RENESAS_NUM_DATA_NIBBLES(node_id)				\
	UTIL_CAT(SENT_NUM_OF_DATA_NIBBLES_,				\
		 DT_PROP(node_id, num_data_nibbles))

#define SENT_RENESAS_UNIT(node_id) \
	DT_PROP(DT_PARENT(node_id), unit)

/*
 * Connect an IRQ without enabling it.
 *
 * Enable is deferred until start_listening(). FSP Open() still receives
 * the real IRQ numbers and registers the FSP ISR context.
 */
#define SENT_RENESAS_IRQ_CONNECT(inst, irq_name, isr)		\
	do {								\
		IRQ_CONNECT(\
			DT_IRQ_BY_NAME(DT_DRV_INST(inst),		\
							irq_name, irq),			\
			DT_IRQ_BY_NAME(DT_DRV_INST(inst),		\
							irq_name, priority),		\
			isr, NULL, 0);					\
		irq_disable(DT_IRQ_BY_NAME(DT_DRV_INST(inst),	\
							irq_name, irq));		\
	} while (0)

#define SENT_RENESAS_IRQ_INIT(inst)				\
	do {								\
		SENT_RENESAS_IRQ_CONNECT(\
			inst, rxi, sent_renesas_rxi_isr);		\
		SENT_RENESAS_IRQ_CONNECT(\
			inst, eri, sent_renesas_eri_isr);		\
	} while (0)

/*
 * Communication-error bits supported by the supplied FSP error handler.
 *
 * The supplied FSP explicitly masks CS with IDE, so these CS masks have
 * the corresponding IDE error-enable bit positions. FRIE is not included:
 * R_RSENT_SENT_Start() adds it separately.
 */

#define SENT_RENESAS_ERROR_IRQ_MASK					\
	(R_RSENT0_CS_FCS_Msk | R_RSENT0_CS_FMS_Msk |		\
	 R_RSENT0_CS_FES_Msk | R_RSENT0_CS_FNS_Msk |		\
	 R_RSENT0_CS_CLS_Msk | R_RSENT0_CS_CVS_Msk |		\
	 R_RSENT0_CS_NRS_Msk | R_RSENT0_CS_SCS_Msk |		\
	 R_RSENT0_CS_SMS_Msk | R_RSENT0_CS_SES_Msk)

/*
 * Convert the generic CRC properties into the FSP channel check settings.
 */
#define SENT_RENESAS_FAST_CRC_CHECK(node_id)                            \
	COND_CODE_1(                                                      \
		IS_EQ(DT_PROP(node_id, fast_crc), FAST_CRC_DISABLE),        \
		(SENT_FAST_CHANNEL_CRC_CHECK_DISABLE),                      \
		(SENT_FAST_CHANNEL_CRC_CHECK_ENABLE))

#define SENT_RENESAS_SERIAL_CRC_CHECK(node_id)                          \
	COND_CODE_1(                                                      \
		IS_EQ(DT_PROP(node_id, short_serial_crc), SHORT_CRC_DISABLE), \
		(SENT_SLOW_CHANNEL_CRC_CHECK_DISABLE),                      \
		(SENT_SLOW_CHANNEL_CRC_CHECK_ENABLE))

/*
 * RH850 RSENT provides one RSENTnCC.DCF bit shared by fast and short serial
 * CRC calculation. Configuration validation guarantees that two enabled CRC
 * checks cannot request different formats. Prefer fast-crc when it is enabled;
 * otherwise derive the shared format from short-serial-crc.
 */
#define SENT_RENESAS_CRC_FORMAT(node_id)                                \
	COND_CODE_1(                                                      \
		IS_EQ(DT_PROP(node_id, fast_crc),                           \
		      FAST_CRC_LEGACY_IMPLEMENTATION),                      \
		(SENT_DATA_NIBBLE_CRC_FORMAT_PRE_SAE_J2716),                       \
		(COND_CODE_1(                                               \
			IS_EQ(DT_PROP(node_id, short_serial_crc),             \
			      SHORT_CRC_LEGACY_IMPLEMENTATION),               \
			(SENT_DATA_NIBBLE_CRC_FORMAT_PRE_SAE_J2716),                 \
			(SENT_DATA_NIBBLE_CRC_FORMAT_SAE_J2716))))

#define SENT_RENESAS_HAS_DMA(node_id) \
	DT_DMAS_HAS_NAME(node_id, rx)

#define SENT_RENESAS_DMA_CTLR(node_id) \
	DT_DMAS_CTLR_BY_NAME(node_id, rx)

#define SENT_RENESAS_DMA_CHANNEL(node_id) \
	DT_DMAS_CELL_BY_NAME(node_id, rx, channel)

#define SENT_RENESAS_DMA_UNIT(node_id) \
	DT_PROP(SENT_RENESAS_DMA_CTLR(node_id), dma_unit)

#define SENT_RENESAS_DMA_IRQ_NAME(channel) UTIL_CAT(ch, channel)

#define SENT_RENESAS_DMA_IRQ_BY_NAME(node_id, name, cell) \
	DT_IRQ_BY_NAME(node_id, name, cell)

#define SENT_RENESAS_DMA_IRQ(node_id) \
	SENT_RENESAS_DMA_IRQ_BY_NAME(\
		SENT_RENESAS_DMA_CTLR(node_id), \
		SENT_RENESAS_DMA_IRQ_NAME(SENT_RENESAS_DMA_CHANNEL(node_id)), irq)

#define SENT_RENESAS_DMA_IRQ_PRIORITY(node_id) \
	SENT_RENESAS_DMA_IRQ_BY_NAME(\
		SENT_RENESAS_DMA_CTLR(node_id), \
		SENT_RENESAS_DMA_IRQ_NAME(SENT_RENESAS_DMA_CHANNEL(node_id)), priority)

#define SENT_RENESAS_DMA_TRIGGER_0 DMAC_TRIGGER_EVENT_RSENT_INTRSENT0RI
#define SENT_RENESAS_DMA_TRIGGER_1 DMAC_TRIGGER_EVENT_RSENT_INTRSENT1RI
#define SENT_RENESAS_DMA_TRIGGER_2 DMAC_TRIGGER_EVENT_RSENT_INTRSENT2RI
#define SENT_RENESAS_DMA_TRIGGER_3 DMAC_TRIGGER_EVENT_RSENT_INTRSENT3RI
#define SENT_RENESAS_DMA_TRIGGER_4 DMAC_TRIGGER_EVENT_RSENT_INTRSENT4RI
#define SENT_RENESAS_DMA_TRIGGER_13 DMAC_TRIGGER_EVENT_RSENT_INTRSENT13RI
#define SENT_RENESAS_DMA_TRIGGER(node_id) \
	UTIL_CAT(SENT_RENESAS_DMA_TRIGGER_, SENT_RENESAS_UNIT(node_id))

#define SENT_RENESAS_DMA_NAME(prefix, node_id) \
	UTIL_CAT(prefix, node_id)

#define SENT_RENESAS_DMA_DEFINE_PRESENT(node_id) \
	static dmac_instance_ctrl_t \
		SENT_RENESAS_DMA_NAME(sent_renesas_dma_ctrl_, node_id); \
	static transfer_info_t \
		SENT_RENESAS_DMA_NAME(sent_renesas_dma_info_, node_id) = { \
			.transfer_mode_cfg.transfer_mode_b = { \
				.transfer_mode = TRANSFER_MODE_NORMAL, \
				.src_addr_mode = TRANSFER_ADDR_MODE_FIXED, \
				.des_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED, \
				.src_trans_size = TRANSFER_SIZE_1_BYTE, \
				.des_trans_size = TRANSFER_SIZE_1_BYTE, \
				.transfer_completion_irq = 1U, \
				.transfer_descriptor_irq = 0U, \
				.channel_address_error_irq = 0U, \
			}, \
			.transfer_count_per_hw_request = 1U, \
		}; \
	static transfer_switchable_redundant_cfg_t \
		SENT_RENESAS_DMA_NAME(sent_renesas_dma_redundant_, node_id) = { \
			.redundant_operation = \
				TRANSFER_SWITCHABLE_REDUNDANT_OPERATION_DISABLE, \
		}; \
	static dmac_extended_cfg_t \
		SENT_RENESAS_DMA_NAME(sent_renesas_dma_extend_, node_id) = { \
			.unit = SENT_RENESAS_DMA_UNIT(node_id), \
			.channel = SENT_RENESAS_DMA_CHANNEL(node_id), \
			.channel_scheduling = DMAC_CHANNEL_SCHEDULING_FIXED, \
			.gather_setting.gather_mode = DMAC_GATHER_MODE_DISABLE, \
			.scatter_setting.scatter_mode = DMAC_SCATTER_MODE_DISABLE, \
			.scatter_setting.zero_fill_mode = DMAC_ZERO_FILL_MODE_DISABLE, \
			.irq = (IRQn_Type)SENT_RENESAS_DMA_IRQ(node_id), \
			.irq_priority_level = \
				(uint8_t)SENT_RENESAS_DMA_IRQ_PRIORITY(node_id), \
			.irq_descriptor = FSP_INVALID_VECTOR, \
			.irq_descriptor_pri_level = BSP_IRQ_DISABLED, \
			.irq_error = FSP_INVALID_VECTOR, \
			.irq_error_priority_level = BSP_IRQ_DISABLED, \
			.activation_source = SENT_RENESAS_DMA_TRIGGER(node_id), \
			.support_operation = DMAC_SUPPORT_OPERATION_NORMAL, \
			.p_switchable_redundant_cfg = \
				&SENT_RENESAS_DMA_NAME(\
					sent_renesas_dma_redundant_, node_id), \
		}; \
	static const transfer_cfg_t \
		SENT_RENESAS_DMA_NAME(sent_renesas_dma_cfg_, node_id) = { \
			.p_info = &SENT_RENESAS_DMA_NAME(\
				sent_renesas_dma_info_, node_id), \
			.p_extend = &SENT_RENESAS_DMA_NAME(\
				sent_renesas_dma_extend_, node_id), \
		}; \
	static const transfer_instance_t \
		SENT_RENESAS_DMA_NAME(sent_renesas_dma_transfer_, node_id) = { \
			.p_ctrl = &SENT_RENESAS_DMA_NAME(\
				sent_renesas_dma_ctrl_, node_id), \
			.p_cfg = &SENT_RENESAS_DMA_NAME(\
				sent_renesas_dma_cfg_, node_id), \
			.p_api = &g_transfer_on_dmac, \
		};

#define SENT_RENESAS_DMA_DEFINE(node_id) \
	COND_CODE_1(SENT_RENESAS_HAS_DMA(node_id), \
			  (SENT_RENESAS_DMA_DEFINE_PRESENT(node_id)), ())

#define SENT_RENESAS_DMA_DEV(node_id) \
	COND_CODE_1(SENT_RENESAS_HAS_DMA(node_id), \
			  (DEVICE_DT_GET(SENT_RENESAS_DMA_CTLR(node_id))), (NULL))

#define SENT_RENESAS_DMA_CHANNEL_OR_ZERO(node_id) \
	COND_CODE_1(SENT_RENESAS_HAS_DMA(node_id), \
			  (SENT_RENESAS_DMA_CHANNEL(node_id)), (0U))

#define SENT_RENESAS_TRANSFER_RX(node_id) \
	COND_CODE_1(SENT_RENESAS_HAS_DMA(node_id), \
			  (&SENT_RENESAS_DMA_NAME(\
				  sent_renesas_dma_transfer_, node_id)), (NULL))

/*
 * Generate the FSP extended configuration and common FSP SENT
 * configuration for one enabled RSENT channel child node.
 *
 * Communication-clock and timestamp prescaler values are populated during
 * device initialization from clock-tick-length-us and the actual RSENT
 * peripheral clock frequency.
 */
#define SENT_RENESAS_FSP_CHANNEL_DEFINE(node_id)				\
	SENT_RENESAS_DMA_DEFINE(node_id)				\
	static rsent_sent_extended_cfg_t				\
		UTIL_CAT(sent_renesas_extended_cfg_, node_id) = {	\
			.com_clk_cfg.bit.scmv = 0U,			\
			.com_clk_cfg.bit.scdv = 0U,			\
			.com_clk_cfg.bit.tti = 0U,			\
			.com_clk_cfg.bit.ttf = 0U,			\
			.timestamp_cfg.bit.timestamp_mode = 0U,		\
			.timestamp_cfg.bit.ttm = 0U,			\
			.timestamp_cfg.bit.ttpv = 0U,			\
			.timestamp_master =				\
				RSENT_SENT_TIMESTAMP_MASTER_NO_MASTER, \
			.timestamp_synchronization_on_start =		\
				RSENT_TIMESTAMP_SYNCHRONIZATION_ON_START_DISABLE, \
			.timestamp_init_value = 0U,			\
			.timestamp_start_value = 0U,			\
			.error_irq_cfg = {				\
				.all = SENT_RENESAS_ERROR_IRQ_MASK,  \
			},						\
		};							\
									\
	static const sent_cfg_t						\
		UTIL_CAT(sent_renesas_fsp_cfg_, node_id) = {		\
			.channel =					\
				(uint8_t)SENT_RENESAS_UNIT(node_id),		\
			.num_of_data_nibbles =				\
				SENT_RENESAS_NUM_DATA_NIBBLES(node_id), \
			.serial_message_format =				\
				SENT_RENESAS_SLOW_FORMAT(node_id),		\
			.frame_check_method =				\
				SENT_RENESAS_FRAME_CHECK(node_id),		\
			.fast_channel_crc_check =				\
				SENT_RENESAS_FAST_CRC_CHECK(node_id), \
			.slow_channel_crc_check =				\
				SENT_RENESAS_SERIAL_CRC_CHECK(node_id), \
			.data_nibble_crc_format =				\
				SENT_RENESAS_CRC_FORMAT(node_id),		\
			.pause_pulse_cfg =				\
				SENT_RENESAS_PAUSE_PULSE(node_id),		\
			.spc_output_polarity =				\
				SENT_SPC_OUTPUT_POLARITY_ACTIVE_HIGH, \
			.spc_cfg = SENT_SPC_CFG_DISABLE,			\
			.rxi_irq =					\
				(IRQn_Type)DT_IRQ_BY_NAME(\
					DT_PARENT(node_id), rxi, irq), \
			.rxi_ipl =					\
				(uint8_t)DT_IRQ_BY_NAME(\
					DT_PARENT(node_id), rxi, priority), \
			.eri_irq =					\
				(IRQn_Type)DT_IRQ_BY_NAME(\
					DT_PARENT(node_id), eri, irq), \
			.eri_ipl =					\
				(uint8_t)DT_IRQ_BY_NAME(\
					DT_PARENT(node_id), eri, priority), \
			.p_transfer_rx =				\
				SENT_RENESAS_TRANSFER_RX(node_id),		\
			.p_spc_timer = NULL,				\
			.p_callback = NULL,				\
			.p_context = NULL,				\
			.p_extend =					\
				&UTIL_CAT(sent_renesas_extended_cfg_, \
					  node_id),			\
		};

/*
 * Generate one immutable Zephyr configuration entry for one enabled
 * RSENT channel child node.
 */
#define SENT_RENESAS_CONFIG_ENTRY(node_id)				\
	{								\
		.pincfg = PINCTRL_DT_DEV_CONFIG_GET(DT_PARENT(node_id)),	\
		.clock_dev = DEVICE_DT_GET(				\
			DT_CLOCKS_CTLR(DT_PARENT(node_id))),		\
		.clock_subsys = (clock_control_subsys_t)(uintptr_t)		\
			DT_CLOCKS_CELL(DT_PARENT(node_id), clk_id),		\
		.clock_tick_length_us =				\
			DT_PROP(node_id, clock_tick_length_us),		\
		.fsp_extended_cfg =					\
			&UTIL_CAT(sent_renesas_extended_cfg_, node_id),	\
		.channel = (uint8_t)DT_REG_ADDR(node_id),		\
		.num_data_nibbles =					\
			(uint8_t)DT_PROP(node_id, num_data_nibbles), \
		.serial_message_format =					\
			SENT_RENESAS_SLOW_FORMAT(node_id),		\
		.fast_crc = DT_PROP(node_id, fast_crc),			\
		.short_serial_crc = DT_PROP(node_id, short_serial_crc),	\
		.fsp_cfg =						\
			&UTIL_CAT(sent_renesas_fsp_cfg_, node_id),		\
		.dma_dev = SENT_RENESAS_DMA_DEV(node_id),		\
		.dma_channel = SENT_RENESAS_DMA_CHANNEL_OR_ZERO(node_id), \
	},

/*
 * Instantiate one Zephyr RSENT device.
 *
 * Each enabled parent RSENT instance must contain exactly one enabled
 * channel child. This macro generates:
 *
 * - The pinctrl configuration.
 * - The FSP channel configuration.
 * - The immutable Zephyr device configuration.
 * - The mutable runtime data.
 * - The final Zephyr struct device.
 */
#define RSENT_DEFINE(inst)						\
	PINCTRL_DT_INST_DEFINE(inst);					\
									\
	BUILD_ASSERT(\
		DT_CHILD_NUM_STATUS_OKAY(DT_DRV_INST(inst)) == 1,	\
		"Each RH850 RSENT instance must have exactly one "	\
		"enabled channel child");				\
									\
	DT_INST_FOREACH_CHILD_STATUS_OKAY(\
		inst, SENT_RENESAS_FSP_CHANNEL_DEFINE)			\
									\
	static const struct sent_renesas_config				\
		sent_renesas_config_##inst[] = {				\
			DT_FOREACH_CHILD_STATUS_OKAY(\
				DT_DRV_INST(inst),			\
				SENT_RENESAS_CONFIG_ENTRY)		\
		};							\
									\
	BUILD_ASSERT(\
		ARRAY_SIZE(sent_renesas_config_##inst) == 1U,		\
		"Invalid RH850 RSENT channel configuration");		\
									\
	static struct sent_renesas_data sent_renesas_data_##inst;	\
									\
	static int sent_renesas_init_##inst(const struct device *dev) \
	{								\
		SENT_RENESAS_IRQ_INIT(inst);				\
		return sent_renesas_init(dev);				\
	}								\
									\
	DEVICE_DT_INST_DEFINE(\
		inst,							\
		sent_renesas_init_##inst,				\
		NULL,							\
		&sent_renesas_data_##inst,				\
		&sent_renesas_config_##inst[0],				\
		POST_KERNEL,						\
		CONFIG_SENT_INIT_PRIORITY,				\
		&sent_renesas_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RSENT_DEFINE)
