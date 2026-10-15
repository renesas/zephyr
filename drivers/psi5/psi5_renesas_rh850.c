/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Supports the synchronous parallel bus mode and the asynchronous mode.
 * Universal bus, daisy-chain bus and wrapper features are not supported.
 */

#define DT_DRV_COMPAT renesas_rh850_psi5

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/psi5/psi5.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <soc.h>
#include "r_psi5.h"

LOG_MODULE_REGISTER(psi5_renesas_rh850, CONFIG_PSI5_LOG_LEVEL);

#define PSI5_RH850_MAX_SLOTS (8U)

/* Data frame error (CRC, parity or syntax) */
#define PSI5_RH850_RXDST_RXSTATUS BIT(0)
/* Serial message error */
#define PSI5_RH850_RXMMST_RXMERR  BIT(16)

/* PSI5 operating clock: 80 MHz */
#define PSI5_RH850_CLOCK_FREQ_HZ         (80000000UL)
#define PSI5_RH850_US_TO_CYCLES(us)      (((uint32_t)(us) * 80U))
/* Counter registers hold the number of clock cycles minus one */
#define PSI5_RH850_US_TO_COUNTER(us)     (PSI5_RH850_US_TO_CYCLES(us) - 1U)
/* A zero offset maps to 0 instead of underflowing */
#define PSI5_RH850_US_TO_COUNTER_SAT(us) (((us) == 0U) ? 0U : PSI5_RH850_US_TO_COUNTER(us))

/* Maximum counter values of the timing register fields */
#define PSI5_RH850_SHORTCNT_MAX (0x3FFFU)
#define PSI5_RH850_LONGCNT_MAX  (0xFFFFU)
#define PSI5_RH850_CNT20_MAX    (0xFFFFFU)

/* Bit time with a 5% clock tolerance margin, in clock cycles minus one */
#define PSI5_RH850_BITRATE_CYCLES(bit_ns)                                                          \
	((((uint32_t)(bit_ns) * (uint32_t)(PSI5_RH850_CLOCK_FREQ_HZ / 1000000UL) * 105U) /         \
	  100000U) -                                                                               \
	 1U)
#define PSI5_RH850_BITRATE_CYCLES_125K PSI5_RH850_BITRATE_CYCLES(8000U)
#define PSI5_RH850_BITRATE_CYCLES_189K PSI5_RH850_BITRATE_CYCLES(5300U)

#define PSI5_RH850_BITRATE_VAL(ch_node)                                                            \
	(DT_PROP_OR(ch_node, rx_bitrate_kbps, 125) == 189 ? PSI5_RH850_BITRATE_CYCLES_189K         \
							   : PSI5_RH850_BITRATE_CYCLES_125K)

/* Sampling period for 16 samples per bit, in clock cycles minus one */
#define PSI5_RH850_SMPL_CYCLES(bit_ns)                                                             \
	((((uint32_t)(bit_ns) * (uint32_t)(PSI5_RH850_CLOCK_FREQ_HZ / 1000000UL)) / 16000U) - 1U)
#define PSI5_RH850_SMPL_CYCLES_125K PSI5_RH850_SMPL_CYCLES(8000U)
#define PSI5_RH850_SMPL_CYCLES_189K PSI5_RH850_SMPL_CYCLES(5300U)

#define PSI5_RH850_SMPL_VAL(ch_node)                                                               \
	(DT_PROP_OR(ch_node, rx_bitrate_kbps, 125) == 189 ? PSI5_RH850_SMPL_CYCLES_189K            \
							   : PSI5_RH850_SMPL_CYCLES_125K)

struct psi5_renesas_rh850_config {
	const struct pinctrl_dev_config *pin_config;
	const psi5_api_t *fsp_api;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint8_t unit_id;
	bool async_mode;
	/* Data bits allowed by the configured TX frame format */
	uint32_t tx_data_mask;
};

/* Each PSI5 unit exposes a single channel (channel 0) */
struct psi5_renesas_rh850_data {
	psi5_instance_ctrl_t *fsp_ctrl;
	psi5_cfg_t *fsp_cfg;

	/* Serializes start, stop and callback registration (thread context) */
	struct k_mutex lock;
	/* Protects started and the TX request state; never held across a sleep or callback */
	struct k_spinlock tx_lock;
	struct k_sem tx_sem;
	struct k_sem tx_sync_sem;
	bool started;

	/* Must stay valid until the TX ISR reads it */
	uint32_t tx_data;
	/* A TX request is waiting to be loaded by the TX ISR */
	bool tx_armed;
	int tx_status;
	psi5_tx_callback_t tx_callback;
	void *tx_user_data;

	/*
	 * Last timestamp seen per slot. The FSP keeps the latest frame of each slot in a
	 * persistent buffer and does not indicate which slot an RX event updated, so only a
	 * slot whose timestamp changed holds a new frame. The baseline is taken in start_sync()
	 * before enabling the channel. Requires the IP timer running in master mode.
	 */
	uint32_t last_ts[PSI5_RH850_MAX_SLOTS];
	uint32_t last_msg_ts[PSI5_RH850_MAX_SLOTS];

	struct psi5_rx_callback_configs callback_configs;
};

extern void r_psi5_rx_isr(void);
extern void r_psi5_tx_isr(void);
extern void r_psi5_status_isr(void);

/*
 * Complete a TX request. Call without tx_lock.
 * Callback mode releases tx_sem first so the callback can send the next frame.
 *
 * Completion means the frame has been loaded into the hardware TX buffer, not that it
 * has been fully sent. The hardware then sends one bit per sync pulse, so stop_sync()
 * may return -EBUSY until the frame has been sent.
 */
static void psi5_renesas_rh850_tx_complete(const struct device *dev, psi5_tx_callback_t cb,
					   void *ud, int status)
{
	struct psi5_renesas_rh850_data *data = dev->data;

	if (cb != NULL) {
		k_sem_give(&data->tx_sem);
		cb(dev, 0, status, ud);
	} else {
		data->tx_status = status;
		k_sem_give(&data->tx_sync_sem);
	}
}

/* Callback of a blocking request whose caller has timed out; tx_complete() releases tx_sem */
static void psi5_renesas_rh850_tx_abandoned(const struct device *dev, uint8_t channel_id,
					    int status, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);
	ARG_UNUSED(status);
	ARG_UNUSED(user_data);
}

static void psi5_renesas_rh850_fsp_callback(psi5_callback_args_t *p_args)
{
	const struct device *dev = (const struct device *)p_args->p_context;
	struct psi5_renesas_rh850_data *data = dev->data;
	const struct psi5_renesas_rh850_config *config = dev->config;
	psi5_extended_cfg_t *p_ext = (psi5_extended_cfg_t *)data->fsp_cfg->p_extend;

	switch (p_args->event) {
	case PSI5_EVENT_RX_DATA_EXIST: {
		struct psi5_rx_callback_config *data_frame_cb = data->callback_configs.data_frame;

		if (data_frame_cb && data_frame_cb->callback && data_frame_cb->frame) {
			uint32_t num_frames = 0;
			bool rx_err = false;

			/*
			 * Scan all slots: the FSP does not report which slot was updated. Only a
			 * slot whose timestamp changed carries a new frame; skip the rest to avoid
			 * duplicate or lost frames.
			 */
			for (uint8_t slot = 1; slot <= PSI5_RH850_MAX_SLOTS; slot++) {
				if (p_ext->rx_setting.p_rx_slot[slot - 1].slot_enable !=
				    PSI5_RX_SLOT_ENABLE) {
					continue;
				}

				psi5_data_frame_t df;

				if (config->fsp_api->dataFrameRead(data->fsp_ctrl, &df, slot) ==
				    FSP_SUCCESS) {
					uint8_t slot_in_status =
						(uint8_t)((df.rx_data_status &
							   R_PSI50_PSI5nEMRXDST_RXSLOTNUM_Msk) >>
							  R_PSI50_PSI5nEMRXDST_RXSLOTNUM_Pos);

					if (slot_in_status != slot) {
						continue;
					}

					/* Skip slots not updated for this event */
					if (df.rx_timestamp == data->last_ts[slot - 1]) {
						continue;
					}
					data->last_ts[slot - 1] = df.rx_timestamp;

					/* Drop frames with a CRC, parity or syntax error */
					if ((df.rx_data_status & PSI5_RH850_RXDST_RXSTATUS) != 0U) {
						rx_err = true;
						continue;
					}

					if (num_frames < data_frame_cb->max_num_frame) {
						struct psi5_frame *f =
							&data_frame_cb->frame[num_frames];
						f->type = PSI5_DATA_FRAME;
						f->data = df.rx_data;
						f->timestamp = df.rx_timestamp;
						f->crc =
							(uint8_t)((df.rx_data_status >> 8) & 0x07U);
						f->slot_number = slot - 1;
						num_frames++;
					}
				}
			}

			if (num_frames > 0) {
				data_frame_cb->callback(dev, 0, num_frames,
							data_frame_cb->user_data);
			}

			if (rx_err) {
				LOG_DBG("PSI5 unit %d RX data frame error", config->unit_id);
				data_frame_cb->callback(dev, 0, 0, data_frame_cb->user_data);
			}
		}
		break;
	}

	case PSI5_EVENT_RX_MESSAGE_EXIST: {
		struct psi5_rx_callback_config *serial_frame_cb =
			data->callback_configs.serial_frame;

		if (serial_frame_cb && serial_frame_cb->callback && serial_frame_cb->frame) {
			uint32_t num_frames = 0;
			bool msg_err = false;
			psi5_status_t sts;

			/*
			 * An erroneous message is still copied to the slot buffer, but its status
			 * is not kept. The module error flag is still set during this event, so
			 * drop the message here; the error is reported by
			 * PSI5_EVENT_RX_MESSAGE_ERR.
			 */
			if ((config->fsp_api->statusGet(data->fsp_ctrl, &sts) == FSP_SUCCESS) &&
			    ((sts.rx_message_status & PSI5_RH850_RXMMST_RXMERR) != 0U)) {
				msg_err = true;
			}

			for (uint8_t slot = 1; slot <= PSI5_RH850_MAX_SLOTS; slot++) {
				if (p_ext->rx_setting.p_rx_slot[slot - 1].serial_message_enable !=
				    PSI5_RX_SERIAL_MESSAGE_CHANNEL_ENABLE) {
					continue;
				}

				psi5_serial_message_frame_t mf;

				if (config->fsp_api->messageFrameRead(data->fsp_ctrl, &mf, slot) ==
				    FSP_SUCCESS) {

					/* Skip slots not updated for this event */
					if (mf.rx_timestamp == data->last_msg_ts[slot - 1]) {
						continue;
					}

					/*
					 * Update the timestamp even for an erroneous message, so it
					 * is not reported as new by a later event.
					 */
					data->last_msg_ts[slot - 1] = mf.rx_timestamp;

					if (msg_err) {
						continue;
					}

					if (num_frames < serial_frame_cb->max_num_frame) {
						struct psi5_frame *f =
							&serial_frame_cb->frame[num_frames];
						/* Configuration bit 0 selects the 8-bit IDs */
						f->type = (mf.configuration == 0)
								  ? PSI5_SERIAL_FRAME_8_BIT_ID
								  : PSI5_SERIAL_FRAME_4_BIT_ID;
						f->serial.id = mf.serial_id;
						f->serial.data = mf.data_field;
						f->timestamp = mf.rx_timestamp;
						f->crc = mf.rx_crc;
						f->slot_number = slot - 1;
						num_frames++;
					}
				}
			}

			if (num_frames > 0) {
				serial_frame_cb->callback(dev, 0, num_frames,
							  serial_frame_cb->user_data);
			}
		}
		break;
	}

	case PSI5_EVENT_TRANSMIT_DATA_EMPTY: {
		/* Ignore the event when no request is armed, so tx_sem is given only once */
		k_spinlock_key_t key = k_spin_lock(&data->tx_lock);
		bool armed = data->tx_armed;
		psi5_tx_callback_t cb = data->tx_callback;
		void *ud = data->tx_user_data;

		data->tx_armed = false;
		data->tx_callback = NULL;
		k_spin_unlock(&data->tx_lock, key);

		if (armed) {
			psi5_renesas_rh850_tx_complete(dev, cb, ud, 0);
		}
		break;
	}

	case PSI5_EVENT_SYNC_PULSE_AND_RX_DATA_CONFLICT:
		LOG_DBG("PSI5 unit %d sync pulse & rx data conflict", config->unit_id);
		break;

	case PSI5_EVENT_RX_DATA_ERR:
		/* Already reported by PSI5_EVENT_RX_DATA_EXIST with num_frame = 0 */
		LOG_DBG("PSI5 unit %d RX data error", config->unit_id);
		break;

	case PSI5_EVENT_RX_DATA_FIFO_OVERFLOW: {
		struct psi5_rx_callback_config *data_frame_cb = data->callback_configs.data_frame;

		LOG_WRN("PSI5 unit %d RX data FIFO overflow", config->unit_id);

		if (data_frame_cb && data_frame_cb->callback) {
			data_frame_cb->callback(dev, 0, 0, data_frame_cb->user_data);
		}
		break;
	}

	case PSI5_EVENT_RX_MESSAGE_ERR:
	case PSI5_EVENT_RX_MESSAGE_FIFO_OVERFLOW: {
		struct psi5_rx_callback_config *serial_frame_cb =
			data->callback_configs.serial_frame;

		LOG_WRN("PSI5 unit %d RX message error/overflow (0x%x)", config->unit_id,
			p_args->event);

		if (serial_frame_cb && serial_frame_cb->callback) {
			serial_frame_cb->callback(dev, 0, 0, serial_frame_cb->user_data);
		}
		break;
	}

	default:
		break;
	}
}

static int psi5_renesas_rh850_start_sync(const struct device *dev, uint8_t channel)
{
	const struct psi5_renesas_rh850_config *config = dev->config;
	struct psi5_renesas_rh850_data *data = dev->data;
	k_spinlock_key_t key;
	fsp_err_t err;

	if (channel != 0) {
		return -EINVAL;
	}

	/* Asynchronous mode has no sync pulse */
	if (config->async_mode) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (data->started) {
		k_mutex_unlock(&data->lock);
		return -EALREADY;
	}

	/*
	 * Take the per-slot timestamps as a baseline before reception starts, so frames left
	 * in the FSP buffers from a previous run are not reported as new. Reception is not yet
	 * running here, so no RX ISR can race with these updates.
	 */
	psi5_extended_cfg_t *p_ext = (psi5_extended_cfg_t *)data->fsp_cfg->p_extend;

	for (uint8_t slot = 1; slot <= PSI5_RH850_MAX_SLOTS; slot++) {
		if (p_ext->rx_setting.p_rx_slot[slot - 1].slot_enable == PSI5_RX_SLOT_ENABLE) {
			psi5_data_frame_t df;

			if (config->fsp_api->dataFrameRead(data->fsp_ctrl, &df, slot) ==
			    FSP_SUCCESS) {
				data->last_ts[slot - 1] = df.rx_timestamp;
			}
		}

		if (p_ext->rx_setting.p_rx_slot[slot - 1].serial_message_enable ==
		    PSI5_RX_SERIAL_MESSAGE_CHANNEL_ENABLE) {
			psi5_serial_message_frame_t mf;

			if (config->fsp_api->messageFrameRead(data->fsp_ctrl, &mf, slot) ==
			    FSP_SUCCESS) {
				data->last_msg_ts[slot - 1] = mf.rx_timestamp;
			}
		}
	}

	err = config->fsp_api->start(data->fsp_ctrl);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Failed to start PSI5 sync (err %d)", err);
		k_mutex_unlock(&data->lock);
		return -EIO;
	}

	key = k_spin_lock(&data->tx_lock);
	data->started = true;
	k_spin_unlock(&data->tx_lock, key);
	k_mutex_unlock(&data->lock);

	return 0;
}

static int psi5_renesas_rh850_stop_sync(const struct device *dev, uint8_t channel)
{
	const struct psi5_renesas_rh850_config *config = dev->config;
	struct psi5_renesas_rh850_data *data = dev->data;
	k_spinlock_key_t key;
	fsp_err_t err;

	if (channel != 0) {
		return -EINVAL;
	}

	/* Asynchronous mode has no sync pulse */
	if (config->async_mode) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (!data->started) {
		k_mutex_unlock(&data->lock);
		return -EALREADY;
	}

	/*
	 * Reject the stop while a software TX request is still pending (armed but not yet
	 * completed by the TX ISR). Checking tx_armed and clearing started under the same lock
	 * keeps the decision atomic with send(), so the TX ISR cannot load the TX buffer after
	 * the hardware-empty check and leave the channel disabled with a non-empty buffer.
	 */
	key = k_spin_lock(&data->tx_lock);
	if (data->tx_armed) {
		k_spin_unlock(&data->tx_lock, key);
		k_mutex_unlock(&data->lock);
		return -EBUSY;
	}
	data->started = false;
	k_spin_unlock(&data->tx_lock, key);

	/* Hardware-busy check for a frame already loaded into the peripheral */
	err = config->fsp_api->stop(data->fsp_ctrl);
	if (err != FSP_SUCCESS) {
		/* The channel is still enabled */
		key = k_spin_lock(&data->tx_lock);
		data->started = true;
		k_spin_unlock(&data->tx_lock, key);
		k_mutex_unlock(&data->lock);

		if (err == FSP_ERR_IN_USE) {
			return -EBUSY;
		}

		LOG_ERR("Failed to stop PSI5 sync (err %d)", err);
		return -EIO;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

/* From ISR context, call only with K_NO_WAIT and a callback */
static int psi5_renesas_rh850_send(const struct device *dev, uint8_t channel,
				   const uint64_t data_val, k_timeout_t timeout,
				   psi5_tx_callback_t callback, void *user_data)
{
	const struct psi5_renesas_rh850_config *config = dev->config;
	struct psi5_renesas_rh850_data *data = dev->data;
	k_spinlock_key_t key;
	k_timepoint_t end;
	fsp_err_t err;
	bool armed;
	int status;

	if (channel != 0) {
		return -EINVAL;
	}

	/* Asynchronous mode has no TX */
	if (config->async_mode) {
		return -ENOTSUP;
	}

	/* Early exit; checked again under tx_lock */
	if (!data->started) {
		return -ENETDOWN;
	}

	/* Reject data wider than the TX frame instead of truncating it */
	if ((data_val & ~(uint64_t)config->tx_data_mask) != 0U) {
		LOG_ERR("TX data 0x%llx exceeds TX frame mask 0x%x", (unsigned long long)data_val,
			config->tx_data_mask);
		return -EMSGSIZE;
	}

	/* One deadline for both waits below */
	end = sys_timepoint_calc(timeout);

	/* Only one request can be pending */
	if (k_sem_take(&data->tx_sem, sys_timepoint_timeout(end)) != 0) {
		return -EAGAIN;
	}

	k_sem_reset(&data->tx_sync_sem);

	/* stop_sync() clears started under the same lock */
	key = k_spin_lock(&data->tx_lock);
	if (!data->started) {
		k_spin_unlock(&data->tx_lock, key);
		k_sem_give(&data->tx_sem);
		return -ENETDOWN;
	}

	data->tx_data = (uint32_t)data_val;
	data->tx_callback = callback;
	data->tx_user_data = user_data;
	data->tx_status = 0;
	data->tx_armed = true;

	err = config->fsp_api->write(data->fsp_ctrl, &data->tx_data);
	if (err != FSP_SUCCESS) {
		data->tx_armed = false;
		data->tx_callback = NULL;
	}
	k_spin_unlock(&data->tx_lock, key);

	if (err != FSP_SUCCESS) {
		LOG_ERR("Failed to write PSI5 TX data (err %d)", err);
		k_sem_give(&data->tx_sem);
		return -EIO;
	}

	if (callback != NULL) {
		/* tx_sem is released when the request completes */
		return 0;
	}

	if (k_sem_take(&data->tx_sync_sem, sys_timepoint_timeout(end)) == 0) {
		status = data->tx_status;
		k_sem_give(&data->tx_sem);
		return status;
	}

	/*
	 * Timed out. A request cannot be cancelled once given to the FSP: if still armed,
	 * the TX ISR completes it later and releases tx_sem through the internal callback.
	 */
	key = k_spin_lock(&data->tx_lock);
	armed = data->tx_armed;
	if (armed) {
		data->tx_callback = psi5_renesas_rh850_tx_abandoned;
	}
	k_spin_unlock(&data->tx_lock, key);

	if (armed) {
		return -EAGAIN;
	}

	/* Completed by the TX ISR right after the timeout */
	(void)k_sem_take(&data->tx_sync_sem, K_FOREVER);
	status = data->tx_status;
	k_sem_give(&data->tx_sem);

	return status;
}

static int psi5_renesas_rh850_register_callback(const struct device *dev, uint8_t channel,
						struct psi5_rx_callback_configs callback_configs)
{
	struct psi5_renesas_rh850_data *data = dev->data;

	if (channel != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->callback_configs = callback_configs;
	k_mutex_unlock(&data->lock);

	return 0;
}

static void psi5_renesas_rh850_rx_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	r_psi5_rx_isr();
}

static void psi5_renesas_rh850_tx_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	r_psi5_tx_isr();
}

static void psi5_renesas_rh850_status_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	r_psi5_status_isr();
}

static DEVICE_API(psi5, psi5_renesas_rh850_driver_api) = {
	.start_sync = psi5_renesas_rh850_start_sync,
	.stop_sync = psi5_renesas_rh850_stop_sync,
	.send = psi5_renesas_rh850_send,
	.register_callback = psi5_renesas_rh850_register_callback,
};

/* Enable the PSI5 module clock. The operating frequency is fixed. */
static int psi5_renesas_rh850_clock_init(const struct device *dev)
{
	const struct psi5_renesas_rh850_config *config = dev->config;
	int ret;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("PSI5 clock device is not ready");
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret < 0) {
		LOG_ERR("Failed to enable PSI5 clock: %d", ret);
		return ret;
	}

	return 0;
}

static int psi5_renesas_rh850_init(const struct device *dev)
{
	const struct psi5_renesas_rh850_config *config = dev->config;
	struct psi5_renesas_rh850_data *data = dev->data;
	int ret;

	ret = psi5_renesas_rh850_clock_init(dev);
	if (ret != 0) {
		return ret;
	}

	ret = pinctrl_apply_state(config->pin_config, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to apply pinctrl (%d)", ret);
		return ret;
	}

	k_mutex_init(&data->lock);
	k_sem_init(&data->tx_sem, 1, 1);
	k_sem_init(&data->tx_sync_sem, 0, 1);
	data->started = false;
	data->tx_armed = false;

	if (config->fsp_api->open(data->fsp_ctrl, data->fsp_cfg) != FSP_SUCCESS) {
		LOG_ERR("Failed to open FSP PSI5 unit %d", config->unit_id);
		return -EIO;
	}

	if (config->async_mode) {
		/* Asynchronous mode has no start_sync: start reception here */
		if (config->fsp_api->start(data->fsp_ctrl) != FSP_SUCCESS) {
			LOG_ERR("Failed to start FSP PSI5 unit %d (async)", config->unit_id);
			(void)config->fsp_api->close(data->fsp_ctrl);
			return -EIO;
		}
		data->started = true;
	}

	LOG_DBG("PSI5 unit %d initialized", config->unit_id);
	return 0;
}

/* Channel node ch@0, or the instance node if it does not exist */
#define PSI5_CHANNEL_NODE(n)                                                                       \
	COND_CODE_1(DT_NODE_EXISTS(DT_CHILD(DT_DRV_INST(n), ch_0)),                                \
		    (DT_CHILD(DT_DRV_INST(n), ch_0)), (DT_DRV_INST(n)))

#define PSI5_SLOT_CONFIG(slot_node)                                                                \
	[DT_REG_ADDR(slot_node)] = {                                                               \
		.slot_enable = PSI5_RX_SLOT_ENABLE,                                                \
		.serial_message_enable = DT_PROP_OR(slot_node, has_smc, false)                     \
						 ? PSI5_RX_SERIAL_MESSAGE_CHANNEL_ENABLE           \
						 : PSI5_RX_SERIAL_MESSAGE_CHANNEL_DISABLE,         \
		.operation_mode = PSI5_OPERATION_MODE_NORMAL,                                      \
		.crc_parity_sel = DT_PROP_OR(slot_node, has_parity, false)                         \
					  ? PSI5_RX_ERR_DETECT_PARITY                              \
					  : PSI5_RX_ERR_DETECT_CRC,                                \
		.data_length = (uint8_t)DT_PROP_OR(slot_node, data_length, 10),                    \
		.offset_counter =                                                                  \
			PSI5_RH850_US_TO_COUNTER_SAT(DT_PROP(slot_node, start_offset_us)),         \
	},

#define PSI5_SLOT_ASSERT(slot_node)                                                                \
	BUILD_ASSERT(DT_REG_ADDR(slot_node) < PSI5_RH850_MAX_SLOTS, "slot index out of range");   \
	BUILD_ASSERT(PSI5_RH850_US_TO_COUNTER_SAT(DT_PROP(slot_node, start_offset_us)) <=          \
			     PSI5_RH850_CNT20_MAX,                                                 \
		     "start-offset-us is too large");

#define PSI5_RH850_IRQ_CONNECT(n, irq_name, isr)                                                   \
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, irq_name, irq),                                         \
		    DT_INST_IRQ_BY_NAME(n, irq_name, priority), isr, DEVICE_DT_INST_GET(n), 0)

#define PSI5_RH850_IRQ_ENABLE(n, irq_name) irq_enable(DT_INST_IRQ_BY_NAME(n, irq_name, irq))

#define PSI5_RH850_ASYNC(n) DT_PROP_OR(PSI5_CHANNEL_NODE(n), async_mode, 0)

#define PSI5_RH850_COMM_MODE(n)                                                                    \
	(PSI5_RH850_ASYNC(n) ? PSI5_COMM_MODE_ASYNCHRONOUS                                         \
			     : PSI5_COMM_MODE_SYNCHRONOUS_PARALLEL_BUS)

/*
 * DT tx-frame to TX frame format. The 31-1s and 5-0s variants of a frame use the same
 * format; the suffix only selects the idle sync pulse value. Default: Short frame.
 */
#define PSI5_RH850_TX_FRAME_IDX_NON_STD (7)
#define PSI5_RH850_TX_FRAME_IDX(n)      DT_ENUM_IDX_OR(PSI5_CHANNEL_NODE(n), tx_frame, 0)

#define PSI5_RH850_TX_FRAME_FORMAT(n)                                                              \
	((PSI5_RH850_TX_FRAME_IDX(n) <= 1)   ? PSI5_TX_FRAME_FORMAT_SHORT                          \
	 : (PSI5_RH850_TX_FRAME_IDX(n) <= 3) ? PSI5_TX_FRAME_FORMAT_LONG                           \
	 : (PSI5_RH850_TX_FRAME_IDX(n) <= 5) ? PSI5_TX_FRAME_FORMAT_XLONG                          \
					     : PSI5_TX_FRAME_FORMAT_XXLONG)

/* TX data width: Short 6, Long 16, X-Long 22, XX-Long 24 bits */
#define PSI5_RH850_TX_DATA_MASK(n)                                                                 \
	((PSI5_RH850_TX_FRAME_IDX(n) <= 1)   ? 0x3FU                                               \
	 : (PSI5_RH850_TX_FRAME_IDX(n) <= 3) ? 0xFFFFU                                             \
	 : (PSI5_RH850_TX_FRAME_IDX(n) <= 5) ? 0x3FFFFFU                                           \
					     : 0xFFFFFFU)

/*
 * Idle sync pulse value: 1 for the 31-1s variants, 0 otherwise.
 * If tx-frame is omitted, the result is a Short frame with idle 0 (same as short-5-0s).
 */
#define PSI5_RH850_TX_DEFTXVAL(n)                                                                  \
	((DT_NODE_HAS_PROP(PSI5_CHANNEL_NODE(n), tx_frame) && (PSI5_RH850_TX_FRAME_IDX(n) <= 5) && \
	  ((PSI5_RH850_TX_FRAME_IDX(n) % 2) == 0))                                                 \
		 ? 1                                                                               \
		 : 0)

#define PSI5_RH850_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	BUILD_ASSERT(DT_INST_PROP(n, renesas_unit_id) < BSP_FEATURE_PSI5_MAX_CHANNEL,              \
		     "PSI5 unit-id exceeds BSP_FEATURE_PSI5_MAX_CHANNEL");                         \
	BUILD_ASSERT(DT_PROP_OR(PSI5_CHANNEL_NODE(n), period_sync_pulse_us, 500) > 0,              \
		     "period-sync-pulse-us must be > 0");                                          \
	BUILD_ASSERT(DT_PROP_OR(PSI5_CHANNEL_NODE(n), pulse_width_0_us, 15) > 0,                   \
		     "pulse-width-0-us must be > 0");                                              \
	BUILD_ASSERT(DT_PROP_OR(PSI5_CHANNEL_NODE(n), pulse_width_1_us, 30) > 0,                   \
		     "pulse-width-1-us must be > 0");                                              \
	BUILD_ASSERT(!DT_NODE_EXISTS(DT_CHILD(DT_DRV_INST(n), channel_0)),                         \
		     "channel@0 is obsolete: configure &psi5X_ch0 (ch@0) from the SoC dtsi");      \
	BUILD_ASSERT(DT_NODE_HAS_STATUS(PSI5_CHANNEL_NODE(n), okay),                               \
		     "PSI5 unit is okay but ch@0 is disabled: set &psi5X_ch0 status = okay");      \
	BUILD_ASSERT(!PSI5_RH850_ASYNC(n) ||                                                       \
			     (DT_CHILD_NUM_STATUS_OKAY(PSI5_CHANNEL_NODE(n)) == 1 &&               \
			      DT_NODE_HAS_STATUS(DT_CHILD(PSI5_CHANNEL_NODE(n), slot_0), okay)),   \
		     "async-mode: only slot@0 is allowed");                                        \
	BUILD_ASSERT(PSI5_RH850_TX_FRAME_IDX(n) != PSI5_RH850_TX_FRAME_IDX_NON_STD,                \
		     "tx-frame = non-standard-frame is not supported");                            \
	BUILD_ASSERT(PSI5_RH850_US_TO_COUNTER(DT_PROP_OR(PSI5_CHANNEL_NODE(n), pulse_width_0_us,   \
							 15)) <= PSI5_RH850_SHORTCNT_MAX,          \
		     "pulse-width-0-us is too large");                                             \
	BUILD_ASSERT(PSI5_RH850_US_TO_COUNTER(DT_PROP_OR(PSI5_CHANNEL_NODE(n), pulse_width_1_us,   \
							 30)) <= PSI5_RH850_LONGCNT_MAX,           \
		     "pulse-width-1-us is too large");                                             \
	BUILD_ASSERT(PSI5_RH850_US_TO_COUNTER(DT_PROP_OR(PSI5_CHANNEL_NODE(n),                     \
							 period_sync_pulse_us, 500)) <=            \
			     PSI5_RH850_CNT20_MAX,                                                 \
		     "period-sync-pulse-us is too large");                                         \
	BUILD_ASSERT(DT_PROP_OR(PSI5_CHANNEL_NODE(n), pulse_width_1_us, 30) >                      \
			     DT_PROP_OR(PSI5_CHANNEL_NODE(n), pulse_width_0_us, 15),               \
		     "pulse-width-1-us must be greater than pulse-width-0-us");                    \
	BUILD_ASSERT(DT_PROP_OR(PSI5_CHANNEL_NODE(n), period_sync_pulse_us, 500) >                 \
			     DT_PROP_OR(PSI5_CHANNEL_NODE(n), pulse_width_1_us, 30),               \
		     "period-sync-pulse-us must be greater than pulse-width-1-us");                \
	DT_FOREACH_CHILD_STATUS_OKAY(PSI5_CHANNEL_NODE(n), PSI5_SLOT_ASSERT)                       \
                                                                                                   \
	static psi5_rx_slot_setting_t g_psi5_##n##_rx_slots[PSI5_RH850_MAX_SLOTS] = {              \
		COND_CODE_1(DT_NODE_EXISTS(PSI5_CHANNEL_NODE(n)),                                 \
			    (DT_FOREACH_CHILD_STATUS_OKAY(PSI5_CHANNEL_NODE(n),                    \
							  PSI5_SLOT_CONFIG)),                      \
			    ()) };                                                                 \
                                                                                                   \
	static psi5_extended_cfg_t g_psi5_##n##_cfg_extend = {                                     \
		.tx_setting =                                                                      \
			{                                                                          \
				.sync_pulse_value = PSI5_RH850_TX_DEFTXVAL(n),                     \
				.short_count = (uint16_t)PSI5_RH850_US_TO_COUNTER(                 \
					DT_PROP_OR(PSI5_CHANNEL_NODE(n), pulse_width_0_us, 15)),   \
				.long_count = (uint16_t)PSI5_RH850_US_TO_COUNTER(                  \
					DT_PROP_OR(PSI5_CHANNEL_NODE(n), pulse_width_1_us, 30)),   \
				.physical_layer = PSI5_TX_MODE_PUSLE_WIDTH,                        \
				.frame_format = PSI5_RH850_TX_FRAME_FORMAT(n),                     \
			},                                                                         \
                                                                                                   \
		.rx_setting =                                                                      \
			{                                                                          \
				.rx_sampl_timing =                                                 \
					{                                                          \
						.sampling_interval =                               \
							PSI5_RH850_SMPL_VAL(PSI5_CHANNEL_NODE(n)), \
						.non_data_tgl_detec_start_pos = 0x0B,              \
						.non_data_tgl_detect_end_pos = 0x03,               \
						.tgl_detect_start_pos = 0x13,                      \
						.tgl_detect_end_pos = 0x0C,                        \
					},                                                         \
				.p_rx_slot = g_psi5_##n##_rx_slots,                                \
			},                                                                         \
                                                                                                   \
		.timestamp_setting =                                                               \
			{                                                                          \
				.timestamp_mode = PSI5_TIMESTAMP_MODE_MASTER,                      \
				.ip_timer_enable = PSI5_IP_TIMER_ENABLE,                           \
				.master_select = 0,                                                \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	static psi5_cfg_t g_psi5_##n##_cfg = {                                                     \
		.channel = DT_INST_PROP(n, renesas_unit_id),                                       \
		.sync_period_counter = PSI5_RH850_US_TO_COUNTER(                                   \
			DT_PROP_OR(PSI5_CHANNEL_NODE(n), period_sync_pulse_us, 500)),              \
		.bitrate = PSI5_RH850_BITRATE_VAL(PSI5_CHANNEL_NODE(n)),                           \
		.communication_mode = PSI5_RH850_COMM_MODE(n),                                     \
		.p_extend = &g_psi5_##n##_cfg_extend,                                              \
		.p_callback = psi5_renesas_rh850_fsp_callback,                                     \
		.p_context = (void *)DEVICE_DT_INST_GET(n),                                        \
		.p_transfer_rx = NULL,                                                             \
		.rx_data_exist_irq = DT_INST_IRQ_BY_NAME(n, rx, irq),                              \
		.rx_data_exist_ipl = DT_INST_IRQ_BY_NAME(n, rx, priority),                         \
		.tx_empty_irq = DT_INST_IRQ_BY_NAME(n, tx, irq),                                   \
		.tx_empty_ipl = DT_INST_IRQ_BY_NAME(n, tx, priority),                              \
		.status_irq = DT_INST_IRQ_BY_NAME(n, status, irq),                                 \
		.status_ipl = DT_INST_IRQ_BY_NAME(n, status, priority),                            \
	};                                                                                         \
                                                                                                   \
	static psi5_instance_ctrl_t g_psi5_##n##_ctrl;                                             \
                                                                                                   \
	static const struct psi5_renesas_rh850_config psi5_renesas_rh850_config_##n = {            \
		.pin_config = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                   \
		.fsp_api = &g_psi5_on_psi5,                                                        \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)(uintptr_t)                                \
			DT_INST_CLOCKS_CELL(n, clk_id),                                            \
		.unit_id = DT_INST_PROP(n, renesas_unit_id),                                       \
		.async_mode = PSI5_RH850_ASYNC(n),                                                 \
		.tx_data_mask = PSI5_RH850_TX_DATA_MASK(n),                                        \
	};                                                                                         \
                                                                                                   \
	static struct psi5_renesas_rh850_data psi5_renesas_rh850_data_##n = {                      \
		.fsp_ctrl = &g_psi5_##n##_ctrl,                                                    \
		.fsp_cfg = &g_psi5_##n##_cfg,                                                      \
	};                                                                                         \
                                                                                                   \
	static int psi5_renesas_rh850_init_##n(const struct device *dev)                           \
	{                                                                                          \
		int ret;                                                                           \
                                                                                                   \
		PSI5_RH850_IRQ_CONNECT(n, rx, psi5_renesas_rh850_rx_isr);                          \
		PSI5_RH850_IRQ_CONNECT(n, tx, psi5_renesas_rh850_tx_isr);                          \
		PSI5_RH850_IRQ_CONNECT(n, status, psi5_renesas_rh850_status_isr);                  \
                                                                                                   \
		/* Enable IRQs after FSP open has set the ISR context */                           \
		ret = psi5_renesas_rh850_init(dev);                                                \
		if (ret == 0) {                                                                    \
			PSI5_RH850_IRQ_ENABLE(n, rx);                                              \
			PSI5_RH850_IRQ_ENABLE(n, tx);                                              \
			PSI5_RH850_IRQ_ENABLE(n, status);                                          \
		}                                                                                  \
                                                                                                   \
		return ret;                                                                        \
	}                                                                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &psi5_renesas_rh850_init_##n, NULL, &psi5_renesas_rh850_data_##n, \
			      &psi5_renesas_rh850_config_##n, POST_KERNEL,                         \
			      CONFIG_PSI5_INIT_PRIORITY, &psi5_renesas_rh850_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PSI5_RH850_INIT)
