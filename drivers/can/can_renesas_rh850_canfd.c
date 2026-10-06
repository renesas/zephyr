/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include "r_can_api.h"
#include "r_canfd.h"
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/pm/device.h>

#ifdef CONFIG_RENESAS_RH850_INTIF_ORED
#include <zephyr/drivers/interrupt_controller/intc_rh850_intif_ored.h>
#endif

LOG_MODULE_REGISTER(can_renesas_rh850_canfd, CONFIG_CAN_LOG_LEVEL);

#define DT_DRV_COMPAT renesas_rh850_canfd

#define CAN_RENESAS_RH850_TIMING_MAX                                                               \
	{                                                                                          \
		.sjw = 32,                                                                         \
		.prop_seg = 1,                                                                     \
		.phase_seg1 = 127,                                                                 \
		.phase_seg2 = 32,                                                                  \
		.prescaler = 1024,                                                                 \
	}

#define CAN_RENESAS_RH850_TIMING_MIN                                                               \
	{                                                                                          \
		.sjw = 1,                                                                          \
		.prop_seg = 1,                                                                     \
		.phase_seg1 = 2,                                                                   \
		.phase_seg2 = 2,                                                                   \
		.prescaler = 1,                                                                    \
	}

#ifdef CONFIG_CAN_FD_MODE
#define CAN_RENESAS_RH850_TIMING_DATA_MAX                                                          \
	{                                                                                          \
		.sjw = 16,                                                                         \
		.prop_seg = 1,                                                                     \
		.phase_seg1 = 31,                                                                  \
		.phase_seg2 = 16,                                                                  \
		.prescaler = 256,                                                                  \
	}
#define CAN_RENESAS_RH850_TIMING_DATA_MIN                                                          \
	{                                                                                          \
		.sjw = 1,                                                                          \
		.prop_seg = 1,                                                                     \
		.phase_seg1 = 2,                                                                   \
		.phase_seg2 = 2,                                                                   \
		.prescaler = 1,                                                                    \
	}
#endif

/* This frame ID will be reserved. Any filter using this ID may cause undefined behavior. */
#define CAN_RENESAS_RH850_RESERVED_ID (CAN_EXT_ID_MASK)

#define CAN_RENESAS_RH850_TX_QUEUE_DEPTH      CONFIG_CAN_RENESAS_RH850_TX_QUEUE_DEPTH
#define CAN_RENESAS_RH850_TX_QUEUE            CAN_BUFFER_TX_PRIORITY_0
#define CAN_RENESAS_RH850_TX_LABEL_SLOT_MASK  0x0fU
#define CAN_RENESAS_RH850_TX_LABEL_GENERATION 0x10U

BUILD_ASSERT(CAN_RENESAS_RH850_TX_QUEUE_DEPTH <= CAN_RENESAS_RH850_TX_LABEL_SLOT_MASK + 1U);

/*
 * Queue interrupts identify a queue, not a frame. Use TX history exclusively for
 * completion: it returns the frame label even when CAN ID priority reorders TX.
 * Queue depth is encoded as the number of messages minus one.
 */
#define CAN_RENESAS_RH850_TX_CFG                                                                   \
	.tx_queue_cfg = {[0] = {.tx_queue_cfg_b =                                                  \
					{                                                          \
						.enable = 1U,                                      \
						.depth = CAN_RENESAS_RH850_TX_QUEUE_DEPTH - 1U,    \
					}}},                                                       \
	.tx_history_cfg = {                                                                        \
		.enable = CANFD_THL_DEDICATED_TX,                                                  \
		.isr_mode = CANFD_THL_ISR_MODE_EVERYTIME,                                          \
	}

/*
 * Channel Error IRQ configurations: refer to '30.2.2 Channel n Control Register (CFDCnCTR)'
 */
#define CANFD_CFG_ERR_IRQ                                                                          \
	(BIT(R_CANFD0_CFDC_CTR_EWIE_Pos) |  /* Error Warning Interrupt Enable */                   \
	 BIT(R_CANFD0_CFDC_CTR_EPIE_Pos) |  /* Error Passive Interrupt Enable */                   \
	 BIT(R_CANFD0_CFDC_CTR_BOEIE_Pos) | /* Bus-Off Entry Interrupt Enable */                   \
	 BIT(R_CANFD0_CFDC_CTR_BORIE_Pos) | /* Bus-Off Recovery Interrupt Enable */                \
	 BIT(R_CANFD0_CFDC_CTR_OLIE_Pos))   /* Overload Interrupt Enable */

#define RULE_NUM_PER_PAGE (16)

K_MUTEX_DEFINE(can_renesas_rh850_global_mutex);

#define CANFD_CFG_RXFIFO_DEFAULT(x)                                                                \
	((CANFD_CFG_RXFIFO##x##_INT_THRESHOLD << R_CANFD0_CFDRFCC_RFIGCV_Pos) |                    \
	 (CANFD_CFG_RXFIFO##x##_DEPTH << R_CANFD0_CFDRFCC_RFDC_Pos) |                              \
	 (CANFD_CFG_RXFIFO##x##_PAYLOAD << R_CANFD0_CFDRFCC_RFPLS_Pos) |                           \
	 (CANFD_CFG_RXFIFO##x##_INT_MODE) | (CANFD_CFG_RXFIFO##x##_ENABLE))

/* Default Dataphase bitrate configuration in case classic mode is enabled */
static const can_bit_timing_cfg_t classic_can_data_timing_default = {
	.baud_rate_prescaler = 1,
	.time_segment_1 = 3,
	.time_segment_2 = 2,
	.synchronization_jump_width = 1,
};

struct can_renesas_rh850_filter {
	bool set;
	struct can_filter filter;
	can_rx_callback_t rx_cb;
	void *rx_usr_data;
};

struct can_renesas_rh850_tx_slot {
	can_tx_callback_t callback;
	void *user_data;
	uint16_t label;
	bool allocated;
};

struct can_renesas_rh850_global_cfg {
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_configure)(void);
};

struct can_renesas_rh850_cfg {
	struct can_driver_config common;
	const struct pinctrl_dev_config *pin_config;
	const struct device *global_dev;
	const uint32_t rx_filter_num;
	const can_api_t *fsp_api;
	uint32_t clock_ext_rate;
};

struct can_renesas_rh850_data {
	struct can_driver_data common;
	struct k_mutex inst_mutex;
	struct k_spinlock tx_lock;
	struct k_sem tx_sem;
	bool txq_disabled;
	struct can_renesas_rh850_tx_slot tx_slots[CAN_RENESAS_RH850_TX_QUEUE_DEPTH];
	struct can_renesas_rh850_filter *rx_filter;
	can_bit_timing_cfg_t data_timing;
#ifdef CONFIG_RENESAS_RH850_INTIF_ORED
	/* ORed controller for TXI interrupt */
	const struct device *txi_ctrl;
	/* ORed controller for RXI interrupt */
	const struct device *rxi_ctrl;
	/* ORed controller for ERI interrupt */
	const struct device *eri_ctrl;
#endif
	canfd_instance_ctrl_t *fsp_ctrl;
	can_cfg_t *fsp_cfg;
};

extern BSP_INTERRUPT_ATTRIBUTE void canfd_channel_error_isr(void);
extern BSP_INTERRUPT_ATTRIBUTE void canfd_global_error_isr(void);
extern BSP_INTERRUPT_ATTRIBUTE void canfd_rx_fifo_isr(void);
extern BSP_INTERRUPT_ATTRIBUTE void canfd_rx_common_fifo_isr(void);
extern BSP_INTERRUPT_ATTRIBUTE void canfd_channel_tx_isr(void);

#ifdef CONFIG_RENESAS_RH850_INTIF_ORED
static void canfd_rh850_txi_isr_cb(void *arg);
static void canfd_rh850_rxi_isr_cb(void *arg);
static void canfd_rh850_eri_isr_cb(void *arg);
static int canfd_rh850_irq_init(const struct device *dev);
#endif

static inline int can_renesas_rh850_apply_default_config(const struct device *dev);

/* Convert FSP frame format to zephyr frame */
static void can_renesas_rh850_frame_r2z(can_frame_t *p_fsp_frame, struct can_frame *p_zephyr_frame)
{
	p_zephyr_frame->dlc = can_bytes_to_dlc(p_fsp_frame->data_length_code),
	p_zephyr_frame->id = p_fsp_frame->id,
	p_zephyr_frame->flags =
		(((p_fsp_frame->id_mode == CAN_ID_MODE_EXTENDED) ? CAN_FRAME_IDE : 0UL) |
		 ((p_fsp_frame->type == CAN_FRAME_TYPE_REMOTE) ? CAN_FRAME_RTR : 0UL) |
		 ((p_fsp_frame->format & CAN_FRAME_FORMAT_FD) != 0 ? CAN_FRAME_FDF : 0UL) |
		 ((p_fsp_frame->options & CAN_FRAME_OPTIONS_BRS) != 0 ? CAN_FRAME_BRS : 0UL) |
		 ((p_fsp_frame->options & CAN_FRAME_OPTIONS_ESI) != 0 ? CAN_FRAME_ESI : 0UL));

	memcpy(p_zephyr_frame->data, p_fsp_frame->data,
	       MIN(p_fsp_frame->data_length_code, sizeof(p_zephyr_frame->data)));
}
/* Convert zephyr frame to renesas FSP frame format*/
static void can_renesas_rh850_frame_z2r(const struct can_frame *p_zephyr_frame,
					can_frame_t *p_fsp_frame)
{

	p_fsp_frame->id = p_zephyr_frame->id,
	p_fsp_frame->id_mode = ((p_zephyr_frame->flags & CAN_FRAME_IDE) != 0)
				       ? CAN_ID_MODE_EXTENDED
				       : CAN_ID_MODE_STANDARD,
	p_fsp_frame->type = ((p_zephyr_frame->flags & CAN_FRAME_RTR) != 0) ? CAN_FRAME_TYPE_REMOTE
									   : CAN_FRAME_TYPE_DATA,
	p_fsp_frame->data_length_code = can_dlc_to_bytes(p_zephyr_frame->dlc),
	p_fsp_frame->format =
		(((p_zephyr_frame->flags & CAN_FRAME_FDF) != 0) ? CAN_FRAME_FORMAT_FD : 0UL),
	p_fsp_frame->options =
		((((p_zephyr_frame->flags & CAN_FRAME_BRS) != 0) ? CAN_FRAME_OPTIONS_BRS : 0UL) |
		 (((p_zephyr_frame->flags & CAN_FRAME_ESI) != 0) ? CAN_FRAME_OPTIONS_ESI : 0UL)),

	memcpy(p_fsp_frame->data, p_zephyr_frame->data, p_fsp_frame->data_length_code);
}

static inline int set_hw_timing_configuration(const struct device *dev,
					      can_bit_timing_cfg_t *f_timing,
					      const struct can_timing *z_timing)
{
	struct can_renesas_rh850_data *data = dev->data;

	if (f_timing == NULL || z_timing == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	*f_timing = (can_bit_timing_cfg_t){
		.baud_rate_prescaler = z_timing->prescaler,
		.time_segment_1 = z_timing->prop_seg + z_timing->phase_seg1,
		.time_segment_2 = z_timing->phase_seg2,
		.synchronization_jump_width = z_timing->sjw,
	};

	k_mutex_unlock(&data->inst_mutex);

	return 0;
}

static inline int get_free_filter_id(const struct device *dev)
{
	struct can_renesas_rh850_data *data = dev->data;
	const struct can_renesas_rh850_cfg *cfg = dev->config;
	int ret = 0;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	for (uint32_t filter_id = 0; filter_id < cfg->rx_filter_num; filter_id++) {
		if (!data->rx_filter[filter_id].set) {
			ret = filter_id;
			k_mutex_unlock(&data->inst_mutex);
			return ret;
		}
	}

	/* there are no free filter */
	ret = -ENOSPC;
	k_mutex_unlock(&data->inst_mutex);
	return ret;
}

static uint32_t can_renesas_rh850_get_global_afl_offset(const struct device *dev,
							uint32_t local_offset)
{
	struct can_renesas_rh850_data *data = dev->data;
	const canfd_extended_cfg_t *extend = data->fsp_cfg->p_extend;
	const canfd_global_cfg_t *global_cfg = extend->p_global_cfg;
	uint32_t channel_index = data->fsp_cfg->channel % CANFD_MAX_CHANNEL_PER_UNIT;
	uint32_t global_offset = local_offset;

	for (uint32_t channel = 0; channel < channel_index; channel++) {
		global_offset += global_cfg->afl_rule_num[channel];
	}

	return global_offset;
}

static void set_afl_rule(const struct device *dev, const struct can_filter *filter,
			 uint32_t afl_offset)
{
	struct can_renesas_rh850_data *data = dev->data;
	canfd_extended_cfg_t const *p_extend = data->fsp_cfg->p_extend;
	uint32_t channel_index = data->fsp_cfg->channel % CANFD_MAX_CHANNEL_PER_UNIT;
	uint32_t global_offset = can_renesas_rh850_get_global_afl_offset(dev, afl_offset);
	canfd_afl_entry_t *afl = (canfd_afl_entry_t *)&p_extend->p_afl[afl_offset];

	*afl = (canfd_afl_entry_t){
		.id.id_b = {.id = filter->id,
#ifndef CONFIG_CAN_ACCEPT_RTR
			    .frame_type = CAN_FRAME_TYPE_DATA,
#endif
			    .id_mode = ((filter->flags & CAN_FILTER_IDE) != 0)
					       ? CAN_ID_MODE_EXTENDED
					       : CAN_ID_MODE_STANDARD},
		.mask.mask_b = {.mask_id = filter->mask,
#ifdef CONFIG_CAN_ACCEPT_RTR
				/* Accept all types of frames */
				.mask_frame_type = 0,
#else
				/* Only accept frames with the configured frametype */
				.mask_frame_type = 1,
#endif
				.mask_id_mode = ((filter->flags & CAN_FILTER_IDE) != 0)
							? CAN_ID_MODE_EXTENDED
							: CAN_ID_MODE_STANDARD},
		.destination.destination_b = {
			.rx_select_flags = CANFD_RX_TARGET_FIFO_0 << channel_index,
			.rx_buffer = CANFD_RX_MB_0,
		}};

	if (data->common.started) {
		/* These steps help update AFL rules while CAN module running */
		canfd_instance_ctrl_t *p_ctrl = (canfd_instance_ctrl_t *)data->fsp_ctrl;
		R_CANFD0_Type *p_reg = p_extend->p_global_cfg->p_canfd_reg;

		k_mutex_lock(&can_renesas_rh850_global_mutex, K_FOREVER);

		/* Unlock AFL entry access */
		p_reg->CFDGAFLECTR_b.AFLDAE = true;

		/** Select the AFL page containing this channel's rule. */
		p_reg->CFDGAFLECTR_b.AFLPN = global_offset / RULE_NUM_PER_PAGE;

		p_reg->CFDGAFL[global_offset % RULE_NUM_PER_PAGE] = *(R_CANFD0_CFDGAFL_Type *)afl;

		/* Lock AFL entry access */
		p_reg->CFDGAFLECTR_b.AFLDAE = false;
		k_mutex_unlock(&can_renesas_rh850_global_mutex);
	}
}

static void remove_afl_rule(const struct device *dev, uint32_t afl_offset)
{
	struct can_renesas_rh850_data *data = dev->data;
	uint32_t global_offset = can_renesas_rh850_get_global_afl_offset(dev, afl_offset);
	canfd_extended_cfg_t const *p_extend = data->fsp_cfg->p_extend;
	canfd_afl_entry_t *afl = (canfd_afl_entry_t *)&p_extend->p_afl[afl_offset];

	/* Set the AFL ID to reserved ID */
	*afl = (canfd_afl_entry_t){
		.id.id_b = {.id = CAN_RENESAS_RH850_RESERVED_ID, .id_mode = CAN_ID_MODE_EXTENDED},
		.mask.mask_b = {.mask_id = CAN_RENESAS_RH850_RESERVED_ID,
				.mask_id_mode = CAN_ID_MODE_EXTENDED}};

	if (data->common.started) {
		/* These steps help update AFL rules while CAN module running */
		canfd_instance_ctrl_t *p_ctrl = (canfd_instance_ctrl_t *)data->fsp_ctrl;
		R_CANFD0_Type *p_reg = p_extend->p_global_cfg->p_canfd_reg;

		k_mutex_lock(&can_renesas_rh850_global_mutex, K_FOREVER);

		/* Unlock AFL entry access */
		p_reg->CFDGAFLECTR_b.AFLDAE = true;

		/** Select the AFL page containing this channel's rule. */
		p_reg->CFDGAFLECTR_b.AFLPN = global_offset / RULE_NUM_PER_PAGE;

		/* Change configurations for AFL entry */
		p_reg->CFDGAFL[global_offset % RULE_NUM_PER_PAGE] = *(R_CANFD0_CFDGAFL_Type *)(afl);
		/* Lock AFL entry access */
		p_reg->CFDGAFLECTR_b.AFLDAE = false;
		k_mutex_unlock(&can_renesas_rh850_global_mutex);
	}
}

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
static int recover_bus(const struct device *dev, k_timeout_t timeout)
{
	struct can_renesas_rh850_data *data = dev->data;
	canfd_extended_cfg_t const *p_extend = data->fsp_cfg->p_extend;
	canfd_instance_ctrl_t *p_ctrl = data->fsp_ctrl;
	R_CANFD0_Type *p_reg = p_extend->p_global_cfg->p_canfd_reg;
	uint32_t channel_index = data->fsp_ctrl->channel_index;

	uint32_t cfdcnctr = p_reg->CFDC[channel_index].CTR;
	int ret = 0;

	if (p_reg->CFDC[channel_index].ERFL_b.BOEF == 0) {
		/* Channel bus-off entry not detected */
		return 0;
	}

	p_reg->CFDC[channel_index].CTR_b.BOM =
		0x00; /* Switch to Normal Bus-Off mode (comply with ISO 11898-1) */
	p_reg->CFDC[channel_index].CTR_b.RTBO = 1; /* Force channel state to return from bus-off */

	int64_t start_ticks = k_uptime_ticks();

	while (p_reg->CFDC[channel_index].ERFL_b.BORF == 0) {
		if ((k_uptime_ticks() - start_ticks) > timeout.ticks) {
			ret = -EAGAIN;
			break;
		}
	}
	p_reg->CFDC[channel_index].CTR = cfdcnctr; /* Restore channel configuration */

	return ret;
}
#endif

static int can_renesas_rh850_alloc_tx_slot(const struct device *dev, can_tx_callback_t callback,
					   void *user_data)
{
	struct can_renesas_rh850_data *data = dev->data;
	/* The caller holds tx_lock until the frame has been submitted to the HAL. */
	int slot = -ENOSPC;

	for (uint32_t i = 0U; i < CAN_RENESAS_RH850_TX_QUEUE_DEPTH; i++) {
		if (!data->tx_slots[i].allocated) {
			/* Keep the slot in the low bits and advance its generation on reuse. */
			data->tx_slots[i].label =
				(uint16_t)(((data->tx_slots[i].label +
					     CAN_RENESAS_RH850_TX_LABEL_GENERATION) &
					    ~CAN_RENESAS_RH850_TX_LABEL_SLOT_MASK) |
					   i);
			data->tx_slots[i].callback = callback;
			data->tx_slots[i].user_data = user_data;
			data->tx_slots[i].allocated = true;
			slot = (int)i;
			break;
		}
	}

	return slot;
}

static bool can_renesas_rh850_release_tx_slot(const struct device *dev, uint32_t slot,
					      uint32_t label, can_tx_callback_t *callback,
					      void **user_data)
{
	struct can_renesas_rh850_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->tx_lock);
	bool allocated = false;

	if ((slot < CAN_RENESAS_RH850_TX_QUEUE_DEPTH) && data->tx_slots[slot].allocated &&
	    data->tx_slots[slot].label == label) {
		if (callback != NULL) {
			*callback = data->tx_slots[slot].callback;
		}
		if (user_data != NULL) {
			*user_data = data->tx_slots[slot].user_data;
		}
		data->tx_slots[slot].callback = NULL;
		data->tx_slots[slot].user_data = NULL;
		data->tx_slots[slot].allocated = false;
		allocated = true;
	}

	k_spin_unlock(&data->tx_lock, key);

	return allocated;
}

static void can_renesas_rh850_call_tx_cb(const struct device *dev, const can_callback_args_t *args)
{
	struct can_renesas_rh850_data *data = dev->data;
	can_tx_callback_t callback = NULL;
	void *user_data = NULL;
	uint32_t slot = args->label & CAN_RENESAS_RH850_TX_LABEL_SLOT_MASK;

	if (args->buffer != CAN_RENESAS_RH850_TX_QUEUE) {
		return;
	}

	if (!can_renesas_rh850_release_tx_slot(dev, slot, args->label, &callback, &user_data)) {
		return;
	}

	k_sem_give(&data->tx_sem);
	if (callback != NULL) {
		callback(dev, 0, user_data);
	}
}

static uint32_t can_renesas_rh850_release_all_tx(
	const struct device *dev,
	struct can_renesas_rh850_tx_slot released[CAN_RENESAS_RH850_TX_QUEUE_DEPTH])
{
	struct can_renesas_rh850_data *data = dev->data;
	const canfd_extended_cfg_t *extend = data->fsp_cfg->p_extend;
	R_CANFD0_Type *regs = extend->p_global_cfg->p_canfd_reg;
	uint32_t channel_index = data->fsp_ctrl->channel_index;
	uint32_t released_count = 0U;
	k_spinlock_key_t key = k_spin_lock(&data->tx_lock);

	/*
	 * Disabling TXQE aborts every queued frame. Do not enable it again until
	 * TXQEMP is set. Keep history enabled so any completion racing the abort
	 * is drained by the HAL; the saved label generation rejects stale events.
	 */
	regs->CFDTXQ[0].CC_b[channel_index].TXQE = 0U;
	data->txq_disabled = true;
	for (uint32_t slot = 0U; slot < CAN_RENESAS_RH850_TX_QUEUE_DEPTH; slot++) {
		if (data->tx_slots[slot].allocated) {
			released[released_count++] = data->tx_slots[slot];
			data->tx_slots[slot].allocated = false;
			data->tx_slots[slot].callback = NULL;
			data->tx_slots[slot].user_data = NULL;
		}
	}
	k_spin_unlock(&data->tx_lock, key);

	for (uint32_t slot = 0U; slot < released_count; slot++) {
		k_sem_give(&data->tx_sem);
	}

	return released_count;
}

static void can_renesas_rh850_call_released_tx(
	const struct device *dev,
	const struct can_renesas_rh850_tx_slot released[CAN_RENESAS_RH850_TX_QUEUE_DEPTH],
	uint32_t released_count, int err)
{
	for (uint32_t slot = 0U; slot < released_count; slot++) {
		if (released[slot].callback != NULL) {
			released[slot].callback(dev, err, released[slot].user_data);
		}
	}
}

static void can_renesas_rh850_abort_all_tx(const struct device *dev, int err)
{
	struct can_renesas_rh850_tx_slot released[CAN_RENESAS_RH850_TX_QUEUE_DEPTH];
	uint32_t released_count = can_renesas_rh850_release_all_tx(dev, released);

	can_renesas_rh850_call_released_tx(dev, released, released_count, err);
}

static bool can_renesas_rh850_rx_filters_exist(const struct device *dev)
{
	const struct can_renesas_rh850_cfg *cfg = dev->config;
	struct can_renesas_rh850_data *data = dev->data;

	for (uint32_t filter_id = 0; filter_id < cfg->rx_filter_num; filter_id++) {
		if (data->rx_filter[filter_id].set) {
			return true;
		}
	}

	return false;
}

static int can_renesas_rh850_pm_control(const struct device *dev, enum pm_device_action action)
{
	const struct can_renesas_rh850_cfg *cfg = dev->config;
	struct can_renesas_rh850_data *data = dev->data;
	can_operation_mode_t operation_mode;
	int ret = 0;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		if (pm_device_is_busy(dev)) {
			LOG_DBG("Cannot suspend while device is busy");
			ret = -EBUSY;
			goto unlock;
		}
		operation_mode = CAN_OPERATION_MODE_SLEEP;
		break;
	case PM_DEVICE_ACTION_RESUME:
		operation_mode = CAN_OPERATION_MODE_HALT;
		break;
	default:
		ret = -ENOTSUP;
		goto unlock;
	}

	if (cfg->fsp_api->modeTransition(data->fsp_ctrl, operation_mode,
					 data->fsp_ctrl->test_mode) != FSP_SUCCESS) {
		LOG_ERR("CAN power state transition failed");
		ret = -EIO;
	}

unlock:
	k_mutex_unlock(&data->inst_mutex);
	return ret;
}

static inline void can_renesas_rh850_call_rx_cb(const struct device *dev,
						can_callback_args_t *p_args)
{
	struct can_renesas_rh850_data *data = dev->data;
	struct can_renesas_rh850_filter *rx_filter = data->rx_filter;
	const struct can_renesas_rh850_cfg *cfg = dev->config;
	can_frame_t fsp_frame = {0};
	struct can_frame frame = {0};

	if (FSP_SUCCESS != cfg->fsp_api->read(data->fsp_ctrl, p_args->buffer, &fsp_frame)) {
		LOG_DBG("CAN receive failed");
		return;
	}

	/* Convert data structure from fsp can_frame_t to zephyr can_frame*/
	can_renesas_rh850_frame_r2z(&fsp_frame, &frame);

	for (uint32_t i = 0; i < cfg->rx_filter_num; i++) {
		can_rx_callback_t cb = rx_filter->rx_cb;
		void *usr_data = rx_filter->rx_usr_data;

		if (cb == NULL) {
			rx_filter++;
			continue; /* this filter has not set yet */
		}

		if (!can_frame_matches_filter(&frame, &rx_filter->filter)) {
			rx_filter++;
			continue; /* filter did not match */
		}

		cb(dev, &frame, usr_data);
		break;
	}
}

static inline void can_renesas_rh850_call_state_change_cb(const struct device *dev,
							  enum can_state state)
{
	struct can_renesas_rh850_data *data = dev->data;
	const struct can_renesas_rh850_cfg *cfg = dev->config;
	can_info_t can_info;

	if (FSP_SUCCESS != cfg->fsp_api->infoGet(data->fsp_ctrl, &can_info)) {
		LOG_DBG("get state info failed");
		return;
	}

	struct can_bus_err_cnt err_cnt = {
		.rx_err_cnt = can_info.error_count_receive,
		.tx_err_cnt = can_info.error_count_transmit,
	};

	if (data->common.state_change_cb != NULL) {
		data->common.state_change_cb(dev, state, err_cnt,
					     data->common.state_change_cb_user_data);
	}
}

static int can_renesas_rh850_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);
	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK;

#ifdef CONFIG_CAN_FD_MODE
	*cap |= CAN_MODE_FD;
#endif

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
	*cap |= CAN_MODE_MANUAL_RECOVERY;
#endif

	return 0;
}

static int can_renesas_rh850_start(const struct device *dev)
{
	struct can_renesas_rh850_data *data = dev->data;
	const struct can_renesas_rh850_cfg *cfg = dev->config;

	const struct device *transceiver_dev = can_get_transceiver(dev);
	int ret = 0;

	if (!device_is_ready(dev)) {
		return -EIO;
	}

	if (data->common.started) {
		return -EALREADY;
	}

	if ((transceiver_dev != NULL) &&
	    can_transceiver_enable(transceiver_dev, data->common.mode)) {
		LOG_DBG("CAN transceiver enable failed");
		return -EIO;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);
	canfd_extended_cfg_t *p_extend = (canfd_extended_cfg_t *)data->fsp_cfg->p_extend;

	p_extend->p_data_timing =
		(can_bit_timing_cfg_t *)((data->common.mode & CAN_MODE_FD) != 0
						 ? &data->data_timing
						 : &classic_can_data_timing_default);
	/* Serialize shared unit state and AFL access with other channels. */
	k_mutex_lock(&can_renesas_rh850_global_mutex, K_FOREVER);

	if (FSP_SUCCESS != cfg->fsp_api->close(data->fsp_ctrl)) {
		LOG_DBG("CAN close failed");
		ret = -EIO;
		goto end;
	}

	if (FSP_SUCCESS != cfg->fsp_api->open(data->fsp_ctrl, data->fsp_cfg)) {
		LOG_DBG("CAN open failed");
		ret = -EIO;
		goto end;
	}

	if ((data->common.mode & CAN_MODE_LOOPBACK) != 0) {
		if (FSP_SUCCESS != cfg->fsp_api->modeTransition(data->fsp_ctrl,
								CAN_OPERATION_MODE_NORMAL,
								CAN_TEST_MODE_LOOPBACK_INTERNAL)) {
			LOG_DBG("CAN mode change failed");
			ret = -EIO;
			goto end;
		}
	}

	data->txq_disabled = false;
	data->common.started = true;
	pm_device_busy_set(dev);
end:
	k_mutex_unlock(&can_renesas_rh850_global_mutex);
	k_mutex_unlock(&data->inst_mutex);
	return ret;
}

static int can_renesas_rh850_stop(const struct device *dev)
{
	struct can_renesas_rh850_data *data = dev->data;
	const struct can_renesas_rh850_cfg *cfg = dev->config;
	const struct device *transceiver_dev = can_get_transceiver(dev);
	struct can_renesas_rh850_tx_slot released[CAN_RENESAS_RH850_TX_QUEUE_DEPTH];
	uint32_t released_count = 0U;
	int ret = 0;

	/** START body of stop function  */
	if (!data->common.started) {
		return -EALREADY;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if (FSP_SUCCESS != cfg->fsp_api->modeTransition(data->fsp_ctrl, CAN_OPERATION_MODE_HALT,
							CAN_TEST_MODE_DISABLED)) {
		LOG_DBG("CAN stop failed");
		ret = -EIO;
		goto end;
	}

	if (((transceiver_dev != NULL) && can_transceiver_disable(transceiver_dev))) {
		LOG_DBG("CAN transceiver disable failed");
		ret = -EIO;
		goto end;
	}

	data->common.started = false;
	released_count = can_renesas_rh850_release_all_tx(dev, released);
	if (!can_renesas_rh850_rx_filters_exist(dev)) {
		pm_device_busy_clear(dev);
	}

end:
	k_mutex_unlock(&data->inst_mutex);
	can_renesas_rh850_call_released_tx(dev, released, released_count, -ENETDOWN);
	return ret;
}

static int can_renesas_rh850_set_mode(const struct device *dev, can_mode_t mode)
{
	struct can_renesas_rh850_data *data = dev->data;
	int ret = 0;

	if (data->common.started) {
		/* CAN controller should be in stopped state */
		return -EBUSY;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	can_mode_t caps = 0;

	ret = can_renesas_rh850_get_capabilities(dev, &caps);
	if (ret != 0) {
		goto end;
	}

	if ((mode & ~caps) != 0) {
		ret = -ENOTSUP;
		goto end;
	}

	data->common.mode = mode;

end:
	k_mutex_unlock(&data->inst_mutex);
	return ret;
}

static int can_renesas_rh850_set_timing(const struct device *dev, const struct can_timing *timing)
{
	struct can_renesas_rh850_data *data = dev->data;

	if (data->common.started) {
		/* Device is not in stopped state */
		return -EBUSY;
	}

	return set_hw_timing_configuration(dev, data->fsp_cfg->p_bit_timing, timing);
}

static int can_renesas_rh850_send(const struct device *dev, const struct can_frame *frame,
				  k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	struct can_renesas_rh850_data *data = dev->data;
	const struct can_renesas_rh850_cfg *cfg = dev->config;
	can_frame_t fsp_frame = {0};

	if (!data->common.started) {
		return -ENETDOWN;
	}

	if ((data->common.mode & CAN_MODE_FD) != 0) {
		if (frame->dlc > CANFD_MAX_DLC) {
			return -EINVAL;
		}
		if ((frame->flags & CAN_FRAME_ESI) != 0) {
			return -ENOTSUP;
		}
	} else {
		if ((frame->flags & CAN_FRAME_FDF) != 0) {
			return -ENOTSUP;
		}
		if (frame->dlc > CAN_MAX_DLC) {
			return -EINVAL;
		}
	}

	if (k_sem_take(&data->tx_sem, timeout) != 0) {
		return -EAGAIN;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);
	int ret = 0;

	if (!data->common.started) {
		k_sem_give(&data->tx_sem);
		ret = -ENETDOWN;
		goto unlock;
	}

	const canfd_extended_cfg_t *extend = data->fsp_cfg->p_extend;
	R_CANFD0_Type *regs = extend->p_global_cfg->p_canfd_reg;
	uint32_t channel_index = data->fsp_ctrl->channel_index;
	k_spinlock_key_t key = k_spin_lock(&data->tx_lock);

	if (regs->CFDC[channel_index].STS_b.BOSTS != 0U) {
		ret = -ENETUNREACH;
		goto release_sem;
	}

	if (data->txq_disabled) {
		if (regs->CFDTXQ[0].STS_b[channel_index].TXQEMP == 0U) {
			ret = -EAGAIN;
			goto release_sem;
		}
		regs->CFDTXQ[0].CC_b[channel_index].TXQE = 1U;
		data->txq_disabled = false;
	}

	int tx_slot = can_renesas_rh850_alloc_tx_slot(dev, callback, user_data);

	if (tx_slot < 0) {
		ret = -EIO;
		goto release_sem;
	}

	can_renesas_rh850_frame_z2r(frame, &fsp_frame);
	fsp_frame.labels = data->tx_slots[tx_slot].label;

	/* Serialize label allocation, the queue window write and the pointer push. */
	fsp_err_t err = cfg->fsp_api->write(data->fsp_ctrl, CAN_RENESAS_RH850_TX_QUEUE, &fsp_frame);

	if (err != FSP_SUCCESS) {
		data->tx_slots[tx_slot].allocated = false;
		data->tx_slots[tx_slot].callback = NULL;
		data->tx_slots[tx_slot].user_data = NULL;
		ret = (err == FSP_ERR_CAN_TRANSMIT_NOT_READY) ? -EAGAIN : -EIO;
		goto release_sem;
	}
	k_spin_unlock(&data->tx_lock, key);
	goto unlock;

release_sem:
	k_spin_unlock(&data->tx_lock, key);
	k_sem_give(&data->tx_sem);

unlock:
	k_mutex_unlock(&data->inst_mutex);
	return ret;
}

static int can_renesas_rh850_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
					   void *user_data, const struct can_filter *filter)
{
	struct can_renesas_rh850_data *data = dev->data;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	int filter_id = get_free_filter_id(dev);

	if (filter_id == -ENOSPC) {
		goto end;
	}

	set_afl_rule(dev, filter, filter_id);

	memcpy(&data->rx_filter[filter_id].filter, filter, sizeof(struct can_filter));
	data->rx_filter[filter_id].rx_cb = callback;
	data->rx_filter[filter_id].rx_usr_data = user_data;
	data->rx_filter[filter_id].set = true;
	pm_device_busy_set(dev);

end:
	k_mutex_unlock(&data->inst_mutex);

	return filter_id;
}

static void can_renesas_rh850_remove_rx_filter(const struct device *dev, int filter_id)
{
	struct can_renesas_rh850_data *data = dev->data;
	const struct can_renesas_rh850_cfg *cfg = dev->config;

	if ((filter_id < 0) || (filter_id >= cfg->rx_filter_num)) {
		return;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	remove_afl_rule(dev, filter_id);

	data->rx_filter[filter_id].rx_cb = NULL;
	data->rx_filter[filter_id].rx_usr_data = NULL;
	data->rx_filter[filter_id].set = false;
	if (!data->common.started && !can_renesas_rh850_rx_filters_exist(dev)) {
		pm_device_busy_clear(dev);
	}

	k_mutex_unlock(&data->inst_mutex);
}

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
static int can_renesas_rh850_recover(const struct device *dev, k_timeout_t timeout)
{
	struct can_renesas_rh850_data *data = dev->data;

	if (!data->common.started) {
		return -ENETDOWN;
	}

	if ((data->common.mode & CAN_MODE_MANUAL_RECOVERY) == 0) {
		return -ENOTSUP;
	}

	return recover_bus(dev, timeout);
}
#endif

static int can_renesas_rh850_get_state(const struct device *dev, enum can_state *state,
				       struct can_bus_err_cnt *err_cnt)
{
	struct can_renesas_rh850_data *data = dev->data;
	const struct can_renesas_rh850_cfg *cfg = dev->config;

	if (!data->common.started) {
		if (state != NULL) {
			*state = CAN_STATE_STOPPED;
		}
	} else {
		can_info_t fsp_info = {0};

		if (FSP_SUCCESS != cfg->fsp_api->infoGet(data->fsp_ctrl, &fsp_info)) {
			LOG_DBG("CAN get state info failed");
			return -EIO;
		}

		if (err_cnt != NULL) {
			err_cnt->tx_err_cnt = fsp_info.error_count_transmit;
			err_cnt->rx_err_cnt = fsp_info.error_count_receive;
		}

		if (state != NULL) {
			if ((fsp_info.status & CANFD_STATUS_BUS_OFF) != 0) {
				*state = CAN_STATE_BUS_OFF;
			} else if ((fsp_info.status & CANFD_STATUS_ERROR_PASSIVE) != 0) {
				*state = CAN_STATE_ERROR_PASSIVE;
			} else if (fsp_info.error_count_transmit >= 96U ||
				   fsp_info.error_count_receive >= 96U) {
				*state = CAN_STATE_ERROR_WARNING;
			} else {
				*state = CAN_STATE_ERROR_ACTIVE;
			}
		}
	}

	return 0;
}

static void can_renesas_rh850_set_state_change_callback(const struct device *dev,
							can_state_change_callback_t callback,
							void *user_data)
{
	struct can_renesas_rh850_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->tx_lock);

	data->common.state_change_cb = callback;
	data->common.state_change_cb_user_data = user_data;

	k_spin_unlock(&data->tx_lock, key);
}

static int can_renesas_rh850_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct can_renesas_rh850_cfg *cfg = dev->config;
	const struct can_renesas_rh850_global_cfg *global_cfg = cfg->global_dev->config;
	int ret;

	/* Match the external clock selected by the global DCS bit. */
	if (cfg->clock_ext_rate != 0U) {
		*rate = cfg->clock_ext_rate;
		return 0;
	}

	ret = clock_control_get_rate(global_cfg->clock_dev, global_cfg->clock_subsys, rate);
	if (ret < 0) {
		return ret;
	}

	return (*rate != 0U) ? 0 : -EINVAL;
}

static int can_renesas_rh850_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(ide);
	const struct can_renesas_rh850_cfg *cfg = dev->config;

	return cfg->rx_filter_num;
}

#ifdef CONFIG_CAN_FD_MODE
static int can_renesas_rh850_set_timing_data(const struct device *dev,
					     const struct can_timing *timing_data)
{
	struct can_renesas_rh850_data *data = dev->data;

	if (data->common.started) {
		return -EBUSY;
	}

	return set_hw_timing_configuration(dev, &data->data_timing, timing_data);
}
#endif /* CONFIG_CAN_FD_MODE */

void can_renesas_rh850_fsp_cb(can_callback_args_t *p_args)
{
	const struct device *dev = p_args->p_context;

	switch (p_args->event) {
	case CAN_EVENT_RX_COMPLETE: {
		can_renesas_rh850_call_rx_cb(dev, p_args);
		break;
	}

	case CAN_EVENT_TX_COMPLETE:
	case CAN_EVENT_TX_QUEUE_EMPTY: {
		/* Both events carry one completed frame's label when using TX history. */
		can_renesas_rh850_call_tx_cb(dev, p_args);
		break;
	}

	case CAN_EVENT_ERR_CHANNEL: {
		if ((p_args->error & R_CANFD0_CFDC_ERFL_BEF_Msk) != 0) {
			can_renesas_rh850_call_state_change_cb(dev, CAN_STATE_ERROR_ACTIVE);
		}
		if ((p_args->error & R_CANFD0_CFDC_ERFL_EWF_Msk) != 0) {
			can_renesas_rh850_call_state_change_cb(dev, CAN_STATE_ERROR_WARNING);
		}
		if ((p_args->error & R_CANFD0_CFDC_ERFL_EPF_Msk) != 0) {
			can_renesas_rh850_call_state_change_cb(dev, CAN_STATE_ERROR_PASSIVE);
		}
		if ((p_args->error & R_CANFD0_CFDC_ERFL_BOEF_Msk) != 0) {
			can_renesas_rh850_abort_all_tx(dev, -ENETUNREACH);
			can_renesas_rh850_call_state_change_cb(dev, CAN_STATE_BUS_OFF);
		}

		/*
		 * Arbitration, ACK and bit errors may be followed by
		 * retransmission.
		 * Only TX history identifies individual
		 * successfully transmitted frames.
		 */
		break;
	}

	default:
		break;
	}
}

static inline int can_renesas_rh850_apply_default_config(const struct device *dev)
{
	struct can_renesas_rh850_cfg *cfg = (struct can_renesas_rh850_cfg *)dev->config;
	int ret;
	struct can_timing timing = {0};

	ret = can_calc_timing(dev, &timing, cfg->common.bitrate, cfg->common.sample_point);

	if (ret < 0) {
		return ret;
	}

	ret = can_renesas_rh850_set_timing(dev, &timing);
	if (ret != 0) {
		return ret;
	}
#ifdef CONFIG_CAN_FD_MODE
	ret = can_calc_timing_data(dev, &timing, cfg->common.bitrate_data,
				   cfg->common.sample_point_data);
	if (ret < 0) {
		return ret;
	}

	ret = can_renesas_rh850_set_timing_data(dev, &timing);
	if (ret < 0) {
		return ret;
	}
#endif
	for (uint32_t filter_id = 0; filter_id < cfg->rx_filter_num; filter_id++) {
		remove_afl_rule(dev, filter_id);
	}

	return 0;
}

static int can_renesas_rh850_init(const struct device *dev)
{
	const struct can_renesas_rh850_cfg *cfg = dev->config;
	struct can_renesas_rh850_data *data = dev->data;
	canfd_extended_cfg_t *p_extend = (canfd_extended_cfg_t *)data->fsp_cfg->p_extend;
	int ret = 0;

	k_mutex_init(&data->inst_mutex);
	k_sem_init(&data->tx_sem, CAN_RENESAS_RH850_TX_QUEUE_DEPTH,
		   CAN_RENESAS_RH850_TX_QUEUE_DEPTH);
	data->common.started = false;

	/* Check external clock is used */
	if (cfg->clock_ext_rate) {
		p_extend->p_global_cfg->global_config |= R_CANFD0_CFDGCFG_DCS_Msk;
	}

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(cfg->pin_config, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("pin function initial failed");
		return ret;
	}

	/* Apply config and setting for CAN controller HW */
	ret = can_renesas_rh850_apply_default_config(dev);
	if (ret) {
		LOG_ERR("invalid default configuration");
		return ret;
	}

	k_mutex_lock(&can_renesas_rh850_global_mutex, K_FOREVER);
	ret = cfg->fsp_api->open(data->fsp_ctrl, data->fsp_cfg);
	k_mutex_unlock(&can_renesas_rh850_global_mutex);
	if (ret != FSP_SUCCESS) {
		LOG_ERR("CAN bus initialize failed");
		return -EIO;
	}

	/* Put CAN controller into stopped state */
	ret = cfg->fsp_api->modeTransition(data->fsp_ctrl, CAN_OPERATION_MODE_HALT,
					   CAN_TEST_MODE_DISABLED);
	if (ret != FSP_SUCCESS) {
		k_mutex_lock(&can_renesas_rh850_global_mutex, K_FOREVER);
		cfg->fsp_api->close(data->fsp_ctrl);
		k_mutex_unlock(&can_renesas_rh850_global_mutex);
		LOG_ERR("CAN stop failed");
		return -EIO;
	}

#ifdef CONFIG_RENESAS_RH850_INTIF_ORED
	ret = canfd_rh850_irq_init(dev);
	if (ret < 0) {
		k_mutex_lock(&can_renesas_rh850_global_mutex, K_FOREVER);
		cfg->fsp_api->close(data->fsp_ctrl);
		k_mutex_unlock(&can_renesas_rh850_global_mutex);
		LOG_ERR("%s: RH850 canfd driver initial interrupt is fail: %d", dev->name, ret);
		return ret;
	}
#endif /* CONFIG_RENESAS_RH850_INTIF_ORED */

	return pm_device_driver_init(dev, can_renesas_rh850_pm_control);
}

#ifdef CONFIG_RENESAS_RH850_INTIF_ORED
static int canfd_rh850_irq_init(const struct device *dev)
{
	struct can_renesas_rh850_data *data = dev->data;
	canfd_extended_cfg_t *canfd_config_extend =
		(canfd_extended_cfg_t *)(data->fsp_cfg->p_extend);
	int err;

	if (canfd_config_extend->tx_ored_int != BSP_ORED_CANFD_DISABLE) {
		err = intc_rh850_intif_ored_set_callback(data->txi_ctrl,
							 canfd_config_extend->tx_ored_int,
							 canfd_rh850_txi_isr_cb, (void *)dev);

		if (err) {
			LOG_ERR("TXI set callback failed");
			return -EIO;
		}

		/* Only enable INTIF ORed interrupts after callbacks are set at least once,
		 * otherwise the INTIF ORed source will be masked
		 */
		intc_rh850_intif_ored_set_enabled(data->txi_ctrl, canfd_config_extend->tx_ored_int,
						  data->fsp_cfg->ipl, true);
	}

	if (canfd_config_extend->rx_ored_int != BSP_ORED_CANFD_DISABLE) {
		err = intc_rh850_intif_ored_set_callback(data->rxi_ctrl,
							 canfd_config_extend->rx_ored_int,
							 canfd_rh850_rxi_isr_cb, (void *)dev);

		if (err) {
			LOG_ERR("RXI set callback failed");
			return -EIO;
		}

		/* Only enable INTIF ORed interrupts after callbacks are set at least once,
		 * otherwise the INTIF ORed source will be masked
		 */
		intc_rh850_intif_ored_set_enabled(data->rxi_ctrl, canfd_config_extend->rx_ored_int,
						  data->fsp_cfg->ipl, true);
	}

	if (canfd_config_extend->err_ored_int != BSP_ORED_CANFD_DISABLE) {
		err = intc_rh850_intif_ored_set_callback(data->eri_ctrl,
							 canfd_config_extend->err_ored_int,
							 canfd_rh850_eri_isr_cb, (void *)dev);

		if (err) {
			LOG_ERR("ERI set callback failed");
			return -EIO;
		}

		/* Only enable INTIF ORed interrupts after callbacks are set at least once,
		 * otherwise the INTIF ORed source will be masked
		 */
		intc_rh850_intif_ored_set_enabled(data->eri_ctrl, canfd_config_extend->err_ored_int,
						  data->fsp_cfg->ipl, true);
	}
	return 0;
}

static void canfd_rh850_txi_isr_cb(void *arg)
{
	ARG_UNUSED(arg);
	canfd_channel_tx_isr();
}

static void canfd_rh850_rxi_isr_cb(void *arg)
{
	ARG_UNUSED(arg);
	canfd_rx_common_fifo_isr();
}

static void canfd_rh850_eri_isr_cb(void *arg)
{
	ARG_UNUSED(arg);
	canfd_channel_error_isr();
}
#endif /* CONFIG_RENESAS_RH850_INTIF_ORED */

static DEVICE_API(can, can_renesas_rh850_driver_api) = {
	.get_capabilities = can_renesas_rh850_get_capabilities,
	.start = can_renesas_rh850_start,
	.stop = can_renesas_rh850_stop,
	.set_mode = can_renesas_rh850_set_mode,
	.set_timing = can_renesas_rh850_set_timing,
	.send = can_renesas_rh850_send,
	.add_rx_filter = can_renesas_rh850_add_rx_filter,
	.remove_rx_filter = can_renesas_rh850_remove_rx_filter,
#if defined(CONFIG_CAN_MANUAL_RECOVERY_MODE)
	.recover = can_renesas_rh850_recover,
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */
	.get_state = can_renesas_rh850_get_state,
	.set_state_change_callback = can_renesas_rh850_set_state_change_callback,
	.get_core_clock = can_renesas_rh850_get_core_clock,
	.get_max_filters = can_renesas_rh850_get_max_filters,
	.timing_min = CAN_RENESAS_RH850_TIMING_MIN,
	.timing_max = CAN_RENESAS_RH850_TIMING_MAX,
#if defined(CONFIG_CAN_FD_MODE)
	.set_timing_data = can_renesas_rh850_set_timing_data,
	.timing_data_min = CAN_RENESAS_RH850_TIMING_DATA_MIN,
	.timing_data_max = CAN_RENESAS_RH850_TIMING_DATA_MAX,
#endif /* CONFIG_CAN_FD_MODE */
};

#ifdef CONFIG_RENESAS_RH850_INTIF_ORED
/* Macro definition to canfd driver get irq number and ipl value */
#define CANFD_IRQ_GET(inst, type)                                                                  \
	COND_CODE_1( \
				DT_NODE_HAS_COMPAT(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), type),  \
									renesas_rh850_intc), \
				(DT_INST_IRQ_BY_NAME(inst, type, irq)), \
				(DT_IRQ(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), type), irq)))

#define CANFD_IPL_GET(inst, type)                                                                  \
	COND_CODE_1( \
				DT_NODE_HAS_COMPAT(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), type), \
									renesas_rh850_intc), \
				(DT_INST_IRQ_BY_NAME(inst, type, priority)), \
				(DT_IRQ(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), type), priority)))

#define _CANFD_TXI_IRQ_IS_ORED(n, channel) BSP_ORED_TRANSMIT_CANFD_CH##n##_ORED##channel
#define CANFD_TXI_IRQ_IS_ORED(n, channel)  _CANFD_TXI_IRQ_IS_ORED(n, channel)
#define _CANFD_RXI_IRQ_IS_ORED(n, channel) BSP_ORED_FIFO_CANFD_CH##n##_ORED##channel
#define CANFD_RXI_IRQ_IS_ORED(n, channel)  _CANFD_RXI_IRQ_IS_ORED(n, channel)
#define _CANFD_ERI_IRQ_IS_ORED(n, channel) BSP_ORED_ERROR_CANFD_CH##n##_ORED##channel
#define CANFD_ERI_IRQ_IS_ORED(n, channel)  _CANFD_ERI_IRQ_IS_ORED(n, channel)

/* Get the ORed interrupt channel associated with the specified IRQ source. */
#define CANFD_ORED_CHAN_GET(n, type) DT_PROP(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(n), type), channel)
/* macro definition for ORed CANFD TX interrupt */
#define CANFD_TXI_ORED_INFO(inst)                                                                  \
	COND_CODE_1( \
	DT_NODE_HAS_COMPAT(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), txi), \
						renesas_rh850_intc), \
	(BSP_ORED_CANFD_DISABLE), \
	(CANFD_TXI_IRQ_IS_ORED(DT_INST_PROP(inst, channel), CANFD_ORED_CHAN_GET(inst, txi))))
#define CANFD_RXI_ORED_INFO(inst)                                                                  \
	COND_CODE_1( \
	DT_NODE_HAS_COMPAT(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), rxi), \
						renesas_rh850_intc), \
	(BSP_ORED_CANFD_DISABLE), \
	(CANFD_RXI_IRQ_IS_ORED(DT_INST_PROP(inst, channel), CANFD_ORED_CHAN_GET(inst, rxi))))
#define CANFD_ERI_ORED_INFO(inst)                                                                  \
	COND_CODE_1( \
	DT_NODE_HAS_COMPAT(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), eri), \
						renesas_rh850_intc), \
	(BSP_ORED_CANFD_DISABLE), \
	(CANFD_ERI_IRQ_IS_ORED(DT_INST_PROP(inst, channel), CANFD_ORED_CHAN_GET(inst, eri))))

#define CANFD_RH850_ORED_CONFIG(n)                                                                 \
	.tx_ored_int = CANFD_TXI_ORED_INFO(n), .rx_ored_int = CANFD_RXI_ORED_INFO(n),              \
	.err_ored_int = CANFD_ERI_ORED_INFO(n),

#define CAN_RENESAS_RH850_CHANNEL_IRQ_INIT(index)
/*
 * Get the interrupt controller device associated with the specified CANFD
 * interrupt source.
 * Returns NULL when the interrupt is managed by the
 * RH850 INTC driver since no external
 * interrupt control device is required.
 */
#define CANFD_CTRL_GET(inst, type)                                                                 \
	COND_CODE_1(                           \
				DT_NODE_HAS_COMPAT(                          \
					DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), type), \
						renesas_rh850_intc),                          \
					(NULL),                          \
					(DEVICE_DT_GET(                          \
						DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), type))))
/* Macro for initializing interrupt control data */
#define CANFD_RH850_INT_CTRL_INIT(n)                                                               \
	.txi_ctrl = CANFD_CTRL_GET(n, txi), .rxi_ctrl = CANFD_CTRL_GET(n, rxi),                    \
	.eri_ctrl = CANFD_CTRL_GET(n, eri),
#else /* CONFIG_RENESAS_RH850_INTIF_ORED */
#define CANFD_RH850_ORED_CONFIG(n)
#define CANFD_RH850_INT_CTRL_INIT(n)

#ifdef CONFIG_RENESAS_RH850_INTIF_INTSEL
/* INTSEL nodes contain the hardware IRQ and priority of the selected MUX. */
#define CANFD_IRQ_GET(inst, type)                                                                  \
	COND_CODE_1(DT_NODE_HAS_COMPAT(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), type),        \
				      renesas_rh850_intif_intsel),                         \
		    (DT_IRQ(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), type), irq)),          \
		    (DT_INST_IRQ_BY_NAME(inst, type, irq)))

#define CANFD_IPL_GET(inst, type)                                                                  \
	COND_CODE_1(DT_NODE_HAS_COMPAT(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), type),        \
				      renesas_rh850_intif_intsel),                         \
		    (DT_IRQ(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), type), priority)),     \
		    (DT_INST_IRQ_BY_NAME(inst, type, priority)))
#else
#define CANFD_IRQ_GET(inst, type) DT_INST_IRQ_BY_NAME(inst, type, irq)
#define CANFD_IPL_GET(inst, type) DT_INST_IRQ_BY_NAME(inst, type, priority)
#endif

#define CAN_RENESAS_RH850_CHANNEL_IRQ_INIT(index)                                                  \
	{                                                                                          \
		IRQ_CONNECT(CANFD_IRQ_GET(index, rxi), CANFD_IPL_GET(index, rxi),                  \
			    canfd_rx_common_fifo_isr, NULL, 0);                                    \
		IRQ_CONNECT(CANFD_IRQ_GET(index, txi), CANFD_IPL_GET(index, txi),                  \
			    canfd_channel_tx_isr, NULL, 0);                                        \
		IRQ_CONNECT(CANFD_IRQ_GET(index, eri), CANFD_IPL_GET(index, eri),                  \
			    canfd_channel_error_isr, NULL, 0);                                     \
		irq_enable(CANFD_IRQ_GET(index, rxi));                                             \
		irq_enable(CANFD_IRQ_GET(index, txi));                                             \
		irq_enable(CANFD_IRQ_GET(index, eri));                                             \
	}
#endif /* CONFIG_RENESAS_RH850_INTIF_ORED */

/* INTSEL routing follows the interrupt-controller phandle, as in the RLIN3 driver. */
#if BSP_FEATURE_DEVICE_HAS_INTSEL_IRQ
/* Use -1 for interrupts that are not routed through INTSEL. */
#define CANFD_RH850_INTSEL_MUX_GET(inst, type)                                                     \
	COND_CODE_1(DT_NODE_HAS_COMPAT(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), type), \
				      renesas_rh850_intif_intsel), \
		    (DT_PROP(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), type), channel)), \
		    (-1))

/* One INTSEL MUX routes all interrupts belonging to a CAN FD channel. */
#define CAN_RENESAS_RH850_INTSEL_MUX_ASSERT(index)                                                 \
	BUILD_ASSERT(CANFD_RH850_INTSEL_MUX_GET(index, rxi) ==                                     \
			     CANFD_RH850_INTSEL_MUX_GET(index, eri),                               \
		     "CAN FD RXI and ERI must use the same INTSEL MUX");                           \
	BUILD_ASSERT(CANFD_RH850_INTSEL_MUX_GET(index, txi) ==                                     \
			     CANFD_RH850_INTSEL_MUX_GET(index, eri),                               \
		     "CAN FD TXI and ERI must use the same INTSEL MUX");

DT_INST_FOREACH_STATUS_OKAY(CAN_RENESAS_RH850_INTSEL_MUX_ASSERT)

#define CANFD_RH850_INTSEL_CONFIG(inst)                                                            \
	.intsel_irq_cfg = COND_CODE_1(                                                    \
		DT_NODE_HAS_COMPAT(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), rxi),             \
				   renesas_rh850_intif_intsel),                            \
		(BSP_INTSEL_CREATE(BSP_INTSEL_MOD_CANFD, 0,                                \
				   DT_INST_PROP(inst, channel) + 1,                         \
				   DT_PROP(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(inst), rxi),      \
					   channel))),                                     \
		(BSP_INTSEL_DISABLED)),
#else
#define CANFD_RH850_INTSEL_CONFIG(inst)
#endif

/* FSP uses one priority for all interrupts belonging to a CAN FD channel. */
#define CAN_RENESAS_RH850_IRQ_PRIORITY_ASSERT(index)                                               \
	BUILD_ASSERT(CANFD_IPL_GET(index, rxi) == CANFD_IPL_GET(index, eri),                       \
		     "CAN FD RXI and ERI priorities must match");                                  \
	BUILD_ASSERT(CANFD_IPL_GET(index, txi) == CANFD_IPL_GET(index, eri),                       \
		     "CAN FD TXI and ERI priorities must match");

DT_INST_FOREACH_STATUS_OKAY(CAN_RENESAS_RH850_IRQ_PRIORITY_ASSERT)

#define CAN_RENESAS_RH850_AFL_RULE_NUM(node_id)                                                    \
	[DT_PROP(node_id, channel) % CANFD_MAX_CHANNEL_PER_UNIT] = DT_PROP(node_id, rx_max_filters),

#define CAN_RENESAS_RH850_GLOBAL_CFG(node_id) CONCAT(can_renesas_rh850_global_cfg_, node_id)

#define CAN_RENESAS_RH850_GLOBAL_IRQ_CFG(node_id) CONCAT(can_renesas_rh850_global_irq_cfg_, node_id)

#define CAN_RENESAS_RH850_GLOBAL_CHANNEL(node_id, channel)                                         \
	(DT_PROP(node_id, unit) * CANFD_MAX_CHANNEL_PER_UNIT + (channel))

#define CAN_RENESAS_RH850_CHANNEL_NUMBER(node_id) DT_PROP(node_id, channel),

/*
 * Keep the global error IRQ owner fixed to the first enabled channel in this unit.
 * With no enabled children, the fallback is unused because no channel opens the unit.
 */
#define CAN_RENESAS_RH850_GLOBAL_ERROR_CHANNEL(node_id)                                            \
	GET_ARG_N(1, DT_FOREACH_CHILD_STATUS_OKAY(node_id, CAN_RENESAS_RH850_CHANNEL_NUMBER)       \
			     CAN_RENESAS_RH850_GLOBAL_CHANNEL(node_id, 0))

static int can_renesas_rh850_global_init(const struct device *dev)
{
	const struct can_renesas_rh850_global_cfg *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("CAN clock device is not ready");
		return -ENODEV;
	}

	/* Release the CAN unit from module standby, including with an external clock. */
	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret < 0) {
		LOG_ERR("CAN module clock enable failed (err %d)", ret);
		return ret;
	}

	cfg->irq_configure();

	return 0;
}

/* Each CAN FD unit owns one global register block and one global configuration. */
#define CAN_RENESAS_RH850_GLOBAL_DEFINE(node_id)                                                   \
	BUILD_ASSERT(DT_PROP(node_id, unit) < BSP_FEATURE_CANFD_NUM_UNIT,                          \
		     "CAN FD unit exceeds the number supported by this SoC");                      \
	static canfd_global_cfg_t CAN_RENESAS_RH850_GLOBAL_CFG(node_id) = {                        \
		.channel_error = CAN_RENESAS_RH850_GLOBAL_ERROR_CHANNEL(node_id),                  \
		.global_interrupts = CANFD_CFG_GLOBAL_ERR_SOURCES,                                 \
		.global_config = (CANFD_CFG_TX_PRIORITY | CANFD_CFG_DLC_CHECK |                    \
				  CANFD_CFD_CLOCK_SOURCE | CANFD_CFG_FD_OVERFLOW),                 \
		.rx_mb_config = (CANFD_CFG_RXMB_NUMBER |                                           \
				 (CANFD_CFG_RXMB_SIZE << R_CANFD0_CFDRMNB_RMPLS_Pos)),             \
		.global_err_ipl = DT_IRQ_BY_NAME(node_id, g_err, priority),                        \
		.rx_fifo_ipl = DT_IRQ_BY_NAME(node_id, g_recc, priority),                          \
		.rx_fifo_config =                                                                  \
			{                                                                          \
				CANFD_CFG_RXFIFO_DEFAULT(0),                                       \
				CANFD_CFG_RXFIFO_DEFAULT(1),                                       \
				CANFD_CFG_RXFIFO_DEFAULT(2),                                       \
				CANFD_CFG_RXFIFO_DEFAULT(3),                                       \
				CANFD_CFG_RXFIFO_DEFAULT(4),                                       \
				CANFD_CFG_RXFIFO_DEFAULT(5),                                       \
				CANFD_CFG_RXFIFO_DEFAULT(6),                                       \
				CANFD_CFG_RXFIFO_DEFAULT(7),                                       \
			},                                                                         \
		.rx_fifo_cb_channel =                                                              \
			{                                                                          \
				CAN_RENESAS_RH850_GLOBAL_CHANNEL(node_id, 0),                      \
				CAN_RENESAS_RH850_GLOBAL_CHANNEL(node_id, 1),                      \
				CAN_RENESAS_RH850_GLOBAL_CHANNEL(node_id, 2),                      \
				CAN_RENESAS_RH850_GLOBAL_CHANNEL(node_id, 3),                      \
				CAN_RENESAS_RH850_GLOBAL_CHANNEL(node_id, 4),                      \
				CAN_RENESAS_RH850_GLOBAL_CHANNEL(node_id, 5),                      \
				CAN_RENESAS_RH850_GLOBAL_CHANNEL(node_id, 6),                      \
				CAN_RENESAS_RH850_GLOBAL_CHANNEL(node_id, 7),                      \
			},                                                                         \
		.p_canfd_reg = (void *)DT_REG_ADDR(node_id),                                       \
		.unit = DT_PROP(node_id, unit),                                                    \
		.afl_rule_num = {DT_FOREACH_CHILD_STATUS_OKAY(node_id,                             \
							      CAN_RENESAS_RH850_AFL_RULE_NUM)},    \
		.rx_fifo_irq = DT_IRQ_BY_NAME(node_id, g_recc, irq),                               \
		.err_glb_irq = DT_IRQ_BY_NAME(node_id, g_err, irq),                                \
	};                                                                                         \
	static void CAN_RENESAS_RH850_GLOBAL_IRQ_CFG(node_id)(void)                                \
	{                                                                                          \
		/* FSP registers the global error ISR context before enabling its IRQ. */          \
		IRQ_CONNECT(DT_IRQ_BY_NAME(node_id, g_err, irq),                                   \
			    DT_IRQ_BY_NAME(node_id, g_err, priority), canfd_global_error_isr,      \
			    NULL, 0);                                                              \
		IRQ_CONNECT(DT_IRQ_BY_NAME(node_id, g_recc, irq),                                  \
			    DT_IRQ_BY_NAME(node_id, g_recc, priority), canfd_rx_fifo_isr, NULL,    \
			    0);                                                                    \
		irq_enable(DT_IRQ_BY_NAME(node_id, g_recc, irq));                                  \
	}                                                                                          \
	static const struct can_renesas_rh850_global_cfg                                           \
		can_renesas_rh850_global_device_cfg_##node_id = {                                  \
			.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(node_id)),                       \
			.clock_subsys = (clock_control_subsys_t)(uintptr_t)DT_CLOCKS_CELL(node_id, \
											  clk_id), \
			.irq_configure = CAN_RENESAS_RH850_GLOBAL_IRQ_CFG(node_id),                \
	};                                                                                         \
	DEVICE_DT_DEFINE(node_id, can_renesas_rh850_global_init, NULL, NULL,                       \
			 &can_renesas_rh850_global_device_cfg_##node_id, PRE_KERNEL_2,             \
			 CONFIG_CAN_INIT_PRIORITY, NULL)

DT_FOREACH_STATUS_OKAY(renesas_rh850_canfd_global, CAN_RENESAS_RH850_GLOBAL_DEFINE)

#define CAN_RENESAS_RH850_CHANNEL_UNIT_ASSERT(index)                                               \
	BUILD_ASSERT(DT_INST_PROP(index, channel) < BSP_FEATURE_CANFD_NUM_CHANNELS,                \
		     "CAN FD channel exceeds the number supported by this SoC");                   \
	BUILD_ASSERT((DT_INST_PROP(index, channel) / CANFD_MAX_CHANNEL_PER_UNIT) ==                \
			     DT_PROP(DT_INST_PARENT(index), unit),                                 \
		     "CAN FD channel does not belong to its parent unit");

DT_INST_FOREACH_STATUS_OKAY(CAN_RENESAS_RH850_CHANNEL_UNIT_ASSERT)

#define CAN_RENESAS_RH850_INIT(index)                                                              \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
	static canfd_afl_entry_t p_canfd_ch##index##_afl[DT_INST_PROP(index, rx_max_filters)];     \
	struct can_renesas_rh850_filter                                                            \
		can_renesas_rh850_rx_filter##index[DT_INST_PROP(index, rx_max_filters)];           \
	static can_bit_timing_cfg_t g_canfd_ch##index##_bit_timing_cfg;                            \
	static const struct can_renesas_rh850_cfg can_renesas_rh850_cfg##index = {                 \
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(index, 0, 8000000),                        \
		.global_dev = DEVICE_DT_GET(DT_INST_PARENT(index)),                                \
		.pin_config = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                               \
		.rx_filter_num = DT_INST_PROP(index, rx_max_filters),                              \
		.fsp_api = &g_can_on_canfd,                                                        \
		.clock_ext_rate = DT_PROP_OR(DT_NODELABEL(can_clk), clock_frequency, 0),           \
	};                                                                                         \
	static canfd_instance_ctrl_t g_canfd_ch##index##_ctrl;                                     \
	static canfd_extended_cfg_t g_canfd_ch##index##_extend_cfg = {                             \
		.p_afl = p_canfd_ch##index##_afl,                                                  \
		CAN_RENESAS_RH850_TX_CFG,                                                          \
		.error_interrupts = CANFD_CFG_ERR_IRQ,                                             \
		.p_global_cfg = &CAN_RENESAS_RH850_GLOBAL_CFG(DT_INST_PARENT(index)),              \
		CANFD_RH850_ORED_CONFIG(index) CANFD_RH850_INTSEL_CONFIG(index)};                  \
	static can_cfg_t g_canfd_ch##index##cfg = {                                                \
		.channel = DT_INST_PROP(index, channel),                                           \
		.ipl = CANFD_IPL_GET(index, eri),                                                  \
		.error_irq = CANFD_IRQ_GET(index, eri),                                            \
		.rx_irq = CANFD_IRQ_GET(index, rxi),                                               \
		.tx_irq = CANFD_IRQ_GET(index, txi),                                               \
		.p_extend = &g_canfd_ch##index##_extend_cfg,                                       \
		.p_bit_timing = &g_canfd_ch##index##_bit_timing_cfg,                               \
		.p_context = DEVICE_DT_INST_GET(index),                                            \
		.p_callback = can_renesas_rh850_fsp_cb,                                            \
	};                                                                                         \
	static struct can_renesas_rh850_data can_renesas_rh850_data##index = {                     \
		.fsp_ctrl = &g_canfd_ch##index##_ctrl,                                             \
		.fsp_cfg = &g_canfd_ch##index##cfg,                                                \
		.rx_filter = can_renesas_rh850_rx_filter##index,                                   \
		CANFD_RH850_INT_CTRL_INIT(index)};                                                 \
	static int can_renesas_rh850_init##index(const struct device *dev)                         \
	{                                                                                          \
		const struct device *global_canfd = DEVICE_DT_GET(DT_INST_PARENT(index));          \
		if (!device_is_ready(global_canfd)) {                                              \
			return -EIO;                                                               \
		}                                                                                  \
		CAN_RENESAS_RH850_CHANNEL_IRQ_INIT(index)                                          \
		return can_renesas_rh850_init(dev);                                                \
	}                                                                                          \
	PM_DEVICE_DT_INST_DEFINE(index, can_renesas_rh850_pm_control);                             \
	CAN_DEVICE_DT_INST_DEFINE(index, can_renesas_rh850_init##index,                            \
				  PM_DEVICE_DT_INST_GET(index), &can_renesas_rh850_data##index,    \
				  &can_renesas_rh850_cfg##index, POST_KERNEL,                      \
				  CONFIG_CAN_INIT_PRIORITY, &can_renesas_rh850_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CAN_RENESAS_RH850_INIT)
