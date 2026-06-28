/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include "soc.h"
#include "r_dmac.h"

LOG_MODULE_REGISTER(rh850_dma, CONFIG_DMA_LOG_LEVEL);

#define DT_DRV_COMPAT renesas_rh850_dma

void dmac_int_isr(void);

struct dma_rh850_channel_config {
	int irq;
	int ipl;
};

struct dma_rh850_config {
	void (*irq_configure)(void);
	const struct dma_rh850_channel_config *const channels;
	const uint32_t channel_count;
	const uint32_t unit;
	const transfer_api_t *fsp_api;
};

struct dma_rh850_channel_context {
	const struct device *dev;
	uint32_t channel;
};

struct dma_rh850_channel_data {
	dmac_instance_ctrl_t fsp_ctrl;
	transfer_cfg_t fsp_cfg;
	transfer_info_t fsp_info;
	dmac_extended_cfg_t fsp_extend;
	transfer_switchable_redundant_cfg_t fsp_switchable_redundant_cfg;
	struct dma_rh850_channel_context context;
	struct dma_config config;
};

struct dma_rh850_data {
	struct dma_context context;
	const struct dma_rh850_config *const config;
	struct dma_rh850_channel_data *const channels;
};

static void dma_rh850_callback_handler(transfer_callback_args_t *args)
{
	struct dma_rh850_channel_context *context =
		(struct dma_rh850_channel_context *)args->p_context;

	const uint32_t channel = context->channel;
	const struct device *dev = context->dev;
	struct dma_rh850_data *data = dev->data;
	dma_callback_t user_cb = data->channels[channel].config.dma_callback;
	void *user_data = data->channels[channel].config.user_data;

	/* Callback when Completion of the last transfer */
	if (user_cb && (args->event == TRANSFER_EVENT_TRANS_END_FLAG)) {
		user_cb(dev, user_data, channel, DMA_STATUS_COMPLETE);
	}
}

static bool dma_rh850_channel_is_valid(const struct device *dev, uint32_t channel)
{
	struct dma_rh850_data *data = dev->data;

	/* Check if the channel index is within the valid range */
	if (channel >= data->config->channel_count) {
		return false;
	}

	return true;
}

static bool dma_rh850_config_is_valid(struct dma_config *config)
{
	/* Check for null configuration, null block, or zero block count */
	if (config == NULL || config->head_block == NULL || config->block_count == 0) {
		return false;
	}

	/* Block size must be a multiple of source data size */
	if (config->head_block->block_size % config->source_data_size != 0) {
		return false;
	}

	/* Source address must be aligned to the source data size */
	if (config->head_block->source_address % config->source_data_size != 0) {
		return false;
	}

	/* Destination address must be aligned to the destination data size */
	if (config->head_block->dest_address % config->dest_data_size != 0) {
		return false;
	}

	return true;
}

static bool dma_rh850_config_is_support(struct dma_config *config)
{
	/* Cyclic mode is not supported */
	if (config->cyclic) {
		return false;
	}

	/* Only single block transfer is supported */
	if (config->block_count > 1) {
		return false;
	}

	return true;
}

static int dma_rh850_config_prepare(const struct device *dev, uint32_t channel,
					 struct dma_config *config)
{
	struct dma_rh850_data *data = dev->data;
	struct dma_rh850_channel_data *ch = &data->channels[channel];
	uint32_t transfers_count = (config->head_block->block_size / config->source_data_size);
	bool activation_with_software_trigger = true;

	/* Set source address adjustment mode */
	switch (config->head_block->source_addr_adj) {
	case DMA_ADDR_ADJ_INCREMENT:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.src_addr_mode =
			TRANSFER_ADDR_MODE_INCREMENTED;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.src_addr_mode =
			TRANSFER_ADDR_MODE_FIXED;
		break;
	case DMA_ADDR_ADJ_DECREMENT:
	default:
		return -EINVAL;
	}

	/* Set destination address adjustment mode */
	switch (config->head_block->dest_addr_adj) {
	case DMA_ADDR_ADJ_INCREMENT:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.des_addr_mode =
			TRANSFER_ADDR_MODE_INCREMENTED;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.des_addr_mode =
			TRANSFER_ADDR_MODE_FIXED;
		break;
	case DMA_ADDR_ADJ_DECREMENT:
	default:
		return -EINVAL;
	}

	/* Set source and destination addresses */
	ch->fsp_info.p_src = (void *)config->head_block->source_address;
	ch->fsp_info.p_dest = (void *)config->head_block->dest_address;

	/* Set source data size for each transfer */
	switch (config->source_data_size) {
	case 1U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.src_trans_size =
			TRANSFER_SIZE_1_BYTE;
		break;
	case 2U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.src_trans_size =
			TRANSFER_SIZE_2_BYTE;
		break;
	case 4U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.src_trans_size =
			TRANSFER_SIZE_4_BYTE;
		break;
	case 8U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.src_trans_size =
			TRANSFER_SIZE_8_BYTE;
		break;
	case 16U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.src_trans_size =
			TRANSFER_SIZE_16_BYTE;
		break;
	case 32U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.src_trans_size =
			TRANSFER_SIZE_32_BYTE;
		break;
	case 64U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.src_trans_size =
			TRANSFER_SIZE_64_BYTE;
		break;
	default:
		return -EINVAL;
	}

	/* Set destination data size for each transfer */
	switch (config->dest_data_size) {
	case 1U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.des_trans_size =
			TRANSFER_SIZE_1_BYTE;
		break;
	case 2U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.des_trans_size =
			TRANSFER_SIZE_2_BYTE;
		break;
	case 4U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.des_trans_size =
			TRANSFER_SIZE_4_BYTE;
		break;
	case 8U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.des_trans_size =
			TRANSFER_SIZE_8_BYTE;
		break;
	case 16U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.des_trans_size =
			TRANSFER_SIZE_16_BYTE;
		break;
	case 32U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.des_trans_size =
			TRANSFER_SIZE_32_BYTE;
		break;
	case 64U:
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.des_trans_size =
			TRANSFER_SIZE_64_BYTE;
		break;
	default:
		return -EINVAL;
	}

	/* Check if the channel has a software or hardware trigger */
	switch (config->channel_direction) {
	case MEMORY_TO_MEMORY:
		activation_with_software_trigger = true;
		break;
	case PERIPHERAL_TO_MEMORY:
		activation_with_software_trigger = false;
		break;
	case MEMORY_TO_PERIPHERAL:
		activation_with_software_trigger = false;
		break;
	default:
		LOG_ERR("%d: Unsupported direction mode.", __LINE__);
		return -ENOTSUP;
	}

	/* Set transfer mode for DMA transfer */
	ch->fsp_info.transfer_mode_cfg.transfer_mode_b.transfer_mode = TRANSFER_MODE_NORMAL;

	/* Transfer count must not exceed hardware limit */
	if (transfers_count > UINT32_MAX) {
		return -EINVAL;
	}

	/* Set number transfer for DMA transfer */
	ch->fsp_info.number_transfer = (uint32_t)transfers_count;

	/* Set trigger source to software trigger or hardware trigger */
	ch->fsp_extend.activation_source = activation_with_software_trigger
					      ? DMAC_TRIGGER_EVENT_SOFTWARE
					      : config->dma_slot;

	/* Initialize context for DMA callbacks */
	ch->context.channel = channel;
	ch->context.dev = dev;

	/* Set up remaining fields in the FSP extension structure */
	ch->fsp_extend.p_context = &ch->context;
	ch->fsp_extend.p_callback = dma_rh850_callback_handler;
	ch->fsp_extend.channel = (uint8_t)channel;

	const struct dma_rh850_config *cfg = dev->config;

	ch->fsp_extend.unit = cfg->unit;
	if (data->config->channels[channel].irq != FSP_INVALID_VECTOR) {
		ch->fsp_extend.irq = (IRQn_Type)data->config->channels[channel].irq;
		ch->fsp_extend.irq_priority_level = data->config->channels[channel].ipl;
		ch->fsp_info.transfer_mode_cfg.transfer_mode_b.transfer_completion_irq = true;
	}

	ch->fsp_info.transfer_mode_cfg.transfer_mode_b.channel_address_error_irq = false;

	/* Config switchable redundant config, not supported yet */
	ch->fsp_extend.p_switchable_redundant_cfg = &ch->fsp_switchable_redundant_cfg;
	ch->fsp_extend.p_switchable_redundant_cfg->redundant_operation =
		TRANSFER_SWITCHABLE_REDUNDANT_OPERATION_DISABLE;

	/* Link transfer info and extension structure to the FSP configuration */
	ch->fsp_cfg.p_info = &ch->fsp_info;
	ch->fsp_cfg.p_extend = &ch->fsp_extend;

	/* Save DMA configuration to channel context */
	memcpy(&ch->config, config, sizeof(struct dma_config));

	return 0;
}

static int dma_rh850_configure(const struct device *dev, uint32_t channel,
				 struct dma_config *config)
{
	struct dma_rh850_data *data = dev->data;
	struct dma_rh850_channel_data *ch = &data->channels[channel];
	fsp_err_t err;
	int ret;

	/* Validate the DMA channel */
	if (!dma_rh850_channel_is_valid(dev, channel)) {
		LOG_ERR("Invalid DMA channel: %d.", channel);
		return -EINVAL;
	}

	/* Validate the provided DMA configuration */
	if (!dma_rh850_config_is_valid(config)) {
		LOG_ERR("Invalid DMA config for channel %d.", channel);
		return -EINVAL;
	}

	/* Check if the configuration is supported by the driver */
	if (!dma_rh850_config_is_support(config)) {
		LOG_ERR("Unsupported DMA config for channel %d.", channel);
		return -ENOTSUP;
	}

	/* Prepare internal structures and hardware settings for the DMA transfer */
	ret = dma_rh850_config_prepare(dev, channel, config);
	if (ret) {
		LOG_ERR("Failed to prepare DMA config for channel %d.", channel);
		return ret;
	}

	/* Open the DMA channel or reconfigure if already open */
	if (ch->fsp_ctrl.open) {
		err = R_DMAC_Reconfigure(&ch->fsp_ctrl, &ch->fsp_info);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Failed to reconfigure DMA channel %d.", channel);
			return -EIO;
		}
	} else {
		err = R_DMAC_Open(&ch->fsp_ctrl, &ch->fsp_cfg);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Failed to open DMA channel %d.", channel);
			return -EIO;
		}
	}

	return 0;
}

static int dma_rh850_reload(const struct device *dev, uint32_t channel, uint32_t src,
				 uint32_t dst, size_t size)
{
	struct dma_rh850_data *data = dev->data;
	struct dma_rh850_channel_data *ch = &data->channels[channel];
	uint32_t data_size = ch->config.source_data_size;
	fsp_err_t err;

	/* Validate the DMA channel */
	if (!dma_rh850_channel_is_valid(dev, channel)) {
		LOG_ERR("DMA channel %d is invalid.", channel);
		return -EINVAL;
	}

	/* Check if the DMA channel is open */
	if (ch->fsp_ctrl.open == 0) {
		LOG_ERR("DMA channel %d is not configured.", channel);
		return -EINVAL;
	}

	/* Validate the transfer size */
	if ((size == 0) || (size % data_size != 0)) {
		LOG_ERR("DMA transfer size is invalid.");
		return -EINVAL;
	}

	/* Reload DMA controller with new source, destination, and size */
	err = R_DMAC_Reset(&ch->fsp_ctrl, (void *)src, (void *)dst, (uint16_t)(size / data_size));
	if (err != FSP_SUCCESS) {
		LOG_ERR("DMA channel %d reload failed: 0x%x", channel, err);
		return -EIO;
	}

	return 0;
}

static int dma_rh850_start(const struct device *dev, uint32_t channel)
{
	struct dma_rh850_data *data = dev->data;
	struct dma_rh850_channel_data *ch = &data->channels[channel];
	fsp_err_t err;

	/* Validate the DMA channel */
	if (!dma_rh850_channel_is_valid(dev, channel)) {
		LOG_ERR("DMA channel %d is invalid.", channel);
		return -EINVAL;
	}

	/* Check if the DMA channel is open */
	if (ch->fsp_ctrl.open == 0) {
		LOG_ERR("DMA channel %d is not configured.", channel);
		return -EINVAL;
	}

	/* Enable the DMA channel */
	err = R_DMAC_Enable(&ch->fsp_ctrl);
	if (err != FSP_SUCCESS) {
		LOG_ERR("DMA channel %d enable failed: 0x%x", channel, err);
		return -EIO;
	}

	/* Check if the DMA channel is software trigger */
	if (DMAC_TRIGGER_EVENT_SOFTWARE == ch->fsp_extend.activation_source) {
		/* Start the DMA transfer using software trigger */
		err = R_DMAC_SoftwareStart(&ch->fsp_ctrl, TRANSFER_START_MODE_REPEAT);
		if (err != FSP_SUCCESS) {
			LOG_ERR("DMA channel %d start failed: 0x%x", channel, err);
			return -EIO;
		}
	}

	return 0;
}

static int dma_rh850_stop(const struct device *dev, uint32_t channel)
{
	struct dma_rh850_data *data = dev->data;
	struct dma_rh850_channel_data *ch = &data->channels[channel];
	fsp_err_t err;

	/* Validate the DMA channel */
	if (!dma_rh850_channel_is_valid(dev, channel)) {
		LOG_ERR("DMA channel %d is invalid.", channel);
		return -EINVAL;
	}

	/* Check if the DMA channel is open */
	if (ch->fsp_ctrl.open == 0) {
		LOG_ERR("DMA channel %d is not configured.", channel);
		return -EINVAL;
	}

	/* Check if the DMA channel is software trigger */
	if (DMAC_TRIGGER_EVENT_SOFTWARE == ch->fsp_extend.activation_source) {
		/* Issue a software stop to halt the DMA transfer */
		err = R_DMAC_SoftwareStop(&ch->fsp_ctrl);
		if (err != FSP_SUCCESS) {
			LOG_ERR("DMA channel %d stop failed: 0x%x", channel, err);
			return -EIO;
		}
	}

	/* Disable the DMA channel */
	err = R_DMAC_Disable(&ch->fsp_ctrl);
	if (err != FSP_SUCCESS) {
		LOG_ERR("DMA channel %d disable failed: 0x%x", channel, err);
		return -EIO;
	}

	return 0;
}

static int dma_rh850_get_status(const struct device *dev, uint32_t channel,
				     struct dma_status *status)
{
	struct dma_rh850_data *data = dev->data;
	struct dma_rh850_channel_data *ch = &data->channels[channel];
	transfer_properties_t info;
	fsp_err_t err;

	/* Validate the DMA channel */
	if (!dma_rh850_channel_is_valid(dev, channel)) {
		LOG_ERR("DMA channel %d is invalid.", channel);
		return -EINVAL;
	}

	/* Check if the DMA channel is open */
	if (ch->fsp_ctrl.open == 0) {
		LOG_ERR("DMA channel %d is not configured.", channel);
		return -EINVAL;
	}

	/* Retrieve current transfer information */
	err = R_DMAC_InfoGet(&ch->fsp_ctrl, &info);
	if (err != FSP_SUCCESS) {
		LOG_ERR("DMA channel %d get info failed: 0x%x", channel, err);
		return -EIO;
	}

	/* Initialize status structure to zero */
	memset(status, 0, sizeof(struct dma_status));

	/* Get transfer direction */
	status->dir = ch->config.channel_direction;

	/* Calculate remaining bytes to transfer */
	status->pending_length = info.transfer_length_remaining;

	/* Indicate whether the DMA is busy */
	status->busy = status->pending_length ? true : false;

	/* Calculate total bytes copied so far */
	status->total_copied = ch->fsp_info.number_transfer - info.transfer_length_remaining;

	return 0;
}

static bool dma_rh850_chan_filter(const struct device *dev, int channel, void *filter_param)
{
	ARG_UNUSED(filter_param);

	struct dma_rh850_data *data = dev->data;
	struct dma_rh850_channel_data *ch = &data->channels[channel];

	/* Check if the DMA channel is valid */
	if (!dma_rh850_channel_is_valid(dev, channel)) {
		return false;
	}

	/* Check if the channel is already open */
	if (ch->fsp_ctrl.open != 0) {
		return false;
	}

	return true;
}

static void dma_rh850_chan_release(const struct device *dev, uint32_t channel)
{
	struct dma_rh850_data *data = dev->data;
	struct dma_rh850_channel_data *ch = &data->channels[channel];

	/* Check if the DMA channel is valid */
	if (!dma_rh850_channel_is_valid(dev, channel)) {
		return;
	}

	/* Close the DMA channel to release resources */
	R_DMAC_Close(&ch->fsp_ctrl);
}

static DEVICE_API(dma, dma_rh850_driver_api) = {
	.config = dma_rh850_configure,
	.reload = dma_rh850_reload,
	.start = dma_rh850_start,
	.stop = dma_rh850_stop,
	.get_status = dma_rh850_get_status,
	.chan_filter = dma_rh850_chan_filter,
	.chan_release = dma_rh850_chan_release,
};

static int dma_rh850_init(const struct device *dev)
{
	struct dma_rh850_data *const data = dev->data;

	/* Configure DMA-related interrupts */
	data->config->irq_configure();

	return 0;
}

/* Macro to get DMA channel IRQ or use default if not defined */
#define DMA_CH_IRQ_BY_NAME_OR(n, inst, cell, default)                                              \
	COND_CODE_1(DT_INST_IRQ_HAS_NAME(inst, ch##n),                                             \
		   (DT_INST_IRQ_BY_NAME(inst, ch##n, cell)),                                       \
		   (default))

/* Macro to define DMA channel configuration struct initializer */
#define DMA_CH_CONFIG(n, inst)                                                                     \
	{                                                                                          \
		.irq = DMA_CH_IRQ_BY_NAME_OR(n, inst, irq, FSP_INVALID_VECTOR),                    \
		.ipl = DMA_CH_IRQ_BY_NAME_OR(n, inst, priority, BSP_IRQ_DISABLED),                 \
	}

/* Macro to create a list of DMA channel configuration initializers */
#define DMA_CH_CONFIG_LIST(inst) LISTIFY(DT_INST_PROP(inst, dma_channels), DMA_CH_CONFIG, (,), inst)


#define DMA_RH850_GET_UNIT(inst)                                                                   \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dma_unit),                                         \
				(DT_INST_PROP(inst, dma_unit)), (0))

/* Macro to configure IRQ for a DMA channel if defined in DTS */
#define DMA_CH_IRQ_CONFIG(n, inst)                                                                 \
	COND_CODE_1(DT_INST_IRQ_HAS_NAME(inst, ch##n),                                             \
		(IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, ch##n, irq),                                \
		DT_INST_IRQ_BY_NAME(inst, ch##n, priority), dmac_int_isr, NULL, 0);), ())

/* Macro to configure IRQs for all DMA channels in a list */
#define DMA_CH_IRQ_CONFIG_LIST(inst)                                                               \
	LISTIFY(DT_INST_PROP(inst, dma_channels), DMA_CH_IRQ_CONFIG, (;), inst)

/* Macro for DMA driver initialization */
#define DMA_RH850_INIT(inst)                                                                       \
	static void dma_rh850_irq_configure##inst(void)                                            \
	{                                                                                          \
		DMA_CH_IRQ_CONFIG_LIST(inst);                                                      \
	}                                                                                          \
                                                                                                   \
	static const struct dma_rh850_channel_config dma_rh850_channel_config##inst[DT_INST_PROP(  \
		inst, dma_channels)] = {DMA_CH_CONFIG_LIST(inst)};                                 \
                                                                                                   \
	static const struct dma_rh850_config dma_rh850_config##inst = {                            \
		.irq_configure = dma_rh850_irq_configure##inst,                                    \
		.channels = dma_rh850_channel_config##inst,                                        \
		.channel_count = DT_INST_PROP(inst, dma_channels),                                 \
		.unit = DMA_RH850_GET_UNIT(inst),                                                  \
		.fsp_api = &g_transfer_on_dmac,                                                    \
	};                                                                                         \
                                                                                                   \
	static struct dma_rh850_channel_data                                                       \
		dma_rh850_channel_data##inst[DT_INST_PROP(inst, dma_channels)] = {0};              \
                                                                                                   \
	ATOMIC_DEFINE(dma_rh850_atomic##inst, DT_INST_PROP(inst, dma_channels));                   \
                                                                                                   \
	static struct dma_rh850_data dma_rh850_data##inst = {                                      \
		.context =                                                                         \
			{                                                                          \
				.magic = DMA_MAGIC,                                                \
				.atomic = dma_rh850_atomic##inst,                                  \
				.dma_channels = DT_INST_PROP(inst, dma_channels),                  \
			},                                                                         \
		.channels = dma_rh850_channel_data##inst,                                          \
		.config = &dma_rh850_config##inst,                                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, dma_rh850_init, NULL, &dma_rh850_data##inst,                   \
			      &dma_rh850_config##inst, PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,     \
			      &dma_rh850_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_RH850_INIT)
