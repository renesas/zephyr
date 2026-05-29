/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_dma

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/cache.h>
#include <stdint.h>
#include "soc.h"
#include "dma_renesas_rcar.h"
#include "dmac_ctrl_common.h"
#include "dmac_reg.h"

LOG_MODULE_REGISTER(dma_renesas_rcar, CONFIG_DMA_LOG_LEVEL);

/* Required by DEVICE_MMIO_NAMED_* macros */
#define DEV_CFG(_dev)  ((const struct dma_renesas_rcar_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct dma_renesas_rcar_data *)(_dev)->data)
struct dma_renesas_descriptor {
	uint32_t DPSAR;
	uint32_t DPDAR;
	uint32_t DPTCR;
	uint32_t DPCHCR;
} __aligned(16);

struct dma_renesas_rcar_channel_data {
	dma_callback_t cb;
	void *user_data;
	uint8_t num_desc;
	enum channel_state state;
	struct dma_status status;
	struct dma_renesas_descriptor desc_list[CONFIG_DMA_RENESAS_RCAR_NUM_DESCRIPTOR];
};

struct dma_renesas_rcar_data {
	DEVICE_MMIO_NAMED_RAM(channel_reg);
	struct dma_context context;
	struct dma_renesas_rcar_channel_data *channel_data;
};

struct dma_renesas_rcar_config {
	DEVICE_MMIO_NAMED_ROM(channel_reg);
	const struct device *dev;
	const uint32_t number_of_channels;
	enum dmac_type type;
	DMAC_t dmac_instance;
	void (*irq_configure)(void);
};

static uint32_t dma_rcar_read(const struct device *dev, uint32_t offs)
{
	return sys_read32(DEVICE_MMIO_NAMED_GET(dev, channel_reg) + offs);
}

static void dma_rcar_write(const struct device *dev, uint32_t offs, uint32_t value)
{
	sys_write32(value, DEVICE_MMIO_NAMED_GET(dev, channel_reg) + offs);
}

static inline int dma_rcar_interrupt_get_status(const struct dma_renesas_rcar_config *cfg,
						uint32_t channel)
{
	const struct device *dev = cfg->dev;
	struct dma_renesas_rcar_data *data = dev->data;
	uint32_t value;
	uint8_t dev_p = GET_PUBLIC_DEV_INDEX(cfg->dmac_instance);

	/* Select Interrupt Ch */
	value = R_RTDMAC_Get_RDMOR(dev_p);

	/* Check Address error Flag */
	if ((value & DRV_RTDMAC_REG_RDMOR_AE) != 0) {
		/* DMAC address error interrupt occurs during DMA transfer */
		R_RTDMAC_Clear_RDMCHCR_CAE(cfg->dmac_instance, channel);
		R_RTDMAC_Clear_RDMCHCR_DE(cfg->dmac_instance, channel);
		data->channel_data[channel].status.busy = false;
		data->channel_data[channel].state = DMA_CONFIGURED;
		return -EFAULT;
	}

	/* Check TE Interrupt */
	value = R_RTDMAC_Get_RDMCHCR(cfg->dmac_instance, channel);
	if ((value & DRV_RTDMAC_REG_RDMCHCR_TE) != 0) {
		R_RTDMAC_Clear_RDMCHCR_TE(cfg->dmac_instance, channel);
		if (DRV_RTDMAC_REG_RDMCHCR_DPM_REP !=
			(value & DRV_RTDMAC_REG_RDMCHCR_DPM)) {
			R_RTDMAC_Clear_RDMCHCR_DE(cfg->dmac_instance, channel);
			data->channel_data[channel].status.busy = false;
			data->channel_data[channel].state = DMA_CONFIGURED;
		}
		return DMA_STATUS_COMPLETE;
	}

	/* Check DSE Interrupt */
	if ((value & DRV_RTDMAC_REG_RDMCHCR_DSE) != 0) {
		R_RTDMAC_Clear_RDMCHCR_DSE(cfg->dmac_instance, channel);
		if (DRV_RTDMAC_REG_RDMCHCR_DPM_READ ==
			(value & DRV_RTDMAC_REG_RDMCHCR_DPM)) {
			R_RTDMAC_Get_RDMDPCR(cfg->dmac_instance, channel);
		}
		return DMA_STATUS_COMPLETE;
	}

	return -EIO;
}

static void dma_rcar_isr(const struct device *dev, uint32_t channel)
{
	struct dma_renesas_rcar_data *data = dev->data;
	const struct dma_renesas_rcar_config *cfg = dev->config;
	int callback_status;

	callback_status = dma_rcar_interrupt_get_status(cfg, channel);

	if (data->channel_data[channel].cb != NULL) {
		data->channel_data[channel].cb(dev, data->channel_data[channel].user_data, channel,
					       callback_status);
	}
}

static int dma_renesas_rcar_configure(const struct device *dev, uint32_t channel,
				      struct dma_config *config)
{
	struct dma_renesas_rcar_data *data = dev->data;
	const struct dma_renesas_rcar_config *cfg = dev->config;
	struct dma_block_config *block_iter;
	uint8_t tsval;
	rDmacCfg_t hal_dmac_cfg = {0};
	rDmacDescCfg_t hal_descriptor_config = {0};
	uint16_t ret;
	uint32_t transfer_size;
	struct dma_renesas_descriptor *p_desc;

	if (channel >= cfg->number_of_channels) {
		LOG_ERR("Invalid channel number");
		return -EINVAL;
	}

	if ((config->source_handshake != 0) || (config->dest_handshake != 0)) {
		LOG_ERR("Handshake is not supported");
		return -ENOTSUP;
	}

	/* Channel chaining is not supported */
	if ((config->source_chaining_en != 0) || (config->dest_chaining_en != 0)) {
		LOG_ERR("Channel chaining is not supported");
		return -ENOTSUP;
	}

	/* Check burst length validity */
	if (config->source_burst_length != config->dest_burst_length) {
		LOG_ERR("Burst length must be Equalled between source and dest");
		return -EINVAL;
	}

	/* Setup descriptors */
	transfer_size = MIN(config->source_burst_length, config->dest_burst_length);

	hal_dmac_cfg.mSrcAddr = config->head_block->source_address;
	hal_dmac_cfg.mDestAddr = config->head_block->dest_address;
	hal_dmac_cfg.mTransferCount = config->head_block->block_size / transfer_size;

	if (config->cyclic != 0) {
		hal_dmac_cfg.mDMAMode = DRV_DMAC_DMA_DESC_REPEAT;
	} else {
		hal_dmac_cfg.mDMAMode = DRV_DMAC_DMA_DESC_NORMAL;
	}

	if (config->head_block->source_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		hal_dmac_cfg.mSrcAddrMode = DRV_RTDMAC_ADDR_DECREMENTED;
	} else if (config->head_block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		hal_dmac_cfg.mSrcAddrMode = DRV_RTDMAC_ADDR_INCREMENTED;
	} else {
		hal_dmac_cfg.mSrcAddrMode = DRV_RTDMAC_ADDR_FIXED;
	}

	if (config->head_block->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		hal_dmac_cfg.mDestAddrMode = DRV_RTDMAC_ADDR_DECREMENTED;
	} else if (config->head_block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		hal_dmac_cfg.mDestAddrMode = DRV_RTDMAC_ADDR_INCREMENTED;
	} else {
		hal_dmac_cfg.mDestAddrMode = DRV_RTDMAC_ADDR_FIXED;
	}

	switch (transfer_size) {
	case 1:
		hal_dmac_cfg.mTransferUnit = DRV_RTDMAC_TRANS_UNIT_1BYTE;
		tsval = DESCRIPTOR_TRANSFER_1_BYTE;
		break;
	case 2:
		hal_dmac_cfg.mTransferUnit = DRV_RTDMAC_TRANS_UNIT_2BYTE;
		tsval = DESCRIPTOR_TRANSFER_2_BYTE;
		break;
	case 4:
		hal_dmac_cfg.mTransferUnit = DRV_RTDMAC_TRANS_UNIT_4BYTE;
		tsval = DESCRIPTOR_TRANSFER_4_BYTE;
		break;
	case 8:
		hal_dmac_cfg.mTransferUnit = DRV_RTDMAC_TRANS_UNIT_8BYTE;
		tsval = DESCRIPTOR_TRANSFER_8_BYTE;
		break;
	case 16:
		hal_dmac_cfg.mTransferUnit = DRV_RTDMAC_TRANS_UNIT_16BYTE;
		tsval = DESCRIPTOR_TRANSFER_16_BYTE;
		break;
	case 32:
		hal_dmac_cfg.mTransferUnit = DRV_RTDMAC_TRANS_UNIT_32BYTE;
		tsval = DESCRIPTOR_TRANSFER_32_BYTE;
		break;
	case 64:
		hal_dmac_cfg.mTransferUnit = DRV_RTDMAC_TRANS_UNIT_64BYTE;
		tsval = DESCRIPTOR_TRANSFER_64_BYTE;
		break;

	default:
		LOG_ERR("Invalid size");
		return -ENOTSUP;
	}

	if (config->channel_direction == MEMORY_TO_MEMORY) {
		hal_dmac_cfg.mResource = DRV_RTDMAC_MEMORY;
	} else {
		hal_dmac_cfg.mResource = DRV_RTDMAC_EXTEND;
		hal_dmac_cfg.mSourceRequest = config->dma_slot;
	}

	hal_dmac_cfg.mLowSpeed = DRV_RTDMAC_SPEED_NORMAL;
	hal_dmac_cfg.mPrioLevel = config->channel_priority;

	hal_descriptor_config.mDescBaseAddr = (uint32_t)&data->channel_data[channel].desc_list[0];
	hal_descriptor_config.mDescUpdate.mSrcAddrUpdate = true;
	hal_descriptor_config.mDescUpdate.mDestAddrUpdate = true;
	hal_descriptor_config.mDescUpdate.mTransCountUpdate = true;
	hal_descriptor_config.mDescUpdate.mCHCRUpdate = true;
	hal_descriptor_config.mDescRead1st = true;
	hal_descriptor_config.mStateEndEnable = false;
	hal_descriptor_config.mDescCount = config->block_count;
	hal_descriptor_config.mDescIndex = 0;

	data->channel_data[channel].num_desc = config->block_count;

	block_iter = config->head_block;
	for (int i = 0; i < config->block_count; i++) {
		if (block_iter == NULL) {
			LOG_ERR("Invalid block config at index %d", i);
			return -EINVAL;
		}

		if (block_iter->block_size > RCAR_DMA_DESCRRIPTOR_MAXIMUM_TRANSFER_SIZE) {
			LOG_ERR("Invalid block_size");
			return -EINVAL;
		}

		if (block_iter->source_address % config->source_data_size != 0) {
			LOG_ERR("Unaligned source address");
			return -EINVAL;
		}

		if (block_iter->dest_address % config->dest_data_size != 0) {
			LOG_ERR("Unaligned dest address");
			return -EINVAL;
		}

		if (block_iter->block_size % transfer_size != 0) {
			LOG_ERR("Block size must be multiple of data size");
			return -EINVAL;
		}

		p_desc = &data->channel_data[channel].desc_list[i];

		p_desc->DPSAR = block_iter->source_address;
		p_desc->DPDAR = block_iter->dest_address;
		p_desc->DPTCR = block_iter->block_size / transfer_size;

		p_desc->DPCHCR = 0;
		p_desc->DPCHCR |= (config->dma_slot & DESCRIPTOR_CHCR_DRS_MASK)
				  << DESCRIPTOR_CHCR_DRS_POS;
		if (block_iter->source_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
			p_desc->DPCHCR |= (DESCRIPTOR_ADDRESS_MODE_DEC & DESCRIPTOR_CHCR_SM_MASK)
					  << DESCRIPTOR_CHCR_SM_POS;
		} else if (block_iter->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			p_desc->DPCHCR |= (DESCRIPTOR_ADDRESS_MODE_INC & DESCRIPTOR_CHCR_SM_MASK)
					  << DESCRIPTOR_CHCR_SM_POS;
		}

		if (block_iter->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
			p_desc->DPCHCR |= (DESCRIPTOR_ADDRESS_MODE_DEC & DESCRIPTOR_CHCR_DM_MASK)
					  << DESCRIPTOR_CHCR_DM_POS;
		} else if (block_iter->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			p_desc->DPCHCR |= (DESCRIPTOR_ADDRESS_MODE_INC & DESCRIPTOR_CHCR_DM_MASK)
					  << DESCRIPTOR_CHCR_DM_POS;
		}

		if (config->channel_direction == MEMORY_TO_MEMORY) {
			p_desc->DPCHCR |= (DESCRIPTOR_RESOURCE_AUTOREQ & DESCRIPTOR_CHCR_RS_MASK)
					  << DESCRIPTOR_CHCR_RS_POS;
		} else {
			p_desc->DPCHCR |= (DESCRIPTOR_RESOURCE_EXTENDED & DESCRIPTOR_CHCR_RS_MASK)
					  << DESCRIPTOR_CHCR_RS_POS;
		}

		p_desc->DPCHCR |= (tsval & DESCRIPTOR_CHCR_TS_MASK) << DESCRIPTOR_CHCR_TS_POS;

		sys_cache_data_flush_range(p_desc, sizeof(*p_desc));

		block_iter = block_iter->next_block;
	}

	ret = R_DMAC_RcarDmacExec(cfg->dmac_instance, channel, &hal_dmac_cfg,
				  &hal_descriptor_config);
	if (ret != 0) {
		LOG_ERR("R DMAC Configure failed");
		return -EIO;
	}

	data->channel_data[channel].cb = config->dma_callback;
	data->channel_data[channel].user_data = config->user_data;
	data->channel_data[channel].status.busy = false;
	data->channel_data[channel].status.dir = config->channel_direction;

	data->channel_data[channel].state = DMA_CONFIGURED;

	return 0;
}

static int dma_renesas_rcar_reload(const struct device *dev, uint32_t channel, uint32_t src,
				   uint32_t dst, size_t size)
{
	struct dma_renesas_rcar_data *data = dev->data;
	const struct dma_renesas_rcar_config *cfg = dev->config;
	uint32_t chcr_val;
	uint32_t chcrb_val;
	uint32_t transfer_size;
	uint32_t total_desc;
	uint32_t current_index;
	uint32_t reload_index;
	struct dma_renesas_descriptor *desc;

	if (channel >= cfg->number_of_channels) {
		LOG_ERR("Invalid channel");
		return -EINVAL;
	}

	struct dma_renesas_rcar_channel_data *ch_data = &data->channel_data[channel];

	if (ch_data->state == DMA_IDLE) {
		LOG_ERR("Channel not configured");
		return -EINVAL;
	}

	chcrb_val = dma_rcar_read(dev, RCAR_DMA_CHCRB + channel * RCAR_CHANNEL_OFFSET);
	total_desc = ((chcrb_val >> CHCRB_DCNT_POS) & CHCRB_DCNT_MASK) + 1;
	current_index = ((chcrb_val >> CHCRB_DPTR_POS) & CHCRB_DPTR_MASK);
	reload_index = (current_index + 1) % total_desc;
	desc = &ch_data->desc_list[reload_index];

	/* Get the transfer size */
	switch (((desc->DPCHCR) >> DESCRIPTOR_CHCR_TS_POS) & DESCRIPTOR_CHCR_TS_MASK) {
	case DESCRIPTOR_TRANSFER_1_BYTE:
		transfer_size = 1;
		break;
	case DESCRIPTOR_TRANSFER_2_BYTE:
		transfer_size = 2;
		break;
	case DESCRIPTOR_TRANSFER_4_BYTE:
		transfer_size = 4;
		break;
	case DESCRIPTOR_TRANSFER_8_BYTE:
		transfer_size = 8;
		break;
	case DESCRIPTOR_TRANSFER_16_BYTE:
		transfer_size = 16;
		break;
	case DESCRIPTOR_TRANSFER_32_BYTE:
		transfer_size = 32;
		break;
	case DESCRIPTOR_TRANSFER_64_BYTE:
		transfer_size = 64;
		break;
	default:
		transfer_size = 1;
		break;
	}

	/* Update information */
	desc->DPSAR = src;
	desc->DPDAR = dst;
	desc->DPTCR = size / transfer_size;

	sys_cache_data_flush_range(desc, sizeof(*desc));

	/* Update Descriptor for channel */
	chcr_val = dma_rcar_read(dev, RCAR_DMA_CHCR + channel * RCAR_CHANNEL_OFFSET);
	chcr_val |= (CHCR_DPB_MASK << CHCR_DPB_POS);
	dma_rcar_write(dev, RCAR_DMA_CHCR + channel * RCAR_CHANNEL_OFFSET, chcr_val);

	return 0;
}

static int dma_renesas_rcar_start(const struct device *dev, uint32_t channel)
{
	struct dma_renesas_rcar_data *data = dev->data;
	const struct dma_renesas_rcar_config *cfg = dev->config;
	uint32_t chcr_val;

	if (channel >= cfg->number_of_channels) {
		LOG_ERR("Invalid channel");
		return -EINVAL;
	}

	struct dma_renesas_rcar_channel_data *ch_data = &data->channel_data[channel];

	if (data->channel_data[channel].status.busy) {
		return 0;
	}

	/* Enable DMA channel transfer */
	chcr_val = dma_rcar_read(dev, RCAR_DMA_CHCR + channel * RCAR_CHANNEL_OFFSET);
	chcr_val |= RCAR_DMA_CHCR_CHANNEL_ENABLE;
	dma_rcar_write(dev, RCAR_DMA_CHCR + channel * RCAR_CHANNEL_OFFSET, chcr_val);

	ch_data->status.busy = true;
	ch_data->state = DMA_RUNNING;

	return 0;
}

static int dma_renesas_rcar_stop(const struct device *dev, uint32_t channel)
{
	struct dma_renesas_rcar_data *data = dev->data;
	const struct dma_renesas_rcar_config *cfg = dev->config;
	uint16_t ret;

	if (channel >= cfg->number_of_channels) {
		LOG_ERR("Invalid channel");
		return -EINVAL;
	}

	struct dma_renesas_rcar_channel_data *ch_data = &data->channel_data[channel];

	if (data->channel_data[channel].status.busy == false) {
		return 0;
	}

	/* Disable DMA channel transfer */
	ret = R_DMAC_RcarDmacStop(cfg->dmac_instance, channel);
	if (ret != 0) {
		LOG_ERR("Failed to stop DMAC");
		return -EIO;
	}

	ch_data->status.busy = false;
	ch_data->state = DMA_CONFIGURED;

	return 0;
}

static int dma_renesas_rcar_get_status(const struct device *dev, uint32_t channel,
				       struct dma_status *status)
{
	struct dma_renesas_rcar_data *data = dev->data;
	const struct dma_renesas_rcar_config *cfg = dev->config;

	if (channel >= cfg->number_of_channels) {
		LOG_ERR("Invalid channel");
		return -EINVAL;
	}

	status->busy = data->channel_data[channel].status.busy;
	status->dir = data->channel_data[channel].status.dir;
	status->pending_length = dma_rcar_read(dev, RCAR_DMA_TSR + channel * RCAR_CHANNEL_OFFSET);

	return 0;
}

static bool dma_renesas_rcar_chan_filter(const struct device *dev, int channel, void *filter_param)
{
	const struct dma_renesas_rcar_config *cfg = dev->config;

	if (channel >= cfg->number_of_channels) {
		LOG_ERR("Invalid channel");
		return false;
	}

	if (filter_param == NULL) {
		return true;
	}

	if (channel == *(int *)filter_param) {
		return true;
	}

	return false;
}

static void dma_renesas_rcar_chan_release(const struct device *dev, uint32_t channel)
{
	struct dma_renesas_rcar_data *data = dev->data;
	const struct dma_renesas_rcar_config *cfg = dev->config;

	if (channel >= cfg->number_of_channels) {
		LOG_ERR("Invalid channel");
		return;
	}

	struct dma_renesas_rcar_channel_data *ch_data = &data->channel_data[channel];

	/* Check if channel is disabled */
	if (ch_data->state == DMA_RUNNING) {
		LOG_ERR("Channel %d Busy", channel);
		return;
	}

	/* Clear channel config registers */
	dma_rcar_write(dev, RCAR_DMA_CHCLR + channel * RCAR_CHANNEL_OFFSET, CHCLR_CLR);
	ch_data->state = DMA_IDLE;
}

static DEVICE_API(dma, dma_renesas_rcar_driver_api) = {
	.config = dma_renesas_rcar_configure,
	.reload = dma_renesas_rcar_reload,
	.start = dma_renesas_rcar_start,
	.stop = dma_renesas_rcar_stop,
	.get_status = dma_renesas_rcar_get_status,
	.chan_filter = dma_renesas_rcar_chan_filter,
	.chan_release = dma_renesas_rcar_chan_release,
};

static int dma_renesas_rcar_init(const struct device *dev)
{
	const struct dma_renesas_rcar_config *cfg = dev->config;
	uint16_t ret;

	DEVICE_MMIO_NAMED_MAP(dev, channel_reg, K_MEM_CACHE_NONE);

	/* Configure DMA-related interrupts */
	cfg->irq_configure();

	/* Enable DMA Controller */
	ret = R_DMAC_RcarDmacCtrlInit(cfg->dmac_instance, DRV_RTDMAC_PRIO_FIX);
	if (ret != 0) {
		return -EIO;
	}

	return 0;
}

#define DMA_RENESAS_RCAR_ISR(channel_num, dma_num)                                                 \
	static void dma_rcar_isr##channel_num(const struct device *dev)                            \
	{                                                                                          \
		volatile uint32_t mintsta =                                                        \
			*(uint32_t *)(DT_INST_REG_ADDR_BY_NAME(dma_num, intsta) +                  \
				      channel_num * 0x2);                                          \
		if ((mintsta & 0x01) != 0) {                                                       \
			dma_rcar_isr(dev, channel_num);                                            \
		} else {                                                                           \
			dma_rcar_isr(dev, channel_num + 1);                                        \
		}                                                                                  \
	}

#define _DMA_RENESAS_RCAR_ISR_DEFINE(channel_num, dma_num)                                         \
	COND_CODE_1(DT_INST_IRQ_HAS_NAME(dma_num, ch##channel_num),                                \
		    (DMA_RENESAS_RCAR_ISR(channel_num, dma_num)), ())

#define DMA_RENESAS_RCAR_ISR_DEFINE(index)                                                         \
	LISTIFY(DT_INST_PROP(index, dma_channels), _DMA_RENESAS_RCAR_ISR_DEFINE, (;), index);

#define DMA_RENESAS_RCAR_IRQ_CONFIGURE(channel_num, dma_num)                                       \
	COND_CODE_1(DT_INST_IRQ_HAS_NAME(dma_num, ch##channel_num),                                \
		    (IRQ_CONNECT(DT_INST_IRQ_BY_NAME(dma_num, ch##channel_num, irq),               \
				 DT_INST_IRQ_BY_NAME(dma_num, ch##channel_num, priority),          \
				 dma_rcar_isr##channel_num,                                        \
				 DEVICE_DT_INST_GET(dma_num), 0);                                  \
		     irq_enable(DT_INST_IRQ_BY_NAME(dma_num, ch##channel_num, irq));), ())

#define DMA_RCAR_IRQ_CONFIG(index)                                                                 \
	LISTIFY(DT_INST_PROP(index, dma_channels), DMA_RENESAS_RCAR_IRQ_CONFIGURE, (;), index);

#define DMA_RENESAS_RCAR_INIT(index)                                                               \
                                                                                                   \
	DMA_RENESAS_RCAR_ISR_DEFINE(index)                                                         \
                                                                                                   \
	static void dma_renesas_rcar_irq_config_##index(void)                                      \
	{                                                                                          \
		DMA_RCAR_IRQ_CONFIG(index)                                                         \
	}                                                                                          \
                                                                                                   \
	static const struct dma_renesas_rcar_config dma_rcar_config##index = {                     \
		.dev = DEVICE_DT_INST_GET(index),                                                  \
		DEVICE_MMIO_NAMED_ROM_INIT(channel_reg, DT_DRV_INST(index)),                       \
		.number_of_channels = DT_INST_PROP(index, dma_channels),                           \
		.type = DT_INST_ENUM_IDX(index, dma_type),                                         \
		.dmac_instance = DMAC_INSTANCE(DT_INST_STRING_UPPER_TOKEN(index, dma_type),        \
					       DT_INST_PROP(index, channel)),                      \
		.irq_configure = dma_renesas_rcar_irq_config_##index,                              \
	};                                                                                         \
                                                                                                   \
	static struct dma_renesas_rcar_channel_data                                                \
		dma_rcar_channel_data##index[DT_INST_PROP(index, dma_channels)] = {0};             \
                                                                                                   \
	ATOMIC_DEFINE(dma_renesas_rcar_atomic##index, DT_INST_PROP(index, dma_channels));          \
                                                                                                   \
	static struct dma_renesas_rcar_data dma_rcar_data##index = {                               \
		.context =                                                                         \
			{                                                                          \
				.magic = DMA_MAGIC,                                                \
				.atomic = dma_renesas_rcar_atomic##index,                          \
				.dma_channels = DT_INST_PROP(index, dma_channels),                 \
			},                                                                         \
		.channel_data = dma_rcar_channel_data##index,                                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, dma_renesas_rcar_init, NULL, &dma_rcar_data##index,           \
			      &dma_rcar_config##index, PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,     \
			      &dma_renesas_rcar_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_RENESAS_RCAR_INIT)
