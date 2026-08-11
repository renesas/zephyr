/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_i2s_ssi

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/i2s/i2s_renesas_rcar_ssi.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/clock_control/renesas_rcar_adg.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>

#ifdef CONFIG_I2S_RENESAS_RCAR_SSI_DMA
#include <zephyr/drivers/dma.h>
#include <zephyr/cache.h>
#endif /* CONFIG_I2S_RENESAS_RCAR_SSI_DMA */

/* Timeout, in microseconds, when polling an SSISR status bit */
#define TIMEOUT_REG_SSISR 1000

/* Width, in bytes, of the SSITDR/SSIRDR data registers */
#define I2S_RCAR_SSI_BUF_SIZE_BYTES 4U

/* SSI interrupt enable/disable bits */
#define I2S_RCAR_SSICR_DIEN BIT(SSI_SSICR_DIEN_POS) /* Data Interrupt Enable */
#define I2S_RCAR_SSICR_OIEN BIT(SSI_SSICR_OIEN_POS) /* Overflow Interrupt Enable */
#define I2S_RCAR_SSICR_UIEN BIT(SSI_SSICR_UIEN_POS) /* Underflow Interrupt Enable */

/* SSIU interrupt enable/disable bits */
#define I2S_RCAR_SSIU_INT_ENABLE_UIRQ_IE BIT(SSIU_INT_ENABLE_UIRQ_IE_POS) /* Underflow */
#define I2S_RCAR_SSIU_INT_ENABLE_OIRQ_IE BIT(SSIU_INT_ENABLE_OIRQ_IE_POS) /* Overflow */
#define I2S_RCAR_SSIU_INT_ENABLE_DIRQ_IE BIT(SSIU_INT_ENABLE_DIRQ_IE_POS) /* Data */

/*
 * Interrupts kept enabled while a stream runs: in DMA mode the DMA controller
 * moves the data, so only the underflow/overflow error interrupts are needed,
 * while in CPU mode the data interrupt drives every word transfer.
 */
#ifdef CONFIG_I2S_RENESAS_RCAR_SSI_DMA
#define I2S_RCAR_SSICR_INT_MSK (I2S_RCAR_SSICR_UIEN | I2S_RCAR_SSICR_OIEN)
#define I2S_RCAR_SSIU_INT_MSK  (I2S_RCAR_SSIU_INT_ENABLE_UIRQ_IE | I2S_RCAR_SSIU_INT_ENABLE_OIRQ_IE)
#else
#define I2S_RCAR_SSICR_INT_MSK (I2S_RCAR_SSICR_DIEN)
#define I2S_RCAR_SSIU_INT_MSK  (I2S_RCAR_SSIU_INT_ENABLE_DIRQ_IE)
#endif

LOG_MODULE_REGISTER(i2s_rcar, CONFIG_I2S_LOG_LEVEL);

/* CPG/MSSR module clock */
struct i2s_rcar_module_clock_config {
	const struct device *dev; /* clock controller */
	struct rcar_cpg_clk cpg;  /* module and domain identifiers */
};

/* ADG audio clock feeding the SSI serial bit clock */
struct i2s_rcar_adg_clock_config {
	const struct device *dev; /* clock controller */
	uint32_t adg_clksrc;      /* audio clock source, see RCAR_ADG_AUDIO_* */
};

/* Queue entry describing one memory block exchanged with the application */
struct i2s_rcar_buf_header {
	void *buf;   /* memory block allocated from the stream mem_slab */
	size_t size; /* number of valid bytes in the memory block */
};

struct i2s_rcar_dma_config {
	const struct device *dev; /* DMA controller */
	const uint32_t channel;   /* DMA channel */
	const uint32_t slot;      /* DMA request slot */
};

/*
 * Placement of a sample inside the 32-bit SSITDR/SSIRDR data registers, which
 * are always accessed MSB aligned.
 */
struct i2s_rcar_ssi_align {
	uint8_t shift;      /* number of bits to shift a sample by */
	uint8_t data_bytes; /* number of bytes consumed per sample in memory */
};

struct stream_cfg {
	enum i2s_state state;  /* internal state */
	struct i2s_config cfg; /* i2s generic config */
	struct k_msgq queue;   /* data queue */
	void *mem_block;       /* current position in the in-progress memory block */
	void *mem_block_orig;  /* start of the in-progress memory block, used to free it */
	size_t mem_block_len;  /* bytes left to transmit (TX) or already received (RX) */
	enum i2s_rcar_ssi_format ssi_format;       /* frame format in use */
	struct i2s_rcar_ssi_align data_align_info; /* sample alignment in SSITDR/SSIRDR */
};

/* Device config */
struct i2s_rcar_config {
	DEVICE_MMIO_NAMED_ROM(ssi);              /* SSI register region */
	DEVICE_MMIO_NAMED_ROM(ssiu);             /* SSIU register region */
	uint32_t ssip_base;                      /* physical base address of SSIP, DMA only */
	uint16_t irq;                            /* SSI interrupt number */
	const struct pinctrl_dev_config *pincfg; /* pins config */

	struct i2s_rcar_module_clock_config ssiclk_domain; /* SSI audio domain clock */
	struct i2s_rcar_module_clock_config ssiclk;        /* SSI clock config */
	struct i2s_rcar_adg_clock_config adgclk;           /* ADG clock control device */

	struct i2s_rcar_buf_header *const tx_buf; /* TX buffer */
	struct i2s_rcar_buf_header *const rx_buf; /* RX buffer */

#ifdef CONFIG_I2S_RENESAS_RCAR_SSI_DMA
	struct i2s_rcar_dma_config tx_dma;
	struct i2s_rcar_dma_config rx_dma;
#endif /* CONFIG_I2S_RENESAS_RCAR_SSI_DMA */

	void (*en_irq)(void); /* connect and enable the SSI interrupt */
};

/* Device data */
struct i2s_rcar_data {
	DEVICE_MMIO_NAMED_RAM(ssi);  /* SSI register region */
	DEVICE_MMIO_NAMED_RAM(ssiu); /* SSIU register region */
	struct stream_cfg tx;        /* transmit stream */
	struct stream_cfg rx;        /* receive stream */
	bool stop_with_draining;     /* last stop request was I2S_TRIGGER_DRAIN */
};

/* Required by the DEVICE_MMIO_NAMED_* accessors */
#define DEV_CFG(dev)  ((const struct i2s_rcar_config *)((dev)->config))
#define DEV_DATA(dev) ((struct i2s_rcar_data *)((dev)->data))

/* Linear base addresses of the two CPU accessible register regions */
#define SSI_BASE(dev)  DEVICE_MMIO_NAMED_GET(dev, ssi)
#define SSIU_BASE(dev) DEVICE_MMIO_NAMED_GET(dev, ssiu)

/* Low level SSI/SSIU helpers */
static inline void i2s_rcar_ssi_enable(const struct device *dev, bool enable);
static inline void i2s_rcar_ssi_dma_enable(const struct device *dev, bool enable);
static inline void i2s_rcar_ssi_clear_status(const struct device *dev);
static inline void i2s_rcar_ssi_int(const struct device *dev, bool enable);
static inline void i2s_rcar_ssiu_busif_enable_transfer(const struct device *dev, bool enable);
static inline void i2s_rcar_ssi_halt(const struct device *dev);

/* Enable or disable the SSI module (SSICR.EN) */
static inline void i2s_rcar_ssi_enable(const struct device *dev, bool enable)
{
	uint32_t ssicr = sys_read32(SSI_BASE(dev) + SSI_SSICR_OFFSET);

	if (enable) {
		ssicr |= BIT(SSI_SSICR_EN_POS);
	} else {
		ssicr &= ~BIT(SSI_SSICR_EN_POS);
	}
	sys_write32(ssicr, SSI_BASE(dev) + SSI_SSICR_OFFSET);
}

/* Enable or disable the DMA request generation of the SSI (SSICR.DMEN) */
static inline void i2s_rcar_ssi_dma_enable(const struct device *dev, bool enable)
{
	uint32_t ssicr = sys_read32(SSI_BASE(dev) + SSI_SSICR_OFFSET);

	if (enable) {
		ssicr |= BIT(SSI_SSICR_DMEN_POS);
	} else {
		ssicr &= ~BIT(SSI_SSICR_DMEN_POS);
	}
	sys_write32(ssicr, SSI_BASE(dev) + SSI_SSICR_OFFSET);
}

/* Clear the latched SSI and SSIU status flags */
static inline void i2s_rcar_ssi_clear_status(const struct device *dev)
{
	sys_write32(0U, SSI_BASE(dev) + SSI_SSISR_OFFSET);
	sys_write32(0U, SSIU_BASE(dev) + SSIU_STATUS_OFFSET);
}

/* Enable or mask the SSI and SSIU interrupt sources used by the current transfer mode */
static inline void i2s_rcar_ssi_int(const struct device *dev, bool enable)
{
	uint32_t cr = sys_read32(SSI_BASE(dev) + SSI_SSICR_OFFSET);
	uint32_t int_en = sys_read32(SSIU_BASE(dev) + SSIU_INT_ENABLE_OFFSET);

	if (enable) {
		cr |= I2S_RCAR_SSICR_INT_MSK;
		int_en |= I2S_RCAR_SSIU_INT_MSK;
	} else {
		cr &= ~(I2S_RCAR_SSICR_INT_MSK);
		int_en &= ~(I2S_RCAR_SSIU_INT_MSK);
	}

	sys_write32(cr, SSI_BASE(dev) + SSI_SSICR_OFFSET);
	sys_write32(int_en, SSIU_BASE(dev) + SSIU_INT_ENABLE_OFFSET);
}

/* Start or stop the data transfer between the BUSIF and the SSI */
static inline void i2s_rcar_ssiu_busif_enable_transfer(const struct device *dev, bool enable)
{
	if (enable) {
		sys_write32(1U, SSIU_BASE(dev) + SSIU_CONTROL_OFFSET(0));
	} else {
		sys_write32(0U, SSIU_BASE(dev) + SSIU_CONTROL_OFFSET(0));
	}
}

/* Disable the interrupts and stop the transmission/reception */
static inline void i2s_rcar_ssi_halt(const struct device *dev)
{
	/* Stop the module */
	i2s_rcar_ssi_enable(dev, false);

	/* Mask all SSI interrupts */
	i2s_rcar_ssi_int(dev, false);

	/* Clear latched status flags */
	i2s_rcar_ssi_clear_status(dev);

	if (WAIT_FOR((sys_read32(SSI_BASE(dev) + SSI_SSISR_OFFSET) & SSI_SSISR_IDST_MSK) != 0,
		     TIMEOUT_REG_SSISR, NULL) == false) {
		LOG_ERR("Serial bus activity has not stopped");
		return;
	}
}

#ifdef CONFIG_I2S_RENESAS_RCAR_SSI_DMA
/*
 * D-cache maintenance for one audio buffer.
 *
 * TX: the CPU has just filled it, so clean the lines out to DRAM before the
 *     DMAC reads them.
 * RX: drop stale lines up front, so the CPU cannot later evict a dirty line
 *     over data the DMAC has written.
 */
static inline void i2s_rcar_dma_cache_prepare(void *buf, size_t size, enum i2s_dir dir)
{
	if (dir == I2S_DIR_TX) {
		sys_cache_data_flush_range(buf, size);
	} else {
		sys_cache_data_invd_range(buf, size);
	}
}

/*
 * Dequeue the next block written by the application and make it the in-progress
 * TX block. Returns a negative errno when the queue is empty.
 */
static int i2s_rcar_tx_get_buffer(struct i2s_rcar_data *data, struct i2s_rcar_buf_header *item)
{
	int ret = 0;

	ret = k_msgq_get(&data->tx.queue, item, K_NO_WAIT);
	if (ret < 0) {
		return ret;
	}

	data->tx.mem_block = item->buf;
	data->tx.mem_block_orig = item->buf;
	data->tx.mem_block_len = item->size;

	return 0;
}

/*
 * Allocate a block from the RX mem_slab and make it the in-progress RX block.
 * Returns a negative errno when no block is available.
 */
static int i2s_rcar_rx_alloc_buffer(struct i2s_rcar_data *data, struct i2s_rcar_buf_header *item)
{
	int ret = 0;

	ret = k_mem_slab_alloc(data->rx.cfg.mem_slab, &data->rx.mem_block, K_NO_WAIT);
	if (ret < 0) {
		return ret;
	}

	data->rx.mem_block_orig = data->rx.mem_block;
	data->rx.mem_block_len = data->rx.cfg.block_size;

	item->buf = data->rx.mem_block;
	item->size = data->rx.cfg.block_size;

	return 0;
}

/* Return the in-progress block of a stream to its mem_slab */
static void i2s_rcar_release_buffer(struct stream_cfg *stream)
{
	k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block_orig);
	stream->mem_block = NULL;
	stream->mem_block_orig = NULL;
	stream->mem_block_len = 0;
}

/*
 * Tear down a DMA driven transfer. The shutdown order differs per direction:
 * TX drains the data already handed to the SSI before the BUSIF is stopped,
 * while RX stops the BUSIF first so that no further sample is pushed to memory.
 */
static int i2s_rcar_dma_stop_transfer(const struct device *dev, enum i2s_dir dir)
{
	const struct i2s_rcar_config *config = dev->config;
	const mm_reg_t ssi_base = SSI_BASE(dev);
	int ret = 0;
	const struct i2s_rcar_dma_config *dma =
		(dir == I2S_DIR_TX) ? &config->tx_dma : &config->rx_dma;

	if (dir == I2S_DIR_TX) {
		/* Disable DMA */
		i2s_rcar_ssi_dma_enable(dev, false);

		if (WAIT_FOR((sys_read32(ssi_base + SSI_SSISR_OFFSET) & SSI_SSISR_DIRQ_MSK) != 0,
			     TIMEOUT_REG_SSISR, NULL) == false) {
			LOG_ERR("Wait for DIRQ flag timeout! Cannot stop I2S DMA");
		}

		/* Disable SSI */
		i2s_rcar_ssi_enable(dev, false);

		if (WAIT_FOR((sys_read32(ssi_base + SSI_SSISR_OFFSET) & SSI_SSISR_IDST_MSK) != 0,
			     TIMEOUT_REG_SSISR, NULL) == false) {
			LOG_ERR("Timeout waiting for SSI to be IDLE!");
		}
	}

	/* Stop BUSIF transfer */
	i2s_rcar_ssiu_busif_enable_transfer(dev, false);

	if (dir == I2S_DIR_RX) {
		/* Disable SSI and DMA */
		i2s_rcar_ssi_dma_enable(dev, false);
		i2s_rcar_ssi_enable(dev, false);

		if (WAIT_FOR((sys_read32(ssi_base + SSI_SSISR_OFFSET) & SSI_SSISR_IDST_MSK) != 0,
			     TIMEOUT_REG_SSISR, NULL) == false) {
			LOG_ERR("Timeout waiting for SSI to be IDLE!");
		}
	}

	/* Stop DMA */
	ret = dma_stop(dma->dev, dma->channel);
	if (ret < 0) {
		LOG_ERR("Failed to stop DMA for I2S");
		return -EIO;
	}

	return 0;
}

/*
 * Bring up a DMA driven transfer. RX enables the BUSIF before the SSI so that
 * the first sample is captured, TX enables it last so that the FIFO is primed
 * by the DMA controller before the serial engine starts consuming data.
 */
static int i2s_rcar_dma_start_transfer(const struct device *dev, enum i2s_dir dir)
{
	const struct i2s_rcar_config *config = dev->config;
	const struct i2s_rcar_dma_config *dma =
		(dir == I2S_DIR_TX) ? &config->tx_dma : &config->rx_dma;
	int ret = 0;

	/* Start DMA */
	ret = dma_start(dma->dev, dma->channel);
	if (ret < 0) {
		LOG_ERR("Failed to start DMA transfer");
		return -EIO;
	}

	if (dir == I2S_DIR_RX) {
		/* Start BUSIF transfer */
		i2s_rcar_ssiu_busif_enable_transfer(dev, true);
	}

	/* Enable interrupts */
	i2s_rcar_ssi_int(dev, true);

	/* Enable SSI and DMA */
	i2s_rcar_ssi_dma_enable(dev, true);
	i2s_rcar_ssi_enable(dev, true);

	if (dir == I2S_DIR_TX) {
		/* Start BUSIF transfer */
		i2s_rcar_ssiu_busif_enable_transfer(dev, true);
	}

	return 0;
}

/* Stop the DMA transfer and force the SSI back to idle, ignoring any error */
static void i2s_rcar_dma_abort(const struct device *dev, enum i2s_dir dir)
{
	i2s_rcar_dma_stop_transfer(dev, dir);
	i2s_rcar_ssi_halt(dev);
}

/*
 * DMA completion callback for the TX stream. Releases the block that has just
 * been sent and chains the next one from the queue, or stops the stream when
 * the application requested it or ran out of data.
 */
static void i2s_rcar_dma_tx_callback(const struct device *dma_dev, void *user_data,
				     uint32_t channel, int status)
{
	struct device *dev = (struct device *)user_data;
	const struct i2s_rcar_config *config = dev->config;
	struct i2s_rcar_data *data = dev->data;
	struct i2s_rcar_buf_header item;
	int ret = 0;

	if (status != DMA_STATUS_COMPLETE) {
		LOG_ERR("DMA error: %d", status);
		data->tx.state = I2S_STATE_ERROR;
		i2s_rcar_dma_abort(dev, I2S_DIR_TX);
		return;
	}

	/* Free the memory block that has just been transmitted */
	i2s_rcar_release_buffer(&data->tx);

	if (data->tx.state == I2S_STATE_STOPPING && data->stop_with_draining == false) {
		data->tx.state = I2S_STATE_READY;
		i2s_rcar_dma_abort(dev, I2S_DIR_TX);
		return;
	}

	/* Restart TX DMA */
	/* Get data from queue */
	ret = i2s_rcar_tx_get_buffer(data, &item);
	if (ret < 0) {
		if (data->tx.state == I2S_STATE_STOPPING) {
			data->tx.state = I2S_STATE_READY;
		} else {
			/* Unexpected TX underflow. */
			LOG_ERR("TX underflow");
			data->tx.state = I2S_STATE_ERROR;
		}
		i2s_rcar_dma_abort(dev, I2S_DIR_TX);
		return;
	}

	/* Push the freshly written samples out of the D-cache */
	sys_cache_data_flush_range(item.buf, item.size);

	/* Reload DMA for I2S TX */
	ret = dma_reload(config->tx_dma.dev, config->tx_dma.channel, (uint32_t)item.buf,
			 config->ssip_base + SSIP_BUSIF_OFFSET(0), item.size);
	if (ret < 0) {
		LOG_ERR("Failed to reload DMA for I2S TX");
		i2s_rcar_release_buffer(&data->tx);
		data->tx.state = I2S_STATE_ERROR;
		i2s_rcar_dma_abort(dev, I2S_DIR_TX);
		return;
	}

	/* Clear latched status */
	i2s_rcar_ssi_clear_status(dev);

	/* Start DMA */
	ret = dma_start(config->tx_dma.dev, config->tx_dma.channel);
	if (ret < 0) {
		LOG_ERR("Failed to start DMA for I2S TX");
		i2s_rcar_release_buffer(&data->tx);
		data->tx.state = I2S_STATE_ERROR;
		i2s_rcar_dma_abort(dev, I2S_DIR_TX);
		return;
	}
}

/*
 * DMA completion callback for the RX stream. Queues the block that has just
 * been filled for i2s_read() and chains a freshly allocated one, or stops the
 * stream when the application requested it.
 */
static void i2s_rcar_dma_rx_callback(const struct device *dma_dev, void *user_data,
				     uint32_t channel, int status)
{
	struct device *dev = (struct device *)user_data;
	const struct i2s_rcar_config *config = dev->config;
	struct i2s_rcar_data *data = dev->data;
	struct i2s_rcar_buf_header item;
	int ret = 0;

	if (status != DMA_STATUS_COMPLETE) {
		LOG_ERR("DMA error: %d", status);
		data->rx.state = I2S_STATE_ERROR;
		i2s_rcar_dma_abort(dev, I2S_DIR_RX);
		return;
	}

	if (data->rx.mem_block == NULL) {
		LOG_ERR("RX mem_block NULL");
		data->rx.state = I2S_STATE_ERROR;
		i2s_rcar_dma_abort(dev, I2S_DIR_RX);
		return;
	}

	item.buf = data->rx.mem_block;
	item.size = data->rx.mem_block_len;

	/* The DMAC wrote straight to DRAM — drop stale lines so the
	 * application reads the captured data, not the cache.
	 */
	sys_cache_data_invd_range(item.buf, item.size);

	ret = k_msgq_put(&data->rx.queue, &item, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("RX queue full");
		i2s_rcar_release_buffer(&data->rx);
		data->rx.state = I2S_STATE_ERROR;
		i2s_rcar_dma_abort(dev, I2S_DIR_RX);
		return;
	}

	if (data->rx.state == I2S_STATE_STOPPING) {
		data->rx.state = I2S_STATE_READY;
		i2s_rcar_dma_abort(dev, I2S_DIR_RX);
		return;
	}

	/* Allocate a new buffer for the next block */
	ret = i2s_rcar_rx_alloc_buffer(data, &item);
	if (ret < 0) {
		LOG_ERR("Failed to allocate mem slab");
		data->rx.state = I2S_STATE_ERROR;
		i2s_rcar_dma_abort(dev, I2S_DIR_RX);
		return;
	}

	/* Discard stale lines so none can be evicted over incoming DMA data */
	sys_cache_data_invd_range(item.buf, item.size);

	/* Reload DMA for I2S RX */
	ret = dma_reload(config->rx_dma.dev, config->rx_dma.channel,
			 config->ssip_base + SSIP_BUSIF_OFFSET(0), (uint32_t)item.buf, item.size);
	if (ret < 0) {
		LOG_ERR("Failed to configure DMA for I2S RX");
		i2s_rcar_release_buffer(&data->rx);
		data->rx.state = I2S_STATE_ERROR;
		i2s_rcar_dma_abort(dev, I2S_DIR_RX);
		return;
	}

	/* Clear latched status */
	i2s_rcar_ssi_clear_status(dev);

	/* Start DMA */
	ret = dma_start(config->rx_dma.dev, config->rx_dma.channel);
	if (ret < 0) {
		LOG_ERR("Failed to start DMA for I2S RX");
		i2s_rcar_release_buffer(&data->rx);
		data->rx.state = I2S_STATE_ERROR;
		i2s_rcar_dma_abort(dev, I2S_DIR_RX);
		return;
	}
}

/*
 * Program one DMA block transfer between the memory block described by
 * buf_header and the BUSIF data port, in the direction given by dir.
 */
static int i2s_rcar_dma_configure(const struct device *dev, struct i2s_rcar_buf_header *buf_header,
				  uint32_t busif, enum i2s_dir dir)
{
	const struct i2s_rcar_config *config = dev->config;
	struct i2s_rcar_data *data = dev->data;
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	const struct i2s_rcar_dma_config *dma = NULL;
	struct stream_cfg *stream = NULL;

	if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else {
		stream = &data->rx;
	}

	dma_cfg.channel_direction =
		(dir == I2S_DIR_TX) ? MEMORY_TO_PERIPHERAL : PERIPHERAL_TO_MEMORY;
	dma_cfg.dma_slot = (dir == I2S_DIR_TX) ? config->tx_dma.slot : config->rx_dma.slot;
	dma_cfg.source_data_size = stream->data_align_info.data_bytes;
	dma_cfg.source_burst_length = stream->data_align_info.data_bytes;
	dma_cfg.dest_data_size = stream->data_align_info.data_bytes;
	dma_cfg.dest_burst_length = stream->data_align_info.data_bytes;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback =
		(dir == I2S_DIR_TX) ? i2s_rcar_dma_tx_callback : i2s_rcar_dma_rx_callback;
	dma_cfg.complete_callback_en = true;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;

	if (dir == I2S_DIR_TX) {
		dma_blk.block_size = buf_header->size;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_blk.source_address = (uint32_t)(buf_header->buf);
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_blk.dest_address = (uint32_t)busif;

		dma = &config->tx_dma;
	} else {
		dma_blk.block_size = buf_header->size;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_blk.source_address = (uint32_t)busif;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_blk.dest_address = (uint32_t)(buf_header->buf);

		dma = &config->rx_dma;
	}

	i2s_rcar_dma_cache_prepare(buf_header->buf, buf_header->size, dir);

	return dma_config(dma->dev, dma->channel, &dma_cfg);
}

/* Kick off the TX stream: take the first queued block and start the DMA on it */
static int i2s_rcar_dma_tx_set(const struct device *dev)
{
	const struct i2s_rcar_config *config = dev->config;
	struct i2s_rcar_data *data = dev->data;
	struct i2s_rcar_buf_header item;
	int ret = 0;

	/* Get data from queue */
	ret = i2s_rcar_tx_get_buffer(data, &item);
	if (ret < 0) {
		LOG_ERR("No data in queue");
		return -ENOMEM;
	}

	/* Setup DMA for I2S TX */
	ret = i2s_rcar_dma_configure(dev, &item, config->ssip_base + SSIP_BUSIF_OFFSET(0),
				     I2S_DIR_TX);
	if (ret < 0) {
		i2s_rcar_release_buffer(&data->tx);
		LOG_ERR("Failed to configure DMA for I2S TX");
		return -EIO;
	}

	/* Start DMA */
	ret = i2s_rcar_dma_start_transfer(dev, I2S_DIR_TX);
	if (ret < 0) {
		LOG_ERR("Failed to start DMA for I2S TX");
		/* Free the memory block that could not be transmitted */
		i2s_rcar_release_buffer(&data->tx);
		return -EIO;
	}

	return 0;
}

/* Kick off the RX stream: allocate the first block and start the DMA into it */
static int i2s_rcar_dma_rx_set(const struct device *dev)
{
	const struct i2s_rcar_config *config = dev->config;
	struct i2s_rcar_data *data = dev->data;
	struct i2s_rcar_buf_header item;
	int ret = 0;

	ret = i2s_rcar_rx_alloc_buffer(data, &item);
	if (ret < 0) {
		LOG_ERR("Failed to allocate mem slab");
		return -ENOMEM;
	}

	/* Setup DMA for I2S RX */
	ret = i2s_rcar_dma_configure(dev, &item, config->ssip_base + SSIP_BUSIF_OFFSET(0),
				     I2S_DIR_RX);
	if (ret < 0) {
		LOG_ERR("Failed to configure DMA for I2S RX");
		i2s_rcar_release_buffer(&data->rx);
		return -EIO;
	}

	/* Start DMA */
	ret = i2s_rcar_dma_start_transfer(dev, I2S_DIR_RX);
	if (ret < 0) {
		LOG_ERR("Failed to start DMA for I2S RX");
		i2s_rcar_release_buffer(&data->rx);
		return -EIO;
	}

	return 0;
}

/*
 * Restart the current block after an underflow or an overflow. The stream is
 * moved to the ERROR state and the SSI is halted if the restart itself fails.
 */
static void i2s_rcar_dma_recover(const struct device *dev, enum i2s_dir dir)
{
	const struct i2s_rcar_config *config = dev->config;
	struct i2s_rcar_data *data = dev->data;
	struct stream_cfg *stream = NULL;
	struct i2s_rcar_buf_header item;
	int ret = 0;

	if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else {
		stream = &data->rx;
	}

	/* Stop DMA */
	ret = i2s_rcar_dma_stop_transfer(dev, dir);
	if (ret < 0) {
		LOG_ERR("Failed to stop DMA for I2S");

		stream->state = I2S_STATE_ERROR;
		i2s_rcar_ssi_halt(dev);
		return;
	}

	item.buf = stream->mem_block_orig;
	item.size = stream->mem_block_len;

	/* Reconfigure DMA for I2S */
	ret = i2s_rcar_dma_configure(dev, &item, config->ssip_base + SSIP_BUSIF_OFFSET(0), dir);
	if (ret < 0) {
		LOG_ERR("Failed to reload DMA for I2S");

		i2s_rcar_release_buffer(stream);
		stream->state = I2S_STATE_ERROR;
		i2s_rcar_ssi_halt(dev);
		return;
	}

	/* Start DMA */
	ret = i2s_rcar_dma_start_transfer(dev, dir);
	if (ret < 0) {
		LOG_ERR("Failed to start DMA for I2S");

		i2s_rcar_release_buffer(stream);
		stream->state = I2S_STATE_ERROR;
		i2s_rcar_ssi_halt(dev);
		return;
	}
}
#else
/* Push one word to the SSI, called from the data interrupt in CPU mode */
static void i2s_rcar_ssi_write(const struct device *dev)
{
	struct i2s_rcar_data *const data = dev->data;
	struct stream_cfg *stream = &data->tx;
	struct i2s_rcar_buf_header item;

	if (stream->state != I2S_STATE_RUNNING && stream->state != I2S_STATE_STOPPING) {
		return;
	}

	/* A new block is required to continue transmitting */
	if (stream->mem_block == NULL) {
		if (stream->state == I2S_STATE_STOPPING && data->stop_with_draining == false) {
			/* Stop transmit. */
			stream->state = I2S_STATE_READY;
			i2s_rcar_ssi_halt(dev);
			return;
		}

		if (k_msgq_get(&stream->queue, &item, K_NO_WAIT) < 0) {
			if (stream->state == I2S_STATE_STOPPING) {
				stream->state = I2S_STATE_READY;
			} else {
				/* Unexpected TX underflow. */
				stream->state = I2S_STATE_ERROR;
			}
			i2s_rcar_ssi_halt(dev);
			return;
		}

		stream->mem_block = item.buf;
		stream->mem_block_orig = item.buf;
		stream->mem_block_len = item.size;
	}

	/* MSB align the sample before writing it to SSITDR */
	uint32_t tx_data = 0;

	memcpy(&tx_data, stream->mem_block, stream->data_align_info.data_bytes);

	tx_data <<= stream->data_align_info.shift;

	/* Push one word; writing SSITDR also clears the pending DIRQ. */
	sys_write32(tx_data, SSI_BASE(dev) + SSI_SSITDR_OFFSET);

	if (stream->mem_block_len <= stream->data_align_info.data_bytes) {
		/* Last word of this block consumed: release it. */
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block_orig);
		stream->mem_block = NULL;
		stream->mem_block_orig = NULL;
		stream->mem_block_len = 0;
	} else {
		stream->mem_block =
			(uint8_t *)stream->mem_block + stream->data_align_info.data_bytes;
		stream->mem_block_len -= stream->data_align_info.data_bytes;
	}
}

/* Pull one word from the SSI, called from the data interrupt in CPU mode */
static void i2s_rcar_ssi_read(const struct device *dev)
{
	struct i2s_rcar_data *const data = dev->data;
	struct stream_cfg *stream = &data->rx;

	if (stream->state != I2S_STATE_RUNNING && stream->state != I2S_STATE_STOPPING) {
		return;
	}

	if (stream->mem_block == NULL) {
		return;
	}

	/* Pull one word; reading SSIRDR also clears the pending DIRQ */
	uint32_t rx_data = sys_read32(SSI_BASE(dev) + SSI_SSIRDR_OFFSET);

	/* Right align the sample before storing it in memory */
	rx_data >>= stream->data_align_info.shift;

	memcpy(stream->mem_block, &rx_data, stream->data_align_info.data_bytes);
	stream->mem_block = (uint8_t *)stream->mem_block + stream->data_align_info.data_bytes;
	stream->mem_block_len += stream->data_align_info.data_bytes;

	if (stream->mem_block_len >= stream->cfg.block_size) {
		struct i2s_rcar_buf_header entry = {
			.buf = stream->mem_block_orig,
			.size = stream->mem_block_len,
		};

		int ret = k_msgq_put(&stream->queue, &entry, K_NO_WAIT);

		if (ret < 0) {
			LOG_WRN("RX queue full, dropping a block");
			k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block_orig);
		}

		stream->mem_block = NULL;
		stream->mem_block_orig = NULL;
		stream->mem_block_len = 0;

		if (stream->state == I2S_STATE_STOPPING) {
			/* Stop receive */
			stream->state = I2S_STATE_READY;
			i2s_rcar_ssi_halt(dev);
			return;
		}

		/* Allocate new memory slab for next block */
		ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block, K_NO_WAIT);

		if (ret < 0) {
			/* No more memory slab left */
			stream->state = I2S_STATE_ERROR;
			i2s_rcar_ssi_halt(dev);
			return;
		}

		stream->mem_block_orig = stream->mem_block;
	}
}
#endif /* CONFIG_I2S_RENESAS_RCAR_SSI_DMA */

/* Translate the i2s_config data format into SSICR/SSIWSR bit fields */
static int i2s_rcar_set_format(uint32_t *ssicr, uint32_t *ssiwsr, const struct i2s_config *i2s_cfg,
			       struct stream_cfg *stream)
{
	uint32_t format = i2s_cfg->format;
	uint32_t channels = i2s_cfg->channels;

	switch (format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		/* I2S Format */
		if (channels != 2) {
			LOG_ERR("Channels must equal 2 for I2S Format");
			return -EINVAL;
		}

		*ssicr &= ~(BIT(SSI_SSICR_DEL_POS) | BIT(SSI_SSICR_SDTA_POS) |
			    BIT(SSI_SSICR_PDTA_POS) | BIT(SSI_SSICR_SWSP_POS) |
			    BIT(SSI_SSICR_SCKP_POS) | BIT(SSI_SSICR_CHNL_POS) |
			    BIT(SSI_SSICR_SPDP_POS));

		*ssiwsr &= ~(BIT(SSI_SSIWSR_WS_MODE_POS) | BIT(SSI_SSIWSR_MONO_POS) |
			     SSI_SSIWSR_WIDTH_MSK);

		stream->ssi_format = I2S_RCAR_FORMAT_STEREO;
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		/* Left-Aligned Format */
		if (channels != 2) {
			LOG_ERR("Channels must equal 2 for Left-Aligned Format");
			return -EINVAL;
		}

		*ssicr &= ~(BIT(SSI_SSICR_SDTA_POS) | BIT(SSI_SSICR_PDTA_POS) |
			    BIT(SSI_SSICR_SCKP_POS) | BIT(SSI_SSICR_SPDP_POS) |
			    BIT(SSI_SSICR_CHNL_POS));

		*ssicr |= (BIT(SSI_SSICR_SWSP_POS) | BIT(SSI_SSICR_DEL_POS));

		*ssiwsr &= ~(BIT(SSI_SSIWSR_WS_MODE_POS) | BIT(SSI_SSIWSR_MONO_POS) |
			     SSI_SSIWSR_WIDTH_MSK);

		stream->ssi_format = I2S_RCAR_FORMAT_STEREO;
		break;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		/* Right-Aligned Format */
		if (channels != 2) {
			LOG_ERR("Channels must equal 2 for Right-Aligned Format");
			return -EINVAL;
		}

		*ssicr &= ~(BIT(SSI_SSICR_PDTA_POS) | BIT(SSI_SSICR_SCKP_POS) |
			    BIT(SSI_SSICR_SPDP_POS) | BIT(SSI_SSICR_CHNL_POS));

		*ssicr |= (BIT(SSI_SSICR_SWSP_POS) | BIT(SSI_SSICR_DEL_POS) |
			   BIT(SSI_SSICR_SDTA_POS));

		*ssiwsr &= ~(BIT(SSI_SSIWSR_WS_MODE_POS) | BIT(SSI_SSIWSR_MONO_POS) |
			     SSI_SSIWSR_WIDTH_MSK);

		stream->ssi_format = I2S_RCAR_FORMAT_STEREO;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		/* Monaural Format */
		if (channels != 1) {
			/* The only corresponding hardware configuration for Zephyr
			 * PCM Short Frame Sync Data Format is monaural, hence limiting
			 * the channel number of this format to 1
			 */
			LOG_ERR("Channels must equal 1 for Monaural Format");
			return -EINVAL;
		}

		*ssicr &= ~(BIT(SSI_SSICR_CHNL_POS) | BIT(SSI_SSICR_SDTA_POS) |
			    BIT(SSI_SSICR_PDTA_POS) | BIT(SSI_SSICR_SPDP_POS) |
			    BIT(SSI_SSICR_SWSP_POS));

		*ssicr |= (BIT(SSI_SSICR_DEL_POS) | BIT(SSI_SSICR_SCKP_POS));

		*ssiwsr &= ~SSI_SSIWSR_WIDTH_MSK;
		*ssiwsr |= (BIT(SSI_SSIWSR_WS_MODE_POS) | BIT(SSI_SSIWSR_MONO_POS) |
			    FIELD_PREP(SSI_SSIWSR_WIDTH_MSK, 1));

		stream->ssi_format = I2S_RCAR_FORMAT_MONAURAL;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		if (channels == 1) {
			/* Monaural Format */
			*ssicr &= ~(BIT(SSI_SSICR_CHNL_POS) | BIT(SSI_SSICR_SDTA_POS) |
				    BIT(SSI_SSICR_PDTA_POS) | BIT(SSI_SSICR_SPDP_POS) |
				    BIT(SSI_SSICR_SWSP_POS));
			*ssicr |= (BIT(SSI_SSICR_DEL_POS) | BIT(SSI_SSICR_SCKP_POS));

			*ssiwsr &= ~SSI_SSIWSR_WIDTH_MSK;
			*ssiwsr |= (BIT(SSI_SSIWSR_WS_MODE_POS) | BIT(SSI_SSIWSR_MONO_POS) |
				    FIELD_PREP(SSI_SSIWSR_WIDTH_MSK, 1));

			stream->ssi_format = I2S_RCAR_FORMAT_MONAURAL;

		} else if (channels == 4 || channels == 6 || channels == 8) {
			/* TDM Format */
			*ssicr &= ~(BIT(SSI_SSICR_SDTA_POS) | BIT(SSI_SSICR_PDTA_POS) |
				    BIT(SSI_SSICR_SPDP_POS) | BIT(SSI_SSICR_SWSP_POS));
			*ssicr |=
				(BIT(SSI_SSICR_DEL_POS) | BIT(SSI_SSICR_SCKP_POS) |
				 FIELD_PREP(SSI_SSICR_CHNL_MSK,
					    (channels >> 1) - 1)); /* Map channels to corresponding
								    *  bit field value
								    */

			*ssiwsr &= ~(BIT(SSI_SSIWSR_MONO_POS) | SSI_SSIWSR_WIDTH_MSK);
			*ssiwsr |=
				(BIT(SSI_SSIWSR_WS_MODE_POS) | FIELD_PREP(SSI_SSIWSR_WIDTH_MSK, 1));

			stream->ssi_format = I2S_RCAR_FORMAT_TDM;
		} else {
			LOG_ERR("Invalid channel number for PCM Long Frame Sync Data Format");
			return -EINVAL;
		}

		break;
	default:
		LOG_ERR("Data format not supported");
		return -EINVAL;
	}

	return 0;
}

/* Translate the word size into the SSICR DWL/SWL and SSIU ADINR OTBL bit fields */
static int i2s_rcar_set_word_size(uint32_t *ssicr, uint32_t *ssiu_adinr, uint8_t word_size,
				  struct stream_cfg *stream)
{
	/* Clear both DWL and SWL fields; SWL must match DWL so BCK/LRCK ratio is correct */
	*ssicr &= ~(SSI_SSICR_DWL_MSK | SSI_SSICR_SWL_MSK);
	*ssiu_adinr &= ~(SSIU_BUSIF_ADINR_OTBL_MSK);

	if (stream->ssi_format == I2S_RCAR_FORMAT_MONAURAL) {
		switch (word_size) {
		case 8:
			*ssicr |= FIELD_PREP(SSI_SSICR_DWL_MSK, I2S_RCAR_SSICR_DWL_MONO_8) |
				  FIELD_PREP(SSI_SSICR_SWL_MSK, I2S_RCAR_SSICR_SWL_MONO_16);
			*ssiu_adinr |= FIELD_PREP(SSIU_BUSIF_ADINR_OTBL_MSK,
						  I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN8);
			break;
		case 16:
			*ssicr |= FIELD_PREP(SSI_SSICR_DWL_MSK, I2S_RCAR_SSICR_DWL_MONO_16) |
				  FIELD_PREP(SSI_SSICR_SWL_MSK, I2S_RCAR_SSICR_SWL_MONO_16);
			*ssiu_adinr |= FIELD_PREP(SSIU_BUSIF_ADINR_OTBL_MSK,
						  I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN16);
			break;
		default:
			return -EINVAL;
		}
	} else {
		switch (word_size) {
		case 8:
			*ssicr |= FIELD_PREP(SSI_SSICR_DWL_MSK, I2S_RCAR_SSICR_DWL_STEREO_8) |
				  FIELD_PREP(SSI_SSICR_SWL_MSK, I2S_RCAR_SSICR_SWL_STEREO_8);
			*ssiu_adinr |= FIELD_PREP(SSIU_BUSIF_ADINR_OTBL_MSK,
						  I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN8);
			break;
		case 16:
			*ssicr |= FIELD_PREP(SSI_SSICR_DWL_MSK, I2S_RCAR_SSICR_DWL_STEREO_16) |
				  FIELD_PREP(SSI_SSICR_SWL_MSK, I2S_RCAR_SSICR_SWL_STEREO_16);
			*ssiu_adinr |= FIELD_PREP(SSIU_BUSIF_ADINR_OTBL_MSK,
						  I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN16);
			break;
		case 18:
			*ssicr |= FIELD_PREP(SSI_SSICR_DWL_MSK, I2S_RCAR_SSICR_DWL_STEREO_18) |
				  FIELD_PREP(SSI_SSICR_SWL_MSK, I2S_RCAR_SSICR_SWL_STEREO_24);
			*ssiu_adinr |= FIELD_PREP(SSIU_BUSIF_ADINR_OTBL_MSK,
						  I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN18);
			break;
		case 20:
			*ssicr |= FIELD_PREP(SSI_SSICR_DWL_MSK, I2S_RCAR_SSICR_DWL_STEREO_20) |
				  FIELD_PREP(SSI_SSICR_SWL_MSK, I2S_RCAR_SSICR_SWL_STEREO_24);
			*ssiu_adinr |= FIELD_PREP(SSIU_BUSIF_ADINR_OTBL_MSK,
						  I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN20);
			break;
		case 22:
			*ssicr |= FIELD_PREP(SSI_SSICR_DWL_MSK, I2S_RCAR_SSICR_DWL_STEREO_22) |
				  FIELD_PREP(SSI_SSICR_SWL_MSK, I2S_RCAR_SSICR_SWL_STEREO_24);
			*ssiu_adinr |= FIELD_PREP(SSIU_BUSIF_ADINR_OTBL_MSK,
						  I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN22);
			break;
		case 24:
			*ssicr |= FIELD_PREP(SSI_SSICR_DWL_MSK, I2S_RCAR_SSICR_DWL_STEREO_24) |
				  FIELD_PREP(SSI_SSICR_SWL_MSK, I2S_RCAR_SSICR_SWL_STEREO_24);
			*ssiu_adinr |= FIELD_PREP(SSIU_BUSIF_ADINR_OTBL_MSK,
						  I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN24);
			break;
		case 32:
			*ssicr |= FIELD_PREP(SSI_SSICR_DWL_MSK, I2S_RCAR_SSICR_DWL_STEREO_32) |
				  FIELD_PREP(SSI_SSICR_SWL_MSK, I2S_RCAR_SSICR_SWL_STEREO_32);

			if (IS_ENABLED(CONFIG_I2S_RENESAS_RCAR_SSI_DMA)) {
				LOG_ERR("32-bit data length is not supported by BUSIF");
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * Generate the requested bit clock. The ADG clock feeding the SSI is divided by
 * SSICR.CKDV, so try every divider until the ADG can produce the resulting rate.
 */
static int i2s_rcar_set_bclk(const struct device *dev, uint32_t *ssicr,
			     const struct i2s_config *i2s_cfg, struct stream_cfg *stream)
{
	const struct i2s_rcar_config *const config = dev->config;

	/* SSICR.CKDV encoding: index is the register value, entry is the divider */
	static const uint32_t ckdv_div_table[] = {1, 2, 4, 8, 16, 6, 12};
	uint32_t target_bclk = i2s_cfg->frame_clk_freq * i2s_cfg->channels * i2s_cfg->word_size;

	for (uint8_t ckdv = 0; ckdv < ARRAY_SIZE(ckdv_div_table); ckdv++) {
		uint32_t ckdv_div = ckdv_div_table[ckdv];
		uint64_t req_clk = (uint64_t)target_bclk * ckdv_div;

		/* Setting SSICR_CKDV = 0 is prohibited for TDM or monaural */
		if ((ckdv_div == 1) && (stream->ssi_format != I2S_RCAR_FORMAT_STEREO)) {
			continue;
		}

		if (clock_control_set_rate(config->adgclk.dev,
					   (clock_control_subsys_t)config->adgclk.adg_clksrc,
					   (clock_control_subsys_rate_t)(uintptr_t)req_clk) == 0) {
			*ssicr = (*ssicr & ~SSI_SSICR_CKDV_MSK) |
				 FIELD_PREP(SSI_SSICR_CKDV_MSK, ckdv);
			return 0;
		}
	}

	return -EINVAL;
}

/*
 * Compute how a sample is laid out in the 32-bit SSITDR/SSIRDR registers: how
 * many bytes it occupies in the application buffer and how far it has to be
 * shifted to be MSB aligned in the data register.
 */
static int i2s_rcar_ssi_data_align_configure(const struct i2s_config *i2s_cfg,
					     struct stream_cfg *stream)
{
	struct i2s_rcar_ssi_align *data_align_info = &stream->data_align_info;

	if (stream->ssi_format == I2S_RCAR_FORMAT_MONAURAL) {
		if (i2s_cfg->word_size == 8U || i2s_cfg->word_size == 16U) {
			data_align_info->data_bytes = i2s_cfg->word_size / BITS_PER_BYTE;
			data_align_info->shift =
				I2S_RCAR_SSI_BUF_SIZE_BYTES * BITS_PER_BYTE - i2s_cfg->word_size;
			return 0U;
		}

		return -EINVAL;
	}

	switch (i2s_cfg->word_size) {
	case 8:
	case 16:
	case 32:
		data_align_info->data_bytes = I2S_RCAR_SSI_BUF_SIZE_BYTES;
		data_align_info->shift = 0U;
		break;
	case 18:
	case 20:
	case 22:
	case 24:
		data_align_info->data_bytes = I2S_RCAR_SSI_BUF_SIZE_BYTES;
		data_align_info->shift =
			I2S_RCAR_SSI_BUF_SIZE_BYTES * BITS_PER_BYTE - i2s_cfg->word_size;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * Apply an i2s_config to the SSI and SSIU registers of one direction. Every
 * register is built up locally and only written back once the whole
 * configuration is known to be supported, so a rejected config leaves the
 * hardware untouched.
 */
static int i2s_rcar_ssi_configure(const struct device *dev, const struct i2s_config *i2s_cfg,
				  enum i2s_dir dir)
{
	const struct i2s_rcar_config *const config = dev->config;
	struct i2s_rcar_data *const data = dev->data;
	struct stream_cfg *stream = NULL;
	int ret = 0;

	if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else {
		return -ENOSYS;
	}

	/* LSB first is not supported by the hardware */
	if (i2s_cfg->format & I2S_FMT_DATA_ORDER_LSB) {
		LOG_ERR("LSB first not supported");
		return -EINVAL;
	}

	if (i2s_cfg->options & I2S_OPT_BIT_CLK_GATED) {
		return -EINVAL;
	}

	/* Read back the registers to be reconfigured, they are written back at the end */
	uint32_t ssicr = sys_read32(SSI_BASE(dev) + SSI_SSICR_OFFSET);
	uint32_t ssiwsr = sys_read32(SSI_BASE(dev) + SSI_SSIWSR_OFFSET);
	uint32_t ssiu_mode = sys_read32(SSIU_BASE(dev) + SSIU_BUSIF_MODE_OFFSET(0));
	uint32_t ssiu_adinr = sys_read32(SSIU_BASE(dev) + SSIU_BUSIF_ADINR_OFFSET(0));
	uint32_t ssiu_clksel = sys_read32(SSIU_BASE(dev) + SSIU_AUDIO_CLK_SEL_OFFSET);

	/* Configure BUSIF mode */
	if (IS_ENABLED(CONFIG_I2S_RENESAS_RCAR_SSI_DMA)) {
		/* BUSIF = DMA mode */
		ssiu_mode |= BIT(SSIU_BUSIF_MODE_DMA_POS);
	} else {
		/* BUSIF = CPU mode */
		ssiu_mode &= ~BIT(SSIU_BUSIF_MODE_DMA_POS);
	}

	/* Set the number of channels in the SSIU */
	ssiu_adinr &= ~SSIU_BUSIF_ADINR_CHNUM_MSK;
	ssiu_adinr |= FIELD_PREP(SSIU_BUSIF_ADINR_CHNUM_MSK, i2s_cfg->channels);

	/* Configure the SSI clock source */
	ssiu_clksel &= ~(SSIU_AUDIO_CLK_SEL_DIVSEL_MSK | SSIU_AUDIO_CLK_SEL_ACLK_SEL_MSK |
			 SSIU_AUDIO_CLK_SEL_DIVCLK_SEL_MSK);

	switch (config->adgclk.adg_clksrc) {
	case RCAR_ADG_AUDIO_BRGA:
		ssiu_clksel |= FIELD_PREP(SSIU_AUDIO_CLK_SEL_ACLK_SEL_MSK, I2S_RCAR_CLKSEL_BRGA);
		break;
	case RCAR_ADG_AUDIO_BRGB:
		ssiu_clksel |= FIELD_PREP(SSIU_AUDIO_CLK_SEL_ACLK_SEL_MSK, I2S_RCAR_CLKSEL_BRGB);
		break;
	case RCAR_ADG_AUDIO_AVB0:
	case RCAR_ADG_AUDIO_AVB1:
	case RCAR_ADG_AUDIO_AVB2:
	case RCAR_ADG_AUDIO_AVB3:
	case RCAR_ADG_AUDIO_AVB4:
	case RCAR_ADG_AUDIO_AVB5:
	case RCAR_ADG_AUDIO_AVB6:
	case RCAR_ADG_AUDIO_AVB7:
		ssiu_clksel |=
			FIELD_PREP(SSIU_AUDIO_CLK_SEL_ACLK_SEL_MSK, I2S_RCAR_CLKSEL_AVBCOUNTER8);
		ssiu_clksel |= FIELD_PREP(SSIU_AUDIO_CLK_SEL_DIVSEL_MSK,
					  config->adgclk.adg_clksrc - RCAR_ADG_AUDIO_AVB0);
		break;
	default:
		LOG_ERR("Invalid audio clock source: %d", config->adgclk.adg_clksrc);
		return -ENOSYS;
	}

	/* Configure transmit/receive mode */
	if (dir == I2S_DIR_TX) {
		ssicr |= BIT(SSI_SSICR_TRMD_POS);
		ssicr &= ~BIT(SSI_SSICR_MUEN_POS);
	} else {
		ssicr &= ~BIT(SSI_SSICR_TRMD_POS);
	}

	/* FORCE must be written as 1, keep the module disabled while configuring it */
	ssicr |= BIT(SSI_SSICR_FORCE_POS);
	ssicr &= ~BIT(SSI_SSICR_EN_POS);

	/* Start from a known state: interrupts masked and DMA disabled */
	ssicr &= ~(I2S_RCAR_SSICR_DIEN | I2S_RCAR_SSICR_OIEN | I2S_RCAR_SSICR_UIEN);
	ssicr &= ~BIT(SSI_SSICR_DMEN_POS);

	/* Configure data format */
	ret = i2s_rcar_set_format(&ssicr, &ssiwsr, i2s_cfg, stream);
	if (ret < 0) {
		LOG_ERR("Unsupported format: 0x%08x", i2s_cfg->format);
		return ret;
	}

	/* Configure word size */
	ret = i2s_rcar_set_word_size(&ssicr, &ssiu_adinr, i2s_cfg->word_size, stream);
	if (ret < 0) {
		LOG_ERR("Unsupported word size: %d", i2s_cfg->word_size);
		return ret;
	}

	/* Configure data alignment */
	ret = i2s_rcar_ssi_data_align_configure(i2s_cfg, stream);
	if (ret < 0) {
		LOG_ERR("SSI data alignment configure error code (%d)", ret);
		return ret;
	}

	/* Configure SSIU BUSIF shift mode */
	ssiu_mode &= ~(SSIU_BUSIF_MODE_SFT_NUM_MSK);
	ssiu_mode |= FIELD_PREP(SSIU_BUSIF_MODE_SFT_NUM_MSK,
				(stream->data_align_info.data_bytes * BITS_PER_BYTE) %
					i2s_cfg->word_size);

	/* For 16-bit stereo, swap word order to get the correct word order of SSIU BUSIF */
	if ((i2s_cfg->word_size == 16) && (stream->ssi_format != I2S_RCAR_FORMAT_MONAURAL)) {
		ssiu_mode |= BIT(SSIU_BUSIF_MODE_WORD_SWAP_POS);
	} else {
		ssiu_mode &= ~BIT(SSIU_BUSIF_MODE_WORD_SWAP_POS);
	}

	/* Bit-shift direction for valid bit position of SSIU BUSIF */
	if (dir == I2S_DIR_TX) {
		ssiu_mode &= ~BIT(SSIU_BUSIF_MODE_SFT_DIR_POS);
	} else {
		ssiu_mode |= BIT(SSIU_BUSIF_MODE_SFT_DIR_POS);
	}

	/* Configure the serial bit clock */
	ret = i2s_rcar_set_bclk(dev, &ssicr, i2s_cfg, stream);
	if (ret < 0) {
		LOG_ERR("Unsupported bit rate: %d",
			i2s_cfg->frame_clk_freq * i2s_cfg->channels * i2s_cfg->word_size);
		return ret;
	}

	if (i2s_cfg->options & I2S_OPT_BIT_CLK_TARGET) {
		ssicr &= ~BIT(SSI_SSICR_SCKD_POS);
	} else {
		ssicr |= BIT(SSI_SSICR_SCKD_POS);
	}

	if (i2s_cfg->options & I2S_OPT_FRAME_CLK_TARGET) {
		ssicr &= ~BIT(SSI_SSICR_SWSD_POS);
	} else {
		ssicr |= BIT(SSI_SSICR_SWSD_POS);
	}

	sys_write32(ssicr, SSI_BASE(dev) + SSI_SSICR_OFFSET);
	sys_write32(ssiwsr, SSI_BASE(dev) + SSI_SSIWSR_OFFSET);
	sys_write32(ssiu_mode, SSIU_BASE(dev) + SSIU_BUSIF_MODE_OFFSET(0));
	sys_write32(ssiu_adinr, SSIU_BASE(dev) + SSIU_BUSIF_ADINR_OFFSET(0));
	sys_write32(ssiu_clksel, SSIU_BASE(dev) + SSIU_AUDIO_CLK_SEL_OFFSET);

	return 0;
}

/* Handle I2S_TRIGGER_START: move the stream from READY to RUNNING */
static int i2s_rcar_start(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_rcar_data *const data = dev->data;
	struct stream_cfg *stream = NULL;
	int ret = 0;

	if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else {
		return -ENOSYS;
	}

	if (stream->state != I2S_STATE_READY) {
		LOG_ERR("State is not ready for i2s_start");
		return -EIO;
	}

	stream->state = I2S_STATE_RUNNING;

#ifdef CONFIG_I2S_RENESAS_RCAR_SSI_DMA
	if (dir == I2S_DIR_TX) {
		ret = i2s_rcar_dma_tx_set(dev);
		if (ret < 0) {
			LOG_ERR("Failed to Setup DMA for I2S TX");
			stream->state = I2S_STATE_READY;
			return -EIO;
		}
	} else if (dir == I2S_DIR_RX) {
		ret = i2s_rcar_dma_rx_set(dev);
		if (ret < 0) {
			LOG_ERR("Failed to setup I2S RX DMA");
			stream->state = I2S_STATE_READY;
			return -EIO;
		}
	} else {
		stream->state = I2S_STATE_READY;
		return -ENOSYS;
	}
#else
	if (dir == I2S_DIR_RX) {
		ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("RX mem_slab alloc failed for i2s_start with error code (%d)", ret);
			stream->state = I2S_STATE_READY;
			return ret;
		}
		stream->mem_block_orig = stream->mem_block;
		stream->mem_block_len = 0;
	}

	/* Clear latched status flags */
	i2s_rcar_ssi_clear_status(dev);

	/* Enable interrupts */
	i2s_rcar_ssi_int(dev, true);

	/* Enable SSI */
	i2s_rcar_ssi_enable(dev, true);
#endif
	return 0;
}

/*
 * Handle I2S_TRIGGER_STOP and I2S_TRIGGER_DRAIN: only request the transition,
 * the hardware is stopped by the interrupt or DMA callback once the current
 * block, and for a drain the whole queue, has been transferred.
 */
static int i2s_rcar_stop(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_rcar_data *const data = dev->data;
	struct stream_cfg *stream = NULL;

	if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else {
		return -ENOSYS;
	}

	if (stream->state != I2S_STATE_RUNNING) {
		LOG_ERR("State is not running for i2s_stop");
		return -EIO;
	}
	stream->state = I2S_STATE_STOPPING;

	return 0;
}

/*
 * Handle I2S_TRIGGER_DROP: stop the hardware immediately, discard all queued
 * and in-progress blocks and return the stream to the READY state.
 */
static int i2s_rcar_drop(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_rcar_data *const data = dev->data;
	struct stream_cfg *stream = NULL;
	struct i2s_rcar_buf_header item = {0};

	if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else {
		return -ENOSYS;
	}

	if (stream->state == I2S_STATE_NOT_READY) {
		LOG_ERR("i2s_drop is not valid in NOT_READY state");
		return -EIO;
	}

	i2s_rcar_ssi_halt(dev);

#ifdef CONFIG_I2S_RENESAS_RCAR_SSI_DMA
	i2s_rcar_dma_stop_transfer(dev, dir);
#endif /* CONFIG_I2S_RENESAS_RCAR_SSI_DMA */

	while (k_msgq_get(&stream->queue, &item, K_NO_WAIT) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, item.buf);
	}
	/* Free the partially-filled in-progress buffer */
	if (stream->mem_block_orig != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block_orig);
	}
	stream->mem_block = NULL;
	stream->mem_block_orig = NULL;
	stream->mem_block_len = 0;
	stream->state = I2S_STATE_READY;

	return 0;
}

/*
 * Handle I2S_TRIGGER_PREPARE: this trigger can be used in ERROR state only and
 * changes the interface state to READY, after discarding the blocks left over
 * by the failed transfer.
 */
static int i2s_rcar_prepare(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_rcar_data *const data = dev->data;
	struct i2s_rcar_buf_header item;
	struct stream_cfg *stream = NULL;

	if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else {
		return -ENOSYS;
	}

	if (stream->state != I2S_STATE_ERROR) {
		LOG_ERR("State must be ERROR for i2s_prepare");
		return -EIO;
	}

	while (k_msgq_get(&stream->queue, &item, K_NO_WAIT) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, item.buf);
	}

	/* Free the partially-filled in-progress buffer */
	if (stream->mem_block_orig != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block_orig);
	}

	stream->mem_block = NULL;
	stream->mem_block_orig = NULL;
	stream->mem_block_len = 0;
	stream->state = I2S_STATE_READY;

	return 0;
}

/* Implements i2s_configure() */
static int i2s_rcar_configure(const struct device *dev, enum i2s_dir dir,
			      const struct i2s_config *i2s_cfg)
{
	struct i2s_rcar_data *const data = dev->data;
	struct stream_cfg *stream = NULL;
	int ret = 0;

	if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else {
		return -ENOSYS;
	}

	/* Configure is only valid in NOT_READY or READY state */
	if (stream->state != I2S_STATE_NOT_READY && stream->state != I2S_STATE_READY) {
		LOG_ERR("State error for i2s_configure");
		return -EIO;
	}

	/* frame_clk_freq = 0 means unconfigure (transition to NOT_READY) */
	if (i2s_cfg->frame_clk_freq == 0) {
		stream->state = I2S_STATE_NOT_READY;
		LOG_ERR("Frame_clk_freq is zero for i2s_configure");
		return -EINVAL;
	}

	ret = i2s_rcar_ssi_configure(dev, i2s_cfg, dir);
	if (ret < 0) {
		LOG_ERR("SSI configure error code (%d)", ret);
		return ret;
	}

	uint32_t frame_bytes = i2s_cfg->channels * stream->data_align_info.data_bytes;

	if ((i2s_cfg->block_size % frame_bytes) != 0U) {
		LOG_ERR("Block size must be a multiple of %u for this format", frame_bytes);
		return -EINVAL;
	}

	stream->mem_block = NULL;
	stream->mem_block_orig = NULL;
	stream->mem_block_len = 0;
	stream->state = I2S_STATE_READY;
	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));

	return 0;
}

/* Implements i2s_config_get() */
static const struct i2s_config *i2s_rcar_config_get(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_rcar_data *const data = dev->data;
	struct stream_cfg *stream = NULL;

	if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else {
		return NULL;
	}

	if (stream->state == I2S_STATE_NOT_READY) {
		LOG_DBG("State is not ready for i2s_config_get");
		return NULL;
	}

	return &stream->cfg;
}

/* Implements i2s_trigger() */
static int i2s_rcar_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	struct i2s_rcar_data *const data = dev->data;
	int ret = 0;

	if (dir == I2S_DIR_BOTH) {
		LOG_ERR("I2S_DIR_BOTH is not supported");
		return -ENOSYS;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		ret = i2s_rcar_start(dev, dir);
		data->stop_with_draining = false;
		break;
	case I2S_TRIGGER_STOP:
		ret = i2s_rcar_stop(dev, dir);
		data->stop_with_draining = false;
		break;
	case I2S_TRIGGER_DRAIN:
		ret = i2s_rcar_stop(dev, dir);
		data->stop_with_draining = true;
		break;
	case I2S_TRIGGER_DROP:
		ret = i2s_rcar_drop(dev, dir);
		data->stop_with_draining = false;
		break;
	case I2S_TRIGGER_PREPARE:
		ret = i2s_rcar_prepare(dev, dir);
		data->stop_with_draining = false;
		break;
	default:
		ret = -EINVAL;
	}

	LOG_DBG("i2s_trigger with return code (%d)", ret);

	return ret;
}

/* Implements i2s_read() */
static int i2s_rcar_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_rcar_data *const data = dev->data;
	struct stream_cfg *rx = &data->rx;
	struct i2s_rcar_buf_header item;
	int ret = 0;

	if (rx->state == I2S_STATE_NOT_READY) {
		LOG_ERR("State is not ready for i2s_read");
		return -EIO;
	}

	ret = k_msgq_get(&rx->queue, &item, SYS_TIMEOUT_MS(rx->cfg.timeout));
	if (ret < 0) {
		if (rx->state == I2S_STATE_ERROR) {
			LOG_ERR("State error for i2s_read");
			return -EIO;
		}

		LOG_ERR("Queue get fail for i2s_read with error code (%d)", ret);
		return -EAGAIN;
	}

	*mem_block = item.buf;
	*size = item.size;

	LOG_DBG("i2s_read successfully");

	return 0;
}

/* Implements i2s_write() */
static int i2s_rcar_write(const struct device *dev, void *mem_block, size_t size)
{
	struct i2s_rcar_data *const data = dev->data;
	struct stream_cfg *tx = &data->tx;
	int ret = 0;

	if (tx->state == I2S_STATE_NOT_READY) {
		LOG_ERR("State is not ready for i2s_write");
		return -EIO;
	}

	if (size > tx->cfg.block_size) {
		LOG_ERR("Size is larger than configured block size for i2s_write");
		return -EINVAL;
	}

	struct i2s_rcar_buf_header entry = {
		.buf = mem_block,
		.size = size,
	};

	ret = k_msgq_put(&tx->queue, &entry, SYS_TIMEOUT_MS(tx->cfg.timeout));

	if (ret < 0) {
		LOG_ERR("Queue put fail for i2s_write with error code (%d)", ret);
		return ret;
	}

	LOG_DBG("i2s_write successfully");

	return 0;
}

/*
 * SSI interrupt handler. In DMA mode only the underflow and overflow errors are
 * reported here and trigger a recovery, in CPU mode the data interrupt carries
 * every single word of the transfer.
 */
static void i2s_rcar_isr(const struct device *dev)
{
	struct i2s_rcar_data *const data = dev->data;
	uint32_t ssisr = sys_read32(SSI_BASE(dev) + SSI_SSISR_OFFSET);
	uint32_t ssicr = sys_read32(SSI_BASE(dev) + SSI_SSICR_OFFSET);

#ifdef CONFIG_I2S_RENESAS_RCAR_SSI_DMA
	/* Check underflow/overflow interrupt */
	if (((ssisr & SSI_SSISR_UIRQ_MSK) && (ssicr & I2S_RCAR_SSICR_UIEN)) ||
	    ((ssisr & SSI_SSISR_OIRQ_MSK) && (ssicr & I2S_RCAR_SSICR_OIEN))) {
		if (data->tx.mem_block_orig != NULL) {
			i2s_rcar_dma_recover(dev, I2S_DIR_TX);
		} else if (data->rx.mem_block_orig != NULL) {
			i2s_rcar_dma_recover(dev, I2S_DIR_RX);
		} else {
			/* Do nothing */
		}
	}
#else
	/* Check data interrupt */
	if ((ssisr & SSI_SSISR_DIRQ_MSK) && (ssicr & I2S_RCAR_SSICR_DIEN)) {
		if (data->rx.state == I2S_STATE_RUNNING || data->rx.state == I2S_STATE_STOPPING) {
			i2s_rcar_ssi_read(dev);
		} else if (data->tx.state == I2S_STATE_RUNNING ||
			   data->tx.state == I2S_STATE_STOPPING) {
			i2s_rcar_ssi_write(dev);
		} else {
			/* Do nothing */
		}
	}
#endif
	/* Clear latched status */
	i2s_rcar_ssi_clear_status(dev);
}

/*
 * Device initialisation: apply the pin configuration, turn on the module and
 * audio clocks, hook up the interrupt and put both streams in NOT_READY state.
 */
static int i2s_rcar_init(const struct device *dev)
{
	const struct i2s_rcar_config *const config = dev->config;
	struct i2s_rcar_data *const data = dev->data;
	int err;

	DEVICE_MMIO_NAMED_MAP(dev, ssi, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, ssiu, K_MEM_CACHE_NONE);

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("Apply pinctrl failed with error code (%d)", err);
		return err;
	}

	/* Enable the audio domain clock */
	if (clock_control_get_status(config->ssiclk_domain.dev,
				     (clock_control_subsys_t)&config->ssiclk_domain.cpg) !=
	    CLOCK_CONTROL_STATUS_ON) {
		err = clock_control_on(config->ssiclk_domain.dev,
				       (clock_control_subsys_t)&config->ssiclk_domain.cpg);

		if (err < 0) {
			LOG_ERR("Apply clock control on failed for audio domain clock with error "
				"code (%d)",
				err);
			return err;
		}
	}

	/* Enable the SSI module clock */
	err = clock_control_on(config->ssiclk.dev, (clock_control_subsys_t)&config->ssiclk.cpg);

	if (err < 0) {
		LOG_ERR("Apply clock control on failed for SSI clock with error code (%d)", err);
		return err;
	}

	/* Enable the ADG audio clock */
	err = clock_control_on(config->adgclk.dev,
			       (clock_control_subsys_t)config->adgclk.adg_clksrc);

	if (err < 0) {
		LOG_ERR("Apply clock control on failed for ADG clock with error code (%d)", err);
		return err;
	}

	/* Connect and enable the SSI interrupt */
	(void)config->en_irq();

	k_msgq_init(&data->tx.queue, (char *)config->tx_buf, sizeof(struct i2s_rcar_buf_header),
		    CONFIG_I2S_RENESAS_RCAR_SSI_TX_BLOCK_COUNT);
	k_msgq_init(&data->rx.queue, (char *)config->rx_buf, sizeof(struct i2s_rcar_buf_header),
		    CONFIG_I2S_RENESAS_RCAR_SSI_RX_BLOCK_COUNT);

	data->rx.state = I2S_STATE_NOT_READY;
	data->tx.state = I2S_STATE_NOT_READY;
	data->stop_with_draining = false;

	return 0;
}

static DEVICE_API(i2s, i2s_rcar_driver_api) = {
	.configure = i2s_rcar_configure,
	.config_get = i2s_rcar_config_get,
	.read = i2s_rcar_read,
	.write = i2s_rcar_write,
	.trigger = i2s_rcar_trigger,
};

#ifdef CONFIG_I2S_RENESAS_RCAR_SSI_DMA
#define RCAR_DT_INST_DMA_CTLR(n, name)                                                             \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas),                                                \
		    (DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, name))), (NULL))

#define RCAR_DT_INST_DMA_CELL(n, name, cell)                                                       \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, dmas), (DT_INST_DMAS_CELL_BY_NAME(n, name, cell)),    \
		    (0xff))

#define RCAR_I2S_TX_DMA_INIT(n)                                                                    \
	.tx_dma.dev = RCAR_DT_INST_DMA_CTLR(n, tx),                                                \
	.tx_dma.channel = RCAR_DT_INST_DMA_CELL(n, tx, channel),                                   \
	.tx_dma.slot = RCAR_DT_INST_DMA_CELL(n, tx, slot),
#define RCAR_I2S_RX_DMA_INIT(n)                                                                    \
	.rx_dma.dev = RCAR_DT_INST_DMA_CTLR(n, rx),                                                \
	.rx_dma.channel = RCAR_DT_INST_DMA_CELL(n, rx, channel),                                   \
	.rx_dma.slot = RCAR_DT_INST_DMA_CELL(n, rx, slot),
#else
#define RCAR_I2S_TX_DMA_INIT(n)
#define RCAR_I2S_RX_DMA_INIT(n)
#endif /* CONFIG_I2S_RENESAS_RCAR_SSI_DMA */

#define I2S_RCAR_SSI_INIT(idx)                                                                     \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
                                                                                                   \
	static struct i2s_rcar_buf_header                                                          \
		i2s_##idx##_tx_msgs[CONFIG_I2S_RENESAS_RCAR_SSI_TX_BLOCK_COUNT];                   \
	static struct i2s_rcar_buf_header                                                          \
		i2s_##idx##_rx_msgs[CONFIG_I2S_RENESAS_RCAR_SSI_RX_BLOCK_COUNT];                   \
                                                                                                   \
	static void i2s_##idx##_en_irq(void)                                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), i2s_rcar_isr,           \
			    DEVICE_DT_GET(DT_DRV_INST(idx)), 0);                                   \
		irq_enable(DT_INST_IRQN(idx));                                                     \
	}                                                                                          \
                                                                                                   \
	static const struct i2s_rcar_config i2s_rcar_config_##idx = {                              \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(ssi, DT_DRV_INST(idx)),                         \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(ssiu, DT_DRV_INST(idx)),                        \
		.ssip_base = DT_INST_REG_ADDR_BY_NAME(idx, ssip),                                  \
		.irq = DT_INST_IRQN(idx),                                                          \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                     \
                                                                                                   \
		.ssiclk_domain.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(idx, ssi_domain)),  \
		.ssiclk_domain.cpg.module = DT_INST_CLOCKS_CELL_BY_NAME(idx, ssi_domain, module),  \
		.ssiclk_domain.cpg.domain = DT_INST_CLOCKS_CELL_BY_NAME(idx, ssi_domain, domain),  \
                                                                                                   \
		.ssiclk.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(idx, ssi)),                \
		.ssiclk.cpg.module = DT_INST_CLOCKS_CELL_BY_NAME(idx, ssi, module),                \
		.ssiclk.cpg.domain = DT_INST_CLOCKS_CELL_BY_NAME(idx, ssi, domain),                \
                                                                                                   \
		.adgclk.dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(idx, audio_clk)),          \
		.adgclk.adg_clksrc = DT_INST_CLOCKS_CELL_BY_NAME(idx, audio_clk, clock_source),    \
                                                                                                   \
		.tx_buf = i2s_##idx##_tx_msgs,                                                     \
		.rx_buf = i2s_##idx##_rx_msgs,                                                     \
		.en_irq = i2s_##idx##_en_irq,                                                      \
		RCAR_I2S_TX_DMA_INIT(idx) RCAR_I2S_RX_DMA_INIT(idx)};                              \
                                                                                                   \
	static struct i2s_rcar_data i2s_rcar_data_##idx = {                                        \
		.tx.state = I2S_STATE_NOT_READY,                                                   \
		.rx.state = I2S_STATE_NOT_READY,                                                   \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, i2s_rcar_init, NULL, &i2s_rcar_data_##idx,                      \
			      &i2s_rcar_config_##idx, POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,       \
			      &i2s_rcar_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_RCAR_SSI_INIT)
