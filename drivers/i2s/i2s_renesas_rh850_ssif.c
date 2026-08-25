/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_i2s_ssif

#include <errno.h>
#include <r_ssi.h>
#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#ifdef CONFIG_OBJ_CORE_SEM
#include <zephyr/kernel/obj_core.h>
#endif
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/slist.h>
#include <zephyr/spinlock.h>

LOG_MODULE_REGISTER(rh850_i2s_ssif, CONFIG_I2S_LOG_LEVEL);

#define I2S_SUPPORTED_OPTIONS \
	(I2S_OPT_FRAME_CLK_CONTROLLER | I2S_OPT_FRAME_CLK_TARGET | \
	 I2S_OPT_BIT_CLK_CONTROLLER | I2S_OPT_BIT_CLK_TARGET | \
	 I2S_OPT_BIT_CLK_CONT)

#define SSI_CLOCK_DIV_INVALID ((ssi_clock_div_t)-1)

/* Wait slots are independent of the public enum i2s_dir encoding. */
#define SSIF_RX 0U
#define SSIF_TX 1U
#define SSIF_STREAMS 2U

struct renesas_rh850_ssif_config {
	const struct pinctrl_dev_config *pinctrl_dev;
	const i2s_api_t *fsp_api;
	const struct device *pclk_dev;
	clock_control_subsys_t pclk_subsys;
	const struct device *audio_clock_dev;
	clock_control_subsys_t audio_clock_subsys;
};

/* A private notification prevents another caller from consuming this wakeup. */
struct rh850_ssif_waiter {
	sys_snode_t node;
	struct k_sem ready;
};

struct renesas_rh850_ssif_stream {
	void *mem_block;
	size_t mem_block_len;
};

enum rh850_ssif_hw_phase {
	RH850_SSIF_HW_IDLE,
	RH850_SSIF_HW_ACTIVE,
	RH850_SSIF_HW_STOP_PENDING,
	RH850_SSIF_HW_TX_TAIL,
	RH850_SSIF_HW_CONFIGURING,
};

struct renesas_rh850_ssif_data {
	ssi_instance_ctrl_t *fsp_ctrl;
	i2s_cfg_t *fsp_config;
	ssi_extended_cfg_t *fsp_ext_config;
	struct k_spinlock lock;
	enum i2s_state state;
	enum i2s_dir active_dir;
	struct k_msgq rx_queue;
	struct k_msgq tx_queue;
	uint32_t io_users;
	/* Entries live on the stack of their I/O caller; data->lock protects them. */
	sys_slist_t io_waiters[SSIF_STREAMS];
	uint32_t io_epoch[SSIF_STREAMS];
	uint32_t duplex_pending;
	uint8_t hw_phase;
	struct i2s_config tx_cfg;
	struct i2s_config rx_cfg;
	struct renesas_rh850_ssif_stream tx_stream;
	struct renesas_rh850_ssif_stream rx_stream;
	struct renesas_rh850_ssif_stream
		tx_msgs[CONFIG_I2S_RENESAS_RH850_SSIF_TX_BLOCK_COUNT];
	struct renesas_rh850_ssif_stream
		rx_msgs[CONFIG_I2S_RENESAS_RH850_SSIF_RX_BLOCK_COUNT];
	bool tx_configured;
	bool rx_configured;
	bool stop_with_draining;
	bool full_duplex;
	bool trigger_drop;
};

/* FSP interrupt handlers */
extern void ssi_txi_isr(void);
extern void ssi_rxi_isr(void);
extern void ssi_int_isr(void);

static void rh850_ssif_txi_isr(const struct device *dev)
{
	struct renesas_rh850_ssif_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Lock the entire HAL ISR, including work performed before its callback. */
	ssi_txi_isr();
	k_spin_unlock(&data->lock, key);
}

static void rh850_ssif_rxi_isr(const struct device *dev)
{
	struct renesas_rh850_ssif_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Lock the entire HAL ISR, including work performed before its callback. */
	ssi_rxi_isr();
	k_spin_unlock(&data->lock, key);
}

static void rh850_ssif_int_isr(const struct device *dev)
{
	struct renesas_rh850_ssif_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	/* Lock the entire HAL ISR, including work performed before its callback. */
	ssi_int_isr();
	k_spin_unlock(&data->lock, key);
}

static k_timeout_t rh850_ssif_timeout(int32_t timeout_ms)
{
	if (timeout_ms == SYS_FOREVER_MS) {
		return K_FOREVER;
	}

	if (timeout_ms <= 0) {
		return K_NO_WAIT;
	}

	return K_MSEC(timeout_ms);
}

static void rh850_ssif_release_stream(struct renesas_rh850_ssif_stream *stream)
{
	__ASSERT(stream != NULL, "Invalid stream");

	stream->mem_block = NULL;
	stream->mem_block_len = 0U;
}

static int rh850_ssif_alloc_stream(struct i2s_config *cfg,
				   struct renesas_rh850_ssif_stream *stream)
{
	int ret;

	__ASSERT((cfg != NULL) && (stream != NULL), "Invalid parameter");

	ret = k_mem_slab_alloc(cfg->mem_slab, &stream->mem_block, K_NO_WAIT);
	if (ret == 0) {
		stream->mem_block_len = cfg->block_size;
	} else {
		rh850_ssif_release_stream(stream);
	}

	return ret;
}

static void rh850_ssif_free_stream(struct i2s_config *cfg,
				   struct renesas_rh850_ssif_stream *stream)
{
	__ASSERT((cfg != NULL) && (stream != NULL), "Invalid parameter");

	if (stream->mem_block != NULL) {
		k_mem_slab_free(cfg->mem_slab, stream->mem_block);
	}

	rh850_ssif_release_stream(stream);
}

static int rh850_ssif_put_stream(struct k_msgq *msgq,
				 struct renesas_rh850_ssif_stream *stream)
{
	__ASSERT((msgq != NULL) && (stream != NULL), "Invalid parameter");

	return k_msgq_put(msgq, stream, K_NO_WAIT);
}

static int rh850_ssif_get_stream(struct k_msgq *msgq,
				 struct renesas_rh850_ssif_stream *stream,
				 k_timeout_t timeout)
{
	__ASSERT((msgq != NULL) && (stream != NULL), "Invalid parameter");

	return k_msgq_get(msgq, stream, timeout);
}

/* Caller holds data->lock. ERROR wakes; queue reset invalidates the session. */
static void rh850_ssif_wake_io(struct renesas_rh850_ssif_data *data,
			       enum i2s_dir dir, bool cancel)
{
	size_t index = (dir == I2S_DIR_TX) ? SSIF_TX : SSIF_RX;
	sys_snode_t *node;

	if (cancel) {
		data->io_epoch[index]++;
	}
	while ((node = sys_slist_get(&data->io_waiters[index])) != NULL) {
		struct rh850_ssif_waiter *waiter =
			CONTAINER_OF(node, struct rh850_ssif_waiter, node);

		k_sem_give(&waiter->ready);
	}
}

static void rh850_ssif_error(const struct device *dev)
{
	struct renesas_rh850_ssif_data *data = dev->data;

	if (data->state != I2S_STATE_ERROR) {
		data->state = I2S_STATE_ERROR;
		rh850_ssif_wake_io(data, I2S_DIR_TX, true);
		rh850_ssif_wake_io(data, I2S_DIR_RX, false);
	}
}

static void rh850_ssif_drop_queue(const struct device *dev, enum i2s_dir dir)
{
	struct renesas_rh850_ssif_data *data = dev->data;
	struct i2s_config *cfg = (dir == I2S_DIR_TX) ? &data->tx_cfg : &data->rx_cfg;
	struct k_msgq *msgq = (dir == I2S_DIR_TX) ? &data->tx_queue : &data->rx_queue;
	struct renesas_rh850_ssif_stream stream;

	while (k_msgq_get(msgq, &stream, K_NO_WAIT) == 0) {
		if ((stream.mem_block != NULL) && (cfg->mem_slab != NULL)) {
			k_mem_slab_free(cfg->mem_slab, stream.mem_block);
		}
	}
	rh850_ssif_wake_io(data, dir, true);
}

static int rh850_ssif_request_stop(const struct device *dev)
{
	const struct renesas_rh850_ssif_config *config = dev->config;
	struct renesas_rh850_ssif_data *data = dev->data;
	fsp_err_t err;

	if ((data->hw_phase != RH850_SSIF_HW_ACTIVE) &&
	    (data->hw_phase != RH850_SSIF_HW_TX_TAIL)) {
		return 0;
	}

	data->hw_phase = RH850_SSIF_HW_STOP_PENDING;
	err = config->fsp_api->stop(data->fsp_ctrl);
	if (err != FSP_SUCCESS) {
		if (data->hw_phase == RH850_SSIF_HW_STOP_PENDING) {
			data->hw_phase = RH850_SSIF_HW_ACTIVE;
		}
		rh850_ssif_error(dev);
		LOG_ERR("%s: failed to request SSIF stop: %d", dev->name, err);
		return -EIO;
	}

	return 0;
}

static bool rh850_ssif_configs_compatible(const struct i2s_config *a,
					 const struct i2s_config *b)
{
	return (a->word_size == b->word_size) && (a->channels == b->channels) &&
	       (a->format == b->format) && (a->options == b->options) &&
	       (a->frame_clk_freq == b->frame_clk_freq) && (a->block_size == b->block_size);
}

static ssi_clock_div_t rh850_ssif_get_clock_div_enum(uint32_t divisor)
{
	switch (divisor) {
	case 1U:
		return SSI_CLOCK_DIV_1;
	case 2U:
		return SSI_CLOCK_DIV_2;
	case 4U:
		return SSI_CLOCK_DIV_4;
	case 6U:
		return SSI_CLOCK_DIV_6;
	case 8U:
		return SSI_CLOCK_DIV_8;
	case 12U:
		return SSI_CLOCK_DIV_12;
	case 16U:
		return SSI_CLOCK_DIV_16;
	case 24U:
		return SSI_CLOCK_DIV_24;
	case 32U:
		return SSI_CLOCK_DIV_32;
	case 48U:
		return SSI_CLOCK_DIV_48;
	case 64U:
		return SSI_CLOCK_DIV_64;
	case 96U:
		return SSI_CLOCK_DIV_96;
	case 128U:
		return SSI_CLOCK_DIV_128;
	default:
		return SSI_CLOCK_DIV_INVALID;
	}
}

static int rh850_ssif_set_clock_divider(const struct device *dev,
					const struct i2s_config *i2s_cfg,
					ssi_clock_div_t *clock_div)
{
	const struct renesas_rh850_ssif_config *config = dev->config;
	static const uint32_t valid_divisors[] = {
		1U, 2U, 4U, 6U, 8U, 12U, 16U, 24U, 32U, 48U, 64U, 96U, 128U,
	};

	const uint64_t target_bclk =
		(uint64_t)i2s_cfg->word_size *
		(uint64_t)i2s_cfg->channels *
		(uint64_t)i2s_cfg->frame_clk_freq;
	uint32_t selected_div = 0U;
	uint64_t error_min = UINT64_MAX;
	uint32_t rate;
	int ret;

	ret = clock_control_get_rate(config->audio_clock_dev, config->audio_clock_subsys, &rate);
	if (ret < 0) {
		LOG_ERR("%s: failed to get audio clock rate: %d", dev->name, ret);
		return ret;
	}

	if (rate == 0U) {
		LOG_ERR("%s: audio clock rate is zero", dev->name);
		return -EINVAL;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(valid_divisors); i++) {
		const uint64_t actual_bclk = DIV_ROUND_CLOSEST((uint64_t)rate, valid_divisors[i]);
		uint64_t err;

		if (actual_bclk >= target_bclk) {
			err = DIV_ROUND_UP((uint64_t)1000U * (actual_bclk - target_bclk),
					   target_bclk);
		} else {
			err = DIV_ROUND_UP((uint64_t)1000U * (target_bclk - actual_bclk),
					   target_bclk);
		}

		if ((selected_div == 0U) || (err < error_min)) {
			selected_div = valid_divisors[i];
			error_min = err;

			if (error_min == 0U) {
				break;
			}
		}
	}

	if ((selected_div == 0U) || (error_min > 100U)) {
		LOG_ERR("%s: no suitable SSIF clock for BCLK %llu Hz", dev->name,
			(unsigned long long)target_bclk);
		return -EIO;
	}

	*clock_div = rh850_ssif_get_clock_div_enum(selected_div);

	if (*clock_div == SSI_CLOCK_DIV_INVALID) {
		return -EINVAL;
	}

	LOG_DBG("%s: SSIF clock error %llu/1000", dev->name,
		(unsigned long long)error_min);

	return 0;
}

static size_t rh850_ssif_frame_size_bytes(uint8_t word_size)
{
	switch (word_size) {
	case 8U:
		return 2U;
	case 16U:
		return 4U;
	case 24U:
	case 32U:
		return 8U;
	default:
		return 0U;
	}
}

static int rh850_ssif_apply_config(const struct device *dev,
				 const struct i2s_config *cfg,
				 ssi_clock_div_t clock_div, bool force)
{
	const struct renesas_rh850_ssif_config *config = dev->config;
	struct renesas_rh850_ssif_data *data = dev->data;
	i2s_cfg_t *fsp_cfg = data->fsp_config;
	i2s_cfg_t saved_cfg = *fsp_cfg;
	ssi_extended_cfg_t saved_ext = *data->fsp_ext_config;
	i2s_pcm_width_t pcm_width;
	i2s_word_length_t word_length;
	i2s_mode_t mode = (cfg->options & I2S_OPT_BIT_CLK_TARGET) != 0U ?
		I2S_MODE_SLAVE : I2S_MODE_MASTER;
	bool was_open = data->fsp_ctrl->open != 0U;
	fsp_err_t err;

	switch (cfg->word_size) {
	case 8U:
		pcm_width = I2S_PCM_WIDTH_8_BITS;
		word_length = I2S_WORD_LENGTH_8_BITS;
		break;
	case 16U:
		pcm_width = I2S_PCM_WIDTH_16_BITS;
		word_length = I2S_WORD_LENGTH_16_BITS;
		break;
	case 24U:
		pcm_width = I2S_PCM_WIDTH_24_BITS;
		word_length = I2S_WORD_LENGTH_24_BITS;
		break;
	case 32U:
		pcm_width = I2S_PCM_WIDTH_32_BITS;
		word_length = I2S_WORD_LENGTH_32_BITS;
		break;
	default:
		return -EINVAL;
	}

	if (!force && was_open && (fsp_cfg->pcm_width == pcm_width) &&
	    (fsp_cfg->word_length == word_length) && (fsp_cfg->operating_mode == mode) &&
	    (data->fsp_ext_config->bit_clock_div == clock_div)) {
		return 0;
	}
	if (was_open) {
		err = config->fsp_api->close(data->fsp_ctrl);
		if (err != FSP_SUCCESS) {
			LOG_ERR("%s: R_SSI_Close error: %d", dev->name, err);
			return -EIO;
		}
	}

	fsp_cfg->pcm_width = pcm_width;
	fsp_cfg->word_length = word_length;
	fsp_cfg->ws_continue = I2S_WS_CONTINUE_ON;
	fsp_cfg->operating_mode = mode;
	fsp_cfg->p_transfer_tx = NULL;
	fsp_cfg->p_transfer_rx = NULL;
	fsp_cfg->p_extend = data->fsp_ext_config;
	data->fsp_ext_config->bit_clock_div = clock_div;

	err = config->fsp_api->open(data->fsp_ctrl, fsp_cfg);
	if (err == FSP_SUCCESS) {
		return 0;
	}
	LOG_ERR("%s: R_SSI_Open error: %d", dev->name, err);
	*fsp_cfg = saved_cfg;
	*data->fsp_ext_config = saved_ext;
	if (!was_open || (config->fsp_api->open(data->fsp_ctrl, fsp_cfg) != FSP_SUCCESS)) {
		rh850_ssif_drop_queue(dev, I2S_DIR_TX);
		rh850_ssif_drop_queue(dev, I2S_DIR_RX);
		data->tx_configured = false;
		data->rx_configured = false;
		data->state = I2S_STATE_NOT_READY;
		LOG_ERR("%s: no usable SSIF configuration after open failure", dev->name);
	}
	return -EIO;
}

static int rh850_ssif_rx_start_transfer(const struct device *dev)
{
	const struct renesas_rh850_ssif_config *config = dev->config;
	struct renesas_rh850_ssif_data *data = dev->data;
	struct renesas_rh850_ssif_stream *stream = &data->rx_stream;
	fsp_err_t err;
	int ret;

	ret = rh850_ssif_alloc_stream(&data->rx_cfg, stream);
	if (ret < 0) {
		return ret;
	}

	data->hw_phase = RH850_SSIF_HW_ACTIVE;
	err = config->fsp_api->read(data->fsp_ctrl, stream->mem_block, stream->mem_block_len);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: failed to start RX: %d", dev->name, err);
		rh850_ssif_error(dev);
		return -EIO;
	}

	return 0;
}

static int rh850_ssif_tx_start_transfer(const struct device *dev)
{
	const struct renesas_rh850_ssif_config *config = dev->config;
	struct renesas_rh850_ssif_data *data = dev->data;
	struct renesas_rh850_ssif_stream *stream = &data->tx_stream;
	fsp_err_t err;
	int ret;

	ret = rh850_ssif_get_stream(&data->tx_queue, stream, K_NO_WAIT);
	if (ret < 0) {
		rh850_ssif_error(dev);
		return -ENOMEM;
	}

	rh850_ssif_wake_io(data, I2S_DIR_TX, false);
	if (data->active_dir == I2S_DIR_BOTH) {
		if (stream->mem_block_len != data->rx_cfg.block_size) {
			rh850_ssif_error(dev);
			return -EINVAL;
		}
		data->duplex_pending++;
	}
	data->hw_phase = RH850_SSIF_HW_ACTIVE;
	err = config->fsp_api->write(data->fsp_ctrl, stream->mem_block, stream->mem_block_len);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: failed to start TX: %d", dev->name, err);
		rh850_ssif_error(dev);
		return -EIO;
	}

	return 0;
}

static int rh850_ssif_tx_rx_start_transfer(const struct device *dev)
{
	const struct renesas_rh850_ssif_config *config = dev->config;
	struct renesas_rh850_ssif_data *data = dev->data;
	struct renesas_rh850_ssif_stream *tx_stream = &data->tx_stream;
	struct renesas_rh850_ssif_stream *rx_stream = &data->rx_stream;
	fsp_err_t err;
	int ret;

	/* START already checked every queued length under the same lock. */
	if (k_msgq_num_used_get(&data->tx_queue) == 0U) {
		return -EAGAIN;
	}

	/*
	 * Allocate RX before removing TX. A failed allocation must not move the
	 * first TX block behind later blocks.
	 */
	ret = rh850_ssif_alloc_stream(&data->rx_cfg, rx_stream);
	if (ret < 0) {
		return ret;
	}
	ret = rh850_ssif_get_stream(&data->tx_queue, tx_stream, K_NO_WAIT);
	if (ret < 0) {
		rh850_ssif_free_stream(&data->rx_cfg, rx_stream);
		return ret;
	}

	rh850_ssif_wake_io(data, I2S_DIR_TX, false);
	data->duplex_pending = 1U;
	data->hw_phase = RH850_SSIF_HW_ACTIVE;
	err = config->fsp_api->writeRead(data->fsp_ctrl, tx_stream->mem_block,
				       rx_stream->mem_block, (uint32_t)tx_stream->mem_block_len);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: failed to start full-duplex transfer: %d", dev->name, err);
		rh850_ssif_error(dev);
		return -EIO;
	}

	return 0;
}

static void rh850_ssif_rx_callback(const struct device *dev)
{
	struct renesas_rh850_ssif_data *data = dev->data;
	struct renesas_rh850_ssif_stream completed;
	bool more_tx;
	int ret;

	if (data->trigger_drop || (data->hw_phase != RH850_SSIF_HW_ACTIVE) ||
	    (data->rx_stream.mem_block == NULL)) {
		return;
	}
	completed = data->rx_stream;
	rh850_ssif_release_stream(&data->rx_stream);
	ret = rh850_ssif_put_stream(&data->rx_queue, &completed);
	if (ret < 0) {
		rh850_ssif_free_stream(&data->rx_cfg, &completed);
		goto error;
	}
	rh850_ssif_wake_io(data, I2S_DIR_RX, false);

	if (data->active_dir == I2S_DIR_BOTH) {
		if (data->duplex_pending == 0U) {
			goto error;
		}
		data->duplex_pending--;
		more_tx = data->stop_with_draining &&
			  (k_msgq_num_used_get(&data->tx_queue) != 0U);
		if ((data->state == I2S_STATE_STOPPING) &&
		    (data->duplex_pending == 0U) && !more_tx) {
			goto stop;
		}
	} else if ((data->active_dir == I2S_DIR_RX) &&
		   (data->state == I2S_STATE_STOPPING)) {
		goto stop;
	}
	if ((data->state != I2S_STATE_RUNNING) && (data->state != I2S_STATE_STOPPING)) {
		return;
	}
	ret = rh850_ssif_rx_start_transfer(dev);
	if (ret < 0) {
		goto error;
	}
	return;

error:
	rh850_ssif_error(dev);
stop:
	(void)rh850_ssif_request_stop(dev);
}

static void rh850_ssif_tx_callback(const struct device *dev)
{
	struct renesas_rh850_ssif_data *data = dev->data;
	bool send_next;

	if (data->trigger_drop || (data->hw_phase != RH850_SSIF_HW_ACTIVE) ||
	    (data->tx_stream.mem_block == NULL)) {
		return;
	}
	/* TX_EMPTY releases source RAM; samples may still be in the FIFO. */
	rh850_ssif_free_stream(&data->tx_cfg, &data->tx_stream);
	if ((data->state != I2S_STATE_RUNNING) && (data->state != I2S_STATE_STOPPING)) {
		return;
	}
	send_next = (data->state == I2S_STATE_RUNNING) || data->stop_with_draining;
	if (send_next && (k_msgq_num_used_get(&data->tx_queue) != 0U)) {
		if (rh850_ssif_tx_start_transfer(dev) < 0) {
			rh850_ssif_error(dev);
			(void)rh850_ssif_request_stop(dev);
		}
		return;
	}
	if (data->state != I2S_STATE_STOPPING) {
		return;
	}
	if (data->active_dir == I2S_DIR_BOTH) {
		return;
	}
	/*
	 * There is no FIFO-drained callback in this HAL. Wait for the final FIFO
	 * samples to leave, then its TUIRQ -> stop -> IDLE path completes this tail.
	 * Stopping at TX_EMPTY would truncate queued FIFO samples at a frame boundary.
	 */
	data->hw_phase = RH850_SSIF_HW_TX_TAIL;
}

static void rh850_ssif_idle_callback(const struct device *dev)
{
	struct renesas_rh850_ssif_data *data = dev->data;
	bool expected;

	if ((data->hw_phase == RH850_SSIF_HW_CONFIGURING) ||
	    (data->hw_phase == RH850_SSIF_HW_IDLE)) {
		return;
	}
	expected = (data->hw_phase == RH850_SSIF_HW_STOP_PENDING) ||
		   ((data->hw_phase == RH850_SSIF_HW_TX_TAIL) &&
		    (data->active_dir == I2S_DIR_TX));
	data->hw_phase = RH850_SSIF_HW_IDLE;
	rh850_ssif_free_stream(&data->tx_cfg, &data->tx_stream);
	rh850_ssif_free_stream(&data->rx_cfg, &data->rx_stream);
	if (expected && (data->trigger_drop || (data->state == I2S_STATE_STOPPING))) {
		data->state = I2S_STATE_READY;
	} else if ((data->state == I2S_STATE_RUNNING) ||
		   (data->state == I2S_STATE_STOPPING)) {
		LOG_ERR("%s: unexpected SSIF idle", dev->name);
		rh850_ssif_error(dev);
	}
	data->trigger_drop = false;
	data->stop_with_draining = false;
	data->duplex_pending = 0U;
}

static void rh850_ssif_callback(i2s_callback_args_t *args)
{
	const struct device *dev;

	if ((args == NULL) || (args->p_context == NULL)) {
		return;
	}

	dev = args->p_context;

	switch (args->event) {
	case I2S_EVENT_IDLE:
		rh850_ssif_idle_callback(dev);
		break;
	case I2S_EVENT_TX_EMPTY:
		rh850_ssif_tx_callback(dev);
		break;
	case I2S_EVENT_RX_FULL:
		rh850_ssif_rx_callback(dev);
		break;
	default:
		LOG_ERR("%s: invalid SSI event: %d", dev->name, args->event);
		break;
	}
}

static int rh850_ssif_start_transfer(const struct device *dev, enum i2s_dir dir,
				     k_spinlock_key_t *key)
{
	struct renesas_rh850_ssif_data *data = dev->data;
	const struct i2s_config *cfg = (dir == I2S_DIR_RX) ? &data->rx_cfg : &data->tx_cfg;
	ssi_clock_div_t clock_div = SSI_CLOCK_DIV_1;
	int ret = 0;

	if ((data->state != I2S_STATE_READY) ||
	    (data->hw_phase != RH850_SSIF_HW_IDLE)) {
		return -EIO;
	}

	switch (dir) {
	case I2S_DIR_BOTH:
		if (!data->full_duplex || !data->tx_configured || !data->rx_configured) {
			return -EIO;
		}
		if (!rh850_ssif_configs_compatible(&data->tx_cfg, &data->rx_cfg)) {
			return -EINVAL;
		}
		for (uint32_t i = 0U; i < k_msgq_num_used_get(&data->tx_queue); i++) {
			struct renesas_rh850_ssif_stream queued;

			if ((k_msgq_peek_at(&data->tx_queue, &queued, i) != 0) ||
			    (queued.mem_block_len != data->rx_cfg.block_size)) {
				return -EINVAL;
			}
		}
		break;
	case I2S_DIR_TX:
		if (!data->tx_configured) {
			return -EIO;
		}
		break;
	case I2S_DIR_RX:
		if (!data->rx_configured) {
			return -EIO;
		}
		break;
	default:
		return -EINVAL;
	}

	data->active_dir = dir;
	data->duplex_pending = 0U;
	data->stop_with_draining = false;
	data->trigger_drop = false;
	data->state = I2S_STATE_RUNNING;
	data->hw_phase = RH850_SSIF_HW_CONFIGURING;

	if ((cfg->options & I2S_OPT_BIT_CLK_TARGET) == 0U) {
		k_spin_unlock(&data->lock, *key);
		ret = rh850_ssif_set_clock_divider(dev, cfg, &clock_div);
		*key = k_spin_lock(&data->lock);
	}
	if (ret == 0) {
		ret = rh850_ssif_apply_config(dev, cfg, clock_div, false);
	}
	data->hw_phase = RH850_SSIF_HW_IDLE;
	if (ret < 0) {
		if (data->state == I2S_STATE_RUNNING) {
			data->state = I2S_STATE_READY;
		}
		return ret;
	}

	switch (dir) {
	case I2S_DIR_BOTH:
		ret = rh850_ssif_tx_rx_start_transfer(dev);
		break;
	case I2S_DIR_TX:
		ret = rh850_ssif_tx_start_transfer(dev);
		break;
	default:
		ret = rh850_ssif_rx_start_transfer(dev);
		break;
	}

	if (ret < 0) {
		if (data->hw_phase != RH850_SSIF_HW_IDLE) {
			rh850_ssif_error(dev);
			(void)rh850_ssif_request_stop(dev);
		} else {
			data->state = I2S_STATE_READY;
		}
	}

	return ret;
}

static int rh850_ssif_stop(const struct device *dev, bool drain)
{
	struct renesas_rh850_ssif_data *data = dev->data;

	if (data->state != I2S_STATE_RUNNING) {
		return -EIO;
	}
	data->stop_with_draining = drain && (data->active_dir != I2S_DIR_RX);
	if (data->active_dir != I2S_DIR_RX) {
		rh850_ssif_wake_io(data, I2S_DIR_TX, true);
	}
	data->trigger_drop = false;
	data->state = I2S_STATE_STOPPING;
	/* Enqueue plus kick is atomic, so queued TX never waits for an idle source. */
	if ((data->active_dir == I2S_DIR_BOTH) && (data->duplex_pending == 0U) &&
	    (!data->stop_with_draining || (k_msgq_num_used_get(&data->tx_queue) == 0U))) {
		return rh850_ssif_request_stop(dev);
	}
	if ((data->active_dir == I2S_DIR_TX) && (data->tx_stream.mem_block == NULL)) {
		data->hw_phase = RH850_SSIF_HW_TX_TAIL;
	}
	return 0;
}

static int rh850_ssif_trigger_drop(const struct device *dev, enum i2s_dir dir)
{
	struct renesas_rh850_ssif_data *data = dev->data;

	/* The trigger dispatcher already validates configured directions. */
	if (data->state == I2S_STATE_NOT_READY) {
		return -EIO;
	}

	if (((data->hw_phase != RH850_SSIF_HW_IDLE) || (data->state == I2S_STATE_ERROR)) &&
	    (dir != data->active_dir) && (dir != I2S_DIR_BOTH)) {
		if (data->active_dir == I2S_DIR_BOTH) {
			return -ENOSYS;
		}
		rh850_ssif_drop_queue(dev, dir);
		return 0;
	}
	data->stop_with_draining = false;

	/* Queued buffers are not owned by hardware and may be discarded now. */
	if ((dir == I2S_DIR_BOTH) || (dir == I2S_DIR_TX)) {
		rh850_ssif_drop_queue(dev, I2S_DIR_TX);
	}
	if ((dir == I2S_DIR_BOTH) || (dir == I2S_DIR_RX)) {
		rh850_ssif_drop_queue(dev, I2S_DIR_RX);
	} else if (data->state == I2S_STATE_ERROR) {
		/* Recovering TX also ends the global ERROR observed by RX readers. */
		rh850_ssif_wake_io(data, I2S_DIR_RX, true);
	}

	if ((data->hw_phase == RH850_SSIF_HW_ACTIVE) ||
	    (data->hw_phase == RH850_SSIF_HW_TX_TAIL) ||
	    (data->hw_phase == RH850_SSIF_HW_STOP_PENDING)) {
		data->trigger_drop = true;
		data->state = I2S_STATE_STOPPING;
		/*
		 * Also submit stop after a software-only STOP/DRAIN transition.
		 * Keep both active buffers until IDLE, even if this request fails.
		 */
		return rh850_ssif_request_stop(dev);
	}

	/* Hardware is already quiescent. */
	if ((dir == I2S_DIR_BOTH) || (dir == I2S_DIR_TX)) {
		rh850_ssif_free_stream(&data->tx_cfg, &data->tx_stream);
	}
	if ((dir == I2S_DIR_BOTH) || (dir == I2S_DIR_RX)) {
		rh850_ssif_free_stream(&data->rx_cfg, &data->rx_stream);
	}
	data->trigger_drop = false;
	data->state = I2S_STATE_READY;

	return 0;
}

static int rh850_ssif_trigger_prepare(const struct device *dev)
{
	struct renesas_rh850_ssif_data *data = dev->data;

	if (data->state != I2S_STATE_ERROR) {
		return -EIO;
	}

	if (data->hw_phase != RH850_SSIF_HW_IDLE) {
		/* A failed/pending stop must not allow PREPARE to free active memory. */
		return -EBUSY;
	}

	if (data->fsp_ctrl->open == 0U) {
		return -EIO;
	}

	if (data->tx_configured) {
		rh850_ssif_drop_queue(dev, I2S_DIR_TX);
		rh850_ssif_free_stream(&data->tx_cfg, &data->tx_stream);
	}

	if (data->rx_configured) {
		rh850_ssif_drop_queue(dev, I2S_DIR_RX);
		rh850_ssif_free_stream(&data->rx_cfg, &data->rx_stream);
	}

	data->duplex_pending = 0U;
	data->stop_with_draining = false;
	data->trigger_drop = false;
	data->state = I2S_STATE_READY;

	return 0;
}

static int i2s_renesas_rh850_ssif_configure(const struct device *dev,
					 enum i2s_dir dir,
					 const struct i2s_config *i2s_cfg)
{
	const struct renesas_rh850_ssif_config *config = dev->config;
	struct renesas_rh850_ssif_data *data = dev->data;
	ssi_clock_div_t clock_div = SSI_CLOCK_DIV_1;
	i2s_mode_t operating_mode = I2S_MODE_SLAVE;
	k_spinlock_key_t key;
	uint32_t frame_size_bytes;
	bool configure_tx = (dir == I2S_DIR_TX) || (dir == I2S_DIR_BOTH);
	bool configure_rx = (dir == I2S_DIR_RX) || (dir == I2S_DIR_BOTH);
	bool was_open;
	fsp_err_t err;
	int ret = 0;

	if ((i2s_cfg == NULL) || (!configure_tx && !configure_rx)) {
		return -EINVAL;
	}

	/* A zero frame clock deconfigures the direction; other fields are ignored. */
	if (i2s_cfg->frame_clk_freq != 0U) {
		if (((i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) !=
		     I2S_FMT_DATA_FORMAT_I2S) ||
		    ((i2s_cfg->format & I2S_FMT_DATA_ORDER_LSB) != 0U) ||
		    ((i2s_cfg->format & I2S_FMT_CLK_FORMAT_MASK) != I2S_FMT_CLK_NF_NB) ||
		    ((i2s_cfg->options & ~I2S_SUPPORTED_OPTIONS) != 0U)) {
			return -ENOTSUP;
		}
		if ((i2s_cfg->channels != 2U) || (i2s_cfg->mem_slab == NULL) ||
		    (i2s_cfg->block_size == 0U)) {
			return -EINVAL;
		}

		frame_size_bytes = rh850_ssif_frame_size_bytes(i2s_cfg->word_size);
		if ((frame_size_bytes == 0U) ||
		    ((i2s_cfg->block_size % frame_size_bytes) != 0U)) {
			return -EINVAL;
		}

		bool bit_clk_target = (i2s_cfg->options & I2S_OPT_BIT_CLK_TARGET) != 0U;
		bool frame_clk_target = (i2s_cfg->options & I2S_OPT_FRAME_CLK_TARGET) != 0U;

		if (bit_clk_target != frame_clk_target) {
			return -ENOTSUP;
		}
		operating_mode = bit_clk_target ? I2S_MODE_SLAVE : I2S_MODE_MASTER;
	}

	key = k_spin_lock(&data->lock);
	if ((data->hw_phase != RH850_SSIF_HW_IDLE) || (data->io_users != 0U)) {
		ret = -EBUSY;
		goto unlock;
	}
	if ((data->state != I2S_STATE_READY) && (data->state != I2S_STATE_NOT_READY)) {
		ret = -EINVAL;
		goto unlock;
	}
	if (!data->full_duplex && (dir == I2S_DIR_BOTH)) {
		ret = -ENOSYS;
		goto unlock;
	}

	/*
	 * Clock APIs run without a spinlock. The configuration phase excludes
	 * trigger/read/write and another configure until the commit is complete.
	 */
	data->hw_phase = RH850_SSIF_HW_CONFIGURING;
	was_open = data->fsp_ctrl->open != 0U;
	k_spin_unlock(&data->lock, key);

	if ((i2s_cfg->frame_clk_freq != 0U) && (operating_mode == I2S_MODE_MASTER)) {
		if (config->audio_clock_dev == NULL) {
			ret = -ENOTSUP;
			goto finish;
		}
		if (!device_is_ready(config->audio_clock_dev)) {
			ret = -ENODEV;
			goto finish;
		}
		ret = clock_control_on(config->audio_clock_dev,
				       config->audio_clock_subsys);
		if (ret < 0) {
			goto finish;
		}
		ret = rh850_ssif_set_clock_divider(dev, i2s_cfg, &clock_div);
		if (ret < 0) {
			goto finish;
		}
	}

	key = k_spin_lock(&data->lock);
	if (i2s_cfg->frame_clk_freq == 0U) {
		bool any_configured = (data->tx_configured && !configure_tx) ||
				      (data->rx_configured && !configure_rx);

		if (!any_configured && was_open) {
			err = config->fsp_api->close(data->fsp_ctrl);
			if (err != FSP_SUCCESS) {
				LOG_ERR("%s: R_SSI_Close error: %d", dev->name, err);
				ret = -EIO;
				goto finish_locked;
			}
		}
		if (configure_tx) {
			rh850_ssif_drop_queue(dev, I2S_DIR_TX);
			rh850_ssif_free_stream(&data->tx_cfg, &data->tx_stream);
			data->tx_configured = false;
		}
		if (configure_rx) {
			rh850_ssif_drop_queue(dev, I2S_DIR_RX);
			rh850_ssif_free_stream(&data->rx_cfg, &data->rx_stream);
			data->rx_configured = false;
		}
		data->state = any_configured ? I2S_STATE_READY : I2S_STATE_NOT_READY;
		goto finish_locked;
	}

	ret = rh850_ssif_apply_config(dev, i2s_cfg, clock_div, true);
	if (ret < 0) {
		goto finish_locked;
	}

	if (configure_tx || (!data->full_duplex && configure_rx)) {
		rh850_ssif_drop_queue(dev, I2S_DIR_TX);
		data->tx_configured = configure_tx;
		if (configure_tx) {
			data->tx_cfg = *i2s_cfg;
		}
	}
	if (configure_rx || (!data->full_duplex && configure_tx)) {
		rh850_ssif_drop_queue(dev, I2S_DIR_RX);
		data->rx_configured = configure_rx;
		if (configure_rx) {
			data->rx_cfg = *i2s_cfg;
		}
	}
	data->state = I2S_STATE_READY;

finish_locked:
	k_spin_unlock(&data->lock, key);
finish:
	key = k_spin_lock(&data->lock);
	data->hw_phase = RH850_SSIF_HW_IDLE;
unlock:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static const struct i2s_config *
i2s_renesas_rh850_ssif_get_config(const struct device *dev, enum i2s_dir dir)
{
	struct renesas_rh850_ssif_data *data = dev->data;
	const struct i2s_config *cfg = NULL;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if ((data->hw_phase != RH850_SSIF_HW_CONFIGURING) ||
	    (data->state == I2S_STATE_RUNNING)) {
		if ((dir == I2S_DIR_TX) && data->tx_configured) {
			cfg = &data->tx_cfg;
		} else if ((dir == I2S_DIR_RX) && data->rx_configured) {
			cfg = &data->rx_cfg;
		} else if ((dir == I2S_DIR_BOTH) && data->tx_configured &&
			   data->rx_configured &&
			   rh850_ssif_configs_compatible(&data->tx_cfg, &data->rx_cfg)) {
			cfg = &data->tx_cfg;
		}
	}

	k_spin_unlock(&data->lock, key);
	return cfg;
}

/* Caller owns data->lock; this function returns with the same lock held. */
static int rh850_ssif_queue_io(const struct device *dev, enum i2s_dir dir,
			       struct renesas_rh850_ssif_stream *stream,
			       k_spinlock_key_t *key)
{
	struct renesas_rh850_ssif_data *data = dev->data;
	bool tx = (dir == I2S_DIR_TX);
	size_t index = tx ? SSIF_TX : SSIF_RX;
	struct k_msgq *queue = tx ? &data->tx_queue : &data->rx_queue;
	int32_t timeout = tx ? data->tx_cfg.timeout : data->rx_cfg.timeout;
	k_timepoint_t deadline = sys_timepoint_calc(rh850_ssif_timeout(timeout));
	uint32_t epoch = data->io_epoch[index];
	struct rh850_ssif_waiter waiter;
	bool initialized = false;
	k_timeout_t wait;
	int ret;

	data->io_users++;
	for (;;) {
		if (epoch != data->io_epoch[index]) {
			ret = tx ? -EIO : -EAGAIN;
			break;
		}
		if (tx && (data->state != I2S_STATE_READY) &&
		    (data->state != I2S_STATE_RUNNING)) {
			ret = -EIO;
			break;
		}
		if (tx && (data->state == I2S_STATE_RUNNING) &&
		    (data->active_dir == I2S_DIR_BOTH) &&
		    (stream->mem_block_len != data->tx_cfg.block_size)) {
			ret = -EINVAL;
			break;
		}
		ret = tx ? k_msgq_put(queue, stream, K_NO_WAIT) :
			   k_msgq_get(queue, stream, K_NO_WAIT);
		if (ret == 0) {
			break;
		}
		if (data->state == I2S_STATE_ERROR) {
			ret = -EIO;
			break;
		}
		wait = sys_timepoint_timeout(deadline);
		if (K_TIMEOUT_EQ(wait, K_NO_WAIT)) {
			ret = (timeout <= 0 && timeout != SYS_FOREVER_MS) ? -EBUSY : -EAGAIN;
			break;
		}
		if (!initialized) {
			k_sem_init(&waiter.ready, 0U, 1U);
			initialized = true;
		}
		sys_slist_append(&data->io_waiters[index], &waiter.node);
		k_spin_unlock(&data->lock, *key);
		(void)k_sem_take(&waiter.ready, wait);
		*key = k_spin_lock(&data->lock);
		(void)sys_slist_find_and_remove(&data->io_waiters[index], &waiter.node);
	}
#ifdef CONFIG_OBJ_CORE_SEM
	if (initialized) {
		k_obj_core_unlink(K_OBJ_CORE(&waiter.ready));
	}
#endif
	data->io_users--;
	return ret;
}

static int i2s_renesas_rh850_ssif_write(const struct device *dev,
				      void *mem_block, size_t size)
{
	struct renesas_rh850_ssif_data *data = dev->data;
	struct renesas_rh850_ssif_stream stream = {
		.mem_block = mem_block,
		.mem_block_len = size,
	};
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	size_t frame_size;
	int ret;

	if (((data->hw_phase == RH850_SSIF_HW_CONFIGURING) &&
	     (data->state != I2S_STATE_RUNNING)) || !data->tx_configured ||
	    ((data->state != I2S_STATE_READY) && (data->state != I2S_STATE_RUNNING))) {
		ret = -EIO;
		goto unlock;
	}
	if ((mem_block == NULL) || (size == 0U) || (size > data->tx_cfg.block_size)) {
		ret = -EINVAL;
		goto unlock;
	}
	frame_size = rh850_ssif_frame_size_bytes(data->tx_cfg.word_size);
	if ((frame_size == 0U) || ((size % frame_size) != 0U)) {
		ret = -EINVAL;
		goto unlock;
	}
	ret = rh850_ssif_queue_io(dev, I2S_DIR_TX, &stream, &key);
	if (ret < 0) {
		goto unlock;
	}
	/* Ownership transferred at enqueue; a later HAL error is asynchronous. */
	if ((data->state == I2S_STATE_RUNNING) &&
	    (data->hw_phase == RH850_SSIF_HW_ACTIVE) &&
	    ((data->active_dir == I2S_DIR_TX) || (data->active_dir == I2S_DIR_BOTH)) &&
	    (data->tx_stream.mem_block == NULL)) {
		if (rh850_ssif_tx_start_transfer(dev) < 0) {
			rh850_ssif_error(dev);
			(void)rh850_ssif_request_stop(dev);
		}
	}
unlock:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int i2s_renesas_rh850_ssif_read(const struct device *dev,
				     void **mem_block, size_t *size)
{
	struct renesas_rh850_ssif_data *data = dev->data;
	struct renesas_rh850_ssif_stream stream;
	k_spinlock_key_t key;
	int ret;

	if ((mem_block == NULL) || (size == NULL)) {
		return -EINVAL;
	}
	key = k_spin_lock(&data->lock);
	if (((data->hw_phase == RH850_SSIF_HW_CONFIGURING) &&
	     (data->state != I2S_STATE_RUNNING)) || !data->rx_configured) {
		ret = -EIO;
		goto unlock;
	}
	ret = rh850_ssif_queue_io(dev, I2S_DIR_RX, &stream, &key);
	if (ret == 0) {
		*mem_block = stream.mem_block;
		*size = stream.mem_block_len;
	}
unlock:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static int i2s_renesas_rh850_ssif_trigger(const struct device *dev,
					enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	struct renesas_rh850_ssif_data *data = dev->data;
	k_spinlock_key_t key;
	int ret;

	if ((dir != I2S_DIR_TX) && (dir != I2S_DIR_RX) && (dir != I2S_DIR_BOTH)) {
		return -EINVAL;
	}
	key = k_spin_lock(&data->lock);
	if (data->hw_phase == RH850_SSIF_HW_CONFIGURING) {
		ret = -EBUSY;
		goto unlock;
	}
	if (((dir == I2S_DIR_TX) && !data->tx_configured) ||
	    ((dir == I2S_DIR_RX) && !data->rx_configured) ||
	    ((dir == I2S_DIR_BOTH) && (!data->tx_configured || !data->rx_configured))) {
		ret = -EIO;
		goto unlock;
	}
	if ((data->state == I2S_STATE_RUNNING) &&
	    ((cmd == I2S_TRIGGER_STOP) || (cmd == I2S_TRIGGER_DRAIN)) &&
	    (dir != data->active_dir)) {
		ret = -EINVAL;
		goto unlock;
	}
	if ((data->state == I2S_STATE_ERROR) && (cmd == I2S_TRIGGER_PREPARE) &&
	    (data->active_dir != I2S_DIR_BOTH) &&
	    (dir != data->active_dir) && (dir != I2S_DIR_BOTH)) {
		ret = -EINVAL;
		goto unlock;
	}
	switch (cmd) {
	case I2S_TRIGGER_START:
		ret = rh850_ssif_start_transfer(dev, dir, &key);
		break;
	case I2S_TRIGGER_STOP:
		ret = rh850_ssif_stop(dev, false);
		break;
	case I2S_TRIGGER_DRAIN:
		ret = rh850_ssif_stop(dev, true);
		break;
	case I2S_TRIGGER_DROP:
		ret = rh850_ssif_trigger_drop(dev, dir);
		break;
	case I2S_TRIGGER_PREPARE:
		ret = rh850_ssif_trigger_prepare(dev);
		break;
	default:
		ret = -EINVAL;
		break;
	}
unlock:
	k_spin_unlock(&data->lock, key);
	return ret;
}

static DEVICE_API(i2s, i2s_renesas_rh850_ssif_api) = {
	.configure = i2s_renesas_rh850_ssif_configure,
	.config_get = i2s_renesas_rh850_ssif_get_config,
	.read = i2s_renesas_rh850_ssif_read,
	.write = i2s_renesas_rh850_ssif_write,
	.trigger = i2s_renesas_rh850_ssif_trigger,
};

static int i2s_renesas_rh850_ssif_init(const struct device *dev)
{
	const struct renesas_rh850_ssif_config *config = dev->config;
	struct renesas_rh850_ssif_data *data = dev->data;
	int ret;

	if (!device_is_ready(config->pclk_dev)) {
		LOG_ERR("%s: PCLK device is not ready", dev->name);
		return -ENODEV;
	}

	ret = clock_control_on(config->pclk_dev, config->pclk_subsys);
	if (ret < 0) {
		LOG_ERR("%s: failed to enable PCLK: %d", dev->name, ret);
		return ret;
	}

	ret = pinctrl_apply_state(config->pinctrl_dev, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		int clock_ret = clock_control_off(config->pclk_dev, config->pclk_subsys);

		LOG_ERR("%s: pinctrl_apply_state failed: %d", dev->name, ret);
		if (clock_ret < 0) {
			LOG_WRN("%s: failed to disable PCLK: %d", dev->name, clock_ret);
		}
		return ret;
	}

	k_msgq_init(&data->tx_queue, (char *)data->tx_msgs,
		    sizeof(struct renesas_rh850_ssif_stream),
		    CONFIG_I2S_RENESAS_RH850_SSIF_TX_BLOCK_COUNT);

	k_msgq_init(&data->rx_queue, (char *)data->rx_msgs,
		    sizeof(struct renesas_rh850_ssif_stream),
		    CONFIG_I2S_RENESAS_RH850_SSIF_RX_BLOCK_COUNT);

	for (size_t i = 0U; i < SSIF_STREAMS; i++) {
		sys_slist_init(&data->io_waiters[i]);
		data->io_epoch[i] = 0U;
	}
	data->state = I2S_STATE_NOT_READY;
	data->active_dir = I2S_DIR_TX;
	data->tx_configured = false;
	data->rx_configured = false;
	data->stop_with_draining = false;
	data->trigger_drop = false;
	data->duplex_pending = 0U;
	data->hw_phase = RH850_SSIF_HW_IDLE;
	data->io_users = 0U;

	rh850_ssif_release_stream(&data->tx_stream);
	rh850_ssif_release_stream(&data->rx_stream);

	return 0;
}

#define RH850_SSIF_IRQ_INIT(n)                                                     \
	do {                                                                        \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, ssi_rxi, irq),                       \
			    DT_INST_IRQ_BY_NAME(n, ssi_rxi, priority),                  \
			    rh850_ssif_rxi_isr, DEVICE_DT_INST_GET(n), 0);          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, ssi_txi, irq),                       \
			    DT_INST_IRQ_BY_NAME(n, ssi_txi, priority),                  \
			    rh850_ssif_txi_isr, DEVICE_DT_INST_GET(n), 0);          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, ssi_inti, irq),                      \
			    DT_INST_IRQ_BY_NAME(n, ssi_inti, priority),                 \
			    rh850_ssif_int_isr, DEVICE_DT_INST_GET(n), 0);          \
		irq_enable(DT_INST_IRQ_BY_NAME(n, ssi_rxi, irq));                       \
		irq_enable(DT_INST_IRQ_BY_NAME(n, ssi_txi, irq));                       \
		irq_enable(DT_INST_IRQ_BY_NAME(n, ssi_inti, irq));                      \
	} while (0)

#define RH850_SSIF_AUDIO_CLOCK_DEV(n)                                      \
	COND_CODE_1(DT_INST_CLOCKS_HAS_NAME(n, audio_clock),                 \
		(DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, audio_clock))), \
		(NULL))

#define RH850_SSIF_AUDIO_CLOCK_SUBSYS(n)                                   \
	COND_CODE_1(DT_INST_CLOCKS_HAS_NAME(n, audio_clock),                 \
		((clock_control_subsys_t)(uintptr_t)                         \
		DT_INST_CLOCKS_CELL_BY_NAME(n, audio_clock, id)),           \
		(NULL))

#define I2S_RH850_SSIF_INIT(n)                                                     \
	PINCTRL_DT_INST_DEFINE(n);                                                 \
                                                                                \
	static ssi_instance_ctrl_t g_i2s##n##_ctrl;                               \
                                                                                \
	static ssi_extended_cfg_t g_i2s##n##_ext_cfg = {                          \
		.bit_clock_div = SSI_CLOCK_DIV_1,                                  \
		.communication_format = SSI_COMMUNICATION_FORMAT_I2S,              \
		.frame_word_number = SSI_FRAME_WORD_NUMBER_4,                            \
		.frametype_monauralformat =                                        \
			SSI_FRAMETYPE_MONAURAL_FORMAT_SHORT_FRAME,                           \
		/* 0 means IRQ at >=1 TX free stage / >=1 RX occupied stage. */    \
		.transmit_data_empty = 0U,                                         \
		.receive_data_full = 0U,                                           \
	};                                                                         \
                                                                                \
	static i2s_cfg_t g_i2s##n##_cfg = {                                       \
		.channel = DT_INST_PROP(n, channel),                                \
		.pcm_width = I2S_PCM_WIDTH_16_BITS,                                \
		.word_length = I2S_WORD_LENGTH_16_BITS,                            \
		.ws_continue = I2S_WS_CONTINUE_ON,                                  \
		.operating_mode = I2S_MODE_MASTER,                                 \
		.p_transfer_tx = NULL,                                             \
		.p_transfer_rx = NULL,                                             \
		.p_callback = rh850_ssif_callback,                                 \
		.p_context = (void *)DEVICE_DT_INST_GET(n),                        \
		.p_extend = &g_i2s##n##_ext_cfg,                                   \
		.txi_ipl = DT_INST_IRQ_BY_NAME(n, ssi_txi, priority),                  \
		.rxi_ipl = DT_INST_IRQ_BY_NAME(n, ssi_rxi, priority),                  \
		.idle_err_ipl = DT_INST_IRQ_BY_NAME(n, ssi_inti, priority),            \
		.txi_irq = DT_INST_IRQ_BY_NAME(n, ssi_txi, irq),                       \
		.rxi_irq = DT_INST_IRQ_BY_NAME(n, ssi_rxi, irq),                       \
		.int_irq = DT_INST_IRQ_BY_NAME(n, ssi_inti, irq),                      \
	};                                                                         \
                                                                                \
	static const struct renesas_rh850_ssif_config rh850_ssif_config_##n = {    \
		.pinctrl_dev = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                   \
		.fsp_api = &g_i2s_on_ssi,                                           \
		.pclk_dev = DEVICE_DT_GET(                                          \
			DT_INST_CLOCKS_CTLR_BY_NAME(n, pclk)),                        \
		.pclk_subsys = (clock_control_subsys_t)(uintptr_t)                  \
			DT_INST_CLOCKS_CELL_BY_NAME(n, pclk, clk_id),                 \
		.audio_clock_dev = RH850_SSIF_AUDIO_CLOCK_DEV(n),                    \
		.audio_clock_subsys = RH850_SSIF_AUDIO_CLOCK_SUBSYS(n),             \
	};                                                                         \
                                                                                \
	static struct renesas_rh850_ssif_data rh850_ssif_data_##n = {              \
		.fsp_ctrl = &g_i2s##n##_ctrl,                                      \
		.fsp_config = &g_i2s##n##_cfg,                                     \
		.fsp_ext_config = &g_i2s##n##_ext_cfg,                             \
		.full_duplex = DT_INST_PROP(n, full_duplex),                       \
	};                                                                         \
                                                                                \
	static int i2s_renesas_rh850_ssif_init_##n(const struct device *dev)       \
	{                                                                          \
		int err = i2s_renesas_rh850_ssif_init(dev);                        \
		if (err != 0) {                                                     \
			return err;                                                  \
		}                                                                  \
		RH850_SSIF_IRQ_INIT(n);                                             \
		return 0;                                                          \
	}                                                                          \
                                                                                \
	DEVICE_DT_INST_DEFINE(n, &i2s_renesas_rh850_ssif_init_##n, NULL,           \
			      &rh850_ssif_data_##n, &rh850_ssif_config_##n,        \
			      POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,                 \
			      &i2s_renesas_rh850_ssif_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_RH850_SSIF_INIT)
