/*
 * Copyright 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/console/console.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(i2s_record, LOG_LEVEL_INF);

#define BYTES_PER_SAMPLE  (CONFIG_SAMPLE_BIT_WIDTH / 8)
#define FRAMES_PER_BLOCK  (CONFIG_SAMPLE_FREQUENCY / 50) /* 20ms per block */
#define SAMPLES_PER_BLOCK (FRAMES_PER_BLOCK * CONFIG_NUMBER_OF_CHANNELS)
#define BLOCK_SIZE        (SAMPLES_PER_BLOCK * BYTES_PER_SAMPLE)
#define BLOCK_COUNT       ((CONFIG_SAMPLE_FREQUENCY * CONFIG_RECORD_TIME) / FRAMES_PER_BLOCK)
#define TOTAL_BYTES       (BLOCK_SIZE * BLOCK_COUNT)

/*
 * Number of DMA/slab blocks used by I2S driver.
 *
 * This is not the final recording buffer.
 * This is only temporary buffering for the I2S driver.
 */
#define RX_SLAB_BLOCK_COUNT 4

/*
 * Alignment may need to be larger on some DMA-capable SoCs.
 * If your I2S driver requires cache-line alignment, try 32 or 64.
 */
#define RX_BLOCK_ALIGN 32

K_MEM_SLAB_DEFINE_STATIC(rx_mem_slab, BLOCK_SIZE, RX_SLAB_BLOCK_COUNT, RX_BLOCK_ALIGN);

/*
 * Full recording buffer.
 *
 * Make sure your MCU/board has enough RAM.
 */
static uint8_t recorded_data[TOTAL_BYTES];

static uint8_t tx_block[BLOCK_SIZE];

/*
 * Devicetree requirement:
 *
 * In your board overlay, add an alias like:
 *
 * / {
 *     aliases {
 *         i2s-codec = &i2s0;
 *     };
 * };
 *
 * Then DT_ALIAS(i2s-codec) will point to that I2S device.
 */
#define I2S_CODEC_NODE DT_ALIAS(i2s_codec)
#define USER_NODE      DT_PATH(zephyr_user)

#if !DT_NODE_EXISTS(I2S_CODEC_NODE)
#error "Please define alias i2s-codec in overlay, for example: aliases { i2s-codec = &i2s0; };"
#endif

static const struct device *i2s_codec_dev = DEVICE_DT_GET(I2S_CODEC_NODE);
static const struct device *const codec_dev = DEVICE_DT_GET(DT_NODELABEL(audio_codec));
static bool had_record_data;

static int configure_i2s(struct i2s_config *i2s_cfg, struct audio_codec_cfg *audio_cfg)
{
	int ret;

	if (!device_is_ready(i2s_codec_dev)) {
		LOG_ERR("I2S device is not ready");
		return -ENODEV;
	}

	if (!device_is_ready(codec_dev)) {
		LOG_ERR("%s is not ready", codec_dev->name);
		return -ENODEV;
	}

	audio_cfg->dai_route = AUDIO_ROUTE_CAPTURE;
	audio_cfg->dai_type = AUDIO_DAI_TYPE_I2S;
	audio_cfg->dai_cfg.i2s.word_size = CONFIG_SAMPLE_BIT_WIDTH;
	audio_cfg->dai_cfg.i2s.channels = CONFIG_NUMBER_OF_CHANNELS;
	audio_cfg->dai_cfg.i2s.format = I2S_FMT_DATA_FORMAT_I2S;
	audio_cfg->dai_cfg.i2s.options = I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_SLAVE;
	audio_cfg->dai_cfg.i2s.frame_clk_freq = CONFIG_SAMPLE_FREQUENCY;
	audio_cfg->dai_cfg.i2s.mem_slab = &rx_mem_slab;
	audio_cfg->dai_cfg.i2s.block_size = BLOCK_SIZE;

	ret = audio_codec_configure(codec_dev, audio_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure codec device: %d", ret);
		return ret;
	}

	i2s_cfg->word_size = CONFIG_SAMPLE_BIT_WIDTH;
	i2s_cfg->channels = CONFIG_NUMBER_OF_CHANNELS;
	i2s_cfg->format = I2S_FMT_DATA_FORMAT_I2S;

	/*
	 * Clock options:
	 *
	 * Use MASTER mode if the MCU generates BCLK/LRCLK.
	 * If your external codec generates clocks, change this to:
	 *
	 *   I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_SLAVE
	 *
	 * Many i2s_codec sample boards use master clock from MCU,
	 * but this depends on your hardware.
	 */
	i2s_cfg->options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg->frame_clk_freq = CONFIG_SAMPLE_FREQUENCY;
	i2s_cfg->mem_slab = &rx_mem_slab;
	i2s_cfg->block_size = BLOCK_SIZE;
	i2s_cfg->timeout = 2000;

	ret = i2s_configure(i2s_codec_dev, I2S_DIR_RX, i2s_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure I2S device: %d", ret);
		return ret;
	}

	printk("I2S configured\n");
	printk("Sampling rate : %d Hz\n", CONFIG_SAMPLE_FREQUENCY);
	printk("Bit width   : %d bits\n", CONFIG_SAMPLE_BIT_WIDTH);
	printk("Channels    : %d\n", CONFIG_NUMBER_OF_CHANNELS);
	printk("Block size  : %d bytes\n", BLOCK_SIZE);
	printk("Block count : %d\n", BLOCK_COUNT);
	printk("Total bytes : %d\n", TOTAL_BYTES);
	printk("DMA mode : %d\n", IS_ENABLED(CONFIG_DMA));

	return 0;
}

static int change_audio_direction_tx(struct i2s_config *i2s_cfg, struct audio_codec_cfg *audio_cfg,
				     bool tx)
{
	int ret;
	const struct gpio_dt_spec audio_codec_gpios =
		GPIO_DT_SPEC_GET_OR(USER_NODE, audio_codec_gpios, {});

	enum i2s_dir dir;
	gpio_flags_t gpio_dir;

	if (tx) {
		dir = I2S_DIR_TX;
		audio_cfg->dai_route = AUDIO_ROUTE_PLAYBACK;
		gpio_dir = GPIO_OUTPUT_LOW;
	} else {
		dir = I2S_DIR_RX;
		audio_cfg->dai_route = AUDIO_ROUTE_CAPTURE;
		gpio_dir = GPIO_OUTPUT_HIGH;
	}

	ret = gpio_pin_configure_dt(&audio_codec_gpios, gpio_dir);
	if (ret < 0) {
		LOG_ERR("Failed to configure board I2S direction");
		return ret;
	}

	ret = audio_codec_configure(codec_dev, audio_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure audio codec: %d", ret);
		return ret;
	}

	ret = i2s_configure(i2s_codec_dev, dir, i2s_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure I2S: %d", ret);
		return ret;
	}

	return 0;
}

static void fill_block(uint8_t *dst, size_t *offset)
{
	size_t off = *offset;
	size_t copied = 0;

	while (copied < BLOCK_SIZE) {
		size_t chunk = MIN((size_t)BLOCK_SIZE - copied, TOTAL_BYTES - off);

		memcpy(dst + copied, recorded_data + off, chunk);
		copied += chunk;
		off += chunk;
		if (off >= TOTAL_BYTES) {
			off = 0;
		}
	}

	*offset = off;
}

static int write_next_block(const struct device *i2s_dev_codec, size_t *offset)
{
	int ret;

	fill_block(tx_block, offset);

	ret = i2s_buf_write(i2s_dev_codec, tx_block, BLOCK_SIZE);
	if (ret < 0) {
		LOG_ERR("Failed to write I2S data: %d\n", ret);
		return ret;
	}

	return 0;
}

static int start_record(struct i2s_config *i2s_cfg, struct audio_codec_cfg *audio_cfg)
{
	int ret = 0;
	int loop_ret = 0;
	void *mem_block = NULL;
	size_t block_size = 0;
	size_t offset = 0;

	memset(recorded_data, 0U, sizeof(recorded_data));

	printk("Audio config changed to Capture mode\n");

	ret = change_audio_direction_tx(i2s_cfg, audio_cfg, false);
	if (ret < 0) {
		LOG_ERR("Change I2S direction failed: %d", ret);
		return ret;
	}

	printk("Recording %d seconds\n", CONFIG_RECORD_TIME);

	/*
	 * Start I2S RX.
	 */
	ret = i2s_trigger(i2s_codec_dev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("Failed to start I2S RX: %d", ret);
		return ret;
	}

	printk("Recording started\n");

	for (int i = 0; i < BLOCK_COUNT; i++) {
		mem_block = NULL;
		block_size = 0;

		ret = i2s_read(i2s_codec_dev, &mem_block, &block_size);
		if (ret < 0) {
			LOG_ERR("i2s_read failed at block %d: %d", i, ret);
			loop_ret = ret;
			break;
		}

		if (mem_block == NULL) {
			LOG_ERR("i2s_read returned NULL block");
			loop_ret = ret;
			break;
		}

		if (block_size != BLOCK_SIZE) {
			LOG_WRN("Unexpected block size: got %u, expected %u",
				(unsigned int)block_size, (unsigned int)BLOCK_SIZE);
		}

		if ((offset + block_size) <= TOTAL_BYTES) {
			memcpy(&recorded_data[offset], mem_block, block_size);
			offset += block_size;
		} else {
			LOG_ERR("Recording buffer overflow");
			k_mem_slab_free(&rx_mem_slab, mem_block);
			loop_ret = -EIO;
			break;
		}

		/*
		 * Return DMA block back to I2S memory slab.
		 */
		k_mem_slab_free(&rx_mem_slab, mem_block);
	}

	/*
	 * Stop/drop RX stream.
	 */
	ret = i2s_trigger(i2s_codec_dev, I2S_DIR_RX, I2S_TRIGGER_DROP);
	if (ret < 0) {
		LOG_ERR("Failed to stop I2S RX: %d", ret);
		return ret;
	}

	if (loop_ret < 0) {
		LOG_ERR("Recording failed: %d", loop_ret);
		return loop_ret;
	}

	printk("Recording finished\n");
	printk("Captured %u bytes\n", (unsigned int)offset);
	had_record_data = true;

	return 0;
}

static int start_playback(struct i2s_config *i2s_cfg, struct audio_codec_cfg *audio_cfg)
{
	if (!had_record_data) {
		printk("No recorded data, please start record first\n");
		return 0;
	}

	int ret = 0;
	size_t playback_offset = 0;

	memset(tx_block, 0U, sizeof(tx_block));

	printk("Audio config changed to Playback mode\n");

	ret = change_audio_direction_tx(i2s_cfg, audio_cfg, true);
	if (ret < 0) {
		LOG_ERR("Change I2S direction failed: %d", ret);
		return ret;
	}

	printk("Pre-buffering %d blocks before starting stream...\n", CONFIG_I2S_INITIAL_BUFFERS);

	/* Pre-fill several blocks so the driver has data queued before
	 * we trigger START - this avoids an immediate underrun.
	 */
	for (int i = 0; i < CONFIG_I2S_INITIAL_BUFFERS; i++) {
		ret = write_next_block(i2s_codec_dev, &playback_offset);
		if (ret < 0) {
			LOG_INF("Stop transfer and recover to ready state");
			i2s_trigger(i2s_codec_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
			return ret;
		}
	}

	printk("Starting I2S TX stream\n");
	ret = i2s_trigger(i2s_codec_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("Failed to start I2S TX: %d", ret);
		return ret;
	}

	/* Continuously feed blocks */
	while (1) {
		ret = write_next_block(i2s_codec_dev, &playback_offset);
		if (ret < 0) {
			LOG_INF("Stop transfer and recover to ready state");
			i2s_trigger(i2s_codec_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
			return ret;
		}

		if (playback_offset == 0) {
			break;
		}
	}

	ret = i2s_trigger(i2s_codec_dev, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	if (ret < 0) {
		LOG_ERR("Failed to drain I2S TX: %d", ret);
		return ret;
	}

	printk("Stream stopped\n");

	return 0;
}

int main(void)
{
	int ret;
	struct i2s_config i2s_cfg = {0};
	struct audio_codec_cfg audio_cfg = {0};

	had_record_data = false;

	console_init();

	printk("Starting I2S record-playback sample\n");

	ret = configure_i2s(&i2s_cfg, &audio_cfg);
	if (ret < 0) {
		LOG_ERR("I2S configuration failed: %d", ret);
		return 0;
	}

	while (1) {
		/* Wait for input from console to start recording */
		printk("Type 'r' to start recording, 'p' to start playback, 'e' to exit, otherwise "
		       "nothing\n");
		while (1) {
			uint8_t c = console_getchar();

			console_putchar(c);
			if (c == '\r') {
				console_putchar('\n');
			} else if (c == 'e') {
				console_putchar('\n');
				printk("Exit sample\n");
				return 0;
			} else if (c == 'r') {
				console_putchar('\n');
				ret = start_record(&i2s_cfg, &audio_cfg);
				if (ret < 0) {
					return ret;
				}
				break;
			} else if (c == 'p') {
				console_putchar('\n');
				ret = start_playback(&i2s_cfg, &audio_cfg);
				if (ret < 0) {
					return ret;
				}
				break;
			}
		}
	}

	return 0;
}
