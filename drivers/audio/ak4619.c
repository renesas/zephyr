/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT asahi_kasei_ak4619

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/audio/codec.h>
#include <zephyr/logging/log.h>

#include "ak4619.h"

LOG_MODULE_REGISTER(ak4619, CONFIG_AUDIO_CODEC_LOG_LEVEL);

struct ak4619_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec pdn_gpio;
	uint8_t adc1_l_input_mode;
	uint8_t adc1_r_input_mode;
	uint8_t adc2_l_input_mode;
	uint8_t adc2_r_input_mode;
	uint8_t dac1_input;
	uint8_t dac2_input;
	uint8_t mic1_gain_l;
	uint8_t mic1_gain_r;
	uint8_t mic2_gain_l;
	uint8_t mic2_gain_r;
	uint8_t adc1_filter_mode;
	uint8_t adc2_filter_mode;
	uint8_t dac1_filter_mode;
	uint8_t dac2_filter_mode;
	bool adc1_disable;
	bool adc2_disable;
	bool dac1_disable;
	bool dac2_disable;
	bool dac1_deemphasis;
	bool dac2_deemphasis;
	const struct device *mclk_dev;
	uint32_t mclk_subsys;
};

struct ak4619_data {
	uint8_t dac1_vol_l;
	uint8_t dac1_vol_r;
	uint8_t dac2_vol_l;
	uint8_t dac2_vol_r;
	uint8_t adc1_vol_l;
	uint8_t adc1_vol_r;
	uint8_t adc2_vol_l;
	uint8_t adc2_vol_r;
};

static int ak4619_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct ak4619_config *cfg = dev->config;
	uint8_t buf[2] = {reg, val};
	int ret;

	ret = i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to write reg 0x%02X = 0x%02X: %d", reg, val, ret);
	}
	return ret;
}

static int ak4619_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct ak4619_config *cfg = dev->config;
	int ret;

	ret = i2c_write_read_dt(&cfg->i2c, &reg, 1, val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read reg 0x%02X: %d", reg, ret);
	}
	return ret;
}

static int ak4619_update_reg(const struct device *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
	uint8_t old_val;
	int ret;

	ret = ak4619_read_reg(dev, reg, &old_val);
	if (ret < 0) {
		return ret;
	}

	return ak4619_write_reg(dev, reg, (old_val & ~mask) | (val & mask));
}

static int ak4619_get_if_regs(audio_dai_type_t dai_type, uint8_t didl_bits, uint8_t dodl_bits,
			      uint8_t dsl_bits, struct ak4619_if_regs *regs)
{
	switch (dai_type) {
	case AUDIO_DAI_TYPE_I2S:
		regs->if1 = AK4619_TDM_STEREO | AK4619_DCF_I2S | dsl_bits | AK4619_BCKP_FALLING;
		regs->if2 = AK4619_SLOT_LRCK_EDGE | didl_bits | dodl_bits;
		break;

	case AUDIO_DAI_TYPE_LEFT_JUSTIFIED:
		regs->if1 = AK4619_TDM_STEREO | AK4619_DCF_MSB | dsl_bits | AK4619_BCKP_FALLING;
		regs->if2 = AK4619_SLOT_LRCK_EDGE | didl_bits | dodl_bits;
		break;

	case AUDIO_DAI_TYPE_PCMA:
		if (didl_bits == AK4619_DIDL_20BIT || dodl_bits == AK4619_DODL_20BIT) {
			LOG_ERR("20-bit word size not supported for PCM Short Frame");
			return -EINVAL;
		}
		regs->if1 =
			AK4619_TDM_STEREO | AK4619_DCF_PCM_SHORT | dsl_bits | AK4619_BCKP_RISING;
		regs->if2 = AK4619_SLOT_LENGTH | didl_bits | dodl_bits;
		break;

	case AUDIO_DAI_TYPE_PCMB:
		if (didl_bits == AK4619_DIDL_20BIT || dodl_bits == AK4619_DODL_20BIT) {
			LOG_ERR("20-bit word size not supported for PCM Long Frame");
			return -EINVAL;
		}
		regs->if1 = AK4619_TDM_STEREO | AK4619_DCF_PCM_LONG | dsl_bits | AK4619_BCKP_RISING;
		regs->if2 = AK4619_SLOT_LENGTH | didl_bits | dodl_bits;
		break;

	default:
		LOG_ERR("Unsupported audio dai type: %d", dai_type);
		return -EINVAL;
	}

	return 0;
}

static int ak4619_get_fs_bits(uint32_t frame_clk_freq, uint32_t mclk_freq, uint8_t *fs_bits)
{
	uint32_t ratio;

	if (frame_clk_freq == 0) {
		return -EINVAL;
	}

	ratio = mclk_freq / frame_clk_freq;

	if (frame_clk_freq <= AUDIO_PCM_RATE_48K) {
		switch (ratio) {
		case AK4619_MCLK_RATIO_256:
			*fs_bits = AK4619_FS_256_48K;
			break;
		case AK4619_MCLK_RATIO_384:
			*fs_bits = AK4619_FS_384_48K;
			break;
		case AK4619_MCLK_RATIO_512:
			*fs_bits = AK4619_FS_512_48K;
			break;
		default:
			LOG_ERR("Unsupported MCLK ratio %d for fs=%d Hz", ratio, frame_clk_freq);
			return -EINVAL;
		}
	} else if (frame_clk_freq == AUDIO_PCM_RATE_96K) {
		if (ratio != AK4619_MCLK_RATIO_256) {
			LOG_ERR("fs=96kHz requires MCLK=256fs, got ratio=%d", ratio);
			return -EINVAL;
		}
		*fs_bits = AK4619_FS_256_96K;
	} else if (frame_clk_freq == AUDIO_PCM_RATE_192K) {
		if (ratio != AK4619_MCLK_RATIO_128) {
			LOG_ERR("fs=192kHz requires MCLK=128fs, got ratio=%d", ratio);
			return -EINVAL;
		}
		*fs_bits = AK4619_FS_128_192K;
	} else {
		LOG_ERR("Unsupported sample rate: %d Hz", frame_clk_freq);
		return -EINVAL;
	}

	return 0;
}

static int ak4619_get_didl_bits(uint8_t word_size, uint8_t *didl)
{
	switch (word_size) {
	case AUDIO_PCM_WIDTH_16_BITS:
		*didl = AK4619_DIDL_16BIT;
		break;
	case AUDIO_PCM_WIDTH_20_BITS:
		*didl = AK4619_DIDL_20BIT;
		break;
	case AUDIO_PCM_WIDTH_24_BITS:
		*didl = AK4619_DIDL_24BIT;
		break;
	case AUDIO_PCM_WIDTH_32_BITS:
		*didl = AK4619_DIDL_32BIT;
		break;
	default:
		LOG_ERR("Unsupported word size: %d", word_size);
		return -EINVAL;
	}
	return 0;
}

static int ak4619_get_dsl_bits(uint8_t slot_size, uint8_t *dsl)
{
	switch (slot_size) {
	case 16:
		*dsl = AK4619_DSL_16;
		break;
	case 20:
		*dsl = AK4619_DSL_20;
		break;
	case 24:
		*dsl = AK4619_DSL_24;
		break;
	case 32:
		*dsl = AK4619_DSL_32;
		break;
	default:
		LOG_ERR("Unsupported slot size: %d", slot_size);
		return -EINVAL;
	}

	return 0;
}

static int ak4619_get_dodl_bits(uint8_t word_size, uint8_t *dodl)
{
	switch (word_size) {
	case AUDIO_PCM_WIDTH_16_BITS:
		*dodl = AK4619_DODL_16BIT;
		break;
	case AUDIO_PCM_WIDTH_20_BITS:
		*dodl = AK4619_DODL_20BIT;
		break;
	case AUDIO_PCM_WIDTH_24_BITS:
		*dodl = AK4619_DODL_24BIT;
		break;
	case AUDIO_PCM_WIDTH_32_BITS:
		LOG_ERR("32-bit word size not supported for SDOUT");
		return -EINVAL;
	default:
		LOG_ERR("Unsupported word size: %d", word_size);
		return -EINVAL;
	}
	return 0;
}

static int ak4619_power_on(const struct device *dev)
{
	const struct ak4619_config *cfg = dev->config;
	int ret;

	if (!gpio_is_ready_dt(&cfg->pdn_gpio)) {
		LOG_ERR("PDN GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_set_dt(&cfg->pdn_gpio, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set PDN low: %d", ret);
		return ret;
	}
	k_msleep(AK4619_PDN_PULSE_MS);

	ret = gpio_pin_set_dt(&cfg->pdn_gpio, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set PDN high: %d", ret);
		return ret;
	}

	k_msleep(AK4619_INIT_WAIT_MS);

	LOG_DBG("AK4619 powered on");
	return 0;
}

static int ak4619_set_mclk_rate(const struct ak4619_config *drv_cfg, const uint32_t *ratios,
				size_t num_ratios, uint32_t frame_clk_freq, uint32_t *mclk_freq)
{
	clock_control_subsys_t subsys = (clock_control_subsys_t)(uintptr_t)drv_cfg->mclk_subsys;
	int ret = -ENOTSUP;

	for (size_t i = 0; i < num_ratios; i++) {
		ret = clock_control_set_rate(
			drv_cfg->mclk_dev, subsys,
			(clock_control_subsys_rate_t)(uintptr_t)(ratios[i] * frame_clk_freq));
		if (ret == 0 || ret == -EALREADY) {
			break;
		}
		LOG_WRN("MCLK ratio %ufs not achievable: %d", ratios[i], ret);
	}

	if (ret < 0 && ret != -EALREADY) {
		return ret;
	}

	return clock_control_get_rate(drv_cfg->mclk_dev, subsys, mclk_freq);
}

static int ak4619_configure(const struct device *dev, struct audio_codec_cfg *cfg)
{
	const struct ak4619_config *drv_cfg = dev->config;
	struct ak4619_data *data = dev->data;
	struct ak4619_if_regs if_regs;
	uint8_t didl_bits;
	uint8_t dsl_bits;
	uint8_t dodl_bits;
	uint8_t fs_bits;
	uint8_t pm_val;
	uint8_t analog_in_val;
	uint32_t mclk_freq;

	int ret;

	LOG_DBG("Configuring AK4619: fs=%d Hz, word_size=%d, channels=%d",
		cfg->dai_cfg.i2s.frame_clk_freq, cfg->dai_cfg.i2s.word_size,
		cfg->dai_cfg.i2s.channels);

	if (cfg->dai_cfg.i2s.channels != 2) {
		LOG_ERR("Unsupported channel count %d: AK4619 only supports stereo (2ch) "
			"for dai_type %d",
			cfg->dai_cfg.i2s.channels, cfg->dai_type);
		return -EINVAL;
	}

	if ((drv_cfg->adc1_filter_mode == AK4619_ADC_FILTER_VOICE ||
	     drv_cfg->adc2_filter_mode == AK4619_ADC_FILTER_VOICE) &&
	    cfg->dai_cfg.i2s.frame_clk_freq > AUDIO_PCM_RATE_48K) {
		LOG_ERR("Voice filter mode requires fs <= 48kHz, got %d Hz",
			cfg->dai_cfg.i2s.frame_clk_freq);
		return -EINVAL;
	}

	switch (cfg->dai_route) {
	case AUDIO_ROUTE_PLAYBACK:
		if (drv_cfg->dac1_disable && drv_cfg->dac2_disable) {
			LOG_ERR("PLAYBACK route requested but both DAC channels are disabled");
			return -EINVAL;
		}
		pm_val = AK4619_PMDA1 | AK4619_PMDA2;
		break;
	case AUDIO_ROUTE_CAPTURE:
		if (drv_cfg->adc1_disable && drv_cfg->adc2_disable) {
			LOG_ERR("CAPTURE route requested but both ADC channels are disabled");
			return -EINVAL;
		}
		pm_val = AK4619_PMAD1 | AK4619_PMAD2;
		break;
	case AUDIO_ROUTE_PLAYBACK_CAPTURE:
		if (drv_cfg->dac1_disable && drv_cfg->dac2_disable) {
			LOG_ERR("PLAYBACK_CAPTURE route requested but both DAC channels are "
				"disabled");
			return -EINVAL;
		}
		if (drv_cfg->adc1_disable && drv_cfg->adc2_disable) {
			LOG_ERR("PLAYBACK_CAPTURE route requested but both ADC channels are "
				"disabled");
			return -EINVAL;
		}
		pm_val = AK4619_PMDA1 | AK4619_PMDA2 | AK4619_PMAD1 | AK4619_PMAD2;
		break;
	case AUDIO_ROUTE_BYPASS:
		pm_val = 0;
		break;
	default:
		LOG_ERR("Unsupported audio route: %d", cfg->dai_route);
		return -EINVAL;
	}

	if (drv_cfg->adc1_disable) {
		pm_val &= ~AK4619_PMAD1;
	}
	if (drv_cfg->adc2_disable) {
		pm_val &= ~AK4619_PMAD2;
	}
	if (drv_cfg->dac1_disable) {
		pm_val &= ~AK4619_PMDA1;
	}
	if (drv_cfg->dac2_disable) {
		pm_val &= ~AK4619_PMDA2;
	}

	ret = ak4619_write_reg(dev, AK4619_REG_POWER_MGMT, pm_val & ~AK4619_RSTN);
	if (ret < 0) {
		return ret;
	}

	if (pm_val & (AK4619_PMDA1 | AK4619_PMDA2)) {
		ret = ak4619_get_didl_bits(cfg->dai_cfg.i2s.word_size, &didl_bits);
		if (ret < 0) {
			return ret;
		}
	} else {
		didl_bits = AK4619_DIDL_32BIT; /* Default register value */
	}

	if (pm_val & (AK4619_PMAD1 | AK4619_PMAD2)) {
		ret = ak4619_get_dodl_bits(cfg->dai_cfg.i2s.word_size, &dodl_bits);
		if (ret < 0) {
			return ret;
		}
	} else {
		dodl_bits = AK4619_DODL_24BIT; /* Default register value */
	}

	ret = ak4619_get_dsl_bits(cfg->dai_cfg.i2s.word_size, &dsl_bits);
	if (ret < 0) {
		return ret;
	}

	ret = ak4619_get_if_regs(cfg->dai_type, didl_bits, dodl_bits, dsl_bits, &if_regs);
	if (ret < 0) {
		return ret;
	}

	ret = ak4619_write_reg(dev, AK4619_REG_AUDIO_IF_1, if_regs.if1);
	if (ret < 0) {
		return ret;
	}

	ret = ak4619_write_reg(dev, AK4619_REG_AUDIO_IF_2, if_regs.if2);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("Audio IF1=0x%02X IF2=0x%02X ", if_regs.if1, if_regs.if2);

	if (cfg->dai_cfg.i2s.frame_clk_freq == AUDIO_PCM_RATE_192K) {
		static const uint32_t ratios[] = {AK4619_MCLK_RATIO_128};

		ret = ak4619_set_mclk_rate(drv_cfg, ratios, ARRAY_SIZE(ratios),
					   cfg->dai_cfg.i2s.frame_clk_freq, &mclk_freq);
	} else if (cfg->dai_cfg.i2s.frame_clk_freq == AUDIO_PCM_RATE_96K) {
		static const uint32_t ratios[] = {AK4619_MCLK_RATIO_256};

		ret = ak4619_set_mclk_rate(drv_cfg, ratios, ARRAY_SIZE(ratios),
					   cfg->dai_cfg.i2s.frame_clk_freq, &mclk_freq);
	} else {
		static const uint32_t ratios[] = {AK4619_MCLK_RATIO_256, AK4619_MCLK_RATIO_384,
						  AK4619_MCLK_RATIO_512};

		ret = ak4619_set_mclk_rate(drv_cfg, ratios, ARRAY_SIZE(ratios),
					   cfg->dai_cfg.i2s.frame_clk_freq, &mclk_freq);
	}

	if (ret < 0) {
		LOG_ERR("No achievable MCLK ratio for fs=%d Hz: %d",
			cfg->dai_cfg.i2s.frame_clk_freq, ret);
		return ret;
	}

	ret = clock_control_on(drv_cfg->mclk_dev,
			       (clock_control_subsys_t)(uintptr_t)drv_cfg->mclk_subsys);
	if (ret < 0 && ret != -ENOTSUP) {
		LOG_ERR("Failed to enable ADG MCLK");
		return ret;
	}

	ret = ak4619_get_fs_bits(cfg->dai_cfg.i2s.frame_clk_freq, mclk_freq, &fs_bits);
	if (ret < 0) {
		return ret;
	}

	ret = ak4619_write_reg(dev, AK4619_REG_SYS_CLK, fs_bits);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("System clock FS=0x%02X (fs=%d Hz, MCLK=%d Hz)", fs_bits,
		cfg->dai_cfg.i2s.frame_clk_freq, mclk_freq);

	if (!drv_cfg->adc1_disable) {
		ret = ak4619_write_reg(
			dev, AK4619_REG_MIC_GAIN_1,
			AK4619_MIC_GAIN_REG(drv_cfg->mic1_gain_l, drv_cfg->mic1_gain_r));
		if (ret < 0) {
			return ret;
		}

		ret = ak4619_write_reg(dev, AK4619_REG_ADC1_VOL_L, data->adc1_vol_l);
		if (ret < 0) {
			return ret;
		}
		ret = ak4619_write_reg(dev, AK4619_REG_ADC1_VOL_R, data->adc1_vol_r);
		if (ret < 0) {
			return ret;
		}
	}

	if (!drv_cfg->adc2_disable) {
		ret = ak4619_write_reg(
			dev, AK4619_REG_MIC_GAIN_2,
			AK4619_MIC_GAIN_REG(drv_cfg->mic2_gain_l, drv_cfg->mic2_gain_r));
		if (ret < 0) {
			return ret;
		}

		ret = ak4619_write_reg(dev, AK4619_REG_ADC2_VOL_L, data->adc2_vol_l);
		if (ret < 0) {
			return ret;
		}
		ret = ak4619_write_reg(dev, AK4619_REG_ADC2_VOL_R, data->adc2_vol_r);
		if (ret < 0) {
			return ret;
		}
	}

	if (!(drv_cfg->adc1_disable && drv_cfg->adc2_disable)) {
		ret = ak4619_write_reg(
			dev, AK4619_REG_ADC_FILTER,
			(drv_cfg->adc2_filter_mode << AK4619_ADC2_FILTER_SHIFT) |
				(drv_cfg->adc1_filter_mode << AK4619_ADC1_FILTER_SHIFT));
		if (ret < 0) {
			return ret;
		}

		analog_in_val = (drv_cfg->adc1_l_input_mode << AK4619_AD1LSEL_SHIFT) |
				(drv_cfg->adc1_r_input_mode << AK4619_AD1RSEL_SHIFT) |
				(drv_cfg->adc2_l_input_mode << AK4619_AD2LSEL_SHIFT) |
				(drv_cfg->adc2_r_input_mode << AK4619_AD2RSEL_SHIFT);

		ret = ak4619_write_reg(dev, AK4619_REG_ADC_ANALOG_IN, analog_in_val);
		if (ret < 0) {
			return ret;
		}

		ret = ak4619_write_reg(dev, AK4619_REG_ADC_MUTE_HPF, 0x00);
		if (ret < 0) {
			return ret;
		}
	}

	if (!drv_cfg->dac1_disable) {
		ret = ak4619_write_reg(dev, AK4619_REG_DAC1_VOL_L, data->dac1_vol_l);
		if (ret < 0) {
			return ret;
		}
		ret = ak4619_write_reg(dev, AK4619_REG_DAC1_VOL_R, data->dac1_vol_r);
		if (ret < 0) {
			return ret;
		}
	}

	if (!drv_cfg->dac2_disable) {
		ret = ak4619_write_reg(dev, AK4619_REG_DAC2_VOL_L, data->dac2_vol_l);
		if (ret < 0) {
			return ret;
		}
		ret = ak4619_write_reg(dev, AK4619_REG_DAC2_VOL_R, data->dac2_vol_r);
		if (ret < 0) {
			return ret;
		}
	}

	if (!(drv_cfg->dac1_disable && drv_cfg->dac2_disable)) {
		ret = ak4619_write_reg(
			dev, AK4619_REG_DAC_INPUT_SEL,
			((drv_cfg->dac2_input & 0x3) << AK4619_DAC2SEL_SHIFT) |
				((drv_cfg->dac1_input & 0x3) << AK4619_DAC1SEL_SHIFT));
		if (ret < 0) {
			return ret;
		}

		uint8_t dem1_mode = AK4619_DEM_OFF;
		uint8_t dem2_mode = AK4619_DEM_OFF;

		if (drv_cfg->dac1_deemphasis && (pm_val & (AK4619_PMDA1))) {
			switch (cfg->dai_cfg.i2s.frame_clk_freq) {
			case AUDIO_PCM_RATE_44P1K:
				dem1_mode = AK4619_DEM_44K1;
				break;
			case AUDIO_PCM_RATE_48K:
				dem1_mode = AK4619_DEM_48K;
				break;
			case AUDIO_PCM_RATE_32K:
				dem1_mode = AK4619_DEM_32K;
				break;
			default:
				break;
			}
		}

		if (drv_cfg->dac2_deemphasis && (pm_val & (AK4619_PMDA2))) {
			switch (cfg->dai_cfg.i2s.frame_clk_freq) {
			case AUDIO_PCM_RATE_44P1K:
				dem2_mode = AK4619_DEM_44K1;
				break;
			case AUDIO_PCM_RATE_48K:
				dem2_mode = AK4619_DEM_48K;
				break;
			case AUDIO_PCM_RATE_32K:
				dem2_mode = AK4619_DEM_32K;
				break;
			default:
				break;
			}
		}

		ret = ak4619_write_reg(dev, AK4619_REG_DAC_DEEMPH,
				       (dem2_mode << AK4619_DEM2_SHIFT) |
					       (dem1_mode << AK4619_DEM1_SHIFT));
		if (ret < 0) {
			return ret;
		}

		ret = ak4619_write_reg(
			dev, AK4619_REG_DAC_MUTE_FILTER,
			(drv_cfg->dac2_filter_mode << AK4619_DAC2_FILTER_SHIFT) |
				(drv_cfg->dac1_filter_mode << AK4619_DAC1_FILTER_SHIFT));
		if (ret < 0) {
			return ret;
		}
	}

	ret = ak4619_write_reg(dev, AK4619_REG_RESERVED, 0x00);
	if (ret < 0) {
		return ret;
	}

	ret = ak4619_write_reg(dev, AK4619_REG_POWER_MGMT, pm_val | AK4619_RSTN);
	if (ret < 0) {
		return ret;
	}

	if (pm_val & (AK4619_PMAD1 | AK4619_PMAD2)) {
		k_msleep(AK4619_ADC_STARTUP_MS);
	}

	LOG_INF("AK4619 configured: fs=%d Hz, word=%d-bit", cfg->dai_cfg.i2s.frame_clk_freq,
		cfg->dai_cfg.i2s.word_size);

	return 0;
}

static void ak4619_start_output(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static void ak4619_stop_output(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static int ak4619_set_property(const struct device *dev, audio_property_t property,
			       audio_channel_t channel, audio_property_value_t val)
{
	struct ak4619_data *data = dev->data;

	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_VOLUME: {
		uint8_t vol_reg;

		if (val.vol < 0x00 || val.vol > 0xFF) {
			return -EINVAL;
		}

		vol_reg = val.vol;

		LOG_DBG("Set output volume: %d → reg=0x%02X (ch=%d)", val.vol, vol_reg, channel);

		switch (channel) {
		case AUDIO_CHANNEL_FRONT_LEFT:
			data->dac1_vol_l = vol_reg;
			return ak4619_write_reg(dev, AK4619_REG_DAC1_VOL_L, vol_reg);
		case AUDIO_CHANNEL_FRONT_RIGHT:
			data->dac1_vol_r = vol_reg;
			return ak4619_write_reg(dev, AK4619_REG_DAC1_VOL_R, vol_reg);
		case AUDIO_CHANNEL_REAR_LEFT:
			data->dac2_vol_l = vol_reg;
			return ak4619_write_reg(dev, AK4619_REG_DAC2_VOL_L, vol_reg);
		case AUDIO_CHANNEL_REAR_RIGHT:
			data->dac2_vol_r = vol_reg;
			return ak4619_write_reg(dev, AK4619_REG_DAC2_VOL_R, vol_reg);
		case AUDIO_CHANNEL_ALL:
			data->dac1_vol_l = vol_reg;
			data->dac1_vol_r = vol_reg;
			data->dac2_vol_l = vol_reg;
			data->dac2_vol_r = vol_reg;
			if (ak4619_write_reg(dev, AK4619_REG_DAC1_VOL_L, vol_reg) < 0 ||
			    ak4619_write_reg(dev, AK4619_REG_DAC1_VOL_R, vol_reg) < 0 ||
			    ak4619_write_reg(dev, AK4619_REG_DAC2_VOL_L, vol_reg) < 0 ||
			    ak4619_write_reg(dev, AK4619_REG_DAC2_VOL_R, vol_reg) < 0) {
				return -EIO;
			}
			return 0;
		default:
			LOG_ERR("Unsupported output channel: %d", channel);
			return -EINVAL;
		}
	}

	case AUDIO_PROPERTY_INPUT_VOLUME: {
		uint8_t vol_reg;

		if (val.vol < 0x00 || val.vol > 0xFF) {
			return -EINVAL;
		}

		vol_reg = val.vol;

		LOG_DBG("Set input volume: %d → reg=0x%02X (ch=%d)", val.vol, vol_reg, channel);

		switch (channel) {
		case AUDIO_CHANNEL_FRONT_LEFT:
			data->adc1_vol_l = vol_reg;
			return ak4619_write_reg(dev, AK4619_REG_ADC1_VOL_L, vol_reg);
		case AUDIO_CHANNEL_FRONT_RIGHT:
			data->adc1_vol_r = vol_reg;
			return ak4619_write_reg(dev, AK4619_REG_ADC1_VOL_R, vol_reg);
		case AUDIO_CHANNEL_REAR_LEFT:
			data->adc2_vol_l = vol_reg;
			return ak4619_write_reg(dev, AK4619_REG_ADC2_VOL_L, vol_reg);
		case AUDIO_CHANNEL_REAR_RIGHT:
			data->adc2_vol_r = vol_reg;
			return ak4619_write_reg(dev, AK4619_REG_ADC2_VOL_R, vol_reg);
		case AUDIO_CHANNEL_ALL:
			data->adc1_vol_l = vol_reg;
			data->adc1_vol_r = vol_reg;
			data->adc2_vol_l = vol_reg;
			data->adc2_vol_r = vol_reg;
			if (ak4619_write_reg(dev, AK4619_REG_ADC1_VOL_L, vol_reg) < 0 ||
			    ak4619_write_reg(dev, AK4619_REG_ADC1_VOL_R, vol_reg) < 0 ||
			    ak4619_write_reg(dev, AK4619_REG_ADC2_VOL_L, vol_reg) < 0 ||
			    ak4619_write_reg(dev, AK4619_REG_ADC2_VOL_R, vol_reg) < 0) {
				return -EIO;
			}
			return 0;
		default:
			LOG_ERR("Unsupported input channel: %d", channel);
			return -EINVAL;
		}
	}

	case AUDIO_PROPERTY_OUTPUT_MUTE: {
		switch (channel) {
		case AUDIO_CHANNEL_FRONT_LEFT:
		case AUDIO_CHANNEL_FRONT_RIGHT:
			return ak4619_update_reg(dev, AK4619_REG_DAC_MUTE_FILTER, AK4619_DA1MUTE,
						 val.mute ? AK4619_DA1MUTE : 0);
		case AUDIO_CHANNEL_REAR_LEFT:
		case AUDIO_CHANNEL_REAR_RIGHT:
			return ak4619_update_reg(dev, AK4619_REG_DAC_MUTE_FILTER, AK4619_DA2MUTE,
						 val.mute ? AK4619_DA2MUTE : 0);
		case AUDIO_CHANNEL_ALL:
			return ak4619_update_reg(dev, AK4619_REG_DAC_MUTE_FILTER,
						 AK4619_DA1MUTE | AK4619_DA2MUTE,
						 val.mute ? (AK4619_DA1MUTE | AK4619_DA2MUTE) : 0);
		default:
			LOG_ERR("Unsupported output channel: %d", channel);
			return -EINVAL;
		}
	}

	case AUDIO_PROPERTY_INPUT_MUTE: {
		switch (channel) {
		case AUDIO_CHANNEL_FRONT_LEFT:
		case AUDIO_CHANNEL_FRONT_RIGHT:
			return ak4619_update_reg(dev, AK4619_REG_ADC_MUTE_HPF, AK4619_AD1MUTE,
						 val.mute ? AK4619_AD1MUTE : 0);
		case AUDIO_CHANNEL_REAR_LEFT:
		case AUDIO_CHANNEL_REAR_RIGHT:
			return ak4619_update_reg(dev, AK4619_REG_ADC_MUTE_HPF, AK4619_AD2MUTE,
						 val.mute ? AK4619_AD2MUTE : 0);
		case AUDIO_CHANNEL_ALL:
			return ak4619_update_reg(dev, AK4619_REG_ADC_MUTE_HPF,
						 AK4619_AD1MUTE | AK4619_AD2MUTE,
						 val.mute ? (AK4619_AD1MUTE | AK4619_AD2MUTE) : 0);
		default:
			LOG_ERR("Unsupported input channel: %d", channel);
			return -EINVAL;
		}
	}

	default:
		LOG_WRN("Unsupported property: %d", property);
		return -ENOTSUP;
	}
}

static int ak4619_apply_properties(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int ak4619_route_input(const struct device *dev, audio_channel_t channel, uint32_t input)
{
	uint8_t sel = (uint8_t)(input & 0x3);
	uint8_t mask;
	uint8_t val;

	switch (channel) {
	case AUDIO_CHANNEL_FRONT_LEFT:
		mask = AK4619_AD1LSEL_MASK;
		val = (sel << AK4619_AD1LSEL_SHIFT);
		break;
	case AUDIO_CHANNEL_FRONT_RIGHT:
		mask = AK4619_AD1RSEL_MASK;
		val = (sel << AK4619_AD1RSEL_SHIFT);
		break;
	case AUDIO_CHANNEL_REAR_LEFT:
		mask = AK4619_AD2LSEL_MASK;
		val = (sel << AK4619_AD2LSEL_SHIFT);
		break;
	case AUDIO_CHANNEL_REAR_RIGHT:
		mask = AK4619_AD2RSEL_MASK;
		val = (sel << AK4619_AD2RSEL_SHIFT);
		break;
	case AUDIO_CHANNEL_ALL:
		val = (sel << AK4619_AD1LSEL_SHIFT) | (sel << AK4619_AD1RSEL_SHIFT) |
		      (sel << AK4619_AD2LSEL_SHIFT) | (sel << AK4619_AD2RSEL_SHIFT);
		return ak4619_write_reg(dev, AK4619_REG_ADC_ANALOG_IN, val);
	default:
		return -EINVAL;
	}

	return ak4619_update_reg(dev, AK4619_REG_ADC_ANALOG_IN, mask, val);
}

static int ak4619_route_output(const struct device *dev, audio_channel_t channel, uint32_t output)
{
	uint8_t sel = (uint8_t)(output & 0x3);
	uint8_t mask;
	uint8_t val;

	switch (channel) {
	case AUDIO_CHANNEL_FRONT_LEFT:
	case AUDIO_CHANNEL_FRONT_RIGHT:
		mask = AK4619_DAC1SEL_MASK;
		val = (sel << AK4619_DAC1SEL_SHIFT);
		break;
	case AUDIO_CHANNEL_REAR_LEFT:
	case AUDIO_CHANNEL_REAR_RIGHT:
		mask = AK4619_DAC2SEL_MASK;
		val = (sel << AK4619_DAC2SEL_SHIFT);
		break;
	case AUDIO_CHANNEL_ALL:
		val = (sel << AK4619_DAC2SEL_SHIFT) | (sel << AK4619_DAC1SEL_SHIFT);
		return ak4619_write_reg(dev, AK4619_REG_DAC_INPUT_SEL, val);
	default:
		return -EINVAL;
	}

	return ak4619_update_reg(dev, AK4619_REG_DAC_INPUT_SEL, mask, val);
}

static int ak4619_init(const struct device *dev)
{
	const struct ak4619_config *cfg = dev->config;
	struct ak4619_data *data = dev->data;
	int ret;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus %s not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->pdn_gpio)) {
		LOG_ERR("PDN GPIO device not ready");
		return -ENODEV;
	}

	if (!device_is_ready(cfg->mclk_dev)) {
		LOG_ERR("MCLK clock control device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->pdn_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure PDN GPIO: %d", ret);
		return ret;
	}

	data->dac1_vol_l = AK4619_DAC_VOL_0DB;
	data->dac1_vol_r = AK4619_DAC_VOL_0DB;
	data->dac2_vol_l = AK4619_DAC_VOL_0DB;
	data->dac2_vol_r = AK4619_DAC_VOL_0DB;
	data->adc1_vol_l = AK4619_ADC_VOL_0DB;
	data->adc1_vol_r = AK4619_ADC_VOL_0DB;
	data->adc2_vol_l = AK4619_ADC_VOL_0DB;
	data->adc2_vol_r = AK4619_ADC_VOL_0DB;

	ret = ak4619_power_on(dev);
	if (ret < 0) {
		LOG_ERR("AK4619 power-on failed: %d", ret);
		return ret;
	}

	LOG_INF("AK4619 initialized (I2C addr=0x%02X)", cfg->i2c.addr);

	return 0;
}

static const struct audio_codec_api ak4619_driver_api = {
	.configure = ak4619_configure,
	.start_output = ak4619_start_output,
	.stop_output = ak4619_stop_output,
	.set_property = ak4619_set_property,
	.apply_properties = ak4619_apply_properties,
	.route_input = ak4619_route_input,
	.route_output = ak4619_route_output,
	.clear_errors = NULL,
	.register_error_callback = NULL,
};

#define AK4619_INIT(inst)                                                                          \
	static struct ak4619_data ak4619_data_##inst;                                              \
                                                                                                   \
	static const struct ak4619_config ak4619_config_##inst = {                                 \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.pdn_gpio = GPIO_DT_SPEC_INST_GET(inst, pdn_gpios),                                \
		.mclk_dev = DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(inst, mclk, 0)),                  \
		.mclk_subsys = DT_INST_PHA_BY_IDX(inst, mclk, 0, clock_source),                    \
		.adc1_l_input_mode = DT_INST_ENUM_IDX(inst, adc1_l_input_mode),                    \
		.adc1_r_input_mode = DT_INST_ENUM_IDX(inst, adc1_r_input_mode),                    \
		.adc2_l_input_mode = DT_INST_ENUM_IDX(inst, adc2_l_input_mode),                    \
		.adc2_r_input_mode = DT_INST_ENUM_IDX(inst, adc2_r_input_mode),                    \
		.dac1_input = DT_INST_PROP(inst, dac1_input),                                      \
		.dac2_input = DT_INST_PROP(inst, dac2_input),                                      \
		.mic1_gain_l = DT_INST_PROP_OR(inst, mic1_gain_l_mode, AK4619_MIC_GAIN_0DB),       \
		.mic1_gain_r = DT_INST_PROP_OR(inst, mic1_gain_r_mode, AK4619_MIC_GAIN_0DB),       \
		.mic2_gain_l = DT_INST_PROP_OR(inst, mic2_gain_l_mode, AK4619_MIC_GAIN_0DB),       \
		.mic2_gain_r = DT_INST_PROP_OR(inst, mic2_gain_r_mode, AK4619_MIC_GAIN_0DB),       \
		.adc1_filter_mode =                                                                \
			DT_INST_PROP_OR(inst, adc1_filter_mode, AK4619_ADC_FILTER_SHARP),          \
		.adc2_filter_mode =                                                                \
			DT_INST_PROP_OR(inst, adc2_filter_mode, AK4619_ADC_FILTER_SHARP),          \
		.dac1_filter_mode =                                                                \
			DT_INST_PROP_OR(inst, dac1_filter_mode, AK4619_DAC_FILTER_SHORT_SHARP),    \
		.dac2_filter_mode =                                                                \
			DT_INST_PROP_OR(inst, dac2_filter_mode, AK4619_DAC_FILTER_SHORT_SHARP),    \
		.adc1_disable = DT_INST_PROP(inst, adc1_disable),                                  \
		.adc2_disable = DT_INST_PROP(inst, adc2_disable),                                  \
		.dac1_disable = DT_INST_PROP(inst, dac1_disable),                                  \
		.dac2_disable = DT_INST_PROP(inst, dac2_disable),                                  \
		.dac1_deemphasis = DT_INST_PROP(inst, dac1_deemphasis),                            \
		.dac2_deemphasis = DT_INST_PROP(inst, dac2_deemphasis)};                           \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, ak4619_init, NULL, &ak4619_data_##inst, &ak4619_config_##inst, \
			      POST_KERNEL, CONFIG_AUDIO_CODEC_INIT_PRIORITY, &ak4619_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AK4619_INIT)
