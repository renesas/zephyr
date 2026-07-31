/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_AUDIO_AK4619_H_
#define ZEPHYR_DRIVERS_AUDIO_AK4619_H_

/* Register map */
#define AK4619_REG_POWER_MGMT      0x00
#define AK4619_REG_AUDIO_IF_1      0x01
#define AK4619_REG_AUDIO_IF_2      0x02
#define AK4619_REG_SYS_CLK         0x03
#define AK4619_REG_MIC_GAIN_1      0x04
#define AK4619_REG_MIC_GAIN_2      0x05
#define AK4619_REG_ADC1_VOL_L      0x06
#define AK4619_REG_ADC1_VOL_R      0x07
#define AK4619_REG_ADC2_VOL_L      0x08
#define AK4619_REG_ADC2_VOL_R      0x09
#define AK4619_REG_ADC_FILTER      0x0A
#define AK4619_REG_ADC_ANALOG_IN   0x0B
#define AK4619_REG_RESERVED        0x0C
#define AK4619_REG_ADC_MUTE_HPF    0x0D
#define AK4619_REG_DAC1_VOL_L      0x0E
#define AK4619_REG_DAC1_VOL_R      0x0F
#define AK4619_REG_DAC2_VOL_L      0x10
#define AK4619_REG_DAC2_VOL_R      0x11
#define AK4619_REG_DAC_INPUT_SEL   0x12
#define AK4619_REG_DAC_DEEMPH      0x13
#define AK4619_REG_DAC_MUTE_FILTER 0x14

/* REG_POWER_MGMT (0x00) */
#define AK4619_PMAD2 BIT(5) /* ADC2 power: 0=power-down, 1=normal */
#define AK4619_PMAD1 BIT(4) /* ADC1 power: 0=power-down, 1=normal */
#define AK4619_PMDA2 BIT(2) /* DAC2 power: 0=power-down, 1=normal */
#define AK4619_PMDA1 BIT(1) /* DAC1 power: 0=power-down, 1=normal */
#define AK4619_RSTN  BIT(0) /* 0=reset, 1=normal operation */

/* REG_AUDIO_IF_1 (0x01) — TDM bit [7] */
#define AK4619_TDM_STEREO (0x0 << 7)
#define AK4619_TDM_ON     (0x1 << 7)

/* DCF[2:0] bits [6:4] — Audio Interface Format */
#define AK4619_DCF_MASK      (0x7 << 4)
#define AK4619_DCF_I2S       (0x0 << 4)
#define AK4619_DCF_MSB       (0x5 << 4)
#define AK4619_DCF_PCM_SHORT (0x6 << 4)
#define AK4619_DCF_PCM_LONG  (0x7 << 4)
#define AK4619_DCF_TDM128    (0x2 << 4) /* BICK=128fs */
#define AK4619_DCF_TDM256    (0x3 << 4) /* BICK=256fs */

/* DSL[1:0] bits [3:2] — Slot Length */
#define AK4619_DSL_MASK (0x3 << 2)
#define AK4619_DSL_24   (0x0 << 2)
#define AK4619_DSL_20   (0x1 << 2)
#define AK4619_DSL_16   (0x2 << 2)
#define AK4619_DSL_32   (0x3 << 2) /* default */

/* BCKP bit [1] — BICK edge */
#define AK4619_BCKP_FALLING (0x0 << 1) /* default */
#define AK4619_BCKP_RISING  (0x1 << 1)

/* SDOPH bit [0] — SDOUT output timing */
#define AK4619_SDOPH_SLOW (0x0 << 0) /* default */
#define AK4619_SDOPH_FAST (0x1 << 0)

/* REG_AUDIO_IF_2 (0x02) — SLOT bit [4] */
#define AK4619_SLOT_LRCK_EDGE (0x0 << 4) /* default */
#define AK4619_SLOT_LENGTH    (0x1 << 4)

/* DIDL[1:0] bits [3:2] — SDIN word length */
#define AK4619_DIDL_MASK  (0x3 << 2)
#define AK4619_DIDL_24BIT (0x0 << 2)
#define AK4619_DIDL_20BIT (0x1 << 2)
#define AK4619_DIDL_16BIT (0x2 << 2)
#define AK4619_DIDL_32BIT (0x3 << 2) /* default */

/* DODL[1:0] bits [1:0] — SDOUT word length (max 24-bit; DODL=11 N/A) */
#define AK4619_DODL_MASK  (0x3 << 0)
#define AK4619_DODL_24BIT (0x0 << 0) /* default */
#define AK4619_DODL_20BIT (0x1 << 0)
#define AK4619_DODL_16BIT (0x2 << 0)

/* REG_SYS_CLK (0x03) — FS[2:0] (datasheet Table 1) */
#define AK4619_FS_MASK     (0x7 << 0)
#define AK4619_FS_256_48K  0x00 /* default */
#define AK4619_FS_256_96K  0x01
#define AK4619_FS_384_48K  0x02
#define AK4619_FS_512_48K  0x03
#define AK4619_FS_128_192K 0x04

/* MCLK/fs ratios */
#define AK4619_MCLK_RATIO_128 128
#define AK4619_MCLK_RATIO_256 256
#define AK4619_MCLK_RATIO_384 384
#define AK4619_MCLK_RATIO_512 512

/* REG_MIC_GAIN (0x04, 0x05) — MGNxL/R[3:0] (datasheet Table 9) */
#define AK4619_MIC_GAIN_MINUS6DB 0x00
#define AK4619_MIC_GAIN_MINUS3DB 0x01
#define AK4619_MIC_GAIN_0DB      0x02 /* default */
#define AK4619_MIC_GAIN_3DB      0x03
#define AK4619_MIC_GAIN_6DB      0x04
#define AK4619_MIC_GAIN_9DB      0x05
#define AK4619_MIC_GAIN_12DB     0x06
#define AK4619_MIC_GAIN_15DB     0x07
#define AK4619_MIC_GAIN_18DB     0x08
#define AK4619_MIC_GAIN_21DB     0x09
#define AK4619_MIC_GAIN_24DB     0x0A
#define AK4619_MIC_GAIN_27DB     0x0B

#define AK4619_MIC_GAIN_REG(l, r) (((l) << 4) | ((r) & 0x0F))

/* REG_ADC_FILTER (0x0A) */
#define AK4619_AD1VO BIT(2)
#define AK4619_AD1SD BIT(1)
#define AK4619_AD1SL BIT(0)
#define AK4619_AD2VO BIT(6)
#define AK4619_AD2SD BIT(5)
#define AK4619_AD2SL BIT(4)

#define AK4619_ADC_FILTER_SHARP       0x00 /* default */
#define AK4619_ADC_FILTER_SLOW        0x01
#define AK4619_ADC_FILTER_SHORT_SHARP 0x02
#define AK4619_ADC_FILTER_SHORT_SLOW  0x03
#define AK4619_ADC_FILTER_VOICE       0x04 /* fs <= 48kHz only */

#define AK4619_ADC1_FILTER_SHIFT 0
#define AK4619_ADC2_FILTER_SHIFT 4

/* REG_ADC_ANALOG_IN (0x0B) — datasheet Table 10 */
#define AK4619_AD1LSEL_SHIFT 6
#define AK4619_AD1RSEL_SHIFT 4
#define AK4619_AD2LSEL_SHIFT 2
#define AK4619_AD2RSEL_SHIFT 0

#define AK4619_AD1LSEL_MASK (0x3 << AK4619_AD1LSEL_SHIFT)
#define AK4619_AD1RSEL_MASK (0x3 << AK4619_AD1RSEL_SHIFT)
#define AK4619_AD2LSEL_MASK (0x3 << AK4619_AD2LSEL_SHIFT)
#define AK4619_AD2RSEL_MASK (0x3 << AK4619_AD2RSEL_SHIFT)

#define AK4619_INPUT_DIFFERENTIAL 0x0 /* default */
#define AK4619_INPUT_SINGLE1      0x1
#define AK4619_INPUT_SINGLE2      0x2
#define AK4619_INPUT_PSEUDO_DIFF  0x3

/* REG_ADC_MUTE_HPF (0x0D) */
#define AK4619_ATSPAD  BIT(7) /* ADC vol ramp: 0=4/fs, 1=16/fs */
#define AK4619_AD2MUTE BIT(6)
#define AK4619_AD1MUTE BIT(5)
#define AK4619_AD2HPFN BIT(2) /* ADC2 HPF: 0=enable, 1=disable */
#define AK4619_AD1HPFN BIT(1) /* ADC1 HPF: 0=enable, 1=disable */

/* REG_DAC_INPUT_SEL (0x12) — datasheet Table 17/18 */
#define AK4619_DAC2SEL_SHIFT 2
#define AK4619_DAC1SEL_SHIFT 0

#define AK4619_DAC2SEL_MASK   (0x3 << AK4619_DAC2SEL_SHIFT)
#define AK4619_DAC2SEL_SDIN1  (0x0 << AK4619_DAC2SEL_SHIFT)
#define AK4619_DAC2SEL_SDIN2  (0x1 << AK4619_DAC2SEL_SHIFT) /* default */
#define AK4619_DAC2SEL_SDOUT1 (0x2 << AK4619_DAC2SEL_SHIFT)
#define AK4619_DAC2SEL_SDOUT2 (0x3 << AK4619_DAC2SEL_SHIFT)

#define AK4619_DAC1SEL_MASK   (0x3 << AK4619_DAC1SEL_SHIFT)
#define AK4619_DAC1SEL_SDIN1  (0x0 << AK4619_DAC1SEL_SHIFT) /* default */
#define AK4619_DAC1SEL_SDIN2  (0x1 << AK4619_DAC1SEL_SHIFT)
#define AK4619_DAC1SEL_SDOUT1 (0x2 << AK4619_DAC1SEL_SHIFT)
#define AK4619_DAC1SEL_SDOUT2 (0x3 << AK4619_DAC1SEL_SHIFT)

/* REG_DAC_DEEMPH (0x13) — datasheet Table 25 */
#define AK4619_DEM2_SHIFT 2
#define AK4619_DEM1_SHIFT 0

#define AK4619_DEM2_MASK (0x3 << AK4619_DEM2_SHIFT)
#define AK4619_DEM1_MASK (0x3 << AK4619_DEM1_SHIFT)
#define AK4619_DEM_44K1  0x0
#define AK4619_DEM_OFF   0x1 /* default */
#define AK4619_DEM_48K   0x2
#define AK4619_DEM_32K   0x3

/* REG_DAC_MUTE_FILTER (0x14) */
#define AK4619_ATSPDA  BIT(7) /* DAC vol ramp: 0=4/fs, 1=16/fs */
#define AK4619_DA2MUTE BIT(5)
#define AK4619_DA1MUTE BIT(4)
#define AK4619_DA2SD   BIT(3)
#define AK4619_DA2SL   BIT(2)
#define AK4619_DA1SD   BIT(1)
#define AK4619_DA1SL   BIT(0)

#define AK4619_DAC_FILTER_SHARP       0x0
#define AK4619_DAC_FILTER_SLOW        0x1
#define AK4619_DAC_FILTER_SHORT_SHARP 0x2 /* default */
#define AK4619_DAC_FILTER_SHORT_SLOW  0x3

#define AK4619_DAC1_FILTER_SHIFT 0
#define AK4619_DAC2_FILTER_SHIFT 2

/* Volume defaults — DAC: 0x18=0dB, ADC: 0x30=0dB, 0xFF=mute */
#define AK4619_DAC_VOL_0DB  0x18
#define AK4619_DAC_VOL_11DB 0x02
#define AK4619_DAC_VOL_MUTE 0xFF
#define AK4619_ADC_VOL_0DB  0x30
#define AK4619_ADC_VOL_MUTE 0xFF

/* Timing constants */
#define AK4619_PDN_PULSE_MS 1 /* min 600ns PDN pulse */
#define AK4619_INIT_WAIT_MS                                                                        \
	100 /* analog input coupling-cap charge after PDN=H, before PMAD=1 (datasheet 9.6) */
#define AK4619_ADC_STARTUP_MS                                                                      \
	100 /* ADC init cycle settle after RSTN=1, ~1056/fs (datasheet Fig. 22) */

struct ak4619_if_regs {
	uint8_t if1;
	uint8_t if2;
};

#endif /* ZEPHYR_DRIVERS_AUDIO_AK4619_H_ */
