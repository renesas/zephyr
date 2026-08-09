/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RENESAS_RCAR_ADG_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RENESAS_RCAR_ADG_H_

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/renesas_rcar_adg.h>

/* ADG registers offset */
#define ADG_BRRA_OFFSET           0x00  /* BRGA Baud Rate Setting Register offset */
#define ADG_BRRB_OFFSET           0x04  /* BRGB Baud Rate Setting Register offset */
#define ADG_BRGCKR_OFFSET         0x08  /* BRG Clock Select Register offset */
#define ADG_TIM_EN_OFFSET         0x30  /* Timing Signal Enable Register offset */
#define ADG_AVBCKR_OFFSET         0x100 /* AVB Clock Select Register offset */
#define ADG_AVB_SYNC_SEL0_OFFSET  0x104 /* AVB Sync Select Register 0 offset */
#define ADG_AVB_SYNC_SEL1_OFFSET  0x10C /* AVB Sync Select Register 1 offset */
#define ADG_AVB_SYNC_SEL2_OFFSET  0x110 /* AVB Sync Select Register 2 offset */
#define ADG_AVB_SYNC0_DIV0_OFFSET 0x114 /* AVB Sync Divide Register 0 offset */
#define ADG_AVB_SYNC_DIV1_OFFSET  0x118 /* AVB Sync Divide Register 1 offset */
#define ADG_AVB_CLK_DIV0_OFFSET   0x11C /* AVB Clock Divider Comparison Value Register 0 offset */
#define ADG_AVB_CLK_DIV1_OFFSET   0x120 /* AVB Clock Divider Comparison Value Register 1 offset */
#define ADG_AVB_CLK_DIV2_OFFSET   0x124 /* AVB Clock Divider Comparison Value Register 2 offset */
#define ADG_AVB_CLK_DIV3_OFFSET   0x128 /* AVB Clock Divider Comparison Value Register 3 offset */
#define ADG_AVB_CLK_DIV4_OFFSET   0x12C /* AVB Clock Divider Comparison Value Register 4 offset */
#define ADG_AVB_CLK_DIV5_OFFSET   0x130 /* AVB Clock Divider Comparison Value Register 5 offset */
#define ADG_AVB_CLK_DIV6_OFFSET   0x134 /* AVB Clock Divider Comparison Value Register 6 offset */
#define ADG_AVB_CLK_DIV7_OFFSET   0x138 /* AVB Clock Divider Comparison Value Register 7 offset */
#define ADG_AVB_CLK_CONFIG_OFFSET 0x13C /* AVB Clock Divider Configuration Register offset */

#define ADG_AVB_CLK_DIVx_OFFSET(x) (ADG_AVB_CLK_DIV0_OFFSET + 4 * x)

/* BRGx Baud Rate Setting Register */
#define ADG_BRRx_CKS_POS  8
#define ADG_BRRx_CKS_MSK  (0x3UL << ADG_BRRx_CKS_POS)
#define ADG_BRRx_BRRx_POS 0
#define ADG_BRRx_BRRx_MSK (0xFFUL << ADG_BRRx_BRRx_POS)

/* BRG Clock Select Register */
#define ADG_BRGCKR_BRGCKR_31_POS    31
#define ADG_BRGCKR_BRGCKR_31_MSK    (0x1UL << ADG_BRGCKR_BRGCKR_31_POS)
#define ADG_BRGCKR_BRGCKR_20_22_POS 20
#define ADG_BRGCKR_BRGCKR_20_22_MSK (0x7UL << ADG_BRGCKR_BRGCKR_20_22_POS)
#define ADG_BRGCKR_BRGCKR_16_18_POS 16
#define ADG_BRGCKR_BRGCKR_16_18_MSK (0x7UL << ADG_BRGCKR_BRGCKR_16_18_POS)

/* Timing Signal Enable Register */
#define ADG_TIM_EN_BRGB_TIM_EN_POS       5
#define ADG_TIM_EN_BRGB_TIM_EN_MSK       (0x1UL << ADG_TIM_EN_BRGB_TIM_EN_POS)
#define ADG_TIM_EN_BRGA_TIM_EN_POS       4
#define ADG_TIM_EN_BRGA_TIM_EN_MSK       (0x1UL << ADG_TIM_EN_BRGA_TIM_EN_POS)
#define ADG_TIM_EN_AUDIO_CLKC_TIM_EN_POS 3
#define ADG_TIM_EN_AUDIO_CLKC_TIM_EN_MSK (0x1UL << ADG_TIM_EN_AUDIO_CLKC_TIM_EN_POS)
#define ADG_TIM_EN_AUDIO_CLKB_TIM_EN_POS 2
#define ADG_TIM_EN_AUDIO_CLKB_TIM_EN_MSK (0x1UL << ADG_TIM_EN_AUDIO_CLKB_TIM_EN_POS)
#define ADG_TIM_EN_AUDIO_CLKA_TIM_EN_POS 1
#define ADG_TIM_EN_AUDIO_CLKA_TIM_EN_MSK (0x1UL << ADG_TIM_EN_AUDIO_CLKA_TIM_EN_POS)

/* AVB Clock Select Register */
#define ADG_AVBCKR_AUDIO_CLKOUT3_POS 24
#define ADG_AVBCKR_AUDIO_CLKOUT3_MSK (0xFUL << ADG_AVBCKR_AUDIO_CLKOUT3_POS)
#define ADG_AVBCKR_AUDIO_CLKOUT2_POS 16
#define ADG_AVBCKR_AUDIO_CLKOUT2_MSK (0xFUL << ADG_AVBCKR_AUDIO_CLKOUT2_POS)
#define ADG_AVBCKR_AUDIO_CLKOUT1_POS 8
#define ADG_AVBCKR_AUDIO_CLKOUT1_MSK (0xFUL << ADG_AVBCKR_AUDIO_CLKOUT1_POS)
#define ADG_AVBCKR_AUDIO_CLKOUT0_POS 0
#define ADG_AVBCKR_AUDIO_CLKOUT0_MSK (0xFUL << ADG_AVBCKR_AUDIO_CLKOUT0_POS)

#define ADG_AVBCKR_AUDIO_CLKOUTx_POS(x) (8 * x)
#define ADG_AVBCKR_AUDIO_CLKOUTx_MSK(x) (0xFUL << (8 * x))

/* AVB Sync Select Register 0 */
#define ADG_AVB_SYNC_SEL0_AVB_ADG11_SYNC_POS 24
#define ADG_AVB_SYNC_SEL0_AVB_ADG11_SYNC_MSK (0xFFUL << ADG_AVB_SYNC_SEL0_AVB_ADG11_SYNC_POS)
#define ADG_AVB_SYNC_SEL0_AVB_ADG10_SYNC_POS 16
#define ADG_AVB_SYNC_SEL0_AVB_ADG10_SYNC_MSK (0xFFUL << ADG_AVB_SYNC_SEL0_AVB_ADG10_SYNC_POS)
#define ADG_AVB_SYNC_SEL0_AVB_ADG9_SYNC_POS  8
#define ADG_AVB_SYNC_SEL0_AVB_ADG9_SYNC_MSK  (0xFFUL << ADG_AVB_SYNC_SEL0_AVB_ADG9_SYNC_POS)
#define ADG_AVB_SYNC_SEL0_AVB_ADG8_SYNC_POS  0
#define ADG_AVB_SYNC_SEL0_AVB_ADG8_SYNC_MSK  (0xFFUL << ADG_AVB_SYNC_SEL0_AVB_ADG8_SYNC_POS)

/* AVB Sync Select Register 1 */
#define ADG_AVB_SYNC_SEL1_AVB_ADG7_SYNC_POS 24
#define ADG_AVB_SYNC_SEL1_AVB_ADG7_SYNC_MSK (0xFFUL << ADG_AVB_SYNC_SEL1_AVB_ADG7_SYNC_POS)
#define ADG_AVB_SYNC_SEL1_AVB_ADG6_SYNC_POS 16
#define ADG_AVB_SYNC_SEL1_AVB_ADG6_SYNC_MSK (0xFFUL << ADG_AVB_SYNC_SEL1_AVB_ADG6_SYNC_POS)
#define ADG_AVB_SYNC_SEL1_AVB_ADG5_SYNC_POS 8
#define ADG_AVB_SYNC_SEL1_AVB_ADG5_SYNC_MSK (0xFFUL << ADG_AVB_SYNC_SEL1_AVB_ADG5_SYNC_POS)
#define ADG_AVB_SYNC_SEL1_AVB_ADG4_SYNC_POS 0
#define ADG_AVB_SYNC_SEL1_AVB_ADG4_SYNC_MSK (0xFFUL << ADG_AVB_SYNC_SEL1_AVB_ADG4_SYNC_POS)

/* AVB Sync Select Register 2 */
#define ADG_AVB_SYNC_SEL2_AVB_ADG3_SYNC_POS 24
#define ADG_AVB_SYNC_SEL2_AVB_ADG3_SYNC_MSK (0xFFUL << ADG_AVB_SYNC_SEL2_AVB_ADG3_SYNC_POS)
#define ADG_AVB_SYNC_SEL2_AVB_ADG2_SYNC_POS 16
#define ADG_AVB_SYNC_SEL2_AVB_ADG2_SYNC_MSK (0xFFUL << ADG_AVB_SYNC_SEL2_AVB_ADG2_SYNC_POS)
#define ADG_AVB_SYNC_SEL2_AVB_ADG1_SYNC_POS 8
#define ADG_AVB_SYNC_SEL2_AVB_ADG1_SYNC_MSK (0xFFUL << ADG_AVB_SYNC_SEL2_AVB_ADG1_SYNC_POS)
#define ADG_AVB_SYNC_SEL2_AVB_ADG0_SYNC_POS 0
#define ADG_AVB_SYNC_SEL2_AVB_ADG0_SYNC_MSK (0xFFUL << ADG_AVB_SYNC_SEL2_AVB_ADG0_SYNC_POS)

/* AVB Sync Divide Register 0 */
#define ADG_AVB_SYNC0_DIV0_DIV2_POS 8
#define ADG_AVB_SYNC0_DIV0_DIV2_MSK (0xFUL << ADG_AVB_SYNC0_DIV0_DIV2_POS)
#define ADG_AVB_SYNC0_DIV0_DIV1_POS 4
#define ADG_AVB_SYNC0_DIV0_DIV1_MSK (0xFUL << ADG_AVB_SYNC0_DIV0_DIV1_POS)
#define ADG_AVB_SYNC0_DIV0_DIV0_POS 0
#define ADG_AVB_SYNC0_DIV0_DIV0_MSK (0xFUL << ADG_AVB_SYNC0_DIV0_DIV0_POS)

/* AVB Sync Divide Register 1 */
#define ADG_AVB_SYNC_DIV1_DIV7_POS 28
#define ADG_AVB_SYNC_DIV1_DIV7_MSK (0xFUL << ADG_AVB_SYNC_DIV1_DIV7_POS)
#define ADG_AVB_SYNC_DIV1_DIV6_POS 24
#define ADG_AVB_SYNC_DIV1_DIV6_MSK (0xFUL << ADG_AVB_SYNC_DIV1_DIV6_POS)
#define ADG_AVB_SYNC_DIV1_DIV5_POS 20
#define ADG_AVB_SYNC_DIV1_DIV5_MSK (0xFUL << ADG_AVB_SYNC_DIV1_DIV5_POS)
#define ADG_AVB_SYNC_DIV1_DIV4_POS 16
#define ADG_AVB_SYNC_DIV1_DIV4_MSK (0xFUL << ADG_AVB_SYNC_DIV1_DIV4_POS)
#define ADG_AVB_SYNC_DIV1_DIV3_POS 12
#define ADG_AVB_SYNC_DIV1_DIV3_MSK (0xFUL << ADG_AVB_SYNC_DIV1_DIV3_POS)
#define ADG_AVB_SYNC_DIV1_DIV2_POS 8
#define ADG_AVB_SYNC_DIV1_DIV2_MSK (0xFUL << ADG_AVB_SYNC_DIV1_DIV2_POS)
#define ADG_AVB_SYNC_DIV1_DIV1_POS 4
#define ADG_AVB_SYNC_DIV1_DIV1_MSK (0xFUL << ADG_AVB_SYNC_DIV1_DIV1_POS)
#define ADG_AVB_SYNC_DIV1_DIV0_POS 0
#define ADG_AVB_SYNC_DIV1_DIV0_MSK (0xFUL << ADG_AVB_SYNC_DIV1_DIV0_POS)

/* AVB Clock Divider Comparison Value Register 0 */
#define ADG_AVB_CLK_DIV0_COMP_VALUE_POS 0
#define ADG_AVB_CLK_DIV0_COMP_VALUE_MSK (0x3FFFFUL << ADG_AVB_CLK_DIV0_COMP_VALUE_POS)

/* AVB Clock Divider Comparison Value Register 1 */
#define ADG_AVB_CLK_DIV1_COMP_VALUE_POS 0
#define ADG_AVB_CLK_DIV1_COMP_VALUE_MSK (0x3FFFFUL << ADG_AVB_CLK_DIV1_COMP_VALUE_POS)

/* AVB Clock Divider Comparison Value Register 2 */
#define ADG_AVB_CLK_DIV2_COMP_VALUE_POS 0
#define ADG_AVB_CLK_DIV2_COMP_VALUE_MSK (0x3FFFFUL << ADG_AVB_CLK_DIV2_COMP_VALUE_POS)

/* AVB Clock Divider Comparison Value Register 3 */
#define ADG_AVB_CLK_DIV3_COMP_VALUE_POS 0
#define ADG_AVB_CLK_DIV3_COMP_VALUE_MSK (0x3FFFFUL << ADG_AVB_CLK_DIV3_COMP_VALUE_POS)

/* AVB Clock Divider Comparison Value Register 4 */
#define ADG_AVB_CLK_DIV4_COMP_VALUE_POS 0
#define ADG_AVB_CLK_DIV4_COMP_VALUE_MSK (0x3FFFFUL << ADG_AVB_CLK_DIV4_COMP_VALUE_POS)

/* AVB Clock Divider Comparison Value Register 5 */
#define ADG_AVB_CLK_DIV5_COMP_VALUE_POS 0
#define ADG_AVB_CLK_DIV5_COMP_VALUE_MSK (0x3FFFFUL << ADG_AVB_CLK_DIV5_COMP_VALUE_POS)

/* AVB Clock Divider Comparison Value Register 6 */
#define ADG_AVB_CLK_DIV6_COMP_VALUE_POS 0
#define ADG_AVB_CLK_DIV6_COMP_VALUE_MSK (0x3FFFFUL << ADG_AVB_CLK_DIV6_COMP_VALUE_POS)

/* AVB Clock Divider Comparison Value Register 7 */
#define ADG_AVB_CLK_DIV7_COMP_VALUE_POS 0
#define ADG_AVB_CLK_DIV7_COMP_VALUE_MSK (0x3FFFFUL << ADG_AVB_CLK_DIV7_COMP_VALUE_POS)

/* AVB Clock Divider Configuration Register */
#define ADG_AVB_CLK_CONFIG_DIV_EN_COM_POS 31
#define ADG_AVB_CLK_CONFIG_DIV_EN_COM_MSK (0x1UL << ADG_AVB_CLK_CONFIG_DIV_EN_COM_POS)
#define ADG_AVB_CLK_CONFIG_DIV_EN_POS     0
#define ADG_AVB_CLK_CONFIG_DIV_EN_MSK     (0xFFUL << ADG_AVB_CLK_CONFIG_DIV_EN_POS)

/* AVB Clock Divider Comparison Value Fields */
#define ADG_AVB_CLK_DIV_INTEGRAL_POS   6
#define ADG_AVB_CLK_DIV_INTEGRAL_MSK   (0xFFFUL << ADG_AVB_CLK_DIV_INTEGRAL_POS)
#define ADG_AVB_CLK_DIV_FRACTIONAL_POS 0
#define ADG_AVB_CLK_DIV_FRACTIONAL_MSK (0x3FUL << ADG_AVB_CLK_DIV_FRACTIONAL_POS)

/*
 * Clock input source of a BRG unit.
 *
 * Values match the BRGCKR.BRGCKR[22:20] (BRGA) and BRGCKR.BRGCKR[18:16] (BRGB) bit fields.
 */
enum i2s_rcar_audio_brg_clkin {
	BRG_CLKIN_AUDIO_CLKA = 0, /* External clock on the AUDIO_CLKA pin */
	BRG_CLKIN_AUDIO_CLKB = 1, /* External clock on the AUDIO_CLKB pin */
	BRG_CLKIN_S0D4 = 2,       /* S0D4 internal clock */
	BRG_CLKIN_S0D4_x = 3,     /* S0D4 internal clock (alias of BRG_CLKIN_S0D4) */
	BRG_CLKIN_AUDIO_CLKC = 4, /* External clock on the AUDIO_CLKC pin */
};

/*
 * Clock source driving an AUDIO_CLKOUTx pin.
 *
 * Values match the AVBCKR.AUDIO_CLKOUTx bit fields. BRGA and BRGB are also used as BRG unit
 * indexes into the brg_rate[] and brg_clk_sel[] arrays. Since BRGCKR.BRGCKR[31] selects a
 * single BRG unit for all AUDIO_CLKOUT pins, BRGA and BRGB cannot be mixed across pins.
 */
enum i2s_rcar_audio_clkout {
	BRGA = 0,            /* BRGA output */
	BRGB = 1,            /* BRGB output */
	AVB_COUNTER8_0 = 8,  /* avb_counter8 channel 0 output */
	AVB_COUNTER8_1 = 9,  /* avb_counter8 channel 1 output */
	AVB_COUNTER8_2 = 10, /* avb_counter8 channel 2 output */
	AVB_COUNTER8_3 = 11, /* avb_counter8 channel 3 output */
	AVB_COUNTER8_4 = 12, /* avb_counter8 channel 4 output */
	AVB_COUNTER8_5 = 13, /* avb_counter8 channel 5 output */
	AVB_COUNTER8_6 = 14, /* avb_counter8 channel 6 output */
	AVB_COUNTER8_7 = 15, /* avb_counter8 channel 7 output */
};

/* Reference to a CPG clock the ADG driver depends on */
struct rcar_adg_clock_config {
	const struct device *clock_dev; /* CPG clock controller device */
	struct rcar_cpg_clk cpg;        /* CPG module/domain identifying the clock */
};

/* Rates of the clock signals the ADG can output on the AUDIO_CLKOUT pins */
struct i2s_rcar_adg_clkout {
	uint32_t brga_hz;            /* BRGA output rate */
	uint32_t brgb_hz;            /* BRGB output rate */
	uint32_t avb_counter8_hz[8]; /* avb_counter8 per-channel output rates */
};

/* Rates of the clock signals that can feed the ADG clock generators */
struct i2s_rcar_adg_clk_rate {
	uint32_t audio_clka_hz; /* Frequency of the signal input to AUDIO_CLKA pin */
	uint32_t audio_clkb_hz; /* Frequency of the signal input to AUDIO_CLKB pin */
	uint32_t audio_clkc_hz; /* Frequency of the signal input to AUDIO_CLKC pin */
	uint32_t audio_s0d4_hz; /* Frequency of the S0D4 internal clock, a BRGx input */
	uint32_t audio_s0d1_hz; /* Frequency of the S0D1 internal clock, avb_counter8 input */
};

/* Device tree derived configuration of an ADG instance */
struct clock_control_renesas_adg_cfg {
	DEVICE_MMIO_ROM;                              /* ADG register region, must be first */
	struct rcar_adg_clock_config dev_pclk;        /* ADG module clock, gates register access */
	struct rcar_adg_clock_config dev_s0d4;        /* S0D4 internal clock control instance */
	struct rcar_adg_clock_config dev_s0d1;        /* S0D1 internal clock control instance */
	struct i2s_rcar_adg_clk_rate *clkin_src_rate; /* Rates of the ADG clock input sources */
	uint8_t brg_clk_sel[2];                       /* Input clock source of each BRG unit */
};

/* Run time state of an ADG instance */
struct clock_control_renesas_adg_data {
	DEVICE_MMIO_RAM;              /* ADG register region, must be first */
	uint32_t brg_rate[2];         /* Current BRGA/BRGB output rates */
	uint32_t avbcounter8_rate[8]; /* Current avb_counter8 channel output rates */
	uint32_t clkout_src[4];       /* Clock source routed to each AUDIO_CLKOUT pin */
	struct k_mutex lock;          /* Mutex lock for this device API */
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RENESAS_RCAR_ADG_H_ */
