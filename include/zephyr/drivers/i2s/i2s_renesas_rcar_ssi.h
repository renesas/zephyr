/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I2S_RENESAS_RCAR_SSI_H
#define ZEPHYR_INCLUDE_DRIVERS_I2S_RENESAS_RCAR_SSI_H

/******************* SSI register offsets *******************/
#define SSI_SSICR_OFFSET  0x00 /* Control Register */
#define SSI_SSISR_OFFSET  0x04 /* Status Register */
#define SSI_SSITDR_OFFSET 0x08 /* Transmit Data Register */
#define SSI_SSIRDR_OFFSET 0x0C /* Receive Data Register */
#define SSI_SSIWSR_OFFSET 0x20 /* WS Mode Register */
#define SSI_SSIFMR_OFFSET 0x24 /* Frequency Switching Detection Mode Register */
#define SSI_SSIFSR_OFFSET 0x28 /* Frequency Switching Detection Status Register */
#define SSI_SSICRE_OFFSET 0x30 /* Control Register Extended */

/* SSI SSICR Register */
#define SSI_SSICR_FORCE_POS 31
#define SSI_SSICR_FORCE_MSK (0x1UL << SSI_SSICR_FORCE_POS) /* Fixed value (write 1) */
#define SSI_SSICR_FIEN_POS  29
#define SSI_SSICR_FIEN_MSK                                                                         \
	(0x1UL << SSI_SSICR_FIEN_POS) /* Frequency Switching Detection Interrupt Enable */
#define SSI_SSICR_DMEN_POS 28
#define SSI_SSICR_DMEN_MSK (0x1UL << SSI_SSICR_DMEN_POS) /* DMA Enable */
#define SSI_SSICR_UIEN_POS 27
#define SSI_SSICR_UIEN_MSK (0x1UL << SSI_SSICR_UIEN_POS) /* Underflow Interrupt Enable */
#define SSI_SSICR_OIEN_POS 26
#define SSI_SSICR_OIEN_MSK (0x1UL << SSI_SSICR_OIEN_POS) /* Overflow Interrupt Enable */
#define SSI_SSICR_IIEN_POS 25
#define SSI_SSICR_IIEN_MSK (0x1UL << SSI_SSICR_IIEN_POS) /* Idle Mode Interrupt Enable */
#define SSI_SSICR_DIEN_POS 24
#define SSI_SSICR_DIEN_MSK (0x1UL << SSI_SSICR_DIEN_POS) /* Data Interrupt Enable */
#define SSI_SSICR_CHNL_POS 22
#define SSI_SSICR_CHNL_MSK (0x3UL << SSI_SSICR_CHNL_POS) /* Channels per system word (23:22) */
#define SSI_SSICR_DWL_POS  19
#define SSI_SSICR_DWL_MSK  (0x7UL << SSI_SSICR_DWL_POS) /* Data Word Length (bits 21:19) */
#define SSI_SSICR_SWL_POS  16
#define SSI_SSICR_SWL_MSK  (0x7UL << SSI_SSICR_SWL_POS) /* System Word Length (bits 18:16) */
#define SSI_SSICR_SCKD_POS 15
#define SSI_SSICR_SCKD_MSK (0x1UL << SSI_SSICR_SCKD_POS) /* Serial Bit Clock Direction */
#define SSI_SSICR_SWSD_POS 14
#define SSI_SSICR_SWSD_MSK (0x1UL << SSI_SSICR_SWSD_POS) /* Serial WS Direction */
#define SSI_SSICR_SCKP_POS 13
#define SSI_SSICR_SCKP_MSK (0x1UL << SSI_SSICR_SCKP_POS) /* Serial Bit Clock Polarity */
#define SSI_SSICR_SWSP_POS 12
#define SSI_SSICR_SWSP_MSK (0x1UL << SSI_SSICR_SWSP_POS) /* Serial WS Polarity */
#define SSI_SSICR_SPDP_POS 11
#define SSI_SSICR_SPDP_MSK (0x1UL << SSI_SSICR_SPDP_POS) /* Serial Padding Polarity */
#define SSI_SSICR_SDTA_POS 10
#define SSI_SSICR_SDTA_MSK (0x1UL << SSI_SSICR_SDTA_POS) /* Serial Data Alignment */
#define SSI_SSICR_PDTA_POS 9
#define SSI_SSICR_PDTA_MSK (0x1UL << SSI_SSICR_PDTA_POS) /* Parallel Data Alignment */
#define SSI_SSICR_DEL_POS  8
#define SSI_SSICR_DEL_MSK  (0x1UL << SSI_SSICR_DEL_POS) /* Serial Data Delay */
#define SSI_SSICR_CKDV_POS 4
#define SSI_SSICR_CKDV_MSK                                                                         \
	(0x7UL << SSI_SSICR_CKDV_POS) /* Serial Oversampling Clock Division Ratio (bits 6:4) */
#define SSI_SSICR_MUEN_POS 3
#define SSI_SSICR_MUEN_MSK (0x1UL << SSI_SSICR_MUEN_POS) /* Mute (Serial Data Output Disable) */
#define SSI_SSICR_TRMD_POS 1
#define SSI_SSICR_TRMD_MSK (0x1UL << SSI_SSICR_TRMD_POS) /* Transmit/Receive Mode Select */
#define SSI_SSICR_EN_POS   0
#define SSI_SSICR_EN_MSK   (0x1UL << SSI_SSICR_EN_POS) /* SSI Module Enable */

/* SSI SSISR Register */
#define SSI_SSISR_DMRQ_POS 28
#define SSI_SSISR_DMRQ_MSK (0x1UL << SSI_SSISR_DMRQ_POS) /* DMA Request Status Flag */
#define SSI_SSISR_UIRQ_POS 27
#define SSI_SSISR_UIRQ_MSK (0x1UL << SSI_SSISR_UIRQ_POS) /* Underflow Interrupt Status Flag */
#define SSI_SSISR_OIRQ_POS 26
#define SSI_SSISR_OIRQ_MSK (0x1UL << SSI_SSISR_OIRQ_POS) /* Overflow Interrupt Status Flag */
#define SSI_SSISR_IIRQ_POS 25
#define SSI_SSISR_IIRQ_MSK (0x1UL << SSI_SSISR_IIRQ_POS) /* Idle Mode Interrupt Status Flag */
#define SSI_SSISR_DIRQ_POS 24
#define SSI_SSISR_DIRQ_MSK (0x1UL << SSI_SSISR_DIRQ_POS) /* Data Interrupt Status Flag */
#define SSI_SSISR_CHNO_POS 2
#define SSI_SSISR_CHNO_MSK (0x3UL << SSI_SSISR_CHNO_POS) /* Channel Number (bits 3:2) */
#define SSI_SSISR_SWNO_POS 1
#define SSI_SSISR_SWNO_MSK (0x1UL << SSI_SSISR_SWNO_POS) /* System Word Number */
#define SSI_SSISR_IDST_POS 0
#define SSI_SSISR_IDST_MSK (0x1UL << SSI_SSISR_IDST_POS) /* Idle Mode Status Flag */

/* SSI SSITDR Register */
#define SSI_SSITDR_SSITDR_POS 0
#define SSI_SSITDR_SSITDR_MSK (0xFFFFFFFFUL << SSI_SSITDR_SSITDR_POS) /* Transmit Data */

/* SSI SSIRDR Register */
#define SSI_SSIRDR_SSIRDR_POS 0
#define SSI_SSIRDR_SSIRDR_MSK (0xFFFFFFFFUL << SSI_SSIRDR_SSIRDR_POS) /* Receive Data */

/* SSI SSIWSR Register */
#define SSI_SSIWSR_WIDTH_POS   16
#define SSI_SSIWSR_WIDTH_MSK   (0x1FUL << SSI_SSIWSR_WIDTH_POS) /* SYNC Pulse Width Change */
#define SSI_SSIWSR_CONT_POS    8
#define SSI_SSIWSR_CONT_MSK    (0x1UL << SSI_SSIWSR_CONT_POS) /* WS Continue Function */
#define SSI_SSIWSR_MONO_POS    1
#define SSI_SSIWSR_MONO_MSK    (0x1UL << SSI_SSIWSR_MONO_POS) /* TDM Format/Monaural Format */
#define SSI_SSIWSR_WS_MODE_POS 0
#define SSI_SSIWSR_WS_MODE_MSK (0x1UL << SSI_SSIWSR_WS_MODE_POS) /* WS Mode */

/* SSI SSIFMR Register */
#define SSI_SSIFMR_DTCT_POS 16
#define SSI_SSIFMR_DTCT_MSK                                                                        \
	(0x3FUL << SSI_SSIFMR_DTCT_POS) /* Frequency Switching Detection Range */
#define SSI_SSIFMR_CTDV_POS 4
#define SSI_SSIFMR_CTDV_MSK (0x3UL << SSI_SSIFMR_CTDV_POS) /* Bus Clock Division Ratio */
#define SSI_SSIFMR_FSEN_POS 0
#define SSI_SSIFMR_FSEN_MSK                                                                        \
	(0x1UL << SSI_SSIFMR_FSEN_POS) /* Frequency Switching Detection Function Enable */

/* SSI SSIFSR Register */
#define SSI_SSIFSR_FCST_POS 15
#define SSI_SSIFSR_FCST_MSK (0x1UL << SSI_SSIFSR_FCST_POS) /* WS Stopped Status */
#define SSI_SSIFSR_DSDT_POS 14
#define SSI_SSIFSR_DSDT_MSK                                                                        \
	(0x1UL << SSI_SSIFSR_DSDT_POS) /* Frequency Switching Detection Status */
#define SSI_SSIFSR_FCNT_POS 0
#define SSI_SSIFSR_FCNT_MSK (0xFFFUL << SSI_SSIFSR_FCNT_POS) /* Frequency Count Monitor */

/* SSI SSICRE Register */
#define SSI_SSICRE_CHL2_POS 0
#define SSI_SSICRE_CHL2_MSK (0x3UL << SSI_SSICRE_CHL2_POS) /* SYNC Pulse Width (bits 1:0) */

/******************* SSIU register offsets *******************/
#define SSIU_BUSIF_MODE_OFFSET(x)   (0x00 + 0x1000 * (x)) /* SSI_BUSIF[x]_MODE */
#define SSIU_BUSIF_ADINR_OFFSET(x)  (0x04 + 0x1000 * (x)) /* SSI_BUSIF[x]_ADINR */
#define SSIU_CONTROL_OFFSET(x)      (0x10 + 0x1000 * (x)) /* SSI_CTRL[x] */
#define SSIU_0_BUSIF_DALIGN_OFFSET  0x20                  /* SSI_BUSIF0_DALIGN */
#define SSIU_0_BUSIF_DALIGN2_OFFSET 0x24                  /* SSI_BUSIF0_DALIGN2 */
#define SSIU_1_BUSIF_DALIGN_OFFSET  0x1020                /* SSI_BUSIF1_DALIGN */
#define SSIU_2_BUSIF_DALIGN_OFFSET  0x2020                /* SSI_BUSIF2_DALIGN */
#define SSIU_3_BUSIF_DALIGN_OFFSET  0x3020                /* SSI_BUSIF3_DALIGN */
#define SSIU_4_BUSIF_DALIGN_OFFSET  0x4020                /* SSI_BUSIF4_DALIGN */
#define SSIU_5_BUSIF_DALIGN_OFFSET  0x5020                /* SSI_BUSIF5_DALIGN */
#define SSIU_6_BUSIF_DALIGN_OFFSET  0x6020                /* SSI_BUSIF6_DALIGN */
#define SSIU_7_BUSIF_DALIGN_OFFSET  0x7020                /* SSI_BUSIF7_DALIGN */
#define SSIU_MODE1_OFFSET           0x8004                /* SSIU_MODE1 */
#define SSIU_MODE2_OFFSET           0x8008                /* SSIU_MODE2 */
#define SSIU_MODE3_OFFSET           0x800C                /* SSIU_MODE3 */
#define SSIU_MODE4_OFFSET           0x8010                /* SSIU_MODE4 */
#define SSIU_MODE5_OFFSET           0x8014                /* SSIU_MODE5 */
#define SSIU_0_CONTROL_OFFSET       0x8018                /* SSIU_0_CONTROL */
#define SSIU_STATUS_OFFSET          0x8040                /* SSIU_STATUS */
#define SSIU_INT_ENABLE_OFFSET      0x8090                /* SSIU_INT_ENABLE */
#define SSIU_AUDIO_CLK_SEL_OFFSET   0xA000                /* SSIU_AUDIO_CLK_SEL */
#define SSIP_BUSIF_OFFSET(x)        (0x00 + 0x1000 * (x)) /* SSIP[x] BUSIF Data Register */
#define PDMA_BUSIFP_OFFSET(x)       (0x00 + 0x400 * (x))  /* PDMA[x] BUSIFp Data Register */

/* SSIU BUSIF Mode Register */
#define SSIU_BUSIF_MODE_SFT_DIR_POS   20
#define SSIU_BUSIF_MODE_SFT_DIR_MSK   (0x1UL << SSIU_BUSIF_MODE_SFT_DIR_POS) /* Shift direction */
#define SSIU_BUSIF_MODE_SFT_NUM_POS   16
#define SSIU_BUSIF_MODE_SFT_NUM_MSK   (0xFUL << SSIU_BUSIF_MODE_SFT_NUM_POS) /* Shift number */
#define SSIU_BUSIF_MODE_WORD_SWAP_POS 8
#define SSIU_BUSIF_MODE_WORD_SWAP_MSK                                                              \
	(0x1UL << SSIU_BUSIF_MODE_WORD_SWAP_POS) /* Word swap enable */
#define SSIU_BUSIF_MODE_DMA_POS 0
#define SSIU_BUSIF_MODE_DMA_MSK (0x1UL << SSIU_BUSIF_MODE_DMA_POS) /* DMA enable */

/* SSIU BUSIF Audio Information Register (ADINR) */
#define SSIU_BUSIF_ADINR_OTBL_POS  16
#define SSIU_BUSIF_ADINR_OTBL_MSK  (0x1FUL << SSIU_BUSIF_ADINR_OTBL_POS) /* Output bit length */
#define SSIU_BUSIF_ADINR_CHNUM_POS 0
#define SSIU_BUSIF_ADINR_CHNUM_MSK (0xFUL << SSIU_BUSIF_ADINR_CHNUM_POS) /* Number of channels */

/* SSIU Control Register */
#define SSIU_CONTROL_START_POS 0
#define SSIU_CONTROL_START_MSK (0x1UL << SSIU_CONTROL_START_POS) /* BUSIF transfer start */

/* SSIU Status Register */
#define SSIU_STATUS_FCST_POS 29
#define SSIU_STATUS_FCST_MSK (0x1UL << SSIU_STATUS_FCST_POS) /* WS Stopped Status */
#define SSIU_STATUS_DTST_POS 28
#define SSIU_STATUS_DTST_MSK (0x1UL << SSIU_STATUS_DTST_POS) /* Frequency Detection Status */
#define SSIU_STATUS_UIRQ_POS 27
#define SSIU_STATUS_UIRQ_MSK (0x1UL << SSIU_STATUS_UIRQ_POS) /* Underflow Interrupt Status */
#define SSIU_STATUS_OIRQ_POS 26
#define SSIU_STATUS_OIRQ_MSK (0x1UL << SSIU_STATUS_OIRQ_POS) /* Overflow Interrupt Status */
#define SSIU_STATUS_IIRQ_POS 25
#define SSIU_STATUS_IIRQ_MSK (0x1UL << SSIU_STATUS_IIRQ_POS) /* Idle Mode Interrupt Status */
#define SSIU_STATUS_DIRQ_POS 24
#define SSIU_STATUS_DIRQ_MSK (0x1UL << SSIU_STATUS_DIRQ_POS) /* Data Interrupt Status */
#define SSIU_STATUS_UF7_POS  23
#define SSIU_STATUS_UF7_MSK  (0x1UL << SSIU_STATUS_UF7_POS) /* BUSIF7 Underflow Status */
#define SSIU_STATUS_UF6_POS  22
#define SSIU_STATUS_UF6_MSK  (0x1UL << SSIU_STATUS_UF6_POS) /* BUSIF6 Underflow Status */
#define SSIU_STATUS_UF5_POS  21
#define SSIU_STATUS_UF5_MSK  (0x1UL << SSIU_STATUS_UF5_POS) /* BUSIF5 Underflow Status */
#define SSIU_STATUS_UF4_POS  20
#define SSIU_STATUS_UF4_MSK  (0x1UL << SSIU_STATUS_UF4_POS) /* BUSIF4 Underflow Status */
#define SSIU_STATUS_UF3_POS  19
#define SSIU_STATUS_UF3_MSK  (0x1UL << SSIU_STATUS_UF3_POS) /* BUSIF3 Underflow Status */
#define SSIU_STATUS_UF2_POS  18
#define SSIU_STATUS_UF2_MSK  (0x1UL << SSIU_STATUS_UF2_POS) /* BUSIF2 Underflow Status */
#define SSIU_STATUS_UF1_POS  17
#define SSIU_STATUS_UF1_MSK  (0x1UL << SSIU_STATUS_UF1_POS) /* BUSIF1 Underflow Status */
#define SSIU_STATUS_UF0_POS  16
#define SSIU_STATUS_UF0_MSK  (0x1UL << SSIU_STATUS_UF0_POS) /* BUSIF0 Underflow Status */
#define SSIU_STATUS_OF7_POS  7
#define SSIU_STATUS_OF7_MSK  (0x1UL << SSIU_STATUS_OF7_POS) /* BUSIF7 Overflow Status */
#define SSIU_STATUS_OF6_POS  6
#define SSIU_STATUS_OF6_MSK  (0x1UL << SSIU_STATUS_OF6_POS) /* BUSIF6 Overflow Status */
#define SSIU_STATUS_OF5_POS  5
#define SSIU_STATUS_OF5_MSK  (0x1UL << SSIU_STATUS_OF5_POS) /* BUSIF5 Overflow Status */
#define SSIU_STATUS_OF4_POS  4
#define SSIU_STATUS_OF4_MSK  (0x1UL << SSIU_STATUS_OF4_POS) /* BUSIF4 Overflow Status */
#define SSIU_STATUS_OF3_POS  3
#define SSIU_STATUS_OF3_MSK  (0x1UL << SSIU_STATUS_OF3_POS) /* BUSIF3 Overflow Status */
#define SSIU_STATUS_OF2_POS  2
#define SSIU_STATUS_OF2_MSK  (0x1UL << SSIU_STATUS_OF2_POS) /* BUSIF2 Overflow Status */
#define SSIU_STATUS_OF1_POS  1
#define SSIU_STATUS_OF1_MSK  (0x1UL << SSIU_STATUS_OF1_POS) /* BUSIF1 Overflow Status */
#define SSIU_STATUS_OF0_POS  0
#define SSIU_STATUS_OF0_MSK  (0x1UL << SSIU_STATUS_OF0_POS) /* BUSIF0 Overflow Status */

/* SSIU Interrupt Enable Register */
#define SSIU_INT_ENABLE_FCST_IE_POS 29
#define SSIU_INT_ENABLE_FCST_IE_MSK                                                                \
	(0x1UL << SSIU_INT_ENABLE_FCST_IE_POS) /* WS Stopped Interrupt Enable */
#define SSIU_INT_ENABLE_DTST_IE_POS 28
#define SSIU_INT_ENABLE_DTST_IE_MSK                                                                \
	(0x1UL << SSIU_INT_ENABLE_DTST_IE_POS) /* Frequency Detection Interrupt Enable */
#define SSIU_INT_ENABLE_UIRQ_IE_POS 27
#define SSIU_INT_ENABLE_UIRQ_IE_MSK                                                                \
	(0x1UL << SSIU_INT_ENABLE_UIRQ_IE_POS) /* Underflow Interrupt Enable */
#define SSIU_INT_ENABLE_OIRQ_IE_POS 26
#define SSIU_INT_ENABLE_OIRQ_IE_MSK                                                                \
	(0x1UL << SSIU_INT_ENABLE_OIRQ_IE_POS) /* Overflow Interrupt Enable */
#define SSIU_INT_ENABLE_IIRQ_IE_POS 25
#define SSIU_INT_ENABLE_IIRQ_IE_MSK                                                                \
	(0x1UL << SSIU_INT_ENABLE_IIRQ_IE_POS) /* Idle Mode Interrupt Enable */
#define SSIU_INT_ENABLE_DIRQ_IE_POS 24
#define SSIU_INT_ENABLE_DIRQ_IE_MSK                                                                \
	(0x1UL << SSIU_INT_ENABLE_DIRQ_IE_POS) /* Data Interrupt Enable */
#define SSIU_INT_ENABLE_UF7_IE_POS 23
#define SSIU_INT_ENABLE_UF7_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_UF7_IE_POS) /* BUSIF7 Underflow Interrupt Enable */
#define SSIU_INT_ENABLE_UF6_IE_POS 22
#define SSIU_INT_ENABLE_UF6_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_UF6_IE_POS) /* BUSIF6 Underflow Interrupt Enable */
#define SSIU_INT_ENABLE_UF5_IE_POS 21
#define SSIU_INT_ENABLE_UF5_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_UF5_IE_POS) /* BUSIF5 Underflow Interrupt Enable */
#define SSIU_INT_ENABLE_UF4_IE_POS 20
#define SSIU_INT_ENABLE_UF4_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_UF4_IE_POS) /* BUSIF4 Underflow Interrupt Enable */
#define SSIU_INT_ENABLE_UF3_IE_POS 19
#define SSIU_INT_ENABLE_UF3_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_UF3_IE_POS) /* BUSIF3 Underflow Interrupt Enable */
#define SSIU_INT_ENABLE_UF2_IE_POS 18
#define SSIU_INT_ENABLE_UF2_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_UF2_IE_POS) /* BUSIF2 Underflow Interrupt Enable */
#define SSIU_INT_ENABLE_UF1_IE_POS 17
#define SSIU_INT_ENABLE_UF1_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_UF1_IE_POS) /* BUSIF1 Underflow Interrupt Enable */
#define SSIU_INT_ENABLE_UF0_IE_POS 16
#define SSIU_INT_ENABLE_UF0_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_UF0_IE_POS) /* BUSIF0 Underflow Interrupt Enable */
#define SSIU_INT_ENABLE_OF7_IE_POS 7
#define SSIU_INT_ENABLE_OF7_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_OF7_IE_POS) /* BUSIF7 Overflow Interrupt Enable */
#define SSIU_INT_ENABLE_OF6_IE_POS 6
#define SSIU_INT_ENABLE_OF6_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_OF6_IE_POS) /* BUSIF6 Overflow Interrupt Enable */
#define SSIU_INT_ENABLE_OF5_IE_POS 5
#define SSIU_INT_ENABLE_OF5_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_OF5_IE_POS) /* BUSIF5 Overflow Interrupt Enable */
#define SSIU_INT_ENABLE_OF4_IE_POS 4
#define SSIU_INT_ENABLE_OF4_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_OF4_IE_POS) /* BUSIF4 Overflow Interrupt Enable */
#define SSIU_INT_ENABLE_OF3_IE_POS 3
#define SSIU_INT_ENABLE_OF3_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_OF3_IE_POS) /* BUSIF3 Overflow Interrupt Enable */
#define SSIU_INT_ENABLE_OF2_IE_POS 2
#define SSIU_INT_ENABLE_OF2_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_OF2_IE_POS) /* BUSIF2 Overflow Interrupt Enable */
#define SSIU_INT_ENABLE_OF1_IE_POS 1
#define SSIU_INT_ENABLE_OF1_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_OF1_IE_POS) /* BUSIF1 Overflow Interrupt Enable */
#define SSIU_INT_ENABLE_OF0_IE_POS 0
#define SSIU_INT_ENABLE_OF0_IE_MSK                                                                 \
	(0x1UL << SSIU_INT_ENABLE_OF0_IE_POS) /* BUSIF0 Overflow Interrupt Enable */

/* SSIU Mode Register 4 */
#define SSIU_MODE4_FS_MODE_POS   13
#define SSIU_MODE4_FS_MODE_MSK   (0x1UL << SSIU_MODE4_FS_MODE_POS) /* FS Mode */
#define SSIU_MODE4_TDM_SPLIT_POS 8
#define SSIU_MODE4_TDM_SPLIT_MSK (0x1UL << SSIU_MODE4_TDM_SPLIT_POS) /* TDM Split */
#define SSIU_MODE4_TDM_EXT_POS   0
#define SSIU_MODE4_TDM_EXT_MSK   (0x1UL << SSIU_MODE4_TDM_EXT_POS) /* TDM Extension */

/* SSIU Mode Register 5 */
#define SSIU_MODE5_EX_FUNC_POS 0
#define SSIU_MODE5_EX_FUNC_MSK (0x1UL << SSIU_MODE5_EX_FUNC_POS) /* Extended Function Enable */

/* SSIU Unit-specific Control Register */
#define SSIU_0_CONTROL_SSI0129_POS 0
#define SSIU_0_CONTROL_SSI0129_MSK (0x1UL << SSIU_0_CONTROL_SSI0129_POS) /* SSI0,1,2,9 Enable */

/* SSIU Unit-specific Mode Register 1 */
#define SSIU_0_MODE1_SSI012_3MOD_POS 4
#define SSIU_0_MODE1_SSI012_3MOD_MSK                                                               \
	(0x1UL << SSIU_0_MODE1_SSI012_3MOD_POS) /* SSI0-2 3-Module Mode */
#define SSIU_0_MODE1_SSI2_PIN_POS 2
#define SSIU_0_MODE1_SSI2_PIN_MSK (0x3UL << SSIU_0_MODE1_SSI2_PIN_POS) /* SSI2 Pin Mode */
#define SSIU_0_MODE1_SSI1_PIN_POS 0
#define SSIU_0_MODE1_SSI1_PIN_MSK (0x3UL << SSIU_0_MODE1_SSI1_PIN_POS) /* SSI1 Pin Mode */

/* SSIU Unit-specific Mode Register 2 */
#define SSIU_0_MODE2_SSI0129_4MOD_POS 4
#define SSIU_0_MODE2_SSI0129_4MOD_MSK                                                              \
	(0x1UL << SSIU_0_MODE2_SSI0129_4MOD_POS) /* SSI0,1,2,9 4-Module Mode */
#define SSIU_0_MODE2_SSI0_9_PIN_POS 0
#define SSIU_0_MODE2_SSI0_9_PIN_MSK (0x3UL << SSIU_0_MODE2_SSI0_9_PIN_POS) /* SSI0-9 Pin Mode */

/* SSIU Unit-specific Mode Register 3 */
#define SSIU_0_MODE3_SSI3_PIN_POS 0
#define SSIU_0_MODE3_SSI3_PIN_MSK (0x3UL << SSIU_0_MODE3_SSI3_PIN_POS) /* SSI3 Pin Mode */

/* SSIU Audio Clock Select Register */
#define SSIU_AUDIO_CLK_SEL_DIVSEL_POS 8
#define SSIU_AUDIO_CLK_SEL_DIVSEL_MSK                                                              \
	(0x7UL << SSIU_AUDIO_CLK_SEL_DIVSEL_POS) /* SSI Frequency Divider Select */
#define SSIU_AUDIO_CLK_SEL_ACLK_SEL_POS 4
#define SSIU_AUDIO_CLK_SEL_ACLK_SEL_MSK                                                            \
	(0x3UL << SSIU_AUDIO_CLK_SEL_ACLK_SEL_POS) /* SSI Audio Clock Source Select */
#define SSIU_AUDIO_CLK_SEL_DIVCLK_SEL_POS 0
#define SSIU_AUDIO_CLK_SEL_DIVCLK_SEL_MSK                                                          \
	(0x7UL << SSIU_AUDIO_CLK_SEL_DIVCLK_SEL_POS) /* SSI Divided Clock Source Select */

/* SSIP BUSIF Data Register */
#define SSIP_BUSIF_DATA_POS 0
#define SSIP_BUSIF_DATA_MSK (0xFFFFFFFFUL << SSIP_BUSIF_DATA_POS) /* BUSIF data register */

/* PDMA BUSIFp Data Register */
#define PDMA_BUSIFP_DATA_POS 0
#define PDMA_BUSIFP_DATA_MSK (0xFFFFFFFFUL << PDMA_BUSIFP_DATA_POS) /* BUSIFp data register */

/* SSIU_STATUS/SSIU_INT_ENABLE underflow and overflow bits owned by an SSI instance:
 * SSI0-4 and SSI9 expose 8 BUSIFs each, SSI5-8 expose a single BUSIF.
 */
#define RCAR_SSIU_BUSIF_ERR_MASK_WIDE   0x00FF00FFU /* SSI0-4, SSI9 */
#define RCAR_SSIU_BUSIF_ERR_MASK_NARROW 0x00010001U /* SSI5-8       */

/* SSIU_AUDIO_CLK_SEL.ACLK_SEL clock source */
enum i2s_rcar_ssi_clock_select {
	I2S_RCAR_CLKSEL_DIVCLK = 0,
	I2S_RCAR_CLKSEL_BRGA = 1,
	I2S_RCAR_CLKSEL_BRGB = 2,
	I2S_RCAR_CLKSEL_AVBCOUNTER8 = 3,
};

/* Frame format the SSI is configured for, derived from the i2s_config data format */
enum i2s_rcar_ssi_format {
	I2S_RCAR_FORMAT_STEREO,
	I2S_RCAR_FORMAT_MONAURAL,
	I2S_RCAR_FORMAT_TDM,
};

/* SSICR.DWL values in stereo/TDM format */
enum i2s_rcar_ssi_dwl_stereo {
	I2S_RCAR_SSICR_DWL_STEREO_8,
	I2S_RCAR_SSICR_DWL_STEREO_16,
	I2S_RCAR_SSICR_DWL_STEREO_18,
	I2S_RCAR_SSICR_DWL_STEREO_20,
	I2S_RCAR_SSICR_DWL_STEREO_22,
	I2S_RCAR_SSICR_DWL_STEREO_24,
	I2S_RCAR_SSICR_DWL_STEREO_32,
};

/* SSICR.DWL values in monaural format */
enum i2s_rcar_ssi_dwl_mono {
	I2S_RCAR_SSICR_DWL_MONO_8,
	I2S_RCAR_SSICR_DWL_MONO_16,
};

/* SSICR.SWL values in stereo/TDM format */
enum i2s_rcar_ssi_swl_stereo {
	I2S_RCAR_SSICR_SWL_STEREO_8,
	I2S_RCAR_SSICR_SWL_STEREO_16,
	I2S_RCAR_SSICR_SWL_STEREO_24,
	I2S_RCAR_SSICR_SWL_STEREO_32,
	I2S_RCAR_SSICR_SWL_STEREO_48,
	I2S_RCAR_SSICR_SWL_STEREO_64,
	I2S_RCAR_SSICR_SWL_STEREO_128,
	I2S_RCAR_SSICR_SWL_STEREO_256,
};

/* SSICR.SWL values in monaural format */
enum i2s_rcar_ssi_swl_mono {
	I2S_RCAR_SSICR_SWL_MONO_16,
	I2S_RCAR_SSICR_SWL_MONO_32,
	I2S_RCAR_SSICR_SWL_MONO_48,
	I2S_RCAR_SSICR_SWL_MONO_64,
	I2S_RCAR_SSICR_SWL_MONO_96,
	I2S_RCAR_SSICR_SWL_MONO_128,
	I2S_RCAR_SSICR_SWL_MONO_256,
	I2S_RCAR_SSICR_SWL_MONO_512,
};

/* SSIU BUSIF_ADINR.OTBL values, encoded as (24 - output bit length) */
enum i2s_rcar_ssiu_busif_bit_len {
	I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN8 = 0x10,
	I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN16 = 0x8,
	I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN18 = 0x6,
	I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN20 = 0x4,
	I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN22 = 0x2,
	I2S_RCAR_SSIU_BUSIF_ADINR_BIT_LEN24 = 0x0,
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2S_RENESAS_RCAR_SSI_H */
