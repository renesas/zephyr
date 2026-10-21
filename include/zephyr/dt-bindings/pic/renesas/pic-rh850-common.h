/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Devicetree macros for the RH850 PIC (Peripheral Interconnect Controller).
 *
 * Each PIC property entry is encoded as a 32-bit cell. Use one of the following macros
 *   RH850_PIC_ADC_TRIG(adc, sg, edge, src)     adc-trigger-selection
 *   RH850_PIC_TRIWAVE(unit, phases, level)     triangle-wave-pwm
 *   RH850_PIC_TAUD_INT(unit, ch)               taud-int-signal
 *
 * The remaining definitions describe the encoded cell layout and argument values
 * used by the driver to decode each property entry. Naming:
 *   RH850_PIC_CELL_TAG_*      identifies the property entry type
 *   RH850_PIC_<CELL>_*_POS    least-significant bit position of a field
 *   RH850_PIC_ARG_<ARG>_*     encoded value of a public macro argument
 *   RH850_PIC_SRC_ENC*        internal encoding of an ADC trigger source
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PIC_RENESAS_PIC_RH850_COMMON_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PIC_RENESAS_PIC_RH850_COMMON_H_

/* Common cell layout. Bit 31 is reserved and remains unused. */

/* Bits [27..24]: entry type tag used to validate the Devicetree property. */
#define RH850_PIC_CELL_TAG_POS      (24U)
#define RH850_PIC_CELL_TAG_MASK     (0xFU)
#define RH850_PIC_CELL_TAG_ADC_TRIG (1U) /* cell made by RH850_PIC_ADC_TRIG */
#define RH850_PIC_CELL_TAG_TRIWAVE  (2U) /* cell made by RH850_PIC_TRIWAVE */
#define RH850_PIC_CELL_TAG_TAUD_INT (3U) /* cell made by RH850_PIC_TAUD_INT */

/* adc-trigger-selection cell layout */
/* Bits [1..0]: ADC unit index (0..3). */
#define RH850_PIC_ADC_TRIG_ADC_POS (0U)
/* Bits [4..2]: scan group index (0..4); 5 selects the diagnostic group. */
#define RH850_PIC_ADC_TRIG_SG_POS (2U)
/* Bits [6..5]: trigger edge (0 rising, 1 falling, 2 both). */
#define RH850_PIC_ADC_TRIG_EDGE_POS (5U)
/* Bit [7]: internal source group selector. */
#define RH850_PIC_ADC_TRIG_REG_POS (7U)
/* Bits [12..8]: source index within the selected source group. */
#define RH850_PIC_ADC_TRIG_SRC_BIT_POS (8U)
/* Bit [13]: indicates that the source is a TAUD channel. */
#define RH850_PIC_ADC_TRIG_SRC_IS_TAUD_POS (13U)
/* Bits [15..14]: TAUD unit index (0..3) for a TAUD source. */
#define RH850_PIC_ADC_TRIG_SRC_TAUD_UNIT_POS (14U)
/* Bits [19..16]: TAUD channel index (0..15) for a TAUD source. */
#define RH850_PIC_ADC_TRIG_SRC_TAUD_CH_POS (16U)

/* adc-trigger-selection argument values */
/* adc: ADC unit index */
#define RH850_PIC_ARG_ADC_ADC0 (0U)
#define RH850_PIC_ARG_ADC_ADC1 (1U)
#define RH850_PIC_ARG_ADC_ADC2 (2U)
#define RH850_PIC_ARG_ADC_ADC3 (3U)

/* sg: scan group index; DIAG is the diagnostic scan group (only on MCUs that have one) */
#define RH850_PIC_ARG_SG_SG0  (0U)
#define RH850_PIC_ARG_SG_SG1  (1U)
#define RH850_PIC_ARG_SG_SG2  (2U)
#define RH850_PIC_ARG_SG_SG3  (3U)
#define RH850_PIC_ARG_SG_SG4  (4U)
#define RH850_PIC_ARG_SG_DIAG (5U)

/* edge: trigger edge selection. */
#define RH850_PIC_ARG_EDGE_RISING  (0U)
#define RH850_PIC_ARG_EDGE_FALLING (1U)
#define RH850_PIC_ARG_EDGE_BOTH    (2U)

/*
 * Internal encoding for a non-TAUD trigger source. The encoded value identifies
 * the source group and the source index within that group.
 */
#define RH850_PIC_SRC_ENC(reg, bit)                                                                \
	(((reg) << RH850_PIC_ADC_TRIG_REG_POS) | ((bit) << RH850_PIC_ADC_TRIG_SRC_BIT_POS))

/*
 * Internal encoding for a TAUD channel source. The TAUD unit and channel are
 * retained in the cell so the driver can configure the complete trigger route.
 */
#define RH850_PIC_SRC_ENC_TAUD(unit, ch)                                                           \
	(RH850_PIC_SRC_ENC((unit) >> 1, (unit) & 1) | (1U << RH850_PIC_ADC_TRIG_SRC_IS_TAUD_POS) | \
	 ((unit) << RH850_PIC_ADC_TRIG_SRC_TAUD_UNIT_POS) |                                        \
	 ((ch) << RH850_PIC_ADC_TRIG_SRC_TAUD_CH_POS))

/**
 * One ADC trigger route: scan group `sg` of ADC `adc` starts on `edge` of source `src`.
 *
 * Example: RH850_PIC_ADC_TRIG(ADC0, SG0, RISING, TAUD0_CH2)
 */
#define RH850_PIC_ADC_TRIG(adc, sg, edge, src)                                                     \
	((RH850_PIC_CELL_TAG_ADC_TRIG << RH850_PIC_CELL_TAG_POS) |                                 \
	 (RH850_PIC_ARG_ADC_##adc << RH850_PIC_ADC_TRIG_ADC_POS) |                                 \
	 (RH850_PIC_ARG_SG_##sg << RH850_PIC_ADC_TRIG_SG_POS) |                                    \
	 (RH850_PIC_ARG_EDGE_##edge << RH850_PIC_ADC_TRIG_EDGE_POS) | RH850_PIC_ARG_SRC_##src)

/* triangle-wave-pwm cell layout */
/* Bits 1:0: TAUD unit index. */
#define RH850_PIC_TRIWAVE_UNIT_POS (0U)
/* Bits 4:2: phase mask (U = 1, V = 2, W = 4). */
#define RH850_PIC_TRIWAVE_PHASES_POS (2U)
/* Bits 6:5: active level (0 none, 1 high, 2 low). */
#define RH850_PIC_TRIWAVE_LEVEL_POS (5U)

/* triangle-wave-pwm argument values */
/* unit: TAUD unit index (also used by taud-int-signal) */
#define RH850_PIC_ARG_UNIT_TAUD0 (0U)
#define RH850_PIC_ARG_UNIT_TAUD1 (1U)
#define RH850_PIC_ARG_UNIT_TAUD2 (2U)
#define RH850_PIC_ARG_UNIT_TAUD3 (3U)

/* phases: mask of the phases U, V, W (bits 0, 1, 2) */
#define RH850_PIC_ARG_PHASE_U   (1U)
#define RH850_PIC_ARG_PHASE_V   (2U)
#define RH850_PIC_ARG_PHASE_W   (4U)
#define RH850_PIC_ARG_PHASE_UV  (3U)
#define RH850_PIC_ARG_PHASE_UW  (5U)
#define RH850_PIC_ARG_PHASE_VW  (6U)
#define RH850_PIC_ARG_PHASE_UVW (7U)

/* level: active level of the selected phase outputs. */
#define RH850_PIC_ARG_LEVEL_NONE (0U)
#define RH850_PIC_ARG_LEVEL_HIGH (1U)
#define RH850_PIC_ARG_LEVEL_LOW  (2U)

/**
 * Enable the phases `phases` of TAUD unit `unit` with active level `level`.
 *
 * Example: RH850_PIC_TRIWAVE(TAUD0, UVW, HIGH)
 */
#define RH850_PIC_TRIWAVE(unit, phases, level)                                                     \
	((RH850_PIC_CELL_TAG_TRIWAVE << RH850_PIC_CELL_TAG_POS) |                                  \
	 (RH850_PIC_ARG_UNIT_##unit << RH850_PIC_TRIWAVE_UNIT_POS) |                               \
	 (RH850_PIC_ARG_PHASE_##phases << RH850_PIC_TRIWAVE_PHASES_POS) |                          \
	 (RH850_PIC_ARG_LEVEL_##level << RH850_PIC_TRIWAVE_LEVEL_POS))

/* taud-int-signal cell layout */
/* Bits 1:0: TAUD unit index. */
#define RH850_PIC_TAUD_INT_UNIT_POS (0U)
/* Bits 3:2: channel selection code (1 CH0, 2 CH2, 3 CH8). */
#define RH850_PIC_TAUD_INT_CH_POS (2U)

/* taud-int-signal cell: values of the macro arguments (unit: see RH850_PIC_ARG_UNIT_*) */
/* ch: TAUD channel used as the INT signal output. */
#define RH850_PIC_ARG_INTCH_CH0 (1U)
#define RH850_PIC_ARG_INTCH_CH2 (2U)
#define RH850_PIC_ARG_INTCH_CH8 (3U)

/**
 * Use channel `ch` of TAUD unit `unit` as INT signal output.
 *
 * Example: RH850_PIC_TAUD_INT(TAUD0, CH2)
 */
#define RH850_PIC_TAUD_INT(unit, ch)                                                               \
	((RH850_PIC_CELL_TAG_TAUD_INT << RH850_PIC_CELL_TAG_POS) |                                 \
	 (RH850_PIC_ARG_UNIT_##unit << RH850_PIC_TAUD_INT_UNIT_POS) |                              \
	 (RH850_PIC_ARG_INTCH_##ch << RH850_PIC_TAUD_INT_CH_POS))

/* ADC trigger sources: values of the macro argument `src` */
/* TAUD channels: RH850_PIC_ARG_SRC_TAUD<unit>_CH<channel> */
#define RH850_PIC_ARG_SRC_TAUD0_CH0 RH850_PIC_SRC_ENC_TAUD(0U, 0U)
#define RH850_PIC_ARG_SRC_TAUD0_CH1 RH850_PIC_SRC_ENC_TAUD(0U, 1U)
#define RH850_PIC_ARG_SRC_TAUD0_CH2 RH850_PIC_SRC_ENC_TAUD(0U, 2U)
#define RH850_PIC_ARG_SRC_TAUD0_CH3 RH850_PIC_SRC_ENC_TAUD(0U, 3U)
#define RH850_PIC_ARG_SRC_TAUD0_CH4 RH850_PIC_SRC_ENC_TAUD(0U, 4U)
#define RH850_PIC_ARG_SRC_TAUD0_CH5 RH850_PIC_SRC_ENC_TAUD(0U, 5U)
#define RH850_PIC_ARG_SRC_TAUD0_CH6 RH850_PIC_SRC_ENC_TAUD(0U, 6U)
#define RH850_PIC_ARG_SRC_TAUD0_CH7 RH850_PIC_SRC_ENC_TAUD(0U, 7U)
#define RH850_PIC_ARG_SRC_TAUD0_CH8 RH850_PIC_SRC_ENC_TAUD(0U, 8U)
#define RH850_PIC_ARG_SRC_TAUD0_CH9 RH850_PIC_SRC_ENC_TAUD(0U, 9U)
#define RH850_PIC_ARG_SRC_TAUD0_CH10 RH850_PIC_SRC_ENC_TAUD(0U, 10U)
#define RH850_PIC_ARG_SRC_TAUD0_CH11 RH850_PIC_SRC_ENC_TAUD(0U, 11U)
#define RH850_PIC_ARG_SRC_TAUD0_CH12 RH850_PIC_SRC_ENC_TAUD(0U, 12U)
#define RH850_PIC_ARG_SRC_TAUD0_CH13 RH850_PIC_SRC_ENC_TAUD(0U, 13U)
#define RH850_PIC_ARG_SRC_TAUD0_CH14 RH850_PIC_SRC_ENC_TAUD(0U, 14U)
#define RH850_PIC_ARG_SRC_TAUD0_CH15 RH850_PIC_SRC_ENC_TAUD(0U, 15U)
#define RH850_PIC_ARG_SRC_TAUD1_CH0 RH850_PIC_SRC_ENC_TAUD(1U, 0U)
#define RH850_PIC_ARG_SRC_TAUD1_CH1 RH850_PIC_SRC_ENC_TAUD(1U, 1U)
#define RH850_PIC_ARG_SRC_TAUD1_CH2 RH850_PIC_SRC_ENC_TAUD(1U, 2U)
#define RH850_PIC_ARG_SRC_TAUD1_CH3 RH850_PIC_SRC_ENC_TAUD(1U, 3U)
#define RH850_PIC_ARG_SRC_TAUD1_CH4 RH850_PIC_SRC_ENC_TAUD(1U, 4U)
#define RH850_PIC_ARG_SRC_TAUD1_CH5 RH850_PIC_SRC_ENC_TAUD(1U, 5U)
#define RH850_PIC_ARG_SRC_TAUD1_CH6 RH850_PIC_SRC_ENC_TAUD(1U, 6U)
#define RH850_PIC_ARG_SRC_TAUD1_CH7 RH850_PIC_SRC_ENC_TAUD(1U, 7U)
#define RH850_PIC_ARG_SRC_TAUD1_CH8 RH850_PIC_SRC_ENC_TAUD(1U, 8U)
#define RH850_PIC_ARG_SRC_TAUD1_CH9 RH850_PIC_SRC_ENC_TAUD(1U, 9U)
#define RH850_PIC_ARG_SRC_TAUD1_CH10 RH850_PIC_SRC_ENC_TAUD(1U, 10U)
#define RH850_PIC_ARG_SRC_TAUD1_CH11 RH850_PIC_SRC_ENC_TAUD(1U, 11U)
#define RH850_PIC_ARG_SRC_TAUD1_CH12 RH850_PIC_SRC_ENC_TAUD(1U, 12U)
#define RH850_PIC_ARG_SRC_TAUD1_CH13 RH850_PIC_SRC_ENC_TAUD(1U, 13U)
#define RH850_PIC_ARG_SRC_TAUD1_CH14 RH850_PIC_SRC_ENC_TAUD(1U, 14U)
#define RH850_PIC_ARG_SRC_TAUD1_CH15 RH850_PIC_SRC_ENC_TAUD(1U, 15U)
#define RH850_PIC_ARG_SRC_TAUD2_CH0 RH850_PIC_SRC_ENC_TAUD(2U, 0U)
#define RH850_PIC_ARG_SRC_TAUD2_CH1 RH850_PIC_SRC_ENC_TAUD(2U, 1U)
#define RH850_PIC_ARG_SRC_TAUD2_CH2 RH850_PIC_SRC_ENC_TAUD(2U, 2U)
#define RH850_PIC_ARG_SRC_TAUD2_CH3 RH850_PIC_SRC_ENC_TAUD(2U, 3U)
#define RH850_PIC_ARG_SRC_TAUD2_CH4 RH850_PIC_SRC_ENC_TAUD(2U, 4U)
#define RH850_PIC_ARG_SRC_TAUD2_CH5 RH850_PIC_SRC_ENC_TAUD(2U, 5U)
#define RH850_PIC_ARG_SRC_TAUD2_CH6 RH850_PIC_SRC_ENC_TAUD(2U, 6U)
#define RH850_PIC_ARG_SRC_TAUD2_CH7 RH850_PIC_SRC_ENC_TAUD(2U, 7U)
#define RH850_PIC_ARG_SRC_TAUD2_CH8 RH850_PIC_SRC_ENC_TAUD(2U, 8U)
#define RH850_PIC_ARG_SRC_TAUD2_CH9 RH850_PIC_SRC_ENC_TAUD(2U, 9U)
#define RH850_PIC_ARG_SRC_TAUD2_CH10 RH850_PIC_SRC_ENC_TAUD(2U, 10U)
#define RH850_PIC_ARG_SRC_TAUD2_CH11 RH850_PIC_SRC_ENC_TAUD(2U, 11U)
#define RH850_PIC_ARG_SRC_TAUD2_CH12 RH850_PIC_SRC_ENC_TAUD(2U, 12U)
#define RH850_PIC_ARG_SRC_TAUD2_CH13 RH850_PIC_SRC_ENC_TAUD(2U, 13U)
#define RH850_PIC_ARG_SRC_TAUD2_CH14 RH850_PIC_SRC_ENC_TAUD(2U, 14U)
#define RH850_PIC_ARG_SRC_TAUD2_CH15 RH850_PIC_SRC_ENC_TAUD(2U, 15U)
#define RH850_PIC_ARG_SRC_TAUD3_CH0 RH850_PIC_SRC_ENC_TAUD(3U, 0U)
#define RH850_PIC_ARG_SRC_TAUD3_CH1 RH850_PIC_SRC_ENC_TAUD(3U, 1U)
#define RH850_PIC_ARG_SRC_TAUD3_CH2 RH850_PIC_SRC_ENC_TAUD(3U, 2U)
#define RH850_PIC_ARG_SRC_TAUD3_CH3 RH850_PIC_SRC_ENC_TAUD(3U, 3U)
#define RH850_PIC_ARG_SRC_TAUD3_CH4 RH850_PIC_SRC_ENC_TAUD(3U, 4U)
#define RH850_PIC_ARG_SRC_TAUD3_CH5 RH850_PIC_SRC_ENC_TAUD(3U, 5U)
#define RH850_PIC_ARG_SRC_TAUD3_CH6 RH850_PIC_SRC_ENC_TAUD(3U, 6U)
#define RH850_PIC_ARG_SRC_TAUD3_CH7 RH850_PIC_SRC_ENC_TAUD(3U, 7U)
#define RH850_PIC_ARG_SRC_TAUD3_CH8 RH850_PIC_SRC_ENC_TAUD(3U, 8U)
#define RH850_PIC_ARG_SRC_TAUD3_CH9 RH850_PIC_SRC_ENC_TAUD(3U, 9U)
#define RH850_PIC_ARG_SRC_TAUD3_CH10 RH850_PIC_SRC_ENC_TAUD(3U, 10U)
#define RH850_PIC_ARG_SRC_TAUD3_CH11 RH850_PIC_SRC_ENC_TAUD(3U, 11U)
#define RH850_PIC_ARG_SRC_TAUD3_CH12 RH850_PIC_SRC_ENC_TAUD(3U, 12U)
#define RH850_PIC_ARG_SRC_TAUD3_CH13 RH850_PIC_SRC_ENC_TAUD(3U, 13U)
#define RH850_PIC_ARG_SRC_TAUD3_CH14 RH850_PIC_SRC_ENC_TAUD(3U, 14U)
#define RH850_PIC_ARG_SRC_TAUD3_CH15 RH850_PIC_SRC_ENC_TAUD(3U, 15U)

/* Other ADC trigger sources encoded by internal source group and source index. */
#define RH850_PIC_ARG_SRC_INTTAUJ0I3 RH850_PIC_SRC_ENC(0U, 3U)
#define RH850_PIC_ARG_SRC_INTTAUJ1I3 RH850_PIC_SRC_ENC(0U, 4U)
#define RH850_PIC_ARG_SRC_ADCTRG0 RH850_PIC_SRC_ENC(0U, 9U)
#define RH850_PIC_ARG_SRC_ADCTRG1 RH850_PIC_SRC_ENC(0U, 10U)
#define RH850_PIC_ARG_SRC_ADCTRG2 RH850_PIC_SRC_ENC(0U, 11U)
#define RH850_PIC_ARG_SRC_ADCTRG3 RH850_PIC_SRC_ENC(0U, 12U)
#define RH850_PIC_ARG_SRC_ADCTRG4 RH850_PIC_SRC_ENC(0U, 13U)
#define RH850_PIC_ARG_SRC_TSG30TRG0 RH850_PIC_SRC_ENC(0U, 5U)
#define RH850_PIC_ARG_SRC_TSG30TRG1 RH850_PIC_SRC_ENC(0U, 6U)
#define RH850_PIC_ARG_SRC_TSG31TRG0 RH850_PIC_SRC_ENC(0U, 7U)
#define RH850_PIC_ARG_SRC_TSG31TRG1 RH850_PIC_SRC_ENC(0U, 8U)
#define RH850_PIC_ARG_SRC_INTTSG30I3 RH850_PIC_SRC_ENC(0U, 14U)
#define RH850_PIC_ARG_SRC_INTTSG30I4 RH850_PIC_SRC_ENC(0U, 15U)
#define RH850_PIC_ARG_SRC_INTTSG30I7 RH850_PIC_SRC_ENC(0U, 16U)
#define RH850_PIC_ARG_SRC_INTTSG30I8 RH850_PIC_SRC_ENC(0U, 17U)
#define RH850_PIC_ARG_SRC_INTTSG30I11 RH850_PIC_SRC_ENC(0U, 18U)
#define RH850_PIC_ARG_SRC_INTTSG30I12 RH850_PIC_SRC_ENC(0U, 19U)
#define RH850_PIC_ARG_SRC_INTTSG31I3 RH850_PIC_SRC_ENC(0U, 22U)
#define RH850_PIC_ARG_SRC_INTTSG31I4 RH850_PIC_SRC_ENC(0U, 23U)
#define RH850_PIC_ARG_SRC_INTTSG31I7 RH850_PIC_SRC_ENC(0U, 24U)
#define RH850_PIC_ARG_SRC_INTTSG31I8 RH850_PIC_SRC_ENC(0U, 25U)
#define RH850_PIC_ARG_SRC_INTTSG31I11 RH850_PIC_SRC_ENC(0U, 26U)
#define RH850_PIC_ARG_SRC_INTTSG31I12 RH850_PIC_SRC_ENC(0U, 27U)
#define RH850_PIC_ARG_SRC_PIC20_TAPA0TADOUT0 RH850_PIC_SRC_ENC(0U, 20U)
#define RH850_PIC_ARG_SRC_PIC20_TAPA0TADOUT1 RH850_PIC_SRC_ENC(0U, 21U)
#define RH850_PIC_ARG_SRC_ENCA RH850_PIC_SRC_ENC(0U, 28U)
#define RH850_PIC_ARG_SRC_GTM_ATU RH850_PIC_SRC_ENC(0U, 2U)
#define RH850_PIC_ARG_SRC_TSG32ADTRG0 RH850_PIC_SRC_ENC(1U, 5U)
#define RH850_PIC_ARG_SRC_TSG32ADTRG1 RH850_PIC_SRC_ENC(1U, 6U)
#define RH850_PIC_ARG_SRC_INTTSG32I3 RH850_PIC_SRC_ENC(1U, 14U)
#define RH850_PIC_ARG_SRC_INTTSG32I4 RH850_PIC_SRC_ENC(1U, 15U)
#define RH850_PIC_ARG_SRC_INTTSG32I7 RH850_PIC_SRC_ENC(1U, 16U)
#define RH850_PIC_ARG_SRC_INTTSG32I8 RH850_PIC_SRC_ENC(1U, 17U)
#define RH850_PIC_ARG_SRC_INTTSG32I11 RH850_PIC_SRC_ENC(1U, 18U)
#define RH850_PIC_ARG_SRC_INTTSG32I12 RH850_PIC_SRC_ENC(1U, 19U)
#define RH850_PIC_ARG_SRC_PIC21_TAPA0TADOUT0 RH850_PIC_SRC_ENC(1U, 7U)
#define RH850_PIC_ARG_SRC_PIC21_TAPA0TADOUT1 RH850_PIC_SRC_ENC(1U, 8U)
#define RH850_PIC_ARG_SRC_TAPA1TADOUT0 RH850_PIC_SRC_ENC(1U, 9U)
#define RH850_PIC_ARG_SRC_TAPA1TADOUT1 RH850_PIC_SRC_ENC(1U, 10U)
#define RH850_PIC_ARG_SRC_TAPA2TADOUT0 RH850_PIC_SRC_ENC(1U, 11U)
#define RH850_PIC_ARG_SRC_TAPA2TADOUT1 RH850_PIC_SRC_ENC(1U, 12U)
#define RH850_PIC_ARG_SRC_RDC3AS0_ET RH850_PIC_SRC_ENC(1U, 29U)
#define RH850_PIC_ARG_SRC_RDC3AS0_CMP0 RH850_PIC_SRC_ENC(1U, 28U)
#define RH850_PIC_ARG_SRC_RDC3AS1_ET RH850_PIC_SRC_ENC(1U, 31U)
#define RH850_PIC_ARG_SRC_RDC3AS1_CMP0 RH850_PIC_SRC_ENC(1U, 30U)
#define RH850_PIC_ARG_SRC_RDC3AL0_ET RH850_PIC_SRC_ENC(1U, 25U)
#define RH850_PIC_ARG_SRC_RDC3AL0_CMP0 RH850_PIC_SRC_ENC(1U, 24U)
#define RH850_PIC_ARG_SRC_RDC3AL1_ET RH850_PIC_SRC_ENC(1U, 27U)
#define RH850_PIC_ARG_SRC_RDC3AL1_CMP0 RH850_PIC_SRC_ENC(1U, 26U)
#define RH850_PIC_ARG_SRC_EMU3S0_ADTRG0 RH850_PIC_SRC_ENC(1U, 20U)
#define RH850_PIC_ARG_SRC_EMU3S0_ADTRG1 RH850_PIC_SRC_ENC(1U, 21U)
#define RH850_PIC_ARG_SRC_EMU3S1_ADTRG0 RH850_PIC_SRC_ENC(1U, 22U)
#define RH850_PIC_ARG_SRC_EMU3S1_ADTRG1 RH850_PIC_SRC_ENC(1U, 23U)
#define RH850_PIC_ARG_SRC_ATU RH850_PIC_SRC_ENC(1U, 2U)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PIC_RENESAS_PIC_RH850_COMMON_H_ */
