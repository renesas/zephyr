/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Build-time configuration for the RH850 PIC driver.
 *
 * This file converts the enabled PIC Devicetree node into the configuration
 * objects used by the driver. It:
 *
 * 1. Decodes the property entries defined by the RH850 PIC dt-bindings.
 * 2. Combines the entries into the required configuration fields.
 * 3. Validates the configuration against the capabilities of the selected MCU.
 * 4. Defines the static configuration objects passed to R_PIC_Open().
 *
 * This file is included once by pic_renesas_rh850.c. Build-time validation is
 * performed before the configuration initializers so configuration errors are
 * reported clearly.
 *
 * Internal macro conventions:
 *
 * CELL_*  Extracts a field from one encoded Devicetree property entry.
 * *_TERM  Produces the contribution of one entry to a configuration value.
 * *_VALUE Combines the contributions of all matching entries.
 * *_INIT  Produces part of a configuration initializer.
 *
 * A macro suffixed with UNITn handles an MCU-specific peripheral instance and
 * expands to nothing when that instance is not supported.
 */

#ifndef ZEPHYR_SOC_RENESAS_RH850_COMMON_FSP_PIC_PIC_RENESAS_RH850_CFG_H_
#define ZEPHYR_SOC_RENESAS_RH850_COMMON_FSP_PIC_PIC_RENESAS_RH850_CFG_H_

#include <stdint.h>
#include <stdbool.h>

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>
#include <zephyr/dt-bindings/pic/renesas/pic-rh850-common.h>

#include <r_pic.h>

/*
 * The FSP defines the feature macros below only on the MCUs that have the feature. Resolve them
 * once here so that they can be used in C expressions as well as in preprocessor conditions.
 */

/* FSP defines BSP_FEATURE_PIC_ADCJ_TRIG_SEL only on MCUs that have ADCJ (no PIC21). */
#if defined(BSP_FEATURE_PIC_ADCJ_TRIG_SEL) && (BSP_FEATURE_PIC_ADCJ_TRIG_SEL != 0)
#define RH850_PIC_HAS_PIC21 0
#else
#define RH850_PIC_HAS_PIC21 1
#endif

/* Non-zero when the INT signal output of TAUD1 exists on this MCU. */
#if defined(BSP_FEATURE_PIC_INT_SIG_OUT_TAUD1) && (BSP_FEATURE_PIC_INT_SIG_OUT_TAUD1 != 0)
#define RH850_PIC_HAS_INT_SIG_OUT_TAUD1 1
#else
#define RH850_PIC_HAS_INT_SIG_OUT_TAUD1 0
#endif

/* Non-zero when the INT signal output of TAUD2 exists on this MCU. */
#if defined(BSP_FEATURE_PIC_INT_SIG_OUT_TAUD2) && (BSP_FEATURE_PIC_INT_SIG_OUT_TAUD2 != 0)
#define RH850_PIC_HAS_INT_SIG_OUT_TAUD2 1
#else
#define RH850_PIC_HAS_INT_SIG_OUT_TAUD2 0
#endif

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(renesas_rh850_pic) <= 1,
	     "pic: only one renesas,rh850-pic node may be okay");

#if DT_HAS_COMPAT_STATUS_OKAY(renesas_rh850_pic)

/* The enabled PIC node. */
#define RH850_PIC_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(renesas_rh850_pic)

/* Add the trailing semicolon required by a generated list of assertions. */
#define RH850_PIC_ASSERT(cond, msg) BUILD_ASSERT(cond, msg);

/* =================================================================================================
 * 1. Decode property entries
 * =================================================================================================
 */

/*
 * Extract a field from encoded cell c. pos is the least-significant bit
 * position, and mask is the unshifted field mask.
 */
#define RH850_PIC_CELL_FIELD(c, pos, mask) ((((uint32_t)(c)) >> (pos)) & (mask))

/* Return the entry type tag used to validate the Devicetree property. */
#define RH850_PIC_CELL_TAG(c) \
	RH850_PIC_CELL_FIELD(c, RH850_PIC_CELL_TAG_POS, RH850_PIC_CELL_TAG_MASK)

/* Fields of an adc-trigger-selection entry. */
/* ADC unit index. */
#define RH850_PIC_CELL_ADC_UNIT(c) RH850_PIC_CELL_FIELD(c, RH850_PIC_ADC_TRIG_ADC_POS, 0x3U)
/* ADC scan group index; 5 selects the diagnostic group. */
#define RH850_PIC_CELL_ADC_SG(c) RH850_PIC_CELL_FIELD(c, RH850_PIC_ADC_TRIG_SG_POS, 0x7U)
/* Trigger edge selection. */
#define RH850_PIC_CELL_ADC_EDGE(c) RH850_PIC_CELL_FIELD(c, RH850_PIC_ADC_TRIG_EDGE_POS, 0x3U)
/* Internal source group selector. */
#define RH850_PIC_CELL_ADC_REG(c) RH850_PIC_CELL_FIELD(c, RH850_PIC_ADC_TRIG_REG_POS, 0x1U)
/* Source index within the selected source group. */
#define RH850_PIC_CELL_ADC_SRC_BIT(c) \
	RH850_PIC_CELL_FIELD(c, RH850_PIC_ADC_TRIG_SRC_BIT_POS, 0x1FU)
/* Return non-zero when the trigger source is a TAUD channel. */
#define RH850_PIC_CELL_ADC_SRC_IS_TAUD(c) \
	RH850_PIC_CELL_FIELD(c, RH850_PIC_ADC_TRIG_SRC_IS_TAUD_POS, 0x1U)
/* TAUD unit index of a TAUD trigger source. */
#define RH850_PIC_CELL_ADC_SRC_TAUD_UNIT(c) \
	RH850_PIC_CELL_FIELD(c, RH850_PIC_ADC_TRIG_SRC_TAUD_UNIT_POS, 0x3U)
/* TAUD channel index of a TAUD trigger source. */
#define RH850_PIC_CELL_ADC_SRC_TAUD_CH(c) \
	RH850_PIC_CELL_FIELD(c, RH850_PIC_ADC_TRIG_SRC_TAUD_CH_POS, 0xFU)

/* Fields of a triangle-wave-pwm entry. */
/* TAUD unit index. */
#define RH850_PIC_CELL_TRIWAVE_UNIT(c) RH850_PIC_CELL_FIELD(c, RH850_PIC_TRIWAVE_UNIT_POS, 0x3U)
/* Mask of the selected U, V and W phases. */
#define RH850_PIC_CELL_TRIWAVE_PHASES(c) RH850_PIC_CELL_FIELD(c, RH850_PIC_TRIWAVE_PHASES_POS, 0x7U)
/* Active level of the selected phase outputs. */
#define RH850_PIC_CELL_TRIWAVE_LEVEL(c) RH850_PIC_CELL_FIELD(c, RH850_PIC_TRIWAVE_LEVEL_POS, 0x3U)

/* Fields of a taud-int-signal entry. */
/* TAUD unit index. */
#define RH850_PIC_CELL_TAUD_INT_UNIT(c) RH850_PIC_CELL_FIELD(c, RH850_PIC_TAUD_INT_UNIT_POS, 0x3U)
/* Selected TAUD channel code for the INT signal output. */
#define RH850_PIC_CELL_TAUD_INT_CH(c) RH850_PIC_CELL_FIELD(c, RH850_PIC_TAUD_INT_CH_POS, 0x3U)

/* Return entry i of each PIC list property. */
#define RH850_PIC_ADC_CELL(i)      DT_PROP_BY_IDX(RH850_PIC_NODE, adc_trigger_selection, i)
#define RH850_PIC_TRIWAVE_CELL(i)  DT_PROP_BY_IDX(RH850_PIC_NODE, triangle_wave_pwm, i)
#define RH850_PIC_TAUD_INT_CELL(i) DT_PROP_BY_IDX(RH850_PIC_NODE, taud_int_signal, i)

/*
 * Apply fn to each encoded entry of property prop. If the property is absent,
 * expand to an empty sequence so the combined configuration value remains zero.
 */
#define RH850_PIC_FOREACH_CELL(prop, fn, ...)                                                      \
	COND_CODE_1(DT_NODE_HAS_PROP(RH850_PIC_NODE, prop),                                        \
		    (DT_FOREACH_PROP_ELEM_VARGS(RH850_PIC_NODE, prop, fn, __VA_ARGS__)), ())
/* Apply fn to each property entry without additional arguments. */
#define RH850_PIC_FOREACH_CELL_NOARG(prop, fn)                                                     \
	COND_CODE_1(DT_NODE_HAS_PROP(RH850_PIC_NODE, prop),                                        \
		    (DT_FOREACH_PROP_ELEM(RH850_PIC_NODE, prop, fn)), ())

/* =================================================================================================
 * 2. Combine property entries into configuration values
 * =================================================================================================
 */

/* ADC scan-group source selection */

/* Return the source bit contributed by entry i when it matches (a, s, r). */
#define RH850_PIC_ADC_SEL_TERM(n, p, i, a, s, r)                                                   \
	| ((RH850_PIC_CELL_ADC_UNIT(RH850_PIC_ADC_CELL(i)) == (a) &&                               \
	    RH850_PIC_CELL_ADC_SG(RH850_PIC_ADC_CELL(i)) == (s) &&                                 \
	    RH850_PIC_CELL_ADC_REG(RH850_PIC_ADC_CELL(i)) == (r))                                  \
		   ? (1UL << RH850_PIC_CELL_ADC_SRC_BIT(RH850_PIC_ADC_CELL(i)))                    \
		   : 0UL)
/* Combine matching entries into the source-selection mask for (a, s, r). */
#define RH850_PIC_ADC_SEL_VALUE(a, s, r)                                                           \
	((uint32_t)(0UL RH850_PIC_FOREACH_CELL(adc_trigger_selection,                              \
					       RH850_PIC_ADC_SEL_TERM, a, s, r)))

/* Return the trigger edge contributed by entry i when it matches (a, s). */
#define RH850_PIC_ADC_EDGE_TERM(n, p, i, a, s)                                                     \
	| ((RH850_PIC_CELL_ADC_UNIT(RH850_PIC_ADC_CELL(i)) == (a) &&                               \
	    RH850_PIC_CELL_ADC_SG(RH850_PIC_ADC_CELL(i)) == (s))                                   \
		   ? RH850_PIC_CELL_ADC_EDGE(RH850_PIC_ADC_CELL(i))                                \
		   : 0U)
/* Mark the edge code used by entry i for conflict detection. */
#define RH850_PIC_ADC_EDGE_SEEN_TERM(n, p, i, a, s)                                                \
	| ((RH850_PIC_CELL_ADC_UNIT(RH850_PIC_ADC_CELL(i)) == (a) &&                               \
	    RH850_PIC_CELL_ADC_SG(RH850_PIC_ADC_CELL(i)) == (s))                                   \
		   ? (1U << RH850_PIC_CELL_ADC_EDGE(RH850_PIC_ADC_CELL(i)))                        \
		   : 0U)
/* Combine the edge selections for scan group s of ADC unit a. */
#define RH850_PIC_ADC_EDGE_VALUE(a, s)                                                             \
	(0U RH850_PIC_FOREACH_CELL(adc_trigger_selection, RH850_PIC_ADC_EDGE_TERM, a, s))
/* Return the set of edge codes used by entries matching (a, s). */
#define RH850_PIC_ADC_EDGE_SEEN(a, s)                                                              \
	(0U RH850_PIC_FOREACH_CELL(adc_trigger_selection, RH850_PIC_ADC_EDGE_SEEN_TERM, a, s))

/*
 * Place the edge selection of scan group s in its configuration field. Include
 * the field only when the scan group has a trigger source in source group r.
 */
#define RH850_PIC_ADC_EDGE_FIELD(a, s, r)                                                          \
	((RH850_PIC_ADC_SEL_VALUE(a, s, r) != 0U)                                                  \
		 ? (uint16_t)(RH850_PIC_ADC_EDGE_VALUE(a, s) << (2U * (s)))                        \
		 : (uint16_t)0U)

/* Add diagnostic scan-group configuration only when supported by the MCU. */
#if (BSP_FEATURE_PIC_ADC_NUMBER_SG == 6)
/* Add the diagnostic scan-group source-selection initializer. */
#define RH850_PIC_ADC_SEL_DIAG_ELEM(a, r), [5] = RH850_PIC_ADC_SEL_VALUE(a, 5, r)
/* Add the diagnostic scan-group edge configuration. */
#define RH850_PIC_ADC_EDGE_DIAG_TERM(a, r) | RH850_PIC_ADC_EDGE_FIELD(a, 5, r)
/* Add the diagnostic scan-group TAUD channel mask. */
#define RH850_PIC_TAUD_CH_DIAG_ELEM(k), [5] = RH850_PIC_TAUD_CH_MASK(k, 5)
#else
#define RH850_PIC_ADC_SEL_DIAG_ELEM(a, r)
#define RH850_PIC_ADC_EDGE_DIAG_TERM(a, r)
#define RH850_PIC_TAUD_CH_DIAG_ELEM(k)
#endif

/* The array row {sg0, ..., sg4 [, diag]} of pic2x_adc_sg_trigger_sel[a] */
#define RH850_PIC_ADC_SEL_ROW(a, r)                                                                \
	{                                                                                          \
		[0] = RH850_PIC_ADC_SEL_VALUE(a, 0, r), [1] = RH850_PIC_ADC_SEL_VALUE(a, 1, r),    \
		[2] = RH850_PIC_ADC_SEL_VALUE(a, 2, r), [3] = RH850_PIC_ADC_SEL_VALUE(a, 3, r),    \
		[4] = RH850_PIC_ADC_SEL_VALUE(a, 4, r) RH850_PIC_ADC_SEL_DIAG_ELEM(a, r)           \
	}
/* The 16-bit word pic2x_adc_sg_trigger_edge[a]: the edge fields of all scan groups ORed */
#define RH850_PIC_ADC_EDGE_WORD(a, r)                                                              \
	((uint16_t)(RH850_PIC_ADC_EDGE_FIELD(a, 0, r) | RH850_PIC_ADC_EDGE_FIELD(a, 1, r) |        \
		    RH850_PIC_ADC_EDGE_FIELD(a, 2, r) | RH850_PIC_ADC_EDGE_FIELD(a, 3, r) |        \
		    RH850_PIC_ADC_EDGE_FIELD(a, 4, r) RH850_PIC_ADC_EDGE_DIAG_TERM(a, r)))

/* INIT of the PIC21 arrays of ADC unit a; PIC21 exists only on MCUs with ADCK */
#if (RH850_PIC_HAS_PIC21)
#define RH850_PIC_ADC_PIC21_INIT(a)                                                                \
	.pic21_adc_sg_trigger_sel[a] = RH850_PIC_ADC_SEL_ROW(a, 1),                                \
	.pic21_adc_sg_trigger_edge[a] = RH850_PIC_ADC_EDGE_WORD(a, 1),
#else
#define RH850_PIC_ADC_PIC21_INIT(a)
#endif
/* INIT of the level 1 arrays (PIC20, and PIC21 where it exists) of ADC unit a */
#define RH850_PIC_ADC_INIT_UNIT(a)                                                                 \
	.pic20_adc_sg_trigger_sel[a] = RH850_PIC_ADC_SEL_ROW(a, 0),                                \
	.pic20_adc_sg_trigger_edge[a] = RH850_PIC_ADC_EDGE_WORD(a, 0),                             \
	RH850_PIC_ADC_PIC21_INIT(a)

/* Only the ADC units of this MCU are written (the checks reject a cell for any other unit) */
#define RH850_PIC_ADC_INIT_UNIT0 RH850_PIC_ADC_INIT_UNIT(0)
#define RH850_PIC_ADC_INIT_UNIT1 RH850_PIC_ADC_INIT_UNIT(1)
#if (BSP_FEATURE_PIC_ADC_NUMBER_UNIT > 2)
#define RH850_PIC_ADC_INIT_UNIT2 RH850_PIC_ADC_INIT_UNIT(2)
#else
#define RH850_PIC_ADC_INIT_UNIT2
#endif
#if (BSP_FEATURE_PIC_ADC_NUMBER_UNIT > 3)
#define RH850_PIC_ADC_INIT_UNIT3 RH850_PIC_ADC_INIT_UNIT(3)
#else
#define RH850_PIC_ADC_INIT_UNIT3
#endif
/* INIT of the level 1 arrays of all ADC units of this MCU */
#define RH850_PIC_ADC_INIT_ALL                                                                     \
	RH850_PIC_ADC_INIT_UNIT0 RH850_PIC_ADC_INIT_UNIT1 RH850_PIC_ADC_INIT_UNIT2                 \
	RH850_PIC_ADC_INIT_UNIT3

/* TAUD channel selection for ADC triggers */
/*
 * Build the TAUD channel selection for an ADC trigger route. The resulting
 * channel mask is shared by all ADC units and therefore has no ADC-unit index.
 * The _OF_ADC variant returns the contribution of one ADC unit.
 */

/* Return the TAUD channel bit contributed by entry i when it matches (k, s). */
#define RH850_PIC_TAUD_CH_TERM(n, p, i, k, s)                                                      \
	| ((RH850_PIC_CELL_ADC_SRC_IS_TAUD(RH850_PIC_ADC_CELL(i)) &&                               \
	    RH850_PIC_CELL_ADC_SRC_TAUD_UNIT(RH850_PIC_ADC_CELL(i)) == (k) &&                      \
	    RH850_PIC_CELL_ADC_SG(RH850_PIC_ADC_CELL(i)) == (s))                                   \
		   ? (1U << RH850_PIC_CELL_ADC_SRC_TAUD_CH(RH850_PIC_ADC_CELL(i)))                 \
		   : 0U)
/* Return the matching TAUD channel contribution from ADC unit a only. */
#define RH850_PIC_TAUD_CH_OF_ADC_TERM(n, p, i, a, k, s)                                            \
	| ((RH850_PIC_CELL_ADC_UNIT(RH850_PIC_ADC_CELL(i)) == (a) &&                               \
	    RH850_PIC_CELL_ADC_SRC_IS_TAUD(RH850_PIC_ADC_CELL(i)) &&                               \
	    RH850_PIC_CELL_ADC_SRC_TAUD_UNIT(RH850_PIC_ADC_CELL(i)) == (k) &&                      \
	    RH850_PIC_CELL_ADC_SG(RH850_PIC_ADC_CELL(i)) == (s))                                   \
		   ? (1U << RH850_PIC_CELL_ADC_SRC_TAUD_CH(RH850_PIC_ADC_CELL(i)))                 \
		   : 0U)
/* Combine all matching entries into the TAUD channel mask for (k, s). */
#define RH850_PIC_TAUD_CH_MASK(k, s)                                                               \
	((uint16_t)(0U RH850_PIC_FOREACH_CELL(adc_trigger_selection, RH850_PIC_TAUD_CH_TERM, k, s)))
/* Channel mask of (k, s) written by the cells of ADC unit a only */
#define RH850_PIC_TAUD_CH_MASK_OF_ADC(a, k, s)                                                     \
	((uint16_t)(0U RH850_PIC_FOREACH_CELL(adc_trigger_selection,                               \
					      RH850_PIC_TAUD_CH_OF_ADC_TERM, a, k, s)))

/* The array row {sg0, ..., sg4 [, diag]} of the channel masks of TAUD unit k */
#define RH850_PIC_TAUD_CH_ROW(k)                                                                   \
	{                                                                                          \
		[0] = RH850_PIC_TAUD_CH_MASK(k, 0), [1] = RH850_PIC_TAUD_CH_MASK(k, 1),            \
		[2] = RH850_PIC_TAUD_CH_MASK(k, 2), [3] = RH850_PIC_TAUD_CH_MASK(k, 3),            \
		[4] = RH850_PIC_TAUD_CH_MASK(k, 4) RH850_PIC_TAUD_CH_DIAG_ELEM(k)                  \
	}

/*
 * Map each logical TAUD unit to the corresponding FSP ADC-trigger configuration
 * group. TAUD0 and TAUD1 use PIC20. TAUD2 and TAUD3 use PIC21 when available.
 */
#if (RH850_PIC_HAS_PIC21)
#define RH850_PIC_PIC21_TAUD_UNITS BSP_FEATURE_PIC21_TAUD_NUMBER_UNIT
#else
#define RH850_PIC_PIC21_TAUD_UNITS 0
#endif

/* INIT of the channel masks of TAUD unit k (arrays pic20/21_adc_convert_trigger_taud_sel) */
#define RH850_PIC_TAUD_CH_INIT_UNIT0                                                               \
	.pic20_adc_convert_trigger_taud_sel[0] = RH850_PIC_TAUD_CH_ROW(0),
#define RH850_PIC_TAUD_CH_INIT_UNIT1                                                               \
	.pic20_adc_convert_trigger_taud_sel[1] = RH850_PIC_TAUD_CH_ROW(1),
#if (RH850_PIC_HAS_PIC21)
#define RH850_PIC_TAUD_CH_INIT_UNIT2                                                               \
	.pic21_adc_convert_trigger_taud_sel[0] = RH850_PIC_TAUD_CH_ROW(2),
#if (BSP_FEATURE_PIC21_TAUD_NUMBER_UNIT > 1)
#define RH850_PIC_TAUD_CH_INIT_UNIT3                                                               \
	.pic21_adc_convert_trigger_taud_sel[1] = RH850_PIC_TAUD_CH_ROW(3),
#else
#define RH850_PIC_TAUD_CH_INIT_UNIT3
#endif
#else
#define RH850_PIC_TAUD_CH_INIT_UNIT2
#define RH850_PIC_TAUD_CH_INIT_UNIT3
#endif
/* INIT of the channel masks of all TAUD units of this MCU */
#define RH850_PIC_TAUD_CH_INIT_ALL                                                                 \
	RH850_PIC_TAUD_CH_INIT_UNIT0 RH850_PIC_TAUD_CH_INIT_UNIT1 RH850_PIC_TAUD_CH_INIT_UNIT2     \
	RH850_PIC_TAUD_CH_INIT_UNIT3

/* triangle-wave-pwm: enable and active level of (TAUD unit k, phase mask ph) */

/*
 * These dt-binding values are copied directly into the FSP configuration.
 * Verify at build time that their numeric encodings remain compatible.
 */
BUILD_ASSERT(PIC_ACTIVE_LEVEL_NONE == RH850_PIC_ARG_LEVEL_NONE &&
		     PIC_ACTIVE_LEVEL_HIGH == RH850_PIC_ARG_LEVEL_HIGH &&
		     PIC_ACTIVE_LEVEL_LOW == RH850_PIC_ARG_LEVEL_LOW,
	     "level codes of the dt-bindings header must equal the FSP values");
BUILD_ASSERT(PIC_TAUD_INT_SIGNAL_OUTPUT_CHANNEL_0 == RH850_PIC_ARG_INTCH_CH0 &&
		     PIC_TAUD_INT_SIGNAL_OUTPUT_CHANNEL_2 == RH850_PIC_ARG_INTCH_CH2 &&
		     PIC_TAUD_INT_SIGNAL_OUTPUT_CHANNEL_8 == RH850_PIC_ARG_INTCH_CH8,
	     "INT signal codes of the dt-bindings header must equal the FSP values");

/* Return 1 when entry i enables phase ph of TAUD unit k. */
#define RH850_PIC_TRIWAVE_EN_TERM(n, p, i, k, ph)                                                  \
	| ((RH850_PIC_CELL_TRIWAVE_UNIT(RH850_PIC_TRIWAVE_CELL(i)) == (k) &&                       \
	    (RH850_PIC_CELL_TRIWAVE_PHASES(RH850_PIC_TRIWAVE_CELL(i)) & (ph)))                     \
		   ? 1U                                                                            \
		   : 0U)
/* Return the active level contributed by entry i when it matches (k, ph). */
#define RH850_PIC_TRIWAVE_LEVEL_TERM(n, p, i, k, ph)                                               \
	| ((RH850_PIC_CELL_TRIWAVE_UNIT(RH850_PIC_TRIWAVE_CELL(i)) == (k) &&                       \
	    (RH850_PIC_CELL_TRIWAVE_PHASES(RH850_PIC_TRIWAVE_CELL(i)) & (ph)))                     \
		   ? RH850_PIC_CELL_TRIWAVE_LEVEL(RH850_PIC_TRIWAVE_CELL(i))                       \
		   : 0U)
/* Non-zero if some cell enables phase ph of TAUD unit k */
#define RH850_PIC_TRIWAVE_ENABLED(k, ph)                                                           \
	(0U RH850_PIC_FOREACH_CELL(triangle_wave_pwm, RH850_PIC_TRIWAVE_EN_TERM, k, ph))
/* Active level code of phase ph of TAUD unit k (OR of the cells; above 2 = two different levels) */
#define RH850_PIC_TRIWAVE_LEVEL(k, ph)                                                             \
	(0U RH850_PIC_FOREACH_CELL(triangle_wave_pwm, RH850_PIC_TRIWAVE_LEVEL_TERM, k, ph))
/* Initializer of one FSP phase structure (pic_taud_triangle_wave_pwm_phase_t) */
#define RH850_PIC_TRIWAVE_PHASE_INIT(k, ph)                                                        \
	{                                                                                          \
		.mode = RH850_PIC_TRIWAVE_ENABLED(k, ph) ? PIC_MODE_ENABLE : PIC_MODE_DISABLE,     \
		.active_level = (pic_active_level_t)RH850_PIC_TRIWAVE_LEVEL(k, ph),                \
	}
/* INIT of pic_taud_triangle_wave_pwm[k]: phases U, V, W */
#define RH850_PIC_TRIWAVE_INIT_UNIT(k)                                                             \
	.pic_taud_triangle_wave_pwm[k] = {                                                         \
		.u_phase = RH850_PIC_TRIWAVE_PHASE_INIT(k, 1U),                                    \
		.v_phase = RH850_PIC_TRIWAVE_PHASE_INIT(k, 2U),                                    \
		.w_phase = RH850_PIC_TRIWAVE_PHASE_INIT(k, 4U),                                    \
	},
#define RH850_PIC_TRIWAVE_INIT_UNIT0 RH850_PIC_TRIWAVE_INIT_UNIT(0)
#define RH850_PIC_TRIWAVE_INIT_UNIT1 RH850_PIC_TRIWAVE_INIT_UNIT(1)
#if (BSP_FEATURE_PIC_TAUD_TRIANGLE_WAVE_NUMBER > 2)
#define RH850_PIC_TRIWAVE_INIT_UNIT2 RH850_PIC_TRIWAVE_INIT_UNIT(2)
#else
#define RH850_PIC_TRIWAVE_INIT_UNIT2
#endif
#if (BSP_FEATURE_PIC_TAUD_TRIANGLE_WAVE_NUMBER > 3)
#define RH850_PIC_TRIWAVE_INIT_UNIT3 RH850_PIC_TRIWAVE_INIT_UNIT(3)
#else
#define RH850_PIC_TRIWAVE_INIT_UNIT3
#endif
/* INIT of the triangle wave phases of all TAUD units that have the function on this MCU */
#define RH850_PIC_TRIWAVE_INIT_ALL                                                                 \
	RH850_PIC_TRIWAVE_INIT_UNIT0 RH850_PIC_TRIWAVE_INIT_UNIT1 RH850_PIC_TRIWAVE_INIT_UNIT2     \
	RH850_PIC_TRIWAVE_INIT_UNIT3

/* TAUD INT-signal output selection */

/* TERM: the channel code of cell i if it is for TAUD unit k */
#define RH850_PIC_TAUD_INT_CH_TERM(n, p, i, k)                                                     \
	| ((RH850_PIC_CELL_TAUD_INT_UNIT(RH850_PIC_TAUD_INT_CELL(i)) == (k))                       \
		   ? RH850_PIC_CELL_TAUD_INT_CH(RH850_PIC_TAUD_INT_CELL(i))                        \
		   : 0U)
/* TERM: 1 if cell i is for TAUD unit k (the sum counts the cells of the unit) */
#define RH850_PIC_TAUD_INT_COUNT_TERM(n, p, i, k)                                                  \
	+ ((RH850_PIC_CELL_TAUD_INT_UNIT(RH850_PIC_TAUD_INT_CELL(i)) == (k)) ? 1U : 0U)
/* Channel code of TAUD unit k: the FSP pic_taud_int_signal_output_t value, 0 = none */
#define RH850_PIC_TAUD_INT_CH_CODE(k)                                                              \
	(0U RH850_PIC_FOREACH_CELL(taud_int_signal, RH850_PIC_TAUD_INT_CH_TERM, k))
/* Number of cells for TAUD unit k (checked to be at most 1) */
#define RH850_PIC_TAUD_INT_COUNT(k)                                                                \
	(0U RH850_PIC_FOREACH_CELL(taud_int_signal, RH850_PIC_TAUD_INT_COUNT_TERM, k))
/* INIT of pic_taud_int_signal_output[k] */
#define RH850_PIC_TAUD_INT_INIT_UNIT(k)                                                            \
	.pic_taud_int_signal_output[k] =                                                           \
		(pic_taud_int_signal_output_t)RH850_PIC_TAUD_INT_CH_CODE(k),
#define RH850_PIC_TAUD_INT_INIT_UNIT0 RH850_PIC_TAUD_INT_INIT_UNIT(0)
#if (BSP_FEATURE_PIC_TAUD_INT_SIGNAL_OUT_NUMBER > 1)
#define RH850_PIC_TAUD_INT_INIT_UNIT1 RH850_PIC_TAUD_INT_INIT_UNIT(1)
#else
#define RH850_PIC_TAUD_INT_INIT_UNIT1
#endif
#if (BSP_FEATURE_PIC_TAUD_INT_SIGNAL_OUT_NUMBER > 2)
#define RH850_PIC_TAUD_INT_INIT_UNIT2 RH850_PIC_TAUD_INT_INIT_UNIT(2)
#else
#define RH850_PIC_TAUD_INT_INIT_UNIT2
#endif
/* INIT of the INT signal outputs of all TAUD units that have one on this MCU */
#define RH850_PIC_TAUD_INT_INIT_ALL                                                                \
	RH850_PIC_TAUD_INT_INIT_UNIT0 RH850_PIC_TAUD_INT_INIT_UNIT1 RH850_PIC_TAUD_INT_INIT_UNIT2

/* =================================================================================================
 * 3. Validate the combined configuration at build time
 * =================================================================================================
 * The dt-binding macros constrain each individual entry. The checks below
 * validate MCU capabilities and detect conflicts between multiple entries.
 * Repeating an identical source is harmless because selections are combined
 * using bitwise OR.
 */

/* True if the INT-signal output of TAUD unit u exists on this MCU */
#define RH850_PIC_TAUD_INT_UNIT_EXISTS(u)                                                          \
	((u) < BSP_FEATURE_PIC_TAUD_INT_SIGNAL_OUT_NUMBER &&                                       \
	 ((u) == 0 || ((u) == 1 && RH850_PIC_HAS_INT_SIG_OUT_TAUD1) ||                             \
	  ((u) == 2 && RH850_PIC_HAS_INT_SIG_OUT_TAUD2)))

/* Validate one ADC trigger entry and its requested MCU capabilities. */
#define RH850_PIC_CHECK_ADC_CELL(n, p, i)                                                          \
	RH850_PIC_ASSERT(RH850_PIC_CELL_TAG(RH850_PIC_ADC_CELL(i)) ==                              \
				 RH850_PIC_CELL_TAG_ADC_TRIG,                                      \
			 "pic: adc-trigger-selection cells must be made with RH850_PIC_ADC_TRIG")  \
	RH850_PIC_ASSERT(RH850_PIC_CELL_ADC_UNIT(RH850_PIC_ADC_CELL(i)) <                          \
				 BSP_FEATURE_PIC_ADC_NUMBER_UNIT,                                  \
			 "pic: ADC unit does not exist on this MCU")                               \
	RH850_PIC_ASSERT(RH850_PIC_CELL_ADC_SG(RH850_PIC_ADC_CELL(i)) <                            \
				 BSP_FEATURE_PIC_ADC_NUMBER_SG,                                    \
			 "pic: scan group does not exist on this MCU "                             \
			 "(DIAG needs a diagnostic scan group)")                                   \
	RH850_PIC_ASSERT(RH850_PIC_CELL_ADC_REG(RH850_PIC_ADC_CELL(i)) == 0U ||                    \
				 RH850_PIC_HAS_PIC21 != 0,                                         \
			 "pic: this source (PIC21) does not exist on this MCU")                    \
	RH850_PIC_ASSERT(                                                                          \
		!RH850_PIC_CELL_ADC_SRC_IS_TAUD(RH850_PIC_ADC_CELL(i)) ||                          \
			(RH850_PIC_CELL_ADC_SRC_TAUD_UNIT(RH850_PIC_ADC_CELL(i)) < 2U &&           \
			 RH850_PIC_CELL_ADC_SRC_TAUD_UNIT(RH850_PIC_ADC_CELL(i)) <                 \
				 BSP_FEATURE_PIC20_TAUD_NUMBER_UNIT) ||                            \
			(RH850_PIC_CELL_ADC_SRC_TAUD_UNIT(RH850_PIC_ADC_CELL(i)) >= 2U &&          \
			 RH850_PIC_CELL_ADC_SRC_TAUD_UNIT(RH850_PIC_ADC_CELL(i)) - 2U <            \
				 RH850_PIC_PIC21_TAUD_UNITS),                                      \
		"pic: this TAUD unit does not exist as ADC trigger source on this MCU")
RH850_PIC_FOREACH_CELL_NOARG(adc_trigger_selection, RH850_PIC_CHECK_ADC_CELL)

/* Validate one triangle-wave PWM entry and its TAUD unit capability. */
#define RH850_PIC_CHECK_TRIWAVE_CELL(n, p, i)                                                      \
	RH850_PIC_ASSERT(RH850_PIC_CELL_TAG(RH850_PIC_TRIWAVE_CELL(i)) ==                          \
				 RH850_PIC_CELL_TAG_TRIWAVE,                                       \
			 "pic: triangle-wave-pwm cells must be made with RH850_PIC_TRIWAVE()")     \
	RH850_PIC_ASSERT(RH850_PIC_CELL_TRIWAVE_UNIT(RH850_PIC_TRIWAVE_CELL(i)) <                  \
				 BSP_FEATURE_PIC_TAUD_TRIANGLE_WAVE_NUMBER,                        \
			 "pic: this TAUD unit has no triangle-wave PWM on this MCU")
RH850_PIC_FOREACH_CELL_NOARG(triangle_wave_pwm, RH850_PIC_CHECK_TRIWAVE_CELL)

/* Validate one TAUD INT-signal entry and its output capability. */
#define RH850_PIC_CHECK_TAUD_INT_CELL(n, p, i)                                                     \
	RH850_PIC_ASSERT(RH850_PIC_CELL_TAG(RH850_PIC_TAUD_INT_CELL(i)) ==                         \
				 RH850_PIC_CELL_TAG_TAUD_INT,                                      \
			 "pic: taud-int-signal cells must be made with RH850_PIC_TAUD_INT()")      \
	RH850_PIC_ASSERT(RH850_PIC_TAUD_INT_UNIT_EXISTS(                                           \
				 RH850_PIC_CELL_TAUD_INT_UNIT(RH850_PIC_TAUD_INT_CELL(i))),        \
			 "pic: this TAUD unit has no INT-signal output on this MCU")
RH850_PIC_FOREACH_CELL_NOARG(taud_int_signal, RH850_PIC_CHECK_TAUD_INT_CELL)

/*
 * Require all trigger entries for the same ADC scan group to use the same edge
 * selection. More than one bit in the set of edge codes indicates a conflict.
 */
#define RH850_PIC_CHECK_ONE_EDGE(a, s)                                                             \
	RH850_PIC_ASSERT((RH850_PIC_ADC_EDGE_SEEN(a, s) &                                          \
			  (RH850_PIC_ADC_EDGE_SEEN(a, s) - 1U)) == 0U,                             \
			 "pic: one scan group has two different trigger edges")
/*
 * Require ADC units that share the same TAUD unit and scan group to select the
 * same TAUD channels. The resulting channel configuration has no ADC-unit index.
 */
#define RH850_PIC_CHECK_SAME_TAUD_CH(a, k, s)                                                      \
	RH850_PIC_ASSERT(RH850_PIC_TAUD_CH_MASK_OF_ADC(a, k, s) == 0U ||                           \
				 RH850_PIC_TAUD_CH_MASK_OF_ADC(a, k, s) ==                         \
					 RH850_PIC_TAUD_CH_MASK(k, s),                             \
			 "pic: ADC units that use the same TAUD unit on the same scan group "      \
			 "must use the same TAUD channels")
/*
 * Loops for the checks above. Every index of the largest MCU is visited; a unit or scan group that
 * the MCU does not have cannot have a cell (rejected by the per-cell checks), so its check is
 * trivial.
 */
/* fn(a, sg) for scan groups 0..5 of ADC unit a */
#define RH850_PIC_FOR_EACH_SG(fn, a)                                                               \
	fn(a, 0) fn(a, 1) fn(a, 2) fn(a, 3) fn(a, 4) fn(a, 5)
/* fn(a, sg) for ADC units 0..3 and scan groups 0..5 */
#define RH850_PIC_FOR_EACH_ADC_SG(fn)                                                              \
	RH850_PIC_FOR_EACH_SG(fn, 0) RH850_PIC_FOR_EACH_SG(fn, 1) RH850_PIC_FOR_EACH_SG(fn, 2)     \
	RH850_PIC_FOR_EACH_SG(fn, 3)
/* fn(a, k, sg) for scan groups 0..5 of ADC unit a and TAUD unit k */
#define RH850_PIC_FOR_EACH_TAUD_SG(fn, a, k)                                                       \
	fn(a, k, 0) fn(a, k, 1) fn(a, k, 2) fn(a, k, 3) fn(a, k, 4) fn(a, k, 5)
/* fn(a, k, sg) for TAUD units 0..3 and scan groups 0..5 of ADC unit a */
#define RH850_PIC_FOR_EACH_ADC_TAUD(fn, a)                                                         \
	RH850_PIC_FOR_EACH_TAUD_SG(fn, a, 0) RH850_PIC_FOR_EACH_TAUD_SG(fn, a, 1)                  \
	RH850_PIC_FOR_EACH_TAUD_SG(fn, a, 2) RH850_PIC_FOR_EACH_TAUD_SG(fn, a, 3)
/* fn(a, k, sg) for ADC units 0..3, TAUD units 0..3 and scan groups 0..5 */
#define RH850_PIC_FOR_EACH_ADC_TAUD_SG(fn)                                                         \
	RH850_PIC_FOR_EACH_ADC_TAUD(fn, 0) RH850_PIC_FOR_EACH_ADC_TAUD(fn, 1)                      \
	RH850_PIC_FOR_EACH_ADC_TAUD(fn, 2) RH850_PIC_FOR_EACH_ADC_TAUD(fn, 3)
RH850_PIC_FOR_EACH_ADC_SG(RH850_PIC_CHECK_ONE_EDGE)
RH850_PIC_FOR_EACH_ADC_TAUD_SG(RH850_PIC_CHECK_SAME_TAUD_CH)

/* Reject conflicting active levels for the same TAUD phase. */
#define RH850_PIC_CHECK_TRIWAVE_ONE_LEVEL(k, ph)                                                   \
	RH850_PIC_ASSERT(RH850_PIC_TRIWAVE_LEVEL(k, ph) <= 2U,                                     \
			 "pic: a phase has two different active levels")
/* fn(k, phase mask) for the phases U, V, W of TAUD unit k */
#define RH850_PIC_FOR_EACH_PHASE(fn, k) fn(k, 1U) fn(k, 2U) fn(k, 4U)
RH850_PIC_FOR_EACH_PHASE(RH850_PIC_CHECK_TRIWAVE_ONE_LEVEL, 0)
RH850_PIC_FOR_EACH_PHASE(RH850_PIC_CHECK_TRIWAVE_ONE_LEVEL, 1)
RH850_PIC_FOR_EACH_PHASE(RH850_PIC_CHECK_TRIWAVE_ONE_LEVEL, 2)
RH850_PIC_FOR_EACH_PHASE(RH850_PIC_CHECK_TRIWAVE_ONE_LEVEL, 3)

/* Require each TAUD unit to select at most one INT-signal channel. */
#define RH850_PIC_CHECK_TAUD_INT_ONE_CH(k)                                                         \
	RH850_PIC_ASSERT(RH850_PIC_TAUD_INT_COUNT(k) <= 1U,                                        \
			 "pic: a TAUD unit has two INT-signal channels")
RH850_PIC_CHECK_TAUD_INT_ONE_CH(0)
RH850_PIC_CHECK_TAUD_INT_ONE_CH(1)
RH850_PIC_CHECK_TAUD_INT_ONE_CH(2)
RH850_PIC_CHECK_TAUD_INT_ONE_CH(3)

/* =================================================================================================
 * 4. The constants handed to R_PIC_Open()
 * =================================================================================================
 */

/*
 * Map the taud-input-control-function Devicetree string to the corresponding
 * FSP function value by name. The string is converted to an uppercase token
 * with '-' replaced by '_', then prefixed with RH850_PIC_FUNC_.
 */
#define RH850_PIC_FUNC_NONE                       PIC10_FUNC_NOT_SELECTED
#define RH850_PIC_FUNC_TIMER_INPUT_SELECT         PIC10_FUNC_TIMER_INPUT_SEL
#define RH850_PIC_FUNC_PWM_OUTPUT_WITH_DEAD_TIME  PIC10_FUNC_PWM_OUTPUT
#define RH850_PIC_FUNC_HIGH_ACCURACY_TRIANGLE_WAVE_PWM_WITH_DEAD_TIME                              \
	PIC10_FUNC_HIGH_ACCURACY_TRIANGLE_WAVE_PWM
#define RH850_PIC_FUNC_DELAY_PULSE_OUTPUT_WITH_DEAD_TIME PIC10_FUNC_DELAY_PULSE_OUTPUT
#define RH850_PIC_FUNC_TRIGGER_PULSE_WIDTH_MEASUREMENT                                             \
	PIC10_FUNC_TRIGGER_PULSE_WIDTH_MEASUREMENT
/* FSP value of the "taud-input-control-function" string of the PIC node */
#define RH850_PIC_FUNC_MODE                                                                        \
	((pic10_func_t)UTIL_CAT(RH850_PIC_FUNC_,                                                   \
				DT_STRING_UPPER_TOKEN(RH850_PIC_NODE, taud_input_control_function)))

/* Extended configuration generated from the PIC Devicetree properties. */
static const pic_extended_cfg_t g_pic_ext_cfg = {
	RH850_PIC_TRIWAVE_INIT_ALL RH850_PIC_TAUD_INT_INIT_ALL RH850_PIC_ADC_INIT_ALL
	RH850_PIC_TAUD_CH_INIT_ALL
};

/*
 * Top-level PIC configuration. Features without a corresponding Devicetree
 * property are explicitly disabled.
 */
static const pic_cfg_t g_pic_cfg = {
	.pic10_function_mode = RH850_PIC_FUNC_MODE,
	.pic10_encoder_trigger = PIC_MODE_DISABLE,
	.pic10_trigger_pulse_width = PIC_MODE_DISABLE,
	.pic_simultaneous = PIC_MODE_DISABLE,
	.pic_timer_input_sel = PIC_MODE_DISABLE,
	.pic_enca_trigger = PIC_MODE_DISABLE,
	.pic_taud_input_select = PIC_MODE_DISABLE,
	.pic_taud_trigger_output = PIC_MODE_DISABLE,
	.p_extend = &g_pic_ext_cfg,
};

#endif /* DT_HAS_COMPAT_STATUS_OKAY(renesas_rh850_pic) */

#endif /* ZEPHYR_SOC_RENESAS_RH850_COMMON_FSP_PIC_PIC_RENESAS_RH850_CFG_H_ */
