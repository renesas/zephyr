/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RENESAS_RCAR_ADG_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RENESAS_RCAR_ADG_H_

/*
 * Clock identifiers for the Renesas R-Car Audio Clock Generator (ADG).
 *
 * A clock identifier is the single clock cell of an ADG consumer, and the subsystem value
 * passed to the clock_control API. It is split in two byte wide fields, of which at most one
 * is non-zero:
 *
 *   [15:8] AUDIO_CLKOUT pin, see RCAR_ADG_AUDIO_CLKOUTx. Acting on a pin acts on whichever
 *          clock source the device tree routed to it through "audio-clkout-sel".
 *   [7:0]  ADG internal clock source, see RCAR_ADG_AUDIO_*. Acting on a source acts on the
 *          generator itself, no matter which pin it is routed to.
 *
 * The values below are driver level ids, they are not the values of the matching ADG register
 * fields. The driver translates them, see enum i2s_rcar_audio_clkout in
 * <zephyr/drivers/clock_control/renesas_rcar_adg.h>.
 */

#define RCAR_ADG_CLKOUT_MASK      (0xFF00) /* Mask of the AUDIO_CLKOUT pin field */
#define RCAR_ADG_SOURCECLOCK_MASK (0x00FF) /* Mask of the clock source field */

#define RCAR_ADG_CLKOUT_POS (8) /* Position of the AUDIO_CLKOUT pin field */
#define RCAR_ADG_CLKSRC_POS (0) /* Position of the clock source field */

/*
 * ADG output pin
 *
 * Encoded one based so that a pin id is never 0, which would collide with a plain clock
 * source id. The driver recovers the pin index by subtracting RCAR_ADG_AUDIO_CLKOUT0.
 */
#define RCAR_ADG_AUDIO_CLKOUT0 ((1 << RCAR_ADG_CLKOUT_POS) & RCAR_ADG_CLKOUT_MASK)
#define RCAR_ADG_AUDIO_CLKOUT1 ((2 << RCAR_ADG_CLKOUT_POS) & RCAR_ADG_CLKOUT_MASK)
#define RCAR_ADG_AUDIO_CLKOUT2 ((3 << RCAR_ADG_CLKOUT_POS) & RCAR_ADG_CLKOUT_MASK)
#define RCAR_ADG_AUDIO_CLKOUT3 ((4 << RCAR_ADG_CLKOUT_POS) & RCAR_ADG_CLKOUT_MASK)

/*
 * Helper to reference an AUDIO_CLKOUT pin by number, e.g. clocks = <&adg0 ADG_CLKOUT(0)>;
 * SOURCE must be a literal pin number in the 0 to 3 range.
 */
#define ADG_CLKOUT(SOURCE) (RCAR_ADG_AUDIO_CLKOUT##SOURCE)

/*
 * ADG clock output sources
 *
 * BRGA and BRGB are the two baud rate generators, AVB0 to AVB7 the eight avb_counter8
 * channels. RCAR_ADG_AUDIO_NONE is 0, so it sets neither field and marks an AUDIO_CLKOUT pin
 * as unused in "audio-clkout-sel".
 */
#define RCAR_ADG_AUDIO_NONE ((0 << RCAR_ADG_CLKSRC_POS) & RCAR_ADG_SOURCECLOCK_MASK)
#define RCAR_ADG_AUDIO_BRGA ((1 << RCAR_ADG_CLKSRC_POS) & RCAR_ADG_SOURCECLOCK_MASK)
#define RCAR_ADG_AUDIO_BRGB ((2 << RCAR_ADG_CLKSRC_POS) & RCAR_ADG_SOURCECLOCK_MASK)
#define RCAR_ADG_AUDIO_AVB0 ((3 << RCAR_ADG_CLKSRC_POS) & RCAR_ADG_SOURCECLOCK_MASK)
#define RCAR_ADG_AUDIO_AVB1 ((4 << RCAR_ADG_CLKSRC_POS) & RCAR_ADG_SOURCECLOCK_MASK)
#define RCAR_ADG_AUDIO_AVB2 ((5 << RCAR_ADG_CLKSRC_POS) & RCAR_ADG_SOURCECLOCK_MASK)
#define RCAR_ADG_AUDIO_AVB3 ((6 << RCAR_ADG_CLKSRC_POS) & RCAR_ADG_SOURCECLOCK_MASK)
#define RCAR_ADG_AUDIO_AVB4 ((7 << RCAR_ADG_CLKSRC_POS) & RCAR_ADG_SOURCECLOCK_MASK)
#define RCAR_ADG_AUDIO_AVB5 ((8 << RCAR_ADG_CLKSRC_POS) & RCAR_ADG_SOURCECLOCK_MASK)
#define RCAR_ADG_AUDIO_AVB6 ((9 << RCAR_ADG_CLKSRC_POS) & RCAR_ADG_SOURCECLOCK_MASK)
#define RCAR_ADG_AUDIO_AVB7 ((10 << RCAR_ADG_CLKSRC_POS) & RCAR_ADG_SOURCECLOCK_MASK)

/*
 * Helper to reference a clock source by name, e.g. clocks = <&adg0 ADG_CLKSRC(BRGA)>;
 * SOURCE must be one of NONE, BRGA, BRGB or AVB0 to AVB7.
 */
#define ADG_CLKSRC(SOURCE) (RCAR_ADG_AUDIO_##SOURCE)

/*
 * BRG input clock select
 *
 * Values of the "brga-clk-sel" and "brgb-clk-sel" properties. Unlike the ids above these are
 * the raw BRGCKR clock select field values, see enum i2s_rcar_audio_brg_clkin. CLKA, CLKB and
 * CLKC are the external clocks on the AUDIO_CLKA, AUDIO_CLKB and AUDIO_CLKC pins, S0D4 is an
 * internal clock supplied by the CPG.
 */
#define RCAR_ADG_BRG_CLKIN_CLKA 0
#define RCAR_ADG_BRG_CLKIN_CLKB 1
#define RCAR_ADG_BRG_CLKIN_S0D4 2
#define RCAR_ADG_BRG_CLKIN_CLKC 4

/*
 * Helper to select a BRG input clock, e.g. brga-clk-sel = <ADG_BRGSRC(CLKA)>;
 * SOURCE must be one of CLKA, CLKB, CLKC or S0D4.
 */
#define ADG_BRGSRC(SOURCE) (RCAR_ADG_BRG_CLKIN_##SOURCE)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RENESAS_RCAR_ADG_H_ */
