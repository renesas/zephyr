/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RENESAS_RH850_CGC_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RENESAS_RH850_CGC_H_

#include <zephyr/drivers/clock_control.h>
#include <r_cgc.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Optional payload for clock_control_configure(). */
struct rh850_cgc_configure {
	/** PLL configuration used when configuring/starting RH850_OSC(PLL/PLL2). */
	const cgc_pll_cfg_t *pll_cfg;

	/** Legacy FSP systemClockSet divider payload. Optional and SoC-dependent. */
	const cgc_divider_cfg_t *divider_cfg;

	/** FSP systemClockGearSet payload. Optional and SoC-dependent. */
	const cgc_system_clock_cfg_t *system_clock_cfg;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RENESAS_RH850_CGC_H_ */
