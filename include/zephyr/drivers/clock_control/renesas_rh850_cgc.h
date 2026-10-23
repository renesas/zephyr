/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RENESAS_RH850_CGC_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RENESAS_RH850_CGC_H_

#include <stdint.h>

#include <zephyr/drivers/clock_control.h>
#include <r_cgc.h>

#ifdef __cplusplus
extern "C" {
#endif

/** RH850 PCLK source/divider configuration payload. */
struct rh850_cgc_pclk_configure {
	/** Concrete RH850_CLOCK_* identifier such as RH850_CLOCK_RLIN3. */
	uint32_t clock_id;

	/** Logical source RH850_CLOCK_* identifier parsed from clock-source. */
	uint32_t source_clock_id;

	/**
	 * HAL divider encoding, translated from the physical DT division factor.
	 * RLIN accepts 0/1 for no division and 4/8 for supported MOSC paths.
	 */
	uint32_t div;
};

/** Optional payload for clock_control_configure(). */
struct rh850_cgc_configure {
	/** PLL configuration used when configuring/starting RH850_OSC(PLL/PLL2). */
	const cgc_pll_cfg_t *pll_cfg;

	/**
	 * Legacy FSP systemClockSet divider payload, forwarded when system_clock_cfg
	 * is NULL. RH850 U2x FSP reports this operation as unsupported.
	 */
	const cgc_divider_cfg_t *divider_cfg;

	/** FSP systemClockGearSet payload. Takes precedence over divider_cfg. */
	const cgc_system_clock_cfg_t *system_clock_cfg;

	/** Zephyr-native PCLK source/divider payload. */
	const struct rh850_cgc_pclk_configure *pclk_cfg;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RENESAS_RH850_CGC_H_ */
