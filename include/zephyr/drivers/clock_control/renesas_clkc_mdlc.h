/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RCAR_CLKC_MDLC_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RCAR_CLKC_MDLC_H_

#include <zephyr/drivers/clock_control.h>

struct rcar_clkc {
	uint32_t domain;
	uint32_t module;
	uint32_t rate;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RCAR_CLKC_MDLC_H_ */
