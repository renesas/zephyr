/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RH850_DEVICE_SUBSYS_H_
#define RH850_DEVICE_SUBSYS_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_SOC_SERIES_RH850U2A)
#include <zephyr/dt-bindings/clock/renesas_rh850u2a_clock.h>
#elif defined(CONFIG_SOC_SERIES_RH850U2B)
#include <zephyr/dt-bindings/clock/renesas_rh850u2b_clock.h>
#elif defined(CONFIG_SOC_SERIES_RH850U2C)
#include <zephyr/dt-bindings/clock/renesas_rh850u2c_clock.h>
#else
#include <zephyr/dt-bindings/clock/renesas_rh850_clock_common.h>
#endif

struct subsys_data {
	clock_control_subsys_t subsys;
	uint32_t startup_us;
};

struct device_data {
	const struct device *dev;
	const struct subsys_data *subsys_data;
	size_t subsys_cnt;
};

/*
 * Do not use RLIN3 here when UART console is enabled; boards commonly route
 * uart0/rlin30 to console. Do not use OSTM either because it is commonly the
 * Zephyr system timer. For this generic clock_control_api test, disable
 * serial/console and test a non-critical module such as RSCFD/CANFD channel 0.
 *
 * The subsystem encodes:
 *   domain  = peripheral/module clock
 *   module  = RSCFD module standby register
 *   channel = 0
 *   clock   = CANFD_C rate source for clock_control_get_rate()
 */
static const struct subsys_data subsys_data[] = {
	{
		.subsys = (clock_control_subsys_t)RH850_PCLK(RSCFD, 0U, CANFD_C),
		.startup_us = 0U,
	},
};

static const struct device_data devices[] = {
	{
		.dev = DEVICE_DT_GET_ONE(renesas_rh850_cgc),
		.subsys_data = subsys_data,
		.subsys_cnt = ARRAY_SIZE(subsys_data),
	},
};

#endif /* RH850_DEVICE_SUBSYS_H_ */
