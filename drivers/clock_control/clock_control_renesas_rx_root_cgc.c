/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_cgc_root_clock

#include <string.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/drivers/clock_control/renesas_rx_cgc.h>

static int clock_control_renesas_rx_root_on(const struct device *dev, clock_control_subsys_t sys)
{
	return -ENOTSUP;
}

static int clock_control_renesas_rx_root_off(const struct device *dev, clock_control_subsys_t sys)
{
	return -ENOTSUP;
}

static int clock_control_renesas_rx_root_get_rate(const struct device *dev,
						  clock_control_subsys_t sys, uint32_t *rate)
{
	const struct clock_control_rx_root_cfg *config = dev->config;

	ARG_UNUSED(sys);

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	*rate = config->rate;
	return 0;
}

static int clock_control_rx_init(const struct device *dev)
{
	ARG_UNUSED(dev);

#if CONFIG_HAS_RENESAS_RX_FSP
	/* Initialize the system clock and peripheral clock */
	bsp_clock_init();

	FSP_HARDWARE_REGISTER_WAIT(R_SYSTEM->PDCTRESWM_b.PDCSF, 0);

	if (1 == R_SYSTEM->PDCTRESWM_b.PDPGSF) {
		/* Turn on ESWM power domain.
		 * This requires MOCO to be enabled, but MOCO is always enabled after
		 * bsp_clock_init().
		 */
		R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_OM_LPC_BATT_SWR);
		FSP_HARDWARE_REGISTER_WAIT((R_SYSTEM->PDCTRESWM & (R_SYSTEM_PDCTRESWM_PDCSF_Msk |
								   R_SYSTEM_PDCTRESWM_PDPGSF_Msk)),
					   R_SYSTEM_PDCTRESWM_PDPGSF_Msk);
		R_SYSTEM->PDCTRESWM = 0;
		FSP_HARDWARE_REGISTER_WAIT((R_SYSTEM->PDCTRESWM & (R_SYSTEM_PDCTRESWM_PDCSF_Msk |
								   R_SYSTEM_PDCTRESWM_PDPGSF_Msk)),
					   0);
		R_BSP_RegisterProtectEnable(BSP_REG_PROTECT_OM_LPC_BATT_SWR);
	}
#endif /* CONFIG_HAS_RENESAS_RX_FSP */

	return 0;
}

static DEVICE_API(clock_control, clock_control_renesas_rx_root_api) = {
	.on = clock_control_renesas_rx_root_on,
	.off = clock_control_renesas_rx_root_off,
	.get_rate = clock_control_renesas_rx_root_get_rate,
};

#define ROOT_CLK_INIT(idx)                                                                         \
	static const struct clock_control_rx_root_cfg clock_control_rx_root_cfg##idx = {           \
		.rate = DT_INST_PROP(idx, clock_frequency),                                        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, NULL, NULL, NULL,                             \
			      &clock_control_rx_root_cfg##idx, PRE_KERNEL_1,                       \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,                                  \
			      &clock_control_renesas_rx_root_api);

DT_INST_FOREACH_STATUS_OKAY(ROOT_CLK_INIT);
