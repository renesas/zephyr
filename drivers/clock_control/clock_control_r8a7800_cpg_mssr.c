/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r8a7800_cpg_mssr

#include <errno.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/dt-bindings/clock/renesas_cpg_mssr.h>
#include <zephyr/dt-bindings/clock/r8a7800_cpg_mssr.h>
#include <zephyr/irq.h>
#include "clock_control_renesas_cpg_mssr.h"

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(clock_control_rcar);

struct r8a7800_cpg_mssr_cfg {
	DEVICE_MMIO_ROM; /* Must be first */
};

struct r8a7800_cpg_mssr_data {
	struct rcar_cpg_mssr_data cmn; /* Must be first */
};

/* NOTE: the array MUST be sorted by module field */
static struct cpg_clk_info_table core_props[] = {
	RCAR_CORE_CLK_INFO_ITEM(R8A78000_CLK_SGASYNCD16_PERW_BUS, RCAR_CPG_NONE, RCAR_CPG_NONE,
				RCAR_CPG_MHZ(66.666)),
};

/* NOTE: the array MUST be sorted by module field */
static struct cpg_clk_info_table mod_props[] = {
	RCAR_MOD_CLK_INFO_ITEM(130, R8A78000_CLK_SGASYNCD16_PERW_BUS),
};

int r8a7800_cpg_mssr_start_stop(const struct device *dev, clock_control_subsys_t sys, bool enable)
{
	struct rcar_cpg_clk *clk = (struct rcar_cpg_clk *)sys;
	int ret;

	if (!dev || !sys) {
		return -EINVAL;
	}

	if (clk->domain == CPG_MOD) {
		/* Gen5 SCIF module is enabled by default (Register MDLC[n]MSRES[l]) */
		/* To do: Support module standby for module domain */
		ret = 0;
	} else if (clk->domain == CPG_CORE) {
		/* To do: Support setting clock for core domain */
		ret = 0;
	} else {
		ret = -EINVAL;
	}

	return ret;
}

static uint32_t r8a7800_get_div_helper(uint32_t reg_val, uint32_t module)
{
	switch (module) {
	case R8A78000_CLK_SGASYNCD16_PERW_BUS:
		return 1;
	default:
		return RCAR_CPG_NONE;
	}
}

static int r8a7800_set_rate_helper(uint32_t module, uint32_t *divider, uint32_t *div_mask)
{
	return -ENOTSUP;
}

static int r8a7800_cpg_mssr_start(const struct device *dev, clock_control_subsys_t sys)
{
	return r8a7800_cpg_mssr_start_stop(dev, sys, true);
}

static int r8a7800_cpg_mssr_stop(const struct device *dev, clock_control_subsys_t sys)
{
	return r8a7800_cpg_mssr_start_stop(dev, sys, false);
}

static int r8a7800_cpg_mssr_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	rcar_cpg_build_clock_relationship(dev);
	rcar_cpg_update_all_in_out_freq(dev);
	return 0;
}

static DEVICE_API(clock_control, r8a7800_cpg_mssr_api) = {
	.on = r8a7800_cpg_mssr_start,
	.off = r8a7800_cpg_mssr_stop,
	.get_rate = rcar_cpg_get_rate,
	.set_rate = rcar_cpg_set_rate,
};

#define R8A7800_MSSR_INIT(inst)                                                                    \
	static struct r8a7800_cpg_mssr_cfg cpg_mssr##inst##_cfg = {                                \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
	};                                                                                         \
                                                                                                   \
	static struct r8a7800_cpg_mssr_data cpg_mssr##inst##_data = {                              \
		.cmn.clk_info_table[CPG_CORE] = core_props,                                        \
		.cmn.clk_info_table_size[CPG_CORE] = ARRAY_SIZE(core_props),                       \
		.cmn.clk_info_table[CPG_MOD] = mod_props,                                          \
		.cmn.clk_info_table_size[CPG_MOD] = ARRAY_SIZE(mod_props),                         \
		.cmn.get_div_helper = r8a7800_get_div_helper,                                      \
		.cmn.set_rate_helper = r8a7800_set_rate_helper};                                   \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &r8a7800_cpg_mssr_init, NULL, &cpg_mssr##inst##_data,          \
			      &cpg_mssr##inst##_cfg, PRE_KERNEL_1,                                 \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &r8a7800_cpg_mssr_api);

DT_INST_FOREACH_STATUS_OKAY(R8A7800_MSSR_INIT)
