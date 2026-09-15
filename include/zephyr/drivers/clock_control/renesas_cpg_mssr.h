/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 BayLibre, SAS
 * Copyright (c) 2017 Linaro Limited.
 * Copyright (c) 2017 RnDity Sp. z o.o.
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RCAR_CLOCK_CONTROL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RCAR_CLOCK_CONTROL_H_

#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/renesas_cpg_mssr.h>

struct rcar_cpg_clk {
	uint32_t domain;
	uint32_t module;
	uint32_t rate;
};

#ifdef CONFIG_CLOCK_CONTROL_ARM_SCMI

struct rcar_scmi_clk {
	clock_control_subsys_t clk;
};

typedef struct rcar_scmi_clk rcar_clk_t;

#define RCAR_DT_CLOCKS_CELL_BY_IDX(node_id, idx)                                                   \
	{.clk = (clock_control_subsys_t)DT_CLOCKS_CELL_BY_IDX(node_id, idx, name)}

#define RCAR_DT_INST_CLOCKS_CELL_BY_IDX(inst, idx)                                                 \
	{.clk = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_IDX(inst, idx, name)}

#define RCAR_DT_INST_CLOCKS_CELL_BY_NAME(inst, cell_name)                                          \
	{.clk = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(inst, cell_name, name)}

#define RCAR_CLOCK_SUBSYS(clock) (clock).clk

#else

typedef struct rcar_cpg_clk rcar_clk_t;

#define RCAR_DT_CLOCKS_CELL_BY_IDX(node_id, idx)                                                   \
	{                                                                                          \
		.domain = DT_CLOCKS_CELL_BY_IDX(node_id, idx, domain),                             \
		.module = DT_CLOCKS_CELL_BY_IDX(node_id, idx, module),                             \
	}

#define RCAR_DT_INST_CLOCKS_CELL_BY_IDX(inst, idx)                                                 \
	{                                                                                          \
		.domain = DT_INST_CLOCKS_CELL_BY_IDX(inst, idx, domain),                           \
		.module = DT_INST_CLOCKS_CELL_BY_IDX(inst, idx, module),                           \
	}

#define RCAR_DT_INST_CLOCKS_CELL_BY_NAME(inst, cell_name)                                          \
	{                                                                                          \
		.domain = DT_INST_CLOCKS_CELL_BY_NAME(inst, cell_name, domain),                    \
		.module = DT_INST_CLOCKS_CELL_BY_NAME(inst, cell_name, module),                    \
	}

#define RCAR_CLOCK_SUBSYS(clock) (clock_control_subsys_t)&(clock)

#endif /* CONFIG_CLOCK_CONTROL_ARM_SCMI */

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_RCAR_CLOCK_CONTROL_H_ */
