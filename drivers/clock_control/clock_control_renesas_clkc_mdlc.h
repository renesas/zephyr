/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_RENESAS_RENESAS_CLKC_MDLC_H_
#define ZEPHYR_DRIVERS_RENESAS_RENESAS_CLKC_MDLC_H_

#include <zephyr/spinlock.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/device_mmio.h>

#define CLKC_NUM_DOMAINS 2

#define RCAR_CLKC_NONE     -1
#define RCAR_CLKC_KHZ(khz) ((khz) * 1000U)
#define RCAR_CLKC_MHZ(mhz) (RCAR_CLKC_KHZ(mhz) * 1000U)

#define RCAR_CLKC_CAP_NONE 0
#define RCAR_CLKC_CAP_STOP BIT(0)

#define CLKC_CORE 0 /* Core Clock */
#define CLKC_MOD  1 /* Module Clock */

#define CLKC_REGION_NUMS 5

enum clkc_region {
	CLKC_REGION_SYSSS_MAIN = 0,
	CLKC_REGION_SYSSS_PERE = 1,
	CLKC_REGION_SYSSS_TOP = 2,
	CLKC_REGION_SYSSS_HSCS = 3,
	CLKC_REGION_SYSSS_SCP_PSO = 4,
};

#define MDLC_INDEX_NUMS 32
#define MDLC_SLOT_NUMS  16

struct rcar_clkc {
	uint32_t domain;
	uint32_t module;
	uint32_t rate;
};

struct clkc_clk_info_table {
	uint32_t domain;
	uint32_t module;
	mem_addr_t offset;
	uint32_t parent_id;
	int64_t in_freq;
	int64_t out_freq;
	struct clkc_clk_info_table *parent;
	struct clkc_clk_info_table *children_list;
	struct clkc_clk_info_table *next_sibling;
	uint32_t capabilities;
	enum clkc_region region_id;
};

struct rcar_clkc_mdlc_data {
	DEVICE_MMIO_RAM; /* Must be first */

	struct clkc_clk_info_table *clk_info_table[CLKC_NUM_DOMAINS];
	const uint32_t clk_info_table_size[CLKC_NUM_DOMAINS];

	struct k_spinlock lock;

	uint32_t (*get_div_helper)(uint32_t reg, uint32_t module);
	int (*set_rate_helper)(uint32_t module, uint32_t *div, uint32_t *div_mask);
};

#define RCAR_CORE_CLK_INFO_ITEM(id, off, region, par_id, in_frq)                                   \
	{                                                                                          \
		.domain = CLKC_CORE,                                                               \
		.module = id,                                                                      \
		.offset = off,                                                                     \
		.parent_id = par_id,                                                               \
		.in_freq = in_frq,                                                                 \
		.out_freq = RCAR_CLKC_NONE,                                                        \
		.parent = NULL,                                                                    \
		.children_list = NULL,                                                             \
		.next_sibling = NULL,                                                              \
		.capabilities = RCAR_CLKC_CAP_NONE,                                                \
		.region_id = region,                                                               \
	}

#define RCAR_MOD_CLK_INFO_ITEM(id, par_id, cap)                                                    \
	{                                                                                          \
		.domain = CLKC_MOD,                                                                \
		.module = id,                                                                      \
		.offset = RCAR_CLKC_NONE,                                                          \
		.parent_id = par_id,                                                               \
		.in_freq = RCAR_CLKC_NONE,                                                         \
		.out_freq = RCAR_CLKC_NONE,                                                        \
		.parent = NULL,                                                                    \
		.children_list = NULL,                                                             \
		.next_sibling = NULL,                                                              \
		.capabilities = cap,                                                               \
		.region_id = RCAR_CLKC_NONE,                                                       \
	}

/* Write protect disable code */
#define CLKC_MDLC_WPR_DIS 0xA5A5A501

/* Write protect enable code */
#define CLKC_MDLC_WPR_EN 0xA5A5A500

/* MDLC keycode register protection offset */
#define MDLCPKCPROT1 0x0CF4

/* CLKC keycode register protection offset */
#define CLKCKCPROT 0x1370

/* MDLC reset status register offset */
#define MDLCMSRESS(l) (0x0960 + (l) * 4)

/* MDLC reset control register offset */
#define MDLCMRES(l) (0x0900 + (l) * 4)

/* MDLC base addresses */
static const uint32_t mdlc_base[] = {
	0xC3060000, /* Region 0  */
	0xC3460000, /* Region 1  */
	0xC5000000, /* Region 2  */
	0xC08F0000, /* Region 3  */
	0xC05D0000, /* Region 4  */
	0xE8000000, /* Region 5  */
	0xE8080000, /* Region 6  */
	0xE8100000, /* Region 7  */
	0xE8180000, /* Region 8  */
	0xE8200000, /* Region 9  */
	0xE8280000, /* Region 10 */
	0xE8300000, /* Region 11 */
	0xE8380000, /* Region 12 */
	0xC9C90000, /* Region 13 */
	0x19440000, /* Region 14 */
	0xC6480000, /* Region 15 */
	0xDE200000, /* Region 16 */
	0xC1990000, /* Region 17 */
	0xC1D90000, /* Region 18 */
	0xCB510000, /* Region 19 */
	0xCBE90000, /* Region 20 */
	0xE9980000, /* Region 21 */
	0xD2C30000, /* Region 22 */
	0xD6C30000, /* Region 23 */
	0xCA410000, /* Region 24 */
	0xCA510000  /* Region 25 */
};

#define MDLC_REGION_NUMS ARRAY_SIZE(mdlc_base)

/* Base addresses for CLKC registers */
static const uint32_t clkc_base[CLKC_REGION_NUMS] = {
	[CLKC_REGION_SYSSS_MAIN] = 0xC1320000,    [CLKC_REGION_SYSSS_PERE] = 0xC08F0000,
	[CLKC_REGION_SYSSS_TOP] = 0xC6480000,     [CLKC_REGION_SYSSS_HSCS] = 0xDE200000,
	[CLKC_REGION_SYSSS_SCP_PSO] = 0xC1330000,
};

enum clock_control_status rcar_mdlc_clock_get_status(uint32_t module);

int rcar_mdlc_clock_enabled(uint32_t module, bool enable);

int rcar_clkc_protect_enabled(uint32_t region, bool enabled);

struct clkc_clk_info_table *rcar_clkc_find_clk_info_by_module_id(const struct device *dev,
								 uint32_t domain, uint32_t id);

void rcar_clkc_build_clock_relationship(const struct device *dev);

void rcar_clkc_update_all_in_out_freq(const struct device *dev);

int rcar_clkc_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate);

int rcar_clkc_set_rate(const struct device *dev, clock_control_subsys_t sys,
		       clock_control_subsys_rate_t rate);

#endif /* ZEPHYR_DRIVERS_RENESAS_RENESAS_CLKC_MDLC_H_ */
