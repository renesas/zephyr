/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdlib.h>
#include "clock_control_renesas_clkc_mdlc.h"
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/r8a7800_clkc_mdlc.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_rcar_clkc_mdlc);

#define MDLC_MSRES_STATE_STANDBY 0x0U
#define MDLC_MSRES_STATE_RESET   0x1U
#define MDLC_MSRES_STATE_STOP    0x2U
#define MDLC_MSRES_STATE_ON      0x3U

#define MDLC_MSRES_FIELD_MASK  0x3U
#define MDLC_MSRES_TIMEOUT_US  1000U
#define CLKC_RATE_TOLERANCE_HZ 1000U

/**
 * @brief Extract the 2-bit state for one module slot from an MDLC MSRES/MSRESS register value
 */
static uint32_t rcar_mdlc_get_slot_state(uint32_t reg_val, uint32_t slot)
{
	uint32_t val = (reg_val >> (slot * 2U)) & MDLC_MSRES_FIELD_MASK;

	return val;
}

/**
 * @brief Wait until one module slot reaches the requested MDLC status state.
 */
static bool rcar_mdlc_wait_state(mem_addr_t base, uint32_t index, uint32_t slot, uint32_t state)
{
	return WAIT_FOR(
		(rcar_mdlc_get_slot_state(sys_read32(base + MDLCMSRESS(index)), slot) == state),
		MDLC_MSRES_TIMEOUT_US, k_busy_wait(1));
}

/**
 * @brief Program one module slot in the MDLC control register to the requested state.
 */
static void rcar_mdlc_set_state(mem_addr_t base, uint32_t index, uint32_t shift, uint32_t state)
{
	uint32_t reg;

	reg = sys_read32(base + MDLCMRES(index));
	reg &= ~(MDLC_MSRES_FIELD_MASK << shift);
	reg |= state << shift;
	sys_write32(reg, base + MDLCMRES(index));
}

/**
 * @brief Return the clock control status for a module clock based on the MDLC MSRESS register
 * values
 */
enum clock_control_status rcar_mdlc_clock_get_status(uint32_t module)
{
	uint32_t region = R8A78000_MOD_REGION(module);
	uint32_t index = R8A78000_MOD_REG_IDX(module);
	uint32_t slot = R8A78000_MOD_SLOT(module);
	uint32_t state;

	if (region >= MDLC_REGION_NUMS || index >= MDLC_INDEX_NUMS || slot >= MDLC_SLOT_NUMS) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	state = rcar_mdlc_get_slot_state(sys_read32(mdlc_base[region] + MDLCMSRESS(index)), slot);

	switch (state) {
	case MDLC_MSRES_STATE_STOP:
		return CLOCK_CONTROL_STATUS_OFF;
	case MDLC_MSRES_STATE_ON:
		return CLOCK_CONTROL_STATUS_ON;
	default:
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}
}

/**
 * @brief Set MDLC write protection state for a region
 *
 * When @p enabled is true, register writes are protected.
 * When @p enabled is false, register writes are allowed.
 */
static int rcar_mdlc_protect_enabled(uint32_t region, bool enabled)
{
	if (region >= MDLC_REGION_NUMS) {
		LOG_ERR("Invalid region 0x%x for MDLC", region);
		return -EINVAL;
	}

	if (enabled) {
		sys_write32(CLKC_MDLC_WPR_EN, mdlc_base[region] + MDLCPKCPROT1);
	} else {
		sys_write32(CLKC_MDLC_WPR_DIS, mdlc_base[region] + MDLCPKCPROT1);
	}

	return 0;
}

/**
 * @brief Set CLKC write protection state for a region
 *
 * When @p enabled is true, register writes are protected.
 * When @p enabled is false, register writes are allowed.
 */
int rcar_clkc_protect_enabled(uint32_t region, bool enabled)
{
	if (region >= CLKC_REGION_NUMS) {
		LOG_ERR("Invalid region 0x%x for CLKC", region);
		return -EINVAL;
	}

	if (enabled) {
		sys_write32(CLKC_MDLC_WPR_EN, clkc_base[region] + CLKCKCPROT);
	} else {
		sys_write32(CLKC_MDLC_WPR_DIS, clkc_base[region] + CLKCKCPROT);
	}

	return 0;
}

/**
 * @brief Enable or disable a module clock via MDLC
 */
int rcar_mdlc_clock_enabled(uint32_t module, bool enabled)
{
	uint32_t region = R8A78000_MOD_REGION(module);
	uint32_t index = R8A78000_MOD_REG_IDX(module);
	uint32_t slot = R8A78000_MOD_SLOT(module);
	uint32_t shift;
	mem_addr_t base;
	uint32_t state;
	uint32_t current_state;
	uint32_t intermediate_state;
	int ret;
	bool stable;

	if (region >= MDLC_REGION_NUMS || index >= MDLC_INDEX_NUMS || slot >= MDLC_SLOT_NUMS) {
		LOG_ERR("Invalid module ID 0x%x for MDLC", module);
		return -EINVAL;
	}

	shift = R8A78000_MOD_SHIFT(module);
	base = mdlc_base[region];
	state = enabled ? MDLC_MSRES_STATE_ON : MDLC_MSRES_STATE_STOP;

	/* Wait until the current control field is reflected in the status field. */
	stable = WAIT_FOR((rcar_mdlc_get_slot_state(sys_read32(base + MDLCMSRESS(index)), slot) ==
			   rcar_mdlc_get_slot_state(sys_read32(base + MDLCMRES(index)), slot)),
			  MDLC_MSRES_TIMEOUT_US, k_busy_wait(1));
	if (!stable) {
		LOG_ERR("MDLC state did not stabilize for module ID 0x%x", module);
		return -ETIMEDOUT;
	}

	current_state = rcar_mdlc_get_slot_state(sys_read32(base + MDLCMSRESS(index)), slot);
	if (current_state == state) {
		/* Condition already met */
		return 0;
	}

	/* Disable MDLC write protection */
	ret = rcar_mdlc_protect_enabled(region, false);
	if (ret) {
		return ret;
	}

	/*
	 * To suppress excessive current fluctuations, some MDLC module standby state
	 * transitions must pass through an intermediate state.
	 */
	intermediate_state = state;
	if (current_state == MDLC_MSRES_STATE_STANDBY && state == MDLC_MSRES_STATE_ON) {
		intermediate_state = MDLC_MSRES_STATE_RESET;
	} else if (current_state == MDLC_MSRES_STATE_RESET && state == MDLC_MSRES_STATE_STOP) {
		intermediate_state = MDLC_MSRES_STATE_ON;
	}

	rcar_mdlc_set_state(base, index, shift, intermediate_state);

	if (intermediate_state != state) {
		stable = rcar_mdlc_wait_state(base, index, slot, intermediate_state);
		if (!stable) {
			LOG_ERR("MDLC intermediate state update timed out for module ID 0x%x",
				module);
			goto protect_enable;
		}

		rcar_mdlc_set_state(base, index, shift, state);
	}

	/* Wait until hardware status reports the requested state for this slot. */
	stable = rcar_mdlc_wait_state(base, index, slot, state);

protect_enable:
	/* Enable MDLC write protection */
	ret = rcar_mdlc_protect_enabled(region, true);
	if (ret) {
		return ret;
	}

	if (!stable) {
		LOG_ERR("MDLC state update timed out for module ID 0x%x", module);
		return -ETIMEDOUT;
	}

	return 0;
}

/**
 * @brief Compare clock table entries by module ID for bsearch().
 */
static int cmp_clkc_clk_info_table_items(const void *key, const void *element)
{
	const struct clkc_clk_info_table *e = element;
	uint32_t module = (uintptr_t)key;

	if (e->module == module) {
		return 0;
	} else if (e->module < module) {
		return 1;
	} else {
		return -1;
	}
}

/**
 * @brief Find one clock description in a core or module clock table.
 */
struct clkc_clk_info_table *rcar_clkc_find_clk_info_by_module_id(const struct device *dev,
								 uint32_t domain, uint32_t id)
{
	struct rcar_clkc_mdlc_data *data = dev->data;
	struct clkc_clk_info_table *item;
	struct clkc_clk_info_table *table;
	uint32_t table_size;
	uintptr_t uintptr_id = id;

	if (domain >= CLKC_NUM_DOMAINS) {
		LOG_ERR("%s: invalid clock domain %u for module 0x%x", dev->name, domain, id);
		return NULL;
	}

	table = data->clk_info_table[domain];
	table_size = data->clk_info_table_size[domain];

	item = bsearch((void *)uintptr_id, table, table_size, sizeof(*item),
		       cmp_clkc_clk_info_table_items);
	if (!item) {
		LOG_ERR("%s: can't find clk info (domain %u module 0x%x)", dev->name, domain, id);
	}

	return item;
}

/**
 * @brief Link each clock table entry to its parent and children.
 */
void rcar_clkc_build_clock_relationship(const struct device *dev)
{
	uint32_t domain;
	k_spinlock_key_t key;
	struct rcar_clkc_mdlc_data *data = dev->data;

	if (!data) {
		return;
	}

	key = k_spin_lock(&data->lock);
	for (domain = 0; domain < CLKC_NUM_DOMAINS; domain++) {
		uint32_t idx;
		uint32_t prev_mod_id = 0;
		struct clkc_clk_info_table *item = data->clk_info_table[domain];

		for (idx = 0; idx < data->clk_info_table_size[domain]; idx++, item++) {
			struct clkc_clk_info_table *parent;

			/* check if an array is sorted by module id or not */
			if (prev_mod_id >= item->module) {
				LOG_ERR("%s: clocks have to be sorted inside clock table in "
					"ascending order by module id field, domain %u "
					"module id 0x%x",
					dev->name, item->domain, item->module);
				k_panic();
			}

			prev_mod_id = item->module;

			if (item->parent_id == RCAR_CLKC_NONE) {
				continue;
			}

			parent = rcar_clkc_find_clk_info_by_module_id(dev, CLKC_CORE,
								      item->parent_id);
			if (!parent) {
				LOG_ERR("%s: can't find parent for clock with valid parent id, "
					"domain %u module id 0x%x",
					dev->name, item->domain, item->module);
				k_panic();
			}

			if (item->parent != NULL) {
				LOG_ERR("%s: trying to set another parent for a clock, domain %u "
					"module id %u, parent for the clock has been already set",
					dev->name, item->domain, item->module);
				k_panic();
			}

			item->parent = parent;

			/* insert in the head of the children list of the parent */
			item->next_sibling = parent->children_list;
			parent->children_list = item;
		}
	}
	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Return the effective divider for a fixed, module, or register-backed clock.
 */
static uint32_t rcar_clkc_get_divider(const struct device *dev,
				      struct clkc_clk_info_table *clk_info)
{
	mem_addr_t reg_addr;
	mm_reg_t reg_val;
	uint32_t divider = RCAR_CLKC_NONE;
	struct rcar_clkc_mdlc_data *data = dev->data;

	if (clk_info->domain == CLKC_MOD) {
		return 1;
	}

	reg_addr = clk_info->offset;
	if (reg_addr == RCAR_CLKC_NONE) {
		/* if we don't have valid offset, this clock is fixed and the divider is 1 */
		return 1;
	}

	reg_addr += clkc_base[clk_info->region_id];
	reg_val = sys_read32(reg_addr);

	if (data->get_div_helper) {
		divider = data->get_div_helper(reg_val, clk_info->module);
	}

	if (!divider) {
		return RCAR_CLKC_NONE;
	}

	return divider;
}

/**
 * @brief Refresh one clock's cached output frequency from its input frequency and divider.
 */
static int rcar_clkc_update_out_freq(const struct device *dev, struct clkc_clk_info_table *clk_info)
{
	uint32_t divider = rcar_clkc_get_divider(dev, clk_info);

	if (divider == RCAR_CLKC_NONE) {
		return -EINVAL;
	}

	clk_info->out_freq = clk_info->in_freq / divider;
	return 0;
}

/**
 * @brief Resolve a clock's input frequency and update its cached output frequency.
 */
static int64_t rcar_clkc_get_in_update_out_freq(const struct device *dev,
						struct clkc_clk_info_table *clk_info)
{
	int64_t freq = -ENOTSUP;
	struct clkc_clk_info_table *parent_clk;

	if (!clk_info) {
		return freq;
	}

	if (clk_info->in_freq != RCAR_CLKC_NONE) {
		if (clk_info->out_freq == RCAR_CLKC_NONE) {
			if (rcar_clkc_update_out_freq(dev, clk_info) < 0) {
				return freq;
			}
		}
		return clk_info->in_freq;
	}

	parent_clk = clk_info->parent;

	freq = rcar_clkc_get_in_update_out_freq(dev, parent_clk);
	if (freq < 0) {
		return freq;
	}

	clk_info->in_freq = parent_clk->out_freq;

	freq = rcar_clkc_update_out_freq(dev, clk_info);
	if (freq < 0) {
		return freq;
	}

	return clk_info->in_freq;
}

/**
 * @brief Return a clock's cached output frequency, updating it if needed.
 */
static int64_t rcar_clkc_get_out_freq(const struct device *dev,
				      struct clkc_clk_info_table *clk_info)
{
	int64_t freq;

	if (clk_info->out_freq != RCAR_CLKC_NONE) {
		return clk_info->out_freq;
	}

	freq = rcar_clkc_get_in_update_out_freq(dev, clk_info);
	if (freq < 0) {
		return freq;
	}

	return clk_info->out_freq;
}

/**
 * @brief Propagate a parent clock rate change through all child clocks.
 */
static void rcar_clkc_change_children_in_out_freq(const struct device *dev,
						  struct clkc_clk_info_table *parent)
{
	struct clkc_clk_info_table *children_list = parent->children_list;

	while (children_list) {
		children_list->in_freq = parent->out_freq;

		if (rcar_clkc_update_out_freq(dev, children_list) < 0) {
			LOG_ERR("%s: error during getting divider from clock register, domain %u "
				"module 0x%x! Please, revise logic related to obtaining divider or "
				"check presentence of clock inside appropriate clk_info_table",
				dev->name, children_list->domain, children_list->module);
			k_panic();
			return;
		}

		/* child can have childrens */
		rcar_clkc_change_children_in_out_freq(dev, children_list);
		children_list = children_list->next_sibling;
	}
}

/**
 * @brief Clock-control API helper for reading a core or module clock rate.
 */
int rcar_clkc_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	int64_t ret;
	struct rcar_clkc_mdlc_data *data;
	struct rcar_clkc *clk = (struct rcar_clkc *)sys;
	k_spinlock_key_t key;

	struct clkc_clk_info_table *clk_info;

	if (!dev || !sys || !rate) {
		LOG_ERR("%s: received null ptr input arg(s) dev %p sys %p rate %p", __func__, dev,
			sys, rate);
		return -EINVAL;
	}

	clk_info = rcar_clkc_find_clk_info_by_module_id(dev, clk->domain, clk->module);
	if (clk_info == NULL) {
		return -EINVAL;
	}

	data = dev->data;

	key = k_spin_lock(&data->lock);
	ret = rcar_clkc_get_out_freq(dev, clk_info);
	k_spin_unlock(&data->lock, key);

	if (ret < 0) {
		LOG_ERR("%s: clk (domain %u module 0x%x) error (%lld) during getting out frequency",
			dev->name, clk->domain, clk->module, ret);
		return -EINVAL;
	} else if (ret > UINT_MAX) {
		LOG_ERR("%s: clk (domain %u module 0x%x) frequency bigger then max uint value",
			dev->name, clk->domain, clk->module);
		return -EINVAL;
	}

	*rate = ret;
	return 0;
}

/**
 * @brief Precompute input and output frequencies for all known clocks.
 */
void rcar_clkc_update_all_in_out_freq(const struct device *dev)
{
	uint32_t domain;
	k_spinlock_key_t key;
	struct rcar_clkc_mdlc_data *data = dev->data;

	if (!data) {
		return;
	}

	key = k_spin_lock(&data->lock);
	for (domain = 0; domain < CLKC_NUM_DOMAINS; domain++) {
		uint32_t idx;
		struct clkc_clk_info_table *item = data->clk_info_table[domain];

		for (idx = 0; idx < data->clk_info_table_size[domain]; idx++, item++) {
			if (rcar_clkc_get_in_update_out_freq(dev, item) < 0) {
				LOG_ERR("%s: can't update in/out freq for clock during init, "
					"domain %u module 0x%x! Please, review correctness of data "
					"inside clk_info_table",
					dev->name, item->domain, item->module);
				k_panic();
			}
		}
	}
	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Clock-control API helper for programming a core clock rate.
 */
int rcar_clkc_set_rate(const struct device *dev, clock_control_subsys_t sys,
		       clock_control_subsys_rate_t rate)
{
	int ret = -ENOTSUP;
	k_spinlock_key_t key;
	struct clkc_clk_info_table *clk_info;
	struct rcar_clkc *clk = (struct rcar_clkc *)sys;
	struct rcar_clkc_mdlc_data *data;
	int64_t in_freq;
	uint32_t region;
	mem_addr_t reg_addr;
	uint32_t divider;
	uint32_t requested_divider;
	uint32_t div_mask;
	uint32_t module;
	uint64_t actual_rate;
	uint64_t rate_diff;
	uintptr_t u_rate = (uintptr_t)rate;

	if (!dev || !sys || !rate) {
		LOG_ERR("%s: received null ptr input arg(s) dev %p sys %p rate %p", __func__, dev,
			sys, rate);
		return -EINVAL;
	}

	clk_info = rcar_clkc_find_clk_info_by_module_id(dev, clk->domain, clk->module);
	if (clk_info == NULL) {
		return -EINVAL;
	}

	if (clk_info->domain == CLKC_MOD) {
		if (!clk_info->parent) {
			LOG_ERR("%s: parent isn't present for module clock, module id 0x%x",
				dev->name, clk_info->module);
			k_panic();
		}
		clk_info = clk_info->parent;
	}

	module = clk_info->module;
	data = dev->data;
	region = clk_info->region_id;

	if (clk_info->offset == RCAR_CLKC_NONE || region == RCAR_CLKC_NONE ||
	    region >= CLKC_REGION_NUMS) {
		return -ENOTSUP;
	}

	reg_addr = clkc_base[region] + clk_info->offset;

	key = k_spin_lock(&data->lock);
	in_freq = rcar_clkc_get_in_update_out_freq(dev, clk_info);
	if (in_freq < 0) {
		ret = in_freq;
		goto unlock;
	}

	divider = DIV_ROUND_CLOSEST(in_freq, u_rate);
	if (divider == 0U) {
		ret = -EINVAL;
		goto unlock;
	}
	requested_divider = divider;

	if (!data->set_rate_helper) {
		ret = -ENOTSUP;
		goto unlock;
	}

	ret = data->set_rate_helper(module, &divider, &div_mask);
	if (!ret) {
		uint32_t reg;

		actual_rate = (uint64_t)in_freq / requested_divider;
		rate_diff =
			(actual_rate > u_rate) ? (actual_rate - u_rate) : (u_rate - actual_rate);
		if (rate_diff > CLKC_RATE_TOLERANCE_HZ) {
			ret = -EINVAL;
			LOG_ERR("%s: clock (domain %u module 0x%x) calculated freq (%llu) "
				"is outside tolerance from requested %lu",
				dev->name, clk->domain, clk->module,
				(unsigned long long)actual_rate, u_rate);
			goto unlock;
		}

		reg = sys_read32(reg_addr);
		reg &= ~div_mask;

		ret = rcar_clkc_protect_enabled(region, false);
		if (ret) {
			goto unlock;
		}

		sys_write32(reg | divider, reg_addr);

		ret = rcar_clkc_protect_enabled(region, true);
		if (ret) {
			goto unlock;
		}

		clk_info->out_freq = actual_rate;
		rcar_clkc_change_children_in_out_freq(dev, clk_info);
	}

unlock:
	k_spin_unlock(&data->lock, key);
	return ret;
}
