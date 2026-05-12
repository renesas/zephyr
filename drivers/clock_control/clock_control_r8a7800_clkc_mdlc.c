/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_r8a7800_clkc_mdlc

#include <errno.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/r8a7800_clkc_mdlc.h>
#include "clock_control_renesas_clkc_mdlc.h"
#include <zephyr/irq.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(clock_control_rcar_clkc_mdlc);

/* Common bit fields used by many CLKC registers */
#define R8A78000_CLKC_COMMON_CLKSTP_BIT 8
#define R8A78000_CLKC_COMMON_DIV_MASK   GENMASK(5, 0)

struct r8a7800_clkc_mdlc_cfg {
	DEVICE_MMIO_ROM; /* Must be first */
};

struct r8a7800_clkc_mdlc_data {
	struct rcar_clkc_mdlc_data cmn; /* Must be first */
};

enum clk_ids {
	/* Core Clock Outputs exported to DT */
	LAST_DT_CORE_CLK = R8A78000_CLK_SASYNCCK_UFS_PERE_MAIN,

	/* Internal Core Clocks */
	CLK_PLL5,
};

/* NOTE: the array MUST be sorted by module field */
static struct clkc_clk_info_table core_props[] = {
	RCAR_CORE_CLK_INFO_ITEM(R8A78000_CLK_SGASYNCD8_PERW_BUS, RCAR_CLKC_NONE, RCAR_CLKC_NONE,
				RCAR_CLKC_NONE, 133333333),
	RCAR_CORE_CLK_INFO_ITEM(R8A78000_CLK_SGASYNCD16_PERW_BUS, RCAR_CLKC_NONE, RCAR_CLKC_NONE,
				RCAR_CLKC_NONE, 66666666),
	RCAR_CORE_CLK_INFO_ITEM(R8A78000_CLK_MSOCK_PERW_BUS, 0x1020, CLKC_REGION_SYSSS_TOP,
				CLK_PLL5, RCAR_CLKC_NONE),
	RCAR_CORE_CLK_INFO_ITEM(R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_NONE, RCAR_CLKC_NONE,
				RCAR_CLKC_NONE, 133333333),
	RCAR_CORE_CLK_INFO_ITEM(R8A78000_CLK_ADGHD1CK_MP_BUS, 0x1024, CLKC_REGION_SYSSS_TOP,
				RCAR_CLKC_NONE, RCAR_CLKC_MHZ(800)),
	RCAR_CORE_CLK_INFO_ITEM(R8A78000_CLK_ADGHD4CK_MP_BUS, 0x1024, CLKC_REGION_SYSSS_TOP,
				RCAR_CLKC_NONE, RCAR_CLKC_MHZ(200)),
	RCAR_CORE_CLK_INFO_ITEM(R8A78000_CLK_S0D24_PERE_MAIN, RCAR_CLKC_NONE, RCAR_CLKC_NONE,
				RCAR_CLKC_NONE, 33333333),
	RCAR_CORE_CLK_INFO_ITEM(CLK_PLL5, RCAR_CLKC_NONE, RCAR_CLKC_NONE, RCAR_CLKC_NONE,
				RCAR_CLKC_MHZ(3200)),
};

/* NOTE: the array MUST be sorted by module field */
static struct clkc_clk_info_table mod_props[] = {
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_PFC_PERE_GPIODM0, R8A78000_CLK_S0D24_PERE_MAIN,
			       RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SCIF0, R8A78000_CLK_SGASYNCD16_PERW_BUS,
			       RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SCIF1, R8A78000_CLK_SGASYNCD16_PERW_BUS,
			       RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SCIF3, R8A78000_CLK_SGASYNCD16_PERW_BUS,
			       RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SCIF4, R8A78000_CLK_SGASYNCD16_PERW_BUS,
			       RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_I2C1, R8A78000_CLK_SGASYNCD8_PERW_BUS,
			       RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_I2C2, R8A78000_CLK_SGASYNCD8_PERW_BUS,
			       RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_I2C3, R8A78000_CLK_SGASYNCD8_PERW_BUS,
			       RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_I2C4, R8A78000_CLK_SGASYNCD8_PERW_BUS,
			       RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_I2C5, R8A78000_CLK_SGASYNCD8_PERW_BUS,
			       RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_I2C6, R8A78000_CLK_SGASYNCD8_PERW_BUS,
			       RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_I2C7, R8A78000_CLK_SGASYNCD8_PERW_BUS,
			       RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_I2C8, R8A78000_CLK_SGASYNCD8_PERW_BUS,
			       RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_ADG0, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_ADG1, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI0, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI00, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI01, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI02, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI03, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI04, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI05, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI06, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI07, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI08, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI09, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI1, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI10, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI11, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI12, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI13, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI14, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI15, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI16, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI17, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI18, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SSI19, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SCU0, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC00, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC01, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC02, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC03, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC04, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC05, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC06, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC07, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC08, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC09, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SCU00, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SCU01, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_DVC00, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_DVC01, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SCU1, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC10, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC11, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC12, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC13, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC14, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC15, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC16, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC17, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC18, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SRC19, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SCU10, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_SCU11, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_DVC10, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
	RCAR_MOD_CLK_INFO_ITEM(R8A78000_MOD_DVC11, R8A78000_CLK_SGD8_MP_BUS, RCAR_CLKC_CAP_STOP),
};

/**
 * @brief Read the current status of an R8A7800 core clock.
 */
static enum clock_control_status
r8a7800_clkc_core_clock_get_status(const struct device *dev, struct clkc_clk_info_table *clk_info)
{
	uint32_t reg;

	if (!clk_info) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	/* Fixed clock is always on */
	if (clk_info->offset == RCAR_CLKC_NONE) {
		return CLOCK_CONTROL_STATUS_ON;
	}

	if (clk_info->region_id == RCAR_CLKC_NONE || clk_info->region_id >= CLKC_REGION_NUMS) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	switch (clk_info->module) {
	case R8A78000_CLK_MSOCK_PERW_BUS:
	case R8A78000_CLK_ADGHD1CK_MP_BUS:
	case R8A78000_CLK_ADGHD4CK_MP_BUS:
		reg = sys_read32(clkc_base[clk_info->region_id] + clk_info->offset);
		return (reg & BIT(R8A78000_CLKC_COMMON_CLKSTP_BIT)) ? CLOCK_CONTROL_STATUS_OFF
								    : CLOCK_CONTROL_STATUS_ON;
	default:
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}
}

/**
 * @brief Enable or disable an R8A7800 core clock through its CLKC control register.
 */
static int r8a7800_clkc_core_clock_enabled(const struct device *dev,
					   struct clkc_clk_info_table *clk_info, bool enable)
{
	struct r8a7800_clkc_mdlc_data *data = dev->data;
	uint32_t region;
	mem_addr_t reg_addr;
	mem_addr_t reg_base;
	uint32_t reg;
	int ret = 0;
	k_spinlock_key_t key;

	if (!clk_info) {
		return -EINVAL;
	}

	/* Fixed clock is always on */
	if (clk_info->offset == RCAR_CLKC_NONE) {
		return enable ? 0 : -ENOTSUP;
	}

	region = clk_info->region_id;
	reg_addr = clk_info->offset;

	if (region == RCAR_CLKC_NONE || region >= CLKC_REGION_NUMS) {
		return -EINVAL;
	}

	reg_base = clkc_base[region];
	key = k_spin_lock(&data->cmn.lock);
	switch (clk_info->module) {
	case R8A78000_CLK_MSOCK_PERW_BUS:
	case R8A78000_CLK_ADGHD1CK_MP_BUS:
	case R8A78000_CLK_ADGHD4CK_MP_BUS:
		reg = sys_read32(reg_base + reg_addr);
		reg &= ~(1 << R8A78000_CLKC_COMMON_CLKSTP_BIT);
		reg |= (!enable << R8A78000_CLKC_COMMON_CLKSTP_BIT);
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	if (!ret) {
		/* Disable CLKC write protection */
		ret = rcar_clkc_protect_enabled(region, false);
		if (ret) {
			goto unlock;
		}
		sys_write32(reg, reg_base + reg_addr);

		/* Enable CLKC write protection */
		ret = rcar_clkc_protect_enabled(region, true);
		if (ret) {
			goto unlock;
		}
	}

unlock:
	k_spin_unlock(&data->cmn.lock, key);
	return ret;
}

/**
 * @brief Dispatch a start or stop request to MDLC module clocks or CLKC core clocks.
 */
static int r8a7800_clkc_mdlc_start_stop(const struct device *dev, clock_control_subsys_t sys,
					bool enable)
{
	struct rcar_clkc *clk = (struct rcar_clkc *)sys;
	struct clkc_clk_info_table *clk_info;
	int ret;

	if (!dev || !sys) {
		return -EINVAL;
	}

	clk_info = rcar_clkc_find_clk_info_by_module_id(dev, clk->domain, clk->module);

	if (!clk_info) {
		return -EINVAL;
	}

	if (clk->domain == CLKC_MOD) {
		struct r8a7800_clkc_mdlc_data *data = dev->data;
		k_spinlock_key_t key;

		if (!enable && !(clk_info->capabilities & RCAR_CLKC_CAP_STOP)) {
			LOG_ERR("Clock domain %u module 0x%x does not support stop", clk->domain,
				clk->module);
			return -ENOTSUP;
		}

		key = k_spin_lock(&data->cmn.lock);
		ret = rcar_mdlc_clock_enabled(clk->module, enable);
		k_spin_unlock(&data->cmn.lock, key);
	} else if (clk->domain == CLKC_CORE) {
		ret = r8a7800_clkc_core_clock_enabled(dev, clk_info, enable);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

/**
 * @brief Return the clock status for an R8A7800 module or core clock.
 */
static enum clock_control_status r8a7800_clkc_mdlc_get_status(const struct device *dev,
							      clock_control_subsys_t sys)
{
	struct r8a7800_clkc_mdlc_data *data = dev->data;
	struct rcar_clkc *clk = (struct rcar_clkc *)sys;
	struct clkc_clk_info_table *clk_info;
	enum clock_control_status status;
	k_spinlock_key_t key;

	if (!dev || !sys) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	clk_info = rcar_clkc_find_clk_info_by_module_id(dev, clk->domain, clk->module);
	if (!clk_info) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	key = k_spin_lock(&data->cmn.lock);

	if (clk->domain == CLKC_MOD) {
		status = rcar_mdlc_clock_get_status(clk->module);
	} else if (clk->domain == CLKC_CORE) {
		status = r8a7800_clkc_core_clock_get_status(dev, clk_info);
	} else {
		status = CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	k_spin_unlock(&data->cmn.lock, key);

	return status;
}

/**
 * @brief Decode an R8A7800 clock divider register field into the effective divider.
 */
static uint32_t r8a7800_get_div_helper(uint32_t reg_val, uint32_t module)
{
	switch (module) {
	case R8A78000_CLK_ADGHD1CK_MP_BUS:
	case R8A78000_CLK_ADGHD4CK_MP_BUS:
		return 1;
	case R8A78000_CLK_MSOCK_PERW_BUS:
		reg_val = FIELD_GET(R8A78000_CLKC_COMMON_DIV_MASK, reg_val);
		/* Setting other than 5 to 63 is prohibited */
		if (reg_val < 5U || reg_val > 63U) {
			LOG_ERR("MSOCK_PERW_BUS clock has invalid divider value %u:", reg_val);
			return RCAR_CLKC_NONE;
		}

		/* Return the actual divider value */
		return 4U * (reg_val + 1U);
	default:
		return RCAR_CLKC_NONE;
	}
}

/**
 * @brief Validate an R8A7800 effective divider and convert it to the register field value.
 */
static int r8a7800_set_rate_helper(uint32_t module, uint32_t *divider, uint32_t *div_mask)
{
	uint32_t reg_val;

	switch (module) {
	case R8A78000_CLK_MSOCK_PERW_BUS:
		if ((*divider % 4U) != 0U) {
			LOG_ERR("The requested divider is not valid");
			return -EINVAL;
		}

		reg_val = (*divider / 4U) - 1U;

		/* Setting other than 5 to 63 is prohibited */
		if (reg_val < 5U || reg_val > 63U) {
			LOG_ERR("The requested divider is not valid");
			return -EINVAL;
		}

		*divider = FIELD_PREP(R8A78000_CLKC_COMMON_DIV_MASK, reg_val);
		*div_mask = R8A78000_CLKC_COMMON_DIV_MASK;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int r8a7800_clkc_mdlc_start(const struct device *dev, clock_control_subsys_t sys)
{
	return r8a7800_clkc_mdlc_start_stop(dev, sys, true);
}

static int r8a7800_clkc_mdlc_stop(const struct device *dev, clock_control_subsys_t sys)
{
	return r8a7800_clkc_mdlc_start_stop(dev, sys, false);
}

/**
 * @brief Initialize the R8A7800 CLKC/MDLC device and precompute clock relationships.
 */
static int r8a7800_clkc_mdlc_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	rcar_clkc_build_clock_relationship(dev);
	rcar_clkc_update_all_in_out_freq(dev);
	return 0;
}

static DEVICE_API(clock_control, r8a7800_clkc_mdlc_api) = {
	.on = r8a7800_clkc_mdlc_start,
	.off = r8a7800_clkc_mdlc_stop,
	.get_rate = rcar_clkc_get_rate,
	.set_rate = rcar_clkc_set_rate,
	.get_status = r8a7800_clkc_mdlc_get_status,
};

#define R8A7800_MDLC_INIT(inst)                                                                    \
	static struct r8a7800_clkc_mdlc_cfg clkc_mdlc##inst##_cfg = {                              \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
	};                                                                                         \
                                                                                                   \
	static struct r8a7800_clkc_mdlc_data clkc_mdlc##inst##_data = {                            \
		.cmn.clk_info_table[CLKC_CORE] = core_props,                                       \
		.cmn.clk_info_table_size[CLKC_CORE] = ARRAY_SIZE(core_props),                      \
		.cmn.clk_info_table[CLKC_MOD] = mod_props,                                         \
		.cmn.clk_info_table_size[CLKC_MOD] = ARRAY_SIZE(mod_props),                        \
		.cmn.get_div_helper = r8a7800_get_div_helper,                                      \
		.cmn.set_rate_helper = r8a7800_set_rate_helper,                                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &r8a7800_clkc_mdlc_init, NULL, &clkc_mdlc##inst##_data,        \
			      &clkc_mdlc##inst##_cfg, PRE_KERNEL_1,                                \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &r8a7800_clkc_mdlc_api);

DT_INST_FOREACH_STATUS_OKAY(R8A7800_MDLC_INIT)
