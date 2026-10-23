/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_cgc

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <zephyr/dt-bindings/clock/renesas_rh850_clock_common.h>
#include <zephyr/drivers/clock_control/renesas_rh850_cgc.h>
#include "rp_bsp_clock_control.h"
#include "bsp_clock_cfg.h"

/*
 * RH850 CGC driver responsibilities
 * ---------------------------------
 * The Zephyr layer decodes the clock subsystem ID, validates the requested
 * domain, serializes API operations, and dispatches to the portable RH850 HAL.
 *
 * PCLK subsystem fields:
 *   module  - logical RH850 module-standby group
 *   channel - bit index in the corresponding module-standby register
 *   clock   - logical peripheral functional-clock identifier
 *
 * SoC-specific module mappings, clock mappings, selector/divider
 * register access, and runtime clock-rate calculation are intentionally kept
 * in the portable HAL. This keeps this source common for the RH850 family.
 */

/*
 * PCLK channel is an MSR bit index, not a count of supported modules.
 * Each RH850 module standby register is 32 bits wide, so valid channel
 * values are 0..31. Future modules spanning multiple MSR registers must be
 * represented as separate module IDs/groups rather than increasing this limit.
 */
#define RH850_CGC_MSR_BIT_MAX 31U

/* DT uses physical division factors; runtime payloads retain HAL encodings. */
struct rh850_cgc_pclk_dt_cfg {
	uint32_t clock_id;
	uint32_t source_clock_id;
	uint32_t div;
};

struct rh850_cgc_config {
	const cgc_api_t *fsp_api;
	const struct rh850_cgc_pclk_dt_cfg *pclk_cfgs;
	size_t pclk_cfg_count;
};

struct rh850_cgc_data {
	cgc_instance_ctrl_t cgc_ctrl;
	atomic_t open_state;
	uint32_t pclk_cfg_applied_mask;
	struct k_spinlock lock;
};

enum rh850_cgc_open_state {
	RH850_CGC_OPEN_STATE_CLOSED,
	RH850_CGC_OPEN_STATE_OPENING,
	RH850_CGC_OPEN_STATE_OPEN,
};

/** Zephyr clock_control_get_status() callback. */
static enum clock_control_status clock_control_rh850_cgc_get_status(const struct device *dev,
								    clock_control_subsys_t sys);

/* Only clocks with a portable runtime selector/divider API are replayed.
 * CPU and bus ratios remain part of the BSP/option-byte clock profile.
 */
#define RH850_CGC_PCLK_NODE(label, fn) \
	COND_CODE_1(DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(label), \
					    renesas_rh850_cgc_pclk, okay), \
		    (fn(DT_NODELABEL(label))), ())
#define RH850_CGC_FOREACH_PCLK(fn) \
	FOR_EACH_FIXED_ARG(RH850_CGC_PCLK_NODE, (), fn, \
		clk_mspi, clk_rlin3, clk_rlin3_ch23, clk_rcanosc, clk_canfd_c, clk_canfd_xin, \
		clk_adc, clka_adc, clk_wdt, clka_wdtb, clk_wdtb, clka_wdt, clka_wdtba, \
		clka_tauj, clka_rtca, clk_swdt, clkc_canxl_ch0, clkc_canxl_ch1, extclk0, extclk1)

#define RH850_CGC_SOURCE_CLOCK_ID(node_id) \
	BSP_CLOCK_DT_ID(DT_PHANDLE(node_id, clock_source))

#define RH850_CGC_VALIDATE_PCLK_SOURCE(node_id) \
	BUILD_ASSERT(BSP_CLOCK_DT_ID(node_id) > RH850_CLOCK_NONE, \
		     "Unsupported RH850 peripheral clock node"); \
	BUILD_ASSERT(RH850_CGC_SOURCE_CLOCK_ID(node_id) > RH850_CLOCK_NONE, \
		     "Unsupported RH850 peripheral clock-source node");

RH850_CGC_FOREACH_PCLK(RH850_CGC_VALIDATE_PCLK_SOURCE)

#define RH850_CGC_PCLK_CFG_INIT(node_id) \
	{ \
		.clock_id = BSP_CLOCK_DT_ID(node_id), \
		.source_clock_id = RH850_CGC_SOURCE_CLOCK_ID(node_id), \
		.div = DT_PROP(node_id, div), \
	},

static const struct rh850_cgc_pclk_dt_cfg rh850_cgc_pclk_cfgs[] = {
	RH850_CGC_FOREACH_PCLK(RH850_CGC_PCLK_CFG_INIT)
};

static int rh850_cgc_fsp_err_to_errno(fsp_err_t err)
{
	switch (err) {
	case FSP_SUCCESS:
		return 0;
	case FSP_ERR_UNSUPPORTED:
		return -ENOTSUP;
	case FSP_ERR_INVALID_ARGUMENT:
	case FSP_ERR_ASSERTION:
	case FSP_ERR_CGC_INVALID_CLOCK_SOURCE:
		return -EINVAL;
	case FSP_ERR_ALREADY_OPEN:
		return -EALREADY;
	case FSP_ERR_NOT_OPEN:
		return -EACCES;
	case FSP_ERR_CGC_CLOCK_INACTIVE:
		return -EIO;
	case FSP_ERR_CGC_CLOCK_IN_USE:
		return -EBUSY;
	case FSP_ERR_CGC_NOT_STABILIZED:
		return -EAGAIN;
	default:
		return -EIO;
	}
}

static uint32_t rh850_cgc_subsys_id(clock_control_subsys_t sys)
{
	return (uint32_t)(uintptr_t)sys;
}

static uint32_t rh850_cgc_subsys_domain(clock_control_subsys_t sys)
{
	return RH850_CGC_DOMAIN(rh850_cgc_subsys_id(sys));
}

static uint32_t rh850_cgc_subsys_module(clock_control_subsys_t sys)
{
	return RH850_CGC_MODULE(rh850_cgc_subsys_id(sys));
}

static uint32_t rh850_cgc_subsys_channel(clock_control_subsys_t sys)
{
	return RH850_CGC_CHANNEL(rh850_cgc_subsys_id(sys));
}

static uint32_t rh850_cgc_subsys_clock(clock_control_subsys_t sys)
{
	return RH850_CGC_CLOCK(rh850_cgc_subsys_id(sys));
}

static int rh850_cgc_clock_to_cgc(uint32_t clock_id, cgc_clock_t *cgc_clock)
{
	switch (clock_id) {
	case RH850_CLOCK_HSIOSC:
	case RH850_CLOCK_IOSC:
		*cgc_clock = CGC_CLOCK_HOCO;
		return 0;
	case RH850_CLOCK_LSIOSC:
		*cgc_clock = CGC_CLOCK_LOCO;
		return 0;
	case RH850_CLOCK_MOSC:
		*cgc_clock = CGC_CLOCK_MAIN_OSC;
		return 0;
	case RH850_CLOCK_PLL:
	case RH850_CLOCK_PLLO:
		/* FSP U2x exposes PLL lock/status for the PLL output path. */
		*cgc_clock = CGC_CLOCK_PLL;
		return 0;
#if defined(BSP_FEATURE_CGC_HAS_CLK_SYS_SSCG) && BSP_FEATURE_CGC_HAS_CLK_SYS_SSCG
	case RH850_CLOCK_SSCG:
	case RH850_CLOCK_SSCG1:
		/* FSP U2B exposes one CGC_CLOCK_SSCG for SSCG/SSCG1. */
		*cgc_clock = CGC_CLOCK_SSCG;
		return 0;
#endif
#if defined(BSP_FEATURE_CGC_SUB_OSC_IS_AVAILABLE) && BSP_FEATURE_CGC_SUB_OSC_IS_AVAILABLE
	case RH850_CLOCK_SOSC:
		*cgc_clock = CGC_CLOCK_SUBCLOCK;
		return 0;
#endif
	default:
		return -EINVAL;
	}
}

static int rh850_cgc_open(const struct device *dev)
{
	const struct rh850_cgc_config *config = dev->config;
	struct rh850_cgc_data *data = dev->data;
	const cgc_cfg_t cfg = {0};
	fsp_err_t err;

	for (;;) {
		atomic_val_t state = atomic_get(&data->open_state);

		if (state == RH850_CGC_OPEN_STATE_OPEN) {
			return 0;
		}

		if (state == RH850_CGC_OPEN_STATE_CLOSED &&
		    atomic_cas(&data->open_state, RH850_CGC_OPEN_STATE_CLOSED,
			       RH850_CGC_OPEN_STATE_OPENING)) {
			break;
		}

		/* Another caller is opening the shared FSP control block. */
		k_busy_wait(1);
	}

	err = config->fsp_api->open((cgc_ctrl_t *)&data->cgc_ctrl, &cfg);
	if ((err == FSP_SUCCESS) || (err == FSP_ERR_ALREADY_OPEN)) {
		atomic_set(&data->open_state, RH850_CGC_OPEN_STATE_OPEN);
		return 0;
	}

	atomic_set(&data->open_state, RH850_CGC_OPEN_STATE_CLOSED);
	return rh850_cgc_fsp_err_to_errno(err);
}

/** Enable one peripheral module gate through the portable HAL. */
static int rh850_cgc_pclk_gate_on(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t module = rh850_cgc_subsys_module(sys);
	uint32_t channel = rh850_cgc_subsys_channel(sys);
	fsp_err_t err;

	ARG_UNUSED(dev);

	if (channel > RH850_CGC_MSR_BIT_MAX) {
		return -EINVAL;
	}

	err = RP_BSP_ModuleStandbyEnableById(module, channel);
	return rh850_cgc_fsp_err_to_errno(err);
}

/** Disable one peripheral module gate through the portable HAL. */
static int rh850_cgc_pclk_gate_off(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t module = rh850_cgc_subsys_module(sys);
	uint32_t channel = rh850_cgc_subsys_channel(sys);
	fsp_err_t err;

	ARG_UNUSED(dev);

	if (channel > RH850_CGC_MSR_BIT_MAX) {
		return -EINVAL;
	}

	err = RP_BSP_ModuleStandbyDisableById(module, channel);
	return rh850_cgc_fsp_err_to_errno(err);
}

/** Return the current MSR gate status for one peripheral channel. */
static enum clock_control_status rh850_cgc_pclk_gate_status(const struct device *dev,
							    clock_control_subsys_t sys)
{
	uint32_t module = rh850_cgc_subsys_module(sys);
	uint32_t channel = rh850_cgc_subsys_channel(sys);
	bool module_standby;
	fsp_err_t err;

	ARG_UNUSED(dev);

	if (channel > RH850_CGC_MSR_BIT_MAX) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	err = RP_BSP_ModuleStandbyStatusGetById(module, channel, &module_standby);
	if (err != FSP_SUCCESS) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	return module_standby ? CLOCK_CONTROL_STATUS_OFF : CLOCK_CONTROL_STATUS_ON;
}

/** Apply one DTS/runtime peripheral selector and divider configuration. */
static int rh850_cgc_apply_pclk_cfg(const struct rh850_cgc_pclk_configure *cfg)
{
	fsp_err_t err;

	err = RP_BSP_PeripheralClockConfigure(cfg->clock_id, cfg->source_clock_id, cfg->div);

	return rh850_cgc_fsp_err_to_errno(err);
}

static const struct rh850_cgc_pclk_dt_cfg *
rh850_cgc_find_pclk_cfg(const struct rh850_cgc_config *config, uint32_t clock_id, size_t *index)
{
	for (size_t i = 0; i < config->pclk_cfg_count; i++) {
		if (config->pclk_cfgs[i].clock_id == clock_id) {
			if (index != NULL) {
				*index = i;
			}

			return &config->pclk_cfgs[i];
		}
	}

	return NULL;
}

static int rh850_cgc_apply_pclk_cfg_from_dts(const struct device *dev, uint32_t clock_id)
{
	const struct rh850_cgc_config *config = dev->config;
	struct rh850_cgc_data *data = dev->data;
	const struct rh850_cgc_pclk_dt_cfg *cfg;
	struct rh850_cgc_pclk_configure raw_cfg;
	fsp_err_t err;
	size_t index;
	uint32_t bit = 0U;
	k_spinlock_key_t key;
	int ret;

	cfg = rh850_cgc_find_pclk_cfg(config, clock_id, &index);
	if (cfg == NULL) {
		return 0;
	}

	err = RP_BSP_PeripheralClockDividerEncode(cfg->clock_id, cfg->source_clock_id,
						 cfg->div, &raw_cfg.div);
	if (err != FSP_SUCCESS) {
		return rh850_cgc_fsp_err_to_errno(err);
	}
	raw_cfg.clock_id = cfg->clock_id;
	raw_cfg.source_clock_id = cfg->source_clock_id;

	bit = BIT(index);
	key = k_spin_lock(&data->lock);

	/* Avoid reprogramming a selector/divider already applied by this driver. */
	if ((data->pclk_cfg_applied_mask & bit) != 0U) {
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	ret = rh850_cgc_apply_pclk_cfg(&raw_cfg);
	if (ret == 0) {
		data->pclk_cfg_applied_mask |= bit;
	}

	k_spin_unlock(&data->lock, key);
	return ret;
}

/** Zephyr clock_control_on() callback. */
static int clock_control_rh850_cgc_on(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t domain = rh850_cgc_subsys_domain(sys);
	int ret;

	if (domain == RH850_CGC_DOMAIN_PCLK) {
		/*
		 * Do not replay all DTS pclk nodes at CGC init. Apply the
		 * matching DTS source/divider only when a consumer requests this
		 * clock through clock_control_on(). This keeps R_BSP_InitClock() as
		 * the boot baseline while still allowing module drivers to opt in to
		 * DTS-driven peripheral clock source/divider configuration.
		 */
		ret = rh850_cgc_apply_pclk_cfg_from_dts(dev, rh850_cgc_subsys_clock(sys));
		if (ret < 0) {
			return ret;
		}

		return rh850_cgc_pclk_gate_on(dev, sys);
	}

	if (domain == RH850_CGC_DOMAIN_OSC) {
		const struct rh850_cgc_config *config = dev->config;
		struct rh850_cgc_data *data = dev->data;
		cgc_clock_t cgc_clock_id;
		fsp_err_t err;

		ret = rh850_cgc_open(dev);
		if (ret < 0) {
			return ret;
		}

		ret = rh850_cgc_clock_to_cgc(rh850_cgc_subsys_clock(sys), &cgc_clock_id);
		if (ret < 0) {
			return ret;
		}

		/* Internal oscillators already run; verify readiness instead of starting. */
		if ((cgc_clock_id == CGC_CLOCK_HOCO) || (cgc_clock_id == CGC_CLOCK_LOCO)) {
			k_spinlock_key_t key = k_spin_lock(&data->lock);

			err = config->fsp_api->clockCheck(
						(cgc_ctrl_t *)&data->cgc_ctrl, cgc_clock_id);
			k_spin_unlock(&data->lock, key);
			return rh850_cgc_fsp_err_to_errno(err);
		}

		/* PLL/PLLO/SSCG start needs a PLL/SSCG configuration payload. */
		if ((cgc_clock_id == CGC_CLOCK_PLL)
#if defined(BSP_FEATURE_CGC_HAS_CLK_SYS_SSCG) && BSP_FEATURE_CGC_HAS_CLK_SYS_SSCG
		    || (cgc_clock_id == CGC_CLOCK_SSCG)
#endif
		) {
			return -EINVAL;
		}

		k_spinlock_key_t key = k_spin_lock(&data->lock);

		err = config->fsp_api->clockStart(
				(cgc_ctrl_t *)&data->cgc_ctrl, cgc_clock_id, NULL);

		k_spin_unlock(&data->lock, key);
		if ((err == FSP_SUCCESS) || (err == FSP_ERR_CGC_CLOCK_IN_USE)) {
			return 0;
		}

		return rh850_cgc_fsp_err_to_errno(err);
	}

	return -ENOTSUP;
}

/** Zephyr clock_control_off() callback. */
static int clock_control_rh850_cgc_off(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t domain = rh850_cgc_subsys_domain(sys);
	int ret;

	if (domain == RH850_CGC_DOMAIN_PCLK) {
		return rh850_cgc_pclk_gate_off(dev, sys);
	}

	if (domain == RH850_CGC_DOMAIN_OSC) {
		const struct rh850_cgc_config *config = dev->config;
		struct rh850_cgc_data *data = dev->data;
		uint32_t clock_id = rh850_cgc_subsys_clock(sys);
		cgc_clock_t cgc_clock_id;
		fsp_err_t err;

		/* LOCO/LSIOSC is always running/in-use on RH850 U2x HAL. */
		if ((clock_id == RH850_CLOCK_LSIOSC) || (clock_id == RH850_CLOCK_HSIOSC) ||
		    (clock_id == RH850_CLOCK_IOSC) || (clock_id == RH850_CLOCK_HVIOSC)) {
			return -ENOTSUP;
		}

		ret = rh850_cgc_open(dev);
		if (ret < 0) {
			return ret;
		}

		ret = rh850_cgc_clock_to_cgc(clock_id, &cgc_clock_id);
		if (ret < 0) {
			return ret;
		}

		k_spinlock_key_t key = k_spin_lock(&data->lock);

		err = config->fsp_api->clockStop((cgc_ctrl_t *)&data->cgc_ctrl, cgc_clock_id);
		k_spin_unlock(&data->lock, key);
		if ((err == FSP_SUCCESS) || (err == FSP_ERR_CGC_CLOCK_INACTIVE)) {
			return 0;
		}

		return rh850_cgc_fsp_err_to_errno(err);
	}

	return -ENOTSUP;
}

/**
 * Return the effective active clock frequency.
 *
 * The portable HAL reads active selector/divider status registers and keeps
 * all SoC-specific and FSP-private mappings out of the common Zephyr driver.
 */
static int clock_control_rh850_cgc_get_rate(const struct device *dev, clock_control_subsys_t sys,
					    uint32_t *rate)
{
	uint32_t domain = rh850_cgc_subsys_domain(sys);
	uint32_t clock_id = rh850_cgc_subsys_clock(sys);
	fsp_err_t err;
	uint32_t hz;

	ARG_UNUSED(dev);

	if (rate == NULL) {
		return -EINVAL;
	}

	if ((domain != RH850_CGC_DOMAIN_PCLK) && (domain != RH850_CGC_DOMAIN_SYSCLK) &&
	    (domain != RH850_CGC_DOMAIN_OSC)) {
		return -ENOTSUP;
	}

	/* Existing pointer and domain validation remains above this call. */
	err = RP_BSP_ClockHzGet(clock_id, &hz);
	if (err != FSP_SUCCESS) {
		return rh850_cgc_fsp_err_to_errno(err);
	}

	if (hz == 0U) {
		return -EAGAIN;
	}

	*rate = hz;
	return 0;
}

static enum clock_control_status rh850_cgc_sysclk_status(const struct device *dev,
							 clock_control_subsys_t sys)
{
	uint32_t rate;
	int ret;

	ret = clock_control_rh850_cgc_get_rate(dev, sys, &rate);
	if (ret != 0) {
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	return (rate != 0U) ? CLOCK_CONTROL_STATUS_ON : CLOCK_CONTROL_STATUS_OFF;
}

static enum clock_control_status clock_control_rh850_cgc_get_status(const struct device *dev,
								    clock_control_subsys_t sys)
{
	uint32_t domain = rh850_cgc_subsys_domain(sys);
	int ret;

	if (domain == RH850_CGC_DOMAIN_PCLK) {
		return rh850_cgc_pclk_gate_status(dev, sys);
	}

	if (domain == RH850_CGC_DOMAIN_SYSCLK) {
		return rh850_cgc_sysclk_status(dev, sys);
	}

	if (domain == RH850_CGC_DOMAIN_OSC) {
		const struct rh850_cgc_config *config = dev->config;
		struct rh850_cgc_data *data = dev->data;
		cgc_clock_t cgc_clock_id;
		fsp_err_t err;

		ret = rh850_cgc_open(dev);
		if (ret < 0) {
			return CLOCK_CONTROL_STATUS_UNKNOWN;
		}

		ret = rh850_cgc_clock_to_cgc(rh850_cgc_subsys_clock(sys), &cgc_clock_id);
		if (ret < 0) {
			return CLOCK_CONTROL_STATUS_UNKNOWN;
		}

		k_spinlock_key_t key = k_spin_lock(&data->lock);

		err = config->fsp_api->clockCheck((cgc_ctrl_t *)&data->cgc_ctrl, cgc_clock_id);
		k_spin_unlock(&data->lock, key);

		switch (err) {
		case FSP_SUCCESS:
			return CLOCK_CONTROL_STATUS_ON;
		case FSP_ERR_CGC_CLOCK_INACTIVE:
		case FSP_ERR_CGC_NOT_STABILIZED:
			return CLOCK_CONTROL_STATUS_OFF;
		default:
			return CLOCK_CONTROL_STATUS_UNKNOWN;
		}
	}

	return CLOCK_CONTROL_STATUS_UNKNOWN;
}

/** Zephyr clock_control_set_rate() callback; exact-Hz changes are unsupported. */
static int clock_control_rh850_cgc_set_rate(const struct device *dev, clock_control_subsys_t sys,
					    clock_control_subsys_rate_t rate)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);
	ARG_UNUSED(rate);

	/* RH850 U2x FSP supports gear changes, not an exact rate in Hz. */
	return -ENOTSUP;
}

/** Zephyr clock_control_configure() callback. */
static int clock_control_rh850_cgc_configure(const struct device *dev, clock_control_subsys_t sys,
					     void *data_in)
{
	const struct rh850_cgc_config *config = dev->config;
	struct rh850_cgc_data *data = dev->data;
	const struct rh850_cgc_configure *cfg = data_in;
	uint32_t domain = rh850_cgc_subsys_domain(sys);
	cgc_clock_t cgc_clock_id;
	int ret;

	ret = rh850_cgc_open(dev);
	if (ret < 0) {
		return ret;
	}

	if (domain == RH850_CGC_DOMAIN_PCLK) {
		if ((cfg == NULL) || (cfg->pclk_cfg == NULL)) {
			/* Apply the matching DTS source/divider for this clock. */
			return rh850_cgc_apply_pclk_cfg_from_dts(dev, rh850_cgc_subsys_clock(sys));
		}

		if (cfg->pclk_cfg->clock_id != rh850_cgc_subsys_clock(sys)) {
			return -EINVAL;
		}

		k_spinlock_key_t key = k_spin_lock(&data->lock);

		ret = rh850_cgc_apply_pclk_cfg(cfg->pclk_cfg);
		if (ret == 0) {
			size_t index;

			if (rh850_cgc_find_pclk_cfg(dev->config, cfg->pclk_cfg->clock_id, &index) !=
				    NULL &&
			    index < 32U) {
				data->pclk_cfg_applied_mask |= BIT(index);
			}
		}
		k_spin_unlock(&data->lock, key);

		return ret;
	}

	if (domain == RH850_CGC_DOMAIN_OSC) {
		ret = rh850_cgc_clock_to_cgc(rh850_cgc_subsys_clock(sys), &cgc_clock_id);
		if (ret < 0) {
			return ret;
		}

		if ((cgc_clock_id != CGC_CLOCK_PLL)
#if defined(BSP_FEATURE_CGC_HAS_CLK_SYS_SSCG) && BSP_FEATURE_CGC_HAS_CLK_SYS_SSCG
		    && (cgc_clock_id != CGC_CLOCK_SSCG)
#endif
		) {
			return -ENOTSUP;
		}

		if ((cfg == NULL) || (cfg->pll_cfg == NULL)) {
			return -EINVAL;
		}

		k_spinlock_key_t key = k_spin_lock(&data->lock);
		fsp_err_t err = config->fsp_api->clockStart((cgc_ctrl_t *)&data->cgc_ctrl,
							 cgc_clock_id, cfg->pll_cfg);
		k_spin_unlock(&data->lock, key);

		return rh850_cgc_fsp_err_to_errno(err);
	}

	if (domain == RH850_CGC_DOMAIN_SYSCLK) {
		if ((cfg != NULL) && (cfg->system_clock_cfg != NULL)) {
			k_spinlock_key_t key = k_spin_lock(&data->lock);
			fsp_err_t err = config->fsp_api->systemClockGearSet(
				(cgc_ctrl_t *)&data->cgc_ctrl, cfg->system_clock_cfg);
			k_spin_unlock(&data->lock, key);

			return rh850_cgc_fsp_err_to_errno(err);
		}

		ret = rh850_cgc_clock_to_cgc(rh850_cgc_subsys_clock(sys), &cgc_clock_id);
		if (ret < 0) {
			return -ENOTSUP;
		}

		k_spinlock_key_t key = k_spin_lock(&data->lock);
		fsp_err_t err = config->fsp_api->systemClockSet(
			(cgc_ctrl_t *)&data->cgc_ctrl, cgc_clock_id,
			(cfg != NULL) ? cfg->divider_cfg : NULL);
		k_spin_unlock(&data->lock, key);

		return rh850_cgc_fsp_err_to_errno(err);
	}

	return -ENOTSUP;
}

static int rh850_cgc_init(const struct device *dev)
{
	int ret;

	ret = rh850_cgc_open(dev);
	if (ret < 0) {
		return ret;
	}

	/*
	 * R_BSP_InitClock() applies the DTS system clock sources and SSCG
	 * modulation through bsp_clock_cfg.h in soc_early_init_hook().
	 * Peripheral DTS settings are applied when a consumer requests them
	 * through clock_control_on()/configure().
	 */
	return 0;
}

static DEVICE_API(clock_control, clock_control_rh850_cgc_api) = {
	.on = clock_control_rh850_cgc_on,
	.off = clock_control_rh850_cgc_off,
	.get_rate = clock_control_rh850_cgc_get_rate,
	.get_status = clock_control_rh850_cgc_get_status,
	.set_rate = clock_control_rh850_cgc_set_rate,
	.configure = clock_control_rh850_cgc_configure,
};

#define RH850_CGC_INIT(inst)                                                                       \
	BUILD_ASSERT(ARRAY_SIZE(rh850_cgc_pclk_cfgs) <= 32U,                                  \
		     "pclk_cfg_applied_mask only tracks 32 entries");                             \
	static struct rh850_cgc_data rh850_cgc_data_##inst;                                        \
	static const struct rh850_cgc_config rh850_cgc_config_##inst = {                           \
		.fsp_api = &g_cgc_on_cgc,                                                          \
		.pclk_cfgs = rh850_cgc_pclk_cfgs,                                                  \
		.pclk_cfg_count = ARRAY_SIZE(rh850_cgc_pclk_cfgs),                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, rh850_cgc_init, NULL, &rh850_cgc_data_##inst,                  \
			      &rh850_cgc_config_##inst, PRE_KERNEL_1,                              \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_rh850_cgc_api);

DT_INST_FOREACH_STATUS_OKAY(RH850_CGC_INIT)
