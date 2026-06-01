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
#include <zephyr/sys/util.h>

#include <zephyr/dt-bindings/clock/renesas_rh850_clock_common.h>
#include <zephyr/drivers/clock_control/renesas_rh850_cgc.h>
#include "rp_bsp_module_standby.h"

#define RH850_CGC_MAX_CHANNEL 31U

struct rh850_cgc_data {
	cgc_instance_ctrl_t cgc_ctrl;
	bool opened;
};

struct rh850_cgc_module_map {
	uint32_t module;
	bsp_module_standby_t fsp_module;
};

static enum clock_control_status rh850_cgc_get_status(
	const struct device *dev,
	clock_control_subsys_t sys);

static const struct rh850_cgc_module_map rh850_cgc_module_map[] = {
#if defined(CONFIG_SOC_SERIES_RH850U2A)
	{ RH850_MODULE_RSCFD, BSP_MODULE_STANDBY_MSR_RSCFD },
	{ RH850_MODULE_FLXA, BSP_MODULE_STANDBY_MSR_FLXA },
	{ RH850_MODULE_GTM, BSP_MODULE_STANDBY_MSR_GTM },
#if defined(BSP_FEATURE_ETNB_IS_AVAILABLE) && BSP_FEATURE_ETNB_IS_AVAILABLE
	{ RH850_MODULE_ETNB, BSP_MODULE_STANDBY_MSR_ETNB },
#endif
	{ RH850_MODULE_RSENT, BSP_MODULE_STANDBY_MSR_RSENT },
	{ RH850_MODULE_MSPI, BSP_MODULE_STANDBY_MSR_MSPI },
	{ RH850_MODULE_RLIN3, BSP_MODULE_STANDBY_MSR_RLIN3 },
	{ RH850_MODULE_ADCJ_ISO, BSP_MODULE_STANDBY_MSR_ADCJ_ISO },
#if defined(BSP_FEATURE_CXPI_IS_AVAILABLE) && BSP_FEATURE_CXPI_IS_AVAILABLE
	{ RH850_MODULE_CXPI, BSP_MODULE_STANDBY_MSR_CXPI },
#endif
#if defined(BSP_FEATURE_MMCA_IS_AVAILABLE) && BSP_FEATURE_MMCA_IS_AVAILABLE
	{ RH850_MODULE_MMCA, BSP_MODULE_STANDBY_MSR_MMCA },
#endif
#if defined(BSP_FEATURE_ENCA_IS_AVAILABLE) && BSP_FEATURE_ENCA_IS_AVAILABLE
	{ RH850_MODULE_ENCA, BSP_MODULE_STANDBY_MSR_ENCA },
#endif
	{ RH850_MODULE_PSI5, BSP_MODULE_STANDBY_MSR_PSI5 },
	{ RH850_MODULE_PSI5S, BSP_MODULE_STANDBY_MSR_PSI5S },
	{ RH850_MODULE_PWMD, BSP_MODULE_STANDBY_MSR_PWMD },
#if defined(BSP_FEATURE_RHSIF_IS_AVAILABLE) && BSP_FEATURE_RHSIF_IS_AVAILABLE
	{ RH850_MODULE_RHSIF, BSP_MODULE_STANDBY_MSR_RHSIF },
#endif
	{ RH850_MODULE_RIIC, BSP_MODULE_STANDBY_MSR_RIIC },
#if defined(BSP_FEATURE_SCI3_IS_AVAILABLE) && BSP_FEATURE_SCI3_IS_AVAILABLE
	{ RH850_MODULE_SCI3, BSP_MODULE_STANDBY_MSR_SCI3 },
#endif
	{ RH850_MODULE_TAPA, BSP_MODULE_STANDBY_MSR_TAPA },
	{ RH850_MODULE_TAUD, BSP_MODULE_STANDBY_MSR_TAUD },
	{ RH850_MODULE_TAUJ_ISO, BSP_MODULE_STANDBY_MSR_TAUJ_ISO },
	{ RH850_MODULE_TPBA, BSP_MODULE_STANDBY_MSR_TPBA },
	{ RH850_MODULE_TSG3, BSP_MODULE_STANDBY_MSR_TSG3 },
	{ RH850_MODULE_OSTM, BSP_MODULE_STANDBY_MSR_OSTM },
	{ RH850_MODULE_ADCJ_AWO, BSP_MODULE_STANDBY_MSR_ADCJ_AWO },
	{ RH850_MODULE_RTCA, BSP_MODULE_STANDBY_MSR_RTCA },
	{ RH850_MODULE_TAUJ_AWO, BSP_MODULE_STANDBY_MSR_TAUJ_AWO },
	{ RH850_MODULE_WDTB_AWO, BSP_MODULE_STANDBY_MSR_WDTB_AWO },
#elif defined(CONFIG_SOC_SERIES_RH850U2B)
	{ RH850_MODULE_RSCFD, BSP_MODULE_STANDBY_MSR_RSCFD },
	{ RH850_MODULE_FLXA, BSP_MODULE_STANDBY_MSR_FLXA },
	{ RH850_MODULE_GTM, BSP_MODULE_STANDBY_MSR_GTM },
	{ RH850_MODULE_ETN, BSP_MODULE_STANDBY_MSR_ETN },
	{ RH850_MODULE_RSENT, BSP_MODULE_STANDBY_MSR_RSENT },
	{ RH850_MODULE_MSPI, BSP_MODULE_STANDBY_MSR_MSPI },
	{ RH850_MODULE_RLIN3, BSP_MODULE_STANDBY_MSR_RLIN3 },
	{ RH850_MODULE_DSADC_CADC, BSP_MODULE_STANDBY_MSR_DSADC_CADC },
	{ RH850_MODULE_ADCK_ISO, BSP_MODULE_STANDBY_MSR_ADCK_ISO },
#if defined(BSP_FEATURE_MMCA_IS_AVAILABLE) && BSP_FEATURE_MMCA_IS_AVAILABLE
	{ RH850_MODULE_MMCA, BSP_MODULE_STANDBY_MSR_MMCA },
#endif
	{ RH850_MODULE_ENCA, BSP_MODULE_STANDBY_MSR_ENCA },
#if defined(BSP_FEATURE_PSI5_IS_AVAILABLE) && BSP_FEATURE_PSI5_IS_AVAILABLE
	{ RH850_MODULE_PSI5, BSP_MODULE_STANDBY_MSR_PSI5 },
#endif
#if defined(BSP_FEATURE_PSI5S_IS_AVAILABLE) && BSP_FEATURE_PSI5S_IS_AVAILABLE
	{ RH850_MODULE_PSI5S, BSP_MODULE_STANDBY_MSR_PSI5S },
#endif
#if defined(BSP_FEATURE_RHSIF_IS_AVAILABLE) && BSP_FEATURE_RHSIF_IS_AVAILABLE
	{ RH850_MODULE_RHSIF, BSP_MODULE_STANDBY_MSR_RHSIF },
#endif
	{ RH850_MODULE_RHSB, BSP_MODULE_STANDBY_MSR_RHSB },
#if defined(BSP_FEATURE_RIIC_IS_AVAILABLE) && BSP_FEATURE_RIIC_IS_AVAILABLE
	{ RH850_MODULE_RIIC, BSP_MODULE_STANDBY_MSR_RIIC },
#endif
	{ RH850_MODULE_SFMA, BSP_MODULE_STANDBY_MSR_SFMA },
	{ RH850_MODULE_TAPA, BSP_MODULE_STANDBY_MSR_TAPA },
	{ RH850_MODULE_TAUD, BSP_MODULE_STANDBY_MSR_TAUD },
	{ RH850_MODULE_TPBA, BSP_MODULE_STANDBY_MSR_TPBA },
	{ RH850_MODULE_TSG3, BSP_MODULE_STANDBY_MSR_TSG3 },
	{ RH850_MODULE_OSTM, BSP_MODULE_STANDBY_MSR_OSTM },
	{ RH850_MODULE_ATU, BSP_MODULE_STANDBY_MSR_ATU },
	{ RH850_MODULE_DFE, BSP_MODULE_STANDBY_MSR_DFE },
#if defined(BSP_FEATURE_DFP_IS_AVAILABLE) && BSP_FEATURE_DFP_IS_AVAILABLE
	{ RH850_MODULE_DFP, BSP_MODULE_STANDBY_MSR_DFP },
#endif
	{ RH850_MODULE_RDC, BSP_MODULE_STANDBY_MSR_RDC },
	{ RH850_MODULE_FCOMP, BSP_MODULE_STANDBY_MSR_FCOMP },
#if defined(BSP_FEATURE_TSG3_EMU3S_IS_AVAILABLE) && BSP_FEATURE_TSG3_EMU3S_IS_AVAILABLE
	{ RH850_MODULE_EMU, BSP_MODULE_STANDBY_MSR_EMU },
#endif
	{ RH850_MODULE_HRPWM, BSP_MODULE_STANDBY_MSR_HRPWM },
#if defined(BSP_FEATURE_DSMIF_IS_AVAILABLE) && BSP_FEATURE_DSMIF_IS_AVAILABLE
	{ RH850_MODULE_DSMIF, BSP_MODULE_STANDBY_MSR_DSMIF },
#endif
#elif defined(CONFIG_SOC_SERIES_RH850U2C)
	{ RH850_MODULE_RSCFD, BSP_MODULE_STANDBY_MSR_RSCFD },
#if defined(BSP_FEATURE_FLXA_IS_AVAILABLE) && BSP_FEATURE_FLXA_IS_AVAILABLE
	{ RH850_MODULE_FLXA, BSP_MODULE_STANDBY_MSR_FLXA },
#endif
	{ RH850_MODULE_GTM, BSP_MODULE_STANDBY_MSR_GTM },
#if defined(BSP_FEATURE_ETND_IS_AVAILABLE) && BSP_FEATURE_ETND_IS_AVAILABLE
	{ RH850_MODULE_ETND, BSP_MODULE_STANDBY_MSR_ETND },
#endif
	{ RH850_MODULE_ETNF, BSP_MODULE_STANDBY_MSR_ETNF },
	{ RH850_MODULE_RSENT, BSP_MODULE_STANDBY_MSR_RSENT },
	{ RH850_MODULE_MSPI, BSP_MODULE_STANDBY_MSR_MSPI },
	{ RH850_MODULE_RLIN3, BSP_MODULE_STANDBY_MSR_RLIN3 },
	{ RH850_MODULE_ADCK_ISO, BSP_MODULE_STANDBY_MSR_ADCK_ISO },
	{ RH850_MODULE_CXPI, BSP_MODULE_STANDBY_MSR_CXPI },
#if defined(BSP_FEATURE_MMCA_IS_AVAILABLE) && BSP_FEATURE_MMCA_IS_AVAILABLE
	{ RH850_MODULE_MMCA, BSP_MODULE_STANDBY_MSR_MMCA },
#endif
	{ RH850_MODULE_ENCA, BSP_MODULE_STANDBY_MSR_ENCA },
	{ RH850_MODULE_PSI5, BSP_MODULE_STANDBY_MSR_PSI5 },
#if defined(BSP_FEATURE_PSI5S_IS_AVAILABLE) && BSP_FEATURE_PSI5S_IS_AVAILABLE
	{ RH850_MODULE_PSI5S, BSP_MODULE_STANDBY_MSR_PSI5S },
#endif
	{ RH850_MODULE_PWMD, BSP_MODULE_STANDBY_MSR_PWMD },
	{ RH850_MODULE_SFMA, BSP_MODULE_STANDBY_MSR_SFMA },
	{ RH850_MODULE_TAPA, BSP_MODULE_STANDBY_MSR_TAPA },
	{ RH850_MODULE_TAUD, BSP_MODULE_STANDBY_MSR_TAUD },
	{ RH850_MODULE_TAUJ_ISO, BSP_MODULE_STANDBY_MSR_TAUJ_ISO },
	{ RH850_MODULE_TSG3, BSP_MODULE_STANDBY_MSR_TSG3 },
	{ RH850_MODULE_OSTM, BSP_MODULE_STANDBY_MSR_OSTM },
	{ RH850_MODULE_RI3C, BSP_MODULE_STANDBY_MSR_RI3C },
	{ RH850_MODULE_SSIF, BSP_MODULE_STANDBY_MSR_SSIF },
	{ RH850_MODULE_ADCK_AWO, BSP_MODULE_STANDBY_MSR_ADCK_AWO },
	{ RH850_MODULE_RTCA, BSP_MODULE_STANDBY_MSR_RTCA },
	{ RH850_MODULE_TAUJ_AWO, BSP_MODULE_STANDBY_MSR_TAUJ_AWO },
	{ RH850_MODULE_WDTB_AWO, BSP_MODULE_STANDBY_MSR_WDTB_AWO },
#endif
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

static int rh850_cgc_module_to_fsp(uint32_t module, bsp_module_standby_t *fsp_module)
{
	for (size_t i = 0; i < ARRAY_SIZE(rh850_cgc_module_map); i++) {
		if (rh850_cgc_module_map[i].module == module) {
			*fsp_module = rh850_cgc_module_map[i].fsp_module;
			return 0;
		}
	}

	return -EINVAL;
}

static int rh850_cgc_return_bsp_rate(uint32_t *rate, uint32_t hz)
{
	*rate = hz;

	return 0;
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
		*cgc_clock = CGC_CLOCK_PLL;
		return 0;
	default:
		return -EINVAL;
	}
}

static int rh850_cgc_open(const struct device *dev)
{
	struct rh850_cgc_data *data = dev->data;
	const cgc_cfg_t cfg = { 0 };
	fsp_err_t err;

	if (data->opened) {
		return 0;
	}

	err = g_cgc_on_cgc.open((cgc_ctrl_t *)&data->cgc_ctrl, &cfg);
	if ((err == FSP_SUCCESS) || (err == FSP_ERR_ALREADY_OPEN)) {
		data->opened = true;
		return 0;
	}

	return rh850_cgc_fsp_err_to_errno(err);
}

static int rh850_cgc_on(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t domain = rh850_cgc_subsys_domain(sys);
	int ret;

	if (domain == RH850_CGC_DOMAIN_PCLK) {
		bsp_module_standby_t module;
		uint32_t channel = rh850_cgc_subsys_channel(sys);

		if (channel > RH850_CGC_MAX_CHANNEL) {
			return -EINVAL;
		}

		ret = rh850_cgc_module_to_fsp(rh850_cgc_subsys_module(sys), &module);
		if (ret < 0) {
			return ret;
		}

		if (rh850_cgc_get_status(dev, sys) == CLOCK_CONTROL_STATUS_ON) {
			return 0;
		}

		/*
		 * FSP RH850:
		 * Enable clears the MSR channel bit and supplies the module clock.
		 */
		R_BSP_ModuleStandbyEnable(module, channel);

		return 0;
	}

	if (domain == RH850_CGC_DOMAIN_OSC) {
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

		/* PLL start needs cgc_pll_cfg_t; use clock_control_configure(). */
		if (cgc_clock_id == CGC_CLOCK_PLL) {
			return -EINVAL;
		}

		err = g_cgc_on_cgc.clockStart((cgc_ctrl_t *)&data->cgc_ctrl, cgc_clock_id, NULL);
		if ((err == FSP_SUCCESS) || (err == FSP_ERR_CGC_CLOCK_IN_USE)) {
			return 0;
		}

		return rh850_cgc_fsp_err_to_errno(err);
	}

	return -ENOTSUP;
}

static int rh850_cgc_off(const struct device *dev, clock_control_subsys_t sys)
{
	uint32_t domain = rh850_cgc_subsys_domain(sys);
	int ret;

	if (domain == RH850_CGC_DOMAIN_PCLK) {
		bsp_module_standby_t module;
		uint32_t channel = rh850_cgc_subsys_channel(sys);

		if (channel > RH850_CGC_MAX_CHANNEL) {
			return -EINVAL;
		}

		ret = rh850_cgc_module_to_fsp(rh850_cgc_subsys_module(sys), &module);
		if (ret < 0) {
			return ret;
		}

		if (rh850_cgc_get_status(dev, sys) == CLOCK_CONTROL_STATUS_OFF) {
			return 0;
		}

		/*
		 * FSP RH850:
		 * Disable sets the MSR channel bit and stops the module clock.
		 */
		R_BSP_ModuleStandbyDisable(module, channel);

		return 0;
	}

	if (domain == RH850_CGC_DOMAIN_OSC) {
		struct rh850_cgc_data *data = dev->data;
		uint32_t clock_id = rh850_cgc_subsys_clock(sys);
		cgc_clock_t cgc_clock_id;
		fsp_err_t err;

		if (clock_id == RH850_CLOCK_LSIOSC) {
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

		err = g_cgc_on_cgc.clockStop((cgc_ctrl_t *)&data->cgc_ctrl, cgc_clock_id);
		if ((err == FSP_SUCCESS) || (err == FSP_ERR_CGC_CLOCK_INACTIVE)) {
			return 0;
		}

		return rh850_cgc_fsp_err_to_errno(err);
	}

	return -ENOTSUP;
}

static int rh850_cgc_clock_to_rate(uint32_t clock_id, uint32_t *rate)
{
	fsp_priv_clock_t fsp_clock;
	uint32_t hz;

	if (rate == NULL) {
		return -EINVAL;
	}

	switch (clock_id) {
#if defined(BSP_CFG_CLOCK_CLK_MOSC_HZ)
	case RH850_CLOCK_MOSC:
		fsp_clock = FSP_PRIV_CLOCK_CLK_MOSC;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_HSIOSC_HZ)
	case RH850_CLOCK_HSIOSC:
		fsp_clock = FSP_PRIV_CLOCK_CLK_HSIOSC;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_LSIOSC_HZ)
	case RH850_CLOCK_LSIOSC:
		fsp_clock = FSP_PRIV_CLOCK_CLK_LSIOSC;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_HVIOSC_HZ)
	case RH850_CLOCK_HVIOSC:
		fsp_clock = FSP_PRIV_CLOCK_CLK_HVIOSC;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_PLL_HZ)
	case RH850_CLOCK_PLL:
		fsp_clock = FSP_PRIV_CLOCK_CLK_PLL;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_PLLO_HZ)
	case RH850_CLOCK_PLLO:
		fsp_clock = FSP_PRIV_CLOCK_CLK_PLLO;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_SSCG_HZ)
	case RH850_CLOCK_SSCG:
		fsp_clock = FSP_PRIV_CLOCK_CLK_SSCG;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_SYS_HZ)
	case RH850_CLOCK_SYS:
		fsp_clock = FSP_PRIV_CLOCK_CLK_SYS;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_SYS_SSCG_HZ)
	case RH850_CLOCK_SYS_SSCG:
		fsp_clock = FSP_PRIV_CLOCK_CLK_SYS_SSCG;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_CPU_HZ)
	case RH850_CLOCK_CPU:
		fsp_clock = FSP_PRIV_CLOCK_CLK_CPU;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_SBUS_HZ)
	case RH850_CLOCK_SBUS:
		fsp_clock = FSP_PRIV_CLOCK_CLK_SBUS;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_HBUS_HZ)
	case RH850_CLOCK_HBUS:
		fsp_clock = FSP_PRIV_CLOCK_CLK_HBUS;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_UHSB_HZ)
	case RH850_CLOCK_UHSB:
		fsp_clock = FSP_PRIV_CLOCK_CLK_UHSB;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_HSB_HZ)
	case RH850_CLOCK_HSB:
		fsp_clock = FSP_PRIV_CLOCK_CLK_HSB;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_LSB_HZ)
	case RH850_CLOCK_LSB:
		fsp_clock = FSP_PRIV_CLOCK_CLK_LSB;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_MSPI_HZ)
	case RH850_CLOCK_MSPI:
		fsp_clock = FSP_PRIV_CLOCK_CLK_MSPI;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_RLIN3_HZ)
	case RH850_CLOCK_RLIN3:
		return rh850_cgc_return_bsp_rate(rate, BSP_CFG_CLOCK_CLK_RLIN3_HZ);
#endif

#if defined(BSP_CFG_CLOCK_CLK_CH23_RLIN3_HZ)
	case RH850_CLOCK_CH23_RLIN3:
		fsp_clock = FSP_PRIV_CLOCK_CLK_CH23_RLIN3;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_GTM_HZ)
	case RH850_CLOCK_GTM:
		fsp_clock = FSP_PRIV_CLOCK_CLK_GTM;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_WDT_HZ)
	case RH850_CLOCK_WDT:
		fsp_clock = FSP_PRIV_CLOCK_CLK_WDT;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_WDTB_HZ)
	case RH850_CLOCK_WDTB:
		fsp_clock = FSP_PRIV_CLOCK_CLK_WDTB;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_CANFD_C_HZ)
	case RH850_CLOCK_CANFD_C:
		fsp_clock = FSP_PRIV_CLOCK_CLK_CANFD_C;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_CANFD_XIN_HZ)
	case RH850_CLOCK_CANFD_XIN:
		fsp_clock = FSP_PRIV_CLOCK_CLK_CANFD_XIN;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLKA_TAUJ_HZ)
	case RH850_CLOCK_CLKA_TAUJ:
		fsp_clock = FSP_PRIV_CLOCK_CLKA_TAUJ;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLKA_RTCA_HZ)
	case RH850_CLOCK_CLKA_RTCA:
		return rh850_cgc_return_bsp_rate(rate, BSP_CFG_CLOCK_CLKA_RTCA_HZ);
#endif

#if defined(BSP_CFG_CLOCK_CLKA_ADC_HZ)
	case RH850_CLOCK_CLKA_ADC:
		fsp_clock = FSP_PRIV_CLOCK_CLKA_ADC;
		break;
#endif

#if defined(BSP_CFG_CLOCK_CLK_ADC_HZ)
	case RH850_CLOCK_ADC:
		fsp_clock = FSP_PRIV_CLOCK_CLK_ADC;
		break;
#endif

	default:
		return -ENOTSUP;
	}

	hz = R_FSP_SystemClockHzGet(fsp_clock);
	if (hz == 0U) {
		return -EINVAL;
	}

	*rate = hz;

	return 0;
}

/*
 * Limitation:
 * - get_rate() returns the BSP configured clock rate only when the
 *   encoded clock ID maps to a concrete RH850_CLOCK_* entry.
 * - For module-standby-only IDs, the driver can enable/disable the
 *   module clock gate but cannot infer the final peripheral clock rate.
 * - Runtime clock source/divider changes are not reflected unless the
 *   BSP clock configuration macros are updated consistently.
 */
static int rh850_cgc_get_rate(const struct device *dev,
			      clock_control_subsys_t sys,
			      uint32_t *rate)
{
	uint32_t domain = rh850_cgc_subsys_domain(sys);
	uint32_t clock_id = rh850_cgc_subsys_clock(sys);

	ARG_UNUSED(dev);

	if (rate == NULL) {
		return -EINVAL;
	}

	/*
	 * This driver can return a rate only when the clk_id contains a
	 * concrete RH850_CLOCK_* value that has a corresponding BSP clock
	 * configuration macro.
	 */
	if ((domain != RH850_CGC_DOMAIN_PCLK) &&
	    (domain != RH850_CGC_DOMAIN_SYSCLK) &&
	    (domain != RH850_CGC_DOMAIN_OSC)) {
		return -ENOTSUP;
	}

	return rh850_cgc_clock_to_rate(clock_id, rate);
}

static enum clock_control_status rh850_cgc_get_status(const struct device *dev,
						      clock_control_subsys_t sys)
{
	uint32_t domain = rh850_cgc_subsys_domain(sys);
	int ret;

	if (domain == RH850_CGC_DOMAIN_PCLK) {
		bsp_module_standby_t module;
		uint32_t channel = rh850_cgc_subsys_channel(sys);
		bool module_standby;

		ARG_UNUSED(dev);

		if (channel > RH850_CGC_MAX_CHANNEL) {
			return CLOCK_CONTROL_STATUS_UNKNOWN;
		}

		ret = rh850_cgc_module_to_fsp(rh850_cgc_subsys_module(sys), &module);
		if (ret < 0) {
			return CLOCK_CONTROL_STATUS_UNKNOWN;
		}

		ret = RP_BSP_ModuleStandbyGet(module, channel, &module_standby);
		if (ret != FSP_SUCCESS) {
			return CLOCK_CONTROL_STATUS_UNKNOWN;
		}

		return module_standby ? CLOCK_CONTROL_STATUS_OFF : CLOCK_CONTROL_STATUS_ON;
	}

	if (domain == RH850_CGC_DOMAIN_OSC) {
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

		err = g_cgc_on_cgc.clockCheck((cgc_ctrl_t *)&data->cgc_ctrl, cgc_clock_id);

		return (err == FSP_SUCCESS) ? CLOCK_CONTROL_STATUS_ON :
					      CLOCK_CONTROL_STATUS_OFF;
	}

	return CLOCK_CONTROL_STATUS_UNKNOWN;
}

static int rh850_cgc_set_rate(const struct device *dev,
			      clock_control_subsys_t sys,
			      clock_control_subsys_rate_t rate)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);
	ARG_UNUSED(rate);

	return -ENOTSUP;
}

static int rh850_cgc_configure(const struct device *dev,
			       clock_control_subsys_t sys,
			       void *data_in)
{
	struct rh850_cgc_data *data = dev->data;
	const struct rh850_cgc_configure *cfg = data_in;
	uint32_t domain = rh850_cgc_subsys_domain(sys);
	cgc_clock_t cgc_clock_id;
	int ret;

	ret = rh850_cgc_open(dev);
	if (ret < 0) {
		return ret;
	}

	if (domain == RH850_CGC_DOMAIN_OSC) {
		ret = rh850_cgc_clock_to_cgc(rh850_cgc_subsys_clock(sys), &cgc_clock_id);
		if (ret < 0) {
			return ret;
		}

		if (cgc_clock_id != CGC_CLOCK_PLL) {
			return -ENOTSUP;
		}

		if ((cfg == NULL) || (cfg->pll_cfg == NULL)) {
			return -EINVAL;
		}

		return rh850_cgc_fsp_err_to_errno(
			g_cgc_on_cgc.clockStart((cgc_ctrl_t *)&data->cgc_ctrl,
						cgc_clock_id, cfg->pll_cfg));
	}

	if (domain == RH850_CGC_DOMAIN_SYSCLK) {
		if ((cfg != NULL) && (cfg->system_clock_cfg != NULL)) {
			return rh850_cgc_fsp_err_to_errno(
				g_cgc_on_cgc.systemClockGearSet(
					(cgc_ctrl_t *)&data->cgc_ctrl,
					cfg->system_clock_cfg));
		}

		ret = rh850_cgc_clock_to_cgc(rh850_cgc_subsys_clock(sys), &cgc_clock_id);
		if (ret < 0) {
			return -ENOTSUP;
		}

		return rh850_cgc_fsp_err_to_errno(
			g_cgc_on_cgc.systemClockSet(
				(cgc_ctrl_t *)&data->cgc_ctrl,
				cgc_clock_id,
				(cfg != NULL) ? cfg->divider_cfg : NULL));
	}

	return -ENOTSUP;
}

static int rh850_cgc_init(const struct device *dev)
{
	return rh850_cgc_open(dev);
}

static DEVICE_API(clock_control, rh850_cgc_api) = {
	.on = rh850_cgc_on,
	.off = rh850_cgc_off,
	.get_rate = rh850_cgc_get_rate,
	.get_status = rh850_cgc_get_status,
	.set_rate = rh850_cgc_set_rate,
	.configure = rh850_cgc_configure,
};

#define RH850_CGC_INIT(inst)							\
	static struct rh850_cgc_data rh850_cgc_data_##inst;			\
	DEVICE_DT_INST_DEFINE(inst,						\
			      rh850_cgc_init,					\
			      NULL,						\
			      &rh850_cgc_data_##inst,				\
			      NULL,						\
			      PRE_KERNEL_1,					\
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,			\
			      &rh850_cgc_api);

DT_INST_FOREACH_STATUS_OKAY(RH850_CGC_INIT)
