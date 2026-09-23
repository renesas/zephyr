/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT renesas_rcar_wwdt

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/watchdog.h>

#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <r_wwdt_api.h>
#include <r_wwdt_reg.h>
#include <r_ecm.h>
#include <r_error_domain_id.h>
#include <r_ecm_reg.h>

#define WDTA0WS_MASK                3
#define RCAR_WWDT_RT_CHANNEL_COUNT  20
#define R_WWDT_CHANNEL_ADDR_STRIDE  0x00010000UL
#define R_WWDT_ECM_DETECTION_ENABLE 1
#define WWDT_TIMEOUT_MIN_CYCLES     BIT(9)  /* 512 */
#define WWDT_TIMEOUT_MAX_CYCLES     BIT(16) /* 65536 */

#define WDT_RENESAS_RCAR_SUPPORTED_FLAGS (WDT_FLAG_RESET_NONE | WDT_FLAG_RESET_SOC)

static const e_ecm_error_id_t wwdt_reset_ecm_id_map[RCAR_WWDT_RT_CHANNEL_COUNT] = {
	[0] = WWDT0_DETECTS_ERROR_RES_IS_OUTPUT,   [1] = WWDT1_DETECTS_ERROR_RES_IS_OUTPUT,
	[2] = WWDT2_DETECTS_ERROR_RES_IS_OUTPUT,   [3] = WWDT3_DETECTS_ERROR_RES_IS_OUTPUT,
	[4] = WWDT4_DETECTS_ERROR_RES_IS_OUTPUT,   [5] = WWDT5_DETECTS_ERROR_RES_IS_OUTPUT,
	[6] = WWDT6_DETECTS_ERROR_RES_IS_OUTPUT,   [7] = WWDT7_DETECTS_ERROR_RES_IS_OUTPUT,
	[8] = WWDT8_DETECTS_ERROR_RES_IS_OUTPUT,   [9] = WWDT9_DETECTS_ERROR_RES_IS_OUTPUT,
	[10] = WWDT10_DETECTS_ERROR_RES_IS_OUTPUT, [11] = WWDT11_DETECTS_ERROR_RES_IS_OUTPUT,
	[12] = WWDT12_DETECTS_ERROR_RES_IS_OUTPUT, [13] = WWDT13_DETECTS_ERROR_RES_IS_OUTPUT,
	[14] = WWDT14_DETECTS_ERROR_RES_IS_OUTPUT, [15] = WWDT15_DETECTS_ERROR_RES_IS_OUTPUT,
	[16] = WWDT16_DETECTS_ERROR_RES_IS_OUTPUT, [17] = WWDT17_DETECTS_ERROR_RES_IS_OUTPUT,
	[18] = WWDT18_DETECTS_ERROR_RES_IS_OUTPUT, [19] = WWDT19_DETECTS_ERROR_RES_IS_OUTPUT,
};

static const e_ecm_error_id_t wwdt_nmi_ecm_id_map[RCAR_WWDT_RT_CHANNEL_COUNT] = {
	[0] = WWDT0_DETECTS_ERROR_NMI_IS_OUTPUT,   [1] = WWDT1_DETECTS_ERROR_NMI_IS_OUTPUT,
	[2] = WWDT2_DETECTS_ERROR_NMI_IS_OUTPUT,   [3] = WWDT3_DETECTS_ERROR_NMI_IS_OUTPUT,
	[4] = WWDT4_DETECTS_ERROR_NMI_IS_OUTPUT,   [5] = WWDT5_DETECTS_ERROR_NMI_IS_OUTPUT,
	[6] = WWDT6_DETECTS_ERROR_NMI_IS_OUTPUT,   [7] = WWDT7_DETECTS_ERROR_NMI_IS_OUTPUT,
	[8] = WWDT8_DETECTS_ERROR_NMI_IS_OUTPUT,   [9] = WWDT9_DETECTS_ERROR_NMI_IS_OUTPUT,
	[10] = WWDT10_DETECTS_ERROR_NMI_IS_OUTPUT, [11] = WWDT11_DETECTS_ERROR_NMI_IS_OUTPUT,
	[12] = WWDT12_DETECTS_ERROR_NMI_IS_OUTPUT, [13] = WWDT13_DETECTS_ERROR_NMI_IS_OUTPUT,
	[14] = WWDT14_DETECTS_ERROR_NMI_IS_OUTPUT, [15] = WWDT15_DETECTS_ERROR_NMI_IS_OUTPUT,
	[16] = WWDT16_DETECTS_ERROR_NMI_IS_OUTPUT, [17] = WWDT17_DETECTS_ERROR_NMI_IS_OUTPUT,
	[18] = WWDT18_DETECTS_ERROR_NMI_IS_OUTPUT, [19] = WWDT19_DETECTS_ERROR_NMI_IS_OUTPUT,
};

/* WDT time-out periods. */
enum e_wwdt_timeout {
	WWDT_TIMEOUT_512 = 0, /* 2^9 clock cycles */
	WWDT_TIMEOUT_1024,    /* 2^10 clock cycles */
	WWDT_TIMEOUT_2048,    /* 2^11 clock cycles */
	WWDT_TIMEOUT_4096,    /* 2^12 clock cycles */
	WWDT_TIMEOUT_8192,    /* 2^13 clock cycles */
	WWDT_TIMEOUT_16384,   /* 2^14 clock cycles */
	WWDT_TIMEOUT_32768,   /* 2^15 clock cycles */
	WWDT_TIMEOUT_65536,   /* 2^16 clock cycles */
};

LOG_MODULE_REGISTER(wdt_renesas_rcar, CONFIG_WDT_LOG_LEVEL);

struct wwdt_rcar_config {
	DEVICE_MMIO_ROM; /* Must be first */
	struct reset_dt_spec ms_reset_id0;
	struct reset_dt_spec ms_reset_id1;
	uint32_t rclk_freq_hz;
	uint8_t channel_id;
	bool irq_75p_enabled;
	wwdt_erm_t error_mode;
	void (*irq_config_func)(const struct device *dev);
};

struct wwdt_rcar_data {
	DEVICE_MMIO_RAM; /* Must be first */
	struct wdt_timeout_cfg timeout_cfg;
	uint8_t overflow_code;
	uint8_t window_code;
	bool timeout_installed;
	bool started;
};

/*
 * ================================================================
 * Hardware-specific helpers
 * ================================================================
 */

static uint8_t rcar_wwdt_read8(const struct device *dev, uint32_t offset)
{
	return sys_read8(DEVICE_MMIO_GET(dev) + offset);
}

static void rcar_wwdt_write8(const struct device *dev, uint32_t offset, uint8_t value)
{
	sys_write8(value, DEVICE_MMIO_GET(dev) + offset);
}

/*
 * Configure ECM handling for WWDT error signals.
 *
 * On R-Car, WWDT error outputs from RT-core WWDT channels 0 to 19
 * are not routed directly to the reset controller or to the CPU as NMI.
 * Instead, each WWDT error signal is first captured by the ECM.
 *
 * Depending on the WWDT error mode, this function selects the corresponding
 * ECM error ID for either:
 *   - WWDT reset-output error signal, or
 *   - WWDT NMI-output error signal.
 *
 * After selecting the ECM error ID, error detection is enabled in ECM.
 *
 * If the Zephyr watchdog timeout configuration requests a SoC reset
 * through WDT_FLAG_RESET_SOC, ECM reset generation is enabled for the
 * selected WWDT error. In this case, when WWDT detects an error such as
 * overflow, ECM forwards an internal reset request to the reset controller.
 *
 * Otherwise, ECM interrupt notification is enabled for the selected WWDT
 * error so that the error is reported as an ECM notification instead of
 * generating a SoC reset.
 */
static int rcar_wwdt_error_detection_config(const struct device *dev)
{
	const struct wwdt_rcar_config *config = dev->config;
	struct wwdt_rcar_data *data = dev->data;
	e_ecm_error_id_t wwdt_ecm_id;
	int ret;

	if (config->channel_id >= RCAR_WWDT_RT_CHANNEL_COUNT) {
		LOG_ERR("WWDT channel id %u is out of range", config->channel_id);
		return -EINVAL;
	}

	if (config->error_mode == ERM_RESET_MODE) {
		wwdt_ecm_id = wwdt_reset_ecm_id_map[config->channel_id];
	} else {
		wwdt_ecm_id = wwdt_nmi_ecm_id_map[config->channel_id];
	}

	ret = R_ECM_SetDetection(wwdt_ecm_id, R_WWDT_ECM_DETECTION_ENABLE);
	if (ret != 0) {
		return -EIO;
	}

	if ((data->timeout_cfg.flags & (WDT_FLAG_RESET_SOC)) != 0U) {
		/* Enable Generating internal reset when WWDT overflow by ECM */
		ret = R_ECM_SetReset(wwdt_ecm_id, R_WWDT_ECM_DETECTION_ENABLE);
		if (ret != 0) {
			return -EIO;
		}
	} else {
		/* Notification interrupt ECM  */
		ret = R_ECM_SetInterruptNotification(wwdt_ecm_id, 1);
		if (ret != 0) {
			return -EIO;
		}
	}

	return 0;
}

/*
 * WWDT only supports a 75% warning interrupt. WWDT overflow/error
 * handling is performed by ECM, which can generate a reset or NMI/ECM
 * notification.
 *
 * Therefore, the Zephyr watchdog timeout callback is mapped to the WWDT 75%
 * warning interrupt, not to a WWDT overflow interrupt. The callback gives
 * software an early warning before the WWDT reaches its overflow condition.
 */
static void wwdt_rcar_75percent_isr(const void *arg)
{
	const struct device *dev = arg;
	const struct wwdt_rcar_config *config = dev->config;
	struct wwdt_rcar_data *data = dev->data;

	/*
	 * Wait for one RCLK pulse with for the 75% interrupt signal to deassert
	 * and prevent the handler from being triggered repeatedly.
	 */
	k_busy_wait(DIV_ROUND_UP(USEC_PER_SEC, config->rclk_freq_hz));

	if (data->timeout_cfg.callback != NULL) {
		data->timeout_cfg.callback(dev, 0U);
	}
}

/*
 * Install one WWDT timeout configuration.
 *
 * Convert Zephyr window.max to a supported WWDT overflow period and map the
 * Zephyr refresh window to one of the supported WWDT window sizes: 25%, 50%,
 * 75%, or 100%.
 *
 * WWDT only supports a local 75% warning interrupt. It does not generate
 * a direct CPU interrupt on overflow; overflow/error is captured by ECM and
 * may be routed to reset or notification/NMI.
 *
 * Therefore, cfg->callback is supported only through the 75% warning
 * interrupt. If the 75% interrupt is disabled, timeout callback on overflow
 * is not supported.
 */
static int wdt_renesas_rcar_install_timeout(const struct device *dev,
					    const struct wdt_timeout_cfg *cfg)
{
	struct wwdt_rcar_data *data = dev->data;
	const struct wwdt_rcar_config *config = dev->config;
	uint64_t requested_cycles;
	uint32_t supported_cycles;
	uint8_t percentage;

	if (cfg->window.min > cfg->window.max || cfg->window.max == 0) {
		return -EINVAL;
	}

	if ((cfg->flags & ~WDT_RENESAS_RCAR_SUPPORTED_FLAGS) != 0) {
		return -ENOTSUP;
	}

	/* The timeout callback can only be serviced by the 75% warning interrupt. */
	if (cfg->callback != NULL && !config->irq_75p_enabled) {
		LOG_ERR("Timeout callback requires the 75 percent warning interrupt; "
			"set 'warning-irq-enabled' on this WWDT node");
		return -ENOTSUP;
	}

	/*
	 * Neither a callback nor a SoC reset was requested: no Zephyr code
	 * responds to the timeout. The WWDT overflow is only reported to the ECM,
	 * which must handle it out-of-band (e.g. NMI).
	 */
	if (cfg->callback == NULL && (cfg->flags & WDT_FLAG_RESET_MASK) == WDT_FLAG_RESET_NONE) {
		LOG_WRN("No Zephyr timeout response configured; WWDT overflow will "
			"only be reported to the ECM");
	}

	if (data->started) {
		LOG_ERR("Cannot change timeout settings after wdt setup");
		return -EBUSY;
	}

	/* Only one timeout configuration is supported when starting the watchdog trigger. */
	if (data->timeout_installed == true) {
		return -ENOMEM;
	}

	/*
	 * Convert the requested timeout from milliseconds to CNTCLK cycles.
	 * Round up to ensure that the selected WWDT overflow period is not
	 * shorter than the timeout requested by the caller.
	 */
	requested_cycles = DIV_ROUND_UP((uint64_t)cfg->window.max * config->rclk_freq_hz, 1000U);

	if (requested_cycles > WWDT_TIMEOUT_MAX_CYCLES) {
		LOG_ERR("Settings window.max exceeds maximum allowed value");
		return -EINVAL;
	}

	/*
	 * WWDT overflow periods are powers of two from 2^9 to 2^16 CNTCLK cycles.
	 * Select the first supported period greater than or equal to the request.
	 */
	supported_cycles = WWDT_TIMEOUT_MIN_CYCLES;
	data->overflow_code = WWDT_TIMEOUT_512;

	while (requested_cycles > supported_cycles) {
		supported_cycles <<= 1;
		data->overflow_code++;
	}

	percentage = ((uint64_t)(cfg->window.max - cfg->window.min) * 100U) / cfg->window.max;

	switch (percentage) {
	case 25:
		data->window_code = WINDOW_25P;
		break;

	case 50:
		data->window_code = WINDOW_50P;
		break;

	case 75:
		data->window_code = WINDOW_75P;
		break;

	case 100:
		data->window_code = WINDOW_100P;
		break;

	default:
		LOG_ERR("The window size value provided for configuration is not supported.");
		return -ENOTSUP;
	}

	data->timeout_cfg = *cfg;
	data->timeout_installed = true;

	return 0;
}

/*
 * Configure and start the WWDT.
 *
 * The timeout must be installed before setup. This function configures the
 * overflow period, refresh window, ECM error handling, optional NMI mode,
 * and optional 75% warning interrupt, then starts the asynchronous WWDT
 * counter. Once started, the WWDT configuration cannot be changed.
 */
static int wdt_renesas_rcar_setup(const struct device *dev, uint8_t options)
{
	const struct wwdt_rcar_config *config = dev->config;
	struct wwdt_rcar_data *data = dev->data;
	uint8_t mode;
	int ret;

	if (!data->timeout_installed) {
		LOG_ERR("Wdt timeout should be installed before");
		return -EFAULT;
	}

	if (data->started) {
		return -EBUSY;
	}

	/* Pausing the watchdog timer when the CPU is halted
	 * by the debugger is supported by default.
	 */
	if ((options & (WDT_OPT_PAUSE_IN_SLEEP)) != 0U) {
		LOG_ERR("Wdt pause in sleep mode not supported");
		return -ENOTSUP;
	}

	mode = rcar_wwdt_read8(dev, WDTA0MD);

	mode |= WSIZE(data->window_code & WDTA0WS_MASK);

	mode |= WDTA0OVF(data->overflow_code);

	ret = rcar_wwdt_error_detection_config(dev);
	if (ret != 0) {
		LOG_ERR("Failed to configure WWDT error detection: %d", ret);
		return ret;
	}

	if (config->error_mode == ERM_NMI_MODE) {
		mode &= ~WDTA0ERM;
	}

	if (config->irq_75p_enabled == true) {
		mode |= WDTA0WIE;
	}

	rcar_wwdt_write8(dev, WDTA0MD, mode);

	/*
	 * H'AC starts the asynchronous WWDT counter.
	 * According to the HWUM, the counter may take up to 3 CNTCLK
	 * cycles to start after the write completes.
	 */
	rcar_wwdt_write8(dev, WDTA0WDTE, WDTA0RUN);

	data->started = true;

	return 0;
}

/*
 * Refresh the WWDT counter.
 *
 * R-Car WWDT supports only one timeout configuration for each instance WWDT.
 * Writing the run trigger value restarts the counter,
 * provided that the watchdog is running and the refresh occurs
 * within the configured WWDT window.
 */
static int wdt_renesas_rcar_feed(const struct device *dev, int channel_id)
{
	struct wwdt_rcar_data *data = dev->data;

	if (!data->started || channel_id != 0) {
		return -EINVAL;
	}

	rcar_wwdt_write8(dev, WDTA0WDTE, WDTA0RUN);

	return 0;
}

/*
 * Disable the WWDT.
 *
 * On R-Car WWDT cannot be stopped by software after it has been started.
 * It returns to the stopped reset state only after a reset of the WWDT
 * reset domain.
 */
static int wdt_renesas_rcar_disable(const struct device *dev)
{
	struct wwdt_rcar_data *data = dev->data;

	/* WWDT is stopped with reset conditions after reset release. */
	if (!data->started) {
		LOG_ERR("wdt has not been enabled yet");
		return -EFAULT;
	}

	LOG_ERR("wdt can not be stopped once it has started");
	return -EPERM;
}

/*****************************************************************************
 * Device initialization
 ****************************************************************************/
static int wwdt_rcar_init(const struct device *dev)
{
	const struct wwdt_rcar_config *config = dev->config;
	int ret;

	ret = reset_line_assert_dt(&config->ms_reset_id0);
	if (ret < 0) {
		return ret;
	}

	ret = reset_line_assert_dt(&config->ms_reset_id1);
	if (ret < 0) {
		return ret;
	}

	ret = reset_line_deassert_dt(&config->ms_reset_id0);
	if (ret < 0) {
		return ret;
	}

	ret = reset_line_deassert_dt(&config->ms_reset_id1);
	if (ret < 0) {
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	if (config->irq_75p_enabled == true) {
		config->irq_config_func(dev);
	}

	return 0;
}

static DEVICE_API(wdt, wdt_renesas_rcar_api) = {
	.setup = wdt_renesas_rcar_setup,
	.disable = wdt_renesas_rcar_disable,
	.install_timeout = wdt_renesas_rcar_install_timeout,
	.feed = wdt_renesas_rcar_feed,
};

/*
 * Convert the string property into an enum at build time.
 */
#define RCAR_WWDT_CHANNEL_FROM_BASE(base)                                                          \
	((uint8_t)(((uintptr_t)(base) - (uintptr_t)R_WWDT0_BASE) /                                 \
		   (uintptr_t)R_WWDT_CHANNEL_ADDR_STRIDE))

#define WWDT_RCAR_ERROR_MODE(n)                                                                    \
	COND_CODE_1(							\
		DT_INST_ENUM_HAS_VALUE(n, error_mode, nmi),	\
		(ERM_NMI_MODE),				\
		(ERM_RESET_MODE))

#define WWDT_RCAR_CONFIG_FUNC(n, compat)                                                           \
	static void irq_config_func_##compat##n(const struct device *dev)                          \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), wwdt_rcar_75percent_isr,    \
			    DEVICE_DT_INST_GET(n), 0);                                             \
                                                                                                   \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

#define WWDT_RCAR_IRQ_CONFIG_DEFINE(n, compat)                                                     \
	IF_ENABLED(DT_INST_PROP(n, warning_irq_enabled), (WWDT_RCAR_CONFIG_FUNC(n, compat)))

#define WWDT_RCAR_IRQ_CONFIG_GET(n, compat)                                                        \
	COND_CODE_1(DT_INST_PROP(n, warning_irq_enabled), (irq_config_func_##compat##n), (NULL))

#define WWDT_RCAR_INIT(n, compat)                                                                  \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, resets) == 2, "WWDT requires exactly two reset domains"); \
                                                                                                   \
	WWDT_RCAR_IRQ_CONFIG_DEFINE(n, compat)                                                     \
                                                                                                   \
	static struct wwdt_rcar_data wwdt_rcar_data_##compat##n;                                   \
                                                                                                   \
	static const struct wwdt_rcar_config wwdt_rcar_config_##compat##n = {                      \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.ms_reset_id0 = RESET_DT_SPEC_INST_GET_BY_IDX(n, 0),                               \
		.ms_reset_id1 = RESET_DT_SPEC_INST_GET_BY_IDX(n, 1),                               \
		.rclk_freq_hz = DT_INST_PROP_BY_PHANDLE(n, clocks, clock_frequency),               \
		.channel_id = RCAR_WWDT_CHANNEL_FROM_BASE(DT_INST_REG_ADDR(n)),                    \
		.irq_75p_enabled = DT_INST_PROP(n, warning_irq_enabled),                           \
		.error_mode = WWDT_RCAR_ERROR_MODE(n),                                             \
		.irq_config_func = WWDT_RCAR_IRQ_CONFIG_GET(n, compat),                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, wwdt_rcar_init, NULL, &wwdt_rcar_data_##compat##n,                \
			      &wwdt_rcar_config_##compat##n, POST_KERNEL,                          \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_renesas_rcar_api);

DT_INST_FOREACH_STATUS_OKAY_VARGS(WWDT_RCAR_INIT, DT_DRV_COMPAT)
