/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_ospi_b_psram

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/flash_controller/xspi.h>
#include <r_ospi_b.h>

LOG_MODULE_REGISTER(memc_renesas_rx_ospi_b_psram, CONFIG_MEMC_LOG_LEVEL);

#define RESET_LOW_PULSE_WIDTH_US    10U
#define RESET_HIGH_BEFORE_CS_LOW_US 10U

#define UNUSED_VALUE 0x00

struct memc_renesas_rx_ospi_b_psram_config {
	const struct device *clock_dev;
	struct clock_control_rx_subsys_cfg clock_config;
	const struct pinctrl_dev_config *pcfg;
	volatile R_XSPI0_Type *ospi_pregs;
	size_t flash_size;
	uint32_t max_frequency;
	spi_flash_cfg_t ospi_b_config;
	ospi_b_extended_cfg_t ospi_b_extended_config;
};

struct memc_renesas_rx_ospi_b_psram_data {
	ospi_b_instance_ctrl_t ospi_b_ctrl;
	struct k_sem sem;
};

static int memc_renesas_rx_ospi_b_psram_init(const struct device *dev)
{
	const struct memc_renesas_rx_ospi_b_psram_config *config = dev->config;
	struct memc_renesas_rx_ospi_b_psram_data *data = dev->data;
	uint32_t clock_freq;
	int ret;
	fsp_err_t err;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->clock_config);
	if (ret < 0) {
		LOG_ERR("Could not initialize clock (%d)", ret);
		return ret;
	}

	ret = clock_control_get_rate(config->clock_dev,
				     (clock_control_subsys_t)&config->clock_config, &clock_freq);
	if (ret) {
		LOG_ERR("Failed to get clock frequency (%d)", ret);
		return ret;
	}

	if (clock_freq > (config->max_frequency)) {
		LOG_ERR("Invalid clock frequency (%u)", clock_freq);
		return -EINVAL;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to configure pins (%d)", ret);
		return ret;
	}

	k_sem_init(&data->sem, 1, 1);

	/** Initialize the OSPI B module */
	err = R_OSPI_B_Open(&data->ospi_b_ctrl, &config->ospi_b_config);
	if (err != FSP_SUCCESS) {
		LOG_ERR("R_OSPI_B_Open failed");
		return -EIO;
	}

	/* Enable array address mode */
	config->ospi_pregs->CMCFGCS[config->ospi_b_extended_config.channel].CMCFG0_b.ARYAMD = 1;

	/* Reset flash device by driving OM_RESET pin */
	config->ospi_pregs->LIOCTL_b.RSTCS0 = 0;
	k_usleep(RESET_LOW_PULSE_WIDTH_US);
	config->ospi_pregs->LIOCTL_b.RSTCS0 = 1;
	k_usleep(RESET_HIGH_BEFORE_CS_LOW_US);

	return 0;
}

#define GET_SPI_PROTOCOL(idx)                                                                      \
	CONCAT(SPI_FLASH_PROTOCOL_, DT_INST_STRING_UPPER_TOKEN(idx, spi_protocol))

#define GET_FRAME_FORMAT(idx)                                                                      \
	CONCAT(OSPI_B_FRAME_FORMAT_, DT_INST_STRING_UPPER_TOKEN(idx, frame_format))

#define RENESAS_RX_OSPI_B_PSRAM_INIT(idx)                                                          \
                                                                                                   \
	PINCTRL_DT_DEFINE(DT_INST_PARENT(idx));                                                    \
                                                                                                   \
	static ospi_b_timing_setting_t ospi_b_timing_setting_##idx = {                             \
		.command_to_command_interval = DT_INST_PROP(idx, command_to_command_interval),     \
		.cs_pullup_lag = DT_INST_PROP(idx, cs_pullup_lag),                                 \
		.cs_pulldown_lead = DT_INST_PROP(idx, cs_pulldown_lead),                           \
		.sdr_drive_timing = OSPI_B_SDR_DRIVE_TIMING_BEFORE_CK,                             \
		.sdr_sampling_edge = OSPI_B_CK_EDGE_FALLING,                                       \
		.sdr_sampling_delay = OSPI_B_SDR_SAMPLING_DELAY_NONE,                              \
		.ddr_sampling_extension = DT_INST_PROP(idx, ddr_sampling_extension),               \
	};                                                                                         \
                                                                                                   \
	static ospi_b_xspi_command_set_t psram_ospi_b_command_set_##idx = {                        \
		.protocol = GET_SPI_PROTOCOL(idx),                                                 \
		.frame_format = GET_FRAME_FORMAT(idx),                                             \
		.latency_mode = DT_INST_PROP(idx, variable_latency),                               \
		.address_bytes = DT_INST_PROP(idx, address_bytes) - 1,                             \
		.address_msb_mask = 0xF0,                                                          \
                                                                                                   \
		.command_bytes = DT_INST_PROP(idx, command_bytes),                                 \
		.read_command = DT_INST_PROP(idx, read_command),                                   \
		.read_dummy_cycles = DT_INST_PROP(idx, read_dummy_cycles),                         \
		.program_command = DT_INST_PROP(idx, write_command),                               \
		.program_dummy_cycles = DT_INST_PROP(idx, write_dummy_cycles),                     \
                                                                                                   \
		/** Unused fields */                                                               \
		.write_enable_command = UNUSED_VALUE,                                              \
		.status_command = UNUSED_VALUE,                                                    \
		.row_load_command = UNUSED_VALUE,                                                  \
		.row_store_command = UNUSED_VALUE,                                                 \
		.status_dummy_cycles = UNUSED_VALUE,                                               \
		.row_load_dummy_cycles = UNUSED_VALUE,                                             \
		.row_store_dummy_cycles = UNUSED_VALUE,                                            \
		.status_needs_address = UNUSED_VALUE,                                              \
		.status_address = UNUSED_VALUE,                                                    \
		.status_address_bytes = UNUSED_VALUE,                                              \
		.p_erase_commands = NULL,                                                          \
	};                                                                                         \
                                                                                                   \
	static ospi_b_table_t psram_ospi_command_table_##idx = {                                   \
		.p_table = &psram_ospi_b_command_set_##idx,                                        \
		.length = 1,                                                                       \
	};                                                                                         \
                                                                                                   \
	static struct memc_renesas_rx_ospi_b_psram_data ospi_b_psram_data_##idx;                   \
                                                                                                   \
	static const struct memc_renesas_rx_ospi_b_psram_config ospi_b_psram_config_##idx = {      \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(idx))),                   \
		.clock_config =                                                                    \
			{                                                                          \
				.mstp = (uint32_t)DT_CLOCKS_CELL(DT_INST_PARENT(idx), mstp),       \
				.stop_bit =                                                        \
					(uint32_t)DT_CLOCKS_CELL(DT_INST_PARENT(idx), stop_bit),   \
			},                                                                         \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_INST_PARENT(idx)),                            \
		.ospi_pregs = (volatile R_XSPI0_Type *)DT_REG_ADDR(DT_INST_PARENT(idx)),                    \
		.flash_size = DT_INST_PROP(idx, size),                                             \
		.max_frequency = DT_INST_PROP(idx, ospi_max_frequency),                            \
		.ospi_b_config =                                                                   \
			{                                                                          \
				.spi_protocol = GET_SPI_PROTOCOL(idx),                             \
				.address_bytes = DT_INST_PROP(idx, address_bytes) - 1,             \
				.page_size_bytes = DT_INST_PROP(idx, write_size_bytes),            \
				.p_extend = &ospi_b_psram_config_##idx.ospi_b_extended_config,     \
			},                                                                         \
		.ospi_b_extended_config =                                                          \
			{                                                                          \
				.ospi_b_unit = DT_PROP(DT_INST_PARENT(idx), unit),                 \
				.channel = DT_INST_REG_ADDR(idx),                                  \
				.p_timing_settings = &ospi_b_timing_setting_##idx,                 \
				.p_xspi_command_set = &psram_ospi_command_table_##idx,             \
				.data_latch_delay_clocks = DT_INST_PROP(idx, ds_latch_delay),      \
				.p_autocalibration_preamble_pattern_addr =                         \
					(uint8_t *)DT_INST_PROP(idx,                               \
								auto_calibration_pattern_address), \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, &memc_renesas_rx_ospi_b_psram_init, NULL,                       \
			      &ospi_b_psram_data_##idx, &ospi_b_psram_config_##idx, POST_KERNEL,   \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(RENESAS_RX_OSPI_B_PSRAM_INIT)
