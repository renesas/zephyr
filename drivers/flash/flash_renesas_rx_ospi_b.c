/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_ospi_b_nor

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/flash_controller/xspi.h>
#include <zephyr/drivers/clock_control/renesas_rx_cgc.h>
#include <r_spi_flash_api.h>
#include <r_ospi_b.h>
#include "spi_nor.h"
#include "jesd216.h"

LOG_MODULE_REGISTER(flash_renesas_rx_ospi_b, CONFIG_FLASH_LOG_LEVEL);

#define USEC_PER_MSEC 1000U

#define SECTOR_OFFSET(sector) (sector * SPI_NOR_SECTOR_SIZE)

#define _GET_SECTOR_ADDRESS(channel, sector)                                                       \
	(uint8_t *)(CONCAT(BSP_FEATURE_OSPI_B_DEVICE_, channel, _START_ADDRESS) +                  \
		    SECTOR_OFFSET(sector))

#define GET_SECTOR_ADDRESS(channel, sector) _GET_SECTOR_ADDRESS(channel, sector)

#define STATUS_BYTE_TO_LENGTH(status_byte) (status_byte + 1)

#define UNSUPPORTED_COMMAND 0
#define UNUSED_COMMAND      0x00

#define SPI_NOR_DUMMY_NONE 0U
#define SPI_OCMD_DUMMY_RD  20U
#define SPI_OCMD_DUMMY_INF 4U
#define SPI_NOR_DUMMY_DATA 0x00U

#define RESET_HIGH_BEFORE_CS_LOW_US 15
#define RESET_LOW_PULSE_WIDTH_US    20

/* Flash erase value */
#define ERASE_VALUE 0xFF

/** Operation timeout values (in milliseconds) */
#define WRITE_STATUS_MAX_TIMEOUT_MS 40
#define SECTOR_ERASE_MAX_TIMEOUT_MS 400
#define BLOCK_ERASE_MAX_TIMEOUT_MS  2000
#define CHIP_ERASE_MAX_TIMEOUT_MS   300000
#define PAGE_PROGRAM_MAX_TIMEOUT_MS 1

#define STATUS_REG_ADDRESS              0x00000000
#define CR2_INTERFACE_MODE_BITS_ADDRESS 0x00000000
#define CR2_INTERFACE_MODE_DTR_OPI      0x02
#define CR2_INTERFACE_MODE_STR_OPI      0x01
#define CR2_INTERFACE_MODE_SPI          0x00

enum flash_command_length {
	FLASH_NO_COMMAND = 0,
	FLASH_1_BYTE_COMMAND,
	FLASH_2_BYTE_COMMAND,
};

enum flash_address_length {
	FLASH_NO_ADDRESS = 0,
	FLASH_1_BYTE_ADDRESS,
	FLASH_2_BYTE_ADDRESS,
	FLASH_3_BYTE_ADDRESS,
	FLASH_4_BYTE_ADDRESS,
};

enum flash_data_length {
	FLASH_NO_DATA = 0,
	FLASH_1_BYTE_DATA,
	FLASH_2_BYTE_DATA,
	FLASH_3_BYTE_DATA,
	FLASH_4_BYTE_DATA,
	FLASH_5_BYTE_DATA,
	FLASH_6_BYTE_DATA,
	FLASH_7_BYTE_DATA,
	FLASH_8_BYTE_DATA,
};

struct flash_renesas_rx_ospi_b_data {
	ospi_b_instance_ctrl_t ospi_b_ctrl;
	struct k_sem sem;
};

struct flash_renesas_rx_ospi_b_config {
	const struct device *clock_dev;
	struct clock_control_rx_subsys_cfg clock_config;
	const struct pinctrl_dev_config *pcfg;
	volatile R_XSPI0_Type *ospi_pregs;
	size_t flash_size;
	uint32_t max_frequency;
	int data_mode; /* SPI or QSPI or OSPI */
	int data_rate; /* DTR or STR */
	ospi_b_timing_setting_t ospi_b_timing_setting;
	ospi_b_extended_cfg_t ospi_b_extended_config;
	spi_flash_cfg_t spi_flash_config;
	struct flash_parameters ospi_b_param;
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout pages_layout;
#endif
};

static spi_flash_erase_command_t erase_command_set[] = {
	{.command = SPI_NOR_CMD_SE_4B, .size = SPI_NOR_SECTOR_SIZE},
	{.command = SPI_NOR_CMD_BE_4B, .size = SPI_NOR_BLOCK_SIZE},
	{.command = SPI_NOR_CMD_CE, .size = SPI_FLASH_ERASE_SIZE_CHIP_ERASE}};

static spi_flash_erase_command_t high_speed_erase_command_set[] = {
	{.command = SPI_NOR_OCMD_SE, .size = SPI_NOR_SECTOR_SIZE},
	{.command = SPI_NOR_OCMD_BE, .size = SPI_NOR_BLOCK_SIZE},
	{.command = SPI_NOR_OCMD_CE, .size = SPI_FLASH_ERASE_SIZE_CHIP_ERASE}};

static ospi_b_table_t erase_command_table = {
	.p_table = erase_command_set,
	.length = ARRAY_SIZE(erase_command_set),
};

static ospi_b_table_t high_speed_erase_command_table = {
	.p_table = high_speed_erase_command_set,
	.length = ARRAY_SIZE(high_speed_erase_command_set),
};

/* These command sets are used for MX25LW51245G IC */
static ospi_b_xspi_command_set_t ospi_command_set[] = {
	{
		.protocol = SPI_FLASH_PROTOCOL_1S_1S_1S,
		.frame_format = OSPI_B_FRAME_FORMAT_STANDARD,
		.latency_mode = OSPI_B_LATENCY_MODE_FIXED,
		.command_bytes = OSPI_B_COMMAND_BYTES_1,
		.address_bytes = SPI_FLASH_ADDRESS_BYTES_4,

		.read_command = SPI_NOR_CMD_READ_FAST_4B,
		.program_command = SPI_NOR_CMD_PP_4B,
		.write_enable_command = SPI_NOR_CMD_WREN,
		.status_command = SPI_NOR_CMD_RDSR,

		.read_dummy_cycles = SPI_NOR_DUMMY_RD,
		.program_dummy_cycles = SPI_NOR_DUMMY_NONE,
		.status_dummy_cycles = SPI_NOR_DUMMY_NONE,

		.address_msb_mask = 0xF0,

		.status_needs_address = false,
		.status_address = 0x00,
		.status_address_bytes = FLASH_NO_ADDRESS,

		.p_erase_commands = &erase_command_table,
	},
	{
		.protocol = SPI_FLASH_PROTOCOL_8D_8D_8D,
		.frame_format = OSPI_B_FRAME_FORMAT_XSPI_PROFILE_1,
		.latency_mode = OSPI_B_LATENCY_MODE_FIXED,
		.command_bytes = OSPI_B_COMMAND_BYTES_2,
		.address_bytes = SPI_FLASH_ADDRESS_BYTES_4,

		.read_command = SPI_NOR_OCMD_DTR_RD,
		.program_command = SPI_NOR_OCMD_PAGE_PRG,
		.write_enable_command = SPI_NOR_OCMD_WREN,
		.status_command = SPI_NOR_OCMD_RDSR,

		.read_dummy_cycles = SPI_OCMD_DUMMY_RD,
		.program_dummy_cycles = SPI_NOR_DUMMY_NONE,
		.status_dummy_cycles = SPI_NOR_DUMMY_REG_OCTAL,

		.address_msb_mask = 0xF0,

		.status_needs_address = true,
		.status_address = 0x00,
		.status_address_bytes = SPI_FLASH_ADDRESS_BYTES_4,

		.p_erase_commands = &high_speed_erase_command_table,
	},
};

static ospi_b_table_t ospi_command_table = {
	.p_table = ospi_command_set,
	.length = ARRAY_SIZE(ospi_command_set),
};

static void acquire_device(const struct device *dev)
{
	struct flash_renesas_rx_ospi_b_data *dev_data = dev->data;

	k_sem_take(&dev_data->sem, K_FOREVER);
}

static void release_device(const struct device *dev)
{
	struct flash_renesas_rx_ospi_b_data *dev_data = dev->data;

	k_sem_give(&dev_data->sem);
}

static int flash_renesas_rx_ospi_b_write_enable(ospi_b_instance_ctrl_t *p_instance_ctrl)
{
	ospi_b_xspi_command_set_t const *const p_cmd_set = p_instance_ctrl->p_cmd_set;
	int err;

	/* If the command is 0x00, then skip sending the write enable. */
	if (0 == p_cmd_set->write_enable_command) {
		return 0;
	}

	spi_flash_direct_transfer_t set_write_enable_command = {
		.command = p_cmd_set->write_enable_command,
		.command_length = (uint8_t)p_cmd_set->command_bytes,
		.address_length = FLASH_NO_ADDRESS,
		.address = STATUS_REG_ADDRESS,
		.data_length = FLASH_NO_DATA,
		.dummy_cycles = SPI_NOR_DUMMY_NONE,
	};

	err = R_OSPI_B_DirectTransfer(p_instance_ctrl, &set_write_enable_command,
				      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Failed to send write enable command (%d)", err);
		return -EIO;
	}

	spi_flash_direct_transfer_t read_status_command = {
		.command = p_cmd_set->status_command,
		.command_length = (uint8_t)p_cmd_set->command_bytes,
		.address_length =
			(p_cmd_set->status_needs_address)
				? (uint8_t)STATUS_BYTE_TO_LENGTH(p_cmd_set->status_address_bytes)
				: FLASH_NO_ADDRESS,
		.address = STATUS_REG_ADDRESS,
		.data_length = FLASH_1_BYTE_DATA,
		.dummy_cycles = p_cmd_set->status_dummy_cycles,
	};

	err = R_OSPI_B_DirectTransfer(p_instance_ctrl, &read_status_command,
				      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Failed to read status command (%d)", err);
		return -EIO;
	}

	if (((read_status_command.data >> p_instance_ctrl->p_cfg->write_enable_bit) & 0x1U) != 1U) {
		LOG_ERR("Write enable failed %d", read_status_command.data);
		return -EIO;
	}

	return 0;
}

static int flash_renesas_rx_ospi_b_set_protocol_to_opi(ospi_b_instance_ctrl_t *p_instance_ctrl)
{
	fsp_err_t err;

	spi_flash_direct_transfer_t opi_set_command = {
		.command = SPI_NOR_CMD_WR_CFGREG2,
		.command_length = (uint8_t)FLASH_1_BYTE_COMMAND,
		.address = CR2_INTERFACE_MODE_BITS_ADDRESS,
		.address_length = (uint8_t)FLASH_4_BYTE_ADDRESS,
		.data = CR2_INTERFACE_MODE_DTR_OPI,
		.data_length = (uint8_t)FLASH_1_BYTE_DATA,
		.dummy_cycles = SPI_NOR_DUMMY_NONE,
	};

	spi_flash_direct_transfer_t config_reg2_read_command_opi = {
		.command = SPI_NOR_OCMD_RD_CFGREG2,
		.command_length = (uint8_t)FLASH_2_BYTE_COMMAND,
		.address = CR2_INTERFACE_MODE_BITS_ADDRESS,
		.address_length = (uint8_t)FLASH_4_BYTE_ADDRESS,
		.data = SPI_NOR_DUMMY_DATA,
		.data_length = (uint8_t)FLASH_1_BYTE_DATA,
		.dummy_cycles = SPI_NOR_DUMMY_REG_OCTAL,
	};

	/* Transfer write enable command */
	err = flash_renesas_rx_ospi_b_write_enable(p_instance_ctrl);
	if (err != 0) {
		LOG_ERR("Write enable failed");
		return -EIO;
	}

	/* Change Flash IC to DTR OSPI protocol */
	err = R_OSPI_B_DirectTransfer(p_instance_ctrl, &opi_set_command,
				      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Failed to write config reg 2 (%d)", err);
		return -EIO;
	}

	/* Change OSPI Module to Octa-SPI protocol */
	err = R_OSPI_B_SpiProtocolSet(p_instance_ctrl, SPI_FLASH_PROTOCOL_8D_8D_8D);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Failed to set OSPI protocol (%d)", err);
		return -EIO;
	}

	/* Read back the configuration register 2 */
	err = R_OSPI_B_DirectTransfer(p_instance_ctrl, &config_reg2_read_command_opi,
				      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Failed to read config reg 2 (%d)", err);
		return -EIO;
	}

	if ((uint8_t)(config_reg2_read_command_opi.data) != CR2_INTERFACE_MODE_DTR_OPI) {
		LOG_ERR("Config reg 2 data incorrect: 0x%02x", config_reg2_read_command_opi.data);
		return -EIO;
	}

	return 0;
}

static int flash_renesas_rx_ospi_b_wait_operation(ospi_b_instance_ctrl_t *p_ctrl,
						  uint32_t timeout_ms)
{
	spi_flash_status_t status = {0};
	uint32_t loop_per_msec = USEC_PER_MSEC / 50; /* 50us per loop */
	uint32_t max_loop_count = timeout_ms * loop_per_msec;

	status.write_in_progress = true;
	while (status.write_in_progress && max_loop_count > 0) {
		k_usleep(50);
		/* Get device status */
		R_OSPI_B_StatusGet(p_ctrl, &status);
		max_loop_count--;
	}

	if (status.write_in_progress) {
		LOG_ERR("Operation timeout after %u ms", timeout_ms);
		return -ETIMEDOUT;
	}

	return 0;
}

static int flash_renesas_rx_ospi_b_setup_calibrate_data(ospi_b_instance_ctrl_t *p_instance_ctrl)
{
	uint32_t autocalibration_data[] = {0xFFFF0000U, 0x0800FF00U, 0xFF0000F7U, 0x00F708F7U};
	const ospi_b_extended_cfg_t *const p_cfg_extend =
		(ospi_b_extended_cfg_t *)(p_instance_ctrl->p_cfg->p_extend);

	/* Verify auto-calibration data */
	if (memcmp((uint8_t *)(p_cfg_extend->p_autocalibration_preamble_pattern_addr),
		   &autocalibration_data, sizeof(autocalibration_data)) != 0) {
		fsp_err_t err;

		/* Erase the flash sector that stores auto-calibration data */
		err = R_OSPI_B_Erase(
			p_instance_ctrl,
			(uint8_t *)(p_cfg_extend->p_autocalibration_preamble_pattern_addr),
			SPI_NOR_SECTOR_SIZE);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Erase flash sector failed");
			return -EIO;
		}

		/* Wait until erase operation completes */
		err = flash_renesas_rx_ospi_b_wait_operation(p_instance_ctrl,
							     SECTOR_ERASE_MAX_TIMEOUT_MS);
		if (err != 0) {
			LOG_DBG("Erase operation timeout");
			return -ETIMEDOUT;
		}

		/* Write auto-calibration data to the flash */
		err = R_OSPI_B_Write(
			p_instance_ctrl, (uint8_t *)&autocalibration_data,
			(uint8_t *)(p_cfg_extend->p_autocalibration_preamble_pattern_addr),
			sizeof(autocalibration_data));
		if (err != FSP_SUCCESS) {
			LOG_DBG("Write auto-calibration data failed");
			return -EIO;
		}

		/* Wait until write operation completes */
		err = flash_renesas_rx_ospi_b_wait_operation(p_instance_ctrl,
							     PAGE_PROGRAM_MAX_TIMEOUT_MS);
		if (err != 0) {
			LOG_DBG("Write operation timeout");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static inline bool flash_renesas_rx_ospi_b_is_valid_address(const struct device *dev, off_t offset,
							    size_t len)
{
	const struct flash_renesas_rx_ospi_b_config *config = dev->config;

	if(offset < 0 || len == 0) {
		return false;
	}

	if(offset >= config->flash_size) {
		return false;
	}

	return (offset + len) <= config->flash_size;
}

static int flash_renesas_rx_ospi_b_erase(const struct device *dev, off_t offset, size_t len)
{
	struct flash_renesas_rx_ospi_b_data *ospi_b_data = dev->data;
	const struct flash_renesas_rx_ospi_b_config *config = dev->config;
	uint32_t erase_size, erase_timeout;
	fsp_err_t err;
	uint32_t flash_base_address;
	struct flash_pages_info page_info_start, page_info_end;
	int ret;

	if (!len) {
		return 0;
	} else if (len % SPI_NOR_SECTOR_SIZE != 0) {
		LOG_ERR("Wrong sector size 0x%zu", len);
		return -EINVAL;
	}

	if (!flash_renesas_rx_ospi_b_is_valid_address(dev, offset, len)) {
		LOG_ERR("Address or size exceeds expected values: "
			"Address 0x%lx, size %zu",
			(long)offset, len);
		return -EINVAL;
	}

	/* check offset and len that valid in sector layout */
	ret = flash_get_page_info_by_offs(dev, offset, &page_info_start);
	if ((ret != 0) || (offset != page_info_start.start_offset)) {
		LOG_ERR("The offset 0x%lx is not aligned with the starting sector", (long)offset);
		return -EINVAL;
	}

	ret = flash_get_page_info_by_offs(dev, (offset + len), &page_info_end);
	if ((ret != 0) || ((offset + len) != page_info_end.start_offset)) {
		LOG_ERR("The size %zu is not aligned with the ending sector", len);
		return -EINVAL;
	}

	if (ospi_b_data->ospi_b_ctrl.channel == OSPI_B_DEVICE_NUMBER_0) {
		flash_base_address = BSP_FEATURE_OSPI_B_DEVICE_0_START_ADDRESS;
	} else {
		flash_base_address = BSP_FEATURE_OSPI_B_DEVICE_1_START_ADDRESS;
	}

	acquire_device(dev);

	while (len > 0) {
		if (offset == 0 && len == config->flash_size) {
			/* Chip erase */
			LOG_INF("Chip Erase");

			erase_size = SPI_FLASH_ERASE_SIZE_CHIP_ERASE;
			erase_timeout = CHIP_ERASE_MAX_TIMEOUT_MS;
		} else if (len >= SPI_NOR_BLOCK_SIZE) {
			erase_size = SPI_NOR_BLOCK_SIZE;
			erase_timeout = BLOCK_ERASE_MAX_TIMEOUT_MS;
		} else {
			erase_size = SPI_NOR_SECTOR_SIZE;
			erase_timeout = SECTOR_ERASE_MAX_TIMEOUT_MS;
		}

		err = R_OSPI_B_Erase(&ospi_b_data->ospi_b_ctrl,
				     (uint8_t *)(flash_base_address + offset), erase_size);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Erase failed at address 0x%lx, size %u, err: %d", offset,
				erase_size, err);
			ret = -EIO;
			break;
		}

		err = flash_renesas_rx_ospi_b_wait_operation(&ospi_b_data->ospi_b_ctrl,
							     erase_timeout);
		if (err != 0) {
			LOG_ERR("Erase operation timeout");
			ret = -ETIMEDOUT;
			break;
		}

		offset += erase_size;
		len -= MIN(len, erase_size);
	}

	release_device(dev);

	return ret;
}

static int flash_renesas_rx_ospi_b_write(const struct device *dev, off_t offset, const void *data,
					 size_t len)
{
	struct flash_renesas_rx_ospi_b_data *ospi_b_data = dev->data;
	const struct flash_renesas_rx_ospi_b_config *config = dev->config;
	fsp_err_t err;
	int ret = 0;
	size_t size;
	const uint8_t *p_src;
	uint32_t flash_base_address;

	if (!len) {
		return 0;
	}

	if (data != NULL) {
		p_src = data;
	} else {
		LOG_ERR("The data buffer is NULL");
		return -EINVAL;
	}

	if (!flash_renesas_rx_ospi_b_is_valid_address(dev, offset, len)) {
		LOG_ERR("Address or size exceeds expected values: "
			"Address 0x%lx, size %zu",
			(long)offset, len);
		return -EINVAL;
	}

	if (ospi_b_data->ospi_b_ctrl.channel == OSPI_B_DEVICE_NUMBER_0) {
		flash_base_address = BSP_FEATURE_OSPI_B_DEVICE_0_START_ADDRESS;
	} else {
		flash_base_address = BSP_FEATURE_OSPI_B_DEVICE_1_START_ADDRESS;
	}

	acquire_device(dev);

	while (len > 0) {
		size = MIN(len, config->spi_flash_config.page_size_bytes);

		err = R_OSPI_B_Write(&ospi_b_data->ospi_b_ctrl, p_src,
				     (uint8_t *)(flash_base_address + offset), size);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Write failed at address 0x%lx, size %zu", offset, size);
			ret = -EIO;
			break;
		}

		err = flash_renesas_rx_ospi_b_wait_operation(&ospi_b_data->ospi_b_ctrl,
							     PAGE_PROGRAM_MAX_TIMEOUT_MS);
		if (err != 0) {
			LOG_ERR("Write operation timeout");
			ret = -ETIMEDOUT;
			break;
		}

		len -= size;
		offset += size;
		p_src = p_src + size;
	}

	release_device(dev);

	return ret;
}

static int flash_renesas_rx_ospi_b_read(const struct device *dev, off_t offset, void *data,
					size_t len)
{
	struct flash_renesas_rx_ospi_b_data *ospi_b_data = dev->data;
	uint32_t flash_base_address;

	if (!len) {
		return 0;
	}

	if (!flash_renesas_rx_ospi_b_is_valid_address(dev, offset, len)) {
		LOG_ERR("Address or size exceeds expected values: "
			"Address 0x%lx, size %zu",
			(long)offset, len);
		return -EINVAL;
	}

	acquire_device(dev);

	if (ospi_b_data->ospi_b_ctrl.channel == OSPI_B_DEVICE_NUMBER_0) {
		flash_base_address = BSP_FEATURE_OSPI_B_DEVICE_0_START_ADDRESS;
	} else {
		flash_base_address = BSP_FEATURE_OSPI_B_DEVICE_1_START_ADDRESS;
	}

	memcpy(data, (uint8_t *)(flash_base_address) + offset, len);

	release_device(dev);

	return 0;
}

static const struct flash_parameters *
flash_renesas_rx_ospi_b_get_parameters(const struct device *dev)
{
	const struct flash_renesas_rx_ospi_b_config *config = dev->config;

	return &config->ospi_b_param;
}

static int flash_renesas_rx_ospi_b_get_size(const struct device *dev, uint64_t *size)
{
	const struct flash_renesas_rx_ospi_b_config *config = dev->config;
	*size = (uint64_t)config->flash_size;

	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
void flash_renesas_rx_ospi_b_page_layout(const struct device *dev,
					 const struct flash_pages_layout **layout,
					 size_t *layout_size)
{
	const struct flash_renesas_rx_ospi_b_config *config = dev->config;
	*layout = &config->pages_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#if defined(CONFIG_FLASH_JESD216_API)
static int flash_renesas_rx_ospi_b_sfdp_read(const struct device *dev, off_t offset, void *data,
					     size_t len)
{
	struct flash_renesas_rx_ospi_b_data *ospi_b_data = dev->data;
	const struct flash_renesas_rx_ospi_b_config *config = dev->config;
	size_t size;
	spi_flash_direct_transfer_t sfdp_read_command;
	fsp_err_t err;

	if (len == 0) {
		return 0;
	}

	if (data == NULL) {
		LOG_ERR("The data buffer is NULL");
		return -EINVAL;
	}

	switch (config->data_mode) {
	case XSPI_SPI_MODE:
		sfdp_read_command.command = JESD216_CMD_READ_SFDP;
		sfdp_read_command.command_length = FLASH_1_BYTE_COMMAND;
		sfdp_read_command.address_length = FLASH_3_BYTE_ADDRESS;
		sfdp_read_command.data_length = FLASH_8_BYTE_DATA;
		sfdp_read_command.dummy_cycles = SPI_NOR_DUMMY_RD;
		break;
	case XSPI_OCTO_MODE:
		sfdp_read_command.command = JESD216_OCMD_READ_SFDP;
		sfdp_read_command.command_length = FLASH_2_BYTE_COMMAND;
		sfdp_read_command.address_length = FLASH_4_BYTE_ADDRESS;
		sfdp_read_command.data_length = FLASH_8_BYTE_DATA;
		sfdp_read_command.dummy_cycles = SPI_OCMD_DUMMY_RD;
		break;
	default:
		LOG_ERR("Unsupported data mode");
		return -ENOTSUP;
	}

	acquire_device(dev);

	while (len > 0) {
		size = MIN(len, FLASH_8_BYTE_DATA); /* 8 bytes is the maximum for direct transfer */
		sfdp_read_command.address = offset;
		sfdp_read_command.data_length = size;

		err = R_OSPI_B_DirectTransfer(&ospi_b_data->ospi_b_ctrl, &sfdp_read_command,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			LOG_ERR("SFDP read failed at address 0x%lx, size %zu ", (long)offset, size);
			release_device(dev);
			return -EIO;
		}

		if (config->data_mode == XSPI_OCTO_MODE) {
			/* With MX25LW512, data will be returned in word unit
			 * so it need to be byte-swapped before copy to output */

			uint8_t *p_data = (uint8_t *)&sfdp_read_command.data;
			for (size_t i = 0; i + 1 < size; i += 2) {
				uint8_t temp = p_data[i];
				p_data[i] = p_data[i + 1];
				p_data[i + 1] = temp;
			}
		}

		memcpy(data, &sfdp_read_command.data, size);

		len -= size;
		offset += size;
		data = (uint8_t *)data + size;
	}

	release_device(dev);

	return 0;
}

static int flash_renesas_rx_ospi_b_read_jedec_id(const struct device *dev, uint8_t *id)
{
	struct flash_renesas_rx_ospi_b_data *ospi_b_data = dev->data;
	const struct flash_renesas_rx_ospi_b_config *config = dev->config;
	spi_flash_direct_transfer_t read_id_command;
	fsp_err_t err;

	if (id == NULL) {
		LOG_ERR("The ID buffer is NULL");
		return -EINVAL;
	}

	switch (config->data_mode) {
	case XSPI_SPI_MODE:
		read_id_command.command = JESD216_CMD_READ_ID;
		read_id_command.command_length = FLASH_1_BYTE_COMMAND;
		read_id_command.address = 0;
		read_id_command.address_length = FLASH_NO_ADDRESS;
		read_id_command.data_length = JESD216_READ_ID_LEN;
		read_id_command.dummy_cycles = SPI_NOR_DUMMY_NONE;
		break;
	case XSPI_OCTO_MODE:
		read_id_command.command = JESD216_OCMD_READ_ID;
		read_id_command.command_length = FLASH_2_BYTE_COMMAND;
		read_id_command.address = 0;
		read_id_command.address_length = FLASH_4_BYTE_ADDRESS;
		read_id_command.data_length = JESD216_READ_ID_LEN * 2;
		read_id_command.dummy_cycles = SPI_OCMD_DUMMY_INF;
		break;
	default:
		LOG_ERR("Unsupported data mode");
		return -ENOTSUP;
	}

	acquire_device(dev);

	err = R_OSPI_B_DirectTransfer(&ospi_b_data->ospi_b_ctrl, &read_id_command,
				      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		LOG_ERR("JEDEC ID read failed (err = %d)", err);
		release_device(dev);
		return -EIO;
	}

	if (config->data_mode == XSPI_OCTO_MODE) {
		/**
		 * With MX25LW512, ID data will be return in STR mode
		 * But this controller only work with OCTO DTR mode
		 * So we need to remove duplicated data
		 */

		uint8_t *p_data = (uint8_t *)&read_id_command.data;

		for (int i = 0; i < JESD216_READ_ID_LEN; i++) {
			*(p_data + i) = *(p_data + i * 2);
		}
	}

	memcpy(id, &read_id_command.data, JESD216_READ_ID_LEN);

	release_device(dev);

	return 0;
}
#endif /* CONFIG_FLASH_JESD216_API */

#if defined(CONFIG_FLASH_EX_OP_ENABLED)
static int flash_renesas_rx_ospi_b_ex_op(const struct device *dev, uint16_t code,
					 const uintptr_t in, void *out)
{
	struct flash_renesas_rx_ospi_b_data *ospi_b_data = dev->data;
	const struct flash_renesas_rx_ospi_b_config *config = dev->config;
	spi_flash_direct_transfer_t command;
	uint16_t enable_reset_code, execute_reset_code;
	uint8_t command_length;
	fsp_err_t err;

	ARG_UNUSED(in);
	ARG_UNUSED(out);

	if (code != FLASH_EX_OP_RESET) {
		LOG_ERR("Unsupported ex op code %d", code);
		return -ENOTSUP;
	}

	switch (config->data_mode) {
	case XSPI_SPI_MODE:
		enable_reset_code = SPI_NOR_CMD_RESET_EN;
		execute_reset_code = SPI_NOR_CMD_RESET_MEM;
		command_length = FLASH_1_BYTE_COMMAND;
		break;
	case XSPI_OCTO_MODE:
		enable_reset_code = SPI_NOR_OCMD_RESET_EN;
		execute_reset_code = SPI_NOR_OCMD_RESET_MEM;
		command_length = FLASH_2_BYTE_COMMAND;
		break;
	default:
		LOG_ERR("Unsupported data mode");
		return -ENOTSUP;
	}

	command.command_length = command_length;
	command.address_length = FLASH_NO_ADDRESS;
	command.data_length = FLASH_NO_DATA;
	command.dummy_cycles = SPI_NOR_DUMMY_NONE;

	/* Send Reset Enable command */
	command.command = enable_reset_code;
	err = R_OSPI_B_DirectTransfer(&ospi_b_data->ospi_b_ctrl, &command,
				      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Reset Enable command Failed");
		return -EIO;
	}

	/* Send Reset Memory command */
	command.command = execute_reset_code;
	err = R_OSPI_B_DirectTransfer(&ospi_b_data->ospi_b_ctrl, &command,
				      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Reset Memory command Failed");
		return -EIO;
	}

	return 0;
}
#endif /* CONFIG_FLASH_EX_OP_ENABLED */

static DEVICE_API(flash, flash_renesas_rx_ospi_b_api) = {
	.erase = flash_renesas_rx_ospi_b_erase,
	.write = flash_renesas_rx_ospi_b_write,
	.read = flash_renesas_rx_ospi_b_read,
	.get_parameters = flash_renesas_rx_ospi_b_get_parameters,
	.get_size = flash_renesas_rx_ospi_b_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_renesas_rx_ospi_b_page_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = flash_renesas_rx_ospi_b_sfdp_read,
	.read_jedec_id = flash_renesas_rx_ospi_b_read_jedec_id,
#endif /* CONFIG_FLASH_JESD216_API */
#if defined(CONFIG_FLASH_EX_OP_ENABLED)
	.ex_op = flash_renesas_rx_ospi_b_ex_op,
#endif /* CONFIG_FLASH_EX_OP_ENABLED */
};

static int flash_renesas_rx_ospi_b_init(const struct device *dev)
{
	const struct flash_renesas_rx_ospi_b_config *config = dev->config;
	struct flash_renesas_rx_ospi_b_data *ospi_b_data = dev->data;
	uint32_t clock_freq;
	int ret;
	fsp_err_t err;

	/* protocol/data_rate of XSPI checking */
	if (config->data_mode == XSPI_DUAL_MODE || config->data_mode == XSPI_QUAD_MODE) {
		LOG_ERR("XSPI DUAL/QUAD modes not supported");
		return -ENOTSUP;
	} else if (((config->data_mode != XSPI_OCTO_MODE) &&
		    (config->data_rate == XSPI_DTR_TRANSFER)) ||
		   ((config->data_mode == XSPI_OCTO_MODE) &&
		    (config->data_rate == XSPI_STR_TRANSFER))) {
		LOG_ERR("XSPI mode SPI/DTR or OPI/STR is not valid");
		return -ENOTSUP;
	}

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

	if ((config->data_mode == XSPI_SPI_MODE && clock_freq > config->max_frequency) ||
	    (config->data_mode == XSPI_OCTO_MODE && (clock_freq / 2) > config->max_frequency)) {
		LOG_ERR("Invalid clock frequency (%u)", clock_freq);
		return -EINVAL;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to configure pins (%d)", ret);
		return ret;
	}

	k_sem_init(&ospi_b_data->sem, 1, 1);

	/** Initialize the OSPI B module */
	err = R_OSPI_B_Open(&ospi_b_data->ospi_b_ctrl, &config->spi_flash_config);
	if (err != FSP_SUCCESS) {
		LOG_ERR("R_OSPI_B_Open failed");
		return -EIO;
	}

	/* Reset flash device by driving OM_RESET pin */
	config->ospi_pregs->LIOCTL_b.RSTCS0 = 0;
	k_usleep(RESET_LOW_PULSE_WIDTH_US);
	config->ospi_pregs->LIOCTL_b.RSTCS0 = 1;
	k_usleep(RESET_HIGH_BEFORE_CS_LOW_US);

	/* Setup calibrate data */
	err = flash_renesas_rx_ospi_b_setup_calibrate_data(&ospi_b_data->ospi_b_ctrl);
	if (err != 0) {
		LOG_ERR("Setup calibrate data Failed");
		return -EIO;
	}

	if (config->data_mode == XSPI_OCTO_MODE) {
		err = flash_renesas_rx_ospi_b_set_protocol_to_opi(
			&ospi_b_data->ospi_b_ctrl);
		if (err != 0) {
			LOG_ERR("Init OPI mode failed");
			return -EIO;
		}
	}

	return ret;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
#define FLASH_RENESAS_RX_OSPI_B_PAGES_LAYOUT(index)                                                \
	.pages_layout = {                                                                          \
		.pages_size = SPI_NOR_SECTOR_SIZE,                                                 \
		.pages_count = DT_INST_PROP(index, size) / SPI_NOR_SECTOR_SIZE,                    \
	}
#else
#define FLASH_RENESAS_RX_OSPI_B_PAGES_LAYOUT(index)
#endif

#define RENESAS_RX_OSPI_B_INIT(index)                                                              \
                                                                                                   \
	PINCTRL_DT_DEFINE(DT_INST_PARENT(index));                                                  \
                                                                                                   \
	static struct flash_renesas_rx_ospi_b_data ospi_b_data##index;                             \
                                                                                                   \
	static const struct flash_renesas_rx_ospi_b_config ospi_b_config##index = {                \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(index))),                 \
		.clock_config =                                                                    \
			{                                                                          \
				.mstp = (uint32_t)DT_CLOCKS_CELL(DT_INST_PARENT(index), mstp),     \
				.stop_bit =                                                        \
					(uint32_t)DT_CLOCKS_CELL(DT_INST_PARENT(index), stop_bit), \
			},                                                                         \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_INST_PARENT(index)),                          \
		.ospi_pregs = (R_XSPI0_Type *)DT_REG_ADDR(DT_INST_PARENT(index)),                  \
		.flash_size = DT_INST_PROP(index, size),                                           \
		.max_frequency = DT_INST_PROP(index, ospi_max_frequency),                          \
		.data_mode = DT_INST_PROP(index, protocol_mode),                                   \
		.data_rate = DT_INST_PROP(index, data_rate),                                       \
		.ospi_b_timing_setting =                                                           \
			{                                                                          \
				.command_to_command_interval =                                     \
					DT_INST_PROP(index, command_interval),                     \
				.cs_pullup_lag = DT_INST_PROP(index, pull_up_delay),               \
				.cs_pulldown_lead = DT_INST_PROP(index, pull_down_lead),           \
				.sdr_drive_timing = DT_INST_PROP(index, sdr_drive_timing),         \
				.sdr_sampling_edge = DT_INST_PROP(index, sdr_sampling_edge),       \
				.sdr_sampling_delay = DT_INST_PROP(index, sdr_sampling_delay),     \
				.ddr_sampling_extension =                                          \
					DT_INST_PROP(index, ddr_sampling_extension),               \
			},                                                                         \
		.ospi_b_extended_config =                                                          \
			{                                                                          \
				.ospi_b_unit = DT_PROP(DT_INST_PARENT(index), unit),               \
				.channel = DT_INST_REG_ADDR(index),                                \
				.p_timing_settings = &ospi_b_config##index.ospi_b_timing_setting,  \
				.p_xspi_command_set = &ospi_command_table,                         \
				.data_latch_delay_clocks = OSPI_B_DS_TIMING_DELAY_NONE,            \
				.p_autocalibration_preamble_pattern_addr = GET_SECTOR_ADDRESS(     \
					DT_INST_PROP_BY_IDX(index, reg, 0),                        \
					DT_INST_PROP(index, auto_calib_pattern_address_sector)),   \
			},                                                                         \
		.spi_flash_config =                                                                \
			{                                                                          \
				.spi_protocol = SPI_FLASH_PROTOCOL_1S_1S_1S,                       \
				.read_mode = UNUSED_COMMAND,                                       \
				.address_bytes = SPI_FLASH_ADDRESS_BYTES_4,                        \
				.dummy_clocks = 0,                                                 \
				.page_program_address_lines = 0,                                   \
				.write_status_bit = DT_INST_PROP(index, write_status_bit),         \
				.write_enable_bit = DT_INST_PROP(index, write_enable_bit),         \
				.page_size_bytes = DT_INST_PROP(index, max_write_size),            \
				.page_program_command = UNUSED_COMMAND,                            \
				.write_enable_command = UNUSED_COMMAND,                            \
				.status_command = UNUSED_COMMAND,                                  \
				.read_command = UNUSED_COMMAND,                                    \
				.xip_enter_command = UNSUPPORTED_COMMAND,                          \
				.xip_exit_command = UNSUPPORTED_COMMAND,                           \
				.erase_command_list_length = 0,                                    \
				.p_erase_command_list = NULL,                                      \
				.p_extend = &ospi_b_config##index.ospi_b_extended_config,          \
			},                                                                         \
		.ospi_b_param =                                                                    \
			{                                                                          \
				.write_block_size = DT_INST_PROP(index, write_block_size),         \
				.erase_value = ERASE_VALUE,                                        \
				.caps =                                                            \
					{                                                          \
						.no_explicit_erase = false,                        \
					},                                                         \
			},                                                                         \
		FLASH_RENESAS_RX_OSPI_B_PAGES_LAYOUT(index)};                                      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, flash_renesas_rx_ospi_b_init, NULL, &ospi_b_data##index,      \
			      &ospi_b_config##index, POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,      \
			      &flash_renesas_rx_ospi_b_api);

DT_INST_FOREACH_STATUS_OKAY(RENESAS_RX_OSPI_B_INIT)
