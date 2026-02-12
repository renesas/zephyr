/*
 * Copyright (c) 2024-2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_ra_ospi_b_nor

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include "flash_renesas_ra_ospi_b.h"

LOG_MODULE_REGISTER(flash_renesas_ra_ospi_b, CONFIG_FLASH_LOG_LEVEL);

struct flash_renesas_ra_ospi_b_data {
	ospi_b_instance_ctrl_t ospi_b_ctrl;
	spi_flash_cfg_t ospi_b_cfg;
	ospi_b_timing_setting_t ospi_b_timing_settings;
	ospi_b_xspi_command_set_t ospi_b_command_set_table[2];
	ospi_b_extended_cfg_t ospi_b_config_extend;
	ospi_b_table_t const ospi_b_command_set;
	struct flash_pages_layout *pages_layouts;
	size_t pages_layouts_size;
	struct k_sem sem;
};

struct flash_renesas_ra_ospi_b_config {
	const struct device *clock_dev;
	struct clock_control_ra_subsys_cfg clock_subsys;
	const struct pinctrl_dev_config *pcfg;
	const struct flash_parameters flash_parameters;
	uint32_t start_address;
	size_t flash_size;
	int protocol;
	int data_rate;
	uint32_t max_frequency;
	const uint32_t erase_block_size;
	const bool spi_word_data_swapped;
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	const struct flash_pages_layout layout;
#endif
};

static void acquire_device(const struct device *dev)
{
	struct flash_renesas_ra_ospi_b_data *dev_data = dev->data;

	k_sem_take(&dev_data->sem, K_FOREVER);
}

static void release_device(const struct device *dev)
{
	struct flash_renesas_ra_ospi_b_data *dev_data = dev->data;

	k_sem_give(&dev_data->sem);
}

static int flash_renesas_ra_ospi_b_wait_operation(ospi_b_instance_ctrl_t *p_ctrl, uint32_t timeout)
{
	spi_flash_status_t status = {RESET_VALUE};

	status.write_in_progress = true;
	while (status.write_in_progress) {
		/* Get device status */
		R_OSPI_B_StatusGet(p_ctrl, &status);
		if (timeout == RESET_VALUE) {
			LOG_DBG("Time out for operation");
			return -EIO;
		}
		k_sleep(K_USEC(50));
		timeout--;
	}

	return 0;
}

static int flash_renesas_ra_ospi_b_write_enable(ospi_b_instance_ctrl_t *p_ctrl)
{
	spi_flash_direct_transfer_t transfer = {RESET_VALUE};
	fsp_err_t err;

	/* Transfer write enable command */
	transfer = (SPI_FLASH_PROTOCOL_EXTENDED_SPI == p_ctrl->spi_protocol)
			   ? direct_transfer[TRANSFER_WRITE_ENABLE_SPI]
			   : direct_transfer[TRANSFER_WRITE_ENABLE_OPI];
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}
	/* Read Status Register */
	transfer = (SPI_FLASH_PROTOCOL_EXTENDED_SPI == p_ctrl->spi_protocol)
			   ? direct_transfer[TRANSFER_READ_STATUS_SPI]
			   : direct_transfer[TRANSFER_READ_STATUS_OPI];
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	/* Check Write Enable bit in Status Register */
	if ((err != FSP_SUCCESS) || (SPI_NOR_WREN_MASK != (transfer.data & SPI_NOR_WREN_MASK))) {
		return -EIO;
	}

	return 0;
}

static int flash_renesas_ra_ospi_b_setup_calibrate_data(ospi_b_instance_ctrl_t *p_ctrl,
							uint8_t *address_calib_sector,
							bool spi_word_data_swapped,
							uint32_t sector_size)
{
	uint32_t autocalibration_data[4];

	if (spi_word_data_swapped) {
		autocalibration_data[0] = 0xFFFF0000U;
		autocalibration_data[1] = 0x0800FF00U;
		autocalibration_data[2] = 0xFF0000F7U;
		autocalibration_data[3] = 0x00F708F7U;
	} else {
		autocalibration_data[0] = 0xFFFF0000U;
		autocalibration_data[1] = 0x000800FFU;
		autocalibration_data[2] = 0x00FFF700U;
		autocalibration_data[3] = 0xF700F708U;
	}

	/* Verify auto-calibration data */
	if (memcmp(address_calib_sector, &autocalibration_data, sizeof(autocalibration_data)) !=
	    0) {
		fsp_err_t err;

		/* Erase the flash sector that stores auto-calibration data */
		err = R_OSPI_B_Erase(p_ctrl, address_calib_sector, sector_size);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Erase the flash sector Failed");
			return -EIO;
		}

		/* Wait until erase operation completes */
		err = flash_renesas_ra_ospi_b_wait_operation(p_ctrl, TIME_ERASE_256K);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Wait for erase operation completes Failed");
			return -EIO;
		}

		/* Write auto-calibration data to the flash */
		err = R_OSPI_B_Write(p_ctrl, (uint8_t *)&autocalibration_data,
				     (uint8_t *)address_calib_sector, sizeof(autocalibration_data));
		if (err != FSP_SUCCESS) {
			LOG_DBG("Write auto-calibration data Failed");
			return -EIO;
		}

		/* Wait until write operation completes */
		err = flash_renesas_ra_ospi_b_wait_operation(p_ctrl, TIME_WRITE);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Wait for write operation completes Failed");
			return -EIO;
		}
	}

	return 0;
}

static int flash_renesas_ra_ospi_b_spi_mode_init(ospi_b_instance_ctrl_t *p_ctrl,
						 spi_flash_cfg_t *p_cfg)
{
	/* By default, the flash device is in SPI mode, so it is necessary to open the OSPI module
	 * in SPI mode
	 */
	spi_flash_direct_transfer_t transfer = {RESET_VALUE};
	fsp_err_t err;

	/* Open OSPI module */
	err = R_OSPI_B_Open(p_ctrl, p_cfg);
	if (err != FSP_SUCCESS) {
		LOG_DBG("OSPI open Failed");
		return -EIO;
	}

	/* Switch OSPI module to 1S-1S-1S mode to configure flash device */
	err = R_OSPI_B_SpiProtocolSet(p_ctrl, SPI_FLASH_PROTOCOL_EXTENDED_SPI);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Switch OSPI module to 1S-1S-1S mode Failed");
		return -EIO;
	}

	/* Reset flash device by driving OM_RESET pin */
	R_XSPI0->LIOCTL_b.RSTCS0 = 0;
	k_sleep(K_USEC(500));
	R_XSPI0->LIOCTL_b.RSTCS0 = 1;
	k_sleep(K_NSEC(50));

	/* Transfer write enable command */
	err = flash_renesas_ra_ospi_b_write_enable(p_ctrl);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Enable write Failed");
		return -EIO;
	}

#if defined(CONFIG_BOARD_EK_RA8M1) || defined(CONFIG_BOARD_EK_RA8D1)
	/* Write to CFR2V to configure Address Byte Length and Memory Array Read Latency */
	transfer = direct_transfer[TRANSFER_WRITE_CFR2V_SPI];
	transfer.address_length = ADDRESS_LENGTH_THREE;
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Configure Address Byte Length and Memory Array Read Latency Failed");
		return -EIO;
	}

	/* Transfer write enable command */
	err = flash_renesas_ra_ospi_b_write_enable(p_ctrl);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Enable write Failed");
		return -EIO;
	}

	/* Write to CFR3V to configure Volatile Register Read Latency */
	transfer = direct_transfer[TRANSFER_WRITE_CFR3V_SPI];
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Configure Volatile Register Read Latency Failed");
		return -EIO;
	}

	/* Read back and verify CFR2V register data */
	transfer = direct_transfer[TRANSFER_READ_CFR2V_SPI];
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Read back CFR2V register Failed");
		return -EIO;
	}

	if (DATA_CFR2V_REGISTER != (uint8_t)transfer.data) {
		LOG_DBG("Verify CFR2V register data Failed");
		return -EIO;
	}

	/* Read back and verify CFR3V register data */
	transfer = direct_transfer[TRANSFER_READ_CFR3V_SPI];
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Read back CFR3V register Failed");
		return -EIO;
	}

	if (DATA_CFR3V_REGISTER != (uint8_t)transfer.data) {
		LOG_DBG("Verify CFR3V register data Failed");
		return -EIO;
	}
#elif defined(CONFIG_BOARD_EK_RA8P1)

	/* Write to ADDR 00000300H of CR2 to configure dummy cycle */
	transfer = direct_transfer[TRANSFER_WRITE_CR2_300H_SPI];
	transfer.data = DATA_SET_CR2_300H;
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Write to CR2 register Failed");
		return -EIO;
	}

	/* Read back and verify CR2 register data */
	transfer = direct_transfer[TRANSFER_READ_CR2_300H_SPI];
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Read back CR2 register Failed");
		return -EIO;
	}

	if (DATA_SET_CR2_300H != (uint8_t)transfer.data) {
		LOG_DBG("Verify CR2 register data Failed");
		return -EIO;
	}

#elif defined(CONFIG_BOARD_EK_RA8D2)
	/* Enter 4-bytes address */
	transfer = direct_transfer[TRANSFER_ENTER_4BYTES_ADDRESS];
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Enter 4-bytes address Failed");
		return -EIO;
	}

	/* Write to ADDR 00000001H of CR to configure dummy cycle */
	transfer = direct_transfer[TRANSFER_WRITE_CR_01H_SPI];
	transfer.data = DATA_SET_CR_01H;
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Write to CR register Failed");
		return -EIO;
	}

	/* Read back and verify CR register data */
	transfer = direct_transfer[TRANSFER_READ_CR_01H_SPI];
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Read back CR register Failed");
		return -EIO;
	}

	if (DATA_SET_CR_01H != (uint8_t)transfer.data) {
		LOG_DBG("Data mismatched in SPI mode");
		return -EIO;
	}

#endif
	return 0;
}

static int flash_renesas_ra_ospi_b_set_protocol_to_opi(ospi_b_instance_ctrl_t *p_ctrl,
						       const struct device *dev)
{
	spi_flash_direct_transfer_t transfer = {RESET_VALUE};
	fsp_err_t err;

	/* Transfer write enable command */
	err = flash_renesas_ra_ospi_b_write_enable(p_ctrl);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Enable write Failed");
		return -EIO;
	}

#if defined(CONFIG_BOARD_EK_RA8M1) || defined(CONFIG_BOARD_EK_RA8D1)
	/* Write to CFR5V Register to Configure flash device interface mode */
	transfer = direct_transfer[TRANSFER_WRITE_CFR5V_SPI];
	transfer.data = DATA_SET_OPI_CFR5V_REGISTER;
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Configure flash device interface mode Failed");
		return -EIO;
	}

	/* Switch OSPI module mode to OPI mode */
	err = R_OSPI_B_SpiProtocolSet(p_ctrl, SPI_FLASH_PROTOCOL_8D_8D_8D);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Switch to OPI mode Failed");
		return -EIO;
	}

	/* Read back and verify CFR5V register data */
	transfer = direct_transfer[TRANSFER_READ_CFR5V_OPI];
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Read back CFR5V register Failed");
		return -EIO;
	}

	if (DATA_SET_OPI_CFR5V_REGISTER != (uint8_t)transfer.data) {
		LOG_DBG("CFR5V register data is Incorrect");
		return -EIO;
	}
#elif defined(CONFIG_BOARD_EK_RA8P1)

	/* Change to DOPI mode */
	transfer = direct_transfer[TRANSFER_WRITE_CR2_000H_SPI];
	transfer.data = DATA_SET_OPI_CR2_000H;
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Write CR2 register Failed");
		return -EIO;
	}

	/* Switch OSPI module mode to OPI mode */
	err = R_OSPI_B_SpiProtocolSet(p_ctrl, SPI_FLASH_PROTOCOL_8D_8D_8D);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Switch to OPI mode Failed");
		return -EIO;
	}

	/* Read back and verify CR2 register data */
	transfer = direct_transfer[TRANSFER_READ_CR2_000H_OPI];
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Read back CR2 register Failed");
		return -EIO;
	}

	if (DATA_SET_OPI_CR2_000H != (uint8_t)transfer.data) {
		LOG_DBG("Data mismatched in OPI mode");
		return -EIO;
	}

#elif defined(CONFIG_BOARD_EK_RA8D2)
	/* Change to DOPI mode */
	transfer = direct_transfer[TRANSFER_WRITE_CR_00H_SPI];
	transfer.data = DATA_SET_OPI_CR_00H;
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Change to DOPI mode Failed");
		return -EIO;
	}

	/* Switch OSPI module mode to OSPI mode */
	err = R_OSPI_B_SpiProtocolSet(p_ctrl, SPI_FLASH_PROTOCOL_8D_8D_8D);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Switch to OPI mode Failed");
		return -EIO;
	}

	/* Read back and verify CR register data */
	transfer = direct_transfer[TRANSFER_READ_CR_00H_OPI];
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		LOG_DBG("Read back CR register Failed");
		return -EIO;
	}

	if (DATA_SET_OPI_CR_00H != (uint8_t)transfer.data) {
		LOG_DBG("Data mismatched in OPI mode");
		return -EIO;
	}

#endif
	return 0;
}

static inline bool flash_renesas_ra_ospi_b_is_valid_address(const struct device *dev, off_t offset,
							    size_t len)
{
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;

	return (offset >= 0 && (offset < (config->flash_size - config->erase_block_size)) &&
		(len <= ((config->flash_size - config->erase_block_size) - offset)));
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)

void flash_renesas_ra_ospi_b_page_layout(const struct device *dev,
					 const struct flash_pages_layout **layout,
					 size_t *layout_size)
{
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;

	*layout = &config->layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#if defined(CONFIG_FLASH_JESD216_API)
static int flash_renesas_ra_ospi_b_read_device_id(ospi_b_instance_ctrl_t *p_ctrl,
						  uint8_t *const p_id)
{
	spi_flash_direct_transfer_t transfer = {RESET_VALUE};
	fsp_err_t err;

	/* Read and check flash device ID */
	transfer = (SPI_FLASH_PROTOCOL_EXTENDED_SPI == p_ctrl->spi_protocol)
			   ? direct_transfer[TRANSFER_READ_DEVICE_ID_SPI]
			   : direct_transfer[TRANSFER_READ_DEVICE_ID_OPI];
	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	/* Get flash device ID */
#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1) ||                            \
	defined(CONFIG_BOARD_EK_RA8D2)
	memcpy(p_id, &transfer.data, sizeof(transfer.data));
#elif defined(CONFIG_BOARD_EK_RA8P1)
	for (int i = 0; i < JESD216_READ_ID_LEN; i++) {
		*(p_id + i) = *((uint8_t *)(&transfer.data_u64) + i * 2);
	}
#endif

	return 0;
}

static int flash_renesas_ra_ospi_b_read_jedec_id(const struct device *dev, uint8_t *id)
{
	struct flash_renesas_ra_ospi_b_data *ospi_b_data = dev->data;
	int ret;

	if (id == NULL) {
		return -EINVAL;
	}

	acquire_device(dev);

	ret = flash_renesas_ra_ospi_b_read_device_id(&ospi_b_data->ospi_b_ctrl, id);
	if (ret) {
		LOG_ERR("Failed to read jedec id");
	} else {
		LOG_INF("Manuf ID = %02x   Memory Type = %02x   Memory Density = %02x   ID Length "
			"= %02x\n",
			id[0], id[1], id[2], id[3]);
	}

	release_device(dev);

	return ret;
}

static int flash_renesas_ra_ospi_b_sfdp_read(const struct device *dev, off_t offset, void *data,
					     size_t len)
{
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;
	struct flash_renesas_ra_ospi_b_data *ospi_b_data = dev->data;
	spi_flash_direct_transfer_t transfer = {RESET_VALUE};
	size_t size;
	fsp_err_t err;

	if (len == 0) {
		return 0;
	}

	if (data == NULL) {
		LOG_ERR("The data buffer is NULL");
		return -EINVAL;
	}
	acquire_device(dev);

	if (ospi_b_data->ospi_b_ctrl.spi_protocol == SPI_FLASH_PROTOCOL_EXTENDED_SPI) {
		transfer = direct_transfer[TRANSFER_READ_SFDP_ID_SPI];
	} else {
		transfer = direct_transfer[TRANSFER_READ_SFDP_ID_OPI];
	}

	while (len > 0) {
		size = len > transfer.data_length ? transfer.data_length : len;
		transfer.address = offset;
		transfer.data_length = size;

		err = R_OSPI_B_DirectTransfer(&ospi_b_data->ospi_b_ctrl, &transfer,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);

		if (err != FSP_SUCCESS) {
			LOG_ERR("Failed to read SFDP id");
			release_device(dev);
			return -EIO;
		}

		if (config->spi_word_data_swapped) {
			uint8_t *dst = (uint8_t *)data;
			uint8_t *src = (uint8_t *)&transfer.data;

			for (size_t i = 0; i < ((size + 1) / 2); i++) {
				dst[i * 2] = src[(i * 2) + 1];
				dst[(i * 2) + 1] = src[i * 2];
			}
		} else {
			memcpy(data, &transfer.data, size);
		}

		len -= size;
		offset += size;
		data = (uint8_t *)data + size;
	}

	release_device(dev);

	return 0;
}
#endif /* CONFIG_FLASH_JESD216_API */

static int flash_renesas_ra_ospi_b_erase(const struct device *dev, off_t offset, size_t len)
{
	struct flash_renesas_ra_ospi_b_data *ospi_b_data = dev->data;
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;
	uint32_t erase_size, erase_timeout;
	fsp_err_t err;
	struct flash_pages_info page_info_start, page_info_end;
	int ret;

	if (!len) {
		return 0;
	} else if (len % SPI_NOR_SECTOR_SIZE != 0) {
		LOG_ERR("Wrong sector size 0x%x", len);
		return -EINVAL;
	}

	if (!flash_renesas_ra_ospi_b_is_valid_address(dev, offset, len)) {
		LOG_ERR("Address or size exceeds expected values: "
			"Address 0x%lx, size %u",
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
		LOG_ERR("The size %u is not aligned with the ending sector", len);
		return -EINVAL;
	}

	acquire_device(dev);

	while (len > 0) {
		if (offset == 0 && len == config->flash_size) {
			/* Chip erase */
			LOG_INF("Chip Erase");

			erase_size = SPI_FLASH_ERASE_SIZE_CHIP_ERASE;
			erase_timeout = UINT32_MAX;
		} else {
			erase_size = config->erase_block_size;
			erase_timeout = TIME_ERASE_256K;
		}

		err = R_OSPI_B_Erase(&ospi_b_data->ospi_b_ctrl,
				     (uint8_t *)(config->start_address + offset), erase_size);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Erase at address 0x%lx, size %zu Failed", offset, erase_size);
			ret = -EIO;
			break;
		}

		err = flash_renesas_ra_ospi_b_wait_operation(&ospi_b_data->ospi_b_ctrl,
							     erase_timeout);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Wait for erase to finish timeout");
			ret = -EIO;
			break;
		}

		offset += erase_size;
		len -= len < erase_size ? len : erase_size;
	}

	release_device(dev);

	return ret;
}

static int flash_renesas_ra_ospi_b_read(const struct device *dev, off_t offset, void *data,
					size_t len)
{
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;

	if (!len) {
		return 0;
	}

	if (!flash_renesas_ra_ospi_b_is_valid_address(dev, offset, len)) {
		LOG_ERR("Address or size exceeds expected values: "
			"Address 0x%lx, size %zu",
			(long)offset, len);
		return -EINVAL;
	}

	memcpy(data, (uint32_t *)(config->start_address + offset), len);

	return 0;
}

static int flash_renesas_ra_ospi_b_write(const struct device *dev, off_t offset, const void *data,
					 size_t len)
{
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;
	struct flash_renesas_ra_ospi_b_data *ospi_b_data = dev->data;
	fsp_err_t err;
	int ret = 0;
	size_t size;
	const uint8_t *p_src;

	if (!len) {
		return 0;
	}

	if (data != NULL) {
		p_src = data;
	} else {
		LOG_ERR("The data buffer is NULL");
		return -EINVAL;
	}

	if (!flash_renesas_ra_ospi_b_is_valid_address(dev, offset, len)) {
		LOG_ERR("Address or size exceeds expected values: "
			"Address 0x%lx, size %zu",
			(long)offset, len);
		return -EINVAL;
	}

	if ((offset % config->flash_parameters.write_block_size) ||
	    (len % config->flash_parameters.write_block_size)) {
		return -EINVAL;
	}

	acquire_device(dev);

	while (len > 0) {
		size = len > ospi_b_data->ospi_b_cfg.page_size_bytes
			       ? ospi_b_data->ospi_b_cfg.page_size_bytes
			       : len;

		err = R_OSPI_B_Write(&ospi_b_data->ospi_b_ctrl, p_src,
				     (uint8_t *)(config->start_address + offset), size);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Write at address 0x%lx, size %zu", offset, size);
			ret = -EIO;
			break;
		}

		err = flash_renesas_ra_ospi_b_wait_operation(&ospi_b_data->ospi_b_ctrl, TIME_WRITE);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Wait for write to finish timeout");
			ret = -EIO;
			break;
		}

		len -= size;
		offset += size;
		p_src = p_src + size;
	}

	release_device(dev);

	return ret;
}

static const struct flash_parameters *
flash_renesas_ra_ospi_b_get_parameters(const struct device *dev)
{
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;

	return &config->flash_parameters;
}

static int flash_renesas_ra_ospi_b_get_size(const struct device *dev, uint64_t *size)
{
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;
	*size = (uint64_t)config->flash_size;

	return 0;
}

static DEVICE_API(flash, flash_renesas_ra_ospi_b_api) = {
	.erase = flash_renesas_ra_ospi_b_erase,
	.write = flash_renesas_ra_ospi_b_write,
	.read = flash_renesas_ra_ospi_b_read,
	.get_parameters = flash_renesas_ra_ospi_b_get_parameters,
	.get_size = flash_renesas_ra_ospi_b_get_size,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_renesas_ra_ospi_b_page_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = flash_renesas_ra_ospi_b_sfdp_read,
	.read_jedec_id = flash_renesas_ra_ospi_b_read_jedec_id,
#endif /* CONFIG_FLASH_JESD216_API */
};

static int flash_renesas_ra_ospi_b_init(const struct device *dev)
{
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;
	struct flash_renesas_ra_ospi_b_data *data = dev->data;
	uint32_t clock_freq;
	int ret;

	/* protocol/data_rate of OSPI checking */
	if (config->protocol == OSPI_DUAL_MODE || config->protocol == OSPI_QUAD_MODE) {
		LOG_ERR("OSPI mode DUAL|QUAD currently not support");
		return -ENOTSUP;
	} else if (((config->protocol != OSPI_OPI_MODE) &&
		    (config->data_rate == OSPI_DTR_TRANSFER)) ||
		   ((config->protocol == OSPI_OPI_MODE) &&
		    (config->data_rate == OSPI_STR_TRANSFER))) {
		LOG_ERR("OSPI mode SPI/DTR or OPI/STR is not valid");
		return -ENOTSUP;
	}

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->clock_subsys);
	if (ret < 0) {
		LOG_ERR("Could not initialize clock (%d)", ret);
		return ret;
	}

	ret = clock_control_get_rate(config->clock_dev,
				     (clock_control_subsys_t)&config->clock_subsys, &clock_freq);
	if (ret) {
		LOG_ERR("Failed to get clock frequency (%d)", ret);
		return ret;
	}

	if ((config->protocol == OSPI_SPI_MODE && (config->max_frequency / 2) < clock_freq) ||
	    (config->protocol == OSPI_OPI_MODE && (config->max_frequency) < clock_freq)) {
		LOG_ERR("Invalid clock frequency (%u)", clock_freq);
		return -EINVAL;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to configure pins (%d)", ret);
		return ret;
	}

	k_sem_init(&data->sem, 1, 1);

	data->ospi_b_config_extend.p_autocalibration_preamble_pattern_addr =
		(uint8_t *)(config->start_address +
			    (config->flash_size - config->erase_block_size));

	ret = flash_renesas_ra_ospi_b_spi_mode_init(&data->ospi_b_ctrl, &data->ospi_b_cfg);
	if (ret) {
		LOG_ERR("Init SPI mode failed");
		return ret;
	}

	/* Setup calibrate data */
	ret = flash_renesas_ra_ospi_b_setup_calibrate_data(
		&data->ospi_b_ctrl,
		data->ospi_b_config_extend.p_autocalibration_preamble_pattern_addr,
		config->spi_word_data_swapped, config->erase_block_size);
	if (ret != FSP_SUCCESS) {
		LOG_ERR("Setup calibrate data Failed");
		return -EIO;
	}

	if (config->protocol == OSPI_OPI_MODE) {
		ret = flash_renesas_ra_ospi_b_set_protocol_to_opi(&data->ospi_b_ctrl, dev);
		if (ret) {
			LOG_ERR("Init OPI mode failed");
			return ret;
		}
	}

	LOG_INF("Mode: %d	Freq: %u", config->protocol, clock_freq);

	return ret;
}
#ifdef CONFIG_FLASH_PAGE_LAYOUT
#define OSPI_B_RENESAS_RA_PAGE_LAYOUT(idx)                                                         \
	.layout = {                                                                                \
		.pages_count = DT_INST_PROP(idx, size) / 8 / DT_INST_PROP(idx, erase_block_size),  \
		.pages_size = DT_INST_PROP(idx, erase_block_size),                                 \
	}
#else
#define OSPI_B_RENESAS_RA_PAGE_LAYOUT(idx)
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#define OSPI_B_RENESAS_RA_INIT(idx)                                                                \
	PINCTRL_DT_DEFINE(DT_INST_PARENT(idx));                                                    \
	static const struct flash_renesas_ra_ospi_b_config ospi_b_config_##idx = {                 \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_INST_PARENT(0)),                              \
		.start_address = DT_INST_REG_ADDR(idx),                                            \
		.flash_size = DT_INST_PROP(idx, size) / 8,                                         \
		.protocol = DT_INST_PROP(idx, protocol_mode),                                      \
		.data_rate = DT_INST_PROP(idx, data_rate),                                         \
		.max_frequency = DT_INST_PROP(idx, ospi_max_frequency),                            \
		.erase_block_size = DT_INST_PROP(idx, erase_block_size),                           \
		.spi_word_data_swapped = DT_INST_PROP_OR(idx, spi_word_data_swapped, false),       \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(0))),                     \
		.clock_subsys =                                                                    \
			{                                                                          \
				.mstp = (uint32_t)DT_CLOCKS_CELL(DT_INST_PARENT(0), mstp),         \
				.stop_bit = (uint32_t)DT_CLOCKS_CELL(DT_INST_PARENT(0), stop_bit), \
			},                                                                         \
		.flash_parameters =                                                                \
			{                                                                          \
				.write_block_size = DT_INST_PROP(idx, write_block_size),           \
				.erase_value = 0xff,                                               \
			},                                                                         \
		OSPI_B_RENESAS_RA_PAGE_LAYOUT(idx),                                                \
	};                                                                                         \
                                                                                                   \
	static struct flash_renesas_ra_ospi_b_data ospi_b_data_##idx = {                           \
		.ospi_b_timing_settings =                                                          \
			{                                                                          \
				.command_to_command_interval = OSPI_B_COMMAND_INTERVAL_CLOCKS_2,   \
				.cs_pullup_lag = OSPI_B_COMMAND_CS_PULLUP_CLOCKS_NO_EXTENSION,     \
				.cs_pulldown_lead =                                                \
					OSPI_B_COMMAND_CS_PULLDOWN_CLOCKS_NO_EXTENSION,            \
				.sdr_drive_timing = OSPI_B_SDR_DRIVE_TIMING_BEFORE_CK,             \
				.sdr_sampling_edge = OSPI_B_CK_EDGE_FALLING,                       \
				.sdr_sampling_delay = OSPI_B_SDR_SAMPLING_DELAY_NONE,              \
				.ddr_sampling_extension = OSPI_B_DDR_SAMPLING_EXTENSION_7,         \
			},                                                                         \
		.ospi_b_command_set_table =                                                        \
			{                                                                          \
				{                                                                  \
					.protocol = SPI_FLASH_PROTOCOL_1S_1S_1S,                   \
					.frame_format = OSPI_B_FRAME_FORMAT_STANDARD,              \
					.latency_mode = OSPI_B_LATENCY_MODE_FIXED,                 \
					.command_bytes = OSPI_B_COMMAND_BYTES_1,                   \
					.address_bytes = SPI_FLASH_ADDRESS_BYTES_4,                \
					.address_msb_mask = 0xF0,                                  \
					.status_needs_address = false,                             \
					.status_address = 0U,                                      \
					.status_address_bytes = (spi_flash_address_bytes_t)0U,     \
					.p_erase_commands = &erase_commands,                       \
					COMMAND_SET_1S_1S_1S,                                      \
				},                                                                 \
				{                                                                  \
					.protocol = SPI_FLASH_PROTOCOL_8D_8D_8D,                   \
					.frame_format = OSPI_B_FRAME_FORMAT_XSPI_PROFILE_1,        \
					.latency_mode = OSPI_B_LATENCY_MODE_FIXED,                 \
					.command_bytes = OSPI_B_COMMAND_BYTES_2,                   \
					.address_bytes = SPI_FLASH_ADDRESS_BYTES_4,                \
					.address_msb_mask = 0xF0,                                  \
					.status_needs_address = true,                              \
					.status_address = 0x00,                                    \
					.status_address_bytes = SPI_FLASH_ADDRESS_BYTES_4,         \
					.p_erase_commands = &high_speed_erase_commands,            \
					COMMAND_SET_8D_8D_8D,                                      \
				},                                                                 \
			},                                                                         \
		.ospi_b_command_set =                                                              \
			{                                                                          \
				.p_table = &ospi_b_data_##idx.ospi_b_command_set_table,            \
				.length = 2U,                                                      \
			},                                                                         \
		.ospi_b_config_extend =                                                            \
			{                                                                          \
				.ospi_b_unit = 0,                                                  \
				.channel = DT_INST_PROP(idx, channel),                             \
				.data_latch_delay_clocks = OSPI_B_DS_TIMING_DELAY_NONE,            \
				.p_timing_settings = &ospi_b_data_##idx.ospi_b_timing_settings,    \
				.p_xspi_command_set = &ospi_b_data_##idx.ospi_b_command_set,       \
			},                                                                         \
		.ospi_b_cfg =                                                                      \
			{                                                                          \
				.spi_protocol = SPI_FLASH_PROTOCOL_1S_1S_1S,                       \
				.read_mode = SPI_FLASH_READ_MODE_STANDARD,                         \
				.address_bytes = SPI_FLASH_ADDRESS_BYTES_4,                        \
				.dummy_clocks = SPI_FLASH_DUMMY_CLOCKS_DEFAULT,                    \
				.page_program_address_lines = (spi_flash_data_lines_t)0U,          \
				.page_size_bytes = 64,                                             \
				.write_status_bit = 0,                                             \
				.write_enable_bit = 1,                                             \
				.erase_command_list_length = 0,                                    \
				.p_erase_command_list = NULL,                                      \
				.p_extend = &ospi_b_data_##idx.ospi_b_config_extend,               \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, flash_renesas_ra_ospi_b_init, NULL, &ospi_b_data_##idx,         \
			      &ospi_b_config_##idx, POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,       \
			      &flash_renesas_ra_ospi_b_api);

DT_INST_FOREACH_STATUS_OKAY(OSPI_B_RENESAS_RA_INIT);
