/*
 * Copyright (c) 2025-2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_xspi_nor

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/init.h>
#include <zephyr/arch/cache.h>
#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/flash_controller/xspi.h>
#include <zephyr/drivers/clock_control/renesas_rx_cgc.h>
#include <r_spi_flash_api.h>
#include <r_xspi_controller.h>
#include "spi_nor.h"
#include "jesd216.h"

LOG_MODULE_REGISTER(flash_renesas_rx_xspi, CONFIG_FLASH_LOG_LEVEL);

#define SECTOR_OFFSET(sector) (sector * SPI_NOR_SECTOR_SIZE)

#define XSPI_FLASH_BASE_ADDR(index) DT_INST_REG_ADDR(index)

#define XSPI_FLASH_ADDR_WITH_CALIB(index)                                                          \
	(uint8_t *)((XSPI_FLASH_BASE_ADDR(index) +                                                 \
		     SECTOR_OFFSET(DT_INST_PROP(index, auto_calib_pattern_address_sector))))

#define _GET_BSP_XSPI_BASE_ADDRESS(unit, channel)                                                  \
	BSP_FEATURE_XSPI_CONTROLLER_UNIT_##unit##_DEVICE_##channel##_START_ADDRESS

#define GET_BSP_XSPI_BASE_ADDRESS(unit, channel) _GET_BSP_XSPI_BASE_ADDRESS(unit, channel)

#define XSPI_BSP_FLASH_BASE_ADDR(index)                                                            \
	GET_BSP_XSPI_BASE_ADDRESS(DT_PROP(DT_INST_PARENT(index), unit), DT_INST_PROP(index, cs))

#define XSPI_FLASH_BASE_ADDR_ASSERT(index)                                                         \
	BUILD_ASSERT(XSPI_FLASH_BASE_ADDR(index) == XSPI_BSP_FLASH_BASE_ADDR(index),               \
		     "XSPI flash base address does not match BSP feature address")

#define UNSUPPORTED_COMMAND 0
#define UNUSED_COMMAND      0x00

#define SPI_NOR_DUMMY_NONE 0U
#define SPI_OCMD_DUMMY_RD  20U

#define RESET_HIGH_BEFORE_CS_LOW_US 15
#define RESET_LOW_PULSE_WIDTH_US    20

/* Flash erase value */
#define ERASE_VALUE 0xFF

#define MAX_TIME_ERASE (50000U)
#define MAX_TIME_WRITE (50U)
#define MAX_TIME_WREN  (5U)

#define SFDP_PARAM_SECTOR_MAP_AVAILABLE    BIT(0)
#define SFDP_PARAM_4BYTE_ADDR_AVAILABLE    BIT(1)
#define SFDP_PARAM_PROFILE_1V0_AVAILABLE   BIT(2)
#define SFDP_PARAM_SCCR_MAP_AVAILABLE      BIT(3)
#define SFDP_PARAM_SEQ_OCTAL_DDR_AVAILABLE BIT(4)

/*
 * Maximum SFDP table sizes (in DWORDs) kept in static buffers - no dynamic
 * allocation. Sized for the largest known device tables (e.g. MX25LW51245G
 * SCCR is 28 DW). Ported from flash_renesas_ra_ospi_b.c.
 */
#define SFDP_BFP_MAX_DWORDS   20
#define SFDP_SMPT_MAX_DWORDS  32
#define SFDP_PR1_MAX_DWORDS   8
#define SFDP_SCCR_MAX_DWORDS  32
#define SFDP_SEQ8D_MAX_DWORDS 16
#define SFDP_MAX_PARAM_HDRS   10

/* Maximum number of flash regions (SMPT configuration map regions). */
#define XSPI_MAX_FLASH_REGIONS 8

/*
 * QPI (4-4-4) mode enter commands, selected from the SFDP BFP DW15 "4-4-4
 * mode enable sequences" bits: 0x35 (ISSI and others), 0x38 (Macronix,
 * Winbond and others).
 */
#define CMD_ENQPI_35      0x35U
#define CMD_ENQPI_38      0x38U
/* EN4B via configuration register (JESD216 DW16 BIT(4)) */
#define CMD_RD_NONVOL_CFG 0xB5U
#define CMD_WR_NONVOL_CFG 0xB1U
/* QER S2B7: read SR2 with 0x3F, write SR2 with 0x3E */
#define CMD_RDSR2_ALT     0x3FU
#define CMD_WRSR2_ALT     0x3EU
/* SERPV - Set Extended Read Parameters (volatile) */
#define IS25_CMD_SERPV    0x83U
/* RDABR - Read AutoBoot Register (detect D vs non-D suffix) */
#define IS25_CMD_RDABR    0x14U
/* SRPV  - Set Read Parameters (reset dummy cycles to default) */
#define IS25_CMD_SRPV     0xC0U
/* Default SRPV value: normal drive strength, dummy reset */
#define IS25_SRPV_DEFAULT 0x00U
/*
 * IS25 Extended Read Register ODS[7:5] drive-strength codes (volatile SERPV,
 * no WREN required). Softer edges suppress SSO/ground bounce on quad reads.
 */
#define IS25_ODS_STR      0x40U /* ODS 010 -> 25% */

/* Quad sub-protocol selector (DTS "quad-protocol", XSPI_QUAD_MODE only) */
enum xspi_quad_protocol {
	XSPI_QUAD_PROTOCOL_IO = 0,  /* 1S-4S-4S */
	XSPI_QUAD_PROTOCOL_QPI = 1, /* 4S-4S-4S */
};

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

struct flash_region_info {
	uint32_t offset;
	uint32_t size;
	uint8_t erase_mask;
};

struct flash_regions_layout {
	struct flash_region_info regions[XSPI_MAX_FLASH_REGIONS];
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout layout[XSPI_MAX_FLASH_REGIONS];
#endif
	size_t region_count;
};

struct flash_sfdp_region {
	/* SFDP tables cached during probe (read in 1S-1S-1S before mode switch) */
	struct jesd216_sfdp_header header;                     /* SFDP header */
	struct jesd216_param_header phdr[SFDP_MAX_PARAM_HDRS]; /* all param headers */
	uint32_t bfp_buf[SFDP_BFP_MAX_DWORDS];                 /* Basic Flash Parameters */
	uint32_t pr1_buf[SFDP_PR1_MAX_DWORDS];                 /* xSPI Profile 1.0 */
	uint32_t addr_4b_buf[2];                               /* 4-Byte Address Instructions */
	uint32_t sccr_buf[SFDP_SCCR_MAX_DWORDS];               /* SCCR map */
	uint32_t octal_ddr_buf[SFDP_SEQ8D_MAX_DWORDS];         /* Octal DDR enable sequences */
	uint32_t smpt_buf[SFDP_SMPT_MAX_DWORDS];               /* Sector Map Parameter table */
	uint32_t param_available;                              /* bitmask of available tables */
	uint8_t pr1_len;                                       /* pr1_buf length in DWORDs */
	uint8_t addr_4b_len;                                   /* addr_4b_buf length in DWORDs */
	uint8_t sccr_len;                                      /* sccr_buf length in DWORDs */
	uint8_t octal_ddr_len;                                 /* octal_ddr_buf length in DWORDs */
	uint8_t smp_len;                                       /* smpt_buf length in DWORDs */
};

/* Since some NOR flash devices have incorrect SFDP values, this special require will override them
 */
struct flash_special_require_info {
	uint16_t enter_4byte_cmd;
	uint8_t def_8d_dummy_cycles;
	uint8_t qer_override;  /* JESD216_DW15_QER_VAL_*, valid only if qer_override_set */
	bool qer_override_set; /* "quad-enable-requirements" present in DT */
	bool rdsr_addr_4bytes;
	bool reset_dummy_cycles;    /* write SRPV before QPI/QSPI */
	bool fix_bfpt_addr_bytes;   /* BFPT lies about 3B-only addressing */
	bool reduce_drive_strength; /* lower ODS to curb quad-read SSO */
};

struct flash_renesas_rx_xspi_controller_data {
	xspi_controller_instance_ctrl_t xspi_controller_ctrl;
	spi_flash_cfg_t xspi_flash_cfg;
	xspi_controller_timing_setting_t xspi_controller_timing_settings;
	xspi_controller_command_set_t xspi_command_set_table;
	xspi_controller_extended_cfg_t xspi_controller_config_extend;
	spi_flash_erase_command_t erase_command_list[JESD216_NUM_ERASE_TYPES];
	xspi_controller_table_t xspi_controller_command_set;
	xspi_controller_table_t xspi_controller_erase_commands;
	struct flash_sfdp_region sfdp;
	struct flash_regions_layout regions_layout;
	uint8_t jedec_id[JESD216_READ_ID_LEN];
	uint8_t address_mode;
	struct k_sem sem;
	uint32_t calib_sector_size;
	spi_flash_direct_transfer_t transfer_write_enable;
	spi_flash_direct_transfer_t transfer_read_status;
	/* WIP polling: RDSR (05h, bit0 1=busy) or FSR (70h, bit7 0=busy) */
	spi_flash_direct_transfer_t transfer_read_wip;
	bool fsr_poll;
	uint16_t clear_fsr_command;
};

struct flash_renesas_rx_xspi_controller_config {
	const struct device *clock_dev;
	struct clock_control_rx_subsys_cfg clock_config;
	const struct pinctrl_dev_config *pcfg;
	volatile R_XSPI0_Type *ospi_pregs;
	size_t flash_size;
	uint32_t max_frequency;
	int data_mode; /* SPI or QSPI or OSPI */
	int data_rate; /* DTR or STR */
	uint32_t flash_base_address;
	const struct flash_parameters xspi_controller_param;
	const struct flash_special_require_info special_require_info;
	const uint8_t quad_protocol; /* enum xspi_quad_protocol */
	const uint8_t jedec_id[JESD216_READ_ID_LEN];
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout pages_layout;
#endif
};

/* Build a 2-byte 8D-8D-8D opcode from a 1-byte opcode per BFP DW18 */
static uint16_t flash_xspi_extend_command(uint8_t cmd, bool invert)
{
	return (uint16_t)((uint16_t)cmd << 8 | (invert ? (uint8_t)~cmd : cmd));
}

static int flash_xspi_wait_operation(struct flash_renesas_rx_xspi_controller_data *data,
				     uint32_t timeout)
{
	fsp_err_t err;
	uint8_t reg;
	int ret;

	while (timeout) {
		err = R_XSPI_CONTROLLER_DirectTransfer(&data->xspi_controller_ctrl,
						       &data->transfer_read_wip,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		reg = (uint8_t)data->transfer_read_wip.data_u.data;

		if (data->fsr_poll) {
			if (reg & SPI_NOR_FLSR_READY) {
				ret = 0;
				if (reg & SPI_NOR_FLSR_ERASE_FAIL) {
					LOG_ERR("Erase failure (FSR=0x%02x)", reg);
					ret = -EIO;
				}
				if (reg & SPI_NOR_FLSR_PROGRAM_FAIL) {
					LOG_ERR("Program failure (FSR=0x%02x)", reg);
					ret = -EIO;
				}
				if (reg & SPI_NOR_FLSR_PROT_ERROR) {
					LOG_ERR("Protection violation (FSR=0x%02x)", reg);
					ret = -EIO;
				}
				if (ret != 0) {
					/* Clear latched FSR error bits */
					spi_flash_direct_transfer_t clrfsr = {
						.command = data->clear_fsr_command,
						.command_length =
							data->transfer_read_wip.command_length,
					};

					(void)R_XSPI_CONTROLLER_DirectTransfer(
						&data->xspi_controller_ctrl, &clrfsr,
						SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
				}
				return ret;
			}
		} else if (!(reg & BIT(0))) {
			return 0;
		}

		k_sleep(K_USEC(200));
		timeout--;
	}

	return -EIO;
}

static int flash_xspi_write_enable(struct flash_renesas_rx_xspi_controller_data *data)
{
	fsp_err_t err;
	uint32_t timeout = MAX_TIME_WREN;

	err = R_XSPI_CONTROLLER_DirectTransfer(&data->xspi_controller_ctrl,
					       &data->transfer_write_enable,
					       SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_ERR("WREN DT failed: err=%d proto=0x%03x", err,
			data->xspi_controller_ctrl.spi_protocol);
		return -EIO;
	}

	while (timeout) {
		err = R_XSPI_CONTROLLER_DirectTransfer(&data->xspi_controller_ctrl,
						       &data->transfer_read_status,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			LOG_ERR("RDSR DT failed: err=%d proto=0x%03x", err,
				data->xspi_controller_ctrl.spi_protocol);
			return -EIO;
		}

		if (((data->transfer_read_status.data_u.data >>
		      data->xspi_controller_ctrl.p_cfg->write_enable_bit) &
		     0x1U) == 1U) {
			break;
		}

		k_sleep(K_USEC(200));
		timeout--;
	}

	if (timeout == 0) {
		LOG_ERR("WREN timeout: last SR=0x%02x proto=0x%03x",
			(uint8_t)data->transfer_read_status.data_u.data,
			data->xspi_controller_ctrl.spi_protocol);
		return -EIO;
	}

	return 0;
}

static void flash_xspi_is25_set_ods(struct flash_renesas_rx_xspi_controller_data *data, uint8_t ods)
{
	spi_flash_direct_transfer_t serpv = {
		.command = IS25_CMD_SERPV,
		.command_length = 1,
		.address_length = 0,
		.data_u.data = ods,
		.data_length = 1,
		.dummy_cycles = 0,
	};

	R_XSPI_CONTROLLER_DirectTransfer(&data->xspi_controller_ctrl, &serpv,
					 SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
}

static bool flash_xspi_is_valid_address(const struct device *dev, off_t offset, size_t len)
{
	struct flash_renesas_rx_xspi_controller_data *data = dev->data;
	struct flash_regions_layout *layout = &data->regions_layout;
	off_t flash_end;

	if (offset < 0 || len == 0 || layout->region_count == 0) {
		return false;
	}

	flash_end = (off_t)(layout->regions[layout->region_count - 1].offset +
			    layout->regions[layout->region_count - 1].size);

	if (data->xspi_controller_ctrl.spi_protocol == SPI_FLASH_PROTOCOL_8D_8D_8D) {
		/* Accessible end: last region's end minus the calibration sector */
		flash_end -= (off_t)data->calib_sector_size;
	}

	if (!(offset < flash_end) || !((off_t)len <= flash_end - offset)) {
		return false;
	}

	return true;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_renesas_rx_xspi_controller_page_layout(const struct device *dev,
							 const struct flash_pages_layout **layout,
							 size_t *layout_size)
{
	const struct flash_renesas_rx_xspi_controller_data *data = dev->data;

	*layout = data->regions_layout.layout;
	*layout_size = data->regions_layout.region_count;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

/* Find the region that contains the given offset, or -EINVAL if none. */
static int flash_xspi_find_region(struct flash_renesas_rx_xspi_controller_data *data, off_t offset,
				  struct flash_region_info **region, size_t *remain_len)
{
	struct flash_regions_layout *layout = &data->regions_layout;

	for (size_t i = 0; i < layout->region_count; i++) {
		if (offset >= (off_t)layout->regions[i].offset &&
		    offset < (off_t)(layout->regions[i].offset + layout->regions[i].size)) {
			*region = &layout->regions[i];
			*remain_len = layout->regions[i].size -
				      (size_t)(offset - layout->regions[i].offset);
			return 0;
		}
	}
	return -EINVAL;
}

/*
 * Return the largest erase type from the region's erase_mask whose size fits
 * within the remaining bytes, or NULL if none fits.
 */
static const spi_flash_erase_command_t *
flash_xspi_find_erase_type(struct flash_renesas_rx_xspi_controller_data *data,
			   struct flash_region_info *region, size_t region_remain)
{
	for (int8_t i = JESD216_NUM_ERASE_TYPES - 1; i >= 0; i--) {
		if ((region->erase_mask & BIT(i)) && (data->erase_command_list[i].size != 0) &&
		    (data->erase_command_list[i].size <= region_remain) &&
		    (region_remain % data->erase_command_list[i].size == 0)) {
			return &data->erase_command_list[i];
		}
	}

	return NULL;
}

/*
 * Wait for the XSPI bus to go fully idle before switching PRTMD mid-call
 * (see flash_xspi_switch_to_1s_once()) -- changing protocol while a
 * transaction is still in flight corrupts WEL, silently dropping the next
 * command.
 */
static void flash_xspi_wait_bus_idle(struct flash_renesas_rx_xspi_controller_data *data)
{
	uint32_t timeout = 100000;
	uint32_t idle_mask = R_XSPI0_COMSTT_MEMACCCH0_Msk | R_XSPI0_COMSTT_WRBUFNECH0_Msk;

	while (data->xspi_controller_ctrl.p_reg->COMSTT & idle_mask) {
		if (--timeout == 0) {
			LOG_ERR("XSPI bus not idle before protocol switch");
			return;
		}

		k_busy_wait(1);
	}
}

/*
 * 1S-4S-4S (IO mode): some flash devices have no program/erase opcode that
 * accepts a 4-line address phase, so write/erase must run entirely under
 * EXTENDED_SPI. The protocol switch happens ONCE for the whole erase()/
 * write() call (not per chunk): a per-chunk SpiProtocolSet round-trip leaves
 * a timing window that corrupts WEL, silently failing alternating chunks.
 */
static int flash_xspi_switch_to_1s_once(struct flash_renesas_rx_xspi_controller_data *data)
{
	fsp_err_t err;

	data->xspi_command_set_table.protocol = SPI_FLASH_PROTOCOL_EXTENDED_SPI;
	err = R_XSPI_CONTROLLER_SpiProtocolSet(&data->xspi_controller_ctrl,
					       SPI_FLASH_PROTOCOL_EXTENDED_SPI);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Switch to EXTENDED_SPI failed");
		return -EIO;
	}

	return 0;
}

static int flash_xspi_restore_1s_4s_4s(struct flash_renesas_rx_xspi_controller_data *data,
				       int prior_ret)
{
	fsp_err_t err;

	flash_xspi_wait_bus_idle(data);

	data->xspi_command_set_table.protocol = SPI_FLASH_PROTOCOL_1S_4S_4S;
	err = R_XSPI_CONTROLLER_SpiProtocolSet(&data->xspi_controller_ctrl,
					       SPI_FLASH_PROTOCOL_1S_4S_4S);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Restore 1S_4S_4S failed");
		return prior_ret != 0 ? prior_ret : -EIO;
	}

	return prior_ret;
}

static int flash_renesas_rx_xspi_controller_erase(const struct device *dev, off_t offset,
						  size_t len)
{
	struct flash_renesas_rx_xspi_controller_data *xspi_data = dev->data;
	const struct flash_renesas_rx_xspi_controller_config *config = dev->config;
	struct flash_region_info *region;
	fsp_err_t err;
	struct flash_pages_info page_info_start, page_info_end;
	int ret = 0;

	if (!len) {
		return 0;
	}

	if (!flash_xspi_is_valid_address(dev, offset, len)) {
		LOG_ERR("Address or size exceeds expected values: "
			"Address 0x%lx, size %zu",
			(long)offset, len);
		return -EINVAL;
	}

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

	ret = k_sem_take(&xspi_data->sem, K_FOREVER);
	if (ret != 0) {
		LOG_ERR("Failed to acquire semaphore for erase");
		return ret;
	}

	bool switched_1s =
		(xspi_data->xspi_controller_ctrl.spi_protocol == SPI_FLASH_PROTOCOL_1S_4S_4S);

	if (switched_1s) {
		ret = flash_xspi_switch_to_1s_once(xspi_data);
		if (ret != 0) {
			k_sem_give(&xspi_data->sem);
			return ret;
		}
	}

	while (len > 0) {
		const spi_flash_erase_command_t *erase_type;
		uint32_t erase_count;
		size_t region_remain;
		size_t erase_len;

		ret = flash_xspi_find_region(xspi_data, offset, &region, &region_remain);
		if (ret != 0) {
			LOG_ERR("No region found for offset 0x%lx", (long)offset);
			break;
		}

		erase_len = MIN(len, region_remain);

		erase_type = flash_xspi_find_erase_type(xspi_data, region, erase_len);
		if (erase_type == NULL) {
			LOG_ERR("region at offset 0x%lx has no erase type for remaining "
				"length %zu",
				(long)offset, erase_len);
			ret = -EINVAL;
			break;
		}

		erase_count = erase_len / erase_type->size;

		for (uint32_t i = 0; i < erase_count; i++) {
			err = R_XSPI_CONTROLLER_Erase(
				&xspi_data->xspi_controller_ctrl,
				(uint8_t *)(config->flash_base_address + offset), erase_type->size);
			if (err != FSP_SUCCESS) {
				LOG_ERR("Erase at 0x%lx size %u failed", (long)offset,
					erase_type->size);
				ret = -EIO;
			} else {
				ret = flash_xspi_wait_operation(xspi_data, MAX_TIME_ERASE);
			}

			if (ret != 0) {
				break;
			}

			offset += erase_type->size;
			len -= erase_type->size;
		}

		if (ret != 0) {
			break;
		}
	}

	if (switched_1s) {
		ret = flash_xspi_restore_1s_4s_4s(xspi_data, ret);
	}

	arch_dcache_flush_and_invd_all();

	k_sem_give(&xspi_data->sem);

	return ret;
}

static int flash_renesas_rx_xspi_controller_write(const struct device *dev, off_t offset,
						  const void *data, size_t len)
{
	const struct flash_renesas_rx_xspi_controller_config *config = dev->config;
	struct flash_renesas_rx_xspi_controller_data *xspi_data = dev->data;
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

	if (!flash_xspi_is_valid_address(dev, offset, len)) {
		LOG_ERR("Address or size exceeds expected values: "
			"Address 0x%lx, size %zu",
			(long)offset, len);
		return -EINVAL;
	}

	if ((offset % config->xspi_controller_param.write_block_size) ||
	    (len % config->xspi_controller_param.write_block_size)) {
		return -EINVAL;
	}

	ret = k_sem_take(&xspi_data->sem, K_FOREVER);
	if (ret != 0) {
		LOG_ERR("Failed to acquire semaphore for write");
		return ret;
	}

	bool switched_1s =
		(xspi_data->xspi_controller_ctrl.spi_protocol == SPI_FLASH_PROTOCOL_1S_4S_4S);

	if (switched_1s) {
		ret = flash_xspi_switch_to_1s_once(xspi_data);
		if (ret != 0) {
			k_sem_give(&xspi_data->sem);
			return ret;
		}
	}

	while (len > 0) {
		size = len > xspi_data->xspi_flash_cfg.page_size_bytes
			       ? xspi_data->xspi_flash_cfg.page_size_bytes
			       : len;

		err = R_XSPI_CONTROLLER_Write(&xspi_data->xspi_controller_ctrl, p_src,
					      (uint8_t *)(config->flash_base_address + offset),
					      size);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Write at 0x%lx len %zu failed", (long)offset, size);
			ret = -EIO;
		} else {
			ret = flash_xspi_wait_operation(xspi_data, MAX_TIME_WRITE);
		}

		if (ret != 0) {
			break;
		}

		len -= size;
		offset += size;
		p_src = p_src + size;
	}

	if (switched_1s) {
		ret = flash_xspi_restore_1s_4s_4s(xspi_data, ret);
	}

	arch_dcache_flush_and_invd_all();

	k_sem_give(&xspi_data->sem);

	return ret;
}

static int flash_renesas_rx_xspi_controller_read(const struct device *dev, off_t offset, void *data,
						 size_t len)
{
	const struct flash_renesas_rx_xspi_controller_config *config = dev->config;
	struct flash_renesas_rx_xspi_controller_data *xspi_data = dev->data;
	int ret;

	if (!len) {
		return 0;
	}

	if (!flash_xspi_is_valid_address(dev, offset, len)) {
		LOG_ERR("Address or size exceeds expected values: "
			"Address 0x%lx, size %zu",
			(long)offset, len);
		return -EINVAL;
	}

	ret = k_sem_take(&xspi_data->sem, K_FOREVER);
	if (ret != 0) {
		LOG_ERR("Failed to acquire semaphore for read");
		return ret;
	}

	{
		/* XIP (memory-mapped) read path for all protocols. */
		uint32_t *src = (uint32_t *)(config->flash_base_address + (uint32_t)offset);
		uint32_t *dst = (uint32_t *)data;

		/* Bus requires the source address and length to be even-aligned */

		if (((uintptr_t)src & 1) && len > 0) {
			uint16_t val = *(volatile uint16_t *)((uint8_t *)src - 1);

			*(uint8_t *)dst = (uint8_t)((val >> 8) & 0xff);
			src = (uint32_t *)((uint8_t *)src + 1);
			dst = (uint32_t *)((uint8_t *)dst + 1);
			len--;
		}

		if (len >= 4) {
			size_t bulk = len & ~3U;

			memcpy(dst, src, bulk);
			src = (uint32_t *)((uint8_t *)src + bulk);
			dst = (uint32_t *)((uint8_t *)dst + bulk);
			len -= bulk;
		}

		if (len > 0 && len < 4) {
			uint32_t tmp = *(volatile uint32_t *)src;
			uint8_t *d = (uint8_t *)dst;

			for (size_t i = 0; i < len; i++) {
				d[i] = (uint8_t)((tmp >> (i * 8)) & 0xff);
			}
		}
	}

	k_sem_give(&xspi_data->sem);

	return ret;
}

static const struct flash_parameters *
flash_renesas_rx_xspi_controller_get_parameters(const struct device *dev)
{
	const struct flash_renesas_rx_xspi_controller_config *config = dev->config;

	return &config->xspi_controller_param;
}

static int flash_renesas_rx_xspi_controller_get_size(const struct device *dev, uint64_t *size)
{
	const struct flash_renesas_rx_xspi_controller_config *config = dev->config;

	*size = (uint64_t)config->flash_size;

	return 0;
}

static int flash_xspi_setup_calibrate_data(struct flash_renesas_rx_xspi_controller_data *data,
					   uint32_t flash_start_addr, size_t flash_size,
					   bool pattern_word_swapped)
{
	uint32_t autocalibration_data[4];
	spi_flash_erase_command_t *erase_type = NULL;
	uint32_t calib_sector_offset, calib_sector_address;
	fsp_err_t err;
	int ret;

	ARG_UNUSED(flash_start_addr);

	if (pattern_word_swapped) {
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

	for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
		if (data->regions_layout.regions[data->regions_layout.region_count - 1].erase_mask &
		    BIT(i)) {
			erase_type = &data->erase_command_list[i];
			break;
		}
	}

	if (erase_type == NULL) {
		return -EIO;
	}

	calib_sector_offset = (uint32_t)flash_size - erase_type->size;
	calib_sector_address =
		(uint32_t)
			data->xspi_controller_config_extend.p_autocalibration_preamble_pattern_addr;
	ARG_UNUSED(calib_sector_offset);

	if (memcmp((uint8_t *)calib_sector_address, &autocalibration_data,
		   sizeof(autocalibration_data)) != 0) {

		err = R_XSPI_CONTROLLER_Erase(&data->xspi_controller_ctrl,
					      (uint8_t *)calib_sector_address, erase_type->size);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Erase the flash sector Failed");
			return -EIO;
		}

		ret = flash_xspi_wait_operation(data, MAX_TIME_ERASE);
		if (ret != 0) {
			LOG_ERR("Wait for erase operation completes Failed");
			return ret;
		}

		err = R_XSPI_CONTROLLER_Write(
			&data->xspi_controller_ctrl, (uint8_t *)&autocalibration_data,
			(uint8_t *)calib_sector_address, sizeof(autocalibration_data));
		if (err != FSP_SUCCESS) {
			LOG_ERR("Write auto-calibration data Failed %d", err);
			return -EIO;
		}

		ret = flash_xspi_wait_operation(data, MAX_TIME_WRITE);
		if (ret != 0) {
			LOG_ERR("Wait for write operation completes Failed");
			return ret;
		}

		if (memcmp((uint8_t *)calib_sector_address, &autocalibration_data,
			   sizeof(autocalibration_data)) != 0) {
			LOG_ERR("Confirm for autocalibration data fail");
			return -EIO;
		}
	}

	data->calib_sector_size = erase_type->size;

	return 0;
}

/* this helper is only used in 1S-1S-1S mode*/
static int flash_xspi_sfdp_read_helper(xspi_controller_instance_ctrl_t *p_ctrl, void *data,
				       off_t offset, size_t len)
{
	fsp_err_t err;
	uint8_t *p_dst = data;
	uint8_t buf_size;
	spi_flash_direct_transfer_t transfer_sfdp = {.command = JESD216_CMD_READ_SFDP,
						     .command_length = 1,
						     .address_length = 3,
						     .dummy_cycles = SPI_NOR_DUMMY_RD};

	while (len > 0) {
		transfer_sfdp.address = offset;

		if (len > 8) {
			transfer_sfdp.data_length = 8;
			buf_size = 8;
		} else {
			transfer_sfdp.data_length = len;
			buf_size = len;
		}

		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer_sfdp,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}

		memcpy(p_dst, &transfer_sfdp.data_u.data_u64, buf_size);
		len -= buf_size;
		offset += buf_size;
		p_dst += buf_size;
	}

	return 0;
}

/* this helper is only used in 1S-1S-1S mode*/
static int flash_xspi_jedec_id_read_helper(xspi_controller_instance_ctrl_t *p_ctrl, uint8_t *data)
{
	fsp_err_t err;
	spi_flash_direct_transfer_t transfer_jedec_id = {.command = SPI_NOR_CMD_RDID,
							 .command_length = 1,
							 .address = 0,
							 .address_length = 0,
							 .data_length = 3,
							 .dummy_cycles = 0};

	err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer_jedec_id,
					       SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}
	memcpy(data, &transfer_jedec_id.data_u.data, transfer_jedec_id.data_length);

	return 0;
}

#if defined(CONFIG_FLASH_JESD216_API)
static int flash_renesas_rx_xspi_controller_sfdp_read(const struct device *dev, off_t offset,
						      void *data, size_t len)
{
	struct flash_renesas_rx_xspi_controller_data *xspi_data = dev->data;
	struct flash_sfdp_region *sfdp = &xspi_data->sfdp;
	off_t hdr_end = sizeof(struct jesd216_sfdp_header);
	off_t phdr_end = hdr_end + (sfdp->header.nph + 1) * sizeof(struct jesd216_param_header);
	uint8_t *dst = data;
	off_t cur = offset;
	size_t remaining = len;

	if (len == 0) {
		return 0;
	}

	if (data == NULL) {
		LOG_ERR("The data buffer is NULL");
		return -EINVAL;
	}

	while (remaining > 0) {
		uint8_t *src;
		size_t available;

		if (cur < hdr_end) {
			src = (uint8_t *)&sfdp->header + cur;
			available = hdr_end - cur;
		} else if (cur < phdr_end) {
			src = (uint8_t *)sfdp->phdr + (cur - hdr_end);
			available = phdr_end - cur;
		} else {
			uint8_t *param_buf = NULL;
			off_t param_start = 0;
			size_t param_size = 0;
			uint8_t nph =
				MIN((uint8_t)(sfdp->header.nph + 1U), (uint8_t)SFDP_MAX_PARAM_HDRS);

			for (uint8_t i = 0; i < nph; i++) {
				off_t start = jesd216_param_addr(&sfdp->phdr[i]);
				size_t sz = sfdp->phdr[i].len_dw * sizeof(uint32_t);

				if (cur < start || cur >= (off_t)(start + sz)) {
					continue;
				}

				switch (jesd216_param_id(&sfdp->phdr[i])) {
				case JESD216_SFDP_PARAM_ID_BFP:
					param_buf = (uint8_t *)sfdp->bfp_buf;
					break;
				case JESD216_SFDP_PARAM_ID_SECTOR_MAP:
					if (sfdp->param_available &
					    SFDP_PARAM_SECTOR_MAP_AVAILABLE) {
						param_buf = (uint8_t *)sfdp->smpt_buf;
					}
					break;
				case JESD216_SFDP_PARAM_ID_SEQ_OCTAL_DDR:
					if (sfdp->param_available &
					    SFDP_PARAM_SEQ_OCTAL_DDR_AVAILABLE) {
						param_buf = (uint8_t *)sfdp->octal_ddr_buf;
					}
					break;
				case JESD216_SFDP_PARAM_ID_SCCR_MAP:
					if (sfdp->param_available & SFDP_PARAM_SCCR_MAP_AVAILABLE) {
						param_buf = (uint8_t *)sfdp->sccr_buf;
					}
					break;
				case JESD216_SFDP_PARAM_ID_XSPI_PROFILE_1V0:
					if (sfdp->param_available &
					    SFDP_PARAM_PROFILE_1V0_AVAILABLE) {
						param_buf = (uint8_t *)sfdp->pr1_buf;
					}
					break;
				case JESD216_SFDP_PARAM_ID_4B_ADDR_INSTR:
					if (sfdp->param_available &
					    SFDP_PARAM_4BYTE_ADDR_AVAILABLE) {
						param_buf = (uint8_t *)sfdp->addr_4b_buf;
					}
					break;
				default:
					break;
				}

				param_start = start;
				param_size = sz;
				break;
			}

			if (param_buf == NULL) {
				*dst = 0;
				dst++;
				cur++;
				remaining--;
				continue;
			}

			src = param_buf + (cur - param_start);
			available = param_size - (size_t)(cur - param_start);
		}

		size_t copy_len = MIN(remaining, available);

		memcpy(dst, src, copy_len);
		dst += copy_len;
		cur += copy_len;
		remaining -= copy_len;
	}

	return 0;
}

static int flash_renesas_rx_xspi_controller_read_jedec_id(const struct device *dev,
							  uint8_t *jedec_id)
{
	const struct flash_renesas_rx_xspi_controller_data *data = dev->data;

	if (jedec_id == NULL) {
		return -EINVAL;
	}

	memcpy(jedec_id, data->jedec_id, JESD216_READ_ID_LEN);

	return 0;
}
#endif /* CONFIG_FLASH_JESD216_API */

static void flash_xspi_parse_erase_4b(spi_flash_erase_command_t *erase_command,
				      uint32_t *add_4b_param)
{
	for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
		if (add_4b_param[0] & BIT(9 + i)) {
			erase_command[i].command =
				JESD216_SFDP_4B_ADDR_DW2_ERASE_TYPE_INSTR(add_4b_param[1], i);
		} else {
			erase_command[i].command = 0;
			erase_command[i].size = 0;
		}
	}
}

static int flash_xspi_4byte_enable(struct flash_renesas_rx_xspi_controller_data *data,
				   uint32_t en4b, uint16_t enter_4byte_cmd)
{
	xspi_controller_instance_ctrl_t *p_ctrl = &data->xspi_controller_ctrl;
	fsp_err_t err;
	int ret;
	spi_flash_direct_transfer_t transfer_4byte_enable = {
		.address = 0,
		.data_u.data = 0,
		.dummy_cycles = 0,
		.command_length = 1,
		.address_length = 0,
		.data_length = 0,
	};

	if (enter_4byte_cmd != 0) {
		transfer_4byte_enable.command = enter_4byte_cmd;
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer_4byte_enable,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
	} else if ((en4b & BIT(0))) {
		transfer_4byte_enable.command = SPI_NOR_CMD_4BA;
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer_4byte_enable,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
	} else if (en4b & BIT(1)) {
		ret = flash_xspi_write_enable(data);
		if (ret != 0) {
			return ret;
		}
		transfer_4byte_enable.command = SPI_NOR_CMD_4BA;
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer_4byte_enable,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}

	} else if (en4b & BIT(4)) {
		transfer_4byte_enable.command = CMD_RD_NONVOL_CFG;
		transfer_4byte_enable.data_length = 2;
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer_4byte_enable,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}

		ret = flash_xspi_write_enable(data);
		if (ret != 0) {
			return ret;
		}

		transfer_4byte_enable.command = CMD_WR_NONVOL_CFG;
		transfer_4byte_enable.data_u.data |= BIT(0);
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer_4byte_enable,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
	} else if ((en4b & BIT(6)) || (en4b & BIT(5))) {
		/* Flash is always in 4 byte mode. We just need to configure LUT */
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int flash_xspi_quad_enable(struct flash_renesas_rx_xspi_controller_data *data, uint8_t qer)
{
	xspi_controller_instance_ctrl_t *p_ctrl = &data->xspi_controller_ctrl;
	spi_flash_direct_transfer_t transfer;
	fsp_err_t err;
	int ret;
	uint8_t sr1_value, sr2_value;

	switch (qer) {
	case JESD216_DW15_QER_VAL_NONE:
		return 0;
	case JESD216_DW15_QER_VAL_S2B1v1:
	case JESD216_DW15_QER_VAL_S2B1v4:
	case JESD216_DW15_QER_VAL_S2B1v5:
		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_RDSR,
			.command_length = 1,
			.address_length = 0,
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		sr1_value = transfer.data_u.data & 0xFF;

		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_RDSR2,
			.command_length = 1,
			.address_length = 0,
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		sr2_value = transfer.data_u.data & 0xFF;

		if (sr2_value & BIT(1)) {
			return 0;
		}

		ret = flash_xspi_write_enable(data);
		if (ret != 0) {
			return ret;
		}

		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_WRSR,
			.command_length = 1,
			.address_length = 0,
			.data_u.data = (uint32_t)sr1_value | ((uint32_t)(sr2_value | BIT(1)) << 8),
			.data_length = 2,
			.dummy_cycles = 0,
		};
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		break;

	case JESD216_DW15_QER_VAL_S1B6:
		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_RDSR,
			.command_length = 1,
			.address_length = 0,
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		sr1_value = transfer.data_u.data & 0xFF;

		if (sr1_value & BIT(6)) {
			return 0;
		}

		ret = flash_xspi_write_enable(data);
		if (ret != 0) {
			return ret;
		}

		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_WRSR,
			.command_length = 1,
			.address_length = 0,
			.data_u.data = sr1_value | BIT(6),
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		break;

	case JESD216_DW15_QER_VAL_S2B7:
		transfer = (spi_flash_direct_transfer_t){
			.command = CMD_RDSR2_ALT,
			.command_length = 1,
			.address_length = 0,
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		sr2_value = transfer.data_u.data & 0xFF;

		if (sr2_value & BIT(7)) {
			return 0;
		}

		ret = flash_xspi_write_enable(data);
		if (ret != 0) {
			return ret;
		}

		transfer = (spi_flash_direct_transfer_t){
			.command = CMD_WRSR2_ALT,
			.command_length = 1,
			.address_length = 0,
			.data_u.data = sr2_value | BIT(7),
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		break;

	case JESD216_DW15_QER_VAL_S2B1v6:
		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_RDSR2,
			.command_length = 1,
			.address_length = 0,
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		sr2_value = transfer.data_u.data & 0xFF;

		if (sr2_value & BIT(1)) {
			return 0;
		}

		ret = flash_xspi_write_enable(data);
		if (ret != 0) {
			return ret;
		}

		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_WRSR2,
			.command_length = 1,
			.address_length = 0,
			.data_u.data = sr2_value | BIT(1),
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		break;

	default:
		return -ENOTSUP;
	}

	return flash_xspi_wait_operation(data, MAX_TIME_WRITE);
}

static int flash_xspi_seq_8d_enable(xspi_controller_instance_ctrl_t *p_ctrl, uint32_t *octal_8d_buf,
				    uint8_t octal_8d_len)
{
	spi_flash_direct_transfer_t transfer_8d_enb[4];
	fsp_err_t err;

	if (octal_8d_len > 8) {
		return -ENOTSUP;
	}

	for (uint8_t i = 0; i < octal_8d_len; i += 2) {
		uint8_t cmd_len = JESD216_SFDP_SODP_LENGTH(octal_8d_buf[i]);
		uint8_t transfer_index = i / 2;

		transfer_8d_enb[transfer_index].command = JESD216_SFDP_SODP_BYTE1(octal_8d_buf[i]);
		transfer_8d_enb[transfer_index].command_length = 1;
		transfer_8d_enb[transfer_index].dummy_cycles = 0;

		if (cmd_len < 5) {
			uint32_t payload = 0;

			transfer_8d_enb[transfer_index].address_length = 0;
			for (uint8_t j = 2; j <= cmd_len; j++) {

				uint8_t byte_value = 0;

				if (j <= 3) {
					byte_value = (octal_8d_buf[i] >> (24 - (j * 8))) & 0xFF;
				} else {
					byte_value = JESD216_SFDP_SODP_BYTE4(octal_8d_buf[i + 1]);
				}

				payload = payload << 8 | (uint32_t)byte_value;
			}

			transfer_8d_enb[transfer_index].data_u.data = payload;
			transfer_8d_enb[transfer_index].data_length = cmd_len - 1;

		} else if (cmd_len == 5) {
			transfer_8d_enb[transfer_index].address_length = 3;
			transfer_8d_enb[transfer_index].address =
				JESD216_SFDP_SODP_BYTE2(octal_8d_buf[i]) << 16 |
				JESD216_SFDP_SODP_BYTE3(octal_8d_buf[i]) << 8 |
				JESD216_SFDP_SODP_BYTE4(octal_8d_buf[i + 1]);
			transfer_8d_enb[transfer_index].data_u.data =
				JESD216_SFDP_SODP_BYTE5(octal_8d_buf[i + 1]);
			transfer_8d_enb[transfer_index].data_length = 1;

		} else if (cmd_len > 5) {
			transfer_8d_enb[transfer_index].address_length = 4;
			transfer_8d_enb[transfer_index].address =
				JESD216_SFDP_SODP_BYTE2(octal_8d_buf[i]) << 24 |
				JESD216_SFDP_SODP_BYTE3(octal_8d_buf[i]) << 16 |
				JESD216_SFDP_SODP_BYTE4(octal_8d_buf[i + 1]) << 8 |
				JESD216_SFDP_SODP_BYTE5(octal_8d_buf[i + 1]);

			for (uint8_t j = 6; j <= cmd_len; j++) {
				transfer_8d_enb[transfer_index].data_u.data =
					(transfer_8d_enb[transfer_index].data_u.data << 8) |
					((octal_8d_buf[i + 1] >> (24 - (j - 4) * 8)) & 0xFF);
			}
			transfer_8d_enb[transfer_index].data_length = cmd_len - 5;
		}
	}

	for (uint8_t i = 0; i < (octal_8d_len / 2); i++) {
		err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer_8d_enb[i],
						       SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Failed to enable octal ddr mode");
			return -EIO;
		}
		k_sleep(K_USEC(50));
	}

	return 0;
}

static int flash_xspi_sccr_8d_enable(struct flash_renesas_rx_xspi_controller_data *data,
				     uint32_t *sccr_buf, uint8_t sccr_len, uint8_t address_length)
{
	xspi_controller_instance_ctrl_t *p_ctrl = &data->xspi_controller_ctrl;
	spi_flash_direct_transfer_t transfer_8d_enb;
	fsp_err_t err;
	int ret;
	uint8_t addr_enb_octa, addr_enb_dtr_octa;
	uint8_t mask_enb_octa, mask_enb_dtr_octa;
	bool need_enb_octa = false;

	if ((sccr_len < 22) || !(sccr_buf[15] & BIT(31)) || !(sccr_buf[21] & BIT(31))) {
		return -ENOTSUP;
	}

	addr_enb_octa = (sccr_buf[15] >> 16) & 0xff;
	addr_enb_dtr_octa = (sccr_buf[21] >> 16) & 0xff;

	if (addr_enb_octa != addr_enb_dtr_octa) {
		return -ENOTSUP;
	}

	transfer_8d_enb.command = (sccr_buf[21] >> 8) & 0xFF;
	transfer_8d_enb.command_length = 1;
	transfer_8d_enb.data_length = 1;
	transfer_8d_enb.dummy_cycles = 0;

	if (sccr_buf[21] & BIT(28)) {
		uint32_t addr = sccr_buf[0];

		if (sccr_buf[21] & BIT(27)) {
			addr += (sccr_buf[21] >> 16 & 0xff) << 8;
		} else {
			addr += sccr_buf[21] >> 16 & 0xff;
		}

		transfer_8d_enb.address = addr;
		transfer_8d_enb.address_length = address_length;
	} else {
		transfer_8d_enb.address_length = 0;
	}

	err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer_8d_enb,
					       SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	mask_enb_octa = BIT((sccr_buf[16] >> 24) & 0x7);
	mask_enb_dtr_octa = BIT((sccr_buf[21] >> 24) & 0x7);

	if (sccr_buf[21] & BIT(30)) {
		transfer_8d_enb.data_u.data &= ~mask_enb_dtr_octa;
	} else {
		transfer_8d_enb.data_u.data |= mask_enb_dtr_octa;
	}

	if (mask_enb_octa != mask_enb_dtr_octa) {
		need_enb_octa = true;
	}

	if ((sccr_buf[19] & BIT(31)) && ((sccr_buf[19] >> 16 & 0xff) == addr_enb_dtr_octa)) {
		uint8_t mask_enb_str_octa = BIT((sccr_buf[19] >> 24) & 0x7);

		if (mask_enb_octa != mask_enb_str_octa) {
			need_enb_octa = true;
		} else {
			need_enb_octa = false;
		}
	}

	if (need_enb_octa) {
		if (sccr_buf[16] & BIT(30)) {
			transfer_8d_enb.data_u.data &= ~mask_enb_octa;
		} else {
			transfer_8d_enb.data_u.data |= mask_enb_octa;
		}
	}

	transfer_8d_enb.command = sccr_buf[21] & 0xFF;

	ret = flash_xspi_write_enable(data);
	if (ret != 0) {
		return ret;
	}

	err = R_XSPI_CONTROLLER_DirectTransfer(p_ctrl, &transfer_8d_enb,
					       SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static int flash_xspi_update_page_layout(struct flash_renesas_rx_xspi_controller_data *data)
{
	struct flash_pages_layout *layout = data->regions_layout.layout;
	uint32_t i, j;

	for (i = 0; i < data->regions_layout.region_count; i++) {
		uint32_t min_size = UINT32_MAX;

		for (j = 0; j < JESD216_NUM_ERASE_TYPES; j++) {
			if ((data->regions_layout.regions[i].erase_mask & BIT(j)) &&
			    data->erase_command_list[j].size > 0 &&
			    data->erase_command_list[j].size < min_size) {
				min_size = data->erase_command_list[j].size;
			}
		}

		if (min_size == UINT32_MAX) {
			LOG_ERR("No valid erase size found for region %u", i);
			return -EINVAL;
		}

		layout[i].pages_size = min_size;
		layout[i].pages_count = data->regions_layout.regions[i].size / min_size;
	}

	return 0;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static int flash_xspi_read_map_format(struct flash_renesas_rx_xspi_controller_data *data,
				      const uint32_t *config_map)
{
	struct flash_region_info *regions = data->regions_layout.regions;
	uint32_t region_count;
	uint32_t offset;
	uint8_t regions_erase_type = 0;
	uint8_t i, j;

	region_count = JESD216_SFDP_SMP_MAP_REGION_COUNT(config_map[0]);
	if (region_count == 0) {
		LOG_ERR("Invalid region count in sector map");
		return -EIO;
	}
	if (region_count > XSPI_MAX_FLASH_REGIONS) {
		LOG_WRN("SMPT region count %u > max %d, clamping", region_count,
			XSPI_MAX_FLASH_REGIONS);
		region_count = XSPI_MAX_FLASH_REGIONS;
	}

	offset = 0;

	for (i = 0; i < region_count; i++) {
		j = i + 1;
		regions[i].offset = offset;
		regions[i].size = JESD216_SFDP_SMP_MAP_REGION_SIZE(config_map[j]);
		regions[i].erase_mask = JESD216_SFDP_SMP_MAP_REGION_ERASE_TYPE(config_map[j]);

		regions_erase_type |= regions[i].erase_mask;

		offset = regions[i].offset + regions[i].size;
	}

	if (!regions_erase_type) {
		return -EINVAL;
	}

	data->regions_layout.region_count = region_count;

	for (i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
		if (!(regions_erase_type & BIT(i)) && (data->erase_command_list[i].command != 0)) {
			data->erase_command_list[i].command = 0;
			data->erase_command_list[i].size = 0;
		}
	}

	return 0;
}

static uint32_t *flash_xspi_get_map_in_use(struct flash_renesas_rx_xspi_controller_data *data,
					   uint32_t *smpt_buf, uint8_t smp_len)
{
	uint32_t *ret;
	uint8_t i;
	uint8_t read_data_mask, map_id;
	fsp_err_t fsp_err;
	spi_flash_direct_transfer_t transfer_read_cfg = {
		.command_length = 1,
		.data_length = 1,
		.address_length = data->address_mode,
	};

	map_id = 0;
	for (i = 0; i < smp_len; i += 2) {
		if (smpt_buf[i] & JESD216_SFDP_SMP_DESC_TYPE_MAP) {
			break;
		}

		read_data_mask = JESD216_SFDP_SMP_CMD_READ_DATA(smpt_buf[i]);
		transfer_read_cfg.address = smpt_buf[i + 1];
		transfer_read_cfg.command = JESD216_SFDP_SMP_CMD_OPCODE(smpt_buf[i]);
		transfer_read_cfg.dummy_cycles =
			JESD216_SFDP_SMP_CMD_READ_DUMMY(smpt_buf[i]) ==
					JESD216_SFDP_SMP_CMD_READ_DUMMY_IS_VARIABLE
				? SPI_NOR_DUMMY_RD
				: JESD216_SFDP_SMP_CMD_READ_DUMMY(smpt_buf[i]);

		fsp_err = R_XSPI_CONTROLLER_DirectTransfer(&data->xspi_controller_ctrl,
							   &transfer_read_cfg,
							   SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (fsp_err != FSP_SUCCESS) {
			ret = NULL;
			return ret;
		}

		map_id = map_id << 1 | !!(transfer_read_cfg.data_u.data & read_data_mask);
	}

	ret = NULL;
	while (i < smp_len) {
		if (JESD216_SFDP_SMP_MAP_ID(smpt_buf[i]) == map_id) {
			ret = smpt_buf + i;
			break;
		}

		if (smpt_buf[i] & JESD216_SFDP_SMP_DESC_END) {
			break;
		}

		i += JESD216_SFDP_SMP_MAP_REGION_COUNT(smpt_buf[i]) + 1;
	}

	return ret;
}

static int flash_xspi_smpt_handle(struct flash_renesas_rx_xspi_controller_data *data,
				  uint32_t *smpt_buf, uint8_t smp_len)
{
	uint32_t *sector_map;
	int ret;

	sector_map = flash_xspi_get_map_in_use(data, smpt_buf, smp_len);
	if (sector_map == NULL) {
		LOG_ERR("Failed to get sector map");
		return -EIO;
	}

	ret = flash_xspi_read_map_format(data, sector_map);
	if (ret) {
		LOG_ERR("Failed to read map format");
		return -EIO;
	}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	ret = flash_xspi_update_page_layout(data);
	if (ret) {
		LOG_ERR("Failed to update page layout");
		return -EIO;
	}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

	return 0;
}

static int flash_xspi_update_flash_config(const struct device *dev)
{
	const struct flash_renesas_rx_xspi_controller_config *config = dev->config;
	struct flash_renesas_rx_xspi_controller_data *data = dev->data;
	struct flash_sfdp_region *sfdp_region = &data->sfdp;
	struct jesd216_bfp *bfp = (struct jesd216_bfp *)sfdp_region->bfp_buf;
	struct jesd216_instr instr;
	struct jesd216_bfp_dw16 dw16;
	struct jesd216_bfp_dw14 dw14;
	struct jesd216_bfp_dw11 dw11;
	enum jesd216_mode_type target_mode;
	bool word_swapped = false;
	int ret;
	fsp_err_t err;
	spi_flash_erase_command_t local_erase_list[JESD216_NUM_ERASE_TYPES] = {0};
	uint8_t erase_exp[JESD216_NUM_ERASE_TYPES] = {
		bfp->dw8 & 0xFF,
		(bfp->dw8 >> 16) & 0xFF,
		bfp->dw9 & 0xFF,
		(bfp->dw9 >> 16) & 0xFF,
	};
	uint8_t erase_cmd[JESD216_NUM_ERASE_TYPES] = {
		(bfp->dw8 >> 8) & 0xFF,
		(bfp->dw8 >> 24) & 0xFF,
		(bfp->dw9 >> 8) & 0xFF,
		(bfp->dw9 >> 24) & 0xFF,
	};
	xspi_controller_table_t local_erase_table = {
		.p_table = local_erase_list,
		.length = JESD216_NUM_ERASE_TYPES,
	};
	xspi_controller_command_set_t local_cmd_set = {
		.protocol = SPI_FLASH_PROTOCOL_EXTENDED_SPI,
		.frame_format = XSPI_CONTROLLER_FRAME_FORMAT_STANDARD,
		.latency_mode = XSPI_CONTROLLER_LATENCY_MODE_FIXED,
		.command_bytes = XSPI_CONTROLLER_COMMAND_BYTES_1,
		.address_bytes = SPI_FLASH_ADDRESS_BYTES_3,
		.address_msb_mask =
			(local_cmd_set.address_bytes == SPI_FLASH_ADDRESS_BYTES_4) ? 0xF0 : 0x00,
		.read_command = SPI_NOR_CMD_READ_FAST,
		.read_dummy_cycles = SPI_NOR_DUMMY_RD,
		.program_command = SPI_NOR_CMD_PP,
		.program_dummy_cycles = 0,
		.write_enable_command = SPI_NOR_CMD_WREN,
		.status_command = SPI_NOR_CMD_RDSR,
		.p_erase_commands = &local_erase_table,
	};

	if (config->special_require_info.fix_bfpt_addr_bytes) {
		sfdp_region->bfp_buf[0] &= ~JESD216_SFDP_BFP_DW1_ADDRBYTES_MASK;
		sfdp_region->bfp_buf[0] |= ((uint32_t)JESD216_SFDP_BFP_DW1_ADDRBYTES_VAL_4B
					    << JESD216_SFDP_BFP_DW1_ADDRBYTES_SHFT);
		LOG_DBG("BFPT address bytes overridden to 4");

		spi_flash_direct_transfer_t en4b = {
			.command = SPI_NOR_CMD_4BA,
			.command_length = 1,
			.address_length = 0,
			.data_length = 0,
			.dummy_cycles = 0,
		};
		err = R_XSPI_CONTROLLER_DirectTransfer(&data->xspi_controller_ctrl, &en4b,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			LOG_ERR("EN4B (0xB7) failed: %d", err);
			return -EIO;
		};
	}

	if (config->special_require_info.reset_dummy_cycles) {
		spi_flash_direct_transfer_t rdabr = {
			.command = IS25_CMD_RDABR,
			.command_length = 1,
			.address_length = 0,
			.data_length = 1,
			.dummy_cycles = 0,
		};
		spi_flash_direct_transfer_t srpv = {
			.command = IS25_CMD_SRPV,
			.command_length = 1,
			.address_length = 0,
			.data_u.data = IS25_SRPV_DEFAULT,
			.data_length = 1,
			.dummy_cycles = 0,
		};

		R_XSPI_CONTROLLER_DirectTransfer(&data->xspi_controller_ctrl, &rdabr,
						 SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		R_XSPI_CONTROLLER_DirectTransfer(&data->xspi_controller_ctrl, &srpv,
						 SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	}

	if (config->special_require_info.reduce_drive_strength) {
		flash_xspi_is25_set_ods(data, IS25_ODS_STR);
	}

	switch (config->data_mode) {
	case XSPI_OCTO_MODE:
		if (jesd216_bfp_read_support(&sfdp_region->phdr[0], bfp, JESD216_MODE_8D8D8D,
					     &instr) == 0 &&
		    (sfdp_region->param_available & SFDP_PARAM_PROFILE_1V0_AVAILABLE) &&
		    ((sfdp_region->param_available & SFDP_PARAM_SEQ_OCTAL_DDR_AVAILABLE) ||
		     (sfdp_region->param_available & SFDP_PARAM_SCCR_MAP_AVAILABLE))) {
			target_mode = JESD216_MODE_8D8D8D;
			break;
		}
		LOG_ERR("Octal DDR (8D-8D-8D) mode is not supported by this NOR flash");
		return -ENOTSUP;
	case XSPI_QUAD_MODE:
		if (config->quad_protocol == XSPI_QUAD_PROTOCOL_QPI) {
			if (jesd216_bfp_read_support(&sfdp_region->phdr[0], bfp, JESD216_MODE_444,
						     &instr) < 0) {
				LOG_ERR("QPI (4S-4S-4S) is not supported by this NOR flash");
				return -ENOTSUP;
			}
			target_mode = JESD216_MODE_444;
		} else {
			if (jesd216_bfp_read_support(&sfdp_region->phdr[0], bfp, JESD216_MODE_144,
						     &instr) < 0) {
				LOG_ERR("Quad (1S-4S-4S) is not supported by this NOR flash");
				return -ENOTSUP;
			}
			target_mode = JESD216_MODE_144;
		}
		break;
	default:
		target_mode = JESD216_MODE_111;
		break;
	}

	for (int i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
		if (erase_exp[i] == 0 || erase_exp[i] >= 32) {
			continue;
		}
		local_erase_list[i].command = erase_cmd[i];
		local_erase_list[i].size = BIT(erase_exp[i]);
	}

	ret = jesd216_bfp_decode_dw11(&sfdp_region->phdr[0], bfp, &dw11);
	if (ret == 0 && dw11.page_size < 64) {
		data->xspi_flash_cfg.page_size_bytes = dw11.page_size;
	}

	ret = jesd216_bfp_decode_dw14(&sfdp_region->phdr[0], bfp, &dw14);
	if (ret != 0) {
		/* DW14 was introduced in JESD216A; assume legacy polling only. */
		dw14.poll_options = JESD216_DW14_POLL_LEGACY_SR;
	}

	/*
	 * The HAL runs its own busy and WEL checks using cmd_set.status_command and
	 * hardcodes "bit == 1 means asserted", which only matches the legacy status
	 * register (WIP = bit 0, WEL = bit 1). Keep RDSR here regardless of DW14.
	 */
	local_cmd_set.status_command = SPI_NOR_CMD_RDSR;

	/*
	 * Prefer the Flag Status Register for this driver's own polling when it is
	 * advertised: it also latches program/erase/protection failures.
	 */
	if (dw14.poll_options & JESD216_DW14_POLL_FSR) {
		data->fsr_poll = true;
		data->transfer_read_wip.command = SPI_NOR_CMD_RDFLSR;
		data->clear_fsr_command = SPI_NOR_CMD_CLRFLSR;
	} else {
		data->fsr_poll = false;
		data->transfer_read_wip.command = SPI_NOR_CMD_RDSR;
		data->clear_fsr_command = 0;
	}

	if (config->flash_size > MB(16)) {
		uint8_t init_addr_width =
			jesd216_bfp_addrbytes(bfp) == JESD216_SFDP_BFP_DW1_ADDRBYTES_VAL_4B ? 32
											    : 24;
		if (init_addr_width == 24) {
			ret = jesd216_bfp_decode_dw16(&sfdp_region->phdr[0], bfp, &dw16);
			if (ret != 0) {
				LOG_ERR("Flash size > 16MB need enter 4 byte address mode but BFPT "
					"dw16 decode failed");
				return -EIO;
			}

			ret = flash_xspi_4byte_enable(data, dw16.enter_4ba,
						      config->special_require_info.enter_4byte_cmd);
			if (ret != 0) {
				LOG_ERR("4 byte address support advertised but failed to "
					"decode DW16");
				return -EIO;
			}
		}

		data->address_mode = 4;
		local_cmd_set.address_bytes = SPI_FLASH_ADDRESS_BYTES_4;

		if (sfdp_region->param_available & SFDP_PARAM_4BYTE_ADDR_AVAILABLE) {
			if (sfdp_region->addr_4b_buf[0] &
			    JESD216_SFDP_4B_ADDR_DW1_1S_1S_1S_FAST_READ_0C_SUP) {
				local_cmd_set.read_command = SPI_NOR_CMD_READ_FAST_4B;
			}

			if (sfdp_region->addr_4b_buf[0] &
			    JESD216_SFDP_4B_ADDR_DW1_1S_1S_1S_PP_12_SUP) {
				local_cmd_set.program_command = SPI_NOR_CMD_PP_4B;
			}

			flash_xspi_parse_erase_4b(local_erase_list, sfdp_region->addr_4b_buf);
		} else {
			LOG_DBG("No 4BA table; using standard 4-byte opcodes");
			local_cmd_set.read_command = SPI_NOR_CMD_READ_FAST_4B;
			local_cmd_set.program_command = SPI_NOR_CMD_PP_4B;

			for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
				if (local_erase_list[i].command == SPI_NOR_CMD_SE) {
					local_erase_list[i].command = SPI_NOR_CMD_SE_4B;
				} else if (local_erase_list[i].command == SPI_NOR_CMD_BE) {
					local_erase_list[i].command = SPI_NOR_CMD_BE_4B;
				}
			}
		}
	} else {
		data->address_mode = 3;
		local_cmd_set.address_bytes = SPI_FLASH_ADDRESS_BYTES_3;
	}

	for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
		data->erase_command_list[i] = local_erase_list[i];
	}

	if (sfdp_region->param_available & SFDP_PARAM_SECTOR_MAP_AVAILABLE) {
		ret = flash_xspi_smpt_handle(data, sfdp_region->smpt_buf, sfdp_region->smp_len);
		if (ret < 0) {
			LOG_ERR("Failed to handle sector map parameter table");
		}
	} else {
		struct flash_region_info *region = &data->regions_layout.regions[0];

		region->offset = 0;
		region->size = config->flash_size;
		region->erase_mask = 0;

		for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
			if (data->erase_command_list[i].command != 0 &&
			    data->erase_command_list[i].size != 0) {
				region->erase_mask |= BIT(i);
			}
		}

		data->regions_layout.region_count = 1;

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
		ret = flash_xspi_update_page_layout(data);
		if (ret) {
			LOG_ERR("Failed to update page layout");
			return -EIO;
		}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
	}

	/* Just mode 8D need to use DS signal and need auto calibration */
	if (target_mode == JESD216_MODE_8D8D8D) {
		data->xspi_controller_command_set.p_table = &local_cmd_set;

		err = R_XSPI_CONTROLLER_SpiProtocolSet(&data->xspi_controller_ctrl,
						       SPI_FLASH_PROTOCOL_EXTENDED_SPI);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Init SPI mode Failed");
			return -EIO;
		}

		if (bfp->dw10[8] & JESD216_SFDP_BFP_DW18_BYTE_ORDER_SWAPPED) {
			word_swapped = true;
			LOG_DBG("Byte order is swapped for this device, please aware that the data "
				"on NOR Flash is not correct when read or programed by other mode");
		}

		ret = flash_xspi_setup_calibrate_data(data, config->flash_base_address,
						      config->flash_size, word_swapped);
		if (ret != 0) {
			LOG_ERR("Setup calibrate data Failed");
			return -EIO;
		}
	}

	data->xspi_command_set_table = local_cmd_set;
	data->xspi_command_set_table.p_erase_commands = &data->xspi_controller_erase_commands;

	if (target_mode == JESD216_MODE_8D8D8D) {
		if (sfdp_region->param_available & SFDP_PARAM_SCCR_MAP_AVAILABLE) {
			uint8_t dummy;

			ret = flash_xspi_sccr_8d_enable(data, sfdp_region->sccr_buf,
							sfdp_region->sccr_len, data->address_mode);
			if (ret != 0) {
				LOG_ERR("Failed to enable DTR Octal mode using SCCR map");
				return ret;
			}

			dummy = FIELD_GET(JESD216_SFDP_PROFILE1_DW4_DUMMY_200MHZ,
					  sfdp_region->pr1_buf[3]);
			if (!dummy) {
				dummy = FIELD_GET(JESD216_SFDP_PROFILE1_DW5_DUMMY_166MHZ,
						  sfdp_region->pr1_buf[4]);
			}
			if (!dummy) {
				dummy = FIELD_GET(JESD216_SFDP_PROFILE1_DW5_DUMMY_133MHZ,
						  sfdp_region->pr1_buf[4]);
			}
			if (!dummy) {
				dummy = FIELD_GET(JESD216_SFDP_PROFILE1_DW5_DUMMY_100MHZ,
						  sfdp_region->pr1_buf[4]);
			}
			if (!dummy) {
				LOG_ERR("Can't find dummy cycles from Profile 1.0 table");
			}

			dummy = (uint8_t)ROUND_UP(dummy, 2);
			data->xspi_command_set_table.read_dummy_cycles = dummy;
		} else if (sfdp_region->param_available & SFDP_PARAM_SEQ_OCTAL_DDR_AVAILABLE) {
			data->xspi_controller_ctrl.p_reg->LIOCTL_b.RSTCS0 = 0;
			k_sleep(K_USEC(500));
			data->xspi_controller_ctrl.p_reg->LIOCTL_b.RSTCS0 = 1;
			k_sleep(K_NSEC(50));

			ret = flash_xspi_seq_8d_enable(&data->xspi_controller_ctrl,
						       sfdp_region->octal_ddr_buf,
						       sfdp_region->octal_ddr_len);
			if (ret != 0) {
				LOG_ERR("Failed to enable DTR Octal mode using Sequence table");
				return -EIO;
			}

			data->xspi_command_set_table.read_dummy_cycles = SPI_OCMD_DUMMY_RD;
		} else {
			LOG_ERR("Target mode is 8D-8D-8D, but not SPI NOR Flash not support "
				"Sequence table or SCCR map to enable it");
			return -EIO;
		}

		data->xspi_command_set_table.protocol = SPI_FLASH_PROTOCOL_8D_8D_8D;
		data->address_mode = 4;
		data->xspi_command_set_table.address_bytes = SPI_FLASH_ADDRESS_BYTES_4;
		data->xspi_command_set_table.command_bytes = XSPI_CONTROLLER_COMMAND_BYTES_2;
		data->xspi_command_set_table.frame_format =
			XSPI_CONTROLLER_FRAME_FORMAT_XSPI_PROFILE_1;

		data->xspi_command_set_table.read_command = sfdp_region->pr1_buf[0] >> 8 & 0xFF;

		if (config->special_require_info.def_8d_dummy_cycles != 0) {
			data->xspi_command_set_table.read_dummy_cycles =
				config->special_require_info.def_8d_dummy_cycles;
		}

		data->xspi_command_set_table.program_command = SPI_NOR_CMD_PP_4B;
		data->xspi_command_set_table.program_dummy_cycles = 0;
		data->xspi_command_set_table.write_enable_command = SPI_NOR_CMD_WREN;

		if (sfdp_region->pr1_buf[0] & JESD216_SFDP_PROFILE1_DW1_RDSR_DUMMY) {
			data->xspi_command_set_table.status_dummy_cycles = 8;
			data->transfer_read_status.dummy_cycles = 8;
		} else {
			data->xspi_command_set_table.status_dummy_cycles = 4;
			data->transfer_read_status.dummy_cycles = 4;
		}

		if ((config->special_require_info.rdsr_addr_4bytes) ||
		    (sfdp_region->pr1_buf[0] & JESD216_SFDP_PROFILE1_DW1_RDSR_ADDR_BYTES)) {
			data->xspi_command_set_table.status_needs_address = true;
			data->xspi_command_set_table.status_address_bytes =
				SPI_FLASH_ADDRESS_BYTES_4;
			data->xspi_command_set_table.status_address = 0;

			data->transfer_read_status.address_length = 4;
			data->transfer_read_status.address = 0;
		} else {
			data->xspi_command_set_table.status_needs_address = false;
		}

		switch (bfp->dw10[8] & JESD216_SFDP_BFP_DW18_CMD_EXT_MASK) {
		case JESD216_SFDP_BFP_DW18_CMD_EXT_REP:
			data->xspi_command_set_table.read_command =
				data->xspi_command_set_table.read_command << 8 |
				data->xspi_command_set_table.read_command;
			data->xspi_command_set_table.program_command =
				data->xspi_command_set_table.program_command << 8 |
				data->xspi_command_set_table.program_command;
			data->xspi_command_set_table.write_enable_command =
				data->xspi_command_set_table.write_enable_command << 8 |
				data->xspi_command_set_table.write_enable_command;
			data->xspi_command_set_table.status_command =
				data->xspi_command_set_table.status_command << 8 |
				data->xspi_command_set_table.status_command;

			for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
				if (data->erase_command_list[i].command != 0) {
					data->erase_command_list[i].command =
						data->erase_command_list[i].command << 8 |
						data->erase_command_list[i].command;
				}
			}
			break;
		case JESD216_SFDP_BFP_DW18_CMD_EXT_INV:
			data->xspi_command_set_table.read_command =
				data->xspi_command_set_table.read_command << 8 |
				(~data->xspi_command_set_table.read_command & 0xFF);
			data->xspi_command_set_table.program_command =
				data->xspi_command_set_table.program_command << 8 |
				(~data->xspi_command_set_table.program_command & 0xFF);
			data->xspi_command_set_table.write_enable_command =
				data->xspi_command_set_table.write_enable_command << 8 |
				(~data->xspi_command_set_table.write_enable_command & 0xFF);
			data->xspi_command_set_table.status_command =
				data->xspi_command_set_table.status_command << 8 |
				(~data->xspi_command_set_table.status_command & 0xFF);

			for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
				if (data->erase_command_list[i].command != 0) {
					data->erase_command_list[i].command =
						data->erase_command_list[i].command << 8 |
						(~data->erase_command_list[i].command & 0xFF);
				}
			}
			break;
		case JESD216_SFDP_BFP_DW18_CMD_EXT_16B:
			LOG_ERR("16-bit opcodes not supported");
			return -ENOTSUP;
		default:
			LOG_ERR("Unknown opcode extension type");
			return -EIO;
		}

		bool cmd_ext_inv = (bfp->dw10[8] & JESD216_SFDP_BFP_DW18_CMD_EXT_MASK) ==
				   JESD216_SFDP_BFP_DW18_CMD_EXT_INV;

		data->transfer_write_enable.command =
			data->xspi_command_set_table.write_enable_command;
		data->transfer_write_enable.command_length = 2;

		/* WEL polling always uses the legacy status register. */
		data->transfer_read_status.command = data->xspi_command_set_table.status_command;
		data->transfer_read_status.command_length = 2;

		/*
		 * WIP polling must keep whichever opcode DW14 selected. Extending
		 * status_command here instead would silently downgrade FSR to RDSR
		 * while flash_xspi_wait_operation() still decodes the reply as an FSR.
		 */
		data->transfer_read_wip.command = flash_xspi_extend_command(
			(uint8_t)data->transfer_read_wip.command, cmd_ext_inv);
		data->transfer_read_wip.command_length = 2;
		data->transfer_read_wip.dummy_cycles = data->transfer_read_status.dummy_cycles;
		data->transfer_read_wip.address_length = data->transfer_read_status.address_length;
		data->transfer_read_wip.address = 0;

		if (data->fsr_poll) {
			data->clear_fsr_command =
				flash_xspi_extend_command(SPI_NOR_CMD_CLRFLSR, cmd_ext_inv);
		}

		data->xspi_controller_command_set.p_table = &data->xspi_command_set_table;

		err = R_XSPI_CONTROLLER_SpiProtocolSet(&data->xspi_controller_ctrl,
						       SPI_FLASH_PROTOCOL_8D_8D_8D);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Switch to OPI mode Failed");
			return -EIO;
		}
	} else if (target_mode == JESD216_MODE_144) {
		struct jesd216_bfp_dw15 dw15;

		ret = jesd216_bfp_decode_dw15(&sfdp_region->phdr[0], bfp, &dw15);
		if (ret != 0) {
			LOG_ERR("dw15 decode failed");
			return -EIO;
		}

		ret = flash_xspi_quad_enable(data,
					     config->special_require_info.qer_override_set
						     ? config->special_require_info.qer_override
						     : dw15.qer);
		if (ret != 0) {
			LOG_ERR("Quad SPI enable failed");
			return -EIO;
		}

		data->xspi_command_set_table.protocol = SPI_FLASH_PROTOCOL_1S_4S_4S;
		data->xspi_command_set_table.frame_format = XSPI_CONTROLLER_FRAME_FORMAT_STANDARD;
		data->xspi_command_set_table.command_bytes = XSPI_CONTROLLER_COMMAND_BYTES_1;

		if (data->address_mode == 4) {
			if (sfdp_region->param_available & SFDP_PARAM_4BYTE_ADDR_AVAILABLE) {
				data->xspi_command_set_table.read_command =
					(sfdp_region->addr_4b_buf[0] &
					 JESD216_SFDP_4B_ADDR_DW1_1S_4S_4_FAST_READ_EC_SUP)
						? SPI_NOR_CMD_4READ_4B
						: SPI_NOR_CMD_READ_FAST_4B;
				data->xspi_command_set_table.program_command =
					(sfdp_region->addr_4b_buf[0] &
					 JESD216_SFDP_4B_ADDR_DW1_1S_4S_4S_PP_3E_SUP)
						? SPI_NOR_CMD_PP_1_4_4_4B
						: SPI_NOR_CMD_PP_4B;
			} else {
				data->xspi_command_set_table.read_command = SPI_NOR_CMD_4READ_4B;
				data->xspi_command_set_table.program_command = SPI_NOR_CMD_PP_4B;
			}
		} else {
			data->xspi_command_set_table.read_command = instr.instr;
			data->xspi_command_set_table.program_command = SPI_NOR_CMD_PP_1_4_4;
		}

		data->xspi_command_set_table.read_dummy_cycles =
			instr.wait_states + instr.mode_clocks;
		data->xspi_command_set_table.program_dummy_cycles = 0;
		data->xspi_command_set_table.status_dummy_cycles = 0;

		data->xspi_controller_command_set.p_table = &data->xspi_command_set_table;

		err = R_XSPI_CONTROLLER_SpiProtocolSet(&data->xspi_controller_ctrl,
						       SPI_FLASH_PROTOCOL_1S_4S_4S);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Switch to Quad SPI mode Failed");
			return -EIO;
		}
	} else if (target_mode == JESD216_MODE_444) {
		struct jesd216_bfp_dw15 dw15;
		uint16_t enqpi_cmd;

		ret = jesd216_bfp_decode_dw15(&sfdp_region->phdr[0], bfp, &dw15);
		if (ret != 0) {
			LOG_ERR("dw15 decode failed");
			return -EIO;
		}

		ret = flash_xspi_quad_enable(data,
					     config->special_require_info.qer_override_set
						     ? config->special_require_info.qer_override
						     : dw15.qer);
		if (ret != 0) {
			LOG_ERR("Quad SPI enable failed");
			return -EIO;
		}

		if (config->special_require_info.fix_bfpt_addr_bytes) {
			enqpi_cmd = CMD_ENQPI_35;
		} else if (dw15.enable_444 & BIT(2)) {
			enqpi_cmd = CMD_ENQPI_35;
		} else if (dw15.enable_444 & (BIT(0) | BIT(1))) {
			enqpi_cmd = CMD_ENQPI_38;
		} else {
			LOG_ERR("Unsupported 4-4-4 enable sequence (DW15 bits 0x%02x)",
				dw15.enable_444);
			return -ENOTSUP;
		}

		spi_flash_direct_transfer_t enqpi = {
			.command = enqpi_cmd,
			.command_length = 1,
			.address_length = 0,
			.data_length = 0,
			.dummy_cycles = 0,
		};

		err = R_XSPI_CONTROLLER_DirectTransfer(&data->xspi_controller_ctrl, &enqpi,
						       SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Enter QPI (0x%02x) failed", enqpi_cmd);
			return -EIO;
		}

		data->xspi_command_set_table.protocol = SPI_FLASH_PROTOCOL_4S_4S_4S;
		data->xspi_command_set_table.frame_format = XSPI_CONTROLLER_FRAME_FORMAT_STANDARD;
		data->xspi_command_set_table.command_bytes = XSPI_CONTROLLER_COMMAND_BYTES_1;

		data->xspi_command_set_table.read_command =
			(data->address_mode == 4) ? SPI_NOR_CMD_4READ_4B : SPI_NOR_CMD_4READ;

		data->xspi_command_set_table.read_dummy_cycles =
			instr.wait_states + instr.mode_clocks;
		data->xspi_command_set_table.program_dummy_cycles = 0;

		data->xspi_command_set_table.program_command = SPI_NOR_CMD_PP;
		for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
			if (data->erase_command_list[i].command == SPI_NOR_CMD_SE_4B) {
				data->erase_command_list[i].command = SPI_NOR_CMD_SE;
			} else if (data->erase_command_list[i].command == SPI_NOR_CMD_BE_4B) {
				data->erase_command_list[i].command = SPI_NOR_CMD_BE;
			} else if (data->erase_command_list[i].command == SPI_NOR_CMD_BE_32K_4B) {
				data->erase_command_list[i].command = SPI_NOR_CMD_BE_32K;
			}
		}
		data->xspi_command_set_table.status_dummy_cycles = 0;
		data->transfer_read_status.dummy_cycles = 0;

		data->xspi_controller_command_set.p_table = &data->xspi_command_set_table;

		err = R_XSPI_CONTROLLER_SpiProtocolSet(&data->xspi_controller_ctrl,
						       SPI_FLASH_PROTOCOL_4S_4S_4S);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Switch to Quad SPI mode Failed");
			return -EIO;
		}
	} else if (target_mode == JESD216_MODE_111) {
		data->xspi_controller_command_set.p_table = &data->xspi_command_set_table;

		err = R_XSPI_CONTROLLER_SpiProtocolSet(&data->xspi_controller_ctrl,
						       SPI_FLASH_PROTOCOL_1S_1S_1S);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Init SPI mode Failed");
			return -EIO;
		}
	}

	return 0;
}

/*
 * Force the flash back to 1S-1S-1S. The device keeps its protocol (e.g. QPI)
 * across MCU resets when it is not power-cycled. Issue the reset first in
 * QPI (in case the device is still there from a previous run) and then in
 * SPI, so it always lands in 1S-1S-1S regardless of prior state.
 */
static int flash_xspi_software_reset(struct flash_renesas_rx_xspi_controller_data *data)
{
	spi_flash_direct_transfer_t rsten = {.command = SPI_NOR_CMD_RESET_EN, .command_length = 1};
	spi_flash_direct_transfer_t rstmem = {.command = SPI_NOR_CMD_RESET_MEM,
					      .command_length = 1};

	data->xspi_command_set_table.protocol = SPI_FLASH_PROTOCOL_4S_4S_4S;
	data->xspi_controller_command_set.p_table = &data->xspi_command_set_table;
	R_XSPI_CONTROLLER_SpiProtocolSet(&data->xspi_controller_ctrl, SPI_FLASH_PROTOCOL_4S_4S_4S);
	R_XSPI_CONTROLLER_DirectTransfer(&data->xspi_controller_ctrl, &rsten,
					 SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	R_XSPI_CONTROLLER_DirectTransfer(&data->xspi_controller_ctrl, &rstmem,
					 SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	k_sleep(K_USEC(100));

	data->xspi_command_set_table.protocol = SPI_FLASH_PROTOCOL_EXTENDED_SPI;
	R_XSPI_CONTROLLER_SpiProtocolSet(&data->xspi_controller_ctrl,
					 SPI_FLASH_PROTOCOL_EXTENDED_SPI);
	k_sleep(K_USEC(100));

	return 0;
}

static int flash_xspi_nor_probe(const struct device *dev)
{
	const struct flash_renesas_rx_xspi_controller_config *config = dev->config;
	struct flash_renesas_rx_xspi_controller_data *data = dev->data;
	struct flash_sfdp_region *sfdp_region = &data->sfdp;
	uint8_t device_id[JESD216_READ_ID_LEN];
	fsp_err_t err;
	int ret;

	err = R_XSPI_CONTROLLER_Open(&data->xspi_controller_ctrl, &data->xspi_flash_cfg);
	if (err != FSP_SUCCESS) {
		LOG_ERR("XSPI open Failed");
		ret = -EIO;
		goto out;
	}

	/* WRAPCFG = 0x00000F00 (DSSFTCS0/1 = 0xF): initial DS shift value used as
	 * the starting point for auto-calibration, per the Renesas reference
	 * sample. R_XSPI_CONTROLLER_Open() leaves DSSFTCSn at 0 since
	 * data_latch_delay_clocks is fixed to XSPI_CONTROLLER_DS_TIMING_DELAY_NONE
	 * for this driver. */
	if (data->xspi_controller_ctrl.channel == XSPI_CONTROLLER_DEVICE_NUMBER_0) {
		config->ospi_pregs->WRAPCFG_b.DSSFTCS0 = 0xF;
	} else {
		config->ospi_pregs->WRAPCFG_b.DSSFTCS1 = 0xF;
	}

	/* Reset SPI NOR flash device by driving OM_RESET pin */
	if (data->xspi_controller_ctrl.channel == XSPI_CONTROLLER_DEVICE_NUMBER_0) {
		config->ospi_pregs->LIOCTL_b.RSTCS0 = 0;
		k_usleep(RESET_LOW_PULSE_WIDTH_US);
		config->ospi_pregs->LIOCTL_b.RSTCS0 = 1;
	} else {
		config->ospi_pregs->LIOCTL_b.RSTCS1 = 0;
		k_usleep(RESET_LOW_PULSE_WIDTH_US);
		config->ospi_pregs->LIOCTL_b.RSTCS1 = 1;
	}
	k_usleep(RESET_HIGH_BEFORE_CS_LOW_US);

	ret = flash_xspi_sfdp_read_helper(&data->xspi_controller_ctrl, &sfdp_region->header, 0,
					  sizeof(sfdp_region->header));
	if (ret != 0) {
		LOG_ERR("Read SFDP header Failed");
		goto out;
	}

	if (jesd216_sfdp_magic(&sfdp_region->header) != JESD216_SFDP_MAGIC) {
		ret = flash_xspi_software_reset(data);
		if (ret != 0) {
			LOG_ERR("Software reset Failed");
			goto out;
		}

		ret = flash_xspi_sfdp_read_helper(&data->xspi_controller_ctrl, &sfdp_region->header,
						  0, sizeof(sfdp_region->header));
		if (ret != 0) {
			LOG_ERR("Read SFDP header Failed");
			goto out;
		}

		if (jesd216_sfdp_magic(&sfdp_region->header) != JESD216_SFDP_MAGIC) {
			LOG_ERR("Invalid header");
			ret = -EIO;
			goto out;
		}
	}

	ret = flash_xspi_jedec_id_read_helper(&data->xspi_controller_ctrl, device_id);
	if (ret != 0) {
		LOG_ERR("Read JEDEC ID Failed");
		goto out;
	}

	memcpy(data->jedec_id, device_id, JESD216_READ_ID_LEN);

	/* Alternate flash parts are configured from SFDP, not the DTS JEDEC ID. */
	if (memcmp(config->jedec_id, device_id, JESD216_READ_ID_LEN) != 0) {
		LOG_WRN("JEDEC ID differs from DTS: expected 0x%02x 0x%02x 0x%02x, "
			"got 0x%02x 0x%02x 0x%02x",
			config->jedec_id[0], config->jedec_id[1], config->jedec_id[2], device_id[0],
			device_id[1], device_id[2]);
	}

	uint8_t nph = MIN((uint8_t)(sfdp_region->header.nph + 1U), (uint8_t)SFDP_MAX_PARAM_HDRS);

	ret = flash_xspi_sfdp_read_helper(&data->xspi_controller_ctrl, sfdp_region->phdr,
					  sizeof(struct jesd216_sfdp_header),
					  nph * sizeof(struct jesd216_param_header));
	if (ret != 0) {
		LOG_ERR("Read parameter headers Failed");
		goto out;
	}

	ret = flash_xspi_sfdp_read_helper(
		&data->xspi_controller_ctrl, sfdp_region->bfp_buf,
		jesd216_param_addr(&sfdp_region->phdr[0]),
		MIN((size_t)sfdp_region->phdr[0].len_dw, SFDP_BFP_MAX_DWORDS) * sizeof(uint32_t));
	if (ret != 0) {
		LOG_ERR("Failed to read BFP table");
		goto out;
	}

	for (uint8_t i = 1; i < nph; i++) {
		uint32_t *dst = NULL;
		size_t max_dwords = 0;
		uint32_t flag = 0;
		uint8_t *len_field = NULL;
		size_t len_dw;

		switch (jesd216_param_id(&sfdp_region->phdr[i])) {
		case JESD216_SFDP_PARAM_ID_SECTOR_MAP:
			dst = sfdp_region->smpt_buf;
			max_dwords = SFDP_SMPT_MAX_DWORDS;
			flag = SFDP_PARAM_SECTOR_MAP_AVAILABLE;
			len_field = &sfdp_region->smp_len;
			break;
		case JESD216_SFDP_PARAM_ID_SEQ_OCTAL_DDR:
			dst = sfdp_region->octal_ddr_buf;
			max_dwords = SFDP_SEQ8D_MAX_DWORDS;
			flag = SFDP_PARAM_SEQ_OCTAL_DDR_AVAILABLE;
			len_field = &sfdp_region->octal_ddr_len;
			break;
		case JESD216_SFDP_PARAM_ID_SCCR_MAP:
			dst = sfdp_region->sccr_buf;
			max_dwords = SFDP_SCCR_MAX_DWORDS;
			flag = SFDP_PARAM_SCCR_MAP_AVAILABLE;
			len_field = &sfdp_region->sccr_len;
			break;
		case JESD216_SFDP_PARAM_ID_XSPI_PROFILE_1V0:
			dst = sfdp_region->pr1_buf;
			max_dwords = SFDP_PR1_MAX_DWORDS;
			flag = SFDP_PARAM_PROFILE_1V0_AVAILABLE;
			len_field = &sfdp_region->pr1_len;
			break;
		case JESD216_SFDP_PARAM_ID_4B_ADDR_INSTR:
			dst = sfdp_region->addr_4b_buf;
			max_dwords = ARRAY_SIZE(sfdp_region->addr_4b_buf);
			flag = SFDP_PARAM_4BYTE_ADDR_AVAILABLE;
			len_field = &sfdp_region->addr_4b_len;
			break;
		default:
			continue;
		}

		len_dw = MIN((size_t)sfdp_region->phdr[i].len_dw, max_dwords);
		ret = flash_xspi_sfdp_read_helper(&data->xspi_controller_ctrl, dst,
						  jesd216_param_addr(&sfdp_region->phdr[i]),
						  len_dw * sizeof(uint32_t));
		if (ret != 0) {
			LOG_ERR("Failed to read SFDP table 0x%04x",
				jesd216_param_id(&sfdp_region->phdr[i]));
			continue;
		}
		*len_field = (uint8_t)len_dw;
		sfdp_region->param_available |= flag;
	}

	ret = flash_xspi_update_flash_config(dev);

out:
	if ((ret != 0) && (data->xspi_controller_ctrl.open)) {
		(void)R_XSPI_CONTROLLER_Close(&data->xspi_controller_ctrl);
	}

	return ret;
}

#if defined(CONFIG_FLASH_EX_OP_ENABLED)
static int flash_renesas_rx_xspi_controller_ex_op(const struct device *dev, uint16_t code,
						  const uintptr_t in, void *out)
{
	struct flash_renesas_rx_xspi_controller_data *xspi_controller_data = dev->data;
	const struct flash_renesas_rx_xspi_controller_config *config = dev->config;
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
		__fallthrough;
	case XSPI_QUAD_MODE:
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

	command.command = enable_reset_code;
	err = R_XSPI_CONTROLLER_DirectTransfer(&xspi_controller_data->xspi_controller_ctrl,
					       &command, SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Reset Enable command Failed");
		return -EIO;
	}

	command.command = execute_reset_code;
	err = R_XSPI_CONTROLLER_DirectTransfer(&xspi_controller_data->xspi_controller_ctrl,
					       &command, SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		LOG_ERR("Reset Memory command Failed");
		return -EIO;
	}

	return 0;
}
#endif /* CONFIG_FLASH_EX_OP_ENABLED */

static DEVICE_API(flash, flash_renesas_rx_xspi_controller_api) = {
	.erase = flash_renesas_rx_xspi_controller_erase,
	.write = flash_renesas_rx_xspi_controller_write,
	.read = flash_renesas_rx_xspi_controller_read,
	.get_parameters = flash_renesas_rx_xspi_controller_get_parameters,
	.get_size = flash_renesas_rx_xspi_controller_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_renesas_rx_xspi_controller_page_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = flash_renesas_rx_xspi_controller_sfdp_read,
	.read_jedec_id = flash_renesas_rx_xspi_controller_read_jedec_id,
#endif /* CONFIG_FLASH_JESD216_API */
#if defined(CONFIG_FLASH_EX_OP_ENABLED)
	.ex_op = flash_renesas_rx_xspi_controller_ex_op,
#endif /* CONFIG_FLASH_EX_OP_ENABLED */
};

static int flash_renesas_rx_xspi_controller_init(const struct device *dev)
{
	const struct flash_renesas_rx_xspi_controller_config *config = dev->config;
	struct flash_renesas_rx_xspi_controller_data *xspi_controller_data = dev->data;
	uint32_t clock_freq;
	int ret;

	if (config->data_mode == XSPI_DUAL_MODE) {
		LOG_ERR("XSPI DUAL mode not supported");
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

	k_sem_init(&xspi_controller_data->sem, 1, 1);

	ret = flash_xspi_nor_probe(dev);
	if (ret) {
		LOG_ERR("NOR probe failed (%d)", ret);
		return ret;
	}

	return ret;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
#define FLASH_RENESAS_RX_XSPI_CONTROLLER_PAGES_LAYOUT(index)                                       \
	.pages_layout = {                                                                          \
		.pages_size = SPI_NOR_SECTOR_SIZE,                                                 \
		.pages_count = DT_INST_PROP(index, size) / SPI_NOR_SECTOR_SIZE,                    \
	}
#else
#define FLASH_RENESAS_RX_XSPI_CONTROLLER_PAGES_LAYOUT(index)
#endif

/* "quad-enable-requirements" (jedec,jesd216.yaml) is a string enum; convert
 * the token to its matching JESD216_DW15_QER_VAL_* value at compile time.
 */
#define DT_INST_QER_PROP_OR(inst)                                                                  \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, quad_enable_requirements),                        \
		    (_CONCAT(JESD216_DW15_QER_VAL_,                                                \
			     DT_INST_STRING_TOKEN(inst, quad_enable_requirements))),              \
		    (JESD216_DW15_QER_VAL_NONE))

#define DT_INST_QUAD_PROTOCOL_OR(inst)                                                             \
	COND_CODE_1(IS_EQ(DT_INST_STRING_TOKEN(inst, quad_protocol), qpi),                        \
		    (XSPI_QUAD_PROTOCOL_QPI), (XSPI_QUAD_PROTOCOL_IO))

#define RENESAS_RX_XSPI_CONTROLLER_INIT(index)                                                     \
                                                                                                   \
	PINCTRL_DT_DEFINE(DT_INST_PARENT(index));                                                  \
	XSPI_FLASH_BASE_ADDR_ASSERT(index);                                                        \
                                                                                                   \
	static struct flash_renesas_rx_xspi_controller_data xspi_controller_data##index = {        \
		.xspi_controller_timing_settings =                                                 \
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
				.output_assert_delay =                                             \
					_CONCAT(XSPI_CONTROLLER_,                                  \
						DT_INST_STRING_TOKEN(index, output_assert_delay)), \
			},                                                                         \
		.xspi_controller_erase_commands =                                                  \
			{                                                                          \
				.p_table = xspi_controller_data##index.erase_command_list,         \
				.length = JESD216_NUM_ERASE_TYPES,                                 \
			},                                                                         \
		.xspi_command_set_table =                                                          \
			{                                                                          \
				.frame_format = XSPI_CONTROLLER_FRAME_FORMAT_STANDARD,             \
				.latency_mode = XSPI_CONTROLLER_LATENCY_MODE_FIXED,                \
				.command_bytes = XSPI_CONTROLLER_COMMAND_BYTES_1,                  \
				.address_bytes = SPI_FLASH_ADDRESS_BYTES_3,                        \
				.status_needs_address = false,                                     \
				.p_erase_commands = &xspi_controller_data##index                   \
							     .xspi_controller_erase_commands,      \
			},                                                                         \
		.xspi_controller_command_set =                                                     \
			{                                                                          \
				.p_table = &xspi_controller_data##index.xspi_command_set_table,    \
				.length = 1,                                                       \
			},                                                                         \
		.transfer_write_enable =                                                           \
			{                                                                          \
				.command = SPI_NOR_CMD_WREN,                                       \
				.command_length = 1,                                               \
				.address_length = 0,                                               \
				.data_length = 0,                                                  \
				.dummy_cycles = 0,                                                 \
			},                                                                         \
		.transfer_read_status =                                                            \
			{                                                                          \
				.command = SPI_NOR_CMD_RDSR,                                       \
				.command_length = 1,                                               \
				.address_length = 0,                                               \
				.data_length = 1,                                                  \
				.dummy_cycles = 0,                                                 \
			},                                                                         \
		.transfer_read_wip =                                                               \
			{                                                                          \
				.command = SPI_NOR_CMD_RDSR,                                       \
				.command_length = 1,                                               \
				.address_length = 0,                                               \
				.data_length = 1,                                                  \
				.dummy_cycles = 0,                                                 \
			},                                                                         \
		.xspi_controller_config_extend =                                                   \
			{                                                                          \
				.xspi_controller_unit = DT_PROP(DT_INST_PARENT(index), unit),      \
				.channel = DT_INST_PROP(index, cs),                                \
				.p_timing_settings = &xspi_controller_data##index                  \
							      .xspi_controller_timing_settings,    \
				.p_xspi_command_set =                                              \
					&xspi_controller_data##index.xspi_controller_command_set,  \
				.data_latch_delay_clocks = XSPI_CONTROLLER_DS_TIMING_DELAY_NONE,   \
				.p_autocalibration_preamble_pattern_addr =                         \
					XSPI_FLASH_ADDR_WITH_CALIB(index),                         \
			},                                                                         \
		.xspi_flash_cfg =                                                                  \
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
				.p_extend = &xspi_controller_data##index                           \
						     .xspi_controller_config_extend,               \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	static const struct flash_renesas_rx_xspi_controller_config                                \
		xspi_controller_config##index = {                                                  \
			.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(index))),         \
			.clock_config =                                                            \
				{                                                                  \
					.mstp = (uint32_t)DT_CLOCKS_CELL(DT_INST_PARENT(index),    \
									 mstp),                    \
					.stop_bit = (uint32_t)DT_CLOCKS_CELL(                      \
						DT_INST_PARENT(index), stop_bit),                  \
				},                                                                 \
			.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_INST_PARENT(index)),                  \
			.ospi_pregs = (R_XSPI0_Type *)DT_REG_ADDR(DT_INST_PARENT(index)),          \
			.flash_size = DT_INST_PROP(index, size),                                   \
			.max_frequency = DT_INST_PROP(index, ospi_max_frequency),                  \
			.data_mode = DT_INST_PROP(index, protocol_mode),                           \
			.data_rate = DT_INST_PROP(index, data_rate),                               \
			.flash_base_address = XSPI_FLASH_BASE_ADDR(index),                         \
			.quad_protocol = (uint8_t)DT_INST_QUAD_PROTOCOL_OR(index),                 \
			.jedec_id = DT_INST_PROP(index, jedec_id),                                 \
			.special_require_info =                                                    \
				{                                                                  \
					.enter_4byte_cmd = (uint16_t)DT_INST_PROP_OR(              \
						index, enter_4byte_command, 0),                    \
					.def_8d_dummy_cycles = (uint8_t)DT_INST_PROP_OR(           \
						index, default_8d_dummy_cycles, 0),                \
					.rdsr_addr_4bytes =                                        \
						DT_INST_PROP_OR(index, rdsr_addr_4bytes, false),   \
					.qer_override = (uint8_t)DT_INST_QER_PROP_OR(index),       \
					.qer_override_set = DT_INST_NODE_HAS_PROP(                 \
						index, quad_enable_requirements),                  \
					.reset_dummy_cycles =                                      \
						DT_INST_PROP_OR(index, reset_dummy_cycles, false), \
					.fix_bfpt_addr_bytes = DT_INST_PROP_OR(                    \
						index, fix_bfpt_addr_bytes, false),                \
					.reduce_drive_strength = DT_INST_PROP_OR(                  \
						index, reduce_drive_strength, false),              \
				},                                                                 \
			.xspi_controller_param =                                                   \
				{                                                                  \
					.write_block_size = DT_INST_PROP(index, write_block_size), \
					.erase_value = ERASE_VALUE,                                \
					.caps =                                                    \
						{                                                  \
							.no_explicit_erase = false,                \
						},                                                 \
				},                                                                 \
			FLASH_RENESAS_RX_XSPI_CONTROLLER_PAGES_LAYOUT(index)};                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, flash_renesas_rx_xspi_controller_init, NULL,                  \
			      &xspi_controller_data##index, &xspi_controller_config##index,        \
			      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,                             \
			      &flash_renesas_rx_xspi_controller_api);

DT_INST_FOREACH_STATUS_OKAY(RENESAS_RX_XSPI_CONTROLLER_INIT)
