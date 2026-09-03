/*
 * Copyright (c) 2024-2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_ra_ospi_b_nor

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/clock_control/renesas_ra_cgc.h>
#include <zephyr/dt-bindings/flash_controller/ospi.h>
#include <zephyr/retention/retention.h>
#include <zephyr/drivers/retained_mem.h>
#include <r_spi_flash_api.h>
#include <r_ospi_b.h>
#include "spi_nor.h"
#include "jesd216.h"
#include <soc.h>

LOG_MODULE_REGISTER(flash_renesas_ra_ospi_b, CONFIG_FLASH_LOG_LEVEL);

/* Flash device timing */
#define MAX_TIME_ERASE (50000U)
#define MAX_TIME_WRITE (50U)
#define MAX_TIME_READ  (5U)
#define MAX_TIME_WREN  (5U)

#if defined(CONFIG_CODE_DATA_RELOCATION)
/*
 * When the application (or MCUboot) executes XIP out of this same OSPI
 * flash, k_sleep()/k_busy_wait() must not be used inside the erase/write
 * critical section below: they dispatch into kernel/timer code that is
 * not relocated to RAM, and that code becomes unreachable the moment the
 * OSPI_B peripheral leaves memory-mapped mode to run a manual command.
 * k_us_to_cyc_ceil32() itself expands to the compile-time constant
 * CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC unless the runtime-rate path is
 * selected, in which case it calls sys_clock_hw_cycles_per_sec_runtime_get()
 * which is *not* relocated and would fault - hence the BUILD_ASSERT below.
 */
BUILD_ASSERT(!IS_ENABLED(CONFIG_TIMER_READS_ITS_FREQUENCY_AT_RUNTIME),
	     "OSPI_B XIP support requires a compile-time-constant system "
	     "clock rate: CONFIG_TIMER_READS_ITS_FREQUENCY_AT_RUNTIME would "
	     "route k_us_to_cyc_ceil32() through a non-relocated helper");

/* __always_inline keeps this loop inside the (relocated) caller instead of
 * leaving a separate symbol that CONFIG_CODE_DATA_RELOCATION did not move.
 */
static __always_inline void flash_ospi_b_xip_safe_wait_us(uint32_t us)
{
	uint32_t count = k_us_to_cyc_ceil32(us);

	while (count--) {
		compiler_barrier();
	}
}
#define FLASH_OSPI_B_POLL_WAIT_US(us) flash_ospi_b_xip_safe_wait_us(us)
#else
#define FLASH_OSPI_B_POLL_WAIT_US(us) k_sleep(K_USEC(us))
#endif

#define SFDP_PARAM_SECTOR_MAP_AVAILABLE    BIT(0)
#define SFDP_PARAM_4BYTE_ADDR_AVAILABLE    BIT(1)
#define SFDP_PARAM_PROFILE_1V0_AVAILABLE   BIT(2)
#define SFDP_PARAM_SCCR_MAP_AVAILABLE      BIT(3)
#define SFDP_PARAM_SEQ_OCTAL_DDR_AVAILABLE BIT(4)

#define OSPI_B_PRV_COMSTT_MEMACCCH_MASK  (0x03 << R_XSPI0_COMSTT_MEMACCCH_Pos)
#define OSPI_B_PRV_COMSTT_WRBUFNECH_MASK (0x03 << R_XSPI0_COMSTT_WRBUFNECH_Pos)

#define OSPI_B_PRV_COMSTT_PENDING_ACTION_MASK                                                      \
	(OSPI_B_PRV_COMSTT_MEMACCCH_MASK | OSPI_B_PRV_COMSTT_WRBUFNECH_MASK)

/*
 * SFDP table buffers below are fixed-size (not heap-allocated) so that
 * probing stays safe to run while the application executes XIP out of
 * this same OSPI flash (see CONFIG_CODE_DATA_RELOCATION): the heap
 * allocator is not relocated to RAM. Sizes for tables whose length is
 * fixed by the JESD216 spec (or already enforced elsewhere in this
 * driver) are plain constants; sizes that genuinely vary with the flash
 * device (region count, sector map) are Kconfig-tunable.
 */
#define OSPI_B_BFP_MAX_DWORDS      20 /* JESD216D Basic Flash Parameter table max length */
#define OSPI_B_PROFILE1_MAX_DWORDS 5  /* JESD216D-01 6.7: xSPI Profile 1.0 table length */
#define OSPI_B_ADDR_4B_MAX_DWORDS  2  /* JESD216D-01 6.6: 4-Byte Address Instruction table length */
/* Matches the cap flash_ospi_b_seq_8d_enable() already enforces on octal_8d_len. */
#define OSPI_B_SEQ_OCTAL_DDR_MAX_DWORDS 8

struct flash_region_info {
	uint32_t offset;
	uint32_t size;
	uint8_t erase_mask;
};

/*
 * DT_INST_PROP(0, ...) below hardcodes instance 0: these bounds size fields
 * of a single shared struct *type*, and this driver's struct definitions
 * can't vary per DT_INST_FOREACH_STATUS_OKAY() instance. Only correct as
 * long as a board has a single renesas,ra-ospi-b-nor node.
 */
struct flash_regions_layout {
	struct flash_region_info regions[DT_INST_PROP(0, max_regions)];
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout layout[DT_INST_PROP(0, max_regions)];
#endif
	size_t region_count;
};

struct flash_sfdp_region {
	/* SFDP tables populated during probe */
	struct jesd216_sfdp_header header; /* SFDP header */
	/* all param headers (sfdp_nph+1 entries) */
	struct jesd216_param_header phdr[DT_INST_PROP(0, max_sfdp_param_headers)];
	uint32_t bfp_buf[OSPI_B_BFP_MAX_DWORDS];                 /* Basic Flash Parameters */
	uint32_t pr1_buf[OSPI_B_PROFILE1_MAX_DWORDS];            /* xSPI Profile 1.0 */
	uint32_t addr_4b_buf[OSPI_B_ADDR_4B_MAX_DWORDS];         /* 4-Byte Address Instructions */
	uint32_t sccr_buf[DT_INST_PROP(0, sccr_max_dwords)];     /* SCCR map (full table) */
	uint32_t octal_ddr_buf[OSPI_B_SEQ_OCTAL_DDR_MAX_DWORDS]; /* Octal DDR enable sequences */
	uint32_t smpt_buf[DT_INST_PROP(0, smpt_max_dwords)];     /* Sector Map Parameter table */
	uint32_t param_available; /* bitmask of available param tables */
	uint8_t pr1_len;          /* pr1_buf length in DWORDs */
	uint8_t addr_4b_len;      /* addr_4b_buf length in DWORDs */
	uint8_t sccr_len;         /* sccr_buf length in DWORDs */
	uint8_t octal_ddr_len;    /* octal_ddr_buf length in DWORDs */
	uint8_t smp_len;          /* smpt_buf length in DWORDs */
};

/* Since some NOR flash devices have incorrect SFDP values, this special require will override them
 */
struct flash_special_require_info {
	uint16_t enter_4byte_cmd;
	uint8_t def_8d_dummy_cycles;
	uint8_t qer_value;
	uint8_t seq_enable_444;
	bool rdsr_addr_4bytes;
};

struct flash_renesas_ra_ospi_b_data {
	ospi_b_instance_ctrl_t ospi_b_ctrl;
	spi_flash_cfg_t ospi_b_cfg;
	ospi_b_timing_setting_t ospi_b_timing_settings;
	ospi_b_xspi_command_set_t ospi_b_command_set_table[2];
	ospi_b_extended_cfg_t ospi_b_config_extend;
	spi_flash_erase_command_t erase_command_list[JESD216_NUM_ERASE_TYPES];
	ospi_b_table_t ospi_b_command_set;
	ospi_b_table_t ospi_b_erase_commands;
	struct flash_sfdp_region sfdp;
	struct flash_regions_layout regions_layout;
	uint8_t address_mode;
	struct k_sem sem;
	uint32_t calib_sector_size;
	bool xip_warm_attach;
};

struct flash_renesas_ra_ospi_b_config {
	const struct device *clock_dev;
	struct clock_control_ra_subsys_cfg clock_subsys;
	const bsp_clocks_source_t clock_src;
	uint32_t clock_div;
	const struct pinctrl_dev_config *pcfg;
	const struct flash_parameters flash_parameters;
	const struct flash_special_require_info special_require_info;
	const uint32_t start_address;
	const uint32_t max_frequency;
	const uint32_t erase_block_size;
	const size_t flash_size;
	const uint8_t desired_protocol;
	const uint8_t jedec_id[JESD216_READ_ID_LEN];
};

static int flash_ospi_b_wait_operation(ospi_b_instance_ctrl_t *p_ctrl, uint32_t timeout)
{
	spi_flash_status_t status;
	fsp_err_t err;
	int ret = 0;
	bool need_update_protocol = false;

	if (p_ctrl->spi_protocol == SPI_FLASH_PROTOCOL_1S_4S_4S) {
		err = R_OSPI_B_SpiProtocolSet(p_ctrl, SPI_FLASH_PROTOCOL_1S_1S_1S);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		need_update_protocol = true;
	}

	status.write_in_progress = true;
	while (status.write_in_progress) {
		FLASH_OSPI_B_POLL_WAIT_US(50);
		/* Get device status */
		err = R_OSPI_B_StatusGet(p_ctrl, &status);
		if (timeout == 0 || err != FSP_SUCCESS) {
			LOG_DBG("Time out for operation");
			ret = -EIO;
			break;
		}
		timeout--;
	}

	if (need_update_protocol) {
		/* Quad I/O read mode: use DirectTransfer PP in 1S-1S-1S.
		 *
		 * Protocol is switched exactly ONCE for the entire write (not
		 * per-chunk).  A per-chunk SpiProtocolSet round-trip caused a
		 * timing window between consecutive calls that corrupted flash WEL,
		 * making every odd-indexed chunk silently fail to program. */
		while (p_ctrl->p_reg->COMSTT & OSPI_B_PRV_COMSTT_PENDING_ACTION_MASK) {
		}

		err = R_OSPI_B_SpiProtocolSet(p_ctrl, SPI_FLASH_PROTOCOL_1S_4S_4S);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
	}

	return ret;
}

static int flash_ospi_b_write_enable(struct flash_renesas_ra_ospi_b_data *data)
{
	fsp_err_t err;
	uint32_t timeout = MAX_TIME_WREN;

	spi_flash_direct_transfer_t transfer_write_enable = {
		.command = SPI_NOR_CMD_WREN,
		.command_length = 1,
		.address_length = 0,
		.data_length = 0,
		.dummy_cycles = 0,
	};

	spi_flash_direct_transfer_t transfer_read_status = {
		.command = SPI_NOR_CMD_RDSR,
		.command_length = 1,
		.address_length = 0,
		.data_length = 1,
		.dummy_cycles = 0,
	};

	/* Transfer write enable command */
	err = R_OSPI_B_DirectTransfer(&data->ospi_b_ctrl, &transfer_write_enable,
				      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	while (timeout) {
		err = R_OSPI_B_DirectTransfer(&data->ospi_b_ctrl, &transfer_read_status,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}

		if (((transfer_read_status.data & BIT(1)) >> 1) == 1U) {
			break;
		}

		FLASH_OSPI_B_POLL_WAIT_US(50);
		timeout--;
	}

	if (timeout == 0) {
		LOG_DBG("Time out for operation");
		return -EIO;
	}

	return 0;
}

static bool flash_ospi_b_is_valid_address(const struct device *dev, off_t offset, size_t len)
{
	struct flash_renesas_ra_ospi_b_data *data = dev->data;
	struct flash_regions_layout *layout = &data->regions_layout;
	off_t flash_end;

	if (offset < 0 || len == 0 || layout->region_count == 0) {
		return false;
	}

	flash_end = (off_t)(layout->regions[layout->region_count - 1].offset +
			    layout->regions[layout->region_count - 1].size);

	if (data->ospi_b_ctrl.spi_protocol == SPI_FLASH_PROTOCOL_8D_8D_8D) {
		/* Accessible end: last region's end minus the calibration sector */
		flash_end -= (off_t)data->calib_sector_size;
	}

	return (offset < flash_end) && ((off_t)len <= flash_end - offset);
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_renesas_ra_ospi_b_page_layout(const struct device *dev,
						const struct flash_pages_layout **layout,
						size_t *layout_size)
{
	const struct flash_renesas_ra_ospi_b_data *data = dev->data;

	*layout = data->regions_layout.layout;
	*layout_size = data->regions_layout.region_count;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#if defined(CONFIG_FLASH_JESD216_API)
static int flash_renesas_ra_ospi_b_read_jedec_id(const struct device *dev, uint8_t *jedec_id)
{
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;

	if (jedec_id == NULL) {
		return -EINVAL;
	}

	memcpy(jedec_id, config->jedec_id, JESD216_READ_ID_LEN);

	return 0;
}

static int flash_renesas_ra_ospi_b_sfdp_read(const struct device *dev, off_t offset, void *data,
					     size_t len)
{
	struct flash_renesas_ra_ospi_b_data *ospi_b_data = dev->data;
	struct flash_sfdp_region *sfdp = &ospi_b_data->sfdp;
	/* SFDP address space layout:
	 *   [0,           hdr_end)  : fixed SFDP header (8 bytes)
	 *   [hdr_end,     phdr_end) : parameter headers  ((nph+1) * 8 bytes)
	 *   [phdr_end, ..)          : parameter table bodies at addresses given
	 *                             by each phdr[i].ptp[], with possible
	 *                             reserved gaps between them.
	 * Any access that falls in a gap or an unread table returns -ENODATA.
	 */
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
			/* Fixed SFDP header [0, hdr_end) */
			src = (uint8_t *)&sfdp->header + cur;
			available = hdr_end - cur;
		} else if (cur < phdr_end) {
			/* Parameter headers [hdr_end, phdr_end) */
			if (sfdp->phdr == NULL) {
				return -ENODATA;
			}
			src = (uint8_t *)sfdp->phdr + (cur - hdr_end);
			available = phdr_end - cur;
		} else {
			/* Parameter table bodies.
			 * Find the phdr whose address range covers cur.
			 * Any offset not covered by a known, successfully-read
			 * table (including reserved gaps between tables) is
			 * treated as unavailable.
			 */
			uint8_t *param_buf = NULL;
			off_t param_start = 0;
			size_t param_size = 0;

			for (uint8_t i = 0; i <= sfdp->header.nph; i++) {
				off_t start = jesd216_param_addr(&sfdp->phdr[i]);
				size_t sz = sfdp->phdr[i].len_dw * sizeof(uint32_t);

				/* cur is not inside this table's range */
				if (cur < start || cur >= (off_t)(start + sz)) {
					continue;
				}

				/* cur is inside phdr[i] — map to the pre-read buffer */
				switch (jesd216_param_id(&sfdp->phdr[i])) {
				case JESD216_SFDP_PARAM_ID_BFP:
					param_buf = (uint8_t *)sfdp->bfp_buf;
					break;
				case JESD216_SFDP_PARAM_ID_SECTOR_MAP:
					param_buf = (uint8_t *)sfdp->smpt_buf;
					break;
				case JESD216_SFDP_PARAM_ID_SEQ_OCTAL_DDR:
					param_buf = (uint8_t *)sfdp->octal_ddr_buf;
					break;
				case JESD216_SFDP_PARAM_ID_SCCR_MAP:
					param_buf = (uint8_t *)sfdp->sccr_buf;
					break;
				case JESD216_SFDP_PARAM_ID_XSPI_PROFILE_1V0:
					param_buf = (uint8_t *)sfdp->pr1_buf;
					break;
				case JESD216_SFDP_PARAM_ID_4B_ADDR_INSTR:
					param_buf = (uint8_t *)sfdp->addr_4b_buf;
					break;
				default:
					break;
				}

				param_start = start;
				param_size = sz;
				break;
			}

			/* cur is in a reserved gap, an unknown table type, or a
			 * table that failed to be read during probe (buf == NULL).
			 */
			if (param_buf == NULL) {
				return -ENODATA;
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
#endif /* CONFIG_FLASH_JESD216_API */

/* Find the region that contains the given offset, or -EINVAL if none. */
static int flash_ospi_b_find_region(struct flash_renesas_ra_ospi_b_data *data, off_t offset,
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
 * Return the largest erase size from the region's erase_mask that is aligned
 * to offset and fits within remaining bytes.
 *
 * The erase loop calls this once per iteration after re-looking up the region,
 * so when offset crosses a region boundary the correct erase types for the new
 * region are automatically selected.
 */
static uint32_t flash_ospi_b_find_erase_size(struct flash_renesas_ra_ospi_b_data *data,
					     struct flash_region_info *region, size_t region_remain)
{
	/* Iterate from largest to smallest erase type */
	for (int8_t i = JESD216_NUM_ERASE_TYPES - 1; i >= 0; i--) {
		if ((region->erase_mask & BIT(i)) &&
		    (data->erase_command_list[i].size <= region_remain) &&
		    (region_remain % data->erase_command_list[i].size == 0)) {
			return data->erase_command_list[i].size;
		}
	}

	return 0;
}

static int flash_renesas_ra_ospi_b_erase(const struct device *dev, off_t offset, size_t len)
{
	struct flash_renesas_ra_ospi_b_data *ospi_b_data = dev->data;
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;
	struct flash_region_info *region;
	fsp_err_t err;
	struct flash_pages_info page_info_start, page_info_end;
	int ret = 0;
	bool need_update_protocol = false;
#if defined(CONFIG_CODE_DATA_RELOCATION)
	unsigned int key;
#endif

	if (!len) {
		return 0;
	}

	if (!flash_ospi_b_is_valid_address(dev, offset, len)) {
		LOG_ERR("Address or size exceeds expected values: "
			"Address 0x%lx, size %zu",
			(long)offset, len);
		return -EINVAL;
	}

	/* Verify offset and end are aligned to sector boundaries in the page layout */
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

	ret = k_sem_take(&ospi_b_data->sem, K_FOREVER);
	if (ret != 0) {
		LOG_ERR("Failed to acquire semaphore for erase");
		return ret;
	}

#if defined(CONFIG_CODE_DATA_RELOCATION)
	/*
	 * From here until the protocol is restored below, OSPI_B is out of
	 * memory-mapped mode. If the application is executing out of this
	 * same flash, no ISR can be allowed to run in that window - its
	 * handler code may not be reachable.
	 */
	key = irq_lock();
#endif

	if (ospi_b_data->ospi_b_ctrl.spi_protocol == SPI_FLASH_PROTOCOL_1S_4S_4S) {
		/* Quad I/O read mode: use DirectTransfer PP in 1S-1S-1S.
		 *
		 * Protocol is switched exactly ONCE for the entire write (not
		 * per-chunk).  A per-chunk SpiProtocolSet round-trip caused a
		 * timing window between consecutive calls that corrupted flash WEL,
		 * making every odd-indexed chunk silently fail to program. */
		err = R_OSPI_B_SpiProtocolSet(&ospi_b_data->ospi_b_ctrl,
					      SPI_FLASH_PROTOCOL_1S_1S_1S);
		if (err != FSP_SUCCESS) {
			ret = -EIO;
			goto out;
		}
		need_update_protocol = true;
	}

	while (len > 0) {
		/* TODO: support entire chip erase */
		uint32_t erase_size, erase_time;
		size_t region_remain;
		size_t erase_len;

		/*
		 * Re-look up the region at every iteration so that when offset crosses a region
		 * boundary the correct erase types and minimum erase granularity for the new region
		 * are used.
		 */
		ret = flash_ospi_b_find_region(ospi_b_data, offset, &region, &region_remain);
		if (ret != 0) {
			LOG_ERR("No region found for offset 0x%lx", (long)offset);
			break;
		}

		erase_len = MIN(len, region_remain);

		erase_size = flash_ospi_b_find_erase_size(ospi_b_data, region, erase_len);
		if (erase_size == 0) {
			LOG_ERR("region at offset 0x%lx has no erase type for remaining "
				"length %zu",
				(long)offset, erase_len);
			ret = -EINVAL;
			break;
		}

		erase_time = erase_len / erase_size;

		for (uint32_t i = 0; i < erase_time; i++) {
			err = R_OSPI_B_Erase(&ospi_b_data->ospi_b_ctrl,
					     (uint8_t *)(config->start_address + offset),
					     erase_size);
			if (err != FSP_SUCCESS) {
				LOG_ERR("Erase at address 0x%lx, size %u Failed %d", (long)offset,
					erase_size, err);
				ret = -EIO;
				break;
			}

			ret = flash_ospi_b_wait_operation(&ospi_b_data->ospi_b_ctrl,
							  MAX_TIME_ERASE);
			if (ret != 0) {
				LOG_ERR("Wait for erase to finish timeout");
				break;
			}

			offset += erase_size;
			len -= erase_size;
		}

		if (ret != 0) {
			break;
		}
	}

	if (need_update_protocol) {
		/* Quad I/O read mode: use DirectTransfer PP in 1S-1S-1S.
		 *
		 * Protocol is switched exactly ONCE for the entire write (not
		 * per-chunk).  A per-chunk SpiProtocolSet round-trip caused a
		 * timing window between consecutive calls that corrupted flash WEL,
		 * making every odd-indexed chunk silently fail to program. */
		while (ospi_b_data->ospi_b_ctrl.p_reg->COMSTT &
		       OSPI_B_PRV_COMSTT_PENDING_ACTION_MASK) {
		}

		err = R_OSPI_B_SpiProtocolSet(&ospi_b_data->ospi_b_ctrl,
					      SPI_FLASH_PROTOCOL_1S_4S_4S);
		if (err != FSP_SUCCESS) {
			ret = -EIO;
			goto out;
		}
	}

out:
#if defined(CONFIG_CODE_DATA_RELOCATION)
	irq_unlock(key);
#endif
	k_sem_give(&ospi_b_data->sem);

	return ret;
}

static int flash_renesas_ra_ospi_b_read(const struct device *dev, off_t offset, void *data,
					size_t len)
{
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;
	struct flash_renesas_ra_ospi_b_data *ospi_b_data = dev->data;
	uint32_t *src = (uint32_t *)(config->start_address + (uint32_t)offset);
	uint32_t *dst = (uint32_t *)data;
	int ret;

	if (!len) {
		return 0;
	}

	if (!flash_ospi_b_is_valid_address(dev, offset, len)) {
		LOG_ERR("Address or size exceeds expected values: "
			"Address 0x%lx, size %zu",
			(long)offset, len);
		return -EINVAL;
	}

	ret = k_sem_take(&ospi_b_data->sem, K_FOREVER);
	if (ret != 0) {
		LOG_ERR("Failed to acquire semaphore for read");
		return ret;
	}

	/* Head: 1 odd byte -> advance to 2-byte alignment */
	if (((uintptr_t)src & 1) && len > 0) {
		uint16_t val = *(volatile uint16_t *)((uint8_t *)src - 1);

		*(uint8_t *)dst = (uint8_t)((val >> 8) & 0xff);
		src = (uint32_t *)((uint8_t *)src + 1);
		dst = (uint32_t *)((uint8_t *)dst + 1);
		len--;
	}

	/* Bulk: copy 4-byte aligned chunks, dst may be unaligned */
	if (len >= 4) {
		size_t bulk = len & ~3U;

		memcpy(dst, src, bulk);
		src = (uint32_t *)((uint8_t *)src + bulk);
		dst = (uint32_t *)((uint8_t *)dst + bulk);
		len -= bulk;
	}

	/* Tail: remaining < 4 bytes -> read a full 4-byte word then copy only
	 * the needed bytes into dst.
	 */
	if (len > 0 && len < 4) {
		uint32_t tmp = *(volatile uint32_t *)src;
		uint8_t *d = (uint8_t *)dst;

		for (size_t i = 0; i < len; i++) {
			d[i] = (uint8_t)((tmp >> (i * 8)) & 0xff);
		}
	}

	k_sem_give(&ospi_b_data->sem);

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
	bool need_update_protocol = false;
#if defined(CONFIG_CODE_DATA_RELOCATION)
	unsigned int key;
#endif

	if (!len) {
		return 0;
	}

	if (data != NULL) {
		p_src = data;
	} else {
		LOG_ERR("The data buffer is NULL");
		return -EINVAL;
	}

	if (!flash_ospi_b_is_valid_address(dev, offset, len)) {
		LOG_ERR("Address or size exceeds expected values: "
			"Address 0x%lx, size %zu",
			(long)offset, len);
		return -EINVAL;
	}

	if ((offset % config->flash_parameters.write_block_size) ||
	    (len % config->flash_parameters.write_block_size)) {
		return -EINVAL;
	}

	ret = k_sem_take(&ospi_b_data->sem, K_FOREVER);
	if (ret != 0) {
		LOG_ERR("Failed to acquire semaphore for erase");
		return ret;
	}

#if defined(CONFIG_CODE_DATA_RELOCATION)
	/*
	 * From here until the protocol is restored below, OSPI_B is out of
	 * memory-mapped mode. If the application is executing out of this
	 * same flash, no ISR can be allowed to run in that window - its
	 * handler code may not be reachable.
	 */
	key = irq_lock();
#endif

	if (ospi_b_data->ospi_b_ctrl.spi_protocol == SPI_FLASH_PROTOCOL_1S_4S_4S) {
		/* Quad I/O read mode: use DirectTransfer PP in 1S-1S-1S.
		 *
		 * Protocol is switched exactly ONCE for the entire write (not
		 * per-chunk).  A per-chunk SpiProtocolSet round-trip caused a
		 * timing window between consecutive calls that corrupted flash WEL,
		 * making every odd-indexed chunk silently fail to program. */

		/* 4. Switch to 1S-1S-1S with CMCFG1 updated to Fast Read 0x0B */
		err = R_OSPI_B_SpiProtocolSet(&ospi_b_data->ospi_b_ctrl,
					      SPI_FLASH_PROTOCOL_1S_1S_1S);
		if (err != FSP_SUCCESS) {
			ret = -EIO;
			goto out;
		}
		need_update_protocol = true;
	}

	while (len > 0) {
		size = len > ospi_b_data->ospi_b_cfg.page_size_bytes
			       ? ospi_b_data->ospi_b_cfg.page_size_bytes
			       : len;

		err = R_OSPI_B_Write(&ospi_b_data->ospi_b_ctrl, p_src,
				     (uint8_t *)(config->start_address + offset), size);
		if (err != FSP_SUCCESS) {
			LOG_ERR("Write at address 0x%lx, size %zu, error: %d", offset, size, err);
			ret = -EIO;
			break;
		}

		ret = flash_ospi_b_wait_operation(&ospi_b_data->ospi_b_ctrl, MAX_TIME_WRITE);
		if (ret != 0) {
			LOG_ERR("Wait for write to finish timeout");
			break;
		}

		len -= size;
		offset += size;
		p_src = p_src + size;
	}

	if (need_update_protocol) {
		/* Quad I/O read mode: use DirectTransfer PP in 1S-1S-1S.
		 *
		 * Protocol is switched exactly ONCE for the entire write (not
		 * per-chunk).  A per-chunk SpiProtocolSet round-trip caused a
		 * timing window between consecutive calls that corrupted flash WEL,
		 * making every odd-indexed chunk silently fail to program. */
		while (ospi_b_data->ospi_b_ctrl.p_reg->COMSTT &
		       OSPI_B_PRV_COMSTT_PENDING_ACTION_MASK) {
		}

		err = R_OSPI_B_SpiProtocolSet(&ospi_b_data->ospi_b_ctrl,
					      SPI_FLASH_PROTOCOL_1S_4S_4S);
		if (err != FSP_SUCCESS) {
			ret = -EIO;
			goto out;
		}
	}

out:
#if defined(CONFIG_CODE_DATA_RELOCATION)
	irq_unlock(key);
#endif
	k_sem_give(&ospi_b_data->sem);

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

static int flash_ospi_b_setup_calibrate_data(struct flash_renesas_ra_ospi_b_data *data,
					     uint32_t flash_start_addr, size_t flash_size,
					     bool pattern_word_swapped)
{
	uint32_t autocalibration_data[4];
	spi_flash_erase_command_t *erase_type = NULL;
	uint32_t calib_sector_offset, calib_sector_address;
	fsp_err_t err;
	int ret;

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

	/* Calculate calib sector offset and sector size */
	for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
		if (data->regions_layout.regions[data->regions_layout.region_count - 1].erase_mask &
		    BIT(i)) {
			/* Use the smallest sector size for erase */
			erase_type = &data->erase_command_list[i];
			break;
		}
	}

	if (erase_type == NULL) {
		return -EIO;
	}

	calib_sector_offset = (uint32_t)flash_size - erase_type->size;
	calib_sector_address = flash_start_addr + calib_sector_offset;
	data->ospi_b_config_extend.p_autocalibration_preamble_pattern_addr =
		(uint8_t *)(calib_sector_address);

	/* Read via direct transfer to bypass the OSPI_B prefetch buffer */
	if (memcmp((uint8_t *)calib_sector_address, &autocalibration_data,
		   sizeof(autocalibration_data)) != 0) {

		/* Erase the flash sector that stores auto-calibration data */
		err = R_OSPI_B_Erase(&data->ospi_b_ctrl, (uint8_t *)calib_sector_address,
				     erase_type->size);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Erase the flash sector Failed");
			return -EIO;
		}

		/* Wait until erase operation completes */
		ret = flash_ospi_b_wait_operation(&data->ospi_b_ctrl, MAX_TIME_ERASE);
		if (ret != 0) {
			LOG_DBG("Wait for erase operation completes Failed");
			return ret;
		}

		/* Write auto-calibration data to the flash */
		err = R_OSPI_B_Write(&data->ospi_b_ctrl, (uint8_t *)&autocalibration_data,
				     (uint8_t *)calib_sector_address, sizeof(autocalibration_data));
		if (err != FSP_SUCCESS) {
			LOG_DBG("Write auto-calibration data Failed %d", err);
			return -EIO;
		}

		/* Wait until write operation completes */
		ret = flash_ospi_b_wait_operation(&data->ospi_b_ctrl, MAX_TIME_WRITE);
		if (ret != 0) {
			LOG_DBG("Wait for write operation completes Failed");
			return ret;
		}

		if (memcmp((uint8_t *)calib_sector_address, &autocalibration_data,
			   sizeof(autocalibration_data)) != 0) {
			LOG_DBG("Confirm for autocalibration data fail");
			return -EIO;
		}
	}

	data->calib_sector_size = erase_type->size;

	return 0;
}

/* this helper is only used in 1S-1S-1S mode*/
static int flash_ospi_b_sfdp_read_helper(ospi_b_instance_ctrl_t *p_ctrl, void *data, off_t offset,
					 size_t len)
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

		/* Max len is 8 bytes*/
		if (len > 8) {
			transfer_sfdp.data_length = 8;
			buf_size = 8;
		} else {
			transfer_sfdp.data_length = len;
			buf_size = len;
		}

		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer_sfdp,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}

		memcpy(p_dst, &transfer_sfdp.data_u64, buf_size);
		len -= buf_size;
		offset += buf_size;
		p_dst += buf_size;
	}

	return 0;
}

/* this helper is only used in 1S-1S-1S mode*/
static int flash_ospi_b_jedec_id_read_helper(ospi_b_instance_ctrl_t *p_ctrl, uint8_t *data)
{
	fsp_err_t err;
	spi_flash_direct_transfer_t transfer_jedec_id = {.command = SPI_NOR_CMD_RDID,
							 .command_length = 1,
							 .address = 0,
							 .address_length = 0,
							 .data_length = 3,
							 .dummy_cycles = 0};

	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer_jedec_id,
				      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}
	memcpy(data, &transfer_jedec_id.data, transfer_jedec_id.data_length);

	return 0;
}

/*
 * Some flash devices stay latched in QPI (4S-4S-4S) mode across a warm reboot
 * even though OM_RESET toggled CS, so the default 1S-1S-1S SFDP read comes back
 * garbled. The Reset Enable/Reset Memory commands must be issued in the protocol
 * the device is currently expecting, so the OSPI unit is switched to 4S-4S-4S to
 * send them, then switched back to 1S-1S-1S once the device has reset back to its
 * default SPI mode.
 *
 * command_set_table[0] is still the only populated table entry at this point in
 * probe (command_set_table[1] is configured later, once the target mode is known),
 * so it is temporarily repurposed to expose a 4S-4S-4S entry for
 * R_OSPI_B_SpiProtocolSet() to match against, then restored.
 */
static int flash_ospi_b_software_reset(struct flash_renesas_ra_ospi_b_data *data)
{
	fsp_err_t err;
	spi_flash_direct_transfer_t transfer_reset_en = {
		.command = SPI_NOR_CMD_RESET_EN,
		.command_length = 1,
		.address_length = 0,
		.data_length = 0,
		.dummy_cycles = 0,
	};
	spi_flash_direct_transfer_t transfer_reset_mem = {
		.command = SPI_NOR_CMD_RESET_MEM,
		.command_length = 1,
		.address_length = 0,
		.data_length = 0,
		.dummy_cycles = 0,
	};

	data->ospi_b_command_set_table[0].protocol = SPI_FLASH_PROTOCOL_4S_4S_4S;
	err = R_OSPI_B_SpiProtocolSet(&data->ospi_b_ctrl, SPI_FLASH_PROTOCOL_4S_4S_4S);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	err = R_OSPI_B_DirectTransfer(&data->ospi_b_ctrl, &transfer_reset_en,
				      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	err = R_OSPI_B_DirectTransfer(&data->ospi_b_ctrl, &transfer_reset_mem,
				      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	/* tRST: device needs time to complete the internal reset before it will
	 * respond to further commands.
	 */
	FLASH_OSPI_B_POLL_WAIT_US(30);

	data->ospi_b_command_set_table[0].protocol = SPI_FLASH_PROTOCOL_1S_1S_1S;
	err = R_OSPI_B_SpiProtocolSet(&data->ospi_b_ctrl, SPI_FLASH_PROTOCOL_1S_1S_1S);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static int flash_ospi_b_update_page_layout(struct flash_renesas_ra_ospi_b_data *data)
{
	struct flash_pages_layout *layout = data->regions_layout.layout;
	uint32_t i, j;

	/* region_count is already bounds-checked against ARRAY_SIZE(layout)
	 * wherever regions_layout.regions/region_count are populated.
	 */
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

static int flash_ospi_b_read_map_format(struct flash_renesas_ra_ospi_b_data *data,
					const uint32_t *config_map)
{
	struct flash_region_info *regions;
	uint32_t region_count;
	uint32_t offset;
	uint8_t regions_erase_type = 0;
	uint8_t i, j;

	region_count = JESD216_SFDP_SMP_MAP_REGION_COUNT(config_map[0]);
	if (region_count == 0) {
		LOG_DBG("Invalid region count in sector map");
		return -EIO;
	}

	if (region_count > ARRAY_SIZE(data->regions_layout.regions)) {
		LOG_ERR("Sector map region count %u exceeds the max-regions DT property "
			"(%zu)",
			region_count, ARRAY_SIZE(data->regions_layout.regions));
		return -ENOMEM;
	}

	regions = data->regions_layout.regions;
	offset = 0;

	/* Populate regions. */
	for (i = 0; i < region_count; i++) {
		j = i + 1; /* index for the region dword */
		regions[i].offset = offset;
		regions[i].size = JESD216_SFDP_SMP_MAP_REGION_SIZE(config_map[j]);
		regions[i].erase_mask = JESD216_SFDP_SMP_MAP_REGION_ERASE_TYPE(config_map[j]);

		/*
		 * regions_erase_type mask will indicate all the erase types
		 * supported in this configuration map.
		 */
		regions_erase_type |= regions[i].erase_mask;

		offset = regions[i].offset + regions[i].size;
	}

	if (!regions_erase_type) {
		return -EINVAL;
	}

	data->regions_layout.region_count = region_count;

	/*
	 * BFPT advertises all the erase types supported by all the possible
	 * map configurations. Mask out the erase types that are not supported
	 * by the current map configuration.
	 */
	for (i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
		if (!(regions_erase_type & BIT(i)) && (data->erase_command_list[i].command != 0)) {
			data->erase_command_list[i].command = 0;
			data->erase_command_list[i].size = 0;
		}
	}

	return 0;
}

static uint32_t *flash_ospi_b_get_map_in_use(struct flash_renesas_ra_ospi_b_data *data,
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
	/* Determine if there are any optional Detection Command Descriptors */
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

		/* Read the configuration data */
		fsp_err = R_OSPI_B_DirectTransfer(&data->ospi_b_ctrl, &transfer_read_cfg,
						  SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (fsp_err != FSP_SUCCESS) {
			ret = NULL;
			return ret;
		}

		/*
		 * Build an index value that is used to select the Sector Map
		 * Configuration that is currently in use.
		 */
		map_id = map_id << 1 | !!(transfer_read_cfg.data & read_data_mask);
	}

	/*
	 * If command descriptors are provided, they always precede map
	 * descriptors in the table. There is no need to start the iteration
	 * over smpt array all over again.
	 *
	 * Find the matching configuration map.
	 */
	ret = NULL;
	while (i < smp_len) {
		if (JESD216_SFDP_SMP_MAP_ID(smpt_buf[i]) == map_id) {
			ret = smpt_buf + i;
			break;
		}

		/*
		 * If there are no more configuration map descriptors and no
		 * configuration ID matched the configuration identifier, the
		 * sector address map is unknown.
		 */
		if (smpt_buf[i] & JESD216_SFDP_SMP_DESC_END) {
			break;
		}

		/* increment the table index to the next map */
		i += JESD216_SFDP_SMP_MAP_REGION_COUNT(smpt_buf[i]) + 1;
	}

	return ret;
}

static int flash_ospi_b_smpt_handle(struct flash_renesas_ra_ospi_b_data *data, uint32_t *smpt_buf,
				    uint8_t smp_len)
{
	uint32_t *sector_map;
	int ret;

	sector_map = flash_ospi_b_get_map_in_use(data, smpt_buf, smp_len);
	if (sector_map == NULL) {
		LOG_DBG("Failed to get sector map");
		return -EIO;
	}

	ret = flash_ospi_b_read_map_format(data, sector_map);
	if (ret) {
		LOG_DBG("Failed to read map format");
		return -EIO;
	}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	ret = flash_ospi_b_update_page_layout(data);
	if (ret) {
		LOG_DBG("Failed to update page layout");
		return -EIO;
	}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

	return 0;
}

static void flash_ospi_b_parse_erase_4b(spi_flash_erase_command_t *erase_command,
					uint32_t *add_4b_param)
{
	/*
	 * JESD216H 6.7.3 — 4-Byte Address Instruction Table, 1st DWORD:
	 *   Bit 9+i: Erase Type (i+1) instruction with 4-byte address is supported.
	 *
	 * JESD216H 6.7.4 — 4-Byte Address Instruction Table, 2nd DWORD:
	 *   Bits [7+8i : 8i]: 4-byte address instruction opcode for Erase Type (i+1).
	 */
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

static int flash_ospi_b_4byte_enable(struct flash_renesas_ra_ospi_b_data *data, uint32_t en4b,
				     uint16_t enter_4byte_cmd)
{
	ospi_b_instance_ctrl_t *p_ctrl = &data->ospi_b_ctrl;
	fsp_err_t err;
	int ret;
	spi_flash_direct_transfer_t transfer_4byte_enable = {
		.address = 0,
		.data = 0,
		.dummy_cycles = 0,
		.command_length = 1,
		.address_length = 0,
		.data_length = 0,
	};

	if (enter_4byte_cmd != 0) {
		transfer_4byte_enable.command = enter_4byte_cmd;
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer_4byte_enable,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
	} else if ((en4b & BIT(0))) {
		/* Issue instruction 0xB7 */
		transfer_4byte_enable.command = 0xB7;
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer_4byte_enable,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
	} else if (en4b & BIT(1)) {
		/* Issue write enable, then instruction 0xB7 */
		ret = flash_ospi_b_write_enable(data);
		if (ret != 0) {
			return ret;
		}
		transfer_4byte_enable.command = 0xB7;
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer_4byte_enable,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}

	} else if (en4b & BIT(4)) {
		/* Set bit 0 of 16 bit configuration register */
		transfer_4byte_enable.command = 0xB5;
		transfer_4byte_enable.data_length = 2;
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer_4byte_enable,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}

		transfer_4byte_enable.command = 0xB1;
		transfer_4byte_enable.data |= BIT(0);
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer_4byte_enable,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
	} else if ((en4b & BIT(6)) || (en4b & BIT(5))) {
		/* Flash is always in 4 byte mode. We just need to configure LUT */
	} else {
		/* Other methods not supported. Include:
		 * BIT(2): 8-bit volatile extended address register used to define A[31:24] bits.
		 * BIT(3): 8-bit volatile bank register used to define A[31:24] bits.
		 */
		return -ENOTSUP;
	}

	return 0;
}

static int flash_ospi_b_quad_enable(struct flash_renesas_ra_ospi_b_data *data, uint8_t qer)
{
	ospi_b_instance_ctrl_t *p_ctrl = &data->ospi_b_ctrl;
	spi_flash_direct_transfer_t transfer;
	fsp_err_t err;
	int ret;
	uint8_t sr1_value, sr2_value;

	switch (qer) {
	case JESD216_DW15_QER_VAL_NONE:
		/* No init needed */
		return 0;
	case JESD216_DW15_QER_VAL_S2B1v1:
	case JESD216_DW15_QER_VAL_S2B1v4:
	case JESD216_DW15_QER_VAL_S2B1v5:
		/* QE = bit 1 of SR2; read SR1 (RDSR) + SR2 (RDSR2), write both with WRSR (2 bytes)
		 */
		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_RDSR,
			.command_length = 1,
			.address_length = 0,
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		sr1_value = transfer.data & 0xFF;
		LOG_DBG("sr1_value: 0x%02x", sr1_value);

		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_RDSR2,
			.command_length = 1,
			.address_length = 0,
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		sr2_value = transfer.data & 0xFF;
		LOG_DBG("sr2_value: 0x%02x", sr2_value);

		if (sr2_value & BIT(1)) {
			return 0;
		}

		ret = flash_ospi_b_write_enable(data);
		if (ret != 0) {
			return ret;
		}

		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_WRSR,
			.command_length = 1,
			.address_length = 0,
			.data = (uint32_t)sr1_value | ((uint32_t)(sr2_value | BIT(1)) << 8),
			.data_length = 2,
			.dummy_cycles = 0,
		};
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		break;

	case JESD216_DW15_QER_VAL_S1B6:
		/* QE = bit 6 of SR1; read with RDSR (0x05), write with WRSR (0x01) 1 byte */
		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_RDSR,
			.command_length = 1,
			.address_length = 0,
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		sr1_value = transfer.data & 0xFF;

		if (sr1_value & BIT(6)) {
			return 0;
		}

		ret = flash_ospi_b_write_enable(data);
		if (ret != 0) {
			return ret;
		}

		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_WRSR,
			.command_length = 1,
			.address_length = 0,
			.data = sr1_value | BIT(6),
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		break;

	case JESD216_DW15_QER_VAL_S2B7:
		/* QE = bit 7 of SR2; read with 0x3F, write with 0x3E 1 byte */
		transfer = (spi_flash_direct_transfer_t){
			.command = 0x3F,
			.command_length = 1,
			.address_length = 0,
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		sr2_value = transfer.data & 0xFF;

		if (sr2_value & BIT(7)) {
			return 0;
		}

		ret = flash_ospi_b_write_enable(data);
		if (ret != 0) {
			return ret;
		}

		transfer = (spi_flash_direct_transfer_t){
			.command = 0x3E,
			.command_length = 1,
			.address_length = 0,
			.data = sr2_value | BIT(7),
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		break;

	case JESD216_DW15_QER_VAL_S2B1v6:
		/* QE = bit 1 of SR2; read with RDSR2 (0x35), write with WRSR2 (0x31) 1 byte */
		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_RDSR2,
			.command_length = 1,
			.address_length = 0,
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		sr2_value = transfer.data & 0xFF;

		if (sr2_value & BIT(1)) {
			return 0;
		}

		ret = flash_ospi_b_write_enable(data);
		if (ret != 0) {
			return ret;
		}

		transfer = (spi_flash_direct_transfer_t){
			.command = SPI_NOR_CMD_WRSR2,
			.command_length = 1,
			.address_length = 0,
			.data = sr2_value | BIT(1),
			.data_length = 1,
			.dummy_cycles = 0,
		};
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		break;

	default:
		return -ENOTSUP;
	}

	return flash_ospi_b_wait_operation(p_ctrl, MAX_TIME_WRITE);
}

static int flash_ospi_b_qpi_enable(struct flash_renesas_ra_ospi_b_data *data,
				   struct jesd216_bfp_dw15 dw15)
{
	fsp_err_t err;
	int ret;
	spi_flash_direct_transfer_t transfer_enter_qpi = {
		.command_length = 1, .address_length = 0, .data_length = 0, .dummy_cycles = 0};

	if (dw15.enable_444 & BIT(0)) {
		ret = flash_ospi_b_quad_enable(data, dw15.qer);
		if (ret != 0) {
			LOG_DBG("QER enable failed");
			return ret;
		}

		transfer_enter_qpi.command = 0x38;
		err = R_OSPI_B_DirectTransfer(&data->ospi_b_ctrl, &transfer_enter_qpi,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Enter QPI mode failed");
			return -EIO;
		}
	} else if (dw15.enable_444 & BIT(1)) {
		transfer_enter_qpi.command = 0x38;
		err = R_OSPI_B_DirectTransfer(&data->ospi_b_ctrl, &transfer_enter_qpi,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Enter QPI mode failed");
			return -EIO;
		}
	} else if (dw15.enable_444 & BIT(2)) {
		transfer_enter_qpi.command = 0x35;
		err = R_OSPI_B_DirectTransfer(&data->ospi_b_ctrl, &transfer_enter_qpi,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Enter QPI mode failed");
			return -EIO;
		}
	} else {
		LOG_DBG("Not support this seq enable 444: %hhu", dw15.enable_444);
		return -ENOTSUP;
	}

	return 0;
}

static int flash_ospi_b_seq_8d_enable(ospi_b_instance_ctrl_t *p_ctrl, uint32_t *octal_8d_buf,
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
			/* Extract bytes starting from byte 2 based on cmd_len */
			for (uint8_t j = 2; j <= cmd_len; j++) {

				uint8_t byte_value = 0;

				/* Extract byte from appropriate word */
				if (j <= 3) {
					/* Bytes 2-3 from first word (octal_ddr_buf[i]) */
					byte_value = (octal_8d_buf[i] >> (24 - (j * 8))) & 0xFF;
				} else {
					/* Bytes 4 from second word (octal_ddr_buf[i+1]) */
					byte_value = JESD216_SFDP_SODP_BYTE4(octal_8d_buf[i + 1]);
				}

				/* Arrange byte in payload variable at correct position */
				payload = payload << 8 | (uint32_t)byte_value;
			}

			transfer_8d_enb[transfer_index].data = payload;
			transfer_8d_enb[transfer_index].data_length = cmd_len - 1;

		} else if (cmd_len == 5) {
			transfer_8d_enb[transfer_index].address_length = 3;
			transfer_8d_enb[transfer_index].address =
				JESD216_SFDP_SODP_BYTE2(octal_8d_buf[i]) << 16 |
				JESD216_SFDP_SODP_BYTE3(octal_8d_buf[i]) << 8 |
				JESD216_SFDP_SODP_BYTE4(octal_8d_buf[i + 1]);
			transfer_8d_enb[transfer_index].data =
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
				/* Extract bytes from appropriate word */
				transfer_8d_enb[transfer_index].data =
					(transfer_8d_enb[transfer_index].data << 8) |
					((octal_8d_buf[i + 1] >> (24 - (j - 4) * 8)) & 0xFF);
			}
			transfer_8d_enb[transfer_index].data_length = cmd_len - 5;
		}
	}

	for (uint8_t i = 0; i < (octal_8d_len / 2); i++) {
		err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer_8d_enb[i],
					      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Failed to enable octal ddr mode");
			return -EIO;
		}
		FLASH_OSPI_B_POLL_WAIT_US(50);
	}

	return 0;
}

static int flash_ospi_b_sccr_8d_enable(struct flash_renesas_ra_ospi_b_data *data,
				       uint32_t *sccr_buf, uint8_t sccr_len, uint8_t address_length)
{
	ospi_b_instance_ctrl_t *p_ctrl = &data->ospi_b_ctrl;
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

	/* Read opcode is bits 15:8 */
	transfer_8d_enb.command = (sccr_buf[21] >> 8) & 0xFF;
	transfer_8d_enb.command_length = 1;
	transfer_8d_enb.data_length = 1;
	transfer_8d_enb.dummy_cycles = 0;

	/* If bit 28 == 1 then this command uses an address (byte in DW1). Bit27
	 * indicates which byte within the DW1 address holds the volatile byte.
	 */
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

	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer_8d_enb, SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	mask_enb_octa = BIT((sccr_buf[16] >> 24) & 0x7);
	mask_enb_dtr_octa = BIT((sccr_buf[21] >> 24) & 0x7);

	/* Compute desired value for that bit according to polarity (bit30):
	 * If polarity bit == 1 (inverted) -> enable = 0, else enable = 1
	 */
	if (sccr_buf[21] & BIT(30)) {
		/* Clear the bit to enable */
		transfer_8d_enb.data &= ~mask_enb_dtr_octa;
	} else {
		/* Set the bit to enable */
		transfer_8d_enb.data |= mask_enb_dtr_octa;
	}

	if (mask_enb_octa != mask_enb_dtr_octa) {
		need_enb_octa = true;
	}

	/* Determine bit position (bits 26-24 indicate bit location in register) */
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
			/* Clear the bit to enable */
			transfer_8d_enb.data &= ~mask_enb_octa;
		} else {
			/* Set the bit to enable */
			transfer_8d_enb.data |= mask_enb_octa;
		}
	}

	/* Prepare write command and address info (bits 7:0 -> write opcode) */
	transfer_8d_enb.command = sccr_buf[21] & 0xFF;

	/* Issue write-enable then direct transfer */
	ret = flash_ospi_b_write_enable(data);
	if (ret != 0) {
		return ret;
	}

	err = R_OSPI_B_DirectTransfer(p_ctrl, &transfer_8d_enb,
				      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
	if (err != FSP_SUCCESS) {
		return -EIO;
	}

	return 0;
}

static int flash_ospi_b_update_flash_config(const struct device *dev)
{
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;
	struct flash_renesas_ra_ospi_b_data *data = dev->data;
	struct flash_sfdp_region *sfdp_region = &data->sfdp;
	/* SFDP param tables were pre-read into data struct by flash_ospi_b_nor_probe */
	struct jesd216_bfp *bfp = (struct jesd216_bfp *)sfdp_region->bfp_buf;
	struct jesd216_instr instr;
	struct jesd216_bfp_dw16 dw16;
	struct jesd216_bfp_dw14 dw14;
	struct jesd216_bfp_dw11 dw11;
	enum jesd216_mode_type target_mode;
	bsp_octaclk_settings_t octaclk_settings;
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
	ospi_b_table_t local_erase_table = {
		.p_table = local_erase_list,
		.length = JESD216_NUM_ERASE_TYPES,
	};
	ospi_b_xspi_command_set_t local_cmd_set = {
		.protocol = SPI_FLASH_PROTOCOL_EXTENDED_SPI,
		.frame_format = OSPI_B_FRAME_FORMAT_STANDARD,
		.command_bytes = OSPI_B_COMMAND_BYTES_1,
		.address_bytes = SPI_FLASH_ADDRESS_BYTES_3,
		.read_command = SPI_NOR_CMD_READ_FAST,
		.read_dummy_cycles = SPI_NOR_DUMMY_RD,
		.program_command = SPI_NOR_CMD_PP,
		.program_dummy_cycles = 0,
		.write_enable_command = SPI_NOR_CMD_WREN,
		.address_msb_mask = 0x00,
		.p_erase_commands = &local_erase_table,
	};

	/* Check supported mode; each case falls through to the next lower mode on failure */
	switch (config->desired_protocol) {
	case OSPI_OPI_MODE:
		if (jesd216_bfp_read_support(&sfdp_region->phdr[0], bfp, JESD216_MODE_8D8D8D,
					     &instr) == 0 &&
		    (sfdp_region->param_available & SFDP_PARAM_PROFILE_1V0_AVAILABLE) &&
		    ((sfdp_region->param_available & SFDP_PARAM_SEQ_OCTAL_DDR_AVAILABLE) ||
		     (sfdp_region->param_available & SFDP_PARAM_SCCR_MAP_AVAILABLE))) {
			target_mode = JESD216_MODE_8D8D8D;
			break;
		}
		LOG_DBG("Octal DDR mode can not be supported on this NOR flash");
		return -ENOTSUP;
	case OSPI_QUAD_MODE:
		if (jesd216_bfp_read_support(&sfdp_region->phdr[0], bfp, JESD216_MODE_444, &instr) >
		    0) {
			target_mode = JESD216_MODE_444;
			break;
		} else if (jesd216_bfp_read_support(&sfdp_region->phdr[0], bfp, JESD216_MODE_144,
						    &instr) > 0) {
			target_mode = JESD216_MODE_144;
			break;
		}
		LOG_DBG("Octal DDR mode can not be supported on this NOR flash");
		return -ENOTSUP;
	default:
		target_mode = JESD216_MODE_111;
		break;
	}

	for (int i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
		/* exp=0 means unsupported; exp>=32 would overflow uint32_t */
		if (erase_exp[i] == 0 || erase_exp[i] >= 32) {
			continue;
		}
		local_erase_list[i].command = erase_cmd[i];
		local_erase_list[i].size = BIT(erase_exp[i]);
	}

	/* Read DW11 to determine the page size */
	ret = jesd216_bfp_decode_dw11(&sfdp_region->phdr[0], bfp, &dw11);
	if (ret == 0 && dw11.page_size < 64) {
		data->ospi_b_cfg.page_size_bytes = dw11.page_size;
	}

	/* Read DW14 to determine the polling method we should use while programming */
	ret = jesd216_bfp_decode_dw14(&sfdp_region->phdr[0], bfp, &dw14);
	if (ret < 0) {
		/* Default to legacy polling mode */
		dw14.poll_options = 0x1;
	}

	if (dw14.poll_options & BIT(0)) {
		/* Read instruction used for polling is 0x05, BIT(0): 1=busy; 0=ready */
		local_cmd_set.status_command = SPI_NOR_CMD_RDSR;
		data->ospi_b_cfg.write_status_bit = 0;
	} else {
		/* Read instruction used for polling is 0x70, BIT(7): 0=busy; 1=ready */
		LOG_DBG("FSP not support status bit: 0=busy; 1=ready");
		return -ENOTSUP;
	}

	if (config->flash_size > MB(16)) {
		uint8_t init_addr_width =
			jesd216_bfp_addrbytes(bfp) == JESD216_SFDP_BFP_DW1_ADDRBYTES_VAL_4B ? 32
											    : 24;
		/* Configure 4 byte address width */
		if (init_addr_width == 24) {
			/* Check to see if we can enable 4 byte addressing */
			ret = jesd216_bfp_decode_dw16(&sfdp_region->phdr[0], bfp, &dw16);
			if (ret != 0) {
				LOG_DBG("Flash size > 16MB need enter 4 byte address mode but BFPT "
					"dw16 decode failed");
				return -EIO;
			}

			/* Attempt to enable 4 byte addressing */
			ret = flash_ospi_b_4byte_enable(
				data, dw16.enter_4ba, config->special_require_info.enter_4byte_cmd);
			if (ret != 0) {
				LOG_DBG("4 byte address support advertised but failed to "
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

			flash_ospi_b_parse_erase_4b(local_erase_list, sfdp_region->addr_4b_buf);
		} else {
			LOG_DBG("Flash size > 16MB but 4-byte address instruction table is not "
				"available");
			return -ENOTSUP;
		}
	} else {
		data->address_mode = 3;
		local_cmd_set.address_bytes = SPI_FLASH_ADDRESS_BYTES_3;
	}

	/* Sync erase list into local table for smpt check */
	for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
		data->erase_command_list[i] = local_erase_list[i];
	}

	/* Handle sector map parameter table */
	if (sfdp_region->param_available & SFDP_PARAM_SECTOR_MAP_AVAILABLE) {
		ret = flash_ospi_b_smpt_handle(data, sfdp_region->smpt_buf, sfdp_region->smp_len);
		if (ret < 0) {
			LOG_DBG("Failed to handle sector map parameter table");
		}
	} else {
		struct flash_region_info *regions = data->regions_layout.regions;

		regions->offset = 0;
		regions->size = config->flash_size;
		regions->erase_mask = 0;

		for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
			if (data->erase_command_list[i].command != 0 &&
			    data->erase_command_list[i].size != 0) {
				regions->erase_mask |= BIT(i);
			}
		}

		data->regions_layout.region_count = 1;

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
		ret = flash_ospi_b_update_page_layout(data);
		if (ret) {
			LOG_DBG("Failed to update page layout");
			return -EIO;
		}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
	}

	/* Just mode 8D need to use DS signal and need auto calibration */
	if (target_mode == JESD216_MODE_8D8D8D) {
		data->ospi_b_command_set.p_table = &local_cmd_set;

		/* Init SPI module mode to SPI mode to using memory map */
		err = R_OSPI_B_SpiProtocolSet(&data->ospi_b_ctrl, SPI_FLASH_PROTOCOL_EXTENDED_SPI);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Init SPI mode Failed");
			return -EIO;
		}

		if (bfp->dw10[8] & JESD216_SFDP_BFP_DW18_BYTE_ORDER_SWAPPED) {
			/* OSPI controller can not enable memory map at 1S-1S-1S */
			word_swapped = true;
			LOG_DBG("Byte order is swapped for this device, please aware that the data "
				"on NOR Flash is not correct when read or programed by other mode");
		}

		/* Setup calibrate data */
		ret = flash_ospi_b_setup_calibrate_data(data, config->start_address,
							config->flash_size, word_swapped);
		if (ret != 0) {
			LOG_DBG("Setup calibrate data Failed");
			return -EIO;
		}
	}

	/* Start other mode config from the resolved 1S state, then override */
	data->ospi_b_command_set_table[0] = local_cmd_set;
	data->ospi_b_command_set_table[0].p_erase_commands = &data->ospi_b_erase_commands;

	data->ospi_b_command_set_table[1] = local_cmd_set;
	data->ospi_b_command_set_table[1].p_erase_commands = &data->ospi_b_erase_commands;

	if (target_mode == JESD216_MODE_8D8D8D) {
		if (sfdp_region->param_available & SFDP_PARAM_SCCR_MAP_AVAILABLE) {
			uint8_t dummy;

			/* Attempt to enable DTR Octal (8D-8D-8D) mode using SCCR map */
			ret = flash_ospi_b_sccr_8d_enable(data, sfdp_region->sccr_buf,
							  sfdp_region->sccr_len,
							  data->address_mode);
			if (ret != 0) {
				LOG_DBG("Failed to enable DTR Octal mode using SCCR map");
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
				LOG_DBG("Can't find dummy cycles from Profile 1.0 table");
			}

			/* Round up to an even value to avoid tripping controllers up. */
			dummy = (uint8_t)ROUND_UP(dummy, 2);
			data->ospi_b_command_set_table[1].read_dummy_cycles = dummy;
		} else if (sfdp_region->param_available & SFDP_PARAM_SEQ_OCTAL_DDR_AVAILABLE) {
			/* Reset SPI NOR flash device by driving OM_RESET pin */
			data->ospi_b_ctrl.p_reg->LIOCTL_b.RSTCS0 = 0;
			FLASH_OSPI_B_POLL_WAIT_US(500);
			data->ospi_b_ctrl.p_reg->LIOCTL_b.RSTCS0 = 1;
			FLASH_OSPI_B_POLL_WAIT_US(50);

			ret = flash_ospi_b_seq_8d_enable(&data->ospi_b_ctrl,
							 sfdp_region->octal_ddr_buf,
							 sfdp_region->octal_ddr_len);
			if (ret != 0) {
				LOG_DBG("Failed to enable DTR Octal mode using Sequence table");
				return -EIO;
			}

			/* JESD216 define when set up using Sequence table */
			data->ospi_b_command_set_table[1].read_dummy_cycles = 20;
		} else {
			LOG_DBG("Target mode is 8D-8D-8D, but not SPI NOR Flash not support "
				"Sequence table or SCCR map to enable it");
			return -EIO;
		}

		data->ospi_b_command_set_table[1].protocol = SPI_FLASH_PROTOCOL_8D_8D_8D;
		data->address_mode = 4;
		data->ospi_b_command_set_table[1].address_bytes = SPI_FLASH_ADDRESS_BYTES_4;
		data->ospi_b_command_set_table[1].command_bytes = OSPI_B_COMMAND_BYTES_2;
		data->ospi_b_command_set_table[1].frame_format = OSPI_B_FRAME_FORMAT_XSPI_PROFILE_1;

		data->ospi_b_command_set_table[1].read_command =
			sfdp_region->pr1_buf[0] >> 8 & 0xFF;

		/* If 8D dummy cycles are base on Sequence table or profile 1 is not correct */
		if (config->special_require_info.def_8d_dummy_cycles != 0) {
			data->ospi_b_command_set_table[1].read_dummy_cycles =
				config->special_require_info.def_8d_dummy_cycles;
		}

		/* Although the PP_4B is not supported in 4-byte instruction table, so far the nor
		 * still use PP_4B + extend for octal mode
		 */
		data->ospi_b_command_set_table[1].program_command = SPI_NOR_CMD_PP_4B;
		data->ospi_b_command_set_table[1].program_dummy_cycles = 0;
		data->ospi_b_command_set_table[1].write_enable_command = SPI_NOR_CMD_WREN;
		/* the status command has been checked and set above */

		if (sfdp_region->pr1_buf[0] & JESD216_SFDP_PROFILE1_DW1_RDSR_DUMMY) {
			data->ospi_b_command_set_table[1].status_dummy_cycles = 8;
		} else {
			data->ospi_b_command_set_table[1].status_dummy_cycles = 4;
		}

		if ((config->special_require_info.rdsr_addr_4bytes) ||
		    (sfdp_region->pr1_buf[0] & JESD216_SFDP_PROFILE1_DW1_RDSR_ADDR_BYTES)) {
			data->ospi_b_command_set_table[1].status_needs_address = true;
			data->ospi_b_command_set_table[1].status_address_bytes =
				SPI_FLASH_ADDRESS_BYTES_4;
			data->ospi_b_command_set_table[1].status_address = 0;
		} else {
			data->ospi_b_command_set_table[1].status_needs_address = false;
		}

		/* 8D-8D-8D command extension. */
		switch (bfp->dw10[8] & JESD216_SFDP_BFP_DW18_CMD_EXT_MASK) {
		case JESD216_SFDP_BFP_DW18_CMD_EXT_REP:
			/* Handle repeat command extension */
			data->ospi_b_command_set_table[1].read_command =
				data->ospi_b_command_set_table[1].read_command << 8 |
				data->ospi_b_command_set_table[1].read_command;
			data->ospi_b_command_set_table[1].program_command =
				data->ospi_b_command_set_table[1].program_command << 8 |
				data->ospi_b_command_set_table[1].program_command;
			data->ospi_b_command_set_table[1].write_enable_command =
				data->ospi_b_command_set_table[1].write_enable_command << 8 |
				data->ospi_b_command_set_table[1].write_enable_command;
			data->ospi_b_command_set_table[1].status_command =
				data->ospi_b_command_set_table[1].status_command << 8 |
				data->ospi_b_command_set_table[1].status_command;

			for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
				if (data->erase_command_list[i].command != 0) {
					data->erase_command_list[i].command =
						data->erase_command_list[i].command << 8 |
						data->erase_command_list[i].command;
				}
			}
			break;
		case JESD216_SFDP_BFP_DW18_CMD_EXT_INV:
			/* Handle invert command extension */
			data->ospi_b_command_set_table[1].read_command =
				data->ospi_b_command_set_table[1].read_command << 8 |
				(~data->ospi_b_command_set_table[1].read_command & 0xFF);
			data->ospi_b_command_set_table[1].program_command =
				data->ospi_b_command_set_table[1].program_command << 8 |
				(~data->ospi_b_command_set_table[1].program_command & 0xFF);
			data->ospi_b_command_set_table[1].write_enable_command =
				data->ospi_b_command_set_table[1].write_enable_command << 8 |
				(~data->ospi_b_command_set_table[1].write_enable_command & 0xFF);
			data->ospi_b_command_set_table[1].status_command =
				data->ospi_b_command_set_table[1].status_command << 8 |
				(~data->ospi_b_command_set_table[1].status_command & 0xFF);

			for (uint8_t i = 0; i < JESD216_NUM_ERASE_TYPES; i++) {
				if (data->erase_command_list[i].command != 0) {
					data->erase_command_list[i].command =
						data->erase_command_list[i].command << 8 |
						(~data->erase_command_list[i].command & 0xFF);
				}
			}
			break;
		case JESD216_SFDP_BFP_DW18_CMD_EXT_16B:
			LOG_DBG("16-bit opcodes not supported");
			return -ENOTSUP;
		default:
			LOG_DBG("Unknown opcode extension type");
			return -EIO;
		}

		data->ospi_b_command_set.p_table = &data->ospi_b_command_set_table;

		octaclk_settings.source_clock = config->clock_src;
		octaclk_settings.divider = BSP_CFG_OCTACLK_DIV;
		R_BSP_OctaclkUpdate(&octaclk_settings);

		/* Switch OSPI module mode to OPI mode */
		err = R_OSPI_B_SpiProtocolSet(&data->ospi_b_ctrl, SPI_FLASH_PROTOCOL_8D_8D_8D);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Switch to OPI mode Failed");
			return -EIO;
		}
	} else if (target_mode == JESD216_MODE_444) {
		struct jesd216_bfp_dw15 dw15;

		ret = jesd216_bfp_decode_dw15(&sfdp_region->phdr[0], bfp, &dw15);
		if (ret != 0) {
			if (config->special_require_info.seq_enable_444 != 0) {
				dw15.enable_444 = config->special_require_info.seq_enable_444;
				dw15.qer = config->special_require_info.qer_value;
			} else {
				LOG_DBG("dw15 decode failed");
				return -EIO;
			}
		}

		ret = flash_ospi_b_qpi_enable(data, dw15);
		if (ret != 0) {
			LOG_DBG("QPI enable failed");
			return -EIO;
		}

		/* Configure command set for 4S-4S-4S (QPI) mode.
		 * Read command / program command are resolved from SFDP:
		 *   - 4-byte address: prefer dedicated 4B instructions from the
		 *     4-Byte Address Instruction table.
		 *   - 3-byte address: use the instruction returned by
		 *     jesd216_bfp_read_support(JESD216_MODE_444) and standard PP.
		 * read_dummy_cycles comes from instr.wait_states + instr.mode_clocks.
		 */
		data->ospi_b_command_set_table[1].protocol = SPI_FLASH_PROTOCOL_4S_4S_4S;
		data->ospi_b_command_set_table[1].frame_format = OSPI_B_FRAME_FORMAT_STANDARD;
		data->ospi_b_command_set_table[1].command_bytes = OSPI_B_COMMAND_BYTES_1;
		/* address_bytes already set from local_cmd_set (3B or 4B) */

		if (data->address_mode == 4 &&
		    (sfdp_region->param_available & SFDP_PARAM_4BYTE_ADDR_AVAILABLE)) {
			/* 0xEC (Fast Read Quad I/O 4B) is the same opcode in QPI mode */
			data->ospi_b_command_set_table[1].read_command =
				(sfdp_region->addr_4b_buf[0] &
				 JESD216_SFDP_4B_ADDR_DW1_1S_4S_4_FAST_READ_EC_SUP)
					? SPI_NOR_CMD_4READ_4B
					: instr.instr; /* TODO: Needs testing to confirm */
			/* PP4B (0x12) works in QPI mode on devices that support it in SPI */
			data->ospi_b_command_set_table[1].program_command =
				(sfdp_region->addr_4b_buf[0] &
				 JESD216_SFDP_4B_ADDR_DW1_1S_1S_1S_PP_12_SUP)
					? SPI_NOR_CMD_PP_4B
					: SPI_NOR_CMD_PP; /* TODO: Needs testing to confirm */
		} else {
			data->ospi_b_command_set_table[1].read_command = instr.instr;
			data->ospi_b_command_set_table[1].program_command = SPI_NOR_CMD_PP;
		}

		data->ospi_b_command_set_table[1].read_dummy_cycles =
			instr.wait_states + instr.mode_clocks;
		data->ospi_b_command_set_table[1].program_dummy_cycles = 0;
		data->ospi_b_command_set_table[1].status_dummy_cycles = 0;

		data->ospi_b_command_set.p_table = &data->ospi_b_command_set_table;

		octaclk_settings.source_clock = config->clock_src;
		octaclk_settings.divider = BSP_CFG_OCTACLK_DIV;
		R_BSP_OctaclkUpdate(&octaclk_settings);

		err = R_OSPI_B_SpiProtocolSet(&data->ospi_b_ctrl, SPI_FLASH_PROTOCOL_4S_4S_4S);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Switch to QPI mode failed");
			return -EIO;
		}

	} else if (target_mode == JESD216_MODE_144) {
		struct jesd216_bfp_dw15 dw15;

		ret = jesd216_bfp_decode_dw15(&sfdp_region->phdr[0], bfp, &dw15);
		if (ret != 0) {
			if (config->special_require_info.qer_value != 0) {
				dw15.qer = config->special_require_info.qer_value;
			} else {
				LOG_DBG("dw15 decode failed");
				return -EIO;
			}
		}

		ret = flash_ospi_b_quad_enable(data, dw15.qer);
		if (ret != 0) {
			LOG_DBG("Quad SPI enable failed");
			return -EIO;
		}

		/* Update command set for 1S-4S-4S mode.
		 * read_command / program_command are resolved from SFDP:
		 *   - 4-byte address: prefer dedicated 4B quad instructions from the
		 *     4-Byte Address Instruction table (DW1 bits 5 and 8).
		 *   - 3-byte address: use the instruction returned by
		 *     jesd216_bfp_read_support(JESD216_MODE_144) and the standard
		 *     1-4-4 page-program opcode.
		 * read_dummy_cycles comes from instr.wait_states decoded above.
		 * row_load_command / row_store_command have no NOR-flash equivalent
		 * in SFDP; set 0 to disable them in the FSP command set.
		 */
		data->ospi_b_command_set_table[1].protocol = SPI_FLASH_PROTOCOL_1S_4S_4S;
		data->ospi_b_command_set_table[1].frame_format = OSPI_B_FRAME_FORMAT_STANDARD;
		data->ospi_b_command_set_table[1].command_bytes = OSPI_B_COMMAND_BYTES_1;
		/* address_bytes already set from local_cmd_set (3B or 4B) */

		if (data->address_mode == 4 &&
		    (sfdp_region->param_available & SFDP_PARAM_4BYTE_ADDR_AVAILABLE)) {
			data->ospi_b_command_set_table[1].read_command =
				(sfdp_region->addr_4b_buf[0] &
				 JESD216_SFDP_4B_ADDR_DW1_1S_4S_4_FAST_READ_EC_SUP)
					? SPI_NOR_CMD_4READ_4B
					: SPI_NOR_CMD_READ_FAST_4B;
			data->ospi_b_command_set_table[1].program_command =
				(sfdp_region->addr_4b_buf[0] &
				 JESD216_SFDP_4B_ADDR_DW1_1S_4S_4S_PP_3E_SUP)
					? SPI_NOR_CMD_PP_1_4_4_4B
					: 0;
		} else {
			data->ospi_b_command_set_table[1].read_command = instr.instr;
			/* To-do: Add flow support 1-4-4 program command
			 * (not 1-1-4, since 1-1-4 is not supported by HW)
			 */
			data->ospi_b_command_set_table[1].program_command = 0;
		}

		data->ospi_b_command_set_table[1].read_dummy_cycles =
			instr.wait_states + instr.mode_clocks;
		data->ospi_b_command_set_table[1].program_dummy_cycles = 0;
		data->ospi_b_command_set_table[1].status_dummy_cycles = 0;

		data->ospi_b_command_set.p_table = &data->ospi_b_command_set_table;

		octaclk_settings.source_clock = config->clock_src;
		octaclk_settings.divider = BSP_CFG_OCTACLK_DIV;
		R_BSP_OctaclkUpdate(&octaclk_settings);

		err = R_OSPI_B_SpiProtocolSet(&data->ospi_b_ctrl, SPI_FLASH_PROTOCOL_1S_4S_4S);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Switch to Quad SPI mode Failed");
			return -EIO;
		}

	} else if (target_mode == JESD216_MODE_111) {
		data->ospi_b_command_set.p_table = &data->ospi_b_command_set_table;

		ret = flash_ospi_b_write_enable(data);
		if (ret != 0) {
			return ret;
		}

		spi_flash_direct_transfer_t transfer_write_sample = {.command = 0x02,
								     .command_length = 1,
								     .address = 0x8,
								     .address_length = 3,
								     .data = 0x87654321,
								     .data_length = 4,
								     .dummy_cycles = 0};

		err = R_OSPI_B_DirectTransfer(&data->ospi_b_ctrl, &transfer_write_sample,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_WRITE);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		ret = flash_ospi_b_wait_operation(&data->ospi_b_ctrl, MAX_TIME_WRITE);
		if (ret != 0) {
			return -EIO;
		}

		spi_flash_direct_transfer_t transfer_read = {.command = 0x0b,
							     .command_length = 1,
							     .address = 0x8,
							     .address_length = 3,
							     .data_length = 1,
							     .dummy_cycles = 8};

		err = R_OSPI_B_DirectTransfer(&data->ospi_b_ctrl, &transfer_read,
					      SPI_FLASH_DIRECT_TRANSFER_DIR_READ);
		if (err != FSP_SUCCESS) {
			return -EIO;
		}
		LOG_DBG("Data 1-1-1: 0x%02x", transfer_read.data);

		/* Init SPI module mode to SPI mode to using memory map */
		err = R_OSPI_B_SpiProtocolSet(&data->ospi_b_ctrl, SPI_FLASH_PROTOCOL_1S_1S_1S);
		if (err != FSP_SUCCESS) {
			LOG_DBG("Init SPI mode Failed");
			return -EIO;
		}
	}

	return 0;
}

/*
 * OSPI_B's register base isn't resolved into data->ospi_b_ctrl.p_reg until
 * R_OSPI_B_Open() runs (r_ospi_b.c: p_reg = R_XSPI0 + unit * (R_XSPI1 -
 * R_XSPI0)). Reproduce that arithmetic so flash_ospi_b_is_memory_mapped()
 * and the XIP handoff import path below can resolve it before/without ever
 * calling Open().
 */
static inline R_XSPI0_Type *flash_ospi_b_reg_from_unit(uint8_t unit)
{
	return (R_XSPI0_Type *)((uint32_t)R_XSPI0 + unit * ((uint32_t)R_XSPI1 - (uint32_t)R_XSPI0));
}

/*
 * Detect whether OSPI_B is already serving memory-mapped reads for this
 * instance's CS - i.e. MCUboot already opened and configured the controller
 * and jumped directly into this image while it executes XIP out of it.
 */
static bool flash_ospi_b_is_memory_mapped(const struct flash_renesas_ra_ospi_b_data *data)
{
	uint8_t unit = data->ospi_b_config_extend.ospi_b_unit;
	R_XSPI0_Type *p_reg;
	uint32_t access_mask;

	/*
	 * BMCTL0 (like every other OSPI_B register) only reflects real state
	 * once this unit's peripheral clock has been started - R_OSPI_B_Open()
	 * is what does that first, via R_BSP_MODULE_START(). Before that has
	 * ever run in this power cycle (a genuine cold boot), reading this
	 * unclocked peripheral's registers reads back as all-1s on this SoC,
	 * which would otherwise satisfy any nonzero BMCTL0 bitmask check below
	 * and produce a false "already memory-mapped" positive - observed in
	 * practice as MCUboot's own first-ever probe refusing to reprobe and
	 * failing device_is_ready(), aborting boot before the app ever runs.
	 * MSTPCRB bit (16 + unit) is FSP's own module-stop bit for FSP_IP_OSPI
	 * (see bsp_module_stop.h: BSP_MSTP_BIT_FSP_IP_OSPI); set means stopped.
	 */
	if (R_MSTP->MSTPCRB & (1U << (16U + unit))) {
		return false;
	}

	p_reg = flash_ospi_b_reg_from_unit(unit);
	access_mask = (data->ospi_b_config_extend.channel == 0)
			      ? (R_XSPI0_BMCTL0_CH0CS0ACC_Msk | R_XSPI0_BMCTL0_CH1CS0ACC_Msk)
			      : (R_XSPI0_BMCTL0_CH0CS1ACC_Msk | R_XSPI0_BMCTL0_CH1CS1ACC_Msk);

	return (p_reg->BMCTL0 & access_mask) == access_mask;
}

#define OSPI_B_ZEPHYR_XIP_HANDOFF_OPEN_SENTINEL (0x78535049U)

#define OSPI_B_XIP_HANDOFF_MAGIC 0x4f585048U /* "OXPH" */

/*
 * On-wire format for flash_ospi_b_xip_handoff_export()/_import(). Uses only
 * fixed-width/FSP-standard value types (no pointers) so a raw byte copy
 * across the MCUboot -> app image boundary is valid, since both images are
 * built from the same FSP headers and this same driver source.
 */
struct ospi_b_xip_handoff {
	uint32_t magic;
	uint32_t spi_protocol;
	uint32_t address_mode;
	uint32_t calib_sector_size;
	uint32_t region_count;
	spi_flash_erase_command_t erase_command_list[JESD216_NUM_ERASE_TYPES];
	ospi_b_xspi_command_set_t command_set;
	struct flash_region_info regions[DT_INST_PROP(0, max_regions)];
};

static int flash_ospi_b_xip_handoff_export(const struct device *dev, void *buf, size_t buf_size)
{
	struct flash_renesas_ra_ospi_b_data *data = dev->data;
	struct ospi_b_xip_handoff handoff = {0};

	if (buf_size < sizeof(handoff)) {
		return -EINVAL;
	}

	if (data->regions_layout.region_count == 0 ||
	    data->regions_layout.region_count > ARRAY_SIZE(handoff.regions)) {
		return -ENODEV;
	}

	handoff.magic = OSPI_B_XIP_HANDOFF_MAGIC;
	handoff.spi_protocol = (uint32_t)data->ospi_b_ctrl.spi_protocol;
	handoff.address_mode = data->address_mode;
	handoff.calib_sector_size = data->calib_sector_size;
	handoff.region_count = data->regions_layout.region_count;
	memcpy(handoff.erase_command_list, data->erase_command_list,
	       sizeof(handoff.erase_command_list));
	handoff.command_set = data->ospi_b_command_set_table[1];
	memcpy(handoff.regions, data->regions_layout.regions,
	       handoff.region_count * sizeof(handoff.regions[0]));

	memcpy(buf, &handoff, sizeof(handoff));

	return (int)sizeof(handoff);
}

/*
 * Counterpart to flash_ospi_b_xip_handoff_export(), used only by this same
 * driver on the application side. Populates data's ctrl bookkeeping and
 * SFDP-derived state from a previously exported blob without touching any
 * OSPI_B register or the flash device itself.
 */
static int flash_ospi_b_xip_handoff_import(struct flash_renesas_ra_ospi_b_data *data,
					   const void *buf, size_t len)
{
	struct ospi_b_xip_handoff handoff;

	if (len < sizeof(handoff)) {
		return -EINVAL;
	}
	memcpy(&handoff, buf, sizeof(handoff));

	if (handoff.magic != OSPI_B_XIP_HANDOFF_MAGIC) {
		return -EINVAL;
	}
	if (handoff.region_count == 0 || handoff.region_count > ARRAY_SIZE(handoff.regions)) {
		return -EINVAL;
	}

	data->ospi_b_ctrl.p_cfg = &data->ospi_b_cfg;
	data->ospi_b_ctrl.p_reg =
		flash_ospi_b_reg_from_unit(data->ospi_b_config_extend.ospi_b_unit);
	data->ospi_b_ctrl.channel = data->ospi_b_config_extend.channel;
	data->ospi_b_ctrl.ospi_b_unit = data->ospi_b_config_extend.ospi_b_unit;
	data->ospi_b_ctrl.spi_protocol = (spi_flash_protocol_t)handoff.spi_protocol;
	data->ospi_b_ctrl.open = OSPI_B_ZEPHYR_XIP_HANDOFF_OPEN_SENTINEL;

	data->address_mode = (uint8_t)handoff.address_mode;
	data->calib_sector_size = handoff.calib_sector_size;
	memcpy(data->erase_command_list, handoff.erase_command_list,
	       sizeof(data->erase_command_list));

	data->ospi_b_command_set_table[1] = handoff.command_set;
	/*
	 * p_erase_commands is a pointer into *this image's* data struct - the
	 * value carried over by the raw copy above points into the exporting
	 * image's own address space and must be re-pointed at our own erase
	 * table wrapper.
	 */
	data->ospi_b_command_set_table[1].p_erase_commands = &data->ospi_b_erase_commands;
	data->ospi_b_ctrl.p_cmd_set = &data->ospi_b_command_set_table[1];

	memcpy(data->regions_layout.regions, handoff.regions,
	       handoff.region_count * sizeof(handoff.regions[0]));
	data->regions_layout.region_count = handoff.region_count;

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	if (flash_ospi_b_update_page_layout(data) != 0) {
		return -EINVAL;
	}
#endif

	return 0;
}

static int flash_ospi_b_nor_probe(const struct device *dev, bool warm_attach)
{
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;
	struct flash_renesas_ra_ospi_b_data *data = dev->data;
	struct flash_sfdp_region *sfdp_region = &data->sfdp;
	uint8_t device_id[JESD216_READ_ID_LEN];
	fsp_err_t err;
	int ret;

	if (warm_attach) {
		/*
		 * OSPI_B is already open and memory-mapped - MCUboot configured it
		 * and jumped directly into this XIP image. Do NOT call
		 * R_OSPI_B_Open()/toggle RSTCS0/redo the SFDP probe
		 * Import the configuration MCUboot already derived
		 */
#if DT_HAS_CHOSEN(zephyr_ospi_xip_handoff)
		const struct device *retained_mem_dev =
			DEVICE_DT_GET(DT_PARENT(DT_CHOSEN(zephyr_ospi_xip_handoff)));
		off_t retained_mem_off = DT_REG_ADDR(DT_CHOSEN(zephyr_ospi_xip_handoff));
		uint8_t handoff_buf[sizeof(struct ospi_b_xip_handoff)];

		if (device_is_ready(retained_mem_dev) &&
		    retained_mem_read(retained_mem_dev, retained_mem_off, handoff_buf,
				      sizeof(handoff_buf)) == 0 &&
		    flash_ospi_b_xip_handoff_import(data, handoff_buf, sizeof(handoff_buf)) == 0) {
			LOG_INF("OSPI_B: adopted MCUboot's XIP configuration (warm handoff)");
			data->xip_warm_attach = true;
			return 0;
		}
#endif
		LOG_ERR("OSPI_B already memory-mapped but no valid MCUboot XIP handoff found, "
			"refusing to reprobe");
		return -ENODEV;
	}

	/* Open OSPI module */
	err = R_OSPI_B_Open(&data->ospi_b_ctrl, &data->ospi_b_cfg);
	if (err != FSP_SUCCESS) {
		LOG_DBG("OSPI open Failed");
		ret = -EIO;
		goto _exit;
	}

	/* Reset SPI NOR flash device by driving OM_RESET pin */
	data->ospi_b_ctrl.p_reg->LIOCTL_b.RSTCS0 = 0;
	FLASH_OSPI_B_POLL_WAIT_US(500);
	data->ospi_b_ctrl.p_reg->LIOCTL_b.RSTCS0 = 1;
	FLASH_OSPI_B_POLL_WAIT_US(100);

	/* Read SFDP header */
	ret = flash_ospi_b_sfdp_read_helper(&data->ospi_b_ctrl, &sfdp_region->header, 0,
					    sizeof(sfdp_region->header));
	if (ret != 0) {
		LOG_DBG("Read SFDP header Failed");
		goto _exit;
	}

	LOG_DBG("SFDP header magic: 0x%x", sfdp_region->header.magic);
	if (jesd216_sfdp_magic(&sfdp_region->header) != JESD216_SFDP_MAGIC) {
		if (config->desired_protocol == OSPI_QUAD_MODE) {
			/* The device may still be latched in QPI mode from a previous
			 * session; issue a software reset over 4S-4S-4S to bring it back
			 * to the default 1S-1S-1S protocol before retrying the SFDP read.
			 */
			ret = flash_ospi_b_software_reset(data);
			if (ret != 0) {
				LOG_DBG("Software reset Failed");
				goto _exit;
			}

			ret = flash_ospi_b_sfdp_read_helper(&data->ospi_b_ctrl,
							    &sfdp_region->header, 0,
							    sizeof(sfdp_region->header));
			if (ret != 0) {
				LOG_DBG("Read SFDP header Failed");
				goto _exit;
			}

			LOG_DBG("SFDP header magic: 0x%x", sfdp_region->header.magic);
			if (jesd216_sfdp_magic(&sfdp_region->header) != JESD216_SFDP_MAGIC) {
				/* Header was read incorrectly */
				LOG_DBG("Invalid header");
				ret = -EIO;
				goto _exit;
			}
		} else {
			/* Header was read incorrectly */
			LOG_ERR("Invalid header");
			ret = -EIO;
			goto _exit;
		}
	}

	/* Read JEDEC ID */
	ret = flash_ospi_b_jedec_id_read_helper(&data->ospi_b_ctrl, device_id);
	if (ret != 0) {
		LOG_DBG("Read JEDEC ID Failed");
		goto _exit;
	}

	if (memcmp(config->jedec_id, device_id, JESD216_READ_ID_LEN) != 0) {
		LOG_ERR("JEDEC ID mismatch: expected 0x%02x 0x%02x 0x%02x, "
			"got 0x%02x 0x%02x 0x%02x",
			config->jedec_id[0], config->jedec_id[1], config->jedec_id[2], device_id[0],
			device_id[1], device_id[2]);
		ret = -ENODEV;
		goto _exit;
	}

	/* Parameter headers are read directly into the fixed-size phdr[]
	 * array (see struct flash_sfdp_region) - no heap allocation, so
	 * probing stays safe to run while OSPI_B is out of memory-mapped
	 * mode. Fail loudly rather than silently truncate if a device
	 * reports more headers than this build was configured for.
	 */
	if ((size_t)(sfdp_region->header.nph + 1) > ARRAY_SIZE(sfdp_region->phdr)) {
		LOG_ERR("SFDP nph+1 (%u) exceeds the max-sfdp-param-headers DT property "
			"(%zu)",
			sfdp_region->header.nph + 1, ARRAY_SIZE(sfdp_region->phdr));
		ret = -ENOMEM;
		goto _exit;
	}

	/* Read parameter headers */
	ret = flash_ospi_b_sfdp_read_helper(
		&data->ospi_b_ctrl, sfdp_region->phdr, sizeof(struct jesd216_sfdp_header),
		(sfdp_region->header.nph + 1) * sizeof(struct jesd216_param_header));
	if (ret != 0) {
		LOG_DBG("Read SFDP header Failed");
		goto _exit;
	}

	if (sfdp_region->phdr[0].len_dw > ARRAY_SIZE(sfdp_region->bfp_buf)) {
		LOG_ERR("BFP table length %u dw exceeds fixed buffer (%zu dw)",
			sfdp_region->phdr[0].len_dw, ARRAY_SIZE(sfdp_region->bfp_buf));
		ret = -ENOMEM;
		goto _exit;
	}

	/* Read BFP (Basic Flash Parameters) */
	ret = flash_ospi_b_sfdp_read_helper(&data->ospi_b_ctrl, sfdp_region->bfp_buf,
					    jesd216_param_addr(&sfdp_region->phdr[0]),
					    sfdp_region->phdr[0].len_dw * sizeof(uint32_t));
	if (ret != 0) {
		LOG_DBG("Failed to read BFP table");
		goto _exit;
	}

	/* Read other parameter tables */
	for (uint8_t i = 1; i <= sfdp_region->header.nph; i++) {
		uint8_t len_dw = sfdp_region->phdr[i].len_dw;

		switch (jesd216_param_id(&sfdp_region->phdr[i])) {
		case JESD216_SFDP_PARAM_ID_SECTOR_MAP:
			if (len_dw > ARRAY_SIZE(sfdp_region->smpt_buf)) {
				LOG_ERR("Sector map length %u dw exceeds the smpt-max-dwords DT "
					"property (%zu)",
					len_dw, ARRAY_SIZE(sfdp_region->smpt_buf));
				break;
			}
			ret = flash_ospi_b_sfdp_read_helper(
				&data->ospi_b_ctrl, sfdp_region->smpt_buf,
				jesd216_param_addr(&sfdp_region->phdr[i]),
				len_dw * sizeof(uint32_t));
			if (ret < 0) {
				LOG_DBG("Failed to read sector map parameter table");
				break;
			}
			sfdp_region->smp_len = len_dw;
			sfdp_region->param_available |= SFDP_PARAM_SECTOR_MAP_AVAILABLE;
			break;
		case JESD216_SFDP_PARAM_ID_SEQ_OCTAL_DDR:
			if (len_dw > ARRAY_SIZE(sfdp_region->octal_ddr_buf)) {
				LOG_ERR("Octal DDR sequence table length %u dw exceeds fixed "
					"buffer (%zu dw)",
					len_dw, ARRAY_SIZE(sfdp_region->octal_ddr_buf));
				break;
			}
			ret = flash_ospi_b_sfdp_read_helper(
				&data->ospi_b_ctrl, sfdp_region->octal_ddr_buf,
				jesd216_param_addr(&sfdp_region->phdr[i]),
				len_dw * sizeof(uint32_t));
			if (ret != 0) {
				LOG_DBG("Read octal DDR sequences failed");
				break;
			}
			sfdp_region->octal_ddr_len = len_dw;
			sfdp_region->param_available |= SFDP_PARAM_SEQ_OCTAL_DDR_AVAILABLE;
			break;
		case JESD216_SFDP_PARAM_ID_SCCR_MAP:
			if (len_dw > ARRAY_SIZE(sfdp_region->sccr_buf)) {
				LOG_ERR("SCCR map length %u dw exceeds the sccr-max-dwords DT "
					"property (%zu dw)",
					len_dw, ARRAY_SIZE(sfdp_region->sccr_buf));
				break;
			}
			ret = flash_ospi_b_sfdp_read_helper(
				&data->ospi_b_ctrl, sfdp_region->sccr_buf,
				jesd216_param_addr(&sfdp_region->phdr[i]),
				len_dw * sizeof(uint32_t));
			if (ret != 0) {
				LOG_DBG("Failed to read SCCR map");
				break;
			}
			sfdp_region->sccr_len = len_dw;
			sfdp_region->param_available |= SFDP_PARAM_SCCR_MAP_AVAILABLE;
			break;
		case JESD216_SFDP_PARAM_ID_XSPI_PROFILE_1V0:
			if (len_dw > ARRAY_SIZE(sfdp_region->pr1_buf)) {
				LOG_ERR("xSPI Profile 1.0 table length %u dw exceeds fixed "
					"buffer (%zu dw)",
					len_dw, ARRAY_SIZE(sfdp_region->pr1_buf));
				break;
			}
			ret = flash_ospi_b_sfdp_read_helper(
				&data->ospi_b_ctrl, sfdp_region->pr1_buf,
				jesd216_param_addr(&sfdp_region->phdr[i]),
				len_dw * sizeof(uint32_t));
			if (ret != 0) {
				LOG_DBG("Failed to read xSPI Profile 1.0 table");
				break;
			}
			sfdp_region->pr1_len = len_dw;
			sfdp_region->param_available |= SFDP_PARAM_PROFILE_1V0_AVAILABLE;
			break;
		case JESD216_SFDP_PARAM_ID_4B_ADDR_INSTR:
			if (len_dw > ARRAY_SIZE(sfdp_region->addr_4b_buf)) {
				LOG_ERR("4-byte address instruction table length %u dw exceeds "
					"fixed buffer (%zu dw)",
					len_dw, ARRAY_SIZE(sfdp_region->addr_4b_buf));
				break;
			}
			ret = flash_ospi_b_sfdp_read_helper(
				&data->ospi_b_ctrl, sfdp_region->addr_4b_buf,
				jesd216_param_addr(&sfdp_region->phdr[i]),
				len_dw * sizeof(uint32_t));
			if (ret != 0) {
				LOG_DBG("Failed to read 4-byte address instruction table");
				break;
			}
			sfdp_region->addr_4b_len = len_dw;
			sfdp_region->param_available |= SFDP_PARAM_4BYTE_ADDR_AVAILABLE;
			break;
		default:
			break;
		}
	}

	/* Update flash configuration */
	ret = flash_ospi_b_update_flash_config(dev);

_exit:
	if ((ret != 0) && (data->ospi_b_ctrl.open)) {
		/* Since this is error case, don't care return value */
		R_OSPI_B_Close(&data->ospi_b_ctrl);
	}

	return ret;
}

static int flash_renesas_ra_ospi_b_init(const struct device *dev)
{
	const struct flash_renesas_ra_ospi_b_config *config = dev->config;
	struct flash_renesas_ra_ospi_b_data *data = dev->data;
	bsp_octaclk_settings_t octaclk_settings;
	uint32_t clock_freq;
	bool warm_attach = flash_ospi_b_is_memory_mapped(data);
	int ret;
#if defined(CONFIG_CODE_DATA_RELOCATION)
	unsigned int key = 0;
#endif

#if defined(CONFIG_CODE_DATA_RELOCATION)
	/*
	 * R_BSP_OctaclkUpdate() stops the OSPI_B module clock while it
	 * reprograms OctaCLK. If the application is executing out of this
	 * same OSPI flash, no ISR can be allowed to run in that window - its
	 * handler code may not be reachable. Not needed for warm_attach: that
	 * path (see below) never calls R_BSP_OctaclkUpdate() or touches any
	 * OSPI_B register at all, only copies already-derived state into RAM.
	 */
	if (!warm_attach) {
		key = irq_lock();
	}
#endif

	/* desired_protocol of OSPI checking */
	if (config->desired_protocol == OSPI_DUAL_MODE) {
		LOG_ERR("OSPI mode DUAL|QUAD currently not support");
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

	if ((config->max_frequency * 2) < clock_freq) {
		LOG_ERR("Invalid clock frequency (%u)", clock_freq);
		return -EINVAL;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to configure pins (%d)", ret);
		return ret;
	}

	k_sem_init(&data->sem, 1, 1);

	if (!warm_attach) {
		/* SFDP spec requires that we downclock the FlexSPI to 50MHz or less */
		if (clock_freq * config->clock_div > MHZ(800)) {
			LOG_ERR("clock source for OctaCLK is too high need <= 800MHz");
			return -EINVAL;
		}
		octaclk_settings.source_clock = config->clock_src;
		octaclk_settings.divider = BSP_CLOCKS_OCTACLK_DIV_8;

		/*
		 * Stops the OSPI_B module clock while reprogramming OctaCLK - see
		 * the irq_lock() comment above. MCUboot already established the
		 * correct OctaCLK configuration; redoing it here on the
		 * warm_attach path would needlessly repeat that hazard for no
		 * benefit, so it's skipped entirely above.
		 */
		R_BSP_OctaclkUpdate(&octaclk_settings);
	}

	ret = flash_ospi_b_nor_probe(dev, warm_attach);
	if (ret) {
		LOG_ERR("NOR probe failed (%d)", ret);
		return ret;
	}

#if defined(CONFIG_CODE_DATA_RELOCATION)
	if (!warm_attach) {
		irq_unlock(key);
	}
#endif
	return ret;
}

#define DT_INST_QER_PROP_OR(inst)                                                                  \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, quad_enable_requirements),	                   \
		    (_CONCAT(JESD216_DW15_QER_VAL_,				                   \
		     DT_INST_STRING_TOKEN(inst, quad_enable_requirements))),	                   \
		    ((JESD216_DW15_QER_VAL_NONE)))

#define OSPI_B_RENESAS_RA_INIT(idx)                                                                \
	PINCTRL_DT_DEFINE(DT_INST_PARENT(idx));                                                    \
	static const struct flash_renesas_ra_ospi_b_config ospi_b_config_##idx = {                 \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_INST_PARENT(idx)),                            \
		.start_address = DT_RANGES_PARENT_BUS_ADDRESS_BY_IDX(DT_DRV_INST(idx), 0),         \
		.flash_size = DT_REG_SIZE(DT_INST_CHILD(idx, nor_flash_0)),                        \
		.desired_protocol = (uint8_t)DT_INST_PROP(idx, protocol_mode),                     \
		.max_frequency = DT_INST_PROP(idx, ospi_max_frequency),                            \
		.erase_block_size = DT_PROP(DT_INST_CHILD(idx, nor_flash_0), erase_block_size),    \
		.jedec_id = DT_INST_PROP(idx, jedec_id),                                           \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(idx))),                   \
		.clock_src = BSP_CFG_OCTACLK_SOURCE,                                               \
		.clock_div = DT_PROP(DT_CLOCKS_CTLR(DT_INST_PARENT(idx)), div),                    \
		.special_require_info =                                                            \
			{                                                                          \
				.enter_4byte_cmd =                                                 \
					(uint16_t)DT_INST_PROP_OR(idx, enter_4byte_command, 0),    \
				.def_8d_dummy_cycles =                                             \
					(uint8_t)DT_INST_PROP_OR(idx, default_8d_dummy_cycles, 0), \
				.rdsr_addr_4bytes = DT_INST_PROP_OR(idx, rdsr_addr_4bytes, false), \
				.qer_value = DT_INST_QER_PROP_OR(idx),                             \
				.seq_enable_444 =                                                  \
					(uint8_t)DT_INST_PROP_OR(idx, seq_enable_qpi, 0),          \
			},                                                                         \
		.clock_subsys =                                                                    \
			{                                                                          \
				.mstp = (uint32_t)DT_CLOCKS_CELL(DT_INST_PARENT(idx), mstp),       \
				.stop_bit =                                                        \
					(uint32_t)DT_CLOCKS_CELL(DT_INST_PARENT(idx), stop_bit),   \
			},                                                                         \
		.flash_parameters =                                                                \
			{                                                                          \
				.write_block_size = DT_PROP(DT_INST_CHILD(idx, nor_flash_0),       \
							    write_block_size),                     \
				.erase_value = 0xff,                                               \
			},                                                                         \
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
				.sdr_sampling_edge = OSPI_B_CK_EDGE_RISING,                        \
				.sdr_sampling_delay = OSPI_B_SDR_SAMPLING_DELAY_NONE,              \
				.ddr_sampling_extension =                                          \
					DT_PROP(DT_INST_PARENT(idx), ddr_sampling_extension),      \
			},                                                                         \
		.ospi_b_erase_commands =                                                           \
			{                                                                          \
				.p_table = &ospi_b_data_##idx.erase_command_list,                  \
				.length = JESD216_NUM_ERASE_TYPES,                                 \
			},                                                                         \
		.ospi_b_command_set_table =                                                        \
			{                                                                          \
				{                                                                  \
					.protocol = SPI_FLASH_PROTOCOL_1S_1S_1S,                   \
					.frame_format = OSPI_B_FRAME_FORMAT_STANDARD,              \
					.latency_mode = OSPI_B_LATENCY_MODE_FIXED,                 \
					.command_bytes = OSPI_B_COMMAND_BYTES_1,                   \
					.address_bytes = SPI_FLASH_ADDRESS_BYTES_3,                \
					.address_msb_mask = 0x00,                                  \
					.status_needs_address = false,                             \
					.p_erase_commands =                                        \
						&ospi_b_data_##idx.ospi_b_erase_commands,          \
					.read_command = 0xb,                                       \
					.read_dummy_cycles = 8,                                    \
					.program_command = 0x2,                                    \
					.program_dummy_cycles = 0,                                 \
					.write_enable_command = SPI_NOR_CMD_WREN,                  \
					.status_command = SPI_NOR_CMD_RDSR,                        \
					.status_dummy_cycles = 0,                                  \
				},                                                                 \
				{                                                                  \
					.frame_format = OSPI_B_FRAME_FORMAT_STANDARD,              \
					.latency_mode = OSPI_B_LATENCY_MODE_FIXED,                 \
					.command_bytes = OSPI_B_COMMAND_BYTES_1,                   \
					.address_bytes = SPI_FLASH_ADDRESS_BYTES_3,                \
					.status_needs_address = false,                             \
					.address_msb_mask = 0x00,                                  \
					.p_erase_commands =                                        \
						&ospi_b_data_##idx.ospi_b_erase_commands,          \
				},                                                                 \
			},                                                                         \
		.ospi_b_command_set =                                                              \
			{                                                                          \
				.p_table = (ospi_b_xspi_command_set_t *)&ospi_b_data_##idx         \
						   .ospi_b_command_set_table,                      \
				.length = 2,                                                       \
			},                                                                         \
                                                                                                   \
		.ospi_b_config_extend =                                                            \
			{                                                                          \
				.ospi_b_unit = DT_PROP(DT_INST_PARENT(idx), unit),                 \
				.channel = DT_INST_REG_ADDR(idx),                                  \
				.data_latch_delay_clocks = OSPI_B_DS_TIMING_DELAY_NONE,            \
				.p_timing_settings = &ospi_b_data_##idx.ospi_b_timing_settings,    \
				.p_xspi_command_set = &ospi_b_data_##idx.ospi_b_command_set,       \
			},                                                                         \
		.ospi_b_cfg =                                                                      \
			{                                                                          \
				.spi_protocol = SPI_FLASH_PROTOCOL_1S_1S_1S,                       \
				.read_mode = SPI_FLASH_READ_MODE_FAST_READ,                        \
				.address_bytes = SPI_FLASH_ADDRESS_BYTES_3,                        \
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

#if DT_HAS_CHOSEN(zephyr_ospi_xip_handoff)
/*
 * Publish this OSPI_B instance's SFDP-derived XIP configuration for a
 * follow-on image to adopt (see flash_ospi_b_xip_handoff_import() above),
 * so it can skip the OM_RESET pin toggle + SFDP reprobe that
 * flash_ospi_b_nor_probe() just performed. On this board's flash part
 * (JESD216_SFDP_BFP_DW18_BYTE_ORDER_SWAPPED), redoing that reprobe while an
 * image depends on this same flash for XIP returns corrupted bytes for its
 * own code/data - see flash_ospi_b_is_memory_mapped() and its callers.
 *
 * Runs at APPLICATION init level, i.e. after every POST_KERNEL device -
 * including this one and the retention device below - is ready, in
 * whichever image runs first (typically MCUboot). No explicit call from
 * that image's own boot flow is needed: a later image that finds OSPI_B
 * already memory-mapped takes the import path in flash_ospi_b_nor_probe()
 * instead and skips this publish step entirely (data->xip_warm_attach).
 */
static int flash_ospi_b_xip_handoff_publish(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
	const struct device *retention_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_ospi_xip_handoff));
	struct flash_renesas_ra_ospi_b_data *data;
	uint8_t handoff_buf[sizeof(struct ospi_b_xip_handoff)];
	ssize_t area_size;
	int handoff_len;
	int ret;

	if (!device_is_ready(dev)) {
		return 0;
	}

	data = dev->data;
	if (data->xip_warm_attach) {
		/* We imported this configuration ourselves; nothing new to publish. */
		return 0;
	}

	if (!device_is_ready(retention_dev)) {
		return 0;
	}

	area_size = retention_size(retention_dev);
	if (area_size <= 0 || (size_t)area_size < sizeof(handoff_buf)) {
		return 0;
	}

	handoff_len = flash_ospi_b_xip_handoff_export(dev, handoff_buf, sizeof(handoff_buf));
	if (handoff_len <= 0) {
		LOG_ERR("Failed to export OSPI XIP handoff data (%d)", handoff_len);
		return 0;
	}

	ret = retention_write(retention_dev, 0, handoff_buf, (size_t)handoff_len);
	if (ret != 0) {
		LOG_ERR("Failed to write OSPI XIP handoff data");
	}

	return 0;
}

SYS_INIT(flash_ospi_b_xip_handoff_publish, APPLICATION, 0);
#endif /* DT_HAS_CHOSEN(zephyr_ospi_xip_handoff) */
