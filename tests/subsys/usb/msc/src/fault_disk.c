/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/storage/disk_access.h>

#include "fault_disk.h"

#define FAULT_DISK_SECTOR_SIZE  512u
#define FAULT_DISK_SECTOR_COUNT 64u

/*
 * `USBD_DEFINE_MSC_LUN(id, ...)` places its LUN in the `usbd_msc_lun` iterable section, which
 * the linker sorts alphabetically by the resulting `usbd_msc_lun_##id` symbol name (SORT_BY_NAME,
 * see include/zephyr/linker/iterable_sections.h) -- NOT by declaration/link order. This id is
 * chosen to sort after ram_disk.c's "ram0"/"ram1" so this LUN lands last (index CONFIG_TEST_NUM_LUN
 * - 1, i.e. "USB0_2"), matching DISK_DRIVE_NAME_FAULT in msc.c.
 */
USBD_DEFINE_MSC_LUN(zzz_fault, FAULT_DISK_NAME, "Zephyr", "FaultDisk", "0.00");

static uint8_t backing[FAULT_DISK_SECTOR_COUNT][FAULT_DISK_SECTOR_SIZE];

/* -1: report the real status; otherwise the DISK_STATUS_* value to report instead */
static int status_override = -1;
static bool read_should_fail;
static bool write_should_fail;

static int fault_disk_init(struct disk_info *disk)
{
	ARG_UNUSED(disk);

	return 0;
}

static int fault_disk_status(struct disk_info *disk)
{
	ARG_UNUSED(disk);

	return status_override >= 0 ? status_override : DISK_STATUS_OK;
}

static int fault_disk_read(struct disk_info *disk, uint8_t *data_buf, uint32_t start_sector,
			   uint32_t num_sector)
{
	ARG_UNUSED(disk);

	if (read_should_fail) {
		return -EIO;
	}

	if (start_sector + num_sector > FAULT_DISK_SECTOR_COUNT) {
		return -EINVAL;
	}

	memcpy(data_buf, backing[start_sector], (size_t)num_sector * FAULT_DISK_SECTOR_SIZE);

	return 0;
}

static int fault_disk_write(struct disk_info *disk, const uint8_t *data_buf, uint32_t start_sector,
			    uint32_t num_sector)
{
	ARG_UNUSED(disk);

	if (write_should_fail) {
		return -EIO;
	}

	if (start_sector + num_sector > FAULT_DISK_SECTOR_COUNT) {
		return -EINVAL;
	}

	memcpy(backing[start_sector], data_buf, (size_t)num_sector * FAULT_DISK_SECTOR_SIZE);

	return 0;
}

static int fault_disk_ioctl(struct disk_info *disk, uint8_t cmd, void *buff)
{
	ARG_UNUSED(disk);

	switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT:
		*(uint32_t *)buff = FAULT_DISK_SECTOR_COUNT;
		return 0;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		*(uint32_t *)buff = FAULT_DISK_SECTOR_SIZE;
		return 0;
	case DISK_IOCTL_CTRL_SYNC:
	case DISK_IOCTL_CTRL_INIT:
	case DISK_IOCTL_CTRL_DEINIT:
		return 0;
	default:
		return -ENOTSUP;
	}
}

static const struct disk_operations fault_disk_ops = {
	.init = fault_disk_init,
	.status = fault_disk_status,
	.read = fault_disk_read,
	.write = fault_disk_write,
	.ioctl = fault_disk_ioctl,
};

static struct disk_info fault_disk_info = {
	.name = FAULT_DISK_NAME,
	.ops = &fault_disk_ops,
};

void fault_disk_setup(void)
{
	disk_access_register(&fault_disk_info);
}

void fault_disk_reset(void)
{
	status_override = -1;
	read_should_fail = false;
	write_should_fail = false;
}

void fault_disk_set_status_override(int status)
{
	status_override = status;
}

void fault_disk_set_read_error(bool fail)
{
	read_should_fail = fail;
}

void fault_disk_set_write_error(bool fail)
{
	write_should_fail = fail;
}
