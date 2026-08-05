/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/logging/log.h>

#include "usbh_msc.h"

LOG_MODULE_DECLARE(usbh_msc, CONFIG_USBH_MSC_LOG_LEVEL);

/*
 * Retrieve the containing `struct lun_data` from a `disk` field pointer.
 */
static inline struct lun_data *get_lun_data_from_disk(struct disk_info *disk)
{
	return CONTAINER_OF(disk, struct lun_data, disk);
}

static int disk_access_status(struct disk_info *disk)
{
	struct lun_data *lun_data = get_lun_data_from_disk(disk);
	struct driver_data *driver_data = lun_data->driver_data;
	int result = 0;

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	switch (lun_data->unit_state) {
	case UNIT_STATE_NO_MEDIUM: {
		result = DISK_STATUS_NOMEDIA;
		break;
	}
	case UNIT_STATE_READY: {
		if (lun_data->write_protect) {
			result = DISK_STATUS_WR_PROTECT;
		}
		/* If the disk is registered the drive has been probed and it's ready */
		else {
			result = DISK_STATUS_OK;
		}
		break;
	}
	default: {
		result = DISK_STATUS_UNINIT;
		break;
	}
	}

	k_mutex_unlock(&driver_data->lock);

	return result;
}

static int disk_access_read(struct disk_info *disk, uint8_t *data_buf, uint32_t start_sector,
			    uint32_t num_sector)
{
	struct lun_data *lun_data = get_lun_data_from_disk(disk);
	struct driver_data *driver_data = lun_data->driver_data;
	int result = 0;

	k_mutex_lock(&driver_data->lock, K_FOREVER);
	if (start_sector + num_sector > lun_data->last_logical_block_address + 1) {
		k_mutex_unlock(&driver_data->lock);
		return -EINVAL;
	}

	if (lun_data->unit_state == UNIT_STATE_READY) {
		result = usbh_msc_read_blocks(driver_data, lun_data->index, start_sector,
					      num_sector, data_buf);
	} else {
		result = unit_state_to_errno(lun_data);
	}
	k_mutex_unlock(&driver_data->lock);

	return result;
}

static int disk_access_write(struct disk_info *disk, const uint8_t *data_buf, uint32_t start_sector,
			     uint32_t num_sector)
{
	struct lun_data *lun_data = get_lun_data_from_disk(disk);
	struct driver_data *driver_data = lun_data->driver_data;
	int result = 0;

	k_mutex_lock(&driver_data->lock, K_FOREVER);
	if (start_sector + num_sector > lun_data->last_logical_block_address + 1) {
		k_mutex_unlock(&driver_data->lock);
		return -EINVAL;
	}

	if (lun_data->unit_state == UNIT_STATE_READY) {
		result = usbh_msc_write_blocks(driver_data, lun_data->index, start_sector,
					       num_sector, data_buf);
	} else {
		result = unit_state_to_errno(lun_data);
	}
	k_mutex_unlock(&driver_data->lock);

	return result;
}

static int disk_access_erase(struct disk_info *disk, uint32_t start_sector, uint32_t num_sector)
{
	ARG_UNUSED(disk);
	ARG_UNUSED(start_sector);
	ARG_UNUSED(num_sector);
	/* Erasing doesn't make sense in the context of USB drives, it should be handled by
	 * the device's own firmware */
	return -ENOTSUP;
}

static int disk_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buff)
{
	struct lun_data *lun_data = get_lun_data_from_disk(disk);
	struct driver_data *driver_data = lun_data->driver_data;
	int result = 0;

	k_mutex_lock(&driver_data->lock, K_FOREVER);
	switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT: {
		if (lun_data->unit_state == UNIT_STATE_READY) {
			*(uint32_t *)buff = lun_data->last_logical_block_address + 1;
		} else {
			result = unit_state_to_errno(lun_data);
		}
		break;
	}
	case DISK_IOCTL_GET_SECTOR_SIZE: {
		if (lun_data->unit_state == UNIT_STATE_READY) {
			*(uint32_t *)buff = lun_data->block_length_in_bytes;
		} else {
			result = unit_state_to_errno(lun_data);
		}
		break;
	}
	case DISK_IOCTL_CTRL_SYNC: {
		if (lun_data->unit_state == UNIT_STATE_READY) {
#ifndef CONFIG_USBH_MSC_IGNORE_SYNC
			result = usbh_msc_synchronize_cache(driver_data, lun_data->index);
#endif /* CONFIG_USBH_MSC_IGNORE_SYNC */
		} else {
			result = unit_state_to_errno(lun_data);
		}
		break;
	}
	case DISK_IOCTL_CTRL_DEINIT: {
		break;
	}
	case DISK_IOCTL_CTRL_INIT: {
		LOG_DBG("Init IOCTL");
		switch (lun_data->unit_state) {
		case UNIT_STATE_READY: {
			/* Unit is ready and responsive */
			break;
		}
		default: {
			/* Unit was not ready; retry */
			result = usbh_msc_initialize_unit(driver_data, lun_data->index);
			break;
		}
		}
		break;
	}
	default: {
		result = -ENOTSUP;
		break;
	}
	}
	k_mutex_unlock(&driver_data->lock);

	return result;
}

static int disk_access_init(struct disk_info *disk)
{
	return disk_access_ioctl(disk, DISK_IOCTL_CTRL_INIT, NULL);
}

/*
 * Disk access vtable
 */
static const struct disk_operations disk_operations = {
	.init = disk_access_init,
	.status = disk_access_status,
	.read = disk_access_read,
	.write = disk_access_write,
	.erase = disk_access_erase,
	.ioctl = disk_access_ioctl,
};

int usbh_msc_disk_register(struct driver_data *driver_data, const struct device *dev,
			   uint8_t driver_index, uint8_t lun_index)
{
	struct lun_data *lun_data = &driver_data->lun_data[lun_index];

	lun_data->disk.dev = dev;
	lun_data->disk.ops = &disk_operations;
	snprintf(lun_data->disk_name, sizeof(lun_data->disk_name), "USB%i_%u", driver_index,
		 lun_index);
	lun_data->disk.name = lun_data->disk_name;
	lun_data->index = lun_index;
	lun_data->driver_data = driver_data;

	return disk_access_register(&lun_data->disk);
}

void usbh_msc_disk_unregister(struct lun_data *lun_data)
{
	disk_access_unregister(&lun_data->disk);
}
