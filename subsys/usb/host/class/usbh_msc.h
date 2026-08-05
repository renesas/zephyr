/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_USB_HOST_CLASS_USBH_MSC_H_
#define ZEPHYR_SUBSYS_USB_HOST_CLASS_USBH_MSC_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/usb_ch9.h>

/*
 * "USB<driver_index>_<lun>", sized for the worst case of both being full-width uint8_t
 * values (up to 3 digits each), plus the NUL terminator.
 */
#define DISK_NAME_LENGTH sizeof("USB255_255")

/* Unit state; whether initialization was successful, if the medium is actually connected */
enum usbh_msc_unit_state {
	UNIT_STATE_NOT_READY,
	UNIT_STATE_READY,
	UNIT_STATE_NO_MEDIUM,
	UNIT_STATE_ERROR,
};

/* Data structure for each logical unit */
struct lun_data {
	/* Pointer back to the original driver data */
	struct driver_data *driver_data;
	/* Logical unit index */
	uint8_t index;
	/* Mountpoint for the Disk Access API */
	char disk_name[DISK_NAME_LENGTH];
	/* Disk access structure */
	struct disk_info disk;
	/* Wether the drive is read-only or writeable */
	bool write_protect;
	/* Address of the last block on the drive */
	uint32_t last_logical_block_address;
	/* Block size */
	uint32_t block_length_in_bytes;
	/* Unit state; whether initialization was successful, if the medium is actually connected
	 */
	enum usbh_msc_unit_state unit_state;
};

struct driver_data {
	/* Connected usb device */
	struct usb_device *udev;
	/* Mutual exclusion lock */
	struct k_mutex lock;
	/* Sync semaphore for waiting on asynchronous operations */
	struct k_sem sync;
	/* Bulk IN endpoint address */
	const struct usb_ep_descriptor *in_bulk_ep;
	/* Bulk OUT endpoint address */
	const struct usb_ep_descriptor *out_bulk_ep;
	/* Index of the last logical unit */
	uint8_t max_logical_unit;
	/* Information about each logical unit on the drive */
	struct lun_data lun_data[CONFIG_USBH_MSC_MAX_SUPPORTED_LUN];
	/* Index of the target interface */
	uint8_t target_iface;
	/* Tag given back as is to identify the response */
	uint32_t tag;
};

/*
 * Convert the unit state to an errno code.
 */
static inline int unit_state_to_errno(struct lun_data *lun_data)
{
	switch (lun_data->unit_state) {
	case UNIT_STATE_READY: {
		return 0;
	}
	case UNIT_STATE_NO_MEDIUM: {
		return -ENOMEDIUM;
	}
	default: {
		return -EIO;
	}
	}
}

/*
 * SCSI/BOT transport operations, implemented in usbh_msc.c and used by the disk access
 * glue in drivers/disk/usbh_disk.c.
 */

int usbh_msc_read_blocks(struct driver_data *driver_data, uint8_t lun_index, uint32_t lba,
			 uint16_t block_count, uint8_t *buffer);

int usbh_msc_write_blocks(struct driver_data *driver_data, uint8_t lun_index, uint32_t lba,
			  uint16_t block_count, uint8_t const *buffer);

int usbh_msc_initialize_unit(struct driver_data *driver_data, uint8_t lun_index);

#ifndef CONFIG_USBH_MSC_IGNORE_SYNC
int usbh_msc_synchronize_cache(struct driver_data *driver_data, uint8_t lun_index);
#endif /* CONFIG_USBH_MSC_IGNORE_SYNC */

/*
 * Disk access glue, implemented in drivers/disk/usbh_disk.c and used by usbh_msc.c to
 * (un)register each logical unit as an accessible disk.
 */

int usbh_msc_disk_register(struct driver_data *driver_data, const struct device *dev,
			   uint8_t driver_index, uint8_t lun_index);

void usbh_msc_disk_unregister(struct lun_data *lun_data);

#endif /* ZEPHYR_SUBSYS_USB_HOST_CLASS_USBH_MSC_H_ */
