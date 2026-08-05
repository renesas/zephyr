/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdio.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "msc.h"
#include "usbh_class.h"
#include "usbh_desc.h"
#include "usbh_ch9.h"
#include "usbh_device.h"
#include "usbh_msc.h"

LOG_MODULE_REGISTER(usbh_msc, CONFIG_USBH_MSC_LOG_LEVEL);

/* Size of a Command Block Wrapper */
#define CBW_SIZE                              31u
/* Size of a Command Status Wrapper */
#define CSW_SIZE                              13u
/* Maximum size for a CBW */
#define CBW_COMMAND_BLOCK_MAX_LENGTH          16u
/* Timeout for any SCSI request */
#define SCSI_REQ_TIMEOUT                      5000u
/* Max sense data returned by a Request Sense command */
#define SCSI_MAX_SENSE_DATA                   18u
/* Code for the current sense data */
#define SCSI_SENSE_DATA_RESPONSE_CODE_CURRENT 0x70u

/* From SPC table 49 */
#define SCSI_SENSE_DATA_KEY_NOT_READY       0x02u
#define SCSI_SENSE_DATA_KEY_MEDIUM_ERROR    0x03u
#define SCSI_SENSE_DATA_KEY_ILLEGAL_REQUEST 0x05u
#define SCSI_SENSE_DATA_KEY_UNIT_ATTENTION  0x06u

/* From SPC table 50 */
#define SCSI_SENSE_DATA_ASC_INVALID_COMMAND_OPERATION_CODE 0x20u
#define SCSI_SENSE_DATA_ASC_NOT_READY_TO_READY             0x28u
#define SCSI_SENSE_DATA_ASC_MEDIUM_NOT_PRESENT             0x3Au

#define SCSI_READ_CAPACITY_10_DATA_LENGTH 8u
#define SCSI_MODE_SENSE_DATA_LENGTH       8u

/* Data direction of an SCSI transaction */
enum scsi_direction {
	SCSI_DIRECTION_DATA_OUT = 0u,
	SCSI_DIRECTION_DATA_IN = 1u,
};

/* SCSI Command Block Wrapper */
struct scsi_cbw {
	/* Target Logical UNit index */
	uint8_t lun;
	/* Transaction direction */
	enum scsi_direction direction;
	/* Size of the attached command block data */
	size_t command_block_length;
	/* Command block data */
	uint8_t const *command_block;
	/* Data phase transfer length */
	size_t data_transfer_length;
};

/* Status code of an SCSI transaction as returned by the Sense Data */
enum scsi_status {
	/* OK */
	SCSI_STATUS_COMMAND_PASSED = 0x00u,
	/* The device is reporting an error */
	SCSI_STATUS_COMMAND_FAILED = 0x01u,
	/* There was an issue during communication */
	SCSI_STATUS_PHASE_ERROR = 0x02u,
};

/* SCSI Command Status Wrapper */
struct scsi_csw {
	/* Tag for the corresponding CBW */
	uint32_t tag;
	/* The amount of unprocessed data (depending on the direction of the transaction), if any */
	uint32_t data_residue;
	/* Returned status code */
	enum scsi_status status;
};

/* See document SPC section 4.5.1, Sense data introduction */
struct scsi_sense_data {
	bool valid;
	uint8_t response_code;
	uint8_t sense_key;
	uint8_t additional_sense_code;
	uint8_t additional_sense_code_qualifier;
};

struct driver_config {
	/* Driver instance index, formatted into the drive name alongside the LUN */
	uint8_t driver_index;
};

static int reset_recovery(struct driver_data *driver_data);
static int clear_feature_endpoint_halt(struct driver_data *driver_data, uint8_t endpoint);

/*
 * Synchronization callback to wait for completion of asynchronous transfers
 * Should be passed to `usbh_xfer_alloc_with_buf` or `usbh_xfer_alloc` before queuing the
 * transfer, to then block on `driver_data->sync` in order to wait for completion. This
 * function only gives way to the semaphore; it doesn't analyze or deallocate anything.
 */
static int sync_cb(struct usb_device *const udev, struct uhc_transfer *const xfer)
{
	ARG_UNUSED(udev);
	struct driver_data *driver_data = xfer->priv;

	if (xfer->err != 0) {
		LOG_DBG("Request finished %p, err %d, sem %i", xfer, xfer->err,
			k_sem_count_get(&driver_data->sync));
	}
	k_sem_give(&driver_data->sync);

	return 0;
}

/*
 * Block on `driver_data->sync` waiting for the `sync_cb` callback
 * This function waits for the last transfer enqueued with `sync_cb` as completion to be
 * done. If it actually completes it returns the error code; in the event of a timeout it
 * makes sure the transfer is no longer pending.
 */
static int wait_for_sync(struct driver_data *driver_data, struct uhc_transfer *xfer)
{
	if (k_sem_take(&driver_data->sync, K_MSEC(SCSI_REQ_TIMEOUT)) != 0) {
		int result = 0;

		LOG_ERR("Timeout");

		result = usbh_xfer_dequeue(driver_data->udev, xfer);
		/* While the semaphore take timed out, the transfer was actually already
		 * done and the callback on its way. */
		if (result == -EALREADY) {
			/* Take the semaphore again to be sure that the callback is done */
			if (k_sem_take(&driver_data->sync, K_MSEC(SCSI_REQ_TIMEOUT)) != 0) {
				/* Should not happen */
				LOG_ERR("Double timeout");
			}
		}
		/* Dequeue failed */
		else if (result != 0) {
			LOG_ERR("Failed to cancel transfer");
		}

		return -ETIMEDOUT;
	}

	return xfer->err;
}

/*
 * Get the endpoint address for the provided direction.
 * Returns the corresponding endpoint address.
 */
static inline uint8_t get_endpoint_for_direction(struct driver_data *driver_data,
						 enum scsi_direction direction)
{
	if (direction == SCSI_DIRECTION_DATA_OUT) {
		return driver_data->out_bulk_ep->bEndpointAddress;
	} else {
		return driver_data->in_bulk_ep->bEndpointAddress;
	}
}

/*
 * Data transfer phase of a SCSI transaction.
 */
static int scsi_transfer_data(struct driver_data *driver_data, size_t data_length, uint8_t *data,
			      enum scsi_direction direction)
{
	struct uhc_transfer *xfer;
	int result = 0;
	/* Pick the target endpoint */
	uint8_t endpoint_address = get_endpoint_for_direction(driver_data, direction);

	if (data_length == 0 || data == NULL) {
		/* Nothing to do */
		return 0;
	}

	xfer = usbh_xfer_alloc_with_buf(driver_data->udev, endpoint_address, sync_cb, driver_data,
					data_length);
	if (xfer == NULL) {
		return -ENOMEM;
	}

	/* If the data goes to the device copy it into the buffer */
	if (direction == SCSI_DIRECTION_DATA_OUT) {
		net_buf_add_mem(xfer->buf, data, data_length);
	}

	result = usbh_xfer_enqueue(driver_data->udev, xfer);
	if (result != 0) {
		LOG_ERR("Unable to enqueue the transfer: %i", result);
		goto error_cleanup;
	}

	/* Wait for completion */
	result = wait_for_sync(driver_data, xfer);
	if (result != 0) {
		goto error_cleanup;
	}

	if (direction == SCSI_DIRECTION_DATA_IN) {
		/* If the data goes into the host copy it into the buffer */
		memcpy(data, xfer->buf->data, xfer->buf->len);
	}

	/* Return the number of bytes transferred */
	result = xfer->buf->len;

error_cleanup:
	/* Done with the buffer and transfer */
	usbh_xfer_buf_free(driver_data->udev, xfer->buf);
	usbh_xfer_free(driver_data->udev, xfer);
	return result;
}

/*
 * Read a pending Command Status Wrapper.
 */
static int scsi_read_status(struct driver_data *driver_data, struct scsi_csw *csw)
{
	uint8_t buffer[CSW_SIZE] = {};
	int result = 0;
	uint32_t signature = 0;

	result = scsi_transfer_data(driver_data, sizeof(buffer), buffer, SCSI_DIRECTION_DATA_IN);
	/* Stall, clear in endpoint and retry once */
	if (result == -ENOTSUP) {
		LOG_DBG("CSW stalled, clearing endpoint and retrying...");
		result = clear_feature_endpoint_halt(driver_data,
						     driver_data->in_bulk_ep->bEndpointAddress);
		if (result != 0) {
			LOG_ERR("Could not restore input endpoint: %i", result);
			return result;
		}

		/* Retry */
		result = scsi_transfer_data(driver_data, sizeof(buffer), buffer,
					    SCSI_DIRECTION_DATA_IN);
	}

	/* Other error */
	if (result < 0) {
		LOG_ERR("Error reading status: %i", result);
		return result;
	}
	/* Not enough data */
	else if (result < CSW_SIZE) {
		LOG_ERR("CSW too short: %i bytes", result);
		return -ENODATA;
	}

	signature = sys_get_le32(&buffer[0]);
	if (signature != CSW_SIGNATURE) {
		LOG_ERR("Invalid signature: 0x%04X", signature);
		return -EINVAL;
	}

	/* Extract tag and status */
	csw->tag = sys_get_le32(&buffer[4]);
	csw->data_residue = sys_get_le32(&buffer[8]);
	csw->status = buffer[12];

	return 0;
}

/*
 * Send an SCSI Command Block Wrapper.
 */
static int scsi_command(struct driver_data *driver_data, struct scsi_cbw cbw)
{
	struct uhc_transfer *xfer;
	int result = 0;
	uint8_t flags = 0;

	/* Refuse a logical unit index bigger than the last supported one on the drive or
	 * over the allocation limit */
	if (cbw.lun > driver_data->max_logical_unit ||
	    cbw.lun >= CONFIG_USBH_MSC_MAX_SUPPORTED_LUN) {
		return -EINVAL;
	}

	/* Too much data */
	if (cbw.command_block_length > CBW_COMMAND_BLOCK_MAX_LENGTH) {
		return -EINVAL;
	}

	xfer = usbh_xfer_alloc_with_buf(driver_data->udev,
					driver_data->out_bulk_ep->bEndpointAddress, sync_cb,
					driver_data, CBW_SIZE);
	if (xfer == NULL) {
		return -ENOMEM;
	}

	if (cbw.direction == SCSI_DIRECTION_DATA_IN) {
		flags |= CBW_FLAGS_DIRECTION_IN;
	}

	/* See USB MSC Bulk-Only Transport specification, section 5.1 */

	/* dCBWSignature */
	net_buf_add_le32(xfer->buf, CBW_SIGNATURE);
	/* dCBWTag */
	net_buf_add_le32(xfer->buf, driver_data->tag);
	/* dCBWDataTransferLength */
	net_buf_add_le32(xfer->buf, cbw.data_transfer_length);
	/* bmCBWFlags */
	net_buf_add_u8(xfer->buf, flags);
	/* bCBWLUN */
	net_buf_add_u8(xfer->buf, cbw.lun);
	/* bCBWCBLength */
	net_buf_add_u8(xfer->buf, cbw.command_block_length);
	/* CBWCB */
	net_buf_add_mem(xfer->buf, cbw.command_block, cbw.command_block_length);
	/* CBW padding */
	while (xfer->buf->len < CBW_SIZE) {
		net_buf_add_u8(xfer->buf, 0);
	}

	result = usbh_xfer_enqueue(driver_data->udev, xfer);
	if (result != 0) {
		LOG_ERR("Unable to enqueue the transfer: %i", result);
		goto error_cleanup;
	}

	result = wait_for_sync(driver_data, xfer);

error_cleanup:
	usbh_xfer_buf_free(driver_data->udev, xfer->buf);
	usbh_xfer_free(driver_data->udev, xfer);
	return result;
}

/*
 * Full SCSI transaction: CBW - Data - CSW.
 */
static int scsi_transaction(struct driver_data *driver_data, struct scsi_cbw cbw, uint8_t *data,
			    bool attempt_recovery)
{
	struct scsi_csw csw = {};
	int result = 0;
	size_t attempts = 0u;

	do {
		attempts++;

		/* If a failure occurred attempt recovery */
		if (result != 0) {
			LOG_WRN("Transaction failed, attempting %ith recovery", attempts - 1);
			result = reset_recovery(driver_data);
			if (result != 0) {
				LOG_ERR("Unable to recover the USB communication: %i", result);
				attempt_recovery = false;
				break;
			} else if (driver_data->lun_data[cbw.lun].unit_state != UNIT_STATE_READY) {
				LOG_ERR("Unit not ready after reset");
				attempt_recovery = false;
				result = -EAGAIN;
				break;
			} else {
				LOG_WRN("Communication recovered, retrying...");
			}
		}

		/* Send the command */
		result = scsi_command(driver_data, cbw);
		if (result != 0) {
			LOG_ERR("Unable to send CBW: %i", result);
			continue;
		}

		/* Data phase */
		if (cbw.data_transfer_length > 0 && data) {
			result = scsi_transfer_data(driver_data, cbw.data_transfer_length, data,
						    cbw.direction);
			/* The device stalled the transaction; it's a valid response, we should just
			 * clear the endpoint and continue with the status to check what happened */
			if (result == -ENOTSUP) {
				/* Pick the target endpoint */
				uint8_t endpoint_address =
					get_endpoint_for_direction(driver_data, cbw.direction);

				LOG_DBG("Data stalled, clearing endpoint");
				result = clear_feature_endpoint_halt(driver_data, endpoint_address);
			}

			/* Any other error */
			if (result < 0) {
				LOG_ERR("Error while transferring data: %i", result);
				continue;
			}
		}

		/* Get the status */
		result = scsi_read_status(driver_data, &csw);
		if (result != 0) {
			LOG_ERR("Unable to get CSW: %i", result);
			continue;
		}

		/* Wrong tag */
		if (csw.tag != driver_data->tag) {
			LOG_ERR("Mismatching CBW and CSW tags: 0x%04X vs 0x%04X", driver_data->tag,
				csw.tag);
			continue;
		}
		/* Increase the tag */
		driver_data->tag++;

		/* Check the status */
		switch (csw.status) {
		case SCSI_STATUS_COMMAND_FAILED: {
			LOG_ERR("SCSI command failed");
			/* This is the device reporting an issue: recovery won't help */
			attempt_recovery = false;
			result = -EACCES;
			break;
		}
		case SCSI_STATUS_PHASE_ERROR: {
			/* Problem with the BOT communication protocol */
			LOG_ERR("SCSI phase error");
			result = -EIO;
			break;
		}
		case SCSI_STATUS_COMMAND_PASSED: {
			if (csw.data_residue > cbw.data_transfer_length) {
				LOG_ERR("Impossible data residue: %i over %i", csw.data_residue,
					cbw.data_transfer_length);
				result = -EIO;
			} else {
				result = cbw.data_transfer_length - csw.data_residue;
			}
			break;
		}
		default: {
			LOG_WRN("Unknown status: 0x%02X", csw.status);
			result = -EIO;
			break;
		}
		}
	} while (result < 0 && attempts <= CONFIG_USBH_MSC_RECOVERY_ATTEMPTS && attempt_recovery &&
		 result != -ENOMEM);

	return result;
}

/*
 * Request SCSI sense data.
 */
static int scsi_request_sense(struct driver_data *driver_data, struct scsi_sense_data *sense_data,
			      uint8_t lun_index)
{
	int result = 0;
	uint8_t buffer[SCSI_MAX_SENSE_DATA] = {0u};
	uint8_t const command_block[6u] = {
		SCSI_COMMAND_REQUEST_SENSE, 0u, 0u, 0u, SCSI_MAX_SENSE_DATA, 0u,
	};
	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_IN,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = sizeof(buffer),
	};

	result = scsi_transaction(driver_data, cbw, buffer, true);

	if (result < 0) {
		LOG_ERR("Request sense failed: %i", result);
		return result;
	}
	/* Not enough data */
	else if (result < 14) {
		LOG_ERR("Not enough sense data: %i", result);
		return -EIO;
	}

	/* See SPC document, section 4.4 */
	sense_data->valid = (buffer[0] & 0x80) > 0;
	sense_data->response_code = buffer[0] & 0x7F;
	sense_data->sense_key = buffer[2] & 0xF;
	sense_data->additional_sense_code = buffer[12];
	sense_data->additional_sense_code_qualifier = buffer[13];

	return 0;
}

/*
 * Request sense data, converting it to a proper errno value.
 */
static int check_sense(struct driver_data *driver_data, uint8_t lun_index)
{
	struct scsi_sense_data sense_data = {};
	int result = scsi_request_sense(driver_data, &sense_data, lun_index);

	if (result != 0) {
		LOG_ERR("Failed to request sense data: %i", result);
		return result;
	}

	LOG_DBG("Sense response: 0x%02X 0x%02X 0x%02X 0x%02X", sense_data.response_code,
		sense_data.sense_key, sense_data.additional_sense_code,
		sense_data.additional_sense_code_qualifier);

	if (sense_data.response_code == SCSI_SENSE_DATA_RESPONSE_CODE_CURRENT) {
		/* The device is specifically reporting not being ready. Wait a bit
		 * and retry */
		if (sense_data.sense_key == SCSI_SENSE_DATA_KEY_UNIT_ATTENTION &&
		    sense_data.additional_sense_code == SCSI_SENSE_DATA_ASC_NOT_READY_TO_READY) {
			result = -EAGAIN;
		}
		/* Somehow medium was removed */
		else if (sense_data.sense_key == SCSI_SENSE_DATA_KEY_MEDIUM_ERROR) {
			result = -ENOMEDIUM;
		} else if (sense_data.additional_sense_code ==
			   SCSI_SENSE_DATA_ASC_MEDIUM_NOT_PRESENT) {
			result = -ENOMEDIUM;
		}
		/* The device is reporting some other condition, use a generic error */
		else {
			result = -EIO;
		}
	}
	/* We are only interested in the current data */
	else {
		result = -EINVAL;
	}

	return result;
}

/*
 * Check if the device is ready.
 */
static int test_unit_ready(struct driver_data *driver_data, uint8_t lun_index)
{
	int result = 0;
	size_t attempts = 0u;
	uint8_t const command_block[6u] = {
		SCSI_COMMAND_TEST_UNIT_READY, 0u, 0u, 0u, 0u, 0u,
	};
	struct scsi_cbw cbw = {
		.direction = SCSI_DIRECTION_DATA_OUT,
		.lun = lun_index,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = 0u,
	};

	/* Repeat the procedure for a number of attempts */
	for (attempts = 0; attempts < CONFIG_USBH_MSC_TEST_UNIT_READY_ATTEMPTS; attempts++) {
		result = scsi_transaction(driver_data, cbw, NULL, false);
		if (result == 0) {
			/* Done! */
			break;
		}
		/* Device is not ready, inspect the sense data */
		if (result == -EACCES) {
			result = check_sense(driver_data, lun_index);
			/* The device is specifically reporting not being ready. Wait a bit
			 * and retry */
			if (result == -EAGAIN) {
				LOG_DBG("Unit not ready yet, retrying...");
			}
			/* No media present, cannot be ready */
			else if (result == -ENOMEDIUM) {
				LOG_DBG("Medium on %i not present yet", lun_index);
				break;
			} else {
				/* The device is reporting some other condition, but the approach is
				 * the same: wait a bit and retry. */
			}
		} else {
			LOG_ERR("Unable to test the unit for readiness: %i", result);
			return result;
		}

		k_msleep(100);
	}

	if (attempts >= CONFIG_USBH_MSC_TEST_UNIT_READY_ATTEMPTS) {
		LOG_ERR("Was not ready in time");
		return -ETIMEDOUT;
	}

	return result;
}

/*
 * Read the capacity information of a logical unit, storing it in `driver_data`.
 */
static int read_capacity(struct driver_data *driver_data, uint8_t lun_index)
{
	int result = 0;
	uint8_t response[SCSI_READ_CAPACITY_10_DATA_LENGTH] = {};
	uint8_t const command_block[10u] = {
		SCSI_COMMAND_READ_CAPACITY_10, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u,
	};
	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_IN,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = sizeof(response),
	};

	result = scsi_transaction(driver_data, cbw, response, true);

	/* Transaction refused */
	if (result == -EACCES) {
		return check_sense(driver_data, lun_index);
	}
	/* Other error */
	else if (result < 0) {
		LOG_ERR("Unable to check the unit's capacity: %i", result);
		return result;
	}
	/* Not enough data */
	else if (result < 8) {
		LOG_ERR("Not enough capacity data: %i", result);
		return -EIO;
	}

	/* SCSI is an old protocol, uses big endian format */
	driver_data->lun_data[lun_index].last_logical_block_address = sys_get_be32(&response[0]);
	driver_data->lun_data[lun_index].block_length_in_bytes = sys_get_be32(&response[4]);

	return 0;
}

/*
 * Read the mode information of a logical unit, storing it in `driver_data`.
 */
static int mode_sense_6(struct driver_data *driver_data, uint8_t lun_index)
{
	int result = 0;
	uint8_t response[SCSI_MODE_SENSE_DATA_LENGTH] = {};
	uint8_t const command_block[6u] = {
		SCSI_COMMAND_MODE_SENSE_6,
		1u << 3u,                            /* Disable block descriptors */
		0x3Fu,                               /* All pages, current values */
		0u,                                  /* Subpage */
		SCSI_MODE_SENSE_DATA_LENGTH & 0xFFu, /* Data length */
		0u,
	};

	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_IN,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = sizeof(response),
	};

	result = scsi_transaction(driver_data, cbw, response, true);

	/* Transaction refused */
	if (result == -EACCES) {
		return check_sense(driver_data, lun_index);
	}
	/* Other error */
	else if (result < 0) {
		LOG_ERR("Unable to check the unit's mode: %i", result);
		return result;
	}
	/* Not enough data */
	else if (result < 4) {
		LOG_ERR("Not enough mode data: %i", result);
		return -EIO;
	}

	driver_data->lun_data[lun_index].write_protect = (response[3u] & (1u << 7u)) > 0;

	return 0;
}

/*
 * Read the mode information of a logical unit, storing it in `driver_data`.
 */
static int mode_sense_10(struct driver_data *driver_data, uint8_t lun_index)
{
	int result = 0;
	uint8_t response[SCSI_MODE_SENSE_DATA_LENGTH] = {};
	uint8_t const command_block[10u] = {
		SCSI_COMMAND_MODE_SENSE_10,
		1u << 3u, /* Disable block descriptors */
		0x3Fu,    /* All pages, current values */
		0u,       /* Subpage */
		0u,
		0u,
		0u,
		(SCSI_MODE_SENSE_DATA_LENGTH >> 8u) & 0xFFu, /* Data length */
		SCSI_MODE_SENSE_DATA_LENGTH & 0xFFu,
		0u,
	};
	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_IN,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = sizeof(response),
	};

	result = scsi_transaction(driver_data, cbw, response, true);

	/* Transaction refused */
	if (result == -EACCES) {
		return check_sense(driver_data, lun_index);
	}
	/* Other error */
	else if (result < 0) {
		LOG_ERR("Unable to check the unit's mode: %i", result);
		return result;
	}
	/* Not enough data */
	else if (result < 4) {
		LOG_ERR("Not enough mode data: %i", result);
		return -EIO;
	}

	driver_data->lun_data[lun_index].write_protect = (response[3u] & (1u << 7u)) > 0;

	return 0;
}

#ifndef CONFIG_USBH_MSC_IGNORE_SYNC
/*
 * Flush the device's caches.
 */
int usbh_msc_synchronize_cache(struct driver_data *driver_data, uint8_t lun_index)
{
	int result = 0;
	uint8_t const command_block[10u] = {
		SCSI_COMMAND_SYNCHRONIZE_CACHE_10,
		0u, /* Sync to medium */
		0u, /* LBA */
		0u,
		0u,
		0u,
		0u, /* Group number */
		0u, /* Number of blocks (all) */
		0u,
		0u,
	};
	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_OUT,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = 0u,
	};

	result = scsi_transaction(driver_data, cbw, NULL, true);

	/* Transaction refused */
	if (result == -EACCES) {
		struct scsi_sense_data sense_data = {};
		result = scsi_request_sense(driver_data, &sense_data, lun_index);

		if (result != 0) {
			LOG_ERR("Failed to request sense data: %i", result);
			return result;
		}

		if (sense_data.response_code == SCSI_SENSE_DATA_RESPONSE_CODE_CURRENT) {
			if ((sense_data.sense_key == SCSI_SENSE_DATA_KEY_ILLEGAL_REQUEST &&
			     sense_data.additional_sense_code ==
				     SCSI_SENSE_DATA_ASC_INVALID_COMMAND_OPERATION_CODE) ||
			    /* Some cheap device cut corners and don't implement proper
			     * response codes */
			    sense_data.additional_sense_code == 0u) {
				LOG_DBG("Sync unsupported, ignoring");
				return 0;
			}
			/* Other error */
			else if (result != 0) {
				LOG_ERR("Unable to sync the unit's caches: %i", result);
				return result;
			}
		}
	}

	return 0;
}
#endif /* CONFIG_USBH_MSC_IGNORE_SYNC */

/*
 * Read a number of blocks from a logical unit.
 */
int usbh_msc_read_blocks(struct driver_data *driver_data, uint8_t lun_index, uint32_t lba,
		       uint16_t block_count, uint8_t *buffer)
{
	int result = 0;
	uint32_t transfer_length =
		block_count * driver_data->lun_data[lun_index].block_length_in_bytes;
	uint8_t const command_block[10u] = {
		SCSI_COMMAND_READ_10,
		0u,                   /* Obsolete flags */
		(lba >> 24u) & 0xFFu, /* Big Endian address */
		(lba >> 16u) & 0xFFu,
		(lba >> 8u) & 0xFFu,
		lba & 0xFFu,
		0u,                          /* Obsolete flags */
		(block_count >> 8u) & 0xFFu, /* Big Endian count */
		block_count & 0xFFu,
		0u,
	};
	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_IN,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = transfer_length,
	};

	LOG_DBG("Read %i blocks starting from %i (%i bytes)", block_count, lba, transfer_length);

	result = scsi_transaction(driver_data, cbw, buffer, true);

	/* Transaction refused */
	if (result == -EACCES) {
		result = check_sense(driver_data, lun_index);
		if (result == -ENOMEDIUM) {
			driver_data->lun_data[lun_index].unit_state = UNIT_STATE_NO_MEDIUM;
			driver_data->lun_data[lun_index].last_logical_block_address = 0;
			driver_data->lun_data[lun_index].block_length_in_bytes = 0;
		}
		return result;
	}
	/* Other error */
	else if (result < 0) {
		LOG_ERR("Unable to read: %i", result);
		return result;
	}
	/* Not enough data */
	else if (result < transfer_length) {
		LOG_ERR("Not enough data read from unit: %i", result);
		return -EIO;
	}

	return 0;
}

/*
 * Write a number of blocks to a logical unit.
 */
int usbh_msc_write_blocks(struct driver_data *driver_data, uint8_t lun_index, uint32_t lba,
			uint16_t block_count, uint8_t const *buffer)
{
	int result = 0;
	uint32_t transfer_length =
		block_count * driver_data->lun_data[lun_index].block_length_in_bytes;
	uint8_t const command_block[10u] = {
		SCSI_COMMAND_WRITE_10,
		0u,                   /* Obsolete flags */
		(lba >> 24u) & 0xFFu, /* Big Endian address */
		(lba >> 16u) & 0xFFu,
		(lba >> 8u) & 0xFFu,
		lba & 0xFFu,
		0u,                          /* Obsolete flags */
		(block_count >> 8u) & 0xFFu, /* Big Endian count */
		block_count & 0xFFu,
		0u,
	};
	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_OUT,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = transfer_length,
	};

	LOG_DBG("Write %i blocks starting from %i (%i bytes)", block_count, lba, transfer_length);

	result = scsi_transaction(driver_data, cbw, (uint8_t *)buffer, true);

	/* Transaction refused */
	if (result == -EACCES) {
		result = check_sense(driver_data, lun_index);
		if (result == -ENOMEDIUM) {
			driver_data->lun_data[lun_index].unit_state = UNIT_STATE_NO_MEDIUM;
		}
		return result;
	}
	/* Other error */
	else if (result < 0) {
		LOG_ERR("Unable to write: %i", result);
		return result;
	}
	/* Not enough data */
	else if (result < transfer_length) {
		LOG_ERR("Unit did not write all data: %i", result);
		return -EIO;
	}

	return 0;
}

/*
 * Get the maximum logical unit index for this device.
 */
static int get_max_lun(struct driver_data *driver_data)
{
	struct net_buf *buf;
	int result = 0;

	buf = usbh_xfer_buf_alloc(driver_data->udev, 1);
	if (!buf) {
		return -ENOMEM;
	}

	/* See USB MSC Bulk-Only Transport specification, section 3.2 */
	const uint8_t bmRequestType = (USB_REQTYPE_DIR_TO_HOST << 7u) |
				      (USB_REQTYPE_TYPE_CLASS << 5u) |
				      (USB_REQTYPE_RECIPIENT_INTERFACE << 0u);
	result = usbh_req_setup(driver_data->udev, bmRequestType, GET_MAX_LUN, 0,
				driver_data->target_iface, 1, buf);
	/* A stalled GET_MAX_LUN request shall be interpreted as a unique unit */
	if (result == -ENOTSUP) {
		driver_data->max_logical_unit = 0;
		result = 0;
	}
	/* Otherwise read the response */
	else if (result == 0 && buf->len == 1) {
		driver_data->max_logical_unit = buf->data[0];
	}

	usbh_xfer_buf_free(driver_data->udev, buf);

	return result;
}

/*
 * Require a soft reset of the device.
 */
static int bulk_only_mass_storage_reset(struct driver_data *driver_data)
{
	/* See USB MSC Bulk-Only Transport specification, section 3.1 */
	const uint8_t bmRequestType = (USB_REQTYPE_DIR_TO_DEVICE << 7u) |
				      (USB_REQTYPE_TYPE_CLASS << 5u) |
				      (USB_REQTYPE_RECIPIENT_INTERFACE << 0u);
	return usbh_req_setup(driver_data->udev, bmRequestType, BULK_ONLY_MASS_STORAGE_RESET, 0,
			      driver_data->target_iface, 0, NULL);
}

/*
 * Clear an endpoint, re-enabling it.
 */
static int clear_feature_endpoint_halt(struct driver_data *driver_data, uint8_t endpoint)
{
	/* See USB specification, section 9.4 */
	const uint8_t bmRequestType = (USB_REQTYPE_DIR_TO_DEVICE << 7u) |
				      (USB_REQTYPE_TYPE_STANDARD << 5u) |
				      (USB_REQTYPE_RECIPIENT_ENDPOINT << 0u);
	return usbh_req_setup(driver_data->udev, bmRequestType, USB_SREQ_CLEAR_FEATURE,
			      USB_SFS_ENDPOINT_HALT, endpoint, 0, NULL);
}

/*
 * Apply a reset recovery procedure (see SPC section 5.3.4).
 */
static int reset_recovery(struct driver_data *driver_data)
{
	int result = 0;

	/* Soft reset the device */
	result = bulk_only_mass_storage_reset(driver_data);
	if (result != 0) {
		LOG_ERR("Could not issue mass storage reset: %i", result);
		return result;
	}

	/* Clear input endpoint */
	result =
		clear_feature_endpoint_halt(driver_data, driver_data->in_bulk_ep->bEndpointAddress);
	if (result != 0) {
		LOG_ERR("Could not restore input endpoint: %i", result);
		return result;
	}

	/* Clear output endpoint */
	result = clear_feature_endpoint_halt(driver_data,
					     driver_data->out_bulk_ep->bEndpointAddress);
	if (result != 0) {
		LOG_ERR("Could not restore output endpoint: %i", result);
		return result;
	}

	/* Check for readiness again */
	for (size_t lun_index = 0; lun_index <= driver_data->max_logical_unit &&
				   lun_index < CONFIG_USBH_MSC_MAX_SUPPORTED_LUN;
	     lun_index++) {
		result = test_unit_ready(driver_data, lun_index);
		if (result == -ENOMEDIUM) {
			driver_data->lun_data[lun_index].unit_state = UNIT_STATE_NO_MEDIUM;
			LOG_WRN("No medium present in unit: %i", lun_index);
		} else if (result != 0) {
			driver_data->lun_data[lun_index].unit_state = UNIT_STATE_NOT_READY;
			LOG_ERR("Unit %i not ready: %i", lun_index, result);
		} else {
			driver_data->lun_data[lun_index].unit_state = UNIT_STATE_READY;
		}
	}

	return 0;
}

/*
 * Scan endpoints in the interface.
 */
static int scan_interface_endpoints(struct driver_data *driver_data, uint8_t iface)
{
	const struct usb_desc_header *desc;
	const struct usb_ep_descriptor *ep_desc;
	const struct usb_if_descriptor *if_desc;
	int ep_count = 0;

	if_desc = (const void *)usbh_desc_get_iface(driver_data->udev, iface);
	if (if_desc == NULL) {
		LOG_ERR("Failed to find interface %u", iface);
		return -ENOSYS;
	}

	/* Iterate through all descriptors following the interface descriptor */
	desc = (const struct usb_desc_header *)if_desc;
	while ((desc = usbh_desc_get_next(desc)) != NULL && ep_count < if_desc->bNumEndpoints &&
	       (driver_data->in_bulk_ep == NULL || driver_data->out_bulk_ep == NULL)) {
		{
			/* Stop if we hit another interface descriptor */
			if (desc->bDescriptorType == USB_DESC_INTERFACE) {
				break;
			}

			/* Process endpoint descriptors */
			if (desc->bDescriptorType == USB_DESC_ENDPOINT) {
				ep_desc = (const void *)desc;

				/* Input bulk endpoint */
				if (USB_EP_DIR_IS_IN(ep_desc->bEndpointAddress) &&
				    driver_data->in_bulk_ep == NULL) {
					LOG_DBG("Input bulk endpoint: 0x%02X",
						ep_desc->bEndpointAddress);
					driver_data->in_bulk_ep = ep_desc;
				}
				/* Output bulk endpoint */
				else if (USB_EP_DIR_IS_OUT(ep_desc->bEndpointAddress) &&
					 driver_data->out_bulk_ep == NULL) {
					LOG_DBG("Output bulk endpoint: 0x%02X",
						ep_desc->bEndpointAddress);
					driver_data->out_bulk_ep = ep_desc;
				}

				ep_count++;
			}
		}
	}

	if (driver_data->in_bulk_ep == NULL || driver_data->out_bulk_ep == NULL) {
		LOG_ERR("Could not find bulk endpoints");
		return -EIO;
	}

	return 0;
}

/*
 * Fetch medatada (write protect status, capacity) for the specified unit.
 */
static int retrieve_unit_metadata(struct driver_data *driver_data, uint8_t lun_index)
{
	int result = 0;
	struct lun_data *lun_data = &driver_data->lun_data[lun_index];

	/* Fetch the unit's capacity */
	result = read_capacity(driver_data, lun_index);
	if (result != 0) {
		LOG_ERR("Could not read the unit's capacity: %i", result);
		return result;
	}

	/* Fetch the unit's mode */
	result = mode_sense_10(driver_data, lun_index);
	if (result != 0) {
		LOG_WRN("Mode sense (10) failed, falling back to Mode sense (6)");
		result = mode_sense_6(driver_data, lun_index);
	}

	if (result != 0) {
		LOG_WRN("Could not figure out if the device is read-only: %i", result);
	} else if (lun_data->write_protect) {
		LOG_INF("Unit %i is read-only", lun_index);
	} else {
		LOG_INF("Unit %i is writeable", lun_index);
	}

	return result;
}

/*
 * Attempt to initialize a unit
 */
int usbh_msc_initialize_unit(struct driver_data *driver_data, uint8_t lun_index)
{
	int result = 0;
	struct lun_data *lun_data = &driver_data->lun_data[lun_index];

	/* Test the unit for readiness */
	result = test_unit_ready(driver_data, lun_index);
	/* No medium connected */
	if (result == -ENOMEDIUM) {
		lun_data->unit_state = UNIT_STATE_NO_MEDIUM;
		return result;
	}
	/* Other error */
	else if (result != 0) {
		lun_data->unit_state = UNIT_STATE_NOT_READY;
		LOG_ERR("Unit not ready: %i", result);
		return result;
	}
	/* Unit is ready */
	else {
		lun_data->unit_state = UNIT_STATE_READY;
		LOG_DBG("Unit %i is ready", lun_index);
	}

	result = retrieve_unit_metadata(driver_data, lun_index);
	if (result == 0) {
		lun_data->unit_state = UNIT_STATE_READY;
	} else {
		lun_data->unit_state = UNIT_STATE_ERROR;
	}

	return result;
}

/*
 * Initialize the MSC host class driver.
 */
static int usbh_msc_init(struct usbh_class_data *const c_data)
{
	const struct device *dev = c_data->priv;
	struct driver_data *driver_data = (void *)dev->data;

	memset(driver_data, 0x00, sizeof(*driver_data));

	k_mutex_init(&driver_data->lock);
	k_sem_init(&driver_data->sync, 0, 1);

	return 0;
}

/*
 * Probe the USB class driver after a device has been found.
 */
static int usbh_msc_probe(struct usbh_class_data *const c_data, struct usb_device *const udev,
			  uint8_t iface)
{
	const struct device *dev = c_data->priv;
	struct driver_data *driver_data = (void *)dev->data;
	struct driver_config const *driver_config = (void *)dev->config;
	int result;

	LOG_INF("MSC device connected");

	if ((udev == NULL) || (udev->state != USB_STATE_CONFIGURED)) {
		LOG_ERR("USB device not properly configured");
		return -ENODEV;
	}

	if (driver_data == NULL) {
		LOG_ERR("No MSC device instance available");
		return -ENODEV;
	}

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	driver_data->udev = udev;

	/* Convert device-level match to interface 0 */
	if (iface == USBH_CLASS_IFNUM_DEVICE) {
		driver_data->target_iface = 0;
	} else {
		driver_data->target_iface = iface;
	}

	/* Fetch bulk endpoints */
	result = scan_interface_endpoints(driver_data, iface);
	if (result != 0) {
		LOG_ERR("Failed to scan endpoints: %d", result);
		goto error_cleanup;
	}

	/* Set control interface to default alternate setting (0) */
	result = usbh_device_interface_set(udev, driver_data->target_iface, 0, false);
	if (result != 0) {
		LOG_ERR("Failed to set control interface alternate setting: %d", result);
		goto error_cleanup;
	}

	/* Fetch max logical unit address */
	result = get_max_lun(driver_data);
	if (result != 0) {
		LOG_ERR("Could not get max LUN: %i", result);
		goto error_cleanup;
	}
	LOG_DBG("MAX LUN %i", driver_data->max_logical_unit);

	for (size_t lun_index = 0; lun_index <= driver_data->max_logical_unit &&
				   lun_index < CONFIG_USBH_MSC_MAX_SUPPORTED_LUN;
	     lun_index++) {
		usbh_msc_initialize_unit(driver_data, lun_index);

		/* Register the unit as an accessible disk */
		result = usbh_msc_disk_register(driver_data, dev, driver_config->driver_index,
						lun_index);
		if (result != 0) {
			LOG_ERR("Unable to register disk access driver: %i", result);
			goto error_cleanup;
		}
	}

	LOG_INF("MSC device (addr=%d) initialization completed", driver_data->udev->addr);

error_cleanup:
	k_mutex_unlock(&driver_data->lock);
	return result;
}

/*
 * Remove the USB class driver on disconnection
 */
static int usbh_msc_remove(struct usbh_class_data *const c_data)
{
	const struct device *dev = c_data->priv;
	struct driver_data *driver_data = (void *)dev->data;

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	/* Unregister disks */
	for (size_t lun_index = 0; lun_index <= driver_data->max_logical_unit &&
				   lun_index < CONFIG_USBH_MSC_MAX_SUPPORTED_LUN;
	     lun_index++) {
		usbh_msc_disk_unregister(&driver_data->lun_data[lun_index]);
	}

	driver_data->in_bulk_ep = NULL;
	driver_data->out_bulk_ep = NULL;
	driver_data->max_logical_unit = 0;

	k_mutex_unlock(&driver_data->lock);

	driver_data->udev = NULL;

	LOG_INF("MSC device removal completed");

	return 0;
}

/*
 * USB Host class API vtable.
 */
static struct usbh_class_api usbh_msc_api = {
	.init = usbh_msc_init,
	.probe = usbh_msc_probe,
	.removed = usbh_msc_remove,
};

/*
 * USB Host class filters.
 */
static struct usbh_class_filter usbh_msc_filters[] = {
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_MASS_STORAGE,
		.sub = SCSI_TRANSPARENT_COMMAND_SET,
		.proto = BULK_ONLY_TRANSPORT,
	},
	{0},
};

#define USBH_DEVICE_DEFINE(index, _)                                                               \
	static struct driver_config const driver_config_##index = {.driver_index = index};         \
	static struct driver_data driver_data_##index;                                             \
                                                                                                   \
	DEVICE_DEFINE(usbh_msc_##index, "usbh_msc_" #index, NULL, NULL, &driver_data_##index,      \
		      &driver_config_##index, POST_KERNEL, 50, NULL);                              \
                                                                                                   \
	USBH_DEFINE_CLASS(usbh_msc_data_##index, &usbh_msc_api,                                    \
			  (void *)DEVICE_GET(usbh_msc_##index), usbh_msc_filters);

LISTIFY(CONFIG_USBH_MSC_INSTANCES_COUNT, USBH_DEVICE_DEFINE, (;), _)
