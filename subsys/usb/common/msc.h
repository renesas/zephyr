/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Mass Storage Class private header
 *
 * Header follows below documentation:
 * - USB Device Class Definition for Mass Storage Class Devices (Revision 1.4)
 * - SCSI Primary Commands 7
 * - SCSI Block Commands 7
 */

#ifndef ZEPHYR_SUBSYS_USB_COMMON_MSC_H
#define ZEPHYR_SUBSYS_USB_COMMON_MSC_H

/* Subclass and Protocol codes */
#define SCSI_TRANSPARENT_COMMAND_SET 0x06
#define BULK_ONLY_TRANSPORT          0x50

/* Control requests */
#define GET_MAX_LUN                  0xFE
#define BULK_ONLY_MASS_STORAGE_RESET 0xFF

/* Command wrapper */
#define CBW_SIGNATURE 0x43425355u
#define CSW_SIGNATURE 0x53425355u

#define CBW_FLAGS_DIRECTION_IN  0x80
#define CBW_FLAGS_RESERVED_MASK 0x3F

/* Used to determine if a device is ready to transfer data */
#define SCSI_COMMAND_TEST_UNIT_READY      0x00u
/* Requests that the device server transfer sense data to the application client */
#define SCSI_COMMAND_REQUEST_SENSE        0x03u
/* Requests that the device server transfer mode data */
#define SCSI_COMMAND_MODE_SENSE_6         0x1Au
/* Requests that the device server transfer capacity and medium format information */
#define SCSI_COMMAND_READ_CAPACITY_10     0x25u
/* Requests that the device server read the specified logical blocks */
#define SCSI_COMMAND_READ_10              0x28u
/* Requests that the device server write the specified logical blocks */
#define SCSI_COMMAND_WRITE_10             0x2Au
/* Requests that the device server ensure that the specified logical blocks have their most recent
data values recorded in non-volatile cache and/or on the medium */
#define SCSI_COMMAND_SYNCHRONIZE_CACHE_10 0x35u
/* Requests that the device server transfer mode data */
#define SCSI_COMMAND_MODE_SENSE_10        0x5Au

#endif /* ZEPHYR_SUBSYS_USB_COMMON_MSC_H */
