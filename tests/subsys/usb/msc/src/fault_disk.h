/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FAULT_DISK_H_INCLUDED
#define FAULT_DISK_H_INCLUDED

#include <stdbool.h>

/* Disk name as registered with the Disk Access API and exposed as a third MSC LUN */
#define FAULT_DISK_NAME "FAULT0"

/**
 * @brief Register the simulated fault-injectable disk
 */
void fault_disk_setup(void);

/**
 * @brief Clear all fault injection, restoring normal disk behavior
 */
void fault_disk_reset(void);

/**
 * @brief Override the value reported by the disk's status callback
 *
 * @param status A DISK_STATUS_* value to report, or -1 to report the real (normal) status
 */
void fault_disk_set_status_override(int status);

/**
 * @brief Force (or stop forcing) the next reads to fail
 *
 * @param fail true to make reads fail, false to let them succeed normally
 */
void fault_disk_set_read_error(bool fail);

/**
 * @brief Force (or stop forcing) the next writes to fail
 *
 * @param fail true to make writes fail, false to let them succeed normally
 */
void fault_disk_set_write_error(bool fail);

#endif /* FAULT_DISK_H_INCLUDED */
