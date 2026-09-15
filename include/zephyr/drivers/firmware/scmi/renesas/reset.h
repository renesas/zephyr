/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_FIRMWARE_SCMI_RENESAS_RESET_H_
#define ZEPHYR_INCLUDE_DRIVERS_FIRMWARE_SCMI_RENESAS_RESET_H_

#include <stdbool.h>

#include <zephyr/drivers/firmware/scmi/protocol.h>

/**
 * Renesas SCMI vendor protocol ID.
 *
 * This stays a decimal token because SCMI_PROTOCOL_NAME() pastes it into a
 * C identifier (scmi_protocol_128).
 */
#define SCMI_PROTOCOL_RENESAS_VENDOR 128

/** Renesas vendor RESET_GET_STATUS command ID. */
#define SCMI_RENESAS_VENDOR_MSG_RESET_GET_STATUS 0x80U

/** Renesas SCMI vendor protocol version supported by this helper. */
#define SCMI_RENESAS_VENDOR_PROTOCOL_SUPPORTED_VERSION 0x10000U

/** SCP response value: reset signal is asserted. */
#define SCMI_RENESAS_VENDOR_RESET_ASSERTED 0U

/** SCP response value: reset signal is released (deasserted). */
#define SCMI_RENESAS_VENDOR_RESET_RELEASED 1U

/**
 * @brief Get the current reset signal state through the Renesas SCMI vendor protocol.
 *
 * This is a Renesas extension, not an SCMI Reset Domain protocol command.
 *
 * @param domain_id Renesas SCMI Reset Domain ID.
 * @param asserted Set to true when reset is asserted, false when released.
 *
 * @retval 0 on success.
 * @retval negative errno otherwise.
 */
int scmi_renesas_reset_status_get(uint32_t domain_id, bool *asserted);

#endif /* ZEPHYR_INCLUDE_DRIVERS_FIRMWARE_SCMI_RENESAS_RESET_H_ */
