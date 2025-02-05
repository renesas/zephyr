/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MISC_RENESAS_LVD_H_
#define ZEPHYR_DRIVERS_MISC_RENESAS_LVD_H_

#include "r_lvd_rx_if.h"

int renesas_rx_lvd_get_status(const struct device *dev, lvd_status_position_t *status_position,
			      lvd_status_cross_t *status_cross);
int renesas_rx_lvd_clear_status(const struct device *dev);
int renesas_rx_lvd_register_callback(const struct device *dev, void (*callback)(void *),
				     void *user_data);
int renesas_rx_pin_set_cmpa2(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_MISC_RENESAS_LVD_H_ */
