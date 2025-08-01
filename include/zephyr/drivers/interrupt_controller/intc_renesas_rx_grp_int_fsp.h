/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_RX_GRP_FSP_H_
#define ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_RX_GRP_FSP_H_

#include "bsp_api.h"

int rx_grp_intc_enable(const struct device *dev, uint16_t factor);
int rx_grp_intc_disable(const struct device *dev, uint16_t factor);
int rx_grp_intc_callback_set(const struct device *dev, uint16_t factor, void (*callback)(void), void *context);

#endif /* ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_RX_GRP_FSP_H_ */
