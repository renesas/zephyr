/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ETH_RENESAS_RX_ESWM_H__
#define ZEPHYR_INCLUDE_DRIVERS_ETH_RENESAS_RX_ESWM_H__

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include "r_rmac.h"
#include "r_rmac_phy.h"
#include "r_layer3_switch.h"

typedef enum e_layer3_switch_agent_mode {
	LAYER3_SWITCH_AGENT_MODE_RESET = (0),
	LAYER3_SWITCH_AGENT_MODE_DISABLE = (1),
	LAYER3_SWITCH_AGENT_MODE_CONFIG = (2),
	LAYER3_SWITCH_AGENT_MODE_OPERATION = (3),
} layer3_switch_agent_mode_t;


struct renesas_rx_eswm_config {
	const struct device *iclk_dev;
	const struct device *pclk_dev;
	const struct device *eswclk_dev;                     /* eswclk clock */
	const struct device *eswphyclk_dev;                  /* eswphyclk clock */
	struct clock_control_rx_subsys_cfg eswclk_subsys;    /* eswclk clock subsys */
	struct clock_control_rx_subsys_cfg eswphyclk_subsys; /* eswphyclk clock subsys */
};

struct renesas_rx_eswm_data {
	ether_switch_instance_t *ether_switch;
	layer3_switch_instance_ctrl_t *fsp_ctrl;
	ether_switch_cfg_t *fsp_cfg;
};

extern void r_layer3_switch_update_etha_operation_mode(uint8_t port,
						       layer3_switch_agent_mode_t mode);
extern void
r_layer3_switch_update_gwca_operation_mode(layer3_switch_instance_ctrl_t *p_instance_ctrl,
					   layer3_switch_agent_mode_t mode);

extern void layer3_switch_gwdi_isr(void);

#endif /* ZEPHYR_INCLUDE_DRIVERS_ETH_RENESAS_RX_ESWM_H__ */
