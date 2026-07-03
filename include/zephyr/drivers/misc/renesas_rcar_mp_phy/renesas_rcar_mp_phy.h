/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MISC_RENESAS_RCAR_MP_PHY_H_
#define ZEPHYR_DRIVERS_MISC_RENESAS_RCAR_MP_PHY_H_

enum mp_phy_renesas_rcar_type {
	MP_PHY_TYPE_PCIE,
	MP_PHY_TYPE_ETH,
	MP_PHY_TYPE_USB,
};

struct mp_phy_renesas_rcar_cfg {
	uint8_t channel;
	uint8_t lane;
	enum mp_phy_renesas_rcar_type type;
};

int mp_phy_renesas_rcar_enable(const struct device *dev, struct mp_phy_renesas_rcar_cfg phy_cfg);

#endif /* ZEPHYR_DRIVERS_MISC_RENESAS_RCAR_MP_PHY_H_ */
