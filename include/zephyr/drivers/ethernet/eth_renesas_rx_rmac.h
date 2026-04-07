/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Renesas RX Ethernet RMAC header file
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ETH_RENESAS_RX_RMAC_H__
#define ZEPHYR_INCLUDE_DRIVERS_ETH_RENESAS_RX_RMAC_H__

#include <stdint.h>
#include <soc.h>

/**
 * @defgroup renesas_rx_eth_rmac_defs Renesas RX Ethernet Definitions
 * @brief Common constants for Renesas RX Ethernet drivers.
 * @{
 */

/* Organizationally Unique Identifier for MAC */
#define RENESAS_OUI_B0 0x74
#define RENESAS_OUI_B1 0x90
#define RENESAS_OUI_B2 0x50

#define RENESAS_RX_MPIC_LSC_10   0 /**< MPIC link speed: 10 Mbps. */
#define RENESAS_RX_MPIC_LSC_100  1 /**< MPIC link speed: 100 Mbps. */
#define RENESAS_RX_MPIC_LSC_1000 2 /**< MPIC link speed: 1000 Mbps. */

#define RENESAS_RX_ETHA_DISABLE_MODE   1 /**< RMAC disable mode. */
#define RENESAS_RX_ETHA_CONFIG_MODE    2 /**< RMAC configuration mode. */
#define RENESAS_RX_ETHA_OPERATION_MODE 3 /**< RMAC operation mode. */

#define RENESAS_RX_ETHA_REG_SIZE (R_ETHA1_BASE - R_ETHA0_BASE) /**< Size of ETHA register area. */
#define RENESAS_RX_RMAC_REG_SIZE (R_RMAC1_BASE - R_RMAC0_BASE)

/** @} end of renesas_rx_eth_rmac_defs group */

/**
 * @brief Get the current PHY operation mode.
 *
 * @param[in] p_instance_ctrl Pointer to RMAC PHY instance control structure.
 *
 * @return Current operation mode of ETHA IP.
 */
static inline uint8_t r_rmac_phy_get_operation_mode(uint8_t channel)
{
	R_ETHA0_Type *p_etha_reg =
		(R_ETHA0_Type *)(R_ETHA0_BASE + (RENESAS_RX_ETHA_REG_SIZE * channel));

	/* Return operation mode of ETHA IP. */
	return p_etha_reg->EAMS_b.OPS;
}

/**
 * @brief Set the PHY operation mode.
 *
 * @param[in] channel RMAC channel number.
 * @param[in] mode    PHY operation mode to set.
 */
static inline void r_rmac_phy_set_operation_mode(uint8_t channel, uint8_t mode)
{
	R_ETHA0_Type *p_etha_reg =
		(R_ETHA0_Type *)(R_ETHA0_BASE + (RENESAS_RX_ETHA_REG_SIZE * channel));
	
	R_RMAC0_Type *  p_reg_rmac = (R_RMAC0_Type *) (R_RMAC0_BASE + (RENESAS_RX_RMAC_REG_SIZE * channel));

	/* Mode transition */
	p_etha_reg->EAMC_b.OPC = R_ETHA0_EAMC_OPC_Msk & mode;

	if (!WAIT_FOR((p_etha_reg->EAMS_b.OPS == mode), 10000, k_busy_wait(1))) {
		p_reg_rmac->MIOC_b.MIOC = 0x01;
        	FSP_HARDWARE_REGISTER_WAIT(p_etha_reg->EAMS_b.OPS, mode);
        	p_reg_rmac->MIOC_b.MIOC = 0x00;
	}
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_ETH_RENESAS_RX_RMAC_H__ */
