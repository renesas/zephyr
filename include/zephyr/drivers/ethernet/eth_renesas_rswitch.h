/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_ETH_RENESAS_RSWITCH_H_
#define ZEPHYR_DRIVERS_ETHERNET_ETH_RENESAS_RSWITCH_H_

#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/phy.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Called whenever the link state of a physical port a TSN-ES forwards
 * to/from changes.
 *
 * The TSN-ES driver uses this to gate its own net_if carrier: the interface
 * should only report carrier-up once the specific external port it is
 * forwarded to (not just its own internal ETHA/MAC) actually has a live
 * link, since traffic can only leave the board through that external port.
 *
 * @param dev Pointer to the R-Switch device structure.
 * @param tsnes_id The unique ID of the TSN-ES this notification is for.
 * @param is_up True if the forwarded port now has a live link, false otherwise.
 * @param user_data The user_data pointer passed to attach_tsnes().
 */
typedef void (*rswitch_fwd_link_cb_t)(const struct device *dev, uint32_t tsnes_id, bool is_up,
				      void *user_data);

/**
 * @brief Attach a TSN endpoint to the R-Switch.
 *
 * This function configures the switch's internal port (ETHA) and forwarding
 * rules to allow communication between the TSN-ES and external ports.
 *
 * @param dev Pointer to the R-Switch device structure.
 * @param tsnes_id The unique ID of the TSN-ES (e.g., 0 for tsnes0).
 * @param rswitch_port The internal switch port number connected to the TSN-ES.
 * @param fwd_mask A bitmask of external ports to forward traffic to/from.
 * @param mac_addr The MAC address of the TSN-ES.
 * @param link_cb Called whenever the link state of one of the ports in
 * fwd_mask changes (see rswitch_fwd_link_cb_t); called immediately, once,
 * with the current state if a forwarded port is already up at attach time.
 * May be NULL if the caller does not need link notifications.
 * @param user_data Opaque pointer passed back unchanged to link_cb.
 * @return 0 on success, a negative error code otherwise.
 */
typedef int (*rswitch_api_attach_tsnes_t)(const struct device *dev, uint32_t tsnes_id,
					  uint32_t rswitch_port, uint32_t fwd_mask,
					  const uint8_t *mac_addr, rswitch_fwd_link_cb_t link_cb,
					  void *user_data);

/**
 * @brief Set the link state for a physical port's MAC.
 *
 * Called when the underlying PHY link state changes. The rswitch3 driver
 * will then configure the corresponding ETHA MAC with the correct speed
 * and duplex.
 *
 * @param dev Pointer to the R-Switch device structure.
 * @param port_id The physical port number.
 * @param state Pointer to the new link state from the PHY.
 * @return 0 on success, a negative error code otherwise.
 */
typedef int (*rswitch_api_set_link_state_t)(const struct device *dev, uint32_t port_id,
					    const struct phy_link_state *state);

struct rswitch_driver_api {
	rswitch_api_attach_tsnes_t attach_tsnes;
	rswitch_api_set_link_state_t set_link_state;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_ETHERNET_ETH_RENESAS_RSWITCH_H_ */
