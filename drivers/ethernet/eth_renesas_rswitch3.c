/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_rswitch3, CONFIG_ETHERNET_LOG_LEVEL);

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/phy.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <ethernet/eth_stats.h>
#include "eth_renesas_rswitch3.h"

#define DT_DRV_COMPAT renesas_rcarsoc_rswitch3

/* Device behind the phandle property @p prop of node @p node_id, or NULL if it has none. */
#define RSWITCH3_DT_DEV_OR_NULL(node_id, prop)                                                     \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, prop), (DEVICE_DT_GET(DT_PHANDLE(node_id, prop))),   \
		    (NULL))

static inline uint32_t rswitch3_read(const struct device *dev, uint32_t reg)
{
	const struct rswitch3_config *config = dev->config;

	return sys_read32(config->secure_base + reg);
}

static inline void rswitch3_write(const struct device *dev, uint32_t reg, uint32_t val)
{
	const struct rswitch3_config *config = dev->config;

	sys_write32(val, config->secure_base + reg);
}

static void rswitch3_modify(const struct device *dev, uint32_t reg, uint32_t clear, uint32_t set)
{
	uint32_t val = rswitch3_read(dev, reg);

	rswitch3_write(dev, reg, (val & ~clear) | set);
}

static int rswitch3_reg_wait(const struct device *dev, uint32_t reg, uint32_t mask,
			     uint32_t expected)
{
	uint32_t val;
	bool ok;

	/* WAIT_FOR() returns the (truthy) expr on success, false on timeout. */
	ok = WAIT_FOR((val = rswitch3_read(dev, reg), (val & mask) == expected),
		      RSWITCH3_TIMEOUT_US, k_msleep(1));

	if (!ok) {
		LOG_ERR("Timeout waiting for reg 0x%x, mask 0x%x, expected 0x%x, value: 0x%x", reg,
			mask, expected, val);
	}

	return ok ? 0 : -ETIMEDOUT;
}

static bool rswitch3_agent_clock_is_enabled(const struct device *dev, uint32_t agent)
{
	uint32_t val = rswitch3_read(dev, RCEC);

	return (val & RCEC_RCE) && (val & BIT(agent));
}

static void rswitch3_agent_clock_ctrl(const struct device *dev, uint32_t agent, bool enable)
{
	if (enable) {
		rswitch3_modify(dev, RCEC, 0, RCEC_RCE | BIT(agent));
	} else {
		rswitch3_modify(dev, RCDC, 0, BIT(agent));
	}
}

static int rswitch3_etha_change_mode(const struct device *dev, uint32_t etha_port,
				     enum rsw3_etha_mode mode)
{
	const struct rswitch3_config *config = dev->config;
	mm_reg_t etha_base =
		config->secure_base + RSWITCH3_ETHA_OFFSET + etha_port * RSWITCH3_ETHA_SIZE;
	int ret;

	if (!rswitch3_agent_clock_is_enabled(dev, etha_port)) {
		rswitch3_agent_clock_ctrl(dev, etha_port, true);
	}

	sys_write32(mode, etha_base + EAMC);

	ret = rswitch3_reg_wait(dev, etha_base - config->secure_base + EAMS, EAMS_OPS_MASK, mode);

	if (mode == EAMC_OPC_DISABLE) {
		rswitch3_agent_clock_ctrl(dev, etha_port, false);
	}

	return ret;
}

static void rswitch3_etha_init_tsn_egress_path(const struct device *dev, uint32_t etha_port)
{
	const struct rswitch3_config *config = dev->config;
	mm_reg_t etha_base =
		config->secure_base + RSWITCH3_ETHA_OFFSET + etha_port * RSWITCH3_ETHA_SIZE;
	int q;

	sys_write32(0, etha_base + EATDRC);
	sys_write32(0, etha_base + EAIRC);
	sys_write32(0, etha_base + EATDQSC);
	sys_write32(0, etha_base + EATDQAC);
	sys_write32(0, etha_base + EATPEC);
	for (q = 0; q < RSWITCH3_NUM_PRIOS; q++) {
		sys_write32(RSWITCH3_TSNA_QUEUE_DEPTH, etha_base + EATDQDC(q));
		sys_write32(EATMFSC_MAX, etha_base + EATMFSC(q));
	}
	sys_write32(0, etha_base + EACTDQDC);
	sys_write32(0, etha_base + EATTFC);
}

static void rswitch3_etha_write_mac_address(const struct device *dev, uint32_t etha_port,
					    const uint8_t *mac)
{
	const struct rswitch3_config *config = dev->config;
	mm_reg_t etha_base =
		config->secure_base + RSWITCH3_ETHA_OFFSET + etha_port * RSWITCH3_ETHA_SIZE;

	sys_write32((mac[0] << 8) | mac[1], etha_base + MRMAC0);
	sys_write32((mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5], etha_base + MRMAC1);
}

static int rswitch3_etha_wait_link_verification(const struct device *dev, uint32_t etha_port)
{
	const struct rswitch3_config *config = dev->config;
	mm_reg_t etha_base =
		config->secure_base + RSWITCH3_ETHA_OFFSET + etha_port * RSWITCH3_ETHA_SIZE;

	sys_write32(MLVC_PLV, etha_base + MLVC);

	return rswitch3_reg_wait(dev, etha_base - config->secure_base + MLVC, MLVC_PLV, 0);
}

void rswitch3_dump_fwd_counters(const struct device *dev, uint32_t port_id, const char *tag)
{
	uint32_t ctfdcn = rswitch3_read(dev, FWCTFDCN(port_id));
	uint32_t pbfdcn = rswitch3_read(dev, FWPBFDCN(port_id));
	uint32_t mhlcn = rswitch3_read(dev, FWMHLCN(port_id));
	uint32_t icrdcn = rswitch3_read(dev, FWICRDCN(port_id));
	uint32_t wmrdcn = rswitch3_read(dev, FWWMRDCN(port_id));
	uint32_t ctrdcn = rswitch3_read(dev, FWCTRDCN(port_id));
	uint32_t pbrdcn = rswitch3_read(dev, FWPBRDCN(port_id));

	LOG_DBG("%s: port %u fwd-discard  TX[cut-through=%u port-based=%u hop-limit=%u]  "
		"RX[ingress-class=%u watermark=%u cut-through=%u port-based=%u]",
		tag, port_id, ctfdcn, pbfdcn, mhlcn, icrdcn, wmrdcn, ctrdcn, pbrdcn);
}

/* Frames dropped at ingress because of an EACKSC checksum error (read clears the counter). */
static uint32_t __maybe_unused rswitch3_etha_cksum_err_read(const struct device *dev,
							    uint32_t etha_port)
{
	return rswitch3_read(dev, RSWITCH3_ETHA_OFFSET + etha_port * RSWITCH3_ETHA_SIZE + EACKSECN);
}

static void rswitch3_fwd_allow_ports(const struct device *dev, uint32_t src_port, uint32_t dst_mask)
{
	if (!dst_mask || src_port >= RSWITCH3_NUM_AGENTS) {
		return;
	}

	dst_mask &= GENMASK(RSWITCH3_NUM_AGENTS - 1, 0);

	/* Configure port-based forwarding */
	rswitch3_modify(dev, FWPBFC(src_port), 0, FIELD_PREP(FWPBFC_PBDV, dst_mask));
	rswitch3_modify(dev, FWPBFC1(src_port), FWPBFC1_PBRP,
			FIELD_PREP(FWPBFC1_PBRP, FWPBFC1_PBRP_TSN));

	/* Allow L2 forwarding to the destination ports */
	rswitch3_modify(dev, FWPC0(src_port), 0, FWPC0_MACSDA);
	rswitch3_modify(dev, FWPC2(src_port), FWPC2_LTWFM_TO_PORT(dst_mask), 0);

	/*
	 * Allow L3-table and direct-descriptor forwarding to the destination ports.
	 */
	rswitch3_modify(dev, FWPC1(src_port), FWCP1_LTHFW_TO_PORT(dst_mask), 0);
}

static void rswitch3_enable_tsn_forwarding(const struct device *dev, uint32_t rswitch_port,
					   uint32_t fwd_mask)
{
	uint32_t external_ports = fwd_mask & GENMASK(7, 0);
	int i;

	if (!external_ports) {
		LOG_ERR("Agent %u has no external ports to forward to.", rswitch_port);
		return;
	}

	/* Allow traffic from the internal (TSN-ES) agent to external ports */
	rswitch3_fwd_allow_ports(dev, rswitch_port, external_ports);

	/* Allow traffic from external ports back to the internal agent */
	for (i = 0; i < 8; i++) {
		if (!(external_ports & BIT(i))) {
			continue;
		}
		rswitch3_fwd_allow_ports(dev, i, BIT(rswitch_port));
	}

	LOG_INF("Configured forwarding between agent %u and external ports mask 0x%x", rswitch_port,
		external_ports);
}

static void rswitch3_fwd_disallow_ports(const struct device *dev, uint32_t src_port,
					uint32_t dst_mask)
{
	if (!dst_mask || src_port >= RSWITCH3_NUM_AGENTS) {
		return;
	}

	dst_mask &= GENMASK(RSWITCH3_NUM_AGENTS - 1, 0);

	/* Undo rswitch3_fwd_allow_ports(): port-based forwarding towards dst_mask */
	rswitch3_modify(dev, FWPBFC(src_port), FIELD_PREP(FWPBFC_PBDV, dst_mask), 0);

	/* ... and L2-table, L3-table and direct-descriptor forwarding */
	rswitch3_modify(dev, FWPC2(src_port), 0, FWPC2_LTWFM_TO_PORT(dst_mask));
	rswitch3_modify(dev, FWPC1(src_port), 0, FWCP1_LTHFW_TO_PORT(dst_mask));
}

static void rswitch3_disable_tsn_forwarding_port(const struct device *dev, uint32_t rswitch_port,
						 uint32_t ext_port_id)
{
	rswitch3_fwd_disallow_ports(dev, rswitch_port, BIT(ext_port_id));
	rswitch3_fwd_disallow_ports(dev, ext_port_id, BIT(rswitch_port));

	LOG_INF("Revoked forwarding between agent %u and external port %u (link down)",
		rswitch_port, ext_port_id);
}

/*
 * net_eth_carrier_on/off() submit the Ethernet L2 carrier work, which only
 * exists once ethernet_init() has run for @p iface. Link changes seen before
 * that are applied later: by rswitch3_port_iface_init() for a port net_if, and
 * by rswitch3_iface_event_handler() for a TSN-ES net_if.
 */
static void rswitch3_set_carrier(struct net_if *iface, bool is_up)
{
	const struct ethernet_context *ctx;

	if (iface == NULL) {
		return;
	}

	ctx = net_if_l2_data(iface);
	if (!ctx->is_init) {
		return;
	}

	if (is_up) {
		net_eth_carrier_on(iface);
	} else {
		net_eth_carrier_off(iface);
	}
}

/*
 * A TSN-ES net_if has carrier while any external port it forwards to has a
 * link. Called with data->lock held.
 */
static void rswitch3_update_tsnes_carrier(const struct rswitch3_data *data,
					  const struct rswitch3_internal_port *ip)
{
	rswitch3_set_carrier(net_if_lookup_by_dev(ip->tsnes_dev),
			     (ip->fwd_mask & data->port_link_mask) != 0U);
}

/* Opens or closes the forwarding between a port and the agents in agent_mask. */
static void rswitch3_set_port_forwarding(const struct device *dev, uint32_t port_id,
					 uint32_t agent_mask, bool open)
{
	uint32_t agent;

	for (agent = 0; agent < RSWITCH3_NUM_AGENTS; agent++) {
		if (!(agent_mask & BIT(agent))) {
			continue;
		}
		if (open) {
			rswitch3_fwd_allow_ports(dev, agent, BIT(port_id));
		} else {
			rswitch3_fwd_disallow_ports(dev, agent, BIT(port_id));
		}
	}

	if (open) {
		rswitch3_fwd_allow_ports(dev, port_id, agent_mask);
	} else {
		rswitch3_fwd_disallow_ports(dev, port_id, agent_mask);
	}

	LOG_INF("Port %u: forwarding to agent mask 0x%x %s", port_id, agent_mask,
		open ? "opened" : "closed");
}

static void rswitch3_update_forwarding_for_link(const struct device *dev, uint32_t port_id,
						bool is_up)
{
	struct rswitch3_data *data = dev->data;
	uint32_t agent;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (is_up) {
		data->port_link_mask |= BIT(port_id);
	} else {
		data->port_link_mask &= ~BIT(port_id);
	}

	for (agent = 0; agent < RSWITCH3_NUM_AGENTS; agent++) {
		struct rswitch3_internal_port *ip = &data->internal_ports[agent];

		if (!ip->tsnes_dev) {
			continue;
		}
		if (!(ip->fwd_mask & BIT(port_id))) {
			continue;
		}

		if (is_up) {
			rswitch3_enable_tsn_forwarding(dev, agent, BIT(port_id));
		} else {
			rswitch3_disable_tsn_forwarding_port(dev, agent, port_id);
		}

		rswitch3_update_tsnes_carrier(data, ip);
	}

	if (data->gwca_fwd_mask[port_id] != 0U) {
		rswitch3_set_port_forwarding(dev, port_id, data->gwca_fwd_mask[port_id], is_up);
	}

	k_mutex_unlock(&data->lock);
}

static int rswitch3_etha_hw_init_tsn_internal(const struct device *dev, uint32_t rswitch_port,
					      const uint8_t *tsnes_mac)
{
	const struct rswitch3_config *config = dev->config;
	mm_reg_t etha_base =
		config->secure_base + RSWITCH3_ETHA_OFFSET + rswitch_port * RSWITCH3_ETHA_SIZE;
	int ret;

	ret = rswitch3_etha_change_mode(dev, rswitch_port, EAMC_OPC_DISABLE);
	if (ret) {
		return ret;
	}

	ret = rswitch3_etha_change_mode(dev, rswitch_port, EAMC_OPC_CONFIG);
	if (ret) {
		return ret;
	}

	rswitch3_etha_init_tsn_egress_path(dev, rswitch_port);

	/* Configure internal ETHA port for TSN-ES connection */
	sys_write32(EAVCC_VEM_NO_TAG, etha_base + EAVCC);
	rswitch3_etha_write_mac_address(dev, rswitch_port, tsnes_mac);
	sys_write32(RSW3_MRAFC_RX_PROMISC, etha_base + MRAFC);

	/* Force link up for internal connection */
	rswitch3_modify(dev, etha_base - config->secure_base + MIOC, 0, MIOC_FORCE_PHY_LINK);
	rswitch3_modify(dev, etha_base - config->secure_base + MPIC,
			MPIC_PIS | MPIC_LSC | MPIC_PLSPP,
			FIELD_PREP(MPIC_PIS, MPIC_PIS_GMII) | FIELD_PREP(MPIC_LSC, MPIC_LSC_2_5G) |
				MPIC_PLSPP);

	ret = rswitch3_etha_change_mode(dev, rswitch_port, EAMC_OPC_DISABLE);
	if (ret) {
		return ret;
	}

	ret = rswitch3_etha_change_mode(dev, rswitch_port, EAMC_OPC_OPERATION);
	if (ret) {
		return ret;
	}

	sys_write32(EATDQC_DISABLE_CUT_THROUGH, etha_base + EATDQC);

	return 0;
}

static int rswitch3_etha_hw_init_physical(const struct device *dev, uint32_t port_id,
					  const char *phy_mode)
{
	const struct rswitch3_config *config = dev->config;
	mm_reg_t etha_base =
		config->secure_base + RSWITCH3_ETHA_OFFSET + port_id * RSWITCH3_ETHA_SIZE;
	uint32_t mipc_pis_val;
	int ret;

	if (strcmp(phy_mode, "usxgmii") == 0) {
		/* Assuming XGMII for USXGMII based on some drivers */
		mipc_pis_val = 4; /* MPIC_PIS_XGMII */
	} else if (strcmp(phy_mode, "sgmii") == 0) {
		mipc_pis_val = 3; /* MPIC_PIS_SGMII */
	} else if (strcmp(phy_mode, "rgmii") == 0) {
		mipc_pis_val = 1; /* MPIC_PIS_RGMII */
	} else {
		LOG_ERR("Unsupported phy-mode '%s' for port %u", phy_mode, port_id);
		return -ENOTSUP;
	}

	LOG_INF("Initializing physical ETHA port %u with mode '%s'", port_id, phy_mode);

	ret = rswitch3_etha_change_mode(dev, port_id, EAMC_OPC_DISABLE);
	if (ret) {
		return ret;
	}

	ret = rswitch3_etha_change_mode(dev, port_id, EAMC_OPC_CONFIG);
	if (ret) {
		return ret;
	}

	/*
	 * Ports 5-7 share their SerDes lane between an external XPCS path and
	 * an internal TSN-ES path. MIOC bit 3 is the hardware
	 * selector that must be set for XPCS-attached ports in this range;
	 * ports 0-4 are hardwired to their external port and must not touch
	 * it.
	 */
	if (port_id >= 5 && port_id <= 7) {
		sys_write32(MIOC_BIT3_SET, etha_base + MIOC);
	}

	rswitch3_etha_init_tsn_egress_path(dev, port_id);
	sys_write32(EAVCC_VEM_SC_TAG, etha_base + EAVCC);

	/*
	 * Ingress checksum check: frames failing it are tagged CKSE and dropped
	 * by MFWD (FWCEPRC0.ECKSEEF = 0), so anything forwarded on to a CPU
	 * agent has a verified IPv4/TCP/UDP/ICMP checksum.
	 */
	sys_write32(IS_ENABLED(CONFIG_NET_CHECKSUM_OFFLOAD) ? EACKSC_ALL : 0, etha_base + EACKSC);

	/*
	 * Unlike the internal port, we don't force the link up.
	 * The link status will be controlled by the actual PHY device.
	 * We only set the PHY interface select (PIS).
	 */
	rswitch3_modify(dev, etha_base - config->secure_base + MPIC, MPIC_PIS,
			FIELD_PREP(MPIC_PIS, mipc_pis_val));

	return 0;
}

static int rswitch3_set_link_state(const struct device *dev, uint32_t port_id,
				   const struct phy_link_state *state)
{
	const struct rswitch3_config *config = dev->config;
	mm_reg_t etha_base =
		config->secure_base + RSWITCH3_ETHA_OFFSET + port_id * RSWITCH3_ETHA_SIZE;
	uint32_t mpic_lsc_val;
	uint32_t mpic_set = 0;
	int ret;

	if (!state->is_up) {
		/* Force link down on the MAC side */
		rswitch3_modify(dev, etha_base - config->secure_base + MIOC, MIOC_FORCE_PHY_LINK,
				0);
		return 0;
	}

	/*
	 * struct phy_link_state only reports full-duplex speed grades above
	 * 10Base (see enum phy_link_speed in zephyr/net/phy.h), so a match
	 * here always implies full duplex.
	 */
	switch (state->speed) {
	case LINK_FULL_100BASE:
		mpic_lsc_val = MPIC_LSC_100M;
		break;
	case LINK_FULL_1000BASE:
		mpic_lsc_val = MPIC_LSC_1G;
		break;
	case LINK_FULL_2500BASE:
		mpic_lsc_val = MPIC_LSC_2_5G;
		break;
	default:
		LOG_ERR("Port %u: Unsupported link speed 0x%x", port_id, state->speed);
		return -ENOTSUP;
	}

	mpic_set |= FIELD_PREP(MPIC_LSC, mpic_lsc_val) | MPIC_PLSPP;

	ret = rswitch3_etha_change_mode(dev, port_id, EAMC_OPC_DISABLE);
	if (ret) {
		return ret;
	}

	ret = rswitch3_etha_change_mode(dev, port_id, EAMC_OPC_CONFIG);
	if (ret) {
		return ret;
	}

	rswitch3_modify(dev, etha_base - config->secure_base + MPIC, MPIC_LSC | MPIC_PLSPP,
			mpic_set);
	rswitch3_modify(dev, etha_base - config->secure_base + MIOC, 0, MIOC_FORCE_PHY_LINK);

	ret = rswitch3_etha_change_mode(dev, port_id, EAMC_OPC_DISABLE);
	if (ret) {
		return ret;
	}

	ret = rswitch3_etha_change_mode(dev, port_id, EAMC_OPC_OPERATION);
	if (ret) {
		return ret;
	}

	ret = rswitch3_etha_wait_link_verification(dev, port_id);
	if (ret) {
		LOG_ERR("Port %u: link verification (MLVC) failed", port_id);
		return ret;
	}

	LOG_INF("rswitch3_etha_wait_link_verification successful");

	sys_write32(EATDQC_DISABLE_CUT_THROUGH, etha_base + EATDQC);

	return 0;
}

static void rswitch3_phys_port_link_changed(const struct device *phy_dev,
					    struct phy_link_state *state, void *user_data)
{
	const struct rswitch3_phys_port_ctx *ctx = user_data;
	int ret;

	ARG_UNUSED(phy_dev);

	LOG_INF("rswitch_port: %d, link up: %d, speed: %d", ctx->port_id, state->is_up,
		state->speed);

	if (!state->is_up) {
		/*
		 * Close forwarding first so MFWD stops routing new traffic
		 * into this port before its MAC link is forced down.
		 */
		rswitch3_update_forwarding_for_link(ctx->rswitch_dev, ctx->port_id, false);
	}

	ret = rswitch3_set_link_state(ctx->rswitch_dev, ctx->port_id, state);
	if (ret) {
		LOG_ERR("Port %u: failed to update MAC link state (err %d)", ctx->port_id, ret);
		return;
	}

	if (state->is_up) {
		if (ctx->pcs_dev) {
			ret = phy_configure_link(ctx->pcs_dev, state->speed, 0);
			if (ret) {
				LOG_ERR("Port %u: failed to bring up PCS for speed 0x%x (err %d)",
					ctx->port_id, state->speed, ret);
				return;
			}
		}

		rswitch3_update_forwarding_for_link(ctx->rswitch_dev, ctx->port_id, true);
	}

	if (ctx->port_net_dev != NULL) {
		rswitch3_set_carrier(net_if_lookup_by_dev(ctx->port_net_dev), state->is_up);
	}
}

static void rswitch3_wire_physical_port(const struct device *dev, uint32_t port_id,
					const struct device *phy_dev, const struct device *pcs_dev)
{
	struct rswitch3_data *data = dev->data;
	struct rswitch3_phys_port_ctx *ctx = &data->phys_ports[port_id];

	if (!device_is_ready(phy_dev)) {
		LOG_ERR("Port %u: external PHY device not ready, link will not be wired up",
			port_id);
		return;
	}

	if (pcs_dev && !device_is_ready(pcs_dev)) {
		LOG_ERR("Port %u: PCS channel device not ready, link will not be wired up",
			port_id);
		return;
	}

	*ctx = (struct rswitch3_phys_port_ctx){
		.rswitch_dev = dev,
		.pcs_dev = pcs_dev,
		.port_id = port_id,
	};

	phy_link_callback_set(phy_dev, rswitch3_phys_port_link_changed, ctx);
}

static int __maybe_unused rswitch3_init_tsnes_port(const struct device *dev, uint32_t rswitch_port,
						   const struct device *tsnes_dev,
						   const uint8_t *mac_addr, uint32_t fwd_mask)
{
	struct rswitch3_data *data = dev->data;
	struct rswitch3_internal_port *ip = &data->internal_ports[rswitch_port];
	uint32_t ready_ports;
	int ret;

	/* Initialize the internal ETHA port connected to the TSN-ES */
	ret = rswitch3_etha_hw_init_tsn_internal(dev, rswitch_port, mac_addr);
	if (ret) {
		LOG_ERR("Agent %u: internal ETHA init for %s failed (err %d)", rswitch_port,
			tsnes_dev->name, ret);
		return ret;
	}

	LOG_INF("Agent %u: internal port for %s, MAC %02x:%02x:%02x:%02x:%02x:%02x, "
		"forward-ports mask 0x%x",
		rswitch_port, tsnes_dev->name, mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3],
		mac_addr[4], mac_addr[5], fwd_mask);

	k_mutex_lock(&data->lock, K_FOREVER);

	ip->tsnes_dev = tsnes_dev;
	ip->fwd_mask = fwd_mask;

	/*
	 * Only open forwarding to external ports that already have a
	 * confirmed physical link; any port still down stays closed until
	 * rswitch3_phys_port_link_changed() sees it come up, so MFWD never
	 * forwards into a port whose MAC link is down.
	 */
	ready_ports = fwd_mask & data->port_link_mask;
	if (ready_ports != 0U) {
		rswitch3_enable_tsn_forwarding(dev, rswitch_port, ready_ports);
	}

	rswitch3_update_tsnes_carrier(data, ip);

	k_mutex_unlock(&data->lock);

	return 0;
}

/*
 * A TSN-ES net_if is initialized after this driver, so rswitch3_set_carrier()
 * skips link changes reported before its ethernet_init(). Apply the current
 * link state once the net_if is brought up.
 */
static void rswitch3_iface_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
					 struct net_if *iface)
{
	struct rswitch3_data *data = CONTAINER_OF(cb, struct rswitch3_data, iface_event_cb);
	const struct device *if_dev = net_if_get_device(iface);
	uint32_t agent;

	ARG_UNUSED(mgmt_event);

	k_mutex_lock(&data->lock, K_FOREVER);

	for (agent = 0U; agent < RSWITCH3_NUM_AGENTS; agent++) {
		const struct rswitch3_internal_port *ip = &data->internal_ports[agent];

		if ((ip->tsnes_dev != NULL) && (ip->tsnes_dev == if_dev)) {
			rswitch3_update_tsnes_carrier(data, ip);
		}
	}

	k_mutex_unlock(&data->lock);
}

static int rswitch3_coma_init(const struct device *dev)
{
	int ret;

	/* Request the shared buffer pool used by the forwarding engine */
	if (!(rswitch3_read(dev, CABPIRM) & CABPIRM_BPR)) {
		rswitch3_write(dev, CABPIRM, CABPIRM_BPIOG);

		ret = rswitch3_reg_wait(dev, CABPIRM, CABPIRM_BPR, CABPIRM_BPR);
		if (ret) {
			LOG_ERR("Timeout waiting for buffer pool configuration");
			return ret;
		}
	}

	rswitch3_write(dev, CABPPFLC0, CABPPFLC_INIT_VALUE);

	return 0;
}

static int rswitch3_fwd_init(const struct device *dev)
{
	uint32_t all_ports_mask = GENMASK(RSWITCH3_NUM_AGENTS - 1, 0);
	int i, ret;

	/* Start with an empty forwarding configuration instead of relying on HW reset state */
	for (i = 0; i < RSWITCH3_NUM_AGENTS; i++) {
		rswitch3_write(dev, FWPC0(i), 0);
		rswitch3_write(dev, FWPC1(i), FIELD_PREP(FWCP1_LTHFW, all_ports_mask));
		rswitch3_write(dev, FWPC2(i), FIELD_PREP(FWPC2_LTWFM, all_ports_mask));
		rswitch3_write(dev, FWPBFC(i), 0);
		rswitch3_write(dev, FWPBFC1(i), 0);
	}

	rswitch3_write(dev, FWMACTIM, FWMACTIM_MACTIOG);
	ret = rswitch3_reg_wait(dev, FWMACTIM, FWMACTIM_MACTR, FWMACTIM_MACTR);
	if (ret) {
		LOG_ERR("Timeout waiting for MAC table initialization");
		return ret;
	}

	return 0;
}

#if defined(CONFIG_ETH_RENESAS_RSWITCH3_GWCA)
/*
 * GWCA (Gateway CPU Agent) core.
 */
static inline uint32_t rswitch3_gwca_read(const struct device *dev, uint32_t offs)
{
	const struct rswitch3_config *cfg = dev->config;

	return rswitch3_read(dev, cfg->gwca.reg_base + offs);
}

static inline void rswitch3_gwca_write(const struct device *dev, uint32_t offs, uint32_t val)
{
	const struct rswitch3_config *cfg = dev->config;

	rswitch3_write(dev, cfg->gwca.reg_base + offs, val);
}

static inline void rswitch3_gwca_modify(const struct device *dev, uint32_t offs, uint32_t clear,
					uint32_t set)
{
	const struct rswitch3_config *cfg = dev->config;

	rswitch3_modify(dev, cfg->gwca.reg_base + offs, clear, set);
}

static inline int rswitch3_gwca_reg_wait(const struct device *dev, uint32_t offs, uint32_t mask,
					 uint32_t expected)
{
	const struct rswitch3_config *cfg = dev->config;

	return rswitch3_reg_wait(dev, cfg->gwca.reg_base + offs, mask, expected);
}

static inline struct rswitch3_gwca_tx_desc *rswitch3_gwca_tx_ring(const struct rswitch3_config *cfg,
								  uint32_t row)
{
	return &cfg->gwca.tx_rings[row * (RSWITCH3_GWCA_TX_RING_SIZE + 1)];
}

static inline uint8_t *rswitch3_gwca_tx_buf(const struct rswitch3_config *cfg, uint32_t row,
					    uint32_t idx)
{
	return &cfg->gwca.tx_bufs[(row * RSWITCH3_GWCA_TX_RING_SIZE + idx) *
				  RSWITCH3_GWCA_FRAME_BUF_SIZE];
}

static inline struct rswitch3_gwca_rx_desc *rswitch3_gwca_rx_ring(const struct rswitch3_config *cfg,
								  uint32_t row)
{
	return &cfg->gwca.rx_rings[row * (RSWITCH3_GWCA_RX_RING_SIZE + 1)];
}

static inline uint8_t *rswitch3_gwca_rx_buf(const struct rswitch3_config *cfg, uint32_t row,
					    uint32_t idx)
{
	return &cfg->gwca.rx_bufs[(row * RSWITCH3_GWCA_RX_RING_SIZE + idx) *
				  RSWITCH3_GWCA_FRAME_BUF_SIZE];
}

/* Cache maintenance around GWCA DMA, unless DMA is coherent or buffers are uncached. */
#define RSWITCH3_GWCA_CACHE_OPS                                                                    \
	(IS_ENABLED(CONFIG_ETH_RENESAS_RSWITCH3_GWCA_ENABLE_DMA_CACHE_OPS) &&                      \
	 !IS_ENABLED(CONFIG_NOCACHE_MEMORY))

static inline void rswitch3_gwca_dcache_flush(const void *addr, size_t size)
{
	if (RSWITCH3_GWCA_CACHE_OPS) {
		sys_cache_data_flush_range((void *)addr, size);
	}
}

static inline void rswitch3_gwca_dcache_invalidate(const void *addr, size_t size)
{
	if (RSWITCH3_GWCA_CACHE_OPS) {
		sys_cache_data_flush_and_invd_range((void *)addr, size);
	}
}

static void rswitch3_gwca_desc_set_addr(struct rswitch3_gwca_desc *desc, uintptr_t addr)
{
	desc->addr_lo = (uint32_t)addr;
	desc->addr_hi = (uint8_t)((uint64_t)addr >> 32);
}

static int rswitch3_gwca_change_mode(const struct device *dev, enum rsw3_gwca_mode mode)
{
	const struct rswitch3_config *cfg = dev->config;
	int ret;

	if (!rswitch3_agent_clock_is_enabled(dev, cfg->gwca.agent)) {
		rswitch3_agent_clock_ctrl(dev, cfg->gwca.agent, true);
	}

	rswitch3_gwca_write(dev, GWMC, mode);

	ret = rswitch3_gwca_reg_wait(dev, GWMS, GWMS_OPS_MASK, mode);

	if (mode == GWMC_OPC_DISABLE) {
		rswitch3_agent_clock_ctrl(dev, cfg->gwca.agent, false);
	}

	return ret;
}

static int rswitch3_gwca_mcast_table_reset(const struct device *dev)
{
	rswitch3_gwca_write(dev, GWMTIRM, GWMTIRM_MTIOG);

	return rswitch3_gwca_reg_wait(dev, GWMTIRM, GWMTIRM_MTR, GWMTIRM_MTR);
}

static int rswitch3_gwca_axi_ram_reset(const struct device *dev)
{
	rswitch3_gwca_write(dev, GWARIRM, GWARIRM_ARIOG);

	return rswitch3_gwca_reg_wait(dev, GWARIRM, GWARIRM_ARR, GWARIRM_ARR);
}

static void rswitch3_gwca_linkfix_init(const struct device *dev)
{
	const struct rswitch3_config *cfg = dev->config;
	uint32_t num_queues = 2U * cfg->gwca.num_ports;
	int i;

	for (i = 0; i < num_queues; i++) {
		cfg->gwca.linkfix_table[i].type = RSWITCH3_GWCA_DESC_EOS;
	}
	rswitch3_gwca_dcache_flush(cfg->gwca.linkfix_table,
				   num_queues * sizeof(*cfg->gwca.linkfix_table));

	/* GWDCBAC1 = lower 32 bits, GWDCBAC0 = upper 32 bits of the table's DMA address. */
	rswitch3_gwca_write(dev, GWDCBAC1, (uint32_t)(uintptr_t)cfg->gwca.linkfix_table);
	rswitch3_gwca_write(dev, GWDCBAC0,
			    (uint32_t)((uint64_t)(uintptr_t)cfg->gwca.linkfix_table >> 32));
}

static void rswitch3_gwca_tx_queue_format(const struct device *dev, uint32_t row)
{
	const struct rswitch3_config *cfg = dev->config;
	struct rswitch3_gwca_tx_queue *q = &cfg->gwca.tx_queues[row];
	size_t ring_bytes = (RSWITCH3_GWCA_TX_RING_SIZE + 1) * sizeof(*q->ring);
	uint32_t i;

	q->ring = rswitch3_gwca_tx_ring(cfg, row);
	for (i = 0; i < RSWITCH3_GWCA_TX_RING_SIZE; i++) {
		q->bufs[i] = rswitch3_gwca_tx_buf(cfg, row, i);
	}

	memset(q->ring, 0, ring_bytes);

	for (i = 0; i < RSWITCH3_GWCA_TX_RING_SIZE; i++) {
		rswitch3_gwca_desc_set_addr(&q->ring[i].hdr, (uintptr_t)q->bufs[i]);
		q->ring[i].hdr.type = RSWITCH3_GWCA_DESC_EEMPTY;
	}

	/* Wrap entry so the ring is circular. */
	rswitch3_gwca_desc_set_addr(&q->ring[RSWITCH3_GWCA_TX_RING_SIZE].hdr, (uintptr_t)q->ring);
	q->ring[RSWITCH3_GWCA_TX_RING_SIZE].hdr.type = RSWITCH3_GWCA_DESC_LINKFIX;

	rswitch3_gwca_dcache_flush(q->ring, ring_bytes);

	rswitch3_gwca_desc_set_addr(&cfg->gwca.linkfix_table[q->index], (uintptr_t)q->ring);
	cfg->gwca.linkfix_table[q->index].type = RSWITCH3_GWCA_DESC_LINKFIX;
	rswitch3_gwca_dcache_flush(&cfg->gwca.linkfix_table[q->index],
				   sizeof(cfg->gwca.linkfix_table[0]));

	/* No TX data interrupt: a completed slot reads back as FEMPTY, which is
	 * both "free" for SW and a terminate descriptor for HW, so nothing needs
	 * reaping.
	 */
	rswitch3_gwca_write(dev, GWDCC_OFFS(q->index),
			    GWDCC_BALR | GWDCC_DCP(RSWITCH3_GWCA_IPV) | GWDCC_DQT | GWDCC_EDE);
}

static void rswitch3_gwca_rx_desc_arm(struct rswitch3_gwca_rx_queue *q, uint32_t idx)
{
	struct rswitch3_gwca_rx_desc *desc = &q->ring[idx];

	desc->hdr.size = RSWITCH3_GWCA_FRAME_BUF_SIZE;
	rswitch3_gwca_desc_set_addr(&desc->hdr, (uintptr_t)q->bufs[idx]);
	desc->meta_lo = 0;
	desc->meta_hi = RSWITCH3_GWCA_META_SPN(q->port_id);
	desc->tsns = 0;
	desc->tss = 0;
	desc->hdr.type = RSWITCH3_GWCA_DESC_FEMPTY | RSWITCH3_GWCA_DESC_DIE;

	rswitch3_gwca_dcache_flush(desc, sizeof(*desc));
}

static void rswitch3_gwca_rx_queue_format(const struct device *dev, uint32_t row)
{
	const struct rswitch3_config *cfg = dev->config;
	struct rswitch3_data *data = dev->data;
	struct rswitch3_gwca_rx_queue *q = &cfg->gwca.rx_queues[row];
	uint32_t i;

	q->ring = rswitch3_gwca_rx_ring(cfg, row);
	for (i = 0; i < RSWITCH3_GWCA_RX_RING_SIZE; i++) {
		q->bufs[i] = rswitch3_gwca_rx_buf(cfg, row, i);
	}

	memset(q->ring, 0, (RSWITCH3_GWCA_RX_RING_SIZE + 1) * sizeof(*q->ring));

	for (i = 0; i < RSWITCH3_GWCA_RX_RING_SIZE; i++) {
		rswitch3_gwca_rx_desc_arm(q, i);
	}

	/* Wrap entry so the ring is circular. */
	rswitch3_gwca_desc_set_addr(&q->ring[RSWITCH3_GWCA_RX_RING_SIZE].hdr, (uintptr_t)q->ring);
	q->ring[RSWITCH3_GWCA_RX_RING_SIZE].hdr.type = RSWITCH3_GWCA_DESC_LINKFIX;
	rswitch3_gwca_dcache_flush(&q->ring[RSWITCH3_GWCA_RX_RING_SIZE], sizeof(q->ring[0]));

	rswitch3_gwca_desc_set_addr(&cfg->gwca.linkfix_table[q->index], (uintptr_t)q->ring);
	cfg->gwca.linkfix_table[q->index].type = RSWITCH3_GWCA_DESC_LINKFIX;
	rswitch3_gwca_dcache_flush(&cfg->gwca.linkfix_table[q->index],
				   sizeof(cfg->gwca.linkfix_table[0]));

	rswitch3_gwca_write(dev, GWDCC_OFFS(q->index), GWDCC_BALR | GWDCC_ETS | GWDCC_EDE);
	rswitch3_write(dev, TPDEMIMC(q->index),
		       TPDEMIMC_GDICM(cfg->gwca.instance, q->index % RSWITCH3_GWCA_NUM_IRQS));
	data->gwca.rx_irq_mask[q->index / 32U] |= GWCA_Q_BIT(q->index);
}

static struct rswitch3_gwca_tx_queue *rswitch3_gwca_tx_queue_alloc(const struct device *dev,
								   uint32_t port_id)
{
	const struct rswitch3_config *cfg = dev->config;
	struct rswitch3_data *data = dev->data;
	uint8_t row;
	struct rswitch3_gwca_tx_queue *q;

	if (port_id >= RSWITCH3_NUM_AGENTS) {
		return NULL;
	}

	row = data->gwca.port_row[port_id];
	if (row == RSWITCH3_GWCA_ROW_INVALID) {
		return NULL;
	}

	q = &cfg->gwca.tx_queues[row];
	if (!q->used) {
		q->index = row;
		q->port_id = port_id;
		q->cur = 0;
		rswitch3_gwca_tx_queue_format(dev, row);
		/* Publish the queue only once its ring is formatted. */
		barrier_dmem_fence_full();
		q->used = true;
	}

	return q;
}

static struct rswitch3_gwca_rx_queue *rswitch3_gwca_rx_queue_alloc(const struct device *dev,
								   uint32_t port_id,
								   rswitch_gwca_rx_cb_t rx_cb,
								   void *user_data)
{
	const struct rswitch3_config *cfg = dev->config;
	struct rswitch3_data *data = dev->data;
	uint8_t row;
	struct rswitch3_gwca_rx_queue *q;

	if (port_id >= RSWITCH3_NUM_AGENTS) {
		return NULL;
	}

	row = data->gwca.port_row[port_id];
	if (row == RSWITCH3_GWCA_ROW_INVALID) {
		return NULL;
	}

	q = &cfg->gwca.rx_queues[row];
	if (!q->used) {
		q->index = cfg->gwca.num_ports + row;
		q->port_id = port_id;
		q->cur = 0;
		q->rx_cb = rx_cb;
		q->rx_cb_user_data = user_data;
		rswitch3_gwca_rx_queue_format(dev, row);
		/* The RX thread polls a queue as soon as it reads used, so publish it
		 * only once its ring is formatted, and enable its interrupt after that.
		 */
		barrier_dmem_fence_full();
		q->used = true;
		rswitch3_gwca_write(dev, GWDIE(q->index), GWCA_Q_BIT(q->index));
	}

	return q;
}

static void rswitch3_gwca_rx_poll(const struct device *dev, struct rswitch3_gwca_rx_queue *q)
{
	uint32_t i;

	for (i = 0; i < RSWITCH3_GWCA_RX_RING_SIZE; i++) {
		struct rswitch3_gwca_rx_desc *desc = &q->ring[q->cur];
		uint32_t len;

		rswitch3_gwca_dcache_invalidate(desc, sizeof(*desc));

		if ((desc->hdr.type & RSWITCH3_GWCA_DESC_TYPE_MASK) != RSWITCH3_GWCA_DESC_FSINGLE) {
			break;
		}

		/* Drop frames whose descriptor reports an AXI, data size or frame error. */
		if (((desc->hdr.type & RSWITCH3_GWCA_DESC_ERR_MASK) == 0U) && (q->rx_cb != NULL)) {
			len = MIN(desc->hdr.size, RSWITCH3_GWCA_FRAME_BUF_SIZE);
			rswitch3_gwca_dcache_invalidate(q->bufs[q->cur], len);
			q->rx_cb(dev, q->port_id, q->bufs[q->cur], len, q->rx_cb_user_data);
		}

		rswitch3_gwca_rx_desc_arm(q, q->cur);

		q->cur = (q->cur + 1) % RSWITCH3_GWCA_RX_RING_SIZE;
	}
}

static void rswitch3_gwca_rx_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = (const struct device *)arg1;
	const struct rswitch3_config *cfg = dev->config;
	struct rswitch3_data *data = dev->data;
	uint32_t i;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		/* k_sem_give() wakes this promptly; the timeout is a fallback poll only. */
		(void)k_sem_take(&data->gwca.rx_sem, K_MSEC(1));

		for (i = 0; i < cfg->gwca.num_ports; i++) {
			if (cfg->gwca.rx_queues[i].used) {
				rswitch3_gwca_rx_poll(dev, &cfg->gwca.rx_queues[i]);
			}
		}
	}
}

static void rswitch3_gwca_work_handler(struct k_work *work)
{
	struct rswitch3_gwca_data *gwca = CONTAINER_OF(work, struct rswitch3_gwca_data, work);
	const struct device *dev = gwca->dev;
	uint32_t status[RSWITCH3_GWCA_NUM_DI_REGS];
	k_spinlock_key_t key;
	uint32_t i;

	key = k_spin_lock(&gwca->irq_lock);
	for (i = 0U; i < RSWITCH3_GWCA_NUM_DI_REGS; i++) {
		status[i] = gwca->pending_bits[i];
		gwca->pending_bits[i] = 0U;
	}
	k_spin_unlock(&gwca->irq_lock, key);

	for (i = 0U; i < RSWITCH3_GWCA_NUM_DI_REGS; i++) {
		if (status[i] != 0U) {
			k_sem_give(&gwca->rx_sem);
			rswitch3_gwca_write(dev, GWDIE(i * 32U), status[i]);
		}
	}
}

static void rswitch3_gwca_isr(const struct device *dev)
{
	struct rswitch3_data *data = dev->data;
	bool pending = false;
	uint32_t status;
	k_spinlock_key_t key;
	uint32_t i;

	/* Only RX queues raise data interrupts; rx_irq_mask has a bit for each of them. */
	for (i = 0U; i < RSWITCH3_GWCA_NUM_DI_REGS; i++) {
		/* i * 32 is the first descriptor chain held by register i */
		if (data->gwca.rx_irq_mask[i] == 0U) {
			continue;
		}

		status = rswitch3_gwca_read(dev, GWDIS(i * 32U)) & data->gwca.rx_irq_mask[i];
		if (status == 0U) {
			continue;
		}

		/* Disable and ack (write-1-to-clear) the queues just seen; the work
		 * handler re-enables them once it has processed this batch.
		 */
		rswitch3_gwca_write(dev, GWDID(i * 32U), status);
		rswitch3_gwca_write(dev, GWDIS(i * 32U), status);

		key = k_spin_lock(&data->gwca.irq_lock);
		data->gwca.pending_bits[i] |= status;
		k_spin_unlock(&data->gwca.irq_lock, key);

		pending = true;
	}

	if (pending) {
		k_work_submit(&data->gwca.work);
	}
}

/*
 * IRQ_CONNECT() needs a compile-time instance number, which this runtime
 * ISR-registration code doesn't have -- RSWITCH3_GWCA_IRQ_INIT(inst) is
 * therefore invoked directly from the RSWITCH3_INIT(inst) macro instead of
 * from a plain function; see the #undef block at the end of this file.
 */
#define RSWITCH3_GWCA_IRQ_INIT(inst)                                                               \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, gwca_gwdis, irq),                            \
			    DT_INST_IRQ_BY_NAME(inst, gwca_gwdis, priority), rswitch3_gwca_isr,    \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, gwca_gwdis, irq));                            \
	} while (0)

#define RSWITCH3_GWCA_RX_PRIO CONFIG_ETH_RENESAS_RSWITCH3_GWCA_RX_THREAD_PRIORITY

BUILD_ASSERT((RSWITCH3_GWCA_RX_PRIO >= 0) && (RSWITCH3_GWCA_RX_PRIO < CONFIG_NUM_COOP_PRIORITIES),
	     "GWCA RX thread priority must be below CONFIG_NUM_COOP_PRIORITIES");

static int rswitch3_gwca_hw_init(const struct device *dev)
{
	const struct rswitch3_config *cfg = dev->config;
	struct rswitch3_data *data = dev->data;
	uint32_t i;
	int ret;

	data->gwca.dev = dev;

	memset(data->gwca.port_row, RSWITCH3_GWCA_ROW_INVALID, sizeof(data->gwca.port_row));
	for (i = 0; i < cfg->gwca.num_ports; i++) {
		data->gwca.port_row[cfg->gwca.port_ids[i]] = i;
	}

	rswitch3_modify(dev, FWPC1(cfg->gwca.agent), 0, FWPC1_DDE);

	ret = rswitch3_gwca_change_mode(dev, GWMC_OPC_DISABLE);
	if (ret) {
		return ret;
	}

	ret = rswitch3_gwca_change_mode(dev, GWMC_OPC_CONFIG);
	if (ret) {
		return ret;
	}

	ret = rswitch3_gwca_mcast_table_reset(dev);
	if (ret) {
		LOG_ERR("Timeout waiting for GWCA multicast table reset");
		return ret;
	}

	ret = rswitch3_gwca_axi_ram_reset(dev);
	if (ret) {
		LOG_ERR("Timeout waiting for GWCA AXI RAM reset");
		return ret;
	}

	rswitch3_gwca_write(dev, GWVCC, GWVCC_VEM_SC_TAG);
	rswitch3_gwca_write(dev, GWTTFC, 0);
	rswitch3_gwca_write(dev, GWMDNC, GWMDNC_TSDMN(1) | GWMDNC_TXDMN(0x1e) | GWMDNC_RXDMN(0x1f));
	rswitch3_gwca_write(dev, GWTPC0, GWTPC_PPPL(RSWITCH3_GWCA_IPV));
	rswitch3_gwca_modify(dev, GWCKSC, 0, GWCKSC_USMFSPE);

	rswitch3_gwca_linkfix_init(dev);

	ret = rswitch3_gwca_change_mode(dev, GWMC_OPC_DISABLE);
	if (ret) {
		return ret;
	}

	ret = rswitch3_gwca_change_mode(dev, GWMC_OPC_OPERATION);
	if (ret) {
		return ret;
	}

	/* IRQ_CONNECT() itself happens in the per-instance init wrapper
	 * generated by RSWITCH3_INIT(inst), via RSWITCH3_GWCA_IRQ_INIT(inst)
	 */
	k_sem_init(&data->gwca.rx_sem, 0, 1);
	k_work_init(&data->gwca.work, rswitch3_gwca_work_handler);
	k_thread_create(&data->gwca.rx_thread_data, data->gwca.rx_stack,
			K_KERNEL_STACK_SIZEOF(data->gwca.rx_stack), rswitch3_gwca_rx_thread,
			(void *)dev, NULL, NULL, K_PRIO_COOP(RSWITCH3_GWCA_RX_PRIO), 0, K_NO_WAIT);

	LOG_INF("Renesas R-Switch 3 GWCA%u initialized", cfg->gwca.instance);

	return 0;
}

static int rswitch3_gwca_attach_port(const struct device *dev, uint32_t port_id,
				     rswitch_gwca_rx_cb_t rx_cb, void *user_data)
{
	const struct rswitch3_config *cfg = dev->config;
	struct rswitch3_data *data = dev->data;
	struct rswitch3_gwca_tx_queue *txq;
	struct rswitch3_gwca_rx_queue *rxq;
	int ret = 0;

	if (!rx_cb || port_id >= RSWITCH3_NUM_AGENTS) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	txq = rswitch3_gwca_tx_queue_alloc(dev, port_id);
	if (!txq) {
		LOG_ERR("Port %u: no free GWCA TX queue", port_id);
		ret = -ENOSPC;
		goto out_unlock;
	}

	rxq = rswitch3_gwca_rx_queue_alloc(dev, port_id, rx_cb, user_data);
	if (!rxq) {
		LOG_ERR("Port %u: no free GWCA RX queue", port_id);
		txq->used = false;
		ret = -ENOSPC;
		goto out_unlock;
	}

	/* Default MFWD steering: this port's CPU-bound traffic lands in its GWCA RX queue. */
	rswitch3_write(dev, FWPBFCSDC(cfg->gwca.instance, port_id), rxq->index);

	/*
	 * The forwarding between the port and the gateway is not opened here.
	 * As for TSN-ES, the link of the port decides it: the mask is recorded
	 * and rswitch3_update_forwarding_for_link() opens the forwarding on link
	 * up and closes it again on link down, so MFWD never forwards frames
	 * still in the TX ring into a port whose MAC link is down.
	 */
	data->gwca_fwd_mask[port_id] = BIT(cfg->gwca.agent);

	if (data->port_link_mask & BIT(port_id)) {
		rswitch3_set_port_forwarding(dev, port_id, BIT(cfg->gwca.agent), true);
	}

	/* The net_if is not initialized yet; rswitch3_port_iface_init() sets its carrier. */
	data->phys_ports[port_id].port_net_dev = user_data;

	LOG_INF("Port %u attached to GWCA (tx queue %u, rx queue %u)", port_id, txq->index,
		rxq->index);

out_unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

/* tx_meta: extra meta_lo flags for the direct descriptor (RSWITCH3_GWCA_META_L3CC/L4CC). */
static int rswitch3_gwca_port_send(const struct device *dev, uint32_t port_id, const uint8_t *frame,
				   size_t len, uint32_t tx_meta)
{
	const struct rswitch3_config *cfg = dev->config;
	struct rswitch3_data *data = dev->data;
	struct rswitch3_gwca_tx_queue *q;
	struct rswitch3_gwca_tx_desc *desc;
	k_spinlock_key_t key;
	uint8_t type;
	uint8_t row;
	int ret = 0;

	if (port_id >= RSWITCH3_NUM_AGENTS) {
		return -ENODEV;
	}

	row = data->gwca.port_row[port_id];
	if (row == RSWITCH3_GWCA_ROW_INVALID) {
		return -ENODEV;
	}

	q = &cfg->gwca.tx_queues[row];
	if (!q->used) {
		return -ENODEV;
	}

	if (len > RSWITCH3_GWCA_FRAME_BUF_SIZE) {
		return -EMSGSIZE;
	}

	key = k_spin_lock(&data->gwca.tx_lock);

	desc = &q->ring[q->cur];

	/* EEMPTY: never used. FEMPTY: HW has fetched the frame and written the
	 * descriptor back. Anything else is still queued: the ring is full.
	 */
	rswitch3_gwca_dcache_invalidate(desc, sizeof(*desc));
	type = desc->hdr.type & RSWITCH3_GWCA_DESC_TYPE_MASK;
	if (type != RSWITCH3_GWCA_DESC_EEMPTY && type != RSWITCH3_GWCA_DESC_FEMPTY) {
		ret = -ENOSPC;
		goto out_unlock;
	}

	memcpy(q->bufs[q->cur], frame, len);
	rswitch3_gwca_dcache_flush(q->bufs[q->cur], len);

	desc->hdr.size = (uint16_t)len;
	rswitch3_gwca_desc_set_addr(&desc->hdr, (uintptr_t)q->bufs[q->cur]);
	desc->meta_lo = RSWITCH3_GWCA_META_FMT | RSWITCH3_GWCA_META_IPV(RSWITCH3_GWCA_IPV) |
			(tx_meta & (RSWITCH3_GWCA_META_L3CC | RSWITCH3_GWCA_META_L4CC));
	desc->meta_hi = RSWITCH3_GWCA_META_DV(BIT(port_id));
	rswitch3_gwca_dcache_flush(desc, sizeof(*desc));

	/* type written last so hardware never observes a partially-filled descriptor. */
	desc->hdr.type = RSWITCH3_GWCA_DESC_FSINGLE;
	rswitch3_gwca_dcache_flush(&desc->hdr.type, sizeof(desc->hdr.type));

	q->cur = (q->cur + 1) % RSWITCH3_GWCA_TX_RING_SIZE;

out_unlock:
	k_spin_unlock(&data->gwca.tx_lock, key);

	if (ret == 0) {
		/* Kick. GWTRCi.TSRt is write-1-to-set, so no read-modify-write is needed. */
		rswitch3_gwca_write(dev, GWTRC(q->index), GWCA_Q_BIT(q->index));
	}

	return ret;
}
#endif /* CONFIG_ETH_RENESAS_RSWITCH3_GWCA */

/*
 * Brings up one physical ETHA port and, if it has an external PHY, wires its
 * link-state callback. Necessary for TSN-ES forwarding to work.
 */
static int __maybe_unused rswitch3_init_physical_port(const struct device *dev, uint32_t port_id,
						      const char *phy_mode,
						      const struct device *phy_dev,
						      const struct device *pcs_dev)
{
	int ret;

	ret = rswitch3_etha_hw_init_physical(dev, port_id, phy_mode);
	if (ret) {
		LOG_ERR("Port %u: physical ETHA init failed (err %d)", port_id, ret);
		return ret;
	}

	if (phy_dev) {
		rswitch3_wire_physical_port(dev, port_id, phy_dev, pcs_dev);
	}

	return 0;
}

#define RSWITCH3_INIT_PHYSICAL_PORT(n)                                                             \
	if (ret == 0) {                                                                            \
		ret = rswitch3_init_physical_port(dev, DT_REG_ADDR(n), DT_PROP(n, phy_mode),       \
						  RSWITCH3_DT_DEV_OR_NULL(n, phy_handle),          \
						  RSWITCH3_DT_DEV_OR_NULL(n, xpcs));               \
	}

#define RSWITCH3_INIT_TSNES_PORT(n)                                                                \
	if (ret == 0) {                                                                            \
		ret = rswitch3_init_tsnes_port(                                                    \
			dev, DT_REG_ADDR(n), DEVICE_DT_GET(DT_PHANDLE(n, renesas_tsnes)),          \
			(const uint8_t[NET_ETH_ADDR_LEN])DT_PROP_OR(DT_PHANDLE(n, renesas_tsnes),  \
								    local_mac_address, {0}),       \
			DT_PROP_OR(n, renesas_forward_ports, 0));                                  \
	}

/*
 * A port node is either external (phy-mode + phy-handle, optional xpcs) or
 * internal (renesas,tsnes); see renesas,rcarsoc-rswitch3-port.yaml.
 */
#define RSWITCH3_CHECK_PORT_EXTERNAL(n)                                                            \
	BUILD_ASSERT(DT_NODE_HAS_PROP(n, phy_mode) && DT_NODE_HAS_PROP(n, phy_handle),             \
		     DT_NODE_FULL_NAME(n) ": external port needs phy-mode and phy-handle");
#define RSWITCH3_CHECK_PORT_INTERNAL(n)                                                            \
	BUILD_ASSERT(!DT_NODE_HAS_PROP(n, xpcs) && !DT_NODE_HAS_PROP(n, phy_handle) &&             \
			     !DT_NODE_HAS_PROP(n, phy_mode),                                       \
		     DT_NODE_FULL_NAME(n) ": renesas,tsnes port must not set xpcs, phy-handle "    \
					  "or phy-mode");
#define RSWITCH3_CHECK_PORT(n)                                                                     \
	COND_CODE_1(DT_NODE_HAS_PROP(n, renesas_tsnes), (RSWITCH3_CHECK_PORT_INTERNAL(n)),         \
		    (RSWITCH3_CHECK_PORT_EXTERNAL(n)))

#define RSWITCH3_INIT_PORT_EXTERNAL(n)                                                             \
	COND_CODE_1(DT_NODE_HAS_PROP(n, renesas_tsnes), (), (RSWITCH3_INIT_PHYSICAL_PORT(n)))
#define RSWITCH3_INIT_PORT_INTERNAL(n)                                                             \
	COND_CODE_1(DT_NODE_HAS_PROP(n, renesas_tsnes), (RSWITCH3_INIT_TSNES_PORT(n)), ())

static int rswitch3_init_common(const struct device *dev)
{
	const struct rswitch3_config *config = dev->config;
	struct rswitch3_data *data = dev->data;
	int ret;

	ret = clock_control_on(config->clock_dev, RCAR_CLOCK_SUBSYS(config->mod_clk));
	if (ret < 0) {
		LOG_ERR("Failed to enable RSwitch3 clock (err %d)", ret);
		return ret;
	}

	k_mutex_init(&data->lock);

	if (IS_ENABLED(CONFIG_NET_MGMT_EVENT)) {
		net_mgmt_init_event_callback(&data->iface_event_cb, rswitch3_iface_event_handler,
					     NET_EVENT_IF_ADMIN_UP);
		net_mgmt_add_event_callback(&data->iface_event_cb);
	}

	ret = rswitch3_coma_init(dev);
	if (ret) {
		return ret;
	}

	ret = rswitch3_fwd_init(dev);
	if (ret) {
		return ret;
	}

#if defined(CONFIG_ETH_RENESAS_RSWITCH3_GWCA)
	ret = rswitch3_gwca_hw_init(dev);
	if (ret) {
		return ret;
	}
#endif

	return 0;
}

#if defined(CONFIG_ETH_RENESAS_RSWITCH3_GWCA)

#define RSWITCH3_GWCA_NUM_ROWS(inst) ARRAY_SIZE(rswitch3_gwca_port_ids_##inst)

/* Which GWCA sub-instance (0=GWCA0, 1=GWCA1) this instance's
 * renesas,gwca-instance devicetree property selects
 */
#define RSWITCH3_GWCA_INSTANCE(inst) DT_INST_PROP_OR(inst, renesas_gwca_instance, 0)

#define RSWITCH3_GWCA_PORT_ID_ENTRY(n)                                                             \
	COND_CODE_1(DT_NODE_HAS_PROP(n, renesas_tsnes), (), (DT_REG_ADDR(n),))

#define RSWITCH3_GWCA_INIT_CONFIG(inst)                                                            \
	static const uint8_t rswitch3_gwca_port_ids_##inst[] = {                                   \
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst), RSWITCH3_GWCA_PORT_ID_ENTRY)};     \
	static struct rswitch3_gwca_tx_queue                                                       \
		rswitch3_gwca_tx_queues_##inst[RSWITCH3_GWCA_NUM_ROWS(inst)];                      \
	static struct rswitch3_gwca_rx_queue                                                       \
		rswitch3_gwca_rx_queues_##inst[RSWITCH3_GWCA_NUM_ROWS(inst)];                      \
	static struct rswitch3_gwca_tx_desc rswitch3_gwca_tx_rings_##inst[RSWITCH3_GWCA_NUM_ROWS(  \
		inst)][RSWITCH3_GWCA_TX_RING_SIZE + 1] RSWITCH3_GWCA_RAM_SECTION                   \
		__aligned(RSWITCH3_GWCA_DESC_ALIGN);                                               \
	static uint8_t rswitch3_gwca_tx_bufs_##inst                                                \
		[RSWITCH3_GWCA_NUM_ROWS(inst)][RSWITCH3_GWCA_TX_RING_SIZE]                         \
		[RSWITCH3_GWCA_FRAME_BUF_SIZE] RSWITCH3_GWCA_RAM_SECTION                           \
		__aligned(RSWITCH3_GWCA_DESC_ALIGN);                                               \
	static struct rswitch3_gwca_rx_desc rswitch3_gwca_rx_rings_##inst[RSWITCH3_GWCA_NUM_ROWS(  \
		inst)][RSWITCH3_GWCA_RX_RING_SIZE + 1] RSWITCH3_GWCA_RAM_SECTION                   \
		__aligned(RSWITCH3_GWCA_DESC_ALIGN);                                               \
	static uint8_t rswitch3_gwca_rx_bufs_##inst                                                \
		[RSWITCH3_GWCA_NUM_ROWS(inst)][RSWITCH3_GWCA_RX_RING_SIZE]                         \
		[RSWITCH3_GWCA_FRAME_BUF_SIZE] RSWITCH3_GWCA_RAM_SECTION                           \
		__aligned(RSWITCH3_GWCA_DESC_ALIGN);                                               \
	static struct rswitch3_gwca_desc                                                           \
		rswitch3_gwca_linkfix_table_##inst[2 * RSWITCH3_GWCA_NUM_ROWS(                     \
							       inst)] RSWITCH3_GWCA_RAM_SECTION    \
		__aligned(RSWITCH3_GWCA_DESC_ALIGN)

#define RSWITCH3_GWCA_CONFIG_INIT(inst)                                                            \
	.gwca = {                                                                                  \
		.tx_rings = (struct rswitch3_gwca_tx_desc *)rswitch3_gwca_tx_rings_##inst,         \
		.tx_bufs = (uint8_t *)rswitch3_gwca_tx_bufs_##inst,                                \
		.rx_rings = (struct rswitch3_gwca_rx_desc *)rswitch3_gwca_rx_rings_##inst,         \
		.rx_bufs = (uint8_t *)rswitch3_gwca_rx_bufs_##inst,                                \
		.linkfix_table = rswitch3_gwca_linkfix_table_##inst,                               \
		.tx_queues = rswitch3_gwca_tx_queues_##inst,                                       \
		.rx_queues = rswitch3_gwca_rx_queues_##inst,                                       \
		.port_ids = rswitch3_gwca_port_ids_##inst,                                         \
		.num_ports = RSWITCH3_GWCA_NUM_ROWS(inst),                                         \
		.reg_base = RSWITCH3_GWCA_INSTANCE(inst) ? RSWITCH3_GWCA1_OFFSET                   \
							 : RSWITCH3_GWCA0_OFFSET,                  \
		.agent = RSWITCH3_GWCA_INSTANCE(inst) ? (RSWITCH3_NUM_AGENTS - 1)                  \
						      : (RSWITCH3_NUM_AGENTS - 2),                 \
		.instance = RSWITCH3_GWCA_INSTANCE(inst),                                          \
	},
#else
#define RSWITCH3_GWCA_INIT_CONFIG(inst)
#define RSWITCH3_GWCA_CONFIG_INIT(inst)
#define RSWITCH3_GWCA_IRQ_INIT(inst)
#endif /* CONFIG_ETH_RENESAS_RSWITCH3_GWCA */

#define RSWITCH3_INIT(inst)                                                                        \
	DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst), RSWITCH3_CHECK_PORT)                       \
	RSWITCH3_GWCA_INIT_CONFIG(inst);                                                           \
                                                                                                   \
	static struct rswitch3_data rswitch3_data_##inst;                                          \
                                                                                                   \
	static const struct rswitch3_config rswitch3_config_##inst = {                             \
		.base = DT_INST_REG_ADDR_BY_NAME(inst, base),                                      \
		.secure_base = DT_INST_REG_ADDR_BY_NAME(inst, secure_base),                        \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_IDX(inst, 0)),                   \
		.mod_clk = RCAR_DT_INST_CLOCKS_CELL_BY_IDX(inst, 0),                               \
		RSWITCH3_GWCA_CONFIG_INIT(inst)};                                                  \
                                                                                                   \
	static int rswitch3_init_##inst(const struct device *dev)                                  \
	{                                                                                          \
		int ret;                                                                           \
                                                                                                   \
		ret = rswitch3_init_common(dev);                                                   \
		if (ret) {                                                                         \
			return ret;                                                                \
		}                                                                                  \
                                                                                                   \
		RSWITCH3_GWCA_IRQ_INIT(inst);                                                      \
                                                                                                   \
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst), RSWITCH3_INIT_PORT_EXTERNAL)       \
		DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst), RSWITCH3_INIT_PORT_INTERNAL)       \
		if (ret) {                                                                         \
			return ret;                                                                \
		}                                                                                  \
                                                                                                   \
		LOG_INF("Renesas R-Switch 3 initialized");                                         \
                                                                                                   \
		return 0;                                                                          \
	}                                                                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &rswitch3_init_##inst, NULL, &rswitch3_data_##inst,            \
			      &rswitch3_config_##inst, POST_KERNEL,                                \
			      CONFIG_ETH_RENESAS_RSWITCH3_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(RSWITCH3_INIT)

#undef RSWITCH3_INIT
#undef RSWITCH3_GWCA_INIT_CONFIG
#undef RSWITCH3_GWCA_CONFIG_INIT
#undef RSWITCH3_INIT_PHYSICAL_PORT
#undef RSWITCH3_INIT_TSNES_PORT
#undef RSWITCH3_INIT_PORT_EXTERNAL
#undef RSWITCH3_INIT_PORT_INTERNAL
#undef RSWITCH3_CHECK_PORT
#undef RSWITCH3_CHECK_PORT_EXTERNAL
#undef RSWITCH3_CHECK_PORT_INTERNAL
#undef RSWITCH3_GWCA_IRQ_INIT
#if defined(CONFIG_ETH_RENESAS_RSWITCH3_GWCA)
#undef RSWITCH3_GWCA_NUM_ROWS
#undef RSWITCH3_GWCA_INSTANCE
#undef RSWITCH3_GWCA_PORT_ID_ENTRY
#endif

#undef DT_DRV_COMPAT

#if defined(CONFIG_ETH_RENESAS_RSWITCH3_PORT)
/*
 * Per-port net_if driver.
 */
#define DT_DRV_COMPAT renesas_rcarsoc_rswitch3_port

#define RSWITCH3_PORT_MAC_ADDR_LEN 6

struct rswitch3_port_config {
	const struct device *rswitch_dev;
	uint32_t port_id;
	uint8_t mac_addr[RSWITCH3_PORT_MAC_ADDR_LEN];
	const struct device *phy_dev;
};

struct rswitch3_port_data {
	struct net_if *iface;
	bool is_started;
	uint8_t tx_frame_buf[NET_ETH_MAX_FRAME_SIZE];
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif
};

static void rswitch3_port_rx_cb(const struct device *rswitch_dev, uint32_t port_id, uint8_t *data,
				size_t len, void *user_data)
{
	const struct device *dev = user_data;
	struct rswitch3_port_data *port_data = dev->data;
	struct net_pkt *pkt;

	ARG_UNUSED(rswitch_dev);
	ARG_UNUSED(port_id);

	if (!port_data->iface) {
		/* Net L2 iface_init() hasn't run yet; nothing to deliver to. */
		return;
	}

	if (!port_data->is_started) {
		return;
	}

	pkt = net_pkt_rx_alloc_with_buffer(port_data->iface, len, AF_UNSPEC, 0, K_NO_WAIT);
	if (!pkt) {
		LOG_DBG("Port %u: net_pkt alloc failed, len %u", port_id, (unsigned int)len);
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
		port_data->stats.error_details.rx_no_buffer_count++;
#endif
		return;
	}

	if (net_pkt_write(pkt, data, len)) {
		net_pkt_unref(pkt);
		LOG_DBG("Port %u: net_pkt_write failed, len %u", port_id, (unsigned int)len);
		return;
	}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	port_data->stats.pkts.rx++;
	port_data->stats.bytes.received += len;
#endif

	net_recv_data(port_data->iface, pkt);
}

#if defined(CONFIG_NET_CHECKSUM_OFFLOAD)
/*
 * Checksums offloaded to hardware: IPv4 header, and TCP, UDP and ICMP over
 * IPv4. Without ETHERNET_CHECKSUM_SUPPORT_IPV6_HEADER the stack computes and
 * verifies TCP/UDP checksums over IPv6 itself: L4CC on an IPv6 frame with
 * extension headers is a GWCA format error, and the stack cannot leave only
 * those frames to software.
 */
#define RSWITCH3_PORT_CHKSUM_SUPPORT                                                               \
	(ETHERNET_CHECKSUM_SUPPORT_IPV4_HEADER | ETHERNET_CHECKSUM_SUPPORT_IPV4_ICMP |             \
	 ETHERNET_CHECKSUM_SUPPORT_TCP | ETHERNET_CHECKSUM_SUPPORT_UDP)

/*
 * Pick the L3CC/L4CC descriptor flags for an outgoing frame. Must cover every
 * frame type in RSWITCH3_PORT_CHKSUM_SUPPORT, because the stack leaves those
 * checksum fields zero. L3CC on a non-IPv4 frame, or L4CC on an IPv4 fragment,
 * is a GWCA format error (the flags get silently dropped), so those cases are
 * filtered out here.
 */
static uint32_t rswitch3_port_tx_chksum_flags(const uint8_t *frame, size_t len)
{
	size_t off = 2 * NET_ETH_ADDR_LEN;
	uint16_t type;
	uint8_t ihl;
	uint8_t proto;

	if (len < off + sizeof(uint16_t)) {
		return 0;
	}

	type = sys_get_be16(&frame[off]);
	off += sizeof(uint16_t);

	if (type == NET_ETH_PTYPE_VLAN) {
		if (len < off + 4) {
			return 0;
		}
		type = sys_get_be16(&frame[off + 2]);
		off += 4;
	}

	if (type == NET_ETH_PTYPE_IP) {
		if (len < off + sizeof(struct net_ipv4_hdr) || (frame[off] >> 4) != 4) {
			return 0;
		}

		ihl = (frame[off] & 0x0f) * 4;
		if (ihl < sizeof(struct net_ipv4_hdr) || len < off + ihl) {
			return 0;
		}

		/* Fragment (MF set or offset != 0): the stack already did the L4 checksum */
		if (sys_get_be16(&frame[off + 6]) & 0x3fff) {
			return RSWITCH3_GWCA_META_L3CC;
		}

		proto = frame[off + 9];
		if (proto == NET_IPPROTO_TCP || proto == NET_IPPROTO_UDP ||
		    proto == NET_IPPROTO_ICMP) {
			return RSWITCH3_GWCA_META_L3CC | RSWITCH3_GWCA_META_L4CC;
		}

		return RSWITCH3_GWCA_META_L3CC;
	}

	return 0;
}
#endif /* CONFIG_NET_CHECKSUM_OFFLOAD */

static int rswitch3_port_send(const struct device *dev, struct net_pkt *pkt)
{
	const struct rswitch3_port_config *cfg = dev->config;
	struct rswitch3_port_data *data = dev->data;
	size_t len = net_pkt_get_len(pkt);
	uint32_t tx_meta = 0;
	int ret;

	if (!data->is_started) {
		return -ENETDOWN;
	}

	if (len > sizeof(data->tx_frame_buf)) {
		return -EMSGSIZE;
	}

	if (net_pkt_read(pkt, data->tx_frame_buf, len)) {
		return -EIO;
	}

#if defined(CONFIG_NET_CHECKSUM_OFFLOAD)
	tx_meta = rswitch3_port_tx_chksum_flags(data->tx_frame_buf, len);
#endif

	/* cfg->rswitch_dev is always this same driver's own rswitch3 parent
	 * device (the only compatible a renesas,rcarsoc-rswitch3-port node
	 * can have)
	 */
	ret = rswitch3_gwca_port_send(cfg->rswitch_dev, cfg->port_id, data->tx_frame_buf, len,
				      tx_meta);
	if (ret) {
		LOG_DBG("Port %u: send failed (err %d)", cfg->port_id, ret);
		return ret;
	}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	data->stats.pkts.tx++;
	data->stats.bytes.sent += len;
#endif

	return 0;
}

static bool rswitch3_port_link_is_up(const struct device *rswitch_dev, uint32_t port_id)
{
	struct rswitch3_data *rswitch_data = rswitch_dev->data;

	return (rswitch_data->port_link_mask & BIT(port_id)) != 0U;
}

static void rswitch3_port_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	const struct rswitch3_port_config *cfg = dev->config;
	struct rswitch3_port_data *data = dev->data;

	data->iface = iface;

	net_if_set_link_addr(iface, (uint8_t *)cfg->mac_addr, RSWITCH3_PORT_MAC_ADDR_LEN,
			     NET_LINK_ETHERNET);

	ethernet_init(iface);

	/*
	 * Link changes reported before ethernet_init() were not applied to this
	 * iface (see rswitch3_set_carrier()), so take the current state here.
	 */
	if (rswitch3_port_link_is_up(cfg->rswitch_dev, cfg->port_id)) {
		net_eth_carrier_on(iface);
	} else {
		net_if_carrier_off(iface);
	}
}

static enum ethernet_hw_caps rswitch3_port_get_capabilities(const struct device *dev)
{
	enum ethernet_hw_caps caps;

	ARG_UNUSED(dev);

	/* Matches the speeds rswitch3_set_link_state() actually programs into MPIC. */
	caps = ETHERNET_LINK_100BASE | ETHERNET_LINK_1000BASE | ETHERNET_LINK_2500BASE;

	if (IS_ENABLED(CONFIG_NET_CHECKSUM_OFFLOAD)) {
		/* TX: GWCA direct descriptor L3CC/L4CC. RX: ETHA EACKSC ingress check. */
		caps |= ETHERNET_HW_TX_CHKSUM_OFFLOAD | ETHERNET_HW_RX_CHKSUM_OFFLOAD;
	}

	return caps;
}

static int rswitch3_port_get_config(const struct device *dev, enum ethernet_config_type type,
				    struct ethernet_config *config)
{
	ARG_UNUSED(dev);

	switch (type) {
#if defined(CONFIG_NET_CHECKSUM_OFFLOAD)
	case ETHERNET_CONFIG_TYPE_RX_CHECKSUM_SUPPORT:
	case ETHERNET_CONFIG_TYPE_TX_CHECKSUM_SUPPORT:
		/* IPv6 checksums are left to software in both directions. */
		config->chksum_support = RSWITCH3_PORT_CHKSUM_SUPPORT;
		return 0;
#endif
	default:
		return -ENOTSUP;
	}
}

static int rswitch3_port_start(const struct device *dev)
{
	struct rswitch3_port_data *data = dev->data;

	data->is_started = true;

	return 0;
}

static int rswitch3_port_stop(const struct device *dev)
{
	struct rswitch3_port_data *data = dev->data;

	data->is_started = false;

	return 0;
}

static const struct device *rswitch3_port_get_phy(const struct device *dev)
{
	const struct rswitch3_port_config *cfg = dev->config;

	return cfg->phy_dev;
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *rswitch3_port_get_stats(const struct device *dev)
{
	const struct rswitch3_port_config *cfg = dev->config;
	struct rswitch3_port_data *data = dev->data;

	if (IS_ENABLED(CONFIG_NET_CHECKSUM_OFFLOAD)) {
		/* Frames the switch dropped on a bad checksum never reach rx_cb. */
		data->stats.errors.rx +=
			rswitch3_etha_cksum_err_read(cfg->rswitch_dev, cfg->port_id);
	}

	return &data->stats;
}
#endif

static const struct ethernet_api rswitch3_port_api = {
	.iface_api.init = rswitch3_port_iface_init,
	.get_capabilities = rswitch3_port_get_capabilities,
	.get_config = rswitch3_port_get_config,
	.send = rswitch3_port_send,
	.get_phy = rswitch3_port_get_phy,
	.start = rswitch3_port_start,
	.stop = rswitch3_port_stop,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = rswitch3_port_get_stats,
#endif
};

static int rswitch3_port_init(const struct device *dev)
{
	const struct rswitch3_port_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->rswitch_dev)) {
		LOG_ERR("Port %u: parent R-Switch device not ready", cfg->port_id);
		return -ENODEV;
	}

	ret = rswitch3_gwca_attach_port(cfg->rswitch_dev, cfg->port_id, rswitch3_port_rx_cb,
					(void *)dev);
	if (ret) {
		LOG_ERR("Port %u: attach_port failed (err %d)", cfg->port_id, ret);
		return ret;
	}

	LOG_INF("Port %u: attached to GWCA", cfg->port_id);

	return 0;
}

/*
 * Only external rswitch3-port instances.
 */
#define RSWITCH3_PORT_INIT_EXTERNAL(n)                                                             \
	static struct rswitch3_port_data rswitch3_port_data_##n;                                   \
	static const struct rswitch3_port_config rswitch3_port_config_##n = {                      \
		.rswitch_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                   \
		.port_id = DT_INST_REG_ADDR(n),                                                    \
		.mac_addr = DT_INST_PROP_OR(n, local_mac_address, {0}),                            \
		.phy_dev = RSWITCH3_DT_DEV_OR_NULL(DT_DRV_INST(n), phy_handle),                    \
	};                                                                                         \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, rswitch3_port_init, NULL, &rswitch3_port_data_##n,        \
				      &rswitch3_port_config_##n, CONFIG_ETH_INIT_PRIORITY,         \
				      &rswitch3_port_api, NET_ETH_MTU);

#define RSWITCH3_PORT_INIT(n)                                                                      \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, renesas_tsnes), (), (RSWITCH3_PORT_INIT_EXTERNAL(n)))

DT_INST_FOREACH_STATUS_OKAY(RSWITCH3_PORT_INIT)

#undef RSWITCH3_PORT_INIT
#undef RSWITCH3_PORT_INIT_EXTERNAL
#undef DT_DRV_COMPAT
#endif /* CONFIG_ETH_RENESAS_RSWITCH3_PORT */
