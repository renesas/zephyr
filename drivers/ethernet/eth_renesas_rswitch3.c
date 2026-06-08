/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include "eth_renesas_rswitch3.h"

LOG_MODULE_REGISTER(eth_rswitch3, CONFIG_ETHERNET_LOG_LEVEL);

#define DT_DRV_COMPAT renesas_rcarsoc_rswitch3

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
}

static void rswitch3_enable_tsn_forwarding(const struct device *dev, uint32_t tsnes_id,
					   uint32_t rswitch_port, uint32_t fwd_mask)
{
	uint32_t external_ports = fwd_mask & GENMASK(7, 0);
	int i;

	if (!external_ports) {
		LOG_ERR("TSN-ES%u has no external ports to forward to.", tsnes_id);
		return;
	}

	/* Allow traffic from TSN-ES port to external ports */
	rswitch3_fwd_allow_ports(dev, rswitch_port, external_ports);

	/* Allow traffic from external ports back to the TSN-ES port */
	for (i = 0; i < 8; i++) {
		if (!(external_ports & BIT(i))) {
			continue;
		}
		rswitch3_fwd_allow_ports(dev, i, BIT(rswitch_port));
	}

	LOG_INF("Configured forwarding between TSN-ES%u (port %u) and external ports mask 0x%x",
		tsnes_id, rswitch_port, external_ports);
}

static void rswitch3_fwd_disallow_ports(const struct device *dev, uint32_t src_port,
					uint32_t dst_mask)
{
	if (!dst_mask || src_port >= RSWITCH3_NUM_AGENTS) {
		return;
	}

	dst_mask &= GENMASK(RSWITCH3_NUM_AGENTS - 1, 0);

	/* Revoke port-based forwarding towards dst_mask */
	rswitch3_modify(dev, FWPBFC(src_port), FIELD_PREP(FWPBFC_PBDV, dst_mask), 0);
}

static void rswitch3_disable_tsn_forwarding_port(const struct device *dev, uint32_t tsnes_id,
						 uint32_t rswitch_port, uint32_t ext_port_id)
{
	rswitch3_fwd_disallow_ports(dev, rswitch_port, BIT(ext_port_id));
	rswitch3_fwd_disallow_ports(dev, ext_port_id, BIT(rswitch_port));

	LOG_INF("Revoked forwarding between TSN-ES%u (port %u) and external port %u (link down)",
		tsnes_id, rswitch_port, ext_port_id);
}

/*
 * Opens or closes MFWD port-based forwarding between every attached TSN-ES
 * and one external port, according to whether that port currently has a
 * real PHY link.
 */
static void rswitch3_update_forwarding_for_link(const struct device *dev, uint32_t port_id,
						bool is_up)
{
	struct rswitch3_data *data = dev->data;
	int tsnes_id;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (is_up) {
		data->port_link_mask |= BIT(port_id);
	} else {
		data->port_link_mask &= ~BIT(port_id);
	}

	for (tsnes_id = 0; tsnes_id < RSWITCH3_NUM_TSNES; tsnes_id++) {
		if (!(data->tsnes_attached & BIT(tsnes_id))) {
			continue;
		}
		if (!(data->tsnes_fwd_mask[tsnes_id] & BIT(port_id))) {
			continue;
		}

		if (is_up) {
			rswitch3_enable_tsn_forwarding(
				dev, tsnes_id, data->tsnes_rswitch_port[tsnes_id], BIT(port_id));
		} else {
			rswitch3_disable_tsn_forwarding_port(
				dev, tsnes_id, data->tsnes_rswitch_port[tsnes_id], port_id);
		}

		if (data->tsnes_link_cb[tsnes_id]) {
			data->tsnes_link_cb[tsnes_id](dev, tsnes_id, is_up,
						      data->tsnes_link_cb_data[tsnes_id]);
		}
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

struct rswitch3_phys_port_ctx {
	const struct device *rswitch_dev;
	const struct device *pcs_dev;
	uint32_t port_id;
};

static struct rswitch3_phys_port_ctx rswitch3_phys_ports[RSWITCH3_NUM_AGENTS];

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
}

static void rswitch3_wire_physical_port(const struct device *dev, uint32_t port_id,
					const struct device *phy_dev, const struct device *pcs_dev)
{
	struct rswitch3_phys_port_ctx *ctx = &rswitch3_phys_ports[port_id];

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

static int rswitch3_attach_tsnes(const struct device *dev, uint32_t tsnes_id, uint32_t rswitch_port,
				 uint32_t fwd_mask, const uint8_t *mac_addr,
				 rswitch_fwd_link_cb_t link_cb, void *user_data)
{
	struct rswitch3_data *data = dev->data;
	uint32_t ready_ports;
	int ret;

	if (tsnes_id >= RSWITCH3_NUM_TSNES) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->tsnes_attached & BIT(tsnes_id)) {
		ret = -EBUSY;
		goto out_unlock;
	}

	/* Initialize the internal ETHA port connected to the TSN-ES */
	ret = rswitch3_etha_hw_init_tsn_internal(dev, rswitch_port, mac_addr);
	if (ret) {
		LOG_ERR("Failed to init internal ETHA for TSN-ES%u", tsnes_id);
		goto out_unlock;
	}

	LOG_INF("attach tsnes device mac address: %02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0],
		mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

	data->tsnes_attached |= BIT(tsnes_id);
	data->tsnes_rswitch_port[tsnes_id] = rswitch_port;
	data->tsnes_fwd_mask[tsnes_id] = fwd_mask;
	data->tsnes_link_cb[tsnes_id] = link_cb;
	data->tsnes_link_cb_data[tsnes_id] = user_data;

	/*
	 * Only open forwarding to external ports that already have a
	 * confirmed physical link; any port still down stays closed until
	 * rswitch3_phys_port_link_changed() sees it come up. See
	 * rswitch3_update_forwarding_for_link() for why forwarding into a
	 * down port must never be opened early.
	 */
	ready_ports = fwd_mask & data->port_link_mask;
	if (ready_ports) {
		rswitch3_enable_tsn_forwarding(dev, tsnes_id, rswitch_port, ready_ports);
	}

	/*
	 * Notify the caller of the forwarded port's current link state right
	 * away too -- if it was already up when this TSN-ES attached, waiting
	 * for the next link CHANGE event would never happen on an already
	 * stable link, leaving the net_if carrier permanently off.
	 */
	if (link_cb && ready_ports) {
		link_cb(dev, tsnes_id, true, user_data);
	}

out_unlock:
	k_mutex_unlock(&data->lock);
	return ret;
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

static int rswitch3_init(const struct device *dev)
{
	struct rswitch3_data *data = dev->data;
	int ret;

	k_mutex_init(&data->lock);

	ret = rswitch3_coma_init(dev);
	if (ret) {
		return ret;
	}

	ret = rswitch3_fwd_init(dev);
	if (ret) {
		return ret;
	}

	/* clang-format off */
/*
 * Initialize physical ports that are defined in the device tree.
 * This is necessary for TSN-ES forwarding to work.
 */
#define INIT_PHYSICAL_PORT(n)                                                    \
	if (ret == 0) {                                                           \
		ret = rswitch3_etha_hw_init_physical(dev, DT_REG_ADDR(n),        \
						      DT_PROP(n, phy_mode));      \
		if (ret) {                                                        \
			LOG_ERR("Port %u: physical ETHA init failed (err %d)",   \
				DT_REG_ADDR(n), ret);                             \
		} else {                                                          \
			IF_ENABLED(DT_NODE_HAS_PROP(n, phy_handle),               \
				(rswitch3_wire_physical_port(                     \
					dev, DT_REG_ADDR(n),                      \
					DEVICE_DT_GET(DT_PHANDLE(n, phy_handle)), \
					COND_CODE_1(DT_NODE_HAS_PROP(n, xpcs),    \
						(DEVICE_DT_GET(                   \
							DT_PHANDLE(n, xpcs))),    \
						(NULL))                           \
				);)                                                \
			)                                                         \
		}                                                                  \
	}
	/* clang-format on */

	DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(0), INIT_PHYSICAL_PORT)

#undef INIT_PHYSICAL_PORT

	if (ret) {
		return ret;
	}

	LOG_INF("Renesas R-Switch 3 initialized");

	return 0;
}

static const struct rswitch_driver_api rswitch3_api = {
	.attach_tsnes = rswitch3_attach_tsnes,
	.set_link_state = rswitch3_set_link_state,
};

static struct rswitch3_data rswitch3_data_0;

static const struct rswitch3_config rswitch3_config_0 = {
	.base = DT_INST_REG_ADDR_BY_NAME(0, base),
	.secure_base = DT_INST_REG_ADDR_BY_NAME(0, secure_base),
};

DEVICE_DT_INST_DEFINE(0, &rswitch3_init, NULL, &rswitch3_data_0, &rswitch3_config_0, POST_KERNEL,
		      CONFIG_ETH_RENESAS_RSWITCH3_INIT_PRIORITY, &rswitch3_api);

#undef DT_DRV_COMPAT
