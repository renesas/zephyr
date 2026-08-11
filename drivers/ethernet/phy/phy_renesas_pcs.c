/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_ether_pcs_channel

#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/net/phy.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control/renesas_clkc_mdlc.h>
#include <zephyr/drivers/misc/renesas_rcar_mp_phy/renesas_rcar_mp_phy.h>

LOG_MODULE_REGISTER(phy_renesas_pcs, CONFIG_PHY_LOG_LEVEL);

#define R8A78000_ETH_PCS_OFFSET           0x0400
#define R8A78000_ETH_PCS_BANK_SELECT      0x03fc
#define R8A78000_ETH_PCS_NUM_RETRY_LINKUP 8

struct pcs_channel_config {
	mm_reg_t chan_base;
	const struct device *mpphy_dev;
	const struct device *clock_dev;
	struct rcar_clkc mod_clk;
	struct mp_phy_renesas_rcar_cfg phy_cfg;
	uint8_t channel_id;
	bool usxgmii; /* false: SGMII, true: USXGMII */
};

struct pcs_channel_data {
	struct k_mutex lock;
	struct phy_link_state state;
	phy_callback_t cb;
	void *cb_data;
	bool ram_initialized;
	bool aneg_on;
};

static void pcs_write32(const struct device *dev, uint32_t offs, uint32_t bank, uint32_t data)
{
	const struct pcs_channel_config *config = dev->config;

	sys_write32(bank, config->chan_base + R8A78000_ETH_PCS_BANK_SELECT);
	sys_write32(data, config->chan_base + offs);
}

static int pcs_reg_wait(const struct device *dev, uint32_t offs, uint32_t bank, uint32_t mask,
			uint32_t expected)
{
	const struct pcs_channel_config *config = dev->config;
	uint32_t val;
	bool ok;

	sys_write32(bank, config->chan_base + R8A78000_ETH_PCS_BANK_SELECT);

	/* WAIT_FOR() returns the (truthy) expr on success, false on timeout. */
	ok = WAIT_FOR((val = sys_read32(config->chan_base + offs), (val & mask) == expected), 30000,
		      k_busy_wait(1000));

	if (!ok) {
		LOG_ERR("PCS ch%u: timeout offs 0x%x bank 0x%x mask 0x%x expected 0x%x",
			config->channel_id, offs, bank, mask, expected);
	}

	return ok ? 0 : -ETIMEDOUT;
}

static bool pcs_link_status_bit(const struct device *dev)
{
	const struct pcs_channel_config *config = dev->config;

	sys_write32(0x300, config->chan_base + R8A78000_ETH_PCS_BANK_SELECT);

	return !!(sys_read32(config->chan_base + 0x0004) & BIT(2));
}

static int pcs_init_ram(const struct device *dev)
{
	const struct pcs_channel_config *config = dev->config;
	int ret;

	ret = pcs_reg_wait(dev, 0x026c, 0x180, BIT(0), 1);

	if (ret) {
		LOG_ERR(" pcs_reg_wait(dev, 0x026c, 0x180, BIT(0), BIT(0)) failed");
		return ret;
	}

	pcs_write32(dev, 0x026c, 0x180, 0x03);

	ret = mp_phy_renesas_rcar_enable(config->mpphy_dev, config->phy_cfg);
	k_busy_wait(1100);
	if (ret) {
		return ret;
	}

	return pcs_reg_wait(dev, 0x0000, 0x300, BIT(15), 0);
}

static int pcs_common_setting(const struct device *dev)
{
	const struct pcs_channel_config *config = dev->config;
	int ret;

	if (!config->usxgmii) {
		pcs_write32(dev, 0x001c, 0x300, 0x0001);
		pcs_write32(dev, 0x0000, 0x380, 0x2000);
		pcs_write32(dev, 0x0000, 0x1f00, 0x0140);
		pcs_write32(dev, 0x0258, 0x180, 0x0018);
		pcs_write32(dev, 0x01dc, 0x180, 0x000d);
		pcs_write32(dev, 0x00f8, 0x180, 0x0016);
		pcs_write32(dev, 0x0248, 0x180, 0x0016);

		pcs_write32(dev, 0x0000, 0x300, 0x0c40);
		ret = pcs_reg_wait(dev, 0x0040, 0x380, GENMASK(4, 2), 0x06 << 2);
		if (ret) {
			return ret;
		}

		pcs_write32(dev, 0x0000, 0x300, 0x0440);
		ret = pcs_reg_wait(dev, 0x0040, 0x380, GENMASK(4, 2), 0x04 << 2);
		if (ret) {
			return ret;
		}

		pcs_write32(dev, 0x0014, 0x380, 0x0050);
		pcs_write32(dev, 0x00d8, 0x180, 0x3000);
		pcs_write32(dev, 0x00dc, 0x180, 0x0000);

		return 0;
	}

	pcs_write32(dev, 0x001c, 0x300, 0x0000);
	pcs_write32(dev, 0x001c, 0x380, 0x0000);
	pcs_write32(dev, 0x0000, 0x380, 0x2200);
	pcs_write32(dev, 0x00f8, 0x180, 0x0022);
	pcs_write32(dev, 0x0248, 0x180, 0x0028);
	pcs_write32(dev, 0x0000, 0x300, 0x0c40);

	ret = pcs_reg_wait(dev, 0x0040, 0x380, GENMASK(4, 2), 0x06 << 2);
	if (ret) {
		return ret;
	}

	pcs_write32(dev, 0x0000, 0x300, 0x0440);
	ret = pcs_reg_wait(dev, 0x0040, 0x380, GENMASK(4, 2), 0x04 << 2);
	if (ret) {
		return ret;
	}

	pcs_write32(dev, 0x0014, 0x380, 0x0050);
	pcs_write32(dev, 0x00d8, 0x180, 0x1800);
	pcs_write32(dev, 0x00dc, 0x180, 0x0012);

	ret = pcs_reg_wait(dev, 0x0080, 0x180, BIT(12), BIT(12));
	if (ret) {
		return ret;
	}

	pcs_write32(dev, 0x0170, 0x180, 0x1000);
	ret = pcs_reg_wait(dev, 0x0260, 0x180, BIT(12), BIT(12));
	if (ret) {
		return ret;
	}

	pcs_write32(dev, 0x0170, 0x180, 0x0000);

	return 0;
}

static int pcs_chan_setting(const struct device *dev)
{
	struct pcs_channel_data *data = dev->data;
	const struct pcs_channel_config *config = dev->config;
	int ret;

	if (!data->aneg_on) {
		return 0;
	}

	if (!config->usxgmii) {
		pcs_write32(dev, 0x0004, 0x1f80, 0x0005);
		pcs_write32(dev, 0x0000, 0x1f80, 0x2200);
		pcs_write32(dev, 0x0000, 0x1f00, 0x3140);

		k_busy_wait(1100);

		ret = pcs_reg_wait(dev, 0x0008, 0x1f80, BIT(0), 0x01);
		if (ret) {
			return ret;
		}

		pcs_write32(dev, 0x0008, 0x1f80, 0x0000);

		return 0;
	}

	pcs_write32(dev, 0x0004, 0x1f80, 0x0001);
	pcs_write32(dev, 0x0028, 0x1f80, 0x0001);
	pcs_write32(dev, 0x0000, 0x1f80, 0x2008);
	pcs_write32(dev, 0x0000, 0x1f00, 0x3140);

	ret = pcs_reg_wait(dev, 0x0008, 0x1f80, BIT(0), 0x01);
	if (ret) {
		return ret;
	}

	pcs_write32(dev, 0x0008, 0x1f80, 0x0000);

	return 0;
}

static int pcs_chan_speed(const struct device *dev, uint32_t speed_mbps)
{
	const struct pcs_channel_config *config = dev->config;

	if (!config->usxgmii) {
		return 0;
	}

	if (speed_mbps == 10000) {
		pcs_write32(dev, 0x0000, 0x1f00, 0x2140);
	} else if (speed_mbps == 2500) {
		pcs_write32(dev, 0x0000, 0x1f00, 0x0120);
	} else {
		LOG_ERR("PCS ch%u: unsupported USXGMII rate %u Mbps", config->channel_id,
			speed_mbps);
		return -ENOTSUP;
	}

	pcs_write32(dev, 0x0000, 0x380, 0x2600);

	return pcs_reg_wait(dev, 0x0000, 0x380, BIT(10), 0);
}

static int pcs_monitor_linkup(const struct device *dev)
{
	int ret = -ETIMEDOUT;

	for (int i = 0; i < R8A78000_ETH_PCS_NUM_RETRY_LINKUP; i++) {
		ret = pcs_reg_wait(dev, 0x0004, 0x300, BIT(2), BIT(2));
		if (!ret) {
			break;
		}

		pcs_write32(dev, 0x0144, 0x180, 0x0010);
		k_busy_wait(1);
		pcs_write32(dev, 0x0144, 0x180, 0x0000);
	}

	return ret;
}

static void pcs_notify_link_cb(const struct device *dev)
{
	struct pcs_channel_data *data = dev->data;

	if (data->cb != NULL) {
		data->cb(dev, &data->state, data->cb_data);
	}
}

/*
 * Brings the SERDES lane up for the requested speed. Callers must disable
 * auto-negotiation and request a single speed grade (this internal lane is
 * configured to a fixed operating point, it does not negotiate over the wire).
 */
static int pcs_channel_cfg_link(const struct device *dev, enum phy_link_speed speeds,
				enum phy_cfg_link_flag flags)
{
	struct pcs_channel_data *data = dev->data;
	uint32_t speed_mbps;
	int ret = 0;

	switch (speeds) {
	case LINK_FULL_2500BASE:
		speed_mbps = 2500;
		break;
	case LINK_FULL_1000BASE:
		speed_mbps = 1000;
		break;
	case LINK_FULL_100BASE:
		speed_mbps = 100;
		break;
	case LINK_FULL_10BASE:
		speed_mbps = 10;
		break;
	default:
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	data->aneg_on = !(flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED);

	ret = pcs_chan_setting(dev);
	if (ret) {
		LOG_ERR("failed to pcs_chan_setting");
		goto unlock;
	}

	ret = pcs_chan_speed(dev, speed_mbps);
	if (ret) {
		LOG_ERR("failed to pcs_chan_speed");
		goto unlock;
	}

	ret = pcs_monitor_linkup(dev);
	if (ret) {
		LOG_ERR("failed to pcs_monitor_linkup");
		goto unlock;
	}

	data->state.speed = speeds;
	data->state.is_up = true;

unlock:
	k_mutex_unlock(&data->lock);

	pcs_notify_link_cb(dev);

	return ret;
}

static int pcs_channel_get_link(const struct device *dev, struct phy_link_state *state)
{
	struct pcs_channel_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->state.is_up = data->ram_initialized && pcs_link_status_bit(dev);
	*state = data->state;
	k_mutex_unlock(&data->lock);

	return 0;
}

static int pcs_channel_link_cb_set(const struct device *dev, phy_callback_t cb, void *user_data)
{
	struct pcs_channel_data *data = dev->data;

	data->cb = cb;
	data->cb_data = user_data;

	pcs_channel_get_link(dev, &data->state);
	pcs_notify_link_cb(dev);

	return 0;
}

static int pcs_channel_init(const struct device *dev)
{
	struct pcs_channel_data *data = dev->data;
	const struct pcs_channel_config *config = dev->config;
	int ret = 0;

	if (!device_is_ready(config->mpphy_dev)) {
		LOG_ERR("MP-PHY device not ready");
		return -ENODEV;
	}

	k_mutex_init(&data->lock);
	data->state.is_up = false;
	data->state.speed = 0;
	data->aneg_on = true;

	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->mod_clk);
	if (ret < 0) {
		return ret;
	}

	k_busy_wait(CONFIG_PHY_RENESAS_PCS_TIMEOUT_US);

	ret = pcs_init_ram(dev);
	if (ret) {
		LOG_ERR("failed to init RAM");
		return ret;
	}

	ret = pcs_common_setting(dev);
	if (ret) {
		LOG_ERR("failed to pcs_common_setting");
		return ret;
	}

	data->ram_initialized = true;

	LOG_INF("Renesas PCS Channel %u initialized", config->channel_id);

	return 0;
}

static DEVICE_API(ethphy, pcs_channel_api) = {
	.cfg_link = pcs_channel_cfg_link,
	.get_link = pcs_channel_get_link,
	.link_cb_set = pcs_channel_link_cb_set,
};

#define PCS_CHANNEL_INIT(n)                                                                        \
	static struct pcs_channel_data pcs_data_##n;                                               \
	static const struct pcs_channel_config pcs_config_##n = {                                  \
		.chan_base = DT_REG_ADDR(DT_INST_PARENT(n)) +                                      \
			     R8A78000_ETH_PCS_OFFSET * DT_INST_REG_ADDR(n),                        \
		.mpphy_dev = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(n), phys)),                      \
		.channel_id = DT_INST_REG_ADDR(n),                                                 \
		.usxgmii = DT_INST_ENUM_HAS_VALUE(n, phy_mode, usxgmii),                           \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.mod_clk.module = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module),                        \
		.mod_clk.domain = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain),                        \
		.phy_cfg =                                                                         \
			{                                                                          \
				.channel = DT_PHA_BY_IDX(DT_DRV_INST(n), phys, 0, channel),        \
				.lane = DT_PHA_BY_IDX(DT_DRV_INST(n), phys, 0, lane),              \
				.type = MP_PHY_TYPE_ETH,                                           \
			},                                                                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, pcs_channel_init, NULL, &pcs_data_##n, &pcs_config_##n,           \
			      POST_KERNEL, CONFIG_PHY_RENESAS_PCS_INIT_PRIORITY,                   \
			      &pcs_channel_api);

DT_INST_FOREACH_STATUS_OKAY(PCS_CHANNEL_INIT)
