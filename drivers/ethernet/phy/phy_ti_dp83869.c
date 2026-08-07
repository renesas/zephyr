/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_dp83869

#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/dt-bindings/ethernet/phy/ti_dp83869.h>

#define LOG_MODULE_NAME phy_ti_dp83869
#define LOG_LEVEL       CONFIG_PHY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL);

#include "phy_mii.h"

#define TI_DP83869_SLEEP_US   1000
#define TI_DP83869_TIMEOUT_US 200000

/* Time the PHY needs before it accepts SMI accesses again after a software reset */
#define TI_DP83869_RESET_TO_SMI_US 30

#define DP83869_SUPPORTED_SPEEDS                                                                   \
	(LINK_HALF_10BASE | LINK_FULL_10BASE | LINK_HALF_100BASE | LINK_FULL_100BASE |             \
	 LINK_HALF_1000BASE | LINK_FULL_1000BASE)

/* MMD Access Control register fields */
#define DP83869_MMD_DEVAD 0x1fU

/* Registers */
#define DP83869_PHYCTRL 0x10
#define DP83869_PHYSTS  0x11
#define DP83869_CTRL    0x1f

/* Extended Registers */
#define DP83869_RGMIICTL    0x0032
#define DP83869_STRAP_STS1  0x006e
#define DP83869_RGMIIDCTL   0x0086
#define DP83869_OP_MODE_REG 0x01df
#define DP83869_FX_CTRL     0x0c00

/* CTRL bits */
#define DP83869_CTRL_SW_RESET   BIT(15)
#define DP83869_CTRL_SW_RESTART BIT(14)

/* PHYSTS bits */
#define DP83869_PHYSTS_SPEED_MASK  GENMASK(15, 14)
#define DP83869_PHYSTS_FULL_DUPLEX BIT(13)
#define DP83869_PHYSTS_LINK_UP     BIT(10)

#define DP83869_PHYSTS_SPEED_10M   0U
#define DP83869_PHYSTS_SPEED_100M  1U
#define DP83869_PHYSTS_SPEED_1000M 2U

/* PHYCTRL bits */
#define DP83869_PHYCTRL_TX_FIFO_DEPTH_MASK GENMASK(15, 14)
#define DP83869_PHYCTRL_RX_FIFO_DEPTH_MASK GENMASK(13, 12)
#define DP83869_PHY_CTRL_DEFAULT (BIT(6) | BIT(3))

/* STRAP_STS1 bits */
#define DP83869_STRAP_STS1_OP_MODE_MASK GENMASK(11, 9)

/* RGMIICTL bits */
#define DP83869_RGMIICTL_TX_CLK_DELAY_DISABLE BIT(1)
#define DP83869_RGMIICTL_RX_CLK_DELAY_DISABLE BIT(0)

/* RGMIIDCTL bits */
#define DP83869_RGMIIDCTL_TX_DELAY_MASK GENMASK(7, 4)
#define DP83869_RGMIIDCTL_RX_DELAY_MASK GENMASK(3, 0)
#define DP83869_RGMII_DELAY_STEP_PS  250U
#define DP83869_RGMII_DELAY_MAX_CODE 0x0fU
#define DP83869_RGMII_DELAY_UNSET    UINT16_MAX

/* BMCR bits */
#define DP83869_BMCR_SETUP (MII_BMCR_AUTONEG_ENABLE | MII_BMCR_DUPLEX_MODE | MII_BMCR_SPEED_1000)

/* GEN_CFG1 bits */
#define DP83869_1KTCR_MASTER_SLAVE_VALUE BIT(11)
#define DP83869_1KTCR_SETUP                                                                        \
	(MII_ADVERTISE_1000_HALF | MII_ADVERTISE_1000_FULL | DP83869_1KTCR_MASTER_SLAVE_VALUE)

#define DP83869_OP_MODE_USE_STRAP 0xffU

struct ti_dp83869_config {
	const struct device *mdio_dev;
	const uint8_t phy_addr;
	const uint8_t tx_fifo_depth;
	const uint8_t rx_fifo_depth;
	const uint16_t tx_internal_delay_ps;
	const uint16_t rx_internal_delay_ps;
	enum phy_link_speed default_speeds;
};

struct ti_dp83869_data {
	const struct device *dev;
	struct k_mutex mutex;
	struct phy_link_state state;
	phy_callback_t cb;
	void *cb_data;
	struct k_work_delayable phy_monitor_work;
	uint8_t op_mode;
};

/**
 * @brief Read from PHY register using C22 access
 */
static int phy_ti_dp83869_read(const struct device *dev, uint16_t reg_addr, uint32_t *data)
{
	const struct ti_dp83869_config *config = dev->config;
	uint16_t value;
	int ret;

	ret = mdio_read(config->mdio_dev, config->phy_addr, reg_addr, &value);
	if (ret < 0) {
		return ret;
	}

	*data = value;

	return 0;
}

/**
 * @brief Write to PHY register using C22 access
 */
static int phy_ti_dp83869_write(const struct device *dev, uint16_t reg_addr, uint32_t data)
{
	const struct ti_dp83869_config *config = dev->config;

	return mdio_write(config->mdio_dev, config->phy_addr, reg_addr, (uint16_t)data);
}

/**
 * @brief Point the MMD access registers at an extended register
 *
 * Leaves the access control register in the given data access mode, so that the
 * next access to the MMD address data register operates on @p reg.
 */
static int phy_ti_dp83869_mmd_select(const struct device *dev, uint16_t reg, uint16_t data_mode)
{
	int ret;

	ret = phy_ti_dp83869_write(dev, MII_MMD_ACR,
				   MII_MMD_ACR_ADDR | (DP83869_MMD_DEVAD & MII_MMD_ACR_DEVAD_MASK));
	if (ret < 0) {
		return ret;
	}

	ret = phy_ti_dp83869_write(dev, MII_MMD_AADR, reg);
	if (ret < 0) {
		return ret;
	}

	return phy_ti_dp83869_write(dev, MII_MMD_ACR,
				    data_mode | (DP83869_MMD_DEVAD & MII_MMD_ACR_DEVAD_MASK));
}

/**
 * @brief Write to PHY extended register using C22 indirect method
 */
static int phy_ti_dp83869_write_mmd(const struct device *dev, uint16_t reg, uint16_t val)
{
	int ret;

	ret = phy_ti_dp83869_mmd_select(dev, reg, MII_MMD_ACR_DATA_NO_POS_INC);
	if (ret < 0) {
		return ret;
	}

	return phy_ti_dp83869_write(dev, MII_MMD_AADR, val);
}

/**
 * @brief Read from PHY extended register using C22 indirect method
 */
static int phy_ti_dp83869_read_mmd(const struct device *dev, uint16_t reg, uint16_t *val)
{
	uint32_t data;
	int ret;

	ret = phy_ti_dp83869_mmd_select(dev, reg, MII_MMD_ACR_DATA_NO_POS_INC);
	if (ret < 0) {
		return ret;
	}

	ret = phy_ti_dp83869_read(dev, MII_MMD_AADR, &data);
	if (ret < 0) {
		return ret;
	}

	*val = (uint16_t)data;

	return 0;
}

/**
 * @brief Get link state
 */
static int phy_ti_dp83869_get_link(const struct device *dev, struct phy_link_state *state)
{
	struct ti_dp83869_data *data = dev->data;
	int ret;
	uint32_t physr = 0;
	bool full_duplex;

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret < 0) {
		LOG_ERR("PHY mutex lock error");
		return ret;
	}

	/* Read PHY status register */
	ret = phy_ti_dp83869_read(dev, DP83869_PHYSTS, &physr);

	/* Unlock mutex */
	(void)k_mutex_unlock(&data->mutex);

	if (ret < 0) {
		LOG_ERR("Unable to read PHY status register");
		return ret;
	}

	state->is_up = (physr & DP83869_PHYSTS_LINK_UP) != 0U;

	/* If state is down, also clear the speed */
	if (!state->is_up) {
		state->speed = 0;
		return 0;
	}

	/* Get speed */
	full_duplex = (physr & DP83869_PHYSTS_FULL_DUPLEX) != 0U;
	switch (FIELD_GET(DP83869_PHYSTS_SPEED_MASK, physr)) {
	case DP83869_PHYSTS_SPEED_1000M:
		if (full_duplex) {
			state->speed = LINK_FULL_1000BASE;
		} else {
			state->speed = LINK_HALF_1000BASE;
		}
		break;
	case DP83869_PHYSTS_SPEED_100M:
		if (full_duplex) {
			state->speed = LINK_FULL_100BASE;
		} else {
			state->speed = LINK_HALF_100BASE;
		}
		break;
	case DP83869_PHYSTS_SPEED_10M:
		if (full_duplex) {
			state->speed = LINK_FULL_10BASE;
		} else {
			state->speed = LINK_HALF_10BASE;
		}
		break;
	default:
		LOG_ERR("Invalid link speed 0x%08x",
			(uint32_t)FIELD_GET(DP83869_PHYSTS_SPEED_MASK, physr));
		state->is_up = false;
		state->speed = 0;
		return -EIO;
	}

	return 0;
}

/**
 * @brief Set callback to be invoked when link state changes. Driver has to invoke callback
 * once after setting it, even if link state has not changed.
 */
static int phy_ti_dp83869_link_cb_set(const struct device *dev, phy_callback_t cb, void *user_data)
{
	struct ti_dp83869_data *data = dev->data;
	int ret;

	data->cb = cb;
	data->cb_data = user_data;

	/* Immediately get current status and invoke the callback to notify the caller */
	if (data->cb) {
		ret = phy_ti_dp83869_get_link(dev, &data->state);
		if (ret < 0) {
			LOG_WRN("Initial callback link read failed: %d", ret);
			return ret;
		}

		data->cb(dev, &data->state, data->cb_data);
	} else {
		LOG_ERR("Callback must be provided!");
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief Configure PHY link speed via auto-negotiation. Disabling auto-negotiation is not supported
 * by this driver.
 */
static int phy_ti_dp83869_cfg_link(const struct device *dev, enum phy_link_speed speeds,
				   enum phy_cfg_link_flag flags)
{
	struct ti_dp83869_data *data = dev->data;
	int ret;

	if (flags & PHY_FLAG_AUTO_NEGOTIATION_DISABLED) {
		LOG_ERR("Disabling auto-negotiation is not supported by this driver");
		return -ENOTSUP;
	}

	if (speeds == 0) {
		LOG_ERR("At least one speed must be specified");
		return -EINVAL;
	}

	if ((speeds & ~DP83869_SUPPORTED_SPEEDS) != 0U) {
		LOG_ERR("Unsupported speed(s) specified: 0x%08x",
			(uint32_t)(speeds & ~DP83869_SUPPORTED_SPEEDS));
		return -ENOTSUP;
	}

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret < 0) {
		LOG_ERR("PHY mutex lock error");
		return ret;
	}

	/* Stop monitoring while the PHY is being reconfigured */
	k_work_cancel_delayable(&data->phy_monitor_work);

	/* Configure advertisement speeds and restart autoneg */
	ret = phy_mii_cfg_link_autoneg(dev, speeds, true);

	/* Unlock mutex */
	(void)k_mutex_unlock(&data->mutex);
	k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));

	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Error configuring PHY link speeds: %d", ret);
	}

	return ret;
}

/**
 * @brief Monitor PHY link state and invoke callback if it changes
 */
static void phy_ti_dp83869_monitor_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct ti_dp83869_data *data =
		CONTAINER_OF(dwork, struct ti_dp83869_data, phy_monitor_work);
	const struct device *dev = data->dev;
	const struct ti_dp83869_config *config = dev->config;
	struct phy_link_state state = {0};
	int ret;

	ret = phy_ti_dp83869_get_link(dev, &state);
	if (ret < 0) {
		LOG_ERR("Error reading PHY link state");
		goto reschedule;
	}

	/* If the link state has changed, update link state and invoke callback */
	if ((state.is_up != data->state.is_up) || (state.speed != data->state.speed)) {
		data->state = state;
		if (data->cb) {
			LOG_DBG("The phy@%d link state has changed", config->phy_addr);
			data->cb(dev, &data->state, data->cb_data);
		}
	}

reschedule:
	/* Continue monitoring */
	k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
}

/**
 * @brief Restart the PHY. This action resets all the PHY circuits except the registers
 */
static int phy_ti_dp83869_sw_restart(const struct device *dev)
{
	uint32_t ctrl;
	bool completed;
	int ret;

	ret = phy_ti_dp83869_write(dev, DP83869_CTRL, DP83869_CTRL_SW_RESTART);

	if (ret < 0) {
		return ret;
	}

	/* Wait for the self-clearing restart bit. */
	completed = WAIT_FOR(((ret = phy_ti_dp83869_read(dev, DP83869_CTRL, &ctrl)) == 0) &&
				     ((ctrl & DP83869_CTRL_SW_RESTART) == 0U),
			     TI_DP83869_TIMEOUT_US, k_usleep(TI_DP83869_SLEEP_US));
	if (ret < 0) {
		return ret;
	}

	if (!completed) {
		LOG_ERR("Failed to perform PHY DP83869 software restart");
		return -ETIMEDOUT;
	}

	return 0;
}

/**
 * @brief Reset the PHY. This action resets all the PHY circuits as well as registers to their
 * default values.
 */
static int phy_ti_dp83869_sw_reset(const struct device *dev)
{
	int ret;

	ret = phy_ti_dp83869_write(dev, DP83869_CTRL, DP83869_CTRL_SW_RESET);
	if (ret < 0) {
		LOG_ERR("Failed to perform PHY DP83869 software reset");
		return ret;
	}

	/* Reset to SMI ready time */
	k_busy_wait(TI_DP83869_RESET_TO_SMI_US);

	return 0;
}

/**
 * @brief Convert RGMII delay in picoseconds to the corresponding register value
 */
static uint16_t phy_ti_dp83869_rgmii_delay_code(uint16_t delay_ps)
{
	if (delay_ps == 0U) {
		return DP83869_RGMII_DELAY_MAX_CODE;
	}

	return (delay_ps / DP83869_RGMII_DELAY_STEP_PS) - 1U;
}

/**
 * @brief Configure the PHY RGMII TX and RX delays
 */
static int phy_ti_dp83869_configure_rgmii_delay(const struct device *dev)
{
	const struct ti_dp83869_config *config = dev->config;
	uint16_t rgmii_delay_ctl;
	uint16_t rgmii_ctl;
	int ret;

	if (config->tx_internal_delay_ps == DP83869_RGMII_DELAY_UNSET &&
	    config->rx_internal_delay_ps == DP83869_RGMII_DELAY_UNSET) {
		return 0;
	}

	ret = phy_ti_dp83869_read_mmd(dev, DP83869_RGMIIDCTL, &rgmii_delay_ctl);
	if (ret < 0) {
		return ret;
	}

	ret = phy_ti_dp83869_read_mmd(dev, DP83869_RGMIICTL, &rgmii_ctl);
	if (ret < 0) {
		return ret;
	}

	if (config->tx_internal_delay_ps != DP83869_RGMII_DELAY_UNSET) {
		rgmii_delay_ctl &= ~DP83869_RGMIIDCTL_TX_DELAY_MASK;
		rgmii_delay_ctl |=
			FIELD_PREP(DP83869_RGMIIDCTL_TX_DELAY_MASK,
				   phy_ti_dp83869_rgmii_delay_code(config->tx_internal_delay_ps));
		if (config->tx_internal_delay_ps == 0U) {
			rgmii_ctl |= DP83869_RGMIICTL_TX_CLK_DELAY_DISABLE;
		} else {
			rgmii_ctl &= ~DP83869_RGMIICTL_TX_CLK_DELAY_DISABLE;
		}
	}

	if (config->rx_internal_delay_ps != DP83869_RGMII_DELAY_UNSET) {
		rgmii_delay_ctl &= ~DP83869_RGMIIDCTL_RX_DELAY_MASK;
		rgmii_delay_ctl |=
			FIELD_PREP(DP83869_RGMIIDCTL_RX_DELAY_MASK,
				   phy_ti_dp83869_rgmii_delay_code(config->rx_internal_delay_ps));
		if (config->rx_internal_delay_ps == 0U) {
			rgmii_ctl |= DP83869_RGMIICTL_RX_CLK_DELAY_DISABLE;
		} else {
			rgmii_ctl &= ~DP83869_RGMIICTL_RX_CLK_DELAY_DISABLE;
		}
	}

	ret = phy_ti_dp83869_write_mmd(dev, DP83869_RGMIIDCTL, rgmii_delay_ctl);
	if (ret < 0) {
		return ret;
	}

	return phy_ti_dp83869_write_mmd(dev, DP83869_RGMIICTL, rgmii_ctl);
}

/**
 * @brief Configure the PHY operation mode
 */
static int phy_ti_dp83869_configure_mode(const struct device *dev)
{
	const struct ti_dp83869_config *config = dev->config;
	struct ti_dp83869_data *data = dev->data;
	uint16_t val;
	int ret;

	if (data->op_mode == DP83869_OP_MODE_USE_STRAP) {
		ret = phy_ti_dp83869_read_mmd(dev, DP83869_STRAP_STS1, &val);
		if (ret < 0) {
			return ret;
		}

		data->op_mode = FIELD_GET(DP83869_STRAP_STS1_OP_MODE_MASK, val);
	}

	/* Only these modes are currently implemented. */
	if (data->op_mode != DP83869_OP_MODE_RGMII_COPPER_ETHERNET &&
	    data->op_mode != DP83869_OP_MODE_SGMII_COPPER_ETHERNET) {
		LOG_ERR("Unsupported operation mode %u", data->op_mode);
		return -ENOTSUP;
	}

	/* Set operation mode */
	ret = phy_ti_dp83869_write_mmd(dev, DP83869_OP_MODE_REG, data->op_mode);
	if (ret < 0) {
		return ret;
	}

	/* Reset BMCR */
	ret = phy_ti_dp83869_write(dev, MII_BMCR, DP83869_BMCR_SETUP);
	if (ret < 0) {
		return ret;
	}

	/* Set GEN_CFG1 */
	ret = phy_ti_dp83869_write(dev, MII_1KTCR, DP83869_1KTCR_SETUP);
	if (ret < 0) {
		return ret;
	}

	/* Configure PHYCTRL */
	val = FIELD_PREP(DP83869_PHYCTRL_RX_FIFO_DEPTH_MASK, config->rx_fifo_depth) |
	      FIELD_PREP(DP83869_PHYCTRL_TX_FIFO_DEPTH_MASK, config->tx_fifo_depth) |
	      DP83869_PHY_CTRL_DEFAULT;
	/* Set PHYCTRL */
	ret = phy_ti_dp83869_write(dev, DP83869_PHYCTRL, val);
	if (ret < 0) {
		return ret;
	}

	if (data->op_mode == DP83869_OP_MODE_RGMII_COPPER_ETHERNET) {
		ret = phy_ti_dp83869_configure_rgmii_delay(dev);
	} else {
		/* Set FX_CTRL */
		ret = phy_ti_dp83869_write_mmd(dev, DP83869_FX_CTRL, DP83869_BMCR_SETUP);
	}

	if (ret < 0) {
		return ret;
	}

	/* Let the PHY pick up the new configuration */
	return phy_ti_dp83869_sw_restart(dev);
}

static int phy_ti_dp83869_init(const struct device *dev)
{
	const struct ti_dp83869_config *config = dev->config;
	struct ti_dp83869_data *data = dev->data;
	int ret;

	if (!device_is_ready(config->mdio_dev)) {
		LOG_ERR("MDIO bus device is not ready");
		return -ENODEV;
	}

	ret = k_mutex_init(&data->mutex);
	if (ret < 0) {
		return ret;
	}

	mdio_bus_enable(config->mdio_dev);

	/* Software reset */
	ret = phy_ti_dp83869_sw_reset(dev);
	if (ret < 0) {
		return ret;
	}

	/* Called during initialization before link monitoring starts */
	ret = phy_ti_dp83869_configure_mode(dev);
	if (ret < 0) {
		LOG_ERR("Failed to init PHY addr (%d)", config->phy_addr);
		return ret;
	}

	k_work_init_delayable(&data->phy_monitor_work, phy_ti_dp83869_monitor_work_handler);

	/* Advertise default speeds */
	ret = phy_ti_dp83869_cfg_link(dev, config->default_speeds, 0);
	if (ret < 0 && ret != -EALREADY) {
		k_work_cancel_delayable(&data->phy_monitor_work);
		LOG_ERR("Failed to configure default advertised speeds: %d", ret);
		return ret;
	}

	LOG_INF("PHY TI DP83869 initialized");

	return 0;
}

static DEVICE_API(ethphy, ti_dp83869_phy_api) = {
	.get_link = phy_ti_dp83869_get_link,
	.cfg_link = phy_ti_dp83869_cfg_link,
	.link_cb_set = phy_ti_dp83869_link_cb_set,
	.read = phy_ti_dp83869_read,
	.write = phy_ti_dp83869_write,
};

/**
 * ************************* DRIVER REGISTER SECTION ***************************
 */

#define TI_DP83869_INIT(idx)                                                                       \
                                                                                                   \
	static const struct ti_dp83869_config ti_dp83869_##idx##_config = {                        \
		.mdio_dev = DEVICE_DT_GET(DT_INST_BUS(idx)),                                       \
		.phy_addr = DT_INST_REG_ADDR(idx),                                                 \
		.tx_fifo_depth =                                                                   \
			DT_INST_ENUM_IDX_OR(idx, ti_tx_fifo_depth, DP83869_FIFO_DEPTH_4B_NIB),     \
		.rx_fifo_depth =                                                                   \
			DT_INST_ENUM_IDX_OR(idx, ti_rx_fifo_depth, DP83869_FIFO_DEPTH_4B_NIB),     \
		.tx_internal_delay_ps =                                                            \
			DT_INST_PROP_OR(idx, ti_tx_internal_delay_ps, DP83869_RGMII_DELAY_UNSET),  \
		.rx_internal_delay_ps =                                                            \
			DT_INST_PROP_OR(idx, ti_rx_internal_delay_ps, DP83869_RGMII_DELAY_UNSET),  \
		.default_speeds = PHY_INST_GENERATE_DEFAULT_SPEEDS(idx),                           \
	};                                                                                         \
                                                                                                   \
	static struct ti_dp83869_data ti_dp83869_##idx##_data = {                                  \
		.dev = DEVICE_DT_INST_GET(idx),                                                    \
		.op_mode = DT_INST_PROP_OR(idx, ti_op_mode, DP83869_OP_MODE_USE_STRAP),            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, &phy_ti_dp83869_init, NULL, &ti_dp83869_##idx##_data,           \
			      &ti_dp83869_##idx##_config, POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,   \
			      &ti_dp83869_phy_api);

DT_INST_FOREACH_STATUS_OKAY(TI_DP83869_INIT);
