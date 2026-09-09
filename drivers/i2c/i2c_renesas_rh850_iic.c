/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_iic

#include <stdint.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include "bsp_api.h"
#include <errno.h>
#include <soc.h>

#ifdef CONFIG_I2C_RENESAS_RH850_IIC_B
#include "r_iic_b_master.h"

#define iic_master_instance_ctrl_t  iic_b_master_instance_ctrl_t
#define iic_master_extended_cfg_t   iic_b_master_extended_cfg_t
#define iic_master_clock_settings_t iic_b_master_clock_settings_t
#define g_i2c_master_on_iic         g_i2c_master_on_iic_b
#define iic_master_rxi_isr          iic_b_master_rxi_isr
#define iic_master_txi_isr          iic_b_master_txi_isr
#define iic_master_tei_isr          iic_b_master_tei_isr
#define iic_master_eri_isr          iic_b_master_eri_isr
#else
#include "r_iic_master.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(renesas_rh850_iic);

typedef void (*init_func_t)(const struct device *dev);

#ifdef CONFIG_I2C_RENESAS_RH850_IIC_B
#define RH850_IIC_B_MIN_PERIOD_COUNT 9U
#define RH850_IIC_B_MAX_PERIOD_COUNT 255U
#define RH850_IIC_B_MAX_BFREC_COUNT  0x1FFU
#define RH850_IIC_B_STANDARD_TBUF_S  0.0000047
#define RH850_IIC_B_FAST_TBUF_S      0.0000013
#endif

struct i2c_rh850_iic_config {
	void (*irq_config_func)(const struct device *dev);
	const i2c_master_api_t *fsp_api;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint32_t noise_filter_stage;
	const double rise_time_s;
	const double fall_time_s;
	uint32_t duty_cycle_percent;
};

struct i2c_rh850_iic_data {
	iic_master_instance_ctrl_t ctrl;
	i2c_master_cfg_t fsp_config;
	struct k_mutex bus_mutex;
	struct k_sem complete_sem;
	i2c_master_event_t event;
	iic_master_extended_cfg_t iic_master_ext_cfg;
	uint32_t dev_config;
	uint32_t input_clock_hz;
	bool is_open;
};

/* Return 0 on success; leave clk_cfg unchanged on failure. */
static int calc_iic_master_clock_setting(const struct device *dev, const uint32_t fsp_i2c_rate,
					  iic_master_clock_settings_t *clk_cfg);

/* FSP interruption handlers. */
void iic_master_rxi_isr(void);
void iic_master_txi_isr(void);
void iic_master_tei_isr(void);
void iic_master_eri_isr(void);

struct rh850_iic_master_bitrate {
	float bitrate;
	float duty;
	uint32_t divider;
	uint32_t brl;
	uint32_t brh;
	float duty_error_percent;
};

static uint32_t ceil_positive_double_to_u32(double value)
{
	uint32_t integer = (uint32_t)value;

	if ((double)integer < value) {
		integer++;
	}

	return integer;
}

#ifdef CONFIG_I2C_RENESAS_RH850_IIC_B
static void i2c_rh850_iic_b_update_extended_cfg(struct i2c_rh850_iic_data *data)
{
	uint32_t i3c_phi_hz =
		data->input_clock_hz >> data->iic_master_ext_cfg.clock_settings.cks_value;
	double bus_free_time_s;

	data->iic_master_ext_cfg.timeout_mode = IIC_B_MASTER_TIMEOUT_MODE_16BIT;
	data->iic_master_ext_cfg.timeout_scl_low = IIC_B_MASTER_TIMEOUT_SCL_LOW_ENABLED;
	data->iic_master_ext_cfg.timeout_scl_high = IIC_B_MASTER_TIMEOUT_SCL_HIGH_ENABLED;
	data->iic_master_ext_cfg.clock_settings.sdod_value = 0U;
	data->iic_master_ext_cfg.clock_settings.sdodcs_value = false;
	data->iic_master_ext_cfg.smbus_operation = false;
	data->iic_master_ext_cfg.iic_clock_freq = data->fsp_config.rate;

	if (data->fsp_config.rate == I2C_MASTER_RATE_STANDARD) {
		bus_free_time_s = RH850_IIC_B_STANDARD_TBUF_S;
	} else {
		bus_free_time_s = RH850_IIC_B_FAST_TBUF_S;
	}

	double bus_free_cycles_raw = bus_free_time_s * (double)i3c_phi_hz;
	uint32_t bus_free_cycles = ceil_positive_double_to_u32(bus_free_cycles_raw);

	if (bus_free_cycles > RH850_IIC_B_MAX_BFREC_COUNT) {
		bus_free_cycles = RH850_IIC_B_MAX_BFREC_COUNT;
	}

	data->iic_master_ext_cfg.bus_free_detection_time = bus_free_cycles;
}
#endif

static int i2c_rh850_iic_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_rh850_iic_config *config = dev->config;
	struct i2c_rh850_iic_data *data = (struct i2c_rh850_iic_data *)dev->data;
	fsp_err_t fsp_err;
	iic_master_clock_settings_t new_clock_settings = {0};
	iic_master_extended_cfg_t old_extended_cfg;
	uint32_t new_rate;
	uint32_t old_rate;
	bool was_open;
	int ret;

	if (!(dev_config & I2C_MODE_CONTROLLER)) {
		LOG_ERR("Only I2C Master mode supported.");
		return -EIO;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		new_rate = I2C_MASTER_RATE_STANDARD;
		break;
	case I2C_SPEED_FAST:
		new_rate = I2C_MASTER_RATE_FAST;
		break;
	case I2C_SPEED_FAST_PLUS:
		new_rate = I2C_MASTER_RATE_FASTPLUS;
		break;
	case I2C_SPEED_HIGH:
		LOG_ERR("%s: High-speed mode is not supported.", __func__);
		return -ENOTSUP;
	default:
		LOG_ERR("%s: Invalid I2C speed rate flag: %d", __func__, I2C_SPEED_GET(dev_config));
		return -EIO;
	}

	k_mutex_lock(&data->bus_mutex, K_FOREVER);

	/* Validate the new timing before changing the active configuration. */
	ret = calc_iic_master_clock_setting(dev, new_rate, &new_clock_settings);
	if (ret < 0) {
		goto unlock;
	}

	old_rate = data->fsp_config.rate;
	old_extended_cfg = data->iic_master_ext_cfg;
	was_open = data->is_open;

	if (was_open) {
		fsp_err = config->fsp_api->close(&data->ctrl);
		if (fsp_err != FSP_SUCCESS) {
			LOG_ERR("%s: Close IIC master failed. FSP_ERR=%d", __func__, fsp_err);
			ret = -EIO;
			goto unlock;
		}
		data->is_open = false;
	}

	data->fsp_config.rate = new_rate;
	data->iic_master_ext_cfg.clock_settings = new_clock_settings;
#ifdef CONFIG_I2C_RENESAS_RH850_IIC_B
	i2c_rh850_iic_b_update_extended_cfg(data);
#endif

	fsp_err = config->fsp_api->open(&data->ctrl, &data->fsp_config);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("%s: Open IIC master failed. FSP_ERR=%d", __func__, fsp_err);
		data->fsp_config.rate = old_rate;
		data->iic_master_ext_cfg = old_extended_cfg;
		if (was_open) {
			fsp_err = config->fsp_api->open(&data->ctrl, &data->fsp_config);
			if (fsp_err == FSP_SUCCESS) {
				data->is_open = true;
			} else {
				LOG_ERR("%s: Restore IIC configuration failed. FSP_ERR=%d",
					__func__, fsp_err);
			}
		}
		ret = -EIO;
		goto unlock;
	}

	data->is_open = true;
	data->dev_config = dev_config;
	ret = 0;

unlock:
	k_mutex_unlock(&data->bus_mutex);
	return ret;
}

static int i2c_rh850_iic_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_rh850_iic_data *data = (struct i2c_rh850_iic_data *)dev->data;
	int ret = 0;

	k_mutex_lock(&data->bus_mutex, K_FOREVER);
	if (data->is_open) {
		*dev_config = data->dev_config;
	} else {
		ret = -EIO;
	}
	k_mutex_unlock(&data->bus_mutex);

	return ret;
}

#define OPERATION(msg) (((struct i2c_msg *)msg)->flags & I2C_MSG_RW_MASK)

static int i2c_rh850_iic_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			       uint16_t addr)
{
	const struct i2c_rh850_iic_config *config = dev->config;
	struct i2c_rh850_iic_data *data = (struct i2c_rh850_iic_data *)dev->data;
	struct i2c_msg *current, *next;
	fsp_err_t fsp_err = FSP_SUCCESS;
	int ret = 0;

	if (!num_msgs) {
		return 0;
	}

	/* Check for validity of all messages before transfer */
	current = msgs;

	/* Set I2C_MSG_RESTART flag on first message in order to send start condition */
	current->flags |= I2C_MSG_RESTART;

	for (int i = 1; i <= num_msgs; i++) {
		if (i < num_msgs) {
			next = current + 1;

			/* Restart condition between messages of different directions is required */
			if (OPERATION(current) != OPERATION(next)) {
				if (!(next->flags & I2C_MSG_RESTART)) {
					LOG_ERR("%s: Restart condition between messages of "
						"different directions is required."
						"Current/Total: [%d/%d]",
						__func__, i, num_msgs);
					ret = -EIO;
					break;
				}
			}

			/* Stop condition is only allowed on last message */
			if (current->flags & I2C_MSG_STOP) {
				LOG_ERR("%s: Invalid stop flag. Stop condition is only allowed on "
					"last message. "
					"Current/Total: [%d/%d]",
					__func__, i, num_msgs);
				ret = -EIO;
				break;
			}
		} else {
			current->flags |= I2C_MSG_STOP;
		}

		current++;
	}

	if (ret) {
		return ret;
	}

	k_mutex_lock(&data->bus_mutex, K_FOREVER);

	if (!data->is_open) {
		ret = -EIO;
		goto RELEASE_BUS;
	}

	/* Set destination address with configured address mode before sending msg. */

	i2c_master_addr_mode_t addr_mode = 0;

	if (data->dev_config & I2C_ADDR_10_BITS) {
		addr_mode = I2C_MASTER_ADDR_MODE_10BIT;
	} else {
		addr_mode = I2C_MASTER_ADDR_MODE_7BIT;
	}

	config->fsp_api->slaveAddressSet(&data->ctrl, addr, addr_mode);

	/* Process input `msgs`. */
	current = msgs;

	while (num_msgs > 0) {
		if (num_msgs > 1) {
			next = current + 1;
		} else {
			next = NULL;
		}

		if (current->flags & I2C_MSG_READ) {
			fsp_err =
				config->fsp_api->read(&data->ctrl, current->buf, current->len,
						  next != NULL && (next->flags & I2C_MSG_RESTART));
		} else {
			fsp_err =
				config->fsp_api->write(&data->ctrl, current->buf, current->len,
						   next != NULL && (next->flags & I2C_MSG_RESTART));
		}

		if (fsp_err != FSP_SUCCESS) {
			switch (fsp_err) {
			case FSP_ERR_INVALID_SIZE:
				LOG_ERR("%s: Provided number of bytes more than uint16_t size "
					"(65535) while DTC is used for data transfer.",
					__func__);
				break;
			case FSP_ERR_IN_USE:
				LOG_ERR("%s: Bus busy condition. Another transfer was in progress.",
					__func__);
				break;
			default:
				/* Should not reach here. */
				LOG_ERR("%s: Unknown error. FSP_ERR=%d\n", __func__, fsp_err);
				break;
			}

			ret = -EIO;
			goto RELEASE_BUS;
		}

		/* Wait for callback to return. */
		k_sem_take(&data->complete_sem, K_FOREVER);

		/* Handle event msg from callback. */
		switch (data->event) {
		case I2C_MASTER_EVENT_ABORTED:
			LOG_ERR("%s: %s failed.", __func__,
				(current->flags & I2C_MSG_READ) ? "Read" : "Write");
			ret = -EIO;
			goto RELEASE_BUS;
		case I2C_MASTER_EVENT_RX_COMPLETE:
			break;
		case I2C_MASTER_EVENT_TX_COMPLETE:
			break;
		default:
			break;
		}

		current++;
		num_msgs--;
	}

RELEASE_BUS:
	k_mutex_unlock(&data->bus_mutex);

	return ret;
}

static void i2c_rh850_iic_callback(i2c_master_callback_args_t *p_args)
{
	const struct device *dev = p_args->p_context;
	struct i2c_rh850_iic_data *data = dev->data;

	data->event = p_args->event;

	k_sem_give(&data->complete_sem);
}

static int i2c_rh850_iic_init(const struct device *dev)
{
	const struct i2c_rh850_iic_config *config = dev->config;
	struct i2c_rh850_iic_data *data = (struct i2c_rh850_iic_data *)dev->data;
	fsp_err_t fsp_err = FSP_SUCCESS;
	uint32_t speed;
	int ret = 0;

	switch (data->fsp_config.rate) {
	case I2C_MASTER_RATE_STANDARD:
		speed = I2C_SPEED_STANDARD;
		break;
	case I2C_MASTER_RATE_FAST:
		speed = I2C_SPEED_FAST;
		break;
	case I2C_MASTER_RATE_FASTPLUS:
		speed = I2C_SPEED_FAST_PLUS;
		break;
	default:
		LOG_ERR("%s: Unsupported I2C speed rate: %u", __func__,
			data->fsp_config.rate);
		return -ENOTSUP;
	}

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret < 0) {
		LOG_ERR("%s: pinctrl config failed.", __func__);
		return ret;
	}

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("%s: Clock controller is not ready.", __func__);
		return -ENODEV;
	}

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret < 0) {
		LOG_ERR("%s: Failed to enable IIC clock: %d", __func__, ret);
		return ret;
	}

	ret = clock_control_get_rate(config->clock_dev, config->clock_subsys,
				     &data->input_clock_hz);
	if (ret < 0) {
		LOG_ERR("%s: Failed to get IIC clock rate: %d", __func__, ret);
		goto err_clock;
	}

	if (data->input_clock_hz == 0U) {
		LOG_ERR("%s: Invalid IIC input clock rate.", __func__);
		ret = -EINVAL;
		goto err_clock;
	}

	LOG_DBG("%s: IIC input clock: %u Hz", __func__, data->input_clock_hz);

	k_mutex_init(&data->bus_mutex);
	k_sem_init(&data->complete_sem, 0, 1);

	ret = calc_iic_master_clock_setting(dev, data->fsp_config.rate,
					  &data->iic_master_ext_cfg.clock_settings);
	if (ret < 0) {
		goto err_clock;
	}

#ifdef CONFIG_I2C_RENESAS_RH850_IIC_B
	i2c_rh850_iic_b_update_extended_cfg(data);
#else
	data->iic_master_ext_cfg.timeout_mode = IIC_MASTER_TIMEOUT_MODE_SHORT;
	data->iic_master_ext_cfg.timeout_scl_low = IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED;
#endif
	data->fsp_config.p_extend = &data->iic_master_ext_cfg;

	fsp_err = config->fsp_api->open(&data->ctrl, &data->fsp_config);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("%s: Open IIC master failed. FSP_ERR=%d", __func__, fsp_err);
		ret = -EIO;
		goto err_clock;
	}

	config->irq_config_func(dev);

	data->dev_config = I2C_MODE_CONTROLLER | I2C_SPEED_SET(speed);
	data->is_open = true;

	return 0;

err_clock:
	if (clock_control_off(config->clock_dev, config->clock_subsys) < 0) {
		LOG_WRN("%s: Failed to disable IIC clock during cleanup.", __func__);
	}

	return ret;
}

#ifndef CONFIG_I2C_RENESAS_RH850_IIC_B
static void calc_iic_master_bitrate(const struct i2c_rh850_iic_config *config,
					uint32_t total_brl_brh,
					uint32_t brh, uint32_t divider,
					uint32_t peripheral_clock,
					struct rh850_iic_master_bitrate *result)
{
	const uint32_t noise_filter_stage = config->noise_filter_stage;
	const double rise_time_s = config->rise_time_s;
	const double fall_time_s = config->fall_time_s;
	const uint32_t requested_duty = config->duty_cycle_percent;

	uint32_t constant_add = 2 + noise_filter_stage;

	if (divider == 0) {
		/* All dividers other than 0 use an addition of 2 + noise_filter_stage. */
		constant_add = 3 + noise_filter_stage;
	}
	/* Use floating-point division when converting clock counts to time. */
	const double divided_pclk = (double)(peripheral_clock >> divider);

	result->bitrate =
		1 / ((total_brl_brh + 2 * constant_add) / divided_pclk + rise_time_s + fall_time_s);
	result->duty =
		100 *
		((rise_time_s + ((brh + constant_add) / divided_pclk)) /
		(rise_time_s + fall_time_s + ((total_brl_brh + 2 * constant_add)) / divided_pclk));
	result->divider = divider;
	result->brh = brh;
	result->brl = total_brl_brh - brh;
	result->duty_error_percent =
		(result->duty > requested_duty ? result->duty - requested_duty
							: requested_duty - result->duty) /
		requested_duty;

	LOG_DBG("%s: [input] total_brl_brh[%u] brh[%u] divider[%u]"
		" [output] bitrate[%u] duty[%u] divider[%u] brh[%u] brl[%u] "
		"duty_error_percent[%u]\n",
		__func__, total_brl_brh, brh, divider, (uint32_t)result->bitrate,
		(uint32_t)result->duty, result->divider, result->brh, result->brl,
		(uint32_t)(result->duty_error_percent * 100.0f));
}

static int calc_iic_master_clock_setting(const struct device *dev, const uint32_t fsp_i2c_rate,
					  iic_master_clock_settings_t *clk_cfg)
{
	const struct i2c_rh850_iic_config *config = dev->config;
	const struct i2c_rh850_iic_data *data = dev->data;
	const uint32_t noise_filter_stage = config->noise_filter_stage;
	const double rise_time_s = config->rise_time_s;
	const double fall_time_s = config->fall_time_s;
	const uint32_t requested_duty = config->duty_cycle_percent;
	const uint32_t peripheral_clock = data->input_clock_hz;

	uint32_t requested_bitrate = 0;

	switch (fsp_i2c_rate) {
	case I2C_MASTER_RATE_STANDARD:
	case I2C_MASTER_RATE_FAST:
	case I2C_MASTER_RATE_FASTPLUS:
		requested_bitrate = fsp_i2c_rate;
		break;
	default:
		LOG_ERR("%s: Invalid I2C speed rate: %d", __func__, fsp_i2c_rate);
		return -ENOTSUP;
	}

	if ((peripheral_clock == 0U) || (requested_duty == 0U) || (requested_duty >= 100U) ||
	    !(rise_time_s >= 0.0) || !(fall_time_s >= 0.0) ||
	    !((rise_time_s + fall_time_s) < (1.0 / (double)requested_bitrate))) {
		LOG_ERR("%s: Invalid IIC clock calculation parameters.", __func__);
		return -EINVAL;
	}

	/* Minimum counts required by the noise filter. */
	uint32_t min_brh = noise_filter_stage + 1;
	uint32_t min_brl_brh = 2 * min_brh;
	struct rh850_iic_master_bitrate bitrate = {};
	bool found = false;

	/* Start with the smallest divider because it gives the most resolution. */
	uint32_t constant_add = 3 + noise_filter_stage;

	for (int temp_divider = 0; temp_divider <= 7; ++temp_divider) {
		if (1 == temp_divider) {
			/* All dividers other than 0 use an addition of 2 + noise_filter_stages.
			 */
			constant_add = 2 + noise_filter_stage;
		}

		/* If the requested bitrate cannot be achieved with this divider, continue.
		 */
		uint32_t divided_pclk = (peripheral_clock >> temp_divider);
		double t_bit = (1 / (double)requested_bitrate);
		double t_time = rise_time_s + fall_time_s;
		double t_total = ((t_bit - t_time)*divided_pclk) - (2*constant_add);

		/* Check the range before converting the floating-point count. */
		if ((divided_pclk == 0U) || (t_total < (double)min_brl_brh) ||
		    (t_total > 62.0)) {
			continue;
		}

		/* Round up so that the SCL period is not shorter than requested. */
		uint32_t total_brl_brh = ceil_positive_double_to_u32(t_total);
		uint32_t temp_brh = total_brl_brh * requested_duty / 100;
		uint32_t min_valid_brh =
			MAX(min_brh, total_brl_brh > 31U ? total_brl_brh - 31U : 0U);
		uint32_t max_valid_brh = MIN(31U, total_brl_brh - min_brh);

		/* Keep both BRH and BRL within the noise-filter and register limits. */
		temp_brh = CLAMP(temp_brh, min_valid_brh, max_valid_brh);

		/* Calculate the actual bitrate and duty cycle. */
		struct rh850_iic_master_bitrate temp_bitrate = {};

		calc_iic_master_bitrate(config, total_brl_brh, temp_brh, temp_divider,
					peripheral_clock, &temp_bitrate);

		/* Adjust duty cycle down if it helps. */
		struct rh850_iic_master_bitrate test_bitrate = temp_bitrate;

		while (test_bitrate.duty > requested_duty) {
			temp_brh -= 1;

			if ((temp_brh < min_brh) || ((total_brl_brh - temp_brh) > 31)) {
				break;
			}

			struct rh850_iic_master_bitrate new_bitrate = {};

			calc_iic_master_bitrate(config, total_brl_brh, temp_brh, temp_divider,
						peripheral_clock, &new_bitrate);

			if (new_bitrate.duty_error_percent < temp_bitrate.duty_error_percent) {
				temp_bitrate = new_bitrate;
			} else {
				break;
			}
		}

		/* Adjust duty cycle up if it helps. */
		while (test_bitrate.duty < requested_duty) {
			temp_brh++;

			if ((temp_brh > total_brl_brh) || (temp_brh > 31) ||
			    ((total_brl_brh - temp_brh) < min_brh)) {
				break;
			}

			struct rh850_iic_master_bitrate new_bitrate = {};

			calc_iic_master_bitrate(config, total_brl_brh, temp_brh, temp_divider,
						peripheral_clock, &new_bitrate);

			if (new_bitrate.duty_error_percent < temp_bitrate.duty_error_percent) {
				temp_bitrate = new_bitrate;
			} else {
				break;
			}
		}

		if ((temp_bitrate.brh >= min_brh) && (temp_bitrate.brl >= min_brh) &&
		    (temp_bitrate.brh <= 31U) && (temp_bitrate.brl <= 31U)) {
			/* Valid setting found. */
			bitrate = temp_bitrate;
			found = true;
			break;
		}
	}

	if (!found) {
		LOG_ERR("%s: Cannot calculate IIC clock setting for rate %u", __func__,
			requested_bitrate);
		return -ERANGE;
	}

	clk_cfg->brl_value = bitrate.brl;
	clk_cfg->brh_value = bitrate.brh;
	clk_cfg->cks_value = bitrate.divider;

	LOG_DBG("%s: [input] rate[%u] [output] brl[%u] brh[%u] cks[%u]\n", __func__, fsp_i2c_rate,
		clk_cfg->brl_value, clk_cfg->brh_value, clk_cfg->cks_value);
	return 0;
}
#else
static int calc_iic_master_clock_setting(const struct device *dev, const uint32_t fsp_i2c_rate,
					  iic_master_clock_settings_t *clk_cfg)
{
	const struct i2c_rh850_iic_config *config = dev->config;
	const struct i2c_rh850_iic_data *data = dev->data;
	const uint32_t requested_duty = config->duty_cycle_percent;
	const double rise_time_s = config->rise_time_s;
	const double fall_time_s = config->fall_time_s;
	uint32_t requested_bitrate;

	switch (fsp_i2c_rate) {
	case I2C_MASTER_RATE_STANDARD:
	case I2C_MASTER_RATE_FAST:
	case I2C_MASTER_RATE_FASTPLUS:
		requested_bitrate = fsp_i2c_rate;
		break;
	default:
		LOG_ERR("%s: Invalid IIC_B speed rate: %u", __func__, fsp_i2c_rate);
		return -ENOTSUP;
	}

	if ((data->input_clock_hz == 0U) || (requested_duty == 0U) ||
	    (requested_duty >= 100U) || !(rise_time_s >= 0.0) || !(fall_time_s >= 0.0) ||
	    !((rise_time_s + fall_time_s) < (1.0 / (double)requested_bitrate))) {
		LOG_ERR("%s: Invalid IIC_B clock calculation parameters.", __func__);
		return -EINVAL;
	}

	for (uint32_t divider = 0U; divider <= 7U; divider++) {
		uint32_t i3c_phi_hz = data->input_clock_hz >> divider;
		double count_period =
			((1.0 / (double)requested_bitrate) - rise_time_s - fall_time_s) *
			(double)i3c_phi_hz;

		if (count_period <= 0.0) {
			continue;
		}

		uint32_t total_count = (uint32_t)ceil_positive_double_to_u32(count_period);
		uint32_t high_count = total_count * requested_duty / 100U;
		uint32_t low_count = total_count - high_count;

		if (high_count < RH850_IIC_B_MIN_PERIOD_COUNT) {
			high_count = RH850_IIC_B_MIN_PERIOD_COUNT;
		}
		if (low_count < RH850_IIC_B_MIN_PERIOD_COUNT) {
			low_count = RH850_IIC_B_MIN_PERIOD_COUNT;
		}

		if ((high_count > RH850_IIC_B_MAX_PERIOD_COUNT) ||
		    (low_count > RH850_IIC_B_MAX_PERIOD_COUNT)) {
			continue;
		}

		clk_cfg->cks_value = (uint8_t)divider;
		clk_cfg->brh_value = (uint8_t)high_count;
		clk_cfg->brl_value = (uint8_t)low_count;
		clk_cfg->sdod_value = 0U;
		clk_cfg->sdodcs_value = false;
		return 0;
	}

	LOG_ERR("%s: Cannot calculate IIC_B clock setting for rate %u", __func__,
		requested_bitrate);
	return -ERANGE;
}
#endif

static DEVICE_API(i2c, i2c_rh850_iic_driver_api) = {
	.configure = i2c_rh850_iic_configure,
	.get_config = i2c_rh850_iic_get_config,
	.transfer = i2c_rh850_iic_transfer,
};

#define EVENT_IIC_RXI(channel) BSP_PRV_IELS_ENUM(CONCAT(EVENT_IIC, channel, _RXI))
#define EVENT_IIC_TXI(channel) BSP_PRV_IELS_ENUM(CONCAT(EVENT_IIC, channel, _TXI))
#define EVENT_IIC_TEI(channel) BSP_PRV_IELS_ENUM(CONCAT(EVENT_IIC, channel, _TEI))
#define EVENT_IIC_ERI(channel) BSP_PRV_IELS_ENUM(CONCAT(EVENT_IIC, channel, _ERI))

#define i2c_rh850_iic_INIT(index)                                                     \
                                                                                      \
	PINCTRL_DT_INST_DEFINE(index);                                                    \
                                                                                      \
	static void i2c_rh850_iic_irq_config_func##index(const struct device *dev)        \
	{                                                                                 \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(index, rxi, irq),                             \
			    DT_INST_IRQ_BY_NAME(index, rxi, priority), iic_master_rxi_isr,        \
			    DEVICE_DT_INST_GET(index), 0);                                        \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(index, txi, irq),                             \
			    DT_INST_IRQ_BY_NAME(index, txi, priority), iic_master_txi_isr,        \
			    DEVICE_DT_INST_GET(index), 0);                                        \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(index, tei, irq),                             \
			    DT_INST_IRQ_BY_NAME(index, tei, priority), iic_master_tei_isr,        \
			    DEVICE_DT_INST_GET(index), 0);                                        \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(index, eri, irq),                             \
			    DT_INST_IRQ_BY_NAME(index, eri, priority), iic_master_eri_isr,        \
			    DEVICE_DT_INST_GET(index), 0);                                        \
                                                                                      \
		irq_enable(DT_INST_IRQ_BY_NAME(index, rxi, irq));                             \
		irq_enable(DT_INST_IRQ_BY_NAME(index, txi, irq));                             \
		irq_enable(DT_INST_IRQ_BY_NAME(index, tei, irq));                             \
		irq_enable(DT_INST_IRQ_BY_NAME(index, eri, irq));                             \
	}                                                                                 \
	static const struct i2c_rh850_iic_config i2c_rh850_iic_config_##index = {              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		.fsp_api = &g_i2c_master_on_iic,                                                   \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(index)),                         \
		.clock_subsys = (clock_control_subsys_t)(uintptr_t)                           \
			DT_INST_CLOCKS_CELL(index, clk_id),                                    \
		.irq_config_func = i2c_rh850_iic_irq_config_func##index,                           \
		.noise_filter_stage = 1, /* Cannot be configured. */                               \
		.rise_time_s = (double)((double)(DT_INST_PROP(index, rise_time_ns))/1000000000.0), \
		.fall_time_s = (double)((double)(DT_INST_PROP(index, fall_time_ns))/1000000000.0), \
		.duty_cycle_percent = DT_INST_PROP(index, duty_cycle_percent),                     \
	};                                                                                     \
	static struct i2c_rh850_iic_data i2c_rh850_iic_data_##index = {                        \
		.fsp_config =                                                                      \
			{                                                                      \
				.channel = DT_INST_PROP(index, channel),                           \
				.slave = 0,                                                        \
				.rate = DT_INST_PROP(index, clock_frequency),                      \
				.addr_mode = I2C_MASTER_ADDR_MODE_7BIT,                            \
				.ipl = DT_INST_PROP(index, interrupt_priority_level),              \
				.rxi_irq = DT_INST_IRQ_BY_NAME(index, rxi, irq),                   \
				.txi_irq = DT_INST_IRQ_BY_NAME(index, txi, irq),                   \
				.tei_irq = DT_INST_IRQ_BY_NAME(index, tei, irq),                   \
				.eri_irq = DT_INST_IRQ_BY_NAME(index, eri, irq),                   \
				.p_callback = i2c_rh850_iic_callback,                              \
				.p_context = (void *)DEVICE_DT_GET(DT_DRV_INST(index)),            \
			},                                                                     \
	};                                                                             \
	I2C_DEVICE_DT_INST_DEFINE(index, i2c_rh850_iic_init,                           \
				NULL, &i2c_rh850_iic_data_##index,                               \
				&i2c_rh850_iic_config_##index, POST_KERNEL,                      \
				CONFIG_I2C_INIT_PRIORITY, &i2c_rh850_iic_driver_api);

DT_INST_FOREACH_STATUS_OKAY(i2c_rh850_iic_INIT)
