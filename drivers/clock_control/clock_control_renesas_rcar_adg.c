/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_adg

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/clock_control/renesas_rcar_adg.h>

LOG_MODULE_REGISTER(clock_control_renesas_rcar_adg, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

/*
 * Helper macro to check if this is ADG clock source output
 *
 * True when the clock identifier addresses a generator directly, see the encoding described in
 * <zephyr/dt-bindings/clock/renesas_rcar_adg.h>.
 */
#define ADG_IS_CLKSRC(SOURCE) ((SOURCE & RCAR_ADG_SOURCECLOCK_MASK) != 0)

/*
 * Helper macro to check if this is ADG clockout pin output
 *
 * True when the clock identifier addresses an AUDIO_CLKOUT pin instead of a generator. Note
 * that RCAR_ADG_AUDIO_NONE satisfies neither this nor ADG_IS_CLKSRC().
 */
#define ADG_IS_CLKOUT(SOURCE) ((SOURCE & RCAR_ADG_CLKOUT_MASK) != 0)

/*
 * Helper macro to calculate BRGx division ratio
 *
 * Total BRGx division as programmed in BRRx: 2 * 4^CKS * (BRR + 1).
 */
#define ADG_BRG_DIV_RATIO(CKS, BRR) (2 * (1U << ((CKS) * 2)) * ((BRR) + 1))

/* Function prototypes, the clock source helpers below call each other across definitions */
static int rcar_adg_clksrc_enable(const struct device *dev, uint32_t clkout_src, bool enable);
static enum clock_control_status rcar_adg_clksrc_get_status(const struct device *dev,
							    uint32_t clksrc);
static uint32_t rcar_adg_clksrc_get_rate(const struct device *dev, uint32_t clksrc);
static int rcar_adg_clksrc_set_rate(const struct device *dev, uint32_t clksrc, uint32_t rate);

static uint32_t rcar_adg_brg_clkin_get_rate(const struct clock_control_renesas_adg_cfg *config,
					    uint8_t brg_unit);
static int rcar_adg_brg_set_rate(const struct device *dev, uint8_t brg_unit, uint32_t rate);

static int rcar_adg_avbcounter8_set_rate(const struct device *dev, uint8_t avbcounter8_unit,
					 uint32_t rate);

/*
 * Enable or disable an ADG clock source
 *
 * The BRG units have no gate of their own, they run as soon as their timing signal is enabled
 * in TIM_EN, which init() does once for all units. Enabling them is therefore a no-op and
 * disabling them is rejected with -ENOTSUP. The avb_counter8 channels are individually gated
 * by the matching AVB_CLK_CONFIG.DIV_EN bit.
 *
 * @param dev ADG device
 * @param clksrc RCAR_ADG_AUDIO_* clock source id
 * @param enable true to enable the source, false to disable it
 *
 * @retval 0 on success
 * @retval -ENOTSUP if the source is unknown or cannot be disabled
 */
static int rcar_adg_clksrc_enable(const struct device *dev, uint32_t clksrc, bool enable)
{
	const struct clock_control_renesas_adg_cfg *config = dev->config;
	int ret = 0;

	switch (clksrc) {
	case RCAR_ADG_AUDIO_BRGA:
	case RCAR_ADG_AUDIO_BRGB:
		ret = enable ? 0 : -ENOTSUP;
		break;
	case RCAR_ADG_AUDIO_AVB0:
	case RCAR_ADG_AUDIO_AVB1:
	case RCAR_ADG_AUDIO_AVB2:
	case RCAR_ADG_AUDIO_AVB3:
	case RCAR_ADG_AUDIO_AVB4:
	case RCAR_ADG_AUDIO_AVB5:
	case RCAR_ADG_AUDIO_AVB6:
	case RCAR_ADG_AUDIO_AVB7: {
		uint32_t reg = sys_read32(config->base + ADG_AVB_CLK_CONFIG_OFFSET);

		if (enable) {
			reg |= BIT(clksrc - RCAR_ADG_AUDIO_AVB0);
		} else {
			reg &= ~BIT(clksrc - RCAR_ADG_AUDIO_AVB0);
		}

		sys_write32(reg, config->base + ADG_AVB_CLK_CONFIG_OFFSET);
		ret = 0;
		break;
	}
	default:
		ret = -ENOTSUP;
	}

	return ret;
}

/*
 * Get the status of an ADG clock source
 *
 * BRGA and BRGB are always reported as running since they are not gated, the avb_counter8
 * channels are reported from their AVB_CLK_CONFIG.DIV_EN bit.
 *
 * @param dev ADG device
 * @param clksrc RCAR_ADG_AUDIO_* clock source id
 *
 * @return clock status, CLOCK_CONTROL_STATUS_UNKNOWN if the source is unknown
 */
static enum clock_control_status rcar_adg_clksrc_get_status(const struct device *dev,
							    uint32_t clksrc)
{
	const struct clock_control_renesas_adg_cfg *config = dev->config;

	switch (clksrc) {
	case RCAR_ADG_AUDIO_BRGA:
	case RCAR_ADG_AUDIO_BRGB:
		return CLOCK_CONTROL_STATUS_ON;
	case RCAR_ADG_AUDIO_AVB0:
	case RCAR_ADG_AUDIO_AVB1:
	case RCAR_ADG_AUDIO_AVB2:
	case RCAR_ADG_AUDIO_AVB3:
	case RCAR_ADG_AUDIO_AVB4:
	case RCAR_ADG_AUDIO_AVB5:
	case RCAR_ADG_AUDIO_AVB6:
	case RCAR_ADG_AUDIO_AVB7:
		if ((sys_read32(config->base + ADG_AVB_CLK_CONFIG_OFFSET) &
		     BIT(clksrc - RCAR_ADG_AUDIO_AVB0)) != 0U) {
			return CLOCK_CONTROL_STATUS_ON;
		} else {
			return CLOCK_CONTROL_STATUS_OFF;
		}
	default:
		return CLOCK_CONTROL_STATUS_UNKNOWN;
	}
}

/*
 * Get the output rate of an ADG clock source
 *
 * Returns the cached rate, which the driver keeps in sync with the BRRx and AVB_CLK_DIVx
 * registers it programs. An avb_counter8 channel reads back 0 until its rate has been set.
 *
 * @param dev ADG device
 * @param clksrc RCAR_ADG_AUDIO_* clock source id
 *
 * @return rate in Hz, 0 if the source is unknown or not configured yet
 */
static uint32_t rcar_adg_clksrc_get_rate(const struct device *dev, uint32_t clksrc)
{
	struct clock_control_renesas_adg_data *data = dev->data;

	switch (clksrc) {
	case RCAR_ADG_AUDIO_BRGA:
		return data->brg_rate[BRGA];
	case RCAR_ADG_AUDIO_BRGB:
		return data->brg_rate[BRGB];
	case RCAR_ADG_AUDIO_AVB0:
	case RCAR_ADG_AUDIO_AVB1:
	case RCAR_ADG_AUDIO_AVB2:
	case RCAR_ADG_AUDIO_AVB3:
	case RCAR_ADG_AUDIO_AVB4:
	case RCAR_ADG_AUDIO_AVB5:
	case RCAR_ADG_AUDIO_AVB6:
	case RCAR_ADG_AUDIO_AVB7:
		return data->avbcounter8_rate[clksrc - RCAR_ADG_AUDIO_AVB0];
	default:
		return 0;
	}
}

/*
 * Set the output rate of an ADG clock source
 *
 * Dispatches to the BRG or avb_counter8 helper, which translate the requested rate into
 * register fields.
 *
 * @param dev ADG device
 * @param clksrc RCAR_ADG_AUDIO_* clock source id
 * @param rate requested rate in Hz
 *
 * @retval 0 on success
 * @retval -ENOTSUP if the source is unknown or the rate cannot be derived
 */
static int rcar_adg_clksrc_set_rate(const struct device *dev, uint32_t clksrc, uint32_t rate)
{
	switch (clksrc) {
	case RCAR_ADG_AUDIO_BRGA:
		return rcar_adg_brg_set_rate(dev, BRGA, rate);
	case RCAR_ADG_AUDIO_BRGB:
		return rcar_adg_brg_set_rate(dev, BRGB, rate);
	case RCAR_ADG_AUDIO_AVB0:
	case RCAR_ADG_AUDIO_AVB1:
	case RCAR_ADG_AUDIO_AVB2:
	case RCAR_ADG_AUDIO_AVB3:
	case RCAR_ADG_AUDIO_AVB4:
	case RCAR_ADG_AUDIO_AVB5:
	case RCAR_ADG_AUDIO_AVB6:
	case RCAR_ADG_AUDIO_AVB7:
		return rcar_adg_avbcounter8_set_rate(dev, clksrc - RCAR_ADG_AUDIO_AVB0, rate);
	default:
		return -ENOTSUP;
	}
}

/*
 * Get the input clock rate of a BRG unit
 *
 * Reads the clock select field of the unit from BRGCKR and returns the rate of the selected
 * input, as recorded in the clkin_src_rate table.
 *
 * @param config ADG device configuration
 * @param brg_unit BRGA or BRGB
 *
 * @return input rate in Hz, 0 for an invalid unit or an unknown selection
 */
static uint32_t rcar_adg_brg_clkin_get_rate(const struct clock_control_renesas_adg_cfg *config,
					    uint8_t brg_unit)
{
	uint32_t brgckr = sys_read32(config->base + ADG_BRGCKR_OFFSET);
	uint32_t sel;

	if (brg_unit == BRGA) {
		sel = (brgckr & ADG_BRGCKR_BRGCKR_20_22_MSK) >> ADG_BRGCKR_BRGCKR_20_22_POS;
	} else if (brg_unit == BRGB) {
		sel = (brgckr & ADG_BRGCKR_BRGCKR_16_18_MSK) >> ADG_BRGCKR_BRGCKR_16_18_POS;
	} else {
		return 0U;
	}

	switch (sel) {
	case BRG_CLKIN_AUDIO_CLKA:
		return config->clkin_src_rate->audio_clka_hz;
	case BRG_CLKIN_AUDIO_CLKB:
		return config->clkin_src_rate->audio_clkb_hz;
	case BRG_CLKIN_S0D4:
	case BRG_CLKIN_S0D4_x:
		return config->clkin_src_rate->audio_s0d4_hz;
	case BRG_CLKIN_AUDIO_CLKC:
		return config->clkin_src_rate->audio_clkc_hz;
	default:
		return 0U;
	}
}

/*
 * Set the output rate of a BRG unit
 *
 * Searches the CKS and BRR field pair that gets closest to the requested rate, then programs
 * BRRx and caches the rate that was actually achieved. The search is rejected if the best
 * candidate is off by more than 1% of the requested rate.
 *
 * @param dev ADG device
 * @param brg_unit BRGA or BRGB
 * @param rate requested rate in Hz
 *
 * @retval 0 on success
 * @retval -ENOTSUP for an invalid unit, a stopped input clock, a zero rate, or when no field
 *         pair gets within 1% of the requested rate
 */
static int rcar_adg_brg_set_rate(const struct device *dev, uint8_t brg_unit, uint32_t rate)
{
	const struct clock_control_renesas_adg_cfg *config = dev->config;
	struct clock_control_renesas_adg_data *data = dev->data;
	static const uint32_t cks_field[] = {0, 1, 2, 3};
	uint32_t brg_reg = 0U;

	if (brg_unit == BRGA) {
		brg_reg = config->base + ADG_BRRA_OFFSET;
	} else if (brg_unit == BRGB) {
		brg_reg = config->base + ADG_BRRB_OFFSET;
	} else {
		return -ENOTSUP;
	}

	uint32_t brg_clksrc_hz = rcar_adg_brg_clkin_get_rate(config, brg_unit);

	if (brg_clksrc_hz == 0 || rate == 0) {
		return -ENOTSUP;
	}

	/* The output clock rate is calculated by: rate = brg_clksrc_hz / (2 * (4^cks) * (brr + 1))
	 * This section calculate the best cks and brr bit field value for requested rate
	 */
	uint32_t best_error = UINT32_MAX;
	uint32_t best_cks = cks_field[0];
	uint32_t best_brr = 0;
	uint32_t best_actual_rate = 0;

	for (size_t i = 0; i < ARRAY_SIZE(cks_field); i++) {
		uint32_t cks = cks_field[i];

		/* aclk_div = 4^cks */
		uint32_t aclk_div = 1U << (cks * 2);

		uint32_t denom_base = 2U * aclk_div;

		for (uint32_t n = 0; n <= 255; n++) {
			uint32_t denom = denom_base * (n + 1);
			uint32_t actual_rate = brg_clksrc_hz / denom;

			uint32_t error =
				(actual_rate > rate) ? (actual_rate - rate) : (rate - actual_rate);

			if (error < best_error) {
				best_error = error;
				best_cks = cks;
				best_brr = n;
				best_actual_rate = actual_rate;

				if (error == 0) {
					/* Stop early if found exact match */
					goto done;
				}
			}
		}
	}

	uint32_t max_allowed_error = rate / 100;

	if (best_error > max_allowed_error) {
		LOG_ERR("Cannot derive acceptable rate: best=%u requested=%u (error=%u > max %u)",
			best_actual_rate, rate, best_error, max_allowed_error);
		return -ENOTSUP;
	}

done:
	LOG_DBG("Calculated best cks [0x%x] and brr [0x%x] bit field with actual rate %d for "
		"requested rate %d",
		best_cks, best_brr, best_actual_rate, rate);

	uint32_t reg_val = (best_cks << ADG_BRRx_CKS_POS) & ADG_BRRx_CKS_MSK;

	reg_val |= (best_brr << ADG_BRRx_BRRx_POS) & ADG_BRRx_BRRx_MSK;
	sys_write32(reg_val, brg_reg);

	data->brg_rate[brg_unit] = best_actual_rate;

	return 0;
}

/*
 * Set the output rate of an avb_counter8 channel
 *
 * The channel divides S0D1 by the comparison value in AVB_CLK_DIVx, made of a 12 bit integral
 * and a 6 bit fractional field. The required value is computed directly from the requested
 * rate, rounded to nearest, and the achieved rate is cached.
 *
 * @param dev ADG device
 * @param avbcounter8_unit avb_counter8 channel index, 0 to 7
 * @param rate requested rate in Hz
 *
 * @retval 0 on success
 * @retval -ENOTSUP for a stopped S0D1 clock, a zero rate, or a requested rate that needs a
 *         comparison value outside the range of the register fields
 */
static int rcar_adg_avbcounter8_set_rate(const struct device *dev, uint8_t avbcounter8_unit,
					 uint32_t rate)
{
	const struct clock_control_renesas_adg_cfg *config = dev->config;
	struct clock_control_renesas_adg_data *data = dev->data;
	uint32_t avbcounter8_clksrc_hz = config->clkin_src_rate->audio_s0d1_hz;

	if (avbcounter8_clksrc_hz == 0 || rate == 0) {
		return -ENOTSUP;
	}

	/* The output clock rate is calculated by: rate = (avbcounter8_clksrc_hz * 32) /
	 * (integral_field * 64 + fractional_field)) This section calculate the best integral and
	 * fractional bit field value for requested rate
	 */
	uint64_t num = (uint64_t)avbcounter8_clksrc_hz * 32U * 2U; /* x2 for exact rounding */
	uint32_t total = (uint32_t)((num + rate) / ((uint64_t)rate * 2U));

	if (total == 0U) {
		LOG_ERR("AVB counter %d: underflow", avbcounter8_unit);
		return -ENOTSUP;
	}

	uint32_t integral_field = total >> ADG_AVB_CLK_DIV_INTEGRAL_POS;
	uint32_t fractional_field = total & ADG_AVB_CLK_DIV_FRACTIONAL_MSK;

	if (integral_field > (ADG_AVB_CLK_DIV_INTEGRAL_MSK >> ADG_AVB_CLK_DIV_INTEGRAL_POS)) {
		LOG_ERR("AVB counter %d: integral overflow (%u)", avbcounter8_unit, integral_field);
		return -ENOTSUP;
	}

	uint32_t reg_val =
		(integral_field << ADG_AVB_CLK_DIV_INTEGRAL_POS) & ADG_AVB_CLK_DIV_INTEGRAL_MSK;

	reg_val |= (fractional_field << ADG_AVB_CLK_DIV_FRACTIONAL_POS) &
		   ADG_AVB_CLK_DIV_FRACTIONAL_MSK;
	sys_write32(reg_val, config->base + ADG_AVB_CLK_DIVx_OFFSET(avbcounter8_unit));

	data->avbcounter8_rate[avbcounter8_unit] =
		(uint32_t)(((uint64_t)avbcounter8_clksrc_hz * 32) /
			   (integral_field * 64 + fractional_field));

	return 0;
}

/*
 * Implementation of the clock_control on() API
 *
 * The subsystem is either a clock source id, which is enabled directly, or an AUDIO_CLKOUT pin
 * id, in which case the source the device tree routed to that pin is enabled.
 *
 * @param dev ADG device
 * @param sys clock identifier, see <zephyr/dt-bindings/clock/renesas_rcar_adg.h>
 *
 * @retval 0 on success
 * @retval -ENOTSUP if the identifier is invalid or the source cannot be enabled
 */
static int clock_control_renesas_rcar_adg_on(const struct device *dev, clock_control_subsys_t sys)
{
	struct clock_control_renesas_adg_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	int clock_source = (int)sys;
	int ret = 0;

	if (ADG_IS_CLKSRC(clock_source)) {
		/* Enable clock source */
		ret = rcar_adg_clksrc_enable(dev, clock_source, true);

	} else if (ADG_IS_CLKOUT(clock_source)) {
		/* Enable the clock source routed to the clkout pin */
		int clkout_pin = clock_source - RCAR_ADG_AUDIO_CLKOUT0;

		switch (clock_source) {
		case RCAR_ADG_AUDIO_CLKOUT0:
		case RCAR_ADG_AUDIO_CLKOUT1:
		case RCAR_ADG_AUDIO_CLKOUT2:
		case RCAR_ADG_AUDIO_CLKOUT3:
			ret = rcar_adg_clksrc_enable(dev, data->clkout_src[clkout_pin], true);
			break;
		default:
			ret = -ENOTSUP;
		}
	} else {
		ret = -ENOTSUP;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

/*
 * Implementation of the clock_control off() API
 *
 * Counterpart of on(), accepting the same clock identifiers. Only the avb_counter8 channels can
 * actually be stopped, so disabling a BRG based clock fails.
 *
 * @param dev ADG device
 * @param sys clock identifier, see <zephyr/dt-bindings/clock/renesas_rcar_adg.h>
 *
 * @retval 0 on success
 * @retval -ENOTSUP if the identifier is invalid or the source cannot be disabled
 */
static int clock_control_renesas_rcar_adg_off(const struct device *dev, clock_control_subsys_t sys)
{
	struct clock_control_renesas_adg_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	int clock_source = (int)sys;
	int ret = 0;

	if (ADG_IS_CLKSRC(clock_source)) {
		/* Disable clock source */
		ret = rcar_adg_clksrc_enable(dev, clock_source, false);

	} else if (ADG_IS_CLKOUT(clock_source)) {
		/* Disable the clock source routed to the clkout pin */
		int clkout_pin = clock_source - RCAR_ADG_AUDIO_CLKOUT0;

		switch (clock_source) {
		case RCAR_ADG_AUDIO_CLKOUT0:
		case RCAR_ADG_AUDIO_CLKOUT1:
		case RCAR_ADG_AUDIO_CLKOUT2:
		case RCAR_ADG_AUDIO_CLKOUT3:
			ret = rcar_adg_clksrc_enable(dev, data->clkout_src[clkout_pin], false);
			break;
		default:
			ret = -ENOTSUP;
		}
	} else {
		ret = -ENOTSUP;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

/*
 * Implementation of the clock_control get_status() API
 *
 * @param dev ADG device
 * @param sys clock identifier, see <zephyr/dt-bindings/clock/renesas_rcar_adg.h>
 *
 * @return status of the addressed source, or of the source routed to the addressed
 *         AUDIO_CLKOUT pin, CLOCK_CONTROL_STATUS_UNKNOWN if the identifier is invalid
 */
static enum clock_control_status
clock_control_renesas_rcar_adg_get_status(const struct device *dev, clock_control_subsys_t sys)
{
	struct clock_control_renesas_adg_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	int clock_source = (int)sys;
	enum clock_control_status ret = CLOCK_CONTROL_STATUS_UNKNOWN;

	if (ADG_IS_CLKSRC(clock_source)) {
		/* Get clock source status */
		ret = rcar_adg_clksrc_get_status(dev, clock_source);

	} else if (ADG_IS_CLKOUT(clock_source)) {
		/* Get status of the clock source routed to the clkout pin */
		int clkout_pin = clock_source - RCAR_ADG_AUDIO_CLKOUT0;

		switch (clock_source) {
		case RCAR_ADG_AUDIO_CLKOUT0:
		case RCAR_ADG_AUDIO_CLKOUT1:
		case RCAR_ADG_AUDIO_CLKOUT2:
		case RCAR_ADG_AUDIO_CLKOUT3:
			ret = rcar_adg_clksrc_get_status(dev, data->clkout_src[clkout_pin]);
			break;
		default:
			ret = CLOCK_CONTROL_STATUS_UNKNOWN;
		}
	} else {
		ret = CLOCK_CONTROL_STATUS_UNKNOWN;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

/*
 * Implementation of the clock_control get_rate() API
 *
 * @param dev ADG device
 * @param sys clock identifier, see <zephyr/dt-bindings/clock/renesas_rcar_adg.h>
 * @param rate output, rate in Hz of the addressed source, or of the source routed to the
 *             addressed AUDIO_CLKOUT pin, left untouched on error
 *
 * @retval 0 on success
 * @retval -ENOTSUP if the identifier is invalid
 */
static int clock_control_renesas_rcar_adg_get_rate(const struct device *dev,
						   clock_control_subsys_t sys, uint32_t *rate)
{
	struct clock_control_renesas_adg_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	int clock_source = (int)sys;
	int ret = 0;

	if (ADG_IS_CLKSRC(clock_source)) {
		/* Get clock source rate */
		*rate = rcar_adg_clksrc_get_rate(dev, clock_source);

	} else if (ADG_IS_CLKOUT(clock_source)) {
		/* Get rate of the clock source routed to the clkout pin */
		int clkout_pin = clock_source - RCAR_ADG_AUDIO_CLKOUT0;

		switch (clock_source) {
		case RCAR_ADG_AUDIO_CLKOUT0:
		case RCAR_ADG_AUDIO_CLKOUT1:
		case RCAR_ADG_AUDIO_CLKOUT2:
		case RCAR_ADG_AUDIO_CLKOUT3:
			*rate = rcar_adg_clksrc_get_rate(dev, data->clkout_src[clkout_pin]);
			break;
		default:
			ret = -ENOTSUP;
		}
	} else {
		ret = -ENOTSUP;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

/*
 * Implementation of the clock_control set_rate() API
 *
 * A clock already running at the requested rate is reported with -EALREADY instead of being
 * reprogrammed.
 *
 * Since a BRG or avb_counter8 channel may feed several consumers, changing its rate affects all
 * of them, including any AUDIO_CLKOUT pin it is routed to.
 *
 * @param dev ADG device
 * @param sys clock identifier, see <zephyr/dt-bindings/clock/renesas_rcar_adg.h>
 * @param rate requested rate in Hz
 *
 * @retval 0 on success
 * @retval -EINVAL if the requested rate is not positive
 * @retval -EALREADY if the clock already runs at the requested rate
 * @retval -ENOTSUP if the identifier is invalid or the rate cannot be derived
 */
static int clock_control_renesas_rcar_adg_set_rate(const struct device *dev,
						   clock_control_subsys_t sys,
						   clock_control_subsys_rate_t rate)
{
	struct clock_control_renesas_adg_data *data = dev->data;

	if (rate <= 0) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	int clock_source = (int)sys;
	uint32_t curr_rate = 0U;
	int ret = clock_control_renesas_rcar_adg_get_rate(dev, sys, &curr_rate);

	if (ret < 0) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	if (curr_rate == (uint32_t)rate) {
		ret = -EALREADY;
		k_mutex_unlock(&data->lock);
		return ret;
	}

	if (ADG_IS_CLKSRC(clock_source)) {
		/* Set clock source rate */
		ret = rcar_adg_clksrc_set_rate(dev, clock_source, (uint32_t)rate);

	} else if (ADG_IS_CLKOUT(clock_source)) {
		/* Set rate of the clock source routed to the clkout pin */
		int clkout_pin = clock_source - RCAR_ADG_AUDIO_CLKOUT0;

		switch (clock_source) {
		case RCAR_ADG_AUDIO_CLKOUT0:
		case RCAR_ADG_AUDIO_CLKOUT1:
		case RCAR_ADG_AUDIO_CLKOUT2:
		case RCAR_ADG_AUDIO_CLKOUT3:
			ret = rcar_adg_clksrc_set_rate(dev, data->clkout_src[clkout_pin],
						       (uint32_t)rate);
			break;
		default:
			ret = -ENOTSUP;
		}
	} else {
		ret = -ENOTSUP;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

/*
 * Initialize an ADG instance
 *
 * Enables the module and internal clocks the ADG needs, records the rates of the internal
 * clock inputs, applies the device tree routing to BRGCKR and AVBCKR, seeds the cached rates
 * from the reset value of the BRRx registers and enables the timing signals of all units.
 *
 * @param dev ADG device
 *
 * @retval 0 on success
 * @retval -ENOTSUP if the device tree selects an unknown AUDIO_CLKOUT clock source
 * @retval -EINVAL if the device tree routes BRGA to one AUDIO_CLKOUT pin and BRGB to another,
 *         which the single BRGCKR[31] selection bit cannot express
 * @retval negative errno propagated from the CPG clock controller
 */
static int clock_control_renesas_rcar_adg_init(const struct device *dev)
{
	const struct clock_control_renesas_adg_cfg *config = dev->config;
	struct clock_control_renesas_adg_data *data = dev->data;
	int ret = 0;

	k_mutex_init(&data->lock);

	/* Enable clock for ADG and its internal clock */
	ret = clock_control_on(config->dev_pclk.clock_dev,
			       (clock_control_subsys_t)&config->dev_pclk.cpg);

	if (ret < 0) {
		LOG_ERR("Failed enable pclk clock");
		return ret;
	}

	ret = clock_control_on(config->dev_s0d1.clock_dev,
			       (clock_control_subsys_t)&config->dev_s0d1.cpg);

	if (ret < 0) {
		LOG_ERR("Failed enable s0d1 clock");
		return ret;
	}

	ret = clock_control_on(config->dev_s0d4.clock_dev,
			       (clock_control_subsys_t)&config->dev_s0d4.cpg);

	if (ret < 0) {
		LOG_ERR("Failed enable s0d4 clock");
		return ret;
	}

	/* Get internal clock sources rate */
	ret = clock_control_get_rate(config->dev_s0d1.clock_dev,
				     (clock_control_subsys_t)&config->dev_s0d1.cpg,
				     &config->clkin_src_rate->audio_s0d1_hz);

	if (ret < 0) {
		LOG_ERR("Failed to get rate of s0d1 clock");
		return ret;
	}

	ret = clock_control_get_rate(config->dev_s0d4.clock_dev,
				     (clock_control_subsys_t)&config->dev_s0d4.cpg,
				     &config->clkin_src_rate->audio_s0d4_hz);

	if (ret < 0) {
		LOG_ERR("Failed to get rate of s0d4 clock");
		return ret;
	}

	uint32_t avbckr_val = sys_read32(config->base + ADG_AVBCKR_OFFSET);
	uint32_t brgckr_val = sys_read32(config->base + ADG_BRGCKR_OFFSET);
	bool uses_brga = false;
	bool uses_brgb = false;

	/* Select BRGA/BRGB input clock source */
	brgckr_val &= ~(ADG_BRGCKR_BRGCKR_20_22_MSK | ADG_BRGCKR_BRGCKR_16_18_MSK);
	brgckr_val |= (config->brg_clk_sel[BRGA] << ADG_BRGCKR_BRGCKR_20_22_POS) &
		      ADG_BRGCKR_BRGCKR_20_22_MSK;
	brgckr_val |= (config->brg_clk_sel[BRGB] << ADG_BRGCKR_BRGCKR_16_18_POS) &
		      ADG_BRGCKR_BRGCKR_16_18_MSK;

	/* Select AUDIO_CLKOUTx clock source */
	for (uint8_t i = 0; i < ARRAY_SIZE(data->clkout_src); i++) {
		avbckr_val &= ~(ADG_AVBCKR_AUDIO_CLKOUTx_MSK(i));
		switch (data->clkout_src[i]) {
		case RCAR_ADG_AUDIO_NONE:
			/* Pin unused */
			break;
		case RCAR_ADG_AUDIO_BRGA:
			uses_brga = true;
			avbckr_val |= (BRGA << ADG_AVBCKR_AUDIO_CLKOUTx_POS(i)) &
				      ADG_AVBCKR_AUDIO_CLKOUTx_MSK(i);
			brgckr_val &= ~BIT(ADG_BRGCKR_BRGCKR_31_POS);
			break;
		case RCAR_ADG_AUDIO_BRGB:
			uses_brgb = true;
			avbckr_val |= (BRGB << ADG_AVBCKR_AUDIO_CLKOUTx_POS(i)) &
				      ADG_AVBCKR_AUDIO_CLKOUTx_MSK(i);
			brgckr_val |= BIT(ADG_BRGCKR_BRGCKR_31_POS);
			break;
		case RCAR_ADG_AUDIO_AVB0:
			avbckr_val |= (AVB_COUNTER8_0 << ADG_AVBCKR_AUDIO_CLKOUTx_POS(i)) &
				      ADG_AVBCKR_AUDIO_CLKOUTx_MSK(i);
			break;
		case RCAR_ADG_AUDIO_AVB1:
			avbckr_val |= (AVB_COUNTER8_1 << ADG_AVBCKR_AUDIO_CLKOUTx_POS(i)) &
				      ADG_AVBCKR_AUDIO_CLKOUTx_MSK(i);
			break;
		case RCAR_ADG_AUDIO_AVB2:
			avbckr_val |= (AVB_COUNTER8_2 << ADG_AVBCKR_AUDIO_CLKOUTx_POS(i)) &
				      ADG_AVBCKR_AUDIO_CLKOUTx_MSK(i);
			break;
		case RCAR_ADG_AUDIO_AVB3:
			avbckr_val |= (AVB_COUNTER8_3 << ADG_AVBCKR_AUDIO_CLKOUTx_POS(i)) &
				      ADG_AVBCKR_AUDIO_CLKOUTx_MSK(i);
			break;
		case RCAR_ADG_AUDIO_AVB4:
			avbckr_val |= (AVB_COUNTER8_4 << ADG_AVBCKR_AUDIO_CLKOUTx_POS(i)) &
				      ADG_AVBCKR_AUDIO_CLKOUTx_MSK(i);
			break;
		case RCAR_ADG_AUDIO_AVB5:
			avbckr_val |= (AVB_COUNTER8_5 << ADG_AVBCKR_AUDIO_CLKOUTx_POS(i)) &
				      ADG_AVBCKR_AUDIO_CLKOUTx_MSK(i);
			break;
		case RCAR_ADG_AUDIO_AVB6:
			avbckr_val |= (AVB_COUNTER8_6 << ADG_AVBCKR_AUDIO_CLKOUTx_POS(i)) &
				      ADG_AVBCKR_AUDIO_CLKOUTx_MSK(i);
			break;
		case RCAR_ADG_AUDIO_AVB7:
			avbckr_val |= (AVB_COUNTER8_7 << ADG_AVBCKR_AUDIO_CLKOUTx_POS(i)) &
				      ADG_AVBCKR_AUDIO_CLKOUTx_MSK(i);
			break;

		default:
			return -ENOTSUP;
		}
	}

	if (uses_brga && uses_brgb) {
		LOG_ERR("CLKOUT cannot mix BRGA and BRGB: BRGCKR_31 selects one BRG for all pins");
		return -EINVAL;
	}

	sys_write32(avbckr_val, config->base + ADG_AVBCKR_OFFSET);
	sys_write32(brgckr_val, config->base + ADG_BRGCKR_OFFSET);

	/* Initialize data struct */
	/* BRGA */
	uint32_t reg = sys_read32(config->base + ADG_BRRA_OFFSET);
	int aclk_ratio = FIELD_GET(ADG_BRRx_CKS_MSK, reg);
	int brr_ratio = FIELD_GET(ADG_BRRx_BRRx_MSK, reg);

	data->brg_rate[BRGA] = rcar_adg_brg_clkin_get_rate(config, BRGA) /
			       ADG_BRG_DIV_RATIO(aclk_ratio, brr_ratio);

	/* BRGB */
	reg = sys_read32(config->base + ADG_BRRB_OFFSET);
	aclk_ratio = FIELD_GET(ADG_BRRx_CKS_MSK, reg);
	brr_ratio = FIELD_GET(ADG_BRRx_BRRx_MSK, reg);
	data->brg_rate[BRGB] = rcar_adg_brg_clkin_get_rate(config, BRGB) /
			       ADG_BRG_DIV_RATIO(aclk_ratio, brr_ratio);

	/* avb_counter8 */
	reg = sys_read32(config->base + ADG_AVB_CLK_CONFIG_OFFSET);
	reg |= ADG_AVB_CLK_CONFIG_DIV_EN_COM_MSK;
	sys_write32(reg, config->base + ADG_AVB_CLK_CONFIG_OFFSET);

	memset(data->avbcounter8_rate, 0, sizeof(data->avbcounter8_rate));

	/* Enable all timing signals */
	uint32_t tim_en = ADG_TIM_EN_BRGB_TIM_EN_MSK | ADG_TIM_EN_BRGA_TIM_EN_MSK |
			  ADG_TIM_EN_AUDIO_CLKC_TIM_EN_MSK | ADG_TIM_EN_AUDIO_CLKB_TIM_EN_MSK |
			  ADG_TIM_EN_AUDIO_CLKA_TIM_EN_MSK;
	sys_write32(tim_en, config->base + ADG_TIM_EN_OFFSET);

	return 0;
}

static DEVICE_API(clock_control, clock_control_renesas_rcar_adg_api) = {
	.on = clock_control_renesas_rcar_adg_on,
	.off = clock_control_renesas_rcar_adg_off,
	.get_status = clock_control_renesas_rcar_adg_get_status,
	.get_rate = clock_control_renesas_rcar_adg_get_rate,
	.set_rate = clock_control_renesas_rcar_adg_set_rate,
};

/* Build a rcar_adg_clock_config from the "clk_name" entry of the instance "clocks" property */
#define RCAR_ADG_CLOCK_DEFINE(node_id, clk_name)                                                   \
	{                                                                                          \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(node_id, clk_name)),        \
		.cpg =                                                                             \
			{                                                                          \
				.module = DT_INST_CLOCKS_CELL_BY_NAME(node_id, clk_name, module),  \
				.domain = DT_INST_CLOCKS_CELL_BY_NAME(node_id, clk_name, domain),  \
			},                                                                         \
	}

/* Define the config, data and device for one ADG instance */
#define INIT_ADG(node_id)                                                                          \
	static struct i2s_rcar_adg_clk_rate clkin_src_rate##node_id = {                            \
		.audio_clka_hz =                                                                   \
			DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(node_id, clk_a), clock_frequency),     \
		.audio_clkb_hz =                                                                   \
			DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(node_id, clk_b), clock_frequency),     \
		.audio_clkc_hz =                                                                   \
			DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(node_id, clk_c), clock_frequency),     \
	};                                                                                         \
                                                                                                   \
	static const struct clock_control_renesas_adg_cfg adg_cfg_##node_id = {                    \
		.base = DT_INST_REG_ADDR(node_id),                                                 \
		.clkin_src_rate = &clkin_src_rate##node_id,                                        \
		.dev_pclk = RCAR_ADG_CLOCK_DEFINE(node_id, pclk),                                  \
		.dev_s0d4 = RCAR_ADG_CLOCK_DEFINE(node_id, s0d4),                                  \
		.dev_s0d1 = RCAR_ADG_CLOCK_DEFINE(node_id, s0d1),                                  \
		.brg_clk_sel = {DT_INST_PROP(node_id, brga_clk_sel),                               \
				DT_INST_PROP(node_id, brgb_clk_sel)},                              \
	};                                                                                         \
                                                                                                   \
	static struct clock_control_renesas_adg_data adg_data_##node_id = {                        \
		.clkout_src = DT_INST_PROP(node_id, audio_clkout_sel)};                            \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(node_id, clock_control_renesas_rcar_adg_init, NULL,                  \
			      &adg_data_##node_id, &adg_cfg_##node_id, POST_KERNEL,                \
			      CONFIG_CLOCK_CONTROL_ADG_INIT_PRIORITY,                              \
			      &clock_control_renesas_rcar_adg_api);

DT_INST_FOREACH_STATUS_OKAY(INIT_ADG);
