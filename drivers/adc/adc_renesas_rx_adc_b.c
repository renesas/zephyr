/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_adc_b

#include "zephyr/dt-bindings/adc/adc.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util_internal.h>
#include <instances/r_adc_b.h>
#include "fsp_common_api.h"
#include "r_adc_device_types.h"
#include "zephyr/drivers/clock_control/renesas_rx_cgc.h"
#include <zephyr/drivers/interrupt_controller/intc_renesas_rx_grp_int_fsp.h>
#include <string.h>

#include <zephyr/irq.h>

LOG_MODULE_REGISTER(renesas_rx_adc_b, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define ADC_B_RX_MAX_VC_PER_GROUP 8U
#define ADC_B_SEC_PER_NSEC        1000000000L
#define ADC_B_SST_MIN             2
#define ADC_B_SST_MAX             1023
/* Maximum of virtual channels */
#define ADC_B_RX_MAX_VC           50

/* Three calibrations end group IRQ use group_irq_al5 */
#define ADC0_NODE DT_NODELABEL(adc0)
#define ADC1_NODE DT_NODELABEL(adc1)
#define ADC2_NODE DT_NODELABEL(adc2)

#define GROUP_INT_AL5 DT_IRQ(DT_NODELABEL(group_irq_al5), irq)

/* Calibration end IRQs for ADC units, also use group_irq_al5 */
#define CALIB_END_UNIT_0_IRQ DT_IRQ_BY_NAME(ADC0_NODE, calend, irq)
#define CALIB_END_UNIT_1_IRQ DT_IRQ_BY_NAME(ADC1_NODE, calend, irq)
#define CALIB_END_UNIT_2_IRQ DT_IRQ_BY_NAME(ADC2_NODE, calend, irq)

#define VECTOR_NUMBER_ADC_CALEND0 ((IRQn_Type)(CALIB_END_UNIT_0_IRQ << 8U | GROUP_INT_AL5))
#define VECTOR_NUMBER_ADC_CALEND1 ((IRQn_Type)(CALIB_END_UNIT_1_IRQ << 8U | GROUP_INT_AL5))
#define VECTOR_NUMBER_ADC_CALEND2 ((IRQn_Type)(CALIB_END_UNIT_2_IRQ << 8U | GROUP_INT_AL5))

/* Conversion error IRQs for ADC units, also use group_irq_al5 */
#if DT_NODE_HAS_STATUS(ADC0_NODE, okay)
#define CONS_UNIT_0_IRQ DT_IRQ_BY_NAME(ADC0_NODE, convserr, irq)
#endif

#if DT_NODE_HAS_STATUS(ADC1_NODE, okay)
#define CONS_UNIT_1_IRQ DT_IRQ_BY_NAME(ADC1_NODE, convserr, irq)
#endif

#if DT_NODE_HAS_STATUS(ADC2_NODE, okay)
#define CONS_UNIT_2_IRQ DT_IRQ_BY_NAME(ADC2_NODE, convserr, irq)
#endif

#define CONS_UNIT_IRQ(unit) UTIL_CAT(UTIL_CAT(CONS_UNIT_, unit), _IRQ)
#define ADC_B_ERR_ISR(unit) UTIL_CAT(UTIL_CAT(adc_b_err, unit), _isr)

#define ISR_CFG_CONVSERR_IRQ(unit) UTIL_CAT(conversion_error_irq_adc_, unit)
#define ISR_CFG_CONVSERR_IPL(unit) UTIL_CAT(conversion_error_ipl_adc_, unit)

#define VECTOR_NUMBER_ADC_CONVSERR(unit) ((IRQn_Type)(CONS_UNIT_IRQ(unit) << 8U | GROUP_INT_AL5))

/* Scan end interrupt */
#define ADC_B_SCAN_END_ISR(grp)       UTIL_CAT(UTIL_CAT(adc_b_adi, grp), _isr)
#define ADC_B_ICU_SCAN_END_EVENT(grp) UTIL_CAT(ICU_EVENT_ADC_ADI, grp)
#define ISR_CFG_SCANEND_IRQ(grp)      UTIL_CAT(scan_end_irq_group_, grp)
#define ISR_CFG_SCANEND_IPL(grp)      UTIL_CAT(scan_end_ipl_group_, grp)

/* ADC Method */
#define ADC_B_DT_METHOD(idx)                                                                       \
	UTIL_CAT(ADC_B_CONVERSION_METHOD_, DT_INST_STRING_UPPER_TOKEN(idx, renesas_adc_method))

#define ADC_B_DFSEL_SINC3_ALL                                                                      \
	{                                                                                          \
		.settings = {                                                                      \
			.idx_0 = ADC_B_DIGITAL_FILTER_MODE_SINC3,                                  \
			.idx_1 = ADC_B_DIGITAL_FILTER_MODE_SINC3,                                  \
			.idx_2 = ADC_B_DIGITAL_FILTER_MODE_SINC3,                                  \
			.idx_3 = ADC_B_DIGITAL_FILTER_MODE_SINC3,                                  \
		}                                                                                  \
	}

#define ADC_B_SGADS(unit, grp) ((unit) << R_ADC_B0_ADSGCR0_SGADS##grp##_Pos)

/* extern for group 0, will be updated flexibly */
extern void adc_b_adi0_isr(void);
extern void adc_b_adi1_isr(void);
extern void adc_b_adi2_isr(void);

extern void adc_b_calend0_isr(void);
extern void adc_b_calend1_isr(void);
extern void adc_b_calend2_isr(void);

extern void adc_b_err0_isr(void);
extern void adc_b_err1_isr(void);
extern void adc_b_err2_isr(void);

struct adc_rx_grp_irq_entry {
	int irq;
	void (*isr)(void);
};

static const struct adc_rx_grp_irq_entry calend_irqs[] = {
	{CALIB_END_UNIT_0_IRQ, adc_b_calend0_isr},
	{CALIB_END_UNIT_1_IRQ, adc_b_calend1_isr},
	{CALIB_END_UNIT_2_IRQ, adc_b_calend2_isr},
};

/**
 * @brief ADC configuration structure
 * This structure holds the configuration parameters for the Renesas RX adc_b driver
 */
struct adc_rx_adc_b_channel_cfg {
	uint8_t vc_id;          /* reg of channel@N, virtual channel id */
	uint8_t input_positive; /* zephyr,input-positive */
	uint8_t resolution;     /* zephyr,resolution */
};

struct adc_rx_adc_b_config {
	R_ADC_B0_Type *regs;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	const struct clock_control_rx_subsys_cfg clock_subsys;
	adc_b_clock_source_t clock_source;
	uint8_t clock_div;

	/** Sampling time in nanoseconds */
	uint32_t sampling_time_ns;

	/* ADC unit id, e.g. 0 for ADC0, 1 for ADC1. */
	uint8_t unit_id;
	/* Bitmask of available channels. */
	uint32_t channel_available_mask;

	/* Channel config array from DTS child nodes */
	const struct adc_rx_adc_b_channel_cfg *dt_channels;
	uint8_t dt_channel_count;

	/** function pointer to irq setup */
	void (*irq_configure)(const struct device *dev);

	/* Group interrupt configuration */
	const struct device *calend_ctrl;
};

struct adc_rx_adc_b_data {
	struct adc_context ctx;
	const struct device *dev;
	adc_b_instance_ctrl_t adc_ctrl;

	adc_cfg_t adc_cfg;
	adc_b_scan_cfg_t scan_cfg;

	/* Generic scan group config */
	adc_b_group_cfg_t group_cfg;
	adc_b_group_cfg_t *groups[1];

	adc_b_virtual_channel_cfg_t vc_cfg[ADC_B_RX_MAX_VC_PER_GROUP];
	adc_b_virtual_channel_cfg_t *group_vcs[ADC_B_RX_MAX_VC_PER_GROUP];
	uint8_t active_vc_count;

	/** Pointer to memory where next sample will be written */
	uint16_t *buf;
	uint16_t buf_id;
	/** Mask with channels that will be sampled */
	uint32_t channels;
	/* Semaphore for calibration */
	struct k_sem calibrate_sem;

	uint32_t configured_channels; /* bitmask of configured channels (channel_setup) */
};

static int adc_rx_validate_phys_channel(uint32_t available_mask, uint8_t phys)
{
	if (phys >= 32U || (available_mask & BIT(phys)) == 0U) {
		return -EINVAL;
	}

	return 0;
}

static adc_b_data_format_t rx_adc_resolution_to_format(uint8_t resolution)
{
	switch (resolution) {
	case 10:
		return ADC_B_DATA_FORMAT_10_BIT;
	case 12:
		return ADC_B_DATA_FORMAT_12_BIT;
	case 14:
		return ADC_B_DATA_FORMAT_14_BIT;
	case 16:
		return ADC_B_DATA_FORMAT_16_BIT;
	default:
		LOG_WRN("Unsupported resolution %d, using 12-bit default", resolution);
		return ADC_B_DATA_FORMAT_12_BIT;
	}
}

/* Initialize virtual channel configuration */
static void adc_rx_adc_b_vc_configure(const struct adc_rx_adc_b_channel_cfg *ch,
				      uint8_t scan_group_id, adc_b_virtual_channel_cfg_t *vc,
				      bool sar_method)
{
	/* Map Zephyr ADC channel configuration to Renesas ADC B virtual channel configuration */
	memset(vc, 0, sizeof(*vc));

	/* Virtual channel ID */
	vc->channel_id = (adc_b_virtual_channel_t)ch->vc_id;
	/* Scan group ID */
	vc->channel_cfg_bits.group = BIT(scan_group_id);

	/* Physical channel */
	vc->channel_cfg_bits.channel = (adc_channel_t)ch->input_positive;
	/* Differential, not used */
	vc->channel_cfg_bits.differential = 0U;
	/* Sampling table ID (BCSSTSL) */
	vc->channel_cfg_bits.sample_table_id = 0U;

	/* Channel control a bits
	 * Digital filter is required for Oversample mode, otherwise not used (SAR mode).
	 */
	/* Support Oversampling mode */
	if (sar_method) {
		vc->channel_control_a_bits.digital_filter_id = 0;
	} else {
		vc->channel_control_a_bits.digital_filter_id = 1;
	}
	vc->channel_control_a_bits.offset_table_id = 0;
	vc->channel_control_a_bits.gain_table_id = 0;

	/* Channel control B bits */
	vc->channel_control_b_bits.addition_average_mode = 0;
	vc->channel_control_b_bits.addition_average_count = 0;
	vc->channel_control_b_bits.compare_match_enable = false;

	vc->channel_control_c_bits.limiter_clip_table_id = 0U;
	vc->channel_control_c_bits.channel_data_format =
		rx_adc_resolution_to_format(ch->resolution);
	vc->channel_control_c_bits.data_is_unsigned = true;
}

/* initialize scan configuration for scan group */
static int adc_rx_adc_b_scan_cfg(const struct device *dev)
{
	const struct adc_rx_adc_b_config *cfg = dev->config;
	struct adc_rx_adc_b_data *data = dev->data;
	adc_b_extended_cfg_t *extend = (adc_b_extended_cfg_t *)data->adc_cfg.p_extend;
	/* Actual channels was configured */
	uint8_t vc_count = 0;
	bool sar_method;

	if (cfg->dt_channel_count == 0U) {
		return -EINVAL;
	}

	if (cfg->dt_channel_count > ADC_B_RX_MAX_VC_PER_GROUP) {
		LOG_DBG("Support maximum 8 channels for 1 scan group");
		return -ENOMEM;
	}

	if (extend->adc_b_converter_mode[cfg->unit_id].method == ADC_B_CONVERSION_METHOD_SAR) {
		sar_method = true;
	} else {
		sar_method = false;
	}

	/*
	 * Initialize virtual channel configurations.
	 * Only channels marked in configured_channels will be added.
	 */
	for (uint8_t i = 0U; i < cfg->dt_channel_count; i++) {
		/* Skip channels that are not configured yet */
		if (!(data->configured_channels & BIT(cfg->dt_channels[i].vc_id))) {
			continue;
		}

		int ret = adc_rx_validate_phys_channel(cfg->channel_available_mask,
						       cfg->dt_channels[i].input_positive);
		if (ret < 0) {
			return ret;
		}

		/* Currently, Scan group is tied to ADC units - Unit x - Scan group x */
		adc_rx_adc_b_vc_configure(&cfg->dt_channels[i], cfg->unit_id,
					  &data->vc_cfg[vc_count], sar_method);
		data->group_vcs[vc_count] = &data->vc_cfg[vc_count];
		vc_count++;
	}

	if (vc_count == 0U) {
		LOG_ERR("No channels configured");
		return -EINVAL;
	}

	data->active_vc_count = vc_count;
	/* Configure scan group 0 */
	memset(&data->group_cfg, 0, sizeof(data->group_cfg));
	/* Use the ADC unit ID as the scan group ID */
	data->group_cfg.scan_group_id = cfg->unit_id;
	data->group_cfg.converter_selection = (adc_b_unit_id_t)cfg->unit_id;

	data->group_cfg.scan_group_enable = true;
	data->group_cfg.scan_end_interrupt_enable = true;
	data->group_cfg.external_trigger_enable_mask =
		(adc_b_external_trigger_t)ADC_B_EXTERNAL_TRIGGER_NONE;

	data->group_cfg.virtual_channel_count = vc_count;
	data->group_cfg.p_virtual_channels = (adc_b_virtual_channel_cfg_t **)data->group_vcs;

	data->groups[0] = &data->group_cfg;

	memset(&data->scan_cfg, 0, sizeof(data->scan_cfg));
	data->scan_cfg.group_count = 1U;
	data->scan_cfg.p_adc_groups = (adc_b_group_cfg_t **)data->groups;

	return 0;
}

static int adc_rx_sampling_to_states(const struct device *dev, uint32_t sampling_time_ns,
				     uint16_t *states_out)
{
	if (NULL == states_out) {
		return -EINVAL;
	}

	/* Config wasn't set */
	if (sampling_time_ns == 0U) {
		/* Use default sampling time */
		*states_out = ADC_B_SST_MIN;
		return 0;
	}

	/* Get ADC clock frequency */
	const struct adc_rx_adc_b_config *cfg = dev->config;
	const struct device *clock_dev = cfg->clock_dev;
	uint16_t clock_div = cfg->clock_div;
	uint32_t clock_source_hz;

	int ret = clock_control_get_rate(clock_dev, (clock_control_subsys_t)&cfg->clock_subsys,
					 &clock_source_hz);
	if (ret < 0) {
		return ret;
	}

	uint64_t tmp = (uint64_t)sampling_time_ns * clock_source_hz / clock_div;

	tmp = (tmp + ADC_B_SEC_PER_NSEC - 1) / ADC_B_SEC_PER_NSEC;

	if (tmp > ADC_B_SST_MAX) {
		tmp = ADC_B_SST_MAX;
	} else if (tmp < ADC_B_SST_MIN) {
		tmp = ADC_B_SST_MIN;
	}

	*states_out = (uint16_t)tmp;
	return 0;
}

static void adc_rx_set_sampling_table0(adc_b_extended_cfg_t *extend, uint16_t sst)
{
	extend->sampling_state_tables[0] =
		(((uint32_t)sst << R_ADC_B0_ADSSTR0_SST0_Pos) & R_ADC_B0_ADSSTR0_SST0_Msk);
}

static const struct adc_rx_adc_b_channel_cfg *
adc_rx_find_dt_channel(const struct adc_rx_adc_b_config *config, uint8_t channel_id)
{
	for (uint8_t i = 0U; i < config->dt_channel_count; i++) {
		if (config->dt_channels[i].vc_id == channel_id) {
			return &config->dt_channels[i];
		}
	}

	return NULL;
}

static adc_b_clock_source_t adc_rx_get_clock_source(const struct device *clock_dev)
{
	if (clock_dev == NULL) {
		return ADC_B_CLOCK_SOURCE_PCLKA;
	}

	const char *name = clock_dev->name;

	if (strcmp(name, "pclka") == 0) {
		return ADC_B_CLOCK_SOURCE_PCLKA;
	} else if (strcmp(name, "gptclk") == 0) {
		return ADC_B_CLOCK_SOURCE_GPT;
	} else if (strcmp(name, "adcclk") == 0) {
		return ADC_B_CLOCK_SOURCE_ADC;
	} else {
		/* Default to PCLKA for unknown clock sources */
		LOG_DBG("Unknown clock source device '%s', defaulting to PCLKA", name);
		return ADC_B_CLOCK_SOURCE_PCLKA;
	}
}

/* This function validates the virtual channel ID. The maximum virtual channel ID is defined by
 * ADC_B_RX_MAX_VC (50 channels) and the channel must be defined in the device tree.
 */
static int adc_rx_validate_vc_id(const struct adc_rx_adc_b_config *config, uint8_t vc_id)
{
	if (vc_id >= ADC_B_RX_MAX_VC) {
		return -EINVAL;
	}

	if (adc_rx_find_dt_channel(config, vc_id) == NULL) {
		return -EINVAL;
	}

	return 0;
}
/* Channel setup */
static int adc_rx_adc_b_channel_setup(const struct device *dev,
				      const struct adc_channel_cfg *channel_cfg)
{
	struct adc_rx_adc_b_data *data = dev->data;
	const struct adc_rx_adc_b_config *config = dev->config;
	const struct adc_rx_adc_b_channel_cfg *dt_ch;
	fsp_err_t fsp_err;
	int ret;

	/* Validate virtual channel ID */
	ret = adc_rx_validate_vc_id(config, channel_cfg->channel_id);
	if (ret < 0) {
		LOG_INF("Invalid virtual channel id '%d'", channel_cfg->channel_id);
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_INF("Acquisition time is not valid");
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		LOG_INF("unsupported differential mode for now");
		return -ENOTSUP;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_INF("Gain is not valid");
		return -EINVAL;
	}

	/* Find the channel configuration in dts, it already validated */
	dt_ch = adc_rx_find_dt_channel(config, channel_cfg->channel_id);

	data->configured_channels |= BIT(channel_cfg->channel_id);

	ret = adc_rx_adc_b_scan_cfg(dev);
	if (ret < 0) {
		LOG_INF("Failed to configure scan");
		return ret;
	}

	fsp_err = R_ADC_B_ScanCfg(&data->adc_ctrl, &data->scan_cfg);
	if (FSP_SUCCESS != fsp_err) {
		LOG_INF("Failed to set scan configuration");
		return -EIO;
	}

	k_sem_reset(&data->calibrate_sem);

	fsp_err = R_ADC_B_Calibrate(&data->adc_ctrl, NULL);
	if (FSP_SUCCESS != fsp_err) {
		LOG_INF("Failed to calibrate ADC");
		return -EIO;
	}

	/* Take the same and waiting for ADC_EVENT_CALIBRATION_COMPLETE event */
	k_sem_take(&data->calibrate_sem, K_MSEC(10));

	return 0;
}

static int adc_rx_adc_b_check_buffer_size(const struct device *dev,
					  const struct adc_sequence *sequence)
{
	struct adc_rx_adc_b_data *data = dev->data;
	size_t needed;

	uint8_t active = POPCOUNT(sequence->channels & data->configured_channels);

	needed = active * sizeof(uint16_t);

	if (sequence->options != NULL) {
		needed *= (1U + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static int adc_rx_adc_b_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct adc_rx_adc_b_config *config = dev->config;
	struct adc_rx_adc_b_data *data = dev->data;
	fsp_err_t fsp_err;
	const adc_b_extended_cfg_t *extend = (const adc_b_extended_cfg_t *)data->adc_cfg.p_extend;

	int err;

	/* SAR mode only supports 12-bit resolution, will be updated later */
	switch (sequence->resolution) {
	case 10:
		break;
	case 12:
		break;
	case 14:
	case 16:
		if (extend->adc_b_converter_mode[config->unit_id].method ==
		    ADC_B_CONVERSION_METHOD_SAR) {
			LOG_ERR("SAR method supports up to 12-bit resolution only");
			return -ENOTSUP;
		}
		break;
	default:
		LOG_ERR("unsupported resolution %d", sequence->resolution);
		return -ENOTSUP;
	}

	if ((sequence->channels & ~data->configured_channels) != 0) {
		LOG_ERR("channels 0x%08x not configured (configured: 0x%08x)", sequence->channels,
			data->configured_channels);
		return -EINVAL;
	}

	err = adc_rx_adc_b_check_buffer_size(dev, sequence);
	if (err) {
		LOG_ERR("buffer size too small");
		return err;
	}

	data->buf_id = 0;
	data->buf = sequence->buffer;

	if (sequence->calibrate) {
		/* Start calibration process */
		k_sem_reset(&data->calibrate_sem);
		fsp_err = R_ADC_B_Calibrate(&data->adc_ctrl, NULL);
		if (FSP_SUCCESS != fsp_err) {
			LOG_INF("Failed to start calibration");
			return -EIO;
		}
		k_sem_take(&data->calibrate_sem, K_MSEC(10));
	}

	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_rx_adc_b_read_async(const struct device *dev, const struct adc_sequence *sequence,
				   struct k_poll_signal *async)
{
	struct adc_rx_adc_b_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, async ? true : false, async);
	err = adc_rx_adc_b_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

static int adc_rx_adc_b_read(const struct device *dev, const struct adc_sequence *sequence)
{
	return adc_rx_adc_b_read_async(dev, sequence, NULL);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_rx_adc_b_data *data = CONTAINER_OF(ctx, struct adc_rx_adc_b_data, ctx);
	const struct adc_rx_adc_b_config *cfg = data->dev->config;

	data->channels = ctx->sequence.channels;

	fsp_err_t fsp_err =
		R_ADC_B_ScanGroupStart(&data->adc_ctrl, (adc_group_mask_t)BIT(cfg->unit_id));

	if (FSP_SUCCESS != fsp_err) {
		LOG_ERR("Failed to start ADC scan group %u", cfg->unit_id);
		adc_context_complete(ctx, -EIO);
	}
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_rx_adc_b_data *data = CONTAINER_OF(ctx, struct adc_rx_adc_b_data, ctx);

	if (repeat_sampling) {
		data->buf_id = 0;
	}
}

static int adc_rx_adc_b_init(const struct device *dev)
{
	const struct adc_rx_adc_b_config *config = dev->config;
	struct adc_rx_adc_b_data *data = dev->data;
	adc_b_extended_cfg_t *extend = (adc_b_extended_cfg_t *)data->adc_cfg.p_extend;
	uint16_t sst;
	int ret;
	fsp_err_t fsp_err;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	k_sem_init(&data->calibrate_sem, 0, 1);

	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->clock_subsys);
	if (ret < 0) {
		return ret;
	}

	ret = adc_rx_sampling_to_states(dev, config->sampling_time_ns, &sst);
	if (ret != 0) {
		LOG_DBG("Failed to convert sampling time to states");
		return ret;
	}

	/* Use 1 sampling state table */
	adc_rx_set_sampling_table0(extend, sst);

	/* Get clock source from device name and update extend config */
	adc_b_clock_source_t clock_source = adc_rx_get_clock_source(config->clock_dev);

	/* mapping DT value into e_adc_b_clock_divider */
	uint8_t divr = config->clock_div - 1U;
	extend->clock_control_data = ((clock_source << R_ADC_B0_ADCLKCR_CLKSEL_Pos) |
				      (divr << R_ADC_B0_ADCLKCR_DIVR_Pos));

	fsp_err = R_ADC_B_Open(&data->adc_ctrl, &data->adc_cfg);
	if (FSP_SUCCESS != fsp_err) {
		return -EIO;
	}

	config->irq_configure(dev);

	adc_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static void renesas_rx_adc_callback(adc_callback_args_t *p_args)
{
	const struct device *dev = p_args->p_context;
	struct adc_rx_adc_b_data *data = dev->data;
	const struct adc_rx_adc_b_config *cfg = dev->config;
	fsp_err_t fsp_err;
	uint16_t *sample_buffer = (uint16_t *)data->buf;

	if (p_args->event == ADC_EVENT_SCAN_COMPLETE) {
		if (sample_buffer == NULL) {
			adc_context_on_sampling_done(&data->ctx, dev);
			return;
		}

		for (uint8_t i = 0U; i < cfg->dt_channel_count; i++) {
			uint8_t vc_id = cfg->dt_channels[i].vc_id;
			/* Skip if not configured or not in current sequence */
			if (!(data->configured_channels & BIT(vc_id)) ||
			    !(data->channels & BIT(vc_id))) {
				continue;
			}

			uint32_t sample = 0;
			uint8_t phys_ch = cfg->dt_channels[i].input_positive;

			fsp_err = R_ADC_B_Read32(&data->adc_ctrl, (adc_channel_t)phys_ch, &sample);
			if (fsp_err != FSP_SUCCESS) {
				LOG_ERR("Failed to read sample for channel %d (phys ch %d)", vc_id,
					phys_ch);
				adc_context_complete(&data->ctx, -EIO);
				return;
			}

			sample_buffer[data->buf_id] = (uint16_t)sample;
			data->buf_id++;
		}

		adc_context_on_sampling_done(&data->ctx, dev);
	} else if (p_args->event == ADC_EVENT_CALIBRATION_COMPLETE) {
		k_sem_give(&data->calibrate_sem);
	} else if (p_args->event == ADC_EVENT_CONVERSION_ERROR) {
		LOG_ERR("Conversion error event received");
		adc_context_complete(&data->ctx, -EIO);
	} else {
		LOG_WRN("Unhandled ADC event: %d", p_args->event);
	}
}

/* ADC_B self-calibration procedure request perform sel-calibration for all unit
 * In the RX ADC, 3 units use the group interrupt AL5.
 */

static int adc_rx_irq_setup(const struct device *dev, const struct adc_rx_grp_irq_entry *irqs,
			    void *context)
{
	int err;

	err = rx_grp_intc_callback_set(dev, irqs->irq, (void (*)(void))irqs->isr, context);

	if (err != 0) {
		return err;
	}

	return rx_grp_intc_enable(dev, irqs->irq);
}

static void adc_rx_group_irq_configure(const struct device *dev)
{
	const struct adc_rx_adc_b_config *cfg = dev->config;
	struct adc_rx_adc_b_data *data = dev->data;
	int err;

	/* Setup calibration end IRQs for 3 units */
	for (int i = 0; i < ARRAY_SIZE(calend_irqs); i++) {
		err = adc_rx_irq_setup(cfg->calend_ctrl, &calend_irqs[i], (void *)&data->adc_ctrl);
		if (err != 0) {
			LOG_ERR("Failed to setup calibration end IRQ for unit %d", i);
			return;
		}
	}
}

/* Use 1 scan group for scan end */
#define IRQ_CONFIGURE_FUNC(idx)                                                                    \
	static void adc_rx_configure_func_##idx(const struct device *dev)                          \
	{                                                                                          \
		const struct adc_rx_adc_b_config *cfg = dev->config;                               \
		struct adc_rx_adc_b_data *data = dev->data;                                        \
                                                                                                   \
		int scanend_irq = DT_INST_IRQ_BY_NAME(idx, scanend, irq);                          \
		if (128 <= scanend_irq && scanend_irq < 144) {                                     \
			R_ICU->SLIXR[scanend_irq - 128].SLIXR =                                    \
				ADC_B_ICU_SCAN_END_EVENT(DT_INST_PROP(idx, unit_id));              \
		} else if (144 <= scanend_irq) {                                                   \
			R_ICU->SLIR[scanend_irq - 144].SLIR =                                      \
				ADC_B_ICU_SCAN_END_EVENT(DT_INST_PROP(idx, unit_id));              \
		}                                                                                  \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(idx, scanend, irq),                                \
			    DT_INST_IRQ_BY_NAME(idx, scanend, priority),                           \
			    ADC_B_SCAN_END_ISR(DT_INST_PROP(idx, unit_id)),                        \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(scanend_irq);                                                           \
                                                                                                   \
		adc_rx_group_irq_configure(dev);                                                   \
                                                                                                   \
		int err = rx_grp_intc_callback_set(                                                \
			cfg->calend_ctrl, CONS_UNIT_IRQ(DT_INST_PROP(idx, unit_id)),               \
			ADC_B_ERR_ISR(DT_INST_PROP(idx, unit_id)), (void *)&data->adc_ctrl);       \
		if (err != 0) {                                                                    \
			LOG_ERR("Failed to setup conversion error IRQ for unit %d",                \
				DT_INST_PROP(idx, unit_id));                                       \
		}                                                                                  \
                                                                                                   \
		err = rx_grp_intc_enable(cfg->calend_ctrl,                                         \
					 CONS_UNIT_IRQ(DT_INST_PROP(idx, unit_id)));               \
		if (err != 0) {                                                                    \
			LOG_ERR("Failed to enable interrupt");                                     \
		}                                                                                  \
	}

#define IRQ_CONFIGURE_DEFINE(idx) .irq_configure = adc_rx_configure_func_##idx

#define ADC_RX_ADC_B_DT_CH_INIT(node_id)                                                           \
	{                                                                                          \
		.vc_id = DT_REG_ADDR(node_id),                                                     \
		.input_positive = DT_PROP(node_id, zephyr_input_positive),                         \
		.resolution = DT_PROP_OR(node_id, zephyr_resolution, 12),                          \
	},

#define ADC_RX_ADC_B_INIT(idx)                                                                     \
	IRQ_CONFIGURE_FUNC(idx)                                                                    \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
	static const struct adc_rx_adc_b_channel_cfg adc_rx_adc_b_channels_##idx[] = {             \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADC_RX_ADC_B_DT_CH_INIT)};                  \
	static adc_b_isr_cfg_t g_adc_isr_cfg_##idx = {                                             \
		.ISR_CFG_SCANEND_IRQ(DT_INST_PROP(idx, unit_id)) =                                 \
			(IRQn_Type)DT_INST_IRQ_BY_NAME(idx, scanend, irq),                         \
		.ISR_CFG_SCANEND_IPL(DT_INST_PROP(idx, unit_id)) =                                 \
			DT_INST_IRQ_BY_NAME(idx, scanend, priority),                               \
		.calibration_end_ipl_adc_0 = DT_IRQ_BY_NAME(ADC0_NODE, calend, priority),          \
		.calibration_end_ipl_adc_1 = DT_IRQ_BY_NAME(ADC1_NODE, calend, priority),          \
		.calibration_end_ipl_adc_2 = DT_IRQ_BY_NAME(ADC2_NODE, calend, priority),          \
		.calibration_end_irq_adc_0 = VECTOR_NUMBER_ADC_CALEND0,                            \
		.calibration_end_irq_adc_1 = VECTOR_NUMBER_ADC_CALEND1,                            \
		.calibration_end_irq_adc_2 = VECTOR_NUMBER_ADC_CALEND2,                            \
		.ISR_CFG_CONVSERR_IRQ(DT_INST_PROP(idx, unit_id)) =                                \
			VECTOR_NUMBER_ADC_CONVSERR(DT_INST_PROP(idx, unit_id)),                    \
		.ISR_CFG_CONVSERR_IPL(DT_INST_PROP(idx, unit_id)) =                                \
			DT_INST_IRQ_BY_NAME(idx, convserr, priority),                              \
	};                                                                                         \
	static DEVICE_API(adc, adc_rx_adc_b_api_##idx) = {                                         \
		.channel_setup = adc_rx_adc_b_channel_setup,                                       \
		.read = adc_rx_adc_b_read,                                                         \
		.ref_internal = DT_PROP_OR(DT_INST_PARENT(idx), vref_mv, 3300),                    \
		IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = adc_rx_adc_b_read_async))};               \
	static const struct adc_rx_adc_b_config adc_rx_adc_b_config_##idx = {                      \
		.regs = (R_ADC_B0_Type *)DT_REG_ADDR(DT_INST_PARENT(idx)),                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                       \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(idx))),                   \
		.clock_subsys =                                                                    \
			{                                                                          \
				.mstp = (uint32_t)DT_CLOCKS_CELL_BY_IDX(DT_INST_PARENT(idx), 0,    \
									mstp),                     \
				.stop_bit = (uint32_t)DT_CLOCKS_CELL_BY_IDX(DT_INST_PARENT(idx),   \
									    0, stop_bit),          \
			},                                                                         \
		.clock_div = DT_PROP_OR(DT_INST_PARENT(idx), clock_div, 3),                        \
		.sampling_time_ns = DT_INST_PROP_OR(idx, sampling_time_ns, 0),                     \
		.unit_id = DT_INST_PROP(idx, unit_id),                                             \
		.channel_available_mask = DT_INST_PROP_OR(idx, channel_available_mask, 0),         \
		.dt_channels = adc_rx_adc_b_channels_##idx,                                        \
		.dt_channel_count = ARRAY_SIZE(adc_rx_adc_b_channels_##idx),                       \
		.calend_ctrl = DEVICE_DT_GET(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(idx), calend)),       \
		IRQ_CONFIGURE_DEFINE(idx),                                                         \
	};                                                                                         \
                                                                                                   \
	static adc_b_extended_cfg_t g_adc_cfg_extend_##idx = {                                     \
		.sync_operation_control = 0,                                                       \
		.adc_b_converter_mode[DT_INST_PROP(idx, unit_id)] =                                \
			{                                                                          \
				.mode = (ADC_B_CONVERTER_MODE_SINGLE_SCAN),                        \
				.method = ADC_B_DT_METHOD(idx),                                    \
			},                                                                         \
		.converter_selection_0 =                                                           \
			ADC_B_SGADS(0, 0) | ADC_B_SGADS(1, 1) | ADC_B_SGADS(2, 2),                 \
		.converter_selection_1 = 0,                                                        \
		.converter_selection_2 = 0,                                                        \
		.fifo_enable_mask = 0,                                                             \
		.fifo_interrupt_enable_mask = 0,                                                   \
		.start_trigger_delay_table = {0},                                                  \
		.calibration_adc_state = ((256 << R_ADC_B0_ADCALSTCR_CALADSST_Pos) |               \
					  (20 << R_ADC_B0_ADCALSTCR_CALADCST_Pos)),                \
		.calibration_sample_and_hold = ((95 << R_ADC_B0_ADCALSHCR_CALSHSST_Pos) |          \
						(5 << R_ADC_B0_ADCALSHCR_CALSHHST_Pos)),           \
		.p_isr_cfg = &g_adc_isr_cfg_##idx,                                                 \
		.sampling_state_tables =                                                           \
			{                                                                          \
				0,                                                                 \
			},                                                                         \
		.sample_and_hold_enable_mask = (ADC_B_SAMPLE_AND_HOLD_MASK_NONE),                  \
		.conversion_state =                                                                \
			((20 << R_ADC_B0_ADCNVSTR_CST0_Pos) | (20 << R_ADC_B0_ADCNVSTR_CST1_Pos) | \
			 (20 << R_ADC_B0_ADCNVSTR_CST2_Pos)),                                      \
		.user_offset_tables = {0},                                                         \
		.user_gain_tables = {0},                                                           \
		.limiter_clip_interrupt_enable_mask = (0x00),                                      \
		.limiter_clip_tables = {0},                                                        \
		.adc_filter_selection =                                                            \
			{                                                                          \
				[DT_INST_PROP(idx, unit_id)] = ADC_B_DFSEL_SINC3_ALL,              \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	static struct adc_rx_adc_b_data adc_rx_adc_b_data_##idx = {                                \
		ADC_CONTEXT_INIT_TIMER(adc_rx_adc_b_data_##idx, ctx),                              \
		ADC_CONTEXT_INIT_LOCK(adc_rx_adc_b_data_##idx, ctx),                               \
		ADC_CONTEXT_INIT_SYNC(adc_rx_adc_b_data_##idx, ctx),                               \
		.dev = DEVICE_DT_INST_GET(idx),                                                    \
		.adc_cfg =                                                                         \
			{                                                                          \
				.unit = DT_INST_PROP(idx, unit_id),                                \
				.alignment = (adc_alignment_t)ADC_ALIGNMENT_RIGHT,                 \
				.p_callback = renesas_rx_adc_callback,                             \
				.p_context = (void *)DEVICE_DT_GET(DT_DRV_INST(idx)),              \
				.p_extend = &g_adc_cfg_extend_##idx,                               \
			},                                                                         \
		.buf = NULL,                                                                       \
		.buf_id = 0,                                                                       \
		.channels = 0,                                                                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, adc_rx_adc_b_init, NULL, &adc_rx_adc_b_data_##idx,              \
			      &adc_rx_adc_b_config_##idx, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,   \
			      &adc_rx_adc_b_api_##idx)

DT_INST_FOREACH_STATUS_OKAY(ADC_RX_ADC_B_INIT);
