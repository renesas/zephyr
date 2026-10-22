/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_adc_b

#include <string.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_rx_cgc.h>
#include <zephyr/drivers/interrupt_controller/intc_renesas_rx_grp_int_fsp.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/adc/adc.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "fsp_common_api.h"
#include "r_adc_device_types.h"
#include <instances/r_adc_b.h>

LOG_MODULE_REGISTER(renesas_rx_adc_b, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE
#include "adc_context.h"

#define ADC_B_RX_MAX_VC            32U /* Max virtual channels addressable via Zephyr ADC API */
#define ADC_B_RX_MAX_VC_PER_GROUP  8U  /* Max virtual channels per scan group */
#define ADC_B_RX_MAX_SCAN_GROUPS   12U /* Max scan groups */
#define ADC_B_RX_MAX_SAMPLE_TABLES 15U /* Max sampling-state tables (ADSSTR0..7) */
#define ADC_B_RX_MAX_UNITS         3U  /* ADC0, ADC1 and ADC2 */

/* Sampling state table */
#define ADC_B_SST_MIN      2U
#define ADC_B_SST_MAX      1023U
#define ADC_B_NSEC_PER_SEC 1000000000L

#define ADC_B_CAL_TIMEOUT K_MSEC(100)

/* Group interrupt shared by every unit's calend/convserr. */
#define GROUP_INT_AL5 DT_IRQ(DT_NODELABEL(group_irq_al5), irq)

#if DT_NODE_EXISTS(DT_NODELABEL(adc0))
#define ADC0_NODE DT_NODELABEL(adc0)
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(adc1))
#define ADC1_NODE DT_NODELABEL(adc1)
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(adc2))
#define ADC2_NODE DT_NODELABEL(adc2)
#endif

#define CALEND_UNIT_IRQ(node)   DT_IRQ_BY_NAME(node, calend, irq)
#define CONVSERR_UNIT_IRQ(node) DT_IRQ_BY_NAME(node, convserr, irq)

#define VECTOR_NUMBER_ADC_CALEND(node) ((IRQn_Type)(CALEND_UNIT_IRQ(node) << 8U | GROUP_INT_AL5))

#define VECTOR_NUMBER_ADC_CONVSERR(node)                                                           \
	((IRQn_Type)(CONVSERR_UNIT_IRQ(node) << 8U | GROUP_INT_AL5))

/* FSP ISR symbols selected per unit / scan group at build time */
#define ADC_B_ERR_ISR(unit)     UTIL_CAT(UTIL_CAT(adc_b_err, unit), _isr)
#define ADC_B_CAL_END_ISR(unit) UTIL_CAT(UTIL_CAT(adc_b_calend, unit), _isr)
#define ADC_B_SCAN_END_ISR(grp) UTIL_CAT(UTIL_CAT(adc_b_adi, grp), _isr)

/* ICU selectable interrupt event for a scan-group's scan-end */
#define ADC_B_ICU_SCAN_END_EVENT(grp) UTIL_CAT(ICU_EVENT_ADC_ADI, grp)

/* Irq/ipl macros */
#define ISR_CFG_CALEND_IRQ(unit)   UTIL_CAT(calibration_end_irq_adc_, unit)
#define ISR_CFG_CALEND_IPL(unit)   UTIL_CAT(calibration_end_ipl_adc_, unit)
#define ISR_CFG_CONVSERR_IRQ(unit) UTIL_CAT(conversion_error_irq_adc_, unit)
#define ISR_CFG_CONVSERR_IPL(unit) UTIL_CAT(conversion_error_ipl_adc_, unit)
#define ISR_CFG_SCANEND_IRQ(grp)   UTIL_CAT(scan_end_irq_group_, grp)
#define ISR_CFG_SCANEND_IPL(grp)   UTIL_CAT(scan_end_ipl_group_, grp)

/* isr_cfg calibration-end fields for one unit (if it declares a calend IRQ). */
#define ADC_RX_UNIT_CALEND_ISR_CFG_FIELD(node_id)                                                  \
	IF_ENABLED(DT_IRQ_HAS_NAME(node_id, calend),                                                \
		   (.ISR_CFG_CALEND_IRQ(DT_PROP(node_id, unit_id)) =                                \
			    VECTOR_NUMBER_ADC_CALEND(node_id),                                     \
		    .ISR_CFG_CALEND_IPL(DT_PROP(node_id, unit_id)) =                                \
			    DT_IRQ_BY_NAME(node_id, calend, priority),))

/* Method definition */
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

/* extern FSP scan-end ISRs (one per hardware scan-group 0..11) */
extern void adc_b_adi0_isr(void);
extern void adc_b_adi1_isr(void);
extern void adc_b_adi2_isr(void);
extern void adc_b_adi3_isr(void);
extern void adc_b_adi4_isr(void);
extern void adc_b_adi5_isr(void);
extern void adc_b_adi6_isr(void);
extern void adc_b_adi7_isr(void);
extern void adc_b_adi8_isr(void);
extern void adc_b_adi9_isr(void);
extern void adc_b_adi10_isr(void);
extern void adc_b_adi11_isr(void);

/* Calibration ISRs (one per converter unit) */
extern void adc_b_calend0_isr(void);
extern void adc_b_calend1_isr(void);
extern void adc_b_calend2_isr(void);

/* Conversion-error ISRs (one per converter unit) */
extern void adc_b_err0_isr(void);
extern void adc_b_err1_isr(void);
extern void adc_b_err2_isr(void);

struct adc_rx_grp_irq_entry {
	int irq;
	void (*isr)(void);
};

/* One calend entry per unit that declares a calend IRQ (calibration is
 * performed on all units, so disabled units still contribute their vector).
 */
#define ADC_RX_CALEND_IRQ_ENTRY(node_id)                                                           \
	IF_ENABLED(DT_IRQ_HAS_NAME(node_id, calend),                                                \
		   ({CALEND_UNIT_IRQ(node_id), ADC_B_CAL_END_ISR(DT_PROP(node_id, unit_id))},))

static const struct adc_rx_grp_irq_entry calend_irqs[] = {ADC_RX_CALEND_IRQ_ENTRY(
	ADC0_NODE) ADC_RX_CALEND_IRQ_ENTRY(ADC1_NODE) ADC_RX_CALEND_IRQ_ENTRY(ADC2_NODE)};

/* Per virtual-channel configuration parsed from DTS. */
struct adc_rx_adc_b_channel_cfg {
	uint8_t vc_id;          /* reg of channel@N, virtual channel id */
	uint8_t input_positive; /* zephyr,input-positive */
	uint8_t resolution;     /* zephyr,resolution */
	uint8_t oversampling;   /* zephyr,oversampling */
	uint8_t scan_group_id;  /* DT_REG_ADDR of renesas,scan-group phandle */
	uint16_t acq_time;      /* zephyr,acquisition-time (ADC_ACQ_TIME encoding) */
};

/* Compile-time map of scan-group ID -> owning converter unit ID. */
struct adc_b_sg_unit_map {
	uint8_t scan_group_id;
	uint8_t unit_id;
};

#define ADC_B_SG_MAP_ENTRY(node_id)                                                                \
	{                                                                                          \
		.scan_group_id = DT_REG_ADDR_RAW(node_id),                                         \
		.unit_id = DT_PROP(node_id, renesas_converter_unit_id),                            \
	},

static const struct adc_b_sg_unit_map adc_b_sg_unit_map[] = {
	DT_FOREACH_STATUS_OKAY(renesas_rx_adc_b_scan_group, ADC_B_SG_MAP_ENTRY)};

/* Owning converter unit ID for a scan-group, or UINT8_MAX if not found. */
static inline uint8_t adc_b_sg_to_unit(uint8_t scan_group_id)
{
	for (size_t i = 0U; i < ARRAY_SIZE(adc_b_sg_unit_map); i++) {
		if (adc_b_sg_unit_map[i].scan_group_id == scan_group_id) {
			return adc_b_sg_unit_map[i].unit_id;
		}
	}
	return UINT8_MAX;
}

struct adc_rx_adc_b_config {
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	const struct clock_control_rx_subsys_cfg clock_subsys;
	uint8_t clock_div;

	/* ADC unit id (0 for ADC0, 1 for ADC1, 2 for ADC2). */
	uint8_t unit_id;

	/* Bitmask of valid zephyr,input-positive values for this unit. */
	uint32_t channel_available_mask;

	/* Channel config array built from this unit's DTS child nodes. */
	const struct adc_rx_adc_b_channel_cfg *dt_channels;
	uint8_t dt_channel_count;

	void (*irq_configure)(const struct device *dev);

	/* RX group interrupt controller carrying calend/convserr. */
	const struct device *group_intc;
};

struct adc_rx_adc_b_data {
	struct adc_context ctx;
	const struct device *dev;

	/** Pointer to memory where next sample will be written */
	uint16_t *buf;
	uint16_t buf_id;
	/** Mask with channels that will be sampled */
	uint32_t channels;

	/* Group-completion tracking for multi-group scans */
	uint32_t pending_group_mask;
	uint32_t completed_group_mask;

	/* Deferred scan-start work: when adc_context_start_sampling is called
	 * from ISR context (extra_samplings chaining with interval_us=0).
	 */
	struct k_work start_work;

	/* bitmask of configured channels (channel_setup) */
	uint32_t configured_channels;
};

/* ADC0/1/2 are units of one physical peripheral: opening and (re)configuring
 * the scan always covers the whole thing. So only one control block is
 * opened, and every scan rebuild includes all units' channels, or
 * reconfiguring one unit would wipe out the others'.
 */
struct adc_rx_adc_b_shared {
	bool opened;
	/* Set by channel_setup() when the channel set changed; cleared once
	 * the scan has been (re)programmed and calibrated in start_read().
	 */
	bool scan_cfg_dirty;

	adc_b_instance_ctrl_t adc_ctrl;
	adc_cfg_t adc_cfg;

	adc_b_scan_cfg_t scan_cfg;
	adc_b_group_cfg_t group_cfg[ADC_B_RX_MAX_SCAN_GROUPS];
	adc_b_group_cfg_t *groups[ADC_B_RX_MAX_SCAN_GROUPS];
	adc_b_virtual_channel_cfg_t vc_cfg[ADC_B_RX_MAX_VC];
	adc_b_virtual_channel_cfg_t *group_vcs[ADC_B_RX_MAX_SCAN_GROUPS][ADC_B_RX_MAX_VC_PER_GROUP];

	struct k_sem calibrate_sem;
};

static struct adc_rx_adc_b_shared adc_b_shared;

/* Serializes access to adc_b_shared across all ADC devices. */
K_MUTEX_DEFINE(adc_b_shared_lock);

/* Held across a whole conversion so the units never overlap. Separate from
 * adc_b_shared_lock to avoid deadlocking that lock's workqueue continuation.
 */
K_SEM_DEFINE(adc_b_conversion_owner, 1, 1);

/* adc_context_complete() is the true end of a sequence (sync or async).
 * Release ADC_B ownership here rather than after wait_for_completion().
 */
static void adc_context_on_complete(struct adc_context *ctx, int status)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(status);

	k_sem_give(&adc_b_conversion_owner);
}

/* unit_id -> owning device, used to route callback events (which only
 * identify a scan-group or unit) back to the right Zephyr device.
 */
static const struct device *adc_b_unit_dev[ADC_B_RX_MAX_UNITS];

static adc_b_extended_cfg_t g_adc_cfg_extend;

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

/* Map zephyr,oversampling (n -> 2^n samples) to the FSP average count. */
static uint8_t adc_rx_ovs_to_avg_count(uint8_t n)
{
	switch (n) {
	case 1:
		return ADC_B_ADD_AVERAGE_2;
	case 2:
		return ADC_B_ADD_AVERAGE_4;
	case 3:
		return ADC_B_ADD_AVERAGE_8;
	case 4:
		return ADC_B_ADD_AVERAGE_16;
	case 5:
		return ADC_B_ADD_AVERAGE_32;
	case 6:
		return ADC_B_ADD_AVERAGE_64;
	case 7:
		return ADC_B_ADD_AVERAGE_128;
	case 8:
		return ADC_B_ADD_AVERAGE_256;
	case 9:
		return ADC_B_ADD_AVERAGE_512;
	case 10:
		return ADC_B_ADD_AVERAGE_1024;
	default:
		return ADC_B_ADD_AVERAGE_1; /* n == 0 -> off */
	}
}

/* Whether a virtual channel uses the oversampling path (SAR: no, OVERSAMPLE:
 * yes, HYBRID: only if oversampling > 0).
 */
static bool adc_rx_vc_uses_oversample(adc_b_conversion_method_t method, uint8_t oversampling)
{
	switch (method) {
	case ADC_B_CONVERSION_METHOD_OVERSAMPLE:
		return true;
	case ADC_B_CONVERSION_METHOD_HYBRID:
		return (oversampling > 0U);
	case ADC_B_CONVERSION_METHOD_SAR:
	default:
		return false;
	}
}

/* Initialize virtual-channel configuration from its DTS channel node. */
static void adc_rx_adc_b_vc_configure(const struct adc_rx_adc_b_channel_cfg *ch,
				      adc_b_virtual_channel_cfg_t *vc,
				      adc_b_conversion_method_t method, uint8_t sample_table_id)
{
	memset(vc, 0, sizeof(*vc));

	vc->channel_id = (adc_b_virtual_channel_t)ch->vc_id;
	/* ADCHCRn.SGSEL: 0 = no group, N + 1 selects scan group N */
	vc->channel_cfg_u.channel_cfg_bits.group = ch->scan_group_id + 1U;
	vc->channel_cfg_u.channel_cfg_bits.channel = (adc_channel_t)ch->input_positive;
	vc->channel_cfg_u.channel_cfg_bits.sample_table_id = sample_table_id;

	if (adc_rx_vc_uses_oversample(method, ch->oversampling)) {
		vc->channel_control_a_u.channel_control_a_bits.digital_filter_id = 1;
		vc->channel_control_b_u.channel_control_b_bits.addition_average_mode =
			ADC_B_ADD_AVERAGE_AVERAGE_ENABLE;
		vc->channel_control_b_u.channel_control_b_bits.addition_average_count =
			adc_rx_ovs_to_avg_count(ch->oversampling);
	}

	vc->channel_control_c_u.channel_control_c_bits.channel_data_format =
		rx_adc_resolution_to_format(ch->resolution);
	vc->channel_control_c_u.channel_control_c_bits.data_is_unsigned = true;
}

/* Decode a Zephyr ADC_ACQ_TIME-encoded value to nanoseconds.
 * ADC_ACQ_TIME_DEFAULT yields 0 (caller falls back to the minimum SST).
 */
static int adc_rx_acq_time_to_ns(uint16_t acq_time, uint32_t *ns_out)
{
	uint16_t value = ADC_ACQ_TIME_VALUE(acq_time);
	uint32_t ns;

	if (acq_time == ADC_ACQ_TIME_DEFAULT) {
		ns = 0U;
	} else {
		switch (ADC_ACQ_TIME_UNIT(acq_time)) {
		case ADC_ACQ_TIME_NANOSECONDS:
			ns = value;
			break;
		case ADC_ACQ_TIME_MICROSECONDS:
			ns = (uint32_t)value * 1000U;
			break;
		default:
			return -ENOTSUP;
		}
	}

	if (ns_out != NULL) {
		*ns_out = ns;
	}

	return 0;
}

/* Convert a sampling time in nanoseconds to an ADC_B sampling-state-table
 * register value (SST cycles), clamped to the legal [MIN, MAX] range.
 * A 0 ns input maps to the minimum legal SST.
 */
static int adc_rx_sampling_to_states(const struct device *dev, uint32_t sampling_time_ns,
				     uint16_t *states_out)
{
	const struct adc_rx_adc_b_config *cfg = dev->config;
	uint32_t clock_source_hz;
	uint64_t tmp;
	int ret;

	if (states_out == NULL) {
		return -EINVAL;
	}

	if (sampling_time_ns == 0U) {
		*states_out = ADC_B_SST_MIN;
		return 0;
	}

	ret = clock_control_get_rate(cfg->clock_dev, (clock_control_subsys_t)&cfg->clock_subsys,
				     &clock_source_hz);
	if (ret < 0) {
		return ret;
	}

	tmp = (uint64_t)sampling_time_ns * clock_source_hz / cfg->clock_div;
	tmp = (tmp + ADC_B_NSEC_PER_SEC - 1) / ADC_B_NSEC_PER_SEC;

	if (tmp > ADC_B_SST_MAX) {
		tmp = ADC_B_SST_MAX;
	} else if (tmp < ADC_B_SST_MIN) {
		tmp = ADC_B_SST_MIN;
	} else {
		/* do nothing */
	}

	*states_out = (uint16_t)tmp;
	return 0;
}

/* Find or allocate a sampling-state table for ns. Returns the table index
 * (== sample_table_id), deduplicating equal acquisition-times, or a negative
 * errno when ADC_B_RX_MAX_SAMPLE_TABLES is exceeded.
 */
static int adc_rx_alloc_sample_table(const struct device *dev, adc_b_extended_cfg_t *extend,
				     uint32_t *table_ns, uint8_t *table_count, uint32_t ns)
{
	uint16_t sst;
	int ret;

	for (uint8_t i = 0U; i < *table_count; i++) {
		if (table_ns[i] == ns) {
			return i;
		}
	}

	if (*table_count >= ADC_B_RX_MAX_SAMPLE_TABLES) {
		LOG_ERR("too many distinct acquisition-times (max %u)", ADC_B_RX_MAX_SAMPLE_TABLES);
		return -ENOMEM;
	}

	ret = adc_rx_sampling_to_states(dev, ns, &sst);
	if (ret < 0) {
		return ret;
	}

	uint8_t idx = (*table_count)++;

	table_ns[idx] = ns;
	/* uint16 view of the ADSSTR0..7 union (two SST per register). */
	extend->sampling_state_table[idx] = sst;

	return idx;
}

/* Populate the virtual-channel list for one already-discovered scan-group g,
 * pulling channels from every registered unit, not just one.
 */
static int adc_rx_adc_b_populate_group(uint8_t g, uint32_t *table_ns, uint8_t *table_count,
				       uint8_t *vc_global_idx)
{
	adc_b_extended_cfg_t *extend = &g_adc_cfg_extend;
	uint8_t sg_id = (uint8_t)adc_b_shared.group_cfg[g].scan_group_id;
	uint8_t vc_in_group = 0U;

	for (uint8_t u = 0U; u < ARRAY_SIZE(adc_b_unit_dev); u++) {
		const struct device *udev = adc_b_unit_dev[u];

		if (udev == NULL) {
			continue;
		}

		const struct adc_rx_adc_b_config *cfg = udev->config;
		struct adc_rx_adc_b_data *udata = udev->data;
		adc_b_conversion_method_t method = extend->adc_b_converter_mode[u].method;

		for (uint8_t i = 0U; i < cfg->dt_channel_count; i++) {
			const struct adc_rx_adc_b_channel_cfg *ch = &cfg->dt_channels[i];
			uint32_t acq_ns;
			int ret;

			if (!(udata->configured_channels & BIT(ch->vc_id))) {
				continue;
			}
			if (ch->scan_group_id != sg_id) {
				continue;
			}

			ret = adc_rx_validate_phys_channel(cfg->channel_available_mask,
							   ch->input_positive);
			if (ret < 0) {
				return ret;
			}

			/* SAR method only supports up to 12-bit. */
			if (!adc_rx_vc_uses_oversample(method, ch->oversampling) &&
			    ch->resolution > 12U) {
				LOG_ERR("vc %u: SAR-path channel limited to 12-bit (got %u)",
					ch->vc_id, ch->resolution);
				return -EINVAL;
			}

			if (vc_in_group >= ADC_B_RX_MAX_VC_PER_GROUP) {
				LOG_ERR("Too many channels in scan-group %u (max %u)", sg_id,
					ADC_B_RX_MAX_VC_PER_GROUP);
				return -ENOMEM;
			}
			if (*vc_global_idx >= ADC_B_RX_MAX_VC) {
				return -ENOMEM;
			}

			/* Allocate a sampling-state table for this channel's
			 * acquisition-time (deduplicated across channels).
			 */
			ret = adc_rx_acq_time_to_ns(ch->acq_time, &acq_ns);
			if (ret < 0) {
				return ret;
			}

			ret = adc_rx_alloc_sample_table(udev, extend, table_ns, table_count,
							acq_ns);
			if (ret < 0) {
				return ret;
			}

			adc_rx_adc_b_vc_configure(ch, &adc_b_shared.vc_cfg[*vc_global_idx], method,
						  (uint8_t)ret);
			adc_b_shared.group_vcs[g][vc_in_group] =
				&adc_b_shared.vc_cfg[*vc_global_idx];
			vc_in_group++;
			(*vc_global_idx)++;
		}
	}

	adc_b_shared.group_cfg[g].virtual_channel_count = vc_in_group;
	adc_b_shared.group_cfg[g].p_virtual_channels =
		(adc_b_virtual_channel_cfg_t **)adc_b_shared.group_vcs[g];

	return 0;
}

/* Build adc_b_scan_cfg_t with one adc_b_group_cfg_t per scan-group that has
 * at least one configured channel, across every registered unit. Must be
 * called with adc_b_shared_lock held.
 */
static int adc_rx_adc_b_scan_cfg(void)
{
	uint8_t group_count = 0U;
	uint8_t vc_global_idx = 0U;
	uint32_t table_ns[ADC_B_RX_MAX_SAMPLE_TABLES];
	uint8_t table_count = 0U;

	/* Discover unique scan-group IDs across every unit's configured channels */
	for (uint8_t u = 0U; u < ARRAY_SIZE(adc_b_unit_dev); u++) {
		const struct device *udev = adc_b_unit_dev[u];

		if (udev == NULL) {
			continue;
		}

		const struct adc_rx_adc_b_config *cfg = udev->config;
		struct adc_rx_adc_b_data *udata = udev->data;

		for (uint8_t i = 0U; i < cfg->dt_channel_count; i++) {
			const struct adc_rx_adc_b_channel_cfg *ch = &cfg->dt_channels[i];
			bool already_added = false;

			if (!(udata->configured_channels & BIT(ch->vc_id))) {
				continue;
			}

			for (uint8_t j = 0U; j < group_count; j++) {
				if (adc_b_shared.group_cfg[j].scan_group_id ==
				    (adc_group_id_t)ch->scan_group_id) {
					already_added = true;
					break;
				}
			}
			if (already_added) {
				continue;
			}

			if (group_count >= ADC_B_RX_MAX_SCAN_GROUPS) {
				return -ENOMEM;
			}

			memset(&adc_b_shared.group_cfg[group_count], 0,
			       sizeof(adc_b_shared.group_cfg[group_count]));
			adc_b_shared.group_cfg[group_count].scan_group_id =
				(adc_group_id_t)ch->scan_group_id;

			adc_b_shared.group_cfg[group_count].converter_selection =
				(adc_b_unit_id_t)adc_b_sg_to_unit(ch->scan_group_id);
			adc_b_shared.group_cfg[group_count].scan_group_enable = true;
			adc_b_shared.group_cfg[group_count].scan_end_interrupt_enable = true;
			adc_b_shared.group_cfg[group_count].external_trigger_enable_mask =
				(adc_b_external_trigger_t)ADC_B_EXTERNAL_TRIGGER_NONE;

			adc_b_shared.groups[group_count] = &adc_b_shared.group_cfg[group_count];
			group_count++;
		}
	}

	if (group_count == 0U) {
		LOG_ERR("No channels configured");
		return -EINVAL;
	}

	/* for each discovered scan-group, populate its VC list */
	for (uint8_t g = 0U; g < group_count; g++) {
		int ret = adc_rx_adc_b_populate_group(g, table_ns, &table_count, &vc_global_idx);

		if (ret < 0) {
			return ret;
		}
	}

	memset(&adc_b_shared.scan_cfg, 0, sizeof(adc_b_shared.scan_cfg));
	adc_b_shared.scan_cfg.group_count = group_count;
	adc_b_shared.scan_cfg.p_adc_groups = (adc_b_group_cfg_t **)adc_b_shared.groups;

	return 0;
}

/* Fill converter_selection[] with the scan-group -> converter map. */
static void adc_rx_setup_converter_selection(adc_b_extended_cfg_t *extend)
{
	for (size_t i = 0U; i < ARRAY_SIZE(adc_b_sg_unit_map); i++) {
		uint8_t gid = adc_b_sg_unit_map[i].scan_group_id;

		if (gid < ARRAY_SIZE(extend->converter_selection)) {
			extend->converter_selection[gid] = adc_b_sg_unit_map[i].unit_id;
		}
	}
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

/* vc_id is a global virtual-channel-id shared by the whole ADC_B peripheral so two channels
 * on different units must not reuse the same vc_id.
 */
static void adc_rx_check_vc_id_overlap(const struct adc_rx_adc_b_config *config)
{
	for (uint8_t u = 0U; u < ARRAY_SIZE(adc_b_unit_dev); u++) {
		const struct device *other_dev = adc_b_unit_dev[u];
		const struct adc_rx_adc_b_config *other;

		if (other_dev == NULL || other_dev->config == config) {
			continue;
		}

		other = other_dev->config;

		for (uint8_t i = 0U; i < config->dt_channel_count; i++) {
			uint8_t vc_id = config->dt_channels[i].vc_id;

			for (uint8_t j = 0U; j < other->dt_channel_count; j++) {
				__ASSERT(other->dt_channels[j].vc_id != vc_id,
					 "vc_id %u reused across ADC_B units", vc_id);
			}
		}
	}
}

/* Validate a virtual channel ID: in range and present in the device tree. */
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

/* Run a hardware calibration and block until calibration-complete is
 * signalled from the FSP callback. Must be called from thread context (it
 * sleeps on the semaphore) with adc_b_shared_lock held.
 */
static int adc_rx_adc_b_calibrate(void)
{
	fsp_err_t fsp_err;

	k_sem_reset(&adc_b_shared.calibrate_sem);

	fsp_err = R_ADC_B_Calibrate(&adc_b_shared.adc_ctrl, NULL);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADC_B_Calibrate failed: %d", fsp_err);
		return -EIO;
	}

	if (k_sem_take(&adc_b_shared.calibrate_sem, ADC_B_CAL_TIMEOUT) != 0) {
		LOG_ERR("calibration timed out");
		return -ETIMEDOUT;
	}

	return 0;
}

/* Called from start_read() only when the combined channel set changed, so
 * repeated channel_setup() calls do not re-run ScanCfg or trigger a
 * calibration each time.
 */
static int adc_rx_adc_b_apply_scan_cfg(void)
{
	fsp_err_t fsp_err;
	int ret;

	ret = adc_rx_adc_b_scan_cfg();
	if (ret < 0) {
		LOG_ERR("failed to configure scan: %d", ret);
		return ret;
	}

	fsp_err = R_ADC_B_ScanCfg(&adc_b_shared.adc_ctrl, &adc_b_shared.scan_cfg);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADC_B_ScanCfg failed: %d", fsp_err);
		return -EIO;
	}

	ret = adc_rx_adc_b_calibrate();
	if (ret < 0) {
		return ret;
	}

	adc_b_shared.scan_cfg_dirty = false;

	return 0;
}

static int adc_rx_adc_b_channel_setup(const struct device *dev,
				      const struct adc_channel_cfg *channel_cfg)
{
	struct adc_rx_adc_b_data *data = dev->data;
	const struct adc_rx_adc_b_config *config = dev->config;
	int ret;

	ret = adc_rx_validate_vc_id(config, channel_cfg->channel_id);
	if (ret < 0) {
		LOG_ERR("invalid virtual channel id %u", channel_cfg->channel_id);
		return -EINVAL;
	}

	/* Validate the acquisition-time encoding (table programmed at scan cfg). */
	if (adc_rx_acq_time_to_ns(channel_cfg->acquisition_time, NULL) < 0) {
		LOG_ERR("unsupported acquisition-time (use ns/us, not ticks)");
		return -ENOTSUP;
	}

	if (channel_cfg->differential) {
		LOG_ERR("differential mode not supported");
		return -ENOTSUP;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("only ADC_GAIN_1 is supported");
		return -EINVAL;
	}

	/* Only record the channel here; the scan is (re)programmed and
	 * calibrated lazily on the next start_read(). configured_channels is
	 * read by the scan rebuild under adc_b_shared_lock, so it must be
	 * written under the same lock.
	 */
	k_mutex_lock(&adc_b_shared_lock, K_FOREVER);
	data->configured_channels |= BIT(channel_cfg->channel_id);
	adc_b_shared.scan_cfg_dirty = true;
	k_mutex_unlock(&adc_b_shared_lock);

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
	int err;

	/* Each virtual channel converts with the resolution/oversampling fixed
	 * by its DTS channel node, so the sequence must match every selected
	 * channel (SAR-path channels are already validated against >12-bit in
	 * adc_rx_adc_b_scan_cfg()).
	 */
	for (uint8_t i = 0U; i < config->dt_channel_count; i++) {
		const struct adc_rx_adc_b_channel_cfg *ch = &config->dt_channels[i];

		if (!(sequence->channels & BIT(ch->vc_id))) {
			continue;
		}

		if (sequence->resolution != ch->resolution) {
			LOG_ERR("vc %u: sequence resolution %u does not match "
				"channel resolution %u",
				ch->vc_id, sequence->resolution, ch->resolution);
			return -EINVAL;
		}

		if (sequence->oversampling != 0U && sequence->oversampling != ch->oversampling) {
			LOG_ERR("vc %u: sequence oversampling %u does not match "
				"channel oversampling %u",
				ch->vc_id, sequence->oversampling, ch->oversampling);
			return -ENOTSUP;
		}
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

	/* Own the shared hardware for this whole conversion, so another unit
	 * cannot start (or rebuild scan config for) a conversion of its own
	 * while this one is in flight.
	 */
	k_sem_take(&adc_b_conversion_owner, K_FOREVER);

	/* Program the combined scan (and calibrate) only when the channel set
	 * of ANY unit changed since the last read; channel_setup() just marks
	 * the shared config dirty.
	 */
	k_mutex_lock(&adc_b_shared_lock, K_FOREVER);

	if (adc_b_shared.scan_cfg_dirty) {
		err = adc_rx_adc_b_apply_scan_cfg();
	} else if (sequence->calibrate) {
		err = adc_rx_adc_b_calibrate();
	} else {
		err = 0;
	}

	k_mutex_unlock(&adc_b_shared_lock);

	if (err < 0) {
		k_sem_give(&adc_b_conversion_owner);
		return err;
	}

	data->buf_id = 0;
	data->buf = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);

	/* adc_b_conversion_owner is released from adc_context_on_complete(),
	 * the true end of this sequence - adc_context_wait_for_completion()
	 * itself returns immediately for async reads, before that happens.
	 */
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

/* Compute the full pending scan-group bitmask from the channels selected
 * in the current adc_sequence. Caller resets completed_group_mask.
 */
static void adc_rx_adc_b_compute_pending(struct adc_rx_adc_b_data *data)
{
	const struct adc_rx_adc_b_config *cfg = data->dev->config;
	uint32_t group_mask = 0U;

	for (uint8_t i = 0U; i < cfg->dt_channel_count; i++) {
		const struct adc_rx_adc_b_channel_cfg *ch = &cfg->dt_channels[i];

		if (!(data->channels & BIT(ch->vc_id))) {
			continue;
		}
		group_mask |= BIT(ch->scan_group_id);
	}

	data->pending_group_mask = group_mask;
	data->completed_group_mask = 0U;
}

/* Trigger the lowest pending scan-group. Groups on the same converter must run
 * one at a time, so the next is triggered from the scan-end ISR.
 */
static int adc_rx_adc_b_scan_next_group(struct adc_rx_adc_b_data *data)
{
	uint32_t to_start = data->pending_group_mask & ~data->completed_group_mask;
	uint32_t next_group;
	fsp_err_t fsp_err;

	if (to_start == 0U) {
		return -ENODATA;
	}

	next_group = to_start & (uint32_t)(-(int32_t)to_start);

	k_mutex_lock(&adc_b_shared_lock, K_FOREVER);
	fsp_err = R_ADC_B_ScanGroupStart(&adc_b_shared.adc_ctrl, (adc_group_mask_t)next_group);
	k_mutex_unlock(&adc_b_shared_lock);

	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_ADC_B_ScanGroupStart(0x%02x) failed: %d", next_group, fsp_err);
		adc_context_complete(&data->ctx, -EIO);
		return -EIO;
	}

	return 0;
}

static void adc_rx_adc_b_start_work_handler(struct k_work *work)
{
	struct adc_rx_adc_b_data *data = CONTAINER_OF(work, struct adc_rx_adc_b_data, start_work);

	(void)adc_rx_adc_b_scan_next_group(data);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_rx_adc_b_data *data = CONTAINER_OF(ctx, struct adc_rx_adc_b_data, ctx);

	data->channels = ctx->sequence.channels;
	adc_rx_adc_b_compute_pending(data);

	if (data->pending_group_mask == 0U) {
		LOG_ERR("No scan-groups derived from sequence channels");
		adc_context_complete(ctx, -EINVAL);
		return;
	}

	if (k_is_in_isr()) {
		k_work_submit(&data->start_work);
	} else {
		(void)adc_rx_adc_b_scan_next_group(data);
	}
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_rx_adc_b_data *data = CONTAINER_OF(ctx, struct adc_rx_adc_b_data, ctx);

	if (repeat_sampling) {
		data->buf_id = 0;
	}
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
	}

	/* Default to PCLKA for unknown clock sources */
	LOG_DBG("Unknown clock source device '%s', defaulting to PCLKA", name);
	return ADC_B_CLOCK_SOURCE_PCLKA;
}

/* Wire one group-interrupt factor (calend/convserr) to its FSP ISR and
 * enable it on the RX group interrupt controller.
 */
static int adc_rx_grp_irq_setup(const struct device *intc, int factor, void (*isr)(void),
				void *context)
{
	int err;

	err = rx_grp_intc_callback_set(intc, factor, isr, context);
	if (err != 0) {
		return err;
	}

	return rx_grp_intc_enable(intc, factor);
}

/* Calibration covers every unit, so every unit's calend vector is wired once
 * (not per instance), whether or not that unit is enabled.
 */
static void adc_rx_calend_irq_setup(const struct device *group_intc)
{
	for (size_t i = 0U; i < ARRAY_SIZE(calend_irqs); i++) {
		int err = adc_rx_grp_irq_setup(group_intc, calend_irqs[i].irq, calend_irqs[i].isr,
					       (void *)&adc_b_shared.adc_ctrl);

		if (err != 0) {
			LOG_ERR("Failed to setup calibration end IRQ %zu", i);
		}
	}
}

static void renesas_rx_adc_callback(adc_callback_args_t *p_args);

static int adc_rx_adc_b_init(const struct device *dev)
{
	const struct adc_rx_adc_b_config *config = dev->config;
	struct adc_rx_adc_b_data *data = dev->data;
	adc_b_extended_cfg_t *extend;
	adc_b_clock_source_t clock_source;
	uint8_t divr;
	fsp_err_t fsp_err;
	int ret;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	k_work_init(&data->start_work, adc_rx_adc_b_start_work_handler);

	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->clock_subsys);
	if (ret < 0) {
		return ret;
	}

	__ASSERT(config->unit_id < ARRAY_SIZE(adc_b_unit_dev), "unsupported ADC_B unit id");
	adc_rx_check_vc_id_overlap(config);
	adc_b_unit_dev[config->unit_id] = dev;

	k_mutex_lock(&adc_b_shared_lock, K_FOREVER);

	extend = &g_adc_cfg_extend;

	adc_rx_setup_converter_selection(extend);

	clock_source = adc_rx_get_clock_source(config->clock_dev);

	/* DT clock-div is the divider (1..8); the register
	 * field DIVR encodes (div - 1). All units share the same parent clock
	 * configuration (single ADCLKCR register), so recomputing this from
	 * any unit's config always yields the same value.
	 */
	divr = config->clock_div - 1U;
	extend->clock_control_u.clock_control_data =
		((clock_source << R_ADC_B0_ADCLKCR_CLKSEL_Pos) |
		 (divr << R_ADC_B0_ADCLKCR_DIVR_Pos));

	if (!adc_b_shared.opened) {
		k_sem_init(&adc_b_shared.calibrate_sem, 0, 1);

		/* adc_cfg.unit is not read back by the FSP; fixed at 0 rather
		 * than tied to whichever unit happens to init first.
		 */
		adc_b_shared.adc_cfg.unit = 0;
		adc_b_shared.adc_cfg.alignment = (adc_alignment_t)ADC_ALIGNMENT_RIGHT;
		adc_b_shared.adc_cfg.p_callback = renesas_rx_adc_callback;
		adc_b_shared.adc_cfg.p_context = NULL;
		adc_b_shared.adc_cfg.p_extend = extend;

		fsp_err = R_ADC_B_Open(&adc_b_shared.adc_ctrl, &adc_b_shared.adc_cfg);
		if (fsp_err != FSP_SUCCESS) {
			k_mutex_unlock(&adc_b_shared_lock);
			LOG_ERR("R_ADC_B_Open failed: %d", fsp_err);
			return -EIO;
		}

		adc_rx_calend_irq_setup(config->group_intc);

		adc_b_shared.opened = true;
	}

	k_mutex_unlock(&adc_b_shared_lock);

	config->irq_configure(dev);

	adc_context_unlock_unconditionally(&data->ctx);
	return 0;
}

/* Read the selected channels into the sequence buffer in ascending channel-id
 * order, once all pending scan-groups have completed.
 */
static int adc_rx_adc_b_drain_results(const struct device *dev)
{
	struct adc_rx_adc_b_data *data = dev->data;
	const struct adc_rx_adc_b_config *cfg = dev->config;
	uint16_t *sample_buffer = data->buf;
	fsp_err_t fsp_err;

	if (sample_buffer == NULL) {
		return 0;
	}

	for (uint8_t vc_id = 0U; vc_id < ADC_B_RX_MAX_VC; vc_id++) {
		const struct adc_rx_adc_b_channel_cfg *ch;
		uint32_t sample = 0;

		if (!(data->configured_channels & BIT(vc_id)) || !(data->channels & BIT(vc_id))) {
			continue;
		}

		ch = adc_rx_find_dt_channel(cfg, vc_id);
		if (ch == NULL) {
			continue;
		}

		bool overflow_flag = false;

		fsp_err = R_ADC_B_Read32(&adc_b_shared.adc_ctrl, (adc_channel_t)ch->input_positive,
					 &sample);
		if (fsp_err == FSP_ERR_INVALID_DATA) {
			/*
			 * To determine whether the error is caused by an overflow,
			 * check the corresponding channel status bit in ADOVFCHSR0/1.
			 *
			 * Per the datasheet (62.6.2), overflowed conversion results are
			 * saturated to either the upper or lower limit of the ADC data range.
			 */
			if (ch->input_positive < 32U) {
				uint32_t ovf_mask = BIT(ch->input_positive);

				if (R_ADC_B->ADOVFCHSR0 & ovf_mask) {
					overflow_flag = true;
					R_ADC_B->ADOVFCHSCR0 = ovf_mask;
				}
			} else {
				uint32_t ovf_mask = BIT(ch->input_positive - 32U);

				if (R_ADC_B->ADOVFCHSR1 & ovf_mask) {
					overflow_flag = true;
					R_ADC_B->ADOVFCHSCR1 = ovf_mask;
				}
			}
		}

		if (fsp_err != FSP_SUCCESS && !overflow_flag) {
			LOG_ERR("R_ADC_B_Read32 failed for vc %u (phys %u): %d", vc_id,
				ch->input_positive, fsp_err);
			return -EIO;
		}

		/* Explicitly clamp the sample here when overflow occurs */
		if (overflow_flag) {
			uint32_t max_val = BIT(ch->resolution) - 1U;

			sample = CLAMP(sample, 0U, max_val);
		}

		sample_buffer[data->buf_id++] = (uint16_t)sample;
	}

	return 0;
}

/* Route a completed scan-group to the device that owns it. */
static void adc_rx_adc_b_handle_scan_complete(uint32_t group_mask)
{
	for (uint8_t gid = 0U; gid < ADC_B_RX_MAX_SCAN_GROUPS; gid++) {
		uint8_t unit;
		const struct device *dev;
		struct adc_rx_adc_b_data *data;
		uint32_t my_group_done;

		if (!(group_mask & BIT(gid))) {
			continue;
		}

		unit = adc_b_sg_to_unit(gid);
		dev = (unit < ARRAY_SIZE(adc_b_unit_dev)) ? adc_b_unit_dev[unit] : NULL;
		if (dev == NULL) {
			continue;
		}

		data = dev->data;
		my_group_done = BIT(gid) & data->pending_group_mask;
		if (my_group_done == 0U) {
			continue;
		}

		data->completed_group_mask |= my_group_done;

		if ((data->completed_group_mask & data->pending_group_mask) !=
		    data->pending_group_mask) {
			/* Kick the next group from the workqueue (not this ISR). */
			k_work_submit(&data->start_work);
			continue;
		}

		if (adc_rx_adc_b_drain_results(dev) < 0) {
			adc_context_complete(&data->ctx, -EIO);
			continue;
		}

		adc_context_on_sampling_done(&data->ctx, dev);
	}
}

static void renesas_rx_adc_callback(adc_callback_args_t *p_args)
{
	switch (p_args->event) {
	case ADC_EVENT_SCAN_COMPLETE:
		adc_rx_adc_b_handle_scan_complete((uint32_t)p_args->group_mask);
		break;

	case ADC_EVENT_CALIBRATION_COMPLETE:
		k_sem_give(&adc_b_shared.calibrate_sem);
		break;

	case ADC_EVENT_CONVERSION_ERROR: {
		const struct device *dev = (p_args->unit < ARRAY_SIZE(adc_b_unit_dev))
						   ? adc_b_unit_dev[p_args->unit]
						   : NULL;

		LOG_ERR("conversion error event (unit %u)", p_args->unit);
		if (dev != NULL) {
			struct adc_rx_adc_b_data *data = dev->data;

			adc_context_complete(&data->ctx, -EIO);
		}
		break;
	}

	default:
		LOG_WRN("unhandled ADC event: %d", p_args->event);
		break;
	}
}

/* Per scan-group scan-end ISR wrapper. */
#define SCANEND_ISR_WRAPPER(node_id)                                                               \
	static void adc_rx_scanend_isr_##node_id(const struct device *dev)                         \
	{                                                                                          \
		uint32_t irq = DT_IRQ_BY_NAME(node_id, scanend, irq);                              \
		ARG_UNUSED(dev);                                                                   \
		BSP_ICU_IRQ_HANDLER(irq, ADC_B_SCAN_END_ISR(DT_REG_ADDR_RAW(node_id)));            \
	}

DT_FOREACH_STATUS_OKAY(renesas_rx_adc_b_scan_group, SCANEND_ISR_WRAPPER)

/* Route a scan-group's scan-end event to its ICU vector (SLIXR/SLIR) and
 * connect/enable its ISR wrapper.
 */
#define ADC_RX_SG_IRQ_WIRE_NODE(node_id)                                                           \
	if (128 <= DT_IRQ_BY_NAME(node_id, scanend, irq) &&                                        \
	    DT_IRQ_BY_NAME(node_id, scanend, irq) < 144) {                                         \
		R_ICU->SLIXR[DT_IRQ_BY_NAME(node_id, scanend, irq) - 128].SLIXR =                  \
			ADC_B_ICU_SCAN_END_EVENT(DT_REG_ADDR_RAW(node_id));                        \
	} else if (DT_IRQ_BY_NAME(node_id, scanend, irq) >= 144) {                                 \
		R_ICU->SLIR[DT_IRQ_BY_NAME(node_id, scanend, irq) - 144].SLIR =                    \
			ADC_B_ICU_SCAN_END_EVENT(DT_REG_ADDR_RAW(node_id));                        \
	}                                                                                          \
	IRQ_CONNECT(DT_IRQ_BY_NAME(node_id, scanend, irq),                                         \
		    DT_IRQ_BY_NAME(node_id, scanend, priority), adc_rx_scanend_isr_##node_id,      \
		    NULL, 0);                                                                      \
	irq_enable(DT_IRQ_BY_NAME(node_id, scanend, irq));

/* Scan-end wiring is keyed by scan-group (not by unit), so program it once. */
static int adc_rx_adc_b_global_init(void)
{
	DT_FOREACH_STATUS_OKAY(renesas_rx_adc_b_scan_group, ADC_RX_SG_IRQ_WIRE_NODE)

	return 0;
}
SYS_INIT(adc_rx_adc_b_global_init, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY);

/* Per-instance IRQ setup: wire this unit's convserr on the group interrupt
 * controller (calend is wired once for all units in init).
 */
#define IRQ_CONFIGURE_FUNC(idx)                                                                    \
	static void adc_rx_configure_func_##idx(const struct device *dev)                          \
	{                                                                                          \
		const struct adc_rx_adc_b_config *cfg = dev->config;                               \
		int err;                                                                           \
                                                                                                   \
		err = adc_rx_grp_irq_setup(cfg->group_intc, CONVSERR_UNIT_IRQ(DT_DRV_INST(idx)),   \
					   ADC_B_ERR_ISR(DT_INST_PROP(idx, unit_id)),              \
					   (void *)&adc_b_shared.adc_ctrl);                        \
		if (err != 0) {                                                                    \
			LOG_ERR("Failed to setup conversion error IRQ for unit %d",                \
				DT_INST_PROP(idx, unit_id));                                       \
		}                                                                                  \
	}

#define IRQ_CONFIGURE_DEFINE(idx) .irq_configure = adc_rx_configure_func_##idx

#define ADC_RX_ADC_B_DT_CH_INIT(node_id)                                                           \
	{                                                                                          \
		.vc_id = DT_REG_ADDR(node_id),                                                     \
		.input_positive = DT_PROP(node_id, zephyr_input_positive),                         \
		.resolution = DT_PROP_OR(node_id, zephyr_resolution, 12),                          \
		.scan_group_id = DT_REG_ADDR_RAW(DT_PHANDLE(node_id, renesas_scan_group)),         \
		.oversampling = DT_PROP_OR(node_id, zephyr_oversampling, 0),                       \
		.acq_time = DT_PROP_OR(node_id, zephyr_acquisition_time, ADC_ACQ_TIME_DEFAULT),    \
	},

/* Compile-time checks per channel node:
 *  - The converter assigned to this channel's scan-group must equal the
 *    unit-id of the parent adc-unit.
 *  - Physical channel must be a bit set in the parent adc-unit's
 *    channel-available-mask.
 */
#define ADC_RX_VALIDATE_CH(node_id)                                                                \
	BUILD_ASSERT(                                                                              \
		DT_PROP(DT_PHANDLE(node_id, renesas_scan_group), renesas_converter_unit_id) ==     \
			DT_PROP(DT_PARENT(node_id), unit_id),                                      \
		"channel '" DT_NODE_FULL_NAME(node_id) "': scan-group's "                          \
						       "renesas,converter-unit-id must equal the " \
						       "parent adc-unit's unit-id");               \
	BUILD_ASSERT(((1U << DT_PROP(node_id, zephyr_input_positive)) &                            \
		      DT_PROP(DT_PARENT(node_id), channel_available_mask)) != 0U,                  \
		     "channel '" DT_NODE_FULL_NAME(                                                \
			     node_id) "': zephyr,input-positive "                                  \
				      "is not in the parent adc-unit's channel-available-mask");

#define ADC_RX_SG_ISR_CFG_FIELD_NODE(node_id)                                                      \
	.ISR_CFG_SCANEND_IRQ(DT_REG_ADDR_RAW(node_id)) =                                           \
		(IRQn_Type)DT_IRQ_BY_NAME(node_id, scanend, irq),                                  \
	.ISR_CFG_SCANEND_IPL(DT_REG_ADDR_RAW(node_id)) =                                           \
		DT_IRQ_BY_NAME(node_id, scanend, priority),

/* Each present unit fills its own slot in the shared config below. */
#define ADC_B_CONVERTER_MODE_ENTRY(idx)                                                            \
	[DT_INST_PROP(idx, unit_id)] = {                                                           \
		.mode = (ADC_B_CONVERTER_MODE_SINGLE_SCAN),                                        \
		.method = ADC_B_DT_METHOD(idx),                                                    \
	},

#define ADC_B_FILTER_SELECTION_ENTRY(idx) [DT_INST_PROP(idx, unit_id)] = ADC_B_DFSEL_SINC3_ALL,

#define ADC_B_CONVSERR_ISR_ENTRY(idx)                                                              \
	.ISR_CFG_CONVSERR_IRQ(                                                                     \
		DT_INST_PROP(idx, unit_id)) = VECTOR_NUMBER_ADC_CONVSERR(DT_DRV_INST(idx)),        \
				  .ISR_CFG_CONVSERR_IPL(DT_INST_PROP(idx, unit_id)) =              \
					  DT_INST_IRQ_BY_NAME(idx, convserr, priority),

static adc_b_isr_cfg_t g_adc_isr_cfg = {
	DT_FOREACH_STATUS_OKAY(renesas_rx_adc_b_scan_group, ADC_RX_SG_ISR_CFG_FIELD_NODE)
		ADC_RX_UNIT_CALEND_ISR_CFG_FIELD(ADC0_NODE)
			ADC_RX_UNIT_CALEND_ISR_CFG_FIELD(ADC1_NODE)
				ADC_RX_UNIT_CALEND_ISR_CFG_FIELD(ADC2_NODE)
					DT_INST_FOREACH_STATUS_OKAY(ADC_B_CONVSERR_ISR_ENTRY)};

static adc_b_extended_cfg_t g_adc_cfg_extend = {
	.sync_operation_control_u = {.sync_operation_control = 0},
	.adc_b_converter_mode = {DT_INST_FOREACH_STATUS_OKAY(ADC_B_CONVERTER_MODE_ENTRY)},
	.fifo_enable_mask_u = {.fifo_enable_mask = 0},
	.fifo_interrupt_enable_mask = 0,
	.start_trigger_delay_table = {0},
	.calibration_adc_state = ((256 << R_ADC_B0_ADCALSTCR_CALADSST_Pos) |
				  (20 << R_ADC_B0_ADCALSTCR_CALADCST_Pos)),
	.calibration_sample_and_hold =
		((95 << R_ADC_B0_ADCALSHCR_CALSHSST_Pos) | (5 << R_ADC_B0_ADCALSHCR_CALSHHST_Pos)),
	.p_isr_cfg = &g_adc_isr_cfg,
	.sampling_state_table = {0},
	.sample_and_hold_enable_mask = (ADC_B_SAMPLE_AND_HOLD_MASK_NONE),
	.conversion_state =
		((20 << R_ADC_B0_ADCNVSTR_CST0_Pos) | (20 << R_ADC_B0_ADCNVSTR_CST1_Pos) |
		 (20 << R_ADC_B0_ADCNVSTR_CST2_Pos)),
	.user_offset_tables = {0},
	.user_gain_tables = {0},
	.limiter_clip_interrupt_enable_mask = (0x00),
	.limiter_clip_tables = {0},
	.adc_filter_selection = {DT_INST_FOREACH_STATUS_OKAY(ADC_B_FILTER_SELECTION_ENTRY)},
};

#define ADC_RX_ADC_B_INIT(idx)                                                                     \
	IRQ_CONFIGURE_FUNC(idx)                                                                    \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADC_RX_VALIDATE_CH)                                 \
	static const struct adc_rx_adc_b_channel_cfg adc_rx_adc_b_channels_##idx[] = {             \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, ADC_RX_ADC_B_DT_CH_INIT)};                  \
	static DEVICE_API(adc, adc_rx_adc_b_api_##idx) = {                                         \
		.channel_setup = adc_rx_adc_b_channel_setup,                                       \
		.read = adc_rx_adc_b_read,                                                         \
		.ref_internal = DT_INST_PROP_OR(idx, vref_mv, 3300),                               \
		IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = adc_rx_adc_b_read_async))};               \
	static const struct adc_rx_adc_b_config adc_rx_adc_b_config_##idx = {                      \
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
		.unit_id = DT_INST_PROP(idx, unit_id),                                             \
		.channel_available_mask = DT_INST_PROP_OR(idx, channel_available_mask, 0),         \
		.dt_channels = adc_rx_adc_b_channels_##idx,                                        \
		.dt_channel_count = ARRAY_SIZE(adc_rx_adc_b_channels_##idx),                       \
		.group_intc = DEVICE_DT_GET(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(idx), calend)),        \
		IRQ_CONFIGURE_DEFINE(idx),                                                         \
	};                                                                                         \
                                                                                                   \
	static struct adc_rx_adc_b_data adc_rx_adc_b_data_##idx = {                                \
		ADC_CONTEXT_INIT_TIMER(adc_rx_adc_b_data_##idx, ctx),                              \
		ADC_CONTEXT_INIT_LOCK(adc_rx_adc_b_data_##idx, ctx),                               \
		ADC_CONTEXT_INIT_SYNC(adc_rx_adc_b_data_##idx, ctx),                               \
		.dev = DEVICE_DT_INST_GET(idx),                                                    \
		.buf = NULL,                                                                       \
		.buf_id = 0,                                                                       \
		.channels = 0,                                                                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, adc_rx_adc_b_init, NULL, &adc_rx_adc_b_data_##idx,              \
			      &adc_rx_adc_b_config_##idx, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,   \
			      &adc_rx_adc_b_api_##idx)

DT_INST_FOREACH_STATUS_OKAY(ADC_RX_ADC_B_INIT);
