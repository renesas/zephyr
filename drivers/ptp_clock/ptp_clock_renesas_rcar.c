/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_ptp_clock_channel

#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

/* Required by DEVICE_MMIO_NAMED_* macros */
#define DEV_CFG(_dev)  ((const struct ptp_clock_renesas_rcar_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct ptp_clock_renesas_rcar_data *)(_dev)->data)

/* Util */
/* Number of nanoseconds in one second. */
#define NSEC_PER_SEC_ULL (1000000000ULL)

/*
 * One second represented in Q27 fixed-point format.
 * The timer period in units of nanoseconds-per-clock scaled by 2^27
 */
#define NSEC_PER_SEC_Q27 (NSEC_PER_SEC_ULL << 27)

#define ROUNDING_OFFSET 0.5

/* Registers offset */
/* Version register */
#define PTPIPV           0x0
/* Configuration registers */
#define PTPTMEC          0x10
#define PTPTMDC          0x14
#define PTPTIVC(chan)    (0x20 + 0x40 * (chan))
#define PTPTOVC0(chan)   (0x30 + 0x40 * (chan))
#define PTPTOVC1(chan)   (0x34 + 0x40 * (chan))
#define PTPTOVC2(chan)   (0x38 + 0x40 * (chan))
/* Monitoring registers */
#define PTPGPTPTM0(chan) (0x50 + 0x40 * (chan))
#define PTPGPTPTM1(chan) (0x54 + 0x40 * (chan))
#define PTPGPTPTM2(chan) (0x58 + 0x40 * (chan))

/* Register configuration */
#define PTPTMEC_TIMER_ENABLE(chan) (BIT(chan))

#define PTP_SECONDS_HIGH_MAX GENMASK(15, 0)
#define PTP_SECONDS_MAX      GENMASK64(47, 0)

struct ptp_clock_renesas_rcar_config {
	DEVICE_MMIO_NAMED_ROM(reg_base);
	int channel;
	uint32_t nominal_freq;
};

struct ptp_clock_renesas_rcar_data {
	DEVICE_MMIO_NAMED_RAM(reg_base);
	struct net_ptp_time current_offset;
};

static inline uint32_t ptp_clock_renesas_rcar_read(const struct device *dev, uint32_t offs)
{
	return sys_read32(DEVICE_MMIO_NAMED_GET(dev, reg_base) + offs);
}

static inline void ptp_clock_renesas_rcar_write(const struct device *dev, uint32_t offs,
						uint32_t value)
{
	sys_write32(value, DEVICE_MMIO_NAMED_GET(dev, reg_base) + offs);
}

static int ptp_clock_renesas_rcar_set(const struct device *dev, struct net_ptp_time *tm);
static int ptp_clock_renesas_rcar_get(const struct device *dev, struct net_ptp_time *tm);
static int ptp_clock_renesas_rcar_adjust(const struct device *dev, int increment);
static int ptp_clock_renesas_rcar_rate_adjust(const struct device *dev, double rate);

static int ptp_clock_renesas_rcar_set(const struct device *dev, struct net_ptp_time *tm)
{
	const struct ptp_clock_renesas_rcar_config *config = dev->config;
	struct ptp_clock_renesas_rcar_data *data = dev->data;
	struct net_ptp_time curr;
	int64_t nsec_offset;
	int64_t sec_offset;

	if (tm->second > PTP_SECONDS_MAX) {
		return -EINVAL;
	}

	if (tm->nanosecond >= NSEC_PER_SEC) {
		return -EINVAL;
	}

	/* Get current time */
	ptp_clock_renesas_rcar_get(dev, &curr);

	/* Re-calculate offset to achieve the requested time */
	sec_offset =
		(int64_t)tm->second - (int64_t)curr.second + (int64_t)data->current_offset.second;

	nsec_offset = (int64_t)tm->nanosecond - (int64_t)curr.nanosecond +
		      (int64_t)data->current_offset.nanosecond;

	/* Normalize time */
	if (nsec_offset < 0) {
		nsec_offset += NSEC_PER_SEC;
		sec_offset--;
	} else if (nsec_offset >= NSEC_PER_SEC) {
		nsec_offset -= NSEC_PER_SEC;
		sec_offset++;
	}

	/* Validate the second offset */
	if (sec_offset < 0 || sec_offset > PTP_SECONDS_MAX) {
		return -EINVAL;
	}

	data->current_offset.second = (uint64_t)sec_offset;
	data->current_offset.nanosecond = (uint32_t)nsec_offset;

	/* Update new value to hardware */
	ptp_clock_renesas_rcar_write(dev, PTPTOVC2(config->channel),
				     data->current_offset._sec.high);
	ptp_clock_renesas_rcar_write(dev, PTPTOVC1(config->channel), data->current_offset._sec.low);
	ptp_clock_renesas_rcar_write(dev, PTPTOVC0(config->channel),
				     data->current_offset.nanosecond);

	return 0;
}

static int ptp_clock_renesas_rcar_get(const struct device *dev, struct net_ptp_time *tm)
{
	const struct ptp_clock_renesas_rcar_config *config = dev->config;

	tm->nanosecond = ptp_clock_renesas_rcar_read(dev, PTPGPTPTM0(config->channel));
	tm->_sec.low = ptp_clock_renesas_rcar_read(dev, PTPGPTPTM1(config->channel));
	tm->_sec.high = (ptp_clock_renesas_rcar_read(dev, PTPGPTPTM2(config->channel)) &
			 PTP_SECONDS_HIGH_MAX);

	/*
	 * Set unused 0 to avoid containing garbarge value
	 * which could lead to wrong tm->second value
	 */
	tm->_sec.unused = 0;

	return 0;
}

static int ptp_clock_renesas_rcar_adjust(const struct device *dev, int increment)
{
	const struct ptp_clock_renesas_rcar_config *config = dev->config;
	struct ptp_clock_renesas_rcar_data *data = dev->data;
	int64_t temp_ns;
	int64_t sec_adjust;

	/* Calculate nanosecond value after increment */
	temp_ns = (int64_t)data->current_offset.nanosecond + increment;

	/* If delta nanosecond >= 1 second, update second and nanosecond */
	sec_adjust = temp_ns / NSEC_PER_SEC;
	temp_ns %= NSEC_PER_SEC;

	/* Normalize the nanosecond */
	if (temp_ns < 0) {
		temp_ns += NSEC_PER_SEC;
		sec_adjust--;
	}

	if (sec_adjust > 0) {
		if (data->current_offset.second > PTP_SECONDS_MAX - (uint64_t)sec_adjust) {
			/* Second offset will be out of range after update */
			return -EINVAL;
		}
		data->current_offset.second += (uint64_t)sec_adjust;
	} else if (sec_adjust < 0) {
		if (data->current_offset.second < (uint64_t)(-sec_adjust)) {
			/* Second offset will be negative */
			return -EINVAL;
		}
		data->current_offset.second -= (uint64_t)(-sec_adjust);
	}

	data->current_offset.nanosecond = (uint32_t)temp_ns;

	/* Configure hardware offset with new value */
	ptp_clock_renesas_rcar_write(dev, PTPTOVC2(config->channel),
				     data->current_offset._sec.high);
	ptp_clock_renesas_rcar_write(dev, PTPTOVC1(config->channel), data->current_offset._sec.low);
	ptp_clock_renesas_rcar_write(dev, PTPTOVC0(config->channel),
				     data->current_offset.nanosecond);

	return 0;
}

static int ptp_clock_renesas_rcar_rate_adjust(const struct device *dev, double rate)
{
	const struct ptp_clock_renesas_rcar_config *config = dev->config;
	uint32_t period_q27;
	double period_val;

	if (rate <= 0.0) {
		return -EINVAL;
	}

	period_val = (double)NSEC_PER_SEC_Q27 * rate / (double)config->nominal_freq;

	if (period_val > UINT32_MAX) {
		return -EINVAL;
	}

	/* Round to nearest integer by adding 0.5 before truncation. */
	period_q27 = (uint32_t)(period_val + ROUNDING_OFFSET);

	ptp_clock_renesas_rcar_write(dev, PTPTIVC(config->channel), period_q27);

	return 0;
}

static DEVICE_API(ptp_clock, ptp_clock_renesas_rcar_api) = {
	.set = ptp_clock_renesas_rcar_set,
	.get = ptp_clock_renesas_rcar_get,
	.adjust = ptp_clock_renesas_rcar_adjust,
	.rate_adjust = ptp_clock_renesas_rcar_rate_adjust,
};

static int ptp_clock_renesas_rcar_init(const struct device *dev)
{
	const struct ptp_clock_renesas_rcar_config *config = dev->config;
	uint32_t val;
	uint32_t period_q27;

	DEVICE_MMIO_NAMED_MAP(dev, reg_base, K_MEM_CACHE_NONE);

	/* Set the nominal clock */
	period_q27 = NSEC_PER_SEC_Q27 / config->nominal_freq;
	ptp_clock_renesas_rcar_write(dev, PTPTIVC(config->channel), period_q27);

	/* Enable timer */
	val = ptp_clock_renesas_rcar_read(dev, PTPTMEC);
	val |= PTPTMEC_TIMER_ENABLE(config->channel);
	ptp_clock_renesas_rcar_write(dev, PTPTMEC, val);

	return 0;
}

#define PTP_CLOCK_RENESAS_RCAR_INIT(n)                                                             \
	static const struct ptp_clock_renesas_rcar_config ptp_clock_renesas_rcar_config_##n = {    \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_INST_PARENT(n)),                           \
		.channel = DT_INST_REG_ADDR(n),                                                    \
		.nominal_freq = DT_PROP(DT_INST_PARENT(n), clock_frequency),                       \
	};                                                                                         \
                                                                                                   \
	static struct ptp_clock_renesas_rcar_data ptp_clock_renesas_rcar_data_##n = {              \
		.current_offset =                                                                  \
			{                                                                          \
				.second = 0,                                                       \
				.nanosecond = 0,                                                   \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ptp_clock_renesas_rcar_init, NULL,                                \
			      &ptp_clock_renesas_rcar_data_##n,                                    \
			      &ptp_clock_renesas_rcar_config_##n, POST_KERNEL,                     \
			      CONFIG_PTP_CLOCK_INIT_PRIORITY, &ptp_clock_renesas_rcar_api)

DT_INST_FOREACH_STATUS_OKAY(PTP_CLOCK_RENESAS_RCAR_INIT)
