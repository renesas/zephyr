/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/sys_clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/drivers/clock_control/renesas_rx_cgc.h>
#include <zephyr/spinlock.h>
#include <soc.h>
#include <stdint.h>

#define DT_DRV_COMPAT renesas_rx_gpt_timer

/* Ensure there are exactly two GPT timers instances
 * enabled in the device tree.
 */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 2,
	     "Requires two instances of the GPT timer to be enabled.");

/* Limit the counter to INT32_MAX to ensure that
 * timeout calculations will never overflow in sys_clock_set_timeout,
 * -1 to avoid overflow for CYCLES_CYCLE_TIMER */
#define COUNTER_MAX        (INT32_MAX - 1)
#define CYCLES_PER_SEC     sys_clock_hw_cycles_per_sec()
#define TICKS_PER_SEC      (CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define CYCLES_PER_TICK    (CYCLES_PER_SEC / TICKS_PER_SEC)
#define MAX_TICKS          ((k_ticks_t)(COUNTER_MAX / CYCLES_PER_TICK) - 1)
#define CYCLES_CYCLE_TIMER (COUNTER_MAX + 1)

#ifdef CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
typedef uint64_t cycle_t;
#define CYCLE_COUNT_MAX (0xffffffffffffffff)
#else
typedef uint32_t cycle_t;
#define CYCLE_COUNT_MAX (0xffffffff)
#endif

/* Get the hw cycles per sec when TIMER_READS_ITS_FREQUENCY_AT_RUNTIME is enabled */
extern unsigned int z_clock_hw_cycles_per_sec;

static cycle_t cycle_count;
static uint32_t clock_cycles_per_tick;
static volatile cycle_t announced_cycle_count;
static struct k_spinlock lock;

#define EVENT_GPT_COUNTER_OVERFLOW(channel)                                                        \
	BSP_PRV_IELS_ENUM(CONCAT(EVENT_GPT, channel, _COUNTER_OVERFLOW))

#define RX_GPT_NODE0 DT_INST_PARENT(0)
#define RX_GPT_NODE1 DT_INST_PARENT(1)

#define CYCL_TIMER_REG       ((R_GPT0_Type *)DT_REG_ADDR(RX_GPT_NODE0))
#define CYCL_TIMER_OVF_IRQN  DT_IRQ_BY_NAME(RX_GPT_NODE0, overflow, irq)
#define CYCL_TIMER_DIVIDER   DT_ENUM_IDX(RX_GPT_NODE0, divider)
#define CYCL_TIMER_OVF_EVENT EVENT_GPT_COUNTER_OVERFLOW(DT_PROP(RX_GPT_NODE0, channel))

#define TICK_TIMER_REG       ((R_GPT0_Type *)DT_REG_ADDR(RX_GPT_NODE1))
#define TICK_TIMER_OVF_IRQN  DT_IRQ_BY_NAME(RX_GPT_NODE1, overflow, irq)
#define TICK_TIMER_OVF_PRIO  DT_IRQ_BY_NAME(RX_GPT_NODE1, overflow, priority)
#define TICK_TIMER_DIVIDER   DT_ENUM_IDX(RX_GPT_NODE1, divider)
#define TICK_TIMER_OVF_EVENT EVENT_GPT_COUNTER_OVERFLOW(DT_PROP(RX_GPT_NODE1, channel))

/* Ensure the two GPT timers have the same divider value */
BUILD_ASSERT(CYCL_TIMER_DIVIDER == TICK_TIMER_DIVIDER,
	     "Requires the two GPT timers have the same divider value");

static const struct clock_control_rx_subsys_cfg clk_cfg = {
	.mstp = DT_CLOCKS_CELL_BY_IDX(DT_INST_PARENT(0), 0, mstp),
	.stop_bit = DT_CLOCKS_CELL_BY_IDX(DT_INST_PARENT(0), 0, stop_bit),
};

static cycle_t cycle_get(void)
{
	uint32_t val1 = CYCL_TIMER_REG->GTCNT;
	uint8_t ir = R_ICU->IR[CYCL_TIMER_OVF_IRQN].IR;
	uint32_t val2 = CYCL_TIMER_REG->GTCNT;

	if ((1 == ir) || (val1 > val2)) {
		cycle_count += CYCLES_CYCLE_TIMER;

		R_ICU->IR[CYCL_TIMER_OVF_IRQN].IR = 0;
	}

	return (val2 + cycle_count);
}

uint32_t sys_clock_cycle_get_32(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	uint32_t ret = (uint32_t)cycle_get();

	k_spin_unlock(&lock, key);

	return ret;
}

#ifdef CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
uint64_t sys_clock_cycle_get_64(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	uint64_t ret = (uint64_t)cycle_get();

	k_spin_unlock(&lock, key);

	return ret;
}
#endif /* CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER */

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		/* Always return 0 for tickful operation */
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);

	uint32_t ret;
	cycle_t current_cycle_count;

	current_cycle_count = cycle_get();

	if (current_cycle_count < announced_cycle_count) {
		/* cycle_count overflowed */
		ret = (uint32_t)((current_cycle_count +
				  (CYCLE_COUNT_MAX - announced_cycle_count + 1)) /
				 CYCLES_PER_TICK);
	} else {
		ret = (uint32_t)((current_cycle_count - announced_cycle_count) / CYCLES_PER_TICK);
	}

	k_spin_unlock(&lock, key);

	return ret;
}

static void tick_isr(void)
{
	k_ticks_t dticks;
	cycle_t current_cycle_count;

	R_BSP_IrqStatusClear(TICK_TIMER_OVF_IRQN);

	k_spinlock_key_t key = k_spin_lock(&lock);

	current_cycle_count = cycle_get();

	if (current_cycle_count < announced_cycle_count) {
		/* cycle_count overflowed */
		dticks = (k_ticks_t)((current_cycle_count +
				      (CYCLE_COUNT_MAX - announced_cycle_count + 1)) /
				     CYCLES_PER_TICK);
	} else {
		dticks = (k_ticks_t)((current_cycle_count - announced_cycle_count) /
				     CYCLES_PER_TICK);
	}

	announced_cycle_count = (current_cycle_count / CYCLES_PER_TICK) * CYCLES_PER_TICK;

	k_spin_unlock(&lock, key);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		sys_clock_announce(1);
	} else {
		sys_clock_announce(dticks);
	}
}

static void icu_connect_event(int irq, int event)
{
	if (128 <= irq && irq < 144) {
		R_ICU->SLIXR[irq - 128].SLIXR = event;
	} else if (144 <= irq) {
		R_ICU->SLIR[irq - 144].SLIR = event;
	}
}

static int sys_clock_driver_init(void)
{
	const struct device *clk = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(0)));
	uint32_t source_freq_hz;
	int ret;

	if (!device_is_ready(clk)) {
		return -ENODEV;
	}

	ret = clock_control_on(clk, (clock_control_subsys_t)&clk_cfg);
	if (ret < 0) {
		return ret;
	}

#if BSP_PERIPHERAL_GPT_GTCLK_PRESENT && !GPT_CFG_GPTCLK_BYPASS
	/* Calculate the GPTCK Divider. */
	const uint8_t divisor =
		(uint8_t)R_FSP_ClockDividerGet((uint32_t)R_SYSTEM->GPTCKDIVCR_b.GPTCKDIV);

	/* Calculate the GPTCK Frequency. */
	source_freq_hz =
		R_BSP_SourceClockHzGet((fsp_priv_source_clock_t)R_SYSTEM->GPTCKCR_b.GPTCKSEL) /
		divisor;
#else
	/* Look up PCLKD frequency. */
	source_freq_hz = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKD);
#endif

	z_clock_hw_cycles_per_sec = source_freq_hz >> CYCL_TIMER_DIVIDER;

	/* Sawtooth-wave PWM mode 1 */
	CYCL_TIMER_REG->GTCR_b.MD = 0;
	TICK_TIMER_REG->GTCR_b.MD = 0;

	/* Up-counting */
	CYCL_TIMER_REG->GTUDDTYC |= 3U;
	CYCL_TIMER_REG->GTUDDTYC |= 1U;

	TICK_TIMER_REG->GTUDDTYC |= 3U;
	TICK_TIMER_REG->GTUDDTYC |= 1U;

	/* count clock */
	CYCL_TIMER_REG->GTCR_b.TPCS = CYCL_TIMER_DIVIDER;
	TICK_TIMER_REG->GTCR_b.TPCS = TICK_TIMER_DIVIDER;

	/* Set counter to 0 */
	CYCL_TIMER_REG->GTCNT = 0;
	TICK_TIMER_REG->GTCNT = 0;

	/* Period */
	CYCL_TIMER_REG->GTPR = (uint32_t)COUNTER_MAX;

	clock_cycles_per_tick = (uint32_t)(CYCLES_PER_TICK);
	TICK_TIMER_REG->GTPR = (uint32_t)clock_cycles_per_tick - 1;

	/* Enable overflow interrupt for tick timer */
	TICK_TIMER_REG->GTINTAD_b.GTINTPR = 1;
	icu_connect_event(TICK_TIMER_OVF_IRQN, TICK_TIMER_OVF_EVENT);
	IRQ_CONNECT(TICK_TIMER_OVF_IRQN, TICK_TIMER_OVF_PRIO, tick_isr, NULL, 0);
	irq_enable(TICK_TIMER_OVF_IRQN);

	/* Enable overflow flag for cycle timer, no need interrupt handler for it */
	CYCL_TIMER_REG->GTINTAD_b.GTINTPR = 1;
	icu_connect_event(CYCL_TIMER_OVF_IRQN, CYCL_TIMER_OVF_EVENT);

	/* Start timer */
	CYCL_TIMER_REG->GTCR_b.CST = 1;
	TICK_TIMER_REG->GTCR_b.CST = 1;

	return 0;
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	if (ticks == K_TICKS_FOREVER || ticks == INT32_MAX) {
		return;
	}

	ticks = CLAMP(ticks - 1, 0, (int32_t)MAX_TICKS);

	k_spinlock_key_t key = k_spin_lock(&lock);

	/* Stop and clear CNT of tick timer */
	TICK_TIMER_REG->GTCR_b.CST = 0;
	TICK_TIMER_REG->GTCNT = 0;

	cycle_t now = cycle_get();
	cycle_t elapsed;

	if (now < announced_cycle_count) {
		/* cycle_count overflowed */
		elapsed = (cycle_t)(now + (CYCLE_COUNT_MAX - announced_cycle_count + 1));
	} else {
		elapsed = (now - announced_cycle_count);
	}

	cycle_t delay = (cycle_t)ticks * CYCLES_PER_TICK;

	delay += elapsed;
	delay = DIV_ROUND_UP(delay, CYCLES_PER_TICK) * CYCLES_PER_TICK;
	delay -= elapsed;

	/* Restart the tick timer */
	TICK_TIMER_REG->GTPR = (uint32_t)(delay - 1);
	TICK_TIMER_REG->GTCR_b.CST = 1;

	k_spin_unlock(&lock, key);
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
