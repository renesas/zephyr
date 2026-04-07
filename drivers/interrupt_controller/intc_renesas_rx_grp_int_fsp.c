/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_grp_intc_fsp

#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/drivers/interrupt_controller/intc_renesas_rx_grp_int_fsp.h>
#include <errno.h>

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 0

/* Get the vector number from DTS.*/
#define VECT_GROUP_IL0 DT_IRQN(DT_NODELABEL(group_irq_il0))
#define VECT_GROUP_BL0 DT_IRQN(DT_NODELABEL(group_irq_bl0))
#define VECT_GROUP_BL1 DT_IRQN(DT_NODELABEL(group_irq_bl1))
#define VECT_GROUP_AL0 DT_IRQN(DT_NODELABEL(group_irq_al0))
#define VECT_GROUP_AL1 DT_IRQN(DT_NODELABEL(group_irq_al1))
#define VECT_GROUP_AL2 DT_IRQN(DT_NODELABEL(group_irq_al2))
#define VECT_GROUP_AL3 DT_IRQN(DT_NODELABEL(group_irq_al3))
#define VECT_GROUP_AL4 DT_IRQN(DT_NODELABEL(group_irq_al4))
#define VECT_GROUP_AL5 DT_IRQN(DT_NODELABEL(group_irq_al5))
#define VECT_GROUP_AL6 DT_IRQN(DT_NODELABEL(group_irq_al6))

#define MIN_FACTOR_NUMBER 0
#define MAX_FACTOR_NUMBER 31

#define DEV_CFG(cfg, dev) struct rx_grp_int_cfg *cfg = (struct rx_grp_int_cfg *)(dev->config)

struct rx_grp_int_cfg {
	/*  Group Interrupt Request Enable Register (GENxxx)  */
	volatile uint32_t *gen;
	/* vector number */
	const uint8_t vector;
	/* priority */
	const uint8_t priority;
	struct k_spinlock lock;
};

bsp_grp_irq_cb_t g_bsp_group_irq_callback_list[BSP_FEATURE_GROUP_IRQ_NUM_INTERRUPTS]
					      [BSP_FEATURE_GROUP_IRQ_NUM_FACTORS];
extern void group_irq_isr(void);
extern uint32_t bsp_group_irq_get_table_index(icu_event_t irq);

/* Handlers for group interrupts */
static void handler_group(const struct device *dev)
{
	DEV_CFG(cfg, dev);
	uint32_t prev = g_current_isr_number;

	switch (cfg->vector) {
	case VECT_GROUP_IL0:
	case VECT_GROUP_BL0:
	case VECT_GROUP_BL1:
	case VECT_GROUP_AL0:
	case VECT_GROUP_AL1:
	case VECT_GROUP_AL2:
	case VECT_GROUP_AL3:
	case VECT_GROUP_AL4:
	case VECT_GROUP_AL5:
	case VECT_GROUP_AL6:
		g_current_isr_number = cfg->vector;
		group_irq_isr();
		g_current_isr_number = prev;
		break;
	default:
		break;
	}
}

/* GROUP INTERRUPT APIs */
/* Enable GEN register and  IER register*/
int rx_grp_intc_enable(const struct device *dev, uint16_t factor)
{
	DEV_CFG(cfg, dev);
	uint32_t irq;
	k_spinlock_key_t key;

	if (((factor < MIN_FACTOR_NUMBER) || (factor > MAX_FACTOR_NUMBER))) {
		return -EINVAL;
	}

	/* Set upper 8 bits as factor and lower 8 bits as vector number in irq */
	irq = (factor << 8) | cfg->vector;

	key = k_spin_lock(&cfg->lock);

	R_BSP_IrqEnable(irq);

	k_spin_unlock(&cfg->lock, key);

	return 0;
}

/* Disable GEN register, and disable IER register if no other factors are set.*/
int rx_grp_intc_disable(const struct device *dev, uint16_t factor)
{
	DEV_CFG(cfg, dev);
	uint32_t irq;
	k_spinlock_key_t key;

	if (((factor < MIN_FACTOR_NUMBER) || (factor > MAX_FACTOR_NUMBER))) {
		return -EINVAL;
	}

	/* Set upper 8 bits as factor and lower 8 bits as vector number in irq */
	irq = (factor << 8) | cfg->vector;

	key = k_spin_lock(&cfg->lock);

	R_BSP_IrqDisable(irq);

	k_spin_unlock(&cfg->lock, key);

	return 0;
}

/* Set callback function.*/
int rx_grp_intc_callback_set(const struct device *dev, uint16_t factor, void (*callback)(void),
			     void *context)
{
	DEV_CFG(cfg, dev);
	uint32_t irq;

	if ((factor < MIN_FACTOR_NUMBER) || (factor > MAX_FACTOR_NUMBER) || (NULL == callback)) {
		return -EINVAL;
	}

	if ((cfg->vector < 0) || (cfg->vector > CONFIG_NUM_IRQS - 1)) {
		return -EINVAL;
	}

	if ((cfg->priority < 0) || (cfg->priority > CONFIG_NUM_IRQ_PRIO_LEVELS - 1)) {
		return -EINVAL;
	}

	uint32_t index = bsp_group_irq_get_table_index((icu_event_t)cfg->vector);

	g_bsp_group_irq_callback_list[index][factor] = callback;

	irq = (factor << 8) | cfg->vector;
	R_BSP_IrqCfg(irq, cfg->priority, context);

	return 0;
}

int rx_grp_intc_set_gen(const struct device *dev, uint8_t vector_num, bool set)
{
	DEV_CFG(cfg, dev);

	if (vector_num > 31) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&cfg->lock);

	if (set) {
		*cfg->gen |= (1U << vector_num);
	} else {
		*cfg->gen &= ~(1U << vector_num);
	}

	k_spin_unlock(&cfg->lock, key);

	return 0;
}

#define GRP_INT_RX_INIT(index)                                                                     \
	static struct rx_grp_int_cfg rx_grp_int_##index##_cfg = {                                  \
		.vector = DT_INST_IRQN(index),                                                     \
		.priority = DT_INST_IRQ(index, priority),                                          \
		.gen = (volatile uint32_t *)DT_INST_REG_ADDR_BY_NAME(index, GEN),                  \
	};                                                                                         \
	static int rx_grp_int_##index##_init(const struct device *dev)                             \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), 0, handler_group, DEVICE_DT_INST_GET(index), 0);  \
		return 0;                                                                          \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(index, rx_grp_int_##index##_init, NULL, NULL,                        \
			      &rx_grp_int_##index##_cfg, PRE_KERNEL_1,                             \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(GRP_INT_RX_INIT);

#endif /* DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 0 */
