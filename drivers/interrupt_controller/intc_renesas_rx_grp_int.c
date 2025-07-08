/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 * Copyright (c) 2025 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_grp_intc

#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/drivers/interrupt_controller/intc_renesas_rx_grp_int.h>
#include <errno.h>

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 0

extern void group_bl0_handler_isr(void);
extern void group_bl1_handler_isr(void);
extern void group_bl2_handler_isr(void);
extern void group_al0_handler_isr(void);
extern void group_al1_handler_isr(void);

#define VECT_GROUP_BL0 DT_IRQN(DT_NODELABEL(group_irq_bl0))
#define VECT_GROUP_BL1 DT_IRQN(DT_NODELABEL(group_irq_bl1))
#define VECT_GROUP_BL2 DT_IRQN(DT_NODELABEL(group_irq_bl2))
#define VECT_GROUP_AL0 DT_IRQN(DT_NODELABEL(group_irq_al0))
#define VECT_GROUP_AL1 DT_IRQN(DT_NODELABEL(group_irq_al1))

#define DEV_CFG(cfg, dev) struct rx_grp_int_cfg *cfg = (struct rx_grp_int_cfg *)(dev->config)

/**
 * @brief configuration data for a group interrupt device
 */
struct rx_grp_int_cfg {
	/* address of the Group Interrupt Request Enable Register (GENxxx)  */
	volatile uint32_t *gen;
	/* vector number of the interrupt */
	const uint8_t vect;
	/* priority of the interrupt */
	const uint8_t prio;
};

static struct k_spinlock lock;

int rx_grp_intc_set_callback(const struct device *dev, bsp_int_src_t vector, bsp_int_cb_t callback,
			     void *context)
{
	bsp_int_err_t err;

	ARG_UNUSED(dev);

	err = R_BSP_InterruptWrite_EX(vector, callback, context);

	if (err != BSP_INT_SUCCESS) {
		err = -EINVAL;
	}

	return err;
}

int rx_grp_intc_set_grp_int(const struct device *dev, bsp_int_src_t vector, bool set)
{
	if (dev == NULL) {
		return -EINVAL;
	}

	DEV_CFG(cfg, dev);

	volatile bsp_int_ctrl_t group_priority;
	bsp_int_err_t err;

	group_priority.ipl = cfg->prio;

	k_spinlock_key_t key = k_spin_lock(&lock);

	if (set) {
		/* ENABLE GROUP INTERRUPTS */
		err = R_BSP_InterruptControl(vector, BSP_INT_CMD_GROUP_INTERRUPT_ENABLE,
					     (void *)&group_priority);
	} else {
		/* DISABLE GROUP INTERRUPTS */
		err = R_BSP_InterruptControl(vector, BSP_INT_CMD_GROUP_INTERRUPT_DISABLE, NULL);
	}

	k_spin_unlock(&lock, key);

	if (err != BSP_INT_SUCCESS) {
		return -EINVAL;
	}

	return err;
}

int rx_grp_intc_set_gen(const struct device *dev, int is_number, bool set)
{

	if ((is_number < 0 || is_number > 31) || dev == NULL) {
		return -EINVAL;
	}

	DEV_CFG(cfg, dev);

	k_spinlock_key_t key = k_spin_lock(&lock);

	if (set) {
		/* ENABLE GROUP INTERRUPTS */
		*cfg->gen |= (1U << is_number);
	} else {
		/* DISABLE GROUP INTERRUPTS */
		*cfg->gen = 0;
	}

	k_spin_unlock(&lock, key);

	return 0;
}

static int rx_grp_intc_init(const struct device *dev)
{

	DEV_CFG(cfg, dev);

	volatile bsp_int_ctrl_t group_priority;
	group_priority.ipl = cfg->prio;
	int err = 0;

	switch (cfg->vect) {

	case VECT_GROUP_BL0:
		IRQ_CONNECT(VECT_GROUP_BL0, 0, group_bl0_handler_isr, NULL, 0);
		break;

	case VECT_GROUP_BL1:
		IRQ_CONNECT(VECT_GROUP_BL1, 0, group_bl1_handler_isr, NULL, 0);
		break;

	case VECT_GROUP_BL2:
		IRQ_CONNECT(VECT_GROUP_BL2, 0, group_bl2_handler_isr, NULL, 0);
		break;

	case VECT_GROUP_AL0:
		IRQ_CONNECT(VECT_GROUP_AL0, 0, group_al0_handler_isr, NULL, 0);
		break;

	case VECT_GROUP_AL1:
		IRQ_CONNECT(VECT_GROUP_AL1, 0, group_al1_handler_isr, NULL, 0);
		break;

	default:
		/* ERROR */
		err = -EINVAL;
		break;
	}

	irq_enable(cfg->vect);

	return err;
}

#endif /* ZEPHYR_INCLUDE_INTC_GRP_RX_ARCH_H_ */ /* DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 0 */

#define GRP_INT_RX_INIT(index)                                                                     \
	static struct rx_grp_int_cfg rx_grp_int_##index##_cfg = {                                  \
		.gen = (uint32_t *)DT_INST_REG_ADDR_BY_NAME(index, GEN),                           \
		.vect = DT_INST_IRQN(index),                                                       \
		.prio = DT_INST_IRQ(index, priority)};                                             \
	static int rx_grp_int_##index##_init(const struct device *dev)                             \
	{                                                                                          \
		return rx_grp_intc_init(dev);                                                      \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(index, rx_grp_int_##index##_init, NULL, NULL,                        \
			      &rx_grp_int_##index##_cfg, PRE_KERNEL_1,                             \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(GRP_INT_RX_INIT);
