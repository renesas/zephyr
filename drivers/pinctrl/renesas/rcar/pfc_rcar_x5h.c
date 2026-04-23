/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_pfc_x5h

#include "pfc_rcar_x5h.h"
#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/device_mmio.h>

#define PFC_RCAR_PMMR            0x0
#define PFC_RCAR_GPSR            0x40
#define PFC_RCAR_ALTSEL0         0x60
#define PFC_RCAR_ALTSEL(i)       (PFC_RCAR_ALTSEL0 + (i) * sizeof(uint32_t))
#define PFC_RCAR_ALTSEL_NUM_REG  4
#define PFC_RCAR_DRVCTRL_NUM_REG 3

/* swap both arguments */
#define PFC_REG_ADDRESS(idx, inst) DT_INST_REG_ADDR_BY_IDX(inst, idx)
static uintptr_t reg_base[] = {LISTIFY(DT_NUM_REGS(DT_DRV_INST(0)), PFC_REG_ADDRESS, (,), 0) };

#define PFC_REG_SIZE(idx, inst) DT_INST_REG_SIZE_BY_IDX(inst, idx)
static const uintptr_t __maybe_unused reg_sizes[] = {
	LISTIFY(DT_NUM_REGS(DT_DRV_INST(0)), PFC_REG_SIZE, (,), 0) };

#define PFC_RCAR_DRIVE_MAX  24
#define PFC_RCAR_DRIVE_STEP (PFC_RCAR_DRIVE_MAX / 8)

/* Some registers such as ALTSEL, GPSR or DRVCTRL are protected and
 * must be preceded to a write to PMMR with the inverse value.
 */
static void pfc_rcar_write(uintptr_t pfc_base, uint32_t offs, uint32_t val)
{
	sys_write32(~val, pfc_base + PFC_RCAR_PMMR);
	sys_write32(val, pfc_base + offs);
}

/* Set the pin either in gpio or peripheral */
static void pfc_rcar_set_gpsr(uintptr_t pfc_base, uint16_t pin, bool peripheral)
{
	uint32_t val = sys_read32(pfc_base + PFC_RCAR_GPSR);
	uint8_t bit = pin % 32;

	if (peripheral) {
		val |= BIT(bit);
	} else {
		val &= ~BIT(bit);
	}
	pfc_rcar_write(pfc_base, PFC_RCAR_GPSR, val);
}

/* Set peripheral function */
static void pfc_rcar_set_altsel(uintptr_t pfc_base, const struct rcar_pin_func *rcar_func)
{
	uint16_t reg_offs;
	uint32_t val;

	for (uint8_t i = 0; i < PFC_RCAR_ALTSEL_NUM_REG; i++) {
		reg_offs = PFC_RCAR_ALTSEL(i);
		val = sys_read32(pfc_base + reg_offs);
		val &= ~BIT(rcar_func->pin);

		if ((rcar_func->func & BIT(i)) != 0U) {
			val |= BIT(rcar_func->pin);
		}
		pfc_rcar_write(pfc_base, reg_offs, val);
	}
}

static const uint32_t *pfc_rcar_get_drive_reg(uint16_t pin, uint8_t *bit)
{
	const struct pfc_drive_reg *drive_regs = pfc_rcar_get_drive_regs();

	while ((drive_regs->drvctrl[0] != 0U) && (drive_regs->drvctrl[1] != 0U) &&
	       (drive_regs->drvctrl[2] != 0U)) {
		for (size_t i = 0U; i < ARRAY_SIZE(drive_regs->pins); i++) {
			if (drive_regs->pins[i] == pin) {
				*bit = i;
				return drive_regs->drvctrl;
			}
		}
		drive_regs++;
	}

	return NULL;
}

static int pfc_rcar_set_drive_strength(uintptr_t pfc_base, uint16_t pin, uint8_t strength)
{
	uint8_t bit, step;
	uint32_t val;
	const uint32_t *drvctrl_regs;

	drvctrl_regs = pfc_rcar_get_drive_reg(pin, &bit);
	if (drvctrl_regs == NULL) {
		return -EINVAL;
	}

	step = PFC_RCAR_DRIVE_STEP;
	if ((strength < step) || (strength > PFC_RCAR_DRIVE_MAX)) {
		return -EINVAL;
	}

	strength = (strength / step) - 1U;

	for (uint8_t i = 0; i < PFC_RCAR_DRVCTRL_NUM_REG; i++) {
		/* clear previous drive strength value */
		val = sys_read32(pfc_base + drvctrl_regs[i]);
		val &= ~BIT(bit);
		if ((strength & BIT(i)) != 0U) {
			val |= BIT(bit);
		}
		pfc_rcar_write(pfc_base, drvctrl_regs[i], val);
	}

	return 0;
}

static const struct pfc_bias_reg *pfc_rcar_get_bias_reg(uint16_t pin, uint8_t *bit)
{
	const struct pfc_bias_reg *bias_regs = pfc_rcar_get_bias_regs();

	/* Loop around all the registers to find the bit for a given pin */
	while (bias_regs->puen && bias_regs->pud) {
		for (size_t i = 0U; i < ARRAY_SIZE(bias_regs->pins); i++) {
			if (bias_regs->pins[i] == pin) {
				*bit = i;
				return bias_regs;
			}
		}
		bias_regs++;
	}

	return NULL;
}

int pfc_rcar_set_bias(uintptr_t pfc_base, uint16_t pin, uint16_t flags)
{
	uint32_t val;
	uint8_t bit;
	const struct pfc_bias_reg *bias_reg = pfc_rcar_get_bias_reg(pin, &bit);

	if (bias_reg == NULL) {
		return -EINVAL;
	}

	/* pull enable/disable*/
	val = sys_read32(pfc_base + bias_reg->puen);
	if ((flags & RCAR_PIN_FLAGS_PUEN) == 0U) {
		pfc_rcar_write(pfc_base, bias_reg->puen, val & ~BIT(bit));
		return 0;
	}
	pfc_rcar_write(pfc_base, bias_reg->puen, val | BIT(bit));

	/* pull - up/down */
	val = sys_read32(pfc_base + bias_reg->pud);
	if (flags & RCAR_PIN_FLAGS_PUD) {
		pfc_rcar_write(pfc_base, bias_reg->pud, val | BIT(bit));
	} else {
		pfc_rcar_write(pfc_base, bias_reg->pud, val & ~BIT(bit));
	}
	return 0;
}

int pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	int ret = 0;
	uint8_t reg_index;
	uintptr_t pfc_base;

	ret = pfc_rcar_get_reg_index(pin->pin, &reg_index);
	if (ret) {
		return ret;
	}

	if (reg_index >= ARRAY_SIZE(reg_base)) {
		return -EINVAL;
	}

	pfc_base = reg_base[reg_index];

	/* Select function for pin */
	if ((pin->flags & RCAR_PIN_FLAGS_FUNC_SET) != 0U) {
		pfc_rcar_set_gpsr(pfc_base, pin->pin, true);
		pfc_rcar_set_altsel(pfc_base, &pin->func);

		if ((pin->flags & RCAR_PIN_FLAGS_PULL_SET) != 0U) {
			ret = pfc_rcar_set_bias(pfc_base, pin->pin, pin->flags);
			if (ret < 0) {
				return ret;
			}
		}
	} else {
		pfc_rcar_set_gpsr(pfc_base, pin->pin, false);
	}

	if (pin->drive_strength != 0U) {
		ret = pfc_rcar_set_drive_strength(pfc_base, pin->pin, pin->drive_strength);
	}

	return ret;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	int ret = 0;

	ARG_UNUSED(reg);
	while (pin_cnt-- > 0U) {
		ret = pinctrl_configure_pin(pins++);
		if (ret < 0) {
			break;
		}
	}

	return ret;
}

#if defined(DEVICE_MMIO_IS_IN_RAM)
__boot_func static int pfc_rcar_driver_init(void)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(reg_base); i++) {
		device_map(reg_base + i, reg_base[i], reg_sizes[i], K_MEM_CACHE_NONE);
	}
	return 0;
}

SYS_INIT(pfc_rcar_driver_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif
