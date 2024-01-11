/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_ra8_gpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include <soc.h>

#define PRCR_KEY   0xA500U
#define VBATT_PORT 4

static const gpio_pin_t vbatt_pins[] = {
	2U,
	3U,
	4U,
};

#define RA8_INVALID_PORT_ADDR 0UL

#define RA8_GPIO_PORT_ADDR(nodelabel) \
	COND_CODE_1(DT_NODE_EXISTS(DT_NODELABEL(nodelabel)), \
	(DT_REG_ADDR(DT_NODELABEL(nodelabel))), (RA8_INVALID_PORT_ADDR))

struct gpio_ra8_config {
	struct gpio_driver_config common;
	uint8_t port_num;
	R_PORT0_Type *port;
};

struct gpio_ra8_data {
	struct gpio_driver_data common;
};

static int gpio_ra8_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_ra8_config *config = dev->config;

	if (((flags & GPIO_INPUT) != 0U) && ((flags & GPIO_OUTPUT) != 0U)) {
		return -ENOTSUP;
	}

	if ((flags & GPIO_PULL_DOWN) != 0U) {
		return -ENOTSUP;
	}

	if (config->port_num == VBATT_PORT) {
		uint32_t clear = 0;

		for (int i = 0; i < ARRAY_SIZE(vbatt_pins); i++) {
			if (vbatt_pins[i] == pin) {
				WRITE_BIT(clear, i, 1);
			}
		}

		R_SYSTEM->PRCR = ((R_SYSTEM->PRCR | PRCR_KEY) | R_SYSTEM_PRCR_PRC1_Msk);

		R_SYSTEM->VBTICTLR &= (uint8_t)~clear;

		R_SYSTEM->PRCR = ((R_SYSTEM->PRCR | PRCR_KEY) & (uint16_t)~R_SYSTEM_PRCR_PRC1_Msk);
	}

#if BSP_CFG_MCU_PART_SERIES == 8
	R_PMISC->PWPRS = 0;
	R_PMISC->PWPRS = BIT(R_PMISC_PWPR_PFSWE_Pos);
#else
	R_PMISC->PWPR = 0;
	R_PMISC->PWPR = BIT(R_PMISC_PWPR_PFSWE_Pos);
#endif

	uint32_t value = R_PFS->PORT[config->port_num].PIN[pin].PmnPFS;

	/* Change mode to general IO mode and disable IRQ and Analog input */
	WRITE_BIT(value, R_PFS_PORT_PIN_PmnPFS_PMR_Pos, 0);
	WRITE_BIT(value, R_PFS_PORT_PIN_PmnPFS_ASEL_Pos, 0);
	WRITE_BIT(value, R_PFS_PORT_PIN_PmnPFS_ISEL_Pos, 0);

	if ((flags & GPIO_OUTPUT) != 0U) {
		/* Set output pin initial value */
		if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
			WRITE_BIT(value, R_PFS_PORT_PIN_PmnPFS_PODR_Pos, 0);
		} else if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
			WRITE_BIT(value, R_PFS_PORT_PIN_PmnPFS_PODR_Pos, 1);
		}

		WRITE_BIT(value, R_PFS_PORT_PIN_PmnPFS_PDR_Pos, 1);
	} else {
		WRITE_BIT(value, R_PFS_PORT_PIN_PmnPFS_PDR_Pos, 0);
	}

	R_PFS->PORT[config->port_num].PIN[pin].PmnPFS = value;

#if BSP_CFG_MCU_PART_SERIES == 8
	R_PMISC->PWPRS = 0;
	R_PMISC->PWPRS = BIT(R_PMISC_PWPR_B0WI_Pos);
#else
	R_PMISC->PWPR = 0;
	R_PMISC->PWPR = BIT(R_PMISC_PWPR_B0WI_Pos);
#endif

	return 0;
}

static int gpio_ra8_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_ra8_config *config = dev->config;
	R_PORT0_Type *port = config->port;

	*value = port->PIDR;

	return 0;
}

static int gpio_ra8_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					gpio_port_value_t value)
{
	const struct gpio_ra8_config *config = dev->config;
	R_PORT0_Type *port = config->port;

	port->PODR = ((port->PODR & ~mask) | (value & mask));

	return 0;
}

static int gpio_ra8_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_ra8_config *config = dev->config;
	R_PORT0_Type *port = config->port;

	port->PODR = (port->PODR | pins);

	return 0;
}

static int gpio_ra8_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_ra8_config *config = dev->config;
	R_PORT0_Type *port = config->port;

	port->PODR = (port->PODR & ~pins);

	return 0;
}

static int gpio_ra8_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_ra8_config *config = dev->config;
	R_PORT0_Type *port = config->port;

	port->PODR = (port->PODR ^ pins);

	return 0;
}

static const struct gpio_driver_api gpio_ra8_drv_api_funcs = {
	.pin_configure = gpio_ra8_pin_configure,
	.port_get_raw = gpio_ra8_port_get_raw,
	.port_set_masked_raw = gpio_ra8_port_set_masked_raw,
	.port_set_bits_raw = gpio_ra8_port_set_bits_raw,
	.port_clear_bits_raw = gpio_ra8_port_clear_bits_raw,
	.port_toggle_bits = gpio_ra8_port_toggle_bits,
	.pin_interrupt_configure = NULL,
	.manage_callback = NULL,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = gpio_ra8_port_get_direction,
#endif
};

static int gpio_ra8_init(const struct device *dev)
{
	return 0;
}

#define GPIO_DEVICE_INIT(__node, __port_num, __suffix, __addr)                                     \
	static const struct gpio_ra8_config gpio_ra8_config_##__suffix = {                         \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_NGPIOS(16U),              \
			},                                                                         \
		.port_num = __port_num,                                                            \
		.port = (R_PORT0_Type *)__addr,                                                    \
	};                                                                                         \
	static struct gpio_ra8_data gpio_ra8_data_##__suffix;                                      \
	DEVICE_DT_DEFINE(__node, gpio_ra8_init, PM_DEVICE_DT_GET(__node),                          \
			 &gpio_ra8_data_##__suffix, &gpio_ra8_config_##__suffix, PRE_KERNEL_1,     \
			 CONFIG_GPIO_INIT_PRIORITY, &gpio_ra8_drv_api_funcs)

#define GPIO_DEVICE_INIT_RA8(__suffix)                                                             \
	GPIO_DEVICE_INIT(DT_NODELABEL(gpio##__suffix),                                             \
			 DT_PROP(DT_NODELABEL(gpio##__suffix), port), __suffix,                    \
			 DT_REG_ADDR(DT_NODELABEL(gpio##__suffix)))

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio0), okay)
GPIO_DEVICE_INIT_RA8(0);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio0), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio1), okay)
GPIO_DEVICE_INIT_RA8(1);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio1), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio2), okay)
GPIO_DEVICE_INIT_RA8(2);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio2), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio3), okay)
GPIO_DEVICE_INIT_RA8(3);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio3), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio4), okay)
GPIO_DEVICE_INIT_RA8(4);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio4), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio5), okay)
GPIO_DEVICE_INIT_RA8(5);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio5), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio6), okay)
GPIO_DEVICE_INIT_RA8(6);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio6), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio7), okay)
GPIO_DEVICE_INIT_RA8(7);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio7), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio8), okay)
GPIO_DEVICE_INIT_RA8(8);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio8), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpio9), okay)
GPIO_DEVICE_INIT_RA8(9);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpio9), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioa), okay)
GPIO_DEVICE_INIT_RA8(a);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioa), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiob), okay)
GPIO_DEVICE_INIT_RA8(b);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpiob), okay) */
