/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#define DT_DRV_COMPAT renesas_rx_external_interrupt_icum

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/misc/renesas_rx_external_interrupt/renesas_rx_external_interrupt.h>
#include <errno.h>
#include <soc.h>

#define ICU_SLIXR_BASE_ADDRESS DT_REG_ADDR_BY_NAME(DT_NODELABEL(icu), SLIXR)
#define SLIXRi_REG(i)          (ICU_SLIXR_BASE_ADDRESS + (i - 128) * 2)

#define ICU_SLIXR_EXT_BASE_IRQ_NUMBER 112
#define ICU_SLIXR_EXTERNAL_IRQ_SOURCE 0x3FF

enum icu_irq_mode {
	ICU_FALLING = 0,
	ICU_RISING,
	ICU_BOTH_EDGE,
	ICU_LOW_LEVEL,
	ICU_MODE_NONE,
};

enum icu_dig_filt {
	DISENABLE_DIG_FILT,
	ENABLE_DIG_FILT,
};

enum irq_sampling_clock {
	IRQ_SAMPLING_CLOCK_DIV_1 = 0,
	IRQ_SAMPLING_CLOCK_DIV_8,
	IRQ_SAMPLING_CLOCK_DIV_32,
	IRQ_SAMPLING_CLOCK_DIV_64,
	IRQ_SAMPLING_CLOCK_DIV_128,
	IRQ_SAMPLING_CLOCK_DIV_4,
	IRQ_SAMPLING_CLOCK_DIV_16,
};

struct gpio_rx_irq_config {
	mem_addr_t reg;
	unsigned int channel;
	enum icu_irq_mode trigger;
	uint8_t sampling_clock;
	enum icu_dig_filt digital_filter;
	unsigned int irq;
};

struct gpio_rx_irq_data {
	struct gpio_rx_callback callback;
	struct k_sem irq_sem;
};

/**
 * @brief setting interrupt for gpio input
 *
 * @param dev devive instance for gpio interrupt line
 * @param callback setting context for the callback
 * @retval 0 if success
 * @retval -EBUSY if interrupt line is inuse
 * @retval -ENOTSUP if interrupt mode is not supported
 */
int gpio_rx_interrupt_set(const struct device *dev, struct gpio_rx_callback *callback)
{
	const struct gpio_rx_irq_config *config = dev->config;
	struct gpio_rx_irq_data *data = dev->data;
	R_ICU_IRQCR_Type *irqcr = (R_ICU_IRQCR_Type *)config->reg;
	enum icu_irq_mode trigger;

	irq_disable(config->irq);

	if (callback->mode == GPIO_INT_MODE_LEVEL) {
		if (callback->trigger != GPIO_INT_TRIG_LOW) {
			return -ENOTSUP;
		}

		trigger = ICU_LOW_LEVEL;
	} else if (callback->mode == GPIO_INT_MODE_EDGE) {
		switch (callback->trigger) {
		case GPIO_INT_TRIG_LOW:
			trigger = ICU_FALLING;
			break;
		case GPIO_INT_TRIG_HIGH:
			trigger = ICU_RISING;
			break;
		case GPIO_INT_TRIG_BOTH:
			trigger = ICU_BOTH_EDGE;
			break;
		default:
			return -ENOTSUP;
		}
	} else {
		return -ENOTSUP;
	}

	if (data->callback.port_num != callback->port_num || data->callback.pin != callback->pin) {
		if (0 != k_sem_take(&data->irq_sem, K_NO_WAIT)) {
			return -EBUSY;
		}
	}

	irqcr->IRQCR_b.IRQMD = trigger;
	data->callback = *callback;
	irq_enable(config->irq);

	return 0;
}

/**
 * @brief unset interrupt configuration for the gpio interrupt
 *
 * @param dev device instance for port irq line
 * @param port_num gpio port number
 * @param pin the pin to disable interrupt
 */
void gpio_rx_interrupt_unset(const struct device *dev, uint8_t port_num, uint8_t pin)
{
	const struct gpio_rx_irq_config *config = dev->config;
	struct gpio_rx_irq_data *data = dev->data;

	if ((port_num != data->callback.port_num) || (pin != data->callback.pin)) {
		return;
	}

	irq_disable(config->irq);
	k_sem_give(&data->irq_sem);
}

static void gpio_rx_isr(const struct device *dev)
{
	const struct gpio_rx_irq_data *data = dev->data;
	const struct gpio_rx_irq_config *config = dev->config;

	R_BSP_IrqStatusClear(config->irq);
	data->callback.isr(data->callback.port, data->callback.pin);
}

static int gpio_rx_config_sampling(uint8_t sampling_clock)
{
	uint8_t fclksel;

	switch (sampling_clock) {
	case 1:
		fclksel = IRQ_SAMPLING_CLOCK_DIV_1;
		break;
	case 8:
		fclksel = IRQ_SAMPLING_CLOCK_DIV_8;
		break;
	case 32:
		fclksel = IRQ_SAMPLING_CLOCK_DIV_32;
		break;
	case 64:
		fclksel = IRQ_SAMPLING_CLOCK_DIV_64;
		break;
	case 128:
		fclksel = IRQ_SAMPLING_CLOCK_DIV_128;
		break;
	case 4:
		fclksel = IRQ_SAMPLING_CLOCK_DIV_4;
		break;
	case 16:
		fclksel = IRQ_SAMPLING_CLOCK_DIV_16;
		break;
	default:
		return -ENOTSUP;
	}

	return fclksel;
}

static int gpio_rx_interrupt_init(const struct device *dev)
{
	const struct gpio_rx_irq_config *config = dev->config;
	struct gpio_rx_irq_data *data = dev->data;
	R_ICU_IRQCR_Type *irqcr = (R_ICU_IRQCR_Type *)config->reg;

	irqcr->IRQCR_b.FLTEN = config->digital_filter;

	if (config->digital_filter) {
		int filter_value = gpio_rx_config_sampling(config->sampling_clock);

		if (filter_value < 0) {
			return filter_value;
		}

		irqcr->IRQCR_b.FCLKSEL = filter_value;
	}

	irqcr->IRQCR_b.IRQMD = config->trigger;

	if (config->channel > 15 && config->channel < 32) {
		/**
		 * The SLIXRn register is used to assign interrupt sources
		 * assigned to software configurable interrupt, or external pin interrupts
		 * (IRQ16 pin interrupt to IRQ31 pin interrupt)
		 * to interrupt vector numbers 128 to 143.
		 */
		int irq_number = ICU_SLIXR_EXT_BASE_IRQ_NUMBER + config->channel;

		if (config->irq == irq_number) {
			volatile uint16_t *icu_slixr = (uint16_t *)SLIXRi_REG(config->irq);

			*icu_slixr = ICU_SLIXR_EXTERNAL_IRQ_SOURCE;
		} else {
			__ASSERT(config->irq == irq_number,
				 "Config IRQ nummber is not valid (config->irq=%d)!", config->irq);
			return -EINVAL;
		}
	}

	k_sem_init(&data->irq_sem, 1, 1);

	return 0;
}

#define GPIO_INTERRUPT_INIT(index)                                                                 \
	static const struct gpio_rx_irq_config gpio_rx_irq_config##index = {                       \
		.reg = DT_INST_REG_ADDR(index),                                                    \
		.channel = DT_INST_PROP(index, channel),                                           \
		.trigger = DT_INST_ENUM_IDX_OR(index, renesas_trigger, ICU_FALLING),               \
		.digital_filter =                                                                  \
			DT_INST_PROP_OR(index, renesas_digital_filtering, DISENABLE_DIG_FILT),     \
		.sampling_clock = DT_INST_PROP_OR(index, renesas_sample_clock_div,                 \
						  IRQ_SAMPLING_CLOCK_DIV_1),                       \
		.irq = DT_INST_IRQ(index, irq),                                                    \
	};                                                                                         \
	static struct gpio_rx_irq_data gpio_rx_irq_data##index;                                    \
	static int gpio_rx_irq_init##index(const struct device *dev)                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ(index, irq), DT_INST_IRQ(index, priority), gpio_rx_isr,    \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		return gpio_rx_interrupt_init(dev);                                                \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(index, gpio_rx_irq_init##index, NULL, &gpio_rx_irq_data##index,      \
			      &gpio_rx_irq_config##index, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY, \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(GPIO_INTERRUPT_INIT)
