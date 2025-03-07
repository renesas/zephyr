/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __TEST_RENESAS_DTC_H__
#define __TEST_RENESAS_DTC_H__

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#define DEV_OUT       DT_GPIO_CTLR(DT_INST(0, test_gpio_basic_api), out_gpios)
#define DEV_IN        DT_GPIO_CTLR(DT_INST(0, test_gpio_basic_api), in_gpios)
#define DEV           DEV_OUT /* DEV_OUT should equal DEV_IN, we test for this */
#define PIN_OUT       DT_GPIO_PIN(DT_INST(0, test_gpio_basic_api), out_gpios)
#define PIN_OUT_FLAGS DT_GPIO_FLAGS(DT_INST(0, test_gpio_basic_api), out_gpios)
#define PIN_IN        DT_GPIO_PIN(DT_INST(0, test_gpio_basic_api), in_gpios)
#define PIN_IN_FLAGS  DT_GPIO_FLAGS(DT_INST(0, test_gpio_basic_api), in_gpios)

#define MAX_INT_CNT 3
struct drv_data {
	struct gpio_callback gpio_cb;
	gpio_flags_t mode;
	int index;
	int aux;
};

struct gpio_rx_irq_info {
	const struct device *port_irq;
	const uint8_t *const pins;
	size_t num;
};

struct gpio_rx_config {
	struct gpio_driver_config common;
	uint8_t port_num;
	volatile uint8_t *pinmux;
	volatile struct {
		volatile uint8_t *pdr;
		volatile uint8_t *podr;
		volatile uint8_t *pidr;
		volatile uint8_t *pmr;
		volatile uint8_t *odr0;
		volatile uint8_t *odr1;
		volatile uint8_t *pcr;
		volatile uint8_t *dscr;
		volatile uint8_t *dscr2;
	} reg;
#if CONFIG_RENESAS_RX_EXTERNAL_INTERRUPT
	const struct gpio_rx_irq_info *irq_info;
	const size_t irq_info_size;
#endif
};

struct gpio_rx_irq_config {
	mem_addr_t reg;
	unsigned int channel;
	enum icu_irq_mode trigger;
	uint8_t sample_clock;
	enum icu_dig_filt digital_filter;
	unsigned int irq;
};

#endif /* __TEST_RENESAS_DTC_H__ */
