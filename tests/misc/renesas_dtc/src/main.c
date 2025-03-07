/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <soc.h>
#include <zephyr/ztest.h>
#include <zephyr/drivers/misc/renesas_rx_dtc/renesas_rx_dtc.h>
#include <zephyr/drivers/misc/renesas_rx_external_interrupt/renesas_rx_external_interrupt.h>
#include <zephyr/drivers/interrupt_controller/intc_rx_icu.h>
#include "test_renesas_dtc.h"

/* Alignment required for transfer_info_t structures. */
#define DTC_TRANSFER_INFO_ALIGNMENT DTC_ALIGN_VARIABLE(4UL)
#define TEST_DTC_BUFFER_SIZE        (128)
#define TRANSFER_TEST_SIZE          (2 * 1024)
/* Configure the number of transfers for normal mode dests */
#define TEST_NORMAL_COUNT           (1024)
/* Configure repeat size and count for repeat mode tests */
#define TEST_REPEAT_SIZE            (256)
#define TEST_REPEAT_COUNT           (4)
/* Configure block size and count for repeat mode tests */
#define TEST_BLOCK_SIZE             (256)
#define TEST_BLOCK_COUNT            (4)

static const struct device *const dev_in = DEVICE_DT_GET(DEV_IN);
static const struct device *const dev_out = DEVICE_DT_GET(DEV_OUT);
static const struct device *const dtc = DEVICE_DT_GET(DT_NODELABEL(dtc));

/* Source address used by all transfer tests. */
uint8_t g_transfer_test_src[TRANSFER_TEST_SIZE] DTC_TRANSFER_INFO_ALIGNMENT;
/* Destination address used by all transfer tests */
uint8_t g_transfer_test_dest[TRANSFER_TEST_SIZE + 1] DTC_TRANSFER_INFO_ALIGNMENT;

ZTEST_BMEM transfer_info_t *tx_transfer_info;
ZTEST_BMEM transfer_info_t *rx_transfer_info;

ZTEST_BMEM static volatile int gv_transfer_complete;
ZTEST_BMEM static struct drv_data cb_data;
volatile int acctivate_dtc;
ZTEST_BMEM static struct k_sem sem;

/* Default transfer info that tests can use when configuring a transfer instance. */
transfer_info_t gc_transfer_test_info DTC_TRANSFER_INFO_ALIGNMENT = {
	.transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED,
	.transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
	.transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
	.length = TEST_NORMAL_COUNT,
	.transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE,
	.p_dest = &g_transfer_test_dest[0],
	.p_src = &g_transfer_test_src[0],
	.transfer_settings_word_b.irq = TRANSFER_IRQ_EACH,
	.transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL,
};

/* Define for test stop transfer */
transfer_info_t transfer_test_info_stop DTC_TRANSFER_INFO_ALIGNMENT = {
	.transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED,
	.transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
	.transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
	.transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE,
	.transfer_settings_word_b.irq = TRANSFER_IRQ_EACH,
	.transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL,
};

static void gpio_callback(const struct device *dev, struct gpio_callback *gpio_cb, uint32_t pins)
{
	k_sem_give(&sem);
	gv_transfer_complete++;
}

static int gpio_init(const struct device *dev_in, const struct device *dev_out,
		     gpio_callback_handler_t handler)
{
	int rc = gpio_pin_interrupt_configure(dev_in, PIN_IN, GPIO_INT_DISABLE);

	if (rc == 0) {
		rc = gpio_pin_interrupt_configure(dev_out, PIN_OUT, GPIO_INT_DISABLE);
	}
	if (rc == 0) {
		/* 1. set PIN_OUT */
		rc = gpio_pin_configure(dev_out, PIN_OUT, (GPIO_OUTPUT_LOW | PIN_OUT_FLAGS));
	}

	if (rc == 0) {
		/* 2. configure PIN_IN callback, but don't enable */
		rc = gpio_pin_configure(dev_in, PIN_IN, (GPIO_INPUT | PIN_IN_FLAGS));
	}

	if (rc == 0) {
		gpio_init_callback(&cb_data.gpio_cb, handler, BIT(PIN_IN));
		rc = gpio_add_callback(dev_in, &cb_data.gpio_cb);
	}

	return rc;
}

static void gpio_config_trigger_callback(const struct device *dev_in, const struct device *dev_out,
					 int enable_cb)
{
	gpio_pin_set(dev_out, PIN_OUT, 0);
	k_sleep(K_MSEC(100));

	gv_transfer_complete = 0;
	if (enable_cb == 1) {
		gpio_pin_interrupt_configure(dev_in, PIN_IN, GPIO_INT_EDGE_RISING);
	} else {
		gpio_pin_interrupt_configure(dev_in, PIN_IN, GPIO_INT_DISABLE);
	}
	k_sleep(K_MSEC(100));
}

static struct gpio_rx_irq_info *query_irq_info(const struct device *dev, uint32_t pin)
{
	const struct gpio_rx_config *config = dev->config;
	int irq_info_size = config->irq_info_size;

	for (int i = 0; i < irq_info_size; i++) {
		const struct gpio_rx_irq_info *info = &config->irq_info[i];

		for (int j = 0; j < info->num; j++) {
			if (info->pins[j] == pin) {
				return (struct gpio_rx_irq_info *)info;
			}
		}
	}

	return NULL;
}

static void *dtc_setup(void)
{

	const struct gpio_rx_irq_info *irq_info = query_irq_info(dev_in, PIN_IN);

	const struct device *ext_irq = irq_info->port_irq;
	const struct gpio_rx_irq_config *config = ext_irq->config;
	int err = 0;

	acctivate_dtc = config->irq;
	printk("activation_irq: %d\n", acctivate_dtc);

	err = gpio_init(dev_in, dev_out, gpio_callback);
	zassert_equal(err, 0, "gpio_init failed");
	/* init semaphore */
	k_sem_init(&sem, 1, 1);

	return NULL;
}

static void dtc_before_fn(void *unused)
{
	ARG_UNUSED(unused);
	/* Write known value to source and destination. */
	for (int i = 0; i < TRANSFER_TEST_SIZE; i++) {
		g_transfer_test_src[i] = i % 256;
	}

	memset(g_transfer_test_dest, 0xff, sizeof(g_transfer_test_dest));

	gc_transfer_test_info.transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED;
	gc_transfer_test_info.transfer_settings_word_b.dest_addr_mode =
		TRANSFER_ADDR_MODE_INCREMENTED;
	gc_transfer_test_info.transfer_settings_word_b.src_addr_mode =
		TRANSFER_ADDR_MODE_INCREMENTED;
	gc_transfer_test_info.length = TEST_DTC_BUFFER_SIZE;
	gc_transfer_test_info.transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE;
	gc_transfer_test_info.p_dest = &g_transfer_test_dest[0];
	gc_transfer_test_info.p_src = &g_transfer_test_src[0];
	gc_transfer_test_info.transfer_settings_word_b.irq = TRANSFER_IRQ_EACH;
	gc_transfer_test_info.transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;

	k_sem_init(&sem, 1, 1);
}

static void log_dest(uint8_t *dest, int length)
{
	printk("dest: ");
	for (int j = 0; j < length; j++) {
		printk("%d ", dest[j]);
	}
	printk("\n");
}

static void log_src(uint8_t *src, int length)
{
	printk("src: ");
	for (int j = 0; j < length; j++) {
		printk("%d ", src[j]);
	}
	printk("\n");
}

static int fn_normal_mode_infoGet(void)
{
	transfer_info_t *info = &gc_transfer_test_info;
	transfer_properties_t transfer_info;

	zassert_equal(0, dtc_renesas_rx_configuration(dtc, acctivate_dtc, info),
		      "dtc_renesas_rx_configuration failed");
	/* Get trasnfer info */
	zassert_equal(0, dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info),
		      "dtc get info failed!");
	zassert_equal(DTC_MAX_NORMAL_TRANSFER_LENGTH, transfer_info.transfer_length_max);
	zassert_equal(0, transfer_info.block_count_max);
	zassert_equal(TEST_DTC_BUFFER_SIZE, transfer_info.transfer_length_remaining);
	zassert_equal(0, transfer_info.block_count_remaining);

	return TC_PASS;
}

static int fn_repeat_mode_infoGet(void)
{
	transfer_info_t *info = &gc_transfer_test_info;
	transfer_properties_t transfer_info;

	info->transfer_settings_word_b.mode = TRANSFER_MODE_REPEAT;
	zassert_equal(0, dtc_renesas_rx_configuration(dtc, acctivate_dtc, info),
		      "dtc_renesas_rx_configuration failed");
	/* Get trasnfer info */
	zassert_equal(0, dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info),
		      "dtc get info failed!");
	zassert_equal(DTC_MAX_REPEAT_TRANSFER_LENGTH, transfer_info.transfer_length_max);
	zassert_equal(0, transfer_info.block_count_max);
	zassert_equal(TEST_DTC_BUFFER_SIZE, transfer_info.transfer_length_remaining);

	return TC_PASS;
}

static int fn_block_mode_infoGet(void)
{
	transfer_info_t *info = &gc_transfer_test_info;
	transfer_properties_t transfer_info;

	info->transfer_settings_word_b.mode = TRANSFER_MODE_BLOCK;
	info->length = TEST_DTC_BUFFER_SIZE;
	info->num_blocks = 16;
	zassert_equal(0, dtc_renesas_rx_configuration(dtc, acctivate_dtc, info),
		      "dtc_renesas_rx_configuration failed");
	/* Get trasnfer info */
	zassert_equal(0, dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info),
		      "dtc get info failed!");
	zassert_equal(DTC_MAX_REPEAT_TRANSFER_LENGTH, transfer_info.transfer_length_max);
	zassert_equal(DTC_MAX_BLOCK_COUNT, transfer_info.block_count_max);
	zassert_equal(TEST_DTC_BUFFER_SIZE, transfer_info.transfer_length_remaining);
	zassert_equal(16, transfer_info.block_count_remaining);

	return TC_PASS;
}

static int fn_on_off_module_dtc(void)
{
	const uint16_t transfer_length = 16;
	/* Split the global buffers in two so each transfer has its own buffer. */
	uint8_t *src = &g_transfer_test_src[0];
	uint8_t *dest = &g_transfer_test_dest[0];
	/* Allocate array of transfers */
	transfer_info_t *info = &gc_transfer_test_info;
	transfer_properties_t transfer_info;

	/* Configure the first transfer to repeat mode */
	info->p_src = src;
	info->p_dest = dest;
	info->length = transfer_length;

	gpio_config_trigger_callback(dev_in, dev_out, 1);

	zassert_equal(dtc_renesas_rx_configuration(dtc, acctivate_dtc, info), 0,
		      "dtc_renesas_rx_configuration failed");

	/* Get trasnfer info */
	zassert_equal(dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info), 0,
		      "dtc_renesas_rx_infoGet failed");

	printf("transfer_length_remaining before start transfer: %d\n",
	       transfer_info.transfer_length_remaining);

	zassert_equal(dtc_renesas_rx_start_transfer(dtc, acctivate_dtc), 0,
		      "dtc_renesas_rx_start_transfer failed");

	/* turn off module DTC */
	zassert_equal(0, dtc_renesas_rx_off(dtc), "dtc_renesas_rx_off failed");

	gpio_pin_set(dev_out, PIN_OUT, 1);
	k_sem_take(&sem, K_FOREVER);
	gpio_pin_set(dev_out, PIN_OUT, 0);

	zassert_not_equal(src[0], dest[0], "Turn off the DTC module failed.");

	/* Get trasnfer info */
	zassert_equal(dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info), 0,
		      "dtc_renesas_rx_infoGet failed");

	printf("transfer_length_remaining after start and turn off DTC module: %d\n",
	       transfer_info.transfer_length_remaining);

	/* turn on module DTC */
	zassert_equal(0, dtc_renesas_rx_on(dtc), "dtc_renesas_rx_off failed");

	gpio_pin_set(dev_out, PIN_OUT, 1);
	k_sem_take(&sem, K_FOREVER);
	gpio_pin_set(dev_out, PIN_OUT, 0);

	/* Get trasnfer info */
	zassert_equal(dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info), 0,
		      "dtc_renesas_rx_infoGet failed");

	uint32_t transfers_to_complete = transfer_info.transfer_length_remaining;
	uint32_t transfer_length_completed = transfer_length - transfers_to_complete;

	printf("transfer_length_remaining after turn on again DTC module: %d\n",
	       transfers_to_complete);
	printf("transfer_length_completed after turn on again DTC module: %d\n",
	       transfer_length_completed);

	log_src(src, transfer_length);
	log_dest(dest, transfer_length);

	zassert_equal(0, memcmp(dest, src, transfer_length_completed), "memcmp failed");

	return TC_PASS;
}

static int fn_stop_and_reset_transfer(void)
{
	const uint16_t transfer_length = 16;
	/* Split the global buffers in two so each transfer has its own buffer. */
	uint8_t *src = &g_transfer_test_src[0];
	uint8_t *dest = &g_transfer_test_dest[0];
	/* Allocate array of transfers */
	transfer_info_t *info = &transfer_test_info_stop;
	transfer_properties_t transfer_info;

	/* Configure the first transfer to repeat mode */
	info->p_src = src;
	info->p_dest = dest;
	info->length = transfer_length;

	gpio_config_trigger_callback(dev_in, dev_out, 1);

	zassert_equal(dtc_renesas_rx_configuration(dtc, acctivate_dtc, info), 0,
		      "dtc_renesas_rx_configuration failed");

	zassert_equal(dtc_renesas_rx_start_transfer(dtc, acctivate_dtc), 0,
		      "dtc_renesas_rx_start_transfer failed");
	/* Get trasnfer info */
	zassert_equal(dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info), 0,
		      "dtc_renesas_rx_infoGet failed");

	printf("transfer_length_remaining before start transfer: %d\n",
	       transfer_info.transfer_length_remaining);
	/* stop transfer */
	zassert_equal(0, dtc_renesas_rx_stop_transfer(dtc, acctivate_dtc),
		      "dtc_renesas_rx_stop_transfer failed");

	gpio_pin_set(dev_out, PIN_OUT, 1);
	gpio_pin_set(dev_out, PIN_OUT, 0);

	zassert_not_equal(src[0], dest[0], "Stop transfer failed.");

	zassert_not_equal(0, dtc_renesas_rx_start_transfer(dtc, acctivate_dtc),
			  "dtc_renesas_rx_stop_transfer failed");

	info = &gc_transfer_test_info;
	info->p_src = src;
	info->p_dest = dest;
	info->length = transfer_length;

	zassert_equal(dtc_renesas_rx_configuration(dtc, acctivate_dtc, info), 0,
		      "dtc_renesas_rx_configuration failed");

	zassert_equal(dtc_renesas_rx_start_transfer(dtc, acctivate_dtc), 0,
		      "dtc_renesas_rx_start_transfer failed");

	gpio_pin_set(dev_out, PIN_OUT, 1);
	k_sem_take(&sem, K_FOREVER);
	gpio_pin_set(dev_out, PIN_OUT, 0);

	/* Get trasnfer info */
	zassert_equal(dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info), 0,
		      "dtc_renesas_rx_infoGet failed");

	uint32_t transfers_to_complete = transfer_info.transfer_length_remaining;

	zassert_equal(0,
		      dtc_renesas_rx_reset_transfer(dtc, acctivate_dtc,
						    &src[transfer_length - transfers_to_complete],
						    &dest[transfer_length - transfers_to_complete],
						    transfers_to_complete),
		      "dtc_renesas_rx_reset_transfer failed");

	/* Get trasnfer info */
	while (info->length) {
		/* Trigger a DTC transfer */
		gpio_pin_set(dev_out, PIN_OUT, 1);
		k_sem_take(&sem, K_FOREVER);
		gpio_pin_set(dev_out, PIN_OUT, 0);
	}
	log_src(src, transfer_length);
	log_dest(dest, transfer_length);
	zassert_equal(0, memcmp(dest, src, transfer_length), "Reset transfer failed.");

	return TC_PASS;
}

static int fn_transfer_completed_irq_each(void)
{
	/* Split the global buffers in two so each transfer has its own buffer. */
	uint8_t *src1 = &g_transfer_test_src[0];
	uint8_t *dest1 = &g_transfer_test_dest[0];
	/* Allocate array of transfers */
	transfer_info_t *info = &gc_transfer_test_info;
	transfer_properties_t transfer_info;

	/* Configure the first transfer to repeat mode */
	info->p_src = src1;
	info->p_dest = dest1;
	info->transfer_settings_word_b.irq = TRANSFER_IRQ_EACH;

	gpio_config_trigger_callback(dev_in, dev_out, 1);

	zassert_equal(dtc_renesas_rx_configuration(dtc, acctivate_dtc, info), 0,
		      "dtc_renesas_rx_configuration failed");

	/* Get trasnfer info */
	zassert_equal(dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info), 0,
		      "dtc_renesas_rx_infoGet failed");

	printf("transfer_length_remaining: %d\n", transfer_info.transfer_length_remaining);

	zassert_equal(dtc_renesas_rx_start_transfer(dtc, acctivate_dtc), 0,
		      "dtc_renesas_rx_start_transfer failed");

	uint32_t transfers_to_complete = info->length;

	while (info->length) {
		/* Trigger a DTC transfer */
		gpio_pin_set(dev_out, PIN_OUT, 1);
		k_sem_take(&sem, K_FOREVER);
		gpio_pin_set(dev_out, PIN_OUT, 0);
	}

	zassert_equal(dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info), 0,
		      "dtc_renesas_rx_infoGet failed");

	printf("transfer_length_remaining: %d\n", transfer_info.transfer_length_remaining);

	zassert_equal(memcmp(src1, dest1, transfers_to_complete), 0, "Repeat transfer failed");

	k_sem_give(&sem);

	return TC_PASS;
}

static int fn_transfer_completed_irq_end(void)
{
	const uint16_t transfer_length = 16;
	/* Allocate array of transfers */
	transfer_info_t *info = &gc_transfer_test_info;

	info->length = transfer_length;
	info->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
	info->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
	printk("activation_irq: %d\n", acctivate_dtc);
	gpio_config_trigger_callback(dev_in, dev_out, 1);

	zassert_equal(dtc_renesas_rx_configuration(dtc, acctivate_dtc, info), 0,
		      "dtc_renesas_rx_configuration failed");

	/* Get trasnfer info */
	transfer_properties_t transfer_info;

	zassert_equal(dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info), 0,
		      "dtc_renesas_rx_infoGet failed");

	printf("transfer_length_remaining: %d\n", transfer_info.transfer_length_remaining);

	zassert_equal(dtc_renesas_rx_start_transfer(dtc, acctivate_dtc), 0,
		      "dtc_renesas_rx_start_transfer failed");

	/* Trigger a DTC transfer */
	while (info->length) {
		/* Trigger a DTC transfer */
		gpio_pin_set(dev_out, PIN_OUT, 1);
		gpio_pin_set(dev_out, PIN_OUT, 0);
	}

	k_sem_take(&sem, K_FOREVER);

	/* log src1 and dest1 */
	log_src(g_transfer_test_src, transfer_length);
	log_dest(g_transfer_test_dest, transfer_length);

	zassert_equal(dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info), 0,
		      "dtc_renesas_rx_infoGet failed");

	printf("transfer_length_remaining: %d\n", transfer_info.transfer_length_remaining);

	/* Verify the repeat transfer */
	zassert_equal(memcmp(g_transfer_test_src, g_transfer_test_dest, transfer_length), 0,
		      "Repeat transfer failed");
	/* Verify rising interrupt one time */
	zassert_equal(gv_transfer_complete, 1, "Completed irq end failed.");
	k_sem_give(&sem);
	gpio_pin_set(dev_out, PIN_OUT, 0);

	return TC_PASS;
}

static int fn_chain_transfer_each_repeat_block(void)
{
	/* Number of block transfers that will execute before the chained transfer has completed */
	/* Note this is also the number of repeat transfers that will execute */
	const uint16_t num_transfers = 16;

	/* Number of bytes per block transfer */
	const uint16_t test_length_block_mode = 16;

	/* Number of transfers per repeat*/
	const uint16_t test_length_repeat_mode = 8;

	/* Split the global buffers in two so each transfer has its own buffer. */
	uint8_t *src1 = &g_transfer_test_src[0];
	uint8_t *dest1 = &g_transfer_test_dest[0];
	uint8_t *src2 = &g_transfer_test_src[num_transfers];
	uint8_t *dest2 = &g_transfer_test_dest[num_transfers];

	/* Allocate array of transfers for chain mode. */
	static transfer_info_t info[2] DTC_TRANSFER_INFO_ALIGNMENT;

	/* Configure the first transfer to repeat mode */
	info[0] = gc_transfer_test_info;
	info[0].p_src = src1;
	info[0].p_dest = dest1;
	info[0].length = test_length_repeat_mode;
	info[0].transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_EACH;
	info[0].transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE;
	info[0].transfer_settings_word_b.mode = TRANSFER_MODE_REPEAT;

	/* Configure the second transfer to block mode */
	info[1] = gc_transfer_test_info;
	info[1].p_src = src2;
	info[1].p_dest = dest2;
	info[1].length = test_length_block_mode;
	info[1].num_blocks = num_transfers;
	info[1].transfer_settings_word_b.mode = TRANSFER_MODE_BLOCK;
	info[1].transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE;

	printk("activation_irq: %d\n", acctivate_dtc);
	gpio_config_trigger_callback(dev_in, dev_out, 1);

	zassert_equal(dtc_renesas_rx_configuration(dtc, acctivate_dtc, &info[0]), 0,
		      "dtc_renesas_rx_configuration failed");

	/* Get trasnfer info */
	transfer_properties_t transfer_info;

	zassert_equal(dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info), 0,
		      "dtc_renesas_rx_infoGet failed");

	printf("transfer_length_remaining: %d\n", transfer_info.transfer_length_remaining);

	zassert_equal(dtc_renesas_rx_start_transfer(dtc, acctivate_dtc), 0,
		      "dtc_renesas_rx_start_transfer failed");

	for (int i = 0; i < num_transfers; i++) {
		/* Trigger a DTC transfer */
		gpio_pin_set(dev_out, PIN_OUT, 1);
		k_sem_take(&sem, K_FOREVER);
		gpio_pin_set(dev_out, PIN_OUT, 0);

		/* Both transfers should have been triggered. */

		/* Verify the repeat transfer */
		zassert_equal(src1[i % test_length_repeat_mode], dest1[i]);

		/* Verify the block transfer */
		zassert_equal(memcmp(&src2[0], &dest2[i * test_length_block_mode],
				     test_length_block_mode),
			      0, "Block transfer failed");
	}
	return TC_PASS;
}

static int fn_chain_transfer_end_normal(void)
{
	const uint32_t transfer_length = 16;

	/* Split the global buffers in two so each transfer has its own buffer. */
	uint8_t *src1 = &g_transfer_test_src[0];
	uint8_t *dest1 = &g_transfer_test_dest[0];
	uint8_t *src2 = &g_transfer_test_src[TRANSFER_TEST_SIZE / 2];
	uint8_t *dest2 = &g_transfer_test_dest[TRANSFER_TEST_SIZE / 2];

	/* Allocate array of transfers for chain mode. */
	static transfer_info_t info[2] DTC_TRANSFER_INFO_ALIGNMENT;

	/* Configure the first transfer to repeat mode */
	info[0] = gc_transfer_test_info;
	info[0].p_src = src1;
	info[0].p_dest = dest1;
	info[0].length = 16;
	info[0].transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_END;

	/* Configure the second transfer to block mode */
	info[1] = gc_transfer_test_info;
	info[1].p_src = src2;
	info[1].p_dest = dest2;
	info[1].length = 16;

	printk("activation_irq: %d\n", acctivate_dtc);
	gpio_config_trigger_callback(dev_in, dev_out, 1);

	zassert_equal(dtc_renesas_rx_configuration(dtc, acctivate_dtc, &info[0]), 0,
		      "dtc_renesas_rx_configuration failed");

	/* Get trasnfer info */
	transfer_properties_t transfer_info;

	zassert_equal(dtc_renesas_rx_infoGet(dtc, acctivate_dtc, &transfer_info), 0,
		      "dtc_renesas_rx_infoGet failed");

	printf("transfer_length_remaining: %d\n", transfer_info.transfer_length_remaining);

	zassert_equal(dtc_renesas_rx_start_transfer(dtc, acctivate_dtc), 0,
		      "dtc_renesas_rx_start_transfer failed");

	for (uint32_t i = 0; i < transfer_length; i++) {
		/* Trigger a DTC transfer */
		gpio_pin_set(dev_out, PIN_OUT, 1);
		k_sem_take(&sem, K_FOREVER);
		gpio_pin_set(dev_out, PIN_OUT, 0);

		if (i < transfer_length - 1) {
			/* Make sure that the second transfer info did not execute. */
			zassert_not_equal(src2[0], dest2[0], "src2[0] equal dest2[0]");
		}
	}

	/* Make sure all of info[0] transfers executed. */
	zassert_equal(memcmp(src1, dest1, transfer_length), 0, "Repeat transfer failed");
	/* Make sure that only one transfer of info[1] executed. */
	/* info[1] is triggered by the end of the last transfer of info[0]. */
	zassert_equal(src2[0], dest2[0], "src2[0] not equal to dest2[0]");
	zassert_not_equal(src2[1], dest2[1], "src2[1] equal dest2[1]");

	return TC_PASS;
}

ZTEST(transfer_dtc, test_normal_mode_infoGet)
{
	zassert_equal(fn_normal_mode_infoGet(), TC_PASS, NULL);
}

ZTEST(transfer_dtc, test_repeat_mode_infoGet)
{
	zassert_equal(fn_repeat_mode_infoGet(), TC_PASS, NULL);
}

ZTEST(transfer_dtc, test_block_mode_infoGet)
{
	zassert_equal(fn_block_mode_infoGet(), TC_PASS, NULL);
}

ZTEST(transfer_dtc, test_on_off_module_dtc)
{
	zassert_equal(fn_on_off_module_dtc(), TC_PASS, NULL);
}

ZTEST(transfer_dtc, test_stop_and_reset_transfer)
{
	zassert_equal(fn_stop_and_reset_transfer(), TC_PASS, NULL);
}

ZTEST(transfer_dtc, test_transfer_completed_irq_each)
{
	zassert_equal(fn_transfer_completed_irq_each(), TC_PASS, NULL);
}

ZTEST(transfer_dtc, test_transfer_completed_irq_end)
{
	zassert_equal(fn_transfer_completed_irq_end(), TC_PASS, NULL);
}

ZTEST(transfer_dtc, test_chain_transfer_each_repeat_block)
{
	zassert_equal(fn_chain_transfer_each_repeat_block(), TC_PASS, NULL);
}

ZTEST(transfer_dtc, test_chain_transfer_end_normal)
{
	zassert_equal(fn_chain_transfer_end_normal(), TC_PASS, NULL);
}

ZTEST_SUITE(transfer_dtc, NULL, dtc_setup, dtc_before_fn, NULL, NULL);
