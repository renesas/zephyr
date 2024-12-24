/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <string.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include "flash_renesas_rx.h"
#include "r_flash_rx_if.h"

/*
 * The extern function below is implemented in the r_flash_nofcu.c source file.
 * For more information, please refer to r_flash_nofcu.c in HAL Renesas
 */
extern void Excep_FCU_FRDYI(void);

#define DT_DRV_COMPAT renesas_rx_flash

LOG_MODULE_REGISTER(flash_rx, CONFIG_FLASH_LOG_LEVEL);

#define ERASE_BLOCK_SIZE_0 DT_PROP(DT_INST(0, renesas_rx_nv_flash), erase_block_size)
#define ERASE_BLOCK_SIZE_1 DT_PROP(DT_INST(1, renesas_rx_nv_flash), erase_block_size)
#define IS_FLASH_DF(addr)  IS_EQ(addr, FLASH_DF_BLOCK_0)

BUILD_ASSERT((ERASE_BLOCK_SIZE_0 % FLASH_CF_BLOCK_SIZE) == 0,
	     "erase-block-size expected to be a multiple of a block size");
BUILD_ASSERT((ERASE_BLOCK_SIZE_1 % FLASH_DF_BLOCK_SIZE) == 0,
	     "erase-block-size expected to be a multiple of a block size");

/* Flags, set from Callback function */
static volatile struct flash_rx_event flash_event = {
	.erase_complete = false,
	.write_complete = false,
	.error = false,
};

static struct k_sem transfer_sem;

static void flash_bgo_callback(void *event)
{
	flash_int_cb_args_t *ready_event = event;
	if (FLASH_INT_EVENT_ERASE_COMPLETE == ready_event->event) {
		flash_event.erase_complete = true;
		k_sem_give(&transfer_sem);
	} else if (FLASH_INT_EVENT_WRITE_COMPLETE == ready_event->event) {
		flash_event.write_complete = true;
		k_sem_give(&transfer_sem);
	} else if (FLASH_INT_EVENT_ERR_FAILURE == ready_event->event) {
		k_sem_give(&transfer_sem);
		flash_event.error = true;
	}
}

static bool flash_rx_valid_range(off_t area_size, off_t offset, size_t len)
{
	if ((offset < 0) || (offset >= area_size) || ((area_size - offset) < len)) {
		return false;
	}

	return true;
}

#if CONFIG_FLASH_PAGE_LAYOUT
static void flash_rx_page_layout(const struct device *dev, const struct flash_pages_layout **layout,
				 size_t *layout_size)
{
	struct flash_rx_data *flash_data = dev->data;
	static struct flash_pages_layout flash_rx_layout[1];

	if (flash_data->FlashRegion == DATA_FLASH) {
		/* Flash layout in data flash region */
		flash_rx_layout[0].pages_count = flash_data->area_size / FLASH_DF_BLOCK_SIZE;
		flash_rx_layout[0].pages_size = FLASH_DF_BLOCK_SIZE;
	} else {
		/* Flash layout in code flash region */
		flash_rx_layout[0].pages_count = flash_data->area_size / FLASH_CF_BLOCK_SIZE;
		flash_rx_layout[0].pages_size = FLASH_CF_BLOCK_SIZE;
	}

	*layout = flash_rx_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *flash_rx_get_parameters(const struct device *dev)
{
	const struct flash_rx_config *config = dev->config;

	return &config->flash_rx_parameters;
}

static int flash_rx_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	struct flash_rx_data *flash_data = dev->data;

	if (!flash_rx_valid_range(flash_data->area_size, offset, len)) {
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	LOG_DBG("read 0x%lx, len: %u", (long)(offset + flash_data->area_address),
		(unsigned int)len);

	memcpy(data, (uint8_t *)(offset + flash_data->area_address), len);

	return 0;
}

static int flash_rx_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	struct flash_rx_data *flash_data = dev->data;
	const struct flash_rx_config *config = dev->config;
	flash_err_t err;
	int key = 0, result = 0;

	if (!flash_rx_valid_range(flash_data->area_size, offset, len)) {
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	LOG_DBG("write 0x%lx, len: %u", (long)(offset + flash_data->area_address),
		(unsigned int)len);

	/* Disable interrupts during code flash operations */
	if (flash_data->FlashRegion == CODE_FLASH) {
		key = irq_lock();
	}

	err = R_FLASH_Write((uint32_t)data, (long)(offset + flash_data->area_address), len);
	if (config->bgo_enabled) {
		/* Waitting for the write complete event flag, if BGO is SET */
		k_sem_take(&transfer_sem, K_FOREVER);

		/* The semaphore will be given in callback function */
		if (flash_event.write_complete) {
			flash_event.write_complete = false;
		} else if (flash_event.error) {
			LOG_ERR("write error=%d", (int)err);
			flash_event.error = false;
			result = -EIO;
			goto out;
		}
	} else {
		if (err != FLASH_SUCCESS) {
			result = -EIO;
			goto out;
		}
	}

out:
	if (flash_data->FlashRegion == CODE_FLASH) {
		irq_unlock(key);
	}

	return result;
}

static int flash_rx_erase(const struct device *dev, off_t offset, size_t len)
{
	struct flash_rx_data *flash_data = dev->data;
	const struct flash_rx_config *config = dev->config;
	static struct flash_pages_info page_info_off;
	flash_err_t err;
	uint32_t block_num;
	int ret, key = 0, result = 0;

	if (!flash_rx_valid_range(flash_data->area_size, offset, len)) {
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	/* Get the info of the input offset */
	ret = flash_get_page_info_by_offs(dev, offset, &page_info_off);
	if (ret != 0) {
		return -EINVAL;
	}

	/* offset is expected to be a start of block address */
	if (offset != page_info_off.start_offset) {
		return -EINVAL;
	}

	/* len expected to a multiple of block size */
	/* code and data region have the same block size */
	if ((len % FLASH_DF_BLOCK_SIZE) != 0) {
		return -EINVAL;
	} else {
		/* get the number block to erase */
		block_num = (uint32_t)(len / FLASH_DF_BLOCK_SIZE);
	}

	LOG_DBG("erase 0x%lx, len: %u", (long)(offset + flash_data->area_address),
		(unsigned int)len);

	if (block_num > 0) {

		/* Disable interrupts during code flash operations */
		if (flash_data->FlashRegion == CODE_FLASH) {
			key = irq_lock();
		}

		err = R_FLASH_Erase((long)(flash_data->area_address + offset), block_num);
		if (config->bgo_enabled) {
			/* Waitting for the write complete event flag, if BGO is SET */
			k_sem_take(&transfer_sem, K_FOREVER);

			/* The semaphore will be given in callback function */
			if (flash_event.erase_complete) {
				flash_event.erase_complete = false;
			} else if (flash_event.error) {
				LOG_ERR("erase error=%d", (int)err);
				flash_event.error = false;
				result = -EIO;
				goto out;
			}
		} else {
			/* Using blocking mode, needs to disable BGO configs in both regions */
			if (err != FLASH_SUCCESS) {
				result = -EIO;
				goto out;
			}
		}

out:
		if (flash_data->FlashRegion == CODE_FLASH) {
			irq_unlock(key);
		}

		return result;
	}

	return 0;
}

#define IRQ_FLASH_CONFIG_INIT(index)                                                               \
	IRQ_CONNECT(DT_IRQ_BY_NAME(DT_DRV_INST(index), frdyi, irq),                                \
		    DT_IRQ_BY_NAME(DT_DRV_INST(index), frdyi, priority), Excep_FCU_FRDYI,          \
		    DEVICE_DT_INST_GET(index), 0);                                                 \
                                                                                                   \
	irq_enable(DT_INST_IRQ_BY_NAME(index, frdyi, irq));

static int flash_rx_controller_init(const struct device *dev)
{
	const struct device *dev_ctrl = DEVICE_DT_INST_GET(0);
	struct flash_rx_data *flash_data = dev->data;
	const struct flash_rx_config *config = dev->config;
	flash_err_t err;

	if (!device_is_ready(dev_ctrl)) {
		return -ENODEV;
	}

	if (flash_data->area_address == FLASH_DF_BLOCK_0) {
		flash_data->FlashRegion = DATA_FLASH;
	} else {
		if (!FLASH_CFG_CODE_FLASH_ENABLE) {
			/* Enables code flash configuration before usage */
			return -ENODEV;
		} else {
			flash_data->FlashRegion = CODE_FLASH;
		}
	}

	flash_data->controller = dev_ctrl->data;

	if (config->bgo_enabled) {

		/* Register irq flash configs */
		IRQ_FLASH_CONFIG_INIT(0);

		flash_interrupt_config_t cb_func_info;
		cb_func_info.pcallback = flash_bgo_callback;
		cb_func_info.int_priority = DT_IRQ_BY_NAME(DT_DRV_INST(0), frdyi, priority);

		/* Set callback function */
		err = R_FLASH_Control(FLASH_CMD_SET_BGO_CALLBACK, (void *)&cb_func_info);
		if (err != FLASH_SUCCESS) {
			LOG_DBG("set bgo callback error=%d", (int)err);
			return -EIO;
		}

		/* Init semaphore to use background operations (BGO) */
		k_sem_init(&transfer_sem, 1, 1);
	}

	return 0;
}

static int flash_rx_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	flash_err_t err;

	err = R_FLASH_Open();
	if (err != FLASH_SUCCESS) {
		LOG_DBG("open error=%d", (int)err);
		return -EIO;
	}

	return 0;
}

static DEVICE_API(flash, flash_rx_api) = {
	.erase = flash_rx_erase,
	.write = flash_rx_write,
	.read = flash_rx_read,
	.get_parameters = flash_rx_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_rx_page_layout,
#endif
};

#define FLASH_RX_INIT(index)                                                                       \
	struct flash_rx_data flash_rx_data_##index = {                                             \
		.area_address = DT_REG_ADDR(index),                                                \
		.area_size = DT_REG_SIZE(index),                                                   \
	};                                                                                         \
	static struct flash_rx_config flash_rx_config_##index = {                                  \
		.bgo_enabled = DT_PROP(index, renesas_bgo_enable),                                 \
		.flash_rx_parameters = {                                                           \
			.erase_value = (0xff),                                                     \
			.write_block_size =                                                        \
				(COND_CODE_1(IS_FLASH_DF(DT_REG_ADDR(index)), (1), (4))),          \
				}};                                                                \
	DEVICE_DT_DEFINE(index, flash_rx_controller_init, NULL, &flash_rx_data_##index,            \
			 &flash_rx_config_##index, POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,        \
			 &flash_rx_api);

DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(0), FLASH_RX_INIT);

/* define the flash controller device just to run the init. */
DEVICE_DT_DEFINE(DT_DRV_INST(0), flash_rx_init, NULL, NULL, NULL, PRE_KERNEL_1,
		 CONFIG_FLASH_INIT_PRIORITY, NULL);
