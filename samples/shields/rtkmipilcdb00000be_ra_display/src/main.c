/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>

LOG_MODULE_REGISTER(sample, LOG_LEVEL_INF);

#define DISPLAY_HSIZE        DT_PROP(DT_CHOSEN(zephyr_display), width)
#define DISPLAY_VSIZE        DT_PROP(DT_CHOSEN(zephyr_display), height)
#define DISPLAY_PIXEL_FORMAT DT_PROP(DT_CHOSEN(zephyr_display), pixel_format)
#if (DISPLAY_PIXEL_FORMAT == PANEL_PIXEL_FORMAT_ARGB_8888) ||                                      \
	(DISPLAY_PIXEL_FORMAT == PANEL_PIXEL_FORMAT_RGB_888)
#define BYTE_PER_PIXEL (4)
#elif (DISPLAY_PIXEL_FORMAT == PANEL_PIXEL_FORMAT_RGB_565)
#define BYTE_PER_PIXEL (2)
#endif

uint8_t frame_buffer[DISPLAY_VSIZE * DISPLAY_HSIZE * BYTE_PER_PIXEL] BSP_ALIGN_VARIABLE(64)
	BSP_PLACE_IN_SECTION(".sdram");

static uint32_t get_color(uint8_t num)
{
	uint32_t color = 0;

	switch (num) {
	case 0:
		color = 0x000000FF;
		break;
	case 1:
		color = 0xFF00FF00;
		break;
	case 2:
		color = 0x00FF0000;
		break;
	case 3:
		color = 0x00000000;
		break;
	case 4:
		color = 0xFFFFFFFF;
		break;
	case 5:
		color = 0xFFFFFF00;
		break;
	case 6:
		color = 0xFF00FFFF;
		break;
	case 7:
		color = 0x00FF00FF;
		break;
	default:
		color = 0x00;
		break;
	}

	return color;
}

static void fill_buffer_rgb(uint8_t num, uint32_t *frame, size_t buf_s)
{
	uint32_t color = get_color(num);

	for (size_t idx = 0; idx < buf_s; idx++) {
		frame[idx] = color;
	}
}

int main(void)
{
	size_t x, y, rect_w, rect_h;
	uint32_t bg_color, index = 0;
	const struct device *display_dev;
	struct display_capabilities capabilities;
	struct display_buffer_descriptor buf_desc;
	uint32_t num_elements;
	size_t buf_size = 0;
	uint32_t *gp_frame_buffer = NULL;

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device %s not found. Aborting sample.", display_dev->name);
		return 0;
	}

	LOG_INF("Display sample for %s", display_dev->name);
	display_get_capabilities(display_dev, &capabilities);

	rect_w = capabilities.x_resolution;
	rect_h = capabilities.y_resolution;

	buf_size = rect_w * rect_h;
	num_elements = buf_size;
	switch (capabilities.current_pixel_format) {
	case PIXEL_FORMAT_ARGB_8888:
	case PIXEL_FORMAT_RGB_888:
		bg_color = 0xFFFFFFFFu;
		buf_size *= sizeof(uint32_t);
		break;
	case PIXEL_FORMAT_RGB_565:
		bg_color = 0xFFFFFFFFu;
		buf_size *= sizeof(uint16_t);
		break;
	default:
		LOG_ERR("Unsupported pixel format. Aborting sample.");
		return 0;
	}

	gp_frame_buffer = (uint32_t *)frame_buffer;
	buf_desc.buf_size = buf_size;
	buf_desc.pitch = 1;
	buf_desc.width = rect_w;
	buf_desc.height = rect_h;
	x = 0;
	y = 0;
	memset(gp_frame_buffer, 0x00, buf_size);

	while (1) {
		fill_buffer_rgb(index, gp_frame_buffer, num_elements);
		display_write(display_dev, x, y, &buf_desc, gp_frame_buffer);
		if (index > 7) {
			index = 0;
		} else {
			index++;
		}
		k_sleep(K_MSEC(2000));
	}

	return 0;
}
