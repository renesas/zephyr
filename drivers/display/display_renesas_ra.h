/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_DISPLAY_RENESAS_RA_H_
#define ZEPHYR_DRIVERS_DISPLAY_RENESAS_RA_H_

#include <zephyr/drivers/display.h>

#define H_TOTAL_CYCLE (559)
#define V_TOTAL_CYCLE (894)
#define FORMAT_PIXEL  DT_INST_PROP(0, pixel_format)

#if (FORMAT_PIXEL == PANEL_PIXEL_FORMAT_RGB_565)
#define BYTE_PER_PIXEL                (2)
#define DISPLAY_OUT_FORMAT            DISPLAY_OUT_FORMAT_16BITS_RGB565
#define DISPLAY_INPUT_FORMAT          DISPLAY_IN_FORMAT_16BITS_RGB565
#define DISPLAY_BITS_PER_PIXEL_INPUT0 (16)
typedef uint16_t pixel_t;
#elif (FORMAT_PIXEL == PANEL_PIXEL_FORMAT_RGB_888)
#define BYTE_PER_PIXEL                (4)
#define DISPLAY_OUT_FORMAT            DISPLAY_OUT_FORMAT_24BITS_RGB888
#define DISPLAY_INPUT_FORMAT          DISPLAY_IN_FORMAT_32BITS_RGB888
#define DISPLAY_BITS_PER_PIXEL_INPUT0 (32)
typedef uint32_t pixel_t;
#elif (FORMAT_PIXEL == PANEL_PIXEL_FORMAT_ARGB_8888)
#define BYTE_PER_PIXEL                (4)
#define DISPLAY_OUT_FORMAT            DISPLAY_OUT_FORMAT_24BITS_RGB888
#define DISPLAY_INPUT_FORMAT          DISPLAY_IN_FORMAT_32BITS_ARGB8888
#define DISPLAY_BITS_PER_PIXEL_INPUT0 (32)
typedef uint32_t pixel_t;
#endif
#define DISPLAY_BITS_PER_PIXEL_INPUT1 (16)
#define DISPLAY_HSIZE                 DT_INST_PROP(0, width)
#define DISPLAY_VSIZE                 DT_INST_PROP(0, height)
#define DISPLAY_BUFFER_STRIDE_BYTES_INPUT0                                                         \
	(((DISPLAY_HSIZE * DISPLAY_BITS_PER_PIXEL_INPUT0 + 0x1FF) >> 9) << 6)
#define DISPLAY_BUFFER_STRIDE_PIXELS_INPUT0                                                        \
	((DISPLAY_BUFFER_STRIDE_BYTES_INPUT0 * 8) / DISPLAY_BITS_PER_PIXEL_INPUT0)
#define DISPLAY_BUFFER_STRIDE_BYTES_INPUT1                                                         \
	(((DISPLAY_HSIZE * DISPLAY_BITS_PER_PIXEL_INPUT1 + 0x1FF) >> 9) << 6)
#define DISPLAY_BUFFER_STRIDE_PIXELS_INPUT1                                                        \
	((DISPLAY_BUFFER_STRIDE_BYTES_INPUT1 * 8) / DISPLAY_BITS_PER_PIXEL_INPUT1)

#define LAYER_GREEN          (255)
#define LAYER_RED            (255)
#define LAYER_BLUE           (255)
#define LAYER_ALPHA          (255)
#define OUTPUT_GREEN         (255)
#define OUTPUT_RED           (255)
#define OUTPUT_BLUE          (255)
#define OUTPUT_ALPHA         (255)
#define GLCDC_BRIGHTNESS_MAX (1023U)
#define BRIGHTNESS_MAX       (255U)
#define GLCDC_CONTRAST_MAX   (255U)

#endif /* ZEPHYR_DRIVERS_DISPLAY_RENESAS_RA_H_ */
