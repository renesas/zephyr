#ifndef _DISPLAY_HANDLE_H_
#define _DISPLAY_HANDLE_H_

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BACKGROUND_COLOR lv_color_make(0x00, 0x33, 0x99)
#define BBOX_COLOR       lv_color_make(0x00, 0xFF, 0x00)

int display_init(void);

void display_task(void *, void *, void *);

#ifdef __cplusplus
}
#endif

#endif
