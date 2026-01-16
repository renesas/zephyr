#include <zephyr/logging/log.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/video.h>

#include "display.h"
#include "common_util.h"

LOG_MODULE_REGISTER(app_display, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *display_dev;
static lv_img_dsc_t video_img;
static lv_obj_t *screen, *bg, *face_count_label;
static lv_obj_t *bbox_rect[CONFIG_AI_MAX_DETECTION_NUM];

static void draw_ai_bboxes_lvgl(st_ai_detection_point_t *ai_detection, uint16_t max_detection_num)
{
	int ai_w = CONFIG_AI_INPUT_WIDTH;
	int ai_h = CONFIG_AI_INPUT_HEIGHT;
	int ai_display_sides = CONFIG_SCREEN_WIDTH < CONFIG_SCREEN_HEIGHT ? CONFIG_SCREEN_WIDTH
									  : CONFIG_SCREEN_HEIGHT;
	int face_count = 0;

	// Loop over all detected faces and update bounding boxes
	for (int i = 0; i < max_detection_num; i++) {
		signed short x = ai_detection[i].m_x;
		signed short y = ai_detection[i].m_y;
		signed short w = ai_detection[i].m_w;
		signed short h = ai_detection[i].m_h;

		if (w > 0 && h > 0) {
			face_count++;
			int x_ai = (y * ai_display_sides) / ai_w;
			int y_ai = (x * ai_display_sides) / ai_h;
			int w_ai = (h * ai_display_sides) / ai_w;
			int h_ai = (w * ai_display_sides) / ai_h;

			lv_obj_set_pos(bbox_rect[i], ai_display_sides - x_ai - w_ai, y_ai + ((CONFIG_SCREEN_WIDTH - ai_display_sides) / 2));
			lv_obj_set_size(bbox_rect[i], w_ai, h_ai);
		} else {
			lv_obj_set_size(bbox_rect[i], 0, 0);
		}
	}

	lv_label_set_text_fmt(face_count_label, "Detected Faces: %d", face_count);
}

int display_init(void)
{
	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Display device not ready");
		return -1;
	}

	display_blanking_off(display_dev);

	video_img.header.w = CONFIG_VIDEO_WIDTH;
	video_img.header.h = CONFIG_VIDEO_HEIGHT;
	video_img.data_size = CONFIG_VIDEO_WIDTH * CONFIG_VIDEO_HEIGHT * sizeof(lv_color_t);
	video_img.header.cf = LV_COLOR_FORMAT_NATIVE;

	bg = lv_obj_create(lv_scr_act());
	lv_obj_set_size(bg, CONFIG_SCREEN_HEIGHT, CONFIG_SCREEN_WIDTH);
	lv_obj_set_style_bg_color(bg, BACKGROUND_COLOR, 0);
	lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, 0);
	lv_obj_align(bg, LV_ALIGN_TOP_MID, 0, 0);

	lv_task_handler();

	screen = lv_img_create(lv_scr_act());
	lv_img_set_src(screen, &video_img);
	lv_obj_align(screen, LV_ALIGN_CENTER, 0, 0);

	// Create and position the label for the face count at the bottom center of the screen
	face_count_label = lv_label_create(lv_scr_act());
	lv_obj_set_style_bg_color(face_count_label, lv_color_black(), 0);
	lv_obj_set_style_text_color(face_count_label, lv_color_white(), 0);
	lv_obj_set_style_text_font(face_count_label, &lv_font_montserrat_16, 0);
	lv_obj_set_style_bg_opa(face_count_label, LV_OPA_50, 0);
	lv_obj_set_style_pad_all(face_count_label, 6, 0);
	lv_obj_set_style_radius(face_count_label, 6, 0);

	lv_label_set_text(face_count_label, "Faces: 0");
	lv_obj_align(face_count_label, LV_ALIGN_BOTTOM_MID, 0, -10); // 10 pixels above the bottom edge

	for (int i = 0; i < CONFIG_AI_MAX_DETECTION_NUM; i++) {
		bbox_rect[i] = lv_obj_create(lv_scr_act());
		lv_obj_set_size(bbox_rect[i], 0, 0);
		lv_obj_set_style_border_width(bbox_rect[i], 5, 0);
		lv_obj_set_style_border_color(bbox_rect[i], BBOX_COLOR, 0);
		lv_obj_set_style_bg_opa(bbox_rect[i], LV_OPA_TRANSP, 0);
		lv_obj_align(bbox_rect[i], LV_ALIGN_TOP_LEFT, 0, 0);
	}

	LOG_INF("- Display initialized");
	return 0;
}

void display_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg3);

	int err;
	struct k_msgq *display_frame_msgq = (struct k_msgq *)arg1;
	struct k_msgq *ai_result_msgq = (struct k_msgq *)arg2;
	static st_ai_detection_point_t *ai_detection;
	camera_frame_msg_t camera_frame_msg;

	while (1) {
		if (k_msgq_get(display_frame_msgq, &camera_frame_msg, K_FOREVER) != 0) {
			continue;
		}
		video_img.data = (const uint8_t *)camera_frame_msg.vbuf->buffer;

		lv_img_set_src(screen, &video_img);
		lv_img_set_angle(screen, CONFIG_SCREEN_ORIENTATION * 10);

		if (k_msgq_get(ai_result_msgq, &ai_detection, K_NO_WAIT) == 0) {
			draw_ai_bboxes_lvgl(ai_detection, CONFIG_AI_MAX_DETECTION_NUM);
		}

		lv_task_handler();

		err = video_enqueue(camera_frame_msg.video_dev, camera_frame_msg.vbuf);
		if (err) {
			LOG_ERR("Unable to requeue video buf");
		}
	}
}
