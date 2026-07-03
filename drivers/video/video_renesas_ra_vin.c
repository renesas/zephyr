/*
 * Copyright (c) 2026 Renesas Electronics Co.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_ra_vin

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/dlist.h>
#include <soc.h>
#include <errno.h>

#include "r_vin.h"
#include "r_vin_device_types.h"
#include "r_mipi_csi_contract_types.h"
#include "video_device.h"

LOG_MODULE_REGISTER(renesas_ra_video_vin, CONFIG_VIDEO_LOG_LEVEL);

/*
 * RA8P1 pixel rate limit per format
 * 2 lanes x 720 Mbps = 1440 Mbps total bit bandwidth
 * Max pixel rate = 1440 Mbps / bits-per-pixel
 */
#define RA8P1_LANE_MBPS  720UL
#define RA8P1_NUM_LANES  2UL
#define RA8P1_TOTAL_MBPS (RA8P1_LANE_MBPS * RA8P1_NUM_LANES)

/* VIN has 3 hardware frame buffer slots: MB1, MB2, MB3 */
#define VIN_NUM_MB       3

/* Frame buffer alignment required by VIN */
#define VIN_BUF_ALIGN	64U

struct video_renesas_ra_vin_config {
	void (*irq_config_func)(const struct device *dev);
	const struct device *source_dev;
	const struct device *cam_xclk_dev;
	const struct device *clock_dev;
	const struct device *main_clock_dev;
	const struct clock_control_ra_subsys_cfg clock_subsys;
	const struct pinctrl_dev_config *pincfg;
};

struct video_renesas_ra_vin_data {
	struct st_vin_instance_ctrl *fsp_ctrl;
	struct st_capture_cfg *fsp_cfg;
	struct st_vin_extended_cfg *fsp_extend_cfg;

	/* Current active video format */
	struct video_format fmt;

	/* buffers given by app, waiting to be used */
	struct k_fifo fifo_in;
	/* buffers filled by VIN, ready for app */ 
	struct k_fifo fifo_out;

	atomic_t streaming;

	/* Tracks the Zephyr video_buffer assigned to each VIN slot */
	atomic_ptr_t mb_buf[VIN_NUM_MB];

#ifdef CONFIG_POLL
	struct k_poll_signal *signal;
#endif
};

extern void vin_status_isr(void);
extern void vin_error_isr(void);

static uint32_t video_renesas_ra_max_pixel_rate(uint32_t pixelformat)
{
	uint8_t bpp = video_bits_per_pixel(pixelformat);

	if (bpp == 0) {
		return 0;
	}

	return (uint32_t)((RA8P1_TOTAL_MBPS * 1000000UL) / bpp);
}

/* Pointer to the R_VIN->MBn register for a given slot index */
static inline volatile uint32_t *video_renesas_ra_vin_mb_register(uint8_t slot)
{
	volatile uint32_t *regs[VIN_NUM_MB] = {
		&R_VIN->MB1,
		&R_VIN->MB2,
		&R_VIN->MB3,
	};

	return regs[slot];
}

/* Find which MB slot currently owns a given buffer pointer */
static int video_renesas_ra_find_mb_slot_by_ptr(struct video_renesas_ra_vin_data *data,
						void *buf_ptr)
{
	for (int i = 0; i < VIN_NUM_MB; i++) {
		struct video_buffer *vbuf = (struct video_buffer *)atomic_ptr_get(&data->mb_buf[i]);

		if (vbuf != NULL && vbuf->buffer == buf_ptr) {
			return i;
		}
	}
	return -1;
}

/* Find a free MB slot, or -1 if all are occupied */
static int video_renesas_ra_find_free_mb_slot(struct video_renesas_ra_vin_data *data)
{
	for (int i = 0; i < VIN_NUM_MB; i++) {
		if (atomic_ptr_get(&data->mb_buf[i]) == NULL) {
			return i;
		}
	}
	return -1;
}

/* Assign a buffer to an MB slot */
static void video_renesas_ra_vin_assign_mb_slot(const struct device *dev, uint8_t slot,
						struct video_buffer *vbuf)
{
	struct video_renesas_ra_vin_data *data = dev->data;
	struct st_vin_extended_cfg *p_extend = data->fsp_extend_cfg;

	atomic_ptr_set(&data->mb_buf[slot], vbuf);
	p_extend->output_ctrl.image_buffer[slot] = vbuf->buffer;
	*video_renesas_ra_vin_mb_register(slot) = (uint32_t)(uintptr_t)vbuf->buffer;
}

/* Pixel rate estimation for a candidate frame interval/format change */
static uint32_t video_renesas_ra_estimate_pixel_rate(const struct video_frmival *cur_ival,
						     const struct video_frmival *new_ival,
						     const struct video_format *cur_fmt,
						     const struct video_format *new_fmt,
						     uint32_t cur_pixel_rate)
{
	uint32_t cur_bits =
		cur_fmt->width * cur_fmt->height * video_bits_per_pixel(cur_fmt->pixelformat);
	uint32_t new_bits =
		new_fmt->width * new_fmt->height * video_bits_per_pixel(new_fmt->pixelformat);

	if (new_bits == 0 || new_ival->numerator == 0) {
		return 0;
	}

	uint64_t num =
		(uint64_t)cur_pixel_rate * cur_bits * new_ival->denominator * cur_ival->numerator;
	uint64_t den = (uint64_t)new_bits * new_ival->numerator * cur_ival->denominator;

	return (uint32_t)(num / den);
}

 /* 
  * Validate bandwidth after format or frame interval changes 
  * This function is only ever called while NOT streaming
  */
static int video_renesas_ra_vin_update_settings(const struct device *dev)
{
	const struct video_renesas_ra_vin_config *cfg = dev->config;
	struct video_format fmt;
	struct video_control sensor_rate = {.id = VIDEO_CID_PIXEL_RATE, .val64 = -1};
	int ret;

	ret = video_get_format(cfg->source_dev, &fmt);
	if (ret) {
		LOG_ERR("Cannot get sensor format: %d", ret);
		return ret;
	}

	ret = video_get_ctrl(cfg->source_dev, &sensor_rate);
	if (ret || sensor_rate.val64 <= 0) {
		LOG_ERR("Cannot get sensor pixel rate: %d", ret);
		return ret;
	}

	uint32_t max_rate = video_renesas_ra_max_pixel_rate(fmt.pixelformat);

	if (max_rate == 0 || (uint32_t)sensor_rate.val64 > max_rate) {
		LOG_ERR("Pixel rate %lld exceeds max %u for pixfmt 0x%08x", sensor_rate.val64,
			max_rate, fmt.pixelformat);
		return -ENOTSUP;
	}

	LOG_DBG("VIN settings validated: pixel_rate=%lld bpp=%u", sensor_rate.val64,
		video_bits_per_pixel(fmt.pixelformat));
	return 0;
}

static void video_renesas_ra_vin_callback(capture_callback_args_t *p_args)
{
	const struct device *dev = (const struct device *)p_args->p_context;
	struct video_renesas_ra_vin_data *data = dev->data;

	vin_interrupt_status_t ints = {.mask = p_args->interrupt_status};

	if (!data->streaming) {
		return;
	}

	if (ints.bits.frame_complete && p_args->p_buffer != NULL) {
		int slot = video_renesas_ra_find_mb_slot_by_ptr(data, p_args->p_buffer);

		if (slot >= 0) {
			struct video_buffer *done = data->mb_buf[slot];

			done->bytesused = data->fmt.width * data->fmt.height *
					  video_bits_per_pixel(data->fmt.pixelformat) / 8;
			done->timestamp = k_uptime_get_32();

			k_fifo_put(&data->fifo_out, done);
			data->mb_buf[slot] = NULL;

			/* Refill this slot from the incoming queue, if available */
			struct video_buffer *next = k_fifo_get(&data->fifo_in, K_NO_WAIT);
			if (next != NULL) {
				video_renesas_ra_vin_assign_mb_slot(dev, slot, next);
			} else {
				LOG_DBG("MB%d: no buffer in queue, frame may be dropped", slot + 1);
			}
		} else {
			LOG_DBG("Frame complete for unknown buffer %p", p_args->p_buffer);
		}
	}

	if (ints.bits.fifo_overfow) {
		LOG_ERR("VIN FIFO overflow - clock too slow or bus congested");
	}
	if (ints.bits.axi_err) {
		LOG_ERR("VIN AXI response error - check MB buffer 64-byte alignment");
	}
	if (ints.bits.preclip_h_err) {
		LOG_ERR("VIN horizontal preclip error - sensor width mismatch");
	}
	if (ints.bits.preclip_v_err) {
		LOG_ERR("VIN vertical preclip error - sensor height mismatch");
	}
	if (ints.bits.resp_overflow) {
		LOG_ERR("VIN response overflow");
	}
}

/* Map Zephyr pixel format to MIPI CSI-2 data type ID */
static int video_renesas_ra_pix_fmt_to_mipi_dt(uint32_t pixfmt, mipi_cmd_id_t *dt_out)
{
	uint8_t mipi_dt = video_mipi_data_type(pixfmt);

	/*
	 * video_mipi_data_type() only maps VIDEO_PIX_FMT_UYVY to
	 * VIDEO_MIPI_CSI2_DT_YUV422_8
	 */
	if (pixfmt == VIDEO_PIX_FMT_YUYV) {
		mipi_dt = VIDEO_MIPI_CSI2_DT_YUV422_8;
	}

	switch (mipi_dt) {
	case VIDEO_MIPI_CSI2_DT_YUV422_8:
		*dt_out = MIPI_CMD_ID_PACKED_PIXEL_STREAM_YCBCR16;
		return 0;
	case VIDEO_MIPI_CSI2_DT_RGB565:
		*dt_out = MIPI_CMD_ID_PACKED_PIXEL_STREAM_16;
		return 0;
	case VIDEO_MIPI_CSI2_DT_RGB888:
		*dt_out = MIPI_CMD_ID_PACKED_PIXEL_STREAM_24;
		return 0;
	case VIDEO_MIPI_CSI2_DT_RAW8:
		*dt_out = (mipi_cmd_id_t)0x2A;
		return 0;

	default:
		return -ENOTSUP;
	}
}

/* Zephyr Video API */

static int video_renesas_ra_vin_get_caps(const struct device *dev, struct video_caps *caps)
{
	const struct video_renesas_ra_vin_config *cfg = dev->config;
	int ret;

	ret = video_get_caps(cfg->source_dev, caps);
	if (ret) {
		LOG_ERR("Sensor get_caps failed: %d", ret);
		return ret;
	}

	caps->min_vbuf_count = 2;

	return 0;
}

static int video_renesas_ra_vin_get_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_renesas_ra_vin_config *cfg = dev->config;

	return video_get_format(cfg->source_dev, fmt);
}

static int video_renesas_ra_vin_set_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_renesas_ra_vin_config *cfg = dev->config;
	struct video_renesas_ra_vin_data *data = dev->data;
	mipi_cmd_id_t dt;
	int ret;

	if (atomic_get(&data->streaming)) {
		LOG_ERR("Cannot change format while streaming");
		return -EBUSY;
	}

	/* Validate pixel format has a known MIPI data type */
	ret = video_renesas_ra_pix_fmt_to_mipi_dt(fmt->pixelformat, &dt);
	if (ret < 0) {
		LOG_ERR("Unsupported pixel format 0x%08x", fmt->pixelformat);
		return -ENOTSUP;
	}

	ret = video_set_format(cfg->source_dev, fmt);
	if (ret) {
		LOG_ERR("Sensor set_format failed: %d", ret);
		return ret;
	}

	ret = video_estimate_fmt_size(fmt);
	if (ret < 0) {
		LOG_ERR("Cannot estimate format size: %d", ret);
		return ret;
	}

	data->fmt = *fmt;

	/*
	 * Re-configure D-PHY for new pixel rate
	 * Pixel rate may have changed because format or frame interval changed
	 */
	return video_renesas_ra_vin_update_settings(dev);
}

static int video_renesas_ra_vin_set_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct video_renesas_ra_vin_config *cfg = dev->config;
	struct video_renesas_ra_vin_data *data = dev->data;
	int ret;

	if (atomic_get(&data->streaming)) {
		LOG_ERR("Cannot change frame interval while streaming");
		return -EBUSY;
	}

	ret = video_set_frmival(cfg->source_dev, frmival);
	if (ret) {
		LOG_ERR("Sensor set_frmival failed: %d", ret);
		return ret;
	}

	return video_renesas_ra_vin_update_settings(dev);
}

static int video_renesas_ra_vin_get_frmival(const struct device *dev, struct video_frmival *frmival)
{
	const struct video_renesas_ra_vin_config *cfg = dev->config;

	return video_get_frmival(cfg->source_dev, frmival);
}

static int video_renesas_ra_vin_enum_frmival(const struct device *dev,
					     struct video_frmival_enum *fie)
{
	const struct video_renesas_ra_vin_config *cfg = dev->config;
	struct video_frmival cur_ival;
	struct video_format cur_fmt;
	struct video_control sensor_rate = {.id = VIDEO_CID_PIXEL_RATE, .val64 = -1};
	uint32_t est, max_rate;
	int ret;

	ret = video_enum_frmival(cfg->source_dev, fie);
	if (ret) {
		return ret;
	}

	ret = video_get_frmival(cfg->source_dev, &cur_ival);
	if (ret) {
		return ret;
	}

	ret = video_get_format(cfg->source_dev, &cur_fmt);
	if (ret) {
		return ret;
	}

	ret = video_get_ctrl(cfg->source_dev, &sensor_rate);
	if (ret || sensor_rate.val64 <= 0) {
		return ret;
	}

	max_rate = video_renesas_ra_max_pixel_rate(fie->format->pixelformat);
	if (max_rate == 0) {
		return -ENOTSUP;
	}

	if (fie->type == VIDEO_FRMIVAL_TYPE_DISCRETE) {
		est = video_renesas_ra_estimate_pixel_rate(&cur_ival, &fie->discrete, &cur_fmt,
							   fie->format,
							   (uint32_t)sensor_rate.val64);
		if (est > max_rate) {
			return -EINVAL;
		}
	} else {
		/* stepwise.min = shortest interval = highest fps -> highest rate */
		est = video_renesas_ra_estimate_pixel_rate(&cur_ival, &fie->stepwise.min, &cur_fmt,
							   fie->format,
							   (uint32_t)sensor_rate.val64);
		if (est > max_rate) {
			return -EINVAL;
		}

		/* stepwise.max = longest interval = lowest fps -> lowest rate.
		 * If even this exceeds max_rate (unusual, but guard anyway),
		 * clamp it to the highest fps the format/lane combo allows. */
		est = video_renesas_ra_estimate_pixel_rate(&cur_ival, &fie->stepwise.max, &cur_fmt,
							   fie->format,
							   (uint32_t)sensor_rate.val64);
		if (est > max_rate) {
			uint32_t new_bits = fie->format->width * fie->format->height *
					    video_bits_per_pixel(fie->format->pixelformat);
			uint32_t cur_bits = cur_fmt.width * cur_fmt.height *
					    video_bits_per_pixel(cur_fmt.pixelformat);

			fie->stepwise.max.denominator =
				((uint64_t)new_bits * max_rate * cur_ival.denominator) /
				((uint64_t)cur_bits * sensor_rate.val64 * cur_ival.numerator);
			fie->stepwise.max.numerator = 1;
		}
	}

	return 0;
}

static int video_renesas_ra_vin_enqueue(const struct device *dev, struct video_buffer *vbuf)
{
	struct video_renesas_ra_vin_data *data = dev->data;
	uint32_t required = data->fmt.width * data->fmt.height *
			    video_bits_per_pixel(data->fmt.pixelformat) / 8;

	if (vbuf->size < required) {
		LOG_ERR("Buffer too small: %u bytes, need %u", vbuf->size, required);
		return -ENOMEM;
	}

	if ((uintptr_t)vbuf->buffer % VIN_BUF_ALIGN != 0) {
		LOG_ERR("Buffer %p not %u-byte aligned", vbuf->buffer, VIN_BUF_ALIGN);
		return -EINVAL;
	}

	vbuf->bytesused = required;
	vbuf->line_offset = 0;

	if (atomic_get(&data->streaming)) {
		int slot = video_renesas_ra_find_free_mb_slot(data);

		if (slot >= 0) {
			/*
			 * atomic_ptr_cas(slot, NULL, vbuf):
			 * Only assign if slot is still NULL
			 * Guards against ISR clearing a slot and refilling it
			 * at the same time enqueue() tries to claim it
			 */
			if (atomic_ptr_cas(&data->mb_buf[slot], NULL, vbuf)) {
				struct st_capture_cfg *p_cfg = data->fsp_cfg;
				struct st_vin_extended_cfg *p_extend =
					(struct st_vin_extended_cfg *)p_cfg->p_extend;
				p_extend->output_ctrl.image_buffer[slot] = vbuf->buffer;
				*video_renesas_ra_vin_mb_register(slot) =
					(uint32_t)(uintptr_t)vbuf->buffer;
				return 0;
			}
		}
	}

	k_fifo_put(&data->fifo_in, vbuf);
	return 0;
}

static int video_renesas_ra_vin_dequeue(const struct device *dev, struct video_buffer **vbuf,
					k_timeout_t timeout)
{
	struct video_renesas_ra_vin_data *data = dev->data;

	*vbuf = k_fifo_get(&data->fifo_out, timeout);
	if (*vbuf == NULL) {
		return -EAGAIN;
	}

	return 0;
}

static int video_renesas_ra_vin_set_stream(const struct device *dev, bool enable,
					   enum video_buf_type type)
{
	const struct video_renesas_ra_vin_config *cfg = dev->config;
	struct video_renesas_ra_vin_data *data = dev->data;
	fsp_err_t fsp_ret;
	int ret;

	if (enable) {
		if (atomic_get(&data->streaming)) {
			return -EBUSY;
		}

		if (!device_is_ready(cfg->source_dev)) {
			LOG_ERR("Sensor not ready");
			return -ENODEV;
		}

		/* Fill MB slots from fifo_in */
		for (int i = 0; i < VIN_NUM_MB; i++) {
			if (atomic_ptr_get(&data->mb_buf[i]) == NULL) {
				struct video_buffer *buf = k_fifo_get(&data->fifo_in, K_NO_WAIT);
				if (buf != NULL) {
					video_renesas_ra_vin_assign_mb_slot(dev, i, buf);
				}
			}
		}

		if (atomic_ptr_get(&data->mb_buf[0]) == NULL) {
			LOG_WRN("Starting stream with no buffers enqueued");
		}

		fsp_ret = R_VIN_CaptureStart(data->fsp_ctrl, NULL);
		if (fsp_ret != FSP_SUCCESS) {
			LOG_ERR("VIN captureStart failed: %d", fsp_ret);
			return -EIO;
		}

		/* Sensor start AFTER VIN start */
		ret = video_stream_start(cfg->source_dev, type);
		if (ret) {
			LOG_ERR("Sensor stream_start failed: %d", ret);
			R_VIN_Close(data->fsp_ctrl);
			return ret;
		}

		atomic_set(&data->streaming, 1);
		LOG_INF("VIN streaming started");

	} else {
		/* Stop sensor FIRST
		 * CSI-2 lanes go back to Stop state (LP-11)
		 */
		ret = video_stream_stop(cfg->source_dev, type);
		if (ret) {
			LOG_WRN("Sensor stream_stop returned: %d", ret);
		}

		fsp_ret = R_VIN_Close(data->fsp_ctrl);
		if (fsp_ret != FSP_SUCCESS) {
			LOG_ERR("VIN close failed: %d", fsp_ret);
			return -EIO;
		}

		atomic_set(&data->streaming, 0);

		/*
		 * Return any in-flight (pending) MB buffers to app via fifo_out.
		 * Use atomic_ptr_cas() to safely claim and clear each slot.
		 */
		for (int i = 0; i < VIN_NUM_MB; i++) {
			struct video_buffer *vbuf =
				(struct video_buffer *)atomic_ptr_get(&data->mb_buf[i]);

			if (vbuf && atomic_ptr_cas(&data->mb_buf[i], vbuf, NULL)) {
				vbuf->bytesused = 0;
				k_fifo_put(&data->fifo_out, vbuf);
			}
		}

		LOG_INF("VIN streaming stopped");
	}

	return 0;
}

static int video_renesas_ra_vin_flush(const struct device *dev, bool cancel)
{
	struct video_renesas_ra_vin_data *data = dev->data;
	struct video_buffer *vbuf;

	if (cancel) {
		/* Stop stream if running */
		if (atomic_cas(&data->streaming, 1, 0)) {
			video_renesas_ra_vin_set_stream(dev, false, VIDEO_BUF_TYPE_OUTPUT);
		}

		/* Return any active MB buffers */
		for (int i = 0; i < VIN_NUM_MB; i++) {
			vbuf = (struct video_buffer *)atomic_ptr_get(&data->mb_buf[i]);
			if (vbuf && atomic_ptr_cas(&data->mb_buf[i], vbuf, NULL)) {
				k_fifo_put(&data->fifo_out, vbuf);
			}
		}

		/* Drain fifo_in to fifo_out */
		while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT)) != NULL) {
			k_fifo_put(&data->fifo_out, vbuf);
		}

#ifdef CONFIG_POLL
		if (data->signal) {
			k_poll_signal_raise(data->signal, VIDEO_BUF_ABORTED);
		}
#endif
	} else {
		/* Wait for all enqueued buffers to be consumed by VIN */
		while (!k_fifo_is_empty(&data->fifo_in)) {
			k_sleep(K_MSEC(1));
		}
	}

	return 0;
}

#ifdef CONFIG_POLL
static int video_renesas_ra_vin_set_signal(const struct device *dev, struct k_poll_signal *sig)
{
	struct video_renesas_ra_vin_data *data = dev->data;

	data->signal = sig;
	return 0;
}
#endif

static DEVICE_API(video, video_renesas_ra_vin_driver_api) = {
	.get_caps = video_renesas_ra_vin_get_caps,
	.get_format = video_renesas_ra_vin_get_fmt,
	.set_format = video_renesas_ra_vin_set_fmt,
	.set_stream = video_renesas_ra_vin_set_stream,
	.set_frmival = video_renesas_ra_vin_set_frmival,
	.get_frmival = video_renesas_ra_vin_get_frmival,
	.enum_frmival = video_renesas_ra_vin_enum_frmival,
	.enqueue = video_renesas_ra_vin_enqueue,
	.dequeue = video_renesas_ra_vin_dequeue,
	.flush = video_renesas_ra_vin_flush,
#ifdef CONFIG_POLL
	.set_signal = video_renesas_ra_vin_set_signal,
#endif
};

static int video_renesas_ra_vin_init(const struct device *dev)
{
	const struct video_renesas_ra_vin_config *cfg = dev->config;
	struct video_renesas_ra_vin_data *data = dev->data;
	fsp_err_t fsp_ret;
	int ret;

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to configure pinctrl");
		return ret;
	}

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_DBG("Clock control device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, (clock_control_subsys_t)&cfg->clock_subsys);
	if (ret < 0) {
		LOG_DBG("Failed to enable clock control");
		return ret;
	}

	cfg->irq_config_func(dev);

	fsp_ret = R_VIN_Open(data->fsp_ctrl, data->fsp_cfg);
	if (fsp_ret != FSP_SUCCESS) {
		LOG_ERR("Failed to open VIN");
		return -EIO;
	}

	atomic_clear(&data->streaming);
	atomic_ptr_clear(&data->mb_buf[0]);
	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);

	return 0;
}

#define EVENT_VIN_IRQ(inst) BSP_PRV_IELS_ENUM(CONCAT(EVENT_VIN, _IRQ))
#define EVENT_VIN_ERR(inst) BSP_PRV_IELS_ENUM(CONCAT(EVENT_VIN, _ERR))

#define VIN_CSI_EP(inst)   DT_INST_ENDPOINT_BY_ID(inst, 0, 0)
#define VIN_CSI_NODE(inst) DT_NODE_REMOTE_DEVICE(VIN_CSI_EP(inst))
#define VIN_PHY_NODE(inst) DT_PHANDLE(VIN_CSI_NODE(inst), phys)

#define VIN_CSI_RX_EP(inst)                                                                        \
	DT_CHILD(DT_CHILD(DT_CHILD(VIN_CSI_NODE(inst), ports), port_0), endpoint)
#define VIN_SENSOR_NODE(inst) DT_NODE_REMOTE_DEVICE(VIN_CSI_RX_EP(inst))

#define RENESAS_RA_MIPI_PHYS_DEFINE(n)                                                             \
	static const mipi_phy_timing_t mipi_phy_##n##_timing = {                                   \
		.t_init =                                                                          \
			CLAMP(DT_PROP(DT_CHILD(VIN_PHY_NODE(n), phys_timing), t_init), 0, 0x7FFF), \
		.dphytim2_b =                                                                      \
			{                                                                          \
				.t_clk_prep =                                                      \
					CLAMP(DT_PROP(DT_CHILD(VIN_PHY_NODE(n), phys_timing),      \
						      t_clk_prep),                                 \
					      0, 0xFF),                                            \
				.t_clk_settle =                                                    \
					CLAMP(DT_PROP(DT_CHILD(VIN_PHY_NODE(n), phys_timing),      \
						      t_clk_settle),                               \
					      0, 0xFF),                                            \
				.t_clk_miss =                                                      \
					CLAMP(DT_PROP(DT_CHILD(VIN_PHY_NODE(n), phys_timing),      \
						      t_clk_miss),                                 \
					      0, 0xFF),                                            \
			},                                                                         \
		.dphytim3_b =                                                                      \
			{                                                                          \
				.t_hs_prep = CLAMP(DT_PROP(DT_CHILD(VIN_PHY_NODE(n), phys_timing), \
							   t_hs_prep),                             \
						   0, 0xFF),                                       \
				.t_hs_sett = CLAMP(DT_PROP(DT_CHILD(VIN_PHY_NODE(n), phys_timing), \
							   t_hs_sett),                             \
						   0, 0xFF),                                       \
			},                                                                         \
		.t_lp_exit = CLAMP(DT_PROP(DT_CHILD(VIN_PHY_NODE(n), phys_timing), t_lp_exit), 0,  \
				   0xFF),                                                          \
		.dphytim4_b =                                                                      \
			{                                                                          \
				.t_clk_zero = DT_PROP_BY_IDX(                                      \
					DT_CHILD(VIN_PHY_NODE(n), phys_timing), dphytim4, 0),      \
				.t_clk_pre = DT_PROP_BY_IDX(                                       \
					DT_CHILD(VIN_PHY_NODE(n), phys_timing), dphytim4, 1),      \
				.t_clk_post = DT_PROP_BY_IDX(                                      \
					DT_CHILD(VIN_PHY_NODE(n), phys_timing), dphytim4, 2),      \
				.t_clk_trail = DT_PROP_BY_IDX(                                     \
					DT_CHILD(VIN_PHY_NODE(n), phys_timing), dphytim4, 3),      \
			},                                                                         \
		.dphytim5_b =                                                                      \
			{                                                                          \
				.t_hs_zero = DT_PROP_BY_IDX(                                       \
					DT_CHILD(VIN_PHY_NODE(n), phys_timing), dphytim5, 0),      \
				.t_hs_trail = DT_PROP_BY_IDX(                                      \
					DT_CHILD(VIN_PHY_NODE(n), phys_timing), dphytim5, 1),      \
				.t_hs_exit = DT_PROP_BY_IDX(                                       \
					DT_CHILD(VIN_PHY_NODE(n), phys_timing), dphytim5, 2),      \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	static const mipi_phy_cfg_t mipi_phy_##n##_cfg = {                                         \
		.pll_settings =                                                                    \
			{                                                                          \
				.div = DT_PROP(VIN_PHY_NODE(n), pll_div) - 1,                      \
				.pll_div = DT_ENUM_IDX(VIN_PHY_NODE(n), pll_out_div),              \
				.mul_frac = DT_ENUM_IDX(VIN_PHY_NODE(n), pll_mul_frac),            \
				.mul_int =                                                         \
					CLAMP(DT_PROP(VIN_PHY_NODE(n), pll_mul_int), 20, 180) - 1, \
			},                                                                         \
		.lp_divisor = CLAMP(DT_PROP(VIN_PHY_NODE(n), lp_divisor), 1, 32) - 1,              \
		.p_timing = &mipi_phy_##n##_timing,                                                \
		.dsi_mode = false, /* enable CSI mode, disable DSI mode */                         \
	};                                                                                         \
                                                                                                   \
	mipi_phy_ctrl_t mipi_phy_##n##_ctrl;                                                       \
                                                                                                   \
	static const mipi_phy_instance_t mipi_phy##n = {                                           \
		.p_ctrl = &mipi_phy_##n##_ctrl,                                                    \
		.p_cfg = &mipi_phy_##n##_cfg,                                                      \
		.p_api = &g_mipi_phy,                                                              \
	};

#define RENESAS_RA_MIPI_PHYS_GET(n) &mipi_phy##n

#define RENESAS_RA_MIPI_CSI_DEFINE(n)                                                              \
	static const mipi_csi_cfg_t mipi_csi_##n##_cfg = {                                         \
		.p_mipi_phy_instance = RENESAS_RA_MIPI_PHYS_GET(n),                                \
		.ctrl_data =                                                                       \
			{                                                                          \
				.control_0_bits =                                                  \
					{                                                          \
						.lane_count =                                      \
							DT_PROP_LEN(VIN_CSI_RX_EP(n), data_lanes), \
						.zero_length_packet_output = false,                \
						.err_frame_notify = 1,                             \
						.reserved_packet_reception = 1,                    \
						.generic_rule_mode = 1,                            \
						.ecc_check_24_bits = 1,                            \
						.descramble_enable = 0,                            \
					},                                                         \
				.control_2_bits =                                                  \
					{                                                          \
						.frrclk = 10,                                      \
						.frrskw = 10,                                      \
					},                                                         \
			},                                                                         \
		.option_data.data_type_enable = MIPI_CSI_RX_DATA_ENABLE_YUV422_8_BIT,              \
		.interrupt_cfg =                                                                   \
			{                                                                          \
				.receive_cfg = {.ipl = BSP_IRQ_DISABLED,                           \
						.irq = FSP_INVALID_VECTOR},                        \
				.data_lane_cfg = {.ipl = BSP_IRQ_DISABLED,                         \
						  .irq = FSP_INVALID_VECTOR},                      \
				.virtual_channel_cfg = {.ipl = BSP_IRQ_DISABLED,                   \
							.irq = FSP_INVALID_VECTOR},                \
				.power_management_cfg = {.ipl = BSP_IRQ_DISABLED,                  \
							 .irq = FSP_INVALID_VECTOR},               \
				.short_packet_cfg = {.ipl = BSP_IRQ_DISABLED,                      \
						     .irq = FSP_INVALID_VECTOR},                   \
			},                                                                         \
		.p_callback = NULL,                                                                \
		.p_context = NULL,                                                                 \
	};                                                                                         \
                                                                                                   \
	mipi_csi_instance_ctrl_t mipi_csi_##n##_ctrl;                                              \
                                                                                                   \
	static const mipi_csi_instance_t mipi_csi##n = {                                           \
		.p_ctrl = &mipi_csi_##n##_ctrl,                                                    \
		.p_cfg = &mipi_csi_##n##_cfg,                                                      \
		.p_api = &g_mipi_csi,                                                              \
	};

#define RENESAS_RA_MIPI_CSI_GET(n) &mipi_csi##n

#define VIDEO_RENESAS_RA_VIN_INIT(inst)                                                            \
	static void video_renesas_ra_vin_irq_config_func##inst(const struct device *dev)           \
	{                                                                                          \
		R_ICU->IELSR[DT_INST_IRQ_BY_NAME(inst, irq, irq)] = EVENT_VIN_IRQ(inst);           \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, irq, irq),                                   \
			    DT_INST_IRQ_BY_NAME(inst, irq, priority), vin_status_isr, NULL, 0);    \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, irq, irq));                                   \
		R_ICU->IELSR[DT_INST_IRQ_BY_NAME(inst, err, irq)] = EVENT_VIN_ERR(inst);           \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, err, irq),                                   \
			    DT_INST_IRQ_BY_NAME(inst, err, priority), vin_error_isr, NULL, 0);     \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, err, irq));                                   \
	}                                                                                          \
                                                                                                   \
	static int video_renesas_ra_vin_cam_clock_init##inst(void)                                 \
	{                                                                                          \
		const struct device *dev = DEVICE_DT_INST_GET(inst);                               \
		const struct video_renesas_ra_vin_config *config = dev->config;                    \
		int ret;                                                                           \
                                                                                                   \
		if (!device_is_ready(config->cam_xclk_dev)) {                                      \
			LOG_DBG("Camera clock control device not ready");                          \
			return -ENODEV;                                                            \
		}                                                                                  \
                                                                                                   \
		ret = clock_control_on(config->cam_xclk_dev, (clock_control_subsys_t)0);           \
		if (ret < 0) {                                                                     \
			LOG_DBG("Failed to enable camera clock control");                          \
			return ret;                                                                \
		}                                                                                  \
		return 0;                                                                          \
	}                                                                                          \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	RENESAS_RA_MIPI_PHYS_DEFINE(inst);                                                         \
	RENESAS_RA_MIPI_CSI_DEFINE(inst);                                                          \
                                                                                                   \
	static const struct video_renesas_ra_vin_config video_renesas_ra_vin_config##inst = {      \
		.main_clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(inst, aclk)),          \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(inst, vclk)),               \
		.cam_xclk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(inst, cam_xclk)),        \
		.source_dev = DEVICE_DT_GET(VIN_SENSOR_NODE(inst)),                                \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		.clock_subsys =                                                                    \
			{                                                                          \
				.mstp = DT_INST_CLOCKS_CELL_BY_NAME(inst, vclk, mstp),             \
				.stop_bit = DT_INST_CLOCKS_CELL_BY_NAME(inst, vclk, stop_bit),     \
			},                                                                         \
		.irq_config_func = video_renesas_ra_vin_irq_config_func##inst,                     \
	};                                                                                         \
                                                                                                   \
	static struct st_vin_instance_ctrl video_renesas_ra_vin_fsp_ctrl##inst = {                 \
		.p_context = (void *)DEVICE_DT_INST_GET(inst),                                     \
		.p_callback_memory = NULL,                                                         \
	};                                                                                         \
	static struct st_vin_extended_cfg video_renesas_ra_vin_fsp_extend_cfg##inst = {            \
		.p_mipi_csi_instance = RENESAS_RA_MIPI_CSI_GET(inst),                              \
		.input_ctrl =                                                                      \
			{                                                                          \
				.cfg_bits =                                                        \
					{                                                          \
						.module_enable = 1,                                \
						.color_space_convert_bypass = 0,                   \
						.interlace_mode =                                  \
							VIN_INTERLACE_MODE_ODD_EVEN_FIELD_CAPTURE, \
						.input_mode = DT_INST_PROP(inst, input_format),    \
						.startup = 1,                                      \
						.scaling_enable =                                  \
							DT_INST_PROP(inst, scaling_enable),        \
					},                                                         \
                                                                                                   \
				.preclip =                                                         \
					{                                                          \
						.line_start = DT_INST_PROP(inst, line_start),      \
						.line_end = DT_INST_PROP(inst, line_end),          \
						.pixel_start = DT_INST_PROP(inst, pixel_start),    \
						.pixel_end = DT_INST_PROP(inst, pixel_end),        \
					},                                                         \
                                                                                                   \
				.csi_mode_bits =                                                   \
					{                                                          \
						.virtual_channel =                                 \
							DT_INST_PROP(inst, virtual_channel),       \
						.data_type = DT_INST_PROP(inst, data_type),        \
						.sign_extend_disable = 1,                          \
					},                                                         \
				.csi_detect_bits =                                                 \
					{                                                          \
						.field_detect_enable = 1,                          \
						.even_field_detect_enable = 1,                     \
						.even_field_number = 0,                            \
					},                                                         \
                                                                                                   \
				.image_stride = DT_INST_PROP(inst, image_stride),                  \
			},                                                                         \
                                                                                                   \
		.output_ctrl =                                                                     \
			{                                                                          \
				.image_buffer =                                                    \
					{                                                          \
						[0] = NULL,                                        \
						[1] = NULL,                                        \
						[2] = NULL,                                        \
					},                                                         \
			},                                                                         \
                                                                                                   \
		.conversion_ctrl =                                                                 \
			{                                                                          \
				.data_mode_bits =                                                  \
					{                                                          \
						.data_conversion_mode = VIN_CONVERSION_MODE_NONE,  \
						.alpha_bit_value = 0,                              \
						.output_data_byte_swap = 1,                        \
						.extend_rgb_converted_data = 0,                    \
						.yc_data_transform_enable = 0,                     \
						.yc_transform_mode = VIN_YC_TRANSFORM_MODE_Y_CBCR, \
						.rgb8888_alpha_value = 0xAA,                       \
					},                                                         \
			},                                                                         \
		.conversion_data =                                                                 \
			{                                                                          \
				.uv_address = NULL,                                                \
				.yc_rgb_conversion_setting_1_bits = {.y_mul = 4767},               \
				.yc_rgb_conversion_setting_2_bits = {.csub2 = 2048, .ysub2 = 256}, \
				.yc_rgb_conversion_setting_3_bits = {.cgrmul2 = 3330,              \
								     .rcrmul2 = 6537},             \
				.yc_rgb_conversion_setting_4_bits = {.bcbmul2 = 8261,              \
								     .gcbmul2 = 1605},             \
				.uds_ctrl_bits =                                                   \
					{                                                          \
						.ne_bcb = 1,                                       \
						.ne_gy = 1,                                        \
						.ne_rcr = 1,                                       \
						.pixel_interpolation = 0,                          \
						.bilinear_advanced = 0,                            \
						.scale_up_pixel_count = 1,                         \
					},                                                         \
				.uds_scale_bits.vertical_mask = 4096,                              \
				.uds_scale_bits.horizontal_mask = 4096,                            \
				.uds_bwidth_bits = {.bwidth_v = 64, .bwidth_h = 64},               \
				.uds_clipping_bits = {.cl_vsize = DT_INST_PROP(inst, line_end) +   \
								  1 -                              \
								  DT_INST_PROP(inst, line_start),  \
						      .cl_hsize =                                  \
							      DT_INST_PROP(inst, pixel_end) + 1 -  \
							      DT_INST_PROP(inst, pixel_start)},    \
			},                                                                         \
                                                                                                   \
		.interrupt_cfg =                                                                   \
			{                                                                          \
				.status_enable_bits =                                              \
					{                                                          \
						.end_of_frame = 1,                                 \
						.frame_write_complete = DT_INST_PROP(              \
							inst, frame_write_complete_interrupt),     \
					},                                                         \
				.status =                                                          \
					{                                                          \
						.ipl = DT_INST_IRQ_BY_NAME(inst, irq, priority),   \
						.irq = DT_INST_IRQ_BY_NAME(inst, irq, irq),        \
					},                                                         \
				.error =                                                           \
					{                                                          \
						.ipl = BSP_IRQ_DISABLED,                           \
						.irq = DT_INST_IRQ_BY_NAME(inst, err, irq),        \
					},                                                         \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	static struct st_capture_cfg video_renesas_ra_vin_fsp_cfg##inst = {                        \
		.x_capture_pixels = 0xFFFF,                                                        \
		.y_capture_pixels = 0xFFFF,                                                        \
		.x_capture_start_pixel = 0xFFFF,                                                   \
		.y_capture_start_pixel = 0xFFFF,                                                   \
		.bytes_per_pixel = 0xFF,                                                           \
		.p_extend = &video_renesas_ra_vin_fsp_extend_cfg##inst,                            \
		.p_callback = video_renesas_ra_vin_callback,                                       \
		.p_context = (void *)DEVICE_DT_INST_GET(inst),                                     \
	};                                                                                         \
                                                                                                   \
	static struct video_renesas_ra_vin_data video_renesas_ra_vin_data##inst = {                \
		.fsp_ctrl = &video_renesas_ra_vin_fsp_ctrl##inst,                                  \
		.fsp_cfg = &video_renesas_ra_vin_fsp_cfg##inst,                                    \
		.fsp_extend_cfg = &video_renesas_ra_vin_fsp_extend_cfg##inst,                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &video_renesas_ra_vin_init, NULL,                              \
			      &video_renesas_ra_vin_data##inst,                                    \
			      &video_renesas_ra_vin_config##inst, POST_KERNEL,                     \
			      CONFIG_VIDEO_INIT_PRIORITY, &video_renesas_ra_vin_driver_api);       \
                                                                                                   \
	VIDEO_DEVICE_DEFINE(renesas_ra_vin##inst, DEVICE_DT_INST_GET(inst),                        \
			    DEVICE_DT_GET(VIN_SENSOR_NODE(inst)));                                 \
                                                                                                   \
	SYS_INIT(video_renesas_ra_vin_cam_clock_init##inst, POST_KERNEL,                           \
		 CONFIG_CLOCK_CONTROL_PWM_INIT_PRIORITY);

DT_INST_FOREACH_STATUS_OKAY(VIDEO_RENESAS_RA_VIN_INIT)
