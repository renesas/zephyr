/*
 * SPDX-FileCopyrightText: Copyright 2025 - 2026 NXP
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/sys/byteorder.h>
#include "usbh_class.h"
#include "usbh_device.h"
#include "usbh_desc.h"
#include "usbh_ch9.h"
#include "usbh_ch11.h"
#include "usbh_hub.h"

LOG_MODULE_REGISTER(usbh_hub, CONFIG_USBH_HUB_LOG_LEVEL);

/* Time the port reset signalling is driven, the specification requires
 * 10-20ms, default to 20ms for the worst case scenario.
 */
#define HUB_PORT_RESET_DELAY_MS 20

static struct {
	uint8_t total_hubs;
	sys_slist_t hub_list;
	struct k_mutex lock;
} hub_mgr;

/*
 * All hubs share a single workqueue thread. Hub status handling and downstream
 * device enumeration run to completion on it, which serializes enumeration
 * across the whole topology so that only one device at a time is in the
 * default state and answers to address 0.
 */
static K_KERNEL_STACK_DEFINE(hub_stack, CONFIG_USBH_HUB_STACK_SIZE);
static struct k_work_q hub_work_q;

static int hub_interrupt_in_cb(struct usb_device *const dev,
			       struct uhc_transfer *const xfer);

static int hub_prepare_interrupt_xfer(struct usbh_hub_data *hub_data,
				      struct uhc_transfer **xfer_out)
{
	struct uhc_transfer *xfer;

	if (hub_data->int_ep == NULL) {
		LOG_ERR("No interrupt endpoint available");
		return -ENODEV;
	}

	xfer = usbh_xfer_alloc(hub_data->udev,
			       hub_data->int_ep->bEndpointAddress,
			       hub_interrupt_in_cb,
			       (void *)hub_data);
	if (xfer == NULL) {
		LOG_ERR("Failed to allocate interrupt transfer");
		return -ENOMEM;
	}

	*xfer_out = xfer;

	return 0;
}

static int hub_enqueue_interrupt(struct usbh_hub_data *hub_data,
				 struct uhc_transfer *xfer)
{
	struct net_buf *buf;
	int ret;

	buf = usbh_xfer_buf_alloc(hub_data->udev,
				  sys_le16_to_cpu(hub_data->int_ep->wMaxPacketSize));
	if (buf == NULL) {
		LOG_ERR("Failed to allocate interrupt buffer");
		usbh_xfer_free(hub_data->udev, xfer);
		return -ENOMEM;
	}

	xfer->buf = buf;

	ret = usbh_xfer_enqueue(hub_data->udev, xfer);
	if (ret != 0) {
		LOG_ERR("Failed to enqueue interrupt transfer: %d", ret);
		usbh_xfer_buf_free(hub_data->udev, buf);
		usbh_xfer_free(hub_data->udev, xfer);
		return ret;
	}

	k_mutex_lock(&hub_data->lock, K_FOREVER);
	hub_data->int_active = true;
	k_mutex_unlock(&hub_data->lock);

	return 0;
}

static int hub_start_interrupt(struct usbh_hub_data *hub_data)
{
	struct uhc_transfer *xfer;
	int ret;

	k_mutex_lock(&hub_data->lock, K_FOREVER);
	if (!hub_data->connected || hub_data->int_active) {
		k_mutex_unlock(&hub_data->lock);
		return -EINVAL;
	}

	if (hub_data->state != HUB_STATE_OPERATIONAL) {
		k_mutex_unlock(&hub_data->lock);
		return -ENOENT;
	}
	k_mutex_unlock(&hub_data->lock);

	ret = hub_prepare_interrupt_xfer(hub_data, &xfer);
	if (ret != 0) {
		return ret;
	}

	hub_data->interrupt_transfer = xfer;

	ret = hub_enqueue_interrupt(hub_data, xfer);
	if (ret != 0) {
		hub_data->interrupt_transfer = NULL;
	}

	return ret;
}

static struct usbh_hub_data *const find_hub_by_udev(const struct usb_device *const udev)
{
	struct usbh_hub_data *hub_data;

	k_mutex_lock(&hub_mgr.lock, K_FOREVER);

	SYS_SLIST_FOR_EACH_CONTAINER(&hub_mgr.hub_list, hub_data, node) {
		if (hub_data->udev == udev) {
			k_mutex_unlock(&hub_mgr.lock);
			return hub_data;
		}
	}

	k_mutex_unlock(&hub_mgr.lock);

	return NULL;
}

static int hub_interrupt_in_cb(struct usb_device *const dev,
			       struct uhc_transfer *const xfer)
{
	struct usbh_hub_data *const hub_data = (void *)xfer->priv;
	struct net_buf *buf = xfer->buf;
	int ret = 0;

	k_mutex_lock(&hub_data->lock, K_FOREVER);

	if (!hub_data->connected) {
		k_mutex_unlock(&hub_data->lock);
		goto cleanup_and_exit;
	}

	hub_data->int_active = false;

	if (buf == NULL || buf->len == 0) {
		LOG_ERR("Hub level %d interrupt transfer failed or no data",
			hub_data->udev->level);
		k_mutex_unlock(&hub_data->lock);
		goto resubmit;
	}

	memcpy(hub_data->int_buffer, buf->data,
	       MIN(buf->len, sizeof(hub_data->int_buffer)));

	LOG_DBG("Hub level %d interrupt data received: length=%d",
		hub_data->udev->level,
		buf->len);

	k_mutex_unlock(&hub_data->lock);

	k_work_submit_to_queue(&hub_work_q, &hub_data->hub_work);

	net_buf_unref(buf);
	usbh_xfer_free(hub_data->udev, xfer);
	return 0;

resubmit:
	k_mutex_lock(&hub_data->lock, K_FOREVER);
	if (hub_data->connected && hub_data->state == HUB_STATE_OPERATIONAL) {
		k_mutex_unlock(&hub_data->lock);

		ret = hub_enqueue_interrupt(hub_data, xfer);
		if (ret != 0) {
			LOG_ERR("Failed to resubmit interrupt transfer: %d", ret);
		}
	} else {
		k_mutex_unlock(&hub_data->lock);
		usbh_xfer_free(hub_data->udev, xfer);
	}

	return 0;

cleanup_and_exit:
	if (buf != NULL) {
		net_buf_unref(buf);
	}
	usbh_xfer_free(dev, xfer);

	return 0;
}

static void hub_log_info(struct usbh_hub_data *const hub_data)
{
	const struct usb_device_descriptor *const dev_desc =
					&hub_data->udev->dev_desc;
	struct usb_device *udev = hub_data->udev;

	LOG_INF("=== USB Hub Information ===");
	LOG_INF("Hub Level: %d", udev->level);
	LOG_INF("Vendor ID: 0x%04x", sys_le16_to_cpu(dev_desc->idVendor));
	LOG_INF("Product ID: 0x%04x", sys_le16_to_cpu(dev_desc->idProduct));
	LOG_INF("Device Address: %d", udev->addr);
	if (udev->hub) {
		LOG_INF("Parent Hub Level: %d, Port: %d", udev->hub->level,
			udev->hub_port);
	} else {
		LOG_INF("Root Hub (no parent)");
	}
	LOG_INF("===========================");
}

static void hub_recursive_disconnect(struct usbh_hub_data *const hub_data)
{
	struct usb_device *port_udev;

	LOG_DBG("Recursively disconnecting Hub level %d and all children",
		hub_data->udev->level);

	k_work_cancel(&hub_data->hub_work);

	k_mutex_lock(&hub_data->lock, K_FOREVER);
	hub_data->int_active = false;
	k_mutex_unlock(&hub_data->lock);

	for (uint8_t i = 0; i < hub_data->port_count; i++) {
		if (hub_data->port_list) {
			hub_data->port_list[i].enum_pending = false;
			if (hub_data->port_list[i].udev != NULL) {
				port_udev = hub_data->port_list[i].udev;
				hub_data->port_list[i].udev = NULL;
				hub_data->port_list[i].state = PORT_STATE_NOT_CONFIGURED;
				usbh_device_disconnect(hub_data->uhs_ctx, port_udev);
			}
		}
	}

	if (hub_data->connected) {
		LOG_DBG("Triggering Hub level %d removal", hub_data->udev->level);
		usbh_device_disconnect(hub_data->uhs_ctx, hub_data->udev);
	}
}

static int enumerate_port_device(struct usbh_hub_data *hub_data,
				 struct usb_hub_port *port_instance,
				 uint8_t port_num,
				 struct usb_hub_port_status *port_sts)
{
	struct usb_device *udev;
	uint8_t speed;

	if ((port_sts->wPortStatus & USB_HUB_PORT_STATUS_HIGH_SPEED) != 0) {
		speed = USB_PORT_SPEED_HS;
	} else if ((port_sts->wPortStatus & USB_HUB_PORT_STATUS_LOW_SPEED) != 0) {
		speed = USB_PORT_SPEED_LS;
	} else {
		speed = USB_PORT_SPEED_FS;
	}

	LOG_INF("Device ready on port %d (speed: %s)", port_num,
		speed == USB_PORT_SPEED_HS ? "HIGH" :
		speed == USB_PORT_SPEED_LS ? "LOW" : "FULL");

	udev = usbh_device_alloc(hub_data->uhs_ctx);
	if (udev == NULL) {
		LOG_ERR("Device enumeration failed for port %d", port_num);
		return -ENOMEM;
	}

	udev->hub_port = port_num;
	udev->speed = speed;
	udev->level = hub_data->udev->level + 1;
	udev->hub = hub_data->udev;

	usbh_device_connect(hub_data->uhs_ctx, udev);
	LOG_DBG("Device enumeration completed for port %d, addr=%d", port_num, udev->addr);

	port_instance->udev = udev;

	return 0;
}

static void hub_handle_hub_change(struct usbh_hub_data *const hub_data)
{
	struct usb_hub_status *const hub_sts = &hub_data->status;
	uint16_t hub_status;
	uint16_t hub_change;
	int ret;

	ret = usbh_req_get_hub_status(hub_data->udev, &hub_status, &hub_change);
	if (ret != 0) {
		LOG_ERR("Failed to get hub status: %d", ret);
		return;
	}

	hub_sts->wHubStatus = hub_status;
	hub_sts->wHubChange = hub_change;

	LOG_DBG("Hub status: 0x%04x, change: 0x%04x", hub_sts->wHubStatus, hub_sts->wHubChange);

	if ((hub_sts->wHubChange & USB_HUB_CHANGE_LOCAL_POWER) != 0) {
		LOG_WRN("Hub local power status changed");
		ret = usbh_req_clear_hcfs_c_hub_local_power(hub_data->udev);
		if (ret != 0) {
			LOG_ERR("Failed to clear hub local power feature: %d", ret);
		}
	}

	if ((hub_sts->wHubChange & USB_HUB_CHANGE_OVER_CURRENT) != 0) {
		LOG_ERR("Hub over-current detected!");
		ret = usbh_req_clear_hcfs_c_hub_over_current(hub_data->udev);
		if (ret != 0) {
			LOG_ERR("Failed to clear hub over-current feature: %d", ret);
		}
	}
}

/*
 * Phase 1: fully drain a port's status change using hub class requests only
 * (GET_PORT_STATUS / CLEAR_FEATURE). This decides whether the port needs to
 * be reset and enumerated, but never issues a general USB request itself -
 * that happens later, one port at a time, in hub_port_process().
 */
static void hub_port_handle_change(struct usbh_hub_data *const hub_data,
				   struct usb_hub_port *const port_instance)
{
	struct usb_hub_port_status port_sts;
	struct usb_device *udev;
	const uint8_t port_num = port_instance->num;
	uint16_t port_status;
	uint16_t port_change;
	int ret;

	ret = usbh_req_get_port_status(hub_data->udev, port_num, &port_status, &port_change);
	if (ret != 0) {
		LOG_ERR("Failed to get port status: %d", ret);
		return;
	}

	port_sts.wPortStatus = port_status;
	port_sts.wPortChange = port_change;
	LOG_DBG("Port %d status: wPortStatus=0x%04x, wPortChange=0x%04x", port_num,
		port_sts.wPortStatus, port_sts.wPortChange);

	if ((port_sts.wPortChange & USB_HUB_PORT_CHANGE_CONNECTION) != 0) {
		ret = usbh_req_clear_hcfs_c_pconnection(hub_data->udev, port_num);
		if (ret != 0) {
			LOG_ERR("Failed to clear port connection change: %d", ret);
		}
		LOG_DBG("Port %d: Cleared connection change bit", port_num);

		/* Re-read to get the settled state after the connection change. */
		ret = usbh_req_get_port_status(hub_data->udev, port_num,
					       &port_status, &port_change);
		if (ret != 0) {
			LOG_ERR("Failed to get port connection status: %d", ret);
			return;
		}

		port_sts.wPortStatus = port_status;
		port_sts.wPortChange = port_change;
	}

	if ((port_sts.wPortChange & USB_HUB_PORT_CHANGE_ENABLE) != 0) {
		ret = usbh_req_clear_hcfs_c_penable(hub_data->udev, port_num);
		if (ret != 0) {
			LOG_ERR("Failed to clear enable change: %d", ret);
		}
	}

	if ((port_sts.wPortChange & USB_HUB_PORT_CHANGE_SUSPEND) != 0) {
		LOG_DBG("Port %d suspend change detected", port_num);
		ret = usbh_req_clear_hcfs_c_psuspend(hub_data->udev, port_num);
		if (ret != 0) {
			LOG_ERR("Failed to clear suspend change: %d", ret);
		}
	}

	if ((port_sts.wPortChange & USB_HUB_PORT_CHANGE_RESET) != 0) {
		LOG_DBG("Port %d reset change detected", port_num);
		ret = usbh_req_clear_hcfs_c_preset(hub_data->udev, port_num);
		if (ret != 0) {
			LOG_ERR("Failed to clear reset change: %d", ret);
		}
	}

	if ((port_sts.wPortChange & USB_HUB_PORT_CHANGE_OVER_CURRENT) != 0) {
		LOG_WRN("Port %d over-current detected", port_num);
		ret = usbh_req_clear_hcfs_c_pover_current(hub_data->udev, port_num);
		if (ret != 0) {
			LOG_ERR("Failed to clear over-current change: %d", ret);
		}
		port_instance->state = PORT_STATE_DISABLED;
		port_instance->enum_pending = false;
		return;
	}

	if ((port_sts.wPortStatus & USB_HUB_PORT_STATUS_CONNECTION) == 0) {
		LOG_DBG("Port %d disconnected", port_num);
		if (port_instance->udev != NULL) {
			udev = port_instance->udev;
			port_instance->udev = NULL;
			usbh_device_disconnect(hub_data->uhs_ctx, udev);
		}
		port_instance->state = PORT_STATE_DISCONNECTED;
		port_instance->enum_pending = false;
		return;
	}

	if (port_instance->udev != NULL &&
	    (port_sts.wPortStatus & USB_HUB_PORT_STATUS_ENABLE) != 0) {
		LOG_DBG("Port %d device still active", port_num);
		port_instance->state = PORT_STATE_ENABLED;
		port_instance->enum_pending = false;
		return;
	}

	if (port_instance->udev != NULL) {
		/* Port dropped out of the enabled state; tear down and re-enumerate. */
		udev = port_instance->udev;
		port_instance->udev = NULL;
		usbh_device_disconnect(hub_data->uhs_ctx, udev);
	}

	LOG_INF("Device connected to port %d, scheduling reset and enumeration", port_num);
	port_instance->state = PORT_STATE_DISABLED;
	port_instance->reset_count = CONFIG_USBH_HUB_PORT_RESET_TIMES;
	port_instance->enum_pending = true;
}

/*
 * Phase 2: reset and enumerate a single downstream device. Entered only for
 * ports flagged PORT_STATE_DISABLED + enum_pending by hub_port_handle_change().
 * Mixes hub class requests (reset) with the general USB requests issued by
 * enumerate_port_device(), and runs to completion for one port before the
 * next port is attempted, so at most one device is ever in the default state.
 */
static int hub_port_process(struct usbh_hub_data *const hub_data,
			    struct usb_hub_port *const port_instance)
{
	struct usb_hub_port_status port_sts;
	struct usb_device *udev;
	const uint8_t port_num = port_instance->num;
	uint16_t port_status;
	uint16_t port_change;
	int ret;

	LOG_DBG("Port %d: starting reset/enumeration", port_num);

	while (hub_data->connected) {
		switch (port_instance->state) {
		case PORT_STATE_DISABLED:
			if (port_instance->reset_count == 0) {
				LOG_ERR("Port %d reset retry count exhausted, giving up",
					port_num);
				return -EIO;
			}

			ret = usbh_req_set_hcfs_prst(hub_data->udev, port_num);
			if (ret != 0) {
				LOG_ERR("Failed to reset port: %d", ret);
				goto error_recovery;
			}

			port_instance->reset_count--;
			port_instance->state = PORT_STATE_RESETTING;
			k_sleep(K_MSEC(HUB_PORT_RESET_DELAY_MS));
			continue;

		case PORT_STATE_RESETTING:
			ret = usbh_req_get_port_status(hub_data->udev, port_num,
						       &port_status, &port_change);
			if (ret != 0) {
				LOG_ERR("Failed to get port status for reset check: %d", ret);
				goto error_recovery;
			}

			port_sts.wPortStatus = port_status;
			port_sts.wPortChange = port_change;

			/* reset is not completed */
			if ((port_sts.wPortChange & USB_HUB_PORT_CHANGE_RESET) == 0) {
				if (port_instance->reset_count == 0) {
					LOG_ERR("Port %d reset max retries exceeded", port_num);
					return -EIO;
				}

				LOG_WRN("Port %d reset timeout, retrying (%d left)",
					port_num, port_instance->reset_count);
				port_instance->state = PORT_STATE_DISABLED;
				k_sleep(K_MSEC(CONFIG_USBH_HUB_ENUM_RETRY_DELAY_MS));
				continue;
			}

			LOG_DBG("Port %d reset completed", port_num);

			ret = usbh_req_clear_hcfs_c_preset(hub_data->udev, port_num);
			if (ret != 0) {
				LOG_ERR("Failed to clear reset feature: %d", ret);
				goto error_recovery;
			}
			LOG_DBG("Port %d: Cleared reset change bit", port_num);

			ret = usbh_req_get_port_status(hub_data->udev, port_num,
						       &port_status, &port_change);
			if (ret != 0) {
				LOG_ERR("Failed to get port status after reset: %d", ret);
				goto error_recovery;
			}

			port_sts.wPortStatus = port_status;
			port_sts.wPortChange = port_change;

			if ((port_sts.wPortStatus & USB_HUB_PORT_STATUS_CONNECTION) == 0) {
				LOG_WRN("Port %d device disconnected during reset", port_num);
				port_instance->state = PORT_STATE_DISCONNECTED;
				return -ENODEV;
			}

			if ((port_sts.wPortStatus & USB_HUB_PORT_STATUS_ENABLE) == 0) {
				if (port_instance->reset_count == 0) {
					LOG_ERR("Port %d max reset retries exceeded", port_num);
					return -EIO;
				}

				LOG_WRN("Port %d not enabled after reset, retrying (%d left)",
					port_num, port_instance->reset_count);
				port_instance->state = PORT_STATE_DISABLED;
				k_sleep(K_MSEC(CONFIG_USBH_HUB_ENUM_RETRY_DELAY_MS));
				continue;
			}

			/* Sleep 10ms for downstream device stable after reset */
			k_msleep(10);

			ret = enumerate_port_device(hub_data, port_instance, port_num,
						    &port_sts);
			if (ret != 0) {
				if (port_instance->reset_count == 0) {
					LOG_ERR("Port %d enumeration max retries exceeded",
						port_num);
					return ret;
				}

				LOG_WRN("Port %d enumeration failed, retrying reset (%d left)",
					port_num, port_instance->reset_count);
				port_instance->state = PORT_STATE_DISABLED;
				k_sleep(K_MSEC(CONFIG_USBH_HUB_ENUM_RETRY_DELAY_MS));
				continue;
			}

			port_instance->state = PORT_STATE_ENABLED;
			port_instance->reset_count = CONFIG_USBH_HUB_PORT_RESET_TIMES;
			return 0;

		default:
			LOG_ERR("Port %d in unsupported state: %d", port_num,
				port_instance->state);
			return -EINVAL;
		}

error_recovery:
		if (port_instance->reset_count == 0) {
			LOG_ERR("Port %d max retries exceeded, moving to disabled", port_num);
			port_instance->state = PORT_STATE_DISABLED;
			return -EIO;
		}

		port_instance->reset_count--;
		if (port_instance->udev != NULL) {
			udev = port_instance->udev;
			port_instance->udev = NULL;
			usbh_device_disconnect(hub_data->uhs_ctx, udev);
		}

		LOG_WRN("Port %d error recovery, %d retries left", port_num,
			port_instance->reset_count);
		k_sleep(K_MSEC(CONFIG_USBH_HUB_ERROR_RECOVERY_DELAY_MS));

		ret = usbh_req_get_port_status(hub_data->udev, port_num,
					       &port_status, &port_change);
		if (ret != 0 || (port_status & USB_HUB_PORT_STATUS_CONNECTION) == 0) {
			port_instance->state = PORT_STATE_DISCONNECTED;
			return -ENODEV;
		}

		port_instance->state = PORT_STATE_DISABLED;
	}

	return -ENODEV;
}

/*
 * Handle one interrupt IN report: drain every pending hub/port status change
 * with hub class requests (phase 1), then reset and enumerate the downstream
 * devices flagged by that pass, one at a time, with general USB requests
 * (phase 2). Keeping the phases separate guarantees every port's change bits
 * are cleared before any device is put through reset, and that only a single
 * device is ever in the default state answering to address 0.
 */
static void hub_process_status_events(struct usbh_hub_data *const hub_data)
{
	uint8_t port_index;
	int ret;

	k_mutex_lock(&hub_data->lock, K_FOREVER);

	if (!hub_data->connected) {
		k_mutex_unlock(&hub_data->lock);
		return;
	}

	for (port_index = 0; port_index <= hub_data->port_count; ++port_index) {
		if (((0x01U << (port_index & 0x07U)) &
		     (hub_data->int_buffer[port_index >> 3U])) == 0) {
			continue;
		}

		if (port_index == 0) {
			LOG_INF("Hub level %d status changed, processing",
				hub_data->udev->level);
			hub_handle_hub_change(hub_data);
		} else {
			LOG_DBG("Hub level %d port %d status changed, processing",
				hub_data->udev->level, port_index);
			hub_port_handle_change(hub_data, &hub_data->port_list[port_index - 1]);
		}
	}

	memset(hub_data->int_buffer, 0, sizeof(hub_data->int_buffer));

	k_mutex_unlock(&hub_data->lock);

	for (port_index = 0; port_index < hub_data->port_count; ++port_index) {
		struct usb_hub_port *const port_instance = &hub_data->port_list[port_index];

		if (!port_instance->enum_pending) {
			continue;
		}

		port_instance->enum_pending = false;

		if (!hub_data->connected) {
			break;
		}

		ret = hub_port_process(hub_data, port_instance);
		if (ret != 0) {
			LOG_DBG("Port %d enumeration finished with %d", port_instance->num, ret);
		}
	}
}

static void hub_process(struct k_work *work)
{
	struct usbh_hub_data *hub_data =
		CONTAINER_OF(work, struct usbh_hub_data, hub_work);
	struct usbh_hub_data *parent_hub;
	struct usb_device *udev;
	uint16_t total_hub_desc_len = 0;
	int ret;

	k_mutex_lock(&hub_data->lock, K_FOREVER);

	if (!hub_data->connected) {
		k_mutex_unlock(&hub_data->lock);
		return;
	}

	if (hub_data->state == HUB_STATE_OPERATIONAL) {
		k_mutex_unlock(&hub_data->lock);
		hub_process_status_events(hub_data);
		goto rearm_interrupt;
	}

	if (hub_data->state != HUB_STATE_INIT) {
		LOG_WRN("Hub not in INIT state, skipping initialization");
		k_mutex_unlock(&hub_data->lock);
		return;
	}

	LOG_DBG("Getting 7-byte hub descriptor");

	ret = usbh_req_desc_hub(hub_data->udev,
				hub_data->hub_desc_buf,
				sizeof(struct usb_hub_descriptor));
	if (ret != 0) {
		LOG_ERR("Failed to get hub descriptor: %d", ret);
		hub_data->state = HUB_STATE_ERROR;
		k_mutex_unlock(&hub_data->lock);
		return;
	}

	udev = hub_data->udev;
	hub_data->port_count = hub_data->hub_desc.bNbrPorts;

	LOG_DBG("Hub has %d port_count", hub_data->port_count);

	/* Store hub think time */
	udev->tt = USB_HUB_GET_THINK_TIME(
		sys_le16_to_cpu(hub_data->hub_desc.wHubCharacteristics));
	LOG_DBG("hub think time: 0x%04x", udev->tt);

	total_hub_desc_len = 7 + ((hub_data->port_count + 7) >> 3) + 1;
	/* Get full hub descriptor */
	LOG_DBG("Getting full hub descriptor (length=%d)", total_hub_desc_len);
	ret = usbh_req_desc_hub(hub_data->udev,
				hub_data->hub_desc_buf,
				total_hub_desc_len);
	if (ret != 0) {
		LOG_ERR("Failed to get full hub descriptor: %d", ret);
		hub_data->state = HUB_STATE_ERROR;
		k_mutex_unlock(&hub_data->lock);
		return;
	}

	/* Allocate port list if not already done */
	if (hub_data->port_list == NULL) {
		hub_data->port_list = k_malloc(hub_data->port_count *
					sizeof(struct usb_hub_port));
		if (hub_data->port_list == NULL) {
			LOG_ERR("Failed to allocate port list");
			hub_data->state = HUB_STATE_ERROR;
			k_mutex_unlock(&hub_data->lock);
			return;
		}
	}

	for (uint8_t i = 0; i < hub_data->port_count; i++) {
		hub_data->port_list[i].udev = NULL;
		hub_data->port_list[i].hub = hub_data;
		hub_data->port_list[i].reset_count = CONFIG_USBH_HUB_PORT_RESET_TIMES;
		hub_data->port_list[i].state = PORT_STATE_POWERED_OFF;
		hub_data->port_list[i].num = i + 1;
		hub_data->port_list[i].enum_pending = false;
	}

	for (uint8_t i = 0; i < hub_data->port_count; i++) {
		LOG_DBG("Setting port %d power", i + 1);
		ret = usbh_req_set_hcfs_ppwr(hub_data->udev, i + 1);
		if (ret != 0) {
			LOG_ERR("Failed to set port %d power: %d", i + 1, ret);
		} else {
			hub_data->port_list[i].state = PORT_STATE_DISCONNECTED;
		}
	}

	/* this hub has a parent hub, add to parent's child list */
	if (hub_data->udev->hub != NULL) {
		parent_hub = find_hub_by_udev(hub_data->udev->hub);
		if (parent_hub != NULL) {
			sys_slist_append(&parent_hub->child_hubs, &hub_data->child_node);
		}
	}

	hub_log_info(hub_data);

	hub_data->state = HUB_STATE_OPERATIONAL;

	k_mutex_unlock(&hub_data->lock);

rearm_interrupt:
	if (hub_data->connected && !hub_data->int_active) {
		ret = hub_start_interrupt(hub_data);
		if (ret != 0) {
			LOG_ERR("Failed to start interrupt monitoring: %d", ret);
		}
	}
}

static int usbh_hub_probe(struct usbh_class_data *const c_data,
			  struct usb_device *const udev,
			  const uint8_t iface)
{
	struct usbh_hub_data *hub_data;
	const struct usb_desc_header *header;
	const void *desc_start;
	const void *desc_end;
	uint8_t target_iface;

	if (hub_mgr.total_hubs == CONFIG_USBH_HUB_INSTANCES_COUNT) {
		LOG_ERR("Maximum number of hubs reached (%d)", CONFIG_USBH_HUB_INSTANCES_COUNT);
		return -ENOTSUP;
	}

	if (udev->level > CONFIG_USBH_HUB_MAX_LEVELS) {
		LOG_ERR("Hub chain depth limit exceeded (%d > %d)",
			udev->level,
			CONFIG_USBH_HUB_MAX_LEVELS);
		return -ENOSPC;
	}

	/* Convert device-level match to interface 0 */
	if (iface == USBH_CLASS_IFNUM_DEVICE) {
		target_iface = 0;
	} else {
		target_iface = iface;
	}

	LOG_DBG("USB HUB device probe at interface %u", target_iface);

	desc_start = usbh_desc_get_iface(udev, target_iface);
	if (desc_start == NULL) {
		LOG_ERR("Failed to find interface %u descriptor", iface);
		return -ENOTSUP;
	}

	/* Get the start of next function as the end of current function */
	desc_end = usbh_desc_get_next_function(desc_start);

	hub_data = k_malloc(sizeof(*hub_data));
	if (!hub_data) {
		LOG_ERR("Failed to allocate HUB management data");
		return -ENOTSUP;
	}

	memset(hub_data, 0, sizeof(*hub_data));

	hub_data->udev = udev;
	hub_data->uhs_ctx = (struct usbh_context *)udev->ctx;
	hub_data->state = HUB_STATE_INIT;
	hub_data->port_list = NULL;
	hub_data->interrupt_transfer = NULL;
	hub_data->int_active = false;

	sys_slist_init(&hub_data->child_hubs);

	/* Parse interrupt endpoint within the interface descriptors */
	header = (const void *)desc_start;
	while (header != NULL) {
		/* Stop if we've reached the next function */
		if ((desc_end != NULL) && ((void *)header >= desc_end)) {
			break;
		}

		if (usbh_desc_is_valid_endpoint(header)) {
			const struct usb_ep_descriptor *ep_desc = (const void *)header;

			if ((ep_desc->bEndpointAddress & USB_EP_DIR_MASK) == USB_EP_DIR_IN &&
			    (ep_desc->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) ==
				    USB_EP_TYPE_INTERRUPT) {
				hub_data->int_ep = ep_desc;
				LOG_DBG("Found hub interrupt IN endpoint 0x%02x",
					ep_desc->bEndpointAddress);
				break;
			}
		}

		header = usbh_desc_get_next(header);
	}

	hub_data->connected = true;
	hub_data->int_active = false;

	k_mutex_init(&hub_data->lock);
	k_work_init(&hub_data->hub_work, hub_process);

	c_data->priv = hub_data;

	k_mutex_lock(&hub_mgr.lock, K_FOREVER);
	sys_slist_append(&hub_mgr.hub_list, &hub_data->node);
	hub_mgr.total_hubs++;
	k_mutex_unlock(&hub_mgr.lock);

	k_work_submit_to_queue(&hub_work_q, &hub_data->hub_work);

	return 0;
}

static int usbh_hub_removed(struct usbh_class_data *const cdata)
{
	struct usbh_hub_data *hub_data;
	uint16_t vendor_id;
	uint16_t product_id;
	uint8_t level;
	int ret;

	hub_data = cdata->priv;

	level = hub_data->udev->level;
	vendor_id = sys_le16_to_cpu(hub_data->udev->dev_desc.idVendor);
	product_id = sys_le16_to_cpu(hub_data->udev->dev_desc.idProduct);

	k_mutex_lock(&hub_data->lock, K_FOREVER);
	hub_data->connected = false;
	k_mutex_unlock(&hub_data->lock);

	/* Recursively disconnect all child hubs and devices */
	hub_recursive_disconnect(hub_data);

	k_work_cancel(&hub_data->hub_work);

	k_mutex_lock(&hub_data->lock, K_FOREVER);
	if (hub_data->interrupt_transfer != NULL && hub_data->int_active) {
		ret = usbh_xfer_dequeue(hub_data->udev,
					hub_data->interrupt_transfer);
		if (ret != 0) {
			LOG_ERR("Failed to dequeue interrupt transfer: %d", ret);
		}

		if (hub_data->interrupt_transfer->buf != NULL) {
			usbh_xfer_buf_free(hub_data->udev,
						hub_data->interrupt_transfer->buf);
		}
		usbh_xfer_free(hub_data->udev, hub_data->interrupt_transfer);

		hub_data->interrupt_transfer = NULL;
		hub_data->int_active = false;

		LOG_DBG("Interrupt transfer cancelled");
	}

	/* Remove all connected devices */
	for (uint8_t i = 0; i < hub_data->port_count; i++) {
		if (hub_data->port_list != NULL &&
			hub_data->port_list[i].udev != NULL) {
			hub_data->port_list[i].udev = NULL;
			hub_data->port_list[i].state = PORT_STATE_NOT_CONFIGURED;
		}
	}

	if (hub_data->udev->hub != NULL) {
		struct usbh_hub_data *parent_hub;

		parent_hub = find_hub_by_udev(hub_data->udev->hub);
		if (parent_hub != NULL) {
			sys_slist_find_and_remove(&parent_hub->child_hubs,
							&hub_data->child_node);
		}
	}
	k_mutex_unlock(&hub_data->lock);

	k_mutex_lock(&hub_mgr.lock, K_FOREVER);
	sys_slist_find_and_remove(&hub_mgr.hub_list, &hub_data->node);
	if (hub_mgr.total_hubs > 0) {
		hub_mgr.total_hubs--;
	}
	k_mutex_unlock(&hub_mgr.lock);

	/* Free port list */
	if (hub_data->port_list) {
		k_free(hub_data->port_list);
		hub_data->port_list = NULL;
	}

	LOG_INF("Hub (level %d, Vendor ID: 0x%04x, Product ID: 0x%04x) removal completed",
		level, vendor_id, product_id);

	k_free(hub_data);

	return 0;
}

static int usbh_hub_init(struct usbh_class_data *const c_data)
{
	sys_slist_init(&hub_mgr.hub_list);
	k_mutex_init(&hub_mgr.lock);
	hub_mgr.total_hubs = 0;

	return 0;
}

static int usbh_hub_init_wq(void)
{
	k_work_queue_init(&hub_work_q);
	k_work_queue_start(&hub_work_q, hub_stack, K_KERNEL_STACK_SIZEOF(hub_stack),
			   CONFIG_SYSTEM_WORKQUEUE_PRIORITY, NULL);
	k_thread_name_set(k_work_queue_thread_get(&hub_work_q), "usbh_hub_wq");

	return 0;
}
SYS_INIT(usbh_hub_init_wq, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static struct usbh_class_filter hub_filters[] = {
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_HUB_CLASS_CODE,
		.sub = USB_HUB_SUBCLASS_CODE,
		.proto = 1,
	},
	{0},
};

static struct usbh_class_api usbh_hub_class_api = {
	.init = usbh_hub_init,
	.probe = usbh_hub_probe,
	.removed = usbh_hub_removed,
};

#define USBH_DEFINE_HUB_CLASS(i, _)                                                                \
	USBH_DEFINE_CLASS(UTIL_CAT(usbh_hub_class_, i), &usbh_hub_class_api, NULL, hub_filters)

LISTIFY(CONFIG_USBH_HUB_INSTANCES_COUNT, USBH_DEFINE_HUB_CLASS, (;), _)
