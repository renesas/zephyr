/*
 * Copyright (c) 2026 Renesas Electronics Corporation, Embedd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/class/usbh_billboard.h>
#include <zephyr/usb/bos.h>

#include "usbh_ch9.h"
#include "usbh_class.h"
#include "usbh_desc.h"
#include "usbh_device.h"

LOG_MODULE_REGISTER(usbh_billboard, CONFIG_USBH_BILLBOARD_LOG_LEVEL);

/* String descriptor memory limit */
#define MAX_STRDESC_LEN (255)

/* NOTE: List of supported codes is at usb.org (LANGIDs) */
#ifndef USB_LANG_ENGLISH_USA
#define USB_LANG_ENGLISH_USA (0x0409)
#endif

/**
 * Driver data
 */
struct usbh_billboard_drv_data {
	/* Mutex */
	struct k_mutex lock;
	/* Device was probed */
	bool probed;
	struct usb_device *udev;
	uint16_t iface;
	/* Language code used for String descriptor requests */
	uint16_t lang_code;
	/* Preallocated buffer for String descriptor requests */
	struct net_buf *string_netbuf;
};

/**
 * @brief Billboard driver initialization
 *
 * @param c_data  Pointer to USB class
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_billboard_class_init(struct usbh_class_data *const c_data)
{
	const struct device *dev = c_data->priv;
	struct usbh_billboard_drv_data *const drv_data = dev->data;

	k_mutex_init(&drv_data->lock);
	drv_data->probed = false;

	return 0;
}

/**
 * @brief Billboard driver probe function, invoked upon USB device attachment
 *
 * @param c_data        Pointer to USB class
 * @param udev          Pointer to USB device
 * @param iface         Interface number
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_billboard_class_probe(struct usbh_class_data *const c_data,
				      struct usb_device *const udev, uint8_t iface)
{
	const struct device *dev = c_data->priv;
	struct usbh_billboard_drv_data *drv_data = (void *)dev->data;
	int result = 0;

	LOG_DBG("Billboard device was attached\n");

	result = k_mutex_lock(&drv_data->lock, K_FOREVER);
	if (result) {
		return result;
	}

	drv_data->udev = udev;
	drv_data->iface = iface;

	/* Using English language as default one */
	drv_data->lang_code = USB_LANG_ENGLISH_USA;

	/* Shared string_netbuf, allocated on first string descriptor request */
	drv_data->string_netbuf = NULL;

	if (result == 0) {
		drv_data->probed = true;
	}
	k_mutex_unlock(&drv_data->lock);

	return result;
}

/**
 * @brief Billboard driver remove function, invoked upon USB device removal
 *
 * @param c_data        Pointer to USB class
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_billboard_class_removed(struct usbh_class_data *const c_data)
{
	const struct device *dev = c_data->priv;
	struct usbh_billboard_drv_data *drv_data = (void *)dev->data;
	int result = 0;

	result = k_mutex_lock(&drv_data->lock, K_FOREVER);
	if (result) {
		return result;
	}

	if (drv_data->string_netbuf != NULL) {
		usbh_xfer_buf_free(drv_data->udev, drv_data->string_netbuf);
		drv_data->string_netbuf = NULL;
	}

	drv_data->udev = NULL;
	drv_data->probed = false;
	k_mutex_unlock(&drv_data->lock);

	LOG_DBG("Billboard device was removed\n");

	return result;
}

/**
 * @brief Fetches the BOS descriptor. Memory is allocated internally and the caller is responsible
 * for freeing it
 *
 * @param drv_data        Pointer to driver data
 * @param bos_netbuf      Output argument, double pointer net_buf, filled internally
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_billboard_fetch_bos(struct usbh_billboard_drv_data *drv_data,
				    struct net_buf **bos_netbuf)
{
	struct usb_bos_descriptor *bos_desc;
	struct net_buf *header_netbuf = NULL;
	struct net_buf *data_netbuf = NULL;
	int result = 0;

	do {
		header_netbuf = usbh_xfer_buf_alloc(drv_data->udev, sizeof(*bos_desc));
		if (header_netbuf == NULL) {
			LOG_ERR("Cannot allocate memory for BOS header");
			result = -ENOMEM;
			break;
		}

		result = usbh_req_desc(drv_data->udev, USB_DESC_BOS, 0, 0, sizeof(*bos_desc),
				       header_netbuf);
		if (result != 0) {
			LOG_ERR("Cannot request BOS header");
			result = -EFAULT;
			break;
		}

		/* Hypotetical situation when malicious device send back 0 length packet */
		bos_desc = (struct usb_bos_descriptor *)header_netbuf->data;
		if (unlikely(bos_desc == NULL)) {
			result = -EFAULT;
			break;
		}

		data_netbuf = usbh_xfer_buf_alloc(drv_data->udev, bos_desc->wTotalLength);
		if (data_netbuf == NULL) {
			LOG_ERR("Cannot allocate memory for BOS");
			result = -ENOMEM;
			break;
		}

		result = usbh_req_desc(drv_data->udev, USB_DESC_BOS, 0, 0, bos_desc->wTotalLength,
				       data_netbuf);
		if (result != 0) {
			LOG_ERR("Cannot request BOS");
			result = -EFAULT;
			break;
		}

		/* Hypotetical situation when malicious device send back 0 length packet */
		if (unlikely(data_netbuf->data == NULL)) {
			result = -EFAULT;
			break;
		}
	} while (0);

	if (header_netbuf != NULL) {
		usbh_xfer_buf_free(drv_data->udev, header_netbuf);
	}

	if (result == 0) {
		*bos_netbuf = data_netbuf;
	} else if (data_netbuf != NULL) {
		usbh_xfer_buf_free(drv_data->udev, data_netbuf);
	}

	return result;
}

/**
 * @brief Fetches the BOS descriptor, parses capability descriptors, and invokes the callback for
 * BILLBOARD and BILLBOARD_EX.
 *
 * @param dev             Pointer to device
 * @param billboard_cb    Pointer to callback
 * @param cb_arg          Pointer to callback argument
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_billboard_fetch_and_parse_api(struct device const *dev,
					      usbh_billboard_cb_t billboard_cb, void *cb_arg)
{
	struct usbh_billboard_drv_data *drv_data;
	struct usb_bos_capability_header *cap_header;
	const struct usb_desc_header *desc;
	struct net_buf *bos_netbuf = NULL;
	struct usb_bos_descriptor *bos_desc = NULL;
	int result = 0;

	if (dev == NULL || billboard_cb == NULL) {
		return -EINVAL;
	}

	drv_data = dev->data;

	result = k_mutex_lock(&drv_data->lock, K_FOREVER);
	if (result) {
		return result;
	}

	do {
		if (!drv_data->probed) {
			LOG_DBG("Driver was not probed yet");
			result = -EAGAIN;
			break;
		}

		result = usbh_billboard_fetch_bos(drv_data, &bos_netbuf);
		if (result) {
			break;
		}

		bos_desc = (struct usb_bos_descriptor *)bos_netbuf->data;
		desc = (struct usb_desc_header *)bos_netbuf->data;
		for (int i = 0; i < bos_desc->bNumDeviceCaps; i++) {
			desc = usbh_desc_get_next(desc);
			if (desc == NULL) {
				LOG_ERR("Number of capability descriptor is less than "
					"'bNumDeviceCaps'");
				result = -EFAULT;
				break;
			}

			cap_header = (struct usb_bos_capability_header *)desc;
			if (cap_header->bDescriptorType != USB_DESC_DEVICE_CAPABILITY) {
				continue;
			}

			/* Invoke callback on billboard_capability and aum_capability_descriptor */
			if ((cap_header->bDevCapabilityType == USB_BOS_CAPABILITY_BILLBOARD) ||
			    (cap_header->bDevCapabilityType == USB_BOS_CAPABILITY_BILLBOARD_EX)) {
				billboard_cb(cb_arg, cap_header);
			}
		}
	} while (0);

	if (bos_netbuf != NULL) {
		usbh_xfer_buf_free(drv_data->udev, bos_netbuf);
	}

	k_mutex_unlock(&drv_data->lock);

	return result;
}

/**
 * @brief Allocates a shared net_buf for string descriptors if it has not been allocated yet
 *
 * @param drv_data        Pointer to driver data
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_billboard_alloc_string_netbuf(struct usbh_billboard_drv_data *drv_data)
{
	/* Allocated and re-used ad hoc */
	if (drv_data->string_netbuf == NULL) {
		drv_data->string_netbuf = usbh_xfer_buf_alloc(drv_data->udev, MAX_STRDESC_LEN);
		if (drv_data->string_netbuf == NULL) {
			LOG_ERR("Cannot allocate string netbuffer");
			return -ENOMEM;
		}
	}

	return 0;
}

/**
 * @brief Fetch string descriptor
 *
 * @param drv_data        Pointer to driver data
 * @param str_idx         String index
 * @param lang_code       Language code
 * @param str_desc        Output argumnet, double pointer to string descriptor
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_billboard_fetch_string_desc_internal(struct usbh_billboard_drv_data *drv_data,
						     uint8_t str_idx, uint16_t lang_code,
						     struct usb_string_descriptor **str_desc)
{
	struct usb_string_descriptor *tmp_str_desc;
	uint8_t str_desc_len;
	int result = 0;

	do {
		result = usbh_billboard_alloc_string_netbuf(drv_data);
		if (result) {
			break;
		}

		/* Phase 1 - retrieve the string descriptor length */
		net_buf_reset(drv_data->string_netbuf);
		result = usbh_req_desc(drv_data->udev, USB_DESC_STRING, str_idx, lang_code, 2,
				       drv_data->string_netbuf);
		if (result) {
			LOG_ERR("Cannot request string desc %d, phase 1", str_idx);
			break;
		}

		tmp_str_desc = (struct usb_string_descriptor *)drv_data->string_netbuf->data;
		if (unlikely(tmp_str_desc == NULL)) {
			result = -EFAULT;
			break;
		}
		str_desc_len = tmp_str_desc->bLength;

		/* Phase 2 - request the full string descriptor */
		net_buf_reset(drv_data->string_netbuf);
		result = usbh_req_desc(drv_data->udev, USB_DESC_STRING, str_idx, lang_code,
				       str_desc_len, drv_data->string_netbuf);
		if (result) {
			LOG_ERR("Cannot request string desc %d, phase 2", str_idx);
			break;
		}

		if (drv_data->string_netbuf->data == NULL) {
			result = -EFAULT;
			break;
		}
		*str_desc = (struct usb_string_descriptor *)drv_data->string_netbuf->data;
	} while (0);

	return result;
}

/**
 * @brief Fetch string descriptor based on string ID
 *
 * @param drv_data        Pointer to driver data
 * @param str_idx         String index
 * @param str_desc        Output argumnet, double pointer to string descriptor
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_billboard_fetch_string_desc_api(struct device const *dev, uint8_t str_idx,
						struct usb_string_descriptor **str_desc)
{
	struct usbh_billboard_drv_data *drv_data;
	int result = 0;

	do {
		if (dev == NULL || str_desc == NULL) {
			result = -EINVAL;
			break;
		}
		drv_data = dev->data;

		result = k_mutex_lock(&drv_data->lock, K_FOREVER);
		if (result) {
			break;
		}
		result = usbh_billboard_fetch_string_desc_internal(drv_data, str_idx,
								  drv_data->lang_code, str_desc);
		k_mutex_unlock(&drv_data->lock);
	} while(0);

	return result;
}

/**
 * @brief Fetches available languages from the device
 *
 * @param drv_data        Pointer to driver data
 * @param str_desc        Output argument, double pointer to string descriptor
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_billboard_fetch_langs_desc_api(struct device const *dev,
					       struct usb_string_descriptor **str_desc)
{
	struct usbh_billboard_drv_data *drv_data;
	int result = 0;

	do {
		if (dev == NULL || str_desc == NULL) {
			result = -EINVAL;
			break;
		}
		drv_data = dev->data;

		result = k_mutex_lock(&drv_data->lock, K_FOREVER);
		if (result) {
			break;
		}
		result = usbh_billboard_fetch_string_desc_internal(drv_data, 0, 0, str_desc);
		k_mutex_unlock(&drv_data->lock);
	} while (0);

	return result;
}

/**
 * @brief Changes the language code used for fetching string descriptors
 *
 * @param drv_data        Pointer to driver data
 * @param lang_code       Language code
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_billboard_use_lang_api(struct device const *dev, uint16_t lang_code)
{
	struct usbh_billboard_drv_data *drv_data;
	struct usb_string_descriptor *str_desc;
	size_t lang_count, i;
	int result = 0;

	if (dev == NULL) {
		return -EINVAL;
	}
	drv_data = dev->data;

	result = k_mutex_lock(&drv_data->lock, K_FOREVER);
	if (result) {
		return result;
	}

	do {
		result = usbh_billboard_fetch_string_desc_internal(drv_data, 0, 0, &str_desc);
		if (result) {
			break;
		}

		result = -ENOTSUP;
		str_desc = (struct usb_string_descriptor *)drv_data->string_netbuf->data;
		lang_count = str_desc->bLength / 2;

		/* Search for requested 'lang_code' in device response */
		for (i = 0; i < lang_count; i++) {
			if (lang_code == ((uint16_t *)(&str_desc->bString))[i]) {
				drv_data->lang_code = lang_code;
				result = 0;
				break;
			}
		}
	} while (0);
	k_mutex_unlock(&drv_data->lock);

	return result;
}

/**
 * @brief USB Host Billboard driver table
 */
DEVICE_API(usbh_billboard, usbh_billboard_api) = {
	.fetch_and_parse = usbh_billboard_fetch_and_parse_api,
	.fetch_string = usbh_billboard_fetch_string_desc_api,
	.fetch_langs = usbh_billboard_fetch_langs_desc_api,
	.use_lang = usbh_billboard_use_lang_api,
};

static struct usbh_class_api usbh_billboard_class_api = {
	.init = usbh_billboard_class_init,
	.probe = usbh_billboard_class_probe,
	.removed = usbh_billboard_class_removed,
};

static struct usbh_class_filter usbh_billboard_filters[] = {
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_BILLBOARD,
		.sub = USB_BILLBOARD_SUBCLASS,
		.proto = USB_BILLBOARD_RUNTIME,
	},
	{0},
};

int usbh_billboard_find_dev(struct usb_device *udev, struct device **dev)
{
	STRUCT_SECTION_FOREACH(usbh_class_node, node) {
		if (node->c_data->api != &usbh_billboard_class_api || node->c_data->udev != udev) {
			continue;
		}
		*dev = (struct device *)node->c_data->priv;
		return 0;
	}

	return -ENODEV;
}

#define USBH_BILLBOARD_DEVICE_DEFINE(n, _)                                                         \
	static struct usbh_billboard_drv_data usbh_billboard_drv_data##n;                          \
	DEVICE_DEFINE(usbh_billboard_##n, "usbh_billboard_" #n, NULL, NULL,                        \
		      &usbh_billboard_drv_data##n, NULL, POST_KERNEL, 91, &usbh_billboard_api);    \
	USBH_DEFINE_CLASS(billboard_host_c_data_##n, &usbh_billboard_class_api,                    \
			  (void *)DEVICE_GET(usbh_billboard_##n), usbh_billboard_filters);

LISTIFY(CONFIG_USBH_BILLBOARD_INSTANCES_COUNT, USBH_BILLBOARD_DEVICE_DEFINE, (;), _)
