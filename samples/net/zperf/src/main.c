/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Zperf sample.
 */
#include <zephyr/usb/usbd.h>
#include <zephyr/net/net_config.h>

LOG_MODULE_REGISTER(zperf, CONFIG_NET_ZPERF_LOG_LEVEL);

#ifdef CONFIG_NET_LOOPBACK_SIMULATE_PACKET_DROP
#include <zephyr/net/loopback.h>
#endif

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include <sample_usbd.h>
#endif

#if defined(CONFIG_USB_HOST_STACK)
#include <zephyr/usb/usbh.h>
#include <zephyr/drivers/usb/uhc.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>

USBH_CONTROLLER_DEFINE(sample_uhs_ctx, DEVICE_DT_GET(DT_NODELABEL(zephyr_uhc0)));

#if defined(CONFIG_NET_CONFIG_SETTINGS)
static struct net_mgmt_event_callback usbh_iface_cb;

static void usbh_iface_up_handler(struct net_mgmt_event_callback *cb,
				  uint64_t mgmt_event, struct net_if *iface)
{
	if (mgmt_event != NET_EVENT_IF_UP) {
		return;
	}

	LOG_INF("USB host interface up, configuring static IP");
	(void)net_config_init_app(NULL, "USB host network");
}
#endif /* CONFIG_NET_CONFIG_SETTINGS */

static int zperf_usbh_init(void)
{
	int ret;

#if defined(CONFIG_NET_CONFIG_SETTINGS)
	net_mgmt_init_event_callback(&usbh_iface_cb, usbh_iface_up_handler,
				     NET_EVENT_IF_UP);
	net_mgmt_add_event_callback(&usbh_iface_cb);
#endif

	ret = usbh_init(&sample_uhs_ctx);
	if (ret) {
		LOG_ERR("Failed to init USB host: %d", ret);
		return ret;
	}

	ret = usbh_enable(&sample_uhs_ctx);
	if (ret) {
		LOG_ERR("Failed to enable USB host: %d", ret);
		return ret;
	}

	ret = uhc_bus_reset(sample_uhs_ctx.dev);
	if (ret) {
		LOG_ERR("Failed to signal bus reset: %d", ret);
		return ret;
	}

	ret = uhc_bus_resume(sample_uhs_ctx.dev);
	if (ret) {
		LOG_ERR("Failed to signal bus resume: %d", ret);
		return ret;
	}

	ret = uhc_sof_enable(sample_uhs_ctx.dev);
	if (ret) {
		LOG_ERR("Failed to enable SoF: %d", ret);
		return ret;
	}

	return 0;
}
#endif /* CONFIG_USB_HOST_STACK */

SYS_INIT(zperf_usbh_init, APPLICATION, 94);

int main(void)
{
#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	struct usbd_context *sample_usbd;
	int err;

	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		return -ENODEV;
	}

	err = usbd_enable(sample_usbd);
	if (err) {
		return err;
	}

	(void)net_config_init_app(NULL, "Initializing network");
#endif /* CONFIG_USB_DEVICE_STACK_NEXT */

#ifdef CONFIG_NET_LOOPBACK_SIMULATE_PACKET_DROP
	loopback_set_packet_drop_ratio(1);
#endif

#if defined(CONFIG_NET_DHCPV4) && !defined(CONFIG_NET_CONFIG_SETTINGS)
	net_dhcpv4_start(net_if_get_default());
#endif
	return 0;
}
