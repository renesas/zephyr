/*
 * Copyright (c) 2026 Embedd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Human Interface Device (HID) driver API
 *
 * Header follows Device Class Definition for Human Interface Devices (HID)
 * Version 1.11 document (HID1_11-1.pdf).
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USBH_HID_H_
#define ZEPHYR_INCLUDE_USB_CLASS_USBH_HID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/device.h>
#include <zephyr/usb/class/hid.h>

/**
 * @brief Starts the input interrupt pipeline
 *
 * @param dev  Pointer to the device
 *
 * @return 0 on success, negative errno value on failure.
 */
typedef int (*usbh_hid_start_input_reports_t)(struct device const *dev);

/**
 * @brief Stops the input interrupt pipeline
 *
 * @param dev  Pointer to the device
 *
 * @return 0 on success, negative errno value on failure.
 */
typedef int (*usbh_hid_stop_input_reports_t)(struct device const *dev);

/**
 * @brief Reads the report descriptor of the device
 *
 * @param dev          Pointer to the device
 * @param report[out]  Pointer to the report structure to fill
 *
 * @return 0 on success, negative errno value on failure.
 */
typedef int (*usbh_hid_get_report_descriptor_t)(struct device const *dev,
						struct hid_report *report);

/**
 * @brief Reads a report from the device
 *
 * @param dev          Pointer to the device
 * @param type         Report type
 * @param report_id    Report ID (0 if not relevant)
 * @param length       Output buffer length (should be inquired from the report descriptor)
 * @param buffer[out]  Output buffer
 *
 * @return 0 on success, negative errno value on failure.
 */
typedef int (*usbh_hid_get_report_t)(struct device const *dev, enum hid_report_field_type type,
				     uint8_t report_id, size_t length, uint8_t *buffer);

/**
 * @brief Sends a report (output or feature) to the device
 *
 * @param dev          Pointer to the device
 * @param type         Report type
 * @param report_id    Report ID (if the device supports multiple repornt variants on the endpoint)
 * @param data_length  Length of the report data
 * @param data         Report data
 *
 * @return 0 on success, negative errno value on failure.
 */
typedef int (*usbh_hid_set_report_t)(struct device const *dev, enum hid_report_field_type type,
				     uint8_t report_id, size_t data_length,
				     uint8_t const data[data_length]);

/**
 * @brief Registers a callback to be invoked on input report
 *
 * @param dev          Pointer to the device
 * @param callback     User-defined callback
 * @param user_data    Optional void pointer to be provided to the callback
 *
 * @return 0 on success, negative errno value on failure.
 */
typedef int (*usbh_hid_set_input_callback_t)(const struct device *dev, hid_report_cb_t callback,
					     void *user_data);

/**
 * @driver_ops{USBH HID}
 */
__subsystem struct usbh_hid_driver_api {
	/**
	 * @driver_ops_optional @copybrief usbh_hid_start_input_reports
	 */
	usbh_hid_start_input_reports_t start_input_reports;
	/**
	 * @driver_ops_optional @copybrief usbh_hid_stop_input_reports
	 */
	usbh_hid_stop_input_reports_t stop_input_reports;
	/**
	 * @driver_ops_optional @copybrief usbh_hid_get_report_descriptor
	 */
	usbh_hid_get_report_descriptor_t get_report_descriptor;
	/**
	 * @driver_ops_optional @copybrief usbh_hid_get_report
	 */
	usbh_hid_get_report_t get_report;
	/**
	 * @driver_ops_optional @copybrief usbh_hid_set_report
	 */
	usbh_hid_set_report_t set_report;
	/**
	 * @driver_ops_optional @copybrief usbh_hid_set_input_callback
	 */
	usbh_hid_set_input_callback_t set_input_callback;
};

/**
 * @brief Start the input report pipeline
 *
 * Activates the interrupt input pipeline, prompting the underlying USB host controller to
 * periodically send updates.
 *
 * @param dev  Pointer to the device structure for the driver instance.
 *
 * @return 0 on success, negative errno value on failure.
 */
__syscall int usbh_hid_start_input_reports(const struct device *dev);

static inline int z_impl_usbh_hid_start_input_reports(struct device const *dev)
{
	struct usbh_hid_driver_api const *api = DEVICE_API_GET(usbh_hid, dev);
	if (api->start_input_reports == NULL) {
		return -ENOSYS;
	}
	return api->start_input_reports(dev);
}

/**
 * @brief Stops the input report pipeline
 *
 * @param dev  Pointer to the device structure for the driver instance.
 *
 * @return 0 on success, negative errno value on failure.
 */
__syscall int usbh_hid_stop_input_reports(const struct device *dev);

static inline int z_impl_usbh_hid_stop_input_reports(struct device const *dev)
{
	struct usbh_hid_driver_api const *api = DEVICE_API_GET(usbh_hid, dev);
	if (api->stop_input_reports == NULL) {
		return -ENOSYS;
	}
	return api->stop_input_reports(dev);
}

/**
 * @brief Reads the report descriptor of the device
 *
 * @param      dev     Pointer to the device structure for the driver instance.
 * @param[out] report  Pointer to the report structure to
 *
 * @return 0 on success, negative errno value on failure.
 */
__syscall int usbh_hid_get_report_descriptor(const struct device *dev, struct hid_report *report);

static inline int z_impl_usbh_hid_get_report_descriptor(struct device const *dev,
							struct hid_report *report)
{
	struct usbh_hid_driver_api const *api = DEVICE_API_GET(usbh_hid, dev);
	if (api->get_report_descriptor == NULL) {
		return -ENOSYS;
	}
	return api->get_report_descriptor(dev, report);
}

/**
 * @brief Reads a report from the device
 *
 * @param      dev        Pointer to the device
 * @param      type       Report type
 * @param      report_id  Report ID (0 if not relevant)
 * @param      length     Output buffer length (should be inquired from the report descriptor)
 * @param[out] buffer     Output buffer
 *
 * @return 0 on success, negative errno value on failure.
 */
__syscall int usbh_hid_get_report(struct device const *dev, enum hid_report_field_type type,
				  uint8_t report_id, size_t length, uint8_t *buffer);

static inline int z_impl_usbh_hid_get_report(struct device const *dev,
					     enum hid_report_field_type type, uint8_t report_id,
					     size_t length, uint8_t *buffer)
{
	struct usbh_hid_driver_api const *api = DEVICE_API_GET(usbh_hid, dev);
	if (api->get_report == NULL) {
		return -ENOSYS;
	}
	return api->get_report(dev, type, report_id, length, buffer);
}

/**
 * @brief Sends a report (output or feature) to the device
 *
 * @param dev          Pointer to the device
 * @param type         Report type
 * @param report_id    Report ID (if the device supports multiple repornt variants on the endpoint)
 * @param data_length  Length of the report data
 * @param data         Report data
 *
 * @return 0 on success, negative errno value on failure.
 */
__syscall int usbh_hid_set_report(const struct device *dev, enum hid_report_field_type type,
				  uint8_t report_id, size_t data_length, uint8_t const *data);

static inline int z_impl_usbh_hid_set_report(struct device const *dev,
					     enum hid_report_field_type type, uint8_t report_id,
					     size_t data_length, uint8_t const *data)
{
	struct usbh_hid_driver_api const *api = DEVICE_API_GET(usbh_hid, dev);
	if (api->set_report == NULL) {
		return -ENOSYS;
	}
	return api->set_report(dev, type, report_id, data_length, data);
}

/**
 * @brief Registers a callback to be invoked on input report
 *
 * @param dev          Pointer to the device
 * @param callback     User-defined callback
 * @param user_data    Optional void pointer to be provided to the callback
 *
 * @return 0 on success, negative errno value on failure.
 */
__syscall int usbh_hid_set_input_callback(const struct device *dev, hid_report_cb_t callback,
					  void *user_data);

static inline int z_impl_usbh_hid_set_input_callback(const struct device *dev,
						     hid_report_cb_t callback, void *user_data)
{
	struct usbh_hid_driver_api const *api = DEVICE_API_GET(usbh_hid, dev);
	if (api->set_input_callback == NULL) {
		return -ENOSYS;
	}
	return api->set_input_callback(dev, callback, user_data);
}

#include <zephyr/syscalls/usbh_hid.h>

#ifdef __cplusplus
}
#endif

#endif
