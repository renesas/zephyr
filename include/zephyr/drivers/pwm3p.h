/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for PWM3P (3-phase PWM) devices.
 *
 * PWM3P models one logical 3-phase PWM device: 3 pairs of complementary
 * high-side/low-side outputs (U/V/W) that share one period and one
 * deadtime, updated together as a single unit.
 *
 * @defgroup pwm3p_interface PWM3P (3-phase PWM)
 * @since 4.5
 * @version 1.0.0
 * @ingroup io_interfaces
 * @{
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PWM3P_H_
#define ZEPHYR_INCLUDE_DRIVERS_PWM3P_H_

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PWM3P cycle callback.
 *
 * Invoked once per PWM period, in ISR context, on devices that support it.
 * The callback must not block.
 *
 * @param dev PWM3P device instance.
 * @param user_data User data pointer passed to @ref pwm3p_set_callback.
 */
typedef void (*pwm3p_callback_t)(const struct device *dev, void *user_data);

/**
 * @brief PWM3P driver API.
 */
__subsystem struct pwm3p_driver_api {
	int (*set_period_cycles)(const struct device *dev, uint32_t period_cycles);
	int (*get_period_cycles)(const struct device *dev, uint32_t *period_cycles);
	int (*set_duty_cycles)(const struct device *dev, uint32_t duty_u, uint32_t duty_v,
				uint32_t duty_w);
	int (*enable_output)(const struct device *dev, bool enable);
	int (*get_cycles_per_sec)(const struct device *dev, uint64_t *cycles);
	int (*reset)(const struct device *dev);
	int (*set_callback)(const struct device *dev, pwm3p_callback_t callback, void *user_data);
};

/**
 * @brief Set the shared PWM period for all 3 phases.
 *
 * @param dev PWM3P device instance.
 * @param period_cycles PWM period, in PWM3P counter cycles. Must be at least 2.
 *
 * @retval 0 On success.
 * @retval -EINVAL If @p period_cycles is not valid.
 * @retval -EBUSY If a previous update has not been applied yet, retry after the next
 *                PWM period. The period may already have been changed in that case.
 * @retval -EIO On other failures of the underlying driver.
 */
static inline int pwm3p_set_period_cycles(const struct device *dev, uint32_t period_cycles)
{
	const struct pwm3p_driver_api *api = DEVICE_API_GET(pwm3p, dev);

	return api->set_period_cycles(dev, period_cycles);
}

/**
 * @brief Get the shared PWM period currently applied to all 3 phases.
 *
 * @param dev PWM3P device instance.
 * @param period_cycles Pointer to the PWM period, in PWM3P counter cycles.
 *
 * @retval 0 On success.
 */
static inline int pwm3p_get_period_cycles(const struct device *dev, uint32_t *period_cycles)
{
	const struct pwm3p_driver_api *api = DEVICE_API_GET(pwm3p, dev);

	return api->get_period_cycles(dev, period_cycles);
}

/**
 * @brief Set the duty cycles for phases U, V, and W.
 *
 * @param dev PWM3P device instance.
 * @param duty_u Duty cycle for phase U, in PWM3P counter cycles.
 * @param duty_v Duty cycle for phase V, in PWM3P counter cycles.
 * @param duty_w Duty cycle for phase W, in PWM3P counter cycles.
 *
 * @retval 0 On success.
 * @retval -EINVAL If any duty cycle is not valid for the current period and deadtime.
 * @retval -EBUSY If a previous update has not been applied yet, retry after the next
 *                PWM period. Some phases may already have been updated in that case.
 * @retval -EIO On other failures of the underlying driver.
 */
static inline int pwm3p_set_duty_cycles(const struct device *dev, uint32_t duty_u,
					 uint32_t duty_v, uint32_t duty_w)
{
	const struct pwm3p_driver_api *api = DEVICE_API_GET(pwm3p, dev);

	return api->set_duty_cycles(dev, duty_u, duty_v, duty_w);
}

/**
 * @brief Enable or disable all 6 PWM outputs (U/V/W high+low) together.
 *
 * @param dev PWM3P device instance.
 * @param enable true to enable the outputs, false to disable them.
 *
 * @retval 0 On success.
 * @retval -EIO On failures of the underlying driver.
 */
static inline int pwm3p_enable_output(const struct device *dev, bool enable)
{
	const struct pwm3p_driver_api *api = DEVICE_API_GET(pwm3p, dev);

	return api->enable_output(dev, enable);
}

/**
 * @brief Get the PWM counter clock rate.
 *
 * @param dev PWM3P device instance.
 * @param cycles Pointer to the counter clock rate in cycles per second.
 *
 * @retval 0 On success.
 * @retval -EIO On failures of the underlying driver.
 */
static inline int pwm3p_get_cycles_per_sec(const struct device *dev, uint64_t *cycles)
{
	const struct pwm3p_driver_api *api = DEVICE_API_GET(pwm3p, dev);

	return api->get_cycles_per_sec(dev, cycles);
}

/**
 * @brief Reset the PWM3P counters to 0.
 *
 * @param dev PWM3P device instance.
 *
 * @retval 0 On success.
 * @retval -EAGAIN If the PWM3P outputs are not currently enabled.
 * @retval -EIO On other failures of the underlying driver.
 */
static inline int pwm3p_reset(const struct device *dev)
{
	const struct pwm3p_driver_api *api = DEVICE_API_GET(pwm3p, dev);

	return api->reset(dev);
}

/**
 * @brief Register a callback fired once per PWM period.
 *
 * The callback runs in ISR context once per PWM cycle. It is intended as a
 * control-loop tick for use cases such as synchronizing an ADC sample to the
 * PWM cycle. Pass @p callback as NULL to unregister a previously set callback.
 *
 * @param dev PWM3P device instance.
 * @param callback Function to call once per PWM period, or NULL to unregister.
 * @param user_data Opaque pointer passed back to @p callback.
 *
 * @retval 0 On success.
 * @retval -ENOTSUP If this device instance does not support per-cycle callbacks.
 * @retval -EIO On other failures of the underlying driver.
 */
static inline int pwm3p_set_callback(const struct device *dev, pwm3p_callback_t callback,
				      void *user_data)
{
	const struct pwm3p_driver_api *api = DEVICE_API_GET(pwm3p, dev);

	return api->set_callback(dev, callback, user_data);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_PWM3P_H_ */
