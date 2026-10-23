.. _pwm3p_api:

Three-Phase PWM (PWM3P)
#######################

Overview
********

The PWM3P API controls a three-phase PWM device with complementary high-side
and low-side outputs for the U, V, and W phases. The phases share one period
and one dead time, while each phase has an independent duty cycle.

PWM3P is separate from the single-output :ref:`PWM API <pwm_api>` because all
three phases and their complementary outputs must be configured and controlled
as one synchronized unit.

The Renesas RH850 implementation is provided by
``drivers/pwm3p/pwm3p_renesas_rh850_taud3p.c``. It coordinates 13 channels of one
TAUD unit to implement one PWM3P instance.

Concepts
********

* The six U, V, and W high-side and low-side outputs are enabled or disabled
  together with :c:func:`pwm3p_enable_output`.
* The three phases share one period, expressed in PWM counter cycles.
* :c:func:`pwm3p_get_cycles_per_sec` returns the PWM counter clock rate.
* U, V, and W duty cycles are expressed in counter cycles. A duty cycle is the
  active time of the high-side output of the phase. The low-side output is
  complementary, with the dead time inserted between the two outputs.
* The dead time is configured at build time with the ``deadtime-counts``
  devicetree property.
* When supported by the driver instance, a callback can run once per PWM
  period in interrupt context. It can be used as a control-loop timing event.

RH850 TAUD3P implementation
***************************

One RH850 TAUD3P instance uses 13 channels from one TAUD unit:

* One master PWM channel.
* Six slave PWM channels for the complementary U, V, and W outputs.
* Three master one-shot channels.
* Three slave one-shot channels.

The channel assignments are fixed by the driver and are not configurable. The
driver sets up all the channels during device initialization.

Constraints
===========

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Parameter
     - Constraint
   * - Period
     - Even, at least 2 counter cycles, and not greater than 131072. The dead
       time must not exceed half of the period.
   * - Duty cycle
     - ``0`` (0%), up to ``period - max(deadtime, 1)``, or exactly ``period``
       (100%). Values in between are rejected with ``-EINVAL``.
   * - Dead time
     - Set with ``deadtime-counts`` in devicetree, at most 65535 counter
       cycles and at most half of the initial period.
   * - Updates
     - A new period or new duty cycles take effect from the next PWM period.
       A call made while the previous update is pending returns ``-EBUSY`` and
       can be retried. When the period changes, the duty cycles are rescaled to
       keep their ratio.

The API is not protected against concurrent use. Do not call the functions of
one device from several threads, or from a thread and the cycle callback, at
the same time.

Devicetree configuration
************************

The SoC devicetree defines disabled TAUD3P nodes for supported TAUD units. The
PWM3P is an opt-in feature: the board devicetree provides the pin control
configuration and an application enables the required node, together with the
PIC node that it refers to.

.. code-block:: devicetree

   &pic0 {
           status = "okay";
   };

   &taud3p1 {
           pinctrl-0 = <&taud3p1_default>;
           pinctrl-names = "default";
           status = "okay";
   };

The ``renesas,rh850-taud3p-pwm3p`` binding uses the following properties:

``taud-three-phase-unit``
   Index of the TAUD unit used by the PWM3P instance.

``clocks``
   Module clock of the TAUD unit, set by the SoC devicetree, for example
   ``clocks = <&cgc RH850_PCLK(TAUD, 0, PCLK)>``. The channel is the index of the
   TAUD unit. The driver enables it with ``clock_control_on()`` before it accesses
   any TAUD register. The clock controller is initialized before the PWM3P device.

``pic``
   Phandle of the PIC node that routes the PWM3P signals, for example
   ``pic = <&pic0>``. Only used, and then required, when
   :kconfig:option:`CONFIG_RENESAS_RH850_PIC` is enabled. The node must be
   enabled and the PIC is initialized before the PWM3P device.

``period-counts``
   Shared PWM period in counter cycles.

``deadtime-counts``
   Dead time in counter cycles. The default is 240.

``operation-clk`` and ``clk-div``
   TAUD operation clock selection and divider. These settings are shared by
   all channels of the TAUD unit. Both default to 0.

``interrupts``
   Optional per-period interrupt. If the interrupt is not defined,
   :c:func:`pwm3p_set_callback` returns ``-ENOTSUP``. The interrupt is only
   enabled while a callback is registered.

Do not enable two nodes that use the same TAUD unit and TAUD channel at the same time.
The pins of the PWM3P outputs can also be used by other peripherals of the board,
check the pin control configuration of the board before use.

PIC output routing
******************

The PWM3P driver does not configure the Peripheral Interconnect Controller (PIC).
The PIC is configured separately through the ``pic0`` devicetree node,
which the PWM3P node refers to with its ``pic`` property.

Enable ``pic0`` and select the high-accuracy triangle-wave PWM function:

.. code-block:: devicetree

   &pic0 {
           taud-input-control-function =
                   "high-accuracy-triangle-wave-pwm-with-dead-time";
           status = "okay";
   };

The board devicetree can provide the corresponding ``triangle-wave-pwm`` phase
configuration. Without the required PIC route, the PWM3P device can initialize
but its outputs are not routed to the pins.

The PIC is initialized before the PWM3P device because of the ``pic``
property. The PWM3P driver fails to initialize with ``-ENODEV`` if the PIC is not
ready.

Usage
*****

.. code-block:: c

   #include <zephyr/drivers/pwm3p.h>

   const struct device *const pwm3p =
           DEVICE_DT_GET(DT_NODELABEL(taud3p1));
   uint32_t period;
   int ret;

   if (!device_is_ready(pwm3p)) {
           return;
   }

   ret = pwm3p_get_period_cycles(pwm3p, &period);
   if (ret != 0) {
           return;
   }

   /* U = 25%, V = 50%, and W = 75% of the period. */
   ret = pwm3p_set_duty_cycles(pwm3p, period / 4U, period / 2U,
                               (period * 3U) / 4U);
   if (ret != 0) {
           return;
   }

   ret = pwm3p_enable_output(pwm3p, true);
   if (ret != 0) {
           return;
   }

:c:func:`pwm3p_set_period_cycles` changes the shared period.
:c:func:`pwm3p_reset` resets the PWM counters. Refer to the API reference for
operation-specific constraints and return values.

A callback can be registered to run once per PWM period, for example to start an
ADC conversion in step with the PWM:

.. code-block:: c

   static void on_pwm_period(const struct device *dev, void *user_data)
   {
           /* Runs in interrupt context, keep it short. */
   }

   ret = pwm3p_set_callback(pwm3p, on_pwm_period, NULL);
   if (ret == -ENOTSUP) {
           /* The devicetree node has no 'interrupts' property. */
   }

   /* Unregister the callback, which also disables the per-period interrupt. */
   ret = pwm3p_set_callback(pwm3p, NULL, NULL);

Configuration options
*********************

* :kconfig:option:`CONFIG_PWM3P`
* :kconfig:option:`CONFIG_PWM3P_RENESAS_RH850_TAUD3P`
* :kconfig:option:`CONFIG_PWM3P_INIT_PRIORITY`

API reference
*************

.. doxygengroup:: pwm3p_interface
