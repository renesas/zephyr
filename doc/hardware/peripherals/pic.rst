.. _pic_rh850:

Peripheral Interconnect Controller (PIC), Renesas RH850
#######################################################

Overview
********

The Peripheral Interconnect Controller (PIC) routes internal signals between
peripherals on supported Renesas RH850 U2x MCUs. For example, it routes TAUD
three-phase PWM outputs and trigger signals that start an ADC scan group.

The PIC has no run-time API. It is configured through devicetree, and the
driver applies the configuration once at boot. The driver is SoC-specific and
is located in ``soc/renesas/rh850/common_fsp/pic/``. It does not provide a
public API header. Applications can only check that the PIC is ready with
:c:func:`device_is_ready`.

Peripheral drivers do not configure the PIC directly. The PIC configuration
and the peripheral configuration are associated through devicetree.

Configuration
*************

Enable the PIC node and define the properties required by the board or
application. An omitted optional property does not select that function.

.. code-block:: devicetree

   #include <zephyr/dt-bindings/pic/renesas/pic-rh850-common.h>

   &pic0 {
           taud-input-control-function =
                   "high-accuracy-triangle-wave-pwm-with-dead-time";
           adc-trigger-selection =
                   <RH850_PIC_ADC_TRIG(ADC0, SG0, RISING, TAUD0_CH2)>;
           status = "okay";
   };

The ``pic0`` node is disabled by default. It is an opt-in feature: the
application enables the node in its overlay and selects the required function.
The board devicetree can provide default settings for the node.

``taud-input-control-function``
   Selects one chip-wide TAUD input-control function. Supported values are
   ``"none"``,
   ``"timer-input-select"``,
   ``"pwm-output-with-dead-time"``,
   ``"high-accuracy-triangle-wave-pwm-with-dead-time"``,
   ``"delay-pulse-output-with-dead-time"``, and
   ``"trigger-pulse-width-measurement"``.

   Select ``"high-accuracy-triangle-wave-pwm-with-dead-time"`` when using the
   RH850 TAUD3P PWM driver (:ref:`pwm3p_api`). This driver version configures
   additional settings only for that function.

``triangle-wave-pwm``
   Configures triangle-wave PWM outputs. Each entry uses
   ``RH850_PIC_TRIWAVE(unit, phases, level)``.

   * ``unit`` is ``TAUD0`` through ``TAUD3``.
   * ``phases`` is ``U``, ``V``, ``W``, ``UV``, ``UW``, ``VW``, or ``UVW``.
   * ``level`` is ``NONE``, ``HIGH``, or ``LOW``.

   A phase that is not listed is disabled. If an application defines this
   property again, the new list replaces the list from the board devicetree.
   Therefore, include every required phase in the overriding list.

``taud-int-signal``
   Selects the TAUD channel used as the INT signal output. Each entry uses
   ``RH850_PIC_TAUD_INT(unit, ch)``, where ``ch`` is ``CH0``, ``CH2``, or
   ``CH8``. A TAUD unit can have at most one entry.

``adc-trigger-selection``
   Configures ADC hardware-trigger routes. Each entry uses
   ``RH850_PIC_ADC_TRIG(adc, sg, edge, src)``.

   * ``adc`` is ``ADC0`` through ``ADC3`` and must match the
     ``renesas,unit`` value of the target ADC node.
   * ``sg`` is ``SG0`` through ``SG4``, or ``DIAG`` on MCUs that provide a
     diagnostic scan group. It must match the ``renesas,scan-group`` value of
     the target ADC node.
   * ``edge`` is ``RISING``, ``FALLING``, or ``BOTH``.
   * ``src`` is ``TAUD<unit>_CH<channel>`` for TAUD units 0 through 3 and
     channels 0 through 15, or another source defined in
     ``pic-rh850-common.h``.

   A scan group can use multiple trigger sources. The driver validates the
   encoded entries at build time.

Linking an ADC node to a PIC route
==================================

An ADC node is not linked to the PIC with a phandle. The encoded PIC entry
identifies the ADC unit and scan group. These values must match
``renesas,unit`` and ``renesas,scan-group`` in the ADC node.

.. code-block:: devicetree

   &adck0 {
           renesas,unit = <0>;
           renesas,scan-group = <0>;
           renesas,data-format = "12-bit-unsigned-right-justified";
           renesas,trigger-method = "hardware";
           renesas,adck0-trigger-source = "pic2";
           status = "okay";
   };

   &adck1 {
           renesas,unit = <1>;
           renesas,scan-group = <1>;
           renesas,data-format = "12-bit-unsigned-right-justified";
           renesas,trigger-method = "hardware";
           renesas,adck0-trigger-source = "pic2";
           status = "okay";
   };

   &pic0 {
           adc-trigger-selection =
                   <RH850_PIC_ADC_TRIG(ADC0, SG0, FALLING, TAUD0_CH4)>,
                   <RH850_PIC_ADC_TRIG(ADC1, SG1, FALLING, TAUD1_CH4)>;
           status = "okay";
   };

The PIC configures only the signal route. The ADC driver must also configure
the corresponding ADC scan group to use a hardware trigger.

Adding an ADC trigger from a TAUD3P PWM
=======================================

To start an ADC scan group from the PWM of a TAUD3P instance, add a route whose
source is a channel of the TAUD unit used by that instance, and enable the
instance. The ADC scan group is then started by the selected TAUD channel.

.. code-block:: devicetree

   &pic0 {
           adc-trigger-selection =
                   <RH850_PIC_ADC_TRIG(ADC0, SG0, FALLING, TAUD0_CH4)>;
           status = "okay";
   };

   &taud3p0 {
           status = "okay";
   };

The TAUD unit of the source must be the ``taud-three-phase-unit`` of the
TAUD3P node.

Configuration rules
*******************

* All trigger entries for the same ADC unit and scan group must use the same
  edge. Use ``BOTH`` instead of separate ``RISING`` and ``FALLING`` entries.
* The TAUD channel selection for a TAUD unit and scan group is shared by all
  ADC units. ADC units using the same TAUD unit and scan group must therefore
  select the same TAUD channels.
* Repeating an identical entry is harmless because source selections are
  combined with bitwise OR.
* An overlay that defines a list property replaces the earlier list. Keep all
  required routes in one complete property value.
* Sources that require additional routing stages work only when those stages
  are configured and supported by the driver.

Build-time validation
*********************

The driver checks the following configuration constraints at build time:

* The selected ADC unit, scan group, TAUD unit, and source group exist on the target MCU.
* An entry is created by the macro associated with its property.
* One ADC scan group does not use conflicting edge selections.
* ADC units sharing a TAUD unit and scan group use the same TAUD channels.
* One triangle-wave phase does not use conflicting active levels.
* One TAUD unit does not select multiple INT-signal channels.
* The dt-binding source encoding remains compatible with the values of the
  underlying hardware support library.

A misspelled macro argument or an invalid token is rejected while processing
the devicetree source. The diagnostic can identify the source file, line, and
column rather than the enclosing binding macro.

Initialization order
********************

The PIC must be initialized before the devices that depend on it. A TAUD3P
node refers to the PIC with its ``pic`` property, so the device initialization
order follows the devicetree dependency. The TAUD3P driver also checks that the
PIC device is ready, and a build-time check requires
:kconfig:option:`CONFIG_RENESAS_PIC_INIT_PRIORITY` to not be greater than
:kconfig:option:`CONFIG_PWM_INIT_PRIORITY`.

If the PIC initialization fails, the PIC device is not ready and the driver
reports the error. The PIC is never closed because closing it would reset the
routing configuration shared by the peripherals.

Limitations
***********

* The PIC10 function selection is chip-wide and cannot be changed at run time.
* This driver supports only the settings documented above. Other PIC
  functions require additional driver and binding support.

Configuration options
*********************

* :kconfig:option:`CONFIG_RENESAS_PIC`
* :kconfig:option:`CONFIG_RENESAS_RH850_PIC`
* :kconfig:option:`CONFIG_RENESAS_PIC_INIT_PRIORITY`
