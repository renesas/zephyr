.. zephyr:code-sample:: display
   :name: Display
   :relevant-api: display_interface

   Draw basic rectangles on a display device.

Overview
********

This sample will draw some basic rectangles onto the display.
The rectangle colors and positions are chosen so that you can check the
orientation of the LCD and correct RGB bit order. The rectangles are drawn
in clockwise order, from top left corner: blue, lime, red, black, white, yellow, aqua, magenta.

Building and Running
********************

As this is a generic sample it should work with any display supported by Zephyr.

Below is an example on how to build for a :ref:`ek_ra8d1`
.. zephyr-app-commands::
   :zephyr-app: samples/shields/rtkmipilcdb00000be_ra_display
   :board: ek_ra8d1
   :goals: build
   :shield: rtkmipilcdb00000be
   :compact:
