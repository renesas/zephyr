.. _msc_test:

MSC Host Driver Test Sample
############################

Overview
********

The test waits a while for an USB drive to be connected and then tries to mount it
and do some file operations, verifying their outcome.

The test can be ran either on the `native_sim` target (in which case all communication
passes through the virtual USB stack) or the `ek_ra8m2` development kit.

If executed on the development kit one should connect a USB thumb drive (or similar
MSC device) to the USB HS interface. On the simulated target on the other hand
a virtual drive is created with two logical units.

Limitations
***********

For some USB devices the test fails because they freeze upon receiving a CSW request.
Communication is unable to be restored. This only occurs when connected to the USBHS port.
Upon switching to the more recent `wip_ra_usbh` branch this issue was fixed.
