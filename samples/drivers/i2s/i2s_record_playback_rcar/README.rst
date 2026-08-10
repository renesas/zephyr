.. zephyr:code-sample:: i2s_record_playback_rcar
   :name: I2S record and playback R-Car
   :relevant-api: i2s_interface

   Record an audio stream from a codec and play it back.

Overview
********

This sample is used to test the R-Car Gen5 I2S driver by capturing audio from a
codec into a RAM buffer and then playing the same buffer back through the codec.

The sample is console driven:

* Type ``r`` to record ``CONFIG_RECORD_TIME`` seconds of audio (I2S RX).
* Type ``p`` to play back the most recently recorded audio (I2S TX).
* Type ``e`` to exit the sample.

Requirements
************

This sample only supports the ``rcar_ironhide_x5h/r8a78000/r52`` board, which
provides the R-Car Gen5 SSI I2S controller wired to an on-board AK4619 codec.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/i2s/i2s_record_playback_rcar
   :board: rcar_ironhide_x5h/r8a78000/r52
   :goals: build flash
   :compact:

After flashing, follow the console prompts to record and play back audio.
