.. zephyr:code-sample:: tflite-face-detection
   :name: Face Detection

   Face detection using TensorFlow Lite for Microcontrollers.

Overview
********

This sample demonstrates a real-time face detection application
using TensorFlow Lite for Microcontrollers running on Zephyr RTOS.

The sample application runs a model that has been downloaded from the
`Emza Visual Sense Model zoo <https://github.com/emza-vs/ModelZoo/tree/master/object_detection>`_. This model has then
been optimized using the
`Vela compiler <https://git.mlplatform.org/ml/ethos-u/ethos-u-vela.git>`_.

Vela takes a tflite file as input and produces another tflite file as output,
where the operators supported by Ethos-U have been replaced by an Ethos-U custom
operator. In an ideal case the complete network would be replaced by a single
Ethos-U custom operator.

The application captures image frames from an Arducam OV5640 camera,
runs a TensorFlow Lite Micro face detection model, and displays the
results on a MIPI LCD display connected to the Renesas EK-RA8P1
development kit.

The sample highlights an end-to-end embedded AI workflow, including:

Camera capture and image preprocessing

TensorFlow Lite Micro inference

Displaying detection results on a MIPI display

Running a memory-intensive ML workload on a high-performance MCU

This sample is intended for boards with sufficient external memory
(SDRAM) and camera/display support.

Generating Vela-compiled model
******************************

Follow the steps below to generate Vela-compiled model and test input/output data.
Use `yolo-fastest_192_face_v4`_ model in this sample:

.. _yolo-fastest_192_face_v4: https://github.com/emza-vs/ModelZoo/tree/master/object_detection

1. Downloading the files below from `yolo-fastest_192_face_v4`_:

   - yolo-fastest_192_face_v4.tflite

2. Optimizing the model for Ethos-U using Vela

   Assuming target Ethos-U is U55 and 256 MAC:

   .. code-block:: console

       $ vela yolo-fastest_192_face_v4.tflite \
       --output-dir . \
       --accelerator-config ethos-u55-256 \
       --system-config Ethos_U55_High_End_Embedded \
       --memory-mode Shared_Sram

3. Converting to C array

   .. code-block:: console

       $ xxd -c 16 -i yolo-fastest_192_face_v4_vela.tflite yolo-fastest_192_face_v4_vela.tflite.h

4. Synchronize the generated header file

   Synchronize the files below to ``ai_application/face_detection/model`` in this sample

   - yolo-fastest_192_face_v4_vela.tflite.h > model_data.h

Hardware Requirements
*********************

The sample is designed and validated on the following hardware:

Renesas EK-RA8P1 Evaluation Kit

Arducam CMOS OV5640 Camera Module

RTKMIPILCDB00000BE MIPI Graphics Expansion Shield

Board Setup
***********

Connect and configure the hardware as follows for EK-RA8P1 board:

+------------+----------------------------------------------------+
| Component  | Connection / Setting                               |
+============+====================================================+
| Camera     | Camera Expansion Board → Camera Port (JP35)        |
+------------+----------------------------------------------------+
| Display    | MIPI Graphics Shield → MIPI Port (JP32)            |
+------------+----------------------------------------------------+
| SW4        | SW4-1 to SW4-8 OFF, except SW4-6 → ON              |
+------------+----------------------------------------------------+

Building and Running
********************

This sample requires the tflite-micro module to be present in your
west workspace.

Add the TensorFlow Lite Micro module and update your workspace:

.. code-block:: console

    $ west config manifest.project-filter -- +tflite-micro
    $ west update


Build the sample for the FVP:

Build the application for the EK-RA8P1 with the required camera and
display shields:

.. zephyr-app-commands::
   :zephyr-app: samples/modules/tflite-micro/face_detection_renesas
   :board: ek_ra8p1/r7ka8p1kflcac/cm85
   :goals: build flash
   :shield: rtkmipilcdb00000be,arducam_cu450_ov5640_dvp

.. code-block:: console

    $ west build -b ek_ra8p1/r7ka8p1kflcac/cm85 --shield rtkmipilcdb00000be --shield arducam_cu450_ov5640_dvp -p always samples/modules/tflite-micro/face_detection_ethos/

A successful build produces a memory usage summary similar to:

Flash the application to the board:

.. code-block:: console

    $ west flash

Sample Output
=============

After flashing and reset:

The camera begins capturing frames

Face detection inference runs continuously

Detection results are rendered on the MIPI LCD display, face bounding boxes are shown

Detected face coordinates are logged via UART console

.. code-block:: console

    *** Booting Zephyr OS build v4.3.0-2617-g7b8d720f677f ***
    [00:00:00.407,000] <inf> main: Zephyr Face Detection sample app!
    [00:00:00.407,000] <inf> camera: - Device name: ceu@40348000
    [00:00:00.425,000] <inf> camera: Set CID=0x980900 val=3 ret=0
    [00:00:00.425,000] <inf> camera: - Camera initialized and capture started
    [00:00:00.428,000] <inf> app_display: - Display initialized
    [00:00:00.505,000] <inf> ai_processing: x=125, y=19, w=58, h=66
    
    [00:00:00.550,000] <inf> ai_processing: x=125, y=20, w=58, h=66
    
    [00:00:00.595,000] <inf> ai_processing: x=126, y=18, w=58, h=66
    
    [00:00:00.685,000] <inf> ai_processing: x=120, y=19, w=70, h=66
    
    [00:00:00.729,000] <inf> ai_processing: x=120, y=19, w=70, h=66
    
    ....
