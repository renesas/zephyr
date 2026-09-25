# MobileNetV2 Live Camera Classification (Renesas EK-RA8P1)

Captures live OV5640 camera frames, classifies them with MobileNetV2 on the
Ethos-U55 NPU via ExecuTorch, and shows the camera feed plus the top-5
labels (1000-class ImageNet) on the MIPI-DSI display.

## Prepare a PTE model file

`--model_name` options:
- `mv2_a05` (recommended) - pretrained MobileNetV2 alpha=0.5 (timm's
  `mobilenetv2_050`; torchvision ships no ImageNet checkpoint for this
  width). Handled directly in `export_quantized_io.py`, not registered in
  executorch's `examples/models`.
- `mv2` - pretrained full-width MobileNetV2 (torchvision).
- `mv2_untrained` - random weights, meaningless classifications; don't use.

### Generate calibration data (recommended)

Without `--calibration_data`, quantization calibrates against a
`torch.randn(1,3,224,224)` example input, i.e. random noise, not anything
resembling a real camera frame. The PTQ observers then pick scale/zero-point
values matched to noise statistics (~N(0,1)) instead of the actual runtime
input range (`(uint8-128)/128`, i.e. [-1,1)), which starves the INT8
quantization of precision on real images and produces classifications that
don't track what's actually in frame. Fix: calibrate on real photos,
preprocessed exactly like the on-device pipeline:

```
python samples/boards/renesas/mv2-ethosu-renesas/scripts/gen_calibration_data.py \
    --input <folder of real photos (.jpg/.png/...)> \
    --output calib_data/
```
A few dozen to a few hundred varied photos (different objects/scenes) is
enough - no labels needed, calibration is unsupervised (it only records
activation ranges, never checks predictions against ground truth).

### Export

```
python samples/boards/renesas/mv2-ethosu-renesas/scripts/export_quantized_io.py \
    --model_name=mv2_a05 --quantize --delegate --target=ethos-u55-256 \
    --calibration_data calib_data/ --output=mv2_a05_u55_256_int8.pte
```

Copy the printed `scale`/`zero_point` values into the board `.conf`'s
`CONFIG_AI_INPUT_QUANT_SCALE`/`ZERO_POINT` and
`CONFIG_AI_OUTPUT_QUANT_SCALE`/`ZERO_POINT` after every re-export - the
runtime can't read them back from the `.pte`, and output's values aren't
stable across export runs.

E.g.

```
...
Delegation summary:
  Model was partially delegated.
  Delegated partitions for silicon acceleration: 1
  Non-delegated ops: 1
  Non-delegated operators:
    - getitem: 1
Quantized input 0 in place of the boundary quantize op. scale=0.007808685302734375, zero_point=0, clamped to [-128, 127] (scalar_type_enum=torch.int8).
Quantized output 0 in place of the boundary dequantize op. scale=0.10048583149909973, zero_point=127, clamped to [-128, 127] (scalar_type_enum=torch.int8).
PTE file saved as mv2_a05_u55_256_int8.pte
```

```
# USER NEED UPDATE AFTER EACH TIME EXPORT MODEL
CONFIG_AI_INPUT_QUANT_SCALE="0.007808685302734375"
CONFIG_AI_INPUT_QUANT_ZERO_POINT=0
CONFIG_AI_OUTPUT_QUANT_SCALE="0.10048583149909973"
CONFIG_AI_OUTPUT_QUANT_ZERO_POINT=127
```

The model file `mv2_a05_u55_256_int8.pte` is also included in the app
with the default `CONFIG_AI_INPUT_QUANT_SCALE`/`ZERO_POINT`.

## Manifest / build

```
west config manifest.project-filter -- '-.*,+zephyr,+executorch,+cmsis,+cmsis_6,+cmsis-nn,+hal_ethos_u'
west update
```

```
west build -b ek_ra8p1/r7ka8p1kflcac/cm85 samples/boards/renesas/mv2-ethosu-renesas \
    --shield rtklcdpar1s00001be --shield arducam_cu450_ov5640_mipi_csi -- \
    -DET_PTE_FILE_PATH=samples/boards/renesas/mv2-ethosu-renesas/mv2_a05_u55_256_int8.pte
```

Must flash `zephyr_merged.hex`, not the plain `zephyr.hex`. The model section
lives in OSPI flash and the board's OSPI controller reads bytes back in
swapped order when enter 8D-8D-8D on ek_ra8p1, so the `split_flash` CMaketarget
(see `CMakeLists.txt`) byte-swaps just that section and remerges it
into `zephyr_merged.hex`. Flashing `zephyr.hex` instead leaves the model
data scrambled at runtime.

When flashing `zephyr_merged.hex`, you need to use the RFP application and enable external flash on the ek_ra8p1.

## Notes

- Dave2D GPU acceleration is enabled (see the board `.conf`'s
  `CONFIG_LV_USE_DRAW_DAVE2D` block) for LVGL's full-panel compositing.
- The camera's video buffer pool and the LVGL display buffer (VDB) are
  routed to external SDRAM (`CONFIG_VIDEO_BUFFER_POOL_ZEPHYR_REGION`,
  `CONFIG_LV_Z_VDB_ZEPHYR_REGION`) - on-chip SRAM alone can't hold them
  alongside everything else. The ExecuTorch method/temp allocator pools
  stay in on-chip RAM (`.bss.method_pool`/`.bss.tensor_arena`, see
  `ai_processing.cpp`).
- The board overlay reclaims the CM33 core's flash/SRAM share for this
  single-core app.
