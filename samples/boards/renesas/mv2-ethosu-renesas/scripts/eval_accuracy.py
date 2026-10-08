#!/usr/bin/env python3
# Copyright 2026 Renesas Electronics Corporation
#
# SPDX-License-Identifier: Apache-2.0
#
"""Top-1 accuracy of mv2_a05: FP32 vs INT8 (PT2E fake-quant) vs INT8 (TOSA
reference model), on an imagenette val folder, using the app's preprocessing
(center crop, nearest resize, RGB565 round trip, (v - 128) / 128).

The INT8 model is re-quantized here with the same calibration data; it is not
the exact .pte (output scale is not reproducible across runs). The result
shows the quantization loss, not the NPU's bit-exact output.
"""

import argparse
import csv
import time
from pathlib import Path

import torch
from timm.data import ImageNetInfo

import export_quantized_io as exp
from gen_calibration_data import SUPPORTED_SUFFIXES, image_to_calibration_tensor

ref = exp.ref

# imagenette wnid -> ImageNet-1k class index
IMAGENETTE_TO_IMAGENET = {
    "n01440764": 0,  # tench
    "n02102040": 217,  # English springer
    "n02979186": 482,  # cassette player
    "n03000684": 491,  # chain saw
    "n03028079": 497,  # church
    "n03394916": 566,  # French horn
    "n03417042": 569,  # garbage truck
    "n03425413": 571,  # gas pump
    "n03445777": 574,  # golf ball
    "n03888257": 701,  # parachute
}


def load_val_set(val_dir, per_class):
    samples = []
    for wnid, label in IMAGENETTE_TO_IMAGENET.items():
        images = sorted(
            p for p in (val_dir / wnid).iterdir() if p.suffix.lower() in SUPPORTED_SUFFIXES
        )
        for p in images[:per_class]:
            samples.append((f"val/{wnid}/{p.name}", image_to_calibration_tensor(p), label))
    return samples


def accuracy(name, fn, samples, preds):
    """Print top-1 and store each image's predicted ImageNet index in preds."""
    top1 = top1_10 = 0
    classes = list(IMAGENETTE_TO_IMAGENET.values())
    start = time.time()
    preds[name] = []
    for _, x, label in samples:
        logits = fn(x).flatten().float()
        pred = int(logits.argmax())
        preds[name].append(pred)
        top1 += pred == label
        top1_10 += classes[int(logits[classes].argmax())] == label
    n = len(samples)
    print(
        f"{name:<22} top-1 (1000 cls) {100 * top1 / n:6.2f}%   "
        f"top-1 (10 cls) {100 * top1_10 / n:6.2f}%   "
        f"[{n} images, {time.time() - start:.0f}s]"
    )


def run_tosa(exported_quant, compile_spec, samples, preds):
    from executorch.backends.arm.test.runner_utils import TosaReferenceModelDispatch

    edge = ref.to_edge_transform_and_lower(
        exported_quant,
        partitioner=[ref.create_partitioner(compile_spec)],
        compile_config=ref.EdgeCompileConfig(_check_ir_validity=False),
    )
    lowered = edge.exported_program().module()

    with torch.no_grad(), TosaReferenceModelDispatch():
        accuracy("int8_tosa", lambda x: lowered(x), samples, preds)


def write_csv(path, samples, preds):
    """One row per image, same layout as imagenette's noisy_imagenette.csv:
    path, ground-truth wnid, one predicted wnid column per model, is_valid.
    """
    wnids = ImageNetInfo().label_names()
    names = list(preds)
    with open(path, "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["path", "label"] + names + ["is_valid"])
        for i, (rel, _, label) in enumerate(samples):
            w.writerow(
                [rel, wnids[label]] + [wnids[preds[n][i]] for n in names] + ["True"]
            )
    print(f"Wrote {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--val_dir", required=True, help="imagenette2-160/val")
    parser.add_argument("--calibration_data", required=True, help="folder of .pt files")
    parser.add_argument("--per_class", type=int, default=50, help="images per class")
    parser.add_argument("--target", default="TOSA-1.0+INT")
    parser.add_argument("--output_csv", help="write per-image predictions to this CSV")
    parser.add_argument("--tosa", action="store_true", help="also run the (slow) TOSA model")
    args = parser.parse_args()

    samples = load_val_set(Path(args.val_dir), args.per_class)

    model, example_inputs = exp._load_mv2_alpha05(None)
    model = model.eval()
    model.requires_grad_(False)
    calibration = ref.load_calibration_samples(args.calibration_data, example_inputs)

    preds = {}
    with torch.no_grad():
        accuracy("fp32", lambda x: model(x), samples, preds)

    from executorch.backends.arm.tosa import TosaSpecification
    from executorch.backends.arm.tosa.compile_spec import TosaCompileSpec

    compile_spec = TosaCompileSpec(TosaSpecification.create_from_string(args.target))
    exported = torch.export.export(model, example_inputs, strict=False)
    model_quant, exported_quant = ref.quantize_model(
        exported.module(check_guards=False),
        example_inputs,
        compile_spec,
        "mv2_a05",
        False,
        ref.QuantMode.INT8,
        calibration,
    )

    with torch.no_grad():
        accuracy("int8_pt2e", lambda x: model_quant(x), samples, preds)

    if args.tosa:
        run_tosa(exported_quant, compile_spec, samples, preds)

    if args.output_csv:
        write_csv(args.output_csv, samples, preds)


if __name__ == "__main__":
    main()
