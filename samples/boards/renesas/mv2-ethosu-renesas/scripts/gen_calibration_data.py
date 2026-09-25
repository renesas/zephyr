#!/usr/bin/env python3
# Copyright 2026 Renesas Electronics Corporation
#
# SPDX-License-Identifier: Apache-2.0

import argparse
from pathlib import Path

import numpy as np
import torch
from PIL import Image, ImageOps

AI_INPUT_SIZE = 224
SUPPORTED_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def center_crop_square(img: Image.Image) -> Image.Image:
    w, h = img.size
    side = min(w, h)
    left = (w - side) // 2
    top = (h - side) // 2
    return img.crop((left, top, left + side, top + side))


def nearest_neighbor_resize(arr: np.ndarray, out_size: int) -> np.ndarray:
    in_size = arr.shape[0]  # square input (in_size x in_size)
    idx = (np.arange(out_size, dtype=np.int64) * in_size) // out_size
    return arr[idx][:, idx]


def simulate_rgb565_roundtrip(r8, g8, b8):
    """Replicate the OV5640's RGB565 output and the proportional-scaling
    formula in camera.c's image_rgb565_to_planar_rgb888() (r*255/31 etc,
    not bit replication) - the real camera pipeline loses precision to
    5/6/5 bits before the app ever sees 8-bit values, so calibration should
    see that same quantization, not full 8-bit source precision."""
    r5 = r8 >> 3
    g6 = g8 >> 2
    b5 = b8 >> 3
    r8q = (r5 * 255) // 31
    g8q = (g6 * 255) // 63
    b8q = (b5 * 255) // 31
    return r8q, g8q, b8q


def image_to_calibration_tensor(image_path: Path) -> torch.Tensor:
    img = Image.open(image_path)
    img = ImageOps.exif_transpose(img)  # respect camera orientation metadata
    img = img.convert("RGB")
    img = center_crop_square(img)

    arr = np.array(img, dtype=np.uint32)  # (side, side, 3), 0-255
    arr = nearest_neighbor_resize(arr, AI_INPUT_SIZE)  # (224, 224, 3)

    r8q, g8q, b8q = simulate_rgb565_roundtrip(arr[..., 0], arr[..., 1], arr[..., 2])

    planar = np.stack([r8q, g8q, b8q], axis=0).astype(np.float32)  # (3, 224, 224)
    normalized = (planar - 128.0) / 128.0  # matches ai_processing.cpp ai_task()

    tensor = torch.from_numpy(normalized).unsqueeze(0)  # (1, 3, 224, 224)
    return tensor.contiguous()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="Folder of source images")
    parser.add_argument("--output", required=True, help="Folder to write .pt files to")
    args = parser.parse_args()

    input_dir = Path(args.input)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    images = sorted(
        p for p in input_dir.rglob("*") if p.suffix.lower() in SUPPORTED_SUFFIXES
    )
    if not images:
        raise SystemExit(f"No images with suffixes {SUPPORTED_SUFFIXES} found in {input_dir}")

    for i, image_path in enumerate(images):
        tensor = image_to_calibration_tensor(image_path)
        out_path = output_dir / f"calib_{i:04d}.pt"
        torch.save(tensor, out_path)
        print(f"[{i + 1}/{len(images)}] {image_path.name} -> {out_path} {tuple(tensor.shape)}")

    print(f"\nWrote {len(images)} calibration samples to {output_dir}")


if __name__ == "__main__":
    main()
