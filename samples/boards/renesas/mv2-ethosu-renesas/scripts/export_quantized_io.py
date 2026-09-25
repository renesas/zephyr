#!/usr/bin/env python3
# Copyright 2026 Renesas Electronics Corporation
#
# SPDX-License-Identifier: Apache-2.0
#

import os
import sys
from pathlib import Path

import timm
import torch

import executorch

_EXECUTORCH_ROOT = Path(next(iter(executorch.__path__))).resolve().parent.parent
sys.path.insert(0, str(_EXECUTORCH_ROOT / "backends" / "arm" / "scripts"))

import aot_arm_compiler as ref  # noqa: E402


def _load_mv2_alpha05(model_input):
    model = timm.create_model("mobilenetv2_050", pretrained=True)
    example_inputs = ref._load_example_inputs(model_input)
    if example_inputs is None:
        example_inputs = (torch.randn(1, 3, 224, 224),)
    return model, example_inputs


def _get_model_and_inputs(model_name, model_input):
    if model_name == "mv2_a05":
        return _load_mv2_alpha05(model_input)
    return ref.get_model_and_inputs_from_name(model_name, model_input)


def to_edge_quantized_io(
    target,
    exported_program,
    compile_spec,
    model,
    quant_mode,
    example_inputs,
    model_name,
    strict_export,
    calibration_samples,
    direct_drive,
):
    model_quant = None
    if quant_mode is not None:
        model_quant, exported_program = ref.quantize_model(
            model,
            example_inputs,
            compile_spec,
            model_name,
            strict_export,
            quant_mode,
            calibration_samples,
        )

    partitioner = ref.create_partitioner(compile_spec)
    edge = ref.to_edge_transform_and_lower(
        exported_program,
        partitioner=[partitioner],
        compile_config=ref.EdgeCompileConfig(_check_ir_validity=False),
    )

    from executorch.exir.passes.quantize_io_pass import (
        QuantizeInputs,
        QuantizeOutputs,
    )

    exported = edge.exported_program()
    input_name = exported.graph_signature.user_inputs[0]
    input_placeholder = next(
        n
        for n in exported.graph_module.graph.nodes
        if n.op == "placeholder" and n.name == input_name
    )
    input_quantize_node = next(iter(input_placeholder.users))
    output_dequantize_node = exported.graph_module.graph.output_node().args[0][0]
    quantize_io_info = {
        "input0": tuple(input_quantize_node.args[1:]),
        "output0": tuple(output_dequantize_node.args[1:]),
    }

    edge = edge.transform(
        passes=[QuantizeInputs(edge, [0]), QuantizeOutputs(edge, [0])]
    )
    edge = ref._apply_replace_quant_nodes(edge, target, direct_drive)

    return model_quant, edge, quantize_io_info


def main():
    args = ref._get_args()
    if not args.delegate or args.target.startswith("cortex-m"):
        raise RuntimeError(
            "export_quantized_io.py only supports the --delegate path "
            f"(got target={args.target!r}, delegate={args.delegate})."
        )

    original_model, example_inputs = _get_model_and_inputs(
        args.model_name, args.model_input
    )
    calibration_samples = ref.load_calibration_samples(
        args.calibration_data, example_inputs
    )
    model = original_model.eval()
    model.requires_grad_(False)

    exported_program = torch.export.export(
        model, example_inputs, strict=args.strict_export
    )
    model = exported_program.module(check_guards=False)
    model_name = os.path.basename(os.path.splitext(args.model_name)[0])

    quant_mode = None
    if args.quantize:
        quant_mode = ref.QuantMode.A16W8 if "int16" in args.target else ref.QuantMode.INT8

    model_quant, edge, quantize_io_info = to_edge_quantized_io(
        args.target,
        exported_program,
        ref._get_compile_spec(args),
        model,
        quant_mode,
        example_inputs,
        args.model_name,
        args.strict_export,
        calibration_samples,
        args.direct_drive,
    )

    ref.dump_delegation_info(edge, args.intermediates)

    for label, key, op in (
        ("input", "input0", "quantize"),
        ("output", "output0", "dequantize"),
    ):
        scale, zero_point, qmin, qmax, dtype = quantize_io_info[key]
        print(
            f"Quantized {label} 0 in place of the boundary {op} op. "
            f"scale={scale}, zero_point={zero_point}, "
            f"clamped to [{qmin}, {qmax}] (scalar_type_enum={dtype})."
        )

    exec_prog = edge.to_executorch(
        config=ref.ExecutorchBackendConfig(extract_delegate_segments=False)
    )

    output_path = args.output or f"{model_name}_arm_delegate_{args.target}.pte"
    ref.save_pte_program(exec_prog, output_path)
    print(f"PTE file saved as {output_path}")


if __name__ == "__main__":
    main()
