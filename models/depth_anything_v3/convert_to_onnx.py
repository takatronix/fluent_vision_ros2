#!/usr/bin/env python3
"""Convert Depth Anything V3 safetensors model to ONNX format.

Usage:
    pip install torch onnx depth-anything-3
    python convert_to_onnx.py --model depth-anything/DA3-SMALL --output DA3-SMALL.onnx
    python convert_to_onnx.py --model ./DA3-SMALL --output DA3-SMALL.onnx
"""
import argparse
import os

import torch


def main():
    parser = argparse.ArgumentParser(description="Convert DA3 to ONNX")
    parser.add_argument("--model", type=str, required=True,
                        help="HuggingFace model ID or local directory (e.g., depth-anything/DA3-SMALL or ./DA3-SMALL)")
    parser.add_argument("--output", type=str, required=True,
                        help="Output ONNX file path")
    parser.add_argument("--input-size", type=int, default=518,
                        help="Input resolution (default: 518)")
    parser.add_argument("--opset", type=int, default=17,
                        help="ONNX opset version")
    args = parser.parse_args()

    from depth_anything_3 import DepthAnything3

    print(f"Loading DA3 model from: {args.model}")
    da3 = DepthAnything3.from_pretrained(args.model)
    da3.model.eval()

    # Extract just the monocular inference path as a wrapper
    class DA3MonoWrapper(torch.nn.Module):
        def __init__(self, da3_model):
            super().__init__()
            self.model = da3_model

        def forward(self, pixel_values):
            # DA3 monocular: single image input
            return self.model.infer_mono(pixel_values)

    wrapper = DA3MonoWrapper(da3)
    wrapper.eval()

    dummy = torch.randn(1, 3, args.input_size, args.input_size, dtype=torch.float32)

    print(f"Exporting to ONNX: {args.output} (opset {args.opset})")
    torch.onnx.export(
        wrapper,
        (dummy,),
        args.output,
        input_names=["pixel_values"],
        output_names=["predicted_depth"],
        dynamic_axes={
            "pixel_values": {0: "batch_size"},
            "predicted_depth": {0: "batch_size"},
        },
        opset_version=args.opset,
        do_constant_folding=True,
    )

    import onnx
    onnx_model = onnx.load(args.output)
    onnx.checker.check_model(onnx_model)
    size_mb = os.path.getsize(args.output) / (1024 * 1024)
    print(f"ONNX model saved: {args.output} ({size_mb:.1f} MB)")
    print("Verification passed!")


if __name__ == "__main__":
    main()
