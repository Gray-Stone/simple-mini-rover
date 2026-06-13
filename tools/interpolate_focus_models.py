#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import mrcal
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interpolate a provisional mrcal camera model between two focus-calibrated anchors."
    )
    parser.add_argument("--low-focus", type=float, required=True)
    parser.add_argument("--low-model", type=Path, required=True)
    parser.add_argument("--high-focus", type=float, required=True)
    parser.add_argument("--high-model", type=Path, required=True)
    parser.add_argument("--target-focus", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--valid-region-source",
        choices=("nearest", "low", "high", "none"),
        default="nearest",
        help="Which anchor valid_intrinsics_region to copy into the provisional model.",
    )
    return parser.parse_args()


def choose_valid_region(
    args: argparse.Namespace,
    low_model: mrcal.cameramodel,
    high_model: mrcal.cameramodel,
) -> np.ndarray | None:
    if args.valid_region_source == "none":
        return None
    if args.valid_region_source == "low":
        return low_model.valid_intrinsics_region()
    if args.valid_region_source == "high":
        return high_model.valid_intrinsics_region()

    low_distance = abs(args.target_focus - args.low_focus)
    high_distance = abs(args.high_focus - args.target_focus)
    if low_distance <= high_distance:
        return low_model.valid_intrinsics_region()
    return high_model.valid_intrinsics_region()


def main() -> int:
    args = parse_args()
    if not args.low_focus < args.high_focus:
        raise ValueError("--low-focus must be less than --high-focus")
    if not args.low_focus <= args.target_focus <= args.high_focus:
        raise ValueError("--target-focus must lie between --low-focus and --high-focus")

    low = mrcal.cameramodel(str(args.low_model))
    high = mrcal.cameramodel(str(args.high_model))
    low_lensmodel, low_intrinsics = low.intrinsics()
    high_lensmodel, high_intrinsics = high.intrinsics()

    if low_lensmodel != high_lensmodel:
        raise ValueError(f"lensmodel mismatch: {low_lensmodel!r} vs {high_lensmodel!r}")

    low_size = np.asarray(low.imagersize(), dtype=np.int32)
    high_size = np.asarray(high.imagersize(), dtype=np.int32)
    if np.any(low_size != high_size):
        raise ValueError(f"imagersize mismatch: {tuple(low_size)} vs {tuple(high_size)}")

    t = (args.target_focus - args.low_focus) / (args.high_focus - args.low_focus)
    interpolated_intrinsics = (1.0 - t) * np.asarray(low_intrinsics) + t * np.asarray(high_intrinsics)
    valid_region = choose_valid_region(args, low, high)

    model = mrcal.cameramodel(
        intrinsics=(low_lensmodel, interpolated_intrinsics),
        imagersize=low_size,
        valid_intrinsics_region=valid_region,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    model.write(str(args.output))

    metadata = {
        "kind": "interpolated_focus_model",
        "note": "Quick-test provisional model only. Do not use for final geometry without validation.",
        "low_focus": args.low_focus,
        "low_model": str(args.low_model.resolve()),
        "high_focus": args.high_focus,
        "high_model": str(args.high_model.resolve()),
        "target_focus": args.target_focus,
        "t": t,
        "lensmodel": low_lensmodel,
        "imagersize": low_size.tolist(),
        "valid_region_source": args.valid_region_source,
        "intrinsics": {
            "fx_px": float(interpolated_intrinsics[0]),
            "fy_px": float(interpolated_intrinsics[1]),
            "cx_px": float(interpolated_intrinsics[2]),
            "cy_px": float(interpolated_intrinsics[3]),
            "distortion": [float(x) for x in interpolated_intrinsics[4:]],
        },
        "output_model": str(args.output.resolve()),
    }
    metadata_path = args.output.with_suffix(args.output.suffix + ".json")
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    print(json.dumps(metadata, indent=2))
    print(f"wrote {args.output}")
    print(f"wrote {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
