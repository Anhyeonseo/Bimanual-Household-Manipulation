#!/usr/bin/env python3
"""Detect one dark planar object and report its board-relative pose."""

from __future__ import annotations

import argparse
import importlib
import json
import sys
from pathlib import Path


PACKAGE_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "ros2_ws"
    / "src"
    / "so101_top_perception"
)
if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))

shared_detector = importlib.import_module("so101_top_perception.detector")
file_sha256 = shared_detector.file_sha256


def detect(args: argparse.Namespace) -> dict:
    config = shared_detector.DetectorConfig(
        threshold=args.threshold,
        min_area_px=args.min_area_px,
        min_width_px=args.min_width_px,
        min_height_px=args.min_height_px,
        min_solidity=args.min_solidity,
    )
    return shared_detector.detect_image_file(
        args.image,
        args.camera_info,
        args.homography,
        config,
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Detect exactly one dark planar object and report x, y, and yaw "
            "in the calibrated Top-camera board frame."
        )
    )
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--camera-info", required=True, type=Path)
    parser.add_argument("--homography", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--threshold", type=int, default=110)
    parser.add_argument("--min-area-px", type=float, default=1000.0)
    parser.add_argument("--min-width-px", type=int, default=20)
    parser.add_argument("--min-height-px", type=int, default=20)
    parser.add_argument("--min-solidity", type=float, default=0.5)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = detect(args)
        serialized = json.dumps(
            result,
            indent=2,
            sort_keys=True,
        )
        if args.output is not None:
            output_path = args.output.resolve()
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_text(serialized + "\n", encoding="utf-8")
        print(serialized)
        return 0
    except Exception as error:
        print(f"TOP_OBJECT_POSE_FAIL reason={error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
