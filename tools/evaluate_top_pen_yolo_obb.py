#!/usr/bin/env python3
"""Evaluate one immutable YOLO-OBB bundle on the frozen Top-pen holdout."""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = ROOT / "ros2_ws" / "src" / "so101_top_perception"
if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))

BASELINE_PATH = ROOT / "tools" / "evaluate_top_pen_detection_baseline.py"
SPEC = importlib.util.spec_from_file_location(
    "evaluate_top_pen_detection_baseline",
    BASELINE_PATH,
)
baseline = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = baseline
SPEC.loader.exec_module(baseline)

obb_detector = importlib.import_module("so101_top_perception.obb_detector")


def evaluate(
    manifest_path: Path,
    contract_path: Path,
    camera_info_path: Path,
    homography_path: Path,
    bundle_manifest_path: Path,
) -> dict:
    """Run the common acceptance metric contract with an ONNX OBB runner."""
    manifest_path = manifest_path.resolve()
    contract_path = contract_path.resolve()
    bundle_manifest_path = bundle_manifest_path.resolve()
    contract = baseline.load_json(contract_path)
    backend = contract.get("detector", {}).get("backend")
    if backend != obb_detector.BACKEND_NAME:
        raise ValueError(
            f"evaluation detector.backend must be {obb_detector.BACKEND_NAME}"
        )
    holdout_hash = baseline.shared_detector.file_sha256(manifest_path)
    detector = obb_detector.OpenCvYoloObbDetector(
        bundle_manifest_path,
        expected_holdout_manifest_sha256=holdout_hash,
    )
    edge_margin = int(contract["detector"]["image_edge_margin_px"])

    def run(image, calibration, require_full_footprint):
        return detector.detect(
            image,
            calibration,
            image_edge_margin_px=edge_margin,
            require_full_footprint=require_full_footprint,
        )

    result = baseline.evaluate(
        manifest_path,
        contract_path,
        camera_info_path,
        homography_path,
        detector_runner=run,
    )
    result["status"] = (
        "TOP_PEN_YOLO_OBB_HOLDOUT_PASS"
        if result["passed"]
        else "TOP_PEN_YOLO_OBB_HOLDOUT_FAIL"
    )
    result["candidate"] = True
    result["model_bundle"] = str(bundle_manifest_path)
    result["model_bundle_sha256"] = baseline.shared_detector.file_sha256(
        bundle_manifest_path
    )
    result["model_sha256"] = detector.config.model_sha256
    result["holdout_used_for_training"] = False
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate a Top-pen YOLO-OBB ONNX candidate."
    )
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--contract", required=True, type=Path)
    parser.add_argument("--camera-info", required=True, type=Path)
    parser.add_argument("--homography", required=True, type=Path)
    parser.add_argument("--bundle-manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = evaluate(
            args.manifest,
            args.contract,
            args.camera_info,
            args.homography,
            args.bundle_manifest,
        )
        output = args.output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(result["status"])
        print(f"TOP_PEN_YOLO_OBB_HOLDOUT_ARTIFACT={output}")
        return 0 if result["passed"] else 2
    except Exception as error:
        print(f"TOP_PEN_YOLO_OBB_HOLDOUT_ERROR reason={error}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
