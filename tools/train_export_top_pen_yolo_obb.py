#!/usr/bin/env python3
"""Train a desktop-only YOLO-OBB candidate and export an immutable ONNX bundle."""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import platform
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = ROOT / "ros2_ws" / "src" / "so101_top_perception"
if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))

VALIDATOR_PATH = ROOT / "tools" / "validate_top_pen_obb_training_dataset.py"
SPEC = importlib.util.spec_from_file_location(
    "validate_top_pen_obb_training_dataset",
    VALIDATOR_PATH,
)
validator = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = validator
SPEC.loader.exec_module(validator)

obb_detector = importlib.import_module("so101_top_perception.obb_detector")


def _ultralytics_version() -> str:
    try:
        module = importlib.import_module("ultralytics")
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "desktop training dependency is missing; install "
            "requirements-training.txt in a dedicated virtual environment"
        ) from error
    version = getattr(module, "__version__", None)
    if version != "8.4.67":
        raise RuntimeError(
            f"ultralytics version must be 8.4.67, found {version}"
        )
    return version


def _dataset_yaml(manifest_path: Path, manifest: dict) -> Path:
    value = manifest.get("ultralytics_data_yaml")
    path = validator.resolve_inside(
        manifest_path,
        value,
        "ultralytics_data_yaml",
    )
    if not path.is_file():
        raise ValueError(f"Ultralytics data YAML is missing: {path}")
    return path


def train_and_export(args: argparse.Namespace) -> dict:
    """Validate, train, export and write a deployment bundle manifest."""
    manifest_path = args.manifest.resolve()
    holdout_path = args.holdout_manifest.resolve()
    contract_path = args.training_contract.resolve()
    gate = validator.validate_training_dataset(
        manifest_path,
        holdout_path,
        contract_path,
    )
    if not gate["passed"]:
        raise RuntimeError(
            "training dataset gate failed: " + ",".join(gate["failures"])
        )
    manifest = validator.load_json(manifest_path)
    data_yaml = _dataset_yaml(manifest_path, manifest)
    if args.dry_run:
        return {
            "protocol_version": 1,
            "status": "TOP_PEN_YOLO_OBB_TRAIN_EXPORT_DRY_RUN_PASS",
            "passed": True,
            "motion_authorized": False,
            "robot_command_topics_created": 0,
            "training_manifest_sha256": validator.file_sha256(manifest_path),
            "holdout_manifest_sha256": validator.file_sha256(holdout_path),
            "training_contract_sha256": validator.file_sha256(contract_path),
            "ultralytics_data_yaml": str(data_yaml),
        }

    version = _ultralytics_version()
    ultralytics = importlib.import_module("ultralytics")
    torch = importlib.import_module("torch")
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    run_root = output_dir / "training_runs"
    model = ultralytics.YOLO(args.base_model)
    base_checkpoint_path = Path(str(model.ckpt_path)).resolve()
    if not base_checkpoint_path.is_file():
        raise RuntimeError("loaded base checkpoint path is unavailable")
    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.image_size,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=str(run_root),
        name="top_pen_obb",
        exist_ok=False,
        seed=args.seed,
        deterministic=True,
        single_cls=True,
        cache=False,
        plots=True,
    )
    best_path = Path(model.trainer.best).resolve()
    if not best_path.is_file():
        raise RuntimeError("training completed without best.pt")

    trained = ultralytics.YOLO(str(best_path))
    exported_value = trained.export(
        format="onnx",
        imgsz=args.image_size,
        batch=1,
        dynamic=False,
        simplify=False,
        opset=17,
        nms=False,
        device="cpu",
    )
    exported_path = Path(str(exported_value)).resolve()
    if not exported_path.is_file():
        raise RuntimeError(f"ONNX export is missing: {exported_path}")
    model_name = "top_pen_yolo11n_obb.onnx"
    deployed_model = output_dir / model_name
    shutil.copy2(exported_path, deployed_model)

    bundle = {
        "protocol_version": 1,
        "backend": obb_detector.BACKEND_NAME,
        "task": "obb",
        "candidate": True,
        "motion_authorized": False,
        "model": {
            "path": model_name,
            "sha256": validator.file_sha256(deployed_model),
            "format": "onnx",
            "opset": 17,
        },
        "input": {
            "width": args.image_size,
            "height": args.image_size,
            "layout": "NCHW",
            "color_order": "RGB",
            "scale": 1.0 / 255.0,
            "letterbox_value": 114,
        },
        "output": {
            "layout": obb_detector.OUTPUT_LAYOUT,
            "class_names": ["pen"],
            "pen_class_id": 0,
            "yaw_semantics": obb_detector.YAW_SEMANTICS,
        },
        "thresholds": {
            "confidence": args.confidence,
            "iou": args.iou,
            "maximum_detections": 10,
        },
        "training": {
            "ultralytics_version": version,
            "base_model": args.base_model,
            "base_checkpoint_sha256": validator.file_sha256(
                base_checkpoint_path
            ),
            "best_checkpoint_sha256": validator.file_sha256(best_path),
            "export_simplify": False,
            "environment": {
                "python": platform.python_version(),
                "numpy": np.__version__,
                "opencv": cv2.__version__,
                "torch": torch.__version__,
            },
            "training_manifest_sha256": validator.file_sha256(manifest_path),
            "training_contract_sha256": validator.file_sha256(contract_path),
            "holdout_manifest_sha256": validator.file_sha256(holdout_path),
            "holdout_used_for_training": False,
            "epochs": args.epochs,
            "batch": args.batch,
            "seed": args.seed,
        },
    }
    bundle_path = output_dir / "top_pen_yolo_obb_bundle.json"
    bundle_path.write_text(
        json.dumps(bundle, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    runtime = obb_detector.OpenCvYoloObbDetector(
        bundle_path,
        expected_holdout_manifest_sha256=validator.file_sha256(
            holdout_path
        ),
    )
    smoke_detections = runtime.infer_detections(
        np.zeros((480, 640, 3), dtype=np.uint8)
    )
    return {
        "protocol_version": 1,
        "status": "TOP_PEN_YOLO_OBB_TRAIN_EXPORT_PASS",
        "passed": True,
        "motion_authorized": False,
        "robot_command_topics_created": 0,
        "bundle_manifest": str(bundle_path),
        "bundle_manifest_sha256": validator.file_sha256(bundle_path),
        "model": str(deployed_model),
        "model_sha256": validator.file_sha256(deployed_model),
        "best_checkpoint": str(best_path),
        "best_checkpoint_sha256": validator.file_sha256(best_path),
        "opencv_dnn_smoke_detection_count": len(smoke_detections),
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train and export a leak-free Top-pen YOLO-OBB candidate."
    )
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--holdout-manifest", required=True, type=Path)
    parser.add_argument("--training-contract", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--base-model", default="yolo11n-obb.pt")
    parser.add_argument("--image-size", type=int, default=320)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--device", default="0")
    parser.add_argument("--confidence", type=float, default=0.4)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.image_size <= 0 or args.image_size % 32:
            raise ValueError("image-size must be a positive multiple of 32")
        if args.epochs <= 0 or args.batch <= 0 or args.workers < 0:
            raise ValueError("epochs/batch/workers are invalid")
        if not 0.0 < args.confidence < 1.0:
            raise ValueError("confidence must be within (0, 1)")
        if not 0.0 < args.iou < 1.0:
            raise ValueError("iou must be within (0, 1)")
        result = train_and_export(args)
        serialized = json.dumps(result, indent=2, sort_keys=True) + "\n"
        if args.output is not None:
            output = args.output.resolve()
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(serialized, encoding="utf-8")
        print(result["status"])
        if "bundle_manifest" in result:
            print(f"TOP_PEN_YOLO_OBB_BUNDLE={result['bundle_manifest']}")
        return 0
    except Exception as error:
        print(f"TOP_PEN_YOLO_OBB_TRAIN_EXPORT_ERROR reason={error}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
