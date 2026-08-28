#!/usr/bin/env python3
"""Evaluate a trained towel YOLO-seg weight on the reviewed validation split."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.towel_yolo_segmentation import parse_yolo_segmentation_line  # noqa: E402


def binary_mask_iou(expected: np.ndarray, predicted: np.ndarray) -> float:
    """Return intersection-over-union for two same-size binary masks."""

    if expected.shape != predicted.shape:
        raise ValueError("expected and predicted masks must have the same shape")
    expected_binary = expected.astype(bool)
    predicted_binary = predicted.astype(bool)
    union = int(np.count_nonzero(expected_binary | predicted_binary))
    if union == 0:
        return 1.0
    intersection = int(np.count_nonzero(expected_binary & predicted_binary))
    return intersection / union


def summarize_records(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Summarize fixed-threshold presence, empty rejection, and mask IoU."""

    nonempty = [record for record in records if not record["empty_expected"]]
    empty = [record for record in records if record["empty_expected"]]
    if not nonempty or not empty:
        raise ValueError("evaluation requires both non-empty and empty validation items")
    ious = [float(record["mask_iou"]) for record in nonempty]
    return {
        "validation_count": len(records),
        "nonempty_count": len(nonempty),
        "empty_count": len(empty),
        "towel_detected_count": sum(
            int(record["prediction_count"]) > 0 for record in nonempty
        ),
        "empty_rejected_count": sum(
            int(record["prediction_count"]) == 0 for record in empty
        ),
        "nonempty_mask_iou_mean": round(sum(ious) / len(ious), 9),
        "nonempty_mask_iou_min": round(min(ious), 9),
        "false_negative_observation_ids": [
            str(record["observation_id"])
            for record in nonempty
            if int(record["prediction_count"]) == 0
        ],
        "empty_false_positive_observation_ids": [
            str(record["observation_id"])
            for record in empty
            if int(record["prediction_count"]) > 0
        ],
    }


def _ground_truth_mask(label_path: Path, width: int, height: int) -> np.ndarray:
    polygon = parse_yolo_segmentation_line(
        label_path.read_text(encoding="utf-8"),
        width=width,
        height=height,
    )
    mask = np.zeros((height, width), dtype=np.uint8)
    if polygon:
        cv2.fillPoly(mask, [np.rint(np.asarray(polygon)).astype(np.int32)], 1)
    return mask


def _prediction_mask(
    result: Any,
    *,
    width: int,
    height: int,
    mask_threshold: float,
) -> tuple[np.ndarray, int, float]:
    mask = np.zeros((height, width), dtype=np.uint8)
    if result.masks is None or result.boxes is None:
        return mask, 0, 0.0
    count = 0
    max_confidence = 0.0
    masks = result.masks.data.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    confidences = result.boxes.conf.cpu().numpy()
    for prediction, class_id, confidence in zip(
        masks, classes, confidences, strict=True
    ):
        if class_id != 0:
            continue
        resized = cv2.resize(
            prediction,
            (width, height),
            interpolation=cv2.INTER_LINEAR,
        )
        mask |= (resized >= mask_threshold).astype(np.uint8)
        count += 1
        max_confidence = max(max_confidence, float(confidence))
    return mask, count, max_confidence


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument(
        "--dataset",
        type=Path,
        default=ROOT / "tmp/towel_yolo_segmentation",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "tmp/towel_yolo_segmentation_evaluation.json",
    )
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--mask-threshold", type=float, default=0.5)
    args = parser.parse_args()

    if not 0.0 < args.confidence < 1.0:
        parser.error("--confidence must be within 0..1")
    if not 0.0 < args.mask_threshold < 1.0:
        parser.error("--mask-threshold must be within 0..1")
    model_path = args.model.resolve()
    dataset_root = args.dataset.resolve()
    manifest_path = dataset_root / "export_manifest.json"
    if not model_path.is_file():
        parser.error(f"model does not exist: {model_path}")
    if not manifest_path.is_file():
        parser.error(f"export manifest does not exist: {manifest_path}")

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("record_kind") != "towel_yolo_segmentation_export":
        parser.error("dataset is not a towel YOLO segmentation export")
    validation_items = [
        item for item in manifest.get("items", []) if item.get("split") == "validation"
    ]
    if len(validation_items) != manifest.get("split_counts", {}).get("validation"):
        parser.error("validation items do not match the export manifest")

    try:
        import torch
        import ultralytics
        from ultralytics import YOLO
    except ImportError as exc:
        parser.error(f"activate the YOLO environment first: {exc}")

    image_paths = [str(dataset_root / item["image"]) for item in validation_items]
    results = YOLO(str(model_path)).predict(
        image_paths,
        imgsz=args.imgsz,
        conf=args.confidence,
        device=args.device,
        verbose=False,
        stream=False,
    )
    if len(results) != len(validation_items):
        raise RuntimeError("prediction result count differs from validation count")

    records: list[dict[str, Any]] = []
    for item, result in zip(validation_items, results, strict=True):
        image_path = dataset_root / item["image"]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"could not decode validation image: {image_path}")
        height, width = image.shape[:2]
        expected = _ground_truth_mask(dataset_root / item["label"], width, height)
        predicted, prediction_count, max_confidence = _prediction_mask(
            result,
            width=width,
            height=height,
            mask_threshold=args.mask_threshold,
        )
        records.append(
            {
                "observation_id": item["observation_id"],
                "source_sha256": item["source_sha256"],
                "empty_expected": bool(item["empty_negative"]),
                "prediction_count": prediction_count,
                "max_confidence": round(max_confidence, 9),
                "mask_iou": round(binary_mask_iou(expected, predicted), 9),
            }
        )

    evaluation = {
        "schema_version": 1,
        "record_kind": "towel_yolo_segmentation_evaluation",
        "model": {
            "path": model_path.as_posix(),
            "sha256": sha256(model_path.read_bytes()).hexdigest(),
        },
        "dataset": {
            "export_manifest": manifest_path.as_posix(),
            "items_sha256": manifest["items_sha256"],
        },
        "configuration": {
            "device": str(args.device),
            "imgsz": args.imgsz,
            "confidence": args.confidence,
            "mask_threshold": args.mask_threshold,
            "torch_version": torch.__version__,
            "torch_cuda_runtime": torch.version.cuda,
            "ultralytics_version": ultralytics.__version__,
        },
        "summary": summarize_records(records),
        "records": records,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(evaluation, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    summary = evaluation["summary"]
    print(
        "TOWEL_YOLO_SEGMENTATION_EVALUATION_COMPLETE "
        f"towel={summary['towel_detected_count']}/{summary['nonempty_count']} "
        f"empty={summary['empty_rejected_count']}/{summary['empty_count']} "
        f"iou_mean={summary['nonempty_mask_iou_mean']:.6f} "
        f"iou_min={summary['nonempty_mask_iou_min']:.6f} "
        f"output={args.output.resolve()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
