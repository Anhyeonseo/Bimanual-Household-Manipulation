#!/usr/bin/env python3
"""Validate a Top-pen OBB training set and block frozen-holdout leakage."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import cv2


YAW_SEMANTICS = "undirected_long_axis_modulo_pi"
LABEL_FORMAT = "ultralytics_obb_four_corners_normalized"


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return document


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_inside(manifest_path: Path, value: object, field: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a non-empty relative path")
    relative = Path(value)
    if relative.is_absolute():
        raise ValueError(f"{field} must be relative to the manifest")
    root = manifest_path.parent.resolve()
    resolved = (root / relative).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ValueError(f"{field} must stay inside the dataset") from error
    return resolved


def validate_condition(case: dict) -> dict[str, str]:
    condition = case.get("condition")
    if not isinstance(condition, dict):
        raise ValueError(f"case {case.get('id')} has no condition")
    result = {}
    for key in ("background", "lighting", "glare"):
        value = condition.get(key)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"case {case.get('id')} condition.{key} is required")
        result[key] = value.strip()
    return result


def parse_obb_label(
    label_path: Path,
    expected_present: bool,
    expected_objects: int,
) -> int:
    text = label_path.read_text(encoding="utf-8")
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if not expected_present:
        if lines:
            raise ValueError(
                f"negative label must be empty: {label_path}"
            )
        return 0
    if len(lines) != expected_objects:
        raise ValueError(
            f"positive label needs {expected_objects} OBB: {label_path}"
        )
    for line in lines:
        tokens = line.split()
        if len(tokens) != 9:
            raise ValueError(
                f"OBB label must have class plus 8 coordinates: {label_path}"
            )
        try:
            class_id = int(tokens[0])
            coordinates = [float(value) for value in tokens[1:]]
        except ValueError as error:
            raise ValueError(f"invalid OBB label values: {label_path}") from error
        if class_id != 0:
            raise ValueError(f"only class 0 (pen) is allowed: {label_path}")
        if any(value < 0.0 or value > 1.0 for value in coordinates):
            raise ValueError(
                f"OBB coordinates must be normalized to 0..1: {label_path}"
            )
        points = [
            (coordinates[index], coordinates[index + 1])
            for index in range(0, 8, 2)
        ]
        area = 0.0
        for index, point in enumerate(points):
            following = points[(index + 1) % len(points)]
            area += point[0] * following[1] - following[0] * point[1]
        if abs(area) * 0.5 <= 1e-6:
            raise ValueError(f"OBB polygon area is zero: {label_path}")
    return len(lines)


def _holdout_hashes(holdout: dict) -> set[str]:
    cases = list(holdout.get("cases", [])) + list(
        holdout.get("auxiliary_cases", [])
    )
    hashes = set()
    for case in cases:
        value = case.get("image_sha256")
        if not isinstance(value, str) or len(value) != 64:
            raise ValueError("holdout case has no valid image_sha256")
        hashes.add(value)
    if not hashes:
        raise ValueError("holdout contains no image hashes")
    return hashes


def validate_training_dataset(
    manifest_path: Path,
    holdout_manifest_path: Path,
    contract_path: Path,
) -> dict:
    """Return a machine-readable coverage and leakage gate artifact."""
    manifest_path = manifest_path.resolve()
    holdout_manifest_path = holdout_manifest_path.resolve()
    contract_path = contract_path.resolve()
    manifest = load_json(manifest_path)
    holdout = load_json(holdout_manifest_path)
    contract = load_json(contract_path)
    for name, document in (
        ("manifest", manifest),
        ("holdout", holdout),
        ("contract", contract),
    ):
        if document.get("protocol_version") != 1:
            raise ValueError(f"{name} protocol_version must be 1")
    if manifest.get("dataset_role") != "training":
        raise ValueError("manifest dataset_role must be training")
    if contract.get("dataset_role") != "training":
        raise ValueError("contract dataset_role must be training")
    if manifest.get("class_names") != ["pen"]:
        raise ValueError("training class_names must be exactly ['pen']")
    if contract.get("class_names") != ["pen"]:
        raise ValueError("contract class_names must be exactly ['pen']")
    if manifest.get("yaw_semantics") != YAW_SEMANTICS:
        raise ValueError(f"manifest yaw_semantics must be {YAW_SEMANTICS}")
    if contract.get("yaw_semantics") != YAW_SEMANTICS:
        raise ValueError(f"contract yaw_semantics must be {YAW_SEMANTICS}")
    if not isinstance(manifest.get("dataset_id"), str) or not manifest[
        "dataset_id"
    ].strip():
        raise ValueError("manifest dataset_id must be a non-empty string")
    if contract.get("motion_authorized") is not False:
        raise ValueError("contract motion_authorized must be false")

    leakage = contract.get("leakage")
    annotation = contract.get("annotation")
    if not isinstance(leakage, dict) or not isinstance(annotation, dict):
        raise ValueError("contract leakage and annotation are required")
    if leakage.get("forbid_holdout_image_sha256") is not True:
        raise ValueError("contract must forbid frozen holdout image reuse")
    if leakage.get("require_disjoint_train_validation_sha256") is not True:
        raise ValueError("contract must require disjoint train/validation images")
    if annotation.get("format") != LABEL_FORMAT:
        raise ValueError(f"contract annotation.format must be {LABEL_FORMAT}")
    if annotation.get("negative_label_must_be_empty") is not True:
        raise ValueError("contract must require empty negative labels")

    splits = manifest.get("splits")
    if not isinstance(splits, dict):
        raise ValueError("manifest.splits is required")
    if set(splits) != {"train", "validation"}:
        raise ValueError("manifest splits must be train and validation")
    holdout_hashes = _holdout_hashes(holdout)
    all_hashes = set()
    split_hashes: dict[str, set[str]] = {}
    conditions = []
    counts = {}
    case_results = []
    expected_objects = int(
        contract["annotation"]["positive_objects_per_image"]
    )
    for split_name in ("train", "validation"):
        cases = splits[split_name]
        if not isinstance(cases, list):
            raise ValueError(f"manifest.splits.{split_name} must be a list")
        positive = 0
        negative = 0
        current_hashes = set()
        identifiers = set()
        for case in cases:
            if not isinstance(case, dict):
                raise ValueError("every training case must be an object")
            identifier = case.get("id")
            if not isinstance(identifier, str) or not identifier:
                raise ValueError("every training case needs a non-empty id")
            if identifier in identifiers:
                raise ValueError(f"duplicate case id in {split_name}: {identifier}")
            identifiers.add(identifier)
            expected_present = case.get("expected_present")
            if not isinstance(expected_present, bool):
                raise ValueError(
                    f"case {identifier} expected_present must be boolean"
                )
            image_path = resolve_inside(manifest_path, case.get("image"), "image")
            label_path = resolve_inside(manifest_path, case.get("label"), "label")
            if not image_path.is_file() or not label_path.is_file():
                raise ValueError(f"case {identifier} image or label is missing")
            image_hash = file_sha256(image_path)
            label_hash = file_sha256(label_path)
            if case.get("image_sha256") != image_hash:
                raise ValueError(f"case {identifier} image_sha256 mismatch")
            if case.get("label_sha256") != label_hash:
                raise ValueError(f"case {identifier} label_sha256 mismatch")
            if image_hash in holdout_hashes:
                raise ValueError(
                    f"case {identifier} reuses a frozen holdout image"
                )
            if image_hash in all_hashes:
                raise ValueError(
                    f"case {identifier} duplicates another training image"
                )
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"case {identifier} image cannot be decoded")
            object_count = parse_obb_label(
                label_path,
                expected_present,
                expected_objects,
            )
            condition = validate_condition(case)
            conditions.append(condition)
            all_hashes.add(image_hash)
            current_hashes.add(image_hash)
            if expected_present:
                positive += 1
            else:
                negative += 1
            case_results.append(
                {
                    "id": identifier,
                    "split": split_name,
                    "image_sha256": image_hash,
                    "label_sha256": label_hash,
                    "expected_present": expected_present,
                    "object_count": object_count,
                    "condition": condition,
                    "width": int(image.shape[1]),
                    "height": int(image.shape[0]),
                }
            )
        split_hashes[split_name] = current_hashes
        counts[split_name] = {
            "positive": positive,
            "negative": negative,
            "total": len(cases),
        }

    if split_hashes["train"] & split_hashes["validation"]:
        raise ValueError("train and validation image SHA-256 sets overlap")

    coverage = contract["coverage"]
    failures = []
    thresholds = (
        ("train", "positive", "minimum_train_positive"),
        ("train", "negative", "minimum_train_negative"),
        ("validation", "positive", "minimum_validation_positive"),
        ("validation", "negative", "minimum_validation_negative"),
    )
    for split_name, kind, key in thresholds:
        if counts[split_name][kind] < int(coverage[key]):
            failures.append(f"insufficient_{split_name}_{kind}")
    for label, key in (
        ("background", "minimum_background_labels"),
        ("lighting", "minimum_lighting_labels"),
        ("glare", "minimum_glare_labels"),
    ):
        if len({condition[label] for condition in conditions}) < int(
            coverage[key]
        ):
            failures.append(f"insufficient_{label}_coverage")

    return {
        "protocol_version": 1,
        "status": (
            "TOP_PEN_OBB_TRAINING_DATASET_PASS"
            if not failures
            else "TOP_PEN_OBB_TRAINING_DATASET_FAIL"
        ),
        "passed": not failures,
        "motion_authorized": False,
        "robot_command_topics_created": 0,
        "manifest": str(manifest_path),
        "manifest_sha256": file_sha256(manifest_path),
        "holdout_manifest": str(holdout_manifest_path),
        "holdout_manifest_sha256": file_sha256(holdout_manifest_path),
        "contract": str(contract_path),
        "contract_sha256": file_sha256(contract_path),
        "yaw_semantics": YAW_SEMANTICS,
        "counts": counts,
        "distinct_conditions": {
            key: sorted({condition[key] for condition in conditions})
            for key in ("background", "lighting", "glare")
        },
        "unique_training_image_sha256": len(all_hashes),
        "holdout_overlap_count": len(all_hashes & holdout_hashes),
        "failures": sorted(set(failures)),
        "cases": case_results,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate a leak-free Top-pen OBB training dataset."
    )
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--holdout-manifest", required=True, type=Path)
    parser.add_argument("--contract", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = validate_training_dataset(
            args.manifest,
            args.holdout_manifest,
            args.contract,
        )
        output = args.output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(result["status"])
        print(f"TOP_PEN_OBB_TRAINING_DATASET_ARTIFACT={output}")
        return 0 if result["passed"] else 2
    except Exception as error:
        print(
            f"TOP_PEN_OBB_TRAINING_DATASET_ERROR reason={error}",
            file=sys.stderr,
        )
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
