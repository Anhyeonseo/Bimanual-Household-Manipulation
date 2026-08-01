#!/usr/bin/env python3
"""Evaluate a frozen Top-camera pen dataset without publishing ROS commands."""

from __future__ import annotations

import argparse
import importlib
import json
import math
import re
import sys
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = ROOT / "ros2_ws" / "src" / "so101_top_perception"
if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))

shared_detector = importlib.import_module("so101_top_perception.detector")


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return document


def percentile(values: list[float], percentile_value: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values), percentile_value))


def undirected_yaw_error_deg(actual: float, expected: float) -> float:
    """Return the smallest long-axis yaw difference modulo 180 degrees."""
    difference = (actual - expected + 90.0) % 180.0 - 90.0
    return abs(float(difference))


def detected_count(error: Exception) -> int | None:
    match = re.search(r"detected (\d+)", str(error))
    return int(match.group(1)) if match else None


def validate_condition(case: dict) -> dict[str, str]:
    condition = case.get("condition")
    if not isinstance(condition, dict):
        raise ValueError(f"case {case.get('id')} has no condition object")
    result = {}
    for key in ("background", "lighting", "glare"):
        value = condition.get(key)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"case {case.get('id')} condition.{key} is required")
        result[key] = value.strip()
    return result


def detector_config(document: dict):
    settings = document["detector"]
    rectangles = tuple(
        tuple(int(value) for value in rectangle)
        for rectangle in settings.get("exclusion_rectangles_px", [])
    )
    return shared_detector.DetectorConfig(
        threshold=int(settings["threshold"]),
        min_area_px=float(settings["min_area_px"]),
        min_width_px=int(settings["min_width_px"]),
        min_height_px=int(settings["min_height_px"]),
        min_solidity=float(settings["min_solidity"]),
        image_edge_margin_px=int(settings["image_edge_margin_px"]),
        exclusion_rectangles_px=rectangles,
    )


def validate_geometry_hashes(
    manifest: dict,
    camera_info_path: Path,
    homography_path: Path,
) -> None:
    geometry = manifest.get("geometry")
    if not isinstance(geometry, dict):
        raise ValueError("manifest.geometry is required")
    expected = {
        "camera_info_sha256": shared_detector.file_sha256(camera_info_path),
        "homography_sha256": shared_detector.file_sha256(homography_path),
    }
    for key, actual in expected.items():
        if geometry.get(key) != actual:
            raise ValueError(f"manifest {key} does not match the supplied file")


def validate_coverage(
    cases: list[dict],
    conditions: list[dict[str, str]],
    contract: dict,
) -> list[str]:
    coverage = contract["coverage"]
    positives = sum(bool(case["expected_present"]) for case in cases)
    negatives = len(cases) - positives
    failures = []
    if positives < int(coverage["minimum_positive_cases"]):
        failures.append("insufficient_positive_cases")
    if negatives < int(coverage["minimum_negative_cases"]):
        failures.append("insufficient_negative_cases")
    for key, minimum_key in (
        ("background", "minimum_background_labels"),
        ("lighting", "minimum_lighting_labels"),
        ("glare", "minimum_glare_labels"),
    ):
        distinct = {condition[key] for condition in conditions}
        if len(distinct) < int(coverage[minimum_key]):
            failures.append(f"insufficient_{key}_coverage")
    return failures


def resolve_dataset_image(manifest_path: Path, image_value: object) -> Path:
    """Resolve an image path while keeping it inside the dataset root."""
    if not isinstance(image_value, str) or not image_value:
        raise ValueError("case image must be a non-empty relative path")
    relative = Path(image_value)
    if relative.is_absolute():
        raise ValueError("case image must be relative to the manifest")
    dataset_root = manifest_path.parent.resolve()
    image_path = (dataset_root / relative).resolve()
    try:
        image_path.relative_to(dataset_root)
    except ValueError as error:
        raise ValueError(
            "case image must stay inside the dataset directory"
        ) from error
    return image_path


def evaluate(
    manifest_path: Path,
    contract_path: Path,
    camera_info_path: Path,
    homography_path: Path,
) -> dict:
    """Evaluate all frozen cases and return a machine-readable gate artifact."""
    manifest_path = manifest_path.resolve()
    contract_path = contract_path.resolve()
    camera_info_path = camera_info_path.resolve()
    homography_path = homography_path.resolve()
    manifest = load_json(manifest_path)
    contract = load_json(contract_path)
    if manifest.get("protocol_version") != 1:
        raise ValueError("manifest protocol_version must be 1")
    if contract.get("protocol_version") != 1:
        raise ValueError("contract protocol_version must be 1")
    validate_geometry_hashes(manifest, camera_info_path, homography_path)

    cases = manifest.get("cases")
    if not isinstance(cases, list) or not cases:
        raise ValueError("manifest.cases must be a non-empty list")
    for case in cases:
        if not isinstance(case, dict):
            raise ValueError("every case must be an object")
    identifiers = [case.get("id") for case in cases]
    if any(
        not isinstance(identifier, str) or not identifier
        for identifier in identifiers
    ):
        raise ValueError("every case requires a non-empty string id")
    if len(set(identifiers)) != len(identifiers):
        raise ValueError("case ids must be unique")
    for case in cases:
        if not isinstance(case.get("expected_present"), bool):
            raise ValueError(
                f"case {case.get('id')} expected_present must be boolean"
            )

    conditions = [validate_condition(case) for case in cases]
    coverage_failures = validate_coverage(cases, conditions, contract)
    calibration = shared_detector.load_calibration(
        camera_info_path,
        homography_path,
    )
    config = detector_config(contract)
    require_full_footprint = bool(contract["detector"]["require_full_footprint"])
    acceptance = contract["acceptance"]

    results = []
    misses = 0
    false_positives = 0
    processing_errors = 0
    center_errors_px: list[float] = []
    yaw_errors_deg: list[float] = []
    positive_count = 0
    negative_count = 0
    for case, condition in zip(cases, conditions):
        expected_present = case.get("expected_present")
        image_path = resolve_dataset_image(manifest_path, case.get("image"))
        if not image_path.is_file():
            raise ValueError(f"case {case['id']} image is missing: {image_path}")
        image_hash = shared_detector.file_sha256(image_path)
        if case.get("image_sha256") != image_hash:
            raise ValueError(f"case {case['id']} image_sha256 mismatch")
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"case {case['id']} image cannot be decoded")

        pose = None
        error_code = None
        error_message = None
        candidate_count = None
        try:
            pose = shared_detector.detect_one_object(
                image,
                calibration,
                config,
                require_full_footprint=require_full_footprint,
            )
            candidate_count = 1
        except shared_detector.DetectionError as error:
            error_code = error.code
            error_message = str(error)
            candidate_count = detected_count(error)

        result = {
            "id": case["id"],
            "image": str(case["image"]),
            "image_sha256": image_hash,
            "condition": condition,
            "expected_present": expected_present,
            "detected": pose is not None,
            "candidate_count": candidate_count,
            "detection_error_code": error_code,
            "detection_error_message": error_message,
        }
        if expected_present:
            positive_count += 1
            expected_center = case.get("expected_center_px")
            expected_yaw = case.get("expected_yaw_deg")
            if (
                not isinstance(expected_center, list)
                or len(expected_center) != 2
                or not all(isinstance(value, (int, float)) for value in expected_center)
                or not isinstance(expected_yaw, (int, float))
            ):
                raise ValueError(
                    f"positive case {case['id']} requires center and yaw annotation"
                )
            if pose is None:
                misses += 1
                if error_code in ("INVALID_IMAGE", "RESOLUTION_MISMATCH"):
                    processing_errors += 1
            else:
                actual_center = pose["raw_center_px"]
                center_error = math.hypot(
                    actual_center[0] - float(expected_center[0]),
                    actual_center[1] - float(expected_center[1]),
                )
                yaw_error = undirected_yaw_error_deg(
                    float(pose["yaw_deg"]),
                    float(expected_yaw),
                )
                center_errors_px.append(center_error)
                yaw_errors_deg.append(yaw_error)
                result.update(
                    {
                        "actual_center_px": actual_center,
                        "actual_yaw_deg": pose["yaw_deg"],
                        "center_error_px": center_error,
                        "yaw_error_deg": yaw_error,
                    }
                )
        else:
            negative_count += 1
            if pose is not None or (
                candidate_count is not None
                and candidate_count > 0
            ):
                false_positives += 1
            elif error_code != "OBJECT_COUNT_INVALID" or candidate_count != 0:
                processing_errors += 1
        results.append(result)

    miss_rate = misses / positive_count if positive_count else 1.0
    false_positive_rate = (
        false_positives / negative_count if negative_count else 1.0
    )
    center_p95 = percentile(center_errors_px, 95.0)
    yaw_p95 = percentile(yaw_errors_deg, 95.0)
    failures = list(coverage_failures)
    if miss_rate > float(acceptance["maximum_miss_rate"]):
        failures.append("miss_rate_exceeded")
    if false_positive_rate > float(acceptance["maximum_false_positive_rate"]):
        failures.append("false_positive_rate_exceeded")
    if center_p95 is None or center_p95 > float(
        acceptance["maximum_center_error_px_p95"]
    ):
        failures.append("center_error_p95_exceeded")
    if yaw_p95 is None or yaw_p95 > float(
        acceptance["maximum_yaw_error_deg_p95"]
    ):
        failures.append("yaw_error_p95_exceeded")
    if processing_errors:
        failures.append("processing_errors_present")

    return {
        "protocol_version": 1,
        "status": (
            "TOP_PEN_DETECTION_BASELINE_PASS"
            if not failures
            else "TOP_PEN_DETECTION_BASELINE_FAIL"
        ),
        "passed": not failures,
        "motion_authorized": False,
        "robot_command_topics_created": 0,
        "manifest": str(manifest_path),
        "manifest_sha256": shared_detector.file_sha256(manifest_path),
        "contract": str(contract_path),
        "contract_sha256": shared_detector.file_sha256(contract_path),
        "detector_backend": contract["detector"].get(
            "backend",
            "unspecified",
        ),
        "opencv_version": cv2.__version__,
        "geometry": {
            "camera_info_sha256": shared_detector.file_sha256(camera_info_path),
            "homography_sha256": shared_detector.file_sha256(homography_path),
        },
        "metrics": {
            "case_count": len(cases),
            "positive_count": positive_count,
            "negative_count": negative_count,
            "misses": misses,
            "false_positives": false_positives,
            "processing_errors": processing_errors,
            "miss_rate": miss_rate,
            "false_positive_rate": false_positive_rate,
            "center_error_px_p95": center_p95,
            "yaw_error_deg_p95": yaw_p95,
        },
        "failures": sorted(set(failures)),
        "cases": results,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate the frozen Top-camera pen detection dataset."
    )
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--contract", required=True, type=Path)
    parser.add_argument("--camera-info", required=True, type=Path)
    parser.add_argument("--homography", required=True, type=Path)
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
        )
        serialized = json.dumps(result, indent=2, sort_keys=True)
        output_path = args.output.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(serialized + "\n", encoding="utf-8")
        print(result["status"])
        print(f"TOP_PEN_DETECTION_BASELINE_ARTIFACT={output_path}")
        return 0 if result["passed"] else 2
    except Exception as error:
        print(f"TOP_PEN_DETECTION_BASELINE_ERROR reason={error}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
