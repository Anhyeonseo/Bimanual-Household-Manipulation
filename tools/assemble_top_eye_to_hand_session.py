#!/usr/bin/env python3
"""Assemble validated stationary captures into one eye-to-hand session."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path

import yaml


EXPECTED_MARKER_IDS = (0, 1, 2, 3)
MIN_TRAINING_CAPTURES = 8
MIN_VALIDATION_CAPTURES = 2
ARM_JOINT_COUNT = 5


def load_yaml(path: Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return document


def capture_yaml_path(value: Path) -> Path:
    path = value.resolve()
    if path.is_dir():
        path = path / "capture.yaml"
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def relative_path(path: Path, output_directory: Path) -> str:
    return os.path.relpath(path.resolve(), output_directory.resolve())


def validated_capture(
    input_value: Path,
    output_directory: Path,
    arm: str = "left",
) -> dict:
    yaml_path = capture_yaml_path(input_value)
    document = load_yaml(yaml_path)
    if document.get("status") != "STATIONARY_READ_ONLY_CAPTURE_PASS":
        raise ValueError(f"capture did not pass stationary gate: {yaml_path}")
    if bool(document.get("motion_authorized", False)):
        raise ValueError(f"capture authorizes motion: {yaml_path}")
    if bool(document.get("robot_target_available", False)):
        raise ValueError(f"capture exposes robot target: {yaml_path}")

    capture = document.get("capture")
    if not isinstance(capture, dict):
        raise ValueError(f"capture mapping is missing: {yaml_path}")
    capture_id = str(capture.get("id", "")).strip()
    if not capture_id:
        raise ValueError(f"capture id is missing: {yaml_path}")
    capture_arm = str(capture.get("arm", "left")).strip()
    if capture_arm != arm:
        raise ValueError(
            f"{capture_id} arm {capture_arm!r} != session arm {arm!r}"
        )

    marker_ids = tuple(int(value) for value in capture.get(
        "detected_marker_ids",
        [],
    ))
    if marker_ids != EXPECTED_MARKER_IDS:
        raise ValueError(
            f"{capture_id} marker IDs {marker_ids} != "
            f"{EXPECTED_MARKER_IDS}"
        )

    measured = [float(value) for value in capture.get(
        "measured_arm_rad",
        [],
    )]
    if len(measured) != ARM_JOINT_COUNT:
        raise ValueError(
            f"{capture_id} measured_arm_rad has {len(measured)} values"
        )
    joint_span = [float(value) for value in capture.get(
        "joint_span_rad",
        [],
    )]
    if len(joint_span) != ARM_JOINT_COUNT:
        raise ValueError(
            f"{capture_id} joint_span_rad has {len(joint_span)} values"
        )

    image_values = capture.get("image_files", [])
    if not image_values:
        raise ValueError(f"{capture_id} has no image files")
    image_paths: list[str] = []
    for image_value in image_values:
        image_path = Path(str(image_value))
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        if not image_path.is_file():
            raise FileNotFoundError(image_path)
        image_paths.append(relative_path(image_path, output_directory))

    return {
        "id": capture_id,
        "arm": capture_arm,
        "measured_arm_rad": measured,
        "joint_span_rad": joint_span,
        "image_files": image_paths,
        "image_source_stamp_first": float(
            capture["image_source_stamp_first"]
        ),
        "image_source_stamp_last": float(
            capture["image_source_stamp_last"]
        ),
        "detected_marker_ids": list(marker_ids),
        "source_capture_yaml": relative_path(
            yaml_path,
            output_directory,
        ),
        "source_capture_sha256": file_sha256(yaml_path),
    }


def assemble_document(
    session_id: str,
    training_values: list[Path],
    validation_values: list[Path],
    output_path: Path,
    arm: str = "left",
) -> dict:
    session_id = session_id.strip()
    if not session_id:
        raise ValueError("session_id must not be empty")
    if arm not in ("left", "right"):
        raise ValueError(f"unsupported arm: {arm}")
    if len(training_values) < MIN_TRAINING_CAPTURES:
        raise ValueError(
            f"training capture count {len(training_values)} < "
            f"{MIN_TRAINING_CAPTURES}"
        )
    if len(validation_values) < MIN_VALIDATION_CAPTURES:
        raise ValueError(
            f"validation capture count {len(validation_values)} < "
            f"{MIN_VALIDATION_CAPTURES}"
        )

    output_directory = output_path.resolve().parent
    training = [
        validated_capture(value, output_directory, arm)
        for value in training_values
    ]
    validation = [
        validated_capture(value, output_directory, arm)
        for value in validation_values
    ]
    capture_ids = [
        capture["id"] for capture in training + validation
    ]
    if len(capture_ids) != len(set(capture_ids)):
        raise ValueError("capture IDs must be unique across the session")

    return {
        "schema_version": 1,
        "session_id": session_id,
        "status": "CAPTURE_SET_VALIDATED_READY_TO_SOLVE",
        "motion_authorized": False,
        "robot_target_available": False,
        "arm": arm,
        "frames": {
            "robot": f"{arm}_base_link",
            "gripper": f"{arm}_gripper_frame_link",
            "camera": "top_camera_optical_frame",
            "target": "tcp_aruco_gridboard",
        },
        "target": {
            "dictionary": "DICT_4X4_50",
            "markers_x": 2,
            "markers_y": 2,
            "marker_length_m": 0.020,
            "marker_separation_m": 0.005,
            "first_marker_id": 0,
        },
        "training_captures": training,
        "validation_captures": validation,
        "required_next_gate": (
            "run solve_top_eye_to_hand.py; keep motion_authorized=false "
            "until independent tabletop x/y/yaw validation passes"
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--arm", choices=("left", "right"), default="left")
    parser.add_argument(
        "--training-capture",
        required=True,
        action="append",
        type=Path,
        help="capture directory or capture.yaml; repeat at least eight times",
    )
    parser.add_argument(
        "--validation-capture",
        required=True,
        action="append",
        type=Path,
        help="held-out capture directory or capture.yaml; repeat twice",
    )
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    document = assemble_document(
        args.session_id,
        args.training_capture,
        args.validation_capture,
        args.output,
        args.arm,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(document, sort_keys=False),
        encoding="utf-8",
    )
    print(
        "TOP_EYE_TO_HAND_SESSION_ASSEMBLY_PASS "
        f"training={len(document['training_captures'])} "
        f"validation={len(document['validation_captures'])} "
        f"motion_authorized=0 output={args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
