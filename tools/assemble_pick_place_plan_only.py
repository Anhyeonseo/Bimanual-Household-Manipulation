#!/usr/bin/env python3
"""Assemble collision-checked segments into a non-executable Pick/Place plan."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.joint_calibration import (  # noqa: E402
    calibration_hash,
    load_calibration,
    raw_to_urad,
)


ARM_JOINTS = (
    "left_base_joint",
    "left_shoulder_joint",
    "left_elbow_joint",
    "left_wrist_flex_joint",
    "left_wrist_roll_joint",
)
MAX_JOINT_STEP_RAD = 0.18
CHAIN_TOLERANCE_RAD = 1e-9
ARM_DURATION_S = 2.0
GRIPPER_DURATION_S = 1.0
WORKSPACE_X_M = (0.20, 0.46)
WORKSPACE_Y_M = (-0.30, 0.08)
WORKSPACE_RADIAL_M = (0.20, 0.46)
WORKSPACE_TCP_Z_M = (0.02, 0.15)
BOARD_X_M = (0.34, 0.52)
BOARD_Y_M = (-0.28, 0.0)
PLACE_GRASP_OFFSET_M = 0.025
PLACE_PREGRASP_OFFSET_M = 0.10


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _joint_vector(value: Any, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != len(ARM_JOINTS):
        raise ValueError(f"{label} must contain exactly five joints")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{label} must contain only finite values")
    return result


def load_phase(
    path: Path,
    expected_target_name: str,
    *,
    reverse: bool = False,
) -> list[dict[str, Any]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    expected_status = f"{expected_target_name.upper()}_SEGMENT_PLAN_ONLY_PASS"
    if document.get("status") != expected_status:
        raise ValueError(f"{path} is not {expected_status}")
    for field in (
        "execution_api_used",
        "motion_authorized",
        "robot_target_available",
    ):
        if document.get(field) is not False:
            raise ValueError(f"{path} must keep {field}=false")
    if document.get("target_name") != expected_target_name:
        raise ValueError(f"{path} target_name is inconsistent")
    if tuple(document.get("joint_names", ())) != ARM_JOINTS:
        raise ValueError(f"{path} joint order is inconsistent")

    maximum = float(document.get("max_joint_step_rad"))
    if not 0.0 < maximum <= MAX_JOINT_STEP_RAD:
        raise ValueError(f"{path} max_joint_step_rad exceeds Stage 7 gate")
    source_segments = document.get("segments")
    if not isinstance(source_segments, list) or not source_segments:
        raise ValueError(f"{path} has no segments")

    normalized: list[dict[str, Any]] = []
    previous_target: tuple[float, ...] | None = None
    for expected_index, item in enumerate(source_segments, start=1):
        if item.get("index") != expected_index or item.get("success") is not True:
            raise ValueError(f"{path} contains an invalid segment")
        start = _joint_vector(
            item.get("expected_start_positions_rad"),
            "segment start",
        )
        target = _joint_vector(
            item.get("target_positions_rad"),
            "segment target",
        )
        if previous_target is not None and any(
            abs(actual - expected) > CHAIN_TOLERANCE_RAD
            for actual, expected in zip(start, previous_target, strict=True)
        ):
            raise ValueError(f"{path} contains a discontinuous segment chain")
        actual_step = max(
            abs(goal - current)
            for current, goal in zip(start, target, strict=True)
        )
        recorded_step = float(item.get("maximum_joint_delta_rad"))
        if not math.isclose(actual_step, recorded_step, abs_tol=1e-9):
            raise ValueError(f"{path} contains an inconsistent joint delta")
        if actual_step > maximum + 1e-9:
            raise ValueError(f"{path} contains an oversized joint delta")
        normalized.append(
            {
                "source_segment_index": expected_index,
                "start_positions_rad": list(start),
                "target_positions_rad": list(target),
                "maximum_joint_delta_rad": actual_step,
            }
        )
        previous_target = target

    if reverse:
        normalized = [
            {
                "source_segment_index": item["source_segment_index"],
                "start_positions_rad": item["target_positions_rad"],
                "target_positions_rad": item["start_positions_rad"],
                "maximum_joint_delta_rad": item["maximum_joint_delta_rad"],
            }
            for item in reversed(normalized)
        ]
    return normalized


def _arm_limits(calibration: dict[str, Any]) -> tuple[tuple[float, float], ...]:
    limits = []
    for index, joint in enumerate(calibration["joints"][:5]):
        endpoint_a = raw_to_urad(calibration, index, joint["minimum_raw"]) / 1e6
        endpoint_b = raw_to_urad(calibration, index, joint["maximum_raw"]) / 1e6
        limits.append((min(endpoint_a, endpoint_b), max(endpoint_a, endpoint_b)))
    return tuple(limits)


def _validate_arm_positions(
    positions: tuple[float, ...],
    limits: tuple[tuple[float, float], ...],
) -> None:
    for name, position, (lower, upper) in zip(
        ARM_JOINTS,
        positions,
        limits,
        strict=True,
    ):
        if not lower <= position <= upper:
            raise ValueError(
                f"{name} target {position:.6f} outside {lower:.6f}..{upper:.6f}"
            )


def _validate_gripper_position(
    position: float,
    calibration: dict[str, Any],
) -> None:
    joint = calibration["joints"][5]
    endpoint_a = raw_to_urad(calibration, 5, joint["minimum_raw"]) / 1e6
    endpoint_b = raw_to_urad(calibration, 5, joint["maximum_raw"]) / 1e6
    lower, upper = min(endpoint_a, endpoint_b), max(endpoint_a, endpoint_b)
    if not math.isfinite(position) or not lower <= position <= upper:
        raise ValueError(
            f"gripper target {position:.6f} outside {lower:.6f}..{upper:.6f}"
        )


def _validate_place_target(
    place_target: tuple[float, float, float, float],
) -> None:
    if len(place_target) != 4 or not all(
        math.isfinite(value) for value in place_target
    ):
        raise ValueError("place target must contain four finite values")
    x_m, y_m, object_z_m, unused_yaw_rad = place_target
    del unused_yaw_rad
    radial_m = math.hypot(x_m, y_m)
    if not WORKSPACE_X_M[0] <= x_m <= WORKSPACE_X_M[1]:
        raise ValueError("place target x is outside approved workspace")
    if not WORKSPACE_Y_M[0] <= y_m <= WORKSPACE_Y_M[1]:
        raise ValueError("place target y is outside approved workspace")
    if not WORKSPACE_RADIAL_M[0] <= radial_m <= WORKSPACE_RADIAL_M[1]:
        raise ValueError("place target radial distance is outside workspace")
    if not BOARD_X_M[0] <= x_m <= BOARD_X_M[1]:
        raise ValueError("place target x is outside the validated table board")
    if not BOARD_Y_M[0] <= y_m <= BOARD_Y_M[1]:
        raise ValueError("place target y is outside the validated table board")
    for name, tcp_z_m in (
        ("grasp", object_z_m + PLACE_GRASP_OFFSET_M),
        ("pregrasp", object_z_m + PLACE_PREGRASP_OFFSET_M),
    ):
        if not WORKSPACE_TCP_Z_M[0] <= tcp_z_m <= WORKSPACE_TCP_Z_M[1]:
            raise ValueError(f"place {name} TCP z is outside workspace")


def assemble(
    phase_specs: list[tuple[str, Path, str, bool]],
    calibration_path: Path,
    place_target: tuple[float, float, float, float],
    gripper_close_rad: float,
    gripper_open_rad: float,
) -> dict[str, Any]:
    calibration = load_calibration(calibration_path)
    limits = _arm_limits(calibration)
    _validate_place_target(place_target)
    _validate_gripper_position(gripper_close_rad, calibration)
    _validate_gripper_position(gripper_open_rad, calibration)

    steps: list[dict[str, Any]] = []
    phase_summaries: list[dict[str, Any]] = []
    previous_arm_target = (0.0,) * len(ARM_JOINTS)

    for phase_name, path, target_name, reverse in phase_specs:
        phase_steps = load_phase(path, target_name, reverse=reverse)
        first_start = tuple(phase_steps[0]["start_positions_rad"])
        if any(
            abs(actual - expected) > CHAIN_TOLERANCE_RAD
            for actual, expected in zip(
                first_start,
                previous_arm_target,
                strict=True,
            )
        ):
            raise ValueError(f"{phase_name} does not continue the arm chain")

        if phase_name == "pick_grasp_to_lift20":
            steps.append(
                {
                    "index": len(steps) + 1,
                    "kind": "gripper",
                    "phase": "pick_close",
                    "target_position_rad": gripper_close_rad,
                    "duration_s": GRIPPER_DURATION_S,
                    "contact_expected": True,
                    "manual_gate_required": True,
                }
            )
        if phase_name == "place_to_retreat":
            steps.append(
                {
                    "index": len(steps) + 1,
                    "kind": "gripper",
                    "phase": "place_release",
                    "target_position_rad": gripper_open_rad,
                    "duration_s": GRIPPER_DURATION_S,
                    "contact_expected": False,
                    "manual_gate_required": True,
                }
            )

        source_sha = sha256_file(path)
        for phase_step_index, item in enumerate(phase_steps, start=1):
            start = tuple(item["start_positions_rad"])
            target = tuple(item["target_positions_rad"])
            _validate_arm_positions(start, limits)
            _validate_arm_positions(target, limits)
            steps.append(
                {
                    "index": len(steps) + 1,
                    "kind": "arm",
                    "phase": phase_name,
                    "source": str(path),
                    "source_sha256": source_sha,
                    "source_segment_index": item["source_segment_index"],
                    "start_positions_rad": list(start),
                    "target_positions_rad": list(target),
                    "maximum_joint_delta_rad": item[
                        "maximum_joint_delta_rad"
                    ],
                    "duration_s": ARM_DURATION_S,
                    "manual_gate_required": phase_step_index == 1,
                }
            )
            previous_arm_target = target

        phase_summaries.append(
            {
                "name": phase_name,
                "source": str(path),
                "source_sha256": source_sha,
                "reversed": reverse,
                "arm_segment_count": len(phase_steps),
            }
        )

    if any(abs(position) > CHAIN_TOLERANCE_RAD for position in previous_arm_target):
        raise ValueError("assembled arm chain does not finish at q0")

    arm_steps = [step for step in steps if step["kind"] == "arm"]
    return {
        "schema_version": 1,
        "status": "FULL_PICK_PLACE_PLAN_ONLY_PASS",
        "execution_api_used": False,
        "motion_authorized": False,
        "robot_target_available": False,
        "automatic_execution_permitted": False,
        "calibration": str(calibration_path),
        "calibration_hash": f"0x{calibration_hash(calibration):08X}",
        "joint_names": list(ARM_JOINTS),
        "place_target_base": {
            "x_m": place_target[0],
            "y_m": place_target[1],
            "z_m": place_target[2],
            "yaw_rad": place_target[3],
        },
        "maximum_joint_step_rad": MAX_JOINT_STEP_RAD,
        "recommended_arm_duration_s": ARM_DURATION_S,
        "recommended_gripper_duration_s": GRIPPER_DURATION_S,
        "arm_segment_count": len(arm_steps),
        "command_step_count": len(steps),
        "phase_summaries": phase_summaries,
        "steps": steps,
        "required_manual_gates": [
            "fresh perception and transform/workspace validation",
            "empty pregrasp and transfer paths",
            "object alignment before close",
            "stable grasp before lift",
            "empty place region before descent",
            "object support before release",
            "diagnostics and temperature review at phase boundaries",
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Assemble a fail-closed, non-executable Pick/Place plan."
    )
    parser.add_argument("--calibration", required=True, type=Path)
    parser.add_argument("--q0-to-pick-pregrasp", required=True, type=Path)
    parser.add_argument("--pick-pregrasp-to-grasp", required=True, type=Path)
    parser.add_argument("--pick-grasp-to-lift20", required=True, type=Path)
    parser.add_argument("--lift-to-place-pregrasp", required=True, type=Path)
    parser.add_argument("--place-pregrasp-to-place", required=True, type=Path)
    parser.add_argument("--place-to-retreat", required=True, type=Path)
    parser.add_argument("--q0-to-place-pregrasp", required=True, type=Path)
    parser.add_argument("--place-x", required=True, type=float)
    parser.add_argument("--place-y", required=True, type=float)
    parser.add_argument("--place-z", required=True, type=float)
    parser.add_argument("--place-yaw", required=True, type=float)
    parser.add_argument("--gripper-close", type=float, default=0.13)
    parser.add_argument("--gripper-open", type=float, default=0.06)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--plan-only", action="store_true")
    args = parser.parse_args()
    if not args.plan_only:
        parser.error("--plan-only is required; no manifest was produced")
    return args


def main() -> int:
    args = parse_args()
    phase_specs = [
        (
            "q0_to_pick_pregrasp",
            args.q0_to_pick_pregrasp,
            "pregrasp",
            False,
        ),
        (
            "pick_pregrasp_to_grasp",
            args.pick_pregrasp_to_grasp,
            "grasp",
            False,
        ),
        (
            "pick_grasp_to_lift20",
            args.pick_grasp_to_lift20,
            "grasp",
            False,
        ),
        (
            "lift_to_place_pregrasp",
            args.lift_to_place_pregrasp,
            "pregrasp",
            False,
        ),
        (
            "place_pregrasp_to_place",
            args.place_pregrasp_to_place,
            "grasp",
            False,
        ),
        (
            "place_to_retreat",
            args.place_to_retreat,
            "pregrasp",
            False,
        ),
        (
            "place_pregrasp_to_q0",
            args.q0_to_place_pregrasp,
            "pregrasp",
            True,
        ),
    ]
    try:
        result = assemble(
            phase_specs,
            args.calibration,
            (args.place_x, args.place_y, args.place_z, args.place_yaw),
            args.gripper_close,
            args.gripper_open,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(
            f"{result['status']} arm_segments={result['arm_segment_count']} "
            f"command_steps={result['command_step_count']} "
            f"output={args.output.resolve()} execution_api_used=false"
        )
        return 0
    except Exception as error:
        print(f"FULL_PICK_PLACE_PLAN_ONLY_FAIL reason={error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
