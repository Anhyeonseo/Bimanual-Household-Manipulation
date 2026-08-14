#!/usr/bin/env python3
"""Derive reviewed, non-authorizing bimanual J1-L limit candidates."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any


JOINT_ORDER = (
    "base",
    "shoulder",
    "elbow",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
RAW_UNITS_PER_TURN = 4096
TURN_RAD = 2.0 * math.pi
STATUS = "J1_OPERATIONAL_LIMIT_CANDIDATE_REVIEW_REQUIRED"


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_sha_bound_json(path: Path, expected_sha256: str) -> dict[str, Any]:
    actual = file_sha256(path)
    if actual != expected_sha256.lower():
        raise ValueError(
            f"manifest SHA mismatch expected={expected_sha256.lower()} "
            f"actual={actual}"
        )
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("motion_authorized") is not False:
        raise ValueError("source manifest must keep motion_authorized=false")
    return document


def load_calibration(path: Path, arm_slot: str) -> tuple[dict[str, Any], ...]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("arm_slot") != arm_slot:
        raise ValueError(f"{path}: expected arm_slot={arm_slot}")
    joints = document.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
        raise ValueError(f"{path}: expected six joints")
    if [joint.get("id") for joint in joints] != list(range(1, 7)):
        raise ValueError(f"{path}: servo IDs must be ordered 1..6")
    names = tuple(str(joint.get("name", "")).lower() for joint in joints)
    if names != JOINT_ORDER:
        raise ValueError(f"{path}: unexpected joint order {names}")
    for joint in joints:
        if int(joint["positive_raw_direction"]) not in (-1, 1):
            raise ValueError(f"{path}: invalid direction")
        if not 0 <= int(joint["zero_raw"]) < RAW_UNITS_PER_TURN:
            raise ValueError(f"{path}: invalid zero_raw")
    return tuple(joints)


def raw_coordinate_to_rad(
    raw: int, *, zero_raw: int, direction: int
) -> float:
    return (raw - zero_raw) * direction * TURN_RAD / RAW_UNITS_PER_TURN


def derive_joint_candidate(
    observation: dict[str, Any],
    calibration: dict[str, Any],
    inset_raw: int,
) -> dict[str, Any]:
    coordinate = observation.get("coordinate")
    if coordinate == "semantic_raw":
        return {
            "status": "BLOCKED_SEMANTIC_GRIPPER_MAPPING_REQUIRED",
            "coordinate": coordinate,
            "automatic_limit_candidate": False,
            "observations": {
                key: value
                for key, value in observation.items()
                if key
                in {
                    "manual_sweep_minimum",
                    "manual_sweep_maximum",
                    "unloaded_closed",
                    "loaded_close_command",
                    "release_command",
                    "task_open",
                    "blocker",
                }
            },
            "runtime_change_authorized": False,
        }
    if coordinate not in ("raw", "unwrapped_raw"):
        raise ValueError(f"unsupported coordinate: {coordinate}")

    observed_minimum = int(observation["observed_minimum"])
    observed_maximum = int(observation["observed_maximum"])
    if observed_minimum >= observed_maximum:
        raise ValueError("observed interval is empty or reversed")
    if coordinate == "raw" and not (
        0 <= observed_minimum < observed_maximum < RAW_UNITS_PER_TURN
    ):
        raise ValueError("non-wrap raw interval must stay within 0..4095")
    if observed_maximum - observed_minimum <= 2 * inset_raw:
        raise ValueError("inset consumes the observed interval")

    candidate_minimum = observed_minimum + inset_raw
    candidate_maximum = observed_maximum - inset_raw
    zero_raw = int(calibration["zero_raw"])
    direction = int(calibration["positive_raw_direction"])
    first_rad = raw_coordinate_to_rad(
        candidate_minimum, zero_raw=zero_raw, direction=direction
    )
    second_rad = raw_coordinate_to_rad(
        candidate_maximum, zero_raw=zero_raw, direction=direction
    )
    contains_q0 = candidate_minimum <= zero_raw <= candidate_maximum
    return {
        "status": (
            "PLAN_ONLY_CONTRACTED_CANDIDATE"
            if contains_q0
            else "BLOCKED_Q0_OUTSIDE_CANDIDATE"
        ),
        "coordinate": coordinate,
        "wrap_aware": coordinate == "unwrapped_raw",
        "observed_desired_minimum_raw": observed_minimum,
        "observed_desired_maximum_raw": observed_maximum,
        "candidate_minimum_unwrapped_raw": candidate_minimum,
        "candidate_maximum_unwrapped_raw": candidate_maximum,
        "manual_observation_margin_lower_raw": inset_raw,
        "manual_observation_margin_upper_raw": inset_raw,
        "zero_raw": zero_raw,
        "positive_raw_direction": direction,
        "candidate_lower_rad": min(first_rad, second_rad),
        "candidate_upper_rad": max(first_rad, second_rad),
        "contains_q0": contains_q0,
        "runtime_change_authorized": False,
    }


def derive_candidate(
    manifest: dict[str, Any],
    calibrations: dict[str, tuple[dict[str, Any], ...]],
    inset_raw: int,
) -> dict[str, Any]:
    if manifest.get("status") != "J0_D_REVIEWED_PASS_J0_M_NOT_MEASURED":
        raise ValueError("source manifest status is not reviewed J0-D")
    if manifest.get("raw_units_per_turn") != RAW_UNITS_PER_TURN:
        raise ValueError("source raw-units contract changed")
    if not 1 <= inset_raw < RAW_UNITS_PER_TURN // 2:
        raise ValueError("inset_raw must be within 1..2047")

    arms: dict[str, Any] = {}
    q0_blockers: list[str] = []
    for arm_slot in ("left", "right"):
        source_joints = manifest["arms"][arm_slot]["joints"]
        output_joints: dict[str, Any] = {}
        for name, calibration in zip(
            JOINT_ORDER, calibrations[arm_slot], strict=True
        ):
            if name not in source_joints:
                raise ValueError(f"{arm_slot}: missing {name}")
            candidate = derive_joint_candidate(
                source_joints[name], calibration, inset_raw
            )
            candidate["evidence"] = source_joints[name].get("evidence")
            output_joints[name] = candidate
            if candidate["status"] == "BLOCKED_Q0_OUTSIDE_CANDIDATE":
                q0_blockers.append(f"{arm_slot}_{name}")
        arms[arm_slot] = {"joints": output_joints}

    blockers = [
        {
            "code": "J0_M_NOT_MEASURED",
            "meaning": (
                "the 64-raw inset is relative to manually traversed J0-D, "
                "not a mechanical endpoint"
            ),
        },
        {
            "code": "TASK_COVERAGE_AFTER_CONTRACTION_NOT_PROVEN",
            "meaning": (
                "representative routes and learned-policy outputs have not "
                "been checked against these contracted candidates"
            ),
        },
        {
            "code": "GRIPPER_SEMANTIC_MAPPING_PENDING",
            "meaning": (
                "jaw gap, contact behavior and hysteresis are not a generic "
                "revolute min/max contract"
            ),
        },
        {
            "code": "PHYSICAL_ZERO_AND_MODEL_ALIGNMENT_PENDING",
            "meaning": (
                "q0 raw=2048 is a software reference; physical link-angle "
                "offsets and the inter-base transform still need validation"
            ),
        },
        {
            "code": "RIGHT_ACTIVE_TRACKING_ENVELOPE_PENDING",
            "meaning": (
                "the right arm has no J2 torque-on tracking evidence yet"
            ),
        },
    ]
    if q0_blockers:
        blockers.append(
            {
                "code": "Q0_OUTSIDE_CANDIDATE",
                "joints": q0_blockers,
            }
        )
    return {
        "schema_version": 1,
        "record_kind": "bimanual_j1_operational_limit_candidate",
        "status": STATUS,
        "motion_authorized": False,
        "runtime_change_authorized": False,
        "execution_api_used": False,
        "candidate_margin_policy": {
            "inset_raw": inset_raw,
            "reference": "inside manually traversed J0-D envelope",
            "mechanical_margin_proven": False,
        },
        "arms": arms,
        "blockers": blockers,
        "parity_targets": {
            "firmware": False,
            "host": False,
            "urdf": False,
            "moveit": False,
            "isaac": False,
        },
        "next_gate": (
            "review contracted task coverage and resolve gripper/physical-zero "
            "blockers before generating any runtime calibration"
        ),
    }


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument(
        "--manifest",
        type=Path,
        default=root / "config/bimanual_j0_desired_envelope.reviewed.json",
    )
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument(
        "--left-calibration",
        type=Path,
        default=root
        / "ros2_ws/src/single_arm_bridge/config/single_arm_calibration.json",
    )
    parser.add_argument(
        "--right-calibration",
        type=Path,
        default=root / "config/right_arm_calibration.candidate.json",
    )
    parser.add_argument("--inset-raw", type=int, default=64)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.plan_only:
        raise SystemExit("--plan-only is required")
    manifest = load_sha_bound_json(args.manifest, args.manifest_sha256)
    calibrations = {
        "left": load_calibration(args.left_calibration, "left"),
        "right": load_calibration(args.right_calibration, "right"),
    }
    document = derive_candidate(manifest, calibrations, args.inset_raw)
    document["inputs"] = {
        "manifest": {
            "path": str(args.manifest),
            "sha256": args.manifest_sha256.lower(),
        },
        "left_calibration": {
            "path": str(args.left_calibration),
            "sha256": file_sha256(args.left_calibration),
        },
        "right_calibration": {
            "path": str(args.right_calibration),
            "sha256": file_sha256(args.right_calibration),
        },
    }
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    digest = file_sha256(output)
    for arm_slot in ("left", "right"):
        for name in JOINT_ORDER:
            joint = document["arms"][arm_slot]["joints"][name]
            if joint["status"] == "PLAN_ONLY_CONTRACTED_CANDIDATE":
                print(
                    "J1_LIMIT_CANDIDATE "
                    f"arm={arm_slot} joint={name} "
                    f"raw={joint['candidate_minimum_unwrapped_raw']}.."
                    f"{joint['candidate_maximum_unwrapped_raw']} "
                    f"rad={joint['candidate_lower_rad']:.6f}.."
                    f"{joint['candidate_upper_rad']:.6f}"
                )
            else:
                print(
                    "J1_LIMIT_BLOCKED "
                    f"arm={arm_slot} joint={name} "
                    f"reason={joint['status']}"
                )
    print(
        f"STATUS={STATUS} motion_authorized=false "
        f"output={output} sha256={digest}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
