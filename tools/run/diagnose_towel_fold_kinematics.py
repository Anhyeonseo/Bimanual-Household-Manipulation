#!/usr/bin/env python3
"""Solve the canonical towel sequence with full FK but without MoveIt.

The output is a motion-locked visualization and kinematic diagnostic.  It
does not claim collision-free transitions, controller reachability, cloth
attachment, or physical fold success.  The strict MoveIt runner remains the
only path that can promote a sequence beyond this diagnostic scope.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import time
from typing import Iterable

import yaml


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib import desk_task_planning as planning  # noqa: E402
from tools.lib.desk_task_runtime import CANONICAL_JOINTS  # noqa: E402
from tools.lib.grasp_yaw_kinematics import GraspYawKinematics  # noqa: E402
from tools.lib.towel_bimanual_then_single_planning import (  # noqa: E402
    build_bimanual_then_single_candidates,
)
from tools.lib.towel_task_pose_planning import (  # noqa: E402
    CandidateSpec,
    PhaseSpec,
    TowelPlanningError,
    build_correction_probes,
    phase_to_dict,
    solve_task_pose_branches,
    towel_bounds_from_worktable,
    validate_phase_contract,
)
from tools.lib.towel_task_runtime import validate_towel_contract  # noqa: E402


STATUS = "CANONICAL_TOWEL_FULL_FK_DIAGNOSTIC_PASS"
DEFAULT_CONTRACT = ROOT / "config/towel_task_contract.candidate.yaml"
DEFAULT_WORKTABLE = (
    ROOT
    / "ros2_ws/src/manipulation_camera_manager/config/"
    "top_worktable_homography.yaml"
)
DEFAULT_URDF = (
    ROOT
    / "ros2_ws/src/so101_description/urdf/"
    "so101_dual_right_data_fit_candidate.urdf"
)
ARM_INDICES = {
    "left": (0, 1, 2, 3, 4),
    "right": (6, 7, 8, 9, 10),
}


def load_mapping(path: Path) -> dict:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError(f"document root must be a mapping: {path}")
    return value


def file_sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def arm_values(full: Iterable[float], arm: str) -> tuple[float, ...]:
    values = tuple(float(value) for value in full)
    return tuple(values[index] for index in ARM_INDICES[arm])


def replace_arm(
    full: Iterable[float], arm: str, arm_positions: Iterable[float]
) -> tuple[float, ...]:
    values = list(float(value) for value in full)
    replacement = tuple(float(value) for value in arm_positions)
    if len(values) != len(CANONICAL_JOINTS) or len(replacement) != 5:
        raise RuntimeError("canonical state and arm replacement sizes are required")
    for index, value in zip(ARM_INDICES[arm], replacement, strict=True):
        values[index] = value
    return tuple(values)


def solve_phases(
    phases: tuple[PhaseSpec, ...],
    clear: tuple[float, ...],
    kinematics: dict[str, GraspYawKinematics],
    bounds: dict[str, tuple[object, object]],
) -> list[dict[str, object]]:
    validate_phase_contract(phases)
    current = clear
    records: list[dict[str, object]] = []
    for phase in phases:
        record = phase_to_dict(phase)
        evaluations: list[dict[str, object]] = []
        if phase.clear_pose:
            target = clear
            if phase.clear_arm is not None:
                target = replace_arm(
                    current,
                    phase.clear_arm,
                    arm_values(clear, phase.clear_arm),
                )
        else:
            target = current
            for task_target in phase.targets:
                lower, upper = bounds[task_target.arm]
                branches = solve_task_pose_branches(
                    kinematics[task_target.arm],
                    task_target,
                    lower,
                    upper,
                    arm_values(target, task_target.arm),
                    arm_values(clear, task_target.arm),
                )
                if not branches:
                    raise TowelPlanningError(
                        f"{phase.name}: no full-FK task-pose branch for "
                        f"{task_target.arm}"
                    )
                selected = dict(branches[0])
                selected["available_branch_count"] = len(branches)
                evaluations.append(selected)
                target = replace_arm(
                    target,
                    task_target.arm,
                    selected["positions_rad"],
                )
        record.update(
            {
                "full_fk_pass": True,
                "joint_positions_rad": list(target),
                "task_pose_evaluations": evaluations,
                "moveit_segment_planned": False,
                "transition_collision_checked": False,
            }
        )
        records.append(record)
        current = target
    return records


def candidate_by_human_direction(
    candidates: tuple[CandidateSpec, ...], active_arm: str, direction: str
) -> CandidateSpec:
    matches = [
        candidate
        for candidate in candidates
        if candidate.second_active_arm == active_arm
        and candidate.second_direction == direction
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"canonical candidate not unique: arm={active_arm} direction={direction}"
        )
    return matches[0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--worktable", type=Path, default=DEFAULT_WORKTABLE)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument(
        "--second-arm", choices=("right", "left"), default="right"
    )
    parser.add_argument(
        "--second-direction",
        choices=("right_to_left", "left_to_right"),
        default="right_to_left",
    )
    parser.add_argument("--skip-corrections", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    for path in (args.contract, args.worktable, args.urdf):
        if not path.is_file():
            parser.error(f"required source does not exist: {path}")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")
    return args


def main() -> int:
    args = parse_args()
    contract = load_mapping(args.contract)
    worktable = load_mapping(args.worktable)
    validate_towel_contract(contract)
    if (
        worktable.get("status")
        != "TABLE_BASE_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
        or worktable.get("base_registration", {}).get("transform_validated")
        is not True
    ):
        raise RuntimeError("worktable metric registration is not validated")
    observe = contract.get("workcell_observation_candidate", {}).get(
        "observe_clear", {}
    )
    clear = tuple(float(value) for value in observe.get("joint_positions_rad", ()))
    if len(clear) != len(CANONICAL_JOINTS):
        raise RuntimeError("contract does not contain the canonical clear state")
    board = worktable.get("board", {})
    towel_bounds = towel_bounds_from_worktable(
        board.get("calibrated_span_m", ()),
        board.get("origin_in_left_base_link_xy_m", ()),
    )
    table_z = float(board.get("table_z_in_left_base_link_m", math.nan))
    candidates = build_bimanual_then_single_candidates(towel_bounds, table_z)
    candidate = candidate_by_human_direction(
        candidates, args.second_arm, args.second_direction
    )
    kinematics = {
        arm: GraspYawKinematics(args.urdf, prefix=f"{arm}_")
        for arm in ("left", "right")
    }
    bounds = {
        arm: planning.load_arm_joint_bounds(arm)
        for arm in ("left", "right")
    }
    print(f"FULL_FK_DIAGNOSTIC_FIRST_BEGIN candidate={candidate.candidate_id}")
    first = solve_phases(
        candidate.first_fold_phases, clear, kinematics, bounds
    )
    print(f"FULL_FK_DIAGNOSTIC_FIRST_PASS phases={len(first)}")
    second = solve_phases(
        candidate.second_fold_phases, clear, kinematics, bounds
    )
    print(f"FULL_FK_DIAGNOSTIC_SECOND_PASS phases={len(second)}")
    corrections = []
    if not args.skip_corrections:
        for probe in build_correction_probes(
            candidate.first_expected_footprint_xyxy_m, table_z
        ):
            records = solve_phases(probe.phases, clear, kinematics, bounds)
            corrections.append(
                {
                    "probe_id": probe.probe_id,
                    "primitive": probe.primitive,
                    "arm": probe.arm,
                    "offset_xy_m": list(probe.offset_xy_m),
                    "phases": records,
                }
            )
            print(f"FULL_FK_DIAGNOSTIC_CORRECTION_PASS id={probe.probe_id}")
    document = {
        "schema_version": 1,
        "record_kind": "canonical_towel_fold_full_fk_diagnostic",
        "status": STATUS,
        "generated_at_unix_s": time.time(),
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "scope": {
            "full_fk_task_pose_checked": True,
            "moveit_segment_planning_checked": False,
            "transition_collision_checked": False,
            "cloth_attachment_checked": False,
            "physical_fold_success_checked": False,
        },
        "direction_labels": {
            "first_fold": "robot_near_bottom_to_far_top",
            "second_fold": candidate.second_direction,
        },
        "towel_placement": {
            "bounds_xyxy_m": list(towel_bounds),
            "table_z_m": table_z,
        },
        "clear_joint_positions_rad": list(clear),
        "selected_candidate": {
            "candidate_id": candidate.candidate_id,
            "first_arm_assignment": candidate.first_arm_assignment,
            "first_direction": candidate.first_direction,
            "second_active_arm": candidate.second_active_arm,
            "second_direction": candidate.second_direction,
            "first_expected_footprint_xyxy_m": list(
                candidate.first_expected_footprint_xyxy_m
            ),
            "final_expected_footprint_xyxy_m": list(
                candidate.final_expected_footprint_xyxy_m
            ),
            "first_fold": first,
            "second_fold": second,
            "correction_envelope": corrections,
        },
        "sources": {
            "contract": {
                "path": str(args.contract.resolve()),
                "sha256": file_sha256(args.contract),
            },
            "worktable": {
                "path": str(args.worktable.resolve()),
                "sha256": file_sha256(args.worktable),
            },
            "urdf": {
                "path": str(args.urdf.resolve()),
                "sha256": file_sha256(args.urdf),
                "registered_collision_model_claimed": False,
            },
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"{STATUS} first_phases={len(first)} second_phases={len(second)} "
        f"corrections={len(corrections)} motion_commands=0 "
        f"output={args.output}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, TowelPlanningError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(1) from None
