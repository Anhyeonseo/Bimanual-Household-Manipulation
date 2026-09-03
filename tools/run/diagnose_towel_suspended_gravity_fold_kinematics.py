#!/usr/bin/env python3
"""Solve the suspended-gravity first fold with full FK and no robot command."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.grasp_yaw_kinematics import GraspYawKinematics  # noqa: E402
from tools.lib.towel_suspended_gravity_fold_planning import (  # noqa: E402
    build_suspended_gravity_first_fold,
)
from tools.lib.towel_task_pose_planning import (  # noqa: E402
    phase_to_dict,
    solve_task_pose_branches,
    towel_bounds_from_worktable,
    validate_phase_contract,
)


STATUS = "TOWEL_SUSPENDED_GRAVITY_FULL_FK_DIAGNOSTIC_PASS"
DEFAULT_WORKTABLE = (
    ROOT
    / "ros2_ws/src/manipulation_camera_manager/config/"
    "top_worktable_homography.yaml"
)
DEFAULT_URDF = (
    ROOT
    / "artifacts/bimanual/preview/"
    "so101_dual_preview_right_registered_r0g.urdf"
)
DEFAULT_LIMITS = ROOT / "config/bimanual_operational_limits.json"
DEFAULT_CONTACT = ROOT / "config/towel_first_fold_vertical_contact.candidate.json"
DEFAULT_CONTRACT = ROOT / "config/towel_task_contract.candidate.yaml"
ARM_LIMIT_NAMES = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll")
ARM_CLEAR_INDICES = {"left": (0, 1, 2, 3, 4), "right": (6, 7, 8, 9, 10)}


def file_sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worktable", type=Path, default=DEFAULT_WORKTABLE)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--operational-limits", type=Path, default=DEFAULT_LIMITS)
    parser.add_argument("--contact-config", type=Path, default=DEFAULT_CONTACT)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--towel-x-offset-m", type=float, default=-0.020)
    parser.add_argument("--contact-tcp-z-offset-m", type=float, default=0.015)
    parser.add_argument(
        "--ik-random-seed-count",
        type=int,
        default=18,
        help="number of deterministic random IK seeds per target",
    )
    parser.add_argument(
        "--laydown-tcp-z-offset-m",
        type=float,
        help=(
            "optional terminal laydown height above the table; the contact "
            "height remains controlled independently"
        ),
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    for path in (
        args.worktable,
        args.urdf,
        args.operational_limits,
        args.contact_config,
        args.contract,
    ):
        if not path.is_file():
            parser.error(f"required source does not exist: {path}")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")
    if not math.isfinite(args.towel_x_offset_m):
        parser.error("--towel-x-offset-m must be finite")
    if not math.isfinite(args.contact_tcp_z_offset_m):
        parser.error("--contact-tcp-z-offset-m must be finite")
    if args.ik_random_seed_count < 0:
        parser.error("--ik-random-seed-count must be nonnegative")
    if (
        args.laydown_tcp_z_offset_m is not None
        and not math.isfinite(args.laydown_tcp_z_offset_m)
    ):
        parser.error("--laydown-tcp-z-offset-m must be finite")
    return args


def main() -> int:
    args = parse_args()
    worktable = yaml.safe_load(args.worktable.read_text(encoding="utf-8"))
    limits = json.loads(args.operational_limits.read_text(encoding="utf-8"))
    contact = json.loads(args.contact_config.read_text(encoding="utf-8"))
    contract = yaml.safe_load(args.contract.read_text(encoding="utf-8"))
    if (
        worktable.get("status")
        != "TABLE_BASE_VALIDATED_MOTION_STILL_NOT_AUTHORIZED"
        or limits.get("status") != "OPERATOR_VERIFIED_FULL_TASK_ENVELOPE"
        or contact.get("motion_authorized") is not False
        or contract.get("motion_authorized") is not False
    ):
        raise RuntimeError("a required motion-locked source is invalid")

    board = worktable["board"]
    bounds = list(
        towel_bounds_from_worktable(
            board["calibrated_span_m"], board["origin_in_left_base_link_xy_m"]
        )
    )
    bounds[0] += args.towel_x_offset_m
    bounds[1] += args.towel_x_offset_m
    table_z = float(board["table_z_in_left_base_link_m"])
    spec = build_suspended_gravity_first_fold(
        bounds,
        table_z,
        contact_tcp_z_offset_m=args.contact_tcp_z_offset_m,
        laydown_tcp_z_offset_m=args.laydown_tcp_z_offset_m,
    )
    validate_phase_contract(spec.phases)
    contact_index = next(
        index
        for index, phase in enumerate(spec.phases)
        if phase.name == "first_contact"
    )
    phases = spec.phases[contact_index:]

    kinematics = {
        arm: GraspYawKinematics(args.urdf, prefix=f"{arm}_")
        for arm in ("left", "right")
    }
    joint_bounds = {}
    for arm in ("left", "right"):
        arm_limits = limits["arms"][arm]
        joint_bounds[arm] = (
            np.asarray(
                [arm_limits[name]["minimum_urad"] / 1.0e6 for name in ARM_LIMIT_NAMES]
            ),
            np.asarray(
                [arm_limits[name]["maximum_urad"] / 1.0e6 for name in ARM_LIMIT_NAMES]
            ),
        )
    clear_full = tuple(
        float(value)
        for value in contract["workcell_observation_candidate"]["observe_clear"][
            "joint_positions_rad"
        ]
    )
    clear_by_arm = {
        arm: tuple(clear_full[index] for index in ARM_CLEAR_INDICES[arm])
        for arm in ("left", "right")
    }
    current = {
        arm: tuple(float(value) for value in contact["arm_joint_positions_rad"][arm])
        for arm in ("left", "right")
    }

    records = []
    minimum_margin = math.inf
    maximum_tilt = 0.0
    for phase in phases:
        record = phase_to_dict(phase)
        evaluations = []
        if phase.clear_pose:
            for arm in ("left", "right"):
                current[arm] = clear_by_arm[arm]
        else:
            for target in phase.targets:
                lower, upper = joint_bounds[target.arm]
                branches = solve_task_pose_branches(
                    kinematics[target.arm],
                    target,
                    lower,
                    upper,
                    current[target.arm],
                    clear_by_arm[target.arm],
                    random_seed_count=args.ik_random_seed_count,
                )
                if not branches:
                    raise RuntimeError(
                        f"{phase.name}: no full-FK branch for {target.arm}"
                    )
                selected = dict(branches[0])
                selected["available_branch_count"] = len(branches)
                evaluations.append(selected)
                current[target.arm] = tuple(
                    float(value) for value in selected["positions_rad"]
                )
                minimum_margin = min(
                    minimum_margin,
                    float(selected["minimum_joint_limit_margin_rad"]),
                )
                maximum_tilt = max(
                    maximum_tilt,
                    float(selected["approach_tilt_from_down_rad"]),
                )
        record["task_pose_evaluations"] = evaluations
        record["arm_joint_positions_rad"] = {
            arm: list(current[arm]) for arm in ("left", "right")
        }
        combined = list(clear_full)
        for arm, indices in ARM_CLEAR_INDICES.items():
            for joint_index, value in zip(indices, current[arm], strict=True):
                combined[joint_index] = float(value)
        record["joint_positions_rad"] = combined
        record["moveit_segment_planned"] = False
        record["transition_collision_checked"] = False
        records.append(record)
        print(f"SUSPENDED_GRAVITY_FULL_FK_PASS phase={phase.name}", flush=True)

    result = {
        "schema_version": 1,
        "record_kind": "towel_suspended_gravity_full_fk_diagnostic",
        "status": STATUS,
        "created_unix_s": time.time(),
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "scope": "full_fk_only_no_moveit_no_robot_command",
        "towel_bounds_xyxy_m": bounds,
        "towel_placement": {"bounds_xyxy_m": bounds},
        "table_z_m": table_z,
        "contact_tcp_z_offset_m": args.contact_tcp_z_offset_m,
        "laydown_tcp_z_offset_m": (
            args.laydown_tcp_z_offset_m
            if args.laydown_tcp_z_offset_m is not None
            else args.contact_tcp_z_offset_m
        ),
        "landmarks": {
            "initial_grasp_x_m": spec.initial_grasp_x_m,
            "free_edge_anchor_x_m": spec.free_edge_anchor_x_m,
            "fold_line_x_m": spec.fold_line_x_m,
            "final_held_grasp_x_m": spec.final_held_grasp_x_m,
            "suspend_tcp_z_m": spec.suspend_tcp_z_m,
            "free_edge_touchdown_tcp_z_m": spec.free_edge_touchdown_tcp_z_m,
            "touchdown_slack_feed_m": spec.touchdown_slack_feed_m,
            "l_shape_tcp_z_m": spec.l_shape_tcp_z_m,
            "supported_lower_layer_fraction": (
                spec.supported_lower_layer_fraction
            ),
            "forward_lay_slack_m": spec.forward_lay_slack_m,
            "final_held_edge_x_m": spec.final_held_edge_x_m,
            "exposed_lower_strip_m": spec.exposed_lower_strip_m,
            "upper_edge_support_depth_m": spec.upper_edge_support_depth_m,
            "raw_fold_alignment_policy": (
                "target_50pct_then_post_release_accept_within_55_45_envelope"
            ),
            "laydown_tcp_z_m": spec.laydown_tcp_z_m,
            "expected_footprint_xyxy_m": list(spec.expected_footprint_xyxy_m),
        },
        "required_observation_gates": {
            "free_edge_straight_after_touchdown": True,
            "post_release_residual_correction": False,
        },
        "minimum_joint_limit_margin_rad": minimum_margin,
        "maximum_approach_tilt_from_down_rad": maximum_tilt,
        "phases": records,
        "selected_candidate": {
            "candidate_id": "first_bimanual_suspended_gravity_near_anchor",
            "first_expected_footprint_xyxy_m": list(
                spec.expected_footprint_xyxy_m
            ),
            "first_fold": records,
        },
        "sources": {
            name: {"path": str(path.resolve()), "sha256": file_sha256(path)}
            for name, path in (
                ("worktable", args.worktable),
                ("urdf", args.urdf),
                ("operational_limits", args.operational_limits),
                ("contact_config", args.contact_config),
                ("contract", args.contract),
            )
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(
        f"{STATUS} phases={len(records)} "
        f"minimum_margin_rad={minimum_margin:.6f} "
        f"maximum_tilt_deg={math.degrees(maximum_tilt):.3f} "
        f"output={args.output}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
