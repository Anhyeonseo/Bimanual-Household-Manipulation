#!/usr/bin/env python3
"""Run the suspended-gravity first fold through the strict MoveIt gate only."""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib import desk_task_planning as planning  # noqa: E402
from tools.lib.grasp_yaw_kinematics import GraspYawKinematics  # noqa: E402
from tools.lib.towel_suspended_gravity_fold_planning import (  # noqa: E402
    build_suspended_gravity_first_fold,
)
from tools.lib.towel_task_pose_planning import (  # noqa: E402
    solve_task_pose_branches,
    towel_bounds_from_worktable,
    validate_phase_contract,
)
from tools.run.plan_towel_fold_sequence_once import (  # noqa: E402
    DEFAULT_CABLE_REVIEW,
    DEFAULT_CONTRACT,
    DEFAULT_MANIFEST,
    DEFAULT_OPERATIONAL_LIMITS,
    DEFAULT_RIGHT_TABLETOP,
    DEFAULT_SHADOW,
    DEFAULT_WORKTABLE,
    MoveItPlanOnlyGate,
    gripper_modes,
    sha256_file,
    solve_and_plan_phases,
    strip_runtime_values,
    summarize_strict_records,
    validate_inputs,
)
from tools.run.diagnose_towel_fold_kinematics import replace_arm  # noqa: E402


STATUS = "TOWEL_SUSPENDED_GRAVITY_MOVEIT_PLAN_ONLY_PASS"
CONTACT_STATUS = "TOWEL_SUSPENDED_GRAVITY_CONTACT_MOVEIT_VALID"
DEFAULT_CONTACT = ROOT / "config/towel_first_fold_vertical_contact.candidate.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true", required=True)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--worktable", type=Path, default=DEFAULT_WORKTABLE)
    parser.add_argument(
        "--operational-limits", type=Path, default=DEFAULT_OPERATIONAL_LIMITS
    )
    parser.add_argument("--cable-review", type=Path, default=DEFAULT_CABLE_REVIEW)
    parser.add_argument(
        "--registered-urdf-manifest", type=Path, default=DEFAULT_MANIFEST
    )
    parser.add_argument(
        "--right-registration-shadow", type=Path, default=DEFAULT_SHADOW
    )
    parser.add_argument(
        "--right-tabletop-validation", type=Path, default=DEFAULT_RIGHT_TABLETOP
    )
    parser.add_argument("--towel-x-offset-m", type=float, default=-0.020)
    parser.add_argument("--contact-tcp-z-offset-m", type=float, default=0.015)
    parser.add_argument("--contact-config", type=Path, default=DEFAULT_CONTACT)
    parser.add_argument(
        "--suffix-from-vertical-contact",
        action="store_true",
        help="audit only the new attached suffix from the measured contact seed",
    )
    parser.add_argument(
        "--contact-only",
        action="store_true",
        help=(
            "resolve and save one collision-valid contact state, then stop "
            "without planning the suspended-fold suffix"
        ),
    )
    parser.add_argument(
        "--all-contact-candidates",
        action="store_true",
        help="save every collision-valid IK branch combination at contact",
    )
    parser.add_argument(
        "--stop-before-reobserve-clear",
        action="store_true",
        help="validate through the collision-safe outboard observation posture",
    )
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive and finite")
    if not math.isfinite(args.towel_x_offset_m):
        parser.error("--towel-x-offset-m must be finite")
    if not math.isfinite(args.contact_tcp_z_offset_m):
        parser.error("--contact-tcp-z-offset-m must be finite")
    if args.suffix_from_vertical_contact and not args.contact_config.is_file():
        parser.error(f"contact config does not exist: {args.contact_config}")
    if args.contact_only and not args.suffix_from_vertical_contact:
        parser.error("--contact-only requires --suffix-from-vertical-contact")
    if args.all_contact_candidates and not args.contact_only:
        parser.error("--all-contact-candidates requires --contact-only")
    if args.stop_before_reobserve_clear and not args.suffix_from_vertical_contact:
        parser.error(
            "--stop-before-reobserve-clear requires --suffix-from-vertical-contact"
        )
    return args


def main() -> int:
    args = parse_args()
    inputs = validate_inputs(args)
    contract = inputs["contract"]
    worktable = inputs["worktable"]
    board = worktable["board"]
    towel_bounds = list(
        towel_bounds_from_worktable(
            board["calibrated_span_m"], board["origin_in_left_base_link_xy_m"]
        )
    )
    towel_bounds[0] += args.towel_x_offset_m
    towel_bounds[1] += args.towel_x_offset_m
    table_origin_x = float(board["origin_in_left_base_link_xy_m"][0])
    table_right_x = table_origin_x + float(board["calibrated_span_m"][0])
    if towel_bounds[0] < table_origin_x or towel_bounds[1] > table_right_x:
        raise RuntimeError("offset towel leaves the validated worktable")
    table_z = float(board["table_z_in_left_base_link_m"])
    spec = build_suspended_gravity_first_fold(
        towel_bounds,
        table_z,
        contact_tcp_z_offset_m=args.contact_tcp_z_offset_m,
    )
    validate_phase_contract(spec.phases)

    clear = tuple(
        float(value)
        for value in contract["workcell_observation_candidate"]["observe_clear"][
            "joint_positions_rad"
        ]
    )
    kinematics = {
        side: GraspYawKinematics(inputs["urdf_path"], prefix=f"{side}_")
        for side in ("left", "right")
    }
    bounds = {
        side: planning.load_arm_joint_bounds(side) for side in ("left", "right")
    }
    grippers = gripper_modes(contract, inputs["cable"])
    initial = clear
    phases = spec.phases
    contact_source = None
    contact_phase = None
    if args.suffix_from_vertical_contact:
        contact_source = json.loads(args.contact_config.read_text(encoding="utf-8"))
        if (
            contact_source.get("record_kind")
            not in {
                "towel_first_fold_vertical_contact_candidate",
                "towel_suspended_gravity_contact_moveit_candidate",
            }
            or contact_source.get("motion_authorized") is not False
        ):
            raise RuntimeError("vertical-contact source is invalid")
        for arm in ("left", "right"):
            initial = replace_arm(
                initial,
                arm,
                contact_source["arm_joint_positions_rad"][arm],
            )
        contact_index = next(
            index
            for index, phase in enumerate(spec.phases)
            if phase.name == "first_contact"
        )
        contact_phase = spec.phases[contact_index]
        phases = spec.phases[contact_index + 1 :]
        if args.stop_before_reobserve_clear:
            phases = tuple(
                phase
                for phase in phases
                if phase.name != "first_gravity_reobserve_clear"
            )

    rclpy.init()
    node = Node("towel_suspended_gravity_plan_only")
    gate = MoveItPlanOnlyGate(node, args.timeout_s, grippers, clear, kinematics)
    try:
        gate.wait()
        gate.apply_table_and_read_matrix(worktable)
        gate.set_exceptions(True)
        initial_valid, initial_reason = gate.endpoint_valid(initial)
        contact_start_re_solved = False
        valid_contact_states = [initial] if initial_valid else []
        if (
            not initial_valid
            and args.suffix_from_vertical_contact
            and contact_phase is not None
        ):
            branch_sets = []
            for target in contact_phase.targets:
                lower, upper = bounds[target.arm]
                branches = solve_task_pose_branches(
                    kinematics[target.arm],
                    target,
                    lower,
                    upper,
                    tuple(
                        float(value)
                        for value in contact_source["arm_joint_positions_rad"][
                            target.arm
                        ]
                    ),
                    tuple(
                        clear[index]
                        for index in (
                            (0, 1, 2, 3, 4)
                            if target.arm == "left"
                            else (6, 7, 8, 9, 10)
                        )
                    ),
                )
                if not branches:
                    raise RuntimeError(
                        f"collision-safe contact has no IK for {target.arm}"
                    )
                branch_sets.append((target.arm, branches))
            rejected_contact_states = []
            for combination in itertools.product(
                *(branches for _, branches in branch_sets)
            ):
                candidate = clear
                for (arm, _), branch in zip(
                    branch_sets, combination, strict=True
                ):
                    candidate = replace_arm(candidate, arm, branch["positions_rad"])
                valid, reason = gate.endpoint_valid(candidate)
                if valid:
                    valid_contact_states.append(candidate)
                    if not initial_valid:
                        initial = candidate
                        initial_valid = True
                        initial_reason = ""
                        contact_start_re_solved = True
                    if not args.all_contact_candidates:
                        break
                rejected_contact_states.append(reason)
            if not initial_valid:
                unique = list(dict.fromkeys(rejected_contact_states))
                raise RuntimeError(
                    "no collision-safe diagonal contact branch; "
                    + " | ".join(unique[:4])
                )
        if not initial_valid:
            raise RuntimeError(
                "selected initial state is invalid before planning: "
                f"{initial_reason}"
            )
        print(
            "TOWEL_SUSPENDED_GRAVITY_INITIAL_STATE_VALID "
            f"scope={'suffix' if args.suffix_from_vertical_contact else 'complete'} "
            f"contact_re_solved={str(contact_start_re_solved).lower()}",
            flush=True,
        )
        if args.contact_only:
            arm_positions = {
                arm: [float(initial[index]) for index in indices]
                for arm, indices in {
                    "left": (0, 1, 2, 3, 4),
                    "right": (6, 7, 8, 9, 10),
                }.items()
            }
            result = {
                "schema_version": 1,
                "record_kind": "towel_suspended_gravity_contact_moveit_candidate",
                "status": CONTACT_STATUS,
                "created_unix_s": time.time(),
                "motion_authorized": False,
                "automatic_execution_permitted": False,
                "scope": "contact_endpoint_only_no_robot_command",
                "contact_start_re_solved_for_moveit": contact_start_re_solved,
                "joint_positions_rad": [float(value) for value in initial],
                "arm_joint_positions_rad": arm_positions,
                "candidates": [
                    {
                        "candidate_index": index,
                        "joint_positions_rad": [float(value) for value in state],
                        "arm_joint_positions_rad": {
                            arm: [float(state[joint_index]) for joint_index in indices]
                            for arm, indices in {
                                "left": (0, 1, 2, 3, 4),
                                "right": (6, 7, 8, 9, 10),
                            }.items()
                        },
                    }
                    for index, state in enumerate(valid_contact_states)
                ],
                "towel_bounds_xyxy_m": towel_bounds,
                "table_z_m": table_z,
                "contact_tcp_z_offset_m": args.contact_tcp_z_offset_m,
                "sources": {
                    "vertical_contact": {
                        "path": str(args.contact_config.resolve()),
                        "sha256": sha256_file(args.contact_config),
                    },
                    "registered_urdf": {
                        "path": str(Path(inputs["urdf_path"]).resolve()),
                        "sha256": sha256_file(Path(inputs["urdf_path"])),
                    },
                },
            }
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps(result, indent=2) + "\n", encoding="utf-8"
            )
            print(f"{CONTACT_STATUS} output={args.output}", flush=True)
            return 0
        end, records = solve_and_plan_phases(
            gate,
            phases,
            initial,
            clear,
            kinematics,
            bounds,
        )
        if not args.stop_before_reobserve_clear and end != clear:
            raise RuntimeError("suspended first fold did not return to OBSERVE_CLEAR")
    finally:
        try:
            gate.restore()
        finally:
            node.destroy_node()
            rclpy.shutdown()

    strict = summarize_strict_records([records])
    minimum_margin = min(
        float(evaluation["minimum_joint_limit_margin_rad"])
        for phase in records
        for evaluation in phase.get("task_pose_evaluations", [])
        if "minimum_joint_limit_margin_rad" in evaluation
    )
    planning_time = sum(
        float(phase["moveit"]["planning_time_s"]) for phase in records
    )
    strip_runtime_values(records)
    result = {
        "schema_version": 1,
        "record_kind": "towel_suspended_gravity_moveit_plan_only",
        "status": STATUS,
        "created_unix_s": time.time(),
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "scope": (
            "attached_suffix_through_outboard_observation"
            if args.stop_before_reobserve_clear
            else "attached_suffix_from_vertical_contact"
            if args.suffix_from_vertical_contact
            else "clear_to_clear_complete_first_fold"
        ),
        "contact_start_re_solved_for_moveit": contact_start_re_solved,
        "towel_bounds_xyxy_m": towel_bounds,
        "table_z_m": table_z,
        "contact_tcp_z_offset_m": args.contact_tcp_z_offset_m,
        "landmarks": {
            "initial_grasp_x_m": spec.initial_grasp_x_m,
            "free_edge_anchor_x_m": spec.free_edge_anchor_x_m,
            "fold_line_x_m": spec.fold_line_x_m,
            "final_held_grasp_x_m": spec.final_held_grasp_x_m,
            "suspend_tcp_z_m": spec.suspend_tcp_z_m,
            "free_edge_touchdown_tcp_z_m": spec.free_edge_touchdown_tcp_z_m,
            "l_shape_tcp_z_m": spec.l_shape_tcp_z_m,
            "expected_footprint_xyxy_m": list(spec.expected_footprint_xyxy_m),
        },
        "required_observation_gates": {
            "free_edge_straight_after_touchdown": True,
            "post_release_residual_correction": False,
        },
        "minimum_joint_limit_margin_rad": minimum_margin,
        "total_moveit_planning_time_s": planning_time,
        "strict_validation": strict,
        "phases": records,
        "sources": {
            "registered_urdf": {
                "path": str(Path(inputs["urdf_path"]).resolve()),
                "sha256": sha256_file(Path(inputs["urdf_path"])),
            },
            "full_fk_diagnostic": {
                "path": str(
                    (ROOT / "tmp/towel_suspended_gravity_full_fk_20260902.json").resolve()
                ),
                "advisory_only": True,
            },
            "vertical_contact": (
                None
                if contact_source is None
                else {
                    "path": str(args.contact_config.resolve()),
                    "sha256": sha256_file(args.contact_config),
                }
            ),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(
        f"{STATUS} phases={len(records)} "
        f"strict_states={strict['strict_state_sample_count']} "
        f"minimum_margin_rad={minimum_margin:.6f} output={args.output}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
