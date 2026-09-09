#!/usr/bin/env python3
"""Solve one conditional right-arm S2 correction from a raw Isaac shape.

The raw simulation mesh is used only as a geometry oracle for this motion-free
kinematic probe.  Runtime execution still requires a fresh settled, clear-arm
top-camera edge observation; this tool does not substitute simulator vertices
for that observation.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
from statistics import median
import sys
import time
from types import SimpleNamespace

import yaml


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib import desk_task_planning as planning  # noqa: E402
from tools.lib.grasp_yaw_kinematics import GraspYawKinematics  # noqa: E402
from tools.lib.towel_bimanual_then_single_planning import (  # noqa: E402
    build_right_arm_second_fold_correction,
)
from tools.lib.towel_second_fold_correction import (  # noqa: E402
    SecondFoldEdgeObservation,
    plan_right_arm_second_fold_correction,
)
from tools.run.diagnose_towel_fold_kinematics import solve_phases  # noqa: E402
from tools.lib.towel_task_pose_planning import (  # noqa: E402
    phase_to_dict,
    solve_task_pose_branches,
)


STATUS = "TOWEL_SECOND_FOLD_CORRECTION_CONDITIONAL_FULL_FK_PASS"
STRICT_MOVEIT_STATUS = (
    "TOWEL_SECOND_FOLD_CORRECTION_CONDITIONAL_STRICT_MOVEIT_PASS"
)
RAW_RESULT_KIND = "towel_isaac_s1_vertex_patch_place_release_result"
RAW_SECOND_FOLD_STATUS = (
    "R2_S2_LEFT_TO_RIGHT_TWO_LAYER_RAW_FOLD_EXECUTED_CAMERA_CORRECTION_NOT_YET_APPLIED"
)
DEFAULT_RAW_RESULT = (
    ROOT
    / "tmp/towel_second_fold_minus20mm_arc9_descent18_medium_open_release_20260903.json"
)
DEFAULT_CONTRACT = ROOT / "config/towel_task_contract.candidate.yaml"
DEFAULT_WORKTABLE = (
    ROOT
    / "ros2_ws/src/manipulation_camera_manager/config/"
    "top_worktable_homography.yaml"
)
DEFAULT_URDF = (
    ROOT
    / "artifacts/bimanual/preview/"
    "so101_dual_preview_right_registered_r0g_newton_baked_scale.urdf"
)
DEFAULT_APPROACH_REPLAY = (
    ROOT
    / "tmp/towel_second_fold_minus20mm_right_surface_press_correction_full_fk_20260904.json"
)
DEFAULT_OPERATIONAL_LIMITS = ROOT / "config/bimanual_operational_limits.json"
DEFAULT_CABLE_REVIEW = ROOT / "config/bimanual_j0_desired_envelope.reviewed.json"
DEFAULT_MANIFEST = (
    ROOT
    / "artifacts/bimanual/preview/"
    "so101_dual_preview_right_registered_r0g.manifest.json"
)
DEFAULT_SHADOW = (
    ROOT
    / "artifacts/calibration/top_eye_to_hand_20260825_r0c/"
    "right_workcell_shadow_validation.yaml"
)
DEFAULT_RIGHT_TABLETOP = (
    ROOT
    / "artifacts/calibration/right_tabletop_target_staged_20260826_r0/"
    "candidate.yaml"
)
DEFAULT_SECOND_LAYER_GRIPPER = (
    ROOT / "config/so101_gripper_s2_four_layer.candidate.json"
)


def file_sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def load_mapping(path: Path) -> dict[str, object]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError(f"document root must be a mapping: {path}")
    return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-result", type=Path, default=DEFAULT_RAW_RESULT)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--worktable", type=Path, default=DEFAULT_WORKTABLE)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument(
        "--approach-replay",
        type=Path,
        default=DEFAULT_APPROACH_REPLAY,
        help="hash-locked earlier correction FK used only for unchanged clear-to-pregrasp phases",
    )
    parser.add_argument(
        "--solve-all",
        action="store_true",
        help="solve all correction phases from the current raw observation instead of reusing an older approach",
    )
    parser.add_argument(
        "--strict-moveit",
        action="store_true",
        help=(
            "replace the FK-only phase rows with hash-locked MoveIt-planned "
            "targets and densely validate every collision state"
        ),
    )
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
    parser.add_argument(
        "--second-layer-gripper-config",
        type=Path,
        default=DEFAULT_SECOND_LAYER_GRIPPER,
    )
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    required_paths = [
        args.raw_result,
        args.contract,
        args.worktable,
        args.urdf,
    ]
    if not args.solve_all:
        required_paths.append(args.approach_replay)
    if args.strict_moveit:
        if not args.solve_all:
            parser.error("--strict-moveit requires --solve-all")
        required_paths.extend(
            (
                args.operational_limits,
                args.cable_review,
                args.registered_urdf_manifest,
                args.right_registration_shadow,
                args.right_tabletop_validation,
                args.second_layer_gripper_config,
            )
        )
    for path in required_paths:
        if not path.is_file():
            parser.error(f"required source does not exist: {path}")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")
    return args


def main() -> int:
    args = parse_args()
    raw = json.loads(args.raw_result.read_text(encoding="utf-8"))
    second = raw.get("second_fold", {}) if isinstance(raw, dict) else {}
    if (
        not isinstance(raw, dict)
        or raw.get("record_kind") != RAW_RESULT_KIND
        or raw.get("motion_authorized") is not False
        or not isinstance(second, dict)
        or second.get("status") != RAW_SECOND_FOLD_STATUS
    ):
        raise RuntimeError("source is not a motion-locked raw S2 Isaac result")
    nodes = raw.get("final_cloth_shape_local_m_env_0")
    if not isinstance(nodes, list):
        raise RuntimeError("raw S2 result has no terminal cloth mesh")
    grid_side = math.isqrt(len(nodes))
    if grid_side * grid_side != len(nodes):
        raise RuntimeError("raw S2 cloth mesh is not a square topology grid")
    parsed = [tuple(float(value) for value in node) for node in nodes]
    if any(len(node) != 3 or not all(math.isfinite(value) for value in node) for node in parsed):
        raise RuntimeError("raw S2 cloth mesh contains invalid XYZ")
    stationary_edge = parsed[:grid_side]
    moving_edge = parsed[-grid_side:]
    observation = SecondFoldEdgeObservation(
        moving_free_edge_y_m=median(node[1] for node in moving_edge),
        stationary_right_edge_y_m=median(node[1] for node in stationary_edge),
        moving_edge_x_span_m=(
            min(node[0] for node in moving_edge),
            max(node[0] for node in moving_edge),
        ),
        confidence=1.0,
        # This is an explicit conditional kinematic probe.  The source's
        # settle result is recorded below and must be replaced by a fresh
        # camera observation before any later runtime authorization.
        settled=True,
        clear_pose_verified=True,
        moving_edge_center_x_m=median(node[0] for node in moving_edge),
    )
    correction = plan_right_arm_second_fold_correction(observation)
    if not correction.required:
        raise RuntimeError("raw S2 result does not require a correction probe")

    contract = load_mapping(args.contract)
    observe = contract.get("workcell_observation_candidate", {})
    clear_record = observe.get("observe_clear", {}) if isinstance(observe, dict) else {}
    clear = tuple(float(value) for value in clear_record.get("joint_positions_rad", ()))
    if len(clear) != 12 or not all(math.isfinite(value) for value in clear):
        raise RuntimeError("contract does not contain the canonical clear state")
    worktable = load_mapping(args.worktable)
    board = worktable.get("board", {})
    table_z = float(board.get("table_z_in_left_base_link_m", math.nan))
    if not math.isfinite(table_z):
        raise RuntimeError("worktable table z is invalid")

    phases = build_right_arm_second_fold_correction(correction, table_z)
    moveit_records: list[dict[str, object]] | None = None
    strict_moveit_validation: dict[str, object] | None = None
    moveit_urdf_path: Path | None = None
    if args.strict_moveit:
        import rclpy
        from rclpy.node import Node

        from tools.run.plan_towel_fold_sequence_once import (
            MoveItPlanOnlyGate,
            gripper_modes,
            solve_and_plan_phases,
            strip_runtime_values,
            summarize_strict_records,
            validate_inputs,
        )

        validated = validate_inputs(
            SimpleNamespace(
                contract=args.contract,
                worktable=args.worktable,
                operational_limits=args.operational_limits,
                cable_review=args.cable_review,
                registered_urdf_manifest=args.registered_urdf_manifest,
                right_registration_shadow=args.right_registration_shadow,
                right_tabletop_validation=args.right_tabletop_validation,
                second_layer_gripper_config=args.second_layer_gripper_config,
            )
        )
        moveit_urdf_path = Path(validated["urdf_path"])
        kinematics = {
            arm: GraspYawKinematics(moveit_urdf_path, prefix=f"{arm}_")
            for arm in ("left", "right")
        }
    else:
        kinematics = {
            arm: GraspYawKinematics(args.urdf, prefix=f"{arm}_")
            for arm in ("left", "right")
        }
    bounds = {arm: planning.load_arm_joint_bounds(arm) for arm in ("left", "right")}
    if args.strict_moveit:
        grippers = gripper_modes(
            contract,
            validated["cable"],
            validated["second_layer_gripper"],
        )
        rclpy.init()
        node = Node("towel_second_fold_correction_strict_plan_only")
        gate = MoveItPlanOnlyGate(
            node, args.timeout_s, grippers, clear, kinematics
        )
        try:
            gate.wait()
            gate.apply_table_and_read_matrix(worktable)
            _, moveit_records = solve_and_plan_phases(
                gate,
                phases,
                clear,
                clear,
                kinematics,
                bounds,
            )
        finally:
            try:
                gate.restore()
            finally:
                node.destroy_node()
                rclpy.shutdown()
        expected_names = {phase.name for phase in phases}
        records = [
            record for record in moveit_records if record.get("name") in expected_names
        ]
        if len(records) != len(phases):
            raise RuntimeError(
                "strict MoveIt output did not preserve every logical correction phase"
            )
        for record in records:
            record["joint_positions_rad"] = list(
                record["moveit"]["target_positions_rad"]
            )
        strict_moveit_validation = summarize_strict_records((moveit_records,))
        strip_runtime_values(moveit_records)
    elif args.solve_all:
        records = solve_phases(
            phases,
            clear,
            kinematics,
            bounds,
            prefer_continuous_seed=True,
        )
    else:
        approach_replay = json.loads(
            args.approach_replay.read_text(encoding="utf-8")
        )
        approach_records = approach_replay.get("phases", [])
        approach_phases = tuple(
            phase
            for phase in phases
            if phase.name.startswith("second_correction_departure_")
        )
        if (
            approach_replay.get("status") != STATUS
            or approach_replay.get("motion_authorized") is not False
            or len(approach_phases) != 40
            or len(approach_records) < len(approach_phases)
            or approach_replay.get("sources", {})
            .get("raw_result", {})
            .get("sha256")
            != file_sha256(args.raw_result)
            or approach_replay.get("sources", {}).get("urdf", {}).get("sha256")
            != file_sha256(args.urdf)
        ):
            raise RuntimeError("approach replay is not the matching hash-locked FK source")
        approach_solution_records: list[dict[str, object]] = []
        for phase, previous in zip(approach_phases, approach_records, strict=False):
            if previous.get("name") != phase.name or len(phase.targets) != 1:
                raise RuntimeError("approach replay phase sequence changed")
            warm_start_joints = tuple(
                float(value) for value in previous.get("joint_positions_rad", ())
            )
            if len(warm_start_joints) != len(clear):
                raise RuntimeError("approach replay joint row has the wrong width")
            target = phase.targets[0]
            lower, upper = bounds[target.arm]
            branches = solve_task_pose_branches(
                kinematics[target.arm],
                target,
                lower,
                upper,
                warm_start_joints[6:11],
                clear[6:11],
                random_seed_count=0,
            )
            if not branches:
                branches = solve_task_pose_branches(
                    kinematics[target.arm],
                    target,
                    lower,
                    upper,
                    warm_start_joints[6:11],
                    clear[6:11],
                )
            if not branches:
                raise RuntimeError(f"warm-started approach FK failed: {phase.name}")
            evaluation = dict(branches[0])
            joints = tuple(clear[:6]) + tuple(
                float(value) for value in evaluation["positions_rad"]
            ) + tuple(clear[11:])
            record = phase_to_dict(phase)
            record.update(
                {
                    "full_fk_pass": True,
                    "joint_positions_rad": list(joints),
                    "task_pose_evaluations": [evaluation],
                    "moveit_segment_planned": False,
                    "transition_collision_checked": False,
                    "warm_started_from_hash_locked_approach_fk": True,
                }
            )
            approach_solution_records.append(record)
        tail_phases = phases[len(approach_phases) :]
        tail_records = solve_phases(
            tail_phases,
            clear,
            kinematics,
            bounds,
            prefer_continuous_seed=True,
            initial_joint_positions=tuple(
                float(value)
                for value in approach_solution_records[-1]["joint_positions_rad"]
            ),
        )
        records = approach_solution_records + tail_records
    margins = [
        float(evaluation["minimum_joint_limit_margin_rad"])
        for record in records
        for evaluation in record.get("task_pose_evaluations", [])
    ]
    if not margins:
        raise RuntimeError("correction diagnostic produced no FK evaluations")

    document = {
        "schema_version": 1,
        "record_kind": "towel_second_fold_correction_full_fk_diagnostic",
        "status": STRICT_MOVEIT_STATUS if args.strict_moveit else STATUS,
        "created_unix_s": time.time(),
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "scope": "conditional_single_correction_step_then_mandatory_reobservation",
        "runtime_preconditions": {
            "fresh_top_camera_edge_observation_required": True,
            "settled_cloth_required": True,
            "both_arms_clear_during_observation": True,
            "simulator_mesh_is_not_a_runtime_observation": True,
        },
        "source_settle_gate_passed": bool(second.get("settle_gate_passed")),
        "geometry_oracle_observation": {
            "moving_free_edge_y_m": observation.moving_free_edge_y_m,
            "stationary_right_edge_y_m": observation.stationary_right_edge_y_m,
            "moving_edge_x_span_m": list(observation.moving_edge_x_span_m),
            "moving_edge_center_x_m": observation.moving_edge_center_x_m,
        },
        "correction_plan": correction.to_dict(),
        "phases": records,
        "minimum_joint_limit_margin_rad": min(margins),
        "sources": {
            name: {"path": str(path.resolve()), "sha256": file_sha256(path)}
            for name, path in (
                ("raw_result", args.raw_result),
                ("contract", args.contract),
                ("worktable", args.worktable),
                ("urdf", args.urdf),
            )
        },
    }
    if args.strict_moveit:
        document["strict_moveit_validated"] = True
        document["strict_moveit_validation"] = strict_moveit_validation
        document["strict_moveit_records"] = moveit_records
        document["sources"]["moveit_urdf"] = {
            "path": str(moveit_urdf_path.resolve()),
            "sha256": file_sha256(moveit_urdf_path),
        }
    if not args.solve_all:
        document["sources"]["approach_replay"] = {
            "path": str(args.approach_replay.resolve()),
            "sha256": file_sha256(args.approach_replay),
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        f"{document['status']} phases={len(records)} correction_step_m="
        f"{correction.correction_delta_y_m:.6f} remaining_m="
        f"{correction.expected_remaining_residual_m:.6f} "
        f"minimum_joint_margin_rad={min(margins):.6f} motion_commands=0 "
        f"strict_states={strict_moveit_validation['strict_state_sample_count'] if strict_moveit_validation else 0} "
        f"output={args.output}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(1) from None
