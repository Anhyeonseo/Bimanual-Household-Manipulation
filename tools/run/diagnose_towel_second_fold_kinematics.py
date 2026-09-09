#!/usr/bin/env python3
"""Solve only the accepted-S1 to left-to-right S2 path with full FK.

This diagnostic deliberately does not recompute the retired first-fold arc.
It hash-locks one accepted S1 terminal cloth shape, derives its actual metric
footprint, and solves the left-arm moving-edge-midpoint second fold.  No ROS
publisher, controller, or motion service is used.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import time

import yaml


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib import desk_task_planning as planning  # noqa: E402
from tools.lib.grasp_yaw_kinematics import GraspYawKinematics  # noqa: E402
from tools.lib.towel_bimanual_then_single_planning import (  # noqa: E402
    build_right_arm_second_fold_edge_handoff,
    build_right_arm_second_fold_stabilizer,
    build_single_arm_second_fold,
)
from tools.lib.towel_second_fold_correction import (  # noqa: E402
    SECOND_FOLD_ACTIVE_ARM,
    SECOND_FOLD_CORRECTION_ARM,
    SECOND_FOLD_DIRECTION,
    SECOND_FOLD_MAXIMUM_CORRECTION_M,
    SECOND_FOLD_MINIMUM_EDGE_CONFIDENCE,
    SECOND_FOLD_TARGET_TOLERANCE_M,
    load_accepted_first_fold_footprint,
)
from tools.run.diagnose_towel_fold_kinematics import solve_phases  # noqa: E402


STATUS = "TOWEL_SECOND_FOLD_LEFT_TO_RIGHT_FULL_FK_DIAGNOSTIC_PASS"
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
DEFAULT_S1_RESULT = (
    ROOT
    / "tmp/"
    "towel_first_fold_surface_drag_half_comp15_measured_headless_20260903.json"
)
DEFAULT_S1_SUMMARY = (
    ROOT
    / "artifacts/bimanual/planning/"
    "towel_first_fold_surface_drag_r2_s1_summary.json"
)
DEFAULT_BASE_REPLAY = (
    ROOT
    / "tmp/"
    "towel_second_fold_horizontal_pinch_minus20mm_arc9_descent18_full_fk_20260903.json"
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
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--worktable", type=Path, default=DEFAULT_WORKTABLE)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--s1-result", type=Path, default=DEFAULT_S1_RESULT)
    parser.add_argument("--s1-summary", type=Path, default=DEFAULT_S1_SUMMARY)
    parser.add_argument(
        "--along-edge-offset-m",
        type=float,
        default=0.0,
        help="S2 grasp offset along the folded edge from its geometric midpoint",
    )
    parser.add_argument(
        "--reuse-second-fold-replay",
        type=Path,
        default=DEFAULT_BASE_REPLAY,
        help="hash-checked prior full-FK S2 replay; only the new stabilizer is solved",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    for path in (
        args.contract,
        args.worktable,
        args.urdf,
        args.s1_result,
        args.s1_summary,
        args.reuse_second_fold_replay,
    ):
        if not path.is_file():
            parser.error(f"required source does not exist: {path}")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")
    return args


def main() -> int:
    args = parse_args()
    footprint = load_accepted_first_fold_footprint(
        args.s1_result, args.s1_summary
    )
    contract = load_mapping(args.contract)
    worktable = load_mapping(args.worktable)
    observe = contract.get("workcell_observation_candidate", {})
    if not isinstance(observe, dict):
        raise RuntimeError("contract observation section is missing")
    clear_record = observe.get("observe_clear", {})
    if not isinstance(clear_record, dict):
        raise RuntimeError("contract clear pose is missing")
    clear = tuple(float(value) for value in clear_record.get("joint_positions_rad", ()))
    if len(clear) != 12 or not all(math.isfinite(value) for value in clear):
        raise RuntimeError("contract does not contain the canonical clear state")
    board = worktable.get("board", {})
    if not isinstance(board, dict):
        raise RuntimeError("worktable board section is missing")
    table_z = float(board.get("table_z_in_left_base_link_m", math.nan))
    if not math.isfinite(table_z):
        raise RuntimeError("worktable table z is invalid")

    phases, final_footprint = build_single_arm_second_fold(
        footprint.bounds_xyxy_m,
        table_z,
        active_arm=SECOND_FOLD_ACTIVE_ARM,
        direction=SECOND_FOLD_DIRECTION,
        along_edge_offset_m=args.along_edge_offset_m,
    )
    kinematics = {
        arm: GraspYawKinematics(args.urdf, prefix=f"{arm}_")
        for arm in ("left", "right")
    }
    bounds = {
        arm: planning.load_arm_joint_bounds(arm) for arm in ("left", "right")
    }
    print(
        f"SECOND_FOLD_FULL_FK_BEGIN phases={len(phases)} "
        f"footprint={footprint.bounds_xyxy_m}",
        flush=True,
    )
    base_replay = load_mapping(args.reuse_second_fold_replay)
    base_candidate = base_replay.get("selected_candidate", {})
    if not isinstance(base_candidate, dict):
        raise RuntimeError("reused S2 replay candidate is missing")
    if not math.isclose(
        float(base_candidate.get("along_edge_offset_from_midpoint_m", math.nan)),
        args.along_edge_offset_m,
        abs_tol=1.0e-12,
    ):
        raise RuntimeError("reused S2 replay has a different along-edge offset")
    if tuple(base_replay.get("first_fold_footprint", {}).get("bounds_xyxy_m", ())) != tuple(
        footprint.bounds_xyxy_m
    ):
        raise RuntimeError("reused S2 replay has a different accepted S1 footprint")
    base_sources = base_replay.get("sources", {})
    for name, path in (
        ("contract", args.contract),
        ("worktable", args.worktable),
        ("urdf", args.urdf),
        ("s1_result", args.s1_result),
        ("s1_summary", args.s1_summary),
    ):
        record = base_sources.get(name, {}) if isinstance(base_sources, dict) else {}
        if not isinstance(record, dict) or record.get("sha256") != file_sha256(path):
            raise RuntimeError(f"reused S2 replay source hash mismatch: {name}")
    records = base_candidate.get("second_fold", ())
    expected_names = [phase.name for phase in phases]
    if not isinstance(records, list) or [item.get("name") for item in records] != expected_names:
        raise RuntimeError("reused S2 replay phase sequence does not match the current plan")
    if not all(item.get("full_fk_pass") is True for item in records):
        raise RuntimeError("reused S2 replay contains a failed FK phase")
    stabilizer_phases = build_right_arm_second_fold_stabilizer(
        footprint.bounds_xyxy_m, table_z
    )
    stabilizer_records = solve_phases(
        stabilizer_phases,
        clear,
        kinematics,
        bounds,
        prefer_continuous_seed=True,
    )
    handoff_phases = build_right_arm_second_fold_edge_handoff(
        footprint.bounds_xyxy_m, table_z
    )
    handoff_records = solve_phases(
        handoff_phases,
        clear,
        kinematics,
        bounds,
        prefer_continuous_seed=True,
    )
    margins = [
        float(evaluation["minimum_joint_limit_margin_rad"])
        for record in (*records, *stabilizer_records, *handoff_records)
        for evaluation in record.get("task_pose_evaluations", [])
    ]
    if not margins:
        raise RuntimeError("second-fold diagnostic produced no FK evaluations")

    document = {
        "schema_version": 1,
        "record_kind": "towel_second_fold_full_fk_diagnostic",
        "status": STATUS,
        "created_unix_s": time.time(),
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "scope": "accepted_s1_terminal_shape_to_second_fold_full_fk_only",
        "first_fold_footprint": footprint.to_dict(),
        "table_z_m": table_z,
        "selected_candidate": {
            "active_arm": SECOND_FOLD_ACTIVE_ARM,
            "direction": SECOND_FOLD_DIRECTION,
            "grasp": "left_free_edge_two_layer_bundle",
            "along_edge_offset_from_midpoint_m": args.along_edge_offset_m,
            "inactive_arm_during_fold": "right_observe_clear",
            "right_arm_post_laydown_role": (
                "actual_contact_gated_bundle_stabilizer_during_left_release"
            ),
            "correction_arm_after_clear_observation": SECOND_FOLD_CORRECTION_ARM,
            "expected_final_footprint_xyxy_m": list(final_footprint),
            "second_fold": records,
            "second_fold_stabilizer": stabilizer_records,
            "second_fold_edge_handoff": handoff_records,
        },
        "minimum_joint_limit_margin_rad": min(margins),
        "post_release_observation_contract": {
            "top_camera_after_both_arms_clear": True,
            "plain_silhouette_is_sufficient": False,
            "moving_upper_bundle_free_edge_required": True,
            "minimum_edge_confidence": SECOND_FOLD_MINIMUM_EDGE_CONFIDENCE,
            "target_edge_alignment_tolerance_m": SECOND_FOLD_TARGET_TOLERANCE_M,
            "maximum_right_arm_correction_m": SECOND_FOLD_MAXIMUM_CORRECTION_M,
        },
        "sources": {
            name: {"path": str(path.resolve()), "sha256": file_sha256(path)}
            for name, path in (
                ("contract", args.contract),
                ("worktable", args.worktable),
                ("urdf", args.urdf),
                ("s1_result", args.s1_result),
                ("s1_summary", args.s1_summary),
                ("reused_second_fold_replay", args.reuse_second_fold_replay),
            )
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"{STATUS} phases={len(records)} "
        f"minimum_joint_margin_rad={min(margins):.6f} "
        f"motion_commands=0 output={args.output}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(1) from None
