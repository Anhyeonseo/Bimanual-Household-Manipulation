#!/usr/bin/env python3
"""Solve the accepted-S1 to bimanual left-to-right S2 path with full FK.

This diagnostic is plan-only.  It verifies that the registered URDF contains
the real wrist-camera mount collision mesh on both grippers, then solves every
two-arm task-pose sample.  MoveIt/FCL transition collision approval remains a
separate, stricter gate.
"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib import desk_task_planning as planning  # noqa: E402
from tools.lib.grasp_yaw_kinematics import GraspYawKinematics  # noqa: E402
from tools.lib.towel_bimanual_then_single_planning import (  # noqa: E402
    SECOND_BIMANUAL_LEFT_ARM_EDGE_INSET_M,
    SECOND_BIMANUAL_MAXIMUM_FINGER_TILT_RAD,
    SECOND_BIMANUAL_RIGHT_ARM_EDGE_INSET_M,
    build_bimanual_second_fold,
)
from tools.lib.towel_second_fold_correction import (  # noqa: E402
    SECOND_FOLD_DIRECTION,
    load_accepted_first_fold_footprint,
)
from tools.run.diagnose_towel_fold_kinematics import solve_phases  # noqa: E402


STATUS = "TOWEL_SECOND_FOLD_BIMANUAL_FULL_FK_DIAGNOSTIC_PASS"
CAMERA_MOUNT_MESH = "wrist_cam_mount_32x32_uvc_module_so101_m.stl"
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


def file_sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def load_mapping(path: Path) -> dict[str, object]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError(f"document root must be a mapping: {path}")
    return value


def validate_wrist_camera_mount_collisions(urdf: Path) -> dict[str, str]:
    root = ET.parse(urdf).getroot()
    result: dict[str, str] = {}
    for arm in ("left", "right"):
        link = next(
            (
                item
                for item in root.findall("link")
                if item.attrib.get("name") == f"{arm}_gripper_link"
            ),
            None,
        )
        if link is None:
            raise RuntimeError(f"registered URDF is missing {arm} gripper link")
        meshes = [
            mesh.attrib.get("filename", "")
            for collision in link.findall("collision")
            for mesh in collision.findall("./geometry/mesh")
        ]
        match = next(
            (name for name in meshes if name.endswith(CAMERA_MOUNT_MESH)), None
        )
        if match is None:
            raise RuntimeError(
                f"registered URDF is missing {arm} wrist-camera mount collision"
            )
        result[arm] = match
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--worktable", type=Path, default=DEFAULT_WORKTABLE)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--s1-result", type=Path, default=DEFAULT_S1_RESULT)
    parser.add_argument("--s1-summary", type=Path, default=DEFAULT_S1_SUMMARY)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    for path in (
        args.contract,
        args.worktable,
        args.urdf,
        args.s1_result,
        args.s1_summary,
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
    clear_record = contract.get("workcell_observation_candidate", {}).get(
        "observe_clear", {}
    )
    clear = tuple(
        float(value) for value in clear_record.get("joint_positions_rad", ())
    )
    if len(clear) != 12 or not all(math.isfinite(value) for value in clear):
        raise RuntimeError("contract does not contain the canonical clear state")
    table_z = float(worktable.get("board", {}).get("table_z_in_left_base_link_m"))
    if not math.isfinite(table_z):
        raise RuntimeError("worktable table z is invalid")

    mount_collisions = validate_wrist_camera_mount_collisions(args.urdf)
    phases, final_footprint = build_bimanual_second_fold(
        footprint.bounds_xyxy_m,
        table_z,
        direction=SECOND_FOLD_DIRECTION,
    )
    kinematics = {
        arm: GraspYawKinematics(args.urdf, prefix=f"{arm}_")
        for arm in ("left", "right")
    }
    bounds = {
        arm: planning.load_arm_joint_bounds(arm) for arm in ("left", "right")
    }
    print(
        f"SECOND_FOLD_BIMANUAL_FULL_FK_BEGIN phases={len(phases)} "
        f"footprint={footprint.bounds_xyxy_m}",
        flush=True,
    )
    records = solve_phases(
        phases,
        clear,
        kinematics,
        bounds,
        prefer_continuous_seed=True,
    )
    evaluations = [
        evaluation
        for record in records
        for evaluation in record.get("task_pose_evaluations", [])
    ]
    if not evaluations:
        raise RuntimeError("bimanual S2 diagnostic produced no FK evaluations")

    document = {
        "schema_version": 1,
        "record_kind": "towel_second_fold_bimanual_full_fk_diagnostic",
        "status": STATUS,
        "created_unix_s": time.time(),
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "scope": {
            "accepted_s1_terminal_shape_to_bimanual_second_fold": True,
            "full_fk_task_pose_checked": True,
            "moveit_segment_planning_checked": False,
            "transition_collision_checked": False,
            "cloth_attachment_checked": False,
            "physical_fold_success_checked": False,
        },
        "first_fold_footprint": footprint.to_dict(),
        "table_z_m": table_z,
        "selected_candidate": {
            "active_arms": ["left", "right"],
            "direction": SECOND_FOLD_DIRECTION,
            "grasp": "two_separated_four_layer_top_down_u_pinches",
            "left_arm_near_low_x_inset_m": (
                SECOND_BIMANUAL_LEFT_ARM_EDGE_INSET_M
            ),
            "left_arm_jaw_yaw_rad": math.pi / 2.0,
            "right_arm_far_high_x_inset_m": (
                SECOND_BIMANUAL_RIGHT_ARM_EDGE_INSET_M
            ),
            "right_arm_jaw_yaw_rad": 0.0,
            "maximum_finger_tilt_rad": (
                SECOND_BIMANUAL_MAXIMUM_FINGER_TILT_RAD
            ),
            "expected_final_footprint_xyxy_m": list(final_footprint),
            "second_fold": records,
        },
        "minimum_joint_limit_margin_rad": min(
            float(item["minimum_joint_limit_margin_rad"])
            for item in evaluations
        ),
        "maximum_actual_finger_tilt_rad": max(
            math.radians(float(item["finger_tilt_from_table_deg"]))
            for item in evaluations
        ),
        "wrist_camera_mount_collision": {
            "exact_registered_mesh_required": True,
            "mesh_by_arm": mount_collisions,
            "moveit_fcl_collision_check_pending": True,
        },
        "sources": {
            name: {"path": str(path.resolve()), "sha256": file_sha256(path)}
            for name, path in (
                ("contract", args.contract),
                ("worktable", args.worktable),
                ("urdf", args.urdf),
                ("s1_result", args.s1_result),
                ("s1_summary", args.s1_summary),
            )
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"{STATUS} minimum_margin_rad="
        f"{document['minimum_joint_limit_margin_rad']:.6f}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
