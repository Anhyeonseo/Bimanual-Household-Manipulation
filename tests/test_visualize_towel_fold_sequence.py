from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools/run/visualize_towel_fold_sequence.py"
SPEC = importlib.util.spec_from_file_location(
    "visualize_towel_fold_sequence", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def target(name: str, arm: str, xyz: list[float]) -> dict:
    return {
        "name": name,
        "arm": arm,
        "xyz_m": xyz,
        "jaw_yaw_rad": 0.0,
        "semantic": "attached_transfer",
        "layer": "one_layer",
        "maximum_approach_tilt_rad": 1.2,
    }


def phase(
    name: str,
    targets: list[dict],
    positions: list[float],
) -> dict:
    return {
        "name": name,
        "targets": targets,
        "clear_pose": False,
        "clear_arm": None,
        "reuse_target_of": None,
        "reverse_of": None,
        "path_cache_key": None,
        "attachment_event": None,
        "full_fk_pass": True,
        "joint_positions_rad": positions,
        "task_pose_evaluations": [],
        "moveit_segment_planned": False,
        "transition_collision_checked": False,
    }


def diagnostic_document() -> dict:
    return {
        "record_kind": "canonical_towel_fold_full_fk_diagnostic",
        "motion_authorized": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "scope": {"transition_collision_checked": False},
        "towel_placement": {
            "bounds_xyxy_m": [0.18, 0.48, -0.27, 0.03],
            "table_z_m": -0.005,
        },
        "clear_joint_positions_rad": [0.0] * 12,
        "selected_candidate": {
            "candidate_id": "canonical",
            "second_active_arm": "right",
            "second_direction": "right_to_left",
            "first_expected_footprint_xyxy_m": [0.33, 0.48, -0.27, 0.03],
            "final_expected_footprint_xyxy_m": [0.33, 0.48, -0.12, 0.03],
            "first_fold": [
                phase(
                    "first_contact",
                    [
                        target("first_left", "left", [0.22, 0.01, 0.01]),
                        target("first_right", "right", [0.22, -0.25, 0.01]),
                    ],
                    [0.1] * 12,
                ),
                phase(
                    "first_fold_01",
                    [
                        target("first_left", "left", [0.40, 0.01, 0.12]),
                        target("first_right", "right", [0.40, -0.25, 0.12]),
                    ],
                    [0.2] * 12,
                ),
            ],
            "second_fold": [
                phase(
                    "second_contact",
                    [target("second_right", "right", [0.405, -0.24, 0.011])],
                    [0.3] * 12,
                ),
                phase(
                    "second_fold_01",
                    [target("second_right", "right", [0.405, -0.10, 0.12])],
                    [0.4] * 12,
                ),
            ],
        },
    }


def test_both_stage_markers_show_dual_first_and_single_right_second():
    markers = MODULE.marker_array(diagnostic_document(), stage="both")
    namespaces = {marker.ns for marker in markers.markers}
    assert "first_left_path" in namespaces
    assert "first_right_path" in namespaces
    assert "second_right_path" in namespaces
    assert "second_left_path" not in namespaces
    assert "initial_towel" in namespaces
    assert "after_first_fold" in namespaces
    assert "final_footprint" in namespaces
    title = next(
        marker for marker in markers.markers
        if marker.ns == "canonical_fold_title"
    )
    assert "양팔 아래→위" in title.text
    assert "오른팔 오른쪽→왼쪽" in title.text
    assert "COLLISION UNCHECKED" in title.text


def test_stage_filter_separates_first_and_second_paths():
    first = MODULE.marker_array(diagnostic_document(), stage="first")
    second = MODULE.marker_array(diagnostic_document(), stage="second")
    first_names = {marker.ns for marker in first.markers}
    second_names = {marker.ns for marker in second.markers}
    assert "first_left_path" in first_names
    assert "second_right_path" not in first_names
    assert "second_right_path" in second_names
    assert "first_left_path" not in second_names


def test_diagnostic_animation_is_marked_not_collision_certified():
    message, certified = MODULE.display_trajectory(
        diagnostic_document(), "both", 0.5
    )
    assert certified is False
    assert message is not None
    trajectory = message.trajectory[0].joint_trajectory
    assert len(trajectory.points) == 5
    assert list(trajectory.points[-1].positions) == [0.4] * 12


def test_strict_moveit_segments_are_reconstructed_as_full_states():
    document = diagnostic_document()
    document["record_kind"] = "towel_bimanual_then_single_task_pose_plan_only"
    first = document["selected_candidate"]["first_fold"][0]
    first["moveit"] = {
        "start_positions_rad": [0.0] * 12,
        "trajectory_joint_names": [
            "right_base_joint",
            "right_shoulder_joint",
            "right_elbow_joint",
            "right_wrist_flex_joint",
            "right_wrist_roll_joint",
        ],
        "trajectory_positions_rad": [[0.1] * 5, [0.2] * 5],
    }
    document["selected_candidate"]["first_fold"] = [first]
    message, certified = MODULE.display_trajectory(document, "first", 0.5)
    assert certified is True
    assert message is not None
    positions = [
        list(point.positions)
        for point in message.trajectory[0].joint_trajectory.points
    ]
    assert positions[-1][6:11] == [0.2] * 5
    assert positions[-1][:5] == [0.0] * 5


def test_loader_rejects_motion_capable_artifact(tmp_path):
    path = tmp_path / "unsafe.json"
    document = diagnostic_document()
    document["motion_commands"] = 1
    path.write_text(json.dumps(document), encoding="utf-8")
    try:
        MODULE.load_artifact(path)
    except RuntimeError as exc:
        assert "motion-locked" in str(exc)
    else:
        raise AssertionError("motion-capable artifact was accepted")
