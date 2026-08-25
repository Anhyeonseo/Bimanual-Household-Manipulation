import importlib.util
import json
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools/run"
SPEC = importlib.util.spec_from_file_location(
    "run_observe_clear_roundtrip_once",
    TOOLS / "run_observe_clear_roundtrip_once.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def plan_document(now=1000.0):
    names = list(MODULE.BOTH_ARM_JOINTS)
    return {
        "schema_version": 1,
        "record_kind": "observe_clear_plan_only",
        "status": MODULE.EXPECTED_PLAN_STATUS,
        "generated_at_unix_s": now,
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "execution_api_used": False,
        "motion_commands": 0,
        "planning_group": "both_arms",
        "planning_scene": {
            "collision_object_id": "validated_worktable_region",
            "apply_planning_scene_success": True,
            "temporary_exceptions_restored_before_strict_validation": True,
            "strict_unapproved_contact_count": 0,
            "strict_path_sample_count": 10,
            "strict_maximum_exception_depth_m": 0.003,
            "strict_maximum_exception_depth_limit_m": 0.004,
        },
        "joint_names": list(MODULE.CANONICAL_JOINTS),
        "arm_joint_names": names,
        "start_positions_rad": [0.0] * 12,
        "target_positions_rad": [0.1] * 12,
        "trajectory": [
            {"positions_rad": [0.05] * 10, "time_from_start_s": 1.0},
            {"positions_rad": [0.1] * 10, "time_from_start_s": 2.0},
        ],
    }


def test_plan_is_sha_pinned_fresh_and_still_nonautomatic(tmp_path):
    path = tmp_path / "plan.json"
    path.write_text(json.dumps(plan_document()))
    digest = MODULE.sha256_file(path)
    loaded = MODULE.load_plan(path, digest, now=1001.0)
    assert loaded["status"] == MODULE.EXPECTED_PLAN_STATUS
    loaded = plan_document()
    loaded["automatic_execution_permitted"] = True
    path.write_text(json.dumps(loaded))
    with pytest.raises(MODULE.ObserveClearExecutionError, match="contract"):
        MODULE.load_plan(path, MODULE.sha256_file(path), now=1001.0)


def test_arm_route_preserves_live_grippers_and_plan_waypoints():
    route = MODULE.arm_route_as_full_positions(plan_document(), (0.2, 0.3))
    assert len(route) == 2
    assert route[0][5] == pytest.approx(0.2)
    assert route[0][11] == pytest.approx(0.3)
    assert all(
        route[-1][index] == pytest.approx(0.1)
        for index, name in enumerate(MODULE.CANONICAL_JOINTS)
        if name in MODULE.BOTH_ARM_JOINTS
    )


def test_resident_retiming_has_two_points_and_conservative_steps():
    start = (0.0,) * 12
    route = ((0.03,) * 12, (0.06,) * 12)
    request = MODULE.finite_route_request(start, route)
    assert len(request.points) >= 2
    previous = start
    previous_time = 0.0
    for point in request.points:
        current_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
        dt = current_time - previous_time
        maximum = max(abs(b - a) for a, b in zip(previous, point.positions))
        assert maximum <= MODULE.COMMAND_RATE_RAD_S * dt + 1e-12
        previous = tuple(point.positions)
        previous_time = current_time


def test_roundtrip_always_stops_and_leaves_visual_review_explicit():
    source = (TOOLS / "run_observe_clear_roundtrip_once.py").read_text()
    assert "OBSERVE_CLEAR_ROUNDTRIP_CAPTURED_AWAITING_VISUAL_REVIEW" in source
    assert '"visual_towel_occlusion_reviewed": False' in source
    assert "if motion_started and not stopped" in source
    assert "stop_request()" in source


def test_pi_executor_has_no_moveit_runtime_dependency():
    source = (TOOLS / "run_observe_clear_roundtrip_once.py").read_text()
    assert "desk_task_planning" not in source
    assert "desk_task_runtime" not in source
    assert "moveit_msgs" not in source
    assert len(MODULE.BOTH_ARM_JOINTS) == 10


def test_top_image_subscription_matches_camera_sensor_data_qos():
    source = (TOOLS / "run_observe_clear_roundtrip_once.py").read_text()
    assert "qos_profile_sensor_data" in source
    assert "Image,\n        TOP_IMAGE_TOPIC,\n        images.append," in source
