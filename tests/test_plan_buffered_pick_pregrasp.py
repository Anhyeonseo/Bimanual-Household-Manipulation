import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
TOOLS_ROOT = ROOT / "tools"
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(TOOLS_ROOT))
MODULE_PATH = TOOLS_ROOT / "plan_buffered_pick_pregrasp.py"
SPEC = importlib.util.spec_from_file_location(
    "plan_buffered_pick_pregrasp",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


CALIBRATION = PACKAGE_ROOT / "config" / "single_arm_calibration.json"
CONTRACT = PACKAGE_ROOT / "config" / "buffered_trajectory_contract.json"
SOURCE_ROUTE = (
    ROOT
    / "artifacts"
    / "stage7"
    / "2026-07-31"
    / "full_pick_place_reindexed_headroom015"
    / "01_q0_to_pick_pregrasp.json"
)
ANCHOR_RAW = (2273, 2330, 1802, 1941, 2142, 2002)
ARTIFACT = (
    ROOT
    / "artifacts"
    / "motion"
    / "2026-08-04"
    / "motion11_buffered_pick_pregrasp_plan_only.json"
)
ARTIFACT_SHA256 = (
    "874f2a50fc8c4bd68d97d57a358481234880bcd6d7c3c80145fc91f1656c5e46"
)


def build():
    return MODULE.build_plan(CALIBRATION, CONTRACT, SOURCE_ROUTE, ANCHOR_RAW)


def test_plan_is_non_executable_and_pins_collision_checked_source():
    document = build()

    assert document["status"] == MODULE.STATUS
    assert document["phase"] == MODULE.PHASE
    assert document["execution_api_used"] is False
    assert document["motion_authorized"] is False
    assert document["buffered_frame_encoded"] is False
    assert document["source_route"] == {
        "path": SOURCE_ROUTE.relative_to(ROOT).as_posix(),
        "sha256": MODULE.EXPECTED_SOURCE_ROUTE_SHA256,
        "segment_count": 12,
        "collision_checked": True,
        "targets_exactly_collinear_from_q0": True,
    }


def test_two_leg_profile_reaches_q0_without_settle_then_pregrasp():
    document = build()

    assert document["analytic_profile"] == {
        "kind": "two_leg_quintic_minimum_jerk",
        "polynomial": "10t^3-15t^4+6t^5",
        "anchor_to_q0_duration_ms": 8000,
        "q0_to_pregrasp_duration_ms": 35000,
        "total_duration_ms": 43000,
        "waypoint_period_ms": 20,
        "waypoint_count": 2151,
        "q0_settle_wait_ms": 0,
    }
    assert document["q0_transition"]["raw_with_preserved_gripper"] == [
        2048, 2048, 2048, 2048, 2048, 2002
    ]
    assert document["target"]["raw"] == [
        2278, 3190, 1625, 1209, 2146, 2002
    ]
    assert document["resampling"]["sample_count"] == 2151


def test_dynamic_and_firmware_output_limits_are_bounded():
    document = build()
    dynamics = document["dynamic_limits"]["finite_difference"]
    output = document["firmware_output_simulation"]

    assert dynamics["maximum_velocity_rad_s"] < 0.5
    assert dynamics["maximum_acceleration_rad_s2"] < 1.0
    assert dynamics["maximum_jerk_rad_s3"] < 3.6
    assert dynamics["q0_inbound_velocity_rad_s"] < 0.0003
    assert dynamics["q0_outbound_velocity_rad_s"] < 0.0003
    assert document["resampling"]["maximum_sample_step_rad"] < 0.01
    assert output["executor_step_period_ms"] == 1
    assert output["servo_sync_write_period_ms"] == 5
    assert output["output_count"] == 8601
    assert output["maximum_arm_step_raw"] <= 2
    assert output["start_raw"] == list(ANCHOR_RAW)
    assert output["q0_raw"] == [2048, 2048, 2048, 2048, 2048, 2002]
    assert output["final_raw"] == document["target"]["raw"]


def test_queue_admission_accepts_every_sample_without_false_success():
    document = build()
    queue = document["queue_contract"]

    assert max(queue["admission_batch_sizes"]) <= 9
    assert sum(queue["admission_batch_sizes"]) == 2151
    terminal = queue["simulation_terminal"]
    assert terminal["state"] == "input_complete"
    assert terminal["accepted_samples"] == 2151
    assert terminal["safe_stop_required"] is False
    assert terminal["success_without_firmware_terminal"] is False


def test_measured_tracking_rate_model_bounds_peak_and_terminal_error():
    tracking = build()["physical_tracking_model"]

    assert tracking["conservative_rate_raw_s"] == 50.0
    assert tracking["measured_rate_evidence_raw_s"] == {
        "left_shoulder_joint": 60.0,
        "left_wrist_flex_joint": 60.8,
    }
    assert tracking["maximum_modeled_peak_error_raw"] <= 100.0
    assert tracking["maximum_modeled_terminal_error_raw"] <= 30.0
    assert set(tracking["legs"]) == {
        "anchor_to_q0",
        "q0_to_pregrasp",
    }


def test_source_route_tamper_is_rejected_before_plan_creation(tmp_path):
    document = json.loads(SOURCE_ROUTE.read_text(encoding="utf-8"))
    document["segments"][0]["success"] = False
    path = tmp_path / "tampered.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="source Pick pregrasp route sha256"):
        MODULE.build_plan(CALIBRATION, CONTRACT, path, ANCHOR_RAW)


def test_artifact_is_exactly_reproducible():
    assert MODULE.sha256_file(ARTIFACT) == ARTIFACT_SHA256
    assert json.loads(ARTIFACT.read_text(encoding="utf-8")) == build()


def test_generator_has_no_ros_serial_or_execution_dependency():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "rclpy" not in source
    assert "serial.Serial" not in source
    assert "ActionClient" not in source
    assert "send_goal" not in source
