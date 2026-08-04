import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
sys.path.insert(0, str(PACKAGE_ROOT))
MODULE_PATH = ROOT / "tools" / "plan_buffered_q0_roundtrip.py"
SPEC = importlib.util.spec_from_file_location(
    "plan_buffered_q0_roundtrip",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


CALIBRATION = PACKAGE_ROOT / "config" / "single_arm_calibration.json"
CONTRACT = PACKAGE_ROOT / "config" / "buffered_trajectory_contract.json"
ANCHOR_RAW = (2068, 2227, 1728, 1831, 2052, 2002)
ARTIFACT = (
    ROOT
    / "artifacts"
    / "motion"
    / "2026-08-04"
    / "motion10_buffered_q0_roundtrip_plan_only.json"
)
ARTIFACT_SHA256 = (
    "032bf81ef3a18cc8126fb2955388b8d0363405d7584327d56f3bbcf534aa72b5"
)


def test_q0_roundtrip_is_non_executable_and_preserves_gripper():
    document = MODULE.build_plan(CALIBRATION, CONTRACT, ANCHOR_RAW)

    assert document["status"] == MODULE.STATUS
    assert document["phase"] == "motion10_q0_roundtrip"
    assert document["execution_api_used"] is False
    assert document["motion_authorized"] is False
    assert document["buffered_frame_encoded"] is False
    assert document["contract_status"] == "PHYSICAL_ACTION_COMMISSIONED"
    assert document["q0"] == {
        "arm_positions_rad": [0.0] * 5,
        "raw_with_preserved_gripper": [2048, 2048, 2048, 2048, 2048, 2002],
        "maximum_arm_error_raw": 0,
        "gripper_preserved": True,
    }
    assert len(document["waypoints"]) == 211
    assert document["waypoints"][105]["time_from_start_ms"] == 2100
    assert document["waypoints"][105]["q0_progress"] == 1.0
    assert document["waypoints"][105]["positions_rad"] == [0.0] * 5
    assert document["analytic_profile"] == {
        "kind": "symmetric_quintic_minimum_jerk",
        "polynomial": "10t^3-15t^4+6t^5",
        "half_trip_duration_ms": 2100,
        "waypoint_period_ms": 20,
        "waypoint_count": 211,
        "zero_velocity_boundaries_ms": [0, 2100, 4200],
        "zero_acceleration_boundaries_ms": [0, 2100, 4200],
    }
    assert document["round_trip"]["maximum_return_error_rad"] == 0.0


def test_q0_roundtrip_resampling_and_queue_admission_are_complete():
    document = MODULE.build_plan(CALIBRATION, CONTRACT, ANCHOR_RAW)

    assert document["resampling"]["period_ms"] == 20
    assert document["resampling"]["duration_ms"] == 4200
    assert document["resampling"]["sample_count"] == 211
    assert document["resampling"]["maximum_sample_step_rad"] < 0.0098
    batches = document["queue_contract"]["admission_batch_sizes"]
    assert batches[:2] == [9, 7]
    assert max(batches) <= 9
    assert sum(batches) == 211
    terminal = document["queue_contract"]["simulation_terminal"]
    assert terminal["state"] == "input_complete"
    assert terminal["accepted_samples"] == 211
    assert terminal["safe_stop_required"] is False
    assert terminal["success_without_firmware_terminal"] is False


def test_q0_roundtrip_stays_within_reviewed_dynamic_limits():
    document = MODULE.build_plan(CALIBRATION, CONTRACT, ANCHOR_RAW)

    dynamics = document["dynamic_limits"]["finite_difference"]
    assert dynamics["maximum_velocity_rad_s"] < 0.5
    assert dynamics["maximum_acceleration_rad_s2"] < 1.0
    assert dynamics["maximum_jerk_rad_s3"] < 3.6
    assert dynamics["start_segment_velocity_rad_s"] < 0.0003
    assert dynamics["q0_inbound_velocity_rad_s"] < 0.0003
    assert dynamics["q0_outbound_velocity_rad_s"] < 0.0003
    assert dynamics["final_segment_velocity_rad_s"] < 0.0003


def test_firmware_5ms_raw_output_is_bounded_and_returns_exactly():
    document = MODULE.build_plan(CALIBRATION, CONTRACT, ANCHOR_RAW)
    output = document["firmware_output_simulation"]

    assert output["executor_step_period_ms"] == 1
    assert output["servo_sync_write_period_ms"] == 5
    assert output["output_count"] == 841
    assert output["maximum_arm_step_raw"] <= 2
    assert output["start_raw"] == list(ANCHOR_RAW)
    assert output["q0_raw"] == [2048, 2048, 2048, 2048, 2048, 2002]
    assert output["final_raw"] == list(ANCHOR_RAW)
    assert output["per_axis"]["left_shoulder_joint"]["maximum_step_raw"] == 1
    assert output["per_axis"]["left_elbow_joint"]["maximum_step_raw"] == 2


def test_rejects_anchor_outside_calibration_before_plan_creation():
    with pytest.raises(Exception, match="outside safe range"):
        MODULE.build_plan(
            CALIBRATION,
            CONTRACT,
            (2047, 2255, 1785, 1981, 2070, 2002),
        )


def test_tool_has_no_ros_serial_or_execution_dependency():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "rclpy" not in source
    assert "serial_port" not in source
    assert "BufferedTransportDriver" not in source
    assert "ActionClient" not in source
    assert "send_goal" not in source


def test_reviewed_artifact_is_exactly_reproducible():
    assert MODULE.sha256_file(ARTIFACT) == ARTIFACT_SHA256
    recorded = json.loads(ARTIFACT.read_text(encoding="utf-8"))
    rebuilt = MODULE.build_plan(CALIBRATION, CONTRACT, ANCHOR_RAW)
    assert recorded == rebuilt
