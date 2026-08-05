"""
q0 복귀 계획 생성기 계약.

Motion-11 은 팔을 Pick pregrasp 에 남기고 `plan_buffered_q0_roundtrip.py` 는
소형 왕복용이라 그 변위를 계획하지 못한다. 이 생성기가 그 간극을 메운다.
"""

import importlib.util
import json
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
TOOLS_ROOT = ROOT / "tools"
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(TOOLS_ROOT))
MODULE_PATH = TOOLS_ROOT / "plan_buffered_q0_return.py"
SPEC = importlib.util.spec_from_file_location("plan_buffered_q0_return", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

CALIBRATION = PACKAGE_ROOT / "config" / "single_arm_calibration.json"
CONTRACT = PACKAGE_ROOT / "config" / "buffered_trajectory_contract.json"
# Motion-11 이 실제로 남긴 pregrasp 자세.
PREGRASP_ANCHOR = (2274, 3194, 1643, 1218, 2142, 2002)


def build(anchor=PREGRASP_ANCHOR, duration_ms=None):
    return MODULE.build_plan(CALIBRATION, CONTRACT, anchor, duration_ms)


def test_plan_is_non_executable_and_lands_exactly_on_q0():
    document = build()

    assert document["status"] == MODULE.STATUS
    assert document["phase"] == MODULE.PHASE
    assert document["execution_api_used"] is False
    assert document["buffered_frame_encoded"] is False
    assert document["motion_authorized"] is False
    assert document["robot_target_available"] is False
    assert document["target"]["raw"][:5] == [2048] * 5
    # gripper 는 anchor 값이 보존되어야 한다.
    assert document["target"]["raw"][5] == PREGRASP_ANCHOR[5]
    assert document["firmware_output_simulation"]["start_raw"] == list(
        PREGRASP_ANCHOR
    )
    assert document["firmware_output_simulation"]["final_raw"] == document[
        "target"
    ]["raw"]


def test_duration_search_leaves_tracking_margin():
    """최소 시간이 아니라 여유를 남기는 시간을 골라야 한다."""
    document = build()
    tracking = document["physical_tracking_model"]["legs"]["anchor_to_q0"]
    budget = (
        MODULE.MAXIMUM_MODELED_PEAK_ERROR_RAW * MODULE.TRACKING_MARGIN_FRACTION
    )
    assert tracking["maximum_peak_error_raw"] <= budget
    assert tracking["maximum_terminal_error_raw"] == 0.0
    assert document["analytic_profile"]["duration_selected_automatically"] is True

    # 한 단계 짧은 시간은 여유 기준을 넘어야 한다. 그래야 탐색이 최소값을 고른 것이다.
    shorter = (
        document["analytic_profile"]["duration_ms"]
        - MODULE.DURATION_SEARCH_STEP_MS
    )
    faster = MODULE.simulate_rate_limited_tracking(
        PREGRASP_ANCHOR, tuple(document["target"]["raw"]), shorter
    )
    assert faster["maximum_peak_error_raw"] > budget


def test_explicit_duration_is_honoured_and_gated():
    document = build(duration_ms=60_000)
    assert document["analytic_profile"]["duration_ms"] == 60_000
    assert document["analytic_profile"]["duration_selected_automatically"] is False
    # 더 긴 시간은 추종 오차가 더 작아야 한다.
    tracking = document["physical_tracking_model"]["legs"]["anchor_to_q0"]
    assert tracking["maximum_peak_error_raw"] < 67.0


def test_too_fast_duration_is_rejected_by_the_tracking_gate():
    """
    20초는 궤적 validator 의 속도/가속도 상한은 통과하지만 보수적 추종
    모델에서 peak 415 raw 가 되어 거부되어야 한다. 이것이 Motion-11 1차
    시도를 죽인 실패 모드이며, 계획 단계에서 걸러야 한다.
    """
    with pytest.raises(ValueError, match="peak tracking error"):
        build(duration_ms=20_000)


def test_grossly_fast_duration_is_rejected_earlier_by_dynamic_limits():
    """더 짧으면 추종 게이트에 닿기 전에 속도 상한에서 먼저 걸린다."""
    with pytest.raises(ValueError, match="velocity exceeds"):
        build(duration_ms=6_000)


def test_duration_must_be_whole_samples():
    with pytest.raises(ValueError, match="whole number of 20 ms samples"):
        build(duration_ms=36_010)


def test_sample_count_matches_the_resampling_period():
    document = build()
    duration_ms = document["analytic_profile"]["duration_ms"]
    expected = duration_ms // document["resampling"]["period_ms"] + 1
    assert document["resampling"]["sample_count"] == expected
    assert document["analytic_profile"]["waypoint_count"] == expected


def test_queue_simulation_has_no_underflow():
    terminal = build()["queue_contract"]["simulation_terminal"]
    assert terminal["safe_stop_required"] is False
    assert terminal["state"] == "input_complete"
    assert terminal["success_without_firmware_terminal"] is False


def test_firmware_output_steps_stay_bounded():
    simulation = build()["firmware_output_simulation"]
    assert simulation["maximum_arm_step_raw"] <= 1
    assert simulation["servo_sync_write_period_ms"] == 5
    assert simulation["executor_step_period_ms"] == 1


def test_generator_has_no_ros_serial_or_execution_dependency():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "rclpy" not in source
    assert "serial.Serial" not in source
    assert "ActionClient" not in source
    assert "send_goal" not in source


def test_plan_records_the_firmware_deployment_gate():
    gate = build()["firmware_deployment_gate"]
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    candidate = contract["servo_uart_receive_candidate"]
    assert gate["deployed"] == candidate["deployed"]
    assert gate["candidate_status"] == candidate["status"]
    assert gate["motion_authorized"] is False
