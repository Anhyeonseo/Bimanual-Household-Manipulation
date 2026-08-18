"""캔 실행기의 계획 수락 계약.

실행기가 막아야 하는 것은 **하드웨어에서만 드러나는 계획**이다. 조가 안 열린
계획, 개방 부호가 뒤집힌 계획, roll 을 안 푼 계획, 하강이 연직이 아닌 계획은
전부 캔을 쓰러뜨리거나 조를 캔에 박는다. 여기서 걸어야 한다.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

pytest.importorskip("rclpy")
pytest.importorskip("so101_interfaces.srv")
pytest.importorskip("top_pick_place_application")

import run_can_pick_left_once as executor  # noqa: E402

OPEN_RAD = -0.9235
GRASP_RAD = -0.5000


def _plan() -> dict:
    def arm(phase: str, index: int) -> dict:
        return {
            "kind": "arm",
            "phase": phase,
            "index": index,
            "target_positions_rad": [0.0, 2.6, 1.2, 1.3, GRASP_RAD],
            "maximum_joint_delta_rad": 0.1,
        }

    return {
        "schema_version": 1,
        "record_kind": "can_pick_left_plan_only",
        "status": "CAN_PICK_LEFT_PLAN_ONLY_PASS",
        "generated_at_unix_s": time.time(),
        "execution_api_used": False,
        "motion_authorized": False,
        "automatic_execution_permitted": False,
        "selected_arm": "left",
        "joint_names": list(executor.ARM_JOINTS_BY_SIDE["left"]),
        "target_lock": {"can_axis_yaw_rad": 0.3},
        "gripper_contract": {
            "open_gap_mm": 70.0,
            "open_command_rad": OPEN_RAD,
            "grasp_gap_mm": 44.0,
            "grasp_command_rad": GRASP_RAD,
            "contact_threshold_raw": 20,
            "release_tolerance_raw": 30,
            "provenance": "artifacts/can_to_bin/2026-08-17/jaw_gap_left_run01.json",
            "minimum_open_gap_for_tolerance_mm": 66.5,
            "required_jaw_width_at_achieved_error_mm": 55.0,
        },
        "acceptance_limits": {
            "crossing_tolerance_rad": math.radians(6.0),
            "position_tolerance_m": 0.0021,
            "maximum_approach_tilt_rad": None,
        },
        "descent_check": {
            "vertical_only": True,
            "wrist_roll_span_rad": 0.0,
            "lateral_travel_m": 0.0001,
        },
        "endpoints": {
            "pick_pregrasp": {
                "final_joint_positions_rad": [0.0, 2.6, 1.2, 1.3, GRASP_RAD]
            },
            "pick_grasp": {
                "wrist_roll_rad": GRASP_RAD,
                "wrist_roll_rotation_from_q0_rad": GRASP_RAD,
                "wrist_roll_branch_index": 0,
                "wrist_roll_branch_count": 2,
                "crossing_error_rad": math.radians(1.2),
                "approach_tilt_from_vertical_deg": 44.2,
                "wrist_roll_policy": (
                    "nearest_in_limit_branch_then_joint_position_crossing_solve"
                ),
                "final_joint_positions_rad": [0.0, 2.7, 1.3, 1.2, GRASP_RAD],
            },
            "pick_lift": {
                "final_joint_positions_rad": [0.0, 2.6, 1.2, 1.3, GRASP_RAD]
            },
        },
        "steps": [
            {
                "kind": "gripper",
                "phase": "pick_open",
                "index": 1,
                "target_position_rad": OPEN_RAD,
            },
            arm("q0_to_pick_pregrasp", 2),
            arm("pick_pregrasp_to_grasp", 3),
            {
                "kind": "gripper",
                "phase": "pick_close",
                "index": 4,
                "target_position_rad": GRASP_RAD,
            },
            arm("pick_grasp_to_lift", 5),
            arm("pick_lift_to_q0", 6),
        ],
    }


def _write(tmp_path: Path, plan: dict) -> tuple[Path, str]:
    path = tmp_path / "plan.json"
    path.write_text(json.dumps(plan, indent=2, sort_keys=True) + "\n", "utf-8")
    return path, executor.sha256_file(path)


def test_a_well_formed_plan_is_accepted(tmp_path):
    path, digest = _write(tmp_path, _plan())
    loaded = executor.load_can_plan(path, digest)
    assert loaded["selected_arm"] == "left"


def test_a_tampered_plan_is_rejected(tmp_path):
    path, digest = _write(tmp_path, _plan())
    with pytest.raises(executor.CanExecutionError, match="sha256 mismatch"):
        executor.load_can_plan(path, "0" * 64)
    assert len(digest) == 64


def test_a_stale_plan_is_rejected(tmp_path):
    plan = _plan()
    plan["generated_at_unix_s"] = time.time() - 400.0
    path, digest = _write(tmp_path, plan)
    with pytest.raises(executor.CanExecutionError, match="plan age"):
        executor.load_can_plan(path, digest)


@pytest.mark.parametrize(
    "field",
    [
        "open_gap_mm",
        "open_command_rad",
        "grasp_command_rad",
        "contact_threshold_raw",
        "release_tolerance_raw",
    ],
)
def test_an_uncommissioned_gripper_field_is_rejected(tmp_path, field):
    plan = _plan()
    plan["gripper_contract"][field] = None
    path, digest = _write(tmp_path, plan)
    with pytest.raises(executor.CanExecutionError, match="not commissioned"):
        executor.load_can_plan(path, digest)


def test_a_plan_without_measurement_provenance_is_rejected(tmp_path):
    plan = _plan()
    plan["gripper_contract"]["provenance"] = "not_measured"
    path, digest = _write(tmp_path, plan)
    with pytest.raises(executor.CanExecutionError, match="provenance"):
        executor.load_can_plan(path, digest)


def test_an_inverted_opening_direction_is_rejected(tmp_path):
    """개방이 파지보다 덜 열린 계획은 조를 닫으면서 캔으로 내려간다."""
    plan = _plan()
    plan["gripper_contract"]["open_command_rad"] = 0.0
    path, digest = _write(tmp_path, plan)
    with pytest.raises(executor.CanExecutionError, match="inverted"):
        executor.load_can_plan(path, digest)


def test_an_opening_too_narrow_for_the_tolerance_is_rejected(tmp_path):
    plan = _plan()
    plan["gripper_contract"]["open_gap_mm"] = 60.0
    path, digest = _write(tmp_path, plan)
    with pytest.raises(executor.CanExecutionError, match="crossing tolerance"):
        executor.load_can_plan(path, digest)


def test_a_crossing_error_over_tolerance_is_rejected(tmp_path):
    plan = _plan()
    plan["endpoints"]["pick_grasp"]["crossing_error_rad"] = math.radians(20.0)
    path, digest = _write(tmp_path, plan)
    with pytest.raises(executor.CanExecutionError, match="crossing error"):
        executor.load_can_plan(path, digest)


def test_the_pen_wrist_policy_is_rejected(tmp_path):
    """펜은 roll 을 q0 에 고정한다. 그 계획이 캔 실행기로 들어오면 안 된다."""
    plan = _plan()
    plan["endpoints"]["pick_grasp"]["wrist_roll_policy"] = "hold_bimanual_q0"
    path, digest = _write(tmp_path, plan)
    with pytest.raises(executor.CanExecutionError, match="nearest-branch"):
        executor.load_can_plan(path, digest)


def test_a_non_vertical_descent_is_rejected(tmp_path):
    plan = _plan()
    plan["descent_check"]["lateral_travel_m"] = 0.01
    path, digest = _write(tmp_path, plan)
    with pytest.raises(executor.CanExecutionError, match="not vertical"):
        executor.load_can_plan(path, digest)


def test_a_roll_that_moves_during_descent_is_rejected(tmp_path):
    plan = _plan()
    plan["descent_check"]["wrist_roll_span_rad"] = 0.2
    path, digest = _write(tmp_path, plan)
    with pytest.raises(executor.CanExecutionError, match="not vertical"):
        executor.load_can_plan(path, digest)


def test_a_plan_whose_first_step_is_not_the_opening_is_rejected(tmp_path):
    plan = _plan()
    plan["steps"] = plan["steps"][1:] + [plan["steps"][0]]
    path, digest = _write(tmp_path, plan)
    with pytest.raises(executor.CanExecutionError, match="one open before"):
        executor.load_can_plan(path, digest)


# --- leg 분할 ---


def test_actions_partition_into_open_descend_close_lift():
    actions = executor.can_actions(_plan(), height_check=False)
    assert [a["kind"] for a in actions] == [
        "gripper",
        "arm_route",
        "gripper",
        "arm_route",
    ]
    assert actions[0]["label"] == "pick_open"
    assert actions[2]["label"] == "pick_close"


def test_height_check_never_emits_the_close_action():
    """캔에 닿기 전 파지 자세만 확인하는 모드다. 닫기가 나가면 안 된다."""
    actions = executor.can_actions(_plan(), height_check=True)
    assert len(actions) == 2
    assert all(a["label"] != "pick_close" for a in actions)
    assert actions[0]["label"] == "pick_open"
    assert actions[1]["kind"] == "arm_route"


def test_validate_only_document_reports_no_resident_contact(tmp_path):
    path, digest = _write(tmp_path, _plan())
    plan = executor.load_can_plan(path, digest)
    document = executor.validate_only_document(plan, path, digest)
    assert document["motion_commands"] == 0
    assert document["resident_services_called"] == 0
    assert document["resident_clients_created"] == 0
    assert document["execution_api_used"] is False
    assert document["status"] == "CAN_PICK_LEFT_VALIDATE_ONLY_PASS"


def test_executor_does_not_inherit_the_pen_contact_threshold():
    """펜의 접촉 임계 14 raw 는 15 mm 펜의 값이다. 캔은 계획값을 쓴다."""
    import top_pick_place_application as shared

    assert shared.CONTACT_THRESHOLD_RAW == 14
    source = Path(executor.__file__).read_text(encoding="utf-8")
    assert "CONTACT_THRESHOLD_RAW" not in source.split('"""', 2)[2].split(
        "def parse_args"
    )[0]
