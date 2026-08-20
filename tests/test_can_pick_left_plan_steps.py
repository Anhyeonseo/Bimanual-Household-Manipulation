"""캔 계획기의 단계 조립과 gripper 계약."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

pytest.importorskip("rclpy")
pytest.importorskip("moveit_msgs.srv")
pytest.importorskip("so101_interfaces.msg")

from tools.run import plan_can_pick_left_once as planner  # noqa: E402
from tools.lib.can_pick_application import (  # noqa: E402
    CanJawContract,
    CanPickContractError,
    CanPickPolicy,
)

OPEN_RAD = -0.9235
GRASP_RAD = -0.5000


def _policy() -> CanPickPolicy:
    return CanPickPolicy(
        crossing_tolerance_rad=math.radians(6.0),
        position_tolerance_m=0.0021,
        maximum_approach_tilt_rad=None,
        jaw=CanJawContract(
            open_gap_mm=70.0,
            grasp_gap_mm=44.0,
            open_command_rad=OPEN_RAD,
            grasp_command_rad=GRASP_RAD,
            contact_threshold_raw=20,
            release_tolerance_raw=30,
            can_diameter_mm=53.0,
            provenance="test_fixture_not_measured",
        ),
    )


def _phases() -> list[dict]:
    def phase(name: str, count: int = 1) -> dict:
        return {
            "name": name,
            "segments": [
                {
                    "target_positions_rad": [0.0] * 5,
                    "maximum_joint_delta_rad": 0.1,
                }
                for _ in range(count)
            ],
        }

    return [
        phase("q0_to_pick_pregrasp", 2),
        phase("pick_pregrasp_to_grasp"),
        phase("pick_grasp_to_lift"),
        phase("pick_lift_to_q0", 2),
    ]


def test_open_precedes_every_arm_step():
    steps = planner.can_steps_from_phases(_phases(), _policy())
    assert steps[0]["kind"] == "gripper"
    assert steps[0]["phase"] == "pick_open"


def test_close_happens_after_arriving_at_grasp_and_before_lifting():
    steps = planner.can_steps_from_phases(_phases(), _policy())
    close = next(i for i, s in enumerate(steps) if s.get("phase") == "pick_close")
    arrivals = [
        i
        for i, s in enumerate(steps)
        if s.get("phase") == "pick_pregrasp_to_grasp"
    ]
    lifts = [
        i for i, s in enumerate(steps) if s.get("phase") == "pick_grasp_to_lift"
    ]
    assert max(arrivals) < close < min(lifts)


def test_gripper_targets_come_from_the_can_contract():
    steps = planner.can_steps_from_phases(_phases(), _policy())
    gripper = [s for s in steps if s["kind"] == "gripper"]
    assert len(gripper) == 2
    assert gripper[0]["target_position_rad"] == pytest.approx(OPEN_RAD)
    assert gripper[1]["target_position_rad"] == pytest.approx(GRASP_RAD)
    assert gripper[0]["target_position_rad"] != pytest.approx(0.0)


def test_opening_uses_the_negative_semantic_direction():
    """개방은 raw 가 커지는 쪽 = semantic rad 가 음수인 쪽이다."""
    steps = planner.can_steps_from_phases(_phases(), _policy())
    assert steps[0]["target_position_rad"] < 0.0


def test_steps_are_indexed_contiguously():
    steps = planner.can_steps_from_phases(_phases(), _policy())
    assert [s["index"] for s in steps] == list(range(1, len(steps) + 1))


def test_a_phase_list_without_the_lift_leg_is_rejected():
    """close 를 끼울 자리가 없으면 조용히 개방만 하고 끝나면 안 된다."""
    phases = [p for p in _phases() if p["name"] != "pick_grasp_to_lift"]
    with pytest.raises(CanPickContractError, match="exactly one open and one close"):
        planner.can_steps_from_phases(phases, _policy())


def test_planner_targets_the_can_topic_and_forces_the_left_arm():
    assert planner.CAN_TARGET_TOPIC == "/perception/top/can_obb/object_pose_board"
    assert planner.SIDE == "left"
    assert planner.LEFT_SCREEN_X_CORRECTION_M == pytest.approx(0.01372)


def test_pregrasp_and_lift_clearance_cover_the_can_radius():
    assert planner.PICK_PREGRASP_OFFSET_M > 0.053
    assert planner.PICK_LIFT_OFFSET_M > 0.053 / 2.0
