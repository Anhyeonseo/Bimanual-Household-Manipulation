"""캔 파지용 wrist_roll 분기 solver 계약.

`solve_wrist_roll` 은 `roll_new = roll_now + Δfinger_yaw` (gain 1) 를 가정한다.
그 가정은 회전축이 연직일 때만 맞고 이 팔은 작업대에서 그렇지 않다. 여기서는
새 `solve_wrist_roll_branches` 가 (a) 실제로 목표 yaw 를 맞추는 해를 내고,
(b) 가동 한계를 존중하며, (c) 여러 해 중 현재 자세에서 가장 가까운 것을
고른다는 것을 확인한다.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tools.lib.grasp_yaw_kinematics import wrap_half_turn  # noqa: E402

URDF = (
    ROOT
    / "ros2_ws/src/so101_description/urdf/so101_dual_right_data_fit_candidate.urdf"
)
LIMITS = ROOT / "config/bimanual_operational_limits.json"
SHORT_NAMES = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll")

# 2026-08-16 session03 에서 실제로 실행된 오른팔 pick_grasp 자세.
REFERENCE_JOINTS_RAD = (
    -0.29366012646773704,
    2.6135725928114755,
    1.2219831294243892,
    1.264178694179015,
    0.0,
)
RECORDED_PLAN_FINGER_YAW_RAD = -1.278421260830095


@pytest.fixture(scope="module")
def kinematics():
    urdf_parser = pytest.importorskip("urdf_parser_py.urdf")
    assert urdf_parser is not None
    if not URDF.is_file():
        pytest.skip(f"dual URDF is not present: {URDF}")
    from tools.lib.grasp_yaw_kinematics import GraspYawKinematics

    return GraspYawKinematics(URDF, prefix="right_")


@pytest.fixture(scope="module")
def roll_limits():
    document = json.loads(LIMITS.read_text(encoding="utf-8"))
    arm = document["arms"]["right"]
    return (
        arm["wrist_roll"]["minimum_urad"] / 1.0e6,
        arm["wrist_roll"]["maximum_urad"] / 1.0e6,
    )


@pytest.fixture(scope="module")
def reference_positions(kinematics):
    return dict(zip(kinematics.arm_joints, REFERENCE_JOINTS_RAD, strict=True))


def test_reference_pose_reproduces_the_recorded_plan(
    kinematics, reference_positions
):
    """모델이 실기 계획과 같은 자세를 보고 있는지부터 확인한다."""
    achieved = kinematics.finger_yaw(reference_positions)
    drift = abs(wrap_half_turn(achieved - RECORDED_PLAN_FINGER_YAW_RAD))
    assert drift < 1.0e-6


def test_roll_is_not_a_unit_gain_handle_on_finger_yaw(
    kinematics, reference_positions
):
    """gain 1 가정이 이 로봇에서 틀리다는 사실 자체를 고정한다."""
    names = kinematics.arm_joints
    step = 1.0e-5
    forward = dict(reference_positions)
    forward[names[4]] = reference_positions[names[4]] + step
    backward = dict(reference_positions)
    backward[names[4]] = reference_positions[names[4]] - step
    gain = wrap_half_turn(
        kinematics.finger_yaw(forward) - kinematics.finger_yaw(backward)
    ) / (2.0 * step)
    assert 0.3 < gain < 0.8
    assert not math.isclose(gain, 1.0, rel_tol=0.1)


@pytest.mark.parametrize("can_yaw_deg", [-89.0, -45.0, -12.0, 0.0, 31.0, 67.0, 89.0])
def test_every_can_yaw_has_an_in_limit_branch_that_actually_crosses(
    kinematics, reference_positions, roll_limits, can_yaw_deg
):
    lower, upper = roll_limits
    target = wrap_half_turn(math.radians(can_yaw_deg) + math.pi / 2.0)
    solution = kinematics.solve_wrist_roll_branches(
        reference_positions, target, lower, upper
    )
    assert solution["branch_count"] >= 1
    for branch in solution["branches"]:
        assert lower <= branch["wrist_roll_rad"] <= upper
        assert branch["residual_rad"] < 1.0e-6
        assert branch["limit_margin_rad"] >= 0.0


def test_branches_are_ordered_by_rotation_from_the_current_pose(
    kinematics, reference_positions, roll_limits
):
    lower, upper = roll_limits
    distances = []
    for index in range(180):
        target = wrap_half_turn(
            math.radians(-90.0 + index) + math.pi / 2.0
        )
        solution = kinematics.solve_wrist_roll_branches(
            reference_positions, target, lower, upper
        )
        rotations = [
            abs(branch["rotation_from_current_rad"])
            for branch in solution["branches"]
        ]
        assert rotations == sorted(rotations)
        if solution["selected"] is not None:
            assert solution["selected"] is solution["branches"][0]
            distances.append(rotations[0])
    # 최단 분기를 고르므로 반바퀴(180도) 회전을 요구하는 경우가 없어야 한다.
    assert max(distances) < math.radians(180.0)


def test_some_can_yaws_have_two_in_limit_branches(
    kinematics, reference_positions, roll_limits
):
    """roll span 이 195도로 180도보다 넓어 두 해가 공존하는 구간이 있다.

    이 구간이 존재하기 때문에 '가까운 쪽 선택'이 실제로 의미를 가진다.
    """
    lower, upper = roll_limits
    two_branch = 0
    for index in range(180):
        target = wrap_half_turn(math.radians(-90.0 + index) + math.pi / 2.0)
        solution = kinematics.solve_wrist_roll_branches(
            reference_positions, target, lower, upper
        )
        if solution["branch_count"] >= 2:
            two_branch += 1
            near, far = solution["branches"][0], solution["branches"][1]
            assert abs(near["rotation_from_current_rad"]) <= abs(
                far["rotation_from_current_rad"]
            )
    assert two_branch > 0


def test_narrowed_limits_drop_the_out_of_range_branch(
    kinematics, reference_positions, roll_limits
):
    """한계 검사가 분기 선택보다 먼저라는 계약."""
    lower, upper = roll_limits
    target = wrap_half_turn(math.radians(20.0) + math.pi / 2.0)
    full = kinematics.solve_wrist_roll_branches(
        reference_positions, target, lower, upper
    )
    assert full["branch_count"] >= 1
    kept = full["branches"][0]["wrist_roll_rad"]

    narrow_lower = kept - 0.02
    narrow_upper = kept + 0.02
    narrowed = kinematics.solve_wrist_roll_branches(
        reference_positions, target, narrow_lower, narrow_upper
    )
    assert narrowed["branch_count"] == 1
    assert narrow_lower <= narrowed["selected"]["wrist_roll_rad"] <= narrow_upper

    # 해가 하나도 없는 창에서는 조용히 성공하지 않는다.
    empty = kinematics.solve_wrist_roll_branches(
        reference_positions, target, kept + 0.05, kept + 0.10
    )
    assert empty["branch_count"] == 0
    assert empty["selected"] is None


def test_solver_rejects_degenerate_arguments(
    kinematics, reference_positions, roll_limits
):
    lower, upper = roll_limits
    with pytest.raises(ValueError):
        kinematics.solve_wrist_roll_branches(
            reference_positions, 0.0, upper, lower
        )
    with pytest.raises(ValueError):
        kinematics.solve_wrist_roll_branches(
            reference_positions, 0.0, lower, upper, samples=2
        )
