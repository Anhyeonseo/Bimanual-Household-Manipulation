"""왼팔 캔 파지 endpoint 해법 계약."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tools.lib.can_pick_application import (  # noqa: E402
    CanJawContract,
    CanPickContractError,
    CanPickPolicy,
    finger_target_yaw,
    load_calibrated_region,
    load_can_pick_policy,
    required_jaw_width_mm,
    solve_can_pick_endpoint,
)

HOMOGRAPHY = (
    ROOT
    / "ros2_ws/src/manipulation_camera_manager/config/top_worktable_homography.yaml"
)

URDF = (
    ROOT
    / "ros2_ws/src/so101_description/urdf/so101_dual_right_data_fit_candidate.urdf"
)
LIMITS = ROOT / "config/bimanual_operational_limits.json"
SHORT_NAMES = ("base", "shoulder", "elbow", "wrist_flex", "wrist_roll")
CAN_LENGTH_MM = 132.44
CAN_DIAMETER_MM = 53.0
GRASP_Z_M = 0.0053


def _jaw(open_gap_mm=60.0, grasp_gap_mm=44.0):
    return CanJawContract(
        open_gap_mm=open_gap_mm,
        grasp_gap_mm=grasp_gap_mm,
        open_command_rad=-0.5,
        grasp_command_rad=0.1,
        contact_threshold_raw=14,
        release_tolerance_raw=30,
        can_diameter_mm=CAN_DIAMETER_MM,
        provenance="test_fixture_not_measured",
    )


@pytest.fixture(scope="module")
def left():
    pytest.importorskip("urdf_parser_py.urdf")
    if not URDF.is_file():
        pytest.skip(f"dual URDF is not present: {URDF}")
    from tools.lib.grasp_yaw_kinematics import GraspYawKinematics

    kinematics = GraspYawKinematics(URDF, prefix="left_")
    document = json.loads(LIMITS.read_text(encoding="utf-8"))
    arm = document["arms"]["left"]
    lower = np.array([arm[n]["minimum_urad"] / 1.0e6 for n in SHORT_NAMES])
    upper = np.array([arm[n]["maximum_urad"] / 1.0e6 for n in SHORT_NAMES])
    return kinematics, lower, upper


@pytest.fixture()
def policy():
    return CanPickPolicy(
        crossing_tolerance_rad=math.radians(3.0),
        position_tolerance_m=0.0021,
        maximum_approach_tilt_rad=None,
        jaw=_jaw(),
    )


# --- 순수 기하 ---


@pytest.mark.parametrize(
    "can_deg,expected_deg",
    [(0.0, 90.0), (45.0, -45.0), (-45.0, 45.0), (89.0, -1.0), (90.0, 0.0)],
)
def test_finger_target_is_perpendicular_and_wrapped(can_deg, expected_deg):
    achieved = math.degrees(finger_target_yaw(math.radians(can_deg)))
    assert achieved == pytest.approx(expected_deg, abs=1.0e-9)


def test_required_jaw_width_grows_with_crossing_error():
    at_zero = required_jaw_width_mm(0.0, CAN_LENGTH_MM, CAN_DIAMETER_MM)
    assert at_zero == pytest.approx(CAN_DIAMETER_MM)
    widths = [
        required_jaw_width_mm(math.radians(d), CAN_LENGTH_MM, CAN_DIAMETER_MM)
        for d in (0.0, 5.0, 10.0, 20.0, 35.9)
    ]
    assert widths == sorted(widths)
    # 35.9도 교차 오차에서는 캔 지름의 두 배가 넘는 개방 폭이 필요하다.
    assert widths[-1] > 2.0 * CAN_DIAMETER_MM


def test_calibrated_region_is_read_from_the_homography_not_hardcoded():
    """R0-B 재보정 span과 300 mm 수건 envelope를 런타임이 따라야 한다."""
    region = load_calibrated_region(HOMOGRAPHY)
    assert region.span_xy_m[0] == pytest.approx(0.377296, abs=1e-5)
    assert region.span_xy_m[1] == pytest.approx(0.371513, abs=1e-5)
    assert min(region.span_xy_m) >= 0.360
    assert region.table_z_m == pytest.approx(-0.005)
    assert len(region.source_sha256) == 64


def test_calibrated_region_gate_rejects_outside():
    region = load_calibrated_region(HOMOGRAPHY)
    low_x, high_x = region.x_bounds_m
    low_y, high_y = region.y_bounds_m
    region.require_inside((low_x + high_x) / 2.0, (low_y + high_y) / 2.0)
    with pytest.raises(CanPickContractError):
        region.require_inside(low_x - 0.01, (low_y + high_y) / 2.0)
    with pytest.raises(CanPickContractError):
        region.require_inside((low_x + high_x) / 2.0, high_y + 0.01)
    with pytest.raises(CanPickContractError):
        region.require_inside(float("nan"), low_y)


def test_board_to_workcell_is_translation_only():
    region = load_calibrated_region(HOMOGRAPHY)
    x, y = region.board_to_workcell_xy_m(0.0, 0.0)
    assert (x, y) == pytest.approx(region.origin_xy_m)
    # 2026-08-17 라이브 검출값이 실제로 보정 영역 안에 떨어지는지.
    x, y = region.board_to_workcell_xy_m(0.09488, 0.23316)
    region.require_inside(x, y)


# --- 그리퍼 null gate ---


def test_uncommissioned_jaw_contract_is_rejected():
    incomplete = CanJawContract(
        open_gap_mm=None,
        grasp_gap_mm=44.0,
        open_command_rad=None,
        grasp_command_rad=0.1,
        contact_threshold_raw=None,
        release_tolerance_raw=None,
        can_diameter_mm=CAN_DIAMETER_MM,
        provenance="not_measured",
    )
    with pytest.raises(CanPickContractError) as error:
        incomplete.require_commissioned()
    assert "open_gap_mm" in str(error.value)


def test_open_gap_narrower_than_the_can_is_rejected():
    """44 mm 로 벌린 채 접근하면 캔을 밀어버린다. 파지 폭과 개방 폭은 다르다."""
    with pytest.raises(CanPickContractError) as error:
        _jaw(open_gap_mm=44.0).require_commissioned()
    assert "does not clear" in str(error.value)


def test_commissioned_jaw_contract_passes():
    _jaw(open_gap_mm=60.0, grasp_gap_mm=44.0).require_commissioned()


# --- 개방 폭과 교차각의 결합 ---


def _policy_with(open_gap_mm: float, tolerance_deg: float) -> CanPickPolicy:
    return CanPickPolicy(
        crossing_tolerance_rad=math.radians(tolerance_deg),
        position_tolerance_m=0.0021,
        maximum_approach_tilt_rad=None,
        jaw=_jaw(open_gap_mm=open_gap_mm),
    )


def test_open_gap_must_cover_the_declared_crossing_tolerance():
    """지름만 넘기면 통과하던 개방 폭 검사가 교차각을 감당하는지도 본다.

    61 mm 개방은 53 mm 캔을 '통과'시키지만 허용 오차가 약 3.4 도뿐이다.
    인식 yaw 오차만 p95 2.36 도이므로 6 도 계약과 함께 쓰면 조가 안 벌어진다.
    """
    # 지름은 넘지만 6도를 감당 못 한다.
    _jaw(open_gap_mm=61.0).require_commissioned()
    with pytest.raises(CanPickContractError) as error:
        _policy_with(61.0, 6.0).require_open_gap_covers_tolerance()
    assert "crossing tolerance" in str(error.value)

    # 같은 개방 폭이라도 허용 오차를 3도로 줄이면 통과한다.
    needed = _policy_with(61.0, 3.0).require_open_gap_covers_tolerance()
    assert needed == pytest.approx(
        required_jaw_width_mm(math.radians(3.0), CAN_LENGTH_MM, CAN_DIAMETER_MM)
    )


def test_open_gap_of_seventy_one_millimetres_covers_eight_degrees():
    """실측 목표를 정하는 근거. 8 도를 감당하려면 70.92 mm 가 필요하다."""
    needed = _policy_with(71.0, 8.0).require_open_gap_covers_tolerance()
    assert needed == pytest.approx(70.92, abs=0.01)


def test_open_gap_check_still_reports_uncommissioned_values_first():
    """실측 전에는 '안 쟀다' 가 먼저 나와야 한다. None 으로 폭을 비교하지 않는다."""
    policy = CanPickPolicy(
        crossing_tolerance_rad=math.radians(6.0),
        position_tolerance_m=0.0021,
        maximum_approach_tilt_rad=None,
        jaw=CanJawContract(
            open_gap_mm=None,
            grasp_gap_mm=44.0,
            open_command_rad=None,
            grasp_command_rad=None,
            contact_threshold_raw=None,
            release_tolerance_raw=None,
            can_diameter_mm=CAN_DIAMETER_MM,
            provenance="not_measured",
        ),
    )
    with pytest.raises(CanPickContractError) as error:
        policy.require_open_gap_covers_tolerance()
    assert "not commissioned" in str(error.value)


# --- 계약 파일 ---


def test_candidate_contract_loads_and_refuses_to_plan_before_measurement():
    """저장소의 후보 계약은 실측 전에는 계획을 거부해야 한다."""
    policy, provenance = load_can_pick_policy(
        ROOT / "config/can_pick_contract.candidate.json"
    )
    assert provenance["status"] == "COMMISSIONING_REQUIRED"
    assert provenance["jaw_provenance"] == "not_measured"
    assert policy.jaw.can_diameter_mm == pytest.approx(CAN_DIAMETER_MM)
    assert policy.jaw.can_length_mm == pytest.approx(CAN_LENGTH_MM)
    assert policy.jaw.grasp_gap_mm == pytest.approx(44.0)
    # 로더가 null 을 기본값으로 채우지 않는다.
    assert policy.jaw.open_command_rad is None
    with pytest.raises(CanPickContractError):
        policy.require_open_gap_covers_tolerance()


def test_candidate_contract_declares_the_left_pick_only_scope():
    document = json.loads(
        (ROOT / "config/can_pick_contract.candidate.json").read_text(
            encoding="utf-8"
        )
    )
    assert document["selected_arm"] == "left"
    assert document["motion_authorized"] is False
    assert document["motion_contract"]["floor_sweep_authorized"] is False
    assert document["motion_contract"]["wrist_roll_locked_during_descent"] is True
    assert document["acceptance_limits"]["maximum_approach_tilt_deg"] is None


# --- 결합 해법 ---


@pytest.mark.parametrize("can_deg", [-80.0, -40.0, -12.0, 0.0, 27.0, 63.0, 88.0])
def test_solution_meets_position_and_crossing_for_every_can_yaw(
    left, policy, can_deg
):
    kinematics, lower, upper = left
    names = kinematics.arm_joints
    target = (0.40, -0.14, GRASP_Z_M)
    result = solve_can_pick_endpoint(
        kinematics,
        names,
        target,
        math.radians(can_deg),
        (0.0,) * 5,
        lower,
        upper,
        policy,
    )
    assert result["position_residual_m"] <= policy.position_tolerance_m
    assert result["crossing_error_rad"] <= policy.crossing_tolerance_rad
    assert result["joint_limit_margin_rad"] >= -1.0e-9
    # 최단 분기를 골랐으므로 반바퀴를 넘게 돌 이유가 없다.
    assert abs(result["wrist_roll_rotation_from_current_rad"]) < math.pi

    # FK 로 독립 재검증: 정말 캔 장축을 90도로 가로지르는가.
    positions = dict(
        zip(names, result["joint_positions_rad"], strict=True)
    )
    can_axis_yaw = math.radians(can_deg)
    finger = kinematics.finger_yaw(positions)
    crossing = abs(math.degrees(finger - can_axis_yaw)) % 180.0
    assert min(abs(crossing - 90.0), abs(crossing + 90.0 - 180.0)) < 3.0


def test_solver_changes_roll_for_different_can_axes(left, policy):
    """캔 방향에 맞춰 wrist roll 해가 실제로 달라져야 한다."""
    kinematics, lower, upper = left
    rolls = []
    for can_deg in (-60.0, -20.0, 20.0, 60.0):
        result = solve_can_pick_endpoint(
            kinematics,
            kinematics.arm_joints,
            (0.40, -0.14, GRASP_Z_M),
            math.radians(can_deg),
            (0.0,) * 5,
            lower,
            upper,
            policy,
        )
        rolls.append(result["wrist_roll_rad"])
    assert max(rolls) - min(rolls) > math.radians(30.0)
    assert any(abs(value) > math.radians(5.0) for value in rolls)


def test_branch_selection_metadata_is_reported(left, policy):
    """어느 분기를 왜 골랐는지가 계획에 남아야 한다."""
    kinematics, lower, upper = left
    result = solve_can_pick_endpoint(
        kinematics,
        kinematics.arm_joints,
        (0.40, -0.14, GRASP_Z_M),
        math.radians(31.0),
        (0.0,) * 5,
        lower,
        upper,
        policy,
    )
    assert result["wrist_roll_branch_count"] >= 1
    assert result["wrist_roll_branch_index"] < result["wrist_roll_branch_count"]
    candidates = result["wrist_roll_candidates_rad"]
    distances = [abs(value) for value in candidates]
    assert distances == sorted(distances)
    assert result["wrist_roll_rad"] == pytest.approx(
        candidates[result["wrist_roll_branch_index"]], abs=math.radians(2.0)
    )


def test_unreachable_target_is_rejected_not_approximated(left, policy):
    """도달 불가 지점을 근사해로 통과시키지 않는다."""
    kinematics, lower, upper = left
    with pytest.raises(CanPickContractError) as error:
        solve_can_pick_endpoint(
            kinematics,
            kinematics.arm_joints,
            (0.52, -0.28, GRASP_Z_M),
            0.0,
            (0.0,) * 5,
            lower,
            upper,
            policy,
        )
    assert "residual" in str(error.value)


def test_tight_crossing_tolerance_is_enforced(left):
    kinematics, lower, upper = left
    impossible = CanPickPolicy(
        crossing_tolerance_rad=1.0e-12,
        position_tolerance_m=0.0021,
        maximum_approach_tilt_rad=None,
        jaw=_jaw(),
    )
    with pytest.raises(CanPickContractError) as error:
        solve_can_pick_endpoint(
            kinematics,
            kinematics.arm_joints,
            (0.40, -0.14, GRASP_Z_M),
            math.radians(31.0),
            (0.0,) * 5,
            lower,
            upper,
            impossible,
        )
    assert "crossing" in str(error.value)


def test_approach_tilt_limit_is_enforced_when_commissioned(left):
    """접근축 기울기 상한이 채워지면 실제로 거부에 쓰인다."""
    kinematics, lower, upper = left
    strict = CanPickPolicy(
        crossing_tolerance_rad=math.radians(3.0),
        position_tolerance_m=0.0021,
        maximum_approach_tilt_rad=math.radians(5.0),
        jaw=_jaw(),
    )
    with pytest.raises(CanPickContractError) as error:
        solve_can_pick_endpoint(
            kinematics,
            kinematics.arm_joints,
            (0.40, -0.14, GRASP_Z_M),
            0.0,
            (0.0,) * 5,
            lower,
            upper,
            strict,
        )
    assert "tilt" in str(error.value)


def test_bad_argument_shapes_are_rejected(left, policy):
    kinematics, lower, upper = left
    with pytest.raises(ValueError):
        solve_can_pick_endpoint(
            kinematics,
            kinematics.arm_joints,
            (0.40, -0.14, GRASP_Z_M),
            0.0,
            (0.0,) * 4,
            lower,
            upper,
            policy,
        )
    with pytest.raises(ValueError):
        solve_can_pick_endpoint(
            kinematics,
            kinematics.arm_joints,
            (0.40, -0.14, GRASP_Z_M),
            0.0,
            (0.0,) * 5,
            upper,
            lower,
            policy,
        )
