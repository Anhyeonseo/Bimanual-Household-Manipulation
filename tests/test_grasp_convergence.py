"""경계된 수렴 계층의 계약.

여기서 무엇이 거부되는지가 곧 "팔이 목표에 도달했다" 의 정의다.

2026-08-06 A4.5 는 인식도 yaw 도 맞았는데 팔이 명령받은 자세에 `11.7 mm`
못 미쳐 파지에 실패했다. 그 층을 닫는 것이 이 모듈이고, 닫히지 않을 때
조용히 반복하거나 조용히 포기하지 않는 것이 이 시험들의 목적이다.

`evaluate()` 는 순수 함수라 ROS 없이 검증한다. FK 는 주입한다.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

_spec = importlib.util.spec_from_file_location(
    "grasp_convergence", ROOT / "tools" / "grasp_convergence.py"
)
GC = importlib.util.module_from_spec(_spec)
sys.modules["grasp_convergence"] = GC
_spec.loader.exec_module(GC)


JOINTS = ("j0", "j1", "j2")
# 관절값이 그대로 미터가 되는 FK. 시험이 무엇을 단언하는지 한눈에 보이도록
# 일부러 투명하게 만든다. 실제 기구학은 아래 URDF 시험이 따로 고정한다.
def linear_fk(positions: dict[str, float]) -> tuple[float, float, float]:
    return (positions["j0"], positions["j1"], positions["j2"])


NOMINAL = (0.0, 0.0, 0.0)


def policy(**overrides) -> GC.ConvergencePolicy:
    # 이 fixture 의 "관절" 은 곧 미터다(항등 FK). 그래서 관절 델타를 raw 로
    # 환산하는 문턱 검사가 의미를 갖지 않는다 — 12 mm 오차가 0.012 rad =
    # 7.8 raw 로 읽혀 실제보다 훨씬 작아 보인다. 아래 시험들은 판정 논리를
    # 투명하게 보려는 것이므로 문턱을 끄고(0), 문턱 자체는 실제 raw 값으로
    # 별도 시험한다.
    values = {"arm": "left", "ineffective_correction_raw": 0}
    values.update(overrides)
    return GC.ConvergencePolicy(**values)


def start(**overrides) -> GC.ConvergenceState:
    return GC.begin(policy(**overrides), JOINTS, NOMINAL)


def step(state, measured, limits=None):
    return GC.evaluate(state, measured, linear_fk, limits)


# ---------------------------------------------------------------------------
# 받아들이기
# ---------------------------------------------------------------------------


def test_a_pose_inside_the_task_tolerance_is_accepted() -> None:
    state, decision = step(start(), (0.0, 0.0, 0.003))
    assert decision.action == GC.ACCEPT
    assert decision.converged is True
    assert decision.requires_motion is False
    assert decision.error_mm() == pytest.approx(3.0)
    assert state.finished == GC.ACCEPT


def test_the_task_tolerance_boundary_is_inclusive() -> None:
    _, decision = step(start(), (0.0, 0.0, GC.TASK_TOLERANCE_M))
    assert decision.action == GC.ACCEPT


def test_a_pose_just_outside_the_tolerance_is_not_accepted() -> None:
    _, decision = step(start(), (0.0, 0.0, GC.TASK_TOLERANCE_M + 1e-6))
    assert decision.action == GC.CORRECT


# ---------------------------------------------------------------------------
# 보정
# ---------------------------------------------------------------------------


def test_sag_produces_a_bounded_correction_to_the_same_goal() -> None:
    """보정 leg 는 같은 목표를 다시 보낸다. 명령을 바꾸지 않는다.

    바뀌는 것은 출발 위치다. 훨씬 가까운 곳에서 출발하므로 동적 오차가
    작아진다. A4 의 `4 s / 201 sample` 짧은 leg 가 model peak 오차 `0.000`,
    post-settle `14 raw` 였던 것이 그 근거다.
    """
    _, decision = step(start(), (0.0, 0.0, 0.012))
    assert decision.action == GC.CORRECT
    assert decision.next_commanded_rad == NOMINAL
    assert decision.requires_motion is True
    assert decision.error_mm() == pytest.approx(12.0)


def test_a_correction_that_reaches_the_tolerance_is_then_accepted() -> None:
    state, first = step(start(), (0.0, 0.0, 0.012))
    assert first.action == GC.CORRECT
    state, second = step(state, (0.0, 0.0, 0.002))
    assert second.action == GC.ACCEPT
    assert second.iteration == 2


def test_the_error_is_always_measured_against_the_original_goal() -> None:
    """보정하는 동안 목표가 표류하면 안 된다.

    명령을 바꿔가며 그 명령에 대해 오차를 재면 팔은 언제나 "거의 도달" 로
    보이고 실제 목표에서는 계속 멀어질 수 있다.
    """
    state = start()
    state, _ = step(state, (0.0, 0.0, 0.012))
    state, _ = step(state, (0.0, 0.0, 0.0115))
    state, third = step(state, (0.0, 0.0, 0.011))
    # 넘겨명령이 명령을 바꿨더라도 오차는 원래 목표 기준 11 mm 다.
    assert third.error_mm() == pytest.approx(11.0)


# ---------------------------------------------------------------------------
# 정체와 넘겨명령
# ---------------------------------------------------------------------------


def test_a_plateau_triggers_the_measured_overshoot_once() -> None:
    """정체하면 같은 명령을 또 보내봐야 같은 곳에 선다.

    남은 것은 중력 하 정상상태 오차이고 계통적이다. 모델을 세우지 않고
    그 회차의 실측 잔차만큼 넘겨 보낸다.
    """
    state = start()
    state, first = step(state, (0.0, 0.0, 0.010))
    assert first.action == GC.CORRECT
    state, second = step(state, (0.0, 0.0, 0.0098))  # 0.2 mm 개선 = 정체
    assert second.action == GC.OVERSHOOT
    # target' = target + (target - measured)
    assert second.next_commanded_rad == pytest.approx((0.0, 0.0, -0.0098))
    assert second.overshoot_m == pytest.approx(0.0098)
    assert state.overshoot_used is True


def test_the_overshoot_is_used_at_most_once() -> None:
    state = start()
    state, _ = step(state, (0.0, 0.0, 0.010))
    state, second = step(state, (0.0, 0.0, 0.0098))
    assert second.action == GC.OVERSHOOT
    state, third = step(state, (0.0, 0.0, 0.0097))
    assert third.action == GC.FAIL
    assert "plateaued after the measured overshoot" in third.reason


def test_an_overshoot_beyond_the_cap_is_refused() -> None:
    """측정이 잘못됐을 때 팔이 크게 튀면 안 된다."""
    state = start(maximum_correction_m=0.030, maximum_overshoot_m=0.005)
    state, _ = step(state, (0.0, 0.0, 0.020))
    _, decision = step(state, (0.0, 0.0, 0.0199))
    assert decision.action == GC.FAIL
    assert "past the goal" in decision.reason
    assert decision.overshoot_m == pytest.approx(0.0199)


def test_a_joint_at_its_limit_is_clamped_not_refused() -> None:
    """관절 하나가 한계 근처라고 전체 보정을 버리지 않는다.

    이 팔은 pregrasp/grasp 자세에서 WRIST_FLEX 가 상시 하한 근처에 선다
    (2026-08-06: nominal 1197, 하한 1194). A4 의 성공 자세도 하한에서
    7 raw 였다. 예외가 아니라 상시 조건이므로 물리고 보고한다.
    """
    state = start()
    limits = {"j0": (-1.0, 1.0), "j1": (-1.0, 1.0), "j2": (-0.005, 1.0)}
    state, _ = step(state, (0.0, 0.0, 0.010), limits)
    _, decision = step(state, (0.0, 0.0, 0.0098), limits)
    assert decision.action == GC.OVERSHOOT
    assert decision.clamped_joints == ("j2",)
    # 물리는 것은 명령을 줄이는 방향이므로 한계를 넘길 수 없고, 한계에
    # 정확히 걸치지도 않는다(µrad 양자화로 올림되면 거부되기 때문).
    assert decision.next_commanded_rad[2] == pytest.approx(
        -0.005 + GC.JOINT_LIMIT_MARGIN_RAD
    )
    assert decision.next_commanded_rad[2] > -0.005
    assert "clamped at their limits: j2" in decision.reason


def test_a_residual_that_no_joint_can_close_is_refused() -> None:
    """전부 한계에 물리면 이 자세에서 닫을 수 없는 잔차다."""
    state = start()
    limits = {"j0": (0.0, 1.0), "j1": (0.0, 1.0), "j2": (0.0, 1.0)}
    state, _ = step(state, (0.0, 0.0, 0.010), limits)
    _, decision = step(state, (0.0, 0.0, 0.0098), limits)
    assert decision.action == GC.FAIL
    assert "cannot be closed at this pose" in decision.reason
    assert "j2" in decision.clamped_joints


def test_an_overshoot_inside_the_joint_limits_proceeds() -> None:
    state = start()
    limits = {"j0": (-1.0, 1.0), "j1": (-1.0, 1.0), "j2": (-1.0, 1.0)}
    state, _ = step(state, (0.0, 0.0, 0.010), limits)
    _, decision = step(state, (0.0, 0.0, 0.0098), limits)
    assert decision.action == GC.OVERSHOOT


def test_a_joint_without_a_recorded_limit_does_not_block_the_overshoot() -> None:
    state = start()
    state, _ = step(state, (0.0, 0.0, 0.010), {})
    _, decision = step(state, (0.0, 0.0, 0.0098), {})
    assert decision.action == GC.OVERSHOOT


# ---------------------------------------------------------------------------
# 실패는 조용하지 않다
# ---------------------------------------------------------------------------


def test_a_gross_miss_is_not_closed_by_a_short_leg() -> None:
    """처짐이 아니라 실행이 크게 어긋난 경우다.

    충돌 검사를 거치지 않은 짧은 보정 leg 로 덮을 일이 아니다.
    """
    _, decision = step(start(), (0.0, 0.0, GC.MAXIMUM_CORRECTION_M + 0.001))
    assert decision.action == GC.FAIL
    assert "must not be closed by an unplanned short leg" in decision.reason


def test_a_diverging_loop_stops(  ) -> None:
    state = start()
    state, _ = step(state, (0.0, 0.0, 0.010))
    _, decision = step(state, (0.0, 0.0, 0.020))
    assert decision.action == GC.FAIL
    assert "diverging" in decision.reason


def test_the_iteration_budget_is_bounded() -> None:
    """조용히 반복하지 않는다."""
    state = start(maximum_iterations=2, plateau_improvement_m=1e-9)
    state, first = step(state, (0.0, 0.0, 0.0100))
    state, second = step(state, (0.0, 0.0, 0.0080))
    assert (first.action, second.action) == (GC.CORRECT, GC.CORRECT)
    _, third = step(state, (0.0, 0.0, 0.0060))
    assert third.action == GC.FAIL
    assert "after 2 bounded corrections" in third.reason


def test_a_finished_state_cannot_be_stepped_again() -> None:
    state, decision = step(start(), (0.0, 0.0, 0.001))
    assert decision.action == GC.ACCEPT
    with pytest.raises(ValueError, match="already finished"):
        step(state, (0.0, 0.0, 0.001))


def test_a_non_finite_measurement_is_refused() -> None:
    with pytest.raises(ValueError, match="non-finite"):
        step(start(), (0.0, 0.0, float("nan")))


def test_a_measurement_of_the_wrong_width_is_refused() -> None:
    with pytest.raises(ValueError, match="does not match the tracked joints"):
        step(start(), (0.0, 0.0))


# ---------------------------------------------------------------------------
# 양팔 모양
# ---------------------------------------------------------------------------


def test_two_arms_keep_separate_state() -> None:
    """한쪽의 진행이 다른 쪽에 새면 안 된다."""
    left = GC.begin(policy(arm="left"), JOINTS, NOMINAL)
    right = GC.begin(policy(arm="right"), JOINTS, NOMINAL)
    left, left_decision = step(left, (0.0, 0.0, 0.012))
    assert left.iteration == 1
    assert right.iteration == 0
    assert left_decision.arm == "left"
    right, right_decision = step(right, (0.0, 0.0, 0.001))
    assert right_decision.arm == "right"
    assert right_decision.action == GC.ACCEPT
    assert left.finished is None


def test_per_arm_policies_may_differ() -> None:
    """두 팔의 처짐은 같지 않다. 허용치도 팔별이어야 한다."""
    tight = GC.begin(policy(arm="left", task_tolerance_m=0.002), JOINTS, NOMINAL)
    loose = GC.begin(policy(arm="right", task_tolerance_m=0.006), JOINTS, NOMINAL)
    _, left_decision = step(tight, (0.0, 0.0, 0.005))
    _, right_decision = step(loose, (0.0, 0.0, 0.005))
    assert left_decision.action == GC.CORRECT
    assert right_decision.action == GC.ACCEPT


def test_the_caller_is_told_which_arms_to_move_together() -> None:
    """순차로 수렴시키면 주기가 팔 수만큼 배가 된다."""
    _, left_decision = step(
        GC.begin(policy(arm="left"), JOINTS, NOMINAL), (0.0, 0.0, 0.012)
    )
    _, right_decision = step(
        GC.begin(policy(arm="right"), JOINTS, NOMINAL), (0.0, 0.0, 0.001)
    )
    decisions = {"left": left_decision, "right": right_decision}
    assert GC.arms_requiring_motion(decisions) == ("left",)
    assert GC.any_failed(decisions) == ()


def test_a_failed_arm_is_reported_for_the_coordinated_stop() -> None:
    """팔 A 가 수렴에 실패했는데 팔 B 가 물체를 들고 있으면 둘 다 멈춰야 한다."""
    _, failed = step(GC.begin(policy(arm="left"), JOINTS, NOMINAL), (0.0, 0.0, 0.9))
    _, fine = step(GC.begin(policy(arm="right"), JOINTS, NOMINAL), (0.0, 0.0, 0.001))
    assert failed.action == GC.FAIL
    assert GC.any_failed({"left": failed, "right": fine}) == ("left",)


def test_an_arm_name_is_required() -> None:
    """`left_` 하드코딩을 구조로 막는다."""
    with pytest.raises(ValueError, match="do not hardcode a side"):
        GC.ConvergencePolicy(arm="")


# ---------------------------------------------------------------------------
# 정책 자체의 정합성
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "field",
    (
        "task_tolerance_m",
        "maximum_correction_m",
        "plateau_improvement_m",
        "maximum_overshoot_m",
    ),
)
def test_every_bound_must_be_finite_and_positive(field) -> None:
    with pytest.raises(ValueError, match="finite and positive"):
        policy(**{field: 0.0})
    with pytest.raises(ValueError, match="finite and positive"):
        policy(**{field: float("inf")})


def test_the_task_tolerance_must_be_tighter_than_the_correction_limit() -> None:
    with pytest.raises(ValueError, match="tighter than"):
        policy(task_tolerance_m=0.05, maximum_correction_m=0.03)


def test_at_least_one_iteration_is_required() -> None:
    with pytest.raises(ValueError, match="at least 1"):
        policy(maximum_iterations=0)


def test_the_defaults_close_the_measured_grasp_budget() -> None:
    """예산이 실제로 닫히는지 산술로 확인한다.

    A4 스윕에서 명령 offset `0.025`(TCP z `0.0313`)는 놓쳤고
    `0.017`(`0.0233`)는 잡았다. 즉 위로 `8 mm` 는 확실한 실패점이다.
    계획 잔차(허용 상자를 `0.001` 로 조인 뒤 grasp 실측 `0.982 mm`)와
    과제 허용치의 합이 그 실패점보다 작아야 한다.
    """
    measured_plan_residual_m = 0.000982
    known_failure_m = 0.008
    assert GC.TASK_TOLERANCE_M + measured_plan_residual_m < known_failure_m


def test_the_overshoot_cap_covers_the_largest_measured_residual() -> None:
    """넘겨명령의 크기는 그 회차의 실측 잔차와 같다.

    A4.5 에서 실제로 관측된 잔차가 `11.7 mm` 였으므로 상한이 그보다 작으면
    바로 그 상황에서 넘겨명령이 거부된다.
    """
    a45_measured_residual_m = 0.0117
    assert GC.MAXIMUM_OVERSHOOT_M > a45_measured_residual_m
    assert GC.MAXIMUM_CORRECTION_M > a45_measured_residual_m


def test_the_plateau_threshold_is_above_the_plan_residual_noise() -> None:
    """계획 잔차보다 작은 개선을 쫓으면 잡음을 쫓는 것이 된다."""
    assert GC.PLATEAU_IMPROVEMENT_M >= 0.000982


def test_the_task_tolerance_is_tighter_than_the_safety_tolerance() -> None:
    """두 허용치는 다른 질문에 답한다. 과제 쪽이 더 엄격해야 의미가 있다.

    안전 허용치 `30 raw` 는 반경 `0.4 m` 에서 약 `19 mm` 다.
    """
    safety_raw = 30
    reach_m = 0.40
    safety_m = safety_raw * (2.0 * math.pi / 4096.0) * reach_m
    assert GC.TASK_TOLERANCE_M < safety_m
    assert safety_m == pytest.approx(0.0184, abs=0.001)


# ---------------------------------------------------------------------------
# 증거
# ---------------------------------------------------------------------------


def test_the_summary_records_how_the_residual_came_down() -> None:
    """수렴했는지만이 아니라 어떻게 줄었는지가 C2 의 실측 근거가 된다."""
    arm_policy = policy()
    state = GC.begin(arm_policy, JOINTS, NOMINAL)
    decisions = []
    for measured in ((0.0, 0.0, 0.012), (0.0, 0.0, 0.006), (0.0, 0.0, 0.002)):
        state, decision = GC.evaluate(state, measured, linear_fk)
        decisions.append(decision)
    summary = GC.summarize(arm_policy, tuple(decisions))
    assert summary["arm"] == "left"
    assert summary["converged"] is True
    assert summary["residual_mm_by_iteration"] == [12.0, 6.0, 2.0]
    assert summary["final_residual_mm"] == 2.0
    assert summary["overshoot_used"] is False
    assert summary["task_tolerance_mm"] == 4.0
    assert summary["maximum_overshoot_mm"] == 15.0


def test_a_failed_convergence_reports_the_residual() -> None:
    """조용한 포기가 없다.

    `19.9 mm` 에서 정체하면 넘겨명령의 크기도 `19.9 mm` 가 되어 상한
    `15 mm` 를 넘는다. 그래서 넘겨 보내지 않고 잔차와 함께 멈춘다.
    """
    arm_policy = policy()
    state = GC.begin(arm_policy, JOINTS, NOMINAL)
    state, first = GC.evaluate(state, (0.0, 0.0, 0.020), linear_fk)
    state, second = GC.evaluate(state, (0.0, 0.0, 0.0199), linear_fk)
    summary = GC.summarize(arm_policy, (first, second))
    assert summary["converged"] is False
    assert summary["final_action"] == GC.FAIL
    assert summary["final_residual_mm"] == pytest.approx(19.9)
    assert summary["residual_mm_by_iteration"] == [20.0, 19.9]
    assert summary["overshoot_used"] is False
    assert "past the goal" in summary["final_reason"]


def test_a_plateau_within_the_overshoot_cap_is_reported_as_used() -> None:
    arm_policy = policy()
    state = GC.begin(arm_policy, JOINTS, NOMINAL)
    state, first = GC.evaluate(state, (0.0, 0.0, 0.010), linear_fk)
    state, second = GC.evaluate(state, (0.0, 0.0, 0.0098), linear_fk)
    state, third = GC.evaluate(state, (0.0, 0.0, 0.003), linear_fk)
    summary = GC.summarize(arm_policy, (first, second, third))
    assert summary["overshoot_used"] is True
    assert summary["converged"] is True
    assert summary["final_residual_mm"] == pytest.approx(3.0)


# ---------------------------------------------------------------------------
# 실제 기구학. FK 가 MoveIt 과 같은 자세를 내야 잔차가 mm 로 의미를 갖는다.
# ---------------------------------------------------------------------------

ARM_URDF = ROOT / "ros2_ws/src/so101_description/urdf/so101_left.urdf.xacro"

# 2026-08-06 에 실행 중인 move_group 의 `/compute_fk` 가 돌려준 값이다.
# 관절값은 docs/test-results/evidence/2026-07-30-stage7-plan-only-expanded-pass.json
# 에 기록된 MoveIt 의 IK 해 그대로다.
MOVEIT_FK_TRUTH = (
    (
        (
            0.3442161135363389,
            1.7769621125924755,
            0.6762137844648894,
            1.2867256661528759,
            0.14500400508445102,
        ),
        (0.368408, -0.126665, 0.110616),
    ),
    (
        (
            0.3408464397563732,
            2.097319372721346,
            0.7575466865952764,
            1.2956934305712577,
            0.11056092170128994,
        ),
        (0.366054, -0.124599, 0.029103),
    ),
)


@pytest.fixture(scope="module")
def kinematics(tmp_path_factory):
    pytest.importorskip("urdf_parser_py")
    try:
        expanded = subprocess.run(
            ["xacro", str(ARM_URDF)],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    except (FileNotFoundError, subprocess.CalledProcessError) as error:
        pytest.skip(f"xacro unavailable: {error}")
    path = tmp_path_factory.mktemp("urdf") / "so101_left.urdf"
    path.write_text(expanded, encoding="utf-8")
    from grasp_yaw_kinematics import GraspYawKinematics

    return GraspYawKinematics(path)


def test_the_offline_fk_matches_moveit_to_the_micrometre(kinematics) -> None:
    """잔차를 재는 모델이 계획을 만든 모델과 달라지면 그 차이가 잔차로 둔갑한다.

    2026-08-06 에 `/compute_fk` 로 받은 값과 이 모듈의 FK 를 대조한다.
    두 값이 같아야 오프라인 판정과 실기 판정이 같은 자를 쓴다.
    """
    from grasp_yaw_kinematics import ARM_JOINTS

    for positions, expected in MOVEIT_FK_TRUTH:
        actual = kinematics.tcp_position(dict(zip(ARM_JOINTS, positions)))
        for value, truth in zip(actual, expected, strict=True):
            assert value == pytest.approx(truth, abs=1.0e-6)


def test_the_real_kinematics_drive_the_convergence_decision(kinematics) -> None:
    """실제 기구학에서도 판정이 성립하는지 본다.

    A4.5 는 명령 자세에 `1.68도` 못 미쳤다. 그 각도 오차를 SHOULDER 에
    넣으면 TCP 가 얼마나 밀리는지 FK 로 구하고, 그것이 과제 허용치 밖으로
    판정되는지 확인한다.
    """
    from grasp_yaw_kinematics import ARM_JOINTS

    nominal = MOVEIT_FK_TRUTH[1][0]
    measured = list(nominal)
    measured[1] += math.radians(1.68)

    def fk(named: dict[str, float]) -> tuple[float, float, float]:
        return tuple(kinematics.tcp_position(named))

    state = GC.begin(policy(), ARM_JOINTS, nominal)
    state, decision = GC.evaluate(state, tuple(measured), fk)
    assert decision.action == GC.CORRECT
    # 2026-08-06 에 계산한 값은 반경 0.40 m 가정의 11.7 mm 였다. FK 로 실제
    # 자세에서 구하면 그 근방이어야 한다.
    assert 8.0 < decision.error_mm() < 16.0


def test_the_kinematics_are_named_by_arm_prefix_not_hardcoded() -> None:
    """양팔이 되면 오른팔이 같은 코드를 그대로 써야 한다.

    지금 하드코딩해두면 그때 갈라진 복사본이 생기고, 두 팔의 기구학이
    조용히 달라진다.
    """
    import grasp_yaw_kinematics as GYK

    assert GYK.arm_joint_names("right_") == (
        "right_base_joint",
        "right_shoulder_joint",
        "right_elbow_joint",
        "right_wrist_flex_joint",
        "right_wrist_roll_joint",
    )
    assert GYK.wrist_roll_joint("right_") == "right_wrist_roll_joint"
    # 기본값은 왼팔이라 기존 호출부가 그대로 동작한다.
    assert GYK.ARM_JOINTS == GYK.arm_joint_names("left_")


def test_no_arm_side_is_hardcoded_in_the_kinematics_body() -> None:
    source = (ROOT / "tools" / "grasp_yaw_kinematics.py").read_text(
        encoding="utf-8"
    )
    body = "\n".join(
        line
        for line in source.splitlines()
        if not line.lstrip().startswith("#")
        and "DEFAULT_PREFIX" not in line
        and '"""' not in line
        and "`left_" not in line
    )
    assert "left_" not in body, "팔 이름이 코드 본문에 남아 있다"


def test_the_convergence_library_never_names_a_side() -> None:
    source = (ROOT / "tools" / "grasp_convergence.py").read_text(
        encoding="utf-8"
    )
    for side in ("left_", "right_"):
        assert side not in source


# ---------------------------------------------------------------------------
# 문턱 아래 명령은 서보를 움직이지 못한다 (2026-08-06 C2 실측)
# ---------------------------------------------------------------------------

RAW = 2.0 * math.pi / 4096.0  # 1 raw 에 해당하는 rad

# 항등 FK 는 관절값을 그대로 미터로 읽어 `18 raw` 를 `27.6 mm` 로 만든다.
# 실제 팔은 반경 `0.4 m` 근처에서 동작하므로 `18 raw` 가 약 `11 mm` 다.
# 문턱과 넘겨명령 상한을 같이 보려면 두 축척이 실제와 같은 관계여야 한다.
REACH_M = 0.4


def reach_fk(positions: dict[str, float]) -> tuple[float, float, float]:
    return tuple(positions[name] * REACH_M for name in JOINTS)


def test_a_sub_threshold_correction_goes_straight_to_the_overshoot() -> None:
    """같은 목표를 다시 보내도 서보가 움직이지 않으면 회차를 낭비하지 않는다.

    2026-08-06 C2: pregrasp 에서 잔차 12.6 mm 로 보정 leg 를 보냈는데 명령
    델타가 `+4, -4, -18, -9, +8 raw` 였고, 문턱을 넘긴 ELBOW 18 raw 조차
    0 raw 움직였다. 잔차는 12.6 -> 15.7 mm 로 오히려 나빠졌다.
    """
    arm_policy = GC.ConvergencePolicy(arm="left")
    state = GC.begin(arm_policy, JOINTS, NOMINAL)
    # 최대 관절 델타 = 10 raw (문턱 18 이하), TCP 로는 6.1 mm (허용치 4 mm 초과).
    _, decision = GC.evaluate(state, (0.0, 0.0, 10 * RAW), reach_fk)
    assert decision.action == GC.OVERSHOOT
    assert decision.iteration == 1  # 회차를 낭비하지 않았다
    assert "would command only 10 raw" in decision.reason


def test_a_delta_of_exactly_the_measured_threshold_is_treated_as_ineffective(
) -> None:
    """정확히 18 raw 가 움직이지 않는 것을 봤다. 부동소수로 새어나가면 안 된다.

    2026-08-06 에 실제로 새어나갔다 — 18 raw 델타가 `18.00000000000001` 로
    계산되어 `<= 18` 을 빠져나갔다. 서보 명령은 정수이므로 반올림해야 한다.
    """
    assert GC.radians_to_raw_delta(18 * RAW) == 18
    arm_policy = GC.ConvergencePolicy(arm="left")
    state = GC.begin(arm_policy, JOINTS, NOMINAL)
    _, decision = GC.evaluate(state, (0.0, 0.0, 18 * RAW), reach_fk)
    assert decision.action == GC.OVERSHOOT


def test_a_supra_threshold_correction_is_still_attempted_first() -> None:
    """문턱을 확실히 넘는 명령이면 보정을 먼저 시도한다. 증거 없이 건너뛰지 않는다."""
    arm_policy = GC.ConvergencePolicy(arm="left")
    state = GC.begin(arm_policy, JOINTS, NOMINAL)
    _, decision = GC.evaluate(state, (0.0, 0.0, 40 * RAW), reach_fk)
    assert decision.action == GC.CORRECT


def test_the_overshoot_doubles_the_command_delta() -> None:
    """넘겨명령이 문턱을 넘는 명령을 만드는 유일한 수단이다."""
    arm_policy = GC.ConvergencePolicy(arm="left")
    state = GC.begin(arm_policy, JOINTS, NOMINAL)
    measured = (0.0, 0.0, 18 * RAW)
    _, decision = GC.evaluate(state, measured, reach_fk)
    commanded_delta = GC.radians_to_raw_delta(
        decision.next_commanded_rad[2] - measured[2]
    )
    assert commanded_delta == 36
    assert commanded_delta > GC.INEFFECTIVE_CORRECTION_RAW


def test_the_threshold_is_traceable_to_the_repository_measurement() -> None:
    """추정한 일반 문턱이 아니라 이 하드웨어에서 재본 값이다."""
    delta_source = (
        ROOT / "tools" / "execute_buffered_joint_delta_once.py"
    ).read_text(encoding="utf-8")
    assert "MINIMUM_OBSERVABLE_COMMAND_RAW = 16" in delta_source
    # 저장소 값(16)은 하한이고, C2 에서 18 raw 가 움직이지 않는 것을 봤다.
    assert GC.INEFFECTIVE_CORRECTION_RAW >= 16
    assert GC.INEFFECTIVE_CORRECTION_RAW == 18


def test_the_summary_reports_clamped_joints() -> None:
    arm_policy = GC.ConvergencePolicy(
        arm="left", task_tolerance_m=0.001, ineffective_correction_raw=0
    )
    limits = {"j0": (-1.0, 1.0), "j1": (-1.0, 1.0), "j2": (-0.005, 1.0)}
    state = GC.begin(arm_policy, JOINTS, NOMINAL)
    state, first = GC.evaluate(state, (0.0, 0.0, 0.010), linear_fk, limits)
    state, second = GC.evaluate(state, (0.0, 0.0, 0.0098), linear_fk, limits)
    summary = GC.summarize(arm_policy, (first, second))
    assert summary["clamped_joints"] == ["j2"]


def test_a_clamped_joint_never_sits_exactly_on_its_limit() -> None:
    """buffered leg 는 위치를 정수 µrad 로 인코딩한다.

    한계에 정확히 물린 값은 올림되어 한계를 넘고, bridge 의 검증 여유
    `1e-9` 는 µrad 양자화 `1e-6` 을 덮지 못해 goal 이 거부된다.
    2026-08-06 C2 에서 WRIST_FLEX 가 상한을 `+4.07e-7 rad` 넘어 거부됐다.
    """
    state = start()
    upper = 0.005
    limits = {"j0": (-1.0, 1.0), "j1": (-1.0, 1.0), "j2": (-1.0, upper)}
    state, _ = step(state, (0.0, 0.0, -0.010), limits)
    _, decision = step(state, (0.0, 0.0, -0.0098), limits)
    assert decision.action == GC.OVERSHOOT
    assert decision.clamped_joints == ("j2",)
    commanded = decision.next_commanded_rad[2]
    assert commanded < upper, "한계에 정확히 걸쳤다"
    assert upper - commanded == pytest.approx(GC.JOINT_LIMIT_MARGIN_RAD)
    # µrad 로 인코딩해도 한계를 넘지 않아야 한다.
    assert round(commanded * 1e6) / 1e6 <= upper


def test_the_limit_margin_covers_the_microradian_quantisation() -> None:
    assert GC.JOINT_LIMIT_MARGIN_RAD >= 1.0e-6
    # 그러면서도 보정량을 의미 있게 깎지 않아야 한다.
    assert GC.radians_to_raw_delta(GC.JOINT_LIMIT_MARGIN_RAD) == 0


def test_a_range_narrower_than_the_margin_leaves_nothing_to_command() -> None:
    """구간이 여유보다 좁으면 중앙으로 보내고, 그러면 편향이 남지 않는다."""
    state = start()
    limits = {"j0": (-1.0, 1.0), "j1": (-1.0, 1.0), "j2": (-1e-6, 1e-6)}
    state, _ = step(state, (0.0, 0.0, 0.010), limits)
    _, decision = step(state, (0.0, 0.0, 0.0098), limits)
    assert decision.action == GC.FAIL
    assert "cannot be closed at this pose" in decision.reason
    assert "j2" in decision.clamped_joints


# ---------------------------------------------------------------------------
# 테이블 바닥. planning scene 이 비어 있어 하위 계획기가 막지 못한다.
# ---------------------------------------------------------------------------


def test_an_overshoot_below_the_workspace_floor_is_refused() -> None:
    """2026-08-06 확인: MoveIt planning scene 에 충돌 객체도 octomap 도 없다.

    자기충돌 외에는 아무것도 막지 못하므로 테이블 바닥은 여기서 지켜야 한다.
    """
    arm_policy = GC.ConvergencePolicy(
        arm="left",
        ineffective_correction_raw=0,
        minimum_tcp_z_m=0.020,
    )
    state = GC.begin(arm_policy, JOINTS, (0.0, 0.0, 0.025))
    # 측정이 목표보다 **위** 라면 넘겨명령은 아래로 나간다.
    state, _ = GC.evaluate(state, (0.0, 0.0, 0.035), linear_fk)
    _, decision = GC.evaluate(state, (0.0, 0.0, 0.0348), linear_fk)
    assert decision.action == GC.FAIL
    assert "workspace floor" in decision.reason
    assert "planning scene carries no table" in decision.reason


def test_the_usual_upward_overshoot_is_unaffected_by_the_floor() -> None:
    """중력 처짐은 아래로 생기므로 넘겨명령은 보통 위로 나간다.

    A4.5 실측: 측정 z 0.0144, 목표 0.0233 -> 넘겨명령 0.0322.
    정상 동작에서 이 검사는 걸리지 않아야 한다.
    """
    arm_policy = GC.ConvergencePolicy(
        arm="left",
        ineffective_correction_raw=0,
        minimum_tcp_z_m=0.020,
    )
    state = GC.begin(arm_policy, JOINTS, (0.0, 0.0, 0.0233))
    state, _ = GC.evaluate(state, (0.0, 0.0, 0.0144), linear_fk)
    _, decision = GC.evaluate(state, (0.0, 0.0, 0.0146), linear_fk)
    assert decision.action == GC.OVERSHOOT
    assert decision.next_commanded_rad[2] > 0.0233


def test_the_floor_is_off_by_default() -> None:
    """pregrasp 처럼 테이블에서 먼 곳에서는 불필요한 제약을 걸지 않는다."""
    assert GC.ConvergencePolicy(arm="left").minimum_tcp_z_m is None


def test_the_floor_default_matches_the_validated_workspace() -> None:
    """새로 지어낸 수가 아니라 저장소가 이미 선언한 작업영역 하한이다."""
    source = (
        ROOT / "tools" / "assemble_pick_place_plan_only.py"
    ).read_text(encoding="utf-8")
    assert "WORKSPACE_TCP_Z_M = (0.02, 0.15)" in source
    assert GC.WORKSPACE_TCP_Z_FLOOR_M == 0.02
