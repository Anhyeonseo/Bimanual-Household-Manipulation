#!/usr/bin/env python3
"""명령한 자세에 실제로 도달할 때까지 경계된 보정을 반복하는 판정 계층.

**무엇이 문제였나.**

2026-08-06 A4.5 가 인식 좌표로 하강했으나 펜을 집지 못했다. 인식은 정확했고
(표본 흔들림 `0.77 mm`) 손가락 방향도 맞았다(`10.7도` 오차로 가로질렀다).
**팔이 명령받은 자세에 `1.68도` 못 미쳤고 반경 `0.40 m` 에서 그것은 `11.7 mm`
였다.** 시스템은 자세를 명령하고 도달했다고 가정한다. post-settle 이 오차를
재긴 하지만 보고만 하고 아무것도 하지 않는다.

A4 에서 `PICK_GRASP_OFFSET_M = 0.017` 이 맞았던 것은 그 회차의 처짐이 작았기
때문이다(post-settle `14 raw`). A4.5 는 `19 raw` 였다. offset 은 상수가 아니며
자세마다 달라진다. 상수를 키우는 것은 답이 아니다.

**이 모듈은 계획하지 않고 움직이지도 않는다.** 순수 판정 함수다. 측정된
자세를 받아 "받아들일지, 다시 보낼지, 넘겨 보낼지, 포기할지" 를 답한다.
실제 이동은 호출자가 기존 `plan_buffered_segment_leg.py` 파이프라인으로 한다.

**왜 bridge 안이 아닌가.** 보정은 현재 anchor 에서 같은 목표로 새 궤적을
만드는 일이고 그것은 계획이다. bridge 는 계획하지 않는다. 또한 Action 이
궤적 종료 후에 팔을 더 움직이면 그 계약이 예측 불가능해지고, 물리 검증을
마친 계층을 흔들게 된다.

**보내기와 판정이 갈라져 있다.** `evaluate()` 는 아무것도 보내지 않고 결정만
돌려준다. 그래서 두 팔의 결정을 각각 구한 뒤 두 이동을 겹쳐서 실행할 수
있다. 팔 A 를 수렴시키고 나서 팔 B 를 수렴시키면 주기가 두 배가 된다.
나중에 합치려면 구조를 다시 짜야 하므로 지금 이 모양으로 만든다.

**허용치는 둘이고 소유자가 다르다.** 이 모듈이 쓰는 것은 과제 허용치이며
"이 파지는 성공할 것이다" 에 답한다. `POST_SETTLE_TOLERANCE_RAW = 30` 은
안전 허용치로 "동작이 잘못되지 않았다" 에 답하며 Action 이 소유하고 여기서
건드리지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math


# ---------------------------------------------------------------------------
# 경계값. 전부 실측에서 유도하며 실패할 때마다 키우지 않는다.
# ---------------------------------------------------------------------------

# **과제 허용치.** A4 스윕에서 유도한다. 명령 offset `0.025`(TCP z `0.0313`)
# 에서 펜을 놓쳤고 `0.017`(`0.0233`)에서 잡았다. 따라서 위쪽으로 `8 mm` 는
# 확실히 실패하고 `0 mm` 는 성공한다. 그 사이 어디가 경계인지는 재지 않았다.
# 절반을 취해 `4 mm` 를 작업값으로 쓴다.
#
# 예산이 닫히는지 확인한다. 계획 잔차는 허용 상자를 `0.001` 로 조인 뒤
# 실측 `0.982 mm`(grasp) 였고 이 허용치는 `4 mm` 이므로 합이 `5 mm` 다.
# 확실한 실패점 `8 mm` 아래다.
#
# C2 실기에서 수렴이 실제로 이 안에 들어오는지 확인해야 한다. 못 들어오면
# 이 값을 키우는 것이 아니라 왜 못 들어오는지 잔차로 보고된다.
TASK_TOLERANCE_M = 0.004

# 보정 leg 최대 횟수. 조용히 반복하지 않는다.
MAXIMUM_ITERATIONS = 3

# 보정 leg 하나가 이동해도 되는 최대 TCP 거리. A4.5 의 실측 오차는
# `11.7 mm` 였다. 이보다 훨씬 큰 값이 나왔다면 처짐이 아니라 실행이 크게
# 어긋난 것이고, 충돌 검사를 거치지 않은 짧은 leg 로 처리할 일이 아니다.
MAXIMUM_CORRECTION_M = 0.030

# 한 회차가 이만큼도 줄이지 못하면 정체로 본다. 계획 잔차가 `1 mm` 수준
# 이므로 그보다 작은 개선을 쫓으면 잡음을 쫓는 것이 된다.
PLATEAU_IMPROVEMENT_M = 0.001

# 넘겨명령 상한. 넘겨명령의 크기는 그 회차의 실측 잔차와 같으므로, 지금까지
# 관측된 최대 잔차(`11.7 mm`)보다는 크고, 측정이 잘못됐을 때 팔이 크게
# 튀지는 않는 값이어야 한다.
MAXIMUM_OVERSHOOT_M = 0.015

# 오차가 처음보다 이 배수 이상으로 커지면 발산이다. 더 보내지 않는다.
DIVERGENCE_RATIO = 1.5

# **같은 목표를 다시 보내는 것이 무의미해지는 지점.**
#
# 2026-08-06 C2 실측이 이것을 드러냈다. pregrasp 에서 잔차 12.6 mm 로 보정
# leg 를 한 번 보냈는데, 관절별 명령 델타가 `+4, -4, -18, -9, +8 raw` 였다.
# 저장소가 이미 기록한 `MINIMUM_OBSERVABLE_COMMAND_RAW = 16`
# (`tools/execute_buffered_joint_delta_once.py`) 기준으로 다섯 중 넷이 문턱
# 아래이고, 문턱을 넘긴 ELBOW 18 raw 조차 **0 raw 움직였다.** 실제 이동은
# 최대 6 raw 였고 그것도 중력 방향이었다. 잔차는 12.6 -> 15.7 mm 로 나빠졌다.
#
# 즉 잔차가 관절 공간에서 작을 때 "같은 목표를 다시" 는 문턱 아래 명령이
# 되어 원리적으로 동작할 수 없다. 그 회차를 낭비하면 중력으로 더 처지기까지
# 한다. 이 값 이하이면 보정을 건너뛰고 바로 넘겨명령으로 간다.
#
# 넘겨명령은 델타를 두 배로 만든다 — 위 사례에서 ELBOW 는 36 raw 가 되어
# 문턱을 확실히 넘는다. **넘겨명령은 정체 탈출 수단이 아니라, 문턱을 넘는
# 명령을 만드는 유일한 수단이다.**
#
# 값은 관측된 그대로다. 18 raw 가 움직이지 않았으므로 18 이하를 무효로 본다.
# 추정한 일반 문턱이 아니라 이 자세에서 실제로 재본 값이다.
INEFFECTIVE_CORRECTION_RAW = 18

# **한계에 정확히 걸치는 명령을 만들지 않는다.**
#
# buffered leg 는 위치를 정수 마이크로라디안으로 인코딩한다. 한계에 정확히
# 물린 값은 올림되어 한계를 넘어서고, bridge 의 검증(`JOINT_LIMIT_EPSILON_RAD
# = 1e-9`)은 µrad 양자화(`1e-6`)를 덮지 못해 goal 이 거부된다.
#
# 2026-08-06 에 두 번 겪었다. A4 스윕의 `0.021` 회차가 "IK 해가 WRIST_FLEX 를
# 한계에 정확히 놓아 표현 오차로 거부됨" 으로 건너뛰어졌고, C2 에서 넘겨명령
# 물림이 WRIST_FLEX 를 상한에 정확히 놓아 `+4.07e-7 rad` 로 거부됐다.
#
# bridge 의 epsilon 을 키우는 대신 명령을 만드는 쪽에서 여유를 둔다. 검증된
# 계층을 흔들지 않고, 애초에 걸치는 명령을 내지 않는 것이 옳다.
# `1e-5 rad` 는 µrad 양자화의 10배이고 `0.0065 raw` 라 보정량 손실은 없다.
JOINT_LIMIT_MARGIN_RAD = 1.0e-5

# **넘겨명령이 테이블 쪽으로 내려가지 않게 하는 바닥.**
#
# 중력 처짐은 아래로 생기고 넘겨명령은 `target + (target - measured)` 이므로
# 보통 **위로** 나간다 (A4.5 실측: 측정 z 0.0144, 목표 0.0233 -> 넘겨명령
# 0.0322). 즉 정상 동작에서는 이 검사가 걸리지 않는다.
#
# 그래도 두는 이유는 측정 부호가 뒤집히면 테이블을 향해 밀기 때문이다.
# 그리고 2026-08-06 확인 결과 **MoveIt planning scene 이 비어 있다** —
# 충돌 객체도 octomap 도 없어 자기충돌 외에는 아무것도 막지 못한다.
# 하위 계획기가 막아줄 것이라고 가정할 수 없다.
#
# 값은 `tools/assemble_pick_place_plan_only.py` 의
# `WORKSPACE_TCP_Z_M = (0.02, 0.15)` 하한이다. 저장소가 이미 검증된 작업영역
# 으로 선언한 값이며 새로 지어낸 수가 아니다.
WORKSPACE_TCP_Z_FLOOR_M = 0.02

RAW_PER_TURN = 4096.0


def radians_to_raw_delta(delta_rad: float) -> int:
    """명령 델타를 raw 개수로. 서보 명령은 정수이므로 반올림이 물리적으로 맞다.

    반올림하지 않으면 정확히 18 raw 인 델타가 부동소수로 `18.00000000000001`
    이 되어 `<= 18` 비교를 빠져나간다. 2026-08-06 에 실제로 그렇게 새어나갔다.
    """
    return round(abs(delta_rad) * RAW_PER_TURN / (2.0 * math.pi))

ACCEPT = "accept"
CORRECT = "correct"
OVERSHOOT = "overshoot"
FAIL = "fail"


@dataclass(frozen=True)
class ConvergencePolicy:
    """팔 하나의 수렴 정책. 팔마다 다른 값을 가질 수 있다.

    두 팔의 처짐은 같지 않다. 링크도 부하도 마모도 다르다. 그래서 정책을
    모듈 상수가 아니라 팔별 값으로 들고 다닌다.
    """

    arm: str
    task_tolerance_m: float = TASK_TOLERANCE_M
    maximum_iterations: int = MAXIMUM_ITERATIONS
    maximum_correction_m: float = MAXIMUM_CORRECTION_M
    plateau_improvement_m: float = PLATEAU_IMPROVEMENT_M
    maximum_overshoot_m: float = MAXIMUM_OVERSHOOT_M
    divergence_ratio: float = DIVERGENCE_RATIO
    ineffective_correction_raw: int = INEFFECTIVE_CORRECTION_RAW
    # None 이면 검사하지 않는다. 테이블 근처에서 수렴할 때 호출자가 넣는다.
    minimum_tcp_z_m: float | None = None

    def __post_init__(self) -> None:
        if not self.arm:
            raise ValueError("arm name is required; do not hardcode a side")
        for name, value in (
            ("task_tolerance_m", self.task_tolerance_m),
            ("maximum_correction_m", self.maximum_correction_m),
            ("plateau_improvement_m", self.plateau_improvement_m),
            ("maximum_overshoot_m", self.maximum_overshoot_m),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.maximum_iterations < 1:
            raise ValueError("maximum_iterations must be at least 1")
        if self.divergence_ratio <= 1.0:
            raise ValueError("divergence_ratio must exceed 1.0")
        if self.task_tolerance_m >= self.maximum_correction_m:
            raise ValueError(
                "task_tolerance_m must be tighter than maximum_correction_m; "
                "otherwise every acceptable pose also looks like a gross miss"
            )


@dataclass(frozen=True)
class ConvergenceDecision:
    """한 회차의 판정. 아무것도 보내지 않는다.

    호출자는 여러 팔의 결정을 모아 필요한 이동만 겹쳐서 실행한다.
    """

    arm: str
    action: str
    reason: str
    iteration: int
    error_m: float
    error_vector_m: tuple[float, float, float]
    next_commanded_rad: tuple[float, ...] | None = None
    overshoot_m: float | None = None
    # 한계에 물려 요청한 만큼 넘겨 보내지 못한 관절들. 그 관절의 잔차는
    # 이 자세에서 구조적으로 닫을 수 없다는 뜻이므로 반드시 보고한다.
    clamped_joints: tuple[str, ...] = ()

    @property
    def requires_motion(self) -> bool:
        return self.action in (CORRECT, OVERSHOOT)

    @property
    def converged(self) -> bool:
        return self.action == ACCEPT

    def error_mm(self) -> float:
        return self.error_m * 1000.0


@dataclass(frozen=True)
class ConvergenceState:
    """팔 하나의 수렴 진행 상태. 불변이며 갱신하면 새 값이 나온다.

    불변으로 두는 이유는 두 팔을 동시에 돌릴 때 한쪽의 갱신이 다른 쪽에
    보이지 않아야 하기 때문이다. 상태 공유로 인한 교차 오염을 구조로 막는다.
    """

    policy: ConvergencePolicy
    joint_names: tuple[str, ...]
    # 원래 목표. 보정하는 동안에도 **절대 움직이지 않는다.** 오차는 언제나
    # 이것에 대해 잰다. 명령을 바꿔가며 명령에 대해 재면 목표가 표류한다.
    nominal_rad: tuple[float, ...]
    iteration: int = 0
    overshoot_used: bool = False
    errors_m: tuple[float, ...] = field(default_factory=tuple)
    finished: str | None = None

    def __post_init__(self) -> None:
        if len(self.joint_names) != len(self.nominal_rad):
            raise ValueError(
                "joint_names and nominal_rad must describe the same joints"
            )
        if not self.joint_names:
            raise ValueError("at least one joint is required")


def begin(
    policy: ConvergencePolicy,
    joint_names: tuple[str, ...],
    nominal_rad: tuple[float, ...],
) -> ConvergenceState:
    """이 목표에 대한 수렴을 시작한다."""
    return ConvergenceState(
        policy=policy,
        joint_names=tuple(joint_names),
        nominal_rad=tuple(float(value) for value in nominal_rad),
    )


def _named(joint_names: tuple[str, ...], values) -> dict[str, float]:
    return dict(zip(joint_names, (float(value) for value in values), strict=True))


def _distance(a, b) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b, strict=True)))


def evaluate(
    state: ConvergenceState,
    measured_rad: tuple[float, ...],
    forward_kinematics,
    joint_limits_rad: dict[str, tuple[float, float]] | None = None,
) -> tuple[ConvergenceState, ConvergenceDecision]:
    """측정된 자세를 보고 다음에 무엇을 할지 정한다. 보내지는 않는다.

    `forward_kinematics` 는 `{관절이름: rad} -> (x, y, z)` 인 호출 가능 객체다.
    주입받는 이유는 두 가지다. 실기에서는 MoveIt `/compute_fk` 를 그대로 써서
    계획을 만든 모델과 잔차를 재는 모델이 갈라지지 않게 하고, 시험에서는
    ROS 없이 돌리기 위해서다.

    오차는 **언제나 `nominal_rad` 에 대해** 잰다. 보정으로 명령이 바뀌어도
    목표는 그대로다.
    """
    if state.finished is not None:
        raise ValueError(
            f"{state.policy.arm} convergence already finished: {state.finished}"
        )

    policy = state.policy
    measured = tuple(float(value) for value in measured_rad)
    if len(measured) != len(state.nominal_rad):
        raise ValueError("measured_rad does not match the tracked joints")
    if not all(math.isfinite(value) for value in measured):
        raise ValueError("measured_rad contains a non-finite value")

    goal_tcp = forward_kinematics(_named(state.joint_names, state.nominal_rad))
    measured_tcp = forward_kinematics(_named(state.joint_names, measured))
    error_vector = tuple(
        float(m) - float(g) for m, g in zip(measured_tcp, goal_tcp, strict=True)
    )
    error = math.sqrt(sum(value * value for value in error_vector))

    iteration = state.iteration + 1
    errors = state.errors_m + (error,)
    advanced = replace(state, iteration=iteration, errors_m=errors)

    def settle(action: str, reason: str, **extra) -> tuple:
        finished = None if action in (CORRECT, OVERSHOOT) else action
        decision = ConvergenceDecision(
            arm=policy.arm,
            action=action,
            reason=reason,
            iteration=iteration,
            error_m=error,
            error_vector_m=error_vector,
            **extra,
        )
        return replace(advanced, finished=finished), decision

    # 1. 과제 허용치 안이면 끝이다.
    if error <= policy.task_tolerance_m:
        return settle(
            ACCEPT,
            f"residual {error * 1000.0:.2f} mm is within the task tolerance "
            f"{policy.task_tolerance_m * 1000.0:.2f} mm",
        )

    # 2. 처짐이 아니라 실행이 크게 어긋난 경우. 짧은 보정 leg 로 덮지 않는다.
    if error > policy.maximum_correction_m:
        return settle(
            FAIL,
            f"residual {error * 1000.0:.2f} mm exceeds the bounded correction "
            f"limit {policy.maximum_correction_m * 1000.0:.2f} mm; this is not "
            "sag and must not be closed by an unplanned short leg",
        )

    # 3. 발산. 더 보내면 더 나빠진다.
    if len(errors) >= 2 and error > errors[0] * policy.divergence_ratio:
        return settle(
            FAIL,
            f"residual grew from {errors[0] * 1000.0:.2f} mm to "
            f"{error * 1000.0:.2f} mm; the loop is diverging",
        )

    # 4. 반복 예산 소진.
    if iteration > policy.maximum_iterations:
        return settle(
            FAIL,
            f"residual {error * 1000.0:.2f} mm after "
            f"{policy.maximum_iterations} bounded corrections",
        )

    plateaued = (
        len(errors) >= 2
        and (errors[-2] - errors[-1]) < policy.plateau_improvement_m
    )

    # 보정이 요구하는 최대 관절 델타. 이것이 문턱 아래면 같은 목표를 다시
    # 보내도 서보가 움직이지 않는다. 회차를 낭비할 이유가 없다.
    correction_raw = max(
        radians_to_raw_delta(nominal - value)
        for nominal, value in zip(state.nominal_rad, measured, strict=True)
    )
    below_threshold = correction_raw <= policy.ineffective_correction_raw

    if not plateaued and not below_threshold:
        # 5. 같은 목표로 짧은 보정 leg 를 한 번 더. 명령은 바꾸지 않는다.
        #    이번에는 훨씬 가까운 곳에서 출발하므로 동적 오차가 작아진다.
        return settle(
            CORRECT,
            f"residual {error * 1000.0:.2f} mm; repeating the same goal as a "
            "bounded short leg from the current pose",
            next_commanded_rad=state.nominal_rad,
        )

    # 6. 같은 명령을 또 보내봐야 소용없다 — 정체했거나(반복해도 줄지 않음)
    #    애초에 문턱 아래 명령이거나(서보가 움직이지 않음). 남은 것은 중력 하
    #    정상상태 오차이고 계통적이다. 실측한 잔차만큼 넘겨 보낸다.
    if state.overshoot_used:
        return settle(
            FAIL,
            f"residual {error * 1000.0:.2f} mm plateaued after the measured "
            "overshoot; convergence is not achievable at this pose",
        )

    requested = tuple(
        nominal + (nominal - value)
        for nominal, value in zip(state.nominal_rad, measured, strict=True)
    )

    # **한계에 걸린 관절은 물리고, 나머지는 그대로 보낸다.**
    #
    # 관절 하나가 한계 근처라고 다섯 관절의 보정을 통째로 버리는 것은 과하다.
    # 이 팔은 pregrasp/grasp 자세에서 WRIST_FLEX 가 상시 하한 근처에 선다
    # (2026-08-06: nominal 1197, 하한 1194). A4 의 성공 자세도 하한에서
    # 7 raw 였다. 즉 이것은 예외가 아니라 상시 조건이다.
    #
    # 물리는 것은 명령을 **줄이는** 방향이므로 한계를 넘길 수 없다. 다만
    # 그 관절의 잔차는 이 자세에서 구조적으로 닫히지 않으므로 보고한다.
    overshot: list[float] = []
    clamped: list[str] = []
    for name, value, nominal in zip(
        state.joint_names, requested, state.nominal_rad, strict=True
    ):
        limits = None if joint_limits_rad is None else joint_limits_rad.get(name)
        if limits is None:
            overshot.append(value)
            continue
        lower, upper = limits
        # 한계에 정확히 걸치지 않도록 안쪽으로 여유를 둔다. 구간이 여유의
        # 두 배보다 좁으면 중앙으로 보낸다.
        if upper - lower <= 2.0 * JOINT_LIMIT_MARGIN_RAD:
            inner_lower = inner_upper = 0.5 * (lower + upper)
        else:
            inner_lower = lower + JOINT_LIMIT_MARGIN_RAD
            inner_upper = upper - JOINT_LIMIT_MARGIN_RAD
        bounded = min(max(value, inner_lower), inner_upper)
        if bounded != value:
            clamped.append(name)
        overshot.append(bounded)
    overshot = tuple(overshot)

    # 한계 여유 때문에 물린 관절은 정확히 nominal 이 되지 않는다. 실제로
    # 편향이 남았는지는 서보가 구분할 수 있는 단위, 즉 raw 로 따져야 한다.
    if all(
        radians_to_raw_delta(value - nominal) == 0
        for value, nominal in zip(overshot, state.nominal_rad, strict=True)
    ):
        return settle(
            FAIL,
            "every joint that needs a measured overshoot is already at its "
            "limit; this residual cannot be closed at this pose. clamped="
            + ",".join(clamped),
            clamped_joints=tuple(clamped),
        )

    overshoot_tcp = forward_kinematics(_named(state.joint_names, overshot))
    overshoot_distance = _distance(overshoot_tcp, goal_tcp)

    if (
        policy.minimum_tcp_z_m is not None
        and overshoot_tcp[2] < policy.minimum_tcp_z_m
    ):
        return settle(
            FAIL,
            f"measured overshoot would put the tool at z="
            f"{overshoot_tcp[2] * 1000.0:.2f} mm, below the "
            f"{policy.minimum_tcp_z_m * 1000.0:.2f} mm workspace floor; the "
            "planning scene carries no table so nothing downstream would "
            "refuse this",
            overshoot_m=overshoot_distance,
            clamped_joints=tuple(clamped),
        )

    if overshoot_distance > policy.maximum_overshoot_m:
        return settle(
            FAIL,
            f"measured overshoot would move the tool "
            f"{overshoot_distance * 1000.0:.2f} mm past the goal, above the "
            f"{policy.maximum_overshoot_m * 1000.0:.2f} mm limit",
            overshoot_m=overshoot_distance,
            clamped_joints=tuple(clamped),
        )

    trigger = (
        f"a plain correction would command only {correction_raw} raw, "
        f"at or below the {policy.ineffective_correction_raw} raw that was "
        "measured to move nothing"
        if below_threshold
        else "the residual plateaued"
    )
    state_after, decision = settle(
        OVERSHOOT,
        f"residual {error * 1000.0:.2f} mm and {trigger}; commanding "
        f"{overshoot_distance * 1000.0:.2f} mm past the goal by the measured "
        "residual, once"
        + (f"; clamped at their limits: {','.join(clamped)}" if clamped else ""),
        next_commanded_rad=overshot,
        overshoot_m=overshoot_distance,
        clamped_joints=tuple(clamped),
    )
    return replace(state_after, overshoot_used=True), decision


def arms_requiring_motion(
    decisions: dict[str, ConvergenceDecision],
) -> tuple[str, ...]:
    """이번 회차에 실제로 움직여야 하는 팔들.

    호출자는 이 팔들의 이동을 **동시에** 실행한다. 순차로 돌리면 주기가
    팔 수만큼 배가 된다.
    """
    return tuple(
        arm for arm, decision in decisions.items() if decision.requires_motion
    )


def any_failed(decisions: dict[str, ConvergenceDecision]) -> tuple[str, ...]:
    """수렴에 실패한 팔들.

    한 팔이라도 실패하면 호출자는 조율된 중단을 해야 한다. 팔 A 가 수렴하지
    못했는데 팔 B 가 물체를 들고 있으면 둘 다 멈춰야 한다.
    """
    return tuple(
        arm for arm, decision in decisions.items() if decision.action == FAIL
    )


def summarize(
    policy: ConvergencePolicy,
    decisions: tuple[ConvergenceDecision, ...],
) -> dict[str, object]:
    """증거로 남길 형태. 회차별 잔차를 mm 로 기록한다.

    수렴했는지만이 아니라 **어떻게 줄어들었는지** 를 남긴다. 그 수열이
    C2 에서 자세별 처짐의 실측 근거가 되고, 양팔 예산의 입력이 된다.
    """
    if not decisions:
        raise ValueError("no decisions to summarize")
    final = decisions[-1]
    return {
        "arm": policy.arm,
        "converged": final.converged,
        "final_action": final.action,
        "final_reason": final.reason,
        "iterations": final.iteration,
        "residual_mm_by_iteration": [
            round(decision.error_mm(), 4) for decision in decisions
        ],
        "final_residual_mm": round(final.error_mm(), 4),
        "final_residual_vector_mm": [
            round(value * 1000.0, 4) for value in final.error_vector_m
        ],
        "overshoot_used": any(
            decision.action == OVERSHOOT for decision in decisions
        ),
        "clamped_joints": sorted(
            {name for decision in decisions for name in decision.clamped_joints}
        ),
        "task_tolerance_mm": round(policy.task_tolerance_m * 1000.0, 4),
        "maximum_iterations": policy.maximum_iterations,
        "maximum_correction_mm": round(policy.maximum_correction_m * 1000.0, 4),
        "plateau_improvement_mm": round(
            policy.plateau_improvement_m * 1000.0, 4
        ),
        "maximum_overshoot_mm": round(policy.maximum_overshoot_m * 1000.0, 4),
    }
