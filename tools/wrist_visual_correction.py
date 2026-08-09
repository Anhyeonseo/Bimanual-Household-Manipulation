#!/usr/bin/env python3
"""손목 카메라가 본 물체 위치로 **목표를** 고치는 판정 계층. 경계되고 fail-closed.

**층을 섞지 않는다.** 이 모듈과 `grasp_convergence.py` 는 다른 질문에 답한다.

    grasp_convergence   자기수용 — "명령한 자세에 도달했는가"
    wrist_visual_correction  외부수용 — "그 자세가 올바른 목표인가"

`grasp_convergence` 는 `state.nominal_rad` 를 절대 움직이지 않는다(그 파일
`:230-232`). 즉 "목표가 틀렸다" 는 입력이 없다. 그것이 이 모듈의 일이며,
결과는 **새 목표**이므로 반드시 재계획으로 들어간다. 수렴 루프에 끼워넣지
않는다. 두 층을 섞으면 실패 원인을 가를 수 없다.

**이 모듈은 계획하지 않고 움직이지도 않는다.** 순수 판정 함수다. 측정된
관절값과 손목 카메라가 본 물체를 받아 "목표를 이만큼 고쳐라, 또는 고치지
말라" 를 답한다. 실제 이동은 호출자가 `ros_moveit_plan_grasp.py` 재호출로
한다.

**Z 를 보정하지 않는다.** 2026-08-09 W3 세션 실측이 이유다. 고정된 표적을
10개 자세에서 보고 base 좌표를 역산했을 때 축별 흩어짐이

    X  std 3.06 mm   max|dev| 6.41 mm
    Y  std 2.35 mm   max|dev| 5.95 mm
    Z  std 5.65 mm   max|dev| 9.13 mm     <- 지배적

였다. 평면 표적을 단안 카메라로 보면 깊이 관측성이 약하다는 것이 그대로
나온 것이고, 전체 잔차 `6.81 mm` 는 대부분 Z 다. XY 만 보면
`평균 3.37 mm / 최대 6.42 mm` 로 쓸 만하다.

그리고 Z 는 애초에 카메라로 복원할 필요가 없다. 작업대 높이와 물체 모델에서
이미 안다 — `CAMERA_COMPUTE_ARCHITECTURE.md:44` 가 상단 카메라에 대해 같은
규칙을 이미 선언했고(`초기 Z 좌표는 작업대와 물체 모델에서 이미 알고 있는
높이로 제한한다`), `top_shadow_target.yaml` 의
`object.center_height_above_board_m` 이 그 구현이다. 손목도 같이 간다.

**보정의 하한은 서보 문턱이 아니라 보정 자신의 불확실성이다.**
`grasp_convergence.INEFFECTIVE_CORRECTION_RAW = 18` 은 "정착한 자세에서 같은
목표를 다시 명령" 하는 경우의 문턱이다. 여기는 다르다 — 새 목표로 재계획하면
팔은 실제로 다른 자세로 이동하므로 그 문턱에 걸리지 않는다. 대신 **보정량이
보정 자신의 오차보다 작으면 잡음을 쫓는 것**이 된다. 그 값이 위의
`XY 평균 3.37 mm` 이며, 아래 `MINIMUM_CORRECTION_M` 이 거기서 나온다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


# ---------------------------------------------------------------------------
# 경계값. 전부 실측에서 유도하며 실패할 때마다 키우지 않는다.
# ---------------------------------------------------------------------------

# 보정 한 번이 목표를 옮겨도 되는 최대 거리.
# `grasp_convergence.MAXIMUM_CORRECTION_M` 과 같은 값을 쓴다. 그 파일이
# "이보다 훨씬 큰 값이 나왔다면 처짐이 아니라 실행이 크게 어긋난 것" 이라고
# 유도했고, 여기서도 논리가 같다 — 이보다 큰 시각 보정은 물체가 그만큼
# 움직인 것이 아니라 **검출이나 보정이 틀린 것**이다.
MAXIMUM_CORRECTION_M = 0.030

# **보정이 잡음과 구별되는 하한. 오탐률을 실측해서 정했다.**
#
# 처음에 W3 의 XY **평균** 오차 `3.37 mm` 에서 `4 mm` 로 잡았는데 **틀렸다.**
# 움직이지 않은 표적을 10개 자세에서 보고 이 판정기를 그대로 돌려보니
# `4 mm` 하한에서 **10회 중 4회가 `apply`** 였다. 표적은 가만히 있었으므로
# 그 4회는 전부 헛보정이다. 평균이 아니라 **최악값**으로 잡아야 한다.
#
# 관측된 헛보정 크기(mm), 정렬:
#     0.95  1.55  1.66  1.75  2.08  3.00  4.67  5.64  5.99  6.42
# 하한별 오탐:  4mm -> 4/10,  5mm -> 3/10,  6mm -> 1/10,  7mm -> 0/10
#
# `7 mm` 에서 오탐이 사라지고 `8 mm` 를 여유로 쓴다.
#
# **이 값이 뜻하는 한계를 분명히 한다.** 보정 대역이 `[8, 30] mm` 이므로 이
# 모듈은 **큰 오차를 잡는 장치이고 미세 조정 장치가 아니다.** C2 실측에서
# 파지는 잔차 `10.17 mm` 에서 성립했으므로, 정상 회차의 파지를 이 모듈이
# 더 좋게 만들지는 못한다. 잡을 수 있는 것은 "상단 카메라가 말한 위치와
# 물체가 실제로 있는 위치가 유의하게 다르다" 는 경우다.
#
# `CAMERA_COMPUTE_ARCHITECTURE.md:51` 이 적은 "마지막 수 cm 구간에서 중심과
# yaw 오차 보정" 은 **절대 Cartesian 보정으로는 이 정확도에서 달성되지
# 않는다.** 그 문서가 병기한 다른 선택지 — 영상 기반 Visual Servo — 가
# 필요하다. 이유는 오차의 성질에 있다:
#
#   - 이 흩어짐은 자세마다 **체계적**이고 무작위가 아니다. 각 capture 가
#     이미 20 frame 을 median 으로 묶은 값이라, 같은 자세에서 frame 을 더
#     모아도 줄지 않는다.
#   - 그리고 이 오차에는 카메라 보정 오차만 있는 것이 아니라 **팔 자신의 FK
#     오차(처짐·백래시)가 섞여 있다.** C2 는 SHOULDER 처짐만 `7.42 mm` 로
#     실측했고 그것은 여기 `6.42 mm` 와 같은 크기다.
#   - 처짐이 섞이는 경로가 중요하다. 카메라는 그리퍼에 붙어 있으므로
#     **그리퍼 기준 상대 오차는 처짐과 무관하게 옳다.** 처짐은 base 좌표로
#     올렸다 내리는 과정에서만 들어온다. 즉 보정을 base 절대 좌표가 아니라
#     그리퍼 상대량으로 표현하면 처짐이 상쇄된다.
MINIMUM_CORRECTION_M = 0.008

# frame 이 이보다 오래됐으면 거부한다. `top_perception.yaml` 의
# `max_frame_age_s` 와 같은 값이다. 손목은 팔과 함께 움직이므로 오래된
# frame 은 상단보다 더 위험하다 — 그 사이 카메라 자세가 바뀌었다.
MAXIMUM_FRAME_AGE_S = 0.2

# 검출 신뢰도 하한. `top_perception.yaml` 의 `minimum_confidence` 와 같다.
MINIMUM_CONFIDENCE = 0.7

APPLY = "apply"
HOLD = "hold"
REJECT = "reject"


@dataclass(frozen=True)
class WristCorrectionPolicy:
    """팔 하나의 시각 보정 정책. 팔마다 다른 값을 가질 수 있다.

    두 팔의 카메라는 같은 모델이어도 같은 보정값이 아니다. 그래서 정책을
    모듈 상수가 아니라 팔별 값으로 들고 다닌다 —
    `grasp_convergence.ConvergencePolicy` 와 같은 이유다.
    """

    arm: str
    maximum_correction_m: float = MAXIMUM_CORRECTION_M
    minimum_correction_m: float = MINIMUM_CORRECTION_M
    maximum_frame_age_s: float = MAXIMUM_FRAME_AGE_S
    minimum_confidence: float = MINIMUM_CONFIDENCE
    # None 이면 검사하지 않는다. 호출자가 검증된 작업영역을 넣는다.
    workspace_x_m: tuple[float, float] | None = None
    workspace_y_m: tuple[float, float] | None = None

    def __post_init__(self) -> None:
        if not self.arm:
            raise ValueError("arm name is required; do not hardcode a side")
        for name, value in (
            ("maximum_correction_m", self.maximum_correction_m),
            ("minimum_correction_m", self.minimum_correction_m),
            ("maximum_frame_age_s", self.maximum_frame_age_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0.0 < self.minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be within (0, 1]")
        if self.minimum_correction_m >= self.maximum_correction_m:
            raise ValueError(
                "minimum_correction_m must be tighter than "
                "maximum_correction_m; otherwise no correction is ever both "
                "large enough to trust and small enough to accept"
            )
        for name, bounds in (
            ("workspace_x_m", self.workspace_x_m),
            ("workspace_y_m", self.workspace_y_m),
        ):
            if bounds is None:
                continue
            lower, upper = bounds
            if not (math.isfinite(lower) and math.isfinite(upper)):
                raise ValueError(f"{name} bounds must be finite")
            if lower >= upper:
                raise ValueError(f"{name} lower bound must be below upper")


@dataclass(frozen=True)
class WristObservation:
    """손목 카메라가 본 물체 하나. 검출기가 무엇이든 이 형태로 들어온다.

    `camera_to_object_xyz_m` 은 카메라 optical frame 기준 물체 위치다.
    `yaw_rad` 는 물체 장축 방위각이며 `None` 이면 yaw 보정을 요구하지 않는다.
    `detection_count` 는 그 frame 에서 검출된 물체 수 — 1 이 아니면 거부한다.
    """

    camera_to_object_xyz_m: tuple[float, float, float]
    frame_age_s: float
    confidence: float
    detection_count: int = 1
    yaw_rad: float | None = None


@dataclass(frozen=True)
class WristCorrectionDecision:
    """한 번의 판정. 아무것도 보내지 않는다."""

    arm: str
    action: str
    reason: str
    # 손목이 본 물체의 base 좌표. Z 도 함께 보고하지만 **보정에는 쓰지
    # 않는다** — 진단용이며 신뢰도가 XY 보다 낮다(docstring 참고).
    observed_base_xyz_m: tuple[float, float, float] | None = None
    nominal_base_xy_m: tuple[float, float] | None = None
    corrected_base_xy_m: tuple[float, float] | None = None
    correction_xy_m: tuple[float, float] | None = None
    correction_magnitude_m: float | None = None
    observed_yaw_rad: float | None = None

    @property
    def requires_replan(self) -> bool:
        return self.action == APPLY

    def correction_mm(self) -> float | None:
        if self.correction_magnitude_m is None:
            return None
        return self.correction_magnitude_m * 1000.0


def _matrix_multiply(a, b):
    return [
        [sum(a[row][k] * b[k][column] for k in range(4)) for column in range(4)]
        for row in range(4)
    ]


def _apply_transform(matrix, point) -> tuple[float, float, float]:
    homogeneous = (*point, 1.0)
    result = [
        sum(matrix[row][column] * homogeneous[column] for column in range(4))
        for row in range(3)
    ]
    return (result[0], result[1], result[2])


def evaluate(
    policy: WristCorrectionPolicy,
    observation: WristObservation,
    base_to_gripper,
    gripper_to_camera,
    nominal_base_xy_m: tuple[float, float],
) -> WristCorrectionDecision:
    """손목이 본 물체로 목표를 고칠지 정한다. 보내지는 않는다.

    `base_to_gripper` 는 측정된 관절값에서 구한 `4x4` 이며 호출자가 넣는다.
    주입받는 이유는 `grasp_convergence.evaluate` 와 같다 — 실기에서는 계획을
    만든 모델과 같은 FK 를 쓰고, 시험에서는 ROS 없이 돌리기 위해서다.

    `gripper_to_camera` 는 W3 eye-in-hand 산출물의 `4x4` 다.

    오차는 `nominal_base_xy_m` 에 대해 재며, 그것은 지금 계획에 들어간 목표다.
    """

    def reject(reason: str) -> WristCorrectionDecision:
        return WristCorrectionDecision(
            arm=policy.arm, action=REJECT, reason=reason
        )

    # 1. 검출 자체를 믿을 수 있는지부터. 못 믿으면 좌표를 계산하지 않는다.
    if observation.detection_count != 1:
        return reject(
            f"detection_count={observation.detection_count}; exactly one "
            "object is required"
        )
    if not math.isfinite(observation.frame_age_s) or observation.frame_age_s < 0.0:
        return reject("frame_age_s must be finite and non-negative")
    if observation.frame_age_s > policy.maximum_frame_age_s:
        return reject(
            f"frame is {observation.frame_age_s * 1000.0:.1f} ms old; limit is "
            f"{policy.maximum_frame_age_s * 1000.0:.1f} ms"
        )
    if not math.isfinite(observation.confidence):
        return reject("confidence must be finite")
    if observation.confidence < policy.minimum_confidence:
        return reject(
            f"confidence {observation.confidence:.3f} is below "
            f"{policy.minimum_confidence:.3f}"
        )
    if not all(
        math.isfinite(value) for value in observation.camera_to_object_xyz_m
    ):
        return reject("camera_to_object_xyz_m contains a non-finite value")

    # 2. base 좌표로 옮긴다. TF 가 매 순간 카메라 자세를 계산하는 것과 같은
    #    합성이며, 여기서는 호출자가 준 두 변환을 그대로 곱한다.
    base_to_camera = _matrix_multiply(base_to_gripper, gripper_to_camera)
    observed = _apply_transform(base_to_camera, observation.camera_to_object_xyz_m)
    if not all(math.isfinite(value) for value in observed):
        return reject("observed base position is not finite")

    correction = (
        observed[0] - float(nominal_base_xy_m[0]),
        observed[1] - float(nominal_base_xy_m[1]),
    )
    magnitude = math.sqrt(correction[0] ** 2 + correction[1] ** 2)
    corrected = (
        float(nominal_base_xy_m[0]) + correction[0],
        float(nominal_base_xy_m[1]) + correction[1],
    )

    def decide(action: str, reason: str) -> WristCorrectionDecision:
        return WristCorrectionDecision(
            arm=policy.arm,
            action=action,
            reason=reason,
            observed_base_xyz_m=observed,
            nominal_base_xy_m=(
                float(nominal_base_xy_m[0]),
                float(nominal_base_xy_m[1]),
            ),
            corrected_base_xy_m=corrected,
            correction_xy_m=correction,
            correction_magnitude_m=magnitude,
            observed_yaw_rad=observation.yaw_rad,
        )

    # 3. 너무 크면 물체가 옮겨진 것이 아니라 검출이나 보정이 틀린 것이다.
    if magnitude > policy.maximum_correction_m:
        return decide(
            REJECT,
            f"correction {magnitude * 1000.0:.2f} mm exceeds "
            f"{policy.maximum_correction_m * 1000.0:.2f} mm; treat this as a "
            "detection or calibration fault, not a moved object",
        )

    # 4. 보정 결과가 검증된 작업영역 밖이면 거부한다. MoveIt planning scene
    #    이 비어 있어 하위 계획기가 막아주지 않는다
    #    (`grasp_convergence.py` 의 `WORKSPACE_TCP_Z_FLOOR_M` 주석 참고).
    for axis, value, bounds in (
        ("x", corrected[0], policy.workspace_x_m),
        ("y", corrected[1], policy.workspace_y_m),
    ):
        if bounds is None:
            continue
        lower, upper = bounds
        if not lower <= value <= upper:
            return decide(
                REJECT,
                f"corrected {axis}={value:.4f} m is outside the validated "
                f"workspace [{lower:.4f}, {upper:.4f}]",
            )

    # 5. 잡음보다 작으면 고치지 않는다. 조용히 넘기지 않고 그렇게 보고한다.
    if magnitude < policy.minimum_correction_m:
        return decide(
            HOLD,
            f"correction {magnitude * 1000.0:.2f} mm is below the "
            f"{policy.minimum_correction_m * 1000.0:.2f} mm floor derived from "
            "this calibration's own XY uncertainty; correcting would chase "
            "noise",
        )

    return decide(
        APPLY,
        f"correction {magnitude * 1000.0:.2f} mm is trustworthy and bounded; "
        "re-plan to the corrected target",
    )
