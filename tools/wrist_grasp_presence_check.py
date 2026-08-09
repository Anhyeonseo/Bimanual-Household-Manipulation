#!/usr/bin/env python3
"""손목 카메라로 "물체를 실제로 잡았는가"를 판정하는 순수 판정 계층. 경계되고 fail-closed.

이 모듈은 위치를 고치지 않는다 — 그건 `wrist_visual_correction.py` 의 일이다.
여기는 다른 질문에 답한다: "그리퍼 사이에 물체가 있는가", "들어올린 뒤에도
그대로인가". `CAMERA_COMPUTE_ARCHITECTURE.md` 가 유효 역할로 남긴 두 항목이
이거다 — 2026-08-09 대화 시점까지 아무것도 구현돼 있지 않았다.

**두 시점에서만 쓴다.**

    PRE_CLOSE   그리퍼를 닫기 직전 — "닫을 가치가 있는가"
    POST_LIFT   들어올린 뒤 — "여전히 물고 있는가"

**카메라 단독으로 판정하지 않는다.** 그리퍼 잔여 간격(residual gap raw)이
이미 펜에는 신뢰할 만한 신호다(`execute_gripper_command_once.py`: 물체
없음 close 잔여 5 raw, 펜 close 잔여 23 raw, 문턱 14 raw). 카메라는 그
신호를 대체하지 않고 **독립적인 두 번째 확인**으로 더한다 — `fuse_with_gap_check`
가 그 결합이다. `grasp_convergence` 의 "층을 섞지 않는다" 규율을 여기서도
지킨다: 이 모듈은 그리퍼를 열거나 닫지 않고, 재시도도 하지 않는다. 판정만
돌려준다.

**입력은 raw 이미지가 아니라 이미 계산된 점유 점수(occupancy score)다.**
카메라가 그리퍼에 강체로 고정돼 있으므로 손가락 사이 영역은 항상 이미지의
같은 픽셀에 있다 — 물체를 "찾을" 필요가 없다(`PLAN_TOWEL_FOLDING_PERCEPTION.md`
5.1절과 같은 논리). 고정 ROI 안에서 빈 상태 참조 프레임과의 차이(픽셀
분산, edge 밀도 등 무엇이든)를 호출자가 계산해 `[0, 1]` 점수 하나로 넘긴다.
그 계산법 자체는 이 모듈의 관심사가 아니다 — 여기는 그 점수를 놓고 판정만
한다.

**임계값에 기본값이 없다.** `ros_moveit_plan_grasp.py` 의 `--grasp-offset`
가 조용한 기본값(`0.025`) 때문에 8월 6~7일 재측정값이 반영 안 됐던 것과
같은 실수를 여기서 반복하지 않는다. `minimum_occupancy_score` 는 실제
빈 손/파지 캡처로만 정해질 수 있는 값이라, 캘리브레이션 전에는 이 모듈을
아예 부를 수 없게 필수 인자로 막는다(`GraspCheckPolicy.__post_init__`).
캘리브레이션 절차는 W4 가 `MINIMUM_CORRECTION_M` 을 잡을 때와 같다 —
평균이 아니라 오탐률로 잡을 것. 고정 표적(빈 손) 여러 자세를 찍어 이
판정기에 그대로 돌려보고, 오탐이 사라지는 값을 문턱으로 쓴다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


PRE_CLOSE = "pre_close"
POST_LIFT = "post_lift"
CHECKPOINTS = (PRE_CLOSE, POST_LIFT)

PRESENT = "present"
ABSENT = "absent"
REJECT = "reject"

# frame 이 이보다 오래됐으면 거부한다. `wrist_visual_correction.MAXIMUM_FRAME_AGE_S`
# 와 같은 값 — 손목은 팔과 함께 움직이므로 오래된 frame 이 상단보다 더 위험하다.
MAXIMUM_FRAME_AGE_S = 0.2

# 검출 신뢰도 하한. `wrist_visual_correction.MINIMUM_CONFIDENCE` 와 같다.
MINIMUM_CONFIDENCE = 0.7


@dataclass(frozen=True)
class GraspCheckPolicy:
    """팔 하나·checkpoint 하나의 파지 판정 정책.

    `minimum_occupancy_score` 에 기본값을 두지 않는다 — 위 모듈 docstring
    참고. 호출자가 오늘 캘리브레이션한 값을 명시적으로 넘겨야 한다.
    """

    arm: str
    minimum_occupancy_score: float
    maximum_frame_age_s: float = MAXIMUM_FRAME_AGE_S
    minimum_confidence: float = MINIMUM_CONFIDENCE

    def __post_init__(self) -> None:
        if not self.arm:
            raise ValueError("arm name is required; do not hardcode a side")
        if not math.isfinite(self.minimum_occupancy_score):
            raise ValueError("minimum_occupancy_score must be finite")
        if not 0.0 < self.minimum_occupancy_score < 1.0:
            raise ValueError(
                "minimum_occupancy_score must be within (0, 1); a value at "
                "or beyond either bound means the score never discriminates"
            )
        if not math.isfinite(self.maximum_frame_age_s) or self.maximum_frame_age_s <= 0.0:
            raise ValueError("maximum_frame_age_s must be finite and positive")
        if not 0.0 < self.minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be within (0, 1]")


@dataclass(frozen=True)
class GraspCheckObservation:
    """손목 카메라 fixed ROI 에서 계산된 점유 신호 하나.

    `occupancy_score` 는 호출자가 이미 계산해 넘긴 `[0, 1]` 스칼라다.
    `detection_count` 는 ROI 안에서 검출된 별개 물체 수 — `wrist_visual_correction`
    과 같은 이유로 1이 아니면 신호를 못 믿는다(둘이면 그림자·반사 오검출,
    0이면 애초에 점유 점수를 낼 수 없는 상태).
    """

    occupancy_score: float
    frame_age_s: float
    confidence: float
    detection_count: int = 1


@dataclass(frozen=True)
class GraspCheckDecision:
    """한 번의 판정. 그리퍼를 열거나 닫지 않는다."""

    arm: str
    checkpoint: str
    action: str
    reason: str
    occupancy_score: float | None = None

    @property
    def confirmed_present(self) -> bool:
        return self.action == PRESENT

    @property
    def trustworthy(self) -> bool:
        return self.action != REJECT


def evaluate(
    policy: GraspCheckPolicy,
    observation: GraspCheckObservation,
    checkpoint: str,
) -> GraspCheckDecision:
    """checkpoint 에서 물체가 있다고 볼지 정한다. fail-closed."""

    if checkpoint not in CHECKPOINTS:
        raise ValueError(
            f"checkpoint must be one of {CHECKPOINTS!r}, got {checkpoint!r}"
        )

    def reject(reason: str) -> GraspCheckDecision:
        return GraspCheckDecision(
            arm=policy.arm, checkpoint=checkpoint, action=REJECT, reason=reason
        )

    # 1. 신호 자체를 믿을 수 있는지부터. 못 믿으면 점수를 판정에 안 쓴다.
    if observation.detection_count != 1:
        return reject(
            f"detection_count={observation.detection_count}; exactly one "
            "ROI reading is required"
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
    if not math.isfinite(observation.occupancy_score):
        return reject("occupancy_score must be finite")
    if not 0.0 <= observation.occupancy_score <= 1.0:
        return reject(
            f"occupancy_score {observation.occupancy_score:.3f} is outside "
            "[0, 1]"
        )

    # 2. 문턱과 비교한다.
    if observation.occupancy_score >= policy.minimum_occupancy_score:
        return GraspCheckDecision(
            arm=policy.arm,
            checkpoint=checkpoint,
            action=PRESENT,
            reason=(
                f"occupancy {observation.occupancy_score:.3f} meets "
                f"{policy.minimum_occupancy_score:.3f}"
            ),
            occupancy_score=observation.occupancy_score,
        )
    return GraspCheckDecision(
        arm=policy.arm,
        checkpoint=checkpoint,
        action=ABSENT,
        reason=(
            f"occupancy {observation.occupancy_score:.3f} is below "
            f"{policy.minimum_occupancy_score:.3f}"
        ),
        occupancy_score=observation.occupancy_score,
    )


PROCEED = "proceed"
STOP_EMPTY = "stop_empty"
STOP_CONTRADICTION = "stop_contradiction"
DEGRADED_GAP_ONLY = "degraded_gap_only"


@dataclass(frozen=True)
class FusedGraspDecision:
    """카메라 판정과 그리퍼 잔여 간격 판정을 합친 최종 결과."""

    action: str
    reason: str
    camera_decision: GraspCheckDecision
    gap_confirmed: bool


def fuse_with_gap_check(
    camera_decision: GraspCheckDecision,
    gap_confirmed: bool,
) -> FusedGraspDecision:
    """카메라 신호와 그리퍼 잔여 간격 신호를 합친다.

    두 신호가 일치하면 그대로 따르고, 어긋나면 **어느 쪽도 믿지 않고
    멈춘다** — `PLAN_TOWEL_FOLDING_PERCEPTION.md` 4절의 R4("이상 감지 →
    정지·보고")와 같은 규율이다. 카메라 신호를 못 믿는 상태(REJECT)면
    간격 판정 단독으로 내려가되 degraded 로 표시해 조용히 넘어가지 않는다.
    """

    def fused(action: str, reason: str) -> FusedGraspDecision:
        return FusedGraspDecision(
            action=action,
            reason=reason,
            camera_decision=camera_decision,
            gap_confirmed=gap_confirmed,
        )

    if not camera_decision.trustworthy:
        return fused(
            DEGRADED_GAP_ONLY,
            f"camera signal rejected ({camera_decision.reason}); falling "
            f"back to gripper residual gap alone (confirmed={gap_confirmed})",
        )

    camera_present = camera_decision.confirmed_present
    if camera_present and gap_confirmed:
        return fused(PROCEED, "camera and gripper residual gap agree: present")
    if not camera_present and not gap_confirmed:
        return fused(
            STOP_EMPTY, "camera and gripper residual gap agree: absent"
        )
    return fused(
        STOP_CONTRADICTION,
        f"camera says {'present' if camera_present else 'absent'} but "
        f"gripper residual gap says {'present' if gap_confirmed else 'absent'}; "
        "treat as a fault, do not guess",
    )
