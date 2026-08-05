"""shadow target 을 파지 좌표로 승격하는 게이트의 계약.

`ShadowObjectTarget` 은 첫 줄에 "Never consume this as a motion goal" 이라고
적혀 있고 발행자는 언제나 `robot_target_available=false` 로 낸다. 그 잠금을
대신할 것이 이 도구의 게이트다. 여기서 무엇이 거부되는지가 곧 인식 기반
파지의 안전 논리다.

ROS 없이 검증한다 — `evaluate()` 는 순수 함수다.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

_spec = importlib.util.spec_from_file_location(
    "capture_top_shadow_target_once",
    ROOT / "tools" / "capture_top_shadow_target_once.py",
)
MODULE = importlib.util.module_from_spec(_spec)
sys.modules["capture_top_shadow_target_once"] = MODULE
_spec.loader.exec_module(MODULE)


def arguments(**overrides) -> argparse.Namespace:
    values = {
        "topic": MODULE.DEFAULT_TOPIC,
        "samples": 5,
        "timeout_s": 20.0,
        "maximum_spread_m": MODULE.DEFAULT_MAXIMUM_SPREAD_M,
        "maximum_spread_yaw_rad": MODULE.DEFAULT_MAXIMUM_SPREAD_YAW_RAD,
        "maximum_frame_age_s": MODULE.DEFAULT_MAXIMUM_FRAME_AGE_S,
        "minimum_confidence": MODULE.DEFAULT_MINIMUM_CONFIDENCE,
        "output": Path("/dev/null"),
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def sample(**overrides) -> dict:
    values = {
        "x_m": 0.368269,
        "y_m": -0.129557,
        "z_m": 0.0063,
        "yaw_rad": -0.020156,
        "source_frame_age_s": 0.12,
        "confidence": 0.94,
        "status": "OK",
        "shadow_pose_available": True,
        "transform_validated": True,
        "inside_workspace": True,
        "fresh": True,
        "source_footprint_inside": True,
        "source_image_fully_visible": True,
        "motion_authorized": False,
        "robot_target_available": False,
    }
    values.update(overrides)
    return values


def settled(count: int = 5, **overrides) -> list[dict]:
    return [sample(**overrides) for _ in range(count)]


def test_a_settled_detection_is_promoted() -> None:
    document = MODULE.evaluate(settled(), arguments())
    assert document["status"] == MODULE.STATUS
    assert document["promoted_to_grasp_input"] is True
    assert document["publisher_claimed_authority"] is False
    assert document["motion_authorized"] is False
    assert document["execution_api_used"] is False
    assert document["target"]["x_m"] == pytest.approx(0.368269)
    assert document["target"]["yaw_rad"] == pytest.approx(-0.020156)


def test_promotion_never_authorizes_motion_by_itself() -> None:
    """좌표를 승격해도 동작 권한은 생기지 않는다."""
    document = MODULE.evaluate(settled(), arguments())
    assert document["motion_authorized"] is False
    assert "operator approves" in document["promotion_gate"]
    source = (
        ROOT / "tools" / "capture_top_shadow_target_once.py"
    ).read_text(encoding="utf-8")
    # 동작 API 를 쓰지 않는다.
    for forbidden in ("ActionClient", "send_goal", "FollowJointTrajectory"):
        assert forbidden not in source


@pytest.mark.parametrize("field", MODULE.REQUIRED_TRUE)
def test_every_publisher_gate_must_hold(field) -> None:
    with pytest.raises(ValueError, match=f"{field}=false"):
        MODULE.evaluate(settled(**{field: False}), arguments())


@pytest.mark.parametrize("field", MODULE.REQUIRED_FALSE)
def test_a_publisher_claiming_authority_is_refused(field) -> None:
    """인식 노드가 스스로 권한을 주장하기 시작하면 그건 계약 위반이다."""
    with pytest.raises(ValueError, match="never claim motion authority"):
        MODULE.evaluate(settled(**{field: True}), arguments())


def test_a_stale_frame_is_refused() -> None:
    stale = settled()
    stale[2]["source_frame_age_s"] = 0.9
    with pytest.raises(ValueError, match="frame age"):
        MODULE.evaluate(stale, arguments())


def test_a_low_confidence_detection_is_refused() -> None:
    weak = settled()
    weak[1]["confidence"] = 0.2
    with pytest.raises(ValueError, match="confidence"):
        MODULE.evaluate(weak, arguments())


def test_a_flickering_position_is_refused() -> None:
    """한 프레임의 검출은 깜빡일 수 있다. 표본이 일치해야 한다."""
    jitter = settled()
    jitter[3]["x_m"] += MODULE.DEFAULT_MAXIMUM_SPREAD_M * 2
    with pytest.raises(ValueError, match="position is not settled"):
        MODULE.evaluate(jitter, arguments())


def test_a_flickering_yaw_is_refused() -> None:
    jitter = settled()
    jitter[3]["yaw_rad"] += math.radians(5.0)
    with pytest.raises(ValueError, match="yaw is not settled"):
        MODULE.evaluate(jitter, arguments())


def test_too_few_samples_is_refused() -> None:
    with pytest.raises(ValueError, match="shadow samples arrived"):
        MODULE.evaluate(settled(2), arguments())


def test_a_non_finite_coordinate_is_refused() -> None:
    broken = settled()
    broken[0]["y_m"] = float("nan")
    with pytest.raises(ValueError, match="not finite"):
        MODULE.evaluate(broken, arguments())


def test_spread_limits_are_far_inside_the_measured_perception_error() -> None:
    """VIS-001 실측은 위치 10 mm, yaw 5도다.

    표본 간 흔들림 한계는 그보다 훨씬 작아야 한다. 인식 정확도만큼 흔들리는
    것을 '안정' 이라고 부르면 게이트가 아무것도 걸러내지 못한다.
    """
    assert MODULE.DEFAULT_MAXIMUM_SPREAD_M < 0.010
    assert MODULE.DEFAULT_MAXIMUM_SPREAD_YAW_RAD < math.radians(5.0)
