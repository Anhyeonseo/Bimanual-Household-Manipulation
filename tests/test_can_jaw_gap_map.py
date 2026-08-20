"""캔 jaw gap 실측 도구의 순수 계약.

ROS 서비스 호출은 시험하지 않는다. 시험하는 것은 **작업자가 잘못 입력했거나
하드웨어 부호가 반대일 때 도구가 조용히 넘어가지 않는지**다. 이 값들이 그대로
실기 close 명령이 되므로 여기서 막지 못하면 조나 캔이 상한다.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

pytest.importorskip("rclpy")
pytest.importorskip("so101_interfaces.srv")
pytest.importorskip("single_arm_bridge.bimanual_stream_adapter")

from tools.setup.can_perception.commission_can_jaw_gap_map_once import (  # noqa: E402
    approved_gripper_raw_bounds,
    interpolate_raw_for_gap,
    parse_raw_steps,
    prepared_positions,
    semantic_rad_to_raw,
    semantic_raw_to_rad,
)

LIMITS = ROOT / "config/bimanual_operational_limits.json"


def _sample(raw: int, gap_mm: float) -> dict:
    return {"measured_raw": raw, "measured_gap_mm": gap_mm}


def test_semantic_mapping_round_trips():
    for raw in (1872, 1948, 2048, 2500, 3257):
        assert semantic_rad_to_raw(semantic_raw_to_rad(raw)) == raw


def test_opening_is_the_negative_rad_direction():
    assert semantic_raw_to_rad(2500) < semantic_raw_to_rad(2048)
    assert semantic_raw_to_rad(1948) > semantic_raw_to_rad(2048)


def test_approved_bounds_match_the_envelope_file():
    document = json.loads(LIMITS.read_text(encoding="utf-8"))
    for side in ("left", "right"):
        gripper = document["arms"][side]["gripper"]
        assert approved_gripper_raw_bounds(side) == (
            gripper["minimum_unwrapped_raw"],
            gripper["maximum_unwrapped_raw"],
        )


def test_left_envelope_clears_a_53mm_can():
    minimum, maximum = approved_gripper_raw_bounds("left")
    assert minimum < 2048 < maximum
    assert maximum - 2048 > 1000


def test_parse_raw_steps_requires_at_least_two_points():
    assert parse_raw_steps("2048,2200") == (2048, 2200)
    with pytest.raises(ValueError):
        parse_raw_steps("2048")


def test_interpolation_brackets_the_target_without_extrapolating():
    samples = [_sample(2048, 10.0), _sample(2200, 30.0), _sample(2350, 50.0)]
    solved = interpolate_raw_for_gap(samples, 44.0)
    assert solved is not None
    assert solved["extrapolated"] is False
    assert 2200 < solved["raw"] < 2350
    assert solved["bracketed_by_raw"] == [2200, 2350]


def test_interpolation_refuses_targets_outside_the_measured_span():
    """실측 밖을 외삽하지 않는다.

    61 mm 개방이 필요한데 50 mm 까지만 쟀다면, 도구는 값을 지어내지 않고
    None 을 돌려주어 계획이 거부되게 한다.
    """
    samples = [_sample(2048, 10.0), _sample(2200, 30.0), _sample(2350, 50.0)]
    assert interpolate_raw_for_gap(samples, 61.0) is None
    assert interpolate_raw_for_gap(samples, 5.0) is None


def test_prepared_positions_rejects_a_stale_epoch():
    document = {
        "prepared_epoch": 3,
        "torque_hold_active": True,
        "prepared_positions_rad": [0.0] * 12,
    }
    with pytest.raises(RuntimeError, match="prepared epoch mismatch"):
        prepared_positions(
            document, label="x", expected_epoch=4, require_torque_hold=True
        )


def test_prepared_positions_requires_proof_of_torque_hold():
    document = {
        "prepared_epoch": 4,
        "torque_hold_active": False,
        "prepared_positions_rad": [0.0] * 12,
    }
    with pytest.raises(RuntimeError, match="torque hold"):
        prepared_positions(
            document, label="x", expected_epoch=4, require_torque_hold=True
        )
    assert prepared_positions(
        document, label="x", expected_epoch=4, require_torque_hold=False
    ) == tuple([0.0] * 12)


def test_prepared_positions_rejects_an_incomplete_anchor():
    document = {
        "prepared_epoch": 1,
        "torque_hold_active": True,
        "prepared_positions_rad": [0.0] * 11,
    }
    with pytest.raises(RuntimeError, match="complete prepared anchor"):
        prepared_positions(
            document, label="x", expected_epoch=1, require_torque_hold=True
        )
