from __future__ import annotations

import math
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/single_arm_bridge"))

from single_arm_bridge.joint_unwrap import (  # noqa: E402
    JointUnwrapError,
    JointUnwrapper,
    nearest_unwrapped_raw,
    radians_to_unwrapped_and_modulo_raw,
    signed_modular_delta,
    unwrapped_raw_to_radians,
)


def test_signed_delta_crosses_both_wrap_directions() -> None:
    assert signed_modular_delta(24, 4078) == 42
    assert signed_modular_delta(4078, 24) == -42
    with pytest.raises(JointUnwrapError, match="half-turn"):
        signed_modular_delta(3048, 1000)


def test_binding_uses_only_a_close_verified_reference() -> None:
    assert nearest_unwrapped_raw(24, 4120, 256) == 4120
    with pytest.raises(JointUnwrapError, match="too far"):
        nearest_unwrapped_raw(24, 2048, 1024)
    with pytest.raises(JointUnwrapError, match="ambiguous"):
        nearest_unwrapped_raw(0, 2048, 2047)


def test_observed_left_shoulder_wrap_stays_continuous() -> None:
    unwrap = JointUnwrapper()
    assert unwrap.bind(3919, 3919, 512) == 3919
    assert unwrap.update(4059) == 4059
    assert unwrap.update(65) == 4161
    assert unwrap.update(2572) == 2572


def test_update_failure_preserves_last_good_state() -> None:
    unwrap = JointUnwrapper()
    unwrap.bind(1000, 1000, 64)
    with pytest.raises(JointUnwrapError, match="half-turn"):
        unwrap.update(3048)
    assert unwrap.previous_raw == 1000
    assert unwrap.unwrapped_raw == 1000


def test_command_is_validated_unwrapped_then_converted_modulo() -> None:
    upper_rad = unwrapped_raw_to_radians(
        4188, zero_raw=2048, positive_raw_direction=1
    )
    unwrapped, modulo = radians_to_unwrapped_and_modulo_raw(
        upper_rad,
        zero_raw=2048,
        positive_raw_direction=1,
        minimum_unwrapped_raw=1859,
        maximum_unwrapped_raw=4188,
    )
    assert unwrapped == 4188
    assert modulo == 92
    assert upper_rad == pytest.approx(2140 * 2.0 * math.pi / 4096.0)

    with pytest.raises(JointUnwrapError, match="outside"):
        radians_to_unwrapped_and_modulo_raw(
            upper_rad + 2.0 * math.pi / 4096.0,
            zero_raw=2048,
            positive_raw_direction=1,
            minimum_unwrapped_raw=1859,
            maximum_unwrapped_raw=4188,
        )
