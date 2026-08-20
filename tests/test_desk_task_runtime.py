from __future__ import annotations

import math

import pytest

from desk_task_runtime import (
    BaseTargetSample,
    DeskTaskContractError,
    bimanual_q0_target,
    lock_target,
    step_target,
    validate_bimanual_q0,
)


def samples() -> list[BaseTargetSample]:
    return [
        BaseTargetSample(
            x_m=0.35 + offset,
            y_m=-0.12,
            z_m=0.0063,
            yaw_rad=math.pi / 2.0 - offset,
            confidence=0.95,
        )
        for offset in (-0.0004, -0.0002, 0.0, 0.0002, 0.0004)
    ]


def test_target_lock_keeps_an_undirected_axis_stable() -> None:
    locked = lock_target(samples())
    assert locked.sample_count == 5
    assert locked.x_m == pytest.approx(0.35)
    assert locked.maximum_position_spread_m == pytest.approx(0.0004)
    assert abs(abs(locked.yaw_rad) - math.pi / 2.0) < 0.001


def test_target_lock_rejects_too_few_samples() -> None:
    with pytest.raises(DeskTaskContractError, match="at least five"):
        lock_target(samples()[:4])


def test_bimanual_q0_preserves_both_grippers() -> None:
    current = (0.01, -0.01, 0.02, 0.0, 0.01, 0.4,
               -0.01, 0.01, -0.02, 0.0, -0.01, 0.5)
    assert bimanual_q0_target(current) == (
        0.0, 0.0, 0.0, 0.0, 0.0, 0.4,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.5,
    )
    assert validate_bimanual_q0(bimanual_q0_target(current)) == 0.0


def test_step_target_holds_the_opposite_arm() -> None:
    current = (0.0,) * 12
    opposite_hold = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    target = step_target(
        current,
        {"kind": "arm", "target_positions_rad": [1, 2, 3, 4, 5]},
        opposite_hold,
        arm="left",
    )
    assert target[:5] == (1.0, 2.0, 3.0, 4.0, 5.0)
    assert target[5] == 0.0
    assert target[6:] == opposite_hold
