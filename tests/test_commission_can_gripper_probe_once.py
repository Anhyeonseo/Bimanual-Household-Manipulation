from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))
PATH = TOOLS / "commission_can_gripper_probe_once.py"
SPEC = importlib.util.spec_from_file_location("commission_can_gripper_probe_once", PATH)
PROBE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = PROBE
SPEC.loader.exec_module(PROBE)


@pytest.mark.parametrize("raw", [1948, 1960, 1980, 2000, 2009])
def test_semantic_gripper_conversion_round_trips(raw: int) -> None:
    assert PROBE.semantic_rad_to_raw(PROBE.semantic_raw_to_rad(raw)) == raw


@pytest.mark.parametrize(
    ("side", "changed"),
    [
        ("left", {5}),
        ("right", {11}),
        ("both", {5, 11}),
    ],
)
def test_target_positions_change_only_selected_grippers(
    side: str,
    changed: set[int],
) -> None:
    current = tuple(index / 100.0 for index in range(12))
    target = PROBE.target_positions(current, side, 2009)
    for index, (before, after) in enumerate(zip(current, target, strict=True)):
        if index in changed:
            assert math.isclose(after, PROBE.semantic_raw_to_rad(2009))
        else:
            assert after == before


def test_target_positions_rejects_unbounded_probe() -> None:
    with pytest.raises(ValueError, match="bounded probe"):
        PROBE.target_positions((0.0,) * 12, "left", 1947)


def test_probe_interpolation_densely_fills_initial_lead_window() -> None:
    start = (0.0,) * 12
    target = PROBE.target_positions(start, "both", 2009)
    points = PROBE.interpolate_probe(start, target, 1500)
    assert len(points) == 31
    assert points[0][0] == 80
    assert points[1][0] == 130
    assert points[-1][0] == 1580
    assert sum(offset <= 400 for offset, _ in points) >= 2
    assert points[-1][1] == target


def test_probe_contract_is_one_shot_and_fail_closed() -> None:
    source = PATH.read_text(encoding="utf-8")
    assert "START_FINITE" in source
    assert "automatic_retry_count" in source
    assert "ARM_MOTION_LIMIT_RAD" in source
    assert "BimanualStreamCommand.Request.STOP" in source
    assert "serial.Serial" not in source
    assert "/dev/tty" not in source
