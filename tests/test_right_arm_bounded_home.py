"""R3 bounded home composes only the already-reviewed right-arm primitives."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools/execute_right_arm_bounded_home_once.py"
SPEC = spec_from_file_location("right_arm_bounded_home", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
SOURCE = TOOL_PATH.read_text(encoding="utf-8")
BRIDGE = (
    ROOT
    / "ros2_ws/src/single_arm_bridge/single_arm_bridge/bridge_node.py"
).read_text(encoding="utf-8")


@pytest.mark.parametrize(
    ("position", "target", "expected"),
    (
        (2048, 2048, 0),
        (2041, 2048, 0),
        (2040, 2048, 0),
        (2037, 2048, 11),
        (2028, 2048, 20),
        (2027, 2048, 13),
        (2021, 2048, 19),
        (1726, 2048, 20),
        (2358, 2048, -20),
        (2060, 2048, -12),
    ),
)
def test_bounded_step_moves_toward_q0_without_illegal_tail(
    position: int,
    target: int,
    expected: int,
) -> None:
    step = MODULE.bounded_step_raw(position, target)
    assert step == expected
    if step:
        assert 8 <= abs(step) <= 20
        assert abs(position + step - target) < abs(position - target)


def test_bounded_step_rejects_impossible_tolerance() -> None:
    with pytest.raises(ValueError):
        MODULE.bounded_step_raw(2047, 2048, tolerance_raw=0)


def test_home_requires_exact_confirmation_and_verified_q0() -> None:
    assert 'CONFIRMATION = "RIGHT_ARM_BOUNDED_HOME_ONCE"' in SOURCE
    assert "raw_after_power_cycle" in SOURCE
    assert "target_raw" in SOURCE
    assert "settle_tolerance_raw" in SOURCE
    assert "general_trajectory_authorized" in SOURCE


def test_home_reuses_existing_bounded_services_and_has_stop_path() -> None:
    for service in (
        "/right_arm_configure_once",
        "/get_right_arm_configuration",
        "/right_arm_torque_enable_once",
        "/right_arm_jog_once",
        "/right_arm_disable",
        "/right_arm_stop",
    ):
        assert service in SOURCE
    assert "MAXIMUM_STEP_RAW = 20" in SOURCE
    assert "STALL_CYCLE_LIMIT = 5" in SOURCE
    assert "call_stop(repr(error))" in SOURCE
    assert "R3_PREFLIGHT_VERIFIED_DISABLE_PASS torque_mask=0x00" in SOURCE
    assert "R3_VERIFIED_DISABLE_PASS torque_mask=0x00" in SOURCE
    assert '"right_arm_stop"' in BRIDGE
    stop_handler = BRIDGE[
        BRIDGE.index("def _on_right_arm_stop"):
        BRIDGE.index("def _on_clear_fault")
    ]
    assert "self._transport.safe_stop()" in stop_handler
    assert "self._right_arm_output_active = False" in stop_handler


def test_success_path_uses_non_latching_verified_disable() -> None:
    assert '"right_arm_disable"' in BRIDGE
    disable_handler = BRIDGE[
        BRIDGE.index("def _on_right_arm_disable"):
        BRIDGE.index("def _on_right_arm_stop")
    ]
    assert "self._transport.disable_right_arm_verified()" in disable_handler
    assert "snapshot.torque_enabled_mask != 0" in disable_handler
    assert "snapshot.failure_count != 0" in disable_handler
    assert "self._right_arm_output_active = False" in disable_handler
    assert "self._transport.safe_stop()" not in disable_handler
    assert "verified_disable_succeeded" in SOURCE
    assert 'document["torque_enabled_at_completion"] = False' in SOURCE
