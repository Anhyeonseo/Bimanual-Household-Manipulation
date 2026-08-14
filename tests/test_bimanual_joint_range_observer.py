from __future__ import annotations

import ast
import importlib.util
import math
from pathlib import Path
import sys


MODULE_PATH = Path("tools/observe_bimanual_joint_range_torque_off.py")
SPEC = importlib.util.spec_from_file_location(
    "observe_bimanual_joint_range_torque_off", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_raw_round_trip_and_range_capture() -> None:
    calibration = MODULE.JointCalibration("left_base_joint", 2048, 1)
    position = (2055 - 2048) * 2.0 * math.pi / 4096
    assert MODULE.radians_to_raw(position, calibration) == 2055

    capture = MODULE.RangeCapture(selected_index=0)
    capture.update((2048,) * 12)
    capture.update((2060,) + (2048,) * 11)
    capture.update((2040,) + (2048,) * 11)
    minimum, maximum, spans = capture.summary()
    assert minimum[0] == 2040
    assert maximum[0] == 2060
    assert spans[0] == 20
    assert capture.selected_direction_reversals == 1


def test_selected_joint_unwraps_across_encoder_zero() -> None:
    assert MODULE.signed_circular_delta(8, 4090) == 14
    assert MODULE.signed_circular_delta(4090, 8) == -14

    capture = MODULE.RangeCapture(selected_index=0)
    for raw in (4080, 4090, 4, 16, 4, 4090, 4080):
        capture.update((raw,) + (2048,) * 11)
    assert capture.selected_wrap_crossings == 2
    assert capture.selected_direction_reversals == 1
    assert capture.selected_unwrapped_minimum_raw == 4080
    assert capture.selected_unwrapped_maximum_raw == 4112
    assert capture.selected_maximum_step_raw == 12


def test_tool_is_subscribe_only_and_evidence_cannot_authorize_motion() -> None:
    source = MODULE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert "create_subscription" in called_attributes
    for forbidden in (
        "arm_and_enable",
        "enable",
        "disable",
        "send_setpoint",
        "safe_stop",
        "clear_fault",
    ):
        assert forbidden not in called_attributes
    assert '"motion_authorized": False' in source
    assert '"apply_to_calibration": False' in source
    assert '"automatic_limit_expansion": False' in source
    assert "maximum_same_arm_other_joint_span_raw" in source
    assert "maximum_opposite_arm_joint_span_raw" in source
    assert "selected_unwrapped_minimum_raw" in source
    assert "selected_wrap_crossings" in source
