from __future__ import annotations

import ast
import importlib.util
import sys
from pathlib import Path


MODULE_PATH = Path("tools/stm32_raw_range_observer.py")
SPEC = importlib.util.spec_from_file_location("stm32_raw_range_observer", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_accumulator_records_raw_extrema_without_expanding_limits() -> None:
    accumulator = MODULE.RawRangeAccumulator()
    accumulator.update((2048, 2050, 2040, 2060, 2030, 2070))
    accumulator.update((2055, 2045, 2042, 2050, 2035, 2065))

    document = accumulator.document(1.0, 0x20700, 0x4D62F8D5)

    assert document["motion_authorized"] is False
    assert document["apply_to_calibration"] is False
    assert document["automatic_limit_expansion"] is False
    assert document["observed_joints"][0]["observed_minimum_raw"] == 2048
    assert document["observed_joints"][0]["observed_maximum_raw"] == 2055


def test_accumulator_rejects_invalid_joint_count_and_raw_value() -> None:
    accumulator = MODULE.RawRangeAccumulator()
    try:
        accumulator.update((2048,))
    except ValueError as error:
        assert "six" in str(error)
    else:
        raise AssertionError("invalid joint count was accepted")

    try:
        accumulator.update((2048, 2048, 2048, 2048, 2048, 4096))
    except ValueError as error:
        assert "0..4095" in str(error)
    else:
        raise AssertionError("invalid raw position was accepted")


def test_tool_has_no_motion_or_fault_clear_calls() -> None:
    tree = ast.parse(MODULE_PATH.read_text(encoding="utf-8"))
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert "disable" in called_attributes
    assert "arm_and_enable" not in called_attributes
    assert "send_setpoint" not in called_attributes
    assert "clear_fault" not in called_attributes
    assert "safe_stop" not in called_attributes
