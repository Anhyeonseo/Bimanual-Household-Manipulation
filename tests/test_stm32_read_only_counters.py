from __future__ import annotations

import ast
import importlib.util
import sys
from pathlib import Path


MODULE_PATH = Path("tools/diagnostics/stm32_read_only_counters.py")
SPEC = importlib.util.spec_from_file_location("stm32_read_only_counters", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_f8_ascii_scan_has_no_motion_or_fault_clear_calls() -> None:
    tree = ast.parse(MODULE_PATH.read_text(encoding="utf-8"))
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert "disable" not in called_attributes
    assert "arm_and_enable" not in called_attributes
    assert "send_setpoint" not in called_attributes
    assert "clear_fault" not in called_attributes
    assert "safe_stop" not in called_attributes


def test_parser_accepts_f8_host_baud(monkeypatch) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        ["stm32_read_only_counters.py", "--baud", "921600"],
    )
    assert MODULE.parse_args().baud == 921600


def test_ascii_scan_reports_failed_axis(capsys) -> None:
    class FakePort:
        def __init__(self) -> None:
            self.lines = iter([
                b"ALL_AXIS_STATUS\r\n",
                b"AXIS ID=1 NAME=BASE POS=2048\r\n",
                b"AXIS ID=2 NAME=SHOULDER READ_FAIL\r\n",
                b"AXIS ID=3 NAME=ELBOW POS=2048\r\n",
                b"AXIS ID=4 NAME=WRIST_FLEX POS=2048\r\n",
                b"AXIS ID=5 NAME=WRIST_ROLL POS=2048\r\n",
                b"AXIS ID=6 NAME=GRIPPER POS=2048\r\n",
                b"ALL_AXIS_STATUS_END\r\n",
            ])

        def reset_input_buffer(self) -> None:
            pass

        def write(self, payload: bytes) -> int:
            assert payload == b"S"
            return 1

        def flush(self) -> None:
            pass

        def readline(self) -> bytes:
            return next(self.lines, b"")

    assert not MODULE.run_f8_ascii_servo_scan(FakePort())
    output = capsys.readouterr().out
    assert "AXIS ID=2 NAME=SHOULDER READ_FAIL" in output
    assert "axes=6" in output
    assert "servo_write_commands=0" in output
