from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "attempt_pick_grasp_once", ROOT / "tools" / "attempt_pick_grasp_once.py"
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def arguments(*, confirmation: str) -> list[str]:
    return [
        "attempt_pick_grasp_once.py",
        "--pick-plan", "candidate.json",
        "--grasp-offset-m", "0.011",
        "--open-position-check",
        "--confirmation", confirmation,
        "--output", "result.json",
        "--workdir", "work",
    ]


def test_open_position_check_requires_its_own_confirmation(monkeypatch) -> None:
    monkeypatch.setattr(
        sys, "argv", arguments(confirmation=MODULE.OPEN_POSITION_CHECK_CONFIRMATION)
    )
    parsed = MODULE.parse_args()
    assert parsed.open_position_check is True
    assert MODULE.confirmation_for(parsed) == MODULE.OPEN_POSITION_CHECK_CONFIRMATION


def test_open_position_check_rejects_the_grasp_confirmation(monkeypatch) -> None:
    monkeypatch.setattr(sys, "argv", arguments(confirmation=MODULE.CONFIRMATION))
    with pytest.raises(SystemExit):
        MODULE.parse_args()


def test_open_position_check_rejects_an_unbounded_hold(monkeypatch) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        arguments(confirmation=MODULE.OPEN_POSITION_CHECK_CONFIRMATION)
        + ["--hold-at-grasp-s", "10.1"],
    )
    with pytest.raises(SystemExit):
        MODULE.parse_args()


def test_held_position_check_requires_its_own_confirmation(monkeypatch) -> None:
    command = arguments(confirmation=MODULE.HELD_OBJECT_POSITION_CHECK_CONFIRMATION)
    command.remove("--open-position-check")
    monkeypatch.setattr(
        sys,
        "argv",
        command + ["--held-object-position-check"],
    )
    parsed = MODULE.parse_args()
    assert parsed.held_object_position_check is True
    assert (
        MODULE.confirmation_for(parsed)
        == MODULE.HELD_OBJECT_POSITION_CHECK_CONFIRMATION
    )


def test_open_position_check_documents_no_gripper_or_convergence() -> None:
    source = (ROOT / "tools" / "attempt_pick_grasp_once.py").read_text(
        encoding="utf-8"
    )
    start = source.rindex("if position_check:")
    block = source[start:source.index("else:", start)]
    assert '"convergence_executed": False' in source
    assert '"gripper_command_sent": False' in source
    assert "converge(work" not in block
    assert "gripper(" not in block
