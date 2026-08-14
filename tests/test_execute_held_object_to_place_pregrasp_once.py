from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "execute_held_object_to_place_pregrasp_once",
    ROOT / "tools" / "execute_held_object_to_place_pregrasp_once.py",
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def arguments(*, confirmation: str) -> list[str]:
    return [
        "execute_held_object_to_place_pregrasp_once.py",
        "--place-plan", "place.json",
        "--confirmation", confirmation,
        "--output", "result.json",
        "--workdir", "work",
    ]


def test_transfer_requires_its_explicit_confirmation(monkeypatch) -> None:
    monkeypatch.setattr(sys, "argv", arguments(confirmation=MODULE.CONFIRMATION))
    assert MODULE.parse_args().tracking_rate_raw_s == 200.0


def test_transfer_rejects_a_wrong_confirmation(monkeypatch) -> None:
    monkeypatch.setattr(sys, "argv", arguments(confirmation="wrong"))
    with pytest.raises(SystemExit):
        MODULE.parse_args()


def test_transfer_has_no_gripper_or_convergence_route() -> None:
    source = (ROOT / "tools" / "execute_held_object_to_place_pregrasp_once.py").read_text(
        encoding="utf-8"
    )
    assert "GRIPPER_COMMANDS=false convergence=false release=false" in source
    assert "gripper(" not in source
    assert "converge(" not in source
