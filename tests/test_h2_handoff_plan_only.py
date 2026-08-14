from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "build_h2_handoff_plan_only", ROOT / "tools/build_h2_handoff_plan_only.py"
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def write_leg(path: Path, start: list[float], end: list[float]) -> str:
    path.write_text(json.dumps({
        "status": "PREGRASP_SEGMENT_PLAN_ONLY_PASS",
        "execution_api_used": False,
        "motion_authorized": False,
        "segments": [{
            "success": True,
            "moveit_error_code": 1,
            "expected_start_positions_rad": start,
            "target_positions_rad": end,
        }],
    }), encoding="utf-8")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pinned(tmp_path: Path, label: str, start: list[float], end: list[float]):
    path = tmp_path / f"{label}.json"
    digest = write_leg(path, start, end)
    return MODULE.load_pinned_leg(label, path, digest)


def complete_envelope() -> dict:
    return {"maximum_error_raw": [1, 2, 3, 4, 5, 0], "valid_leg_count": 2}


def test_h2_handoff_refuses_to_interpolate_an_unchecked_gap(tmp_path: Path) -> None:
    first = pinned(tmp_path, "first", [0.0] * 5, [1.0] * 5)
    second = pinned(tmp_path, "second", [1.0, 1.0, 1.02, 1.0, 1.0], [2.0] * 5)
    report = MODULE.build_report([first, second], complete_envelope())
    assert report["status"] == MODULE.STATUS_REJECTED
    assert report["execution_api_used"] is False
    assert report["motion_authorized"] is False
    assert report["collision_checked"] is False
    assert report["handoffs"][0]["maximum_gap_rad"] == pytest.approx(0.02)
    assert "unchecked gap" in report["rejection_reasons"][0]


def test_h2_handoff_stays_plan_only_even_when_endpoints_match(tmp_path: Path) -> None:
    first = pinned(tmp_path, "first", [0.0] * 5, [1.0] * 5)
    second = pinned(tmp_path, "second", [1.0] * 5, [2.0] * 5)
    report = MODULE.build_report([first, second], complete_envelope())
    assert report["status"] == MODULE.STATUS_READY
    assert report["rejection_reasons"] == []
    assert report["next_required_step"] == "fresh collision check with tracking-expanded geometry"
    assert report["motion_authorized"] is False


def test_h2_handoff_reports_missing_envelope_as_a_blocker(tmp_path: Path) -> None:
    first = pinned(tmp_path, "first", [0.0] * 5, [1.0] * 5)
    second = pinned(tmp_path, "second", [1.0] * 5, [2.0] * 5)
    report = MODULE.build_report([first, second], None)
    assert report["status"] == MODULE.STATUS_REJECTED
    assert report["rejection_reasons"] == [
        "complete H2 tracking envelope evidence is missing"
    ]


@pytest.mark.parametrize("value", ["first", "first=path", "=path@" + "a" * 64])
def test_pinned_argument_requires_label_path_and_digest(value: str) -> None:
    with pytest.raises(MODULE.argparse.ArgumentTypeError):
        MODULE.parse_pinned_argument(value)
