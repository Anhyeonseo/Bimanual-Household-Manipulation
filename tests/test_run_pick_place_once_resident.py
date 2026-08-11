"""H1 gate history provenance checks. ROS나 하드웨어 없이 검증한다."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import run_pick_place_once_resident as RUNNER  # noqa: E402


PLAN_HASHES = {"pick": "a", "pick_lift": "b", "place": "c"}
CALIBRATION_HASHES = {"planner": "d", "bridge": "e"}
PARAMETERS = {
    "minimum_tcp_z_m": None,
    "tracking_rate_raw_s": 300.0,
    "q0_swing_tracking_rate_raw_s": 250.0,
}


def timed_leg(tag: str, *, fresh: int, prime: int, duration: int) -> dict:
    return {
        "tag": tag,
        "ok": True,
        "fresh_tick_ms": fresh,
        "prime_tick_ms": prime,
        "first_sample_lead_ms": 100,
        "duration_ms": duration,
    }


def write_evidence(path: Path, **overrides: object) -> None:
    document = {
        "schema_version": 1,
        "status": f"{RUNNER.STATUS}_COMPLETE",
        "operator_confirmation": RUNNER.CONFIRMATION,
        "arm": "left",
        "plan_sha256": PLAN_HASHES,
        "calibration_sha256": CALIBRATION_HASHES,
        "execution_parameters": PARAMETERS,
        "h1_gate": {"metric": RUNNER.H1_GATE_METRIC},
        "legs": [
            timed_leg("pick_pregrasp", fresh=1000, prime=1100, duration=1000),
            timed_leg("pick_grasp", fresh=2400, prime=2500, duration=500),
        ],
    }
    document.update(overrides)
    path.write_text(json.dumps(document), encoding="utf-8")


def load(paths: list[Path]) -> tuple[list[int], list[dict]]:
    return RUNNER.load_h1_gate_history(
        paths,
        expected_arm="left",
        expected_plan_sha256=PLAN_HASHES,
        expected_calibration_sha256=CALIBRATION_HASHES,
        expected_execution_parameters=PARAMETERS,
    )


def test_history_accepts_one_local_transition_and_records_provenance(
    tmp_path: Path,
) -> None:
    evidence = tmp_path / "run.json"
    write_evidence(evidence)

    values, sources = load([evidence])

    assert values == [200]
    assert len(sources) == 1
    assert sources[0]["sample_ms"] == 200
    assert len(sources[0]["sha256"]) == 64


def test_history_rejects_the_same_artifact_twice(tmp_path: Path) -> None:
    evidence = tmp_path / "run.json"
    write_evidence(evidence)

    with pytest.raises(ValueError, match="중복"):
        load([evidence, evidence])


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("status", f"{RUNNER.STATUS}_STOPPED"),
        ("arm", "right"),
        ("plan_sha256", {"pick": "different"}),
        ("calibration_sha256", {"planner": "different"}),
        ("execution_parameters", {"tracking_rate_raw_s": 50.0}),
        ("h1_gate", {"metric": "wall_clock_interval"}),
    ],
)
def test_history_rejects_incompatible_runs(
    tmp_path: Path, field: str, value: object
) -> None:
    evidence = tmp_path / f"{field}.json"
    write_evidence(evidence, **{field: value})

    with pytest.raises(ValueError, match="조건이 현재 실행과 다르다"):
        load([evidence])
