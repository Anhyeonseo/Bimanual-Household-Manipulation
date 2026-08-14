from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools/check_j1_left_nominal_route_coverage_plan_only.py"
SPEC = importlib.util.spec_from_file_location("check_j1_routes", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
SOURCE = TOOL_PATH.read_text(encoding="utf-8")
CANDIDATE_PATH = (
    ROOT
    / "artifacts/joint_ranges/2026-08-13/"
    "j1_operational_limits_candidate_inset64.json"
)
MANIFEST_PATH = (
    ROOT / "artifacts/h2/2026-08-12/offset011_nominal_routes/manifest.json"
)


def load(path: Path, label: str) -> dict:
    return MODULE.load_bound(path, MODULE.file_sha256(path), label)


def test_all_saved_nominal_trajectory_points_fit_candidate() -> None:
    report = MODULE.check_routes(
        ROOT,
        load(CANDIDATE_PATH, "candidate"),
        load(MANIFEST_PATH, "manifest"),
    )
    assert report["status"] == MODULE.STATUS
    assert report["motion_authorized"] is False
    assert report["runtime_change_authorized"] is False
    assert report["execution_api_used"] is False
    assert report["left_route_coverage"] is True
    assert report["right_route_coverage"] is False
    assert report["trajectory_point_count"] > 1000
    assert len(report["phases"]) == 7
    assert all(
        joint["minimum_limit_clearance_rad"] > 0.0
        for joint in report["joints"].values()
    )


def test_out_of_candidate_trajectory_fails_closed(tmp_path: Path) -> None:
    candidate = load(CANDIDATE_PATH, "candidate")
    manifest = load(MANIFEST_PATH, "manifest")
    source = ROOT / manifest["phase_summaries"][0]["source"]
    phase = json.loads(source.read_text(encoding="utf-8"))
    phase["segments"][0]["trajectory_positions_rad"][0][0] = 9.0
    changed = tmp_path / "phase.json"
    changed.write_text(json.dumps(phase), encoding="utf-8")
    manifest["phase_summaries"][0]["source"] = str(changed)
    manifest["phase_summaries"][0]["source_sha256"] = MODULE.file_sha256(
        changed
    )
    with pytest.raises(ValueError, match="outside J1 candidate"):
        MODULE.check_routes(ROOT, candidate, manifest)


def test_sha_and_motion_contract_fail_closed(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="SHA mismatch"):
        MODULE.load_bound(CANDIDATE_PATH, "0" * 64, "candidate")
    document = load(MANIFEST_PATH, "manifest")
    document["motion_authorized"] = True
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="motion_authorized=false"):
        MODULE.load_bound(path, MODULE.file_sha256(path), "manifest")


def test_tool_cannot_execute_or_apply_candidate() -> None:
    assert "--plan-only is required" in SOURCE
    assert '"motion_authorized": False' in SOURCE
    assert '"runtime_change_authorized": False' in SOURCE
    for forbidden in (
        "rclpy",
        "serial",
        "send_goal_async",
        ".arm_and_enable(",
        "openocd",
        "Servo_",
        "RightServoBus_",
    ):
        assert forbidden not in SOURCE
