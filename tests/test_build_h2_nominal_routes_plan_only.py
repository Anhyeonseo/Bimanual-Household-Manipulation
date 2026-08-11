from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "build_h2_nominal_routes_plan_only",
    ROOT / "tools" / "build_h2_nominal_routes_plan_only.py",
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def write_endpoint(path: Path, offset: float = 0.0) -> str:
    document = {
        "status": "PLAN_ONLY_PASS",
        "execution_api_used": False,
        "motion_authorized": False,
        "plans": [
            {
                "name": "pregrasp", "success": True, "moveit_error_code": 1,
                "final_joint_positions_rad": [offset + 0.1] * 5,
                "target_m": [0.4, -0.1, 0.1], "yaw_rad": 0.0,
            },
            {
                "name": "grasp", "success": True, "moveit_error_code": 1,
                "final_joint_positions_rad": [offset + 0.2] * 5,
                "target_m": [0.4, -0.1, 0.02], "yaw_rad": 0.0,
            },
        ],
    }
    path.write_text(json.dumps(document), encoding="utf-8")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_phase_chain_uses_exact_preceding_planned_endpoints(tmp_path: Path) -> None:
    paths = [tmp_path / f"p{index}.json" for index in range(3)]
    plans = []
    for index, path in enumerate(paths):
        plans.append(MODULE.load_pinned_endpoint(path, write_endpoint(path, index)))
    phases = MODULE.build_phase_specs(*plans)
    assert phases[0].start == MODULE.Q0
    assert phases[1].start == plans[0].targets["pregrasp"]
    assert phases[2].start == plans[0].targets["grasp"]
    assert phases[3].start == plans[1].targets["grasp"]
    assert phases[4].start == plans[2].targets["pregrasp"]
    assert phases[5].start == plans[2].targets["grasp"]
    assert phases[6].start == MODULE.Q0


def test_endpoint_sha_is_fail_closed(tmp_path: Path) -> None:
    path = tmp_path / "endpoint.json"
    write_endpoint(path)
    with pytest.raises(ValueError, match="sha256 mismatch"):
        MODULE.load_pinned_endpoint(path, "0" * 64)


def test_builder_source_contains_no_execution_client() -> None:
    source = (ROOT / "tools" / "build_h2_nominal_routes_plan_only.py").read_text(
        encoding="utf-8"
    )
    assert "ActionClient" not in source
    assert '"motion_authorized": False' in source
    assert '"tracking_envelope_collision_checked": False' in source
    assert '"tracking_envelope_route_matches_inputs": envelope_route_matches' in source
    assert '"environment_collision_geometry_verified": False' in source
    assert '"--max-joint-step", str(STAGE7_MAX_JOINT_STEP_RAD)' in source
