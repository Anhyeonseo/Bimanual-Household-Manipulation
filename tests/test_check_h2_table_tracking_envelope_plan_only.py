from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "check_h2_table_tracking_envelope_plan_only",
    ROOT / "tools" / "check_h2_table_tracking_envelope_plan_only.py",
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_error_vertices_cover_each_sign_combination() -> None:
    vertices = MODULE.error_vertices([1, 2, 0, 4, 5, 6])
    assert len(vertices) == 32
    assert all(vertex[2] == 0.0 for vertex in vertices)
    assert min(vertex[0] for vertex in vertices) < 0.0
    assert max(vertex[0] for vertex in vertices) > 0.0


def test_clamp_variant_stays_inside_calibrated_limits() -> None:
    result = MODULE.clamp_variant(
        (0.0, 0.0), (-2.0, 2.0), ((-1.0, 1.0), (-0.5, 0.5))
    )
    assert result == (-1.0, 0.5)


def test_phase_envelopes_do_not_mix_unrelated_legs() -> None:
    legs = []
    for index, tag in enumerate(MODULE.PHASE_TO_H2_LEG.values(), start=1):
        legs.append({"tag": tag, "ok": True, "h2_tracking_error_max_raw": [index] * 6})
    values = MODULE.load_phase_envelopes({"legs": legs})
    assert values["pick_pregrasp_to_grasp"] == [2] * 6
    assert values["lift_to_place_pregrasp"] == [4] * 6


def test_retained_waypoints_rejects_old_phase_artifact(tmp_path: Path) -> None:
    phase = tmp_path / "phase.json"
    phase.write_text(json.dumps({"segments": [{"index": 1}]}), encoding="utf-8")
    digest = MODULE.sha256_file(phase)
    manifest = {
        "phase_summaries": [{"source": str(phase), "source_sha256": digest, "reversed": False}],
        "steps": [
            {"kind": "arm", "index": 1, "phase": "x", "source": str(phase), "source_sha256": digest, "source_segment_index": 1},
            {"kind": "gripper", "phase": "place_release", "target_position_rad": 0.06},
        ],
    }
    with pytest.raises(ValueError, match="before exact MoveIt waypoints"):
        MODULE.retained_waypoints(manifest, ROOT)


def test_source_has_no_action_client() -> None:
    source = (ROOT / "tools" / "check_h2_table_tracking_envelope_plan_only.py").read_text(encoding="utf-8")
    assert "ActionClient" not in source
    assert '"motion_authorized": False' in source
    assert '"temporary_scene_cleanup_succeeded"' in source
