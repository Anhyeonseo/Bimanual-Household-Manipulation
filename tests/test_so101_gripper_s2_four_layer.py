from hashlib import sha256
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
S2 = ROOT / "config/so101_gripper_s2_four_layer.candidate.json"


def test_s2_uses_measured_four_layer_anchor_without_interpolation():
    document = json.loads(S2.read_text(encoding="utf-8"))
    base = ROOT / document["base_gripper_geometry"]["path"]
    assert sha256(base.read_bytes()).hexdigest() == document[
        "base_gripper_geometry"
    ]["sha256"]
    assert document["record_kind"] == "so101_gripper_s2_four_layer_candidate"
    assert document["simulation_only"] is True
    assert document["motion_authorized"] is False
    assert document["control_contract"]["command_is_force_claim"] is False
    assert document["control_contract"][
        "retention_only_after_four_layer_contact_gate"
    ] is True
    assert document["control_contract"][
        "collision_validation_uses_exact_four_layer_target"
    ] is True

    base_document = json.loads(base.read_text(encoding="utf-8"))
    model_q0 = base_document["geometry"][
        "detailed_stl_model_q_at_physical_q0_rad"
    ]
    for side in ("left", "right"):
        measured = base_document["grasp_commands"][side]["four_layer"][
            "operational_candidate_rad"
        ]
        assert base_document["grasp_commands"][side]["four_layer"][
            "operational_candidate_revalidated"
        ] is True
        project = document["four_layer_project_contact_target_rad"][side]
        model = document["four_layer_model_contact_target_rad"][side]
        assert project == pytest.approx(measured)
        assert model == pytest.approx(model_q0 - project)
