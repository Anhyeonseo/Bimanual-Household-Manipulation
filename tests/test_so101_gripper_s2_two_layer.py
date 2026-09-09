from hashlib import sha256
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
S2 = ROOT / "config/so101_gripper_s2_two_layer.candidate.json"


def test_s2_two_layer_target_is_derived_without_mutating_s1_provenance():
    document = json.loads(S2.read_text(encoding="utf-8"))
    base = ROOT / document["base_gripper_geometry"]["path"]
    assert sha256(base.read_bytes()).hexdigest() == document["base_gripper_geometry"][
        "sha256"
    ]
    assert document["simulation_only"] is True
    assert document["motion_authorized"] is False
    assert document["control_contract"]["command_is_force_claim"] is False
    assert document["control_contract"]["close_only_after_two_layer_contact_gate"] is True
    assert document["control_contract"][
        "collision_validation_uses_exact_two_layer_target"
    ] is True
    assert document["control_contract"]["both_arms_used_for_nominal_second_fold"] is True
    assert document["control_contract"][
        "simultaneous_release_after_dual_laydown_gate"
    ] is True

    base_document = json.loads(base.read_text(encoding="utf-8"))
    fraction = document["interpolation_fraction_one_to_four"]
    for side in ("left", "right"):
        one = base_document["grasp_commands"][side]["one_layer"][
            "operational_candidate_rad"
        ]
        four = base_document["grasp_commands"][side]["four_layer"][
            "operational_candidate_rad"
        ]
        expected = one + fraction * (four - one)
        project = document["two_layer_project_contact_target_rad"][side]
        assert project == pytest.approx(expected)
        model_q0 = base_document["geometry"][
            "detailed_stl_model_q_at_physical_q0_rad"
        ]
        assert document["two_layer_model_contact_target_rad"][side] == pytest.approx(
            model_q0 - project
        )
