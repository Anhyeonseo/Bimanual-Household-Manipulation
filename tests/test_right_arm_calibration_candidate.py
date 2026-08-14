"""R2 candidate stays evidence-bound and cannot silently authorize motion."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from single_arm_bridge.calibration import load_calibration


ROOT = Path(__file__).resolve().parents[1]
CANDIDATE_PATH = ROOT / "config/right_arm_calibration.candidate.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_candidate_is_right_slot_and_backend_authority_is_explicit() -> None:
    document = json.loads(CANDIDATE_PATH.read_text(encoding="utf-8"))
    calibration = load_calibration(CANDIDATE_PATH)
    assert document["arm_slot"] == calibration.arm_slot == "right"
    # The old single-arm right backend remains blocked. Motion approval is
    # limited to the separately validated resident protocol-v2 12-axis path.
    assert document["motion_authorized"] is False
    assert document["resident_bimanual_stream_motion_authorized"] is True
    assert (
        document["authorized_motion_backend"]
        == "protocol_v2_resident_bimanual_stream_only"
    )
    assert document["geometric_model_precision_complete"] is False
    assert calibration.calibration_hash == 0x2D90167E
    assert document["identity"]["arm_bound_identity"] == "right:0x2D90167E"


def test_candidate_evidence_hashes_are_current() -> None:
    document = json.loads(CANDIDATE_PATH.read_text(encoding="utf-8"))
    for evidence in document["evidence"].values():
        path = ROOT / evidence["path"]
        assert path.is_file()
        assert sha256(path) == evidence["sha256"]


def test_r1_1_acceptance_covers_all_ids_without_authorizing_more_motion() -> None:
    acceptance = json.loads(
        (ROOT / "artifacts/right_arm/2026-08-12/r11_acceptance.json").read_text(
            encoding="utf-8"
        )
    )
    assert acceptance["motion_authorized_beyond_this_gate"] is False
    assert acceptance["overall_verdict"] == "R1_1_SINGLE_AXIS_GATE_PASS"
    assert [item["servo_id"] for item in acceptance["joint_results"]] == list(
        range(1, 7)
    )
    for item in acceptance["joint_results"]:
        assert item["verdict"] == "PASS"
        assert item["torque_enable_present_raw"] == item["torque_enable_held_goal_raw"]
        assert item["torque_enable_present_raw"] == item["torque_enable_observed_raw"]
        assert item["jog_target_raw"] - item["jog_start_raw"] == 8
        assert abs(item["target_residual_raw"]) <= 3


def test_candidate_mapping_matches_operator_confirmed_left_mapping() -> None:
    right = load_calibration(CANDIDATE_PATH)
    left = load_calibration(
        ROOT / "ros2_ws/src/single_arm_bridge/config/single_arm_calibration.json"
    )
    assert [
        (j.servo_id, j.name, j.zero_raw, j.direction) for j in right.joints
    ] == [
        (j.servo_id, j.name, j.zero_raw, j.direction) for j in left.joints
    ]
