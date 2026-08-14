from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools/derive_bimanual_operational_limits_plan_only.py"
SPEC = importlib.util.spec_from_file_location("derive_j1_limits", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
SOURCE = TOOL_PATH.read_text(encoding="utf-8")
MANIFEST_PATH = ROOT / "config/bimanual_j0_desired_envelope.reviewed.json"
LEFT_PATH = (
    ROOT / "ros2_ws/src/single_arm_bridge/config/single_arm_calibration.json"
)
RIGHT_PATH = ROOT / "config/right_arm_calibration.candidate.json"


def candidate(inset_raw: int = 64) -> dict:
    manifest = MODULE.load_sha_bound_json(
        MANIFEST_PATH, MODULE.file_sha256(MANIFEST_PATH)
    )
    calibrations = {
        "left": MODULE.load_calibration(LEFT_PATH, "left"),
        "right": MODULE.load_calibration(RIGHT_PATH, "right"),
    }
    return MODULE.derive_candidate(manifest, calibrations, inset_raw)


def test_reviewed_manifest_is_fail_closed_and_evidence_bound() -> None:
    document = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    assert document["status"] == "J0_D_REVIEWED_PASS_J0_M_NOT_MEASURED"
    assert document["motion_authorized"] is False
    assert document["operator_confirmation"] == {
        "all_sweeps_cable_safe": True,
        "cable_or_connector_issue_observed": False,
        "mechanical_hard_stops_measured": False,
        "meaning": (
            "manually traversed desired task workspace, "
            "not mechanical endpoints"
        ),
    }
    assert document["arms"]["left"]["joints"]["shoulder"][
        "wrap_crossings"
    ] == 4
    assert document["arms"]["right"]["joints"]["shoulder"][
        "wrap_crossings"
    ] == 8


def test_default_contraction_is_per_arm_and_keeps_q0() -> None:
    document = candidate()
    expected = {
        ("left", "base"): (1047, 2977),
        ("left", "shoulder"): (1963, 4123),
        ("left", "elbow"): (350, 2428),
        ("left", "wrist_flex"): (234, 2320),
        ("left", "wrist_roll"): (651, 2774),
        ("right", "base"): (1172, 2932),
        ("right", "shoulder"): (1923, 4124),
        ("right", "elbow"): (361, 2459),
        ("right", "wrist_flex"): (441, 2374),
        ("right", "wrist_roll"): (813, 2906),
    }
    for (arm, name), interval in expected.items():
        joint = document["arms"][arm]["joints"][name]
        assert joint["status"] == "PLAN_ONLY_CONTRACTED_CANDIDATE"
        assert joint["contains_q0"] is True
        assert (
            joint["candidate_minimum_unwrapped_raw"],
            joint["candidate_maximum_unwrapped_raw"],
        ) == interval
        assert joint["manual_observation_margin_lower_raw"] == 64
        assert joint["manual_observation_margin_upper_raw"] == 64
        assert joint["runtime_change_authorized"] is False


def test_shoulders_remain_explicitly_unwrapped() -> None:
    document = candidate()
    left = document["arms"]["left"]["joints"]["shoulder"]
    right = document["arms"]["right"]["joints"]["shoulder"]
    assert left["coordinate"] == right["coordinate"] == "unwrapped_raw"
    assert left["wrap_aware"] is right["wrap_aware"] is True
    assert left["candidate_maximum_unwrapped_raw"] > 4095
    assert right["candidate_maximum_unwrapped_raw"] > 4095
    assert left["candidate_upper_rad"] > 3.0
    assert right["candidate_upper_rad"] > 3.0


def test_grippers_are_not_reduced_to_generic_joint_limits() -> None:
    document = candidate()
    left = document["arms"]["left"]["joints"]["gripper"]
    right = document["arms"]["right"]["joints"]["gripper"]
    for item in (left, right):
        assert item["status"] == "BLOCKED_SEMANTIC_GRIPPER_MAPPING_REQUIRED"
        assert item["automatic_limit_candidate"] is False
        assert item["runtime_change_authorized"] is False
        assert "candidate_minimum_unwrapped_raw" not in item
    assert left["observations"]["task_open"] == 3257
    assert left["observations"]["loaded_close_command"] == 1963
    assert right["observations"]["task_open"] == 3062


def test_candidate_cannot_claim_parity_or_motion() -> None:
    document = candidate()
    assert document["status"] == MODULE.STATUS
    assert document["motion_authorized"] is False
    assert document["runtime_change_authorized"] is False
    assert document["execution_api_used"] is False
    assert set(document["parity_targets"].values()) == {False}
    codes = {item["code"] for item in document["blockers"]}
    assert {
        "J0_M_NOT_MEASURED",
        "TASK_COVERAGE_AFTER_CONTRACTION_NOT_PROVEN",
        "GRIPPER_SEMANTIC_MAPPING_PENDING",
        "PHYSICAL_ZERO_AND_MODEL_ALIGNMENT_PENDING",
        "RIGHT_ACTIVE_TRACKING_ENVELOPE_PENDING",
    } <= codes


def test_manifest_sha_and_bad_inset_fail_closed(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="SHA mismatch"):
        MODULE.load_sha_bound_json(MANIFEST_PATH, "0" * 64)
    with pytest.raises(ValueError, match="inset_raw"):
        candidate(0)
    with pytest.raises(ValueError, match="consumes"):
        candidate(1000)


def test_tool_has_no_motion_or_runtime_mutation_api() -> None:
    assert "--plan-only is required" in SOURCE
    assert '"motion_authorized": False' in SOURCE
    assert '"runtime_change_authorized": False' in SOURCE
    for forbidden in (
        "rclpy",
        "serial",
        ".arm_and_enable(",
        ".enable(",
        "send_goal_async",
        "openocd",
        "Servo_",
        "RightServoBus_",
    ):
        assert forbidden not in SOURCE
