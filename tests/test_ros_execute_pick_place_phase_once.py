import copy
import importlib.util
import json
from pathlib import Path
import sys

import pytest
from sensor_msgs.msg import JointState


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools" / "ros_execute_pick_place_phase_once.py"
SPEC = importlib.util.spec_from_file_location(
    "ros_execute_pick_place_phase_once",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

MANIFEST = (
    ROOT
    / "artifacts"
    / "stage7"
    / "2026-07-31"
    / "full_pick_place"
    / "full_pick_place_plan_only_manifest.json"
)
CALIBRATION = ROOT / "config" / "single_arm_calibration.json"
MANIFEST_SHA256 = (
    "1ced3a178692a5ebac26b70a39d0750c3daa71277aa128c599af345535b2a190"
)


def load(
    phase: str,
    manifest: Path = MANIFEST,
    manifest_sha256: str = MANIFEST_SHA256,
):
    return MODULE.load_phase(
        manifest,
        manifest_sha256,
        phase,
        CALIBRATION,
    )


def current_calibration_manifest(tmp_path: Path) -> tuple[Path, str]:
    """역사 artifact를 수정하지 않고 현재 hash의 합성 loader fixture를 만든다."""
    document = json.loads(MANIFEST.read_text(encoding="utf-8"))
    document["calibration_hash"] = "0x2D90167E"
    path = tmp_path / "current-calibration-manifest.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    return path, MODULE.sha256_file(path)


def valid_diagnostics() -> dict:
    names = (*MODULE.ARM_JOINTS, MODULE.GRIPPER_JOINT)
    return {
        "protocol_version": 1,
        "joint_count": 6,
        "calibration_hash": "0x2D90167E",
        "joints": [
            {
                "name": name,
                "servo_id": index + 1,
                "torque_enabled": True,
                "position_raw": 1984 if index == 5 else 2048,
                "goal_position_raw": 1963 if index == 5 else 2048,
                "load_magnitude_raw": 96 if index == 5 else 0,
                "voltage_v": 12.2,
                "temperature_c": 40,
                "p_gain": MODULE.EXPECTED_P_GAINS[index],
                "torque_limit_raw": MODULE.EXPECTED_TORQUE_LIMITS[index],
            }
            for index, name in enumerate(names)
        ],
    }


def parse_diagnostics(document: dict, contact=True, open_=False):
    return MODULE.parse_diagnostics_message(
        json.dumps(document),
        "0x2D90167E",
        contact,
        open_,
        1963,
        2009,
    )


def test_historical_manifest_fails_closed_and_current_fixture_is_bounded(
    tmp_path: Path,
) -> None:
    # 7월 artifact는 보존한다. 현재 calibration으로 실행하려 하면 거부돼야 한다.
    with pytest.raises(ValueError, match="calibration hashes differ"):
        load("q0_to_pick_pregrasp")

    manifest, digest = current_calibration_manifest(tmp_path)
    phases = [load(name, manifest, digest) for name in MODULE.PHASE_ORDER]
    assert [phase.name for phase in phases] == list(MODULE.PHASE_ORDER)
    assert phases[0].expected_arm_start == (0.0,) * 5
    assert phases[-1].expected_arm_end == (0.0,) * 5
    assert len(phases[0].steps) == 10
    assert len(phases[-1].steps) == 10
    assert phases[2].steps[0]["kind"] == "gripper"
    assert phases[6].steps[0]["kind"] == "gripper"
    assert all(phase.maximum_joint_step_rad == 0.18 for phase in phases)

    with pytest.raises(ValueError, match="manifest sha256 mismatch"):
        MODULE.load_phase(manifest, "0" * 64, "pick_close", CALIBRATION)


def test_manifest_source_tamper_and_phase_gate_tamper_fail_closed(
    tmp_path: Path,
) -> None:
    document = json.loads(MANIFEST.read_text(encoding="utf-8"))
    document["automatic_execution_permitted"] = True
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    digest = MODULE.sha256_file(path)
    with pytest.raises(ValueError, match="automatic_execution_permitted=false"):
        MODULE.load_phase(path, digest, "pick_close", CALIBRATION)

    document = json.loads(MANIFEST.read_text(encoding="utf-8"))
    # 이 시험의 관심사는 phase 연속성이다. 현재 calibration hash의 합성
    # fixture로 만들어 hash gate 다음의 discontinuity gate까지 도달시킨다.
    document["calibration_hash"] = "0x2D90167E"
    document["steps"][13]["start_positions_rad"][0] += 0.01
    path.write_text(json.dumps(document), encoding="utf-8")
    digest = MODULE.sha256_file(path)
    with pytest.raises(ValueError, match="discontinuous"):
        MODULE.load_phase(
            path,
            digest,
            "pick_grasp_to_lift20",
            CALIBRATION,
        )


def test_joint_state_requires_all_six_joints_and_reorders() -> None:
    message = JointState()
    message.name = [
        MODULE.GRIPPER_JOINT,
        MODULE.ARM_JOINTS[4],
        MODULE.ARM_JOINTS[0],
        MODULE.ARM_JOINTS[2],
        MODULE.ARM_JOINTS[1],
        MODULE.ARM_JOINTS[3],
    ]
    message.position = [0.6, 0.5, 0.1, 0.3, 0.2, 0.4]
    arm, gripper = MODULE.positions_from_joint_state(message)
    assert arm == (0.1, 0.2, 0.3, 0.4, 0.5)
    assert gripper == 0.6

    message.name.pop()
    message.position.pop()
    with pytest.raises(ValueError, match="missing"):
        MODULE.positions_from_joint_state(message)


def test_start_final_and_actual_step_gates_are_strict() -> None:
    assert MODULE.validate_pose(
        (0.01,) * 5,
        (0.0,) * 5,
        0.05,
        "start",
    ) == pytest.approx(0.01)
    with pytest.raises(ValueError, match="left_elbow_joint"):
        MODULE.validate_pose(
            (0.0, 0.0, 0.051, 0.0, 0.0),
            (0.0,) * 5,
            0.05,
            "start",
        )
    with pytest.raises(ValueError, match="current-to-target"):
        MODULE.validate_actual_step((0.0,) * 5, (0.181,) * 5, 0.18)
    assert MODULE.validate_gripper_position(0.098, 0.13) == pytest.approx(0.032)
    with pytest.raises(ValueError, match="gripper phase-state mismatch"):
        MODULE.validate_gripper_position(0.13, 0.06)


def test_arm_phase_resume_is_checkpoint_pinned_and_fail_closed(
    tmp_path: Path,
) -> None:
    manifest, digest = current_calibration_manifest(tmp_path)
    phase = load("q0_to_pick_pregrasp", manifest, digest)
    resumed = MODULE.resume_arm_phase(phase, 1)
    assert resumed.steps == phase.steps[1:]
    assert resumed.expected_arm_start == tuple(
        phase.steps[0]["target_positions_rad"]
    )
    assert resumed.expected_arm_end == phase.expected_arm_end

    assert MODULE.resume_arm_phase(phase, 0) is phase
    with pytest.raises(ValueError, match="cannot be negative"):
        MODULE.resume_arm_phase(phase, -1)
    with pytest.raises(ValueError, match="leave at least one"):
        MODULE.resume_arm_phase(phase, len(phase.steps))
    with pytest.raises(ValueError, match="arm-only"):
        MODULE.resume_arm_phase(load("pick_close", manifest, digest), 1)


def test_diagnostics_require_identity_torque_settings_voltage_and_temperature() -> None:
    document = valid_diagnostics()
    assert parse_diagnostics(document)["joint_count"] == 6

    cases = [
        ("torque_enabled", False, "torque is not enabled"),
        ("p_gain", 31, "P gain mismatch"),
        ("torque_limit_raw", 999, "torque limit mismatch"),
        ("voltage_v", 11.4, "voltage is below"),
        ("temperature_c", 56, "temperature exceeds"),
    ]
    for key, value, message in cases:
        bad = copy.deepcopy(document)
        bad["joints"][1][key] = value
        with pytest.raises(ValueError, match=message):
            parse_diagnostics(bad)


def test_contact_and_release_readback_are_fail_closed() -> None:
    contact = valid_diagnostics()
    assert parse_diagnostics(contact)["joints"][-1]["load_magnitude_raw"] == 96

    no_contact = copy.deepcopy(contact)
    no_contact["joints"][-1]["load_magnitude_raw"] = 0
    no_contact["joints"][-1]["position_raw"] = 1963
    with pytest.raises(ValueError, match="no retained-contact evidence"):
        parse_diagnostics(no_contact)

    released = copy.deepcopy(contact)
    released["joints"][-1]["goal_position_raw"] = 2009
    released["joints"][-1]["position_raw"] = 2003
    released["joints"][-1]["load_magnitude_raw"] = 0
    assert parse_diagnostics(released, contact=False, open_=True)


def test_executor_has_one_send_site_and_no_retry_construct() -> None:
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert source.count("send_goal_async") == 1
    assert "for attempt" not in source
    assert "while attempt" not in source
    assert "--execute-phase-once" in source
    assert "--resume-after-arm-steps" in source
