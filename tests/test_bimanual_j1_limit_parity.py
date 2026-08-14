from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools/check_bimanual_j1_limit_parity_plan_only.py"
SPEC = importlib.util.spec_from_file_location("check_j1_limit_parity", TOOL)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
APPROVED = ROOT / "config/bimanual_j1_operational_limits.approved.json"
FIRMWARE_LIMITS = ROOT / (
    "firmware/stm32_g474_single_arm/Core/Src/"
    "bimanual_operational_limits.c"
)
CMAKE = (ROOT / "firmware/stm32_g474_single_arm/CMakeLists.txt").read_text()
BINARY = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Src/binary_control.c"
).read_text()
SOURCE = TOOL.read_text(encoding="utf-8")


def load_approved() -> dict:
    return MODULE.load_bound_json(
        APPROVED, MODULE.file_sha256(APPROVED), "approved"
    )


def test_approved_arm_limits_match_firmware_and_host_math() -> None:
    report = MODULE.check_parity(ROOT, load_approved())
    assert report["status"] == MODULE.STATUS
    assert report["motion_authorized"] is False
    assert report["runtime_change_authorized"] is False
    assert report["arm_joint_count"] == 10
    assert report["parity"] == {
        "firmware_unwrapped_urad": True,
        "host_unwrapped_rad": True,
        "urdf": False,
        "moveit": False,
        "isaac": False,
    }
    assert report["projections"]["left"]["shoulder"][
        "maximum_unwrapped_raw"
    ] == 4123
    assert report["projections"]["right"]["shoulder"][
        "maximum_unwrapped_raw"
    ] == 4124


def test_firmware_table_has_ten_approved_limits_and_two_gripper_sentinels() -> None:
    limits = MODULE.parse_firmware_limits(
        FIRMWARE_LIMITS.read_text(encoding="utf-8")
    )
    assert len(limits) == 12
    assert limits[1] == (-130388, 3183010)
    assert limits[7] == (-191748, 3184544)
    assert limits[5] == limits[11] == MODULE.GRIPPER_SENTINEL


def test_candidate_identity_is_unique_and_still_no_output() -> None:
    assert "PROTOCOL_V2_J1_LIMITS_SHADOW_CANDIDATE" in CMAKE
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024100UL" in CMAKE
    assert "HOST_PROTOCOL_V2_J1_LIMITS_VALIDATION_BUILD=1U" in CMAKE
    assert "HOST_PROTOCOL_V2_UNWRAP_SHADOW_VALIDATION_BUILD=1U" in CMAKE
    assert "HOST_F25_VALIDATION_ONLY_BUILD=1U" in CMAKE
    assert "BimanualOperationalLimits_LoadJ1LShadow(limits)" in BINARY
    service = BINARY.split("static void Host_ServiceV2Executor", 1)[1].split(
        "static void Host_SendV2ExecutorDiagnostics", 1
    )[0]
    assert "host_v2_discarded_output_urad" in service
    for forbidden in ("Servo_", "RightServoBus_", "SyncWrite", "Torque"):
        assert forbidden not in service


def test_approval_and_evidence_sha_fail_closed(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="SHA mismatch"):
        MODULE.load_bound_json(APPROVED, "0" * 64, "approved")
    document = load_approved()
    document["operator_approved"] = False
    with pytest.raises(ValueError, match="not operator-approved"):
        MODULE.check_parity(ROOT, document)

    document = load_approved()
    document["inputs"]["derived_candidate"]["sha256"] = "0" * 64
    with pytest.raises(ValueError, match="SHA mismatch"):
        MODULE.check_parity(ROOT, document)


def test_firmware_drift_is_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    approved = load_approved()
    original = MODULE.parse_firmware_limits

    def drifted(source: str) -> tuple[tuple[int, int], ...]:
        values = list(original(source))
        values[0] = (values[0][0] - 1, values[0][1])
        return tuple(values)

    monkeypatch.setattr(MODULE, "parse_firmware_limits", drifted)
    with pytest.raises(ValueError, match="do not match"):
        MODULE.check_parity(ROOT, approved)


def test_tool_has_no_motion_or_runtime_mutation_api() -> None:
    assert "--plan-only is required" in SOURCE
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
