"""R4 publishes 12-axis observations only after verified torque-off."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import struct
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws/src/single_arm_bridge"
sys.path.insert(0, str(PACKAGE))

from single_arm_bridge.bimanual_feedback import (  # noqa: E402
    compose_bimanual_feedback,
    validate_bimanual_calibrations,
)
from single_arm_bridge.calibration import load_calibration  # noqa: E402
from single_arm_bridge.protocol import (  # noqa: E402
    ProtocolError,
    RIGHT_ARM_DISABLE,
    RightArmDiscovery,
    parse_right_arm_disable,
)


FIRMWARE = ROOT / "firmware/stm32_g474_single_arm"
CONFIG = (FIRMWARE / "Core/Inc/single_arm_config.h").read_text()
RIGHT_HEADER = (FIRMWARE / "Core/Inc/right_servo_bus.h").read_text()
RIGHT_BUS = (FIRMWARE / "Core/Src/right_servo_bus.c").read_text()
BINARY = (FIRMWARE / "Core/Src/binary_control.c").read_text()
BRIDGE = (PACKAGE / "single_arm_bridge/bridge_node.py").read_text()
TRANSPORT = (PACKAGE / "single_arm_bridge/transport.py").read_text()
LAUNCH = (PACKAGE / "launch/bridge.launch.py").read_text()
LEFT_CALIBRATION = PACKAGE / "config/single_arm_calibration.json"
RIGHT_CALIBRATION = PACKAGE / "config/right_arm_calibration.candidate.json"
SOAK_PATH = ROOT / "tools/soak_bimanual_joint_states_read_only.py"
SOAK_SPEC = importlib.util.spec_from_file_location("r4_bimanual_soak", SOAK_PATH)
assert SOAK_SPEC is not None and SOAK_SPEC.loader is not None
SOAK = importlib.util.module_from_spec(SOAK_SPEC)
SOAK_SPEC.loader.exec_module(SOAK)
SOAK_SOURCE = SOAK_PATH.read_text(encoding="utf-8")


def complete_right_snapshot(
    positions: tuple[int, ...] = (2048,) * 6,
) -> RightArmDiscovery:
    return RightArmDiscovery(
        status_code=0,
        joint_count=6,
        present_mask=0x3F,
        positions_raw=positions,
        read_statuses=(0,) * 6,
        transaction_count=6,
        failure_count=0,
    )


def test_verified_disable_wire_schema_is_strict() -> None:
    payload = struct.pack("<BBBB", 0, 6, 0, 0)
    assert len(payload) == RIGHT_ARM_DISABLE.size == 4
    snapshot = parse_right_arm_disable(payload)
    assert snapshot.status_code == 0
    assert snapshot.torque_enabled_mask == 0
    assert snapshot.failure_count == 0

    with pytest.raises(ProtocolError):
        parse_right_arm_disable(struct.pack("<BBBB", 0, 5, 0, 0))
    with pytest.raises(ProtocolError):
        parse_right_arm_disable(struct.pack("<BBBB", 0, 6, 0x80, 0))
    with pytest.raises(ProtocolError):
        parse_right_arm_disable(struct.pack("<BBBB", 0, 6, 0, 7))


def test_combined_feedback_has_unique_left_then_right_names() -> None:
    left = load_calibration(LEFT_CALIBRATION)
    right = load_calibration(RIGHT_CALIBRATION)
    validate_bimanual_calibrations(left, right)
    sample = compose_bimanual_feedback(
        left,
        right,
        (2048,) * 6,
        complete_right_snapshot(),
    )
    assert len(sample.names) == len(set(sample.names)) == 12
    assert sample.names[:6] == tuple(left.ros_joint_names)
    assert sample.names[6:] == tuple(right.ros_joint_names)
    assert sample.positions == (0.0,) * 12


@pytest.mark.parametrize(
    "snapshot",
    (
        RightArmDiscovery(2, 6, 0x1F, (2048,) * 6, (0,) * 6, 6, 1),
        RightArmDiscovery(0, 6, 0x3F, (2048,) * 6, (0, 0, 3, 0, 0, 0), 6, 1),
    ),
)
def test_combined_feedback_rejects_partial_right_bus(
    snapshot: RightArmDiscovery,
) -> None:
    with pytest.raises(ValueError):
        compose_bimanual_feedback(
            load_calibration(LEFT_CALIBRATION),
            load_calibration(RIGHT_CALIBRATION),
            (2048,) * 6,
            snapshot,
        )


def test_firmware_verifies_all_right_torque_registers_before_ack() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023B00)" in CONFIG
    assert "HOST_RIGHT_ARM_VERIFIED_DISABLE_CAPABILITY UINT32_C(0x00400000)" in CONFIG
    assert "RightServoBus_DisableTorqueAllVerified" in RIGHT_HEADER
    disable = RIGHT_BUS[RIGHT_BUS.index("RightServoDisableSnapshot RightServoBus_DisableTorqueAllVerified"):]
    assert "RightServo_WriteData" in disable
    assert "RightServo_ReadData" in disable
    assert "RIGHT_SERVO_TORQUE_ENABLE_ADDRESS" in disable
    assert "torque_readback[0] != 0U" in disable
    response = BINARY[BINARY.index("static void Host_SendRightArmDisableVerified"):]
    assert "ACTUATOR_MSG_RIGHT_ARM_DISABLE_RESPONSE" in response
    assert response.index("RightServoBus_DisableTorqueAllVerified") < response.index("Host_SendBinaryFrame")


def test_bridge_mode_is_observation_only_and_separate_from_left_topic() -> None:
    assert "RIGHT_ARM_VERIFIED_DISABLE_CAPABILITY" in TRANSPORT
    assert "disable_right_arm_verified" in TRANSPORT
    assert 'self.declare_parameter("publish_bimanual_read_only", False)' in BRIDGE
    assert '"bimanual_joint_states"' in BRIDGE
    assert "self._disable_both_arms_verified()" in BRIDGE
    assert "publish_bimanual_read_only is mutually exclusive" in BRIDGE
    assert 'mode = "BIMANUAL_READ_ONLY"' in BRIDGE
    assert '"publish_bimanual_read_only"' in LAUNCH
    assert '"bimanual_read_only_feedback"' in BRIDGE
    assert '!= "right_arm_calibration_candidate"' in BRIDGE
    assert 'right_document.get("motion_authorized") is not False' in BRIDGE
    assert "hardware-synchronous" in BRIDGE


def test_packaged_right_candidate_matches_evidence_bound_source() -> None:
    assert RIGHT_CALIBRATION.read_bytes() == (
        ROOT / "config/right_arm_calibration.candidate.json"
    ).read_bytes()


def test_soak_tool_is_read_only_and_checks_both_topics() -> None:
    assert SOAK.BIMANUAL_TOPIC == "/bimanual_joint_states"
    assert SOAK.LEFT_TOPIC == "/joint_states"
    assert SOAK.calibration_joint_names(LEFT_CALIBRATION, "left")[0] == (
        "left_base_joint"
    )
    assert SOAK.calibration_joint_names(RIGHT_CALIBRATION, "right")[0] == (
        "right_base_joint"
    )
    assert '"motion_authorized": False' in SOAK_SOURCE
    assert '"hardware_synchronous": False' in SOAK_SOURCE
    for forbidden in (
        "arm_and_enable",
        "clear_fault",
        "send_goal_async",
        "right_arm_jog_once",
        "right_arm_torque_enable_once",
    ):
        assert forbidden not in SOAK_SOURCE
