from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = (
    ROOT
    / "firmware"
    / "stm32_g474_single_arm"
    / "Core"
    / "Inc"
    / "single_arm_config.h"
).read_text(encoding="utf-8")
BINARY = (
    ROOT
    / "firmware"
    / "stm32_g474_single_arm"
    / "Core"
    / "Src"
    / "binary_control.c"
).read_text(encoding="utf-8")
TRANSPORT = (
    ROOT
    / "ros2_ws"
    / "src"
    / "single_arm_bridge"
    / "single_arm_bridge"
    / "transport.py"
).read_text(encoding="utf-8")
ARM_ACTION = (
    ROOT
    / "ros2_ws"
    / "src"
    / "single_arm_bridge"
    / "single_arm_bridge"
    / "follow_joint_trajectory_server.py"
).read_text(encoding="utf-8")
GRIPPER_ACTION = (
    ROOT
    / "ros2_ws"
    / "src"
    / "single_arm_bridge"
    / "single_arm_bridge"
    / "parallel_gripper_command_server.py"
).read_text(encoding="utf-8")


def test_identity_and_capability_are_bumped_for_diagnostics() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00023400)" in CONFIG
    assert "HOST_BINARY_CAPABILITIES UINT32_C(0x0000FFFF)" in CONFIG
    assert "F0_METRICS_CANDIDATE_FIRMWARE_VERSION = 0x00023000" in (
        ROOT
        / "ros2_ws"
        / "src"
        / "single_arm_bridge"
        / "single_arm_bridge"
        / "hardware_identity.py"
    ).read_text(encoding="utf-8")


def test_diagnostics_are_on_demand_per_joint_and_motion_exclusive() -> None:
    assert "response.message_type = ACTUATOR_MSG_DIAGNOSTICS;" in BINARY
    assert "request->payload[0] == 2U" in BINARY
    assert "request->payload[1] < servo_joint_count" in BINARY
    assert "host_binary_motion.active != 0U" in BINARY
    assert "bytes((2, joint_index))" in TRANSPORT
    assert "self.heartbeat()" in TRANSPORT
    assert "DIAGNOSTIC_RESPONSE_TIMEOUT_S = 0.5" in TRANSPORT


def test_endpoint_uses_bounded_multi_sample_settling_with_safety_poll() -> None:
    assert "SERVO_FINAL_SETTLE_SAMPLE_MS UINT32_C(100)" in CONFIG
    assert "SERVO_FINAL_SETTLE_MAX_MS UINT32_C(1000)" in CONFIG
    assert "SERVO_FINAL_SETTLE_CONSECUTIVE UINT8_C(2)" in CONFIG
    assert "SERVO_FINAL_ERROR_TOLERANCE_RAW UINT16_C(30)" in CONFIG

    verify_start = BINARY.index("if (host_binary_motion.verifying != 0U)")
    verify_end = BINARY.index(
        "if ((uint32_t)(now -",
        verify_start,
    )
    verify = BINARY[verify_start:verify_end]
    assert "Servo_MotionSafetyPoll()" in verify
    assert "verify_consecutive" in verify
    assert "SERVO_FINAL_SETTLE_MAX_MS" in verify
    assert "SERVO_FINAL_ERROR_TOLERANCE_RAW" in verify
    assert "Servo_PositionSweepStep(" in verify
    assert "Servo_ReadAllPositions(" not in verify
    assert "Servo_MotionSafetyEnd();" in verify


def test_ros_actions_allow_firmware_settling_window() -> None:
    assert "completion_timeout_s: float = 3.5" in ARM_ACTION
    assert "completion_timeout_s: float = 3.5" in GRIPPER_ACTION
