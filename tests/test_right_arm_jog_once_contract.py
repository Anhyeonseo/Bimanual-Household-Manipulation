"""R1 must remain a bounded, single-servo migration primitive."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware/stm32_g474_single_arm"
HEADER = (FIRMWARE / "Core/Inc/right_servo_bus.h").read_text(encoding="utf-8")
BUS = (FIRMWARE / "Core/Src/right_servo_bus.c").read_text(encoding="utf-8")
BINARY = (FIRMWARE / "Core/Src/binary_control.c").read_text(encoding="utf-8")
PROTOCOL_CORE = (ROOT / "firmware/stm32_actuator/src/protocol.c").read_text(
    encoding="utf-8"
)
BRIDGE = (ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/bridge_node.py").read_text(encoding="utf-8")
LAUNCH = (ROOT / "ros2_ws/src/single_arm_bridge/launch/bridge.launch.py").read_text(encoding="utf-8")
TOOL = (ROOT / "tools/execute_right_arm_jog_once.py").read_text(encoding="utf-8")
TORQUE_TOOL = (ROOT / "tools/enable_right_arm_torque_once.py").read_text(
    encoding="utf-8"
)


def test_jog_has_a_small_hard_coded_single_servo_envelope() -> None:
    assert "RIGHT_SERVO_JOG_MINIMUM_ABSOLUTE_DELTA_RAW INT16_C(8)" in HEADER
    assert "RIGHT_SERVO_JOG_MAXIMUM_ABSOLUTE_DELTA_RAW INT16_C(20)" in HEADER
    assert "RIGHT_SERVO_GOAL_POSITION_ADDRESS UINT8_C(42)" in BUS
    assert "RightServoBus_JogOnce" in BUS
    for forbidden in ("SyncWrite", "PID", "SPEED", "BROADCAST"):
        assert forbidden not in BUS


def test_jog_never_enables_torque_and_stop_disables_active_right_bus() -> None:
    jog = BUS[
        BUS.index("RightServoJogSnapshot RightServoBus_JogOnce"):
        BUS.index("RightServoTorqueEnableSnapshot")
    ]
    assert "RIGHT_SERVO_TORQUE_ENABLE_ADDRESS" in jog
    assert "torque[0] != 1U" in jog
    assert "RightServo_WriteData(servo_id, RIGHT_SERVO_TORQUE_ENABLE_ADDRESS" not in jog
    assert "RightServoBus_DisableTorqueAll" in BINARY
    assert "host_right_arm_output_active" in BINARY


def test_jog_requires_exact_operator_confirmation_in_bridge_and_client() -> None:
    assert 'RIGHT_ARM_JOG_CONFIRMATION = "RIGHT_ARM_JOG_ONCE"' in BRIDGE
    assert "request.confirmation != RIGHT_ARM_JOG_CONFIRMATION" in BRIDGE
    assert 'CONFIRMATION = "RIGHT_ARM_JOG_ONCE"' in TOOL
    assert "--confirmation" in TOOL


def test_jog_message_ids_are_accepted_by_the_firmware_protocol_core() -> None:
    assert "ACTUATOR_MSG_RIGHT_ARM_JOG_ONCE_REQUEST" in PROTOCOL_CORE
    assert "ACTUATOR_MSG_RIGHT_ARM_JOG_ONCE_RESPONSE" in PROTOCOL_CORE


def test_torque_enable_is_a_distinct_confirmed_one_servo_primitive() -> None:
    enable = BUS[
        BUS.index("RightServoTorqueEnableSnapshot"):
        BUS.index("HAL_StatusTypeDef RightServoBus_DisableTorqueAll")
    ]
    assert "RightServoBus_EnableTorqueAtPresentPositionOnce" in enable
    assert "RIGHT_SERVO_GOAL_POSITION_ADDRESS" in enable
    assert "RIGHT_SERVO_TORQUE_ENABLE_ADDRESS" in enable
    assert "RIGHT_SERVO_TORQUE_ENABLE_READBACK_FAILED" in enable
    for forbidden in ("SyncWrite", "PID", "SPEED", "BROADCAST"):
        assert forbidden not in enable
    assert (
        'RIGHT_ARM_TORQUE_ENABLE_CONFIRMATION = '
        '"RIGHT_ARM_TORQUE_ENABLE_ONCE"'
    ) in BRIDGE
    assert "request.confirmation != RIGHT_ARM_TORQUE_ENABLE_CONFIRMATION" in BRIDGE
    assert 'CONFIRMATION = "RIGHT_ARM_TORQUE_ENABLE_ONCE"' in TORQUE_TOOL
    assert "ACTUATOR_MSG_RIGHT_ARM_TORQUE_ENABLE_ONCE_REQUEST" in PROTOCOL_CORE
    assert "ACTUATOR_MSG_RIGHT_ARM_TORQUE_ENABLE_ONCE_RESPONSE" in PROTOCOL_CORE


def test_jog_is_isolated_from_left_arm_motion_enable() -> None:
    handler = BINARY[
        BINARY.index("static uint8_t Host_RightArmOneShotPermitted"):
        BINARY.index("static void Host_SendBinaryHello")
    ]
    assert "host_binary_safety.state == ACTUATOR_STATE_SAFE_DISABLED" in handler
    assert "actuator_safety_accepts_setpoint" not in handler
    assert 'self.declare_parameter("allow_right_arm_jog", False)' in BRIDGE
    assert 'self.declare_parameter("left_arm_power_off_confirmed", False)' in BRIDGE
    assert "self._allow_motion and self._allow_right_arm_jog" in BRIDGE
    assert "right-arm jog requires left_arm_power_off_confirmed:=true" in BRIDGE
    assert "if not self._allow_right_arm_jog" in BRIDGE
    assert '"allow_right_arm_jog"' in LAUNCH
    assert '"left_arm_power_off_confirmed"' in LAUNCH


def test_power_off_confirmed_isolation_never_calls_left_bus_disable() -> None:
    init = BRIDGE[
        BRIDGE.index("arm_attempted = False"):
        BRIDGE.index("if self._motion_armed:")
    ]
    assert "elif self._allow_right_arm_jog:" in init
    assert "if not self._allow_right_arm_jog:" in init
    isolated = init[
        init.index("elif self._allow_right_arm_jog:"):
        init.index("else:\n                # READ_ONLY")
    ]
    assert "self._transport.disable()" not in isolated
    shutdown = BRIDGE[BRIDGE.index("def destroy_node"):BRIDGE.index("def _motion_backend_ready")]
    assert "self._transport.safe_stop()" in shutdown


def test_right_jog_connection_loss_disables_right_bus() -> None:
    service = BINARY[BINARY.index("void BinaryControl_Service(void)"):]
    assert "host_right_arm_output_active != 0U" in service
    assert "host_binary_last_heartbeat_ms" in service
    assert "RightServoBus_DisableTorqueAll()" in service
