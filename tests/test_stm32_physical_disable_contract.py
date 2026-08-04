import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SERVO_SOURCE = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Src/servo_bus.c"
).read_text()
BINARY_SOURCE = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Src/binary_control.c"
).read_text()
CONFIG_HEADER = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Inc/single_arm_config.h"
).read_text()
TRANSPORT_SOURCE = (
    ROOT / "ros2_ws/src/single_arm_bridge/single_arm_bridge/transport.py"
).read_text()


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1 : index]
    raise AssertionError(f"unterminated function: {signature}")

def hal_uart_timeouts_ms(source: str, signature: str) -> list[int]:
    definition = source.rindex(signature)
    body = function_body(source[definition:], signature)
    return [
        int(value)
        for value in re.findall(
            r"HAL_UART_(?:Transmit|Receive)\s*\(.*?(\d+)U\s*\);",
            body,
            re.DOTALL,
        )
    ]



class Stm32PhysicalDisableContractTest(unittest.TestCase):
    def test_disable_writes_and_reads_back_all_servo_torque_registers(self):
        body = function_body(
            SERVO_SOURCE,
            "HAL_StatusTypeDef Servo_DisableTorqueAll(void)",
        )
        self.assertGreaterEqual(body.count("servo_joint_count"), 2)
        self.assertRegex(
            body,
            re.compile(r"Servo_WriteData\s*\([^;]*40U", re.DOTALL),
        )
        self.assertRegex(
            body,
            re.compile(r"Servo_ReadData\s*\([^;]*40U", re.DOTALL),
        )
        self.assertIn("torque_readback[0] != 0U", body)

    def test_binary_disable_cannot_ack_before_physical_disable(self):
        case_start = BINARY_SOURCE.index("case ACTUATOR_MSG_DISABLE:")
        case_end = BINARY_SOURCE.index("default:", case_start)
        body = BINARY_SOURCE[case_start:case_end]
        physical_call = body.index("Servo_DisableTorqueAll()")
        response_call = body.index("Host_SendBinaryState(")
        self.assertLess(physical_call, response_call)
        self.assertIn("host_binary_servos_configured = 0U", body)
        self.assertIn("ACTUATOR_SAFETY_HEALTH_FAILED", body)

    def test_fault_latch_cannot_block_or_be_cleared_by_physical_disable(self):
        case_start = BINARY_SOURCE.index("case ACTUATOR_MSG_DISABLE:")
        case_end = BINARY_SOURCE.index("default:", case_start)
        body = BINARY_SOURCE[case_start:case_end]

        self.assertIn("ACTUATOR_SAFETY_OK", body)
        self.assertIn(
            "host_binary_safety.state != ACTUATOR_STATE_FAULT",
            body,
        )
        self.assertIn(
            "host_binary_safety.state != ACTUATOR_STATE_ESTOPPED",
            body,
        )
        self.assertNotIn("host_stop_latched = 0U", body)
        self.assertLess(
            body.index("Servo_DisableTorqueAll()"),
            body.index("Host_SendBinaryState("),
        )

    def test_host_disable_timeout_covers_firmware_worst_case(self):
        write_timeouts_ms = hal_uart_timeouts_ms(
            SERVO_SOURCE,
            "HAL_StatusTypeDef Servo_WriteData(",
        )
        read_timeouts_ms = hal_uart_timeouts_ms(
            SERVO_SOURCE,
            "HAL_StatusTypeDef Servo_ReadData(",
        )
        disable_body = function_body(
            SERVO_SOURCE,
            "HAL_StatusTypeDef Servo_DisableTorqueAll(void)",
        )
        delay_match = re.search(r"HAL_Delay\((\d+)U\)", disable_body)
        timeout_match = re.search(
            r"DISABLE_RESPONSE_TIMEOUT_S\s*=\s*([0-9.]+)",
            TRANSPORT_SOURCE,
        )
        read_timeout_match = re.search(
            r"SERVO_BUS_READ_TIMEOUT_MS\s+UINT32_C\((\d+)\)",
            SERVO_SOURCE,
        )
        recovery_quiet_match = re.search(
            r"SERVO_BUS_RECOVERY_QUIET_MS\s+UINT32_C\((\d+)\)",
            SERVO_SOURCE,
        )
        write_settle_match = re.search(
            r"SERVO_BUS_WRITE_REPLY_SETTLE_MS\s+UINT32_C\((\d+)\)",
            SERVO_SOURCE,
        )

        self.assertEqual(write_timeouts_ms, [100])
        self.assertEqual(read_timeouts_ms, [100])
        self.assertIsNotNone(delay_match)
        self.assertIsNotNone(timeout_match)
        self.assertIsNotNone(read_timeout_match)
        self.assertIsNotNone(recovery_quiet_match)
        self.assertIsNotNone(write_settle_match)

        firmware_worst_case_ms = (
            6 * (
                sum(write_timeouts_ms)
                + int(write_settle_match.group(1))
            )
            + int(delay_match.group(1))
            + 6 * (
                sum(read_timeouts_ms)
                + int(read_timeout_match.group(1))
                + int(recovery_quiet_match.group(1))
            )
        )
        host_timeout_ms = float(timeout_match.group(1)) * 1000.0

        self.assertEqual(firmware_worst_case_ms, 1529)
        self.assertGreaterEqual(
            host_timeout_ms,
            firmware_worst_case_ms + 500,
        )

    def test_safety_change_bumps_firmware_identity(self):
        self.assertIn(
            "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00022200)",
            CONFIG_HEADER,
        )


if __name__ == "__main__":
    unittest.main()
