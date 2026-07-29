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

    def test_safety_change_bumps_firmware_identity(self):
        self.assertIn(
            "HOST_BINARY_FIRMWARE_VERSION UINT32_C(0x00020B00)",
            CONFIG_HEADER,
        )


if __name__ == "__main__":
    unittest.main()
