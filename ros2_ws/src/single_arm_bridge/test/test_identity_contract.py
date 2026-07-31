import unittest

from single_arm_bridge.hardware_identity import (
    EXPECTED_FIRMWARE_VERSION,
    EXPECTED_JOINT_COUNT,
    EXPECTED_PROTOCOL_VERSION,
)


class DeployedIdentityContractTest(unittest.TestCase):
    def test_deployed_identity_contract(self) -> None:
        self.assertEqual(EXPECTED_FIRMWARE_VERSION, 0x00021800)
        self.assertEqual(EXPECTED_PROTOCOL_VERSION, 1)
        self.assertEqual(EXPECTED_JOINT_COUNT, 6)
