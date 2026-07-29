"""Fail-closed identity checks for the verified STM32 firmware."""

from __future__ import annotations

from .protocol import Hello


EXPECTED_FIRMWARE_VERSION = 0x00020B00
EXPECTED_PROTOCOL_VERSION = 1
EXPECTED_JOINT_COUNT = 6
POSITION_FEEDBACK_CAPABILITY = 0x00000008


class HardwareIdentityError(RuntimeError):
    """Raised before ARM/ENABLE when the connected device is not verified."""


def validate_hardware_identity(
    hello: Hello,
    expected_calibration_hash: int,
) -> None:
    if hello.firmware_version != EXPECTED_FIRMWARE_VERSION:
        raise HardwareIdentityError(
            "firmware version mismatch: "
            f"expected=0x{EXPECTED_FIRMWARE_VERSION:08X} "
            f"actual=0x{hello.firmware_version:08X}"
        )
    if hello.protocol_version != EXPECTED_PROTOCOL_VERSION:
        raise HardwareIdentityError(
            "protocol version mismatch: "
            f"expected={EXPECTED_PROTOCOL_VERSION} "
            f"actual={hello.protocol_version}"
        )
    if hello.joint_count != EXPECTED_JOINT_COUNT:
        raise HardwareIdentityError(
            "joint count mismatch: "
            f"expected={EXPECTED_JOINT_COUNT} actual={hello.joint_count}"
        )
    if (hello.capabilities & POSITION_FEEDBACK_CAPABILITY) == 0:
        raise HardwareIdentityError("position feedback capability is missing")
    if hello.calibration_hash != expected_calibration_hash:
        raise HardwareIdentityError(
            "calibration hash mismatch: "
            f"expected=0x{expected_calibration_hash:08X} "
            f"actual=0x{hello.calibration_hash:08X}"
        )
