"""Fail-closed identity checks for the verified STM32 firmware."""

from __future__ import annotations

from .protocol import Hello


EXPECTED_FIRMWARE_VERSION = 0x00022F00
F0_METRICS_CANDIDATE_FIRMWARE_VERSION = 0x00023000
F2_ASYNC_HOST_TX_CANDIDATE_FIRMWARE_VERSION = 0x00023100
F1_HEARTBEAT_RX_TIMESTAMP_CANDIDATE_FIRMWARE_VERSION = 0x00023200
H2_IN_MOTION_TELEMETRY_CANDIDATE_FIRMWARE_VERSION = 0x00023300
H2_TELEMETRY_TIMING_CANDIDATE_FIRMWARE_VERSION = 0x00023400
SUPPORTED_FIRMWARE_VERSIONS = frozenset(
    {
        EXPECTED_FIRMWARE_VERSION,
        F0_METRICS_CANDIDATE_FIRMWARE_VERSION,
        F2_ASYNC_HOST_TX_CANDIDATE_FIRMWARE_VERSION,
        F1_HEARTBEAT_RX_TIMESTAMP_CANDIDATE_FIRMWARE_VERSION,
        H2_IN_MOTION_TELEMETRY_CANDIDATE_FIRMWARE_VERSION,
        H2_TELEMETRY_TIMING_CANDIDATE_FIRMWARE_VERSION,
    }
)
EXPECTED_PROTOCOL_VERSION = 1
EXPECTED_JOINT_COUNT = 6
POSITION_FEEDBACK_CAPABILITY = 0x00000008
SERVO_DIAGNOSTICS_CAPABILITY = 0x00000010
ACKNOWLEDGED_HEARTBEAT_CAPABILITY = 0x00000020
BUFFERED_HOST_RX_CAPABILITY = 0x00000040
SERVO_COMMAND_CONFIGURATION_DIAGNOSTICS_CAPABILITY = 0x00000080
POSITION_READ_FAILURE_DIAGNOSTICS_CAPABILITY = 0x00000100
SERVO_BUS_RECOVERY_DIAGNOSTICS_CAPABILITY = 0x00000200
BUFFERED_VALIDATION_ROUTE_CAPABILITY = 0x00000400
BUFFERED_EXECUTION_ROUTE_CAPABILITY = 0x00000800
F0_METRICS_CAPABILITY = 0x00001000
F2_ASYNC_HOST_TX_CAPABILITY = 0x00002000
F1_HEARTBEAT_RX_TIMESTAMP_CAPABILITY = 0x00004000
H2_IN_MOTION_TELEMETRY_CAPABILITY = 0x00008000


class HardwareIdentityError(RuntimeError):
    """Raised before ARM/ENABLE when the connected device is not verified."""


def validate_hardware_identity(
    hello: Hello,
    expected_calibration_hash: int,
) -> None:
    if hello.firmware_version not in SUPPORTED_FIRMWARE_VERSIONS:
        expected = ",".join(
            f"0x{version:08X}" for version in sorted(SUPPORTED_FIRMWARE_VERSIONS)
        )
        raise HardwareIdentityError(
            "firmware version mismatch: "
            f"expected one of={expected} "
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
    if (hello.capabilities & SERVO_DIAGNOSTICS_CAPABILITY) == 0:
        raise HardwareIdentityError("servo diagnostics capability is missing")
    if (hello.capabilities & ACKNOWLEDGED_HEARTBEAT_CAPABILITY) == 0:
        raise HardwareIdentityError("acknowledged heartbeat capability is missing")
    if (hello.capabilities & BUFFERED_HOST_RX_CAPABILITY) == 0:
        raise HardwareIdentityError("interrupt-buffered host RX capability is missing")
    if (
        hello.capabilities
        & SERVO_COMMAND_CONFIGURATION_DIAGNOSTICS_CAPABILITY
    ) == 0:
        raise HardwareIdentityError(
            "servo command/configuration diagnostics capability is missing"
        )
    if (
        hello.capabilities
        & POSITION_READ_FAILURE_DIAGNOSTICS_CAPABILITY
    ) == 0:
        raise HardwareIdentityError(
            "position read failure diagnostics capability is missing"
        )
    if (
        hello.capabilities
        & SERVO_BUS_RECOVERY_DIAGNOSTICS_CAPABILITY
    ) == 0:
        raise HardwareIdentityError(
            "servo bus recovery diagnostics capability is missing"
        )
    if (hello.capabilities & BUFFERED_VALIDATION_ROUTE_CAPABILITY) == 0:
        raise HardwareIdentityError(
            "buffered validation route capability is missing"
        )
    if (hello.capabilities & BUFFERED_EXECUTION_ROUTE_CAPABILITY) == 0:
        raise HardwareIdentityError(
            "buffered execution route capability is missing"
        )
    if (
        hello.firmware_version
        == F1_HEARTBEAT_RX_TIMESTAMP_CANDIDATE_FIRMWARE_VERSION
        and (
            hello.capabilities & F1_HEARTBEAT_RX_TIMESTAMP_CAPABILITY
        )
        == 0
    ):
        raise HardwareIdentityError(
            "heartbeat RX timestamp capability is missing"
        )
    if (
        hello.firmware_version in (
            H2_IN_MOTION_TELEMETRY_CANDIDATE_FIRMWARE_VERSION,
            H2_TELEMETRY_TIMING_CANDIDATE_FIRMWARE_VERSION,
        )
        and (hello.capabilities & H2_IN_MOTION_TELEMETRY_CAPABILITY) == 0
    ):
        raise HardwareIdentityError(
            "in-motion telemetry capability is missing"
        )
    if hello.calibration_hash != expected_calibration_hash:
        raise HardwareIdentityError(
            "calibration hash mismatch: "
            f"expected=0x{expected_calibration_hash:08X} "
            f"actual=0x{hello.calibration_hash:08X}"
        )
