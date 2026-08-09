from single_arm_bridge.protocol import (
    DIAGNOSTICS_BASE,
    DIAGNOSTICS_BUS_HEALTH,
    DIAGNOSTICS_JOINT,
    parse_servo_diagnostic,
)
from single_arm_bridge.transport import ServoDiagnosticReadError


def diagnostic_payload(*, status: int = 0, read_status: int = 0) -> bytes:
    base = DIAGNOSTICS_BASE.pack(status, 0, 6, 1, 0x2D90167E, 123)
    joint_values = [1, read_status, 0, 16, 32, 0, 123, 31] + [0] * 18
    joint = DIAGNOSTICS_JOINT.pack(*joint_values)
    health = DIAGNOSTICS_BUS_HEALTH.pack(
        2, 3, 1, 0, 0, 2, 14, 37,
        0x8, 0x6210D4, 0,
        10, 9, 1, 1, 2, 1, 0, 7,
        0, 0, 0, 1, 0, 0,
        10, 1, 4, 0, bytes.fromhex("ffff0104") + bytes(12),
    )
    return base + joint + health


def test_extended_diagnostics_decodes_uart_dma_health() -> None:
    sample = parse_servo_diagnostic(diagnostic_payload())
    assert sample.bus_health is not None
    assert sample.bus_health.schema_version == 2
    assert not sample.bus_health.dma_started
    assert sample.bus_health.uart_error_code == 0x8
    assert sample.bus_health.ore_count == 1
    assert sample.bus_health.transaction_count == 10
    assert sample.bus_health.lazy_arm_count == 10
    assert sample.bus_health.receiver_resync_count == 1
    assert sample.bus_health.failure_snapshot == bytes.fromhex("ffff0104")
    assert not sample.bus_health.receiver_armed


def test_legacy_diagnostics_remains_decodable() -> None:
    payload = diagnostic_payload()[: DIAGNOSTICS_BASE.size + DIAGNOSTICS_JOINT.size]
    assert parse_servo_diagnostic(payload).bus_health is None


def test_failure_exception_includes_first_uart_dma_evidence() -> None:
    sample = parse_servo_diagnostic(diagnostic_payload(status=2, read_status=0x1F))
    error = ServoDiagnosticReadError(0, sample)
    message = str(error)
    assert "read_status=0x1F" in message
    assert "reason=uart" in message
    assert "uart_error=0x00000008" in message
    assert "pe/ne/fe/ore/rto/dma=0/0/0/1/0/0" in message
