"""Protocol and q0-record tests for the direct right-arm centering utility."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / "tools" / "right_arm_home.py"
SPEC = importlib.util.spec_from_file_location("right_arm_home", PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_read_position_packet_matches_firmware_protocol_zero() -> None:
    assert MODULE.instruction_packet(3, 0x02, bytes((56, 2))) == bytes(
        (0xFF, 0xFF, 3, 4, 2, 56, 2, 188)
    )


def test_one_key_center_packet_matches_firmware_sequence() -> None:
    assert MODULE.instruction_packet(4, 0x03, bytes((40, 128))) == bytes(
        (0xFF, 0xFF, 4, 4, 3, 40, 128, 76)
    )


def test_status_parser_rejects_wrong_checksum_and_accepts_stale_prefix() -> None:
    valid = bytes((0xFF, 0xFF, 4, 4, 0, 0, 8, 239))
    assert MODULE.parse_status_frame(b"\x12\x34" + valid, 4, 2) == bytes((0, 8))
    broken = valid[:-1] + bytes((0,))
    try:
        MODULE.parse_status_frame(broken, 4, 2)
    except MODULE.ServoProtocolError:
        pass
    else:  # pragma: no cover - assertion message is clearer than pytest.raises here
        raise AssertionError("checksum failure must be rejected")


def test_q0_record_requires_post_power_cycle_raw_2048_verification() -> None:
    positions = {servo_id: 2048 for servo_id in MODULE.SERVO_IDS}
    record = MODULE.q0_record(positions, 10)
    assert record["arm_slot"] == "right"
    assert record["raw_after_power_cycle"] == [2048] * 6
    assert record["arm_joint_positions_rad"] == [0.0] * 5
    assert "ID-to-joint mapping" in record["next_required_calibration"]
