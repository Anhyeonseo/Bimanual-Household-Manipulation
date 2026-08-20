import json
from pathlib import Path
import struct
import unittest

from tools.lib.actuator_protocol import (
    Frame,
    MessageType,
    ProtocolError,
    StreamDecoder,
    cobs_decode,
    cobs_encode,
    crc32c,
    decode_frame,
    encode_frame,
    parse_state_feedback,
)


class ActuatorProtocolTests(unittest.TestCase):
    def test_state_feedback_accepts_legacy_and_position_extension(self) -> None:
        base = struct.pack(
            "<BBBBIIII",
            0,
            0,
            6,
            1,
            4,
            2,
            0x4D62F8D5,
            1234,
        )
        legacy = parse_state_feedback(base)
        self.assertIsNone(legacy.raw_positions)

        extended = parse_state_feedback(
            base + struct.pack("<6H", 2048, 2049, 2047, 2050, 2046, 2048)
        )
        self.assertEqual(
            extended.raw_positions,
            (2048, 2049, 2047, 2050, 2046, 2048),
        )

        failure = parse_state_feedback(
            base
            + struct.pack(
                "<BBBBBBHH2xII",
                1,
                2,
                3,
                5,
                1,
                0,
                8,
                6,
                0x02,
                0xA0,
            )
        )
        self.assertIsNone(failure.raw_positions)
        self.assertEqual(failure.position_read_failed_servo_id, 1)
        self.assertEqual(failure.position_read_failure_reason, 5)
        self.assertEqual(failure.position_read_hal_status, 1)
        self.assertEqual(failure.position_read_recovery_count, 8)
        self.assertEqual(failure.position_read_discarded_bytes, 6)
        self.assertEqual(failure.position_read_uart_error_code, 0x02)
        self.assertEqual(failure.position_read_uart_isr, 0xA0)

        snapshot_failure = parse_state_feedback(
            base
            + struct.pack(
                "<BBBBBBHH2xIIBB16s",
                1, 1, 3, 4, 3, 0, 9, 4,
                0, 0x006010C0, 4, 0,
                bytes.fromhex("ffff0104") + bytes(12),
            )
        )
        self.assertEqual(
            snapshot_failure.position_read_snapshot,
            bytes.fromhex("ffff0104"),
        )
        self.assertFalse(snapshot_failure.position_read_receiver_armed)

    def test_state_feedback_rejects_unknown_payload_size(self) -> None:
        with self.assertRaises(ProtocolError):
            parse_state_feedback(b"\x00" * 21)

    def test_crc32c_known_vector_matches_c_core(self) -> None:
        self.assertEqual(crc32c(b"123456789"), 0xE3069283)

    def test_cobs_round_trip_including_zeroes_and_long_block(self) -> None:
        source = bytes(range(256)) + b"\x00tail"
        self.assertEqual(cobs_decode(cobs_encode(source)), source)

    def test_frame_round_trip(self) -> None:
        source = Frame(
            message_type=MessageType.HEARTBEAT,
            flags=0x0102,
            sequence=0x10203040,
            sender_time_ms=1234,
            payload=b"\x00\x01\x00\x02",
        )
        encoded = encode_frame(source)
        self.assertEqual(encoded[-1], 0)
        self.assertEqual(decode_frame(encoded), source)

    def test_crc_bit_flip_is_rejected(self) -> None:
        encoded = bytearray(
            encode_frame(Frame(message_type=MessageType.HEARTBEAT, sequence=1))
        )
        encoded[len(encoded) // 2] ^= 1
        with self.assertRaises(ProtocolError):
            decode_frame(bytes(encoded))

    def test_stream_decoder_resynchronizes_on_delimiter(self) -> None:
        source = Frame(message_type=MessageType.GET_STATE, sequence=7)
        decoder = StreamDecoder()
        result = None
        for byte in b"noise\x00" + encode_frame(source):
            try:
                candidate = decoder.push(byte)
            except ProtocolError:
                candidate = None
            if candidate is not None:
                result = candidate
        self.assertEqual(result, source)

    def test_message_enum_matches_manifest(self) -> None:
        manifest_path = Path("protocol/message_ids.json")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        expected = {message["name"]: message["id"] for message in manifest["messages"]}
        actual = {message.name: int(message) for message in MessageType}
        self.assertEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
