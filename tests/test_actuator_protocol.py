import json
from pathlib import Path
import struct
import unittest

from tools.actuator_protocol import (
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
