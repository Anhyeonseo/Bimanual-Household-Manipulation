"""Shared COBS and CRC-32C framing, independent of ROS and serial I/O."""

import struct

MAGIC = 0xA55A

MAX_PAYLOAD = 512

HEADER = struct.Struct("<HBBHHII")

CRC = struct.Struct("<I")

class ProtocolError(RuntimeError):
    pass

def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = ((crc >> 1) ^ (0x82F63B78 & mask)) & 0xFFFFFFFF
    return (~crc) & 0xFFFFFFFF

def cobs_encode(data: bytes) -> bytes:
    output = bytearray(b"\x00")
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)

def cobs_decode(encoded: bytes) -> bytes:
    if not encoded:
        raise ProtocolError("empty COBS frame")
    output = bytearray()
    index = 0
    while index < len(encoded):
        code = encoded[index]
        if code == 0:
            raise ProtocolError("zero inside COBS frame")
        index += 1
        block_end = index + code - 1
        if block_end > len(encoded):
            raise ProtocolError("truncated COBS frame")
        output.extend(encoded[index:block_end])
        index = block_end
        if code != 0xFF and index < len(encoded):
            output.append(0)
    return bytes(output)
