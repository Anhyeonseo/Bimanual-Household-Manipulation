"""Non-motion smoke test for the STM32 actuator binary protocol."""

from __future__ import annotations

import argparse
from pathlib import Path
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.actuator_protocol import (
    Frame,
    MessageType,
    cobs_decode,
    cobs_encode,
    decode_frame,
    encode_frame,
    parse_state_feedback,
)


HELLO_PAYLOAD = struct.Struct("<BBBBIIII")
STATE_PAYLOAD = struct.Struct("<BBBBIIII")
ARM_RESPONSE_PAYLOAD = struct.Struct("<BB2xI")
SETPOINT_STATUS_PAYLOAD = struct.Struct("<BBBBIII")


def read_binary_frame(port: object, stage: str) -> Frame:
    packet = port.read_until(b"\x00")
    if not packet.endswith(b"\x00"):
        raise TimeoutError(
            f"{stage}: timed out waiting for a 0x00 frame delimiter; "
            f"raw={packet.hex()}"
        )
    try:
        return decode_frame(packet)
    except Exception as error:
        raise RuntimeError(
            f"{stage}: invalid binary frame ({error}); raw={packet.hex()}"
        ) from error


def send_request(
    port: object,
    message_type: MessageType,
    sequence: int,
    payload: bytes = b"",
    flags: int = 0,
) -> None:
    now_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
    port.write(
        encode_frame(
            Frame(
                message_type=message_type,
                flags=flags,
                sequence=sequence,
                sender_time_ms=now_ms,
                payload=payload,
            )
        )
    )
    port.flush()


def clear_stop(port: object, first_sequence: int, stage: str) -> int:
    time.sleep(0.1)
    clear_status = None
    cleared_values = None

    for clear_attempt in range(3):
        sequence = first_sequence + clear_attempt
        send_request(port, MessageType.CLEAR_FAULT, sequence)
        cleared = read_binary_frame(
            port,
            f"{stage}_CLEAR_ATTEMPT_{clear_attempt + 1}",
        )
        if cleared.message_type is not MessageType.STATE_FEEDBACK:
            raise RuntimeError("CLEAR_FAULT did not return STATE_FEEDBACK")
        cleared_values = STATE_PAYLOAD.unpack(cleared.payload)
        clear_status = cleared_values[1]
        if cleared_values[0] == 0 and clear_status == 0:
            return sequence + 1
        if clear_status != 2:
            break
        time.sleep(0.1)

    assert cleared_values is not None
    raise RuntimeError(
        f"{stage} remained latched after the position safety check: "
        f"latched={cleared_values[0]} status={clear_status} "
        "(2=servo read failure, 3=position outside safe range)"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify HELLO, CRC rejection, HEARTBEAT, and GET_STATE without motion."
    )
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument(
        "--safe-stop",
        action="store_true",
        help="also verify SAFE_STOP latch and CLEAR_FAULT recovery without motion",
    )
    parser.add_argument(
        "--heartbeat-loss",
        action="store_true",
        help="also verify automatic stop after a 500 ms heartbeat timeout",
    )
    parser.add_argument(
        "--setpoint-validate-home",
        action="store_true",
        help="ARM with the calibration hash and validate a zero-urad home setpoint without motion",
    )
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print(
            "pyserial is missing. Run: .\\.venv-host\\Scripts\\python.exe "
            "-m pip install pyserial",
            file=sys.stderr,
        )
        return 2

    try:
        with serial.Serial(args.port, args.baud, timeout=args.timeout) as port:
            port.reset_input_buffer()
            port.write(b"P")
            port.flush()
            ready = port.readline().decode("ascii", errors="replace").strip()
            if ready != "BINARY_PROTOCOL_READY_RESET_TO_EXIT":
                # A previous host may already have selected binary mode. End
                # the partial ASCII byte as a rejected COBS packet, then prove
                # the current mode with the HELLO exchange below.
                port.write(b"\x00")
                port.flush()
                port.reset_input_buffer()

            send_request(port, MessageType.HELLO_REQUEST, 1)
            hello = read_binary_frame(port, "HELLO_RESPONSE")
            if hello.message_type is not MessageType.HELLO_RESPONSE:
                raise RuntimeError(f"unexpected HELLO response: {hello.message_type.name}")
            if len(hello.payload) != HELLO_PAYLOAD.size:
                raise RuntimeError("unexpected HELLO payload size")

            (
                protocol_version,
                joint_count,
                stop_latched,
                _,
                firmware_version,
                calibration_hash,
                capabilities,
                rejected_before,
            ) = HELLO_PAYLOAD.unpack(hello.payload)

            valid_heartbeat = encode_frame(
                Frame(
                    message_type=MessageType.HEARTBEAT,
                    sequence=2,
                    sender_time_ms=int(time.monotonic() * 1000) & 0xFFFFFFFF,
                )
            )
            decoded = bytearray(cobs_decode(valid_heartbeat[:-1]))
            decoded[-1] ^= 0x01
            port.write(cobs_encode(bytes(decoded)) + b"\x00")
            port.flush()
            time.sleep(0.02)

            send_request(port, MessageType.HEARTBEAT, 3)
            time.sleep(0.02)
            send_request(port, MessageType.GET_STATE, 4)
            state = read_binary_frame(port, "GET_STATE")
            if state.message_type is not MessageType.STATE_FEEDBACK:
                raise RuntimeError(f"unexpected state response: {state.message_type.name}")
            if len(state.payload) != STATE_PAYLOAD.size:
                raise RuntimeError("unexpected STATE payload size")

            (
                state_stop_latched,
                status_code,
                state_joint_count,
                state_protocol_version,
                heartbeat_count,
                rejected_after,
                state_calibration_hash,
                last_heartbeat_ms,
            ) = STATE_PAYLOAD.unpack(state.payload)

            if protocol_version != 1 or state_protocol_version != 1:
                raise RuntimeError("protocol version mismatch")
            if joint_count != 6 or state_joint_count != 6:
                raise RuntimeError("joint count mismatch")
            if status_code != 0:
                raise RuntimeError(f"STM32 state status is {status_code}")
            if heartbeat_count < 1:
                raise RuntimeError("valid heartbeat was not counted")
            if rejected_after <= rejected_before:
                raise RuntimeError("corrupted CRC frame was not rejected")
            if state_calibration_hash != calibration_hash:
                raise RuntimeError("calibration hash changed during the test")

            next_sequence = 5
            heartbeat_loss_tested = False
            if args.heartbeat_loss:
                send_request(
                    port,
                    MessageType.ARM_REQUEST,
                    next_sequence,
                    struct.pack("<I", calibration_hash),
                )
                timeout_arm = read_binary_frame(port, "HEARTBEAT_TIMEOUT_ARM")
                next_sequence += 1
                arm_result, arm_state, arm_hash = ARM_RESPONSE_PAYLOAD.unpack(
                    timeout_arm.payload
                )
                if arm_result != 0 or arm_state != 2 or arm_hash != calibration_hash:
                    raise RuntimeError(
                        "heartbeat timeout ARM rejected: "
                        f"result={arm_result} state={arm_state}"
                    )

                send_request(port, MessageType.HEARTBEAT, next_sequence)
                next_sequence += 1
                send_request(port, MessageType.ENABLE, next_sequence)
                timeout_enabled = read_binary_frame(port, "HEARTBEAT_TIMEOUT_ENABLE")
                next_sequence += 1
                if STATE_PAYLOAD.unpack(timeout_enabled.payload)[1] != 0:
                    raise RuntimeError("heartbeat timeout ENABLE rejected")

                time.sleep(0.75)
                send_request(port, MessageType.GET_STATE, next_sequence)
                timed_out = read_binary_frame(port, "HEARTBEAT_TIMEOUT")
                next_sequence += 1
                if timed_out.message_type is not MessageType.STATE_FEEDBACK:
                    raise RuntimeError("heartbeat timeout did not return STATE_FEEDBACK")
                timed_out_values = STATE_PAYLOAD.unpack(timed_out.payload)
                if timed_out_values[0] != 1 or timed_out_values[1] != 0:
                    raise RuntimeError(
                        "heartbeat loss did not latch the stop: "
                        f"latched={timed_out_values[0]} status={timed_out_values[1]}"
                    )

                send_request(port, MessageType.HEARTBEAT, next_sequence)
                next_sequence += 1
                next_sequence = clear_stop(
                    port,
                    next_sequence,
                    "HEARTBEAT_TIMEOUT",
                )
                heartbeat_loss_tested = True

            safe_stop_tested = False
            if args.safe_stop:
                send_request(port, MessageType.SAFE_STOP, next_sequence)
                stopped = read_binary_frame(port, "SAFE_STOP")
                next_sequence += 1
                if stopped.message_type is not MessageType.STATE_FEEDBACK:
                    raise RuntimeError("SAFE_STOP did not return STATE_FEEDBACK")
                stopped_values = STATE_PAYLOAD.unpack(stopped.payload)
                if stopped_values[0] != 1 or stopped_values[1] != 0:
                    raise RuntimeError(
                        "SAFE_STOP was not latched cleanly: "
                        f"latched={stopped_values[0]} status={stopped_values[1]}"
                    )

                next_sequence = clear_stop(port, next_sequence, "SAFE_STOP")
                safe_stop_tested = True

            setpoint_validation_tested = False
            if args.setpoint_validate_home:
                send_request(
                    port,
                    MessageType.ARM_REQUEST,
                    next_sequence,
                    struct.pack("<I", calibration_hash),
                )
                armed = read_binary_frame(port, "ARM_REQUEST")
                next_sequence += 1
                if armed.message_type is not MessageType.ARM_RESPONSE:
                    raise RuntimeError("ARM_REQUEST did not return ARM_RESPONSE")
                arm_result, arm_state, arm_hash = ARM_RESPONSE_PAYLOAD.unpack(
                    armed.payload
                )
                if arm_result != 0 or arm_state != 2 or arm_hash != calibration_hash:
                    raise RuntimeError(
                        "ARM_REQUEST rejected: "
                        f"result={arm_result} state={arm_state} "
                        f"hash=0x{arm_hash:08X}"
                    )

                send_request(port, MessageType.HEARTBEAT, next_sequence)
                next_sequence += 1
                send_request(port, MessageType.ENABLE, next_sequence)
                enabled = read_binary_frame(port, "ENABLE")
                next_sequence += 1
                enabled_values = STATE_PAYLOAD.unpack(enabled.payload)
                if enabled_values[0] != 0 or enabled_values[1] != 0:
                    raise RuntimeError(
                        "ENABLE rejected: "
                        f"latched={enabled_values[0]} status={enabled_values[1]}"
                    )

                send_request(port, MessageType.HEARTBEAT, next_sequence)
                next_sequence += 1
                send_request(port, MessageType.GET_STATE, next_sequence)
                active_state = read_binary_frame(port, "ACTIVE_GET_STATE")
                next_sequence += 1
                active_values = STATE_PAYLOAD.unpack(active_state.payload)
                first_apply_tick = (active_values[7] + 300) & 0xFFFFFFFF

                setpoint_payload = struct.pack(
                    "<IBBH" + "I" + "i" * 12,
                    first_apply_tick,
                    1,
                    1,
                    0,
                    0,
                    *([0] * 12),
                )
                send_request(
                    port,
                    MessageType.SETPOINT_BATCH,
                    next_sequence,
                    setpoint_payload,
                    flags=1,
                )
                setpoint_status = read_binary_frame(port, "SETPOINT_BATCH_HOME")
                next_sequence += 1
                if setpoint_status.message_type is not MessageType.SETPOINT_STATUS:
                    raise RuntimeError("SETPOINT_BATCH did not return SETPOINT_STATUS")
                (
                    setpoint_result,
                    accepted_samples,
                    setpoint_state,
                    _,
                    _,
                    returned_apply_tick,
                    setpoint_hash,
                ) = SETPOINT_STATUS_PAYLOAD.unpack(setpoint_status.payload)
                if (
                    setpoint_result != 5
                    or accepted_samples != 1
                    or setpoint_state != 3
                    or returned_apply_tick != first_apply_tick
                    or setpoint_hash != calibration_hash
                ):
                    raise RuntimeError(
                        "home setpoint validation failed: "
                        f"status={setpoint_result} samples={accepted_samples} "
                        f"state={setpoint_state} apply_tick={returned_apply_tick}"
                    )

                send_request(port, MessageType.DISABLE, next_sequence)
                disabled = read_binary_frame(port, "DISABLE")
                next_sequence += 1
                disabled_values = STATE_PAYLOAD.unpack(disabled.payload)
                if disabled_values[1] != 0:
                    raise RuntimeError(
                        f"DISABLE rejected: status={disabled_values[1]}"
                    )
                setpoint_validation_tested = True

            position_feedback = None
            if (capabilities & 0x00000008) != 0:
                send_request(
                    port,
                    MessageType.GET_STATE,
                    next_sequence,
                    payload=b"\x01",
                )
                position_state = read_binary_frame(port, "GET_STATE_POSITIONS")
                next_sequence += 1
                if position_state.message_type is not MessageType.STATE_FEEDBACK:
                    raise RuntimeError(
                        "position GET_STATE did not return STATE_FEEDBACK"
                    )
                parsed_position_state = parse_state_feedback(position_state.payload)
                if (
                    parsed_position_state.status_code != 0
                    or parsed_position_state.raw_positions is None
                    or len(parsed_position_state.raw_positions) != 6
                ):
                    raise RuntimeError(
                        "position feedback missing or invalid: "
                        f"status={parsed_position_state.status_code}"
                    )
                position_feedback = parsed_position_state.raw_positions

            print("BINARY_SMOKE_OK")
            print(f"PROTOCOL_VERSION={protocol_version}")
            print(f"JOINT_COUNT={joint_count}")
            print(f"FIRMWARE_VERSION=0x{firmware_version:08X}")
            print(f"CALIBRATION_HASH=0x{calibration_hash:08X}")
            print(f"CAPABILITIES=0x{capabilities:08X}")
            print(f"STOP_LATCHED={state_stop_latched or stop_latched}")
            print(f"HEARTBEAT_COUNT={heartbeat_count}")
            print(f"CRC_REJECT_COUNT={rejected_after}")
            print(f"LAST_HEARTBEAT_MS={last_heartbeat_ms}")
            if safe_stop_tested:
                print("SAFE_STOP_LATCH_CLEAR=OK")
            if heartbeat_loss_tested:
                print("HEARTBEAT_TIMEOUT_LATCH_CLEAR=OK")
            if setpoint_validation_tested:
                print("SETPOINT_HOME_VALIDATION_ONLY=OK")
            if position_feedback is not None:
                print(f"RAW_POSITION_FEEDBACK={list(position_feedback)}")
            return 0
    except Exception as error:
        print(f"BINARY_SMOKE_FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
