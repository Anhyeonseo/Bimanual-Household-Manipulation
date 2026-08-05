import json
import struct
import sys
import tempfile
import time
import unittest
from pathlib import Path


PACKAGE_ROOT = Path("ros2_ws/src/single_arm_bridge")
sys.path.insert(0, str(PACKAGE_ROOT))

from single_arm_bridge.calibration import load_calibration  # noqa: E402
from single_arm_bridge.buffered_transport_driver import (  # noqa: E402
    BufferedOutboundCommand,
)
from single_arm_bridge.device_discovery import resolve_serial_device  # noqa: E402
from single_arm_bridge.protocol import (  # noqa: E402
    BufferedSetpointFlags,
    BufferedSetpointSample,
    Frame,
    MessageType,
    decode_frame,
    encode_frame,
    encode_buffered_setpoint_payload,
)
from single_arm_bridge.transport import (  # noqa: E402
    ActuatorTransport,
    HEARTBEAT_RESPONSE_TIMEOUT_S,
    POSITION_STATE_RESPONSE_TIMEOUT_S,
    PositionReadError,
    StateResponseDeferred,
)


CALIBRATION_PATH = PACKAGE_ROOT / "config" / "single_arm_calibration.json"


class FakeSerial:
    def __init__(
        self,
        already_binary: bool = False,
        include_async_result: bool = False,
        async_result_delay_s: float = 0.0,
        drop_state_after_async_result: bool = False,
        terminal_before_safe_stop_ack: bool = False,
        position_failure_streak: int = 0,
        position_failure_servo_id: int = 0,
    ) -> None:
        self._responses: list[bytes] = []
        self._ascii_response = b""
        self._already_binary = already_binary
        self._include_async_result = include_async_result
        self._async_result_delay_s = async_result_delay_s
        self._drop_state_after_async_result = drop_state_after_async_result
        self._terminal_before_safe_stop_ack = terminal_before_safe_stop_ack
        self._position_failure_streak = position_failure_streak
        self._position_failure_servo_id = position_failure_servo_id
        self._delay_next_async_result = False
        self._async_result_sent = False
        self.get_state_request_count = 0
        self.heartbeat_request_count = 0
        self.disable_request_count = 0
        self.buffered_accepted_samples = 0

    @property
    def in_waiting(self) -> int:
        return sum(len(response) for response in self._responses)

    def queue_terminal_motion_result(self) -> None:
        self._responses.append(
            encode_frame(
                Frame(
                    message_type=MessageType.SETPOINT_STATUS,
                    sequence=77,
                    sender_time_ms=1200,
                    payload=struct.pack(
                        "<BBBBIII",
                        6,
                        1,
                        3,
                        4,
                        77,
                        1200,
                        0x8AD27897,
                    ),
                )
            )
        )

    def queue_buffered_terminal_motion_result(self) -> None:
        self._responses.append(
            encode_frame(
                Frame(
                    message_type=MessageType.SETPOINT_STATUS,
                    sequence=78,
                    sender_time_ms=1200,
                    payload=struct.pack(
                        "<BBBBIII" "BBBBHHII",
                        6,
                        0,
                        3,
                        0,
                        78,
                        1200,
                        0x8AD27897,
                        3,
                        0,
                        0,
                        0,
                        0,
                        16,
                        16,
                        16,
                    ),
                )
            )
        )

    def reset_input_buffer(self) -> None:
        self._responses.clear()

    def flush(self) -> None:
        pass

    def write(self, data: bytes) -> int:
        if data == b"P":
            if not self._already_binary:
                self._ascii_response = b"BINARY_PROTOCOL_READY_RESET_TO_EXIT\r\n"
            return 1
        if data == b"\x00":
            return 1

        request = decode_frame(data)
        if request.message_type is MessageType.HELLO_REQUEST:
            payload = struct.pack(
                "<BBBBIIII",
                1,
                6,
                0,
                0,
                0x00022500,
                0x8AD27897,
                0x00000FFF,
                0,
            )
            response_type = MessageType.HELLO_RESPONSE
        elif request.message_type is MessageType.HEARTBEAT:
            self.heartbeat_request_count += 1
            payload = struct.pack(
                "<BBBBIIII",
                0,
                0,
                6,
                1,
                3,
                self.heartbeat_request_count,
                0x8AD27897,
                1200,
            )
            response_type = MessageType.STATE_FEEDBACK
        elif (
            request.message_type is MessageType.GET_STATE
            and len(request.payload) == 2
            and request.payload[0] == 2
        ):
            joint_index = request.payload[1]
            torque_limits = (400, 780, 650, 400, 250, 150)
            p_gains = (16, 32, 24, 16, 16, 16)
            payload = struct.pack(
                "<BBBBII8B7H2B2H2BH4B",
                0,
                joint_index,
                6,
                1,
                0x8AD27897,
                1200 + joint_index,
                joint_index + 1,
                0,
                1,
                p_gains[joint_index],
                32,
                0,
                120,
                30,
                2048 + joint_index,
                0,
                100 + joint_index,
                20 + joint_index,
                torque_limits[joint_index],
                2050 + joint_index,
                777,
                2,
                55,
                1000,
                20,
                1,
                1,
                500,
                0,
                20,
                100,
                100,
            )
            response_type = MessageType.DIAGNOSTICS
        elif request.message_type is MessageType.GET_STATE:
            self.get_state_request_count += 1
            emit_async_result = (
                self._include_async_result
                and not self._async_result_sent
            )
            if emit_async_result:
                self._responses.append(
                    encode_frame(
                        Frame(
                            message_type=MessageType.SETPOINT_STATUS,
                            sequence=77,
                            sender_time_ms=1200,
                            payload=struct.pack(
                                "<BBBBIII",
                                6,
                                1,
                                3,
                                4,
                                77,
                                1200,
                                0x8AD27897,
                            ),
                        )
                    )
                )
                self._async_result_sent = True
            self._delay_next_async_result = emit_async_result
            if emit_async_result and self._drop_state_after_async_result:
                return len(data)
            payload = struct.pack(
                "<BBBBIIII6H",
                0,
                0,
                6,
                1,
                3,
                0,
                0x8AD27897,
                1200,
                2048,
                2050,
                2046,
                2047,
                2051,
                2045,
            )
            if self._position_failure_streak:
                payload = struct.pack(
                    "<BBBBIIIIBBBB",
                    int(self._position_failure_streak >= 3),
                    2,
                    6,
                    1,
                    3,
                    0,
                    0x8AD27897,
                    1200,
                    self._position_failure_servo_id,
                    self._position_failure_streak,
                    3,
                    0,
                )
            response_type = MessageType.STATE_FEEDBACK
        elif request.message_type is MessageType.DISABLE:
            self.disable_request_count += 1
            payload = struct.pack(
                "<BBBBIIII",
                0,
                0,
                6,
                1,
                1,
                0,
                0x8AD27897,
                1200,
            )
            response_type = MessageType.STATE_FEEDBACK
        elif request.message_type is MessageType.SETPOINT_BATCH:
            apply_tick = struct.unpack_from("<I", request.payload)[0]
            if request.flags & int(BufferedSetpointFlags.VALIDATION_ONLY):
                payload = struct.pack(
                    "<BBBBIII" "BBBBHHII",
                    5,
                    request.payload[4],
                    3,
                    0,
                    request.sequence,
                    apply_tick,
                    0x8AD27897,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            elif request.flags & int(BufferedSetpointFlags.CANDIDATE):
                sample_count = request.payload[4]
                self.buffered_accepted_samples += sample_count
                executor_state = (
                    1
                    if request.flags & int(BufferedSetpointFlags.START)
                    else 0
                )
                payload = struct.pack(
                    "<BBBBIII" "BBBBHHII",
                    0,
                    sample_count,
                    3,
                    0,
                    request.sequence,
                    apply_tick,
                    0x8AD27897,
                    executor_state,
                    0,
                    0,
                    0,
                    self.buffered_accepted_samples,
                    self.buffered_accepted_samples,
                    self.buffered_accepted_samples,
                    0,
                )
            else:
                payload = struct.pack(
                    "<BBBBIII",
                    0,
                    1,
                    3,
                    0,
                    request.sequence,
                    apply_tick,
                    0x8AD27897,
                )
            response_type = MessageType.SETPOINT_STATUS
        elif request.message_type is MessageType.SAFE_STOP:
            if self._terminal_before_safe_stop_ack:
                self.queue_terminal_motion_result()
            payload = struct.pack(
                "<BBBBIIII",
                1,
                0,
                6,
                1,
                4,
                0,
                0x8AD27897,
                1200,
            )
            response_type = MessageType.STATE_FEEDBACK
        elif request.message_type is MessageType.CLEAR_FAULT:
            payload = struct.pack(
                "<BBBBIIII",
                0,
                0,
                6,
                1,
                4,
                0,
                0x8AD27897,
                1200,
            )
            response_type = MessageType.STATE_FEEDBACK
        else:
            return len(data)

        self._responses.append(
            encode_frame(
                Frame(
                    message_type=response_type,
                    sequence=request.sequence,
                    sender_time_ms=1201,
                    payload=payload,
                )
            )
        )
        return len(data)

    def readline(self) -> bytes:
        response = self._ascii_response
        self._ascii_response = b""
        return response

    def read_until(self, delimiter: bytes) -> bytes:
        del delimiter
        if self._delay_next_async_result:
            self._delay_next_async_result = False
            time.sleep(self._async_result_delay_s)
        return self._responses.pop(0) if self._responses else b""


class SingleArmBridgeCoreTests(unittest.TestCase):
    def test_explicit_serial_device_is_preserved(self) -> None:
        self.assertEqual(resolve_serial_device("COM3"), "COM3")

    def test_auto_serial_device_uses_unique_stlink_by_id(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            by_id = Path(directory)
            device = by_id / "usb-STMicroelectronics_STLINK-V3_TEST-if02"
            device.touch()
            self.assertEqual(
                resolve_serial_device(
                    "auto",
                    by_id_directory=by_id,
                    fallback_device=by_id / "missing",
                ),
                str(device),
            )

    def test_auto_serial_device_rejects_ambiguous_stlinks(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            by_id = Path(directory)
            for serial in ("A", "B"):
                (by_id / f"usb-STMicroelectronics_STLINK-V3_{serial}-if02").touch()
            with self.assertRaisesRegex(RuntimeError, "multiple ST-LINK"):
                resolve_serial_device(
                    "auto",
                    by_id_directory=by_id,
                    fallback_device=by_id / "missing",
                )

    def test_auto_serial_device_falls_back_to_ttyacm0(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fallback = root / "ttyACM0"
            fallback.touch()
            self.assertEqual(
                resolve_serial_device(
                    "auto",
                    by_id_directory=root / "missing",
                    fallback_device=fallback,
                ),
                str(fallback),
            )

    def test_packaged_calibration_matches_repository_source(self) -> None:
        source = json.loads(
            Path("config/single_arm_calibration.json").read_text(encoding="utf-8")
        )
        packaged = json.loads(CALIBRATION_PATH.read_text(encoding="utf-8"))
        self.assertEqual(packaged["joints"], source["joints"])
        self.assertEqual(packaged["arm_slot"], source["arm_slot"])

    def test_calibration_hash_and_feedback_conversion(self) -> None:
        calibration = load_calibration(CALIBRATION_PATH)
        self.assertEqual(calibration.calibration_hash, 0x8AD27897)
        radians = calibration.raw_feedback_to_radians(
            (2048, 2048, 2048, 2048, 2048, 2048)
        )
        self.assertEqual(radians, [0.0] * 6)

    def test_transport_enters_binary_mode_and_reads_positions(self) -> None:
        transport = ActuatorTransport(FakeSerial(), response_timeout_s=0.01)
        hello = transport.enter_binary_mode()
        self.assertEqual(hello.firmware_version, 0x00022500)
        state = transport.get_state(include_positions=True)
        self.assertEqual(
            state.raw_positions,
            (2048, 2050, 2046, 2047, 2051, 2045),
        )

    def test_transport_reports_transient_position_read_axis_and_streak(self) -> None:
        transport = ActuatorTransport(
            FakeSerial(position_failure_streak=1, position_failure_servo_id=3),
            response_timeout_s=0.01,
        )
        transport.enter_binary_mode()

        with self.assertRaises(PositionReadError) as raised:
            transport.get_state(include_positions=True)

        self.assertEqual(raised.exception.servo_id, 3)
        self.assertEqual(raised.exception.streak, 1)
        self.assertEqual(raised.exception.limit, 3)
        self.assertFalse(raised.exception.stop_latched)

    def test_transport_reports_third_position_read_failure_as_latched(self) -> None:
        transport = ActuatorTransport(
            FakeSerial(position_failure_streak=3, position_failure_servo_id=2),
            response_timeout_s=0.01,
        )
        transport.enter_binary_mode()

        with self.assertRaises(PositionReadError) as raised:
            transport.get_state(include_positions=True)

        self.assertEqual(raised.exception.servo_id, 2)
        self.assertEqual(raised.exception.streak, 3)
        self.assertTrue(raised.exception.stop_latched)

    def test_transport_reads_on_demand_servo_diagnostics(self) -> None:
        transport = ActuatorTransport(FakeSerial(), response_timeout_s=0.01)
        transport.enter_binary_mode()
        snapshot = transport.get_diagnostics()

        self.assertEqual(snapshot.joint_count, 6)
        self.assertEqual(snapshot.calibration_hash, 0x8AD27897)
        self.assertEqual(len(snapshot.joints), 6)
        shoulder = snapshot.joints[1]
        self.assertEqual(shoulder.servo_id, 2)
        self.assertTrue(shoulder.torque_enabled)
        self.assertEqual(shoulder.p_gain, 32)
        self.assertEqual(shoulder.d_gain, 32)
        self.assertEqual(shoulder.position_raw, 2049)
        self.assertEqual(shoulder.load_raw, 101)
        self.assertEqual(shoulder.current_raw, 21)
        self.assertEqual(shoulder.voltage_raw, 120)
        self.assertEqual(shoulder.torque_limit_raw, 780)
        self.assertEqual(shoulder.goal_position_raw, 2051)
        self.assertEqual(shoulder.model_number, 777)
        self.assertEqual(shoulder.firmware_major_version, 2)
        self.assertEqual(shoulder.firmware_minor_version, 55)
        self.assertEqual(shoulder.maximum_torque_limit_raw, 1000)
        self.assertEqual(shoulder.minimum_startup_force_raw, 20)
        self.assertEqual(shoulder.cw_dead_zone_raw, 1)
        self.assertEqual(shoulder.ccw_dead_zone_raw, 1)
        self.assertEqual(shoulder.protection_current_raw, 500)
        self.assertEqual(shoulder.operating_mode, 0)
        self.assertEqual(shoulder.protective_torque_raw, 20)
        self.assertEqual(shoulder.protection_time_raw, 100)
        self.assertEqual(shoulder.overload_torque_raw, 100)

    def test_position_get_state_uses_extended_response_timeout(self) -> None:
        transport = ActuatorTransport(FakeSerial(), response_timeout_s=0.12)
        observed_timeouts: list[float | None] = []
        receive_matching = transport._receive_matching

        def record_timeout(*args, **kwargs):
            observed_timeouts.append(kwargs.get("timeout_s"))
            return receive_matching(*args, **kwargs)

        transport._receive_matching = record_timeout
        transport.enter_binary_mode()
        transport.get_state(include_positions=False)
        transport.get_state(include_positions=True)

        self.assertEqual(observed_timeouts[-2], None)
        self.assertEqual(
            observed_timeouts[-1],
            POSITION_STATE_RESPONSE_TIMEOUT_S,
        )
        self.assertEqual(POSITION_STATE_RESPONSE_TIMEOUT_S, 0.5)

    def test_transport_reconnects_when_mcu_is_already_binary(self) -> None:
        transport = ActuatorTransport(
            FakeSerial(already_binary=True),
            response_timeout_s=0.01,
        )
        hello = transport.enter_binary_mode()
        self.assertEqual(hello.capabilities, 0x00000FFF)

    def test_transport_validates_candidate_without_motion_state(self) -> None:
        transport = ActuatorTransport(FakeSerial(), response_timeout_s=0.01)
        transport.enter_binary_mode()

        result = transport.validate_buffered_candidate(
            1500,
            (
                BufferedSetpointSample(10, (0, 0, 0, 0, 0, 0)),
                BufferedSetpointSample(20, (1, 2, 3, 4, 5, 6)),
            ),
        )

        self.assertEqual(result.status_code, 5)
        self.assertEqual(result.sample_count, 2)
        self.assertEqual(result.queued_samples, 0)
        self.assertEqual(result.accepted_samples, 0)
        self.assertEqual(result.applied_samples, 0)

    def test_transport_exchanges_one_physical_buffered_frame_once(self) -> None:
        serial = FakeSerial()
        transport = ActuatorTransport(serial, response_timeout_s=0.01)
        transport.enter_binary_mode()
        samples = (
            BufferedSetpointSample(0, (0, 0, 0, 0, 0, 0)),
            BufferedSetpointSample(20, (1, 2, 3, 4, 5, 6)),
        )
        command = BufferedOutboundCommand(
            flags=int(
                BufferedSetpointFlags.CANDIDATE
                | BufferedSetpointFlags.BEGIN
            ),
            payload=encode_buffered_setpoint_payload(1500, samples),
            first_apply_tick_ms=1500,
            sample_count=2,
            first_sample_index=1,
            accepted_samples_after_ack=2,
        )

        response = transport.exchange_buffered_command(
            command,
            timeout_s=0.1,
        )

        self.assertEqual(response.result.status_code, 0)
        self.assertEqual(response.result.sample_count, 2)
        self.assertEqual(response.result.executor_state, 0)
        self.assertEqual(response.result.queued_samples, 2)
        self.assertEqual(response.result.accepted_samples, 2)
        self.assertEqual(serial.buffered_accepted_samples, 2)

    def test_heartbeat_requires_matching_state_acknowledgement(self) -> None:
        serial = FakeSerial()
        transport = ActuatorTransport(serial, response_timeout_s=0.01)
        transport.enter_binary_mode()

        state = transport.heartbeat()

        self.assertEqual(serial.heartbeat_request_count, 1)
        self.assertFalse(state.stop_latched)
        self.assertEqual(state.status_code, 0)
        self.assertEqual(HEARTBEAT_RESPONSE_TIMEOUT_S, 0.25)

    def test_disable_requires_firmware_acknowledgement(self) -> None:
        serial = FakeSerial()
        transport = ActuatorTransport(serial, response_timeout_s=0.01)
        transport.enter_binary_mode()

        transport.disable()

        self.assertEqual(serial.disable_request_count, 1)

    def test_transport_collects_unsolicited_motion_completion(self) -> None:
        serial = FakeSerial()
        transport = ActuatorTransport(serial, response_timeout_s=0.01)
        transport.enter_binary_mode()
        serial.queue_terminal_motion_result()
        results = transport.drain_motion_results()
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].status_code, 6)
        self.assertEqual(results[0].detail, 4)
        self.assertEqual(serial.get_state_request_count, 0)

    def test_transport_preserves_buffered_terminal_outer_sequence(self) -> None:
        serial = FakeSerial()
        transport = ActuatorTransport(serial, response_timeout_s=0.01)
        transport.enter_binary_mode()
        serial.queue_buffered_terminal_motion_result()

        buffered = transport.drain_buffered_motion_results()

        self.assertEqual(len(buffered), 1)
        self.assertEqual(buffered[0].frame_sequence, 78)
        self.assertEqual(buffered[0].result.request_sequence, 78)
        self.assertEqual(buffered[0].result.executor_state, 3)
        self.assertEqual(buffered[0].result.accepted_samples, 16)
        self.assertEqual(buffered[0].result.applied_samples, 16)
        legacy = transport.drain_motion_results()
        self.assertEqual(len(legacy), 1)
        self.assertEqual(legacy[0].status_code, 6)

    def test_async_result_defers_state_to_next_feedback_cycle(self) -> None:
        serial = FakeSerial(
            include_async_result=True,
            async_result_delay_s=0.015,
            drop_state_after_async_result=True,
        )
        transport = ActuatorTransport(
            serial,
            response_timeout_s=0.01,
        )
        transport.enter_binary_mode()
        with self.assertRaises(StateResponseDeferred):
            transport.get_state(include_positions=True)
        results = transport.drain_motion_results()
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].status_code, 6)
        self.assertEqual(serial.get_state_request_count, 1)
        state = transport.get_state(include_positions=True)
        self.assertIsNotNone(state.raw_positions)
        self.assertEqual(serial.get_state_request_count, 2)

    def test_transport_returns_setpoint_acceptance_identity(self) -> None:
        transport = ActuatorTransport(FakeSerial(), response_timeout_s=0.01)
        transport.enter_binary_mode()
        accepted = transport.send_setpoint([0] * 6, 300)
        self.assertEqual(accepted.status_code, 0)
        self.assertEqual(accepted.sample_count, 1)
        self.assertEqual(accepted.safety_state, 3)
        self.assertGreater(accepted.request_sequence, 0)
        self.assertEqual(accepted.apply_tick_ms, 1500)
        self.assertEqual(accepted.calibration_hash, 0x8AD27897)

    def test_safe_stop_ack_survives_interleaved_terminal_result(self) -> None:
        transport = ActuatorTransport(
            FakeSerial(terminal_before_safe_stop_ack=True),
            response_timeout_s=0.01,
        )
        transport.enter_binary_mode()
        transport.safe_stop()
        results = transport.drain_motion_results()
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].status_code, 6)

    def test_transport_clear_fault_requires_unlatched_success(self) -> None:
        transport = ActuatorTransport(FakeSerial(), response_timeout_s=0.01)
        transport.enter_binary_mode()
        state = transport.clear_fault()
        self.assertFalse(state.stop_latched)
        self.assertEqual(state.status_code, 0)


if __name__ == "__main__":
    unittest.main()
