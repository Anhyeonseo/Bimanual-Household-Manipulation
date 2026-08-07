"""
응답 타임아웃이 원인을 구분할 수 있는 증거를 남기는지 검증한다.

2026-08-06 buffered startup 중단에서 host 로그는
`timeout waiting for STATE_FEEDBACK` 한 줄뿐이었다. 그것만으로는
host 가 다른 트래픽을 소비하며 예산을 태운 것인지, MCU 가 조용한 링크에서
그냥 늦게 답한 것인지 구분할 수 없다. 두 경우의 대응이 완전히 다르므로
타임아웃이 관측 내역을 함께 보고해야 한다.
"""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "single_arm_bridge"))

from single_arm_bridge.protocol import (  # noqa: E402
    MessageType,
    encode_frame,
    Frame,
)
from single_arm_bridge import transport as transport_module  # noqa: E402
from single_arm_bridge.transport import (  # noqa: E402
    ActuatorTransport,
    ResponseTimeoutError,
    TransportError,
)

PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"


class SilentPort:
    """빈 read 만 반환한다. MCU 가 늦게 답하는 조용한 링크."""

    in_waiting = 0

    def __init__(self):
        self.writes = []

    def read_until(self, terminator):
        return b""

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        pass


class NoisePort(SilentPort):
    """대기 중인 응답과 무관한 프레임을 계속 돌려준다."""

    def __init__(self, frames):
        super().__init__()
        self._frames = list(frames)
        self._index = 0

    def read_until(self, terminator):
        if self._index >= len(self._frames):
            return b""
        frame = self._frames[self._index]
        self._index += 1
        return frame


def make_transport(port):
    return ActuatorTransport(port, response_timeout_s=0.02)


def test_silent_link_reports_empty_reads_and_no_observed_frames():
    transport = make_transport(SilentPort())
    with pytest.raises(ResponseTimeoutError) as info:
        transport._receive_matching(7, MessageType.STATE_FEEDBACK)

    error = info.value
    assert error.sequence == 7
    assert error.message_type is MessageType.STATE_FEEDBACK
    assert error.empty_reads > 0
    assert error.observed_frames == ()
    # 조용한 링크였다는 사실이 메시지에서 읽혀야 한다.
    assert "observed=none" in str(error)
    assert "empty_reads=" in str(error)


def test_foreign_traffic_is_named_in_the_timeout():
    """다른 트래픽에 예산을 태웠다면 무엇을 봤는지 남아야 한다."""
    noise = [
        encode_frame(
            Frame(
                message_type=MessageType.STATE_FEEDBACK,
                sequence=99,
                sender_time_ms=0,
                payload=b"",
            )
        )
        for _ in range(3)
    ]
    transport = make_transport(NoisePort(noise))
    with pytest.raises(ResponseTimeoutError) as info:
        transport._receive_matching(7, MessageType.STATE_FEEDBACK)

    error = info.value
    assert error.observed_frames
    assert all("STATE_FEEDBACK#99" == name for name in error.observed_frames)
    assert "STATE_FEEDBACK#99" in str(error)


def test_elapsed_and_budget_are_reported():
    transport = make_transport(SilentPort())
    with pytest.raises(ResponseTimeoutError) as info:
        transport._receive_matching(1, MessageType.STATE_FEEDBACK)

    error = info.value
    assert error.timeout_s == pytest.approx(0.02)
    assert error.elapsed_s >= 0.02
    assert "budget_ms=" in str(error)
    assert "elapsed_ms=" in str(error)


def test_remains_a_transport_error_for_existing_handlers():
    """bridge 의 기존 예외 처리 경로가 그대로 동작해야 한다."""
    assert issubclass(ResponseTimeoutError, TransportError)
    transport = make_transport(SilentPort())
    with pytest.raises(TransportError):
        transport._receive_matching(1, MessageType.STATE_FEEDBACK)


def test_observed_frame_list_is_bounded():
    """긴 정체에서도 예외 메시지가 무한히 길어지면 안 된다."""
    noise = [
        encode_frame(
            Frame(
                message_type=MessageType.STATE_FEEDBACK,
                sequence=index,
                sender_time_ms=0,
                payload=b"",
            )
        )
        for index in range(200)
    ]
    transport = ActuatorTransport(NoisePort(noise), response_timeout_s=0.5)
    with pytest.raises(ResponseTimeoutError) as info:
        transport._receive_matching(9999, MessageType.STATE_FEEDBACK)
    assert len(info.value.observed_frames) <= 8


def test_rejected_packet_bytes_are_captured_and_bounded():
    """
    거부된 패킷의 실체를 봐야 원인이 갈린다.

    read timeout 에 잘린 부분 frame 인지, 구조는 온전한데 checksum 이 틀린
    frame 인지, 잡음인지에 따라 대응이 완전히 다르다. 개수만으로는 셋을
    구분할 수 없다.
    """
    garbage = [b"\xde\xad\xbe\xef\x00" for _ in range(5)]
    transport = make_transport(NoisePort(garbage))
    with pytest.raises(ResponseTimeoutError) as info:
        transport._receive_matching(3, MessageType.STATE_FEEDBACK)

    error = info.value
    assert error.undecodable_frames > 0
    assert error.rejected_packets
    assert error.rejected_packets[0].startswith(b"\xde\xad\xbe\xef")
    assert "deadbeef" in str(error)
    # 예외 메시지가 무한히 길어지면 안 된다.
    assert len(error.rejected_packets) <= 2
    assert all(len(p) <= 48 for p in error.rejected_packets)


class TruncatedPort(SilentPort):
    """구분자 없이 끊긴 부분 패킷을 돌려준다. read timeout 에 잘린 경우."""

    def __init__(self, chunks):
        super().__init__()
        self._chunks = list(chunks)
        self._index = 0

    def read_until(self, terminator):
        if self._index >= len(self._chunks):
            return b""
        chunk = self._chunks[self._index]
        self._index += 1
        return chunk


def test_partial_read_lengths_report_the_retained_fragment():
    """
    잘린 조각은 버리지 않고 다음 read 로 이어붙인다. 보고되는 길이는 그
    시점까지 누적된 조각 크기다. 조각을 버리던 동작이 프레임을 파괴해
    2026-08-06 buffered startup 에서 응답을 잃게 만들었다.
    """
    transport = make_transport(TruncatedPort([b"\x01\x02\x03", b"\x04\x05"]))
    with pytest.raises(ResponseTimeoutError) as info:
        transport._receive_matching(3, MessageType.STATE_FEEDBACK)

    error = info.value
    # 3 바이트 뒤 2 바이트가 더 붙어 누적 5 가 된다.
    assert error.partial_read_lengths[:2] == (3, 5)
    assert "partial_lengths=3,5" in str(error)


def test_a_frame_split_across_reads_is_reassembled():
    """
    구분자 없이 끊긴 앞부분과 뒤늦게 오는 뒷부분이 하나의 프레임으로
    복원되어야 한다. 이것이 실기에서 잃어버린 STATE_FEEDBACK 의 구조다.
    """
    whole = encode_frame(
        Frame(
            message_type=MessageType.STATE_FEEDBACK,
            sequence=41,
            sender_time_ms=7,
            payload=b"",
        )
    )
    head, tail = whole[:11], whole[11:]
    transport = make_transport(TruncatedPort([head, tail]))
    frame = transport._receive_matching(41, MessageType.STATE_FEEDBACK)

    assert frame.sequence == 41
    assert frame.message_type is MessageType.STATE_FEEDBACK


def test_residual_buffer_is_bounded_against_a_delimiterless_stream():
    """구분자가 영원히 오지 않아도 메모리가 무한히 자라면 안 된다."""
    from single_arm_bridge.transport import RX_RESIDUAL_LIMIT_BYTES

    noise = [b"\xAB" * 512 for _ in range(40)]
    transport = ActuatorTransport(TruncatedPort(noise), response_timeout_s=0.05)
    with pytest.raises(ResponseTimeoutError):
        transport._receive_matching(1, MessageType.STATE_FEEDBACK)
    assert len(transport._rx_residual) <= RX_RESIDUAL_LIMIT_BYTES


def test_pure_silence_reports_no_partial_lengths():
    transport = make_transport(SilentPort())
    with pytest.raises(ResponseTimeoutError) as info:
        transport._receive_matching(3, MessageType.STATE_FEEDBACK)
    assert info.value.partial_read_lengths == ()
    assert "partial_lengths=none" in str(info.value)
    assert "rejected=none" in str(info.value)


def test_stop_latched_error_preserves_the_state_it_decoded() -> None:
    """latch 는 동작을 막는 게이트이지 관측을 막는 게이트가 아니다.

    2026-08-06 복구가 여기서 멈췄다. torque 가 꺼진 사이 팔이 중력으로 처져
    팔꿈치가 상한을 넘었는데, 그 사실을 말해줄 유일한 판독이 예외와 함께
    버려졌다. CLEAR_FAULT 는 6축이 범위 안일 때만 수락되므로, 운영자는 latch
    상태에서도 자세를 읽어야 복구할 수 있다.
    """
    source = (
        PACKAGE_ROOT / "single_arm_bridge" / "transport.py"
    ).read_text(encoding="utf-8")
    # 두 raise 지점 모두 state 를 넘겨야 한다.
    assert source.count("state=state,") == 2
    for anchor in ("HEARTBEAT rejected: ", "GET_STATE rejected: status="):
        index = source.index(anchor)
        assert "state=state," in source[index:index + 400], anchor

    latched = SimpleNamespace(
        stop_latched=True, status_code=0, raw_positions=(1, 2, 3, 4, 5, 6)
    )
    error = transport_module.StopLatchedError("latched", state=latched)
    assert error.state is latched
    assert error.state.raw_positions == (1, 2, 3, 4, 5, 6)
    # state 없이도 만들 수 있어야 한다. 기존 호출부를 깨지 않는다.
    assert transport_module.StopLatchedError("latched").state is None
