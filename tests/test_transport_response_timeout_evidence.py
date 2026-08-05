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

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "single_arm_bridge"))

from single_arm_bridge.protocol import (  # noqa: E402
    MessageType,
    encode_frame,
    Frame,
)
from single_arm_bridge.transport import (  # noqa: E402
    ActuatorTransport,
    ResponseTimeoutError,
    TransportError,
)


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
