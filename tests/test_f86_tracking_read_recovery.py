from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CMAKE = (ROOT / "firmware/stm32_g474_single_arm/CMakeLists.txt").read_text()
CONFIG = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Inc/single_arm_config.h"
).read_text()
HEADER = (
    ROOT
    / "firmware/stm32_g474_single_arm/Core/Inc/bimanual_tracking_feedback.h"
).read_text()
TRACKING = (
    ROOT
    / "firmware/stm32_g474_single_arm/Core/Src/bimanual_tracking_feedback.c"
).read_text()
BINARY = (
    ROOT / "firmware/stm32_g474_single_arm/Core/Src/binary_control.c"
).read_text()
ADAPTER = (
    ROOT
    / "ros2_ws/src/single_arm_bridge/single_arm_bridge/"
    "bimanual_stream_adapter.py"
).read_text()


def test_f87_preserves_f86_and_has_a_unique_firmware_identity() -> None:
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024806UL" in CMAKE
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024807UL" in CMAKE
    assert "F8_FIRMWARE_VERSION = 0x00024807" in ADAPTER


def test_tracking_read_failure_limit_is_exactly_three() -> None:
    assert (
        "HOST_BIMANUAL_TRACKING_READ_FAILURE_LIMIT UINT8_C(3)" in CONFIG
    )
    assert "BIMANUAL_TRACKING_TRANSIENT_FAILURE = 3" in HEADER
    assert "BIMANUAL_TRACKING_FAULT = 4" in HEADER


def test_one_failed_pair_is_degraded_and_success_clears_the_streak() -> None:
    assert "static BimanualTrackingFeedbackResult RecordPairFailure" in TRACKING
    assert "tracking.snapshot.failed_pairs++;" in TRACKING
    assert "tracking.snapshot.consecutive_failed_pairs++;" in TRACKING
    assert "return BIMANUAL_TRACKING_TRANSIENT_FAILURE;" in TRACKING
    assert "tracking.snapshot.consecutive_failed_pairs = 0U;" in TRACKING


def test_only_the_limit_crossing_requests_the_existing_coordinated_stop() -> None:
    assert "feedback_result == BIMANUAL_TRACKING_FAULT" in BINARY
    assert "feedback_result == BIMANUAL_TRACKING_TRANSIENT_FAILURE" not in BINARY
    assert "Host_RequestV2CoordinatedStop();" in BINARY
    assert "if tracking.failed_pairs" not in ADAPTER


def test_measured_tracking_errors_remain_immediately_fail_closed() -> None:
    assert "actuator_v2_stream_executor_check_joint_feedback" in BINARY
    assert "if (result != ACTUATOR_V2_EXECUTOR_OK)" in BINARY
    tracking_check = BINARY.index(
        "actuator_v2_stream_executor_check_joint_feedback"
    )
    stop_after_check = BINARY.index("Host_RequestV2CoordinatedStop();", tracking_check)
    assert stop_after_check > tracking_check
