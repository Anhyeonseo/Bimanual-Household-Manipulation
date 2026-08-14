from pathlib import Path
import subprocess

from single_arm_bridge.stream_protocol_v2 import (
    FEEDBACK_SNAPSHOT_V2,
    StreamMessageTypeV2,
    parse_feedback_snapshot_v2,
)


ROOT = Path(__file__).resolve().parents[1]
STM32 = ROOT / "firmware/stm32_g474_single_arm"
ACTUATOR = ROOT / "firmware/stm32_actuator"


def text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_f81_has_isolated_identity_and_compile_gate() -> None:
    cmake = text(STM32 / "CMakeLists.txt")
    config = text(STM32 / "Core/Inc/single_arm_config.h")
    protocol = text(ACTUATOR / "src/protocol.c")
    assert "BIMANUAL_RESIDENT_FEEDBACK_CANDIDATE" in cmake
    assert "HOST_BINARY_FIRMWARE_VERSION=0x00024800UL" in cmake
    assert "HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD=1U" in cmake
    assert "ACTUATOR_ENABLE_BIMANUAL_FEEDBACK_SNAPSHOT_MESSAGES=1U" in cmake
    assert "#define HOST_BIMANUAL_FEEDBACK_SNAPSHOT_BUILD 0U" in config
    assert "ACTUATOR_ENABLE_BIMANUAL_FEEDBACK_SNAPSHOT_MESSAGES" in protocol


def test_f81_wire_layout_is_exact_and_source_agnostic() -> None:
    positions = tuple(-60_000 + index * 10_000 for index in range(12))
    ages = tuple(index * 3 for index in range(12))
    payload = FEEDBACK_SNAPSHOT_V2.pack(
        0,
        12,
        0x0FFF,
        42,
        1234,
        5000,
        36,
        *positions,
        *ages,
    )
    snapshot = parse_feedback_snapshot_v2(payload)
    assert len(payload) == FEEDBACK_SNAPSHOT_V2.size == 116
    assert StreamMessageTypeV2.GET_FEEDBACK_SNAPSHOT == 61
    assert StreamMessageTypeV2.FEEDBACK_SNAPSHOT == 62
    assert snapshot.present_mask == 0x0FFF
    assert snapshot.request_sequence == 42
    assert snapshot.sender_time_ms_echo == 1234
    assert snapshot.firmware_tick_ms == 5000
    assert snapshot.completed_pairs == 36
    assert snapshot.positions_urad == positions
    assert snapshot.sample_age_ms == ages


def test_f81_cache_is_seeded_then_updated_from_real_tracking() -> None:
    source = text(STM32 / "Core/Src/binary_control.c")
    cache = text(STM32 / "Core/Src/bimanual_feedback_snapshot.c")
    assert "BimanualFeedbackSnapshot_Seed(" in source
    assert "host_v2_shadow_anchor_urad" in source
    conversion = source.index("Host_V2TrackingRawToUrad(")
    update = source.index("BimanualFeedbackSnapshot_UpdatePair(")
    safety = source.index(
        "actuator_v2_stream_executor_check_joint_feedback(", update
    )
    assert conversion < update < safety
    assert "sampled_at_ms[joint]" in cache
    assert "now_ms - feedback.sampled_at_ms[joint]" in cache
    assert "feedback.completed_pairs++" in cache


def test_f81_cache_age_and_pair_update_execute_on_host(tmp_path: Path) -> None:
    harness = tmp_path / "feedback_snapshot_test.c"
    executable = tmp_path / "feedback_snapshot_test"
    harness.write_text(
        r"""
#include "bimanual_feedback_snapshot.h"
#include <stdint.h>

int main(void)
{
    int32_t seed[12];
    BimanualFeedbackSnapshot snapshot;
    for (uint8_t joint = 0U; joint < 12U; joint++)
    {
        seed[joint] = (int32_t)joint * 1000;
    }
    BimanualFeedbackSnapshot_Seed(seed, UINT32_C(100));
    BimanualFeedbackSnapshot_Copy(UINT32_C(130), &snapshot);
    if ((snapshot.present_mask != UINT16_C(0x0FFF)) ||
        (snapshot.completed_pairs != 0U) ||
        (snapshot.sample_age_ms[0] != 30U) ||
        (snapshot.sample_age_ms[11] != 30U))
    {
        return 1;
    }
    BimanualFeedbackSnapshot_UpdatePair(
        2U, INT32_C(12345), INT32_C(-54321), UINT32_C(140));
    BimanualFeedbackSnapshot_Copy(UINT32_C(145), &snapshot);
    if ((snapshot.completed_pairs != 1U) ||
        (snapshot.positions_urad[2] != INT32_C(12345)) ||
        (snapshot.positions_urad[8] != INT32_C(-54321)) ||
        (snapshot.sample_age_ms[2] != 5U) ||
        (snapshot.sample_age_ms[8] != 5U) ||
        (snapshot.sample_age_ms[1] != 45U))
    {
        return 2;
    }
    return 0;
}
""",
        encoding="utf-8",
    )
    subprocess.run(
        [
            "cc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Wpedantic",
            "-Werror",
            "-I",
            str(STM32 / "Core/Inc"),
            str(harness),
            str(STM32 / "Core/Src/bimanual_feedback_snapshot.c"),
            "-o",
            str(executable),
        ],
        check=True,
    )
    subprocess.run([str(executable)], check=True)


def test_f81_ros_contract_publishes_standard_and_fresh_feedback() -> None:
    interface = text(
        ROOT / "ros2_ws/src/so101_interfaces/msg/BimanualJointFeedback.msg"
    )
    node = text(
        ROOT
        / "ros2_ws/src/single_arm_bridge/single_arm_bridge/"
        "bimanual_stream_node.py"
    )
    adapter = text(
        ROOT
        / "ros2_ws/src/single_arm_bridge/single_arm_bridge/"
        "bimanual_stream_adapter.py"
    )
    assert "float64[12] positions" in interface
    assert "uint32[12] sample_age_ms" in interface
    assert '"~/joint_states"' in node
    assert '"~/feedback"' in node
    assert "snapshot.sample_age_ms" in node
    assert "F8_FIRMWARE_VERSION = 0x00024807" in adapter
    assert "def feedback_snapshot" in adapter
    rolling = text(
        ROOT
        / "tools/execute_resident_bimanual_rolling_base_small_roundtrip_once.py"
    )
    assert "feedback_maximum_sample_age_ms" in rolling
    assert "maximum_observed_base_delta_rad" in rolling
