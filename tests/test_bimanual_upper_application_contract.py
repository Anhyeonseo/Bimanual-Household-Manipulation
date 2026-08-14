from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "docs/BIMANUAL_UPPER_APPLICATION_INTERFACE.md"
PROMPT = ROOT / "docs/prompts/BIMANUAL_UPPER_APPLICATION_HANDOFF_PROMPT.md"
COMMAND = ROOT / "ros2_ws/src/so101_interfaces/srv/BimanualStreamCommand.srv"
FEEDBACK = ROOT / "ros2_ws/src/so101_interfaces/msg/BimanualJointFeedback.msg"
LIMITS = ROOT / "config/bimanual_operational_limits.json"
README = ROOT / "README.md"
CURRENT_STATE = ROOT / "docs/CURRENT_STATE_AND_NEXT_ROADMAP.md"
VERIFICATION_MATRIX = ROOT / "docs/VERIFICATION_MATRIX.md"
FINAL_ACCEPTANCE = (
    ROOT / "docs/test-results/2026-08-15-f87-resident-top-camera-pick-place.md"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_contract_pins_the_current_resident_interfaces() -> None:
    contract = CONTRACT.read_text(encoding="utf-8")
    assert "0x00024807" in contract
    assert "0xEFFFFFFF" in contract
    assert "9a9cd49247428478cae831d948977274d1188e9b0b0756d02de8c7c47fd431aa" in contract
    assert sha256(COMMAND) in contract
    assert sha256(FEEDBACK) in contract
    assert sha256(LIMITS) in contract


def test_contract_covers_every_public_operation_topic_and_joint() -> None:
    contract = CONTRACT.read_text(encoding="utf-8")
    command = COMMAND.read_text(encoding="utf-8")
    for operation in ("START_FINITE", "START_OPEN", "APPEND", "SPLICE", "STOP"):
        assert operation in command
        assert operation in contract
    for topic in (
        "/bimanual_stream_adapter/command",
        "/bimanual_stream_adapter/status",
        "/bimanual_stream_adapter/refresh_anchor",
        "/bimanual_stream_adapter/anchor_joint_states",
        "/bimanual_stream_adapter/joint_states",
        "/bimanual_stream_adapter/feedback",
    ):
        assert topic in contract
    for arm in ("left", "right"):
        for joint in (
            "base", "shoulder", "elbow", "wrist_flex", "wrist_roll", "gripper"
        ):
            assert f"{arm}_{joint}_joint" in contract


def test_legacy_general_backend_remains_unavailable() -> None:
    limits = json.loads(LIMITS.read_text(encoding="utf-8"))
    contract = CONTRACT.read_text(encoding="utf-8")
    assert limits["general_trajectory_output_available"] is False
    assert "legacy `single_arm_bridge`" in contract
    assert "resident 12축 경로뿐" in contract


def test_handoff_prompt_preserves_the_source_agnostic_safety_boundary() -> None:
    prompt = PROMPT.read_text(encoding="utf-8")
    for required in (
        "이미 학습",
        "source-agnostic",
        "serial port",
        "canonical 12축 absolute radians",
        "sample_age_ms <= 150",
        "START_OPEN",
        "APPEND",
        "SPLICE",
        "STOP exactly once",
        "refresh_anchor",
        "자동 reset/clear/retry",
        "46,020 µrad",
        "90,000 µrad",
        "완전한 finite route 전체",
        "READY_ARMED_HOLD",
    ):
        assert required in prompt


def test_contract_records_proven_application_and_session_semantics() -> None:
    contract = CONTRACT.read_text(encoding="utf-8")
    for required in (
        "ready(owner=null, epoch=0)",
        "armed READY/HOLD",
        "HOLD_REQUIRED",
        "67d2d1de5035c937c670a5f23ed0447392479ec81145c607a00ec4ca41aebd1a",
        "c887c8c723a5b870841cd404ab7673040f7dd0e26c58994ea068c45d0f1edd4c",
    ):
        assert required in contract


def test_contract_distinguishes_ros_finite_route_from_wire_batches() -> None:
    contract = CONTRACT.read_text(encoding="utf-8")
    assert "완전한 finite route 전체" in contract
    assert "최대 9점/400 ms wire window" in contract
    assert "finite route를 APPEND로 수동 분할하지 않는다" in contract


def test_readme_points_to_the_current_resident_contract() -> None:
    readme = README.read_text(encoding="utf-8")
    assert "현재 resident 후보 firmware: `0x00024807`" in readme
    assert "docs/BIMANUAL_UPPER_APPLICATION_INTERFACE.md" in readme
    assert "legacy `single_arm_bridge` 일반 trajectory backend는 계속 비승인" in readme
    assert "docs/test-results/2026-08-15-f87-resident-top-camera-pick-place.md" in readme


def test_final_acceptance_and_current_state_preserve_the_proven_boundary() -> None:
    acceptance = FINAL_ACCEPTANCE.read_text(encoding="utf-8")
    current = CURRENT_STATE.read_text(encoding="utf-8")
    matrix = VERIFICATION_MATRIX.read_text(encoding="utf-8")
    for required in (
        "0x00024807",
        "12회 연속 fresh measured joint pair",
        "46,020 urad",
        "90,000 urad",
        "67d2d1de5035c937c670a5f23ed0447392479ec81145c607a00ec4ca41aebd1a",
        "c887c8c723a5b870841cd404ab7673040f7dd0e26c58994ea068c45d0f1edd4c",
        "1416 passed",
    ):
        assert required in acceptance
    assert "finite trajectory 전체를 ROS 요청 하나로 제출" in current
    assert "오른팔 task와 범용 policy 성능은 별도 gate" in matrix
