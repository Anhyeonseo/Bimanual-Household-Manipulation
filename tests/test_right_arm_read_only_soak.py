"""Contract checks for the no-write right-arm discovery soak tool."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOL = (ROOT / "tools/soak_right_arm_read_only_discovery.py").read_text(
    encoding="utf-8"
)


def test_soak_uses_only_the_read_only_discovery_service() -> None:
    assert 'SERVICE = "/discover_right_arm_read_only"' in TOOL
    assert '"commanded_operations": ["READ present_position"]' in TOOL
    for forbidden in (
        ".arm_and_enable(",
        ".clear_fault(",
        ".disable(",
        "FollowJointTrajectory",
        "send_goal_async",
    ):
        assert forbidden not in TOOL


def test_soak_requires_all_six_reads_without_a_failure() -> None:
    assert 'record["present_mask"] == "0x3F"' in TOOL
    assert 'record["read_statuses"] == [0, 0, 0, 0, 0, 0]' in TOOL
    assert 'record["failure_count"] == 0' in TOOL


def test_soak_can_record_the_raw_2048_q0_convention() -> None:
    assert '"--expect-position-raw"' in TOOL
    assert '"--position-tolerance-raw"' in TOOL
    assert '"all_expected_positions"' in TOOL
