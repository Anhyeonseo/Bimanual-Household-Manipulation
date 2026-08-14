"""J2 axis targets remain SHA-bound, interior, and plan-only."""

from importlib.util import module_from_spec, spec_from_file_location
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "tools/bimanual_j2_targets.py"
SPEC = spec_from_file_location("bimanual_j2_targets_test", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
APPROVED = ROOT / "config/bimanual_j1_operational_limits.approved.json"
TARGETS = ROOT / "artifacts/joint_ranges/2026-08-13/j2_axis_targets_plan_only.json"
APPROVED_SHA = "ab5a352cac757e87242986e4018b7d89e2302789795bf1e36896648abedf34ff"
TARGETS_SHA = "63e99ca8a0fa5231e50777486d6c051ef5db8aaca7f5bd45ccd672958e20e87c"


def test_checked_in_targets_are_bound_and_reproducible() -> None:
    approved = MODULE.load_bound_json(APPROVED, APPROVED_SHA, "approved")
    checked_in = MODULE.load_bound_json(TARGETS, TARGETS_SHA, "targets")
    derived = MODULE.derive_targets(approved)
    for key, value in derived.items():
        assert checked_in[key] == value
    assert checked_in["inputs"]["approved"]["sha256"] == APPROVED_SHA
    assert checked_in["motion_authorized"] is False
    assert checked_in["execution_api_used"] is False
    assert checked_in["endpoint_commands_forbidden"] is True
    assert checked_in["multi_joint_commands_forbidden"] is True


@pytest.mark.parametrize("arm", ("left", "right"))
def test_all_axis_targets_are_strictly_inside_approved_limits(arm: str) -> None:
    document = json.loads(TARGETS.read_text(encoding="utf-8"))
    for joint in MODULE.ARM_JOINTS:
        record = document["arms"][arm]["joints"][joint]
        minimum = record["approved_minimum_unwrapped_raw"]
        maximum = record["approved_maximum_unwrapped_raw"]
        for direction in ("lower", "upper"):
            distances = []
            for fraction in (25, 50, 75):
                target = record["directions"][direction][str(fraction)]
                assert minimum < target["target_unwrapped_raw"] < maximum
                distances.append(target["distance_from_q0_raw"])
            assert distances == sorted(distances)
            assert len(set(distances)) == 3
    assert document["arms"][arm]["joints"]["gripper"]["status"].startswith("BLOCKED_")


def test_first_right_base_target_is_exact_and_selectable() -> None:
    document = json.loads(TARGETS.read_text(encoding="utf-8"))
    selected = MODULE.select_target(document, "right", "base", "upper", 25)
    assert selected["servo_id"] == 1
    assert selected["q0_unwrapped_raw"] == 2048
    assert selected["target_unwrapped_raw"] == 2269
    assert selected["distance_from_q0_raw"] == 221


@pytest.mark.parametrize(
    ("position", "target", "expected"),
    ((2048, 2269, 20), (2241, 2269, 20), (2242, 2269, 19),
     (2250, 2269, 19), (2259, 2269, 0), (2269, 2048, -20),
     (2067, 2048, -19), (2058, 2048, 0)),
)
def test_bounded_steps_never_create_an_illegal_tail(position, target, expected) -> None:
    step = MODULE.bounded_step_raw(position, target)
    assert step == expected
    if step:
        assert 8 <= abs(step) <= 20
        assert abs(position + step - target) < abs(position - target)


def test_sha_mismatch_fails_closed() -> None:
    with pytest.raises(ValueError, match="SHA mismatch"):
        MODULE.load_bound_json(APPROVED, "0" * 64, "approved")


def test_plan_only_generator_has_no_ros_motion_api() -> None:
    source = (ROOT / "tools/derive_bimanual_j2_axis_targets_plan_only.py").read_text(encoding="utf-8")
    assert "--plan-only" in source
    for forbidden in ("rclpy", "ActionClient", "FollowJointTrajectory", "call_async"):
        assert forbidden not in source
