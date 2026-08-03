import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
sys.path.insert(0, str(PACKAGE_ROOT))
MODULE_PATH = ROOT / "tools" / "execute_buffered_action_plan_once.py"
SPEC = importlib.util.spec_from_file_location(
    "execute_buffered_action_plan_once",
    MODULE_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


PLAN = (
    ROOT
    / "artifacts"
    / "motion"
    / "2026-08-04"
    / "motion9_buffered_action_roundtrip_plan_only.json"
)
PLAN_SHA = "d5378b6c0eb5eb4069e79e609ee12efb14750d228b61b009d29555fb573f47f8"
CALIBRATION = PACKAGE_ROOT / "config" / "single_arm_calibration.json"
CONTRACT = PACKAGE_ROOT / "config" / "buffered_trajectory_contract.json"


class CompletedFuture:
    def __init__(self, value):
        self._value = value

    def done(self):
        return True

    def result(self):
        return self._value


class FakeGoalHandle:
    accepted = True

    def __init__(self, response):
        self._response = response
        self.result_requests = 0
        self.cancel_requests = 0

    def get_result_async(self):
        self.result_requests += 1
        return CompletedFuture(self._response)

    def cancel_goal_async(self):
        self.cancel_requests += 1
        return CompletedFuture(SimpleNamespace())


class FakeActionClient:
    def __init__(self, goal_handle):
        self.goal_handle = goal_handle
        self.send_count = 0

    def send_goal_async(self, goal):
        self.send_count += 1
        self.goal = goal
        return CompletedFuture(self.goal_handle)


def immediate_wait(unused_node, future, unused_timeout):
    del unused_node, unused_timeout
    return future.result()


def current_contract_plan(tmp_path):
    document = json.loads(PLAN.read_text(encoding="utf-8"))
    document["contract_sha256"] = MODULE.sha256_file(CONTRACT)
    path = tmp_path / "current_contract_plan.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    return path, MODULE.sha256_file(path)


def test_historical_plan_is_invalidated_by_deployed_contract():
    with pytest.raises(ValueError, match="plan buffered contract sha256 mismatch"):
        MODULE.load_commissioning_plan(
            PLAN,
            PLAN_SHA,
            CALIBRATION,
            CONTRACT,
        )


def test_current_contract_copy_recomputes_all_samples(tmp_path):
    path, digest = current_contract_plan(tmp_path)
    plan = MODULE.load_commissioning_plan(
        path,
        digest,
        CALIBRATION,
        CONTRACT,
    )

    assert plan.sha256 == digest
    assert plan.duration_ms == 1200
    assert plan.sample_count == 61
    assert len(plan.waypoints) == 7
    assert plan.waypoints[0].positions_rad == plan.anchor_positions_rad
    assert plan.waypoints[-1].positions_rad == plan.anchor_positions_rad


def test_rejects_sha_mismatch_before_loading_plan():
    with pytest.raises(ValueError, match="plan sha256 mismatch"):
        MODULE.load_commissioning_plan(
            PLAN,
            "0" * 64,
            CALIBRATION,
            CONTRACT,
        )


def test_rejects_tampered_reviewed_delta_even_with_matching_sha(tmp_path):
    document = json.loads(PLAN.read_text(encoding="utf-8"))
    document["contract_sha256"] = MODULE.sha256_file(CONTRACT)
    document["requested_deltas_rad"]["left_base_joint"] = 0.02
    tampered = tmp_path / "tampered.json"
    tampered.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="requested deltas"):
        MODULE.load_commissioning_plan(
            tampered,
            MODULE.sha256_file(tampered),
            CALIBRATION,
            CONTRACT,
        )


def test_fresh_start_uses_shoulder_specific_tolerance():
    anchor = (0.0,) * 5
    assert MODULE.validate_fresh_start(
        (0.049, 0.054, 0.049, 0.049, 0.049),
        anchor,
    ) == pytest.approx(0.054)
    with pytest.raises(ValueError, match="joint_index=1"):
        MODULE.validate_fresh_start(
            (0.0, 0.056, 0.0, 0.0, 0.0),
            anchor,
        )


def test_goal_spec_preserves_reviewed_joint_order_and_waypoints(tmp_path):
    path, digest = current_contract_plan(tmp_path)
    plan = MODULE.load_commissioning_plan(
        path,
        digest,
        CALIBRATION,
        CONTRACT,
    )
    specification = MODULE.build_goal_spec(plan)

    assert tuple(specification["joint_names"]) == plan.arm_joint_names
    assert len(specification["points"]) == 7
    assert tuple(specification["points"][0]["positions"]) == (
        plan.anchor_positions_rad
    )
    assert specification["points"][-1]["time_from_start_ms"] == 1200


def test_ros_goal_preserves_reviewed_joint_order_and_waypoints(tmp_path):
    pytest.importorskip("control_msgs.action")
    path, digest = current_contract_plan(tmp_path)
    plan = MODULE.load_commissioning_plan(
        path,
        digest,
        CALIBRATION,
        CONTRACT,
    )
    goal = MODULE.build_goal(plan)

    assert tuple(goal.trajectory.joint_names) == plan.arm_joint_names
    assert len(goal.trajectory.points) == 7
    assert tuple(goal.trajectory.points[0].positions) == (
        plan.anchor_positions_rad
    )
    assert goal.trajectory.points[-1].time_from_start.sec == 1
    assert goal.trajectory.points[-1].time_from_start.nanosec == 200_000_000


def test_one_shot_sender_calls_send_exactly_once_and_accepts_terminal(tmp_path):
    result = SimpleNamespace(
        error_code=MODULE.FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL,
        error_string=(
            "buffered trajectory completed; "
            "maximum_apply_lateness_ms=2 "
            "post_settle_max_error_raw=8"
        ),
    )
    response = SimpleNamespace(
        status=MODULE.ACTION_STATUS_SUCCEEDED,
        result=result,
    )
    handle = FakeGoalHandle(response)
    client = FakeActionClient(handle)
    path, digest = current_contract_plan(tmp_path)
    plan = MODULE.load_commissioning_plan(
        path,
        digest,
        CALIBRATION,
        CONTRACT,
    )

    status, received = MODULE.send_goal_once(
        SimpleNamespace(),
        client,
        MODULE.build_goal_spec(plan),
        wait=immediate_wait,
    )
    evidence = MODULE.validate_action_terminal(status, received)

    assert client.send_count == 1
    assert handle.result_requests == 1
    assert handle.cancel_requests == 0
    assert evidence.maximum_apply_lateness_ms == 2
    assert evidence.post_settle_max_error_raw == 8


def test_terminal_requires_success_lateness_and_post_settle_evidence():
    valid = SimpleNamespace(
        error_code=MODULE.FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL,
        error_string=(
            "buffered trajectory completed; "
            "maximum_apply_lateness_ms=1 "
            "post_settle_max_error_raw=30"
        ),
    )
    evidence = MODULE.validate_action_terminal(
        MODULE.ACTION_STATUS_SUCCEEDED,
        valid,
    )
    assert evidence.maximum_apply_lateness_ms == 1
    assert evidence.post_settle_max_error_raw == 30

    with pytest.raises(RuntimeError, match="did not succeed"):
        MODULE.validate_action_terminal(
            6,
            valid,
        )
    invalid_lateness = SimpleNamespace(
        error_code=MODULE.FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL,
        error_string=(
            "buffered trajectory completed; "
            "maximum_apply_lateness_ms=6 "
            "post_settle_max_error_raw=0"
        ),
    )
    with pytest.raises(RuntimeError, match="outside 0..5"):
        MODULE.validate_action_terminal(
            MODULE.ACTION_STATUS_SUCCEEDED,
            invalid_lateness,
        )


def test_source_has_one_send_site_and_no_retry_loop():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert source.count(".send_goal_async(") == 1
    assert "AUTOMATIC_RETRY_COUNT=0" in source
