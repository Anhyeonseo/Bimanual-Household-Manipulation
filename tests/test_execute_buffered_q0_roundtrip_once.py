import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
TOOLS_ROOT = ROOT / "tools"
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(TOOLS_ROOT))
MODULE_PATH = TOOLS_ROOT / "execute_buffered_q0_roundtrip_once.py"
SPEC = importlib.util.spec_from_file_location(
    "execute_buffered_q0_roundtrip_once",
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
    / "motion10_buffered_q0_roundtrip_plan_only.json"
)
PLAN_SHA = "28ec9511a1a94c020138fe6ad908300671bf60a5938a933953d3a4f155ad634d"
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


def test_loads_exact_q0_roundtrip_and_recomputes_all_samples():
    plan = MODULE.load_q0_roundtrip_plan(
        PLAN,
        PLAN_SHA,
        CALIBRATION,
        CONTRACT,
    )

    assert plan.sha256 == PLAN_SHA
    assert plan.duration_ms == 4200
    assert plan.sample_count == 211
    assert len(plan.waypoints) == 211
    assert plan.waypoints[105].positions_rad == (0.0,) * 5
    assert plan.waypoints[0].positions_rad == plan.anchor_positions_rad
    assert plan.waypoints[-1].positions_rad == plan.anchor_positions_rad


def test_rejects_sha_mismatch_before_loading_plan():
    with pytest.raises(ValueError, match="plan sha256 mismatch"):
        MODULE.load_q0_roundtrip_plan(
            PLAN,
            "0" * 64,
            CALIBRATION,
            CONTRACT,
        )


def test_rejects_tampered_q0_even_with_matching_sha(tmp_path):
    import json

    document = json.loads(PLAN.read_text(encoding="utf-8"))
    document["q0"]["arm_positions_rad"][0] = 0.01
    path = tmp_path / "tampered.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="q0 evidence"):
        MODULE.load_q0_roundtrip_plan(
            path,
            MODULE.sha256_file(path),
            CALIBRATION,
            CONTRACT,
        )


def test_rejects_sparse_or_nonanalytic_profile_even_with_matching_sha(tmp_path):
    import json

    document = json.loads(PLAN.read_text(encoding="utf-8"))
    document["analytic_profile"]["waypoint_count"] = 9
    path = tmp_path / "sparse.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="analytic minimum-jerk"):
        MODULE.load_q0_roundtrip_plan(
            path,
            MODULE.sha256_file(path),
            CALIBRATION,
            CONTRACT,
        )


def test_one_shot_sender_calls_send_exactly_once():
    result = SimpleNamespace(
        error_code=MODULE.FOLLOW_JOINT_TRAJECTORY_SUCCESSFUL,
        error_string=(
            "buffered trajectory completed; "
            "maximum_apply_lateness_ms=3 "
            "post_settle_max_error_raw=7"
        ),
    )
    response = SimpleNamespace(
        status=MODULE.ACTION_STATUS_SUCCEEDED,
        result=result,
    )
    handle = FakeGoalHandle(response)
    client = FakeActionClient(handle)
    plan = MODULE.load_q0_roundtrip_plan(
        PLAN,
        PLAN_SHA,
        CALIBRATION,
        CONTRACT,
    )

    status, received = MODULE.send_goal_once(
        SimpleNamespace(),
        client,
        MODULE.build_goal_spec(plan),
        result_timeout_s=15.0,
        wait=immediate_wait,
    )
    evidence = MODULE.validate_action_terminal(status, received)

    assert client.send_count == 1
    assert handle.result_requests == 1
    assert handle.cancel_requests == 0
    assert evidence.maximum_apply_lateness_ms == 3
    assert evidence.post_settle_max_error_raw == 7


def test_sender_confirmation_and_retry_contract_are_fixed():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert MODULE.CONFIRMATION == "EXECUTE_MOTION10_Q0_ROUNDTRIP_ONCE"
    assert "ACTION_SEND_COUNT=1" in source
    assert "AUTOMATIC_RETRY_COUNT=0" in source
    assert "while" not in source
