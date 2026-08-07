"""갓 계획된 segment 를 buffered leg 로 바꾸는 경로의 계약.

Motion-13 은 SHA 로 고정된 manifest 만 실행할 수 있어 과거에 검토된 경로를
재생하는 것밖에 못 한다. A4(offset 재계측)와 A4.5(Top 인식 기반 파지)는
매 회 새로 계획한 경로를 실행해야 하므로, 그 경로가 manifest 와 **같은
규율**을 통과하는지를 여기서 강제한다.

가장 중요한 단언은 두 가지다.

  1. endpoint 만 있는 계획은 받지 않는다. `ros_moveit_plan_grasp.py` 는
     궤적 점을 저장하지 않으므로 그것으로는 MoveIt 이 검사한 경로를 재생할
     수 없다. 경계된 스텝 체인을 담은 segment 파일이어야 한다.
  2. segment 체인이 관절공간 직선이 아니면 거부한다. 이 계획기는 경유점을
     버리고 minimum-jerk 하나로 잇기 때문에, 직선이 아니면 검사된 경로를
     벗어난다.
"""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(ROOT / "tools"))

_spec = importlib.util.spec_from_file_location(
    "plan_buffered_segment_leg", ROOT / "tools" / "plan_buffered_segment_leg.py"
)
MODULE = importlib.util.module_from_spec(_spec)
sys.modules["plan_buffered_segment_leg"] = MODULE
_spec.loader.exec_module(MODULE)

from single_arm_bridge.calibration import load_calibration  # noqa: E402
from plan_buffered_q0_roundtrip import radians_to_raw  # noqa: E402

CALIBRATION = PACKAGE_ROOT / "config" / "single_arm_calibration.json"
CONTRACT = PACKAGE_ROOT / "config" / "buffered_trajectory_contract.json"
# manifest 의 실제 phase 파일과 segment 생성기의 출력은 같은 스키마다.
# 그래서 이미 검토된 파일을 표본으로 쓸 수 있다.
SEGMENTS = (
    ROOT
    / "artifacts/stage7/2026-07-31/full_pick_place_reindexed_headroom015"
    / "02_pick_pregrasp_to_grasp.json"
)
GRIPPER_RAD = 0.069


@pytest.fixture(scope="module")
def calibration():
    return load_calibration(CALIBRATION)


@pytest.fixture(scope="module")
def digest():
    return hashlib.sha256(SEGMENTS.read_bytes()).hexdigest()


@pytest.fixture(scope="module")
def anchor(calibration):
    document = json.loads(SEGMENTS.read_text(encoding="utf-8"))
    start = tuple(document["segments"][0]["expected_start_positions_rad"])
    return radians_to_raw(calibration, start + (GRIPPER_RAD,))


def build(digest_value, anchor_value, path=SEGMENTS):
    return MODULE.build_plan(
        CALIBRATION, CONTRACT, path, digest_value, anchor_value
    )


def tampered(tmp_path: Path, mutate) -> tuple[Path, str]:
    document = json.loads(SEGMENTS.read_text(encoding="utf-8"))
    mutate(document)
    path = tmp_path / "segments.json"
    path.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")
    return path, hashlib.sha256(path.read_bytes()).hexdigest()


def test_plans_a_leg_from_a_freshly_checked_segment_route(
    digest, anchor
) -> None:
    document = build(digest, anchor)

    assert document["status"] == MODULE.STATUS
    assert document["execution_api_used"] is False
    assert document["buffered_frame_encoded"] is False
    assert document["motion_authorized"] is False
    assert document["robot_target_available"] is False

    route = document["segment_route"]
    assert route["sha256"] == digest
    assert route["status"].endswith("_SEGMENT_PLAN_ONLY_PASS")
    assert route["collision_checked_in_this_session"] is True
    assert route["segment_count"] == 3
    assert max(document["anchor"]["deviation_raw"]) == 0


def test_gripper_never_moves_and_endpoints_are_exact(digest, anchor) -> None:
    simulation = build(digest, anchor)["firmware_output_simulation"]
    assert tuple(simulation["start_raw"]) == tuple(anchor)
    assert simulation["start_raw"][5] == simulation["final_raw"][5]


def test_admission_never_underflows(digest, anchor) -> None:
    queue = build(digest, anchor)["queue_contract"]
    assert queue["simulation_terminal"]["state"] == "input_complete"
    assert queue["simulation_terminal"]["safe_stop_required"] is False
    assert queue["maximum_batch_samples"] <= 9


def test_sha_is_pinned_by_the_operator_not_baked_into_the_source(
    anchor,
) -> None:
    """매 회 새로 만들어지는 파일이라 상수로 고정할 수 없다.

    대신 계획과 실행이 같은 digest 를 요구해 그 사이에 파일이 바뀌지
    못하게 한다.
    """
    source = (ROOT / "tools" / "plan_buffered_segment_leg.py").read_text(
        encoding="utf-8"
    )
    assert "SEGMENTS_SHA256 =" not in source
    with pytest.raises(ValueError, match="segment plan sha256 mismatch"):
        build("0" * 64, anchor)


def test_refuses_a_route_that_is_not_a_straight_joint_space_path(
    tmp_path, anchor
) -> None:
    """경유점을 버리고 잇는 것이 허용되려면 직선이어야 한다."""

    def bend(document):
        # 체인은 유지한 채 중간 경유점만 옆으로 민다. 그러면 시작·끝은
        # 그대로인데 경로가 꺾인다 — 직선성 검사만 걸려야 한다.
        document["segments"][0]["target_positions_rad"][1] += 0.05
        document["segments"][1]["expected_start_positions_rad"][1] += 0.05

    path, digest_value = tampered(tmp_path, bend)
    with pytest.raises(ValueError, match="not a straight joint-space path"):
        build(digest_value, anchor, path)


def test_refuses_a_broken_segment_chain(tmp_path, anchor) -> None:
    def unlink(document):
        document["segments"][1]["expected_start_positions_rad"][0] += 0.02

    path, digest_value = tampered(tmp_path, unlink)
    with pytest.raises(ValueError, match="chain breaks|straight joint-space"):
        build(digest_value, anchor, path)


def test_refuses_a_failed_or_error_coded_segment(tmp_path, anchor) -> None:
    path, digest_value = tampered(
        tmp_path, lambda d: d["segments"][1].__setitem__("success", False)
    )
    with pytest.raises(ValueError, match="did not plan successfully"):
        build(digest_value, anchor, path)

    path, digest_value = tampered(
        tmp_path, lambda d: d["segments"][1].__setitem__("moveit_error_code", -1)
    )
    with pytest.raises(ValueError, match="non-success error code"):
        build(digest_value, anchor, path)


def test_refuses_a_segment_that_exceeds_its_own_step_limit(
    tmp_path, anchor
) -> None:
    def widen(document):
        document["segments"][0]["maximum_joint_delta_rad"] = (
            document["max_joint_step_rad"] + 0.01
        )

    path, digest_value = tampered(tmp_path, widen)
    with pytest.raises(ValueError, match="exceeds its own step limit"):
        build(digest_value, anchor, path)


@pytest.mark.parametrize(
    "flag", ("execution_api_used", "motion_authorized", "robot_target_available")
)
def test_refuses_a_route_that_relaxed_a_fail_closed_flag(
    tmp_path, anchor, flag
) -> None:
    path, digest_value = tampered(tmp_path, lambda d: d.__setitem__(flag, True))
    with pytest.raises(ValueError, match=f"{flag}=false"):
        build(digest_value, anchor, path)


def test_refuses_a_route_whose_status_is_not_a_pass(tmp_path, anchor) -> None:
    path, digest_value = tampered(
        tmp_path,
        lambda d: d.__setitem__("status", "GRASP_SEGMENT_PLAN_ONLY_FAIL"),
    )
    with pytest.raises(ValueError, match="status is not a pass"):
        build(digest_value, anchor, path)


def test_refuses_an_anchor_off_the_planned_route(digest, anchor) -> None:
    moved = list(anchor)
    moved[1] += MODULE.ANCHOR_DEVIATION_LIMIT_RAW + 1
    with pytest.raises(ValueError, match="off the freshly planned route"):
        build(digest, tuple(moved))


def test_tracking_model_stays_inside_the_contract(digest, anchor) -> None:
    model = build(digest, anchor)["physical_tracking_model"]
    stage = model["legs"]["anchor_to_segment_target"]
    assert stage["maximum_peak_error_raw"] <= (
        model["maximum_allowed_peak_error_raw"]
    )
    assert stage["maximum_terminal_error_raw"] <= (
        model["maximum_allowed_terminal_error_raw"]
    )


def test_endpoint_only_grasp_plans_are_not_accepted() -> None:
    """`ros_moveit_plan_grasp.py` 출력은 궤적 점을 담지 않는다.

    그것을 직접 받으면 MoveIt 이 검사하지 않은 경로를 직선으로 이어
    실행하게 된다. 이 계획기는 segment 파일만 받는다.
    """
    endpoint_plan = (
        ROOT
        / "artifacts/stage7/2026-07-31/full_pick_place_reindexed_headroom015"
        / "pick_pose_plan_only.json"
    )
    document = json.loads(endpoint_plan.read_text(encoding="utf-8"))
    # 전제 확인: 정말로 궤적 점이 없다.
    assert "segments" not in document
    assert all("points" not in plan for plan in document["plans"])
    digest_value = hashlib.sha256(endpoint_plan.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match="status is not a pass"):
        MODULE.build_plan(
            CALIBRATION, CONTRACT, endpoint_plan, digest_value,
            (2278, 3190, 1625, 1209, 2146, 2003),
        )
