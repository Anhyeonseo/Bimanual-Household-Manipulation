"""Motion-13 leg 계획기의 계약. `tests/test_plan_buffered_pick_pregrasp.py` 형태.

이 계획기는 pose 를 하나도 자기가 만들지 않는다. SHA 로 고정된
collision-checked manifest 에서 유도할 뿐이다. 시험은 그 유도가 실제로
manifest 에 묶여 있는지, 그리고 팔이 경로 위에 있지 않으면 계획을 거부하는지를
본다.
"""

from __future__ import annotations

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
    "plan_buffered_pick_place_leg",
    ROOT / "tools" / "plan_buffered_pick_place_leg.py",
)
MODULE = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(MODULE)

CALIBRATION = PACKAGE_ROOT / "config" / "single_arm_calibration.json"
CONTRACT = PACKAGE_ROOT / "config" / "buffered_trajectory_contract.json"
MANIFEST = (
    ROOT
    / "artifacts"
    / "stage7"
    / "2026-07-31"
    / "full_pick_place_reindexed_headroom015"
    / "full_pick_place_plan_only_manifest.json"
)

# 각 leg 의 시작 pose 에 정확히 있는 anchor. 실기에서는 fresh anchor 를 쓴다.
LEG_ANCHORS = {
    "A": (2048, 2048, 2048, 2048, 2048, 2004),
    "B": (2270, 3413, 1548, 1210, 2127, 1963),
    "C": (2177, 3329, 1615, 1264, 2021, 2009),
}


_BUILD_CACHE: dict[tuple[str, tuple[int, ...]], dict] = {}


def build(leg: str, anchor=None):
    # duration 탐색이 leg 당 수백 회 시뮬레이션을 돌린다. 같은 입력은
    # 같은 결과이므로 캐시한다. 재현성 시험만 캐시를 우회한다.
    key = (leg, tuple(anchor or LEG_ANCHORS.get(leg, ())))
    if key not in _BUILD_CACHE:
        _BUILD_CACHE[key] = MODULE.build_plan(
            CALIBRATION, CONTRACT, MANIFEST, leg, anchor or LEG_ANCHORS[leg]
        )
    return _BUILD_CACHE[key]


@pytest.fixture(scope="module")
def key_poses():
    return MODULE.load_key_poses(MANIFEST)


def test_the_three_legs_cover_the_whole_route_and_split_at_the_gripper() -> None:
    """leg 경계가 gripper 동작 지점이어야 한다. 이것이 설계의 전부다."""
    legs = MODULE.LEG_DEFINITIONS
    assert tuple(legs) == ("A", "B", "C")
    chain = [legs["A"]["start_pose"]]
    for leg in ("A", "B", "C"):
        chain.extend(legs[leg]["waypoints"])
    assert chain == [
        "q0", "pick_pregrasp", "pick_grasp",
        "lift20", "place_pregrasp", "place",
        "retreat", "q0",
    ]
    # 각 leg 의 시작은 앞 leg 의 끝이다. 사이에 q0 복귀가 없다.
    assert legs["B"]["start_pose"] == legs["A"]["waypoints"][-1]
    assert legs["C"]["start_pose"] == legs["B"]["waypoints"][-1]
    # gripper 동작은 leg 경계에만 있다.
    assert legs["A"]["gripper_action_after"] == "pick_close"
    assert legs["B"]["gripper_action_after"] == "place_release"
    assert legs["C"]["gripper_action_after"] is None


def test_key_poses_come_only_from_the_pinned_manifest(key_poses) -> None:
    assert set(key_poses) == {
        "q0", "pick_pregrasp", "pick_grasp", "lift20",
        "place_pregrasp", "place", "retreat",
    }
    assert key_poses["q0"] == (0.0,) * 5
    # retreat 은 place_pregrasp 로 되돌아온다. manifest 가 그렇게 계획했다.
    assert key_poses["retreat"] == key_poses["place_pregrasp"]


def test_manifest_sha_is_pinned(tmp_path: Path) -> None:
    document = json.loads(MANIFEST.read_text(encoding="utf-8"))
    document["recommended_arm_duration_s"] = 3.0
    tampered = tmp_path / "manifest.json"
    tampered.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="manifest sha256 mismatch"):
        MODULE.load_key_poses(tampered)


@pytest.mark.parametrize("leg", ("A", "B", "C"))
def test_plan_is_non_executable_and_gripper_preserved(leg: str) -> None:
    document = build(leg)

    assert document["status"] == MODULE.STATUS
    assert document["phase"] == MODULE.PHASE
    assert document["leg"] == leg
    assert document["execution_api_used"] is False
    assert document["buffered_frame_encoded"] is False
    assert document["motion_authorized"] is False
    assert document["robot_target_available"] is False
    assert document["manifest_sha256"] == MODULE.MANIFEST_SHA256
    assert document["target"]["gripper_preserved"] is True
    # gripper 는 buffered leg 안에서 절대 움직이지 않는다.
    simulation = document["firmware_output_simulation"]
    assert simulation["start_raw"][5] == simulation["final_raw"][5]
    assert simulation["start_raw"][5] == LEG_ANCHORS[leg][5]


@pytest.mark.parametrize("leg", ("A", "B", "C"))
def test_leg_starts_at_the_anchor_and_ends_at_its_key_pose(
    leg: str, key_poses
) -> None:
    document = build(leg)
    simulation = document["firmware_output_simulation"]
    assert tuple(simulation["start_raw"]) == LEG_ANCHORS[leg]
    assert tuple(simulation["final_raw"]) == tuple(document["target"]["raw"])
    assert document["target"]["name"] == MODULE.LEG_DEFINITIONS[leg]["waypoints"][-1]


@pytest.mark.parametrize("leg", ("A", "B", "C"))
def test_admission_simulation_never_underflows(leg: str) -> None:
    """계획 크기가 queue 계약 안에 들어가는지 하드웨어 전에 확인한다."""
    queue = build(leg)["queue_contract"]
    terminal = queue["simulation_terminal"]
    assert terminal["state"] == "input_complete"
    assert terminal["safe_stop_required"] is False
    assert terminal["success_without_firmware_terminal"] is False
    assert terminal["accepted_samples"] == build(leg)["resampling"]["sample_count"]
    assert queue["maximum_batch_samples"] <= 9


@pytest.mark.parametrize("leg", ("A", "B", "C"))
def test_modeled_tracking_stays_inside_the_contract(leg: str) -> None:
    model = build(leg)["physical_tracking_model"]
    for name, stage in model["legs"].items():
        assert stage["maximum_peak_error_raw"] <= (
            model["maximum_allowed_peak_error_raw"]
        ), name
        assert stage["maximum_terminal_error_raw"] <= (
            model["maximum_allowed_terminal_error_raw"]
        ), name


def test_tracking_error_is_carried_between_stages() -> None:
    """경유점마다 팔이 목표에 정확히 있다고 가정하면 누적 지연을 놓친다.

    Motion-11 1차 시도는 서보가 못 따라오는 계획을 실기에서 확인했다. 여러
    경유점을 지나는 경로에서는 그 오차가 다음 구간의 출발 상태가 된다.
    """
    document = build("C")
    stages = document["physical_tracking_model"]["legs"]
    assert stages["retreat"]["maximum_entry_error_raw"] == 0.0
    # retreat 구간이 남긴 오차가 q0 구간의 진입 오차로 이어진다.
    assert stages["q0"]["maximum_entry_error_raw"] == pytest.approx(
        stages["retreat"]["maximum_terminal_error_raw"]
    )
    assert stages["q0"]["maximum_entry_error_raw"] > 0.0


def test_anchor_off_the_collision_checked_route_is_refused() -> None:
    """팔이 계획된 시작 자세에 없으면 계획하지 않는다."""
    anchor = list(LEG_ANCHORS["A"])
    anchor[1] += MODULE.ANCHOR_DEVIATION_LIMIT_RAW + 1
    with pytest.raises(ValueError, match="off the collision-checked route"):
        build("A", tuple(anchor))


def test_anchor_within_the_limit_is_accepted() -> None:
    anchor = list(LEG_ANCHORS["A"])
    anchor[1] += MODULE.ANCHOR_DEVIATION_LIMIT_RAW
    document = build("A", tuple(anchor))
    assert max(document["anchor"]["deviation_raw"]) == (
        MODULE.ANCHOR_DEVIATION_LIMIT_RAW
    )


def test_unknown_leg_is_refused() -> None:
    with pytest.raises(ValueError, match="unknown leg"):
        build("D", LEG_ANCHORS["A"])


def test_planning_records_the_deployment_gate_without_enforcing_it() -> None:
    """배포 강제는 실행기의 일이다.

    계획 시점에 강제하면 펌웨어 후보를 검증하는 동안 계획조차 만들 수 없다.
    실행 거부는 test_execute_motion13_senders 가 지킨다.
    """
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    declared = contract["servo_uart_receive_candidate"]
    document = build("A")
    assert document["firmware_deployment_gate"] == {
        "candidate_status": declared["status"],
        "deployed": declared["deployed"],
        "motion_authorized": declared["motion_authorized"],
    }


def test_motion_authorized_contract_is_refused(tmp_path: Path) -> None:
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    contract["servo_uart_receive_candidate"]["motion_authorized"] = True
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(contract), encoding="utf-8")
    with pytest.raises(ValueError):
        MODULE.build_plan(CALIBRATION, path, MANIFEST, "A", LEG_ANCHORS["A"])


@pytest.mark.parametrize("leg", ("A", "B", "C"))
def test_plan_is_exactly_reproducible(leg: str) -> None:
    fresh = MODULE.build_plan(
        CALIBRATION, CONTRACT, MANIFEST, leg, LEG_ANCHORS[leg]
    )
    assert fresh == build(leg)
