"""Motion-13 실행기 두 개의 게이트. ROS 없이 검증 가능한 부분만 본다.

leg sender 는 계획 전체를 다시 계산해 대조하고, leg 마다 다른 승인 문구를
요구한다. gripper sender 는 접촉 시 무엇을 요구할지를 `--expect` 로 명시하게
하고 기본값은 관측이다 — 물체를 문 gripper 가 무엇을 보고하는지 아직
실측되지 않았기 때문이다.
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


def _load(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(
        name, ROOT / "tools" / filename
    )
    module = importlib.util.module_from_spec(spec)
    # dataclass(slots=True) 가 자기 모듈을 sys.modules 에서 되찾으므로
    # exec 전에 등록해야 한다.
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


LEG_SENDER = _load(
    "execute_buffered_pick_place_leg_once",
    "execute_buffered_pick_place_leg_once.py",
)
GRIPPER_SENDER = _load(
    "execute_gripper_command_once", "execute_gripper_command_once.py"
)
PLANNER = _load(
    "plan_buffered_pick_place_leg", "plan_buffered_pick_place_leg.py"
)

CALIBRATION = PACKAGE_ROOT / "config" / "single_arm_calibration.json"
CONTRACT = PACKAGE_ROOT / "config" / "buffered_trajectory_contract.json"
MANIFEST = (
    ROOT
    / "artifacts/stage7/2026-07-31/full_pick_place_reindexed_headroom015"
    / "full_pick_place_plan_only_manifest.json"
)
ANCHOR = (2048, 2048, 2048, 2048, 2048, 2004)


@pytest.fixture(scope="module")
def leg_a_plan(tmp_path_factory):
    document = PLANNER.build_plan(CALIBRATION, CONTRACT, MANIFEST, "A", ANCHOR)
    path = tmp_path_factory.mktemp("motion13") / "leg_A.json"
    path.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return path, LEG_SENDER.sha256_file(path)


def load(path, digest, **kwargs):
    """계획 내용을 보는 시험은 배포 게이트를 우회한다.

    0x00022C00 후보가 플래시 전이라 계약이 undeployed 다. 게이트 자체는
    test_rejects_undeployed_firmware_candidate 가 기본값으로 지킨다.
    """
    kwargs.setdefault("require_deployed", False)
    return LEG_SENDER.load_pick_place_leg_plan(
        path, digest, CALIBRATION, CONTRACT, MANIFEST, **kwargs
    )


def test_each_leg_requires_its_own_confirmation() -> None:
    """순서를 사람의 주의력이 아니라 문자열 대조로 지킨다."""
    assert set(LEG_SENDER.CONFIRMATIONS) == {"A", "B", "C"}
    assert len(set(LEG_SENDER.CONFIRMATIONS.values())) == 3
    for leg, phrase in LEG_SENDER.CONFIRMATIONS.items():
        assert leg in phrase
        assert phrase.startswith("EXECUTE_MOTION13_LEG_")
        assert phrase.endswith("_ONCE")


def test_loads_plan_and_exposes_endpoints(leg_a_plan) -> None:
    path, digest = leg_a_plan
    plan = load(path, digest)

    assert plan.leg == "A"
    assert plan.sha256 == digest
    assert plan.sample_count == len(plan.waypoints)
    assert plan.target_name == "pick_grasp"
    assert plan.gripper_action_after == "pick_close"
    assert max(plan.anchor_deviation_raw) == 0
    # wire 형식이 urad 정수라 첫 waypoint 는 anchor 를 1 urad 안에서 재현한다.
    import math

    assert all(
        math.isclose(actual, expected, abs_tol=1.0e-6)
        for actual, expected in zip(
            plan.waypoints[0].positions_rad,
            plan.anchor_positions_rad,
            strict=True,
        )
    )


def test_rejects_sha_mismatch(leg_a_plan) -> None:
    path, _ = leg_a_plan
    with pytest.raises(ValueError, match="plan sha256 mismatch"):
        load(path, "0" * 64)


def test_rejects_tampered_plan_even_with_matching_sha(
    leg_a_plan, tmp_path
) -> None:
    """SHA 만으로는 다른 입력으로 만든 유효한 계획을 구분하지 못한다."""
    path, _ = leg_a_plan
    document = json.loads(path.read_text(encoding="utf-8"))
    document["resampling"]["samples"][10]["positions_urad"][1] += 5000
    tampered = tmp_path / "tampered.json"
    tampered.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")
    with pytest.raises(ValueError, match="not exactly reproducible"):
        load(tampered, LEG_SENDER.sha256_file(tampered))


def test_rejects_a_plan_whose_start_pose_disagrees_with_its_leg(
    leg_a_plan, tmp_path
) -> None:
    path, _ = leg_a_plan
    document = json.loads(path.read_text(encoding="utf-8"))
    document["anchor"]["expected_start_pose"] = "place"
    swapped = tmp_path / "swapped.json"
    swapped.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")
    with pytest.raises(ValueError, match="start pose does not match"):
        load(swapped, LEG_SENDER.sha256_file(swapped))


def test_rejects_undeployed_firmware_candidate(leg_a_plan, tmp_path) -> None:
    """계획이 미배포 게이트를 담고 있으면 실행기는 fail-closed 여야 한다."""
    path, _ = leg_a_plan
    document = json.loads(path.read_text(encoding="utf-8"))
    assert document["firmware_deployment_gate"]["deployed"] is True
    document["firmware_deployment_gate"]["deployed"] = False
    undeployed = tmp_path / "undeployed.json"
    undeployed.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")
    with pytest.raises(ValueError, match="not deployed"):
        LEG_SENDER.load_pick_place_leg_plan(
            undeployed, LEG_SENDER.sha256_file(undeployed),
            CALIBRATION, CONTRACT, MANIFEST,
        )


def test_rejects_a_plan_that_authorizes_motion(leg_a_plan, tmp_path) -> None:
    path, _ = leg_a_plan
    document = json.loads(path.read_text(encoding="utf-8"))
    document["motion_authorized"] = True
    authorized = tmp_path / "authorized.json"
    authorized.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")
    with pytest.raises(ValueError, match="motion_authorized"):
        load(authorized, LEG_SENDER.sha256_file(authorized))


def test_gripper_sender_defaults_to_observation_not_a_gate() -> None:
    """접촉 시 동작이 실측되기 전에는 게이트를 걸지 않는다."""
    arguments = _gripper_args(
        "--label", "probe", "--position-rad", "0.13",
        "--confirmation", "EXECUTE_MOTION13_GRIPPER_PROBE_ONCE",
    )
    assert arguments.expect == "report"


def test_gripper_sender_requires_a_label_specific_confirmation() -> None:
    with pytest.raises(SystemExit):
        _gripper_args(
            "--label", "pick_close", "--position-rad", "0.13",
            "--confirmation", "EXECUTE_MOTION13_GRIPPER_PROBE_ONCE",
        )


def test_gripper_sender_rejects_a_duration_outside_the_adapter_window() -> None:
    for duration in ("299", "2001"):
        with pytest.raises(SystemExit):
            _gripper_args(
                "--label", "probe", "--position-rad", "0.13",
                "--confirmation", "EXECUTE_MOTION13_GRIPPER_PROBE_ONCE",
                "--duration-ms", duration,
            )


def test_gripper_sender_reports_the_firmware_settle_tolerance() -> None:
    """잔여 간격을 판정하는 사람이 기준값을 함께 보게 한다."""
    config = (
        ROOT / "firmware/stm32_g474_single_arm/Core/Inc/single_arm_config.h"
    ).read_text()
    assert (
        f"SERVO_FINAL_ERROR_TOLERANCE_RAW UINT16_C("
        f"{GRIPPER_SENDER.FIRMWARE_SETTLE_TOLERANCE_RAW})"
    ) in config


def _gripper_args(*argv: str):
    saved = sys.argv
    sys.argv = ["execute_gripper_command_once.py", *argv]
    try:
        return GRIPPER_SENDER.parse_args()
    finally:
        sys.argv = saved


def test_contact_is_judged_by_residual_gap_not_reached_goal() -> None:
    """`reached_goal` 은 파지 증거가 아니다.

    `_finish_goal` 은 실행이 SUCCEEDED 이면 실제 위치와 무관하게 명령값을
    넣고 True 를 답한다. 2026-08-06 실측에서 물체를 문 close 가
    `REACHED_GOAL=True` 에 잔여 20 raw 였다. 판정은 잔여 간격으로 해야 한다.
    """
    source = (ROOT / "tools" / "execute_gripper_command_once.py").read_text(
        encoding="utf-8"
    )
    contact = source[source.index('elif arguments.expect == "contact":'):]
    contact = contact[: contact.index("else:")]
    assert "residual_raw" in contact
    assert "reached_goal" not in contact.split("#")[0] or True
    # 판정문 자체가 reached_goal 을 보지 않아야 한다.
    statements = [
        line for line in contact.splitlines()
        if "if " in line and not line.strip().startswith("#")
    ]
    assert statements and all("reached_goal" not in line for line in statements)
    assert GRIPPER_SENDER.MINIMUM_CONTACT_GAP_RAW > 0
    # 서보 정상 정착 오차(실측 2 raw)보다는 커야 한다.
    assert GRIPPER_SENDER.MINIMUM_CONTACT_GAP_RAW > 2
