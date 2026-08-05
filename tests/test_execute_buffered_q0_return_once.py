"""q0 복귀 sender 계약: SHA 고정, 재현성, 배포 게이트, 재시도 금지."""

import importlib.util
import json
import math
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
TOOLS_ROOT = ROOT / "tools"
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(TOOLS_ROOT))
MODULE_PATH = TOOLS_ROOT / "execute_buffered_q0_return_once.py"
SPEC = importlib.util.spec_from_file_location(
    "execute_buffered_q0_return_once", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)

PLANNER_SPEC = importlib.util.spec_from_file_location(
    "plan_buffered_q0_return_for_sender",
    TOOLS_ROOT / "plan_buffered_q0_return.py",
)
PLANNER = importlib.util.module_from_spec(PLANNER_SPEC)
sys.modules[PLANNER_SPEC.name] = PLANNER
PLANNER_SPEC.loader.exec_module(PLANNER)

CALIBRATION = PACKAGE_ROOT / "config" / "single_arm_calibration.json"
CONTRACT = PACKAGE_ROOT / "config" / "buffered_trajectory_contract.json"
ANCHOR = (2271, 3232, 1559, 1212, 2143, 2004)


def write_plan(tmp_path, anchor=ANCHOR, duration_ms=None):
    document = PLANNER.build_plan(CALIBRATION, CONTRACT, anchor, duration_ms)
    path = tmp_path / "q0_return.json"
    path.write_text(
        json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return path, MODULE.sha256_file(path)


def test_loads_plan_and_exposes_endpoints(tmp_path):
    path, digest = write_plan(tmp_path)
    plan = MODULE.load_q0_return_plan(path, digest, CALIBRATION, CONTRACT)

    assert plan.sha256 == digest
    assert plan.sample_count == len(plan.waypoints)
    # wire 형식이 urad 정수라 첫 waypoint 는 anchor 를 1 urad 안에서 재현한다.
    assert all(
        math.isclose(actual, expected, abs_tol=1.0e-6)
        for actual, expected in zip(
            plan.waypoints[0].positions_rad,
            plan.anchor_positions_rad,
            strict=True,
        )
    )
    # 마지막 waypoint 는 q0 이므로 전 축 정확히 0 이어야 한다.
    assert plan.waypoints[-1].positions_rad == (0.0,) * 5
    assert plan.target_positions_rad == (0.0,) * 5


def test_rejects_sha_mismatch(tmp_path):
    path, _ = write_plan(tmp_path)
    with pytest.raises(ValueError, match="plan sha256 mismatch"):
        MODULE.load_q0_return_plan(path, "0" * 64, CALIBRATION, CONTRACT)


def test_rejects_tampered_plan_even_with_matching_sha(tmp_path):
    path, _ = write_plan(tmp_path)
    document = json.loads(path.read_text(encoding="utf-8"))
    document["target"]["raw"][1] += 1
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="not exactly reproducible"):
        MODULE.load_q0_return_plan(
            path, MODULE.sha256_file(path), CALIBRATION, CONTRACT
        )


def test_rejects_undeployed_firmware_gate(tmp_path):
    path, digest = write_plan(tmp_path)
    document = json.loads(path.read_text(encoding="utf-8"))
    document["firmware_deployment_gate"]["deployed"] = False
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="firmware candidate is not deployed"):
        MODULE.load_q0_return_plan(
            path, MODULE.sha256_file(path), CALIBRATION, CONTRACT
        )


def test_rejects_motion_authorized_plan(tmp_path):
    path, _ = write_plan(tmp_path)
    document = json.loads(path.read_text(encoding="utf-8"))
    document["motion_authorized"] = True
    path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="motion_authorized=false"):
        MODULE.load_q0_return_plan(
            path, MODULE.sha256_file(path), CALIBRATION, CONTRACT
        )


def test_confirmation_and_retry_contract_are_fixed():
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert MODULE.CONFIRMATION == "EXECUTE_MOTION12_Q0_RETURN_ONCE"
    assert "ACTION_SEND_COUNT=1" in source
    assert "AUTOMATIC_RETRY_COUNT=0" in source
    # 재시도 루프가 없어야 한다.
    assert "while" not in source


def test_terminal_diagnostics_are_printed_for_the_lateness_profile():
    """0x00022600 의 lateness 분포를 실행 증거로 남겨야 한다."""
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "TERMINAL_DIAGNOSTICS=" in source
    assert "terminal_diagnostics" in source
