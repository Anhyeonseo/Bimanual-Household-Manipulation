import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "single_arm_bridge"
TOOLS_ROOT = ROOT / "tools"
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(TOOLS_ROOT))
MODULE_PATH = TOOLS_ROOT / "execute_buffered_pick_pregrasp_once.py"
SPEC = importlib.util.spec_from_file_location(
    "execute_buffered_pick_pregrasp_once",
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
    / "motion11_buffered_pick_pregrasp_plan_only.json"
)
PLAN_SHA = "745fecf3766a2e7edae76e0f94c5afd0135a619f34ae405e9fccab32dbccd0fa"
CALIBRATION = PACKAGE_ROOT / "config" / "single_arm_calibration.json"
CONTRACT = PACKAGE_ROOT / "config" / "buffered_trajectory_contract.json"
SOURCE_ROUTE = (
    ROOT
    / "artifacts"
    / "stage7"
    / "2026-07-31"
    / "full_pick_place_reindexed_headroom015"
    / "01_q0_to_pick_pregrasp.json"
)


def load():
    return MODULE.load_pick_pregrasp_plan(
        PLAN,
        PLAN_SHA,
        CALIBRATION,
        CONTRACT,
        SOURCE_ROUTE,
        require_deployed=False,
    )


def test_loads_exact_motion11_plan_and_endpoints():
    plan = load()

    assert plan.sha256 == PLAN_SHA
    assert plan.duration_ms == 47000
    assert plan.sample_count == 2351
    assert len(plan.waypoints) == 2351
    assert plan.waypoints[0].positions_rad == plan.anchor_positions_rad
    assert plan.waypoints[600].positions_rad == (0.0,) * 5
    assert plan.waypoints[-1].positions_rad == plan.target_positions_rad


def test_rejects_undeployed_firmware_candidate(tmp_path):
    """계약이 undeployed 로 되돌아가면 실행기는 fail-closed 여야 한다."""
    contract = json.loads(CONTRACT.read_text(encoding="utf-8"))
    contract["servo_uart_receive_candidate"]["deployed"] = False
    undeployed = tmp_path / "buffered_trajectory_contract.json"
    undeployed.write_text(json.dumps(contract), encoding="utf-8")
    with pytest.raises(ValueError, match="must stay deployed"):
        MODULE.load_pick_pregrasp_plan(
            PLAN, PLAN_SHA, CALIBRATION, undeployed, SOURCE_ROUTE,
        )


def test_rejects_sha_mismatch():
    with pytest.raises(ValueError, match="plan sha256 mismatch"):
        MODULE.load_pick_pregrasp_plan(
            PLAN,
            "0" * 64,
            CALIBRATION,
            CONTRACT,
            SOURCE_ROUTE,
            require_deployed=False,
        )


def test_rejects_tampered_plan_even_with_matching_sha(tmp_path):
    document = json.loads(PLAN.read_text(encoding="utf-8"))
    document["target"]["raw"][0] += 1
    path = tmp_path / "tampered.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="not exactly reproducible"):
        MODULE.load_pick_pregrasp_plan(
            path,
            MODULE.sha256_file(path),
            CALIBRATION,
            CONTRACT,
            SOURCE_ROUTE,
            require_deployed=False,
        )


def test_fresh_start_and_target_gates_use_per_axis_tolerances():
    plan = load()

    assert MODULE.validate_fresh_start(
        plan.anchor_positions_rad,
        plan.anchor_positions_rad,
        MODULE.START_TOLERANCES_RAD,
    ) == 0.0
    assert MODULE.validate_fresh_start(
        plan.target_positions_rad,
        plan.target_positions_rad,
        MODULE.TARGET_TOLERANCES_RAD,
    ) == 0.0

    unsafe = list(plan.target_positions_rad)
    unsafe[1] += 0.056
    with pytest.raises(ValueError, match="fresh start mismatch"):
        MODULE.validate_fresh_start(
            tuple(unsafe),
            plan.target_positions_rad,
            MODULE.TARGET_TOLERANCES_RAD,
        )


def test_sender_confirmation_and_retry_contract_are_fixed():
    source = MODULE_PATH.read_text(encoding="utf-8")

    assert MODULE.CONFIRMATION == "EXECUTE_MOTION11_PICK_PREGRASP_ONCE"
    assert "ACTION_SEND_COUNT=1" in source
    assert "AUTOMATIC_RETRY_COUNT=0" in source
    assert "while" not in source
    assert MODULE.ACTION_RESULT_TIMEOUT_S == 60.0
