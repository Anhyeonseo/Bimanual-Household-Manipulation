from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
TOOL = TOOLS / "validate_j1l_arm_limits_shadow_no_output.py"
SPEC = importlib.util.spec_from_file_location("validate_j1l_hardware", TOOL)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
SOURCE = TOOL.read_text(encoding="utf-8")


def test_checks_all_ten_arm_anchors_against_approved_manifest() -> None:
    _, approved = MODULE.load_approved_limits(ROOT)
    anchor = []
    for arm in ("left", "right"):
        for name in MODULE.ARM_JOINTS:
            joint = approved["arms"][arm][name]
            anchor.append(int(joint["minimum_urad"]))
        anchor.append(0)
    MODULE.verify_anchor_inside_arm_limits(tuple(anchor), approved)
    anchor[0] -= 1
    with pytest.raises(RuntimeError, match="left_base anchor"):
        MODULE.verify_anchor_inside_arm_limits(tuple(anchor), approved)


def test_reuses_j1w_torque_off_discarded_output_route() -> None:
    approved = ROOT / "config/bimanual_j1_operational_limits.approved.json"
    assert MODULE.EXPECTED_FIRMWARE_VERSION == 0x00024100
    assert MODULE.APPROVED_SHA256 == MODULE.file_sha256(approved)
    assert "redirect_stdout" in SOURCE
    assert "j1w.main()" in SOURCE
    assert 'document["firmware_arm_limit_admission_verified"] = True' in SOURCE
    for forbidden in (
        ".arm_and_enable(",
        "send_goal_async",
        "openocd",
        "Servo_",
        "RightServoBus_",
    ):
        assert forbidden not in SOURCE
