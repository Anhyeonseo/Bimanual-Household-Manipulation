"""J2 right-Base acceptance is SHA-bound, plan-only, and narrowly scoped."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools/check_right_arm_j2_base_acceptance_plan_only.py"
EVIDENCE = ROOT / "artifacts/joint_ranges/2026-08-14"
REQUIRED_INPUTS = (
    "j2b_no_motion_run01.json",
    "j2_right_base_upper25_run03.json",
    "j2_right_base_upper50_run01.json",
    "j2_right_base_upper75_run01.json",
    "j2_right_base_lower25_run01.json",
    "j2_right_base_lower50_run01.json",
    "j2_right_base_lower75_run01.json",
)


def run_checker(evidence: Path, output: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(TOOL),
            "--plan-only",
            "--evidence-directory",
            str(evidence),
            "--output",
            str(output),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def test_acceptance_report_is_narrow_and_fail_closed(tmp_path: Path) -> None:
    output = tmp_path / "acceptance.json"
    result = run_checker(EVIDENCE, output)
    assert result.returncode == 0, result.stderr
    document = json.loads(output.read_text(encoding="utf-8"))
    assert document["status"] == (
        "J2_RIGHT_BASE_BIDIRECTIONAL_75_PERCENT_ACTIVE_PASS"
    )
    assert document["actively_validated_targets_raw"] == [1391, 2711]
    assert document["approved_command_limits_raw"] == [1172, 2932]
    assert document["endpoint_commands_forbidden"] is True
    assert document["motion_authorized"] is False
    assert document["general_trajectory_authorized"] is False
    assert document["runtime_limit_promotion_authorized"] is False
    assert document["interpretation"]["full_approved_endpoints_physically_tested"] is False


def test_acceptance_rejects_sha_tampering(tmp_path: Path) -> None:
    evidence = tmp_path / "evidence"
    evidence.mkdir()
    for name in REQUIRED_INPUTS:
        shutil.copyfile(EVIDENCE / name, evidence / name)
    tampered = evidence / "j2_right_base_upper75_run01.json"
    tampered.write_bytes(tampered.read_bytes() + b"\n")

    result = run_checker(evidence, tmp_path / "rejected.json")
    assert result.returncode != 0
    assert "SHA mismatch" in result.stderr


def test_acceptance_tool_cannot_reach_motion_apis() -> None:
    source = TOOL.read_text(encoding="utf-8")
    assert "--plan-only is required" in source
    for forbidden in (
        "import rclpy",
        "RightArmJogOnce",
        "RightArmTorqueEnableOnce",
        "create_client",
        "call_async",
    ):
        assert forbidden not in source
