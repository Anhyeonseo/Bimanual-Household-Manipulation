#!/usr/bin/env python3
"""Run one freshly perceived left-to-right pen transfer without retries."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import os
from pathlib import Path
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
CONFIRMATION = "RUN_LEFT_RIGHT_PEN_TRANSFER_ONCE"
RIGHT_PLACE_CONFIRMATION = "RUN_RIGHT_PLACE_HEIGHT_CHECK_ONCE"
EXPECTED_DUAL_URDF_SHA256 = (
    "72a392dddf0a7cbec40cd1718c2a0be7604fd3aaccfb1a564cc8fca015b1e495"
)


def file_sha256(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirmation", required=True)
    parser.add_argument("--right-place-validation", required=True)
    parser.add_argument("--target-samples", type=int, default=7)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    parser.add_argument("--perception-settle-s", type=float, default=2.0)
    parser.add_argument(
        "--dual-urdf",
        type=Path,
        default=(
            ROOT
            / "ros2_ws/install/so101_description/share/"
            "so101_description/urdf/so101_dual_right_data_fit_candidate.urdf"
        ),
    )
    parser.add_argument(
        "--session-directory",
        type=Path,
        default=(
            ROOT
            / "artifacts/top_pick_place/2026-08-16/"
            "pen_interarm_continuous_session01"
        ),
    )
    args = parser.parse_args()
    if args.confirmation != CONFIRMATION:
        parser.error(f"--confirmation must be {CONFIRMATION}")
    if args.right_place_validation != RIGHT_PLACE_CONFIRMATION:
        parser.error(
            f"--right-place-validation must be {RIGHT_PLACE_CONFIRMATION}"
        )
    if args.target_samples < 5:
        parser.error("--target-samples must be at least 5")
    if args.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive")
    if not 1.0 <= args.perception_settle_s <= 10.0:
        parser.error("--perception-settle-s must be within 1..10")
    return args


def run_stage(label: str, command: list[str], environment: dict[str, str]) -> None:
    print(f"LEFT_RIGHT_PEN_TRANSFER_STAGE_START label={label}", flush=True)
    completed = subprocess.run(command, cwd=ROOT, env=environment, check=False)
    if completed.returncode != 0:
        raise RuntimeError(
            f"stage failed without retry: label={label} exit={completed.returncode}"
        )
    print(f"LEFT_RIGHT_PEN_TRANSFER_STAGE_PASS label={label}", flush=True)


def require_plan_side(path: Path, expected_side: str) -> dict:
    document = json.loads(path.read_text(encoding="utf-8"))
    actual = document.get("routing", {}).get("selected_arm")
    if document.get("schema_version") != 12 or actual != expected_side:
        raise RuntimeError(
            f"fresh plan routing mismatch: expected={expected_side} actual={actual}"
        )
    return document


def write_journal(path: Path, document: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def main() -> int:
    args = parse_args()
    dual_urdf = args.dual_urdf.resolve()
    if not dual_urdf.is_file():
        raise RuntimeError(f"dual URDF is missing: {dual_urdf}")
    actual_urdf_sha = file_sha256(dual_urdf)
    if actual_urdf_sha != EXPECTED_DUAL_URDF_SHA256:
        raise RuntimeError(
            "right data-fit URDF hash mismatch: "
            f"expected={EXPECTED_DUAL_URDF_SHA256} actual={actual_urdf_sha}"
        )
    session = args.session_directory.resolve()
    if session.exists() and any(session.iterdir()):
        raise RuntimeError(f"session directory is not empty: {session}")
    session.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ)
    environment["SO101_DUAL_URDF_PATH"] = str(dual_urdf)
    planner = str(ROOT / "tools/plan_top_camera_pick_place_once.py")
    runner = str(ROOT / "tools/run_top_pick_place_application_once.py")
    journal_path = session / "transfer_journal.json"
    journal = {
        "schema_version": 1,
        "record_kind": "left_right_pen_transfer_once",
        "status": "started",
        "automatic_retry_count": 0,
        "operator_confirmation": args.confirmation,
        "right_place_validation": args.right_place_validation,
        "dual_urdf": {"path": str(dual_urdf), "sha256": actual_urdf_sha},
        "stages": [],
    }
    write_journal(journal_path, journal)

    def record(label: str, outputs: dict[str, str]) -> None:
        journal["status"] = label
        journal["stages"].append({"label": label, **outputs})
        write_journal(journal_path, journal)

    left_plan = session / "left_plan.json"
    run_stage(
        "left_plan",
        [
            sys.executable,
            planner,
            "--plan-only",
            "--interarm-place",
            "--target-samples",
            str(args.target_samples),
            "--timeout-s",
            str(args.timeout_s),
            "--output",
            str(left_plan),
        ],
        environment,
    )
    require_plan_side(left_plan, "left")
    left_plan_sha = file_sha256(left_plan)
    record("left_plan_pass", {"plan": str(left_plan), "sha256": left_plan_sha})

    left_validate = session / "left_validate.json"
    run_stage(
        "left_validate",
        [
            sys.executable,
            runner,
            "--validate-only",
            "--plan",
            str(left_plan),
            "--plan-sha256",
            left_plan_sha,
            "--output",
            str(left_validate),
        ],
        environment,
    )
    record("left_validate_pass", {"output": str(left_validate)})

    left_execute = session / "left_execute.json"
    run_stage(
        "left_execute",
        [
            sys.executable,
            runner,
            "--confirmation",
            "RUN_TOP_CAMERA_RESIDENT_PICK_PLACE_ONCE",
            "--plan",
            str(left_plan),
            "--plan-sha256",
            left_plan_sha,
            "--output",
            str(left_execute),
        ],
        environment,
    )
    record(
        "left_execute_pass",
        {"output": str(left_execute), "sha256": file_sha256(left_execute)},
    )

    time.sleep(args.perception_settle_s)

    right_plan = session / "right_plan.json"
    run_stage(
        "right_plan",
        [
            sys.executable,
            planner,
            "--plan-only",
            "--target-samples",
            str(args.target_samples),
            "--timeout-s",
            str(args.timeout_s),
            "--output",
            str(right_plan),
        ],
        environment,
    )
    require_plan_side(right_plan, "right")
    right_plan_sha = file_sha256(right_plan)
    record("right_plan_pass", {"plan": str(right_plan), "sha256": right_plan_sha})

    right_validate = session / "right_validate.json"
    run_stage(
        "right_validate",
        [
            sys.executable,
            runner,
            "--validate-only",
            "--plan",
            str(right_plan),
            "--plan-sha256",
            right_plan_sha,
            "--output",
            str(right_validate),
        ],
        environment,
    )
    record("right_validate_pass", {"output": str(right_validate)})

    right_execute = session / "right_execute.json"
    run_stage(
        "right_execute",
        [
            sys.executable,
            runner,
            "--confirmation",
            "RUN_TOP_CAMERA_RESIDENT_PICK_PLACE_ONCE",
            "--right-place-validation",
            args.right_place_validation,
            "--plan",
            str(right_plan),
            "--plan-sha256",
            right_plan_sha,
            "--output",
            str(right_execute),
        ],
        environment,
    )
    record(
        "right_execute_pass",
        {"output": str(right_execute), "sha256": file_sha256(right_execute)},
    )
    journal["status"] = "LEFT_RIGHT_PEN_TRANSFER_ONCE_PASS"
    journal["completed_at_unix_s"] = time.time()
    write_journal(journal_path, journal)
    print(
        "LEFT_RIGHT_PEN_TRANSFER_ONCE_PASS "
        f"stages={len(journal['stages'])} output={journal_path} "
        f"sha256={file_sha256(journal_path)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
