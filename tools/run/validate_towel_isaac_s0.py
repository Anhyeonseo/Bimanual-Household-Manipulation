#!/usr/bin/env python3
"""Validate pinned R2 S0 inputs without claiming an Isaac simulation run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.towel_isaac_s0 import validate_s0_contract  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--contract", type=Path, default=ROOT / "config/towel_isaac_s0.json"
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = validate_s0_contract(args.contract, ROOT)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(
        f"{report['status']} envs={report['environment_count']} "
        f"segments={report['planning_segment_count']} "
        f"strict_states={report['strict_state_sample_count']} "
        f"isaac_lab={str(report['isaac_lab_available_in_current_python']).lower()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
