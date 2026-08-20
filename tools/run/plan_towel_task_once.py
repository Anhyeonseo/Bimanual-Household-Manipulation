#!/usr/bin/env python3
"""Create a motion-free next-step and fold plan from one towel observation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.towel_task_planning import (  # noqa: E402
    build_towel_plan,
    load_json_object,
    sha256_file,
)
from tools.lib.towel_task_runtime import (  # noqa: E402
    TowelTaskContractError,
    load_towel_contract,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("observation", type=Path)
    parser.add_argument(
        "--contract",
        type=Path,
        default=Path("config/towel_task_contract.candidate.yaml"),
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        contract = load_towel_contract(args.contract)
        observation = load_json_object(args.observation)
        plan = build_towel_plan(
            contract,
            observation,
            contract_sha256=sha256_file(args.contract),
            observation_sha256=sha256_file(args.observation),
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(plan, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    except (OSError, TowelTaskContractError) as exc:
        print(f"[FAIL] {exc}")
        return 1
    print(
        f"[PASS] {plan['estimated_state']} -> {plan['next_phase']}; "
        f"motion_commands={plan['motion_commands']} output={args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
