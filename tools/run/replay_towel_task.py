#!/usr/bin/env python3
"""Replay a towel observation sequence without creating execution clients."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.towel_task_replay import replay_towel_task  # noqa: E402
from tools.lib.towel_task_runtime import (  # noqa: E402
    TowelTaskContractError,
    load_towel_contract,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence", type=Path)
    parser.add_argument(
        "--contract",
        type=Path,
        default=Path("config/towel_task_contract.candidate.yaml"),
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        contract = load_towel_contract(args.contract)
        loaded = json.loads(args.sequence.read_text(encoding="utf-8"))
        observations = (
            loaded.get("observations")
            if isinstance(loaded, dict)
            else loaded
        )
        if not isinstance(observations, list):
            raise TowelTaskContractError(
                "replay input must be a list or an observations object"
            )
        result = replay_towel_task(contract, observations)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    except (OSError, json.JSONDecodeError, TowelTaskContractError) as exc:
        print(f"[FAIL] {exc}")
        return 1
    print(
        f"[PASS] terminal_phase={result['terminal_phase']} "
        f"motion_commands={result['motion_commands']} output={args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
