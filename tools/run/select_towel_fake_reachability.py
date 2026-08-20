#!/usr/bin/env python3
"""Select a motion-free towel fold candidate from a fake reachability fixture."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.lib.towel_fake_reachability import evaluate_fake_reachability  # noqa: E402
from tools.lib.towel_task_runtime import TowelTaskContractError  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "fixture", nargs="?", type=Path,
        default=Path("config/towel_fake_reachability.example.json"),
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        raw = args.fixture.read_bytes()
        fixture = json.loads(raw.decode("utf-8"))
        if not isinstance(fixture, dict) or fixture.get("schema_version") != 1:
            raise TowelTaskContractError("fixture schema_version must be 1")
        if fixture.get("record_kind") != "towel_fake_reachability_fixture":
            raise TowelTaskContractError("invalid fixture record_kind")
        candidates = fixture.get("candidates")
        if not isinstance(candidates, list):
            raise TowelTaskContractError("fixture candidates must be a list")
        result = evaluate_fake_reachability(
            candidates, fixture_sha256=sha256(raw).hexdigest()
        )
    except (OSError, json.JSONDecodeError, TowelTaskContractError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 2
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(
        f"[PASS] {result['status']} selected={result['selected_candidate_id']} "
        "motion_commands=0"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
