#!/usr/bin/env python3
"""Build the motion-free host contract for the R2 Isaac S0 smoke test."""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.towel_isaac_s0 import (  # noqa: E402
    S0_STATUS,
    TowelIsaacS0Error,
    build_s0_vectorized_manifest,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="strict MoveIt plan-only JSON")
    parser.add_argument("--environments", type=int, default=8)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not args.source.is_file():
        parser.error(f"source does not exist: {args.source}")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")
    return args


def main() -> int:
    args = parse_args()
    source_bytes = args.source.read_bytes()
    document = json.loads(source_bytes)
    if not isinstance(document, dict):
        raise TowelIsaacS0Error("source document root must be a mapping")
    manifest = build_s0_vectorized_manifest(
        document,
        source_sha256=sha256(source_bytes).hexdigest(),
        environment_count=args.environments,
        seed=args.seed,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"{S0_STATUS} environments={args.environments} seed={args.seed} "
        "s0_smoke_test_passed=false motion_commands=0 "
        f"output={args.output}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(1) from None
