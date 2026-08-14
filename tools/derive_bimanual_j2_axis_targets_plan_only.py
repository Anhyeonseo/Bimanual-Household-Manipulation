#!/usr/bin/env python3
"""Derive two-sided 25/50/75% J2 arm-axis targets without motion APIs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from bimanual_j2_targets import (
    STATUS,
    derive_targets,
    file_sha256,
    load_bound_json,
)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument(
        "--approved",
        type=Path,
        default=root / "config/bimanual_j1_operational_limits.approved.json",
    )
    parser.add_argument("--approved-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not args.plan_only:
        raise SystemExit("--plan-only is required")

    approved = load_bound_json(
        args.approved,
        args.approved_sha256,
        "approved J1-L manifest",
    )
    document = derive_targets(approved)
    document["inputs"] = {
        "approved": {
            "path": str(args.approved),
            "sha256": args.approved_sha256.lower(),
        }
    }
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        f"STATUS={STATUS} arm_joints=10 fractions=25,50,75 "
        f"endpoints=false motion_authorized=false output={output} "
        f"sha256={file_sha256(output)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
