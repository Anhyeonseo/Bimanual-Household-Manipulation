#!/usr/bin/env python3
"""Validate the motion-locked square-towel task contract."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.towel_task_runtime import (  # noqa: E402
    TowelTaskContractError,
    load_towel_contract,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "path",
        nargs="?",
        type=Path,
        default=Path("config/towel_task_contract.candidate.yaml"),
    )
    args = parser.parse_args()
    try:
        contract = load_towel_contract(args.path)
        artifacts = contract.get("software_artifacts")
        if not isinstance(artifacts, dict) or not artifacts:
            raise TowelTaskContractError("software_artifacts must be an object")
        resolved_root = ROOT.resolve()
        for name, raw_path in artifacts.items():
            if not isinstance(raw_path, str) or not raw_path:
                raise TowelTaskContractError(
                    f"software artifact {name} must be a path"
                )
            relative_path = Path(raw_path)
            resolved_path = (resolved_root / relative_path).resolve()
            if (
                relative_path.is_absolute()
                or resolved_root not in resolved_path.parents
                or not resolved_path.is_file()
            ):
                raise TowelTaskContractError(
                    f"software artifact {name} does not resolve to a repository file"
                )
    except TowelTaskContractError as exc:
        print(f"[FAIL] {exc}")
        return 1
    null_hardware = sum(
        value is None
        for name, value in contract["hardware_limits"].items()
        if name != "provenance"
    )
    print(
        "[PASS] Towel contract is motion-locked; "
        f"{null_hardware} hardware limits remain unmeasured."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
