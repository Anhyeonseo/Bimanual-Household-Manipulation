#!/usr/bin/env python3
"""Validate towel schemas and all repository-owned example instances."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.lib.towel_schemas import (  # noqa: E402
    TowelSchemaError,
    load_json,
    load_schema,
    validate_instance,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    try:
        annotation_schema = load_schema(root / "config/towel_annotation.schema.json")
        runtime_schema = load_schema(
            root / "config/towel_state_observation.schema.json"
        )
        validate_instance(
            annotation_schema,
            load_json(root / "config/towel_annotation.example.json"),
            label="towel annotation example",
        )
        validate_instance(
            runtime_schema,
            load_json(root / "config/towel_observation.example.json"),
            label="towel runtime observation example",
        )
        replay = load_json(root / "config/towel_replay.example.json")
        if not isinstance(replay, dict) or not isinstance(replay.get("observations"), list):
            raise TowelSchemaError("replay example must contain observations")
        for index, observation in enumerate(replay["observations"]):
            validate_instance(
                runtime_schema, observation, label=f"replay observation {index}"
            )
    except TowelSchemaError as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 2
    print(
        f"[PASS] 2 towel schemas; {len(replay['observations']) + 2} examples"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
