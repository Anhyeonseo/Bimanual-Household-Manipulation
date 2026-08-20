#!/usr/bin/env python3
"""Validate towel annotation files and emit a deterministic manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.towel_dataset import (  # noqa: E402
    TowelDatasetError,
    build_dataset_manifest,
    load_annotation,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("annotations", type=Path, nargs="+")
    parser.add_argument("--dataset-root", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    paths = []
    for value in args.annotations:
        if value.is_dir():
            paths.extend(sorted(value.rglob("*.json")))
        else:
            paths.append(value)
    try:
        if not paths:
            raise TowelDatasetError("no annotation JSON files found")
        manifest = build_dataset_manifest(
            (load_annotation(path) for path in paths),
            dataset_root=args.dataset_root,
        )
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
    except (OSError, TowelDatasetError) as exc:
        print(f"[FAIL] {exc}")
        return 1
    print(
        f"[PASS] {manifest['annotation_count']} annotations; "
        f"items_sha256={manifest['items_sha256']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
