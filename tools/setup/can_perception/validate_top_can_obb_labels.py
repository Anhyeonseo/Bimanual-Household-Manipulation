#!/usr/bin/env python3
"""Validate source hashes and manual Top-can OBB labels before dataset splitting."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from label_top_can_obb import INDEX_NAME, file_sha256, parse_cases, parse_label


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=Path("datasets/top_can_obb_source"))
    parser.add_argument("--labels-dir", type=Path, default=None)
    parser.add_argument("--index", type=Path, default=None)
    parser.add_argument("--require-complete", action="store_true")
    parser.add_argument("--allow-state-overrides", action="store_true")
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        root = args.source_root.resolve()
        labels_dir = (args.labels_dir or root / "labels").resolve()
        index_path = (args.index or root / INDEX_NAME).resolve()
        cases = parse_cases(root, verify_sha256=True)
        entries = []
        missing = 0
        positives = 0
        negatives = 0
        overrides = []
        for case in cases:
            label = labels_dir / f"{case.capture_id}.txt"
            if not label.is_file():
                missing += 1
                continue
            present, _ = parse_label(label, case.width, case.height)
            positives += int(present)
            negatives += int(not present)
            if present != case.expected_present:
                overrides.append(case.capture_id)
            entries.append({
                "capture_id": case.capture_id,
                "image": case.image.name,
                "image_sha256": file_sha256(case.image),
                "label": str(label.relative_to(root)),
                "label_sha256": file_sha256(label),
                "expected_present": case.expected_present,
                "label_present": present,
            })
        if args.require_complete and missing:
            raise ValueError(f"incomplete labeling: {missing}/{len(cases)} labels are missing")
        if overrides and not args.allow_state_overrides:
            raise ValueError(f"metadata/label presence mismatch: {', '.join(overrides[:8])}")
        document = {
            "schema_version": 1,
            "dataset_id": "dual_arm_can_disposal_v1",
            "selection": "sharpest_frame_per_capture",
            "captures": len(cases),
            "labeled": len(entries),
            "missing": missing,
            "positive_labels": positives,
            "negative_labels": negatives,
            "state_overrides": overrides,
            "index_exists": index_path.is_file(),
            "entries": entries,
        }
        if args.output:
            output = args.output.resolve()
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            print(f"OUTPUT={output}")
            print(f"OUTPUT_SHA256={file_sha256(output)}")
        status = "PASS" if not missing else "PARTIAL"
        print(f"TOP_CAN_OBB_LABEL_VALIDATION_{status} captures={len(cases)} labeled={len(entries)} positives={positives} negatives={negatives} missing={missing}")
        return 0
    except Exception as error:
        print(f"TOP_CAN_OBB_LABEL_VALIDATION_ERROR reason={error}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
