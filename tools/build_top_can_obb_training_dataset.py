#!/usr/bin/env python3
"""Build a capture-disjoint Top-can OBB train/validation/holdout dataset."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
from collections import defaultdict
from pathlib import Path

from label_top_can_obb import Case, file_sha256, parse_cases, parse_label


def stable_order(cases: list[Case]) -> list[Case]:
    return sorted(
        cases,
        key=lambda case: hashlib.sha256(case.capture_id.encode("utf-8")).hexdigest(),
    )


def split_cases(cases: list[Case]) -> dict[str, list[Case]]:
    """Split each presence/condition group 70/15/15 without frame leakage."""
    groups: dict[tuple[bool, str, str], list[Case]] = defaultdict(list)
    for case in cases:
        groups[(case.expected_present, case.condition["lighting"], case.condition["glare"])].append(case)
    result = {"train": [], "validation": [], "holdout": []}
    for group_cases in groups.values():
        ordered = stable_order(group_cases)
        size = len(ordered)
        validation_count = max(1, round(size * 0.15))
        holdout_count = max(1, round(size * 0.15))
        if validation_count + holdout_count >= size:
            raise ValueError("condition group is too small for three capture-disjoint splits")
        result["validation"].extend(ordered[:validation_count])
        result["holdout"].extend(ordered[validation_count : validation_count + holdout_count])
        result["train"].extend(ordered[validation_count + holdout_count :])
    return {name: sorted(items, key=lambda case: case.capture_id) for name, items in result.items()}


def materialize(source: Path, destination: Path) -> str:
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source, destination)
        return "hardlink"
    except OSError:
        shutil.copy2(source, destination)
        return "copy"


def manifest_case(root: Path, case: Case, label: Path, split: str) -> dict:
    present, _ = parse_label(label, case.width, case.height)
    if present != case.expected_present:
        raise ValueError(f"metadata/label presence mismatch: {case.capture_id}")
    return {
        "id": case.capture_id,
        "split": split,
        "image": str((Path("images") / ("val" if split == "validation" else split) / case.image.name)),
        "image_sha256": file_sha256(case.image),
        "label": str((Path("labels") / ("val" if split == "validation" else split) / f"{case.image.stem}.txt")),
        "label_sha256": file_sha256(label),
        "expected_present": case.expected_present,
        "condition": case.condition,
        "source_record": case.source_record,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=Path("datasets/top_can_obb_source"))
    parser.add_argument("--labels-dir", type=Path, default=None)
    parser.add_argument("--output-root", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        source_root = args.source_root.resolve()
        labels_dir = (args.labels_dir or source_root / "labels").resolve()
        output_root = args.output_root.resolve()
        if output_root.exists():
            raise ValueError(f"output root already exists: {output_root}")
        cases = parse_cases(source_root, verify_sha256=True)
        for case in cases:
            label = labels_dir / f"{case.capture_id}.txt"
            if not label.is_file():
                raise ValueError(f"label is missing: {label}")
            parse_label(label, case.width, case.height)
        splits = split_cases(cases)
        copies = {"hardlink": 0, "copy": 0}
        manifests = {"train": [], "validation": [], "holdout": []}
        for split, split_cases_list in splits.items():
            directory_name = "val" if split == "validation" else split
            for case in split_cases_list:
                image_destination = output_root / "images" / directory_name / case.image.name
                label_source = labels_dir / f"{case.capture_id}.txt"
                label_destination = output_root / "labels" / directory_name / f"{case.image.stem}.txt"
                copies[materialize(case.image, image_destination)] += 1
                copies[materialize(label_source, label_destination)] += 1
                manifests[split].append(manifest_case(output_root, case, label_source, split))
        data_yaml = output_root / "data.yaml"
        data_yaml.write_text(
            "\n".join(
                [
                    f"path: {output_root}",
                    "train: images/train",
                    "val: images/val",
                    "test: images/holdout",
                    "names:",
                    "  0: can",
                    "",
                ]
            ),
            encoding="utf-8",
        )
        training_manifest = {
            "protocol_version": 1,
            "dataset_id": "top_can_obb_training_v1",
            "dataset_role": "training",
            "class_names": ["can"],
            "yaw_semantics": "undirected_long_axis_modulo_pi",
            "ultralytics_data_yaml": "data.yaml",
            "splits": {"train": manifests["train"], "validation": manifests["validation"]},
        }
        holdout_manifest = {
            "protocol_version": 1,
            "dataset_id": "top_can_obb_holdout_v1",
            "dataset_role": "holdout",
            "class_names": ["can"],
            "yaw_semantics": "undirected_long_axis_modulo_pi",
            "cases": manifests["holdout"],
        }
        for path, document in ((output_root / "training_manifest.json", training_manifest), (output_root / "holdout_manifest.json", holdout_manifest)):
            path.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        counts = {
            split: {
                "positive": sum(case.expected_present for case in split_cases_list),
                "negative": sum(not case.expected_present for case in split_cases_list),
                "total": len(split_cases_list),
            }
            for split, split_cases_list in splits.items()
        }
        report = {
            "protocol_version": 1,
            "status": "TOP_CAN_OBB_DATASET_BUILD_PASS",
            "motion_authorized": False,
            "class_names": ["can"],
            "selection": "sharpest_frame_per_capture",
            "capture_disjoint_splits": True,
            "counts": counts,
            "materialization": copies,
            "data_yaml_sha256": file_sha256(data_yaml),
            "training_manifest_sha256": file_sha256(output_root / "training_manifest.json"),
            "holdout_manifest_sha256": file_sha256(output_root / "holdout_manifest.json"),
        }
        report_path = output_root / "build_report.json"
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print("TOP_CAN_OBB_DATASET_BUILD_PASS")
        print(f"OUTPUT={output_root}")
        print(f"COUNTS={json.dumps(counts, sort_keys=True)}")
        print(f"BUILD_REPORT_SHA256={file_sha256(report_path)}")
        return 0
    except Exception as error:
        print(f"TOP_CAN_OBB_DATASET_BUILD_ERROR reason={error}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
