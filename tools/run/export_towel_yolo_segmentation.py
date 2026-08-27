#!/usr/bin/env python3
"""Export human-reviewed towel polygons as a split-safe YOLO-seg dataset."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.towel_yolo_segmentation import (  # noqa: E402
    TowelYoloExportError,
    collect_review_roots,
    export_reviewed_yolo_dataset,
)


DEFAULT_REVIEW_ROOTS = (
    ROOT / "datasets/towel_yolo_annotations/20260827_pilot_reviewed",
    ROOT / "datasets/towel_yolo_annotations/20260827_review_batch2",
    ROOT / "datasets/towel_yolo_annotations/20260827_validation_reviewed",
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "review_roots",
        type=Path,
        nargs="*",
        default=DEFAULT_REVIEW_ROOTS,
        help="review directories; defaults to the three approved R1 batches",
    )
    parser.add_argument(
        "--source-root",
        type=Path,
        default=ROOT / "datasets/towel_yolo_source",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "tmp/towel_yolo_segmentation",
    )
    args = parser.parse_args()
    try:
        manifest = export_reviewed_yolo_dataset(
            collect_review_roots(args.review_roots),
            source_root=args.source_root.resolve(),
            output_root=args.output.resolve(),
        )
    except (OSError, TowelYoloExportError) as exc:
        print(f"TOWEL_YOLO_SEGMENTATION_EXPORT_FAIL {exc}")
        return 1
    print(
        "TOWEL_YOLO_SEGMENTATION_EXPORT_PASS "
        f"items={manifest['item_count']} "
        f"train={manifest['split_counts']['train']} "
        f"val={manifest['split_counts']['validation']} "
        f"empty_train={manifest['empty_negative_counts']['train']} "
        f"empty_val={manifest['empty_negative_counts']['validation']} "
        f"roundtrip_iou_min={manifest['minimum_roundtrip_iou']:.6f} "
        f"output={args.output.resolve()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
