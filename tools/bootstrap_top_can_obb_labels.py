#!/usr/bin/env python3
"""Create review-first OBB proposals for the red lying-can capture set.

The can is red and the capture board is nearly neutral, so HSV segmentation is
used only to bootstrap labels.  The script does not write training labels
unless ``--write-labels`` is supplied.  It always emits proposal JSON and
contact sheets for visual review.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

from label_top_can_obb import CLASS_NAME, Case, file_sha256, label_text, order_box_points, parse_cases


LONG_AXIS_EXPANSION = 1.18
SHORT_AXIS_EXPANSION = 1.10


def expanded_box(rect: tuple[tuple[float, float], tuple[float, float], float]) -> np.ndarray:
    center, dimensions, angle = rect
    width, height = dimensions
    if width <= 0.0 or height <= 0.0:
        raise ValueError("empty candidate rectangle")
    if width >= height:
        width *= LONG_AXIS_EXPANSION
        height *= SHORT_AXIS_EXPANSION
    else:
        width *= SHORT_AXIS_EXPANSION
        height *= LONG_AXIS_EXPANSION
    return order_box_points(cv2.boxPoints((center, (width, height), angle)).astype(np.float32))


def fit_box_to_image(points: np.ndarray, width: int, height: int) -> np.ndarray:
    """Shrink an expanded OBB only when its corners cross the image edge."""
    center = np.mean(points, axis=0)
    scale = 1.0
    limits = (float(width - 1), float(height - 1))
    for point in points:
        for axis, limit in enumerate(limits):
            delta = float(point[axis] - center[axis])
            if delta > 0.0:
                scale = min(scale, (limit - float(center[axis])) / delta)
            elif delta < 0.0:
                scale = min(scale, -float(center[axis]) / delta)
    if scale < 0.0:
        raise ValueError("candidate OBB center is outside the image")
    return order_box_points((center + min(1.0, scale) * (points - center)).astype(np.float32))


def red_candidates(image: np.ndarray) -> list[dict]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red_low = cv2.inRange(hsv, (0, 65, 35), (14, 255, 255))
    red_high = cv2.inRange(hsv, (160, 65, 35), (180, 255, 255))
    mask = cv2.bitwise_or(red_low, red_high)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < 350.0:
            continue
        rect = cv2.minAreaRect(contour)
        width, height = rect[1]
        long_side = max(width, height)
        short_side = min(width, height)
        if short_side <= 0.0:
            continue
        aspect = long_side / short_side
        if not (55.0 <= long_side <= 220.0 and 16.0 <= short_side <= 110.0 and 1.25 <= aspect <= 5.0):
            continue
        points = expanded_box(rect)
        polygon = np.round(points).astype(np.int32)
        candidate_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.fillConvexPoly(candidate_mask, polygon, 255)
        selected = candidate_mask > 0
        red_fraction = float(np.mean(mask[selected] > 0)) if np.any(selected) else 0.0
        center_x, center_y = rect[0]
        height_px, width_px = image.shape[:2]
        edge_margin = min(center_x, center_y, width_px - center_x, height_px - center_y)
        aspect_score = math.exp(-abs(math.log(aspect / 2.31)))
        score = 0.001 * area + 2.5 * red_fraction + 0.4 * aspect_score + min(edge_margin, 100.0) / 1000.0
        candidates.append(
            {
                "points": points,
                "score": score,
                "red_area_px": area,
                "red_fraction": red_fraction,
                "long_side_px": long_side,
                "short_side_px": short_side,
                "aspect": aspect,
            }
        )
    return sorted(candidates, key=lambda candidate: candidate["score"], reverse=True)


def draw_review(image: np.ndarray, case: Case, proposal: dict | None, ordinal: int) -> np.ndarray:
    review = image.copy()
    color = (0, 220, 0) if case.expected_present else (0, 170, 255)
    if proposal:
        points = np.round(proposal["points"]).astype(np.int32)
        cv2.polylines(review, [points], True, color, 3, cv2.LINE_AA)
        for index, point in enumerate(points, start=1):
            cv2.circle(review, tuple(point), 4, (0, 200, 255), -1, cv2.LINE_AA)
            cv2.putText(review, str(index), (int(point[0]) + 5, int(point[1]) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
    title = f"{ordinal:03d} {'CAN' if case.expected_present else 'EMPTY'} {case.capture_id}"
    suffix = "NO PROPOSAL" if proposal is None else f"score={proposal['score']:.2f} red={proposal['red_fraction']:.2f}"
    cv2.rectangle(review, (0, 0), (review.shape[1], 42), (0, 0, 0), -1)
    cv2.putText(review, title, (7, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(review, suffix, (7, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
    return review


def write_review_sheets(reviews: list[np.ndarray], directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    width, height = 320, 240
    columns, rows = 4, 4
    for start in range(0, len(reviews), columns * rows):
        page = np.full((rows * height, columns * width, 3), 32, dtype=np.uint8)
        for offset, review in enumerate(reviews[start : start + columns * rows]):
            tile = cv2.resize(review, (width, height), interpolation=cv2.INTER_AREA)
            row, column = divmod(offset, columns)
            page[row * height : (row + 1) * height, column * width : (column + 1) * width] = tile
        cv2.imwrite(str(directory / f"proposal_review_{start // (columns * rows) + 1:02d}.jpg"), page)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=Path("datasets/top_can_obb_source"))
    parser.add_argument("--labels-dir", type=Path, default=None)
    parser.add_argument("--proposal-json", type=Path, required=True)
    parser.add_argument("--review-dir", type=Path, required=True)
    parser.add_argument("--write-labels", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        root = args.source_root.resolve()
        labels_dir = (args.labels_dir or root / "labels").resolve()
        proposal_json = args.proposal_json.resolve()
        review_dir = args.review_dir.resolve()
        for path in (labels_dir, proposal_json, review_dir):
            try:
                path.relative_to(root)
            except ValueError as error:
                raise ValueError("all outputs must stay under source root") from error
        cases = parse_cases(root, verify_sha256=True)
        proposals = []
        reviews = []
        failures = []
        for ordinal, case in enumerate(cases, start=1):
            image = cv2.imread(str(case.image), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"failed to decode {case.image}")
            selected = None
            if case.expected_present:
                candidates = red_candidates(image)
                if candidates:
                    selected = candidates[0]
                    selected["points"] = fit_box_to_image(
                        selected["points"], image.shape[1], image.shape[0]
                    )
                else:
                    failures.append(case.capture_id)
                proposals.append(
                    {
                        "capture_id": case.capture_id,
                        "image": case.image.name,
                        "image_sha256": case.image_sha256,
                        "candidate_count": len(candidates),
                        "selected": None if selected is None else {
                            key: (value.tolist() if key == "points" else value)
                            for key, value in selected.items()
                        },
                    }
                )
                if args.write_labels and selected is not None:
                    path = labels_dir / f"{case.capture_id}.txt"
                    if args.overwrite or not path.exists():
                        labels_dir.mkdir(parents=True, exist_ok=True)
                        path.write_text(label_text(selected["points"], case.width, case.height), encoding="utf-8")
            elif args.write_labels:
                path = labels_dir / f"{case.capture_id}.txt"
                if args.overwrite or not path.exists():
                    labels_dir.mkdir(parents=True, exist_ok=True)
                    path.write_text("", encoding="utf-8")
            reviews.append(draw_review(image, case, selected, ordinal))
        document = {
            "schema_version": 1,
            "dataset_id": "dual_arm_can_disposal_v1",
            "class_names": [CLASS_NAME],
            "method": "hsv_red_bootstrap_review_required",
            "long_axis_expansion": LONG_AXIS_EXPANSION,
            "short_axis_expansion": SHORT_AXIS_EXPANSION,
            "captures": len(cases),
            "positive_captures": sum(case.expected_present for case in cases),
            "negative_captures": sum(not case.expected_present for case in cases),
            "proposal_failures": failures,
            "proposals": proposals,
        }
        proposal_json.parent.mkdir(parents=True, exist_ok=True)
        proposal_json.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        write_review_sheets(reviews, review_dir)
        print(f"TOP_CAN_OBB_BOOTSTRAP_PASS captures={len(cases)} proposal_failures={len(failures)} labels_written={args.write_labels}")
        print(f"PROPOSALS={proposal_json}")
        print(f"PROPOSALS_SHA256={file_sha256(proposal_json)}")
        print(f"REVIEW_DIR={review_dir}")
        return 0 if not failures else 2
    except Exception as error:
        print(f"TOP_CAN_OBB_BOOTSTRAP_ERROR reason={error}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
