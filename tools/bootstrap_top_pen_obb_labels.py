#!/usr/bin/env python3
"""Bootstrap single-pen OBB labels from matching pen-free reference frames.

The tool is deliberately review-first.  It always emits proposal JSON and
review sheets, but writes Ultralytics OBB labels only with --write-labels.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return document


def resolve_under(root: Path, value: object, field: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a non-empty relative path")
    resolved = (root / value).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ValueError(f"{field} must stay inside the dataset root") from error
    return resolved


def order_box_points(points: np.ndarray) -> np.ndarray:
    center = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    ordered = points[np.argsort(angles)]
    start = int(np.argmin(ordered[:, 0] + ordered[:, 1]))
    return np.roll(ordered, -start, axis=0)


def candidate_rectangles(image: np.ndarray, reference: np.ndarray) -> list[dict]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    reference_gray = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    reference_gray = cv2.GaussianBlur(reference_gray, (3, 3), 0)
    signed_delta = reference_gray.astype(np.int16) - gray.astype(np.int16)
    absolute_delta = cv2.absdiff(reference_gray, gray)
    dark = cv2.inRange(gray, 0, 122)
    if float(np.mean(dark > 0)) > 0.10:
        dark = np.uint8(signed_delta > 50) * 255
    dark = cv2.morphologyEx(
        dark,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)),
    )
    dark = cv2.morphologyEx(
        dark,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    signed_delta = reference_gray.astype(np.int16) - gray.astype(np.int16)
    absolute_delta = cv2.absdiff(reference_gray, gray)
    results = []
    for contour in contours:
        contour_area = float(cv2.contourArea(contour))
        if contour_area < 250.0:
            continue
        rect = cv2.minAreaRect(contour)
        width, height = rect[1]
        long_side = max(float(width), float(height))
        short_side = min(float(width), float(height))
        if short_side <= 0.0:
            continue
        aspect = long_side / short_side
        if not (55.0 <= long_side <= 220.0):
            continue
        if not (8.0 <= short_side <= 65.0):
            continue
        if aspect < 2.25:
            continue
        points = order_box_points(cv2.boxPoints(rect).astype(np.float32))
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.fillConvexPoly(mask, np.round(points).astype(np.int32), 255)
        selected = mask > 0
        if not np.any(selected):
            continue
        new_dark_fraction = float(
            np.mean((signed_delta[selected] > 18) & (gray[selected] < 122))
        )
        change_fraction = float(np.mean(absolute_delta[selected] > 18))
        dark_fraction = float(np.mean(gray[selected] < 122))
        positive_delta = float(np.mean(np.maximum(signed_delta[selected], 0)))
        aspect_score = math.exp(-abs(math.log(max(aspect, 1e-6) / 5.0)))
        score = (
            4.5 * new_dark_fraction
            + 1.5 * change_fraction
            + 0.010 * positive_delta
            + 0.35 * dark_fraction
            + 0.20 * aspect_score
        )
        results.append(
            {
                "points": points,
                "score": score,
                "long_side_px": long_side,
                "short_side_px": short_side,
                "aspect": aspect,
                "new_dark_fraction": new_dark_fraction,
                "change_fraction": change_fraction,
                "dark_fraction": dark_fraction,
                "positive_delta": positive_delta,
            }
        )
    return sorted(results, key=lambda item: item["score"], reverse=True)


def choose_reference(case: dict, negative_cases: list[dict]) -> dict:
    condition = case["condition"]
    exact = [
        item
        for item in negative_cases
        if item["condition"]["background"] == condition["background"]
        and item["condition"]["lighting"] == condition["lighting"]
    ]
    if not exact:
        raise ValueError(f"no background/lighting negative for {case['id']}")
    train = [item for item in exact if item["split"] == "train"]
    return (train or exact)[0]


def label_text(points: np.ndarray, width: int, height: int) -> str:
    normalized = points.copy().astype(np.float64)
    normalized[:, 0] /= float(width)
    normalized[:, 1] /= float(height)
    normalized = np.clip(normalized, 0.0, 1.0)
    values = " ".join(f"{value:.8f}" for value in normalized.reshape(-1))
    return f"0 {values}\n"


def draw_review(image: np.ndarray, case_id: str, proposal: dict) -> np.ndarray:
    review = image.copy()
    points = np.round(proposal["points"]).astype(np.int32)
    cv2.polylines(review, [points], True, (0, 255, 0), 3, cv2.LINE_AA)
    for index, point in enumerate(points):
        cv2.circle(review, tuple(point), 4, (0, 180, 255), -1, cv2.LINE_AA)
        cv2.putText(
            review,
            str(index + 1),
            (int(point[0]) + 5, int(point[1]) - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )
    title = f"{case_id} score={proposal['score']:.2f}"
    cv2.rectangle(review, (0, 0), (review.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(
        review,
        title,
        (7, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.46,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return review


def write_review_sheets(review_images: list[np.ndarray], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    tile_width, tile_height = 320, 240
    columns, rows = 4, 3
    page_size = columns * rows
    for page_start in range(0, len(review_images), page_size):
        page = np.full(
            (rows * tile_height, columns * tile_width, 3),
            230,
            dtype=np.uint8,
        )
        for offset, image in enumerate(review_images[page_start : page_start + page_size]):
            tile = cv2.resize(image, (tile_width, tile_height), interpolation=cv2.INTER_AREA)
            row, column = divmod(offset, columns)
            page[
                row * tile_height : (row + 1) * tile_height,
                column * tile_width : (column + 1) * tile_width,
            ] = tile
        page_number = page_start // page_size + 1
        cv2.imwrite(str(output_dir / f"obb_review_{page_number:02d}.jpg"), page)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--metadata", default="metadata.json")
    parser.add_argument("--review-dir", required=True, type=Path)
    parser.add_argument("--proposal-json", required=True, type=Path)
    parser.add_argument("--write-labels", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = args.dataset_root.resolve()
    metadata_path = resolve_under(root, args.metadata, "metadata")
    metadata = load_json(metadata_path)
    cases = metadata.get("cases")
    if not isinstance(cases, list):
        raise ValueError("metadata.cases must be a list")
    negative_cases = [case for case in cases if case.get("expected_present") is False]
    positive_cases = [case for case in cases if case.get("expected_present") is True]
    proposals = []
    reviews = []
    for case in positive_cases:
        image_path = resolve_under(root, case.get("image"), "image")
        reference_case = choose_reference(case, negative_cases)
        reference_path = resolve_under(root, reference_case.get("image"), "reference image")
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        reference = cv2.imread(str(reference_path), cv2.IMREAD_COLOR)
        if image is None or reference is None:
            raise ValueError(f"image decode failed for {case['id']}")
        if image.shape != reference.shape:
            raise ValueError(f"image shape mismatch for {case['id']}")
        candidates = candidate_rectangles(image, reference)
        if not candidates:
            raise ValueError(f"no OBB candidate for {case['id']}")
        selected = candidates[0]
        points = selected["points"]
        label_path = resolve_under(root, case.get("label"), "label")
        proposal = {
            "id": case["id"],
            "image": str(image_path.relative_to(root)),
            "reference": str(reference_path.relative_to(root)),
            "label": str(label_path.relative_to(root)),
            "score": selected["score"],
            "points_px": points.tolist(),
            "long_side_px": selected["long_side_px"],
            "short_side_px": selected["short_side_px"],
            "aspect": selected["aspect"],
            "new_dark_fraction": selected["new_dark_fraction"],
            "change_fraction": selected["change_fraction"],
            "candidate_count": len(candidates),
        }
        proposals.append(proposal)
        reviews.append(draw_review(image, case["id"], selected))
        if args.write_labels:
            label_path.parent.mkdir(parents=True, exist_ok=True)
            label_path.write_text(
                label_text(points, image.shape[1], image.shape[0]),
                encoding="utf-8",
            )
    args.proposal_json.parent.mkdir(parents=True, exist_ok=True)
    args.proposal_json.write_text(
        json.dumps(
            {
                "protocol_version": 1,
                "dataset_id": metadata.get("dataset_id"),
                "proposal_count": len(proposals),
                "labels_written": bool(args.write_labels),
                "proposals": proposals,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    write_review_sheets(reviews, args.review_dir)
    print("TOP_PEN_OBB_LABEL_BOOTSTRAP_PASS")
    print(f"PROPOSALS={len(proposals)}")
    print(f"LABELS_WRITTEN={int(args.write_labels)}")
    print(f"PROPOSAL_JSON={args.proposal_json.resolve()}")
    print(f"REVIEW_DIR={args.review_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
