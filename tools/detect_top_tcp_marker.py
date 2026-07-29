#!/usr/bin/env python3
"""Detect exactly one yellow TCP marker in a captured Top-camera frame."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


class MarkerDetectionError(RuntimeError):
    """Fail-closed yellow marker detection error."""


@dataclass(frozen=True)
class YellowMarkerConfig:
    lower_hsv: tuple[int, int, int] = (15, 80, 80)
    upper_hsv: tuple[int, int, int] = (45, 255, 255)
    min_area_px: int = 100
    max_area_px: int = 10000
    border_margin_px: int = 3

    def validate(self) -> None:
        if any(value < 0 or value > 255 for value in (*self.lower_hsv, *self.upper_hsv)):
            raise ValueError("HSV bounds must be within 0..255")
        if any(low > high for low, high in zip(self.lower_hsv, self.upper_hsv)):
            raise ValueError("lower HSV bounds must not exceed upper bounds")
        if self.min_area_px <= 0 or self.max_area_px < self.min_area_px:
            raise ValueError("invalid marker area bounds")
        if self.border_margin_px < 0:
            raise ValueError("border margin must not be negative")


def detect_yellow_marker(
    image: np.ndarray,
    config: YellowMarkerConfig = YellowMarkerConfig(),
) -> dict:
    config.validate()
    if image.ndim != 3 or image.shape[2] != 3:
        raise MarkerDetectionError("image must be BGR8")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.asarray(config.lower_hsv, dtype=np.uint8),
        np.asarray(config.upper_hsv, dtype=np.uint8),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        np.ones((3, 3), dtype=np.uint8),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        np.ones((5, 5), dtype=np.uint8),
    )

    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    candidates = [
        index
        for index in range(1, count)
        if config.min_area_px
        <= int(stats[index, cv2.CC_STAT_AREA])
        <= config.max_area_px
    ]
    if len(candidates) != 1:
        raise MarkerDetectionError(
            f"expected exactly 1 yellow marker, detected {len(candidates)}"
        )

    index = candidates[0]
    x = int(stats[index, cv2.CC_STAT_LEFT])
    y = int(stats[index, cv2.CC_STAT_TOP])
    width = int(stats[index, cv2.CC_STAT_WIDTH])
    height = int(stats[index, cv2.CC_STAT_HEIGHT])
    area = int(stats[index, cv2.CC_STAT_AREA])
    image_height, image_width = image.shape[:2]
    margin = config.border_margin_px
    if (
        x <= margin
        or y <= margin
        or x + width >= image_width - margin
        or y + height >= image_height - margin
    ):
        raise MarkerDetectionError("yellow marker touches the image border")

    center = centroids[index]
    return {
        "status": "TOP_TCP_MARKER_PASS",
        "center_px": [float(center[0]), float(center[1])],
        "bbox_xywh_px": [x, y, width, height],
        "area_px": area,
        "image_size_px": [image_width, image_height],
        "motion_authorized": False,
        "robot_target_available": False,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Detect one yellow TCP marker in a Top-camera image."
    )
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    image = cv2.imread(str(args.image.resolve()), cv2.IMREAD_COLOR)
    if image is None:
        print(f"TOP_TCP_MARKER_FAIL failed to read image: {args.image}")
        return 2
    try:
        result = detect_yellow_marker(image)
    except Exception as error:
        print(f"TOP_TCP_MARKER_FAIL {error}")
        return 2

    serialized = json.dumps(result, indent=2, sort_keys=True)
    if args.output is not None:
        output = args.output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(serialized + "\n", encoding="utf-8")
    print(serialized)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
