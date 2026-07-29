#!/usr/bin/env python3
"""Generate the raster source for the Top eye-to-hand GridBoard target."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import yaml


DICTIONARY_NAME = "DICT_4X4_50"
MARKERS_X = 2
MARKERS_Y = 2
MARKER_LENGTH_M = 0.020
MARKER_SEPARATION_M = 0.005
FIRST_MARKER_ID = 0
BOARD_SIDE_M = (
    MARKERS_X * MARKER_LENGTH_M
    + (MARKERS_X - 1) * MARKER_SEPARATION_M
)


def create_board():
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, DICTIONARY_NAME)
    )
    return cv2.aruco.GridBoard_create(
        MARKERS_X,
        MARKERS_Y,
        MARKER_LENGTH_M,
        MARKER_SEPARATION_M,
        dictionary,
        FIRST_MARKER_ID,
    )


def draw_board(pixel_size: int):
    if pixel_size < 900:
        raise ValueError("pixel_size must be at least 900")
    return create_board().draw(
        (pixel_size, pixel_size),
        marginSize=0,
        borderBits=1,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-png", required=True, type=Path)
    parser.add_argument("--output-yaml", required=True, type=Path)
    parser.add_argument("--pixel-size", type=int, default=1800)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    image = draw_board(args.pixel_size)
    args.output_png.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output_png), image):
        raise RuntimeError(f"failed to write {args.output_png}")
    board = create_board()
    document = {
        "schema_version": 1,
        "dictionary": DICTIONARY_NAME,
        "markers_x": MARKERS_X,
        "markers_y": MARKERS_Y,
        "marker_length_m": MARKER_LENGTH_M,
        "marker_separation_m": MARKER_SEPARATION_M,
        "first_marker_id": FIRST_MARKER_ID,
        "marker_ids": [
            int(value) for value in board.ids.reshape(-1)
        ],
        "board_side_m": BOARD_SIDE_M,
        "raster_pixel_size": args.pixel_size,
        "print_contract": {
            "scale": "ACTUAL_SIZE_100_PERCENT",
            "board_side_mm": BOARD_SIDE_M * 1000.0,
            "marker_side_mm": MARKER_LENGTH_M * 1000.0,
        },
    }
    args.output_yaml.parent.mkdir(parents=True, exist_ok=True)
    args.output_yaml.write_text(
        yaml.safe_dump(document, sort_keys=False),
        encoding="utf-8",
    )
    print(
        "TOP_EYE_TO_HAND_GRIDBOARD_PASS "
        f"ids={document['marker_ids']} "
        f"board_side_mm={BOARD_SIDE_M * 1000.0:.3f} "
        f"png={args.output_png} yaml={args.output_yaml}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
