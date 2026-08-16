from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from tools.label_top_can_obb import label_text, parse_cases, parse_label


def _write_case(root: Path, capture_id: str, state: str, frames: list[tuple[str, float]]) -> None:
    frame_items = []
    for filename, sharpness in frames:
        image = np.full((12, 16, 3), 180, dtype=np.uint8)
        path = root / filename
        assert cv2.imwrite(str(path), image)
        frame_items.append(
            {
                "file": filename,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "width": 16,
                "height": 12,
                "sharpness": sharpness,
            }
        )
    (root / f"{capture_id}.json").write_text(
        json.dumps(
            {
                "capture_id": capture_id,
                "object": {"class_name": "can", "state": state},
                "frames": frame_items,
                "conditions": {"background": "board", "lighting": "on", "glare": "low"},
            }
        ),
        encoding="utf-8",
    )


def test_parse_cases_selects_one_sharpest_frame_per_capture(tmp_path: Path) -> None:
    _write_case(tmp_path, "can_001", "lying", [("can_001_a.png", 1.0), ("can_001_b.png", 2.0)])
    _write_case(tmp_path, "empty_001", "absent", [("empty_001.png", 3.0)])

    cases = parse_cases(tmp_path, verify_sha256=True)

    assert [case.capture_id for case in cases] == ["can_001", "empty_001"]
    assert cases[0].image.name == "can_001_b.png"
    assert cases[0].expected_present is True
    assert cases[1].expected_present is False


def test_label_round_trip_orders_and_normalizes_points(tmp_path: Path) -> None:
    label = tmp_path / "can.txt"
    points = np.asarray([[15, 10], [2, 1], [2, 10], [15, 1]], dtype=np.float32)
    label.write_text(label_text(points, width=16, height=12), encoding="utf-8")

    present, parsed = parse_label(label, width=16, height=12)

    assert present is True
    assert parsed.shape == (4, 2)
    assert np.all(parsed >= 0)
    assert np.all(parsed[:, 0] <= 16)
    assert np.all(parsed[:, 1] <= 12)
