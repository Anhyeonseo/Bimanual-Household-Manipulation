#!/usr/bin/env python3
"""Review and manually label one top-camera can OBB per capture.

The source capture set can contain several near-identical frames from one
physical scene.  This tool deliberately selects only the sharpest frame per
capture so that adjacent frames cannot later leak across train and validation
splits.  Positive cases receive one Ultralytics OBB; ``absent`` cases receive
an empty label file.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


CLASS_ID = 0
CLASS_NAME = "can"
INDEX_NAME = "can_obb_labeling_index.json"


@dataclass(frozen=True)
class Case:
    capture_id: str
    expected_present: bool
    image: Path
    image_sha256: str
    width: int
    height: int
    source_record: str
    condition: dict[str, str]


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def parse_cases(root: Path, *, verify_sha256: bool) -> list[Case]:
    cases: list[Case] = []
    ids: set[str] = set()
    for record_path in sorted(root.glob("*.json")):
        document = load_json(record_path)
        capture_id = document.get("capture_id")
        object_document = document.get("object")
        frames = document.get("frames")
        if not isinstance(capture_id, str) or not isinstance(object_document, dict):
            continue
        if object_document.get("class_name") != CLASS_NAME:
            continue
        state = object_document.get("state")
        if state not in {"lying", "absent"}:
            raise ValueError(f"unsupported can state for {capture_id}: {state!r}")
        if not isinstance(frames, list) or not frames:
            raise ValueError(f"capture has no frames: {capture_id}")
        if capture_id in ids:
            raise ValueError(f"duplicate capture_id: {capture_id}")
        ids.add(capture_id)
        candidates: list[dict] = []
        for frame in frames:
            if not isinstance(frame, dict):
                raise ValueError(f"invalid frame in {capture_id}")
            filename = frame.get("file")
            digest = frame.get("sha256")
            width = frame.get("width")
            height = frame.get("height")
            sharpness = frame.get("sharpness", 0.0)
            if not isinstance(filename, str) or not filename.endswith(".png"):
                raise ValueError(f"invalid frame filename in {capture_id}")
            if not isinstance(digest, str) or len(digest) != 64:
                raise ValueError(f"invalid frame SHA-256 in {capture_id}")
            if not isinstance(width, int) or not isinstance(height, int):
                raise ValueError(f"invalid frame dimensions in {capture_id}")
            image = (root / filename).resolve()
            if image.parent != root or not image.is_file():
                raise ValueError(f"capture frame is missing: {image}")
            if verify_sha256 and file_sha256(image) != digest:
                raise ValueError(f"capture frame SHA-256 mismatch: {image.name}")
            candidates.append(
                {
                    "image": image,
                    "sha256": digest,
                    "width": width,
                    "height": height,
                    "sharpness": float(sharpness),
                }
            )
        selected = max(candidates, key=lambda item: (item["sharpness"], item["image"].name))
        conditions = document.get("conditions")
        if not isinstance(conditions, dict):
            raise ValueError(f"conditions are missing: {capture_id}")
        condition = {}
        for key in ("background", "lighting", "glare"):
            value = conditions.get(key)
            if not isinstance(value, str) or not value:
                raise ValueError(f"condition.{key} is missing: {capture_id}")
            condition[key] = value
        cases.append(
            Case(
                capture_id=capture_id,
                expected_present=state == "lying",
                image=selected["image"],
                image_sha256=selected["sha256"],
                width=selected["width"],
                height=selected["height"],
                source_record=record_path.name,
                condition=condition,
            )
        )
    if not cases:
        raise ValueError("no can capture records were found")
    return cases


def order_box_points(points: np.ndarray) -> np.ndarray:
    if points.shape != (4, 2):
        raise ValueError("an OBB needs exactly four points")
    center = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    ordered = points[np.argsort(angles)]
    start = int(np.argmin(ordered[:, 0] + ordered[:, 1]))
    return np.roll(ordered, -start, axis=0)


def label_text(points: np.ndarray, width: int, height: int) -> str:
    ordered = order_box_points(points).astype(np.float64)
    ordered[:, 0] /= float(width)
    ordered[:, 1] /= float(height)
    if np.any(ordered < 0.0) or np.any(ordered > 1.0):
        raise ValueError("OBB point is outside image bounds")
    return f"{CLASS_ID} " + " ".join(f"{value:.8f}" for value in ordered.reshape(-1)) + "\n"


def parse_label(path: Path, width: int, height: int) -> tuple[bool, np.ndarray]:
    if not path.is_file() or not path.read_text(encoding="utf-8").strip():
        return False, np.empty((0, 2), dtype=np.float32)
    tokens = path.read_text(encoding="utf-8").strip().split()
    if len(tokens) != 9 or tokens[0] != str(CLASS_ID):
        raise ValueError(f"invalid can OBB label: {path}")
    values = np.asarray([float(token) for token in tokens[1:]], dtype=np.float32).reshape(4, 2)
    if np.any(values < 0.0) or np.any(values > 1.0):
        raise ValueError(f"normalized OBB outside [0, 1]: {path}")
    values[:, 0] *= width
    values[:, 1] *= height
    return True, values


def index_document(root: Path, cases: list[Case], labels_dir: Path) -> dict:
    items = []
    for case in cases:
        label = labels_dir / f"{case.capture_id}.txt"
        if not label.is_file():
            continue
        present, _ = parse_label(label, case.width, case.height)
        items.append(
            {
                "capture_id": case.capture_id,
                "expected_present": case.expected_present,
                "label_present": present,
                "image": case.image.name,
                "image_sha256": case.image_sha256,
                "label": str(label.relative_to(root)),
                "label_sha256": file_sha256(label),
                "source_record": case.source_record,
                "condition": case.condition,
            }
        )
    return {
        "schema_version": 1,
        "dataset_id": "dual_arm_can_disposal_v1",
        "class_names": [CLASS_NAME],
        "frame_selection": "sharpest_frame_per_capture",
        "selection_count": len(cases),
        "labeled_count": len(items),
        "items": items,
    }


class Labeler:
    def __init__(self, root: Path, cases: list[Case], labels_dir: Path, index_path: Path, scale: float, review_existing: bool):
        self.root = root
        self.cases = cases
        self.labels_dir = labels_dir
        self.index_path = index_path
        self.scale = scale
        self.points: list[tuple[float, float]] = []
        self.negative = False
        self.index = self._first_index(review_existing)
        self.window = "Top Can OBB Labeler"

    def _first_index(self, review_existing: bool) -> int:
        if review_existing:
            return 0
        for index, case in enumerate(self.cases):
            if not (self.labels_dir / f"{case.capture_id}.txt").is_file():
                return index
        return 0

    @property
    def case(self) -> Case:
        return self.cases[self.index]

    @property
    def label_path(self) -> Path:
        return self.labels_dir / f"{self.case.capture_id}.txt"

    def load_current(self) -> None:
        self.points = []
        self.negative = False
        if self.label_path.is_file():
            self.negative, existing = parse_label(self.label_path, self.case.width, self.case.height)
            self.points = [tuple(point) for point in existing]
            self.negative = not self.negative

    def _mouse(self, event: int, x: int, y: int, _flags: int, _data: object) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or self.negative or len(self.points) >= 4:
            return
        point = (x / self.scale, y / self.scale)
        self.points.append(point)

    def _render(self) -> np.ndarray:
        image = cv2.imread(str(self.case.image), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"failed to decode {self.case.image}")
        canvas = image.copy()
        if len(self.points) == 4:
            box = np.round(order_box_points(np.asarray(self.points, dtype=np.float32))).astype(np.int32)
            cv2.polylines(canvas, [box], True, (0, 255, 0), 2, cv2.LINE_AA)
        for number, point in enumerate(self.points, start=1):
            coordinate = (int(round(point[0])), int(round(point[1])))
            cv2.circle(canvas, coordinate, 5, (0, 200, 255), -1, cv2.LINE_AA)
            cv2.putText(canvas, str(number), (coordinate[0] + 6, coordinate[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)
        expected = "CAN" if self.case.expected_present else "ABSENT"
        current = "NEGATIVE" if self.negative else f"{len(self.points)}/4 corners"
        title = f"{self.index + 1}/{len(self.cases)}  {self.case.capture_id}  expected={expected}  current={current}"
        help_text = "click 4 corners | Enter save | n negative | u undo | r reset | b previous | s skip | q quit"
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 48), (0, 0, 0), -1)
        cv2.putText(canvas, title, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, help_text, (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 255, 180), 1, cv2.LINE_AA)
        return cv2.resize(canvas, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)

    def _save_index(self) -> None:
        self.index_path.write_text(json.dumps(index_document(self.root, self.cases, self.labels_dir), indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def save_current(self) -> None:
        self.labels_dir.mkdir(parents=True, exist_ok=True)
        if self.negative:
            self.label_path.write_text("", encoding="utf-8")
        else:
            if len(self.points) != 4:
                raise ValueError("click exactly four corners before saving a positive case")
            self.label_path.write_text(label_text(np.asarray(self.points, dtype=np.float32), self.case.width, self.case.height), encoding="utf-8")
        self._save_index()

    def move(self, delta: int) -> None:
        self.index = (self.index + delta) % len(self.cases)
        self.load_current()

    def run(self) -> int:
        self.labels_dir.mkdir(parents=True, exist_ok=True)
        self.load_current()
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window, self._mouse)
        while True:
            cv2.imshow(self.window, self._render())
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (13, 10, ord(" ")):
                try:
                    self.save_current()
                    print(f"TOP_CAN_OBB_LABEL_SAVED capture_id={self.case.capture_id} present={not self.negative}")
                    self.move(1)
                except ValueError as error:
                    print(f"TOP_CAN_OBB_LABEL_REJECTED reason={error}", file=sys.stderr)
            elif key == ord("n"):
                self.negative = True
                self.points = []
            elif key == ord("u"):
                if self.points:
                    self.points.pop()
            elif key == ord("r"):
                self.points = []
                self.negative = False
            elif key == ord("b"):
                self.move(-1)
            elif key == ord("s"):
                self.move(1)
        cv2.destroyAllWindows()
        self._save_index()
        document = index_document(self.root, self.cases, self.labels_dir)
        print(f"TOP_CAN_OBB_LABEL_SESSION_STOPPED labeled={document['labeled_count']}/{document['selection_count']} index={self.index_path}")
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=Path("datasets/top_can_obb_source"))
    parser.add_argument("--labels-dir", type=Path, default=None)
    parser.add_argument("--index", type=Path, default=None)
    parser.add_argument("--display-scale", type=float, default=1.25)
    parser.add_argument("--review-existing", action="store_true")
    parser.add_argument("--skip-sha256-verify", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        root = args.source_root.resolve()
        if not root.is_dir():
            raise ValueError(f"source root does not exist: {root}")
        if args.display_scale <= 0.0 or args.display_scale > 2.0:
            raise ValueError("display scale must be in (0, 2]")
        labels_dir = (args.labels_dir or root / "labels").resolve()
        index_path = (args.index or root / INDEX_NAME).resolve()
        for path in (labels_dir, index_path):
            try:
                path.relative_to(root)
            except ValueError as error:
                raise ValueError("labels and index must stay under source root") from error
        cases = parse_cases(root, verify_sha256=not args.skip_sha256_verify)
        positives = sum(case.expected_present for case in cases)
        print(f"TOP_CAN_OBB_LABELER_READY captures={len(cases)} positives={positives} negatives={len(cases) - positives} selection=sharpest_frame_per_capture")
        return Labeler(root, cases, labels_dir, index_path, args.display_scale, args.review_existing).run()
    except Exception as error:
        print(f"TOP_CAN_OBB_LABELER_ERROR reason={error}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
