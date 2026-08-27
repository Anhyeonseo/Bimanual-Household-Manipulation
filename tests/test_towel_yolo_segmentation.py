from __future__ import annotations

from hashlib import sha256
import json
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from tools.lib.towel_dataset import build_dataset_manifest
from tools.lib.towel_yolo_segmentation import (
    TowelYoloExportError,
    annotation_to_yolo_line,
    export_reviewed_yolo_dataset,
    parse_yolo_segmentation_line,
    polygon_roundtrip_iou,
)


def _annotation(
    observation_id: str,
    split: str,
    image_relative: Path,
    image_path: Path,
    *,
    empty: bool = False,
) -> dict:
    return {
        "schema_version": 1,
        "record_kind": "towel_observation_annotation",
        "observation_id": observation_id,
        "split": split,
        "state_label": "EMPTY" if empty else "AMBIGUOUS",
        "image_width_px": 64,
        "image_height_px": 48,
        "segmentation_polygon_px": (
            []
            if empty
            else [[8.0, 6.0], [56.0, 6.0], [56.0, 42.0], [8.0, 42.0]]
        ),
        "corners": [],
        "fold_lines_px": [],
        "height_available": False,
        "occluded": False,
        "ambiguous_reason": None if empty else "human_reviewed_segmentation_only",
        "source": {
            "image_path": image_relative.as_posix(),
            "sha256": sha256(image_path.read_bytes()).hexdigest(),
            "capture_id": f"capture-{observation_id}",
        },
    }


def _review_root(
    root: Path,
    source_root: Path,
    *,
    name: str,
    session: str,
    split: str,
    entries: tuple[tuple[str, bool], ...],
    authorized: bool = True,
) -> Path:
    session_root = source_root / session
    review_root = root / name
    annotation_root = review_root / "annotations"
    annotation_root.mkdir(parents=True)
    records = []
    annotations = []
    for index, (observation_id, empty) in enumerate(entries):
        image_relative = Path("frames") / f"frame_{index}.jpg"
        image_path = session_root / image_relative
        image_path.parent.mkdir(parents=True, exist_ok=True)
        assert cv2.imwrite(
            str(image_path),
            np.full(
                (48, 64, 3),
                32 + (sum(observation_id.encode("utf-8")) + index) % 192,
                dtype=np.uint8,
            ),
        )
        annotation = _annotation(
            observation_id,
            split,
            image_relative,
            image_path,
            empty=empty,
        )
        annotation_relative = Path("annotations") / f"{observation_id}.json"
        (review_root / annotation_relative).write_text(
            json.dumps(annotation), encoding="utf-8"
        )
        annotations.append(annotation)
        records.append(
            {
                "image": image_relative.as_posix(),
                "annotation": annotation_relative.as_posix(),
                "source_sha256": annotation["source"]["sha256"],
            }
        )
    dataset_manifest = build_dataset_manifest(annotations)
    review_manifest = {
        "schema_version": 1,
        "record_kind": "towel_segmentation_pilot_review",
        "source_session": session,
        "accepted_annotation_count": len(records),
        "segmentation_labels_authorized": authorized,
        "robot_occluded_training_labels_authorized": False,
        "dataset_items_sha256": dataset_manifest["items_sha256"],
        "records": records,
    }
    (review_root / "review_manifest.json").write_text(
        json.dumps(review_manifest), encoding="utf-8"
    )
    return review_root


def test_yolo_row_normalizes_polygon_and_preserves_empty_negative(tmp_path):
    image = tmp_path / "frame.jpg"
    assert cv2.imwrite(str(image), np.full((48, 64, 3), 127, dtype=np.uint8))
    annotation = _annotation("towel", "train", Path("frame.jpg"), image)
    row = annotation_to_yolo_line(annotation)
    polygon = parse_yolo_segmentation_line(row, width=64, height=48)
    assert np.allclose(polygon, annotation["segmentation_polygon_px"])
    assert polygon_roundtrip_iou(annotation, row) == pytest.approx(1.0)

    empty = _annotation("empty", "train", Path("frame.jpg"), image, empty=True)
    assert annotation_to_yolo_line(empty) == ""
    assert polygon_roundtrip_iou(empty, "") == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("row", "message"),
    (
        ("1 0.1 0.1 0.2 0.1 0.2 0.2", "class"),
        ("0 0.1 0.1 0.2 0.1", "three"),
        ("0 0.1 0.1 1.2 0.1 0.2 0.2", "within"),
    ),
)
def test_yolo_row_parser_fails_closed(row, message):
    with pytest.raises(TowelYoloExportError, match=message):
        parse_yolo_segmentation_line(row, width=64, height=48)


def test_export_preserves_reviewed_splits_empty_labels_and_digest(tmp_path):
    source_root = tmp_path / "sources"
    train = _review_root(
        tmp_path,
        source_root,
        name="train-reviewed",
        session="train-session",
        split="train",
        entries=(("train-towel", False), ("train-empty", True)),
    )
    validation = _review_root(
        tmp_path,
        source_root,
        name="validation-reviewed",
        session="validation-session",
        split="validation",
        entries=(("validation-towel", False),),
    )
    output = tmp_path / "output"
    manifest = export_reviewed_yolo_dataset(
        (train, validation),
        source_root=source_root,
        output_root=output,
    )

    assert manifest["item_count"] == 3
    assert manifest["split_counts"] == {
        "train": 2,
        "validation": 1,
        "test": 0,
    }
    assert manifest["empty_negative_counts"]["train"] == 1
    empty_item = next(item for item in manifest["items"] if item["empty_negative"])
    assert (output / empty_item["label"]).read_text(encoding="utf-8") == ""
    assert all(item["roundtrip_iou"] == pytest.approx(1.0) for item in manifest["items"])
    dataset_yaml = yaml.safe_load((output / "dataset.yaml").read_text(encoding="utf-8"))
    assert dataset_yaml == {
        "train": "images/train",
        "val": "images/val",
        "names": {0: "towel"},
    }
    assert manifest["review_sources"][0]["manifest"] == "review_manifest.json"
    assert not Path(manifest["review_sources"][0]["review_root"]).is_absolute()


def test_export_rejects_unreviewed_or_split_leaked_sources(tmp_path):
    source_root = tmp_path / "sources"
    unauthorized = _review_root(
        tmp_path,
        source_root,
        name="unauthorized",
        session="unauthorized-session",
        split="train",
        entries=(("unreviewed", False),),
        authorized=False,
    )
    with pytest.raises(TowelYoloExportError, match="not authorized"):
        export_reviewed_yolo_dataset(
            (unauthorized,),
            source_root=source_root,
            output_root=tmp_path / "unauthorized-output",
        )

    train = _review_root(
        tmp_path,
        source_root,
        name="leaked-train",
        session="leaked-train-session",
        split="train",
        entries=(("leaked-train", False),),
    )
    validation = _review_root(
        tmp_path,
        source_root,
        name="leaked-validation",
        session="leaked-validation-session",
        split="validation",
        entries=(("leaked-validation", False),),
    )
    train_annotation_path = next((train / "annotations").glob("*.json"))
    validation_annotation_path = next((validation / "annotations").glob("*.json"))
    train_annotation = json.loads(train_annotation_path.read_text(encoding="utf-8"))
    validation_annotation = json.loads(
        validation_annotation_path.read_text(encoding="utf-8")
    )
    train_image = source_root / "leaked-train-session" / train_annotation["source"][
        "image_path"
    ]
    validation_image = (
        source_root
        / "leaked-validation-session"
        / validation_annotation["source"]["image_path"]
    )
    validation_image.write_bytes(train_image.read_bytes())
    validation_annotation["source"]["sha256"] = train_annotation["source"]["sha256"]
    validation_annotation_path.write_text(
        json.dumps(validation_annotation), encoding="utf-8"
    )
    validation_manifest_path = validation / "review_manifest.json"
    validation_manifest = json.loads(validation_manifest_path.read_text(encoding="utf-8"))
    validation_manifest["records"][0]["source_sha256"] = train_annotation["source"][
        "sha256"
    ]
    validation_manifest["dataset_items_sha256"] = build_dataset_manifest(
        [validation_annotation]
    )["items_sha256"]
    validation_manifest_path.write_text(
        json.dumps(validation_manifest), encoding="utf-8"
    )
    with pytest.raises(TowelYoloExportError, match="multiple splits"):
        export_reviewed_yolo_dataset(
            (train, validation),
            source_root=source_root,
            output_root=tmp_path / "leaked-output",
        )


def test_export_rejects_nonempty_output_directory(tmp_path):
    source_root = tmp_path / "sources"
    review = _review_root(
        tmp_path,
        source_root,
        name="reviewed",
        session="session",
        split="train",
        entries=(("item", False),),
    )
    output = tmp_path / "output"
    output.mkdir()
    (output / "stale.txt").write_text("stale", encoding="utf-8")
    with pytest.raises(TowelYoloExportError, match="not empty"):
        export_reviewed_yolo_dataset(
            (review,), source_root=source_root, output_root=output
        )
