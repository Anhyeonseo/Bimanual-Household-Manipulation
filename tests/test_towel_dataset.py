from __future__ import annotations

from copy import deepcopy
from hashlib import sha256

import pytest

from tools.lib.towel_dataset import (
    TowelDatasetError,
    build_dataset_manifest,
    validate_annotation,
)


def annotation(observation_id="item-1", split="train", digest="a" * 64):
    return {
        "schema_version": 1,
        "record_kind": "towel_observation_annotation",
        "observation_id": observation_id,
        "split": split,
        "state_label": "ALIGNED",
        "image_width_px": 640,
        "image_height_px": 480,
        "segmentation_polygon_px": [
            [100, 100], [500, 100], [500, 400], [100, 400]
        ],
        "corners": [
            {
                "label": "top_left",
                "point_px": [100, 100],
                "visible": True,
                "graspable": True,
                "confidence": 0.95,
            }
        ],
        "fold_lines_px": [],
        "height_available": False,
        "occluded": False,
        "ambiguous_reason": None,
        "source": {"image_path": f"images/{observation_id}.png", "sha256": digest},
    }


def test_annotation_validates_bounds_labels_and_digest():
    result = validate_annotation(annotation())
    assert result["observation_id"] == "item-1"
    assert result["corner_count"] == 1


@pytest.mark.parametrize(
    ("change", "message"),
    (
        (lambda value: value.update(image_width_px=0), "dimensions"),
        (
            lambda value: value["segmentation_polygon_px"].__setitem__(0, [-1, 10]),
            "outside",
        ),
        (
            lambda value: value["corners"].append(deepcopy(value["corners"][0])),
            "unique",
        ),
        (
            lambda value: value["source"].update(image_path="../escape.png"),
            "relative path",
        ),
    ),
)
def test_invalid_annotations_fail_closed(change, message):
    value = annotation()
    change(value)
    with pytest.raises(TowelDatasetError, match=message):
        validate_annotation(value)


def test_ambiguous_annotation_requires_a_reason():
    value = annotation()
    value["state_label"] = "AMBIGUOUS"
    with pytest.raises(TowelDatasetError, match="ambiguous_reason"):
        validate_annotation(value)


def test_annotation_rejects_implicit_booleans_and_degenerate_polygons():
    value = annotation()
    value["corners"][0]["visible"] = "yes"
    with pytest.raises(TowelDatasetError, match="boolean"):
        validate_annotation(value)
    value = annotation()
    value["segmentation_polygon_px"] = [[1, 1], [2, 2], [3, 3]]
    with pytest.raises(TowelDatasetError, match="area"):
        validate_annotation(value)


def test_annotation_rejects_unknown_fields_and_numeric_strings():
    value = annotation()
    value["motion_authorized"] = True
    with pytest.raises(TowelDatasetError, match="unknown"):
        validate_annotation(value)
    value = annotation()
    value["corners"][0]["confidence"] = "0.95"
    with pytest.raises(TowelDatasetError, match="numeric"):
        validate_annotation(value)
    value = annotation()
    value["observation_id"] = 7
    with pytest.raises(TowelDatasetError, match="observation_id"):
        validate_annotation(value)


@pytest.mark.parametrize(
    ("state", "line_count"),
    (("FOLD_1_COMPLETE", 0), ("FOLD_2_COMPLETE", 1)),
)
def test_fold_complete_annotations_require_matching_fold_lines(state, line_count):
    value = annotation()
    value["state_label"] = state
    value["fold_lines_px"] = [[[100, 100], [500, 100]]] * line_count
    with pytest.raises(TowelDatasetError, match="requires"):
        validate_annotation(value)


def test_manifest_is_order_independent_and_split_safe():
    first = annotation("a", "train", "a" * 64)
    second = annotation("b", "test", "b" * 64)
    forward = build_dataset_manifest([first, second])
    reverse = build_dataset_manifest([second, first])
    assert forward["items_sha256"] == reverse["items_sha256"]
    assert forward["split_counts"] == {
        "train": 1, "validation": 0, "test": 1
    }


def test_manifest_rejects_id_duplicates_and_source_leakage():
    with pytest.raises(TowelDatasetError, match="observation_id"):
        build_dataset_manifest([annotation(), annotation()])
    with pytest.raises(TowelDatasetError, match="multiple splits"):
        build_dataset_manifest([
            annotation("a", "train", "c" * 64),
            annotation("b", "test", "c" * 64),
        ])


def test_dataset_root_checks_file_existence_and_sha(tmp_path):
    images = tmp_path / "images"
    images.mkdir()
    source = images / "item-1.png"
    source.write_bytes(b"synthetic image bytes")
    digest = sha256(source.read_bytes()).hexdigest()
    validate_annotation(annotation(digest=digest), dataset_root=tmp_path)
    with pytest.raises(TowelDatasetError, match="SHA mismatch"):
        validate_annotation(annotation(digest="f" * 64), dataset_root=tmp_path)
