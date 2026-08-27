from __future__ import annotations

from pathlib import Path
from hashlib import sha256
import json

import cv2
import numpy as np
import pytest

from tools.run.bootstrap_towel_segmentation_pilot import (
    REVIEW_CONFIRMATION,
    SegmentationBootstrapError,
    deterministic_subset,
    finalize_labelme_review,
    labelme_document,
    load_capture_session_index,
    mask_polygon,
    propose_towel_mask,
    reviewed_source_paths,
    reviewed_annotation_from_labelme,
)


def test_capture_session_index_preserves_heldout_split_and_episode(tmp_path):
    session = tmp_path / "validation-session"
    (session / "01_flat").mkdir(parents=True)
    metadata = {
        "record_kind": "towel_capture_session",
        "session_id": session.name,
        "split": "validation",
    }
    (session / "session.json").write_text(
        json.dumps(metadata), encoding="utf-8"
    )
    record = {
        "record_kind": "towel_capture_episode",
        "session_id": session.name,
        "split": "validation",
        "image_path": "01_flat/flat_0001.jpg",
        "capture_id": "validation-flat-0001",
        "physical_reposition_confirmed": True,
    }
    (session / "capture_manifest.jsonl").write_text(
        json.dumps(record) + "\n", encoding="utf-8"
    )
    split, index = load_capture_session_index(session)
    assert split == "validation"
    assert index == {"01_flat/flat_0001.jpg": "validation-flat-0001"}


def test_capture_session_index_fails_closed_on_unconfirmed_heldout(tmp_path):
    session = tmp_path / "validation-session"
    session.mkdir()
    (session / "session.json").write_text(
        json.dumps(
            {
                "record_kind": "towel_capture_session",
                "session_id": session.name,
                "split": "validation",
            }
        ),
        encoding="utf-8",
    )
    (session / "capture_manifest.jsonl").write_text(
        json.dumps(
            {
                "record_kind": "towel_capture_episode",
                "session_id": session.name,
                "split": "validation",
                "image_path": "frame.jpg",
                "capture_id": "capture-1",
                "physical_reposition_confirmed": False,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    with pytest.raises(SegmentationBootstrapError, match="reposition"):
        load_capture_session_index(session)


def test_deterministic_subset_spans_the_source_sequence():
    paths = [Path(f"frame_{index:03d}.jpg") for index in range(21)]
    assert deterministic_subset(paths, 5) == [
        paths[0], paths[5], paths[10], paths[15], paths[20]
    ]
    with pytest.raises(SegmentationBootstrapError, match="only"):
        deterministic_subset(paths, 22)


def test_blue_towel_proposal_rejects_white_background():
    image = np.full((480, 640, 3), (205, 200, 205), dtype=np.uint8)
    cv2.rectangle(image, (150, 100), (500, 390), (190, 150, 125), -1)
    mask = propose_towel_mask(image)
    expected = np.zeros(mask.shape, dtype=np.uint8)
    cv2.rectangle(expected, (150, 100), (500, 390), 255, -1)
    intersection = np.count_nonzero((mask > 0) & (expected > 0))
    union = np.count_nonzero((mask > 0) | (expected > 0))
    assert intersection / union >= 0.95
    polygon = mask_polygon(mask)
    assert len(polygon) >= 4


def test_polygon_rejects_an_empty_mask():
    with pytest.raises(SegmentationBootstrapError, match="no contour"):
        mask_polygon(np.zeros((100, 100), dtype=np.uint8))


def proposal_for(image_path: Path, *, empty: bool = False):
    return {
        "schema_version": 1,
        "record_kind": "towel_observation_annotation",
        "observation_id": "pilot-review",
        "split": "train",
        "state_label": "EMPTY" if empty else "AMBIGUOUS",
        "image_width_px": 64,
        "image_height_px": 48,
        "segmentation_polygon_px": (
            [] if empty else [[10.0, 10.0], [50.0, 10.0], [50.0, 38.0]]
        ),
        "corners": [],
        "fold_lines_px": [],
        "height_available": False,
        "occluded": False,
        "ambiguous_reason": (
            None if empty else "bootstrap_segmentation_requires_human_review"
        ),
        "source": {
            "image_path": image_path.name,
            "sha256": sha256(image_path.read_bytes()).hexdigest(),
            "capture_id": None,
        },
    }


def test_labelme_review_round_trip_keeps_segmentation_only_authority(tmp_path):
    image_path = tmp_path / "frame.jpg"
    assert cv2.imwrite(
        str(image_path), np.full((48, 64, 3), 127, dtype=np.uint8)
    )
    proposal = proposal_for(image_path)
    labelme = labelme_document(
        image_name=image_path.name,
        image_width=64,
        image_height=48,
        polygon=[[8.0, 9.0], [52.0, 9.0], [52.0, 40.0], [8.0, 40.0]],
    )
    reviewed = reviewed_annotation_from_labelme(
        proposal, labelme, dataset_root=tmp_path
    )
    assert reviewed["segmentation_polygon_px"] == labelme["shapes"][0]["points"]
    assert reviewed["state_label"] == "AMBIGUOUS"
    assert reviewed["ambiguous_reason"] == "human_reviewed_segmentation_only"


def test_labelme_review_rejects_extra_shapes_and_nonempty_empty_frame(tmp_path):
    image_path = tmp_path / "frame.jpg"
    assert cv2.imwrite(
        str(image_path), np.full((48, 64, 3), 127, dtype=np.uint8)
    )
    labelme = labelme_document(
        image_name=image_path.name,
        image_width=64,
        image_height=48,
        polygon=[[8.0, 9.0], [52.0, 9.0], [52.0, 40.0]],
    )
    labelme["shapes"].append(dict(labelme["shapes"][0]))
    with pytest.raises(SegmentationBootstrapError, match="exactly one"):
        reviewed_annotation_from_labelme(
            proposal_for(image_path), labelme, dataset_root=tmp_path
        )


def test_reviewed_source_paths_ignores_manifests(tmp_path):
    image_path = tmp_path / "frame.jpg"
    assert cv2.imwrite(
        str(image_path), np.full((48, 64, 3), 127, dtype=np.uint8)
    )
    annotation_root = tmp_path / "annotations"
    annotation_root.mkdir()
    (annotation_root / "frame.json").write_text(
        __import__("json").dumps(proposal_for(image_path)), encoding="utf-8"
    )
    (annotation_root / "manifest.json").write_text(
        '{"record_kind":"towel_dataset_manifest"}', encoding="utf-8"
    )
    assert reviewed_source_paths(annotation_root) == {"frame.jpg"}


def test_finalize_labelme_review_records_explicit_rejection(tmp_path):
    session_root = tmp_path / "session"
    session_root.mkdir()
    image_path = session_root / "frame.jpg"
    assert cv2.imwrite(
        str(image_path), np.full((48, 64, 3), 127, dtype=np.uint8)
    )
    pilot_root = tmp_path / "pilot"
    pilot_root.mkdir()
    proposal = proposal_for(image_path)
    (pilot_root / "proposal.json").write_text(
        __import__("json").dumps(proposal), encoding="utf-8"
    )
    labelme = labelme_document(
        image_name=image_path.name,
        image_width=64,
        image_height=48,
        polygon=proposal["segmentation_polygon_px"],
    )
    labelme["flags"]["review_rejected"] = True
    (pilot_root / "labelme.json").write_text(
        __import__("json").dumps(labelme), encoding="utf-8"
    )
    manifest = {
        "source_session": session_root.name,
        "annotation_proposal_count": 1,
        "records": [
            {
                "category": "01_flat",
                "image": image_path.name,
                "annotation": "proposal.json",
                "labelme_annotation": "labelme.json",
            }
        ],
    }
    (pilot_root / "pilot_manifest.json").write_text(
        __import__("json").dumps(manifest), encoding="utf-8"
    )
    with pytest.raises(SegmentationBootstrapError, match="accepted no"):
        finalize_labelme_review(
            session_root=session_root,
            pilot_root=pilot_root,
            output_root=tmp_path / "reviewed",
            confirmation=REVIEW_CONFIRMATION,
        )
    with pytest.raises(SegmentationBootstrapError, match="EMPTY"):
        reviewed_annotation_from_labelme(
            proposal_for(image_path, empty=True),
            labelme_document(
                image_name=image_path.name,
                image_width=64,
                image_height=48,
                polygon=[[8.0, 9.0], [52.0, 9.0], [52.0, 40.0]],
            ),
            dataset_root=tmp_path,
        )
