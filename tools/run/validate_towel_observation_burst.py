#!/usr/bin/env python3
"""Validate a real three-frame Top-camera towel observation session."""

from __future__ import annotations

import argparse
from collections import defaultdict
from hashlib import sha256
import json
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.lib.towel_perception import blue_towel_image_observation  # noqa: E402
from tools.lib.towel_task_runtime import (
    PerceptionLimits,
    TowelObservation,
    estimate_towel_state,
    load_towel_contract,
)  # noqa: E402


DEFAULT_SESSION = (
    ROOT
    / "datasets/towel_yolo_source/20260827_top_lifecycle_validation_01"
)
DEFAULT_CONTRACT = ROOT / "config/towel_task_contract.candidate.yaml"
DEFAULT_CAMERA_INFO = (
    ROOT
    / "ros2_ws/src/manipulation_camera_manager/config/top_camera_info.yaml"
)
DEFAULT_WORKTABLE = (
    ROOT
    / "ros2_ws/src/manipulation_camera_manager/config/top_worktable_homography.yaml"
)
REQUIRED_CATEGORIES = {
    "00_empty_table",
    "01_flat",
    "03_heavy_wrinkle",
    "05_first_fold",
    "06_second_fold",
}


def sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def matrix(document: dict[str, Any], key: str) -> np.ndarray:
    value = document[key]
    return np.asarray(value["data"], dtype=float).reshape(
        int(value["rows"]), int(value["cols"])
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", nargs="?", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--camera-info", type=Path, default=DEFAULT_CAMERA_INFO)
    parser.add_argument("--worktable", type=Path, default=DEFAULT_WORKTABLE)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    session_root = args.session.resolve()
    session = json.loads((session_root / "session.json").read_text())
    if session.get("capture_protocol") != (
        "independent_physical_reposition_episode_burst_v1"
    ):
        raise ValueError("session is not an independent episode burst")
    frames_per_episode = session.get("frames_per_episode")
    if frames_per_episode != 3:
        raise ValueError("R1 validation requires exactly three frames per episode")

    records = [
        json.loads(line)
        for line in (session_root / "capture_manifest.jsonl")
        .read_text()
        .splitlines()
        if line
    ]
    if len({item["capture_id"] for item in records}) != len(records):
        raise ValueError("capture ids must be unique")
    by_episode: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for item in records:
        image_path = session_root / item["image_path"]
        if sha256_file(image_path) != item["sha256"]:
            raise ValueError(f"source SHA mismatch: {image_path}")
        if item.get("physical_reposition_confirmed") is not True:
            raise ValueError("held-out episode lacks physical reposition evidence")
        by_episode[item["episode_id"]].append(item)

    categories = {items[0]["category"] for items in by_episode.values()}
    missing = REQUIRED_CATEGORIES - categories
    if missing:
        raise ValueError(f"required R1 burst categories are missing: {sorted(missing)}")

    contract = load_towel_contract(args.contract)
    limits = PerceptionLimits.from_contract(contract)
    lifecycle_limits = contract["observation_lifecycle"]
    maximum_area_span = float(
        lifecycle_limits["maximum_visible_area_ratio_span"]
    )
    maximum_flatness_span = float(
        lifecycle_limits["maximum_flatness_score_span"]
    )
    maximum_topology_span = float(
        lifecycle_limits["maximum_topology_confidence_span"]
    )
    maximum_outline_span = float(
        lifecycle_limits["maximum_fold_outline_iou_span"]
    )
    camera = yaml.safe_load(args.camera_info.read_text())
    worktable = yaml.safe_load(args.worktable.read_text())
    camera_matrix = matrix(camera, "camera_matrix")
    distortion = matrix(camera, "distortion_coefficients").reshape(-1)
    projection = matrix(camera, "projection_matrix")
    homography = matrix(
        worktable["homography"], "rectified_pixel_to_board_m"
    )
    measured_sides = contract["towel"]["measured_sides_mm"]
    towel_size = (
        float(measured_sides["top"]) / 1000.0,
        float(measured_sides["right"]) / 1000.0,
    )
    expected_area = towel_size[0] * towel_size[1]

    summaries: list[dict[str, Any]] = []
    for episode_id, items in sorted(by_episode.items()):
        items.sort(key=lambda value: value["frame_index"])
        if [item["frame_index"] for item in items] != [1, 2, 3]:
            raise ValueError(f"episode frame sequence is incomplete: {episode_id}")
        if len({item["category"] for item in items}) != 1:
            raise ValueError(f"episode mixes categories: {episode_id}")
        category = items[0]["category"]
        states: list[str] = []
        presence: list[bool] = []
        clear_view: list[bool] = []
        areas: list[float] = []
        flatness: list[float] = []
        topology: list[float] = []
        outlines: list[float] = []
        for item in items:
            image_path = session_root / item["image_path"]
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None or image.shape[:2] != (960, 1280):
                raise ValueError(f"invalid Top image: {image_path}")
            fold_context: dict[str, Any] = {}
            if category == "05_first_fold":
                fold_context = {
                    "expected_fold_count": 1,
                    "fold_action_context_verified": True,
                    "unfolded_towel_size_m": towel_size,
                }
            elif category == "06_second_fold":
                fold_context = {
                    "expected_fold_count": 2,
                    "fold_action_context_verified": True,
                    "unfolded_towel_size_m": towel_size,
                }
            document = blue_towel_image_observation(
                image,
                observation_id=item["capture_id"],
                source_sha256=item["sha256"],
                calibration_sha256=sha256_file(args.camera_info),
                camera_matrix=camera_matrix,
                distortion_coefficients=distortion,
                projection_matrix=projection,
                rectified_pixel_to_workcell_homography=homography,
                expected_full_towel_area_m2=expected_area,
                **fold_context,
            )
            estimate = estimate_towel_state(
                TowelObservation.from_dict(document), limits
            )
            states.append(estimate.state.value)
            presence.append(bool(document["blue_evidence"]["towel_present"]))
            clear_view.append(bool(document["clear_view_valid"]))
            areas.append(float(document["visible_area_ratio"]))
            flatness.append(float(document["flatness_score"]))
            topology.append(float(document["topology_confidence"]))
            if document["outline_iou"] is not None:
                outlines.append(float(document["outline_iou"]))

        expected_empty = category == "00_empty_table"
        if expected_empty:
            if any(presence) or any(clear_view):
                raise ValueError(f"empty episode produced a towel: {episode_id}")
        elif not all(presence) or not all(clear_view):
            raise ValueError(f"towel episode failed presence/clear-view: {episode_id}")
        if len(set(states)) != 1:
            raise ValueError(f"state flicker detected: {episode_id}: {states}")
        if max(areas) - min(areas) > maximum_area_span:
            raise ValueError(f"visible-area span is unstable: {episode_id}")
        if max(flatness) - min(flatness) > maximum_flatness_span:
            raise ValueError(f"flatness span is unstable: {episode_id}")
        if max(topology) - min(topology) > maximum_topology_span:
            raise ValueError(f"topology span is unstable: {episode_id}")
        if outlines and max(outlines) - min(outlines) > maximum_outline_span:
            raise ValueError(f"fold outline span is unstable: {episode_id}")
        expected_state = {
            "05_first_fold": "FOLD_1_COMPLETE",
            "06_second_fold": "FOLD_2_COMPLETE",
        }.get(category)
        if expected_state is not None and states != [expected_state] * 3:
            raise ValueError(
                f"fold postcondition failed: {episode_id}: {states}"
            )
        summaries.append(
            {
                "episode_id": episode_id,
                "category": category,
                "state": states[0],
                "presence": presence,
                "clear_view_valid": clear_view,
                "visible_area_ratio_span": max(areas) - min(areas),
                "flatness_score_span": max(flatness) - min(flatness),
                "topology_confidence_span": max(topology) - min(topology),
                "fold_outline_iou_min": min(outlines) if outlines else None,
                "fold_outline_iou_span": (
                    max(outlines) - min(outlines) if outlines else None
                ),
            }
        )

    print(json.dumps({
        "schema_version": 1,
        "record_kind": "towel_observation_burst_validation",
        "status": "PASS",
        "session_id": session["session_id"],
        "frames": len(records),
        "episodes": summaries,
        "identities": {
            "contract_sha256": sha256_file(args.contract),
            "camera_info_sha256": sha256_file(args.camera_info),
            "worktable_sha256": sha256_file(args.worktable),
        },
    }, indent=2, sort_keys=True))
    print(
        "TOWEL_OBSERVATION_BURST_VALIDATION_PASS "
        f"episodes={len(summaries)} frames={len(records)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
