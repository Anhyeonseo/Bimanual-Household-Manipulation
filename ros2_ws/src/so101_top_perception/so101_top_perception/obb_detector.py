"""Fail-closed OpenCV-DNN runtime for a single-class Ultralytics OBB model."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .detector import (
    Calibration,
    DetectionError,
    file_sha256,
    normalize_axis_yaw,
    transform_to_board,
)


BACKEND_NAME = "opencv_dnn_ultralytics_obb"
OUTPUT_LAYOUT = "ultralytics_obb_raw_v1"
YAW_SEMANTICS = "undirected_long_axis_modulo_pi"


@dataclass(frozen=True)
class LetterboxTransform:
    """Map model-input coordinates back to the source image."""

    scale: float
    pad_x: int
    pad_y: int
    input_width: int
    input_height: int
    source_width: int
    source_height: int


@dataclass(frozen=True)
class ObbRuntimeConfig:
    """Validated inference settings stored beside the ONNX model."""

    model_path: Path
    model_sha256: str
    input_width: int
    input_height: int
    letterbox_value: int
    confidence_threshold: float
    iou_threshold: float
    maximum_detections: int
    class_names: tuple[str, ...]
    pen_class_id: int
    holdout_manifest_sha256: str


def _load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return document


def _resolve_bundle_file(bundle_manifest: Path, value: object) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError("model.path must be a non-empty relative path")
    relative = Path(value)
    if relative.is_absolute():
        raise ValueError("model.path must be relative to the bundle manifest")
    root = bundle_manifest.parent.resolve()
    resolved = (root / relative).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ValueError("model.path must stay inside the bundle directory") from error
    return resolved


def load_runtime_config(
    bundle_manifest_path: Path,
    expected_holdout_manifest_sha256: str | None = None,
) -> ObbRuntimeConfig:
    """Validate an immutable ONNX deployment bundle manifest."""
    bundle_manifest_path = bundle_manifest_path.resolve()
    document = _load_json(bundle_manifest_path)
    if document.get("protocol_version") != 1:
        raise ValueError("bundle protocol_version must be 1")
    if document.get("backend") != BACKEND_NAME:
        raise ValueError(f"bundle backend must be {BACKEND_NAME}")
    if document.get("task") != "obb":
        raise ValueError("bundle task must be obb")

    model = document.get("model")
    input_contract = document.get("input")
    output = document.get("output")
    thresholds = document.get("thresholds")
    training = document.get("training")
    if not all(
        isinstance(value, dict)
        for value in (model, input_contract, output, thresholds, training)
    ):
        raise ValueError("bundle model/input/output/thresholds/training are required")

    model_path = _resolve_bundle_file(bundle_manifest_path, model.get("path"))
    if not model_path.is_file():
        raise ValueError(f"ONNX model is missing: {model_path}")
    expected_model_hash = model.get("sha256")
    actual_model_hash = file_sha256(model_path)
    if expected_model_hash != actual_model_hash:
        raise ValueError("ONNX model SHA-256 does not match the bundle manifest")
    if model.get("format") != "onnx":
        raise ValueError("model.format must be onnx")

    width = int(input_contract.get("width", 0))
    height = int(input_contract.get("height", 0))
    if width <= 0 or height <= 0 or width % 32 or height % 32:
        raise ValueError("model input dimensions must be positive multiples of 32")
    if input_contract.get("layout") != "NCHW":
        raise ValueError("model input layout must be NCHW")
    if input_contract.get("color_order") != "RGB":
        raise ValueError("model input color_order must be RGB")
    if not math.isclose(
        float(input_contract.get("scale", 0.0)),
        1.0 / 255.0,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ValueError("model input scale must be 1/255")
    letterbox_value = int(input_contract.get("letterbox_value", -1))
    if not 0 <= letterbox_value <= 255:
        raise ValueError("letterbox_value must be within 0..255")

    class_names = output.get("class_names")
    if (
        not isinstance(class_names, list)
        or not class_names
        or not all(isinstance(name, str) and name for name in class_names)
    ):
        raise ValueError("output.class_names must be a non-empty string list")
    if output.get("layout") != OUTPUT_LAYOUT:
        raise ValueError(f"output.layout must be {OUTPUT_LAYOUT}")
    if output.get("yaw_semantics") != YAW_SEMANTICS:
        raise ValueError(f"output.yaw_semantics must be {YAW_SEMANTICS}")
    pen_class_id = int(output.get("pen_class_id", -1))
    if not 0 <= pen_class_id < len(class_names):
        raise ValueError("output.pen_class_id is outside class_names")
    if class_names[pen_class_id] != "pen":
        raise ValueError("pen_class_id must select the pen class")

    confidence = float(thresholds.get("confidence", 0.0))
    iou = float(thresholds.get("iou", 0.0))
    maximum = int(thresholds.get("maximum_detections", 0))
    if not 0.0 < confidence < 1.0:
        raise ValueError("confidence threshold must be within (0, 1)")
    if not 0.0 < iou < 1.0:
        raise ValueError("iou threshold must be within (0, 1)")
    if maximum <= 0:
        raise ValueError("maximum_detections must be positive")

    if training.get("holdout_used_for_training") is not False:
        raise ValueError("bundle must attest holdout_used_for_training=false")
    holdout_hash = training.get("holdout_manifest_sha256")
    if not isinstance(holdout_hash, str) or len(holdout_hash) != 64:
        raise ValueError("training.holdout_manifest_sha256 is required")
    if (
        expected_holdout_manifest_sha256 is not None
        and holdout_hash != expected_holdout_manifest_sha256
    ):
        raise ValueError("bundle was not attested against this holdout manifest")

    return ObbRuntimeConfig(
        model_path=model_path,
        model_sha256=actual_model_hash,
        input_width=width,
        input_height=height,
        letterbox_value=letterbox_value,
        confidence_threshold=confidence,
        iou_threshold=iou,
        maximum_detections=maximum,
        class_names=tuple(class_names),
        pen_class_id=pen_class_id,
        holdout_manifest_sha256=holdout_hash,
    )


def letterbox(
    image: np.ndarray,
    input_width: int,
    input_height: int,
    value: int = 114,
) -> tuple[np.ndarray, LetterboxTransform]:
    """Resize with unchanged aspect ratio and deterministic centered padding."""
    if image.ndim != 3 or image.shape[2] != 3:
        raise DetectionError("INVALID_IMAGE", "image must be BGR8")
    source_height, source_width = image.shape[:2]
    if source_width <= 0 or source_height <= 0:
        raise DetectionError("INVALID_IMAGE", "image dimensions must be positive")
    if input_width <= 0 or input_height <= 0:
        raise ValueError("letterbox input dimensions must be positive")

    scale = min(input_width / source_width, input_height / source_height)
    resized_width = max(1, int(round(source_width * scale)))
    resized_height = max(1, int(round(source_height * scale)))
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    )
    pad_x = (input_width - resized_width) // 2
    pad_y = (input_height - resized_height) // 2
    canvas = np.full(
        (input_height, input_width, 3),
        int(value),
        dtype=np.uint8,
    )
    canvas[
        pad_y:pad_y + resized_height,
        pad_x:pad_x + resized_width,
    ] = resized
    return canvas, LetterboxTransform(
        scale=scale,
        pad_x=pad_x,
        pad_y=pad_y,
        input_width=input_width,
        input_height=input_height,
        source_width=source_width,
        source_height=source_height,
    )


def _prediction_rows(output: np.ndarray, class_count: int) -> np.ndarray:
    values = np.asarray(output, dtype=np.float32)
    if values.ndim == 3:
        if values.shape[0] != 1:
            raise DetectionError(
                "MODEL_OUTPUT_INVALID",
                "ONNX output batch dimension must be 1",
            )
        values = values[0]
    if values.ndim != 2:
        raise DetectionError(
            "MODEL_OUTPUT_INVALID",
            "ONNX output must have two prediction dimensions",
        )
    feature_count = 5 + class_count
    if values.shape[0] == feature_count:
        values = values.T
    elif values.shape[1] != feature_count:
        raise DetectionError(
            "MODEL_OUTPUT_INVALID",
            f"ONNX output needs {feature_count} features per prediction",
        )
    if not np.all(np.isfinite(values)):
        raise DetectionError(
            "MODEL_OUTPUT_INVALID",
            "ONNX output contains non-finite values",
        )
    return values


def decode_ultralytics_obb(
    output: np.ndarray,
    transform: LetterboxTransform,
    class_count: int,
    pen_class_id: int,
    confidence_threshold: float,
    iou_threshold: float,
    maximum_detections: int,
) -> list[dict]:
    """Decode non-end-to-end Ultralytics xywhr+class ONNX output."""
    rows = _prediction_rows(output, class_count)
    boxes = []
    scores = []
    decoded = []
    for row in rows:
        class_scores = row[4:4 + class_count]
        class_id = int(np.argmax(class_scores))
        confidence = float(class_scores[class_id])
        if class_id != pen_class_id or confidence < confidence_threshold:
            continue
        center_x, center_y, width, height = (
            float(row[0]),
            float(row[1]),
            float(row[2]),
            float(row[3]),
        )
        angle_rad = float(row[4 + class_count])
        if width <= 0.0 or height <= 0.0:
            continue
        center_x = (center_x - transform.pad_x) / transform.scale
        center_y = (center_y - transform.pad_y) / transform.scale
        width /= transform.scale
        height /= transform.scale
        rotated = (
            (center_x, center_y),
            (width, height),
            math.degrees(angle_rad),
        )
        boxes.append(rotated)
        scores.append(confidence)
        decoded.append(
            {
                "class_id": class_id,
                "confidence": confidence,
                "raw_center_px": [center_x, center_y],
                "raw_size_px": [width, height],
                "raw_angle_rad": angle_rad,
                "raw_corners_px": cv2.boxPoints(rotated).astype(
                    np.float64
                ),
            }
        )

    if not boxes:
        return []
    indices = cv2.dnn.NMSBoxesRotated(
        boxes,
        scores,
        confidence_threshold,
        iou_threshold,
        1.0,
        maximum_detections,
    )
    if len(indices) == 0:
        return []
    selected = np.asarray(indices).reshape(-1).tolist()
    return [decoded[index] for index in selected[:maximum_detections]]


def _pose_from_detection(detection: dict, calibration: Calibration) -> dict:
    raw_box = np.asarray(detection["raw_corners_px"], dtype=np.float64)
    raw_center = np.mean(raw_box, axis=0)
    board_center = transform_to_board(raw_center, calibration)[0]
    board_box = transform_to_board(raw_box, calibration)
    edges = np.roll(board_box, -1, axis=0) - board_box
    lengths = np.linalg.norm(edges, axis=1)
    longest = edges[int(np.argmax(lengths))]
    yaw = normalize_axis_yaw(
        math.atan2(float(longest[1]), float(longest[0]))
    )
    return {
        "raw_center_px": [float(raw_center[0]), float(raw_center[1])],
        "raw_corners_px": [
            [float(point[0]), float(point[1])] for point in raw_box
        ],
        "board_position_m": [
            float(board_center[0]),
            float(board_center[1]),
        ],
        "size_m": [float(np.max(lengths)), float(np.min(lengths))],
        "board_corners_m": [
            [float(point[0]), float(point[1])] for point in board_box
        ],
        "yaw_rad": float(yaw),
        "yaw_deg": float(math.degrees(yaw)),
        "yaw_semantics": YAW_SEMANTICS,
        "confidence": float(detection["confidence"]),
        "class_id": int(detection["class_id"]),
    }


def select_one_pose(
    detections: list[dict],
    calibration: Calibration,
    image_edge_margin_px: int,
    require_full_footprint: bool,
) -> dict:
    """Apply the existing calibrated-region and visibility safety contract."""
    poses = [_pose_from_detection(item, calibration) for item in detections]
    relevant = []
    for pose in poses:
        corners = np.asarray(pose["board_corners_m"], dtype=np.float64)
        intersects = bool(
            np.all(np.max(corners, axis=0) >= 0.0)
            and np.all(np.min(corners, axis=0) <= calibration.board_span)
        )
        if intersects:
            relevant.append(pose)
    if len(relevant) != 1:
        raise DetectionError(
            "OBJECT_COUNT_INVALID",
            "expected exactly 1 OBB intersecting the calibrated region, "
            f"detected {len(relevant)} "
            f"(ignored {len(poses) - len(relevant)} fully outside)",
        )

    pose = relevant[0]
    raw = np.asarray(pose["raw_corners_px"], dtype=np.float64)
    margin = float(image_edge_margin_px)
    fully_visible = bool(
        np.all(raw[:, 0] >= margin)
        and np.all(raw[:, 1] >= margin)
        and np.all(raw[:, 0] <= calibration.image_width - 1 - margin)
        and np.all(raw[:, 1] <= calibration.image_height - 1 - margin)
    )
    if not fully_visible:
        raise DetectionError(
            "IMAGE_FOOTPRINT_CLIPPED",
            "OBB reaches the camera image safety margin",
        )
    center = np.asarray(pose["board_position_m"], dtype=np.float64)
    center_inside = bool(
        np.all(center >= 0.0) and np.all(center <= calibration.board_span)
    )
    if not center_inside:
        raise DetectionError(
            "CENTER_OUTSIDE_CALIBRATED_REGION",
            "OBB center is outside the calibrated board region",
        )
    corners = np.asarray(pose["board_corners_m"], dtype=np.float64)
    footprint_inside = bool(
        np.all(corners >= 0.0) and np.all(corners <= calibration.board_span)
    )
    if require_full_footprint and not footprint_inside:
        raise DetectionError(
            "OUTSIDE_CALIBRATED_REGION",
            "OBB footprint is outside the calibrated board region",
        )
    pose["calibration_region"] = {
        "span_m": calibration.board_span.astype(float).tolist(),
        "center_inside": True,
        "footprint_inside": footprint_inside,
        "image_fully_visible": True,
        "extrapolated": not footprint_inside,
        "ignored_fully_outside_count": len(poses) - len(relevant),
    }
    return pose


class OpenCvYoloObbDetector:
    """Immutable ONNX model runner with deterministic preprocessing."""

    def __init__(
        self,
        bundle_manifest_path: Path,
        expected_holdout_manifest_sha256: str | None = None,
    ):
        self.config = load_runtime_config(
            bundle_manifest_path,
            expected_holdout_manifest_sha256,
        )
        try:
            self._network = cv2.dnn.readNetFromONNX(
                str(self.config.model_path)
            )
        except cv2.error as error:
            raise ValueError(f"failed to load ONNX model: {error}") from error
        self._network.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self._network.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def infer_detections(self, image: np.ndarray) -> list[dict]:
        """Run one image through the fixed-shape ONNX output contract."""
        prepared, transform = letterbox(
            image,
            self.config.input_width,
            self.config.input_height,
            self.config.letterbox_value,
        )
        blob = cv2.dnn.blobFromImage(
            prepared,
            scalefactor=1.0 / 255.0,
            size=(self.config.input_width, self.config.input_height),
            mean=(0.0, 0.0, 0.0),
            swapRB=True,
            crop=False,
        )
        self._network.setInput(blob)
        try:
            output = self._network.forward()
        except cv2.error as error:
            raise DetectionError(
                "MODEL_INFERENCE_FAILED",
                f"OpenCV DNN inference failed: {error}",
            ) from error
        return decode_ultralytics_obb(
            output,
            transform,
            len(self.config.class_names),
            self.config.pen_class_id,
            self.config.confidence_threshold,
            self.config.iou_threshold,
            self.config.maximum_detections,
        )

    def detect(
        self,
        image: np.ndarray,
        calibration: Calibration,
        image_edge_margin_px: int,
        require_full_footprint: bool,
    ) -> dict:
        if (
            image.shape[1] != calibration.image_width
            or image.shape[0] != calibration.image_height
        ):
            raise DetectionError(
                "RESOLUTION_MISMATCH",
                "image and camera-info resolution mismatch",
            )
        detections = self.infer_detections(image)
        return select_one_pose(
            detections,
            calibration,
            image_edge_margin_px,
            require_full_footprint,
        )
