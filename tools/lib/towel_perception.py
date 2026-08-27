"""Reviewed-polygon and fixed blue-towel image perception backends."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence

import cv2
import numpy as np

from tools.lib.towel_dataset import validate_annotation
from tools.lib.towel_geometry import polygon_area, quadrilateral_metrics
from tools.lib.towel_task_runtime import TowelTaskContractError


@dataclass(frozen=True, slots=True)
class MaskDiagnostics:
    image_width_px: int
    image_height_px: int
    foreground_area_ratio: float
    component_count: int
    largest_component_area_ratio: float
    secondary_component_area_ratio: float
    touches_frame_border: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "image_width_px": self.image_width_px,
            "image_height_px": self.image_height_px,
            "foreground_area_ratio": self.foreground_area_ratio,
            "component_count": self.component_count,
            "largest_component_area_ratio": self.largest_component_area_ratio,
            "secondary_component_area_ratio": self.secondary_component_area_ratio,
            "touches_frame_border": self.touches_frame_border,
        }


@dataclass(frozen=True, slots=True)
class BlueTowelEvidence:
    total_area_ratio: float
    largest_component_area_ratio: float
    touches_frame_border: bool
    minimum_frame_margin_px: int | None
    towel_present: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "total_area_ratio": self.total_area_ratio,
            "largest_component_area_ratio": self.largest_component_area_ratio,
            "touches_frame_border": self.touches_frame_border,
            "minimum_frame_margin_px": self.minimum_frame_margin_px,
            "towel_present": self.towel_present,
        }


@dataclass(frozen=True, slots=True)
class TowelMaskCandidate:
    mask: np.ndarray
    mask_diagnostics: MaskDiagnostics
    blue_evidence: BlueTowelEvidence
    conservative_border_contact: bool
    clear_view_valid: bool
    rejection_reason: str | None


@dataclass(frozen=True, slots=True)
class MaskShapeFeatures:
    rectangularity: float
    solidity: float
    rectangle_iou: float
    simplified_vertex_count: int
    outline_quadrilateral_px: tuple[tuple[float, float], ...]
    topology_confidence: float
    flatness_score: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "rectangularity": self.rectangularity,
            "solidity": self.solidity,
            "rectangle_iou": self.rectangle_iou,
            "simplified_vertex_count": self.simplified_vertex_count,
            "outline_quadrilateral_px": [
                list(point) for point in self.outline_quadrilateral_px
            ],
            "topology_confidence": self.topology_confidence,
            "flatness_score": self.flatness_score,
        }


@dataclass(frozen=True, slots=True)
class FoldOutlineMatch:
    expected_fold_count: int
    normalized_metric_outline_iou: float
    observed_long_side_m: float
    observed_short_side_m: float
    target_long_side_m: float
    target_short_side_m: float
    ignored_accessory_width_m: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "expected_fold_count": self.expected_fold_count,
            "normalized_metric_outline_iou": (
                self.normalized_metric_outline_iou
            ),
            "observed_long_side_m": self.observed_long_side_m,
            "observed_short_side_m": self.observed_short_side_m,
            "target_long_side_m": self.target_long_side_m,
            "target_short_side_m": self.target_short_side_m,
            "ignored_accessory_width_m": self.ignored_accessory_width_m,
        }


# Measured on the 103-image reviewed development set and the independent
# 35-image held-out set.  The largest blue component was <= 0.0254 for all
# 18 accepted empty frames and >= 0.0838 for all 120 towel frames.  This
# midpoint remains a candidate gate, not a claim about other towel colours.
BLUE_TOWEL_MINIMUM_COMPONENT_AREA_RATIO = 0.05
BLUE_HUE_RANGE = (85, 130)
BLUE_MINIMUM_SATURATION = 35
BLUE_MINIMUM_VALUE = 45
BLUE_TOWEL_CLEAR_VIEW_MARGIN_PX = 3


def _validate_bgr_image(image_bgr: np.ndarray) -> tuple[int, int]:
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise TowelTaskContractError("source image must be BGR color")
    height, width = image_bgr.shape[:2]
    if height < 64 or width < 64:
        raise TowelTaskContractError("source image is too small")
    return height, width


def _largest_component(mask: np.ndarray) -> np.ndarray:
    binary = (np.asarray(mask) > 0).astype(np.uint8)
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        binary, connectivity=8
    )
    result = np.zeros(binary.shape, dtype=np.uint8)
    if component_count <= 1:
        return result
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    result[labels == largest] = 255
    return result


def _fill_internal_holes(mask: np.ndarray) -> np.ndarray:
    height, width = mask.shape
    flooded = mask.copy()
    flood_mask = np.zeros((height + 2, width + 2), dtype=np.uint8)
    cv2.floodFill(flooded, flood_mask, (0, 0), 255)
    return cv2.bitwise_or(mask, cv2.bitwise_not(flooded))


def blue_towel_evidence_mask(image_bgr: np.ndarray) -> np.ndarray:
    """Return the largest fixed-colour component used only as evidence.

    This is intentionally separate from the final GrabCut mask.  It prevents
    GrabCut from inventing a foreground on an empty table and preserves weak
    border pixels as a conservative clear-view rejection signal.
    """
    _validate_bgr_image(image_bgr)
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array(
        [BLUE_HUE_RANGE[0], BLUE_MINIMUM_SATURATION, BLUE_MINIMUM_VALUE],
        dtype=np.uint8,
    )
    upper = np.array([BLUE_HUE_RANGE[1], 255, 255], dtype=np.uint8)
    return _largest_component(cv2.inRange(hsv, lower, upper))


def inspect_blue_towel_evidence(
    image_bgr: np.ndarray,
    *,
    minimum_component_area_ratio: float = BLUE_TOWEL_MINIMUM_COMPONENT_AREA_RATIO,
) -> BlueTowelEvidence:
    if (
        not math.isfinite(minimum_component_area_ratio)
        or not 0.0 < minimum_component_area_ratio < 1.0
    ):
        raise TowelTaskContractError(
            "minimum blue component area ratio must be within 0..1"
        )
    evidence = blue_towel_evidence_mask(image_bgr)
    diagnostics = inspect_binary_mask(evidence)
    foreground_y, foreground_x = np.where(evidence > 0)
    minimum_frame_margin_px = (
        None
        if foreground_x.size == 0
        else int(
            min(
                int(np.min(foreground_x)),
                diagnostics.image_width_px - 1 - int(np.max(foreground_x)),
                int(np.min(foreground_y)),
                diagnostics.image_height_px - 1 - int(np.max(foreground_y)),
            )
        )
    )
    return BlueTowelEvidence(
        total_area_ratio=diagnostics.foreground_area_ratio,
        largest_component_area_ratio=diagnostics.largest_component_area_ratio,
        touches_frame_border=diagnostics.touches_frame_border,
        minimum_frame_margin_px=minimum_frame_margin_px,
        towel_present=(
            diagnostics.largest_component_area_ratio
            >= minimum_component_area_ratio
        ),
    )


def propose_blue_towel_mask(image_bgr: np.ndarray) -> np.ndarray:
    """Return the conservative largest-component mask for the blue towel.

    An explicit blue-presence gate runs before GrabCut.  Therefore an empty
    table produces an empty mask instead of the old central-rectangle false
    foreground.
    """
    height, width = _validate_bgr_image(image_bgr)
    evidence = inspect_blue_towel_evidence(image_bgr)
    if not evidence.towel_present:
        return np.zeros((height, width), dtype=np.uint8)

    scale = min(1.0, 640.0 / width, 480.0 / height)
    proposal_width = max(64, round(width * scale))
    proposal_height = max(64, round(height * scale))
    proposal_image = cv2.resize(
        image_bgr,
        (proposal_width, proposal_height),
        interpolation=cv2.INTER_AREA,
    )
    margin_x = max(2, round(proposal_width * 0.0375))
    margin_y = max(2, round(proposal_height * 0.0375))
    grabcut_mask = np.zeros((proposal_height, proposal_width), dtype=np.uint8)
    background_model = np.zeros((1, 65), dtype=np.float64)
    foreground_model = np.zeros((1, 65), dtype=np.float64)
    cv2.setRNGSeed(0)
    cv2.grabCut(
        proposal_image,
        grabcut_mask,
        (
            margin_x,
            margin_y,
            proposal_width - 2 * margin_x,
            proposal_height - 2 * margin_y,
        ),
        background_model,
        foreground_model,
        5,
        cv2.GC_INIT_WITH_RECT,
    )
    grabcut_foreground = (grabcut_mask == cv2.GC_FGD) | (
        grabcut_mask == cv2.GC_PR_FGD
    )

    blue_score = proposal_image[:, :, 0].astype(np.int16) - proposal_image[
        :, :, 2
    ].astype(np.int16)
    border_width = max(4, round(min(proposal_height, proposal_width) / 12))
    border_scores = np.concatenate(
        (
            blue_score[:border_width].ravel(),
            blue_score[-border_width:].ravel(),
            blue_score[border_width:-border_width, :border_width].ravel(),
            blue_score[border_width:-border_width, -border_width:].ravel(),
        )
    )
    relative_blue_threshold = float(np.median(border_scores)) + 6.0
    color_foreground = blue_score > relative_blue_threshold
    mask = (grabcut_foreground & color_foreground).astype(np.uint8) * 255
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8)
    )
    closing_size = max(5, round(min(proposal_height, proposal_width) / 32))
    if closing_size % 2 == 0:
        closing_size += 1
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        np.ones((closing_size, closing_size), dtype=np.uint8),
    )
    mask = _fill_internal_holes(_largest_component(mask))
    return cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)


def blue_towel_mask_candidate(image_bgr: np.ndarray) -> TowelMaskCandidate:
    """Build a fail-closed image candidate without authorizing topology."""
    evidence = inspect_blue_towel_evidence(image_bgr)
    mask = propose_blue_towel_mask(image_bgr)
    diagnostics = inspect_binary_mask(mask)
    conservative_border_contact = bool(
        evidence.minimum_frame_margin_px is not None
        and evidence.minimum_frame_margin_px <= BLUE_TOWEL_CLEAR_VIEW_MARGIN_PX
    )
    if not evidence.towel_present:
        rejection_reason = "towel_not_present"
    elif diagnostics.component_count != 1:
        rejection_reason = "segmentation_component_count_not_one"
    elif conservative_border_contact:
        rejection_reason = "towel_evidence_touches_frame_border"
    else:
        rejection_reason = None
    return TowelMaskCandidate(
        mask=mask,
        mask_diagnostics=diagnostics,
        blue_evidence=evidence,
        conservative_border_contact=conservative_border_contact,
        clear_view_valid=rejection_reason is None,
        rejection_reason=rejection_reason,
    )


def mask_shape_features(mask: np.ndarray) -> MaskShapeFeatures:
    """Describe visible outline geometry without inferring hidden layers."""
    diagnostics = inspect_binary_mask(mask)
    if diagnostics.component_count != 1:
        return MaskShapeFeatures(0.0, 0.0, 0.0, 0, (), 0.0, 0.0)
    binary = (np.asarray(mask) > 0).astype(np.uint8) * 255
    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    contour = max(contours, key=cv2.contourArea)
    contour_area = float(cv2.contourArea(contour))
    hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
    rectangle = cv2.minAreaRect(contour)
    rectangle_area = float(rectangle[1][0] * rectangle[1][1])
    rectangle_points = cv2.boxPoints(rectangle)
    rectangle_mask = np.zeros(binary.shape, dtype=np.uint8)
    cv2.fillConvexPoly(
        rectangle_mask, np.rint(rectangle_points).astype(np.int32), 255
    )
    intersection = float(
        np.count_nonzero((binary > 0) & (rectangle_mask > 0))
    )
    union = float(np.count_nonzero((binary > 0) | (rectangle_mask > 0)))
    perimeter = float(cv2.arcLength(contour, True))
    simplified = cv2.approxPolyDP(
        contour, max(1.0, 0.02 * perimeter), True
    ).reshape(-1, 2)
    rectangularity = contour_area / rectangle_area if rectangle_area > 0 else 0.0
    solidity = contour_area / hull_area if hull_area > 0 else 0.0
    rectangle_iou = intersection / union if union > 0 else 0.0
    quadrilateral = (
        tuple((float(x), float(y)) for x, y in simplified)
        if len(simplified) == 4
        else ()
    )
    # A non-quadrilateral silhouette must not cross the 0.8 topology gate,
    # even when its minimum-area rectangle happens to fit well.
    topology_confidence = (
        min(rectangle_iou, solidity) if quadrilateral else min(0.79, rectangle_iou)
    )
    return MaskShapeFeatures(
        rectangularity=rectangularity,
        solidity=solidity,
        rectangle_iou=rectangle_iou,
        simplified_vertex_count=len(simplified),
        outline_quadrilateral_px=quadrilateral,
        topology_confidence=topology_confidence,
        flatness_score=min(rectangle_iou, solidity),
    )


def inspect_binary_mask(mask: np.ndarray) -> MaskDiagnostics:
    """Measure components and border contact without approving the mask."""
    array = np.asarray(mask)
    if array.ndim != 2 or array.shape[0] <= 0 or array.shape[1] <= 0:
        raise TowelTaskContractError("binary mask must be a nonempty 2D array")
    if not np.issubdtype(array.dtype, np.number) and array.dtype != np.bool_:
        raise TowelTaskContractError("binary mask must contain numeric values")
    if np.issubdtype(array.dtype, np.floating) and not np.all(np.isfinite(array)):
        raise TowelTaskContractError("binary mask must contain finite values")
    binary = (array > 0).astype(np.uint8)
    height, width = binary.shape
    component_total, _, stats, _ = cv2.connectedComponentsWithStats(
        binary, connectivity=8
    )
    areas = sorted(
        (int(value) for value in stats[1:, cv2.CC_STAT_AREA]),
        reverse=True,
    )
    pixel_count = float(width * height)
    foreground = float(sum(areas))
    touches_border = bool(
        np.any(binary[0])
        or np.any(binary[-1])
        or np.any(binary[:, 0])
        or np.any(binary[:, -1])
    )
    return MaskDiagnostics(
        image_width_px=width,
        image_height_px=height,
        foreground_area_ratio=foreground / pixel_count,
        component_count=component_total - 1,
        largest_component_area_ratio=(areas[0] / pixel_count if areas else 0.0),
        secondary_component_area_ratio=(
            areas[1] / pixel_count if len(areas) > 1 else 0.0
        ),
        touches_frame_border=touches_border,
    )


def rasterize_annotation_mask(annotation: Mapping[str, Any]) -> np.ndarray:
    """Rasterize one validated segmentation polygon into a binary mask."""
    validate_annotation(annotation)
    height = int(annotation["image_height_px"])
    width = int(annotation["image_width_px"])
    mask = np.zeros((height, width), dtype=np.uint8)
    polygon = annotation["segmentation_polygon_px"]
    if not polygon:
        return mask
    points = np.rint(np.asarray(polygon, dtype=float)).astype(np.int32)
    points[:, 0] = np.clip(points[:, 0], 0, width - 1)
    points[:, 1] = np.clip(points[:, 1], 0, height - 1)
    cv2.fillPoly(mask, [points], 1)
    return mask


def _homography(value: Sequence[Sequence[float]]) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise TowelTaskContractError(
            "pixel_to_workcell_homography must be a finite 3x3 matrix"
        )
    if abs(float(np.linalg.det(matrix))) <= 1.0e-12:
        raise TowelTaskContractError(
            "pixel_to_workcell_homography must be invertible"
        )
    return matrix


def project_pixel_to_workcell(
    point_px: Sequence[float],
    pixel_to_workcell_homography: Sequence[Sequence[float]],
) -> tuple[float, float]:
    matrix = _homography(pixel_to_workcell_homography)
    homogeneous = matrix @ np.array(
        [float(point_px[0]), float(point_px[1]), 1.0], dtype=float
    )
    if abs(float(homogeneous[2])) <= 1.0e-12:
        raise TowelTaskContractError(
            "homography projects point to infinity"
        )
    point = (
        float(homogeneous[0] / homogeneous[2]),
        float(homogeneous[1] / homogeneous[2]),
    )
    if not all(math.isfinite(value) for value in point):
        raise TowelTaskContractError(
            "homography produced a non-finite workcell point"
        )
    return point


def project_raw_pixel_to_workcell(
    point_px: Sequence[float],
    *,
    camera_matrix: Sequence[Sequence[float]],
    distortion_coefficients: Sequence[float],
    projection_matrix: Sequence[Sequence[float]],
    rectified_pixel_to_workcell_homography: Sequence[Sequence[float]],
) -> tuple[float, float]:
    """Undistort one raw pixel before applying the plane homography."""
    projected = project_raw_pixels_to_workcell(
        [point_px],
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion_coefficients,
        projection_matrix=projection_matrix,
        rectified_pixel_to_workcell_homography=(
            rectified_pixel_to_workcell_homography
        ),
    )
    return float(projected[0, 0]), float(projected[0, 1])


def project_raw_pixels_to_workcell(
    points_px: Sequence[Sequence[float]],
    *,
    camera_matrix: Sequence[Sequence[float]],
    distortion_coefficients: Sequence[float],
    projection_matrix: Sequence[Sequence[float]],
    rectified_pixel_to_workcell_homography: Sequence[Sequence[float]],
) -> np.ndarray:
    """Vectorized raw-pixel rectification and table-plane projection."""
    intrinsic = np.asarray(camera_matrix, dtype=float)
    distortion = np.asarray(distortion_coefficients, dtype=float).reshape(-1)
    projection = np.asarray(projection_matrix, dtype=float)
    if intrinsic.shape != (3, 3) or not np.all(np.isfinite(intrinsic)):
        raise TowelTaskContractError("camera_matrix must be a finite 3x3 matrix")
    if distortion.size < 4 or not np.all(np.isfinite(distortion)):
        raise TowelTaskContractError(
            "distortion_coefficients must contain at least four finite values"
        )
    if projection.shape == (3, 4):
        projection = projection[:, :3]
    if projection.shape != (3, 3) or not np.all(np.isfinite(projection)):
        raise TowelTaskContractError(
            "projection_matrix must be a finite 3x3 or 3x4 matrix"
        )
    raw = np.asarray(points_px, dtype=float)
    if raw.ndim != 2 or raw.shape[0] <= 0 or raw.shape[1] != 2:
        raise TowelTaskContractError("raw pixels must be an Nx2 array")
    if not np.all(np.isfinite(raw)):
        raise TowelTaskContractError("raw pixels must be finite")
    rectified = cv2.undistortPoints(
        raw.reshape(-1, 1, 2),
        intrinsic,
        distortion,
        P=projection,
    )
    matrix = _homography(rectified_pixel_to_workcell_homography)
    return cv2.perspectiveTransform(
        rectified.astype(np.float64), matrix
    ).reshape(-1, 2)


def metric_fold_outline_match(
    metric_contour_xy_m: Sequence[Sequence[float]],
    *,
    expected_fold_count: int,
    unfolded_towel_size_m: Sequence[float],
    raster_pixels_per_m: float = 2000.0,
    maximum_ignored_accessory_width_m: float = 0.02,
) -> FoldOutlineMatch:
    """Compare a fold result with its nominal metric outline.

    Translation and in-plane rotation are normalized so this score measures
    fold shape and physical size, not where the operator placed the towel.
    The fold count is supplied by a verified action context; it is never
    inferred from RGB area.
    """
    if expected_fold_count not in (1, 2):
        raise TowelTaskContractError(
            "expected_fold_count must be 1 or 2"
        )
    size = np.asarray(unfolded_towel_size_m, dtype=float).reshape(-1)
    if (
        size.shape != (2,)
        or not np.all(np.isfinite(size))
        or np.any(size <= 0.0)
    ):
        raise TowelTaskContractError(
            "unfolded_towel_size_m must contain two positive finite values"
        )
    if not math.isfinite(raster_pixels_per_m) or raster_pixels_per_m <= 0.0:
        raise TowelTaskContractError(
            "raster_pixels_per_m must be finite and positive"
        )
    if (
        not math.isfinite(maximum_ignored_accessory_width_m)
        or maximum_ignored_accessory_width_m < 0.0
    ):
        raise TowelTaskContractError(
            "maximum ignored accessory width must be finite and nonnegative"
        )
    contour = np.asarray(metric_contour_xy_m, dtype=float)
    if (
        contour.ndim != 2
        or contour.shape[0] < 3
        or contour.shape[1] != 2
        or not np.all(np.isfinite(contour))
    ):
        raise TowelTaskContractError(
            "metric contour must be a finite Nx2 polygon"
        )

    # The measured towel has a narrow hanging loop.  It remains part of the
    # segmentation label, but the user identified it as a non-graspable
    # accessory rather than foldable body.  Remove only structures narrower
    # than the explicit metric allowance before scoring the folded body.
    contour_lower = np.min(contour, axis=0) - 0.01
    contour_upper = np.max(contour, axis=0) + 0.01
    contour_raster_size = np.ceil(
        (contour_upper - contour_lower) * raster_pixels_per_m
    ).astype(int) + 1
    contour_mask = np.zeros(
        (
            int(contour_raster_size[1]),
            int(contour_raster_size[0]),
        ),
        dtype=np.uint8,
    )
    contour_px = np.rint(
        (contour - contour_lower) * raster_pixels_per_m
    ).astype(np.int32)
    cv2.fillPoly(contour_mask, [contour_px], 255)
    if maximum_ignored_accessory_width_m > 0.0:
        kernel_size = max(
            1,
            round(maximum_ignored_accessory_width_m * raster_pixels_per_m),
        )
        if kernel_size % 2 == 0:
            kernel_size += 1
        contour_mask = cv2.morphologyEx(
            contour_mask,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
            ),
        )
        contour_mask = _largest_component(contour_mask)
    cleaned_contours, _ = cv2.findContours(
        contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not cleaned_contours:
        raise TowelTaskContractError(
            "accessory exclusion removed the complete metric contour"
        )
    cleaned_px = max(cleaned_contours, key=cv2.contourArea).reshape(-1, 2)
    contour = (
        cleaned_px.astype(float) / raster_pixels_per_m + contour_lower
    )

    rectangle = cv2.minAreaRect(contour.astype(np.float32))
    center = rectangle[0]
    observed_width, observed_height = rectangle[1]
    angle_deg = float(rectangle[2])
    if observed_width <= 0.0 or observed_height <= 0.0:
        raise TowelTaskContractError("metric contour has zero extent")
    if observed_width < observed_height:
        observed_width, observed_height = observed_height, observed_width
        angle_deg += 90.0

    unfolded_width, unfolded_height = (float(size[0]), float(size[1]))
    if expected_fold_count == 1:
        target_sides = (unfolded_width, unfolded_height / 2.0)
    else:
        target_sides = (unfolded_width / 2.0, unfolded_height / 2.0)
    target_long, target_short = sorted(target_sides, reverse=True)
    target = cv2.boxPoints(
        (center, (target_long, target_short), angle_deg)
    ).astype(np.float32)

    combined = np.vstack((contour, target))
    lower = np.min(combined, axis=0) - 0.01
    upper = np.max(combined, axis=0) + 0.01
    raster_size = np.ceil(
        (upper - lower) * raster_pixels_per_m
    ).astype(int) + 1
    observed_mask = np.zeros(
        (int(raster_size[1]), int(raster_size[0])), dtype=np.uint8
    )
    target_mask = np.zeros_like(observed_mask)
    observed_px = np.rint(
        (contour - lower) * raster_pixels_per_m
    ).astype(np.int32)
    target_px = np.rint(
        (target - lower) * raster_pixels_per_m
    ).astype(np.int32)
    cv2.fillPoly(observed_mask, [observed_px], 1)
    cv2.fillConvexPoly(target_mask, target_px, 1)
    intersection = int(
        np.count_nonzero((observed_mask > 0) & (target_mask > 0))
    )
    union = int(
        np.count_nonzero((observed_mask > 0) | (target_mask > 0))
    )
    score = float(intersection / union) if union else 0.0
    return FoldOutlineMatch(
        expected_fold_count=expected_fold_count,
        normalized_metric_outline_iou=score,
        observed_long_side_m=float(observed_width),
        observed_short_side_m=float(observed_height),
        target_long_side_m=target_long,
        target_short_side_m=target_short,
        ignored_accessory_width_m=maximum_ignored_accessory_width_m,
    )


def blue_towel_image_observation(
    image_bgr: np.ndarray,
    *,
    observation_id: str,
    source_sha256: str,
    calibration_sha256: str,
    camera_matrix: Sequence[Sequence[float]],
    distortion_coefficients: Sequence[float],
    projection_matrix: Sequence[Sequence[float]],
    rectified_pixel_to_workcell_homography: Sequence[Sequence[float]],
    expected_full_towel_area_m2: float,
    stale: bool = False,
    capture_stamp_ns: int | None = None,
    lifecycle_phase: str | None = None,
    model_sha256: str | None = None,
    robot_model_sha256: str | None = None,
    settled: bool | None = None,
    clear_pose_verified: bool | None = None,
    expected_fold_count: int | None = None,
    fold_action_context_verified: bool = False,
    unfolded_towel_size_m: Sequence[float] | None = None,
) -> dict[str, Any]:
    """Convert one Top image into a fail-closed mask/outline observation.

    The backend may approve four *visible outline* corners for a clear,
    quadrilateral silhouette. It never infers hidden cloth layers or a fold
    count from RGB area alone. A fold count is accepted only when the caller
    supplies an explicit verified fold-action context and the metric outline
    independently passes the corresponding target.
    """
    if not isinstance(observation_id, str) or not observation_id:
        raise TowelTaskContractError("observation_id is required")
    for label, digest in (
        ("source_sha256", source_sha256),
        ("calibration_sha256", calibration_sha256),
    ):
        if len(digest) != 64 or any(
            character not in "0123456789abcdef" for character in digest
        ):
            raise TowelTaskContractError(f"{label} must be lowercase SHA-256")
    if (
        not math.isfinite(expected_full_towel_area_m2)
        or expected_full_towel_area_m2 <= 0.0
    ):
        raise TowelTaskContractError(
            "expected_full_towel_area_m2 must be finite and positive"
        )
    matrix = _homography(rectified_pixel_to_workcell_homography)
    candidate = blue_towel_mask_candidate(image_bgr)
    shape = mask_shape_features(candidate.mask)
    contours, _ = cv2.findContours(
        candidate.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    visible_mask_area_m2 = 0.0
    metric_contour: np.ndarray | None = None
    if contours:
        raw_contour = max(contours, key=cv2.contourArea).reshape(-1, 2)
        metric_contour = project_raw_pixels_to_workcell(
            raw_contour,
            camera_matrix=camera_matrix,
            distortion_coefficients=distortion_coefficients,
            projection_matrix=projection_matrix,
            rectified_pixel_to_workcell_homography=matrix,
        )
        visible_mask_area_m2 = abs(
            float(cv2.contourArea(metric_contour.astype(np.float32)))
        )
    visible_area_ratio = min(
        1.0, visible_mask_area_m2 / expected_full_towel_area_m2
    )
    corners = [
        {
            "point_xy_m": list(
                project_raw_pixel_to_workcell(
                    point,
                    camera_matrix=camera_matrix,
                    distortion_coefficients=distortion_coefficients,
                    projection_matrix=projection_matrix,
                    rectified_pixel_to_workcell_homography=matrix,
                )
            ),
            "confidence": shape.topology_confidence,
            "visible": True,
            "graspable": candidate.clear_view_valid,
        }
        for point in shape.outline_quadrilateral_px
    ]
    fold_match: FoldOutlineMatch | None = None
    if expected_fold_count is not None:
        if fold_action_context_verified is not True:
            raise TowelTaskContractError(
                "fold count requires a verified fold action context"
            )
        if unfolded_towel_size_m is None:
            raise TowelTaskContractError(
                "fold postcondition requires unfolded_towel_size_m"
            )
        if metric_contour is not None and candidate.clear_view_valid:
            fold_match = metric_fold_outline_match(
                metric_contour,
                expected_fold_count=expected_fold_count,
                unfolded_towel_size_m=unfolded_towel_size_m,
            )
    elif fold_action_context_verified:
        raise TowelTaskContractError(
            "verified fold action context requires expected_fold_count"
        )

    outline_iou = (
        None
        if fold_match is None
        else fold_match.normalized_metric_outline_iou
    )
    fold_count = 0 if expected_fold_count is None else expected_fold_count
    topology_confidence = (
        shape.topology_confidence if candidate.clear_view_valid else 0.0
    )
    if outline_iou is not None:
        topology_confidence = max(topology_confidence, outline_iou)
    document: dict[str, Any] = {
        "schema_version": 1,
        "record_kind": "towel_state_observation",
        "observation_id": observation_id,
        "source_sha256": source_sha256,
        "calibration_sha256": calibration_sha256,
        "visible_area_ratio": visible_area_ratio,
        "topology_confidence": topology_confidence,
        "flatness_score": (
            shape.flatness_score if candidate.clear_view_valid else 0.0
        ),
        "fold_count": fold_count,
        "outline_iou": outline_iou,
        "stale": stale,
        "clear_view_valid": candidate.clear_view_valid,
        "corners": corners,
        "backend": "blue_towel_grabcut_outline_v1",
        "height_available": False,
        "visible_mask_area_m2": visible_mask_area_m2,
        "mask_diagnostics": candidate.mask_diagnostics.to_dict(),
        "blue_evidence": candidate.blue_evidence.to_dict(),
        "shape_features": shape.to_dict(),
        "rejection_reason": candidate.rejection_reason,
    }
    if fold_match is not None:
        document["fold_postcondition"] = fold_match.to_dict()
    optional = {
        "capture_stamp_ns": capture_stamp_ns,
        "lifecycle_phase": lifecycle_phase,
        "model_sha256": model_sha256,
        "robot_model_sha256": robot_model_sha256,
        "settled": settled,
        "clear_pose_verified": clear_pose_verified,
    }
    document.update(
        {key: value for key, value in optional.items() if value is not None}
    )
    return document


def prerecorded_annotation_observation(
    annotation: Mapping[str, Any],
    *,
    pixel_to_workcell_homography: Sequence[Sequence[float]],
    calibration_sha256: str,
    expected_full_mask_area_px: float,
    topology_confidence: float,
    stale: bool = False,
    outline_iou: float | None = None,
    fold_axis_costs: Mapping[str, float] | None = None,
    fold_direction_costs: Mapping[str, float] | None = None,
) -> dict[str, Any]:
    """Convert reviewed polygon evidence into a runtime observation.

    This is intentionally not an image model. It lets the rest of the
    perception→state→plan pipeline run against prerecorded reviewed evidence.
    """
    if not isinstance(stale, bool):
        raise TowelTaskContractError("stale must be boolean")
    normalized = validate_annotation(annotation)
    if not math.isfinite(expected_full_mask_area_px) or expected_full_mask_area_px <= 0.0:
        raise TowelTaskContractError(
            "expected_full_mask_area_px must be finite and positive"
        )
    if not math.isfinite(topology_confidence) or not 0.0 <= topology_confidence <= 1.0:
        raise TowelTaskContractError(
            "topology_confidence must be within 0..1"
        )
    if len(calibration_sha256) != 64 or any(
        character not in "0123456789abcdef"
        for character in calibration_sha256
    ):
        raise TowelTaskContractError(
            "calibration_sha256 must be lowercase SHA-256"
        )
    matrix = _homography(pixel_to_workcell_homography)
    mask_diagnostics = inspect_binary_mask(
        rasterize_annotation_mask(annotation)
    )
    mask_area = polygon_area(annotation["segmentation_polygon_px"])
    visible_area_ratio = min(1.0, mask_area / expected_full_mask_area_px)
    corners = [
        {
            "point_xy_m": list(
                project_pixel_to_workcell(corner["point_px"], matrix)
            ),
            "confidence": float(corner["confidence"]),
            "visible": bool(corner["visible"]),
            "graspable": bool(corner["graspable"]),
        }
        for corner in annotation["corners"]
    ]
    flatness_score = 0.0
    usable_points = [
        corner["point_xy_m"]
        for corner in corners
        if corner["visible"] and corner["graspable"]
    ]
    if len(usable_points) == 4:
        metrics = quadrilateral_metrics(usable_points)
        flatness_score = max(
            0.0,
            1.0
            - max(
                metrics.edge_relative_spread,
                metrics.diagonal_relative_difference,
            ),
        )
    state = annotation["state_label"]
    expected_component_count = 0 if state == "EMPTY" else 1
    clear_view_valid = bool(
        not annotation.get("occluded", False)
        and mask_diagnostics.component_count == expected_component_count
        and not mask_diagnostics.touches_frame_border
    )
    fold_count = (
        2 if state == "FOLD_2_COMPLETE"
        else 1 if state == "FOLD_1_COMPLETE"
        else 0
    )
    document: dict[str, Any] = {
        "schema_version": 1,
        "record_kind": "towel_state_observation",
        "observation_id": normalized["observation_id"],
        "source_sha256": normalized["source_sha256"],
        "calibration_sha256": calibration_sha256,
        "visible_area_ratio": visible_area_ratio,
        "topology_confidence": topology_confidence,
        "flatness_score": flatness_score,
        "fold_count": fold_count,
        "outline_iou": outline_iou,
        "stale": bool(stale),
        "clear_view_valid": clear_view_valid,
        "corners": corners,
        "backend": "prerecorded_reviewed_polygon_v1",
        "height_available": bool(annotation.get("height_available", False)),
    }
    if fold_axis_costs is not None:
        document["fold_axis_costs"] = dict(fold_axis_costs)
    if fold_direction_costs is not None:
        document["fold_direction_costs"] = dict(fold_direction_costs)
    return document
