"""Fail-closed board-to-base shadow target calculations."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
import yaml


class ShadowTargetError(RuntimeError):
    """A source observation or shadow configuration failed a safety gate."""

    def __init__(self, code: str, reason: str) -> None:
        super().__init__(reason)
        self.code = code


@dataclass(frozen=True)
class WorkspaceBounds:
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    z_min_m: float
    z_max_m: float
    radial_min_m: float
    radial_max_m: float

    def contains(self, point: np.ndarray) -> bool:
        x_m, y_m, z_m = (float(value) for value in point)
        radius_m = math.hypot(x_m, y_m)
        return (
            self.x_min_m <= x_m <= self.x_max_m
            and self.y_min_m <= y_m <= self.y_max_m
            and self.z_min_m <= z_m <= self.z_max_m
            and self.radial_min_m <= radius_m <= self.radial_max_m
        )


@dataclass(frozen=True)
class ShadowConfig:
    status: str
    reference_generation_compatible: bool
    source_frame: str
    output_frame: str
    base_from_board: np.ndarray
    transform_validated: bool
    board_span_xy_m: tuple[float, float]
    require_source_image_fully_visible: bool
    object_center_height_m: float
    max_frame_age_s: float
    future_tolerance_s: float
    minimum_confidence: float
    workspace: WorkspaceBounds
    rejected_mixed_reference_disagreement_mm: float


@dataclass(frozen=True)
class BoardObservation:
    source_frame: str
    x_m: float
    y_m: float
    yaw_rad: float
    frame_age_s: float
    confidence: float
    footprint_inside: bool
    image_fully_visible: bool
    motion_authorized: bool
    robot_target_available: bool


@dataclass(frozen=True)
class ShadowResult:
    position_m: np.ndarray
    yaw_rad: float
    inside_workspace: bool
    transform_validated: bool
    status: str


def _finite_float(document: dict, key: str) -> float:
    value = float(document[key])
    if not math.isfinite(value):
        raise ValueError(f'{key} must be finite')
    return value


def _load_matrix(document: dict) -> np.ndarray:
    rows = int(document.get('rows', 0))
    cols = int(document.get('cols', 0))
    values = np.asarray(document.get('data', []), dtype=np.float64)
    if rows != 4 or cols != 4 or values.size != 16:
        raise ValueError('base_from_board must be a 4x4 matrix')
    matrix = values.reshape(4, 4)
    if not np.all(np.isfinite(matrix)):
        raise ValueError('base_from_board must be finite')
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12):
        raise ValueError('base_from_board has an invalid homogeneous row')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8):
        raise ValueError('base_from_board rotation is not orthonormal')
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-8):
        raise ValueError('base_from_board rotation determinant must be +1')
    return matrix


def load_shadow_config(path: Path) -> ShadowConfig:
    with path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError('shadow config must be a mapping')
    if int(document.get('schema_version', 0)) != 1:
        raise ValueError('unsupported shadow config schema')
    if bool(document.get('motion_authorized', False)):
        raise ValueError('shadow config must not authorize motion')
    if bool(document.get('robot_target_available', False)):
        raise ValueError('shadow config must not expose a robot target')

    status = str(document.get('status', ''))
    if not status.startswith('SHADOW_ONLY_'):
        raise ValueError('shadow config status must be SHADOW_ONLY')
    reference_generation_compatible = bool(
        document.get('reference_generation_compatible', False)
    )
    frames = document['frames']
    source_frame = str(frames['source'])
    output_frame = str(frames['output'])
    if not source_frame or not output_frame or source_frame == output_frame:
        raise ValueError('shadow frames must be non-empty and distinct')

    transform_document = document['transform']
    base_from_board = _load_matrix(transform_document['base_from_board'])
    transform_validated = bool(transform_document.get('validated', False))
    disagreement = _finite_float(
        transform_document,
        'rejected_mixed_reference_disagreement_mm',
    )

    source_gate = document['source_gate']
    span_values = tuple(float(value) for value in source_gate['board_span_xy_m'])
    if len(span_values) != 2 or any(value <= 0.0 for value in span_values):
        raise ValueError('board_span_xy_m must contain two positive values')
    max_age = _finite_float(source_gate, 'max_frame_age_s')
    future_tolerance = _finite_float(source_gate, 'future_tolerance_s')
    minimum_confidence = _finite_float(source_gate, 'minimum_confidence')
    require_source_image_fully_visible = bool(
        source_gate.get('require_image_fully_visible', True)
    )
    if max_age <= 0.0 or future_tolerance < 0.0:
        raise ValueError('freshness limits are invalid')
    if not 0.0 <= minimum_confidence <= 1.0:
        raise ValueError('minimum_confidence must be in [0, 1]')

    object_height = _finite_float(
        document['object'],
        'center_height_above_board_m',
    )
    if object_height < 0.0:
        raise ValueError('object center height must not be negative')

    workspace_document = document['workspace']
    workspace = WorkspaceBounds(
        **{
            key: _finite_float(workspace_document, key)
            for key in WorkspaceBounds.__dataclass_fields__
        }
    )
    if not (
        workspace.x_min_m < workspace.x_max_m
        and workspace.y_min_m < workspace.y_max_m
        and workspace.z_min_m < workspace.z_max_m
        and 0.0 <= workspace.radial_min_m < workspace.radial_max_m
    ):
        raise ValueError('workspace bounds are invalid')

    return ShadowConfig(
        status=status,
        reference_generation_compatible=reference_generation_compatible,
        source_frame=source_frame,
        output_frame=output_frame,
        base_from_board=base_from_board,
        transform_validated=transform_validated,
        board_span_xy_m=span_values,
        require_source_image_fully_visible=(
            require_source_image_fully_visible
        ),
        object_center_height_m=object_height,
        max_frame_age_s=max_age,
        future_tolerance_s=future_tolerance,
        minimum_confidence=minimum_confidence,
        workspace=workspace,
        rejected_mixed_reference_disagreement_mm=disagreement,
    )


def source_stamp_age_seconds(
    now_ns: int,
    stamp_sec: int,
    stamp_nanosec: int,
    max_age_s: float,
    future_tolerance_s: float,
) -> float:
    stamp_ns = int(stamp_sec) * 1_000_000_000 + int(stamp_nanosec)
    if stamp_ns <= 0:
        raise ShadowTargetError('SOURCE_STAMP_MISSING', 'source stamp is zero')
    age_s = (int(now_ns) - stamp_ns) / 1_000_000_000.0
    if age_s < -future_tolerance_s:
        raise ShadowTargetError(
            'SOURCE_STAMP_IN_FUTURE',
            f'source stamp is {-age_s:.6f}s in the future',
        )
    if age_s > max_age_s:
        raise ShadowTargetError(
            'SOURCE_STALE',
            f'source stamp age {age_s:.6f}s exceeds {max_age_s:.6f}s',
        )
    return max(0.0, age_s)


def evaluate_shadow(
    config: ShadowConfig,
    observation: BoardObservation,
) -> ShadowResult:
    if not config.reference_generation_compatible:
        raise ShadowTargetError(
            'TRANSFORM_REFERENCE_REJECTED',
            'shadow transform mixes incompatible calibration generations',
        )
    if observation.source_frame != config.source_frame:
        raise ShadowTargetError(
            'SOURCE_FRAME_MISMATCH',
            f'expected {config.source_frame}, got {observation.source_frame}',
        )
    if observation.motion_authorized or observation.robot_target_available:
        raise ShadowTargetError(
            'SOURCE_AUTHORIZATION_CONTRACT_VIOLATION',
            'board observation unexpectedly authorizes a robot target',
        )
    values = (
        observation.x_m,
        observation.y_m,
        observation.yaw_rad,
        observation.frame_age_s,
        observation.confidence,
    )
    if not all(math.isfinite(value) for value in values):
        raise ShadowTargetError('SOURCE_NONFINITE', 'source values must be finite')
    if not 0.0 <= observation.frame_age_s <= config.max_frame_age_s:
        raise ShadowTargetError(
            'SOURCE_STALE',
            f'source frame age {observation.frame_age_s:.6f}s is invalid',
        )
    if observation.confidence < config.minimum_confidence:
        raise ShadowTargetError(
            'SOURCE_LOW_CONFIDENCE',
            f'confidence {observation.confidence:.6f} is below threshold',
        )
    if (
        config.require_source_image_fully_visible
        and not observation.image_fully_visible
    ):
        raise ShadowTargetError(
            'SOURCE_IMAGE_FOOTPRINT_CLIPPED',
            'object footprint is not fully visible in the camera image',
        )
    span_x, span_y = config.board_span_xy_m
    if not (0.0 <= observation.x_m <= span_x):
        raise ShadowTargetError('SOURCE_OUTSIDE_BOARD', 'source x is outside board')
    if not (0.0 <= observation.y_m <= span_y):
        raise ShadowTargetError('SOURCE_OUTSIDE_BOARD', 'source y is outside board')

    board_point = np.asarray(
        [
            observation.x_m,
            observation.y_m,
            config.object_center_height_m,
            1.0,
        ],
        dtype=np.float64,
    )
    base_point = (config.base_from_board @ board_point)[:3]
    board_axis = np.asarray(
        [math.cos(observation.yaw_rad), math.sin(observation.yaw_rad), 0.0]
    )
    base_axis = config.base_from_board[:3, :3] @ board_axis
    axis_norm_xy = math.hypot(float(base_axis[0]), float(base_axis[1]))
    if axis_norm_xy <= 1e-9:
        raise ShadowTargetError('YAW_PROJECTION_INVALID', 'yaw axis is vertical')
    yaw_rad = math.atan2(float(base_axis[1]), float(base_axis[0]))
    inside_workspace = config.workspace.contains(base_point)
    status = (
        'SHADOW_CANDIDATE_TRANSFORM_UNVALIDATED'
        if inside_workspace
        else 'SHADOW_OUTSIDE_WORKSPACE'
    )
    if config.transform_validated and inside_workspace:
        status = 'SHADOW_CANDIDATE_VALIDATED_NON_ACTIONABLE'
    return ShadowResult(
        position_m=base_point,
        yaw_rad=yaw_rad,
        inside_workspace=inside_workspace,
        transform_validated=config.transform_validated,
        status=status,
    )
