import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    ROOT
    / 'ros2_ws'
    / 'src'
    / 'so101_top_perception'
    / 'so101_top_perception'
    / 'shadow_target.py'
)
CONFIG_PATH = (
    ROOT
    / 'ros2_ws'
    / 'src'
    / 'so101_top_perception'
    / 'config'
    / 'top_shadow_target.yaml'
)
SPEC = importlib.util.spec_from_file_location('shadow_target', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def observation(**overrides):
    values = {
        'source_frame': 'top_board',
        'x_m': 0.111740,
        'y_m': 0.158531,
        'yaw_rad': math.radians(86.669084),
        'frame_age_s': 0.05,
        'confidence': 0.90,
        'footprint_inside': True,
        'image_fully_visible': True,
        'motion_authorized': False,
        'robot_target_available': False,
    }
    values.update(overrides)
    return MODULE.BoardObservation(**values)


def test_shadow_config_is_permanently_non_actionable():
    config = MODULE.load_shadow_config(CONFIG_PATH)
    assert config.transform_validated is True
    assert config.status.startswith('SHADOW_ONLY_')
    assert config.reference_generation_compatible is True
    assert config.rejected_mixed_reference_disagreement_mm == pytest.approx(
        118.216
    )


def test_camera_visible_pose_is_outside_approved_hardware_workspace():
    config = MODULE.load_shadow_config(CONFIG_PATH)
    result = MODULE.evaluate_shadow(config, observation())
    assert result.position_m == pytest.approx(
        [0.4713886143024932, -0.165461710262537, 0.006300],
        abs=1e-6,
    )
    assert result.inside_workspace is False
    assert result.transform_validated is True
    assert result.status == 'SHADOW_OUTSIDE_WORKSPACE'


def test_stage7_pick_center_is_inside_expanded_shadow_workspace_but_non_actionable():
    config = MODULE.load_shadow_config(CONFIG_PATH)
    result = MODULE.evaluate_shadow(
        config,
        observation(
            x_m=0.035923,
            y_m=0.150361,
            yaw_rad=-0.034,
            footprint_inside=False,
            image_fully_visible=True,
        ),
    )
    assert result.position_m == pytest.approx(
        [0.3955716143024932, -0.173631710262537, 0.006300],
        abs=1e-6,
    )
    assert result.inside_workspace is True
    assert result.status == 'SHADOW_CANDIDATE_VALIDATED_NON_ACTIONABLE'
    assert result.transform_validated is True


@pytest.mark.parametrize(
    ('field', 'value', 'code'),
    [
        ('frame_age_s', 0.201, 'SOURCE_STALE'),
        ('confidence', 0.69, 'SOURCE_LOW_CONFIDENCE'),
        (
            'image_fully_visible',
            False,
            'SOURCE_IMAGE_FOOTPRINT_CLIPPED',
        ),
        ('source_frame', 'left_base_link', 'SOURCE_FRAME_MISMATCH'),
        ('x_m', -0.001, 'SOURCE_OUTSIDE_BOARD'),
        ('y_m', 0.394, 'SOURCE_OUTSIDE_BOARD'),
        (
            'robot_target_available',
            True,
            'SOURCE_AUTHORIZATION_CONTRACT_VIOLATION',
        ),
        (
            'motion_authorized',
            True,
            'SOURCE_AUTHORIZATION_CONTRACT_VIOLATION',
        ),
    ],
)
def test_invalid_source_observations_fail_closed(field, value, code):
    config = MODULE.load_shadow_config(CONFIG_PATH)
    config = MODULE.ShadowConfig(
        **{**config.__dict__, 'reference_generation_compatible': True}
    )
    with pytest.raises(MODULE.ShadowTargetError) as error:
        MODULE.evaluate_shadow(config, observation(**{field: value}))
    assert error.value.code == code


def test_far_board_corner_is_shadow_only_and_outside_workspace():
    config = MODULE.load_shadow_config(CONFIG_PATH)
    config = MODULE.ShadowConfig(
        **{**config.__dict__, 'reference_generation_compatible': True}
    )
    result = MODULE.evaluate_shadow(
        config,
        observation(x_m=0.18, y_m=0.0, yaw_rad=0.0),
    )
    assert result.inside_workspace is False
    assert result.status == 'SHADOW_OUTSIDE_WORKSPACE'


def test_long_object_may_extend_beyond_board_when_center_and_image_are_valid():
    config = MODULE.load_shadow_config(CONFIG_PATH)
    result = MODULE.evaluate_shadow(
        config,
        observation(
            x_m=0.035923,
            y_m=0.150361,
            footprint_inside=False,
            image_fully_visible=True,
        ),
    )
    assert result.inside_workspace is True
    assert result.status == 'SHADOW_CANDIDATE_VALIDATED_NON_ACTIONABLE'


def test_source_stamp_freshness_rejects_zero_future_and_stale():
    with pytest.raises(MODULE.ShadowTargetError) as zero:
        MODULE.source_stamp_age_seconds(10, 0, 0, 0.2, 0.05)
    assert zero.value.code == 'SOURCE_STAMP_MISSING'
    with pytest.raises(MODULE.ShadowTargetError) as future:
        MODULE.source_stamp_age_seconds(
            1_000_000_000,
            1,
            100_000_000,
            0.2,
            0.05,
        )
    assert future.value.code == 'SOURCE_STAMP_IN_FUTURE'
    with pytest.raises(MODULE.ShadowTargetError) as stale:
        MODULE.source_stamp_age_seconds(
            1_300_000_000,
            1,
            0,
            0.2,
            0.05,
        )
    assert stale.value.code == 'SOURCE_STALE'


def test_matrix_contract_is_rigid():
    config = MODULE.load_shadow_config(CONFIG_PATH)
    rotation = config.base_from_board[:3, :3]
    assert rotation.T @ rotation == pytest.approx(np.eye(3), abs=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1e-12)


def test_message_contract_contains_no_actionable_flag(tmp_path):
    message = (
        ROOT
        / 'ros2_ws'
        / 'src'
        / 'so101_interfaces'
        / 'msg'
        / 'ShadowObjectTarget.msg'
    ).read_text(encoding='utf-8')
    assert 'bool motion_authorized' in message
    assert 'bool robot_target_available' in message
    assert 'bool transform_validated' in message
    assert 'bool source_image_fully_visible' in message
    assert 'trajectory' not in message.lower()
