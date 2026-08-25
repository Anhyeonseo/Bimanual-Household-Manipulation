from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools/setup/camera_calibration"
sys.path.insert(0, str(TOOLS))
SPEC = importlib.util.spec_from_file_location(
    "validate_right_tabletop_targets",
    TOOLS / "validate_right_tabletop_targets.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def transform(x: float, y: float, z: float, yaw: float = 0.0) -> np.ndarray:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]
    result[:3, 3] = [x, y, z]
    return result


def sample(
    capture_id: str,
    x: float,
    y: float,
    *,
    xy_error_m: float = 0.004,
    z_error_m: float = 0.003,
    yaw_error_deg: float = 1.0,
) -> dict:
    return {
        "id": capture_id,
        "source": {"sha256": f"sha256-{capture_id}"},
        "top_target_xyz_m": [x, y, 0.0],
        "measured_right_arm_rad": [2.0 * x, 0.0, 0.0, 0.0, 0.0],
        "residual": {
            "xy_error_m": xy_error_m,
            "z_error_m": z_error_m,
            "yaw_error_rad": math.radians(yaw_error_deg),
        },
    }


def valid_capture_document() -> dict:
    return {
        "schema_version": 1,
        "record_kind": "right_tabletop_dual_view_capture",
        "status": MODULE.CAPTURE_STATUS,
        "motion_authorized": False,
        "source_motion_authorized": True,
        "robot_target_available": False,
        "capture": {
            "arm": "right",
            "measured_arm_rad": [0.0] * 5,
            "joint_state_source": "resident_terminal_measured_anchor",
            "top_image_files": [f"top_{index}.png" for index in range(5)],
            "wrist_image_files": [f"wrist_{index}.png" for index in range(5)],
            "maximum_pair_skew_s": 0.1,
            "pair_skew_limit_s": 0.25,
        },
        "resident_torque_hold": {
            "status_service": "/bimanual_stream_adapter/status",
            "owner": MODULE.REQUIRED_CONTROLLED_HOLD_OWNER,
            "arbiter_epoch": 7,
            "torque_hold_active": True,
            "terminal_anchor_stamp": 10.0,
            "required_owner": MODULE.REQUIRED_CONTROLLED_HOLD_OWNER,
            "required_epoch": 7,
        },
    }


def valid_staged_capture_document() -> dict:
    document = valid_capture_document()
    document["record_kind"] = "right_tabletop_staged_capture"
    capture = document["capture"]
    capture["capture_mode"] = "staged_top_then_wrist"
    capture.pop("maximum_pair_skew_s")
    capture.pop("pair_skew_limit_s")
    capture["top_source_stamps"] = [1.0, 1.2, 1.4, 1.6, 1.8]
    capture["wrist_source_stamps"] = [10.0, 10.2, 10.4, 10.6, 10.8]
    document["staged_capture"] = {
        "top_stage_file": "top_stage.yaml",
        "top_stage_sha256": "0" * 64,
        "stationary_board_confirmation": (
            "RIGHT_TABLETOP_BOARD_FIXED_BETWEEN_STAGES"
        ),
        "top_source_stamp_last": 1.8,
        "wrist_source_stamp_first": 10.0,
        "top_completed_before_wrist": True,
    }
    return document


def test_compare_target_poses_reports_planar_error() -> None:
    top = transform(0.30, -0.10, 0.0)
    right = transform(0.306, -0.092, 0.004, math.radians(2.0))
    result = MODULE.compare_target_poses(top, right)
    assert result["xy_error_m"] == pytest.approx(0.010)
    assert result["z_error_m"] == pytest.approx(0.004)
    assert result["yaw_error_deg"] == pytest.approx(2.0)


def test_top_target_uses_rectified_worktable_mapping() -> None:
    camera_to_target = transform(0.0, 0.0, 1.0)
    camera_info = {
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 1.0],
        },
        "distortion_coefficients": {
            "rows": 1,
            "cols": 5,
            "data": [0.0] * 5,
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [
                100.0, 0.0, 0.0, 0.0,
                0.0, 100.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ],
        },
    }
    worktable = {
        "homography": {
            "rectified_pixel_to_board_m": {
                "rows": 3,
                "cols": 3,
                "data": [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0],
            }
        },
        "base_registration": {
            "base_from_board": {
                "rows": 4,
                "cols": 4,
                "data": [
                    [1.0, 0.0, 0.0, 0.2],
                    [0.0, 1.0, 0.0, -0.3],
                    [0.0, 0.0, 1.0, -0.005],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            }
        },
    }
    result = MODULE.top_target_from_worktable(
        camera_to_target, camera_info, worktable
    )
    assert result[:3, 3] == pytest.approx([0.2, -0.3, -0.005])
    assert result[:3, :3] == pytest.approx(np.eye(3))


def test_three_separated_accurate_targets_pass() -> None:
    status, failures, metrics = MODULE.classify(
        [
            sample("center", 0.30, -0.10),
            sample("near", 0.20, -0.10),
            sample("far", 0.30, -0.20),
        ]
    )
    assert status == MODULE.PASS_STATUS
    assert failures == []
    assert metrics["target_xy_span_m"] >= 0.08


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        ({"xy_error_m": 0.016}, "XY"),
        ({"z_error_m": 0.021}, "Z"),
        ({"yaw_error_deg": 3.1}, "yaw"),
    ],
)
def test_coordinate_limit_violation_is_rejected(overrides, reason) -> None:
    captures = [
        sample("center", 0.30, -0.10),
        sample("near", 0.20, -0.10),
        sample("far", 0.30, -0.20, **overrides),
    ]
    status, failures, _ = MODULE.classify(captures)
    assert status == MODULE.REJECTED_STATUS
    assert any(reason in failure for failure in failures)


def test_reused_or_unseparated_targets_are_rejected() -> None:
    status, failures, _ = MODULE.classify(
        [
            sample("same", 0.30, -0.10),
            sample("same", 0.31, -0.10),
            sample("third", 0.32, -0.10),
        ]
    )
    assert status == MODULE.REJECTED_STATUS
    assert any("not unique" in failure for failure in failures)
    assert any("span" in failure for failure in failures)


def test_static_right_arm_configuration_is_rejected() -> None:
    captures = [
        sample("center", 0.30, -0.10),
        sample("near", 0.20, -0.10),
        sample("far", 0.30, -0.20),
    ]
    for capture in captures:
        capture["measured_right_arm_rad"] = [0.0] * 5
    status, failures, _ = MODULE.classify(captures)
    assert status == MODULE.REJECTED_STATUS
    assert any("joint configuration span" in failure for failure in failures)


def test_collinear_target_coverage_is_rejected() -> None:
    status, failures, _ = MODULE.classify(
        [
            sample("one", 0.30, -0.05),
            sample("two", 0.30, -0.15),
            sample("three", 0.30, -0.25),
        ]
    )
    assert status == MODULE.REJECTED_STATUS
    assert any("triangle altitude" in failure for failure in failures)


def test_manual_current_pose_hold_is_not_final_validation_provenance() -> None:
    document = valid_capture_document()
    document["resident_torque_hold"]["owner"] = "resident_hold_validator"
    document["resident_torque_hold"]["required_owner"] = (
        "resident_hold_validator"
    )
    with pytest.raises(RuntimeError, match="resident hold provenance"):
        MODULE.validate_capture_provenance(document)


def test_capture_requires_exact_resident_hold_provenance() -> None:
    document = valid_capture_document()
    MODULE.validate_capture_provenance(document)
    document["resident_torque_hold"]["required_epoch"] = 8
    with pytest.raises(RuntimeError, match="resident hold provenance"):
        MODULE.validate_capture_provenance(document)


def test_capture_pair_skew_is_fail_closed() -> None:
    document = valid_capture_document()
    document["capture"]["maximum_pair_skew_s"] = 0.3
    with pytest.raises(RuntimeError, match="synchronization"):
        MODULE.validate_capture_provenance(document)


def test_staged_capture_accepts_ordered_fixed_board_provenance() -> None:
    MODULE.validate_capture_provenance(valid_staged_capture_document())


@pytest.mark.parametrize(
    "mutation",
    ("confirmation", "ordering", "mode"),
)
def test_staged_capture_provenance_is_fail_closed(mutation: str) -> None:
    document = valid_staged_capture_document()
    if mutation == "confirmation":
        document["staged_capture"]["stationary_board_confirmation"] = ""
    elif mutation == "ordering":
        document["capture"]["wrist_source_stamps"][0] = 1.0
    else:
        document["capture"]["capture_mode"] = "simultaneous"
    with pytest.raises(RuntimeError, match="fixed-board provenance"):
        MODULE.validate_capture_provenance(document)


def test_capture_tool_contains_no_motion_api() -> None:
    source = (
        TOOLS / "capture_right_tabletop_dual_view_sample.py"
    ).read_text(encoding="utf-8")
    for forbidden in (
        "BimanualStreamCommand",
        "ActionClient",
        "FollowJointTrajectory",
        "send_goal",
    ):
        assert forbidden not in source


def test_capture_tool_rejects_stale_and_reused_camera_frames() -> None:
    source = (
        TOOLS / "capture_right_tabletop_dual_view_sample.py"
    ).read_text(encoding="utf-8")
    assert '"stale_camera_frame"' in source
    assert '"reused_camera_frame"' in source
    assert "_latest_top_received_monotonic" in source
    assert "_latest_wrist_received_monotonic" in source


def test_capture_tool_supports_occlusion_aware_staging() -> None:
    source = (
        TOOLS / "capture_right_tabletop_dual_view_sample.py"
    ).read_text(encoding="utf-8")
    assert '"top-stage"' in source
    assert '"wrist-finalize"' in source
    assert "RIGHT_TABLETOP_BOARD_FIXED_BETWEEN_STAGES" in source
    assert "top_completed_before_wrist" in source


def test_translation_correction_is_fit_in_gripper_frame_only(
    monkeypatch,
) -> None:
    correction = np.asarray([0.006, 0.016, 0.002])
    rotations = {
        "one": np.eye(3),
        "two": MODULE.Rotation.from_euler("z", 40.0, degrees=True).as_matrix(),
    }
    samples = []
    for capture_id, rotation in rotations.items():
        value = sample(capture_id, 0.3, -0.1)
        value["residual"]["delta_xyz_m"] = list(-(rotation @ correction))
        samples.append(value)
    monkeypatch.setattr(
        MODULE,
        "_workcell_to_gripper_rotation",
        lambda value, _registration, _urdf: rotations[value["id"]],
    )
    fitted = MODULE.solve_gripper_translation_correction(samples, {}, "")
    assert fitted == pytest.approx(correction)


def test_translation_correction_is_bounded(monkeypatch) -> None:
    samples = [sample("one", 0.3, -0.1), sample("two", 0.2, -0.1)]
    for value in samples:
        value["residual"]["delta_xyz_m"] = [-0.030, 0.0, 0.0]
    monkeypatch.setattr(
        MODULE,
        "_workcell_to_gripper_rotation",
        lambda *_args: np.eye(3),
    )
    with pytest.raises(RuntimeError, match="exceeds"):
        MODULE.solve_gripper_translation_correction(samples, {}, "")


def test_calibrated_partitions_require_disjoint_held_out_data() -> None:
    training = [
        sample("train_one", 0.30, -0.10),
        sample("train_two", 0.42, -0.10),
    ]
    validation = [
        sample("validation_one", 0.30, -0.22),
        sample("validation_two", 0.20, -0.15),
    ]
    status, failures, metrics = MODULE.classify_calibrated_partitions(
        training, validation
    )
    assert status == MODULE.PASS_STATUS
    assert failures == []
    assert metrics["validation"]["capture_count"] == 2

    validation[0]["source"]["sha256"] = training[0]["source"]["sha256"]
    status, failures, _ = MODULE.classify_calibrated_partitions(
        training, validation
    )
    assert status == MODULE.REJECTED_STATUS
    assert any("source hashes overlap" in reason for reason in failures)


def test_corrected_wrist_transform_changes_translation_not_rotation() -> None:
    candidate = {
        "gripper_to_camera": MODULE.matrix_document(transform(0.1, 0.2, 0.3)),
        "mount_center_to_camera": MODULE.matrix_document(
            transform(0.02, 0.03, 0.04, 0.2)
        ),
    }
    correction = np.asarray([0.006, 0.016, 0.002])
    result = MODULE.corrected_wrist_transform_document(candidate, correction)
    original = np.asarray(candidate["gripper_to_camera"]["data"])
    corrected = np.asarray(result["gripper_to_camera"]["data"])
    assert corrected[:3, :3] == pytest.approx(original[:3, :3])
    assert corrected[:3, 3] == pytest.approx(original[:3, 3] + correction)
