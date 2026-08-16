from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / "tools/lying_can_upright_application.py"
SPEC = importlib.util.spec_from_file_location(
    "lying_can_upright_application", PATH
)
APP = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = APP
SPEC.loader.exec_module(APP)


def policy(**overrides):
    values = {
        "target_class": "lying_can",
        "maximum_frame_age_s": 0.2,
        "minimum_confidence": 0.8,
        "minimum_aspect_ratio": 1.5,
        "expected_length_m": 0.120,
        "expected_diameter_m": 0.066,
        "length_tolerance_m": 0.010,
        "diameter_tolerance_m": 0.008,
        "routing_deadband_px": 40.0,
        "maximum_top_wrist_center_disagreement_m": 0.025,
        "maximum_top_wrist_yaw_disagreement_rad": math.radians(15.0),
        "maximum_wrist_correction_m": 0.025,
        "maximum_wrist_yaw_correction_rad": math.radians(90.0),
    }
    values.update(overrides)
    return APP.CanPerceptionPolicy(**values)


def observation(source="top", **overrides):
    values = {
        "source": source,
        "class_name": "lying_can",
        "detection_count": 1,
        "confidence": 0.94,
        "frame_age_s": 0.05,
        "center_x_m": 0.42,
        "center_y_m": -0.05,
        "long_axis_yaw_rad": math.radians(10.0),
        "image_long_axis_yaw_rad": math.radians(10.0),
        "major_axis_m": 0.121,
        "minor_axis_m": 0.065,
        "center_x_px": 210.0,
        "center_y_px": 230.0,
        "image_width_px": 640,
        "image_height_px": 480,
        "footprint_inside": True,
        "image_fully_visible": True,
        "bottom_end_sign": 1 if source == "wrist" else None,
    }
    values.update(overrides)
    return APP.CanObservation(**values)


def test_nearest_crossing_yaw_never_commands_a_full_turn() -> None:
    result = APP.nearest_gripper_crossing_yaw(
        math.radians(170.0),
        math.radians(85.0),
    )
    assert abs(result["required_delta_rad"]) <= math.pi / 2.0
    assert result["target_finger_yaw_rad"] == pytest.approx(
        math.radians(80.0)
    )
    assert result["required_delta_rad"] == pytest.approx(math.radians(-5.0))


@pytest.mark.parametrize("count", [0, 2, 3])
def test_observation_requires_exactly_one_can(count: int) -> None:
    with pytest.raises(APP.LyingCanContractError, match="exactly one"):
        APP.validate_observation(policy(), observation(detection_count=count))


def test_observation_rejects_ambiguous_axis_and_wrong_dimensions() -> None:
    with pytest.raises(APP.LyingCanContractError, match="ambiguous"):
        APP.validate_observation(
            policy(), observation(major_axis_m=0.080, minor_axis_m=0.065)
        )
    with pytest.raises(APP.LyingCanContractError, match="length"):
        APP.validate_observation(policy(), observation(major_axis_m=0.160))


def test_routing_uses_source_pixels_and_rejects_center_deadband() -> None:
    assert APP.select_arm(observation(center_x_px=200.0), 40.0) == "left"
    assert APP.select_arm(observation(center_x_px=440.0), 40.0) == "right"
    with pytest.raises(APP.LyingCanContractError, match="deadband"):
        APP.select_arm(observation(center_x_px=320.0), 40.0)


def test_wrist_refinement_requires_physical_bottom_semantics() -> None:
    with pytest.raises(APP.LyingCanContractError, match="physical can bottom"):
        APP.validate_wrist_refinement(
            policy(),
            observation("top"),
            observation("wrist", bottom_end_sign=None),
        )


def test_wrist_refinement_is_bounded_in_xy_and_undirected_yaw() -> None:
    top = observation("top", long_axis_yaw_rad=math.radians(88.0))
    wrist = observation(
        "wrist",
        center_x_m=0.430,
        center_y_m=-0.055,
        long_axis_yaw_rad=math.radians(-88.0),
    )
    result = APP.validate_wrist_refinement(policy(), top, wrist)
    assert result["correction_norm_m"] == pytest.approx(
        math.hypot(0.010, -0.005)
    )
    assert result["yaw_error_rad"] == pytest.approx(math.radians(4.0))


@pytest.mark.parametrize(
    ("image_yaw", "diagnostic"),
    [(0.0, "horizontal"), (math.pi / 2.0, "vertical")],
)
def test_every_image_orientation_still_requires_ninety_degree_upright_pitch(
    image_yaw: float, diagnostic: str
) -> None:
    top = observation(
        "top",
        image_long_axis_yaw_rad=image_yaw,
        long_axis_yaw_rad=math.radians(8.0),
    )
    wrist = observation(
        "wrist",
        image_long_axis_yaw_rad=image_yaw,
        long_axis_yaw_rad=math.radians(9.0),
    )
    plan = APP.build_task_space_plan(policy(), top, wrist, 0.0)
    assert plan["image_orientation_diagnostic"] == diagnostic
    assert plan["upright_contract"]["required_pitch_change_rad"] == pytest.approx(
        math.pi / 2.0
    )
    assert plan["upright_contract"]["position_only_ik_is_sufficient"] is False
    assert plan["execution_authorized"] is False
    assert plan["motion_commands"] == 0
    stages = {stage["name"]: stage for stage in plan["stages"]}
    assert stages["vertical_descend"]["floor_sweep"] is False
    assert stages["rotate_can_upright"][
        "orientation_must_be_verified"
    ] is True
    assert stages["return_q0_hold"]["torque_hold"] is True


def test_plan_only_core_has_no_motion_or_ros_client() -> None:
    source = PATH.read_text(encoding="utf-8")
    for forbidden in (
        "rclpy",
        "BimanualStreamCommand",
        "serial.Serial",
        "create_publisher",
        "create_client",
    ):
        assert forbidden not in source


def test_candidate_contract_is_explicitly_uncommissioned() -> None:
    contract = json.loads(
        (
            ROOT / "config/lying_can_upright_contract.candidate.json"
        ).read_text(encoding="utf-8")
    )
    assert contract["status"] == "COMMISSIONING_REQUIRED"
    assert contract["motion_authorized"] is False
    assert contract["perception"]["expected_length_m"] == 0.13244
    assert contract["perception"]["expected_diameter_m"] == 0.053
    assert (
        contract["perception"]["physical_measurement_evidence"]["mass_g"]
        == 13.0
    )
    assert contract["perception"]["length_tolerance_m"] is None
    assert contract["wrist_refinement"]["maximum_correction_m"] is None
    assert contract["motion_contract"]["floor_sweep_authorized"] is False
    assert (
        contract["motion_contract"]["position_only_ik_authorized_for_upright"]
        is False
    )


def test_generic_observation_message_keeps_obb_geometry_and_semantics() -> None:
    message = (
        ROOT
        / "ros2_ws/src/so101_interfaces/msg/OrientedObjectObservation.msg"
    ).read_text(encoding="utf-8")
    for field in (
        "string class_name",
        "uint32 detection_count",
        "float64 long_axis_yaw_rad",
        "float64 image_long_axis_yaw_rad",
        "float64 major_axis_m",
        "float64 minor_axis_m",
        "string endpoint_semantics",
        "bool motion_authorized",
    ):
        assert field in message
    cmake = (
        ROOT / "ros2_ws/src/so101_interfaces/CMakeLists.txt"
    ).read_text(encoding="utf-8")
    assert '"msg/OrientedObjectObservation.msg"' in cmake
