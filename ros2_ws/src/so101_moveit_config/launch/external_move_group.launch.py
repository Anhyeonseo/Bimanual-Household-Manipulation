"""MoveGroup settings for the remote single-point STM32 backend."""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


# The STM32 Action adapter intentionally accepts exactly one target point, so
# MoveIt compares that target (rather than a separate start point) with the
# current state. Keep the tolerance bounded just above the largest approved
# registration preset (+0.40 rad) while the hardware adapter continues to
# enforce calibrated joint limits, fresh feedback, and a 2 s duration ceiling.
SINGLE_POINT_START_TOLERANCE_RAD = 0.45
EXECUTION_DURATION_SCALING = 1.2
GOAL_DURATION_MARGIN_S = 1.0


def _moveit_config():
    config = (
        MoveItConfigsBuilder("so101_left", package_name="so101_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    # MoveIt normally expects point zero to equal the current state. The B
    # milestone deliberately sends one target point only. Hardware safety is
    # still enforced by the STM32 Action adapter's stricter calibrated limits.
    config.trajectory_execution["trajectory_execution"] = {
        "allowed_start_tolerance": SINGLE_POINT_START_TOLERANCE_RAD,
        "allowed_execution_duration_scaling": EXECUTION_DURATION_SCALING,
        "allowed_goal_duration_margin": GOAL_DURATION_MARGIN_S,
    }
    return config


def generate_launch_description():
    return generate_move_group_launch(_moveit_config())
