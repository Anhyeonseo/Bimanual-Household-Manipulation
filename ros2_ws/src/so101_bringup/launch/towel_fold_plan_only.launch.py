"""Start dual-arm MoveIt and RViz with every execution path disabled."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package: str, filename: str, *, arguments=None, condition=None):
    path = Path(get_package_share_directory(package)) / "launch" / filename
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def _setup(context):
    urdf_path = Path(LaunchConfiguration("urdf_path").perform(context)).resolve()
    if not urdf_path.is_file():
        raise RuntimeError(f"towel plan-only URDF does not exist: {urdf_path}")
    os.environ["SO101_DUAL_URDF_PATH"] = str(urdf_path)
    use_rviz = LaunchConfiguration("use_rviz")
    return [
        _include("so101_moveit_config", "dual_rsp.launch.py"),
        _include("so101_moveit_config", "dual_static_virtual_joint_tfs.launch.py"),
        _include(
            "so101_moveit_config",
            "dual_move_group.launch.py",
            arguments={
                "allow_trajectory_execution": "false",
                "disable_capabilities": (
                    "move_group/MoveGroupExecuteTrajectoryAction "
                    "move_group/MoveGroupMoveAction"
                ),
            },
        ),
        _include(
            "so101_moveit_config",
            "dual_moveit_rviz.launch.py",
            condition=IfCondition(use_rviz),
        ),
    ]


def generate_launch_description():
    default_urdf = (
        Path(get_package_share_directory("so101_description"))
        / "urdf/so101_dual_right_data_fit_candidate.urdf"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=str(default_urdf),
                description="Exact dual-arm URDF used for plan-only inspection",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz without enabling execution",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
