"""Run MoveIt here while the STM32 bridge runs on another ROS host."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def _include_moveit(launch_file, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("so101_moveit_config"),
                    "launch",
                    launch_file,
                ]
            )
        ),
        condition=condition,
    )


def _moveit_actions():
    # The remote Pi owns serial and exports the controller Actions. Starting a
    # second provider here would violate the single-backend contract.
    return [
        _include_moveit("rsp.launch.py"),
        _include_moveit("static_virtual_joint_tfs.launch.py"),
        _include_moveit("external_move_group.launch.py"),
        _include_moveit(
            "moveit_rviz.launch.py",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz; set false for headless execution",
            ),
            *_moveit_actions(),
        ]
    )
