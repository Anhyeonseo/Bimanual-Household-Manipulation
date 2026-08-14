"""Run dual-arm MoveIt planning while the resident executor runs on the Pi."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(filename: str, condition=None, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_moveit_config"), "launch", filename]
            )
        ),
        condition=condition,
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            _include("dual_rsp.launch.py"),
            _include("dual_static_virtual_joint_tfs.launch.py"),
            _include(
                "dual_move_group.launch.py",
                launch_arguments={
                    "allow_trajectory_execution": "false",
                    "disable_capabilities": (
                        "move_group/MoveGroupExecuteTrajectoryAction "
                        "move_group/MoveGroupMoveAction"
                    ),
                },
            ),
            _include(
                "dual_moveit_rviz.launch.py",
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
