from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_slot = LaunchConfiguration("arm_slot")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("so101_description"), "urdf", "so101_left.urdf.xacro"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [FindExecutable(name="xacro"), " ", xacro_file, " arm_slot:=", arm_slot]
            ),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_slot", default_value="left"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="so101_read_only_robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
        ]
    )
