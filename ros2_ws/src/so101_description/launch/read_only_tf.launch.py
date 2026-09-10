"""Publish the standalone arm model; no driver or command publisher is started."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = PathJoinSubstitution([FindPackageShare("so101_description"), "urdf", "so101_arm.urdf.xacro"])
    description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ", model,
        " arm_slot:=", LaunchConfiguration("arm_slot"),
    ]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("arm_slot", default_value="left"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}], output="screen"),
    ])
