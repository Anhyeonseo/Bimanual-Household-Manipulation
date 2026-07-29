from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_slot = LaunchConfiguration("arm_slot")
    use_overhead_webcam_mount = LaunchConfiguration("use_overhead_webcam_mount")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("so101_description"), "urdf", "so101_left.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("so101_description"), "config", "display.rviz"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    xacro_file,
                    " arm_slot:=",
                    arm_slot,
                    " use_overhead_webcam_mount:=",
                    use_overhead_webcam_mount,
                ]
            ),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_slot", default_value="left"),
            DeclareLaunchArgument("use_overhead_webcam_mount", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[robot_description],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
