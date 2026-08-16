import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("single_arm_bridge")
    config = os.path.join(share, "config", "bimanual_stream.yaml")
    limits = os.path.join(
        share,
        "config",
        "bimanual_operational_limits.json",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motion_authorized",
                default_value="false",
                description="Allow protocol-v2 bimanual torque and stream output",
            ),
            DeclareLaunchArgument(
                "serial_device",
                default_value="auto",
                description="STM32 ST-LINK virtual serial port",
            ),
            DeclareLaunchArgument(
                "unarmed_feedback_refresh_period_s",
                default_value="0.0",
                description=(
                    "Periodically recapture measured 12-axis feedback while "
                    "motion is unauthorized; zero disables it"
                ),
            ),
            Node(
                package="single_arm_bridge",
                executable="bimanual_stream_node",
                name="bimanual_stream_adapter",
                output="screen",
                parameters=[
                    config,
                    {
                        "motion_authorized": LaunchConfiguration(
                            "motion_authorized"
                        ),
                        "serial_device": LaunchConfiguration("serial_device"),
                        "unarmed_feedback_refresh_period_s": LaunchConfiguration(
                            "unarmed_feedback_refresh_period_s"
                        ),
                        "operational_limits_file": limits,
                    },
                ],
            ),
        ]
    )
