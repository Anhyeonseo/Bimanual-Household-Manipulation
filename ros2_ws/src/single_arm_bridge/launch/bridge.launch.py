from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory("single_arm_bridge")
    config_directory = os.path.join(share, "config")
    parameter_files = [os.path.join(config_directory, "bridge.yaml")]
    local_config = os.path.join(config_directory, "bridge.local.yaml")
    if os.path.isfile(local_config):
        parameter_files.append(local_config)
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "allow_motion",
                default_value="false",
                description="Enable the left-arm trajectory backend",
            ),
            DeclareLaunchArgument(
                "allow_right_arm_jog",
                default_value="false",
                description="Enable only the bounded right-arm jog service",
            ),
            DeclareLaunchArgument(
                "publish_bimanual_read_only",
                default_value="false",
                description="Publish verified torque-off 12-axis observations",
            ),
            DeclareLaunchArgument(
                "bimanual_feedback_rate_hz",
                default_value="2.0",
                description="Sequential dual-bus observation rate",
            ),
            DeclareLaunchArgument(
                "left_arm_power_off_confirmed",
                default_value="false",
                description="Confirm physical removal of left-arm 12 V power",
            ),
            DeclareLaunchArgument(
                "require_right_arm_j2_base_limits",
                default_value="false",
                description="Require exact J2-B firmware and command-limit manifest",
            ),
            DeclareLaunchArgument(
                "require_bimanual_operational_limits",
                default_value="false",
                description="Required operator-verified full bimanual operational limits",
            ),
            Node(
                package="single_arm_bridge",
                executable="bridge_node",
                name="single_arm_bridge",
                output="screen",
                parameters=parameter_files
                + [
                    {
                        "allow_motion": LaunchConfiguration("allow_motion"),
                        "allow_right_arm_jog": LaunchConfiguration(
                            "allow_right_arm_jog"
                        ),
                        "publish_bimanual_read_only": LaunchConfiguration(
                            "publish_bimanual_read_only"
                        ),
                        "bimanual_feedback_rate_hz": LaunchConfiguration(
                            "bimanual_feedback_rate_hz"
                        ),
                        "left_arm_power_off_confirmed": LaunchConfiguration(
                            "left_arm_power_off_confirmed"
                        ),
                        "require_right_arm_j2_base_limits": LaunchConfiguration(
                            "require_right_arm_j2_base_limits"
                        ),
                        "require_bimanual_operational_limits": LaunchConfiguration(
                            "require_bimanual_operational_limits"
                        ),
                    }
                ],
            )
        ]
    )
