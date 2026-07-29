from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('so101_top_perception')
    detector_launch = PathJoinSubstitution(
        [package_share, 'launch', 'top_perception.launch.py']
    )
    shadow_config = PathJoinSubstitution(
        [package_share, 'config', 'top_shadow_target.yaml']
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'shadow_config',
                default_value=shadow_config,
                description='Fail-closed Top-to-base shadow target YAML.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(detector_launch),
            ),
            Node(
                package='so101_top_perception',
                executable='top_shadow_target_node',
                name='top_shadow_target',
                output='screen',
                parameters=[
                    {'config_path': LaunchConfiguration('shadow_config')}
                ],
            ),
        ]
    )
