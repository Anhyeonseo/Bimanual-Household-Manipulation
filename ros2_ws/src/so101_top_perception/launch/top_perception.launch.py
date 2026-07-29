from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    detector_config = PathJoinSubstitution(
        [FindPackageShare("so101_top_perception"), "config", "top_perception.yaml"]
    )
    camera_config = FindPackageShare("manipulation_camera_manager")
    default_camera_info = PathJoinSubstitution(
        [camera_config, "config", "top_camera_info.yaml"]
    )
    default_homography = PathJoinSubstitution(
        [camera_config, "config", "top_worktable_homography.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_info",
                default_value=default_camera_info,
                description="Fixed Top-camera intrinsic calibration YAML.",
            ),
            DeclareLaunchArgument(
                "homography",
                default_value=default_homography,
                description="Top-camera board homography YAML.",
            ),
            Node(
                package="so101_top_perception",
                executable="top_object_pose_node",
                name="top_object_pose",
                output="screen",
                parameters=[
                    detector_config,
                    {
                        "camera_info_path": LaunchConfiguration("camera_info"),
                        "homography_path": LaunchConfiguration("homography"),
                    },
                ],
            ),
        ]
    )
