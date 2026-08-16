"""Launch the hash-pinned Top YOLO-OBB backend without robot targets."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


FROZEN_HOLDOUT_V2_SHA256 = (
    "da7ea8a03a264ea798b049dc00ae0579517da1f6cfa59e92c9e6998c8dcbf7f2"
)


def generate_launch_description():
    package_share = FindPackageShare("so101_top_perception")
    detector_config = PathJoinSubstitution(
        [package_share, "config", "top_perception.yaml"]
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
            DeclareLaunchArgument("bundle_manifest"),
            DeclareLaunchArgument(
                "python_executable",
                description=(
                    "Python from the isolated OpenCV 4.10 ROS runtime venv."
                ),
            ),
            DeclareLaunchArgument(
                "expected_holdout_manifest_sha256",
                default_value=FROZEN_HOLDOUT_V2_SHA256,
            ),
            DeclareLaunchArgument("inference_hz", default_value="4.0"),
            DeclareLaunchArgument(
                "debug_image_topic",
                default_value="/perception/top/yolo_obb_debug",
            ),
            DeclareLaunchArgument("camera_info", default_value=default_camera_info),
            DeclareLaunchArgument("homography", default_value=default_homography),
            Node(
                package="so101_top_perception",
                executable="top_object_pose_node",
                name="top_object_pose",
                output="screen",
                prefix=[LaunchConfiguration("python_executable")],
                parameters=[
                    detector_config,
                    {
                        "camera_info_path": LaunchConfiguration("camera_info"),
                        "homography_path": LaunchConfiguration("homography"),
                        "detector_backend": "opencv_dnn_ultralytics_obb",
                        "obb_bundle_manifest_path": LaunchConfiguration(
                            "bundle_manifest"
                        ),
                        "obb_expected_holdout_manifest_sha256": (
                            LaunchConfiguration(
                                "expected_holdout_manifest_sha256"
                            )
                        ),
                        "inference_hz": LaunchConfiguration("inference_hz"),
                        "debug_image_topic": LaunchConfiguration(
                            "debug_image_topic"
                        ),
                    },
                ],
            ),
        ]
    )
