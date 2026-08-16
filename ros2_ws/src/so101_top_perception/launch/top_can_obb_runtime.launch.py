"""Run the hash-pinned Top-can YOLO-OBB bundle over the shared top camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CAN_HOLDOUT_MANIFEST_SHA256 = (
    "01d2b08c7526f707969f1a9d60736b064f55910c6c283fcb271a5b6a9b7fd936"
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
            DeclareLaunchArgument("python_executable"),
            DeclareLaunchArgument(
                "expected_holdout_manifest_sha256",
                default_value=CAN_HOLDOUT_MANIFEST_SHA256,
            ),
            DeclareLaunchArgument("inference_hz", default_value="4.0"),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/perception/top/can_obb/object_pose_board",
            ),
            DeclareLaunchArgument(
                "diagnostics_topic",
                default_value="/perception/top/can_obb/diagnostics",
            ),
            DeclareLaunchArgument(
                "debug_image_topic",
                default_value="/perception/top/can_obb/debug",
            ),
            DeclareLaunchArgument("camera_info", default_value=default_camera_info),
            DeclareLaunchArgument("homography", default_value=default_homography),
            Node(
                package="so101_top_perception",
                executable="top_object_pose_node",
                name="top_can_object_pose",
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
                        "pose_topic": LaunchConfiguration("pose_topic"),
                        "diagnostics_topic": LaunchConfiguration("diagnostics_topic"),
                        "debug_image_topic": LaunchConfiguration(
                            "debug_image_topic"
                        ),
                    },
                ],
            ),
        ]
    )
