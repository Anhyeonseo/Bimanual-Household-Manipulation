import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from single_arm_bridge.backend_lease import (
    SUPPORTED_BACKENDS,
    acquire_backend_lease,
)


def _include(
    package_name,
    launch_file,
    launch_arguments=None,
    condition=None,
):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(package_name), "launch", launch_file]
            )
        ),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def _validate_backend(value):
    backend = value.strip().lower()
    if backend not in SUPPORTED_BACKENDS:
        supported = ", ".join(sorted(SUPPORTED_BACKENDS))
        raise RuntimeError(
            f"invalid backend {value!r}; expected exactly one of: {supported}"
        )
    return backend


def _stm32_parameters(allow_motion):
    share = get_package_share_directory("single_arm_bridge")
    config_directory = os.path.join(share, "config")
    parameter_files = [os.path.join(config_directory, "bridge.yaml")]
    local_config = os.path.join(config_directory, "bridge.local.yaml")
    if os.path.isfile(local_config):
        parameter_files.append(local_config)
    parameter_files.append(
        {
            "allow_motion": ParameterValue(allow_motion, value_type=bool),
        }
    )
    return parameter_files


def _backend_actions(backend, allow_motion, lease_owner_pid):
    if backend == "mock":
        return [
            _include(
                "so101_bringup",
                "mock_controllers.launch.py",
            )
        ]
    if backend == "isaac":
        return [
            Node(
                package="so101_isaac_bridge",
                executable="bridge_node",
                name="so101_isaac_bridge",
                output="screen",
                on_exit=Shutdown(reason="Isaac backend exited"),
            )
        ]
    if backend == "stm32":
        return [
            Node(
                package="single_arm_bridge",
                executable="bridge_node",
                name="single_arm_bridge",
                output="screen",
                parameters=_stm32_parameters(allow_motion),
                additional_env={
                    "SO101_BACKEND_LEASE_OWNER_PID": str(lease_owner_pid),
                },
                on_exit=Shutdown(reason="STM32 backend exited"),
            )
        ]
    raise AssertionError(f"validated backend has no provider: {backend}")


def _common_actions(backend, use_rviz):
    actions = []
    if backend != "mock":
        actions.append(
            _include("so101_moveit_config", "rsp.launch.py")
        )
    actions.extend(
        [
            _include(
                "so101_moveit_config",
                "static_virtual_joint_tfs.launch.py",
            ),
            _include("so101_moveit_config", "move_group.launch.py"),
            _include(
                "so101_moveit_config",
                "moveit_rviz.launch.py",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
    return actions


def _launch_setup(context):
    backend = _validate_backend(
        LaunchConfiguration("backend").perform(context)
    )
    allow_motion = LaunchConfiguration("allow_motion")
    use_rviz = LaunchConfiguration("use_rviz")

    try:
        ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    except ValueError as error:
        raise RuntimeError("ROS_DOMAIN_ID must be an integer") from error

    lease = acquire_backend_lease(backend, ros_domain_id)

    def release_lease(_event, _context):
        lease.release()

    context.register_event_handler(OnShutdown(on_shutdown=release_lease))
    try:
        return _backend_actions(
            backend,
            allow_motion,
            os.getpid(),
        ) + _common_actions(backend, use_rviz)
    except Exception:
        lease.release()
        raise


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="mock",
                description="Exactly one backend: mock, isaac, or stm32",
            ),
            DeclareLaunchArgument(
                "allow_motion",
                default_value="false",
                description=(
                    "STM32 motion opt-in; false keeps the hardware backend "
                    "READ_ONLY"
                ),
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz; false enables headless validation",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
