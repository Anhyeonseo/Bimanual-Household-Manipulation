import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def _dual_moveit_config():
    default_urdf = Path(get_package_share_directory("so101_description")) / "urdf" / "so101_dual_preview.urdf.xacro"
    dual_urdf = Path(os.environ.get("SO101_DUAL_URDF_PATH", default_urdf))
    if not dual_urdf.is_file():
        raise RuntimeError(f"dual robot description does not exist: {dual_urdf}")
    config = (
        MoveItConfigsBuilder(
            "so101_dual_preview", package_name="so101_moveit_config"
        )
        .robot_description(file_path=str(dual_urdf))
        .robot_description_semantic(file_path="config/so101_dual.srdf")
        .robot_description_kinematics(file_path="config/kinematics_dual.yaml")
        .joint_limits(file_path="config/joint_limits_dual.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    config.trajectory_execution = {"moveit_manage_controllers": False}
    return config


def generate_launch_description():
    return generate_move_group_launch(_dual_moveit_config())
