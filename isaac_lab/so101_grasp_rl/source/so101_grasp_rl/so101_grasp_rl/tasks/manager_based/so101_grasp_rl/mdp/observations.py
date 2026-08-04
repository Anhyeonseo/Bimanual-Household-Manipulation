# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Gate 6 observation terms not already covered by isaaclab.envs.mdp.

Joint state (position/velocity) and previous action already come from the
stable built-ins (joint_pos_limit_normalized, joint_vel_rel, last_action).
What's missing is object-relative state: gripper-frame object position,
object yaw, and object height above the table — see the RL gate plan's
Gate 6 observation spec (config/rl_task_contract.json's task definition).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

import isaaclab.utils.math as math_utils
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.assets import Articulation, RigidObject
    from isaaclab.envs import ManagerBasedRLEnv


def _gripper_pose_w(
    env: ManagerBasedRLEnv, robot_cfg: SceneEntityCfg, gripper_body_name: str
) -> tuple[torch.Tensor, torch.Tensor]:
    """World position/quaternion (x, y, z, w) of the named gripper body."""
    robot: Articulation = env.scene[robot_cfg.name]
    body_ids, _ = robot.find_bodies([gripper_body_name])
    body_id = body_ids[0]
    pos_w = robot.data.body_pos_w.torch[:, body_id, :]
    quat_w = robot.data.body_quat_w.torch[:, body_id, :]
    return pos_w, quat_w


def object_position_in_gripper_frame(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("training_object"),
    gripper_body_name: str = "gripper_link",
) -> torch.Tensor:
    """TrainingObject position expressed in the gripper body's local frame. Shape (N, 3)."""
    obj: RigidObject = env.scene[object_cfg.name]
    gripper_pos_w, gripper_quat_w = _gripper_pose_w(env, robot_cfg, gripper_body_name)
    obj_pos_rel, _ = math_utils.subtract_frame_transforms(
        gripper_pos_w, gripper_quat_w, obj.data.root_pos_w.torch, obj.data.root_quat_w.torch
    )
    return obj_pos_rel


def object_yaw_sin_cos(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("training_object"),
) -> torch.Tensor:
    """sin/cos of the TrainingObject's yaw (world frame). Shape (N, 2)."""
    obj: RigidObject = env.scene[object_cfg.name]
    _, _, yaw = math_utils.euler_xyz_from_quat(obj.data.root_quat_w.torch)
    return torch.stack((torch.sin(yaw), torch.cos(yaw)), dim=1)


def object_height_above_table(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("training_object"),
    table_top_z: float = -0.00859,
) -> torch.Tensor:
    """TrainingObject height above the table top surface (world z). Shape (N, 1).

    ``table_top_z`` default matches the Table transform confirmed in
    config/manual_grasp_poses.json (translate_m[2]=-0.01859 + scale_m[2]/2=0.01).
    """
    obj: RigidObject = env.scene[object_cfg.name]
    height = obj.data.root_pos_w.torch[:, 2] - table_top_z
    return height.unsqueeze(1)
