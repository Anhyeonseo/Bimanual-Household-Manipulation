# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Gate 5: scene only (SO-101 + table + graspable object).

Actions/rewards/terminations below are the minimum needed for a
ManagerBasedRLEnvCfg to construct and step — they are placeholders, not the
real MDP design. Those are Gate 6 (observations), Gate 7 (actions), and
Gate 8 (rewards/success) work, tracked separately in the RL gate plan.

Table/object transforms and the confirmed Q_HOME joint pose come from
config/manual_grasp_poses.json and config/so101_joint_contract.json at the
repo root (Gate 1-3), not re-derived here.
"""

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.shapes.shapes_cfg import CuboidCfg
from isaaclab.utils.configclass import configclass

from . import mdp

##
# Repo-relative asset path
##

# isaac_lab/so101_grasp_rl/source/so101_grasp_rl/so101_grasp_rl/tasks/manager_based/so101_grasp_rl/
# -> repo root is 8 parents up (verified against the actual generated file layout).
_REPO_ROOT = Path(__file__).resolve().parents[8]
_SO101_USD_PATH = str(_REPO_ROOT / "isaac_sim/assets/so101_new_calib/so101_new_calib.usda")

##
# Gate 2/3-confirmed constants (config/manual_grasp_poses.json, config/so101_joint_contract.json)
##

# Q_HOME, deg -> rad. All arm joints 0 deg, gripper -10 deg.
_Q_HOME_JOINT_POS_RAD = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": -0.174533,
}

# TrainingScene/Table transform, captured from the Isaac Sim Property panel.
_TABLE_POS = (0.11715, -0.00018, -0.01859)
_TABLE_SIZE = (0.62974, 0.67526, 0.02)

# TrainingScene/TrainingObject transform (80x25x25mm/20g, confirmed 10/10 grasp).
_OBJECT_POS = (0.19523, 0.02106, 0.00391)
_OBJECT_ROT_XYZW = (0.0, 0.0, 0.590704, 0.806888)  # 72.414 deg about Z
_OBJECT_SIZE = (0.08, 0.025, 0.025)
_OBJECT_MASS_KG = 0.02

# Gate 2 friction (tools/isaac_update_gate2_friction.py).
_GRASP_PHYSICS_MATERIAL = sim_utils.RigidBodyMaterialCfg(
    static_friction=1.0,
    dynamic_friction=0.9,
    restitution=0.0,
)


##
# Scene definition
##


@configclass
class So101GraspRlSceneCfg(InteractiveSceneCfg):
    """SO-101 left arm + fixed table + graspable training object.

    No separate ground plane: the robot's own asset already carries its
    mounting plate (Workcell/arm_base), and adding a z=0 ground plane would
    slice through the table (table top sits at roughly z=-0.0086, see
    _TABLE_POS/_TABLE_SIZE), which is not part of the validated Gate 2/3 setup.
    """

    # robot — our calibrated asset, not Isaac Lab's stock SO101_CFG (which points at
    # Isaac Sim's Nucleus asset and would lose our wrist_flex q0 offset / camera mount).
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=_SO101_USD_PATH,
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(joint_pos=_Q_HOME_JOINT_POS_RAD),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
                effort_limit_sim=10.0,
                velocity_limit_sim=10.0,
                stiffness=17.8,
                damping=0.60,
            ),
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=["gripper"],
                effort_limit_sim=10.0,
                velocity_limit_sim=10.0,
                stiffness=17.8,
                damping=0.60,
            ),
        },
        soft_joint_pos_limit_factor=1.0,
    )

    # table (static)
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=_TABLE_POS),
        spawn=CuboidCfg(
            size=_TABLE_SIZE,
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=_GRASP_PHYSICS_MATERIAL,
        ),
    )

    # graspable training object (dynamic)
    training_object = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/TrainingObject",
        init_state=RigidObjectCfg.InitialStateCfg(pos=_OBJECT_POS, rot=_OBJECT_ROT_XYZW),
        spawn=CuboidCfg(
            size=_OBJECT_SIZE,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=_OBJECT_MASS_KG),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=_GRASP_PHYSICS_MATERIAL,
        ),
    )

    # light
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )


##
# MDP settings (Gate 5 placeholders — see module docstring)
##


@configclass
class ActionsCfg:
    """Gate 7: joint_position_delta control, matching config/rl_task_contract.json.

    ``target_q = current_q + clip(action, -1, 1) * scale`` per control step —
    isaaclab's RelativeJointPositionAction adds the (scaled) action to the
    joint's *current* position each step (not the default pose, unlike
    JointPositionActionCfg's offset semantics), which is exactly the
    incremental design from the RL gate plan's Gate 7 spec. Resulting
    targets beyond a joint's physical limit are enforced by the PhysX joint
    drive itself (confirmed in Gate 1 — wrist_roll clamps at its authored
    72.79 deg upper limit), so no separate software clamp is added here.

    Scales (rad/step, 20 Hz): arm 0.015 (~0.86 deg/step), gripper 0.01 —
    doc defaults, not yet tuned against real hardware.
    """

    joint_pos = mdp.RelativeJointPositionActionCfg(
        asset_name="robot",
        joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
        scale={
            "shoulder_pan": 0.015,
            "shoulder_lift": 0.015,
            "elbow_flex": 0.015,
            "wrist_flex": 0.015,
            "wrist_roll": 0.015,
            "gripper": 0.01,
        },
        use_zero_offset=True,
        clip={".*": (-1.0, 1.0)},
    )


@configclass
class ObservationsCfg:
    """Gate 6: state-based observations (no camera yet — that's Gate 11+).

    24-dim total, matching the RL gate plan's Gate 6 spec:
    joint_pos(6, limit-normalized) + joint_vel(6, rel-to-default)
    + object_pos_in_gripper_frame(3) + object_yaw_sin_cos(2)
    + object_height_above_table(1) + last_action(6).
    """

    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_limit_normalized)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        object_pos_in_gripper_frame = ObsTerm(func=mdp.object_position_in_gripper_frame)
        object_yaw_sin_cos = ObsTerm(func=mdp.object_yaw_sin_cos)
        object_height = ObsTerm(func=mdp.object_height_above_table)
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """No reset randomization yet — Gate 9 curriculum territory.

    Robot/object/table all reset to their authored init_state each episode
    by default; no explicit EventTerm is required for that.
    """


@configclass
class RewardsCfg:
    """Placeholder so the manager is non-empty. Real design is Gate 8."""

    alive = RewTerm(func=mdp.is_alive, weight=1.0)


@configclass
class TerminationsCfg:
    """Placeholder: time-out only. Success/failure conditions are Gate 8."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)


##
# Environment configuration
##


@configclass
class So101GraspRlEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings — num_envs=1 per Gate 5 ("먼저 환경 하나만 실행").
    scene: So101GraspRlSceneCfg = So101GraspRlSceneCfg(num_envs=1, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings — matches config/rl_task_contract.json (episode 8s, 20Hz control).
        self.decimation = 6
        self.episode_length_s = 8.0
        # simulation settings — 120Hz physics / 20Hz control per the RL gate plan.
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
