# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import math

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.assets import AssetBaseCfg
from omni.isaac.orbit.assets.articulation import ArticulationCfg
from omni.isaac.orbit.managers import EventTermCfg as EventTerm
from omni.isaac.orbit.managers import SceneEntityCfg
from omni.isaac.orbit.utils import configclass
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR

import orbit.surgical.tasks.surgical.reach_dual.mdp as mdp
from orbit.surgical.tasks.surgical.reach_dual.reach_env_cfg import ReachEnvCfg

##
# Pre-defined configs
##
from orbit.surgical.assets.star import STAR_CFG  # isort: skip


##
# Environment configuration
##


@configclass
class STARReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        self.scene.table = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Table",
            spawn=sim_utils.UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd",
            ),
            init_state=AssetBaseCfg.InitialStateCfg(pos=(0.55, 0.0, 0.0), rot=(0.70711, 0.0, 0.0, 0.70711)),
        )

        # switch robot to star
        self.scene.robot_1 = STAR_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot_1",
            init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0), rot=(1.0, 0.0, 0.0, 0.0)),
        )
        self.scene.robot_2 = STAR_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot_2",
            init_state=ArticulationCfg.InitialStateCfg(pos=(0.8, 0.0, 0.0), rot=(0.0, 0.0, 0.0, 1.0)),
        )
        # override rewards
        self.rewards.end_effector_1_position_tracking.params["asset_cfg"].body_names = ["endo360_needle"]
        self.rewards.end_effector_1_orientation_tracking.params["asset_cfg"].body_names = ["endo360_needle"]
        self.rewards.end_effector_2_position_tracking.params["asset_cfg"].body_names = ["endo360_needle"]
        self.rewards.end_effector_2_orientation_tracking.params["asset_cfg"].body_names = ["endo360_needle"]
        # override actions
        self.actions.arm_action_1 = mdp.JointPositionActionCfg(
            asset_name="robot_1", joint_names=["star_joint_.*"], scale=0.5, use_default_offset=True
        )
        self.actions.arm_action_2 = mdp.JointPositionActionCfg(
            asset_name="robot_2", joint_names=["star_joint_.*"], scale=0.5, use_default_offset=True
        )
        # override command generator body
        # end-effector is along z-direction
        self.commands.ee_pose_1 = mdp.UniformPoseCommandCfg(
            asset_name="robot_1",
            body_name="endo360_needle",
            resampling_time_range=(4.0, 4.0),
            debug_vis=True,
            ranges=mdp.UniformPoseCommandCfg.Ranges(
                pos_x=(0.35, 0.45),
                pos_y=(0.0, 0.2),
                pos_z=(0.15, 0.4),
                roll=(0.0, 0.0),
                pitch=(math.pi, math.pi),
                yaw=(-3.14, 3.14),
            ),
        )

        self.commands.ee_pose_2 = mdp.UniformPoseCommandCfg(
            asset_name="robot_2",
            body_name="endo360_needle",
            resampling_time_range=(4.0, 4.0),
            debug_vis=True,
            ranges=mdp.UniformPoseCommandCfg.Ranges(
                pos_x=(0.35, 0.45),
                pos_y=(0.0, 0.2),
                pos_z=(0.15, 0.4),
                roll=(0.0, 0.0),
                pitch=(math.pi, math.pi),
                yaw=(-3.14, 3.14),
            ),
        )

        self.events.reset_robot_1_joints = EventTerm(
            func=mdp.reset_joints_by_scale,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot_1"),
                "position_range": (0.5, 1.5),
                "velocity_range": (0.0, 0.0),
            },
        )

        self.events.reset_robot_2_joints = EventTerm(
            func=mdp.reset_joints_by_scale,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot_2"),
                "position_range": (0.5, 1.5),
                "velocity_range": (0.0, 0.0),
            },
        )


@configclass
class STARReachEnvCfg_PLAY(STARReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
