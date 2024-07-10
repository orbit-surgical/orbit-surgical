# Copyright (c) 2024, The ORBIT-Surgical Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import math

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import AssetBaseCfg
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.sensors import FrameTransformerCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

import orbit.surgical.tasks.surgical.reach.mdp as mdp
from orbit.surgical.tasks.surgical.reach.reach_env_cfg import ReachEnvCfg

##
# Pre-defined configs
##
from omni.isaac.lab.markers.config import FRAME_MARKER_CFG  # isort: skip
from orbit.surgical.assets.star import STAR_CFG  # isort: skip


##
# Environment configuration
##


@configclass
class STARReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # simulation settings
        self.viewer.eye = (2.0, 2.0, 1.0)

        self.scene.table = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Table",
            spawn=sim_utils.UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd",
            ),
            init_state=AssetBaseCfg.InitialStateCfg(pos=(0.55, 0.0, 0.0), rot=(0.70711, 0.0, 0.0, 0.70711)),
        )

        # switch robot to star
        self.scene.robot = STAR_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["endo360_needle"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["endo360_needle"]
        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["star_joint_.*"], scale=0.5, use_default_offset=True
        )
        # override command generator body
        # end-effector is along z-direction
        self.commands.ee_pose = mdp.UniformPoseCommandCfg(
            asset_name="robot",
            body_name="endo360_needle",
            resampling_time_range=(4.0, 4.0),
            debug_vis=True,
            ranges=mdp.UniformPoseCommandCfg.Ranges(
                pos_x=(0.45, 0.55),
                pos_y=(0.0, 0.3),
                pos_z=(0.2, 0.4),
                roll=(0.0, 0.0),
                pitch=((2/3)*math.pi, (2/3)*math.pi),
                yaw=(0.0, 0.0),
            ),
        )

        self.events.reset_robot_joints = EventTerm(
            func=mdp.reset_joints_by_scale,
            mode="reset",
            params={
                "position_range": (0.5, 1.5),
                "velocity_range": (0.0, 0.0),
            },
        )

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/star_link_0",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/endo360_needle",
                    name="end_effector",
                ),
            ],
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
