# Copyright (c) 2024, The ORBIT-Surgical Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from omni.isaac.lab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from omni.isaac.lab.utils import configclass

from . import joint_pos_env_cfg

##
# Pre-defined configs
##
from orbit.surgical.assets.star import STAR_HIGH_PD_CFG  # isort: skip


@configclass
class STARReachEnvCfg(joint_pos_env_cfg.STARReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set STAR as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot_1 = STAR_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot_1")
        self.scene.robot_1.init_state.pos = (0.0, 0.0, 0.0)
        self.scene.robot_1.init_state.rot = (1.0, 0.0, 0.0, 0.0)
        self.scene.robot_2 = STAR_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot_2")
        self.scene.robot_2.init_state.pos = (0.8, 0.0, 0.0)
        self.scene.robot_2.init_state.rot = (1.0, 0.0, 0.0, 0.0)
        # Set actions for the specific robot type (STAR)
        self.actions.arm_1_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot_1",
            joint_names=["star_joint_.*"],
            body_name="endo360_needle",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
        )
        self.actions.arm_2_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot_2",
            joint_names=["star_joint_.*"],
            body_name="endo360_needle",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
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
