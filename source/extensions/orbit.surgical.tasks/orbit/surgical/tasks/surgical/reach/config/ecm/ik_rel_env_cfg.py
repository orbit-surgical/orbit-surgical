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
from orbit.surgical.assets.ecm import ECM_HIGH_PD_CFG  # isort: skip


@configclass
class ECMReachEnvCfg(joint_pos_env_cfg.ECMReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set ECM as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = ECM_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (ECM)
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=[
                "ecm_yaw_joint",
                "ecm_pitch_end_joint",
                "ecm_main_insertion_joint",
                "ecm_tool_joint",
            ],
            body_name="ecm_end_link",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"),
            scale=0.5,
        )


@configclass
class ECMReachEnvCfg_PLAY(ECMReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
