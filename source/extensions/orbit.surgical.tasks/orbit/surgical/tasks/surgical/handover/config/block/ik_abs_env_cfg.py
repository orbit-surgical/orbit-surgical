# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.orbit.assets.articulation import ArticulationCfg
from omni.isaac.orbit.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from omni.isaac.orbit.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from omni.isaac.orbit.utils import configclass

from . import joint_pos_env_cfg

##
# Pre-defined configs
##
from orbit.surgical.assets.psm import PSM_HIGH_PD_CFG  # isort: skip


@configclass
class BlockHandoverEnvCfg(joint_pos_env_cfg.BlockHandoverEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set PSM as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot_1 = PSM_HIGH_PD_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot_1",
            init_state=ArticulationCfg.InitialStateCfg(pos=(0.2, 0.0, 0.15), rot=(1.0, 0.0, 0.0, 0.0)),
        )
        self.scene.robot_2 = PSM_HIGH_PD_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot_2",
            init_state=ArticulationCfg.InitialStateCfg(pos=(-0.2, 0.0, 0.15), rot=(1.0, 0.0, 0.0, 0.0)),
        )

        # Set actions for the specific robot type (PSM)
        self.actions.body_1_joint_pos = DifferentialInverseKinematicsActionCfg(
            asset_name="robot_1",
            joint_names=[
                "psm_yaw_joint",
                "psm_pitch_end_joint",
                "psm_main_insertion_joint",
                "psm_tool_roll_joint",
                "psm_tool_pitch_joint",
                "psm_tool_yaw_joint",
            ],
            body_name="psm_tool_yaw_link",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=(0.0, 0.009, 0.0)),
        )
        self.actions.body_2_joint_pos = DifferentialInverseKinematicsActionCfg(
            asset_name="robot_2",
            joint_names=[
                "psm_yaw_joint",
                "psm_pitch_end_joint",
                "psm_main_insertion_joint",
                "psm_tool_roll_joint",
                "psm_tool_pitch_joint",
                "psm_tool_yaw_joint",
            ],
            body_name="psm_tool_yaw_link",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=(0.0, 0.009, 0.0)),
        )


@configclass
class BlockHandoverEnvCfg_PLAY(BlockHandoverEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
