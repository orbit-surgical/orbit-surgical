# Copyright (c) 2024, The ORBIT-Surgical Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the da Vinci Research Kit (dVRK) Patient Side Manipulator (PSM) robots.

The following configurations are available:

* :obj:`PSM_CFG`: dVRK PSM robot arm
* :obj:`PSM_HIGH_PD_CFG`: dVRK PSM robot arm with stiffer PD control

Reference: https://github.com/med-air/SurRoL
           https://github.com/WPI-AIM/dvrk_env
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg

from orbit.surgical.assets import ORBITSURGICAL_ASSETS_DATA_DIR

##
# Configuration
##

PSM_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ORBITSURGICAL_ASSETS_DATA_DIR}/Robots/dVRK/PSM/psm_col.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "psm_yaw_joint": 0.01,
            "psm_pitch_end_joint": 0.01,
            "psm_main_insertion_joint": 0.07,
            "psm_tool_roll_joint": 0.01,
            "psm_tool_pitch_joint": 0.01,
            "psm_tool_yaw_joint": 0.01,
            "psm_tool_gripper1_joint": -0.09,
            "psm_tool_gripper2_joint": 0.09,
        },
        pos=(0.0, 0.0, 0.15),
    ),
    actuators={
        "psm": ImplicitActuatorCfg(
            joint_names_expr=[
                "psm_yaw_joint",
                "psm_pitch_end_joint",
                "psm_main_insertion_joint",
                "psm_tool_roll_joint",
                "psm_tool_pitch_joint",
                "psm_tool_yaw_joint",
            ],
            effort_limit=12.0,
            velocity_limit=1.0,
            stiffness=800.0,
            damping=40.0,
        ),
        "psm_tool": ImplicitActuatorCfg(
            joint_names_expr=["psm_tool_gripper.*"],
            effort_limit=0.1,
            velocity_limit=0.2,
            stiffness=500,
            damping=0.1,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of dVRK PSM robot arm."""


PSM_HIGH_PD_CFG = PSM_CFG.copy()
PSM_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
PSM_HIGH_PD_CFG.actuators["psm"].stiffness = 800.0
PSM_HIGH_PD_CFG.actuators["psm"].damping = 40.0
"""Configuration of dVRK PSM robot arm with stiffer PD control.

This configuration is useful for task-space control using differential IK.
"""
