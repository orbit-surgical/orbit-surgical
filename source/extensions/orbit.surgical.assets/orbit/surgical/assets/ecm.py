# Copyright (c) 2024, The ORBIT-Surgical Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the da Vinci Research Kit (dVRK) Endoscopic Camera Manipulator (ECM) robots.

The following configurations are available:

* :obj:`ECM_CFG`: dVRK ECM robot arm
* :obj:`ECM_HIGH_PD_CFG`: dVRK ECM robot arm with stiffer PD control

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

ECM_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ORBITSURGICAL_ASSETS_DATA_DIR}/Robots/dVRK/ECM/ecm.usd",
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
            "ecm_yaw_joint": 0.0,
            "ecm_pitch_end_joint": 0.0,
            "ecm_main_insertion_joint": 0.0,
            "ecm_tool_joint": 0.0,
        },
        pos=(0.0, 0.0, 0.15),
    ),
    actuators={
        "ecm": ImplicitActuatorCfg(
            joint_names_expr=[
                "ecm_yaw_joint",
                "ecm_pitch_end_joint",
                "ecm_main_insertion_joint",
                "ecm_tool_joint",
            ],
            effort_limit=12.0,
            velocity_limit=1.0,
            stiffness=800.0,
            damping=40.0,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of dVRK ECM robot arm."""


ECM_HIGH_PD_CFG = ECM_CFG.copy()
ECM_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
ECM_HIGH_PD_CFG.actuators["ecm"].stiffness = 800.0
ECM_HIGH_PD_CFG.actuators["ecm"].damping = 40.0
"""Configuration of dVRK ECM robot arm with stiffer PD control.

This configuration is useful for task-space control using differential IK.
"""
