# Copyright (c) 2024, The ORBIT-Surgical Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Smart Tissue Autonomous Robot (STAR) robots.

The following configurations are available:

* :obj:`STAR_CFG`: STAR robot
* :obj:`STAR_HIGH_PD_CFG`: STAR robot with stiffer PD control

Reference: https://github.com/SamuelSchmidgall/SurgicalGym
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg

from orbit.surgical.assets import ORBITSURGICAL_ASSETS_DATA_DIR

##
# Configuration
##

STAR_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ORBITSURGICAL_ASSETS_DATA_DIR}/Robots/STAR/star.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "star_joint_1": 0.0,
            "star_joint_2": -0.569,
            "star_joint_3": 0.0,
            "star_joint_4": -2.0,
            "star_joint_5": 0.0,
            "star_joint_6": 2.037,
            "star_joint_7": 0.741,
            "endo360_joint_1": 0.04,
        },
    ),
    actuators={
        "star": ImplicitActuatorCfg(
            joint_names_expr=["star_joint_[1-7]"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "endo360": ImplicitActuatorCfg(
            joint_names_expr=["endo360_joint_1"],
            effort_limit=200.0,
            velocity_limit=0.2,
            stiffness=2e3,
            damping=1e2,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of STAR robot."""


STAR_HIGH_PD_CFG = STAR_CFG.copy()
STAR_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
STAR_HIGH_PD_CFG.actuators["star"].stiffness = 400.0
STAR_HIGH_PD_CFG.actuators["star"].damping = 80.0
"""Configuration of STAR robot with stiffer PD control.

This configuration is useful for task-space control using differential IK.
"""
