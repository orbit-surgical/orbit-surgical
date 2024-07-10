# Copyright (c) 2024, The ORBIT-Surgical Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


import gymnasium as gym

from . import agents, ik_abs_env_cfg, ik_rel_env_cfg, joint_pos_env_cfg

##
# Register Gym environments.
##

##
# Joint Position Control
##

gym.register(
    id="Isaac-Handover-Needle-Dual-PSM-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": joint_pos_env_cfg.NeedleHandoverEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.HandoverNeedlePPORunnerCfg,
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Handover-Needle-Dual-PSM-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": joint_pos_env_cfg.NeedleHandoverEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.HandoverNeedlePPORunnerCfg,
    },
    disable_env_checker=True,
)

##
# Inverse Kinematics - Absolute Pose Control
##

gym.register(
    id="Isaac-Handover-Needle-Dual-PSM-IK-Abs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_abs_env_cfg.NeedleHandoverEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.HandoverNeedlePPORunnerCfg,
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Handover-Needle-Dual-PSM-IK-Abs-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_abs_env_cfg.NeedleHandoverEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.HandoverNeedlePPORunnerCfg,
    },
    disable_env_checker=True,
)

##
# Inverse Kinematics - Relative Pose Control
##

gym.register(
    id="Isaac-Handover-Needle-Dual-PSM-IK-Rel-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_rel_env_cfg.NeedleHandoverEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.HandoverNeedlePPORunnerCfg,
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Handover-Needle-Dual-PSM-IK-Rel-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_rel_env_cfg.NeedleHandoverEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.HandoverNeedlePPORunnerCfg,
    },
    disable_env_checker=True,
)
