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
    id="Isaac-Reach-Dual-STAR-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": joint_pos_env_cfg.STARReachEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.STARReachPPORunnerCfg,
    },
)

gym.register(
    id="Isaac-Reach-Dual-STAR-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": joint_pos_env_cfg.STARReachEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.STARReachPPORunnerCfg,
    },
)

##
# Inverse Kinematics - Absolute Pose Control
##

gym.register(
    id="Isaac-Reach-Dual-STAR-IK-Abs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_abs_env_cfg.STARReachEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.STARReachPPORunnerCfg,
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Reach-Dual-STAR-IK-Abs-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_abs_env_cfg.STARReachEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.STARReachPPORunnerCfg,
    },
    disable_env_checker=True,
)

##
# Inverse Kinematics - Relative Pose Control
##

gym.register(
    id="Isaac-Reach-Dual-STAR-IK-Rel-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_rel_env_cfg.STARReachEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.STARReachPPORunnerCfg,
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Reach-Dual-STAR-IK-Rel-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": ik_rel_env_cfg.STARReachEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.STARReachPPORunnerCfg,
    },
    disable_env_checker=True,
)
