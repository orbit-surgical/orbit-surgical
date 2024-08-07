# Copyright (c) 2024, The ORBIT-Surgical Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to run an environment with a reach state machine.

The state machine is implemented in the kernel function `infer_state_machine`.
It uses the `warp` library to run the state machine in parallel on the GPU.

.. code-block:: bash

    ${IsaacLab_PATH}/isaaclab.sh -p source/standalone/environments/state_machine/reach_dual_psm_sm.py --num_envs 1

"""

"""Launch Omniverse Toolkit first."""

import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Reach state machine for psm platform environments.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(headless=args_cli.headless)
simulation_app = app_launcher.app

"""Rest everything else."""

import gymnasium as gym
import torch
from collections.abc import Sequence

import warp as wp

from omni.isaac.lab.assets import RigidObject

from omni.isaac.lab_tasks.utils.parse_cfg import parse_env_cfg

from omni.isaac.lab.utils.math import subtract_frame_transforms

import orbit.surgical.tasks  # noqa: F401
from orbit.surgical.tasks.surgical.reach_dual.reach_env_cfg import ReachEnvCfg

# initialize warp
wp.init()


class ReachSmState:
    """States for the reach state machine."""

    REST = wp.constant(0)
    REACH = wp.constant(1)


class ReachSmWaitTime:
    """Additional wait times (in s) for states for before switching."""

    REST = wp.constant(0.5)
    REACH = wp.constant(1.0)


@wp.kernel
def infer_state_machine(
    dt: wp.array(dtype=float),
    sm_state: wp.array(dtype=int),
    sm_wait_time: wp.array(dtype=float),
    ee_1_pose: wp.array(dtype=wp.transform),
    ee_2_pose: wp.array(dtype=wp.transform),
    des_final_pose_1: wp.array(dtype=wp.transform),
    des_final_pose_2: wp.array(dtype=wp.transform),
    des_ee_1_pose: wp.array(dtype=wp.transform),
    des_ee_2_pose: wp.array(dtype=wp.transform),
):
    # retrieve thread id
    tid = wp.tid()
    # retrieve state machine state
    state = sm_state[tid]
    # decide next state
    if state == ReachSmState.REST:
        des_ee_1_pose[tid] = ee_1_pose[tid]
        des_ee_2_pose[tid] = ee_2_pose[tid]
        # wait for a while
        if sm_wait_time[tid] >= ReachSmWaitTime.REST:
            # move to next state and reset wait time
            sm_state[tid] = ReachSmState.REACH
            sm_wait_time[tid] = 0.0
    elif state == ReachSmState.REACH:
        des_ee_1_pose[tid] = des_final_pose_1[tid]
        des_ee_2_pose[tid] = des_final_pose_2[tid]
        # TODO: error between current and desired ee pose below threshold
        # wait for a while
        if sm_wait_time[tid] >= ReachSmWaitTime.REACH:
            # move to next state and reset wait time
            sm_state[tid] = ReachSmState.REACH
            sm_wait_time[tid] = 0.0
    # increment wait time
    sm_wait_time[tid] = sm_wait_time[tid] + dt[tid]


class ReachSm:
    """A simple state machine in a robot's task space for a reach task.

    The state machine is implemented as a warp kernel. It takes in the current state of
    the robot's end-effector, and outputs the desired state of the robot's end-effector.
    The state machine is implemented as a finite state machine with the following states:

    1. REST: The robot is at rest.
    2. REACH: The robot reaches to the desired pose. This is the final state.
    """

    def __init__(self, dt: float, num_envs: int, device: torch.device | str = "cpu"):
        """Initialize the state machine.

        Args:
            dt: The environment time step.
            num_envs: The number of environments to simulate.
            device: The device to run the state machine on.
        """
        # save parameters
        self.dt = float(dt)
        self.num_envs = num_envs
        self.device = device
        # initialize state machine
        self.sm_dt = torch.full((self.num_envs,), self.dt, device=self.device)
        self.sm_state = torch.full((self.num_envs,), 0, dtype=torch.int32, device=self.device)
        self.sm_wait_time = torch.zeros((self.num_envs,), device=self.device)

        # desired state
        self.des_ee_1_pose = torch.zeros((self.num_envs, 7), device=self.device)
        self.des_ee_2_pose = torch.zeros((self.num_envs, 7), device=self.device)

        # convert to warp
        self.sm_dt_wp = wp.from_torch(self.sm_dt, wp.float32)
        self.sm_state_wp = wp.from_torch(self.sm_state, wp.int32)
        self.sm_wait_time_wp = wp.from_torch(self.sm_wait_time, wp.float32)
        self.des_ee_1_pose_wp = wp.from_torch(self.des_ee_1_pose, wp.transform)
        self.des_ee_2_pose_wp = wp.from_torch(self.des_ee_2_pose, wp.transform)

    def reset_idx(self, env_ids: Sequence[int] = None):
        """Reset the state machine."""
        if env_ids is None:
            env_ids = slice(None)
        self.sm_state[env_ids] = 0
        self.sm_wait_time[env_ids] = 0.0

    def compute(
            self,
            ee_1_pose: torch.Tensor,
            ee_2_pose: torch.Tensor,
            des_final_pose_1: torch.Tensor,
            des_final_pose_2: torch.Tensor
    ):
        """Compute the desired state of the robot's end-effector."""
        # convert all transformations from (w, x, y, z) to (x, y, z, w)
        ee_1_pose = ee_1_pose[:, [0, 1, 2, 4, 5, 6, 3]]
        ee_2_pose = ee_2_pose[:, [0, 1, 2, 4, 5, 6, 3]]
        des_final_pose_1 = des_final_pose_1[:, [0, 1, 2, 4, 5, 6, 3]]
        des_final_pose_2 = des_final_pose_2[:, [0, 1, 2, 4, 5, 6, 3]]

        # convert to warp
        ee_1_pose_wp = wp.from_torch(ee_1_pose.contiguous(), wp.transform)
        ee_2_pose_wp = wp.from_torch(ee_2_pose.contiguous(), wp.transform)
        des_final_pose_1_wp = wp.from_torch(des_final_pose_1.contiguous(), wp.transform)
        des_final_pose_2_wp = wp.from_torch(des_final_pose_2.contiguous(), wp.transform)

        # run state machine
        wp.launch(
            kernel=infer_state_machine,
            dim=self.num_envs,
            inputs=[
                self.sm_dt_wp,
                self.sm_state_wp,
                self.sm_wait_time_wp,
                ee_1_pose_wp,
                ee_2_pose_wp,
                des_final_pose_1_wp,
                des_final_pose_2_wp,
                self.des_ee_1_pose_wp,
                self.des_ee_2_pose_wp,
            ],
            device=self.device,
        )

        # convert transformations back to (w, x, y, z)
        des_ee_1_pose = self.des_ee_1_pose[:, [0, 1, 2, 6, 3, 4, 5]]
        des_ee_2_pose = self.des_ee_2_pose[:, [0, 1, 2, 6, 3, 4, 5]]
        # convert to torch
        return torch.cat([des_ee_1_pose, des_ee_2_pose], dim=-1)


def main():
    # parse configuration
    env_cfg: ReachEnvCfg = parse_env_cfg(
        "Isaac-Reach-Dual-PSM-IK-Abs-v0",
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )
    # create environment
    env = gym.make("Isaac-Reach-Dual-PSM-IK-Abs-v0", cfg=env_cfg)
    # reset environment at start
    env.reset()
    env.sim.step()

    # create action buffers (position + quaternion)
    actions = torch.zeros(env.unwrapped.action_space.shape, device=env.unwrapped.device)
    actions[:, 3] = 1.0

    # create state machine
    reach_sm = ReachSm(env_cfg.sim.dt * env_cfg.decimation, env.unwrapped.num_envs, env.unwrapped.device)

    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # step environment
            dones = env.step(actions)[-2]

            # observations
            robot_1: RigidObject = env.scene["robot_1"]
            robot_2: RigidObject = env.scene["robot_2"]
            # -- end-effector frame
            ee_1_frame_sensor = env.unwrapped.scene["ee_1_frame"]
            tcp_1_rest_position = ee_1_frame_sensor.data.target_pos_w[..., 0, :].clone() - env.unwrapped.scene.env_origins
            tcp_1_rest_position_b, _ = subtract_frame_transforms(
                robot_1.data.root_state_w[:, :3], robot_1.data.root_state_w[:, 3:7], tcp_1_rest_position
            )
            tcp_1_rest_orientation = ee_1_frame_sensor.data.target_quat_w[..., 0, :].clone()
            ee_2_frame_sensor = env.unwrapped.scene["ee_2_frame"]
            tcp_2_rest_position = ee_2_frame_sensor.data.target_pos_w[..., 0, :].clone() - env.unwrapped.scene.env_origins
            tcp_2_rest_position_b, _ = subtract_frame_transforms(
                robot_2.data.root_state_w[:, :3], robot_2.data.root_state_w[:, 3:7], tcp_2_rest_position
            )
            tcp_2_rest_orientation = ee_2_frame_sensor.data.target_quat_w[..., 0, :].clone()
            # -- target end-effector frame
            desired_pose_1 = env.unwrapped.command_manager.get_command("ee_1_pose")
            desired_pose_2 = env.unwrapped.command_manager.get_command("ee_2_pose")

            # advance state machine
            actions = reach_sm.compute(
                torch.cat([tcp_1_rest_position_b, tcp_1_rest_orientation], dim=-1),
                torch.cat([tcp_2_rest_position_b, tcp_2_rest_orientation], dim=-1),
                desired_pose_1,
                desired_pose_2,
            )

            # reset state machine
            if dones.any():
                reach_sm.reset_idx(dones.nonzero(as_tuple=False).squeeze(-1))

    # close the environment
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
