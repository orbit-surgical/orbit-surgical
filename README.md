![ORBIT-Surgical](media/teaser.png)

---

# ORBIT-Surgical

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.1.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-1.0.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://pre-commit.com/)

## Overview

ORBIT-Surgical is a physics-based surgical robot simulation framework with photorealistic rendering in NVIDIA Omniverse. ORBIT-Surgical leverages GPU parallelization to train reinforcement learning and imitation learning algorithms to facilitate study of robot learning to augment human surgical skills. ORBIT-Surgical is built upon [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html) to leverage the latest simulation capabilities for photo-realistic scenes and fast and accurate simulation.


## Setup

### Basic Setup

ORBIT-Surgical is built upon NVIDIA [Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html) and [Isaac Lab](https://github.com/isaac-sim/IsaacLab). For detailed instructions on how to install these dependencies, please refer to the [Isaac Lab installation guide](https://isaac-sim.github.io/IsaacLab/source/setup/installation/index.html).

Please follow the Isaac Sim [documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html) to install the latest Isaac Sim release.

Clone [Isaac Lab repository](https://github.com/isaac-sim/IsaacLab):

```bash
git clone https://github.com/isaac-sim/IsaacLab.git
```

Define the following environment variable to specify the path to your Isaac Lab installation directory:

```bash
# Set the IsaacLab_PATH environment variable to point to your Isaac Lab installation directory
export IsaacLab_PATH=<your_IsaacLab_PATH>
```

Set up a symbolic link between the installed Isaac Sim root folder and `_isaac_sim` in the Isaac Lab directory.

```bash
# create a symbolic link
ln -s ~/.local/share/ov/pkg/isaac-sim-4.1.0 ${IsaacLab_PATH}/_isaac_sim
```

Although using a virtual environment is optional, we recommend using `conda` (detailed instructions [here](https://isaac-sim.github.io/IsaacLab/source/setup/installation/binaries_installation.html#setting-up-the-conda-environment-optional)).

Create and activate your `conda` environment, followed by installing Isaac Lab:

```bash
# Create conda environment
${IsaacLab_PATH}/isaaclab.sh --conda orbitsurgical

# Activate conda environment
conda activate orbitsurgical

# Install all isaac lab extensions
${IsaacLab_PATH}/isaaclab.sh --install
```

Once you are in the virtual environment, you do not need to use `${IsaacLab_PATH}/isaaclab.sh -p` to run python scripts. You can use the default python executable in your environment by running `python` or `python3`. However, for the rest of the documentation, we will assume that you are using `${IsaacLab_PATH}/isaaclab.sh -p` to run python scripts.

<!-- Download and install the [Git Large File Storage (LFS)](https://git-lfs.com/). Once downloaded and installed, set up Git LFS for your user account by running:
```bash
git lfs install
``` -->

Clone [ORBIT-Surgical repository](https://github.com/orbit-surgical/orbit-surgical) to a directory **outside** the Isaac Lab installation directory:

```bash
git clone https://github.com/orbit-surgical/orbit-surgical
```

From within the ORBIT-Surgical directory, install ORBIT-Surgical:

```bash
./orbitsurgical.sh
```

### Set up IDE

To setup the IDE, please follow these instructions:

1. Open the `ORBIT-Surgical` directory on Visual Studio Code IDE.
2. Run VSCode Tasks, by pressing `Ctrl+Shift+P`, selecting `Tasks: Run Task` and running the `setup_python_env` in the drop down menu. When running this task, you will be prompted to add the absolute path to your Isaac Sim installation.

Set your `conda` environment as the default interpreter in VSCode by opening the command palette (`Ctrl+Shift+P`), choosing `Python: Select Interpreter` and selecting your `conda` environment.

If everything executes correctly, it should create a file .python.env in the .vscode directory. The file contains the python paths to all the extensions provided by Isaac Sim and Omniverse. This helps in indexing all the python modules for intelligent suggestions while writing code.


## Workflows

We adopt all robot learning workflows from Isaac Lab. We use the OpenAI Gym registry to register these environments. For each environment, we provide a default configuration file that defines the scene, observations, rewards and action spaces.

The list of environments available registered with OpenAI Gym can be found by running:

```bash
${IsaacLab_PATH}/isaaclab.sh -p source/standalone/environments/list_envs.py
```

### Basic agents

These include basic agents that output zero or random agents. They are useful to ensure that the environments are configured correctly.

Zero-action agent on `Isaac-Reach-PSM-v0`:

```bash
${IsaacLab_PATH}/isaaclab.sh -p source/standalone/environments/zero_agent.py --task Isaac-Reach-PSM-v0 --num_envs 32
```

Random-action agent on `Isaac-Reach-PSM-v0`:

```bash
${IsaacLab_PATH}/isaaclab.sh -p source/standalone/environments/random_agent.py --task Isaac-Reach-PSM-v0 --num_envs 32
```

### Teleoperation

We provide interfaces for providing commands in SE(2) and SE(3) space for robot control. In case of SE(2) teleoperation, the returned command is the linear x-y velocity and yaw rate, while in SE(3), the returned command is a 6-D vector representing the change in pose.

To play inverse kinematics (IK) control with a keyboard device:

```bash
${IsaacLab_PATH}/isaaclab.sh -p source/standalone/environments/teleoperation/teleop_se3_agent.py --task Isaac-Lift-Needle-PSM-IK-Rel-v0 --num_envs 1 --teleop_device keyboard
```

The script prints the teleoperation events configured. For keyboard, these are as follows:

```
Keyboard Controller for SE(3): Se3Keyboard
    Reset all commands: L
    Toggle gripper (open/close): K
    Move arm along x-axis: W/S
    Move arm along y-axis: A/D
    Move arm along z-axis: Q/E
    Rotate arm along x-axis: Z/X
    Rotate arm along y-axis: T/G
    Rotate arm along z-axis: C/V
```

### Imitation Learning

Using the teleoperation devices, it is also possible to collect data for learning from demonstrations (LfD). For this, we support the learning framework [Robomimic](https://robomimic.github.io/) and allow saving data in [HDF5](https://robomimic.github.io/docs/tutorials/dataset_contents.html#viewing-hdf5-dataset-structure) format.

1. Collect demonstrations with teleoperation for the environment `Isaac-Lift-Needle-PSM-IK-Rel-v0`:

```bash
# step a: collect data with keyboard
${IsaacLab_PATH}/isaaclab.sh -p source/standalone/workflows/robomimic/collect_demonstrations.py --task Isaac-Lift-Needle-PSM-IK-Rel-v0 --num_envs 1 --num_demos 10 --teleop_device keyboard
# step b: inspect the collected dataset
${IsaacLab_PATH}/isaaclab.sh -p source/standalone/workflows/robomimic/tools/inspect_demonstrations.py logs/robomimic/Isaac-Lift-Needle-PSM-IK-Rel-v0/hdf_dataset.hdf5
```

2. Split the dataset into train and validation set:

```bash
# split data
${IsaacLab_PATH}/isaaclab.sh -p source/standalone/workflows/robomimic/tools/split_train_val.py logs/robomimic/Isaac-Lift-Needle-PSM-IK-Rel-v0/hdf_dataset.hdf5 --ratio 0.2
```

3. Train a BC agent for `Isaac-Lift-Needle-PSM-IK-Rel-v0` with [Robomimic](https://robomimic.github.io/):

```bash
${IsaacLab_PATH}/isaaclab.sh -p source/standalone/workflows/robomimic/train.py --task Isaac-Lift-Needle-PSM-IK-Rel-v0 --algo bc --dataset logs/robomimic/Isaac-Lift-Needle-PSM-IK-Rel-v0/hdf_dataset.hdf5
```

4. Play the learned model to visualize results:

```bash
${IsaacLab_PATH}/isaaclab.sh -p source/standalone/workflows/robomimic/play.py --task Isaac-Lift-Needle-PSM-IK-Rel-v0 --checkpoint /PATH/TO/model.pth
```

### Reinforcement Learning

Train an agent on `Isaac-Reach-PSM-v0` with [RSL-RL](https://github.com/leggedrobotics/rsl_rl):

```bash
# run script for training
${IsaacLab_PATH}/isaaclab.sh -p source/standalone/workflows/rsl_rl/train.py --task Isaac-Reach-PSM-v0 --headless
# run script for playing with 32 environments
${IsaacLab_PATH}/isaaclab.sh -p source/standalone/workflows/rsl_rl/play.py --task Isaac-Reach-PSM-v0 --num_envs 32 --load_run run_folder_name --checkpoint model.pt
```

### TensorBoard: TensorFlow's visualization toolkit

Monitor the training progress stored in the `logs` directory on [Tensorboard](https://www.tensorflow.org/tensorboard):

```bash
# execute from the root directory of the repository
${IsaacLab_PATH}/isaaclab.sh -p -m tensorboard.main --logdir=logs
```

## Pre-Commit

Pre-committing involves using a framework to automate the process of enforcing code quality standards before code is actually committed to a version control system, like Git. This process involves setting up hooks that run automated checks, such as code formatting, linting (checking for programming errors, bugs, stylistic errors, and suspicious constructs), and running tests. If these checks pass, the commit is allowed; if not, the commit is blocked until the issues are resolved. This ensures that all code committed to the repository adheres to the defined quality standards, leading to a cleaner, more maintainable codebase. To do so, we use the [pre-commit](https://pre-commit.com/) module. Install the module using:

```bash
pip install pre-commit
```

Run the pre-commit with:

```bash
pre-commit run --all-files
```

## Acknowledgement

NVIDIA Isaac Sim is available freely under [individual license](https://www.nvidia.com/en-us/omniverse/download/). For more information about its license terms, please check [here](https://docs.omniverse.nvidia.com/app_isaacsim/common/NVIDIA_Omniverse_License_Agreement.html#software-support-supplement).

Isaac Lab is released under [BSD-3-Clause License](https://github.com/isaac-sim/IsaacLab/blob/main/LICENSE).

Project template is partially from [Template for Isaac Lab Projects](https://github.com/isaac-sim/IsaacLabExtensionTemplate).

### License

This source code is released under [BSD-3-Clause License](https://github.com/orbit-surgical/orbit-surgical/blob/main/LICENCE).


**Author: Masoud Moghani, moghani@cs.toronto.edu**

## Citing

If you use this framework in your work, please cite [this paper](https://arxiv.org/abs/2404.16027):

```text
@article{yu2024orbit,
  title={ORBIT-Surgical: An Open-Simulation Framework for Learning Surgical Augmented Dexterity},
  author={Yu, Qinxi and Moghani, Masoud and Dharmarajan, Karthik and Schorp, Vincent and Panitch, William Chung-Ho and Liu, Jingzhou and Hari, Kush and Huang, Huang and Mittal, Mayank and Goldberg, Ken and others},
  journal={arXiv preprint arXiv:2404.16027},
  year={2024}
}
```
