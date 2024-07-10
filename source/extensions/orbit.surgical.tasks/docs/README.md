# ORBIT-Surgical: Environment Suite

The environments follow the `gym.Env` API from OpenAI Gym version `0.21.0`. The environments are registered using
the Gym registry.

Each environment's name is composed of `Isaac-<Task>-<Robot>-v<X>`, where `<Task>` indicates the skill to learn
in the environment, `<Robot>` indicates the embodiment of the acting agent, and `<X>` represents the version of
the environment (which can be used to suggest different observation or action spaces).

The environments are configured using either Python classes (wrapped using `configclass` decorator) or through
YAML files. The template structure of the environment is always put at the same level as the environment file
itself. However, its various instances are included in directories within the environment directory itself.
This looks like as follows:

```tree
orbit/surgical/tasks/surgical/
├── __init__.py
└── lift
    ├── config
    │   └── needle
    │       ├── agent  # <- this is where we store the learning agent configurations
    │       ├── __init__.py  # <- this is where we register the environment and configurations to gym registry
    │       ├── ik_abs_env_cfg.py
    │       ├── ik_rel_env_cfg.py
    │       └── joint_pos_env_cfg.py
    ├── __init__.py
    └── lift_env_cfg.py  # <- this is the base task configuration
```

The environments are then registered in the `orbit/surgical/tasks/surgical/lift/config/needle/__init__.py`:

```python
gym.register(
    id="Isaac-Lift-Needle-PSM-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": joint_pos_env_cfg.NeedleLiftEnvCfg,
    },
    disable_env_checker=True,
)
```
