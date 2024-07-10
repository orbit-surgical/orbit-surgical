# ORBIT-Surgical: Assets for Robots and Objects

This extension contains configurations for various assets. The configuration instances are
used to spawn and configure the instances in the simulation. They are passed to their corresponding
classes during construction.

## Organizing custom assets

The recommended directory structure inside `data` is as follows:

* **`Robots/<Company-Name>/<Robot-Name>`**: The USD files should be inside `<Robot-Name>` directory with
  the name of the robot.
* **`Props/<Prop-Type>/<Prop-Name>`**: The USD files should be inside `<Prop-Name>` directory with the name
  of the prop. This includes mounts, objects and markers.
* **`ActuatorNets/<Company-Name>`**: The actuator networks should inside `<Company-Name>` directory with the
  name of the actuator that it models.
* **`Policies/<Task-Name>`**: The policy should be JIT/ONNX compiled with the name `policy.pt`. It should also
  contain the parameters used for training the checkpoint. This is to ensure reproducibility.
* **`Test/<Test-Name>`**: The asset used for unit testing purposes.

## Referring to the assets in your code

You can use the following snippet to refer to the assets:

```python

from orbit.surgical.assets import ORBITSURGICAL_ASSETS_DATA_DIR


# PSM
PSM_USD_PATH = f"{ORBITSURGICAL_ASSETS_DATA_DIR}/Robots/dVRK/PSM/psm.usd"
```
