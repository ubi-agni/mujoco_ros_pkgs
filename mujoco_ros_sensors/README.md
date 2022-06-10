# MuJoCo ROS Sensors Plugin

This plugin is used to read the state of the default MuJoCo [sensors](https://mujoco.readthedocs.io/en/latest/XMLreference.html#sensor) and publish readings as ROS messages.

Userdefined sensors will not be supported. Instead, we encurage to implement userdefined sensors as packaged plugins that include message definitions and use the provided `MujocoPlugin` callback functions to compute and publish custom sensor readings.
