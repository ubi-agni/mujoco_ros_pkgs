# MuJoCo Ros

This is a ROS package that wraps the [MuJoCo physics engine](https://mujoco.org/) into a ros package.
It is an extension of the MuJoCo [simulate](https://github.com/deepmind/mujoco/blob/2.1.1/sample/simulate.cc) program, with ROS integration and the possibility to load plugins via pluginlib.

As an example for extended functionality via plugin, take a look at the [mujoco_ros_control](https://github.com/DavidPL1/mujoco_ros_pkgs/mujoco_ros_control) package.

# Licensing

This work is licensed under the BSD 3-Clause License (see LICENSE).
It is built on top of MuJoCo 2.1.1, which was released under an Apache 2.0 License. For the original MuJoCo and further third party licenses, see [THIRD_PARTY_NOTICES](./THIRD_PARTY_NOTICES).
