# MuJoCo ROS Mocap Plugin

With this plugin position and orientation of named mocap bodies can be set with either publishing `mujoco_ros_msgs/MocapState` messages or the `set_mocap_state` service.

## Usecase

This can be used in conjunction with a MuJoCo weld constraint to set position and orientation of a non-mocap body. By changing the solver parameters of the constraint the constraint can be made soft, scaling the generated force on the welded non-mocap body with the extent of constraint violation.


# Licensing

This work is licensed under the BSD 3-Clause License (see LICENSE).
