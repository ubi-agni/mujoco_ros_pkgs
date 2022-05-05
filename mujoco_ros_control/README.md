# MuJoCo ros_control Interfaces

This code is based on the [gazebo_ros_control package](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/noetic-devel/gazebo_ros_control) and has been written to provide roughly the same functionality for mujoco_ros.

This is a ROS package for integrating the `ros_control` controller architecture
with the [MuJoCo](https://mujoco.org/) simulator.

This package provides a `mujoco_ros` plugin which instantiates a ros_control
controller manager and connects it to a robot model simulated in MuJoCo.

## How to Load
mujoco_ros parses the ros parameter server under the path /MujocoPlugins for plugin configurations.

Including the following code in a ros launch file that starts the mujoco_ros node, will load the `DefaultRobotHWSim` hardware interface with a control period of 0.001 seconds for a robot with the namespace "my_robot_ns":

```xml
  <rosparam>
    MujocoPlugins:
      - type: mujoco_ros_control/MujocoRosControlPlugin
        hardware:
          type: mujoco_ros_control/DefaultRobotHWSim
          robot_namespace: my_robot_ns # optional
          control_period: 0.001
  </rosparam>
```

The `type` member directly below `MujocoPlugins` tells mujoco_ros to load the `MujocoRosControlPlugin` (via pluginlib). `MujocoRosControlPlugin` then fetches the robot description form the parameter server taking into account the robot namespace, if one was specified, parses the control period and (also via pluginlib) loads the given hardware interface plugin.


# Licensing

This work is licensed under the BSD 3-Clause License (see LICENSE).
The original work it is based on was released under a BSD 3-Clause License (see LICENSE-ORIGINAL).
