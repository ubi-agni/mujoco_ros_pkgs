# MuJoCo Ros

This is a ROS package that wraps the [MuJoCo physics engine](https://mujoco.org/) into a ros package.
It is an extension of the MuJoCo [simulate](https://github.com/deepmind/mujoco/blob/2.1.1/sample/simulate.cc) program, with ROS integration and the possibility to load plugins via pluginlib.

As an example for extended functionality via plugin, take a look at the [mujoco_ros_control](https://github.com/DavidPL1/mujoco_ros_pkgs/mujoco_ros_control) package.

## Plugins
Plugins provide an easy way of including new functionality into _mujoco\_ros_. The lifecycle of a Plugin is as follows:
1. Once an _mjData_ instance is created (and stored in a __MujocoEnv_), each configured plugin is instanciated and a pointer to it is stored in the responsible _MujocoEnv_. Directly after creation its `init` function is called, which provides the plugin instance with its specific configuration, if one was provided. This function should take care of generic, non-instance specific configuration.
Afterwards the `load` function is called, which makes _mjModel_ and _mjData_ available to the plugin. This function should handle the model/instance specific setup of the plugin.

plugins have different callback functions defined in their base class, users should override the callback functions they intend to use. These are the `controlCallback`, `passiveCallback`, and `renderCallback` functions. The first two are automatically configured to run when the `mjcb_passive` function is called by MuJoCo. The latter allows plugins to add visualisation objects before a scene is rendered.

> **Warning**
> A plugin should __never__ override the base mujoco callback functions! _mujoco\_ros_ uses them to resolve the appropriate environment and run their list of registered plugin instance callbacks. Instance agnostic plugins should be realized as singletons that return a reference to the singleton in their constructor.

___

# Build Instructions
1. Make sure MuJoCo is installed (the current build uses version 2.1.1) and runs on your machine.
2. Create a new ROS workspace or include this repository into an existing workspace.
3. Before building, make sure that your compiler knows where to find the MuJoCo library, e.g. by running
```bash
export MUJOCO_HOME=PATH/TO/MUJOCO/DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_HOME/lib
export LIBRARY_PATH=$LIBRARY_PATH:$MUJOCO_HOME/lib
```
where `PATH/TO/MUJOCO/DIR` is `~/.mujoco/mujoco211` if you used the recommended location to install mujoco.
4. Build with `catkin_build` or `catkin b`.
5. Source your workspace and try `roslaunch mujoco_ros demo.launch` to test if it runs.

# Licensing

This work is licensed under the BSD 3-Clause License (see LICENSE).
It is built on top of MuJoCo 2.1.1, which was released under an Apache 2.0 License. For the original MuJoCo and further third party licenses, see [THIRD_PARTY_NOTICES](./THIRD_PARTY_NOTICES).
