# MuJoCo ROS

Tools that combine the MuJoCo simulator with ROS. Meant to recreate a base port of [gazebo\_ros\_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) for MuJoCo.

This is a ROS software Project that wraps the [MuJoCo physics engine](https://mujoco.org/) into a ROS package.
It is an extension of the MuJoCo [simulate](https://github.com/deepmind/mujoco/blob/3.2.0/sample/simulate.cc) program, with ROS integration and the possibility to load plugins via pluginlib.

### ROS Versions

This project is mainly built for Ubuntu Focal with ROS Noetic. But we are working on adaptations for more recent Ubuntu systems with ROS One and Humble (ROS 2).

### Continuous Integration

service    | Noetic / One | Humble (coming soon)
---------- | :-----: | :----:
GitHub | [![Format](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/format.yaml/badge.svg?branch=noetic-devel)](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/format.yaml?query=branch%3Anoetic-devel) [![CI](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/ci.yaml/badge.svg?branch=noetic-devel)](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/ci.yaml?query=branch%3Anoetic-devel) | - |
CodeCov | [![codecov](https://codecov.io/gh/ubi-agni/mujoco_ros_pkgs/branch/noetic-devel/graph/badge.svg?token=W7uHKcY0ly)](https://codecov.io/gh/ubi-agni/mujoco_ros_pkgs) | - |

# Features at a Glance

Feature | Noetic / One | Humble
------- | :----------: | :-----:
Interactive GUI  | ✔️ | ✔️ |
Headless Mode (OSMESA\EGL) | ✔️ | ✔️ |
Virtual Camera Streams (RGB, Depth, Segmentation Masks) | ✔️ | ✔️ |
Loading Custom PluginLib Plugins | ✔️ | ✔️ |
Simulation Controls Through Services/Actions | ✔️ | ✔️ |
ROS Control |️ ✔️ | ✖️ ([WIP](https://github.com/tenfoldpaper/mujoco_ros_pkgs/tree/wip_ros_control_humble))|
MuJoCo Sensor Streams  | ✔️ | ✖️ |
Laser Sensors | ✔️ | ✖️ |
Mocap Body Control Topic/Service | ✔️ | ✖️ |
Python Bindings          | (✔️) WIP | ✖️ |
Spawning Objects via ROS | ✖️ | ✖️ |

# Build Instructions
1. Make sure MuJoCo is installed (the current build uses version 3.2.0) and runs on your machine.
2. Create a new ROS workspace or include this repository into an existing workspace.
3. Before building, make sure that your compiler knows where to find the MuJoCo library, e.g. by running
```bash
export MUJOCO_DIR=PATH/TO/MUJOCO/DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_DIR/lib
export LIBRARY_PATH=$LIBRARY_PATH:$MUJOCO_DIR/lib
```
where `PATH/TO/MUJOCO/DIR` is `~/.mujoco/mujoco-3.2.0` if you used the recommended location to install mujoco (if downloaded as tarball). If you built MuJoCo from source and the install path is known to catkin, you can skip this step.

4. Build with `catkin_build`, `catkin b` or `colcon build`.
5. Source your workspace and try `roslaunch mujoco_ros launch_server.launch use_sim_time:=true` to test if it runs.

> **Warning**
> To prevent action servers ignoring actions for a limited time after resetting the simulation, until https://github.com/ros/actionlib/pull/203 is merged, you need to build the PR branch and any packages implementing action servers (like MoveIt) yourself.


### Plugin Examples

As an example for extended functionality through plugins, take a look at [mujoco_ros_control](https://github.com/ubi-agni/mujoco_ros_pkgs/tree/noetic-devel/mujoco_ros_control), [mujoco_screw_plugin](https://github.com/ubi-agni/mujoco_screw_plugin), [mujoco_contact_surfaces](https://github.com/ubi-agni/mujoco_contact_surfaces) or [mujoco_ros_sensors](https://github.com/ubi-agni/mujoco_ros_pkgs/tree/noetic-devel/mujoco_ros_sensors).

We provide some code examples in our [demo repository](https://github.com/ubi-agni/mujoco_ros_demos)


### Documentation

We are currently working on setting up more detailed documentation including tutorials and guides. The current prototype can be found [here](https://davidpl1.github.io/mujoco_ros_pkgs) (though note that this will migrate once its ready for an initial proper release).

Some more structural and configuration info, which is not yet included in the documentation, can be found [here](./mujoco_ros/README.md).

# Licensing

This work is licensed under the BSD 3-Clause License (see LICENSE).
It is built on top of MuJoCo, which was released under an Apache 2.0 License. For the original MuJoCo and further third party licenses, see [THIRD_PARTY_NOTICES](./THIRD_PARTY_NOTICES).

# Cite

If you are using this framework in your research, please cite the following work in your publications:

```bibtex
@inproceedings{leinsMuJoCoROSIntegrating2025,
  author={Leins, David P. and Haschke, Robert and Ritter, Helge},
  title={MuJoCo ROS: Integrating ROS with the MuJoCo Engine for Accurate and Scalable Robotic Simulation},
  booktitle={2025 IEEE International Conference on Simulation, Modeling, and Programming for Autonomous Robots (SIMPAR)},
  year={2025},
  doi={10.1109/SIMPAR62925.2025.10979045}
}
```
