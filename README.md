# MuJoCo ROS

MuJoCo ROS wraps the [MuJoCo physics engine](https://mujoco.org/) with ROS
interfaces for launching, controlling, observing, and extending simulations.
The project uses a hybrid ROS 1 / ROS 2 layout where shared MuJoCo behavior
lives in one implementation and ROS-version-specific code is isolated at the
boundary.

The 1.0.0 release line supports ROS 1 Noetic/One-style builds and ROS 2
Humble-style builds from the same repository.

### Continuous Integration

Check | Status
----- | ----:
ROS One | [![CI ROS 1](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/ci.ros1.yaml/badge.svg?branch=hybrid-devel)](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/ci.ros1.yaml?query=branch%3Ahybrid-devel) |
ROS Humble | [![CI ROS 2](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/ci.ros2.yaml/badge.svg?branch=hybrid-devel)](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/ci.ros2.yaml?query=branch%3Ahybrid-devel) |
Format | [![Format](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/format.yaml/badge.svg?branch=hybrid-devel)](https://github.com/ubi-agni/mujoco_ros_pkgs/actions/workflows/format.yaml?query=branch%3Ahybrid-devel) |
CodeCov | [![codecov](https://codecov.io/gh/ubi-agni/mujoco_ros_pkgs/branch/hybrid-devel/graph/badge.svg?token=W7uHKcY0ly)](https://codecov.io/gh/ubi-agni/mujoco_ros_pkgs) |

Pushes and pull requests for `dev/**` branches run a reduced CI matrix by
default: one GLFW build/test job per ROS version. The ROS 1 and ROS 2 CI
workflows can be started manually with `full_matrix` and `coverage` enabled
when a development branch needs the full render-backend matrix and Codecov
upload before integration. The ROS 2 workflow also has an optional Humble GLFW
clang-tidy job for development branches.


## Package Status

Active hybrid packages:

| Package | Purpose |
| --- | --- |
| `mujoco_ros` | Core simulator wrapper, server, rendering, plugin loading, ROS API |
| `mujoco_ros_msgs` | Shared ROS 1 / ROS 2 messages, services, and actions |
| `mujoco_ros_testing_utils` | Shared test fixtures and test assets |
| `mujoco_ros_sensors` | MuJoCo native sensor publisher plugin |
| `mujoco_ros_laser` | Raycast-based laser scan plugin |
| `mujoco_ros_mocap` | Mocap body topic/service plugin |
| `mujoco_ros_control` | ROS 1 `ros_control` and ROS 2 `ros2_control` integration |

## Features

| Feature | ROS 1 | ROS 2 |
| --- | :---: | :---: |
| Core MuJoCo server | yes | yes |
| GUI and headless rendering | yes | yes |
| Camera streams | yes | yes |
| Pluginlib-based MuJoCo plugins | yes | yes |
| Services/actions for simulation control | yes | yes |
| MuJoCo sensor plugin | yes | yes |
| Laser plugin | yes | yes |
| Mocap plugin | yes | yes |
| Control plugin | yes | yes |
| Python bindings | yes | yes |

## Build

Install MuJoCo first. If MuJoCo is installed from a tarball, make sure the
environment points at it:

```bash
export MUJOCO_DIR=$HOME/.mujoco/mujoco-3.3.5
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_DIR/lib
export LIBRARY_PATH=$LIBRARY_PATH:$MUJOCO_DIR/lib
```

For ROS 1:

```bash
catkin build --verbose
catkin test --verbose
catkin_test_results --verbose
```

For ROS 2:

```bash
colcon build --event-handlers console_direct+
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Install-space builds should also be checked before release because header
install regressions have previously only appeared there.

The `mujoco_node` startup log reports the package version and configured Git
description. C++ code can also include `mujoco_ros/version.hpp` for the generated
`MJR_PROJECT_VERSION`, `MJR_GIT_DESCRIBE`, `MJR_GIT_COMMIT`, `MJR_GIT_BRANCH`,
and `MJR_GIT_DIRTY` macros.

## Examples

Core server:

```bash
roslaunch mujoco_ros launch_server.launch
ros2 launch mujoco_ros launch_server.launch.xml
```

Plugin examples:

```bash
roslaunch mujoco_ros_sensors sensors_plugin_example.launch
ros2 launch mujoco_ros_sensors sensors_plugin_example.launch.py

roslaunch mujoco_ros_laser laser_plugin_example.launch
ros2 launch mujoco_ros_laser laser_plugin_example.launch.py

roslaunch mujoco_ros_mocap mocap_example.launch
ros2 launch mujoco_ros_mocap mocap_example.launch.py
```

Control examples:

```bash
roslaunch mujoco_ros_control mujoco_ros_control.launch
ros2 launch mujoco_ros_control mujoco_ros_control.launch.py

roslaunch mujoco_ros_control mujoco_ros_control_ignore_actuators.launch
ros2 launch mujoco_ros_control mujoco_ros_control_ignore_actuators.launch.py
```

The control examples use four pendulums to demonstrate effort fallback,
effort-motor routing, velocity-actuator routing, position-actuator routing, and
the `ignore_actuators` generalized-force fallback mode.

## Documentation

The documentation is published at
[ubi-agni.github.io/mujoco_ros_pkgs](https://ubi-agni.github.io/mujoco_ros_pkgs/).
The Sphinx sources live under `docs/`.
The Extended Params schema reference lives at
`docs/concepts/robot_description/robot_description.rst`.

## Licensing

This work is licensed under the BSD 3-Clause License, see `LICENSE`.
It is built on top of MuJoCo, which is released under the Apache 2.0 License.
For MuJoCo and third-party notices, see `THIRD_PARTY_NOTICES`.

## Cite

If you use this framework in research, please cite:

```bibtex
@inproceedings{leinsMuJoCoROSIntegrating2025,
  author={Leins, David P. and Haschke, Robert and Ritter, Helge},
  title={MuJoCo ROS: Integrating ROS with the MuJoCo Engine for Accurate and Scalable Robotic Simulation},
  booktitle={2025 IEEE International Conference on Simulation, Modeling, and Programming for Autonomous Robots (SIMPAR)},
  year={2025},
  doi={10.1109/SIMPAR62925.2025.10979045}
}
```
