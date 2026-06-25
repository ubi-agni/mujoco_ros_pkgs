# MuJoCo ROS

`mujoco_ros` is the shared simulator core for the hybrid ROS 1 / ROS 2
package set. Most MuJoCo behavior lives here, while ROS-version-specific API
glue lives under `src/ros_one` and `src/ros_two`.

The full documentation is available at
[ubi-agni.github.io/mujoco_ros_pkgs](https://ubi-agni.github.io/mujoco_ros_pkgs/).

## Core Runtime

The core package provides the physics loop, event loop, clock publishing,
viewer integration, offscreen rendering, plugin loading, and the ROS services
and actions used to control a simulation.

ROS 1 exposes runtime tuning through dynamic reconfigure. ROS 2 exposes the
same runtime option names as regular node parameters.

`MujocoEnv` provides `AddNodeToExecutor()` and `RemoveNodeFromExecutor()` for
ROS 2 plugins that create additional lifecycle nodes. Top-level plugins are
registered automatically.

## Python Bindings

The Python bindings expose a hybrid wrapper around `MujocoEnv` through the
`mujoco_ros.MujocoEnv` Python class and the native `pymujoco_ros` module. They
support C++-owned and Python-owned models, runtime settings, service
cross-checks, plugin objects, and offscreen camera buffer helpers. For details,
see [python/README.md](python/README.md) and the Sphinx Python binding docs.

## Plugins

Plugins provide a way to include new simulation functionality in `mujoco_ros`.
After an `mjData` instance is created and stored in a `MujocoEnv`, each
configured plugin is instantiated and initialized with its configuration. The
plugin `Load()` method then receives the current `mjModel` and `mjData`.

Plugins may implement control, passive, render, reset, and last-stage
callbacks. A plugin should not override MuJoCo's global callback functions
directly; `mujoco_ros` owns those callbacks and dispatches to loaded plugin
instances.

## Initial Joint States

Initial joint positions and velocities can be set with ROS parameters. Values
are fetched and applied when a world model is loaded, reset, or reloaded.

For each joint, provide values for all degrees of freedom as a single string so
both ROS parameter systems preserve the intended type:

```yaml
initial_joint_positions:
  joint_map:
    joint1: "-1.57"
    ball_joint: "1.0 0 0 0"
    free_joint: "2.0 1.0 1.06 0.0 0.707 0.0 0.707"

initial_joint_velocities:
  joint_map:
    joint1: "-1.57"
    ball_joint: "0 0 20.0"
    free_joint: "1.0 2.0 3.0 10 20 30"
```

## Camera Streams

Camera streams are implemented in the core package. Each camera in the MuJoCo
model can publish RGB, depth, and segmentation streams.

By default, a named model camera publishes an RGB stream at 15 Hz. Parameters
under `cam_config/CAMERA_NAME/` can override stream type, frequency, size, and
topic names:

```yaml
cam_config:
  workspace_cam:
    stream_type: 3
    frequency: 10
  workspace_cam2:
    stream_type: 5
    use_segid: false
```

As long as image transport topics have no subscribers, offscreen camera images
are not rendered.
