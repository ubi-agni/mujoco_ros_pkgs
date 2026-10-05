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

Runtime Options use one transaction boundary shared by ROS and Python. Field
names match `RuntimeOptionsSnapshot` (integrator, solver, timestep,
iterations, gravity, solimp, disable/enable flags, and related arrays). A
rejected patch leaves every effective field unchanged and does not advance the
Options Epoch. During the Loading Window, reads and writes reject with
`Runtime Options unavailable during Loading Window`. Python exposes
`env.runtime_options` and `env.apply_runtime_options(dict)`; validation failures
raise `ValueError` with `field + ": " + message`.

`MujocoEnv` provides `AddNodeToExecutor()` and `RemoveNodeFromExecutor()` for
ROS 2 plugins that create additional lifecycle nodes. Top-level plugins are
registered automatically.

## Python Bindings

The Python bindings expose a hybrid wrapper around `MujocoEnv` through the
`mujoco_ros.MujocoEnv` Python class and the native `pymujoco_ros` module. They
support C++-owned and Python-owned models, runtime settings, service
cross-checks, plugin objects, and direct offscreen camera frame access. Python
borrowed views are read-only and lease-backed. Copying APIs return stable
buffers. Python demand is independent from ROS demand, while both consumers
share one `RenderCore` capture. Reload advances the frame generation and
rebinds existing Python managers and refreshes wrapper layout/name metadata;
held leases remain readable. `OffcamManager` supports `close()` and the
context-manager protocol. Per-camera `cam.buffer` remains a compatibility
object and is also callable for aggregate copied snapshots; missing aggregate
planes are `None`, while explicit missing-plane access raises. Plugin handles
retained across reload raise `RuntimeError("plugin handle belongs to an inactive
Plugin Generation")`. For details, see
[python/README.md](python/README.md) and the Sphinx Python binding docs.

## Plugins

Plugins provide a way to include new simulation functionality in `mujoco_ros`.
After an `mjData` instance is created and stored in a `MujocoEnv`, each
configured plugin is instantiated and initialized with its configuration. The
plugin `Load()` method then receives the current `mjModel` and `mjData`.

Each loaded plugin belongs to one **Plugin Generation** — the complete adapter
set for one Simulation Model and runtime data pair. Reload replaces the
generation; callbacks and handles do not cross the boundary. Python plugin
objects reacquire the host on every access and reject stale handles explicitly.

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

Offscreen rendering is demand-driven. ROS subscriber demand and Python camera
demand are tracked independently. A capture selected by both paths is produced
once by `RenderCore` and is then consumed by both ROS and Python.

## Offscreen RenderCore

Offscreen graphics execution lives in the standalone `mujoco_ros_render_core`
library. It links only `mujoco::mujoco` and the selected offscreen graphics
target (EGL, OSMesa, or none). Visible GLFW remains outside this target. Select
visible GLFW GUI with `WITH_GUI=ON` or `WITH_GUI=OFF`, and select the offscreen
backend with `OFFSCREEN_BACKEND` (`ANY`, `EGL`, `OSMESA`, or `DISABLE`). GLFW
is not a supported offscreen backend.

The Frame Boundary inside RenderCore owns bounded frame-slot and byte budgets.
When configured camera history or byte demand exceeds those budgets, model setup
fails loudly with an explicit error rather than silently truncating history or
returning empty frames.

At runtime, slot or generation-capacity exhaustion is reported through explicit
non-terminal `FrameStatusCode` values such as `kFrameSlotsExhausted` and
`kGenerationCapacityExhausted`, not collapsed into `kTerminalError`. Backend
initialization failures, disabled-backend captures, and other context-integrity
loss (`kTerminalError`, `kBackendFailure`, `kBackendUnavailable`) are terminal:
consumers observe explicit frame status instead of blank images.

`render_backpressure_policy` defaults to `drop`, preserving non-blocking
physics submission. Set it to `wait_for_slot` in ROS 1 dynamic reconfigure,
ROS 2 parameters, or Python when frame frequency matters; this intentionally
waits on the physics side, outside `physics_thread_mutex_`, until a leased
frame slot is released. Cancellation, reload, shutdown, and switching back to
`drop` interrupt the wait. Other values are rejected without changing the
effective policy.
