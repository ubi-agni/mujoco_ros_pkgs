# Python Bindings

The `mujoco_ros` Python bindings provide a small ROS-version-neutral wrapper
around the hybrid C++ `MujocoEnv` API.

## Build Dependencies

The bindings are built as part of the `mujoco_ros` package.

- ROS 1: `py_binding_tools` and `pybind11_catkin`
- ROS 2: `py_binding_tools`, `ament_cmake_python`, and `pybind11_vendor`

## Minimal Usage

```python
from pathlib import Path

from mujoco_ros import MujocoEnv

model_path = Path("/path/to/model.xml")

with MujocoEnv() as env:
    env.load_model_from_string(str(model_path))
    env.pause()
    env.step(100)
    print(env.sim_info.model_valid)
```

The wrapper uses the core package defaults in both ROS paths: headless mode is
enabled and offscreen rendering is enabled unless parameters override them.
For ROS 1, the Python helper sets `/use_sim_time=true` before constructing the
C++ environment by default. Pass `MujocoEnv(use_sim_time=False)` only when a
standalone script intentionally wants wall-clock ROS time.


## URDF/SRDF Tutorials

There are two main ways to build a `MujocoEnv` from a robot description bundle.

### 1. Load URDF/SRDF directly from file paths

Use `MujocoEnv.from_description(...)` when the URDF and optional SRDF already
exist on disk:

```python
from pathlib import Path

from mujoco_ros import MujocoEnv

urdf_path = Path("/absolute/path/to/robot.urdf")
srdf_path = Path("/absolute/path/to/robot.srdf")

with MujocoEnv.from_description(
    urdf_path=urdf_path,
    srdf_path=srdf_path,
) as env:
    env.pause()
    env.step(1)
    print(env.filename)
    print(env.sim_info.model_valid)
```

This path compiles the description bundle first, then loads the resulting
MuJoCo model into the wrapped C++ environment. If you do not have an SRDF,
pass `""` for `srdf_path`.

### 2. Load URDF/SRDF from ROS topics

Use the regular constructor and pass the same parameters that `mujoco_server`
accepts in launch files:

```python
from mujoco_ros import MujocoEnv

parameters = {
    "urdf.source": "topic",
    "urdf.topic": "/robot_description",
    "srdf.source": "topic",
    "srdf.topic": "/robot_description_semantic",
    "modelfile": "",
}

with MujocoEnv(parameters=parameters) as env:
    env.pause()
    env.step(1)
    print(env.filename)
    print(env.sim_info.model_valid)
```

`urdf.topic` defaults to `robot_description`, and `srdf.topic` defaults to
`robot_description_semantic`, so you can omit those keys when you use the
standard topic names.

The topics must publish latched `std_msgs/String` payloads because the server
resolves the topic source once during environment construction.

### 3. Use file-backed description parameters through the constructor

If you want the server-style parameter path, but your descriptions are still on
disk, you can also supply file-backed bundle parameters directly:

```python
from mujoco_ros import MujocoEnv

parameters = {
    "urdf.source": "file",
    "urdf.path": "/absolute/path/to/robot.urdf",
    "srdf.source": "file",
    "srdf.path": "/absolute/path/to/robot.srdf",
    "description.generate_actuators": "true",
    "description.attach_prefix": "robot1_",
    "modelfile": "/absolute/path/to/world.xml",
}

with MujocoEnv(parameters=parameters) as env:
    env.step(10)
```

This is useful when you want one code path that mirrors your ROS launch
configuration exactly.

## ROS 1 Core Ownership

`MujocoEnv` does not silently start or stop a ROS 1 master by default. This
keeps ownership predictable for applications and tests.

Launch-managed usage, such as `rostest`, should rely on the launch system's
master. `MujocoEnv` still sets `/use_sim_time=true` before ROS C++
initialization unless disabled explicitly:

```python
from mujoco_ros import MujocoEnv

with MujocoEnv() as env:
    env.load_model_from_string("/path/to/model.xml")
```

Standalone scripts can either start `roscore` themselves or use the explicit
managed context:

```python
from mujoco_ros import MujocoEnv, RosCore

with RosCore(managed=True):
    with MujocoEnv() as env:
        env.load_model_from_string("/path/to/model.xml")
```

For simple scripts, `MujocoEnv(manage_ros_core=True)` is also available. It
starts a private `roscore` only if no master is reachable and shuts down only
the process it started. The managed context also sets `/use_sim_time=true`
before the environment starts. More complex applications should prefer explicit
launch files or an explicit `RosCore` context.

The package tests use regular Python `unittest` cases. They exercise
native-module import, existing ROS context handling, model loading, stepping,
runtime settings, services, plugin wrappers, and status/stat access through the
public Python API.

## Supported API

The native binding mirrors the hybrid C++ API and is split internally into
focused binding units for environment wrappers, status structures, and plugin
objects. The public Python class keeps the surface small:

- Load a model from a path or XML string with `load_model_from_string()`.
- Load Python-owned `mujoco.MjModel`/`mujoco.MjData` pairs with
  `load_from_path()` or `load_from_string()`. This requires the Python
  `mujoco` package at runtime.
- Control execution with `pause()`, `unpause()`, `step()`, `reset()`, and
  `set_rt_factor()`.
- Inspect `settings`, `sim_info`, `plugin_stats`, `plugins`,
  `plugin_names`, `filename`, `handle_namespace`, and `is_running`.
- Read and update model gravity with `get_gravity()` and `set_gravity()`.

The settings proxy is a runtime facade. Writable fields such as running, rt_factor, busywait, and gravity call thread-safe C++ binding methods.
The raw _EnvSettings snapshot exposes configuration and internal loading
markers only. Lifecycle requests are not writable or mirrored through settings.
Use env.settings.snapshot() for read-only configuration inspection.

## Interactive Viewer

Import the viewer module from the package root:

```python
from mujoco_ros import MujocoEnv, viewer
```

### Blocking mode

`viewer.launch(env)` blocks the calling thread and owns GLFW on that thread until
the user closes the window. It returns only after the window closes.

```python
with MujocoEnv(model_path="/path/to/model.xml") as env:
    viewer.launch(env)
```

### Passive mode

`viewer.launch_passive(env, auto_sync=False)` returns immediately with a native
lifetime handle. The viewer runs on a dedicated GUI thread. Only one live viewer
is allowed per environment; a second launch raises
`RuntimeError("a viewer is already running for this MujocoEnv")`.

The handle supports `close()`, `is_running()`, `sync(state_only=False)`,
`lock()` (reentrant context manager), and context-manager exit (which calls
`close()`). Stale handles from an earlier viewer generation are
generation-checked: `close()` on a stale or already-closed handle is an
idempotent no-op; `is_running()` returns `False`; `sync()` and `lock()` raise
`RuntimeError("viewer is not running")`.

Closing a passive window (Exit button, `handle.close()`, or handle context exit)
stops only that viewer. The environment and physics loop stay available.

Automatic passive mode (`auto_sync=True`) synchronizes the viewer on each
rendered frame and around supported binding access while Python is idle.
Manual passive mode (the default) never auto-syncs; call `handle.sync()` after
binding changes, typically while holding `handle.lock()`:

```python
with MujocoEnv(model_path="/path/to/model.xml") as env:
    with viewer.launch_passive(env, auto_sync=True) as handle:
        env.unpause()
        env.set_gravity([0.0, 0.0, -9.81])
```

```python
with MujocoEnv(model_path="/path/to/model.xml") as env:
    with viewer.launch_passive(env) as handle:
        with handle.lock():
            env.pause()
        handle.sync()
```

#### Covered auto-sync operations

When `auto_sync=True`, these binding reads pull viewer-side state into Python
before returning:

- gravity (`get_gravity()`)
- Runtime Options (`runtime_options`, `apply_runtime_options()` reads)
- settings (`settings`, `settings.snapshot()`)
- simulation state/info (`sim_state`, `sim_info`)
- model/data snapshots (`model`, `data`)
- running state (`is_running`, `settings.running`)

These binding writes push Python-side state to the viewer after succeeding:

- load (`load_model_from_string()`, `load_from_path()`, `load_from_string()`)
- step, reset
- pause/unpause (`pause()`, `unpause()`, `toggle_paused()`)
- real-time factor (`set_rt_factor()`, `settings.rt_factor`)
- busywait (`settings.busywait`)
- gravity (`set_gravity()`, `settings.gravity`)
- Runtime Options (`apply_runtime_options()`)
- enable/disable flag helpers (`set_enableflag()`, `set_disableflag()`,
  `toggle_enableflag()`, `toggle_disableflag()`)

Raw writes through `env.model` or `env.data` are unsupported and do not sync the
viewer.

#### Build requirement and backend reporting

Interactive viewing requires a build configured with `WITH_GUI=ON`. Builds with
`WITH_GUI=OFF` raise
`RuntimeError("mujoco_ros.viewer requires a build configured with WITH_GUI=ON")`.
Missing-display and GLFW initialization errors propagate; they are not swallowed.

Visible GLFW and offscreen RenderCore are independent:

- `WITH_GUI` controls the visible GLFW viewer backend.
- `OFFSCREEN_BACKEND` controls RenderCore offscreen capture.
- `pymujoco_ros.__viewer_backend__` reports the first (`"GLFW"` or `"NONE"`).
- `pymujoco_ros.__render_backend__` reports the second.

#### Deprecation

`MujocoEnv.attach_viewer(active=True)` is deprecated. It maps `active=True` to
blocking `viewer.launch(self)` and `active=False` to
`viewer.launch_passive(self, auto_sync=True)`.

#### Known limitations

Passive GLFW runs on a dedicated non-process-main viewer thread (see ADR-0026).
This follows MuJoCo's passive-viewer shape but may be platform- or driver-sensitive
on some systems.

Hybrid-NVIDIA laptop frozen-frame behavior remains a post-implementation manual
verification item, not a guaranteed v1 fix. Cooperative `pump()` and a
separate-process GUI remain future options.

## Runtime Options

`env.runtime_options` returns an immutable snapshot of active MuJoCo option
fields. `env.apply_runtime_options(dict)` applies a partial patch through the
same transaction path as ROS dynamic reconfigure and ROS 2 parameters. Field
names match `RuntimeOptionsSnapshot` (integrator, solver, timestep, iterations,
gravity, solimp, disable/enable flags, and related arrays).

Rejected patches leave the previous snapshot unchanged and do not advance the
Options Epoch. Validation failures raise `ValueError` with
`field + ": " + message`. During the Loading Window, reads and writes raise
`RuntimeError("Runtime Options unavailable during Loading Window")`. Constructor
keyword `runtime_options={...}` applies startup-only patches before the first
model load.

Plugin configuration can be supplied before native construction:

```python
plugin_config = [{
    "type": "mujoco_ros/TestPlugin",
    "example_param": 0.0,
}]

with MujocoEnv(plugin_config=plugin_config) as env:
    env.load_from_path("/path/to/model.xml")
```

A complete documented example that loads a model with a plugin, changes runtime
settings, steps the simulation, and reads simulation/plugin information lives
at `docs/python_bindings/examples/core_plugin_example.py`.

Additional parameter YAML files can be passed with `config_files=[...]`, and
flat parameters can be passed with `parameters={...}`. Launch-provided params
continue to work normally. In ROS 2, these Python-provided params are folded
into `sys.argv` before the native `rclcpp` node is constructed, so pass them
to the first `MujocoEnv` created in a Python process.

For Python-owned models, `MujocoEnv(python_reload_service=True)` replaces the
native reload service with a Python-owned reload service. Empty reload requests
reload the current Python model source, while non-empty requests load the
provided path or XML string.

The test suite also calls the regular ROS services from Python using `rospy`
or `rclpy`, but service-client convenience wrappers are not part of the public
`MujocoEnv` API yet.

## Plugins And Rendering

`env.plugins` returns generic `_MujocoPlugin` objects for the loaded plugins.
Each object exposes the plugin name, type, load/reset timing, and callback
timing EMAs. `env.plugin_names` provides the old name-list convenience view.

Each plugin belongs to the current **Plugin Generation**. Reload replaces the
generation. Python handles reacquire the host on every access; a handle kept
from before reload raises
`RuntimeError("plugin handle belongs to an inactive Plugin Generation")`.

Downstream packages can provide richer Python plugin wrappers by calling
`mujoco_ros.plugins.register_plugin_binding()` when imported, or through Python
entry points in the `mujoco_ros.plugins` group. Entry point names may match the
plugin type suffix, the full plugin type with non-identifier characters
sanitized, or the plugin instance name. Loaded objects can be a callable wrapper
class, or expose `bind(plugin)` / `from_mujoco_plugin(plugin)`.

```python
# setup.py / setup.cfg equivalent
entry_points={
    "mujoco_ros.plugins": [
        "MyPlugin = my_package.my_plugin:MyPlugin",
    ],
}
```

The same entry points are importable as dynamic submodules, for example
`import mujoco_ros.plugins.MyPlugin`.

The hybrid plugin packages register specialized wrappers when imported:

- `mujoco_ros_sensors.MujocoRosSensorsPlugin`
- `mujoco_ros_laser.LaserPlugin`
- `mujoco_ros_mocap.MocapPlugin`
- `mujoco_ros_control.MujocoRosControlPlugin`

Offscreen camera metadata and frame access are exposed through
`mujoco_ros.rendering`:

```python
from mujoco_ros.rendering import OffcamManager

with MujocoEnv(model_path="/path/to/camera_model.xml") as env:
    with OffcamManager(env.binding._camera_publication_transport, env.model, cam_buff_size=2) as cameras:
        rgb, depth, segment = cameras.buffer(0)
```

Python consumers read the same committed `RenderCore` capture as ROS camera
publishers. The Python path does not subscribe to ROS image topics and does
not keep a private image ring. Python demand is registered independently, so a
Python consumer can request a capture without changing the configured ROS
publication cadence.

`buffer(last_n=...)` returns stable copied NumPy arrays. RGB and segmentation
arrays have dtype `uint8`; depth arrays have dtype `float32`. For zero-copy
access, use a borrowed view as a context manager:

```python
with cameras.camera(0).borrow_latest_rgb() as rgb:
    assert not rgb.flags.writeable
    capture_id = rgb.capture_id
```

The borrowed view is read-only and remains valid until its context exits. A
copy remains valid after later captures, resize operations, and reloads.
Borrowed and copied frames carry one camera-specific capture identity. ROS and
Python consumers therefore observe one shared capture rather than separate
render results.

Call `OffcamManager.close()`, or use it as a context manager, to release
Python's continuous render demand. Manager destruction performs the same
cleanup.

Reloads advance the frame generation. Existing managers rebind their Python
demand to the new generation, while a borrowed view already acquired keeps its
old lease and remains readable. Acquire a new borrowed view after reload.

Camera wrappers are also rebound by camera ID after reload. Their `width`,
`height`, `cam_name`, `fps`, and plane metadata describe the active camera
descriptor. A native camera descriptor held separately is an old-layout
snapshot. Name lookups are refreshed: an old camera name is removed and the
new name is available after the next manager lookup.

`cam.buffer` remains a stable compatibility object for legacy code. It still
delegates native names such as `getBufferHandles()` and legacy frame-count
properties, and it is callable as `cam.buffer(last_n=2)` for copied aggregate
snapshots. Legacy ring indices, lock methods, reset mutators, and counter
setters are unsupported and raise `RuntimeError`; an unconfigured plane's
read-only frame count remains `0` for compatibility.

The aggregate `buffer()` result always has three elements and uses `None` for
an unconfigured plane. Its explicit `borrow_latest_*()` and `copy_*()`
accessors raise `RuntimeError` for an unconfigured plane. They never return an
empty or fabricated frame. For a configured plane with no committed frame yet,
`borrow_latest_*()` raises `RuntimeError`; `copy_*()` and aggregate `buffer()`
snapshots may return `None`. The old native ring-buffer indices, lock methods,
counter setters, and reset mutators raise `RuntimeError`. `getBufferHandles()` follows the same
three-slot shape and returns `None` for missing planes.

Offscreen rendering uses the shared `mujoco_ros_render_core` library. The
offscreen backend is selected at build time through `OFFSCREEN_BACKEND` (ANY,
EGL, OSMesa, or DISABLE). Visible GLFW GUI is controlled independently by
`WITH_GUI=ON` or `WITH_GUI=OFF`. When configured camera history or byte demand exceeds
the RenderCore budget, model setup fails loudly with an explicit error rather
than shrinking history or returning empty placeholders. At runtime, slot or
generation-capacity exhaustion surfaces as non-terminal `FrameStatusCode` values
such as `kFrameSlotsExhausted` and `kGenerationCapacityExhausted`. Backend
disabled, initialization failure, and other context-integrity loss
(`kTerminalError`, `kBackendFailure`, `kBackendUnavailable`) are terminal rather
than empty or fabricated frames.

## Exposed Data

The Python API exposes read-only value objects:

- `EnvSettings`: a low-level snapshot of headless/offscreen and simulation-time
  configuration, busy-wait mode, thread count, and internal runtime markers.
  Lifecycle requests are not exposed through this object.
- `SimInfo`: model path, loading status, pause state, pending steps, and
  real-time factor information.
- `SimState`: low-level compatibility object for measured slowdown, model
  validity, and load count. Prefer `sim_info` for new application code unless
  you specifically need measured slowdown.
- `PluginStat`: plugin name, type, load/reset time, and callback timing EMAs.

State changes are performed through explicit methods such as `step()`,
`reset()`, `pause()`, `unpause()`, `toggle_paused()`, `set_rt_factor()`, and
`set_gravity()`.

## Current Limitations

The current port intentionally skips MoveIt helpers and direct pixel rendering.
Cooperative viewer `pump()` and a separate-process GUI are not implemented yet.
Passive GLFW runs on a dedicated non-process-main viewer thread (ADR-0026); this
follows MuJoCo's passive-viewer shape but may be platform- or driver-sensitive.
Hybrid-NVIDIA laptop frozen-frame behavior is tracked separately and is not
guaranteed fixed in v1.
