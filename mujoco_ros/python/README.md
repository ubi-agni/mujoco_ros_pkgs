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
- Attach the GLFW viewer with `attach_viewer(active=True)` when the package is
  built with the GLFW render backend.

`settings` is a runtime proxy. It reads from the latest `_EnvSettings`
snapshot, but writable fields such as `running`, `run`, `rt_factor`,
`busywait`, and `gravity` call thread-safe C++ binding methods. Other
snapshot fields remain read-only. Use `env.settings.snapshot()` when a raw
read-only `_EnvSettings` value object is needed for debugging or tests.

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

Offscreen camera metadata and image ring buffers are exposed through
`mujoco_ros.rendering`:

```python
from mujoco_ros.rendering import OffcamManager

with MujocoEnv(model_path="/path/to/camera_model.xml") as env:
    cameras = OffcamManager(env.binding._offscreen_context, env.model, cam_buff_size=2)
    rgb, depth, segment = cameras.buffer(0)
```

The offscreen helpers subscribe to the ROS image topics published by the core
offscreen renderer. RGB and segmentation buffers are exposed as read-only
`uint8` NumPy arrays, depth buffers as read-only `float` arrays.

## Exposed Data

The Python API exposes read-only value objects:

- `EnvSettings`: runtime settings such as headless/offscreen mode, real-time
  index, run state, load/reset requests, and thread count.
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

The current port intentionally skips MoveIt helpers, direct pixel rendering,
and passive viewer attachment.
