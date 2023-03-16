<a name="0.4.0"></a>
## [0.4.0] - 2023-03-16

### Added
* NO_OPTIM and ENABLE_SANITIZER cmake options for valgrind/asan dev build optimization.
* Central launchfile for the server to include from other packages.
* `no_x` parameter to completely disable OpenGL.
* Service calls to set and get gravity.
* Service call to fetch loadrequest state over ROS.
* Option to disable offscreen rendering.
* Made camera stream resolutions configurable.
* Callback to notify plugins of a geom change at runtime.

### Changed
* Use same package version for all packages in mujoco_ros_pkgs, following common versioning conventions.

### Fixed
* Time reset issue caused by delayed ros clock message processing.
* Out of bounds quaternion array access in debug print.
* Make sure env name is not an empty string.
* Memory leaks caused by wrong mjModelPtr and mjDataPtr deallocation.
* Sensor tests.
* Disable x usage in tests.
* Now reusing offscreen buffer glfw window on reset.
* Freeing textures from GPU manually causing an error.
* Stopping simulation on model compilation warning if in headless mode.
* Not setting ctrl signal when loading a keyframe.
* Render callbacks using the wrong scene (#17).

### Refactored
* Updated and improved tests.
* More thorough memory deallocation.

Contributors: @DavidPL1, @fpatzelt

<a name="0.3.1"></a>
## [0.3.1] - 2023-01-17

### Added
* Service call for reset.
* Unit tests for single environment library functions.
* Added support for multi-DoF for initial positions and velocities (#3).
* Added support for supplying a model as rosparam (generated e.g. by urdf2mjcf).
* Ensures compatibility with MuJoCo 2.2.2.
* Adds interprocedural optimization compile policy.
* Service call to get/set bodys with freejoints.
* Service call to set geom properties (mass, friction, size, geom_type).
* Allows using wall time instead of ROS time (set `/use_sim_time` to false).
* Adds additional "unbound" speed mode to run as fast as possbile.
* Adds offscreen rendering RGB, depth, and/or segmentation mask images.
* Added `eval/train` mode settings.
* Added `admin_hash` for permission checks in eval mode.


### Changed
* `SINGLE` and `PARALLEL` mode are distinguished by a mode variable in shared code instead of requiring separate lib function calls.
* Step service call changed to be an action call.
* **cmake**: Renamed mujoco_INCLUDE_DIR -> mujoco_INCLUDE_DIRS.
* Time is set to 0 on reset.
* Adds more general debug info.


### Fixed
* Not counting multiple steps in CPU sync.
* Step counting depending on `num_sim_steps` enabled or disabled.
* Wrong declaration of `publishSimTime`.
* Pose quaternion is now normalized before application to prevent setting invalid quaternions in `SetModelState`.
* catkin_lint / cmake issues.
* Missing package.xml dependencies.
* Set clock publisher queue to 1 (#7).
* Internal ROS time updates (#8).
* Transitive dependency propagation with catkin.
* Avoid publishing /clock twice per cycle.
* Makes sure destructors of plugins and envs are called on shutdown (#12).
* Instantly replacing a rosparam loaded xml.
* `get_geom_properties` service not being advertised (#16).
* Cross-thread ints and bools are now atomic.
* slow_down not being applied.
* Missing test world.


### Performance Improvements
* Improved multi-threading concept for `PARALLEL` mode.


### Refactored
* Init function improvement and cleanup.
* Improved `Get/SetModelState` service:
  - Flags are now used to specify which parts of the state message are used to update the state.
* Mutex usage.
* Rendering functionallity making it much more runtime efficient.

Contributors: @DavidPL1, @rhaschke, @fpatzelt, @lbergmann

<a name="0.3.0"></a>
## [0.3.0] - 2022-06-10

### Added
* Added configurable initial joint velocities.
* `MujocoEnv` uses namespacing coherent with ROS namespaces.
* Added `lastStageCallback` to define behavior for plugins at the very end of a simulation step.

### Fixed
* Moved mjENABLED_ros/mjDISABLED_ros definitions to non-static context (#2).
* Removed `collision_function` typedef (duplicate of `mjfCollision`).

### Removed
* Removed `runContactFilterCbs` since setting a contact filter callback fully overrides default behavior.


Contributors: @DavidPL1, @balandbal

<a name="0.2.0"></a>
## 0.2.0 - 2022-05-30

### Added
* Plugin loading.
* Service call to shutdown.
* Service call to pause/unpause.
* Headless mode and number of steps to run before termination as optional argument.
* Safety wrappers for plugin loading and execution.
* Exporting mujoco_ros as library for external usage.
* Introduced parallel environments.

### Changed
* Now using boost::shared_ptr for mjModel and mjData instances.
* Added licensing information.
* Added build instructions.
* How plugins are retrieved from the parameter server.

### Fixed
* Several bugs related to ROS time publishing.
* Disabled backward stepping (impractical with ROS time).
* Setting initial joint positions.

Contributors: @DavidPL1

[0.4.0]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/0.3.1...0.4.0
[0.3.1]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/0.3.0...0.3.1
[0.3.0]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/0.2.0...0.3.0
[0.2.0]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/6c8bbe2...0.2.0
