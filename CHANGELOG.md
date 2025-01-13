<a name="unreleased"></a>
## Unreleased

### Added
* Now it's possible to use MuJoCos intenal threadpool to speed up simulation. The number ob threads can be set with either `mujoco_threads` in the server launchfile or directly with `num_mj_threads` for the server node.
If a number above 1 is used, multithreading is enabled. MuJoCo will then use the threadpool for parallel computations in the engine itself, but also plugins may use the threadpool to dispatch tasks (via (mjTask)[https://mujoco.readthedocs.io/en/3.2.0/APIreference/APItypes.html#mjtask] and (mju_threadPoolEnqueue)[https://mujoco.readthedocs.io/en/3.2.0/APIreference/APIfunctions.html#mju-threadpoolenqueue]).
The default is `min(#available_threads - 1, 4)`.
* Added profiling of plugin load and callback execution times. For each callback an EMA with sensitivity of 1000 steps is computed. In case a plugin has lower frequency than simulation step size, the callback should set `skip_ema_ = true` when skipping computations.
Loading and reset times are reported in the server debug log. All plugin stats can be retrieved by the `get_plugin_stats` service call.
* Added ros laser plugin.
* Added dynamic reconfigure for all MuJoCo settings for fast model testing.

### Fixed
* Added missing call to render callbacks in viewer. While the callbacks were still being run for offscreen rendering, the viewer did not render additional geoms added by plugins.
* *mujoco_ros_control*: fixed sometimes using wrong joint id in default hardware interface (would only be correct, if the joints appear first and in the same order in the compiled MuJoCo model).
* *mujoco_ros_sensors*: now skipping user sensors, as they should be handled in separate, dedicated plugins.
* When loading takes more than 0.25 seconds the simulation is no longer automatically paused.
* Fixed fetching of body quaternion in `get_body_state` service.
* *tests*: PendulumEnvFixture now makes sure `mj_forward` has been run at least once. This ensures the data object is populated with correct initial positions and velocities.
* Re-added services for getting and setting gravity, that somehow vanished.
* Fixed flaky tests that did not consider control callbacks being called in paused mode, too (#40).
* Fixed bug that would not allow breaking out of *as fast as possible* stepping in headless mode without shutting down the simulation.
* Fixed occasional segfault when offscreen context was freed on shutdown.
* Fixed segmented image never being rendered/published.
* Fixed thread synchronization between MujocoEnv and Viewer.

### Changed
* Moved `mujoco_ros::Viewer::Clock` definition to `mujoco_ros::Clock` (into common_types.h).
* Increased test coverage of `mujoco_ros_sensors` plugin.
* Split monolithic ros interface tests into more individual tests.
* Added sleeping at least until the next lowerbound GUI refresh when paused to reduce cpu load.
* deprecated `no_x` launchparameter in favor of using `no_render`, as offscreen rendering now is also available without X.
* Optimized camera render configurations where RGB, Segmented and Depth streams are active, but only Segmented and Depth are subscribed. Previously, this would result in two separate low-level render calls, now it's done in one.

Contributors: @DavidPL1

<a name="0.9.0"></a>
## [0.9.0] - 2024-07-31

### Added
* Mocap plugin to programmatically control mocap bodies.
* Services to alter equality constraints. Note that by MuJoCo's design no new constraints can be added, though.
* The Server node now gives a more verbose error message on failure due to mismatched header and library versions.
* Service to get basic simulation state info (loaded model path, model valid, load counter, loading state, paused, pending manual steps, measured rt factor, and rt factor setting).
* Service to set the real-time factor (applies the closest available factor found in the settings).

### Fixed
* Repaired SIGINT handler callback. `C-c` in the roslaunch terminal now shuts down the MuJoCo ROS node instead of escalating to SIGTERM.
* Added actionlib to the list of mujoco_ros' dependencies.
* Updated CI actions (#33).
* Fixed linking order for mujoco_ros_mocap tests (#33).
* Fixed glx BadAccess error on exit when running with GUI.
* Fixed bug where stepping multiple times to get up to speed with the desired realtime factor would only happen if a viewer was attached.
* Enabling and disabling model flags in the GUI's 'Physics' tab is now applied to the attached environment.

### Changed
* replaced `boost::shared_ptr` with `std::shared_ptr` or `std::unique_ptr` wherever possible (ROS 1 fast intra-process message-passing requires boost::shared_ptr).
* replaced `shared_ptr` with `unique_ptr` wherever possible.
* replaced smart pointer constructor initialization with `make_shared` or `make_unique` wherever possible.
* Updated to MuJoCo library version 3.2.0.
* * VFS now only saves models provided as strings. Model loading and caching might change once the `mjSpec` API is more stable and better documented.
* * Added [jointactuatorfrc](https://mujoco.readthedocs.io/en/2.3.7/XMLreference.html#sensor-jointactuatorfrc) support in `mujoco_ros_sensors`.
* Updated to clang-tidy-14 (default on jammy).
* Added catkin_lint to pre-commit (i.e. format workflow).
* Introduced helper function to validate service call permissions in evaluation mode.

Contributors: @DavidPL1, @acodeodyssey

<a name="0.8.0"></a>
## [0.8.0] - 2023-10-23

### Added
* Manual steps now run as fast as possbile. I.e., if a viewer is connected, stepping is interrupted to render the UI at 30Hz. (This now also applies to running with unbound real-time, which previously was interrupted at 30Hz regardless if any viewer was connected).
* Added GitHub Actions for building docker images, CI, and formatting based on MoveIt's configuration (#30).

### Fixed
* re-added setting realtime settings via ros param or in the mujoco model xml.
* Add missing install of mujoco_ros's `config` and `assets` directories (#28).
* Added missing example config for `mujoco_ros_sensors`.
* Fixed catkin lint errors (#31).
* Resolved clang-tidy warnings.

### Changed
* Reduced sensor noise std in tests to reduce wrongful fails due to too stochasticity.
* Updated toplevel README to be more informative.
* mujoco_ros_control & mujoco_ros_sensors have a Sanitize build type (#31).

> [!IMPORTANT]
> #### Breaking Changes
> * `MujocoPlugin::node_ptr_` now is a NodeHandle instead of a NodeHandlePtr.
> * MujocoPlugin callbacks now use raw pointers to mjModel and mjData instead of shared pointers in their callbacks (mjModel is even const, thus calls to functions like `mj_id2name` that expect a non-const mjModel pointer need a `const_cast<mjModel *>` to not throw compile errors).

Contributors: @DavidPL1, @LeroyR, @rhaschke

<a name="0.7.0"></a>
## [0.7.0] - 2023-08-15

### Added
* Updated to MuJoCo library version 2.3.6.
* Added a `step(num_steps=1, blocking=true)` public function.
* Unit tests for plugin loading and resetting and running callbacks correctly.
* Now automatically building with AVX instructions, if possible.
* CMake tooling borrowed from deepminds MuJoCo repo and https://github.com/osjacky430/ros_pkg_template/.

### Changed
* Moved from static namespace to object oriented application model.
* `get_load_requeststate` service topic changed to `get_load_request_state`.
* Renamed namespaces:
* * `mujoco_ros_sensors` -> `mujoco_ros::sensors`.
* * `mujoco_ros_control` -> `mujoco_ros::control`.
* * `MujocoSim` -> `mujoco_ros`.
* * `MujocoSim::detail` -> moved mostly to private access members, namespace is not necessary anymore.
* * `MujocoSim::jointName2id` -> `mujoco_ros::util::jointName2id`.
* Node launchfile arg changed from `visualize` to `headless` for consistency.
* Moved most contents of `MujocoSim::rendering::VisualStruct` into `mujoco_ros::MujocoEnv::offscreen`.
* Renamed `MujocoSim::rendering::CameraStream` -> `mujoco_ros::rendering::OffscreenCamera`.
* `env_id` members were removed from messages, since env_ids are no longer used.
* Added more compiler warnings (see [CompilerWarnings.cmake](./cmake/CompilerWarnings.cmake)).

Contributors: @DavidPL1

<a name="0.6.0"></a>
## [0.6.0] - 2023-06-30

### Added
* Signal handling to gracefully exit on SIGINT and SIGTERM.
* **docs**: Contribution guideline along with commit template.
* Service call to load initial joint states from parameter server.

### Changed
* Set GUI default to show info overlay and hide UI1.

### Fixed
* Fixed publish rate of camera images (#22).
* Now publishing clock on reload.
* Fixed memory leak caused by missing deallocation of opengl contexts on reload.
* Cameras now publish their first image at timestep 0 instead of waiting for `t = 1/pub_frequency`.

### Refactored
* Deprecated `getEnv`, `getEnvById`, and `unregisterEnv` functions.

Contributors: @DavidPL1, @lbergmann

<a name="0.5.0"></a>
## [0.5.0] - 2023-03-24

### Added
* Reload service call to either reload current model or supply a new model to load.
* Camera parameters are now available via camera info topics.
* Camera frames are now available in the TF trees.

### Changed
* Updated UI functionalities to match simulate version 2.3.3.
* Removed parallel env mode
* Bump MuJoCo required version to 2.3.3

### Fixed
* Stepping action server is now sensitive to preemts caused by reload/reset/shutdown and will inform clients in time instead of just severing the connection.
* Missing roll back of collision functions to default on reload caused calling unloaded code.
* An invalid model will not result in trying to reload indefinitely anymore.

Contributors: @DavidPL1, @fpatzelt

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

[unreleased]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/0.9.0...HEAD
[0.9.0]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/0.8.0...0.9.0
[0.8.0]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/0.7.0...0.8.0
[0.7.0]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/0.6.0...0.7.0
[0.6.0]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/0.5.0...0.6.0
[0.5.0]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/0.4.0...0.5.0
[0.4.0]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/0.3.1...0.4.0
[0.3.1]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/0.3.0...0.3.1
[0.3.0]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/0.2.0...0.3.0
[0.2.0]: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/6c8bbe2...0.2.0
