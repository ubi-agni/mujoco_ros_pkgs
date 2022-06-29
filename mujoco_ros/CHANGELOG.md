# Unreleased

## Changes
1. Better setup and cleanup in init function. This enables running the application multiple times (consecutively) when used as library:
  - exitrequest is set to 0 on start.
  - if ros time is not zero, the simulation is initialized with the current ros time as start time.
  - run i.e. the pause flag is now explicitly set according to the unpause ros param.
  - The services are explicitly unregistered on shutdown and the static services list is cleared.
2. Async spinner is now manually shutdown and ros::shutdown is called on server node termination.
3. Added unit tests for `mujoco_ros` single environment library functions.

## Fixes
1. Fixed step counter ignoring multiple steps during synchronization if desynchronized.
2. Fixed wrong declaration of publishSimTime in header.



# 3.0.0

## Changes
1. Changed how `MujocoEnv` is initialized/loaded/reloaded/resetted.

The `mujoco\_server` node will search for a `ns` (robot namespace) parameter, which defaults to "/", i.e. ROS root (no sub-namespace). The single `MujocoEnv` receives this namespace to initialize a ros `NodeHandle` in its assigned namespace. By passing a pointer of the handle to its plugins, the plugins can fetch the namespace from the handle or simply use the handle to register topics and services in the correct namespace.

Introduced new reset and reload function in `MujocoEnv` that trigger the respective functions of its members.
Plugin parsing and initialization moved to `MujocoEnv`.

Due to the changes in `MujocoPlugin`, plugins need to be rebuilt (hence the major version bump).

2. Added some more documentation.
3. Added `lastStageCallback` function to `MujocoPlugin` which can be used to define behavior which should be run at the end of a simulation step. Note that this function gets called by the `mujoco\_ros` simulation loop and not by the mujoco engine, hence it won't be called in between engine sub-steps.
4. Changed mjENABLED_ros/mjDISABLED_ros definitions to not rely on static context.

## Fixes
1. Removed `collision_function` typedef, which is a duplicate defenition of mjfCollision.
2. Removed `runContactFilterCbs` from `MujocoEnv`, since contact filter callbacks fully override the standard behavior. For contact filtering a similar approach to collision function registration should be used.
