# 3.0.0

## Changes
1. Changed how `MujocoEnv` is initialized/loaded/reloaded/resetted.

The `mujoco\_server` node will search for a `ns` (robot namespace) parameter, which defaults to "/", i.e. ROS root (no sub-namespace). The single `MujocoEnv` receives this namespace to initialize a ros `NodeHandle` in its assigned namespace. By passing a pointer of the handle to its plugins, the plugins can fetch the namespace from the handle or simply use the handle to register topics and services in the correct namespace.

Introduced new reset and reload function in `MujocoEnv` that trigger the respective functions of its members.
Plugin parsing and initialization moved to `MujocoEnv`.

Due to the changes in `MujocoPlugin`, plugins need to be rebuilt (hence the major version bump).

2. Added some more documentation.
3. Added `lastStageCallback` function to `MujocoPlugin` which can be used to define behavior which should be run at the end of a simulation step. Note that this function gets called by the `mujoco\_ros` simulation loop and not by the mujoco engine, hence it won't be called in between engine sub-steps.

## Fixes
1. Removed `collision_function` typedef, which is a duplicate defenition of mjfCollision.
2. Removed `runContactFilterCbs` from `MujocoEnv`, since contact filter callbacks fully override the standard behavior. For contact filtering a similar approach to collision function registration should be used.
