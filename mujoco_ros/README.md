# MuJoCo Ros

This is a ROS package that wraps the [MuJoCo physics engine](https://mujoco.org/) into a ros package.
It is an extension of the MuJoCo [simulate](https://github.com/deepmind/mujoco/blob/2.1.1/sample/simulate.cc) program, with ROS integration and the possibility to load plugins via pluginlib.

As an example for extended functionality via plugin, take a look at [mujoco_ros_control](https://github.com/ubi-agni/mujoco_ros_pkgs/tree/noetic-devel/mujoco_ros_control), [mujoco_screw_plugin](https://github.com/ubi-agni/mujoco_screw_plugin), [mujoco_contact_surfaces](https://github.com/ubi-agni/mujoco_contact_surfaces) or [mujoco_ros_sensors](https://github.com/ubi-agni/mujoco_ros_pkgs/tree/noetic-devel/mujoco_ros_sensors).

For an example of the Franka Emika Panda using ROS control in MuJoCo, see [franka_ros_mujoco](https://github.com/ubi-agni/franka_ros_mujoco).

## Plugins
Plugins provide an easy way of including new functionality into _mujoco\_ros_. The lifecycle of a Plugin is as follows:
1. Once an _mjData_ instance is created (and stored in a __MujocoEnv_), each configured plugin is instanciated and a pointer to it is stored in the responsible _MujocoEnv_. Directly after creation its `init` function is called, which provides the plugin instance with its specific configuration, if one was provided. This function should take care of generic, non-instance specific configuration.
Afterwards the `load` function is called, which makes _mjModel_ and _mjData_ available to the plugin. This function should handle the model/instance specific setup of the plugin.

plugins have different callback functions defined in their base class, users should override the callback functions they intend to use. These are the `controlCallback`, `passiveCallback`, and `renderCallback` functions. The first two are automatically configured to run when the `mjcb_control` and the `mjcb_passive` functions are called by MuJoCo. The latter allows plugins to add visualisation objects before a scene is rendered.

> **Warning**
> A plugin should __never__ override the base mujoco callback functions! _mujoco\_ros_ uses them to resolve the appropriate environment and run their list of registered plugin instance callbacks. Instance agnostic plugins should be realized as singletons that return a reference to the singleton in their constructor.

___

## Initial Joint States
Initial joint states, i.e. positions and/or velocities, can be set using ros parameters. The joint configuration is fetched and applied when the world model is loaded, reset or reloaded.
For each joint values for all degrees of freedom (depending on the joint type) need to be provided. To ensure that the ros parameter server correctly interprets the data type, the values should be explicitly given as *one* string. This is especially important when providing single values for hinge or slide joints, as ros will otherwise interpret them as double or int and `mujoco_ros` won`t be able to read them (and in most cases will be unable to detect that something went wrong and simply ignore the value).
The following sample config shows an example how to correctly provide values for each joint type:
```yaml
# Set positions
initial_joint_positions:
  joint_map:
    joint1 : "-1.57"                                #Hinge/Slide joint: single axis value
    ball_joint : "1.0 0 0 0"                        #Ball joint: quaternion (w x y z) relative to parent orientation
    free_joint: "2.0 1.0 1.06 0.0 0.707 0.0 0.707"  #Free joint: Position (x y z) followed by a quaternion (w x y z) in world coordinates

# Set velocities
initial_joint_velocities:
  joint_map:
    joint1 : "-1.57"                     #Hinge/Slide joint: single axis value
    ball_joint : "0 0 20.0"              #Ball joint: r p y
    free_joint : "1.0 2.0 3.0 10 20 30"  #Free joint: x y z r p y
```

## Sensors

[mujoco_ros_sensors](https://github.com/ubi-agni/mujoco_ros_pkgs/tree/noetic-devel/mujoco_ros_sensors) supports most of the native MuJoCo sensors to be converted into and published as ROS messages. Note that this requires to configure the plugin to be loaded.

### Camera Streams

Camera streams are a special case, because their implementation can not easily be separated from the core `mujoco_ros` code. Thus camera streams do not require the sensors plugin to be loaded.

To make a camera stream available, the respective camera has to be defined in the model file with a specific name, e.g. `workspace_cam`.
By default an RGB-stream with a frequency of 15 Hz will be created for each camera. These defaults can be overwritten by specifying respective values as ROS parameters under "cam_config/CAMERA_NAME/".
The frequency can be specified as floating point value in Hz (note that the maximum frequency is bound to the step size of the simulation).
As stream types RGB (=`1`), DEPTH (=`2`), and SEGMENTED (=`4`) are available.
SEGMENTED provides an image where each visible geom is colored either randomly (`use_segid: false`) or colored by segid (`use_segid: true` and used by default).
A camera can also provide multiple streams simultaneously, e.g. stream type `7` would provide all three stream types on separate [image_transport](http://wiki.ros.org/image_transport) topics (`cameras/CAMERA_NAME/{rgb,depth,segmented}`). See below for an example configuration:
```yaml
cam_config:
  workspace_cam:
    stream_type: 3 # RGB and DEPTH
    frequency: 10
  workspace_cam2:
    stream_type: 5 # RGB and SEGMENTED
    use_segid: false # visible geoms colored randomly
```

As long as the image transport topics have no subscribers, the offscreen camera images are not rendered. This way no computational overhead is caused until the images are requested explicitly.

___

# Build Instructions
1. Make sure MuJoCo is installed (the current build uses version 2.2.2) and runs on your machine.
2. Create a new ROS workspace or include this repository into an existing workspace.
3. Before building, make sure that your compiler knows where to find the MuJoCo library, e.g. by running
```bash
export MUJOCO_DIR=PATH/TO/MUJOCO/DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_DIR/lib
export LIBRARY_PATH=$LIBRARY_PATH:$MUJOCO_DIR/lib
```
where `PATH/TO/MUJOCO/DIR` is `~/.mujoco/mujoco222` if you used the recommended location to install mujoco (if downloaded as tarball). If you built MuJoCo from source and the install path is known to catkin, you can skip this step.

4. Build with `catkin_build` or `catkin b`.
5. Source your workspace and try `roslaunch mujoco_ros demo.launch` to test if it runs.

> **Warning**
> To prevent action servers ignoring actions for a limited time after resetting the simulation, until https://github.com/ros/actionlib/pull/203 is merged, you need to build the PR branch and any packages implementing action servers (like MoveIt) yourself. Additionally, for the Franka Emika Panda robot, until https://github.com/frankaemika/franka_ros/pull/292 is merged, you need to build the PR branch yourself, otherwise controlling the robot after time resets is buggy.

# Licensing

This work is licensed under the BSD 3-Clause License (see LICENSE).
It is built on top of MuJoCo 2.2.2, which was released under an Apache 2.0 License. For the original MuJoCo and further third party licenses, see [THIRD_PARTY_NOTICES](./THIRD_PARTY_NOTICES).
