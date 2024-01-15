# MuJoCo Ros


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
