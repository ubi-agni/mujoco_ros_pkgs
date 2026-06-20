``hybrid-devel``
================

``hybrid-devel`` is the active 1.0.0 release line. It supports ROS 1 and ROS 2 from a shared MuJoCo core and replaces the older split ROS 1 / ROS 2 package layout.

Single Source of Truth
----------------------

The guiding philosophy is to keep simulation behavior in one implementation and isolate ROS-version differences at the boundary. Physics, loading, rendering, threading, camera setup, and most environment logic should remain shared. ROS-specific code lives in adapter layers such as ``ros_one`` and ``ros_two``.

This keeps fixes to MuJoCo behavior from being duplicated across ROS versions and makes branch drift visible: if code changes physics semantics, it should normally change the shared core, not a ROS-specific copy.

ROS 1 Parity
------------

The ROS 1 side of ``hybrid-devel`` preserves the user-facing feature set from ``noetic-devel`` for launch behavior, services/actions, camera streams, dynamic reconfigure, plugin loading, ros_control, sensors, laser, mocap, and practical migration paths for existing ROS 1 users.

Architecture
------------

The branch generates a ROS-version header from ``ROS_VERSION`` and uses it to select ROS 1 or ROS 2 code paths at compile time. Public headers are renamed to ``.hpp`` and include ROS-specific APIs through:

* ``mujoco_ros/ros_one/ros_api.hpp``
* ``mujoco_ros/ros_two/ros_api.hpp``
* ``mujoco_ros/ros_one/plugin_utils.hpp``
* ``mujoco_ros/ros_two/plugin_utils.hpp``

Source files are split similarly:

* shared files such as ``mujoco_env.cpp``, ``physics.cpp``, ``loading.cpp``,   ``interface.cpp``, ``offscreen_camera.cpp``, and ``viewer.cpp``
* ROS 1 adapters under ``src/ros_one``
* ROS 2 adapters under ``src/ros_two``

Launch Differences
------------------

The ROS 1 server launch file lives directly at ``mujoco_ros/launch/launch_server.launch`` so source/devel-space includes match the install-space layout.
ROS 2 launch files live under ``mujoco_ros/launch/ros2`` and use the ``.launch.xml`` suffix to avoid ROS 1 recursive launch-file ambiguity.
The ROS 2 launch file mirrors the ROS 1 options but uses ROS 2 launch substitutions, ROS 2 logging arguments, and ``<param from=...>`` for YAML input.

ROS 2 Control
-------------

``hybrid-devel`` uses ``mujoco_ros_control`` as the active hybrid control package.
The ROS 1 side keeps the ``ros_control`` plugin and ``DefaultRobotHWSim`` path.
The ROS 2 side provides the MuJoCo control plugin and ``MujocoRosSystem`` in the same package.
The older ``mujoco_ros2_control`` and ``mujoco_ros2_control_system`` packages are not the intended long-term active control path.

Plugin Porting
--------------

The bundled sensors, laser, mocap, and control plugins have hybrid implementations, Python bindings, example launch files, and package-level tests.
ROS 1 devel/install builds, ROS 2 builds, and CI jobs are in place.
