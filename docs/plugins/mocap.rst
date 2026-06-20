Mocap Plugin
============

``mujoco_ros_mocap_plugin`` controls MuJoCo mocap bodies from ROS.
It can be used directly for tracked objects or together with a MuJoCo weld constraint to drive a non-mocap body with tunable softness.

Configuration
-------------

.. code-block:: yaml

   MujocoPlugins:
     - type: mujoco_ros_mocap/MocapPlugin

Interfaces
----------

``mocap_poses``
  Subscribes to ``mujoco_ros_msgs/MocapState``.

``set_mocap_state``
  Service using ``mujoco_ros_msgs/SetMocapState``.

Pose headers must either be empty or use the ``world`` frame.
The plugin rejects unknown body names and names that do not correspond to MuJoCo mocap bodies.
