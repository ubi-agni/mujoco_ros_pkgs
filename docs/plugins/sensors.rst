MuJoCo Sensors Plugin
=====================

``mujoco_ros_sensors`` publishes native MuJoCo sensor readings as ROS messages.
It does not handle MuJoCo user sensors; implement custom sensors as separate MuJoCo ROS plugins with their own message definitions.

Configuration
-------------

.. code-block:: yaml

   MujocoPlugins:
     - type: mujoco_ros_sensors/MujocoRosSensorsPlugin

Topics
------

Each supported MuJoCo sensor is published on a topic named after the sensor.
Depending on sensor type, the plugin uses:

* ``geometry_msgs/Vector3Stamped``
* ``geometry_msgs/PointStamped``
* ``geometry_msgs/QuaternionStamped``
* ``mujoco_ros_msgs/ScalarStamped``
* ``mujoco_ros_msgs/CamProjectionStamped``
* ``mujoco_ros_msgs/GeomFromToStamped``

In training mode, the plugin also publishes a ground-truth topic with ``_GT`` appended to the sensor name.
In ``eval_mode``, ground-truth topics are not created.

Unsupported MuJoCo sensor types are skipped with a warning at load; they are never published.

ROS publish is gated per channel: a topic is only written when it has at least one
subscriber. Optional per-sensor ``update_rate`` (Hz, sim time) throttles that ROS
publish further. Unset or ``<= 0`` publishes every step when subscribed.
In-process Python ``plugin.sensors[<name>].data`` / ``.gt_data`` still update every
step and do not wait for subscribers.

ROS 1: set ``update_rate`` under the plugin entry keyed by sensor name
(``MujocoPlugins[].<sensor_name>.update_rate``).
ROS 2: set ``MujocoPlugins.<plugin_name>.<sensor_name>.update_rate``.
See ``mujoco_ros_sensors/config/example_sensors.yaml`` and ``example_sensors.ros2.yaml``.

Compound sensors
----------------

Compounds group several native MJCF sensors into one ROS message.
Configure them under the sensors plugin as ``compound_sensors``.
Each compound has a ``name``, a ``type``, and required **role** keys that name MJCF sensors.

Supported types and roles:

* ``Wrench`` → ``geometry_msgs/WrenchStamped`` — roles ``force``, ``torque``
* ``Twist`` → ``geometry_msgs/TwistStamped`` — roles ``linear``, ``angular`` (same ``frame_id``)
* ``Accel`` → ``geometry_msgs/AccelStamped`` — roles ``linear``, ``angular`` (same ``frame_id``)
* ``Pose`` → ``geometry_msgs/PoseStamped`` — roles ``position``, ``orientation`` (same body and reference)
* ``IMU`` → ``sensor_msgs/Imu`` — roles ``linear_acceleration``, ``angular_velocity``, ``orientation`` (one body; IMU covariances stay unknown)
* ``Odometry`` → ``nav_msgs/Odometry`` — roles ``position``, ``orientation``, ``linear``, ``angular`` (one body; header frame from position, ``child_frame_id`` from twist roles)

A sensor claimed by a role is no longer published or exposed on its own.
The compound topic is ``<name>`` (and ``<name>_GT`` outside ``eval_mode``).

Optional ``update_rate`` (Hz, sim time) throttles the ROS publish only.
Unset or ``<= 0`` publishes every step.
In-process Python ``plugin.sensors[<name>].data`` / ``.gt_data`` still update every step.

ROS 1 uses a list of maps under the plugin entry.
ROS 2 uses a string array of names plus flat ``compound_sensors.<name>.<key>`` parameters (same pattern as the laser plugin).
See ``mujoco_ros_sensors/config/example_compound_sensors.yaml`` and ``example_compound_sensors.ros2.yaml``.

Invalid compound config fails plugin ``Load`` loudly (unknown type/role, wrong MuJoCo type, frame/body mismatches, name collisions).

Noise Models
------------

Noise is registered through ``/sensors/register_noise_models`` with ``mujoco_ros_msgs/RegisterSensorNoiseModels``.
Each ``SensorNoiseModel`` names a sensor and provides ``mean`` and ``std`` arrays.
For quaternion sensors, noise is expressed as Euler-angle noise and then converted to a quaternion.

For compounds, register role noise on the dotted name ``<compound>.<role>`` (for example ``ee_wrench.force``), not on the demoted part sensor name.

Python access
-------------

Import ``mujoco_ros_sensors`` so the plugin binding registers, then use ``bind_plugin`` on the loaded plugin.
Public streams appear as ``plugin.sensors[<name>]`` with ``.config``, ``.data``, and ``.gt_data`` (ground truth raises in ``eval_mode``).
Claimed compound parts are absent from that map.
