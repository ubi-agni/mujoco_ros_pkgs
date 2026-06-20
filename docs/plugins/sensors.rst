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

In training mode, the plugin also publishes a ground-truth topic with ``_GT`` appended to the sensor name.
In ``eval_mode``, ground-truth topics are not created.

Noise Models
------------

Noise is registered through ``/sensors/register_noise_models`` with ``mujoco_ros_msgs/RegisterSensorNoiseModels``.
Each ``SensorNoiseModel`` names a sensor and provides ``mean`` and ``std`` arrays.
For quaternion sensors, noise is expressed as Euler-angle noise and then converted to a quaternion.
