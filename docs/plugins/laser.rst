Laser Plugin
============

``mujoco_ros_laser`` provides a CPU-based ``sensor_msgs/LaserScan`` publisher using MuJoCo's ray-collision functions.
It uses MuJoCo's thread pool when the server is configured with more than one MuJoCo worker thread.
This server threading option is deprecated and values greater than ``1`` can increase CPU usage because MuJoCo worker threads busy-wait.

This plugin is raycast-based. It does not include the older ROS 2 sensors rangefinder/lidar aggregation behavior; that should live in another plugin if it is needed again.

ROS 1 Configuration
-------------------

.. code-block:: yaml

   MujocoPlugins:
     - type: mujoco_ros_laser/LaserPlugin
       sensors:
         - site_attached: laser_site
           name: scan
           frame_id: scan
           visualize: true
           update_rate: 10.0
           min_range: 0.1
           max_range: 30.0
           range_resolution: 0.01
           angular_resolution: 0.02
           min_angle: -1.57
           max_angle: 1.57

ROS 2 Configuration
-------------------

.. code-block:: yaml

   MujocoPlugins:
     names:
       - mujoco_ros_laser
     mujoco_ros_laser:
       type: mujoco_ros_laser/LaserPlugin
       sensors:
         - scan
       scan:
         site_attached: laser_site
         frame_id: scan
         visualize: true
         update_rate: 10.0
         min_range: 0.1
         max_range: 30.0
         range_resolution: 0.01
         angular_resolution: 0.02
         min_angle: -1.57
         max_angle: 1.57

Each configured laser attaches to a MuJoCo site. The scan topic is named from the laser configuration/model setup and the frame defaults to the parent body unless overridden.

Use the deprecated ``mujoco_threads`` option on the server launch file to allow parallel ray work for larger scans.
