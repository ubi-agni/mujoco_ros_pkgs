Laser Plugin
============

``mujoco_ros_laser`` provides a CPU-based ``sensor_msgs/LaserScan`` publisher using MuJoCo's ray-collision functions.
Rays are cast serially in one ``mj_multiRay`` call per scan. The plugin does not use ``mujoco_threads`` or any worker pool for ray casting, because MuJoCo 3.10 no longer exposes a public task API for plugins.

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

Scan casting is single-threaded regardless of ``mujoco_threads``; that option only sizes MuJoCo's internal engine threadpool, which exists only for values above 1.
