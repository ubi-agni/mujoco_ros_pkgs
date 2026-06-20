ROS Control Plugin
==================

``mujoco_ros_control`` integrates ROS 1 ``ros_control`` and ROS 2 ``ros2_control`` with MuJoCo ROS.
It is based on the same idea as ``gazebo_ros_control`` / ``gz_ros2_control``: a MuJoCo plugin creates a controller manager and a simulated hardware interface for the robot.

Configuration
-------------

Load the plugin through ``MujocoPlugins``:

.. code-block:: yaml

   MujocoPlugins:
     - type: mujoco_ros_control/MujocoRosControlPlugin
       robot_namespace: my_robot_ns
       hardware:
         type: mujoco_ros_control/DefaultRobotHWSim
         control_period: 0.001
         robot_description: robot_description
         ignore_actuators: false
         eStopTopic: emergency_stop

``hardware.type`` and ``hardware.control_period`` are required.
``robot_namespace`` is optional and scopes the controller manager and robot node handle.
``robot_description`` defaults to ``robot_description``.
``eStopTopic`` optionally subscribes to ``std_msgs/Bool`` and stops commands while active.

For ROS 2, load ``mujoco_ros_control/MujocoRosControlPlugin`` through ``MujocoPlugins`` and use ``mujoco_ros_control/MujocoRosSystem`` in the ``<ros2_control>`` hardware block.
``ignore_actuators`` can be set as a hardware parameter:

.. code-block:: xml

   <ros2_control name="MujocoRosSystem" type="system">
     <hardware>
       <plugin>mujoco_ros_control/MujocoRosSystem</plugin>
       <param name="ignore_actuators">false</param>
     </hardware>
   </ros2_control>

Robot Model Requirements
------------------------

The plugin reads the robot description from the parameter server and parses transmissions.
The default hardware simulation supports effort, position, and velocity joint command interfaces.
Prefer fully qualified hardware interface names such as ``hardware_interface/EffortJointInterface`` in transmission tags.

The control period should be greater than or equal to the MuJoCo timestep.
The plugin warns when it is shorter than the simulation timestep.

Actuator Mapping
----------------

ROS controllers always own the command value.
The hardware interface decides how that command is applied to MuJoCo.

The current supported actuator mapping uses suffix conventions:

* ``<joint>_act_eff`` for effort commands.
* ``<joint>_act_pos`` for position commands.
* ``<joint>_act_vel`` for velocity commands.

If a matching MuJoCo actuator exists and ``ignore_actuators`` is false, commands are written to ``mjData.ctrl`` for that actuator.
This lets MuJoCo's actuator model apply the actual generalized forces.

If the matching actuator does not exist, or if ``ignore_actuators`` is true, the hardware interface falls back to generalized force application through ``mjData.qfrc_applied``.
Effort commands are applied directly.
Position and velocity commands require configured gains so they can be converted to efforts.
In ROS 1, use the existing ``mujoco_ros_control/pid_gains/<joint>`` parameters.
In ROS 2, provide ``kp`` for position fallback and ``kv`` for velocity fallback as joint parameters in the ``ros2_control`` description.
ROS 2 fallback efforts can be limited with an optional per-joint ``effort_limit`` parameter.

Directly setting ``qpos`` or ``qvel`` is not part of the default behavior.
A missing position or velocity actuator without fallback gains is treated as a configuration error.

Example
-------

The package ships a four-pendulum example.
The first pendulum has no MuJoCo actuator and demonstrates effort fallback through ``qfrc_applied``.
The second uses an effort motor, the third uses a velocity actuator, and the fourth uses a position actuator.

After launching it in ROS 1, try:

.. code-block:: bash

   rostopic pub /fallback_effort_controller/command std_msgs/Float64 "data: 0.2" -r 20
   rostopic pub /motor_effort_controller/command std_msgs/Float64 "data: 0.2" -r 20
   rostopic pub /velocity_controller/command std_msgs/Float64 "data: 0.5" -r 20
   rostopic pub /position_controller/command std_msgs/Float64 "data: 0.5" -r 20

For ROS 2:

.. code-block:: bash

   ros2 topic pub /fallback_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.2]}" -r 20
   ros2 topic pub /motor_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.2]}" -r 20
   ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5]}" -r 20
   ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5]}" -r 20

The same MuJoCo model can also be launched with actuator routing disabled:

.. code-block:: bash

   roslaunch mujoco_ros_control mujoco_ros_control_ignore_actuators.launch
   ros2 launch mujoco_ros_control mujoco_ros_control_ignore_actuators.launch.py

In this mode, the effort, velocity, and position command interfaces all use the generalized-force fallback path.
The position and velocity examples include the required fallback gains in their ROS 1 YAML / ROS 2 control description.

Branch Status
-------------

This package is hybrid in ``hybrid-devel``.
The ROS 1 and ROS 2 implementations use different control framework APIs, but share the same actuator mapping semantics.
