Loading a Custom MuJoCo Model
=============================

Use the ``modelfile`` launch argument to start the server with your own MuJoCo XML or MJB file:

.. code-block:: bash

   roslaunch mujoco_ros launch_server.launch \
     use_sim_time:=true \
     modelfile:=/absolute/path/to/model.xml

The default model is ``$(find mujoco_ros)/assets/pendulum_world.xml``.

Load Plugin Configuration
-------------------------

Plugin configuration is loaded from YAML before the server node starts.
Pass a file through ``mujoco_plugin_config``:

.. code-block:: bash

   roslaunch mujoco_ros launch_server.launch \
     use_sim_time:=true \
     modelfile:=/absolute/path/to/model.xml \
     mujoco_plugin_config:=/absolute/path/to/plugins.yaml

The file must define a top-level ``MujocoPlugins`` list:

.. code-block:: yaml

   MujocoPlugins:
     - type: mujoco_ros_sensors/MujocoRosSensorsPlugin

Load Initial Joint States
-------------------------

Initial joint positions and velocities are loaded from ``initial_joint_positions/joint_map`` and ``initial_joint_velocities/joint_map`` when the model is loaded, reset, or reloaded.
Values should be strings so the ROS parameter server preserves the intended vector shape:

.. code-block:: yaml

   initial_joint_positions:
     joint_map:
       joint1: "-1.57"
       ball_joint: "1.0 0 0 0"
       free_joint: "2.0 1.0 1.06 0.0 0.707 0.0 0.707"

   initial_joint_velocities:
     joint_map:
       joint1: "-1.57"
       ball_joint: "0 0 20.0"
       free_joint: "1.0 2.0 3.0 10 20 30"

For positions, hinge and slide joints use one value, ball joints use a quaternion ``w x y z`` relative to the parent, and free joints use position ``x y z`` followed by quaternion ``w x y z`` in world coordinates.

For velocities, hinge and slide joints use one value, ball joints use angular velocity ``r p y``, and free joints use linear velocity ``x y z`` followed by angular velocity ``r p y``.

Load XML from the Parameter Server
----------------------------------

Set ``wait_for_xml:=true`` when another process will provide the model XML on the parameter server instead of passing a file path.
In normal file-based workflows, leave this disabled.
