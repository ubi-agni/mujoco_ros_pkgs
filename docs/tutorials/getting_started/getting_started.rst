Installation and First Launch
=============================

Supported Branches
------------------

``noetic-devel`` is the legacy ROS 1 reference branch.
It targets Ubuntu Focal with ROS Noetic.

``hybrid-devel`` is the new release line.
It keeps the MuJoCo core in one implementation and moves ROS-specific behavior into small adapter layers for ROS 1 and ROS 2.
See :doc:`/branches/hybrid_devel` for architecture notes.

Python bindings are integrated into ``hybrid-devel``.
They provide a Python ``MujocoEnv`` wrapper around the C++ environment and are documented in
:doc:`/python_bindings/python_bindings`.

Install Dependencies
--------------------

Install ROS and the catkin command-line tools:

.. code-block:: bash

   sudo rosdep init
   rosdep update
   sudo apt update
   sudo apt install python3-catkin-tools

Create a workspace and clone the repository:

.. code-block:: bash

   mkdir -p ~/mujoco_ws/src
   cd ~/mujoco_ws/src
   git clone https://github.com/ubi-agni/mujoco_ros_pkgs -b hybrid-devel

MuJoCo ROS currently expects MuJoCo 3.3.5. Make sure the headers and shared library come from the same MuJoCo installation.

If MuJoCo was installed from an archive, expose it to CMake and the dynamic linker:

.. code-block:: bash

   export MUJOCO_DIR=$HOME/.mujoco/mujoco-3.3.5
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MUJOCO_DIR/lib
   export LIBRARY_PATH=$LIBRARY_PATH:$MUJOCO_DIR/lib

If MuJoCo was built from source and installed into a prefix known to the workspace, these variables are usually not needed.

.. warning::

   On ``hybrid-devel``, building MuJoCo from source can require pinning MuJoCo's tinyxml2 dependency to avoid a runtime incompatibility with rospack-linked tinyxml2. The branch README records the required CMake option for that branch.

Build
-----

Install package dependencies:

.. code-block:: bash

   cd ~/mujoco_ws
   rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

Build with catkin:

.. code-block:: bash

   catkin init
   catkin config --install
   catkin build

Or build with colcon:

.. code-block:: bash

   colcon build

Do not run ``catkin init`` in a workspace that you intend to manage only with colcon.

Launch the Example Server
-------------------------

Source the workspace and launch the default pendulum world:

.. code-block:: bash

   source ~/mujoco_ws/install/setup.bash
   roslaunch mujoco_ros launch_server.launch use_sim_time:=true

The launch file requires ``use_sim_time`` explicitly.
Use ``true`` for a fully simulated setup.
Use ``false`` only when another component owns ROS time, such as a mixed real/simulated system.

For a headless server with no GUI or offscreen rendering:

.. code-block:: bash

   roslaunch mujoco_ros launch_server.launch use_sim_time:=true no_render:=true

.. warning::

   Resetting or reloading a simulation resets simulated time to zero. Until https://github.com/ros/actionlib/pull/203 is available in your ROS installation, action servers can ignore goals until simulated time catches up to the previous value. Build the patched actionlib branch in workspaces that depend heavily on actions.

Next Step
---------

Next, load your own model with
:doc:`/tutorials/load_custom_model/load_custom_model`.
