Server Launch Options
=====================

The ROS 1 server is launched with ``mujoco_ros/launch/launch_server.launch``:

.. code-block:: bash

   roslaunch mujoco_ros launch_server.launch use_sim_time:=true

Required Argument
-----------------

``use_sim_time``
  Required. Sets ``/use_sim_time``. Use ``true`` for fully simulated systems.
  Use ``false`` only when another clock source owns ROS time.

Runtime Arguments
-----------------

``ns``
  Optional namespace passed to the server.

``unpause``
  Starts physics immediately when ``true``. Defaults to ``false``.

``headless``
  Disables the interactive GUI when ``true``. Offscreen rendering can still be enabled. Defaults to ``false``.

``render_offscreen``
  Allocates offscreen camera rendering resources when ``true``. Camera streams are still lazy and render only while subscribed. Defaults to ``true``.

``no_render``
  Shorthand for ``headless:=true`` and ``render_offscreen:=false``. Use this for compute-only server runs. Defaults to ``false``.

``no_x``
  Deprecated compatibility option. It is treated as a no-render request when set and should be replaced by ``no_render``.

``modelfile``
  MuJoCo XML or MJB file to load. Defaults to ``$(find mujoco_ros)/assets/pendulum_world.xml``.

``wait_for_xml``
  Waits for model XML on the parameter server instead of immediately loading a file path. Defaults to ``false``.

``realtime``
  Desired real-time factor. Values in ``(0, 1]`` cap the simulation speed; ``-1`` runs as fast as possible. If unset, the model's MuJoCo realtime value is used.

``num_sim_steps``
  Automatically exits after this many MuJoCo simulation steps. ``-1`` disables the limit.

``mujoco_threads``
  Deprecated. Number of MuJoCo worker threads requested by the server. The launch file defaults to ``1``; the server falls back to its internal default when the parameter is not set. Values greater than ``1`` can increase CPU usage because MuJoCo worker threads busy-wait; see `google-deepmind/mujoco#2746 <https://github.com/google-deepmind/mujoco/pull/2746>`_. MuJoCo upstream is changing the public threading API to internal-only usage, so this behavior will change in the future and ``mujoco_ros`` will need its own threading API.

``mujoco_plugin_config``
  YAML file loaded before the node starts. Use it to define ``MujocoPlugins`` and plugin-specific configuration.

``initial_joint_states``
  YAML file loaded into the server namespace. Defaults to ``$(find mujoco_ros)/config/initial_joint_states.yaml``.

Access Control and Logging
--------------------------

``eval_mode``
  Enables evaluation restrictions. Mutating service calls require a matching ``admin_hash``, and the sensors plugin suppresses ground-truth topics.
  Defaults to ``false``.

``admin_hash``
  Hash required by protected service calls while ``eval_mode`` is enabled.

``verbose``
  Loads the configured ROS console file and enables more detailed logs.
  Defaults to ``false``.

``console_config_file``
  ROS console configuration used when ``verbose`` is true. Defaults to ``$(find mujoco_ros)/config/rosconsole.config``.

Developer Arguments
-------------------

``debug``
  Runs the server under ``gdb --args``.

``debug_server``
  Runs the server under ``gdbserver localhost:1234``. This overrides normal debug mode.

``valgrind``
  Runs the server under ``valgrind``. Do not combine this with ``debug``.

``valgrind_args``
  Extra arguments passed to valgrind.

``profile``
  Sets ``CPUPROFILE=/tmp/profile.out`` for profiler-enabled builds.

ROS 2 Launch Differences
------------------------

The ``hybrid-devel`` branch provides a ROS 2 launch file at ``mujoco_ros/launch/ros2/launch/launch_server.launch.xml``.
It mirrors the ROS 1 arguments but uses ROS 2 launch syntax, ``$(find-pkg-share ...)`` paths, ``exec=`` instead of ``type=``, ROS 2 log-level arguments, and ``<param from=...>`` for YAML loading.
It also adds ``gdb_term_cmd`` for opening gdb in a terminal.
