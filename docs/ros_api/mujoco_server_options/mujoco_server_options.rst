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
  MuJoCo XML or MJB file. Defaults to empty (no implicit ``pendulum_world.xml``).

  **Without a description bundle** (``urdf_source`` unset): ``modelfile`` is the
  full model to load. An empty value starts an empty simulation with a warning;
  set a path such as ``$(find mujoco_ros)/assets/pendulum_world.xml`` for a
  complete robot-and-world model.

  **With a description bundle** (``urdf_source`` set): ``modelfile`` is the
  world MJCF to compose the URDF robot into. An empty value uses the built-in
  ``default_world.xml`` (checker ground plane, light, spawn frame). SRDF and
  Extended Params launch args remain optional.

``wait_for_xml``
  Waits for model XML on the parameter server instead of immediately loading a file path. Defaults to ``false``.

Description Bundle Arguments
----------------------------

Optional overrides that load the model from a URDF description bundle instead
of a standalone ``modelfile``. The bundle activates when ``urdf_source`` is set;
SRDF args are optional (URDF-only bundles are valid). When active, the server
reads bundle params (``urdf.*`` / ``srdf.*``),
converts the URDF (and optional SRDF / Extended Params), and composes the robot
into the world given by ``modelfile`` (empty ``modelfile`` → built-in
``default_world.xml``).

``urdf_source``
  URDF source kind: ``file`` or ``topic``. Empty skips the description bundle.

``urdf_path``
  URDF file path when ``urdf_source:=file``.

``urdf_topic``
  Topic publishing the URDF as a latched ``std_msgs/String`` when
  ``urdf_source:=topic`` (e.g. ``/robot_description``).
  The publisher must use ``transient_local`` durability (ROS 2) / ``latch=true``
  (ROS 1) -- a non-latched publisher is not readable through this path. If
  unset, the server defaults to ``robot_description``.

``srdf_source``
  Optional. SRDF source kind: ``file`` or ``topic``.

``srdf_path``
  SRDF file path when ``srdf_source:=file``.

``srdf_topic``
  Topic publishing the SRDF as a latched ``std_msgs/String`` when
  ``srdf_source:=topic``. Same latching requirement as ``urdf_topic``. If
  unset, the server defaults to ``robot_description_semantic``.

``convert_ascii_stl``
  Optional. Sets ``description.convert_ascii_stl`` to ``true`` or ``false``.
  When ``true``, ASCII visual/collision STLs are converted to binary under
  ``/tmp/mujoco_ros_stl_cache/`` (package files are never modified). When
  unset/empty (default), conversion is off and ASCII STLs use the collision
  OBJ fallback (``<stem>.obj`` / ``<stem>_convex_hull.obj``) or fail with a
  hint naming this option.

Example (URDF-only, default world)::

   roslaunch mujoco_ros launch_server.launch use_sim_time:=true \\
     urdf_source:=file urdf_path:=/path/to/robot.urdf

Example (URDF + SRDF + custom world)::

   roslaunch mujoco_ros launch_server.launch use_sim_time:=true \\
     urdf_source:=file urdf_path:=/path/to/robot.urdf \\
     srdf_source:=file srdf_path:=/path/to/robot.srdf \\
     modelfile:=/path/to/my_world.xml

Example (ASCII STL convert on)::

   roslaunch mujoco_ros launch_server.launch use_sim_time:=true \\
     urdf_source:=file urdf_path:=/path/to/robot.urdf \\
     convert_ascii_stl:=true

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

The ``hybrid-devel`` branch provides ROS 2 launch files at:

* ``mujoco_ros/launch/ros2/launch/launch_server.launch.xml``
* ``mujoco_ros/launch/ros2/launch/launch_server.launch.py``

Both mirror the ROS 1 arguments (including the description-bundle overrides)
but use ROS 2 launch syntax, ``$(find-pkg-share ...)`` / ``get_package_share_directory``
paths, ``exec=`` instead of ``type=``, ROS 2 log-level arguments, and
``<param from=...>`` / ``ParameterFile`` for YAML loading.
They also add ``gdb_term_cmd`` for opening gdb in a terminal.
