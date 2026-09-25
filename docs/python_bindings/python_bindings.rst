Python Bindings
===============

The ``mujoco_ros`` Python bindings expose a ROS-version-neutral wrapper around the hybrid C++ ``MujocoEnv`` API.
They build in both ROS 1 and ROS 2 workspaces and use ``py_binding_tools`` for ROS initialization support.

Basic Use
---------

.. code-block:: python

   from pathlib import Path

   from mujoco_ros import MujocoEnv

   model_path = Path("/absolute/path/to/model.xml")

   with MujocoEnv(model_path=model_path) as env:
       env.pause()
       env.step(100)
       print(env.sim_info.model_valid)

Model and config path arguments accept strings and ``pathlib.Path`` objects.
``load_model_from_string()`` uses the C++ loader, while ``load_from_path()`` and ``load_from_string()`` create Python-owned ``mujoco.MjModel`` and ``mujoco.MjData`` objects and pass their addresses to the C++ environment.


Loading A URDF/SRDF Description Bundle
--------------------------------------

There are two Python-facing ways to create a ``MujocoEnv`` from a robot
description bundle:

* **File-backed bundle**: use ``MujocoEnv.from_description(...)`` when you
  already have URDF/SRDF files on disk.
* **Topic-backed bundle**: construct ``MujocoEnv(...)`` with the same ROS
  parameters the server launch files use. This is the path to use when the
  URDF and SRDF are published as latched ``std_msgs/String`` topics such as
  ``/robot_description`` and ``/robot_description_semantic``.

File-backed URDF/SRDF
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from pathlib import Path

   from mujoco_ros import MujocoEnv

   urdf_path = Path("/absolute/path/to/robot.urdf")
   srdf_path = Path("/absolute/path/to/robot.srdf")

   with MujocoEnv.from_description(
       urdf_path=urdf_path,
       srdf_path=srdf_path,
   ) as env:
       env.pause()
       env.step(1)
       print(env.filename)
       print(env.sim_info.model_valid)

``MujocoEnv.from_description(...)`` compiles the URDF/SRDF bundle into a MuJoCo
model in Python, then loads that model into the wrapped C++ environment.
The SRDF path may be an empty string when no SRDF is needed.

Topic-backed URDF/SRDF
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from mujoco_ros import MujocoEnv

   parameters = {
       "urdf.source": "topic",
       "urdf.topic": "/robot_description",
       "srdf.source": "topic",
       "srdf.topic": "/robot_description_semantic",
       "modelfile": "",
   }

   with MujocoEnv(parameters=parameters) as env:
       env.pause()
       env.step(1)
       print(env.filename)
       print(env.sim_info.model_valid)

This uses the same description-bundle parameter interface as the ROS launch
files. ``urdf.topic`` defaults to ``robot_description`` and ``srdf.topic``
defaults to ``robot_description_semantic`` when omitted.

If your publisher also provides Extended Params or you want extra converter
options, add them to the same ``parameters`` mapping:

.. code-block:: python

   parameters = {
       "urdf.source": "file",
       "urdf.path": "/absolute/path/to/robot.urdf",
       "srdf.source": "file",
       "srdf.path": "/absolute/path/to/robot.srdf",
       "description.generate_actuators": "true",
       "description.attach_prefix": "robot1_",
       "modelfile": "/absolute/path/to/world.xml",
   }

   with MujocoEnv(parameters=parameters) as env:
       env.step(10)

Model With Plugin Example
-------------------------

The following example loads an installed model, injects a plugin configuration, changes runtime settings, steps the simulation, and prints simulation and plugin information.

.. literalinclude:: examples/core_plugin_example.py
   :language: python

Runtime Settings
----------------

``env.settings`` is a runtime facade. Reads use a fresh read-only ``_EnvSettings`` copy, while supported writes call thread-safe C++ setters.
Writable fields are:

* ``running`` / ``run``
* ``rt_factor``
* ``busywait``
* ``gravity``

Use ``env.settings.snapshot()`` to retrieve the raw read-only settings value object for debugging or tests.

Plugins
-------

Plugin configuration can be injected at construction time with ``plugin_config``, ``config_files``, and ``parameters``. Launch-provided configuration continues to work unchanged.

``env.sim_info`` is the preferred public status object.
It reports the model path, validity, load count, loading state, pause state, pending manual steps, and real-time factor values.
``env.sim_state`` remains available as a low-level compatibility object for measured slowdown, model validity, and load count.

``env.plugins`` returns generic bound plugin objects.
Each plugin exposes its name, type, load/reset timing, and callback timing EMAs.
``env.plugin_names`` provides a convenience list of plugin names.

Downstream packages can provide plugin-specific wrappers by registering with ``mujoco_ros.plugins`` when imported, or through entry points in the ``mujoco_ros.plugins`` group.
Entry point names may match the plugin type suffix, the sanitized full plugin type, or the plugin instance name.
The loaded object can be a callable wrapper class, or expose ``bind(plugin)`` / ``from_mujoco_plugin(plugin)``.

.. code-block:: python

   entry_points={
       "mujoco_ros.plugins": [
           "MyPlugin = my_package.my_plugin:MyPlugin",
       ],
   }

The same entry points are importable as dynamic submodules, for example ``import mujoco_ros.plugins.MyPlugin``.

Rendering
---------

When offscreen rendering is enabled, camera metadata and ROS image ring buffers are available through ``mujoco_ros.rendering``:

.. code-block:: python

   from mujoco_ros import MujocoEnv
   from mujoco_ros.rendering import OffcamManager

   with MujocoEnv(model_path="/absolute/path/to/camera_model.xml") as env:
       cameras = OffcamManager(env.binding._offscreen_context, env.model)
       rgb, depth, segment = cameras.buffer(0)

``attach_viewer(active=True)`` attaches the GLFW viewer when the package was compiled with the GLFW render backend.
Passive viewer attachment is not implemented.

Current Limitations
-------------------

The core plugin packages currently provide specialized wrappers for sensors, laser, mocap, and control when their Python packages are importable.
The current port intentionally skips direct pixel rendering, passive viewer attachment, and MoveIt helpers.
