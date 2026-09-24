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

* ``running``
* ``rt_factor``
* ``busywait``
* ``gravity``

Use ``env.settings.snapshot()`` to retrieve the raw read-only settings value object for debugging or tests.

Runtime Options
---------------

``env.runtime_options`` returns an immutable ``RuntimeOptionsSnapshot`` of the
active MuJoCo ``mjOption`` fields. ``env.apply_runtime_options(dict)`` applies a
partial patch through the same transaction path as ROS dynamic reconfigure and
ROS 2 parameters.

Field names match the configuration reference: integrator and solver settings,
scalars such as ``timestep`` and ``iterations``, space-delimited arrays such as
``gravity`` and ``solimp``, disable flags ending in ``_disabled``, and enable
flags such as ``energy`` and ``multiccd``.

A rejected patch leaves the previous snapshot unchanged and does not advance the
Options Epoch. Validation failures raise ``ValueError`` with
``field + ": " + message``:

.. code-block:: python

   before = env.runtime_options
   with self.assertRaisesRegex(ValueError, "solimp"):
       env.apply_runtime_options({"timestep": 0.002, "solimp": "0.9 0.95"})
   self.assertEqual(before, env.runtime_options)

During the Loading Window, reads and writes raise ``RuntimeError`` with
``Runtime Options unavailable during Loading Window``. Constructor keyword
``runtime_options={...}`` applies startup-only patches before the first model
load; invalid startup values raise ``ValueError`` at construction time.

Plugins
-------

Plugin configuration can be injected at construction time with ``plugin_config``, ``config_files``, and ``parameters``. Launch-provided configuration continues to work unchanged.

``env.sim_info`` is the preferred public status object.
It reports the model path, validity, load count, loading state, pause state, pending manual steps, and real-time factor values.
``env.sim_state`` remains available as a low-level compatibility object for measured slowdown, model validity, and load count.

``env.plugins`` returns generic bound plugin objects.
Each plugin exposes its name, type, load/reset timing, and callback timing EMAs.
``env.plugin_names`` provides a convenience list of plugin names.

Each loaded plugin belongs to the current **Plugin Generation** — the complete
adapter set for one Simulation Model and runtime data pair. Reload replaces the
generation. Python plugin handles reacquire the host on every property or method
access and compare the handle's generation to the active one. A handle retained
across reload raises ``RuntimeError("plugin handle belongs to an inactive Plugin
Generation")`` rather than returning stale backend state.

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

When offscreen rendering is enabled, camera metadata and frame access are
available through ``mujoco_ros.rendering``:

.. code-block:: python

   from mujoco_ros import MujocoEnv
   from mujoco_ros.rendering import OffcamManager

   with MujocoEnv(model_path="/absolute/path/to/camera_model.xml") as env:
       with OffcamManager(env.binding._camera_publication_transport, env.model) as cameras:
           rgb, depth, segment = cameras.buffer(0)

The Python camera helper reads committed ``RenderCore`` leases directly. It
does not subscribe to ROS image topics and does not maintain a private ring.
Python demand is independent from ROS subscriber demand. A Python consumer can
force a capture while ROS publishers keep their configured cadence. When both
consumers select a frame, they use the same camera-specific capture identity.

``buffer_size == 1`` registers a one-shot consumer: idle managers do not render
until ``borrow_latest_*`` or ``copy_*`` requests a frame. ``buffer_size > 1``
registers an explicit async cadence (one capture per simulation step,
independent from ROS) so step-then-read history fills without continuous idle
demand.

``buffer(last_n=...)`` returns stable copied NumPy arrays. Borrowed access is
read-only and keeps its lease alive for the context-manager lifetime:

.. code-block:: python

   with cameras.camera(0).borrow_latest_rgb() as rgb:
       assert not rgb.flags.writeable
       print(rgb.capture_id, rgb.frame_generation)

Copies remain valid after later captures, resize operations, and reloads.
Use ``OffcamManager.close()`` or a ``with OffcamManager(...)`` block to release
Python render registrations explicitly. Manager destruction performs the same
cleanup.
Reload advances the frame generation. Existing managers rebind their Python
demand to the new generation. A borrowed view already acquired keeps its old
lease and remains readable. Acquire a new borrowed view after reload.

Camera wrappers rebind by camera ID after reload. Their ``width``, ``height``,
``cam_name``, ``fps``, and plane metadata always describe the active camera
descriptor. A separately held native camera descriptor remains an old-layout
snapshot, and an already acquired borrowed view keeps its old shape and frame
generation. Name lookups are refreshed: the old name disappears and the new
name becomes available after the next manager lookup.

``cam.buffer`` remains a stable compatibility object. It delegates legacy
native names such as ``getBufferHandles()`` and frame-count properties, while
also supporting copied aggregate snapshots through
``cam.buffer(last_n=2)``. Legacy ring indices, lock methods, reset mutators,
and counter setters raise ``RuntimeError``. An unconfigured plane's read-only
frame count remains ``0`` for compatibility.

The aggregate ``buffer()`` result keeps its three-element shape. It uses
``None`` for a plane that is not configured. The explicit
``borrow_latest_*()`` and ``copy_*()`` accessors raise ``RuntimeError`` when
their requested plane is not configured. For a configured plane with no
committed frame yet, ``borrow_latest_*()`` raises ``RuntimeError`` while
``copy_*()`` and aggregate ``buffer()`` snapshots may return ``None``. The
removed native ring-buffer indices, lock methods, counter setters, and reset
mutators raise ``RuntimeError``.
``getBufferHandles()`` follows the same three-slot shape and returns ``None``
for a missing plane.

Interactive Viewer
------------------

Import the viewer module from the package root:

.. code-block:: python

   from mujoco_ros import MujocoEnv, viewer

Blocking mode
~~~~~~~~~~~~~

``viewer.launch(env)`` blocks the calling thread and owns GLFW on that thread
until the user closes the window. It returns only after the window closes.

.. code-block:: python

   with MujocoEnv(model_path="/path/to/model.xml") as env:
       viewer.launch(env)

Passive mode
~~~~~~~~~~~~

``viewer.launch_passive(env, auto_sync=False)`` returns immediately with a
native lifetime handle. The viewer runs on a dedicated GUI thread. Only one live
viewer is allowed per environment; a second launch raises
``RuntimeError("a viewer is already running for this MujocoEnv")``.

The handle supports ``close()``, ``is_running()``, ``sync(state_only=False)``,
``lock()`` (reentrant context manager), and context-manager exit (which calls
``close()``). Stale handles from an earlier viewer generation are
generation-checked: ``close()`` on a stale or already-closed handle is an
idempotent no-op; ``is_running()`` returns ``False``; ``sync()`` and ``lock()``
raise ``RuntimeError("viewer is not running")``.

Closing a passive window (Exit button, ``handle.close()``, or handle context
exit) stops only that viewer. The environment and physics loop stay available.

Automatic passive mode (``auto_sync=True``) synchronizes the viewer on each
rendered frame and around supported binding access while Python is idle.
Manual passive mode (the default) never auto-syncs; call ``handle.sync()`` after
binding changes, typically while holding ``handle.lock()``:

.. code-block:: python

   with MujocoEnv(model_path="/path/to/model.xml") as env:
       with viewer.launch_passive(env, auto_sync=True) as handle:
           env.unpause()
           env.set_gravity([0.0, 0.0, -9.81])

.. code-block:: python

   with MujocoEnv(model_path="/path/to/model.xml") as env:
       with viewer.launch_passive(env) as handle:
           with handle.lock():
               env.pause()
           handle.sync()

Covered auto-sync operations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When ``auto_sync=True``, these binding reads pull viewer-side state into Python
before returning:

* gravity (``get_gravity()``)
* Runtime Options (``runtime_options``, ``apply_runtime_options()`` reads)
* settings (``settings``, ``settings.snapshot()``)
* simulation state/info (``sim_state``, ``sim_info``)
* model/data snapshots (``model``, ``data``)
* running state (``is_running``, ``settings.running``)

These binding writes push Python-side state to the viewer after succeeding:

* load (``load_model_from_string()``, ``load_from_path()``, ``load_from_string()``)
* step, reset
* pause/unpause (``pause()``, ``unpause()``, ``toggle_paused()``)
* real-time factor (``set_rt_factor()``, ``settings.rt_factor``)
* busywait (``settings.busywait``)
* gravity (``set_gravity()``, ``settings.gravity``)
* Runtime Options (``apply_runtime_options()``)
* enable/disable flag helpers (``set_enableflag()``, ``set_disableflag()``,
  ``toggle_enableflag()``, ``toggle_disableflag()``)

Raw writes through ``env.model`` or ``env.data`` are unsupported and do not sync
the viewer.

Build requirement and backend reporting
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Interactive viewing requires a build configured with ``WITH_GUI=ON``. Builds
with ``WITH_GUI=OFF`` raise
``RuntimeError("mujoco_ros.viewer requires a build configured with WITH_GUI=ON")``.
Missing-display and GLFW initialization errors propagate; they are not swallowed.

Visible GLFW and offscreen RenderCore are independent:

* ``WITH_GUI`` controls the visible GLFW viewer backend.
* ``OFFSCREEN_BACKEND`` controls RenderCore offscreen capture.
* ``pymujoco_ros.__viewer_backend__`` reports the first (``"GLFW"`` or ``"NONE"``).
* ``pymujoco_ros.__render_backend__`` reports the second.

Deprecation
~~~~~~~~~~~

``MujocoEnv.attach_viewer(active=True)`` is deprecated. It maps ``active=True``
to blocking ``viewer.launch(self)`` and ``active=False`` to
``viewer.launch_passive(self, auto_sync=True)``.

Known limitations
~~~~~~~~~~~~~~~~~

Passive GLFW runs on a dedicated non-process-main viewer thread (ADR-0026).
This follows MuJoCo's passive-viewer shape but may be platform- or driver-sensitive
on some systems.

Hybrid-NVIDIA laptop frozen-frame behavior remains a post-implementation manual
verification item, not a guaranteed v1 fix. Cooperative ``pump()`` and a
separate-process GUI remain future options.

Render failures and capacity limits
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Offscreen rendering uses the shared ``mujoco_ros_render_core`` library. The
offscreen backend is selected at build time through ``OFFSCREEN_BACKEND``
(``ANY``, EGL, OSMesa, or ``DISABLE``). Visible GLFW GUI is controlled
independently by ``WITH_GUI=ON`` or ``WITH_GUI=OFF``. GLFW is not an offscreen
backend.

When configured camera history or byte demand exceeds the RenderCore budget,
model setup fails loudly with an explicit error such as ``configured frame slot
capacity ... exceeds RenderCore frame boundary budget ...``. The server does
not shrink history or return empty placeholders.

At runtime, capacity pressure surfaces as non-terminal ``FrameStatusCode`` values
such as ``kFrameSlotsExhausted`` and ``kGenerationCapacityExhausted``. These are
distinct from backend integrity failures
(``kTerminalError``, ``kBackendFailure``, ``kBackendUnavailable``).

When the configured backend is disabled or fails to initialize, frame access
returns terminal status rather than empty placeholders.

Python plane accessors raise ``RuntimeError`` for unconfigured planes and for
legacy ring-buffer APIs; they do not fabricate zero-filled frames.
``borrow_latest_*()`` also raises ``RuntimeError`` when a configured plane has
no committed frame yet. ``copy_*()`` and aggregate ``buffer()`` snapshots may
return ``None`` for a configured plane that has not committed a frame yet.

Current Limitations
-------------------

The core plugin packages currently provide specialized wrappers for sensors, laser, mocap, and control when their Python packages are importable.
The current port intentionally skips direct pixel rendering and MoveIt helpers.
Cooperative viewer ``pump()`` and a separate-process GUI are not implemented yet.
Passive GLFW runs on a dedicated non-process-main viewer thread (ADR-0026); this
follows MuJoCo's passive-viewer shape but may be platform- or driver-sensitive.
Hybrid-NVIDIA laptop frozen-frame behavior is tracked separately and is not
guaranteed fixed in v1.
