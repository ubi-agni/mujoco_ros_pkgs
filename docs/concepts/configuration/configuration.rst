Configuration
=============

MuJoCo ROS configuration is intentionally parameter-driven. Launch files load YAML into the ROS parameter server, the server reads those parameters during setup and reload, and plugins receive their own nested configuration through ``MujocoPlugins``.

Runtime Options
---------------

Runtime Options are the typed contract for reading and updating active MuJoCo
``mjOption`` fields. ROS 1 exposes the same field names through dynamic
reconfigure. ROS 2 exposes them as regular node parameters. Python exposes
``env.runtime_options`` and ``env.apply_runtime_options({...})``.

Supported field names match ``RuntimeOptionsSnapshot``:

* Enumerations: ``integrator``, ``cone``, ``jacobian``, ``solver``
* Scalars: ``timestep``, ``iterations``, ``tolerance``, ``ls_iterations``,
  ``ls_tolerance``, ``noslip_iterations``, ``noslip_tolerance``,
  ``ccd_iterations``, ``ccd_tolerance``, ``sdf_iterations``,
  ``sdf_initpoints``, ``density``, ``viscosity``, ``impratio``, ``margin``
* Space-delimited arrays: ``gravity``, ``wind``, ``magnetic``, ``solimp``,
  ``solref``, ``friction``
* Disable flags: ``constraint_disabled``, ``equality_disabled``,
  ``frictionloss_disabled``, ``limit_disabled``, ``contact_disabled``,
  ``passive_disabled``, ``gravity_disabled``, ``clampctrl_disabled``,
  ``warmstart_disabled``, ``filterparent_disabled``, ``actuation_disabled``,
  ``refsafe_disabled``, ``sensor_disabled``, ``midphase_disabled``,
  ``eulerdamp_disabled``
* Enable flags: ``override_contacts``, ``energy``, ``fwd_inv``,
  ``inv_discrete``, ``multiccd``, ``island``

Legacy short names such as ``ls_iter``, ``noslip_tol``, and ``ccd_iter`` are
still accepted by the parser.

Updates use one transaction boundary. A rejected patch leaves every effective
field unchanged and does not advance the Options Epoch. Validation failures
identify the offending field. Python raises ``ValueError`` with
``field + ": " + message``; C++ and ROS adapters surface the same text.

During the Loading Window — from a model replacement request through completion
of model setup and generation activation — ``GetRuntimeOptions`` and
``ApplyRuntimeOptions`` reject with ``Runtime Options unavailable during
Loading Window``. Python raises ``RuntimeError`` with that message. Startup-only
patches may be supplied before the first model load through constructor
``runtime_options`` or ``SetPendingRuntimeOptions``; after loading begins that
path closes with ``Runtime Options startup options are unavailable after
loading begins``.

Camera Configuration
--------------------

MuJoCo cameras can publish ROS image streams without loading the sensors plugin.
Configure each named camera under ``cam_config/<camera_name>``:

.. code-block:: yaml

   cam_config:
     workspace_cam:
       stream_type: 7
       frequency: 15
       width: 720
       height: 480
       use_segid: true
       topic: cameras/workspace_cam
       name_rgb: rgb
       name_depth: depth
       name_segment: segmented

``stream_type`` is a bit mask: RGB is ``1``, depth is ``2``, and segmentation is ``4``.
Add values to enable multiple streams; ``7`` enables all three.

The default topic layout is:

* ``cameras/<camera_name>/rgb/image_raw``
* ``cameras/<camera_name>/rgb/camera_info``
* ``cameras/<camera_name>/depth/image_raw``
* ``cameras/<camera_name>/depth/camera_info``
* ``cameras/<camera_name>/segmented/image_raw``
* ``cameras/<camera_name>/segmented/camera_info``

Relative ``topic`` values are resolved in the server namespace.
Absolute topics are used as provided. Rendering is lazy: an offscreen camera only renders the streams that currently have image or camera-info subscribers.

Python offscreen consumers add demand independently from ROS subscribers.
Their requests use the same ``RenderCore`` capture as ROS publication. A
Python request can therefore force a capture without replacing the configured
ROS cadence. When both paths consume a frame, the camera-specific capture
identity is shared.

Python borrowed frames are read-only NumPy views backed by a frame lease. The
lease keeps the committed storage alive for the borrowed-view context. Use the
copying buffer API when the data must remain stable across later captures,
camera resize, or reload. Reload advances the frame generation and invalidates
old Python camera handles. A borrowed view already acquired keeps its lease
and remains readable. Acquire a new handle after reload.

Requesting RGB, depth, or segmentation when that plane is not enabled by
``stream_type`` is a terminal error. It does not produce an empty frame.

Offscreen RenderCore and Frame Boundary
---------------------------------------

Offscreen rendering is coordinated by the core package through ``RenderCore``,
which owns the ``Render Backend`` and the transport-neutral ``Frame Boundary``.
The standalone ``mujoco_ros_render_core`` library links only ``mujoco::mujoco``
and the selected offscreen graphics target (EGL, OSMesa, or none). Its public
include graph does not depend on ROS headers, ``MujocoEnv``, pluginlib, or the
visible GLFW backend. Visible GUI rendering remains outside this target.

Configure visible GLFW GUI with ``WITH_GUI`` (``ON`` or ``OFF``) and the
offscreen backend with ``OFFSCREEN_BACKEND`` (``ANY``, ``EGL``, ``OSMESA``, or
``DISABLE``). GLFW is not a supported offscreen backend.
When neither EGL nor OSMesa is available and ``DISABLE`` is not selected,
offscreen rendering is disabled rather than silently falling back to another
backend.

The Frame Boundary owns a bounded pool of frame slots and byte storage. Model
load and camera reconfiguration compute required capacity from camera layout and
Python history depth. When configured demand exceeds the RenderCore budget,
model setup fails loudly with an explicit error such as ``configured frame slot
capacity ... exceeds RenderCore frame boundary budget ...`` or ``configured
frame byte capacity ... exceeds RenderCore frame boundary byte budget ...``. The
server does not substitute empty frames or shrink history silently.

At runtime, capacity pressure is reported through explicit non-terminal
``FrameStatusCode`` values such as ``kFrameSlotsExhausted`` (no free slot for a
new capture) and ``kGenerationCapacityExhausted`` (retained old-generation
leases block reconfiguration). These are not collapsed into
``kTerminalError``; rendering may remain unavailable until leases release and
the caller retries.

The ``render_backpressure_policy`` setting controls what happens when a
capture cannot obtain its bounded frame capacity. It defaults to ``drop``,
which keeps physics submission non-blocking. ``wait_for_slot`` is opt-in:
the physics-side submission waits until capacity is available, while the
RenderCore render thread remains non-blocking. The wait is outside
``physics_thread_mutex_`` and is interrupted by cancellation, reload,
shutdown, or changing the policy back to ``drop``. ROS 1 dynamic reconfigure,
ROS 2 parameters, and Python accept exactly ``drop`` and ``wait_for_slot``;
invalid values are rejected without changing the effective policy.

Backend and graphics-context integrity failures are terminal. Examples include
``render backend is disabled``, ``backend is not initialized``, and
``graphics context initialization failed``. Consumers observe these through
``FrameStatusCode::kTerminalError``, ``kBackendFailure``, or
``kBackendUnavailable`` (see ``IsContextIntegrityFailure``) rather than blank
images.

Initial Joint States
--------------------

Initial joint states are configured as maps from MuJoCo joint names to string values:

.. code-block:: yaml

   initial_joint_positions:
     joint_map:
       hinge_joint: "0.0"
       ball_joint: "1 0 0 0"
       free_joint: "0 0 1 1 0 0 0"

   initial_joint_velocities:
     joint_map:
       hinge_joint: "0.0"
       ball_joint: "0 0 0"
       free_joint: "0 0 0 0 0 0"

The values are applied when the model loads, resets, reloads, or when ``load_initial_joint_states`` is called.

Evaluation Mode
---------------

``eval_mode`` restricts operations that should not be available during evaluation.
Requests that change simulation state include an ``admin_hash`` field; in evaluation mode the request hash must match the server's configured hash.
The sensors plugin also suppresses ground-truth topics in evaluation mode.
