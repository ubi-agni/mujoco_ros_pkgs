Services, Actions, and Runtime Parameters
=========================================

Service names below are relative to the server namespace, usually ``/mujoco_server``.

Simulation Control
------------------

``set_pause`` (``mujoco_ros_msgs/SetPause``)
  Pauses or resumes the simulation.

``reset`` (``std_srvs/Empty``)
  Resets the current simulation state.

``reload`` (``mujoco_ros_msgs/Reload``)
  Reloads the current model or loads a replacement model string/path from the request.

``shutdown`` (``std_srvs/Empty``)
  Requests server shutdown.

``set_rt_factor`` (``mujoco_ros_msgs/SetFloat``)
  Changes the desired real-time factor.

``load_initial_joint_states`` (``std_srvs/Empty``)
  Re-applies configured initial joint positions and velocities.

``get_loading_request_state`` (``mujoco_ros_msgs/GetStateUint``)
  Reports the current loading request state.

``get_sim_info`` (``mujoco_ros_msgs/GetSimInfo``)
  Returns model path, validity, load count, loading state, pause state, pending step count, measured real-time factor, and desired real-time factor.

State and Property Services
---------------------------

``get_body_state`` / ``set_body_state``
  Read or update a body by body name or by a geom belonging to that body. The setter can selectively apply pose, twist, mass, and qpos reset behavior.

``get_geom_properties`` / ``set_geom_properties``
  Read or update geom type, body mass, friction, and size fields.

``get_eq_constraint_parameters`` / ``set_eq_constraint_parameters``
  Read or update equality constraint parameters by name, including active state, solver parameters, anchor/relative pose fields, torquescale, and polynomial coefficients.

``get_gravity`` / ``set_gravity``
  Read or update the simulation gravity vector.

``get_plugin_stats`` (``mujoco_ros_msgs/GetPluginStats``)
  Returns plugin load/reset timing and exponential moving averages for control, passive, render, and last-stage callbacks.

Step Action
-----------

``step`` (``mujoco_ros_msgs/StepAction``)
  Steps a paused simulation for ``num_steps`` MuJoCo steps. Feedback reports ``steps_left``. The action succeeds when all requested steps complete and is preempted if the simulation is unpaused, reset, shut down, or the action is canceled.

Admin Hash
----------

Protected requests include an ``admin_hash`` field.
When ``eval_mode`` is disabled, the server accepts the operations normally.
When ``eval_mode`` is enabled, mutating or sensitive operations require the request hash to match the hash configured at launch.

Runtime Parameters
------------------

ROS 1 exposes ``mujoco_ros/SimParams`` through dynamic reconfigure.
ROS 2 exposes the same runtime tuning surface as standard node parameters on ``/mujoco_server``.
The parameter names are intentionally flat and match the ROS 1 dynamic reconfigure names.

The most important user-facing groups are:

* Pause/resume through ``running``.
* Runtime ``admin_hash`` updates.
* Physics options: integrator, friction cone, Jacobian type, and solver.
* Algorithmic options: timestep, solver iterations/tolerances, CCD, and SDF iteration settings.
* Physical options: gravity, wind, magnetic field, density, viscosity, and impedance ratio.
* MuJoCo enable/disable flags such as gravity, contact, actuation, warmstart, and related solver/runtime flags.

``unpause`` is a startup parameter.
Use ``running`` for runtime pause/resume through dynamic reconfigure in ROS 1 or ``ros2 param set`` in ROS 2.

Example ROS 2 runtime updates:

.. code-block:: bash

   ros2 param set /mujoco_server running false
   ros2 param set /mujoco_server timestep 0.002
   ros2 param set /mujoco_server gravity "0 0 -3.71"

Use services for scripted state changes and runtime parameters for interactive physics tuning.

Plugin Services
---------------

``/sensors/register_noise_models``
  Provided by ``mujoco_ros_sensors``. Registers per-sensor noise models.

``/set_mocap_state``
  Provided by ``mujoco_ros_mocap_plugin``. Applies a ``mujoco_ros_msgs/MocapState`` to named mocap bodies.
