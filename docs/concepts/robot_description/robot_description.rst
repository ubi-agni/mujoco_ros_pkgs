Robot Description
==================

URDF bundle loading
-------------------

``mujoco_ros`` can load a robot from a URDF description bundle instead of a
standalone MJCF model. The smallest installed example is a two-link robot with
primitive visual geometry only:

.. code-block:: bash

   roslaunch mujoco_ros simple_visual_robot.launch

.. code-block:: bash

   ros2 launch mujoco_ros simple_visual_robot.launch.py
   ros2 launch mujoco_ros simple_visual_robot.launch.xml

The example URDF lives at
``mujoco_ros/examples/robot_description/simple_visual_robot.urdf``. Its root
link is named ``chassis`` rather than ``base_link`` so the converter exercises
the inferred-root path instead of relying on a conventional link name.

This example intentionally uses only URDF primitive visual geometries
(``box`` and ``cylinder``). When a link has primitive visuals but no collision
geometry, the converter mirrors those primitives into collision geometry before
handing the URDF to MuJoCo so the resulting robot has real geoms in the
Simulation Model.

Mesh ``filename`` URIs are normalized before MuJoCo parse:

* ``file://`` and absolute paths are used as-is.
* ``package://<pkg>/...`` resolves via the ROS package share directory.
* Relative paths (e.g. ``meshes/collision/foo.obj``) are resolved against the
  URDF file's directory and are only legal for file-backed URDFs. Param-sourced
  URDF strings must use absolute, ``file://``, or ``package://`` mesh URIs.

MuJoCo 3.3.5 does not load ``.glb``. Visual ``.glb`` meshes are rewritten in an
in-memory working copy to a sibling ``.stl``/``.STL`` (case-insensitive), or
else ``../collision/<stem>.stl``. The on-disk URDF is never modified. Missing
replacements and unsupported formats fail conversion loudly.

Meshes are supplied to MuJoCo through ``mjVFS`` with basename keys while
``strippath`` remains enabled.

MuJoCo also loads only **binary** STL (and OBJ), not ASCII STL. When a
visual mesh is ASCII STL, mesh prep first looks for a collision OBJ
fallback next to the mesh (``collision/<stem>.obj``, then
``collision/<stem>_convex_hull.obj``, case-insensitive). If no fallback
exists and the ``description.convert_ascii_stl`` param (default ``false``)
is set to ``true``, the ASCII STL is converted in-process to binary and
cached under ``/tmp/mujoco_ros_stl_cache/``; the source file is never
modified. With no fallback and conversion disabled, conversion fails
loudly and names ``description.convert_ascii_stl:=true`` as the fix.

Geom groups
~~~~~~~~~~~

After parsing, robot geoms are assigned MuJoCo geom groups so collision
hulls don't dominate the viewer over visual meshes: geoms with
``contype == 0 && conaffinity == 0`` (visual) are put in **group 1** with
``contype``/``conaffinity`` forced to 0, and all other geoms (collision)
are put in **group 2** when the robot has visuals. When the robot has no
visual geoms, collision geoms use **group 1** so they remain visible under
default viewer flags.

The interactive viewer calls ``ApplyInteractiveViewerGeomDefaults`` after
``mjv_defaultOption`` and turns **geom group 2 off** by default, hiding
description collision hulls while keeping visual meshes visible. Users can
re-enable group 2 in the viewer UI.

Extended Params
---------------

Extended Params is a generic, public SRDF extension carrying per-joint
physical characterization. See ``docs/GLOSSARY.md``'s "Extended Params"
glossary entry for the one-paragraph definition; this page describes the
canonical ``mujoco_ros::ParseSrdfExtensions`` path
(``mujoco_ros/include/mujoco_ros/extended_params.hpp``,
``mujoco_ros/src/extended_params.cpp``).

Document shape
--------------

.. code-block:: xml

   <robot name="<robot name>">
     <extended_params name="<joint name, matching the URDF joint name exactly>">
       <mujoco_actuator kp="..." kv="..." armature="..."/>
       <mujoco_gravcomp value="..."/>
       <vendor_tag vendor_attribute="..."/>
     </extended_params>
     <!-- repeat <extended_params> per joint -->
   </robot>

SRDF supplied through ``srdf.path`` (or the equivalent SRDF argument) is
the only supported input for Extended Params. A standalone
``<extended_params>`` XML document is not accepted -- put ``<extended_params>``
entries directly in the robot's SRDF.

Core parses only two built-in child elements:

* ``mujoco_actuator``: optional, independently validated ``kp``, ``kv``, and
  ``armature`` overrides for generated actuators. Each value must be finite
  and non-negative.
* ``mujoco_gravcomp``: required ``value`` attribute. The value must be finite and is
  applied to the joint's parent body.

All other child elements are custom tags. Core preserves their XML and sends
each tag to a handler registered with
``mujoco_ros::RegisterExtendedParamsHandler``. A handler receives the joint
name, tag name, XML element, and a pass-through
``ConverterExtensionContext``. Core does not validate custom-tag attributes or
interpret their values. If no handler is registered, core warns and skips the
tag. This allows downstream packages to own schemas such as
``friction_parameters``, ``motor``, and ``compliance``.

Fields
------

Built-in ``mujoco_actuator`` values are stored in
``mujoco_ros::ExtendedParamsByJoint``. Custom-tag fields are not part of the
core schema. The package that registers the handler defines, validates, and
consumes them.

What Extended Params does not do
--------------------------------

Built-in ``mujoco_actuator`` values are used during optional actuator generation.
Custom-tag values are passed to registered handlers through
``ConverterExtensionContext``. Core does not derive damping, friction, motor,
compliance, or other physical values from custom tags.

Actuator generation
--------------------

When a URDF handed to the converter contains a ``<ros2_control>`` block (standard ROS 2 syntax --
``<joint name="..."><command_interface name="position|velocity|effort"/></joint>``), the converter
can generate one native MuJoCo ``<actuator>`` per ``(joint, command_interface)`` pair, **before**
``mj_compile()``. This is opt-in: set ``description.generate_actuators:=true`` (default ``false``)
on the description-bundle params, or pass ``generate_actuators=true`` directly to
``ConvertDescription``/``SaveDescriptionToTempMjb``/``load_model_from_description``.

**Why opt-in:** ``mujoco_ros_control`` already dispatches control commands through its own
``ros2_control``-driven Actuator-Routing/Generalized-Force-Fallback split and does not need native
actuators to function. Auto-generating them unconditionally would silently change the physical
behavior of every existing ``mujoco_ros_control`` deployment (its qfrc-fallback path defers
entirely to whichever actuator gains exist, once one exists for a joint). Only callers that
structurally need native actuators should set this ``true``.

**Naming:** ``<joint_name>_act_pos`` / ``_act_vel`` / ``_act_eff`` -- this matches
``mujoco_ros_control``'s existing, already-shipped actuator lookup convention exactly, so it picks
up generated actuators automatically with no changes on its side.

**Ranges:** ``ctrlrange``/``forcerange`` always come from the URDF's own ``<joint><limit .../>`` --
never invented. A joint requesting any command_interface with a missing ``<limit>``, or a
``<limit>`` missing the attribute that interface needs (``effort`` always; additionally
``lower``/``upper`` for ``position``, ``velocity`` for ``velocity``), is a hard error.

**Gains:** fixed, conservative, un-tuned-for-any-robot defaults (documented in
``description_converter.cpp``). Override per joint via a new optional Extended Params child
element:

.. code-block:: xml

   <robot name="<robot name>">
     <extended_params name="<joint name>">
       <mujoco_actuator kp="..." kv="..." armature="..."/> <!-- each attribute independently optional -->
     </extended_params>
   </robot>

``forcerange``/``ctrlrange`` are deliberately **not** overridable through this element -- they
always track the URDF's own ``<limit>``.

**Mimic joints:** a joint that is a URDF ``<mimic>`` follower and is also claimed by
``<ros2_control>`` is a hard error (it would fight the driver joint's actuator). Full
``<mimic>`` → MJCF ``<equality>`` porting is separate, tracked roadmap work; this converter does
not generate equality constraints today.

**Attach prefix:** the compose-time body/joint/actuator name prefix (``ComposeOptions::prefix``,
``description.attach_prefix``) defaults to an empty string, not a hardcoded value -- a
single-robot bundle produces unprefixed names by default. Multi-robot callers needing distinct
namespacing pass an explicit prefix (e.g. ``"r0_"``/``"r1_"``).
