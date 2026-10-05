Plugin System
=============

MuJoCo ROS uses pluginlib to load simulation extensions at runtime. The core server owns MuJoCo's global callback hooks and dispatches them to each loaded ``MujocoPlugin`` instance.

Plugin Configuration
--------------------

Plugins are configured under the ``MujocoPlugins`` parameter.
The server first searches within its private namespace and then globally.

.. code-block:: yaml

   MujocoPlugins:
     - type: package_name/PluginClass
       custom_option: 1.0
     - type: other_package/OtherPlugin

Each entry must provide ``type``. Additional keys are passed to that plugin as its configuration.

Lifecycle
---------

``Discovery``
  The server finds the ``MujocoPlugins`` list on the parameter server.

``Registration``
  Each list entry is checked and the plugin type is resolved through pluginlib.

``Instantiation``
  The plugin instance is created and receives its ROS node handle, configuration, and owning ``MujocoEnv``.

``Load``
  After the MuJoCo model and data exist, the plugin receives ``mjModel`` and ``mjData`` pointers and can bind model-specific resources.

``Run``
  The server calls plugin callbacks while stepping physics and rendering.
  Callback order follows the load order.

``Unload``
  Plugins are unloaded when the server shuts down or reloads the model.

Callbacks
---------

Plugins override only the callbacks they need:

* ``controlCallback`` runs from MuJoCo's control callback.
* ``passiveCallback`` runs from MuJoCo's passive callback.
* ``lastStageCallback`` runs at the end of a physics step, before the offscreen render turn for that step.
  From ``load``, a plugin calls ``MujocoEnv::OpenOffscreenCamera(name)``, ``SetGeomGroup``, ``SetVisualFlag``, and
  ``EnableFrames``. While a reload is still in progress those setter calls and ``EnableFrames`` are recorded on the
  handle and applied once the camera exists. ``GeomGroupEnabled``, ``VisualFlagEnabled``, and ``TakeLatest`` need a live
  camera and throw ``std::runtime_error`` with message ``camera is unavailable during reload`` if reload is still in
  progress. While camera retirement is pending, every call on the handle is rejected with the same message. Do not open
  the handle from ``lastStageCallback`` or other step callbacks; use the load-time bind and queued settings instead.
  ``lastStageCallback`` calls ``TakeLatest`` to borrow the previous completed frame. The lease field
  ``simulation_time_ns`` is the simulated time of the rendered state, not wall clock. The handle rebinds by name after
  reload; a missing name throws. Calling ``TakeLatest`` before ``EnableFrames``, or calling ``EnableFrames`` twice,
  throws ``std::logic_error``.
* ``renderCallback`` can add visualization geoms before a GUI or offscreen scene is rendered.
* ``reset`` lets a plugin clear internal state after a simulation reset.

.. warning::

   A plugin must not replace MuJoCo's global callback functions directly. The server owns those hooks so it can resolve the active environment and dispatch to all loaded plugin instances.

Reload Behavior
---------------

On reload, the server unloads plugins, reloads the model/data, reads plugin configuration again, and loads plugins again.
Configuration changes therefore only require a reload, not a full server restart.
