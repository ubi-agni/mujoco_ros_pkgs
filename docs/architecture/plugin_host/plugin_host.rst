Plugin Host
===========

``PluginHost`` centralizes plugin adapter lifecycle, dispatch, and statistics behind one generation-tagged contract, owned by ``MujocoEnv``. ROS 1 and ROS 2 keep their own parameter parsing, pluginlib construction, and node setup — that stays on the adapter side of the boundary; ``PluginHost`` only ever sees the ``IPluginAdapter`` interface.

Ownership
---------

``PluginHost`` owns the loaded adapter set, its generation tag, callback dispatch, reset and geometry notifications, and statistics/diagnostics for that set. It does not own ``mjModel``, ``mjData``, physics stepping, or MuJoCo's global callback hooks — those stay with ``MujocoEnv``, which is the only thing that ever calls into ``PluginHost``.

Old adapters are always gone before new ones exist
-----------------------------------------------------

``LoadGeneration(model, data, model_generation, plugin_generation)`` starts by unconditionally calling ``QuiesceAndDestroy()``, which clears the entry list and marks the generation inactive — regardless of whether a generation was already loaded. Only after that does it ask the factory for a fresh set of adapters, load each one, collect statistics, and activate the ones that loaded successfully. There is never a window where an old and a new generation's adapters both exist in ``entries_``.

The callback-free gap
-------------------------

``generation_active_`` is ``false`` from the moment ``QuiesceAndDestroy()`` runs until ``LoadGeneration`` has finished its entire pass — factory construction, every adapter's ``Load``, every adapter's ``Statistics``, every loaded adapter's ``Activate``. Every dispatch entry point (``DispatchControl``, ``DispatchPassive``, ``DispatchRender``, ``DispatchLastStage``, ``Reset``, ``NotifyGeometryChanged``) checks this first, under the same mutex that ``LoadGeneration`` holds while flipping it, and simply returns without dispatching while it is false.

This is what makes the reload-time ``mj_forward()`` call (see :doc:`../overview/overview`) callback-free by construction rather than by convention: at the point ``MujocoEnv`` calls it, the old generation has already been destroyed and the new one has not yet been activated, so there is no adapter set for any dispatch call to reach.

Stale vs. absent are different failures
-------------------------------------------

Once a generation *is* active, a dispatch call tagged with a ``ModelGeneration`` that doesn't match the currently active one throws, rather than quietly returning. The two situations look similar from the caller's side — "nothing happened" — but mean different things: no generation loaded yet is expected and unremarkable; a call arriving tagged for a generation that has already been replaced is a caller holding a stale reference, and that fails loudly instead of silently doing nothing.

Per-adapter failure isolation
---------------------------------

During ``LoadGeneration``, one adapter's exception at any phase (factory construction, ``Load``, ``Statistics``, ``Activate``) is caught, recorded as a ``PluginLoadFailure`` with the phase it happened in, and does not stop the other adapters in the same pass. A failed adapter's statistics stay visible (``Statistics()`` returns a fallback entry with just its name/type if even that call failed) but it is never marked ``ready``, so it never receives a callback.

During live dispatch (``Control``/``Passive``/``Render``/``LastStage``/``Reset``/``GeometryChanged``), an adapter exception is still recorded the same way — but it is then rethrown rather than swallowed. A plugin misbehaving during a live physics step is not something the host can safely paper over by continuing as if nothing happened.

Scoped access for backend objects
--------------------------------------

``PluginHost::AcquireScopedAccess()`` (and its generation-pinned overloads) returns a ``ScopedPluginAccess`` that holds ``PluginHost``'s mutex for its own lifetime and is tied to the ``ModelGeneration``/``PluginGeneration`` it was acquired against. Acquiring it with an expected generation that no longer matches throws immediately, rather than handing back access that could act on an adapter set that has already been torn down. This is the mechanism a caller uses to reach a specific adapter's backend object (``ScopedPluginAccess::BackendObject<Backend>``) without ever holding a raw, un-pinned pointer to it.
