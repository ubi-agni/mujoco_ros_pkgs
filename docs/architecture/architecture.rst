Architecture
============

These pages describe how the core server is put together internally: who owns which piece of state, how concurrent access is ordered, and how a model reload moves through the system without letting old and new state mix. They are written for someone extending or maintaining the server itself, not for someone configuring or authoring a plugin — see :doc:`../concepts/concepts` and :doc:`../plugins/plugins` for that audience.

.. toctree::
    :maxdepth: 1

    overview/overview
    render_core/render_core
    plugin_host/plugin_host
    runtime_options/runtime_options
