Troubleshooting
===============

Start with ``verbose:=true`` when launching the server.
It loads the configured ROS console settings and usually exposes the missing parameter, plugin, model, or rendering detail quickly.

Build Issues
------------

MuJoCo version mismatch
  Make sure the MuJoCo headers used at compile time and the shared library used at runtime come from the same installation and match the branch you are building.

Archive installation not found
  Set ``MUJOCO_DIR``, ``LD_LIBRARY_PATH``, and ``LIBRARY_PATH`` so CMake and the runtime linker can find MuJoCo.

Source build not found
  Install MuJoCo into a prefix visible to your workspace or pass the correct CMake prefix/library paths.

Hybrid tinyxml2 crash
  On ``hybrid-devel``, check the branch README for the MuJoCo source-build tinyxml2 pin if the server segfaults around XML/rospack interactions.

Runtime Issues
--------------

Action goals ignored after reset or reload
  Reset and reload set simulated time back to zero. Build the actionlib branch containing the time-reset fix when your workspace depends on action servers during repeated resets.

No camera images
  Confirm ``render_offscreen:=true`` and that the image or camera-info topic has a subscriber. Offscreen cameras are intentionally lazy.

No ground-truth sensor topics
  Check ``eval_mode``. Ground-truth ``_GT`` topics are suppressed in evaluation mode.

Plugin not loaded
  Confirm the plugin YAML is loaded through ``mujoco_plugin_config`` and that the top-level key is ``MujocoPlugins``. The plugin ``type`` must match the
  class exported in the package plugin XML.

Viewer window opens but stays blank from Python
  This has been observed with dedicated NVIDIA GPUs when ``prime-select`` is set to ``on-demand``. Check ``prime-select query`` and try switching to the NVIDIA profile, then reboot.
