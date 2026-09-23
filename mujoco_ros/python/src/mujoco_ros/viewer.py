"""Interactive viewer entry points for :class:`mujoco_ros.MujocoEnv`."""


def _binding(env):
    try:
        return env.binding
    except AttributeError as exc:
        raise TypeError('env must be a mujoco_ros.MujocoEnv') from exc


def launch(env):
    """Run a blocking interactive viewer on the calling thread."""
    return _binding(env)._launch_viewer()


def launch_passive(env, auto_sync=False):
    """Start a passive viewer and return its lifetime handle."""
    return _binding(env)._launch_passive(auto_sync=bool(auto_sync))
