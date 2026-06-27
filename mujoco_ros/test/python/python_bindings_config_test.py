#!/usr/bin/env python3

from pathlib import Path
import sys
import time
import unittest

from mujoco_ros import MujocoEnv


def get_package_share_directory(package_name):
    try:
        from ament_index_python.packages import get_package_share_directory as get_ament_share

        return Path(get_ament_share(package_name))
    except ImportError:
        import rospkg

        return Path(rospkg.RosPack().get_path(package_name))


def is_ros1():
    try:
        import rosgraph  # noqa: F401
    except ImportError:
        return False
    return True


def require_python_mujoco():
    try:
        import mujoco  # noqa: F401
    except ImportError:
        raise unittest.SkipTest("Python mujoco package is not available")


def wait_for_idle(env, timeout=5.0):
    deadline = time.monotonic() + timeout
    while env.operational_status != 0 and time.monotonic() < deadline:
        time.sleep(0.01)
    if env.operational_status != 0:
        raise AssertionError("environment did not become idle before timeout")


class PythonBindingsConfigTest(unittest.TestCase):
    def test_python_plugin_config_loads_test_plugin(self):
        require_python_mujoco()
        model_path = get_package_share_directory("mujoco_ros") / "assets" / "pendulum_world.xml"
        plugin_config = [
            {
                "type": "mujoco_ros/TestPlugin",
                "example_param": 0.0,
            }
        ]

        with MujocoEnv(plugin_config=plugin_config) as env:
            self.assertTrue(env.load_from_path(str(model_path)))
            wait_for_idle(env)
            self.assertEqual(1, len(env.plugins))
            self.assertEqual(1, len(env.plugin_names))
            self.assertEqual("mujoco_ros/TestPlugin", env.plugins[0].type)
            self.assertEqual("mujoco_ros/TestPlugin", env.plugin_stats[0].type)


if __name__ == "__main__":
    if is_ros1():
        import rostest

        rostest.rosrun("mujoco_ros", "python_bindings_config_test", PythonBindingsConfigTest)
    else:
        unittest.main(argv=[sys.argv[0]])
