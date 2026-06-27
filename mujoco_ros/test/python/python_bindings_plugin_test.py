#!/usr/bin/env python3

from pathlib import Path
import importlib
import sys
import time
import unittest

import mujoco_ros.plugins as plugin_bindings
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


def wait_for_idle(env, timeout=5.0):
    deadline = time.monotonic() + timeout
    while env.operational_status != 0 and time.monotonic() < deadline:
        time.sleep(0.01)
    if env.operational_status != 0:
        raise AssertionError("environment did not become idle before timeout")


class PythonBindingsPluginTest(unittest.TestCase):
    def test_plugin_packages_register_bindings_when_imported(self):
        plugin_packages = {
            "mujoco_ros_sensors": "mujoco_ros_sensors/MujocoRosSensorsPlugin",
            "mujoco_ros_laser": "mujoco_ros_laser/LaserPlugin",
            "mujoco_ros_mocap": "mujoco_ros_mocap/MocapPlugin",
            "mujoco_ros_control": "mujoco_ros_control/MujocoRosControlPlugin",
        }

        missing = []
        for package_name, plugin_type in plugin_packages.items():
            try:
                importlib.import_module(package_name)
            except ImportError:
                missing.append(package_name)
                continue
            self.assertIn(plugin_type, plugin_bindings.available_plugins())

        if len(missing) == len(plugin_packages):
            self.skipTest("no plugin binding packages are importable in this test environment")

    def test_plugin_entry_point_binding_contract(self):
        class GenericPlugin:
            name = "test_plugin"
            type = "mujoco_ros/TestPlugin"

        class BoundPlugin:
            def __init__(self, plugin):
                self.plugin = plugin

        class FakeEntryPoint:
            name = "TestPlugin"

            def load(self):
                return BoundPlugin

        original_entry_points = plugin_bindings._entry_points
        try:
            plugin_bindings._entry_points = lambda: [FakeEntryPoint()]
            bound_plugin = plugin_bindings.bind_plugin(GenericPlugin())
            self.assertIsInstance(bound_plugin, BoundPlugin)
            self.assertEqual("mujoco_ros/TestPlugin", bound_plugin.plugin.type)

            sys.modules.pop("mujoco_ros.plugins.TestPlugin", None)
            imported = importlib.import_module("mujoco_ros.plugins.TestPlugin")
            self.assertIs(imported.TestPlugin, BoundPlugin)
        finally:
            plugin_bindings._entry_points = original_entry_points
            sys.modules.pop("mujoco_ros.plugins.TestPlugin", None)

    def test_plugin_registry_binding_contract(self):
        class GenericPlugin:
            name = "registered_plugin"
            type = "mujoco_ros/RegisteredPlugin"

        class BoundPlugin:
            def __init__(self, plugin):
                self.plugin = plugin

        plugin_bindings.register_plugin_binding("RegisteredPlugin", BoundPlugin)
        try:
            self.assertIn("RegisteredPlugin", plugin_bindings.available_plugins())
            bound_plugin = plugin_bindings.bind_plugin(GenericPlugin())
            self.assertIsInstance(bound_plugin, BoundPlugin)
            self.assertEqual("mujoco_ros/RegisteredPlugin", bound_plugin.plugin.type)

            sys.modules.pop("mujoco_ros.plugins.RegisteredPlugin", None)
            imported = importlib.import_module("mujoco_ros.plugins.RegisteredPlugin")
            self.assertIs(imported.RegisteredPlugin, BoundPlugin)
        finally:
            plugin_bindings.unregister_plugin_binding("RegisteredPlugin")
            sys.modules.pop("mujoco_ros.plugins.RegisteredPlugin", None)

    def test_plugin_stats_are_exposed(self):
        model_path = get_package_share_directory("mujoco_ros") / "assets" / "pendulum_world.xml"

        with MujocoEnv() as env:
            self.assertTrue(env.load_model_from_string(str(model_path)))
            wait_for_idle(env)

            self.assertEqual(1, len(env.plugins))
            self.assertEqual(1, len(env.plugin_names))
            self.assertEqual(1, len(env.plugin_stats))

            stat = env.plugin_stats[0]
            self.assertEqual("mujoco_ros/TestPlugin", stat.type)
            self.assertGreaterEqual(stat.load_time, 0.0)
            self.assertGreaterEqual(stat.reset_time, -1.0)
            self.assertGreaterEqual(stat.ema_steptime_control, 0.0)
            self.assertGreaterEqual(stat.ema_steptime_passive, 0.0)
            self.assertGreaterEqual(stat.ema_steptime_render, 0.0)
            self.assertGreaterEqual(stat.ema_steptime_last_stage, 0.0)

            plugin = env.plugins[0]
            self.assertEqual(stat.name, plugin.name)
            self.assertEqual(stat.type, plugin.type)
            self.assertEqual(stat.name, env.plugin_names[0])
            self.assertGreaterEqual(plugin.load_time, 0.0)
            self.assertGreaterEqual(plugin.reset_time, -1.0)
            self.assertGreaterEqual(plugin.ema_steptime_control, 0.0)
            self.assertGreaterEqual(plugin.ema_steptime_passive, 0.0)
            self.assertGreaterEqual(plugin.ema_steptime_render, 0.0)
            self.assertGreaterEqual(plugin.ema_steptime_last_stage, 0.0)


if __name__ == "__main__":
    if is_ros1():
        import rostest

        rostest.rosrun("mujoco_ros", "python_bindings_plugin_test", PythonBindingsPluginTest)
    else:
        unittest.main(argv=[sys.argv[0]])
