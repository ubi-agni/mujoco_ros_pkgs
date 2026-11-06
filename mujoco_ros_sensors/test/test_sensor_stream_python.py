#!/usr/bin/env python3
"""Python streams-only sensor API: plugin.sensors[name].config / .data / .gt_data."""

from pathlib import Path
import subprocess
import sys
import time
import unittest

from ament_index_python.packages import get_package_share_directory

import mujoco_ros_sensors  # noqa: F401  (registers the plugin binding)
from mujoco_ros import MujocoEnv
from mujoco_ros.plugins import bind_plugin

PLUGIN = {"name": "mujoco_ros_sensors", "type": "mujoco_ros_sensors/MujocoRosSensorsPlugin"}
MODEL = Path(get_package_share_directory("mujoco_ros_sensors")) / "test" / "sensors_world.xml"


def wait_for_idle(env, timeout=5.0):
    deadline = time.monotonic() + timeout
    while env.operational_status != 0 and time.monotonic() < deadline:
        time.sleep(0.01)
    if env.operational_status != 0:
        raise AssertionError("environment did not become idle before timeout")


def sensor_adr(env, name):
    import mujoco

    return env.model.sensor_adr[mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SENSOR, name)]


def register_noise(env, sensor, sigma):
    import rclpy
    from mujoco_ros_msgs.msg import SensorNoiseModel
    from mujoco_ros_msgs.srv import RegisterSensorNoiseModels

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("sensor_noise_client")
    try:
        client = node.create_client(RegisterSensorNoiseModels, "/mujoco_server/mujoco_ros_sensors/sensors/register_noise_models")
        assert client.wait_for_service(timeout_sec=5.0), "register_noise_models service missing"
        model = SensorNoiseModel()
        model.sensor_name = sensor
        model.set_flag = 1
        model.mean = [0.0, 0.0, 0.0]
        model.std = [sigma, 0.0, 0.0]
        req = RegisterSensorNoiseModels.Request()
        req.noise_models = [model]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        assert future.result() is not None and future.result().success
    finally:
        node.destroy_node()


class SensorStreamPythonTest(unittest.TestCase):
    def open_env(self, eval_mode=False, parameters=None):
        params = dict(parameters or {})
        if eval_mode:
            params["eval_mode"] = True
        env = MujocoEnv(
            plugin_config=[PLUGIN], admin_hash="some_hash" if eval_mode else None, parameters=params
        )
        self.addCleanup(env.__exit__, None, None, None)
        self.assertTrue(env.load_from_path(str(MODEL)))
        wait_for_idle(env)
        # Paused: only explicit step() advances physics, so reads are deterministic.
        env.pause("some_hash" if eval_mode else "")
        time.sleep(0.2)
        plugin = bind_plugin(env.plugins[0])
        return env, plugin

    def test_stream_data_without_subscriber(self):
        env, plugin = self.open_env()
        env.data.qvel[:] = 0.0
        env.data.qvel[env.model.jnt_dofadr[env.model.joint("joint2").id]] = 0.7
        env.step(3)
        s = plugin.sensors["vel_EE"]
        self.assertTrue(s.config.frame_id)
        msg = s.data
        adr = sensor_adr(env, "vel_EE")
        # Truth-aligned: no noise registered, so data equals sensordata (float32 cast).
        self.assertAlmostEqual(msg.vector.x, env.data.sensordata[adr], delta=1e-4)
        self.assertAlmostEqual(msg.vector.y, env.data.sensordata[adr + 1], delta=1e-4)
        self.assertAlmostEqual(msg.vector.z, env.data.sensordata[adr + 2], delta=1e-4)
        self.assertEqual(msg.header.frame_id, s.config.frame_id)
        self.assertTrue(abs(msg.vector.x) > 1e-3 or abs(msg.vector.z) > 1e-3, "sensor must actually move")

    def test_gt_data_available_in_train_mode(self):
        env, plugin = self.open_env()
        gt = plugin.sensors["vel_joint2"].gt_data
        self.assertTrue(hasattr(gt, "value"))

    def run_isolated(self, inner_name):
        # ROS params reach the node only through the first MujocoEnv of a process (argv params-file),
        # so tests that need their own params run in a fresh interpreter.
        proc = subprocess.run(
            [sys.executable, __file__, "--inner", inner_name], capture_output=True, text=True, timeout=120
        )
        self.assertEqual(0, proc.returncode, proc.stdout[-3000:] + proc.stderr[-3000:])

    def test_gt_data_unavailable_in_eval_mode(self):
        self.run_isolated("inner_gt_data_unavailable_in_eval_mode")

    def test_data_ignores_update_rate(self):
        self.run_isolated("inner_data_ignores_update_rate")

    def inner_gt_data_unavailable_in_eval_mode(self):
        env, plugin = self.open_env(eval_mode=True)
        self.assertTrue(env.settings.eval_mode)
        with self.assertRaisesRegex(RuntimeError, "eval mode"):
            _ = plugin.sensors["vel_EE"].gt_data
        # Noisy data stays readable in eval mode.
        _ = plugin.sensors["vel_EE"].data

    def inner_data_ignores_update_rate(self):
        # 1 Hz would publish ~once per 1000 steps of 1 ms; Python must still see every step.
        env, plugin = self.open_env(
            parameters={"MujocoPlugins.mujoco_ros_sensors.vel_joint2.update_rate": 1.0}
        )
        s = plugin.sensors["vel_joint2"]
        self.assertEqual(s.config.update_rate, 1.0)
        adr = sensor_adr(env, "vel_joint2")
        env.data.qvel[env.model.jnt_dofadr[env.model.joint("joint2").id]] = 1.0
        seen = []
        for _ in range(5):
            env.step(1)
            seen.append(s.data.value)
            self.assertAlmostEqual(seen[-1], env.data.sensordata[adr], delta=1e-4)
        self.assertGreater(len(set(seen)), 1, "value must change step to step")

    def test_removed_configs_api(self):
        env, plugin = self.open_env()
        self.assertFalse(hasattr(plugin, "configs"))
        self.assertFalse(hasattr(plugin, "configs_dict"))

    def test_dict_like_map(self):
        env, plugin = self.open_env()
        self.assertIn("vel_EE", plugin.sensors)
        self.assertNotIn("nope", plugin.sensors)
        self.assertIn("vel_EE", list(plugin.sensors))
        self.assertEqual(len(plugin.sensors), len(plugin.sensors.keys()))
        with self.assertRaises(KeyError):
            plugin.sensors["nope"]

    def test_two_data_reads_same_step_equal_with_noise(self):
        env, plugin = self.open_env()
        register_noise(env, "vel_joint2", sigma=1.0)
        s = plugin.sensors["vel_joint2"]
        env.step(1)
        first = s.data
        self.assertEqual(first, s.data)
        self.assertEqual(first.value, s.data.value)
        self.assertNotEqual(first.value, s.gt_data.value, "noise must be applied")
        self.assertEqual(s.gt_data, s.gt_data)
        env.step(1)
        self.assertNotEqual(first.value, s.data.value, "new step draws new noise")

    def test_held_stream_fails_loud_after_reload(self):
        env, plugin = self.open_env()
        held = plugin.sensors["vel_EE"]
        self.assertTrue(env.load_from_path(str(MODEL)))
        wait_for_idle(env)
        for attr in ("config", "data", "gt_data"):
            with self.assertRaises((RuntimeError, KeyError)):
                getattr(held, attr)


if __name__ == "__main__":
    result = None
    try:
        if len(sys.argv) == 3 and sys.argv[1] == "--inner":
            result = unittest.TextTestRunner().run(SensorStreamPythonTest(sys.argv[2]))
        else:
            result = unittest.main(argv=[sys.argv[0]], exit=False).result
    finally:
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        except ImportError:
            pass
    sys.exit(0 if result is not None and result.wasSuccessful() else 1)
