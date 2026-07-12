#!/usr/bin/env python3

from pathlib import Path
import sys
import time
import unittest

import pymujoco_ros

from mujoco_ros import MujocoEnv
from mujoco_ros import RosCore


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


def service_name(env, name):
    return f"{env.handle_namespace}/{name}"


def wait_for_idle(env, timeout=5.0):
    deadline = time.monotonic() + timeout
    while env.operational_status != 0 and time.monotonic() < deadline:
        time.sleep(0.01)
    if env.operational_status != 0:
        raise AssertionError("environment did not become idle before timeout")


def call_service(env, name, srv_type, **fields):
    full_name = service_name(env, name)
    if is_ros1():
        import rospy

        rospy.wait_for_service(full_name, timeout=5.0)
        proxy = rospy.ServiceProxy(full_name, srv_type)
        return proxy(**fields)

    import rclpy

    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node("python_bindings_service_client")
    try:
        client = node.create_client(srv_type, full_name)
        if not client.wait_for_service(timeout_sec=5.0):
            raise AssertionError(f"service {full_name} did not become available")
        request = srv_type.Request()
        for key, value in fields.items():
            setattr(request, key, value)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result() is None:
            raise AssertionError(f"service call {full_name} did not return a result")
        return future.result()
    finally:
        node.destroy_node()


def shutdown_rclpy_if_needed():
    try:
        import rclpy
    except ImportError:
        return
    if rclpy.ok():
        rclpy.shutdown()


class PythonBindingsTest(unittest.TestCase):
    def test_native_module_exposes_version(self):
        self.assertTrue(pymujoco_ros.__mujoco_version__)

    def test_existing_ros_context_is_accepted(self):
        with RosCore(managed=False) as core:
            self.assertFalse(core.owns_core)

    def test_loads_multiple_worlds_and_reports_status(self):
        worlds = [
            get_package_share_directory("mujoco_ros") / "assets" / "pendulum_world.xml",
            get_package_share_directory("mujoco_ros_testing_utils") / "assets" / "empty_world.xml",
            get_package_share_directory("mujoco_ros_testing_utils")
            / "assets"
            / "equality_world.xml",
            get_package_share_directory("mujoco_ros_testing_utils")
            / "assets"
            / "camera_world.xml",
        ]

        with MujocoEnv() as env:
            previous_load_count = env.load_count
            for world in worlds:
                self.assertTrue(env.load_model_from_string(world))
                wait_for_idle(env)

                settings = env.settings
                sim_info = env.sim_info

                self.assertTrue(settings.headless)
                if pymujoco_ros.__render_backend__ == "NONE":
                    self.assertFalse(settings.render_offscreen)
                if is_ros1():
                    self.assertTrue(settings.use_sim_time)
                self.assertTrue(sim_info.model_valid)
                self.assertGreater(env.load_count, previous_load_count)
                self.assertGreaterEqual(sim_info.load_count, env.load_count)
                self.assertIn(world.name, sim_info.model_path)
                self.assertIn(world.name, env.filename)
                self.assertEqual([], env.plugins)
                self.assertEqual([], env.plugin_names)
                self.assertEqual([], env.plugin_stats)
                previous_load_count = env.load_count

    def test_python_api_mutations_reach_cpp_state(self):
        model_path = get_package_share_directory("mujoco_ros") / "assets" / "pendulum_world.xml"

        with MujocoEnv() as env:
            self.assertTrue(env.load_model_from_string(str(model_path)))
            wait_for_idle(env)

            self.assertTrue(env.pause())
            self.assertFalse(env.is_running)
            self.assertTrue(env.unpause())
            self.assertTrue(env.is_running)
            self.assertTrue(env.pause())

            self.assertTrue(env.set_rt_factor(0.5))
            self.assertAlmostEqual(env.sim_info.rt_setting, 0.5, delta=0.01)

            self.assertTrue(env.set_gravity([0.0, 0.0, -3.21]))
            self.assertSequenceAlmostEqual(env.get_gravity(), [0.0, 0.0, -3.21])

            self.assertTrue(env.step(100))
            self.assertTrue(env.sim_info.model_valid)

            sim_state = env.sim_state
            self.assertTrue(sim_state.model_valid)
            self.assertGreater(sim_state.measured_slowdown, 0.0)

    def test_runtime_settings_proxy_writes_through_cpp(self):
        model_path = get_package_share_directory("mujoco_ros") / "assets" / "pendulum_world.xml"

        with MujocoEnv() as env:
            self.assertTrue(env.load_model_from_string(str(model_path)))
            wait_for_idle(env)

            env.settings.running = True
            self.assertTrue(env.is_running)
            env.settings.run = False
            self.assertFalse(env.is_running)

            env.settings.rt_factor = 0.25
            self.assertAlmostEqual(env.settings.rt_factor, 0.25, delta=0.01)

            env.settings.busywait = 1
            self.assertEqual(1, env.settings.busywait)

            env.settings.gravity = [0.0, 0.0, -4.56]
            self.assertSequenceAlmostEqual(env.settings.gravity, [0.0, 0.0, -4.56])

            snapshot = env.settings.snapshot()
            self.assertTrue(snapshot.headless)
            self.assertFalse(hasattr(env, "settings_snapshot"))

            with self.assertRaises(AttributeError):
                env.settings.headless = False

    def test_pathlike_inputs_are_accepted(self):
        require_python_mujoco()
        model_path = get_package_share_directory("mujoco_ros") / "assets" / "pendulum_world.xml"

        with MujocoEnv(model_path=model_path) as env:
            wait_for_idle(env)
            self.assertTrue(env.model_valid)

            self.assertTrue(env.load_from_path(model_path))
            wait_for_idle(env)
            self.assertIn("pendulum_world.xml", env.filename)

            self.assertTrue(env.load_model_from_string(model_path))
            wait_for_idle(env)
            self.assertIn("pendulum_world.xml", env.filename)

    def test_python_owned_models_load_from_path_and_xml_string(self):
        require_python_mujoco()
        model_path = get_package_share_directory("mujoco_ros") / "assets" / "pendulum_world.xml"

        with MujocoEnv() as env:
            self.assertTrue(env.load_from_path(str(model_path)))
            wait_for_idle(env)
            self.assertIsNotNone(env.model)
            self.assertIsNotNone(env.data)
            self.assertTrue(env.model_valid)
            self.assertIn("pendulum_world.xml", env.filename)
            self.assertTrue(env.pause())
            self.assertTrue(env.step(10))

            with open(model_path, "r", encoding="utf-8") as stream:
                model_xml = stream.read()
            previous_load_count = env.load_count
            self.assertTrue(env.load_from_string(model_xml, filename="python_string_model.xml"))
            wait_for_idle(env)
            self.assertGreater(env.load_count, previous_load_count)
            self.assertIn("python_string_model.xml", env.filename)

    def test_python_reload_service_uses_python_model_loader(self):
        require_python_mujoco()
        from mujoco_ros_msgs.srv import Reload

        model_path = get_package_share_directory("mujoco_ros") / "assets" / "pendulum_world.xml"
        empty_world = (
            get_package_share_directory("mujoco_ros_testing_utils") / "assets" / "empty_world.xml"
        )

        with MujocoEnv(model_path=str(model_path), python_reload_service=True) as env:
            wait_for_idle(env)
            previous_load_count = env.load_count

            same_response = call_service(env, "reload", Reload, model="", admin_hash="")
            self.assertTrue(same_response.success, same_response.status_message)
            wait_for_idle(env)
            self.assertGreater(env.load_count, previous_load_count)
            self.assertIsNotNone(env.model)
            self.assertIsNotNone(env.data)

            new_response = call_service(
                env, "reload", Reload, model=str(empty_world), admin_hash=""
            )
            self.assertTrue(new_response.success, new_response.status_message)
            wait_for_idle(env)
            self.assertIn("empty_world.xml", env.filename)

    def test_ros_services_match_python_accessors(self):
        from mujoco_ros_msgs.srv import GetGravity
        from mujoco_ros_msgs.srv import GetSimInfo
        from mujoco_ros_msgs.srv import Reload
        from mujoco_ros_msgs.srv import SetFloat
        from mujoco_ros_msgs.srv import SetGravity
        from mujoco_ros_msgs.srv import SetPause

        model_path = get_package_share_directory("mujoco_ros") / "assets" / "pendulum_world.xml"
        empty_world = (
            get_package_share_directory("mujoco_ros_testing_utils") / "assets" / "empty_world.xml"
        )

        with MujocoEnv() as env:
            self.assertTrue(env.load_model_from_string(str(model_path)))
            wait_for_idle(env)

            pause_response = call_service(env, "set_pause", SetPause, paused=True, admin_hash="")
            self.assertTrue(pause_response.success)
            self.assertFalse(env.is_running)
            self.assertTrue(env.sim_info.paused)

            unpause_response = call_service(
                env, "set_pause", SetPause, paused=False, admin_hash=""
            )
            self.assertTrue(unpause_response.success)
            self.assertTrue(env.is_running)
            self.assertFalse(env.sim_info.paused)

            rt_response = call_service(env, "set_rt_factor", SetFloat, value=0.25, admin_hash="")
            self.assertTrue(rt_response.success)
            sim_info_response = call_service(env, "get_sim_info", GetSimInfo)
            self.assertAlmostEqual(
                sim_info_response.state.rt_setting, env.sim_info.rt_setting, delta=0.01
            )

            self.assertTrue(env.set_rt_factor(0.5))
            sim_info_response = call_service(env, "get_sim_info", GetSimInfo)
            self.assertAlmostEqual(sim_info_response.state.rt_setting, 0.5, delta=0.01)

            gravity_response = call_service(
                env, "set_gravity", SetGravity, gravity=[0.0, 0.0, -1.23], admin_hash=""
            )
            self.assertTrue(gravity_response.success)
            self.assertSequenceAlmostEqual(env.get_gravity(), [0.0, 0.0, -1.23])

            self.assertTrue(env.set_gravity([0.0, 0.0, -2.34]))
            get_gravity_response = call_service(env, "get_gravity", GetGravity, admin_hash="")
            self.assertSequenceAlmostEqual(get_gravity_response.gravity, env.get_gravity())

            previous_load_count = env.sim_info.load_count
            reload_response = call_service(
                env, "reload", Reload, model=str(empty_world), admin_hash=""
            )
            self.assertTrue(reload_response.success)
            wait_for_idle(env)
            self.assertIn("empty_world.xml", env.sim_info.model_path)
            self.assertGreater(env.sim_info.load_count, previous_load_count)

    def test_viewer_binding_reports_unsupported_modes_without_blocking(self):
        with MujocoEnv() as env:
            with self.assertRaises(RuntimeError):
                env.attach_viewer(active=False)
            if pymujoco_ros.__render_backend__ != "GLFW":
                with self.assertRaises(RuntimeError):
                    env.attach_viewer(active=True)

    def test_offscreen_camera_bindings_expose_camera_metadata_and_buffers(self):
        if pymujoco_ros.__render_backend__ == "NONE":
            raise unittest.SkipTest("offscreen camera bindings require a render backend")
        require_python_mujoco()
        try:
            from mujoco_ros.rendering import OffcamManager
        except ImportError as exc:
            raise unittest.SkipTest(f"offscreen rendering helpers are unavailable: {exc}")

        model_path = (
            get_package_share_directory("mujoco_ros_testing_utils") / "assets" / "camera_world.xml"
        )

        with MujocoEnv(model_path=model_path) as env:
            wait_for_idle(env)
            if not env.settings.render_offscreen:
                raise unittest.SkipTest("offscreen rendering is unavailable at runtime")
            if env.model.ncam == 0:
                raise unittest.SkipTest("camera test world has no cameras")

            manager = OffcamManager(env.binding._offscreen_context, env.model, cam_buff_size=2)
            self.assertGreaterEqual(manager.num_cams, 1)

            cam = manager.cam(0)
            self.assertIsNotNone(cam)
            self.assertGreater(cam.width, 0)
            self.assertGreater(cam.height, 0)
            self.assertIs(manager.cam(cam.cam_name), cam)

            self.assertTrue(env.pause())
            self.assertTrue(env.step(20))
            rgb, depth, segment = cam.get_buffered_frames()
            if rgb is not None:
                self.assertLessEqual(rgb.shape[0], 2)
                self.assertEqual((cam.height, cam.width, 3), rgb.shape[1:])
            if depth is not None:
                self.assertLessEqual(depth.shape[0], 2)
                self.assertEqual((cam.height, cam.width), depth.shape[1:])
            if segment is not None:
                self.assertLessEqual(segment.shape[0], 2)
                self.assertEqual((cam.height, cam.width, 3), segment.shape[1:])

            del cam
            del manager

    def assertSequenceAlmostEqual(self, actual, expected, delta=1e-6):
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, delta=delta)


if __name__ == "__main__":
    if is_ros1():
        import rostest

        rostest.rosrun("mujoco_ros", "python_bindings_test", PythonBindingsTest)
    else:
        result = None
        try:
            result = unittest.main(argv=[sys.argv[0]], exit=False).result
        finally:
            shutdown_rclpy_if_needed()
        sys.exit(0 if result is not None and result.wasSuccessful() else 1)
