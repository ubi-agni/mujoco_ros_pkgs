#!/usr/bin/env python3

from pathlib import Path
from contextlib import contextmanager
import faulthandler
import itertools
import os
import sys
import threading
import time
import unittest

import numpy as np
import pymujoco_ros

from mujoco_ros import MujocoEnv
from mujoco_ros import RosCore
from mujoco_ros import viewer

_service_client_ids = itertools.count()


@contextmanager
def service_client_node():
    if is_ros1():
        yield None
        return

    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node(f"python_bindings_service_client_{next(_service_client_ids)}")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    node._mujoco_service_executor = executor
    try:
        yield node
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()


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


def native_display_is_advertised():
    return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))


def require_visible_viewer():
    if pymujoco_ros.__viewer_backend__ != 'GLFW':
        raise unittest.SkipTest('visible viewer requires WITH_GUI=ON')
    if not native_display_is_advertised():
        raise unittest.SkipTest('native display is unavailable')


def service_name(env, name):
    return f"{env.handle_namespace}/{name}"


def wait_for_idle(env, timeout=5.0):
    deadline = time.monotonic() + timeout
    while env.operational_status != 0 and time.monotonic() < deadline:
        time.sleep(0.01)
    if env.operational_status != 0:
        raise AssertionError('environment did not become idle before timeout')


def assert_other_thread_runs_during_blocking_call(blocking_call):
    worker_at_barrier = threading.Event()
    continue_worker = threading.Event()
    progress = threading.Event()

    def worker():
        worker_at_barrier.set()
        continue_worker.wait(timeout=2.0)
        progress.set()

    worker_thread = threading.Thread(target=worker)
    worker_thread.start()
    assert worker_at_barrier.wait(timeout=2.0), 'worker did not reach blocked section'
    blocking_thread = threading.Thread(target=blocking_call)
    blocking_thread.start()
    continue_worker.set()
    blocking_thread.join(timeout=10.0)
    worker_thread.join(timeout=2.0)
    assert not blocking_thread.is_alive(), 'blocking call did not finish'
    assert progress.is_set(), 'expected another Python thread to run during blocking call'


def call_service(env, name, srv_type, client_node=None, **fields):
    full_name = service_name(env, name)
    if is_ros1():
        import rospy

        rospy.wait_for_service(full_name, timeout=5.0)
        proxy = rospy.ServiceProxy(full_name, srv_type)
        return proxy(**fields)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    if not rclpy.ok():
        rclpy.init()

    owns_node = client_node is None
    node = client_node or rclpy.create_node(
        f"python_bindings_service_client_{next(_service_client_ids)}"
    )
    executor = getattr(node, "_mujoco_service_executor", None)
    owns_executor = executor is None
    if owns_executor:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
    try:
        client = node.create_client(srv_type, full_name)
        if not client.wait_for_service(timeout_sec=5.0):
            raise AssertionError(f"service {full_name} did not become available")
        request = srv_type.Request()
        for key, value in fields.items():
            setattr(request, key, value)
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            raise AssertionError(f"service call {full_name} did not return a result")
        return result
    finally:
        if owns_executor:
            executor.remove_node(node)
            executor.shutdown()
        if owns_node:
            node.destroy_node()


def shutdown_rclpy_if_needed():
    try:
        import rclpy
    except ImportError:
        return
    if rclpy.ok():
        rclpy.shutdown()


class PythonBindingsTest(unittest.TestCase):
    def _arm_close_watchdog(self, seconds: float = 10.0) -> None:
        """Arm a faulthandler watchdog for a blocking call.

        Under ROS 1 rostest, ``sys.stderr`` is a pipe/StringIO without a real file
        descriptor, so the default target raises ``UnsupportedOperation: fileno``.
        Target the original stderr (``sys.__stderr__``), which always has a fileno.
        """
        faulthandler.dump_traceback_later(seconds, exit=True, file=sys.__stderr__)

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

            import mujoco

            original_energy = env.runtime_options.energy
            original_sensor_disabled = env.runtime_options.sensor_disabled
            try:
                env.set_enableflag(mujoco.mjtEnableBit.mjENBL_ENERGY, True)
                self.assertTrue(env.runtime_options.energy)
                unchanged = env.runtime_options
                env.set_enableflag(mujoco.mjtEnableBit.mjENBL_ENERGY, True)
                self.assertEqual(env.runtime_options, unchanged)
                env.set_enableflag(mujoco.mjtEnableBit.mjENBL_ENERGY, False)
                self.assertFalse(env.runtime_options.energy)
                env.toggle_enableflag(mujoco.mjtEnableBit.mjENBL_ENERGY)
                self.assertTrue(env.runtime_options.energy)

                env.set_disableflag(mujoco.mjtDisableBit.mjDSBL_SENSOR, True)
                self.assertTrue(env.runtime_options.sensor_disabled)
                unchanged = env.runtime_options
                env.set_disableflag(mujoco.mjtDisableBit.mjDSBL_SENSOR, True)
                self.assertEqual(env.runtime_options, unchanged)
                env.set_disableflag(mujoco.mjtDisableBit.mjDSBL_SENSOR, False)
                self.assertFalse(env.runtime_options.sensor_disabled)
                env.toggle_disableflag(mujoco.mjtDisableBit.mjDSBL_SENSOR)
                self.assertTrue(env.runtime_options.sensor_disabled)
                env.toggle_disableflag(mujoco.mjtDisableBit.mjDSBL_SENSOR)
                self.assertFalse(env.runtime_options.sensor_disabled)
            finally:
                env.apply_runtime_options(
                    {
                        'energy': original_energy,
                        'sensor_disabled': original_sensor_disabled,
                    }
                )

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
            env.settings.running = False
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

    def test_render_backpressure_policy_uses_canonical_values(self):
        with MujocoEnv() as env:
            env.settings.render_backpressure_policy = "wait_for_slot"
            self.assertEqual(env.render_backpressure_policy, "wait_for_slot")
            self.assertEqual(env.settings.render_backpressure_policy, "wait_for_slot")

            with self.assertRaises(ValueError):
                env.settings.render_backpressure_policy = "wait"
            self.assertEqual(env.render_backpressure_policy, "wait_for_slot")

    def test_runtime_options_snapshot_apply_and_rollback(self):
        model_path = get_package_share_directory("mujoco_ros") / "assets" / "pendulum_world.xml"

        with MujocoEnv(runtime_options={"timestep": 0.002, "iterations": 50}) as env:
            self.assertTrue(env.load_model_from_string(str(model_path)))
            wait_for_idle(env)

            before = env.runtime_options
            self.assertAlmostEqual(before.timestep, 0.002)
            self.assertEqual(before.iterations, 50)

            applied = env.apply_runtime_options({"timestep": 0.003})
            self.assertAlmostEqual(applied.timestep, 0.003)

            with self.assertRaisesRegex(ValueError, "solimp"):
                env.apply_runtime_options({"timestep": 0.004, "solimp": "0.9 0.95"})
            self.assertAlmostEqual(env.runtime_options.timestep, 0.003)
            self.assertEqual(env.runtime_options, applied)

    def test_runtime_options_reject_invalid_startup_values(self):
        with self.assertRaisesRegex(ValueError, "solimp"):
            MujocoEnv(runtime_options={"solimp": "0.9 0.95"})

    def test_legacy_control_settings_are_rejected(self):
        model_path = get_package_share_directory("mujoco_ros") / "assets" / "pendulum_world.xml"

        with MujocoEnv() as env:
            self.assertTrue(env.load_model_from_string(str(model_path)))
            wait_for_idle(env)

            legacy_fields = (
                "run",
                "exit_request",
                "load_request",
                "reset_request",
                "speed_changed",
                "env_steps_request",
                "real_time_index",
            )
            for field in legacy_fields:
                with self.assertRaises(AttributeError):
                    setattr(env.settings, field, 1)

            env.settings.running = True
            self.assertTrue(env.is_running)
            env.settings.running = False
            self.assertFalse(env.is_running)

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

            with service_client_node() as client_node:
                same_response = call_service(
                    env, "reload", Reload, client_node=client_node, model="", admin_hash=""
                )
                self.assertTrue(same_response.success, same_response.status_message)
                wait_for_idle(env)
                self.assertGreater(env.load_count, previous_load_count)
                self.assertIsNotNone(env.model)
                self.assertIsNotNone(env.data)

                new_response = call_service(
                    env,
                    "reload",
                    Reload,
                    client_node=client_node,
                    model=str(empty_world),
                    admin_hash="",
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

            with service_client_node() as client_node:
                pause_response = call_service(
                    env, "set_pause", SetPause, client_node=client_node, paused=True, admin_hash=""
                )
                self.assertTrue(pause_response.success)
                self.assertFalse(env.is_running)
                self.assertTrue(env.sim_info.paused)

                unpause_response = call_service(
                    env,
                    "set_pause",
                    SetPause,
                    client_node=client_node,
                    paused=False,
                    admin_hash="",
                )
                self.assertTrue(unpause_response.success)
                self.assertTrue(env.is_running)
                self.assertFalse(env.sim_info.paused)

                rt_response = call_service(
                    env,
                    "set_rt_factor",
                    SetFloat,
                    client_node=client_node,
                    value=0.25,
                    admin_hash="",
                )
                self.assertTrue(rt_response.success)
                sim_info_response = call_service(
                    env, "get_sim_info", GetSimInfo, client_node=client_node
                )
                self.assertAlmostEqual(
                    sim_info_response.state.rt_setting, env.sim_info.rt_setting, delta=0.01
                )

                self.assertTrue(env.set_rt_factor(0.5))
                sim_info_response = call_service(
                    env, 'get_sim_info', GetSimInfo, client_node=client_node
                )
                self.assertAlmostEqual(sim_info_response.state.rt_setting, 0.5, delta=0.01)

                gravity_response = call_service(
                    env,
                    'set_gravity',
                    SetGravity,
                    client_node=client_node,
                    gravity=[0.0, 0.0, -1.23],
                    admin_hash='',
                )
                self.assertTrue(gravity_response.success)
                self.assertSequenceAlmostEqual(env.get_gravity(), [0.0, 0.0, -1.23])

                self.assertTrue(env.set_gravity([0.0, 0.0, -2.34]))
                get_gravity_response = call_service(
                    env, 'get_gravity', GetGravity, client_node=client_node, admin_hash=''
                )
                self.assertSequenceAlmostEqual(get_gravity_response.gravity, env.get_gravity())

                previous_load_count = env.sim_info.load_count
                reload_response = call_service(
                    env,
                    'reload',
                    Reload,
                    client_node=client_node,
                    model=str(empty_world),
                    admin_hash='',
                )
                self.assertTrue(reload_response.success)
                wait_for_idle(env)
                self.assertIn('empty_world.xml', env.sim_info.model_path)
                self.assertGreater(env.sim_info.load_count, previous_load_count)

    def test_public_viewer_module_exports_mujoco_shaped_calls(self):
        self.assertTrue(callable(viewer.launch))
        self.assertTrue(callable(viewer.launch_passive))

    def test_public_passive_viewer_lifecycle_and_environment_shutdown(self):
        require_visible_viewer()
        with MujocoEnv() as env:
            handle = viewer.launch_passive(env, auto_sync=True)
            self.assertTrue(handle.is_running())
        self.assertFalse(handle.is_running())

    def test_auto_sync_viewer_allows_unpaused_binding_mutations(self):
        require_visible_viewer()
        model_path = get_package_share_directory('mujoco_ros') / 'assets' / 'pendulum_world.xml'
        with MujocoEnv(model_path=model_path) as env:
            wait_for_idle(env)
            with viewer.launch_passive(env, auto_sync=True) as handle:
                self.assertTrue(env.unpause())
                data = env.data
                start_time = data.time
                deadline = time.monotonic() + 2.0
                while data.time <= start_time and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertGreater(data.time, start_time)

                self.assertTrue(env.pause())
                self.assertTrue(env.set_gravity([0.0, 0.0, -3.21]))
                self.assertSequenceAlmostEqual(env.get_gravity(), [0.0, 0.0, -3.21])
                env.apply_runtime_options({'timestep': 0.003})
                self.assertAlmostEqual(env.runtime_options.timestep, 0.003)
                self.assertTrue(env.step(5))
                self.assertTrue(handle.is_running())

    def test_manual_passive_viewer_keeps_explicit_sync(self):
        require_visible_viewer()
        model_path = get_package_share_directory('mujoco_ros') / 'assets' / 'pendulum_world.xml'
        with MujocoEnv(model_path=model_path) as env:
            wait_for_idle(env)
            with viewer.launch_passive(env, auto_sync=False) as handle:
                self.assertTrue(env.set_gravity([0.0, 0.0, -3.21]))
                self.assertSequenceAlmostEqual(env.get_gravity(), [0.0, 0.0, -3.21])
                with handle.lock():
                    handle.sync()
                self.assertTrue(handle.is_running())

    def test_attach_viewer_is_a_deprecated_alias(self):
        require_visible_viewer()
        with MujocoEnv() as env:
            with self.assertWarnsRegex(DeprecationWarning, 'mujoco_ros.viewer'):
                handle = env.attach_viewer(active=False)
            self.assertTrue(handle.is_running())
            handle.close()

    def test_viewer_backend_reports_visible_gui_independently_from_offscreen(self):
        self.assertIn(pymujoco_ros.__viewer_backend__, ('GLFW', 'NONE'))

    def test_viewer_launch_refuses_build_without_gui(self):
        if pymujoco_ros.__viewer_backend__ == 'GLFW':
            raise unittest.SkipTest('refusal contract requires WITH_GUI=OFF')
        with MujocoEnv() as env:
            with self.assertRaisesRegex(RuntimeError, 'WITH_GUI=ON'):
                env.binding._launch_viewer()
            with self.assertRaisesRegex(RuntimeError, 'WITH_GUI=ON'):
                env.binding._launch_passive()

    def test_passive_launch_propagates_missing_display(self):
        if pymujoco_ros.__viewer_backend__ != 'GLFW':
            raise unittest.SkipTest('startup propagation requires WITH_GUI=ON')
        if native_display_is_advertised():
            raise unittest.SkipTest('missing-display contract requires no display')
        with MujocoEnv() as env:
            with self.assertRaisesRegex(RuntimeError, 'GLFW|display|monitor'):
                env.binding._launch_passive()

    def test_native_passive_handle_lifecycle(self):
        require_visible_viewer()
        model_path = get_package_share_directory('mujoco_ros') / 'assets' / 'pendulum_world.xml'
        with MujocoEnv(model_path=model_path) as env:
            handle = viewer.launch_passive(env, auto_sync=False)
            self.assertTrue(handle.is_running())
            with handle.lock():
                handle.sync()
            self._arm_close_watchdog()
            try:
                handle.close()
            finally:
                faulthandler.cancel_dump_traceback_later()
            self.assertFalse(handle.is_running())
            handle.close()

    def test_native_passive_handle_context_and_double_launch(self):
        require_visible_viewer()
        with MujocoEnv() as env:
            self._arm_close_watchdog()
            try:
                with viewer.launch_passive(env, auto_sync=True) as handle:
                    self.assertTrue(handle.is_running())
                    with self.assertRaisesRegex(
                        RuntimeError, 'a viewer is already running for this MujocoEnv'
                    ):
                        viewer.launch_passive(env, auto_sync=True)
                    with self.assertRaisesRegex(
                        RuntimeError, 'a viewer is already running for this MujocoEnv'
                    ):
                        viewer.launch(env)
            finally:
                faulthandler.cancel_dump_traceback_later()
            self.assertFalse(handle.is_running())
            self.assertTrue(env.settings.headless)
            self.assertIsNotNone(env.settings)

    def test_passive_relaunch_waits_for_prior_teardown(self):
        require_visible_viewer()
        with MujocoEnv() as env:
            first = env.binding._launch_passive(auto_sync=False)
            self.assertTrue(first.is_running())
            first.close()
            self.assertFalse(first.is_running())
            second = env.binding._launch_passive(auto_sync=False)
            try:
                self.assertTrue(second.is_running())
            finally:
                second.close()

    def test_close_while_holding_viewer_lock_fails_loud(self):
        require_visible_viewer()
        with MujocoEnv() as env:
            handle = env.binding._launch_passive(auto_sync=False)
            with handle.lock():
                with self.assertRaisesRegex(
                    RuntimeError, 'cannot close viewer while holding viewer lock'
                ):
                    handle.close()
            handle.close()

    def test_shutdown_rethrows_retained_passive_viewer_error(self):
        if pymujoco_ros.__viewer_backend__ != 'GLFW':
            raise unittest.SkipTest('startup propagation requires WITH_GUI=ON')
        if native_display_is_advertised():
            raise unittest.SkipTest('missing-display contract requires no display')
        env = MujocoEnv()
        try:
            with self.assertRaisesRegex(RuntimeError, 'GLFW|display|monitor'):
                env.binding._launch_passive()
            with self.assertRaisesRegex(RuntimeError, 'GLFW|display|monitor'):
                env.shutdown()
        finally:
            if getattr(env, '_env', None) is not None:
                try:
                    env.shutdown()
                except RuntimeError:
                    pass

    def test_sync_releases_gil_for_other_python_threads(self):
        require_visible_viewer()
        with MujocoEnv() as env:
            handle = env.binding._launch_passive(auto_sync=False)
            try:
                assert_other_thread_runs_during_blocking_call(lambda: handle.sync())
            finally:
                handle.close()

    def test_nested_viewer_lock_close_fails_loud(self):
        require_visible_viewer()
        with MujocoEnv() as env:
            handle = env.binding._launch_passive(auto_sync=False)
            lock1 = handle.lock()
            lock2 = handle.lock()
            lock1.__enter__()
            lock2.__enter__()
            try:
                with self.assertRaisesRegex(
                    RuntimeError, 'cannot close viewer while holding viewer lock'
                ):
                    handle.close()
            finally:
                lock2.__exit__(None, None, None)
                lock1.__exit__(None, None, None)
                handle.close()

    def test_shutdown_while_holding_nested_viewer_lock_preserves_model(self):
        require_visible_viewer()
        model_path = (
            get_package_share_directory('mujoco_ros_testing_utils')
            / 'assets'
            / 'pendulum_world.xml'
        )
        env = MujocoEnv(model_path=str(model_path))
        try:
            self.assertTrue(env.binding.model_valid)
            handle = env.binding._launch_passive(auto_sync=False)
            lock1 = handle.lock()
            lock2 = handle.lock()
            lock1.__enter__()
            lock2.__enter__()
            try:
                with self.assertRaisesRegex(
                    RuntimeError, 'cannot close viewer while holding viewer lock'
                ):
                    env.binding.shutdown()
                self.assertTrue(env.binding.model_valid)
            finally:
                lock2.__exit__(None, None, None)
                lock1.__exit__(None, None, None)
                handle.close()
        finally:
            env.shutdown()

    def test_passive_viewer_model_reload_completes_without_deadlock(self):
        require_visible_viewer()
        pendulum_path = (
            get_package_share_directory('mujoco_ros_testing_utils')
            / 'assets'
            / 'pendulum_world.xml'
        )
        empty_path = (
            get_package_share_directory('mujoco_ros_testing_utils') / 'assets' / 'empty_world.xml'
        )
        with MujocoEnv(model_path=str(pendulum_path)) as env:
            handle = env.binding._launch_passive(auto_sync=False)
            try:
                self.assertTrue(handle.is_running())
                reload_error = []

                def reload_model():
                    try:
                        self.assertTrue(env.load_model_from_string(str(empty_path)))
                        wait_for_idle(env)
                    except Exception as exc:  # noqa: BLE001 - capture for assertion
                        reload_error.append(exc)

                reload_thread = threading.Thread(target=reload_model)
                reload_thread.start()
                reload_thread.join(timeout=10.0)
                self.assertFalse(
                    reload_thread.is_alive(),
                    'model reload deadlocked with passive viewer',
                )
                self.assertEqual(reload_error, [])
                self.assertTrue(handle.is_running())
                self.assertTrue(env.binding.model_valid)
            finally:
                handle.close()

    def test_model_reload_during_passive_viewer_close_does_not_deadlock(self):
        require_visible_viewer()
        pendulum_path = (
            get_package_share_directory('mujoco_ros_testing_utils')
            / 'assets'
            / 'pendulum_world.xml'
        )
        empty_path = (
            get_package_share_directory('mujoco_ros_testing_utils') / 'assets' / 'empty_world.xml'
        )
        with MujocoEnv(model_path=str(pendulum_path)) as env:
            handle = env.binding._launch_passive(auto_sync=False)
            reload_errors = []
            reload_results = []
            close_errors = []

            def reload_model():
                try:
                    reload_results.append(env.load_model_from_string(str(empty_path)))
                    wait_for_idle(env)
                except Exception as exc:  # noqa: BLE001 - capture for assertion
                    reload_errors.append(exc)

            def close_viewer():
                try:
                    handle.close()
                except Exception as exc:  # noqa: BLE001 - capture for assertion
                    close_errors.append(exc)

            reload_thread = threading.Thread(target=reload_model)
            close_thread = threading.Thread(target=close_viewer)
            reload_thread.start()
            close_thread.start()
            reload_thread.join(timeout=10.0)
            close_thread.join(timeout=10.0)
            self.assertFalse(reload_thread.is_alive(), "reload deadlocked during viewer close")
            self.assertFalse(close_thread.is_alive(), "viewer close deadlocked during reload")
            self.assertEqual(reload_errors, [])
            self.assertEqual(reload_results, [True])
            self.assertEqual(close_errors, [])
            self.assertFalse(handle.is_running())
            self.assertTrue(env.binding.model_valid)
            self.assertIn("empty_world.xml", env.sim_info.model_path)

    def test_passive_launch_waits_for_inflight_model_reload(self):
        require_visible_viewer()
        pendulum_path = (
            get_package_share_directory("mujoco_ros_testing_utils")
            / "assets"
            / "pendulum_world.xml"
        )
        empty_path = (
            get_package_share_directory("mujoco_ros_testing_utils") / "assets" / "empty_world.xml"
        )
        with MujocoEnv(model_path=str(pendulum_path)) as env:
            reload_done = threading.Event()
            reload_errors = []

            def reload_model():
                try:
                    self.assertTrue(env.load_model_from_string(str(empty_path)))
                    wait_for_idle(env)
                    reload_done.set()
                except Exception as exc:  # noqa: BLE001 - capture for assertion
                    reload_errors.append(exc)

            reload_thread = threading.Thread(target=reload_model)
            reload_thread.start()
            handle = env.binding._launch_passive(auto_sync=False)
            try:
                reload_thread.join(timeout=10.0)
                self.assertFalse(
                    reload_thread.is_alive(), "model reload did not finish before viewer launch"
                )
                self.assertEqual(reload_errors, [])
                self.assertTrue(reload_done.is_set())
                self.assertTrue(handle.is_running())
                self.assertTrue(env.binding.model_valid)
                self.assertIn("empty_world.xml", env.sim_info.model_path)
            finally:
                handle.close()

    def test_close_from_other_thread_waits_for_viewer_lock_release(self):
        require_visible_viewer()
        with MujocoEnv() as env:
            handle = env.binding._launch_passive(auto_sync=False)
            closed = threading.Event()
            close_error = []

            def close_worker():
                try:
                    handle.close()
                    closed.set()
                except Exception as exc:  # noqa: BLE001 - capture for assertion
                    close_error.append(exc)

            with handle.lock():
                worker = threading.Thread(target=close_worker)
                worker.start()
                time.sleep(0.2)
                self.assertFalse(closed.is_set())
            worker.join(timeout=5.0)
            self.assertEqual(close_error, [])
            self.assertTrue(closed.is_set())
            self.assertFalse(handle.is_running())

    def test_auto_passive_viewer_model_reload_completes_without_deadlock(self):
        require_visible_viewer()
        pendulum_path = (
            get_package_share_directory('mujoco_ros_testing_utils')
            / 'assets'
            / 'pendulum_world.xml'
        )
        empty_path = (
            get_package_share_directory('mujoco_ros_testing_utils') / 'assets' / 'empty_world.xml'
        )
        with MujocoEnv(model_path=str(pendulum_path)) as env:
            handle = env.binding._launch_passive(auto_sync=True)
            try:
                self.assertTrue(handle.is_running())
                reload_error = []

                def reload_model():
                    try:
                        self.assertTrue(env.load_model_from_string(str(empty_path)))
                        wait_for_idle(env)
                    except Exception as exc:  # noqa: BLE001 - capture for assertion
                        reload_error.append(exc)

                reload_thread = threading.Thread(target=reload_model)
                reload_thread.start()
                reload_thread.join(timeout=10.0)
                self.assertFalse(
                    reload_thread.is_alive(),
                    'auto-passive model reload deadlocked with passive viewer',
                )
                self.assertEqual(reload_error, [])
                self.assertTrue(handle.is_running())
                self.assertTrue(env.binding.model_valid)
            finally:
                handle.close()

    def test_passive_viewer_without_model_can_close(self):
        require_visible_viewer()
        with MujocoEnv() as env:
            handle = viewer.launch_passive(env, auto_sync=True)
            try:
                self.assertTrue(handle.is_running())
            finally:
                handle.close()
            self.assertFalse(handle.is_running())

    def test_environment_shutdown_during_passive_launch_does_not_hang(self):
        require_visible_viewer()
        model_path = (
            get_package_share_directory('mujoco_ros_testing_utils')
            / 'assets'
            / 'pendulum_world.xml'
        )
        env = MujocoEnv(model_path=str(model_path))
        wait_for_idle(env)
        launch_errors = []
        try:

            def launch_worker():
                try:
                    viewer.launch_passive(env, auto_sync=True)
                except Exception as exc:  # noqa: BLE001 - capture for assertion
                    launch_errors.append(exc)

            launch_thread = threading.Thread(target=launch_worker)
            launch_thread.start()
            time.sleep(0.05)
            try:
                env.shutdown()
            except Exception as exc:  # noqa: BLE001 - capture for assertion
                launch_errors.append(exc)
            launch_thread.join(timeout=10.0)
            self.assertFalse(
                launch_thread.is_alive(), 'passive launch hung during environment shutdown'
            )
        finally:
            if getattr(env, '_env', None) is not None:
                try:
                    env.shutdown()
                except RuntimeError:
                    pass

    def test_passive_launch_during_model_reload_and_shutdown_completes(self):
        require_visible_viewer()
        pendulum_path = (
            get_package_share_directory('mujoco_ros_testing_utils')
            / 'assets'
            / 'pendulum_world.xml'
        )
        empty_path = (
            get_package_share_directory('mujoco_ros_testing_utils') / 'assets' / 'empty_world.xml'
        )
        env = MujocoEnv(model_path=str(pendulum_path))
        wait_for_idle(env)
        errors = []
        handle = None
        try:
            reload_thread = threading.Thread(
                target=lambda: env.load_model_from_string(str(empty_path))
            )
            reload_thread.start()
            try:
                handle = viewer.launch_passive(env, auto_sync=True)
            except Exception as exc:  # noqa: BLE001 - capture for assertion
                errors.append(exc)
            reload_thread.join(timeout=10.0)
            self.assertFalse(reload_thread.is_alive(), 'reload hung during passive launch')
            if handle is not None:
                handle.close()
            try:
                env.shutdown()
            except Exception as exc:  # noqa: BLE001 - capture for assertion
                errors.append(exc)
        finally:
            if getattr(env, '_env', None) is not None:
                try:
                    env.shutdown()
                except RuntimeError:
                    pass

    def test_stale_viewer_handle_lifecycle_after_relaunch(self):
        require_visible_viewer()
        with MujocoEnv() as env:
            first = viewer.launch_passive(env, auto_sync=False)
            first.close()
            self.assertFalse(first.is_running())
            first.close()
            with self.assertRaisesRegex(RuntimeError, 'viewer is not running'):
                first.sync()
            with self.assertRaisesRegex(RuntimeError, 'viewer is not running'):
                with first.lock():
                    pass
            second = viewer.launch_passive(env, auto_sync=False)
            try:
                self.assertTrue(second.is_running())
            finally:
                second.close()

    def test_repeated_close_after_viewer_join_is_idempotent(self):
        require_visible_viewer()
        with MujocoEnv() as env:
            handle = viewer.launch_passive(env, auto_sync=False)
            handle.close()
            self.assertFalse(handle.is_running())
            handle.close()

    def test_load_model_releases_gil_for_other_python_threads(self):
        require_visible_viewer()
        model_path = (
            get_package_share_directory('mujoco_ros_testing_utils')
            / 'assets'
            / 'pendulum_world.xml'
        )
        with MujocoEnv() as env:
            handle = viewer.launch_passive(env, auto_sync=True)
            try:
                assert_other_thread_runs_during_blocking_call(
                    lambda: env.load_from_path(str(model_path))
                )
            finally:
                handle.close()

    def test_step_releases_gil_for_other_python_threads(self):
        require_visible_viewer()
        model_path = (
            get_package_share_directory('mujoco_ros_testing_utils')
            / 'assets'
            / 'pendulum_world.xml'
        )
        with MujocoEnv(model_path=str(model_path)) as env:
            wait_for_idle(env)
            handle = viewer.launch_passive(env, auto_sync=True)
            try:
                assert_other_thread_runs_during_blocking_call(lambda: env.step(5))
            finally:
                handle.close()

    def test_reset_releases_gil_for_other_python_threads(self):
        require_visible_viewer()
        model_path = (
            get_package_share_directory('mujoco_ros_testing_utils')
            / 'assets'
            / 'pendulum_world.xml'
        )
        with MujocoEnv(model_path=str(model_path)) as env:
            wait_for_idle(env)
            handle = viewer.launch_passive(env, auto_sync=True)
            try:
                assert_other_thread_runs_during_blocking_call(env.reset)
            finally:
                handle.close()

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

            manager = OffcamManager(
                env.binding._camera_publication_transport, env.model, cam_buff_size=2
            )
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

    def test_offscreen_camera_uses_direct_read_leases_and_stable_snapshots(self):
        if pymujoco_ros.__render_backend__ == "NONE":
            raise unittest.SkipTest("offscreen camera bindings require a render backend")
        require_python_mujoco()
        from mujoco_ros.rendering import OffcamManager

        model_path = (
            get_package_share_directory("mujoco_ros_testing_utils") / "assets" / "camera_world.xml"
        )
        with MujocoEnv(model_path=model_path) as env:
            wait_for_idle(env)
            if not env.settings.render_offscreen:
                raise unittest.SkipTest("offscreen rendering is unavailable at runtime")

            manager = OffcamManager(
                env.binding._camera_publication_transport, env.model, cam_buff_size=2
            )
            cam = manager.cam(0)
            self.assertTrue(env.pause())
            self.assertTrue(env.step(3))

            borrowed = cam.borrow_latest_rgb()
            with borrowed as view:
                self.assertFalse(view.flags.writeable)
                self.assertGreater(view.size, 0)
                self.assertGreaterEqual(view.capture_id, 1)
                generation = view.frame_generation
                view_copy = view.copy()

            snapshot = cam.get_buffered_frames(last_n=2)[0]
            self.assertIsNotNone(snapshot)
            self.assertLessEqual(snapshot.shape[0], 2)
            self.assertFalse(snapshot.flags.writeable)
            self.assertTrue((view_copy == snapshot[-1]).all())

            snapshot_copy = snapshot.copy()
            self.assertTrue(env.step(2))
            self.assertTrue(np.array_equal(snapshot, snapshot_copy))

            self.assertTrue(env.load_model_from_string(str(model_path)))
            wait_for_idle(env)
            self.assertEqual(generation, view.frame_generation)
            self.assertEqual(view.shape, view_copy.shape)
            self.assertTrue(np.array_equal(view, view_copy))
            self.assertTrue(env.step(2))
            with cam.borrow_latest_rgb() as reloaded_view:
                self.assertNotEqual(generation, reloaded_view.frame_generation)
                self.assertGreater(reloaded_view.capture_id, 0)
            self.assertIsNotNone(cam.get_buffered_frames(last_n=1)[0])

            manager.close()
            for accessor in (
                lambda: cam.get_buffered_frames(last_n=1),
                cam.borrow_latest_rgb,
                cam.borrow_latest_depth,
                cam.borrow_latest_segment,
                lambda: cam.copy_rgb(1),
                lambda: cam.copy_depth(1),
                lambda: cam.copy_segment(1),
            ):
                with self.assertRaisesRegex(RuntimeError, "closed"):
                    accessor()

    def test_offscreen_python_acquisition_rejects_pending_retirement(self):
        if pymujoco_ros.__render_backend__ == "NONE":
            raise unittest.SkipTest("offscreen camera bindings require a render backend")
        require_python_mujoco()
        from mujoco_ros.rendering import OffcamManager

        model_path = (
            get_package_share_directory("mujoco_ros_testing_utils") / "assets" / "camera_world.xml"
        )
        with MujocoEnv(model_path=model_path) as env:
            wait_for_idle(env)
            if not env.settings.render_offscreen:
                raise unittest.SkipTest("offscreen rendering is unavailable at runtime")

            manager = OffcamManager(
                env.binding._camera_publication_transport, env.model, cam_buff_size=2
            )
            cam = manager.cam(0)
            self.assertTrue(env.pause())
            self.assertTrue(env.step(3))
            state = env.binding._camera_publication_transport
            with cam.borrow_latest_rgb() as retained_view:
                retained_copy = retained_view.copy()
                state._set_retirement_pending_for_test(True)
                try:
                    self.assertTrue(np.array_equal(retained_view, retained_copy))
                    for accessor in (cam.borrow_latest_rgb, lambda: cam.copy_rgb(1)):
                        with self.assertRaisesRegex(RuntimeError, "retirement"):
                            accessor()
                finally:
                    state._set_retirement_pending_for_test(False)

            self.assertTrue(env.load_model_from_string(str(model_path)))
            wait_for_idle(env)
            self.assertTrue(env.step(2))
            with cam.borrow_latest_rgb() as rebound_view:
                self.assertGreater(rebound_view.capture_id, 0)
                self.assertNotEqual(retained_view.frame_generation, rebound_view.frame_generation)
            manager.close()

    def test_offscreen_python_buffers_have_independent_demand_and_close(self):
        if pymujoco_ros.__render_backend__ == "NONE":
            raise unittest.SkipTest("offscreen camera bindings require a render backend")
        require_python_mujoco()
        from mujoco_ros.rendering import OffcamManager

        model_path = (
            get_package_share_directory("mujoco_ros_testing_utils") / "assets" / "camera_world.xml"
        )
        with MujocoEnv(model_path=model_path) as env:
            wait_for_idle(env)
            if not env.settings.render_offscreen:
                raise unittest.SkipTest("offscreen rendering is unavailable at runtime")

            first_manager = OffcamManager(
                env.binding._camera_publication_transport, env.model, cam_buff_size=2
            )
            second_manager = OffcamManager(
                env.binding._camera_publication_transport, env.model, cam_buff_size=2
            )
            first_camera = first_manager.cam(0)
            second_camera = second_manager.cam(0)
            self.assertTrue(env.pause())
            self.assertTrue(env.step(3))
            self.assertIsNotNone(first_camera.get_buffered_frames(last_n=1)[0])
            self.assertIsNotNone(second_camera.get_buffered_frames(last_n=1)[0])
            with first_camera.borrow_latest_rgb() as first_view:
                with second_camera.borrow_latest_rgb() as second_view:
                    self.assertEqual(first_view.capture_id, second_view.capture_id)

            first_manager.close()
            self.assertTrue(env.step(2))
            self.assertIsNotNone(second_camera.get_buffered_frames(last_n=1)[0])
            second_manager.close()

    def test_offscreen_camera_legacy_accessors_skip_missing_planes(self):
        if pymujoco_ros.__render_backend__ == "NONE":
            raise unittest.SkipTest("offscreen camera bindings require a render backend")
        require_python_mujoco()
        from mujoco_ros.rendering import OffcamManager

        model_path = (
            get_package_share_directory("mujoco_ros_testing_utils") / "assets" / "camera_world.xml"
        )
        with MujocoEnv(model_path=model_path) as env:
            wait_for_idle(env)
            if not env.settings.render_offscreen:
                raise unittest.SkipTest("offscreen rendering is unavailable at runtime")

            manager = OffcamManager(
                env.binding._camera_publication_transport, env.model, cam_buff_size=2
            )
            cam = manager.cam(0)
            self.assertTrue(env.pause())
            self.assertTrue(env.step(2))

            handles = cam._buffer.getBufferHandles()
            self.assertEqual(3, len(handles))
            self.assertIsNotNone(handles[0])
            self.assertIsNone(handles[1])
            self.assertIsNone(handles[2])
            rgb, depth, segment = cam.get_buffered_frames()
            self.assertIsNotNone(rgb)
            self.assertIsNone(depth)
            self.assertIsNone(segment)
            self.assertGreater(cam._buffer._rgb_frame_count, 0)
            self.assertEqual(0, cam._buffer._depth_frame_count)
            self.assertEqual(0, cam._buffer._segment_frame_count)

            for accessor in (
                cam._buffer.borrow_latest_depth,
                cam._buffer.borrow_latest_segment,
                lambda: cam._buffer.copy_depth(1),
                lambda: cam._buffer.copy_segment(1),
            ):
                with self.assertRaisesRegex(RuntimeError, "not configured"):
                    accessor()

            for accessor in (
                lambda: cam._buffer._rgb_buf_idx,
                lambda: cam._buffer._depth_buf_idx,
                lambda: cam._buffer._segment_buf_idx,
                lambda: setattr(cam._buffer, "_rgb_frame_count", 1),
                lambda: setattr(cam._buffer, "_depth_frame_count", 1),
                lambda: setattr(cam._buffer, "_segment_frame_count", 1),
                lambda: cam._buffer.set_buffers_read(),
            ):
                with self.assertRaisesRegex(RuntimeError, "legacy"):
                    accessor()

            self.assertIsNotNone(cam.get_buffered_frames(last_n=1)[0])

            with self.assertRaisesRegex(RuntimeError, "legacy"):
                with cam._buffer:
                    pass

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
