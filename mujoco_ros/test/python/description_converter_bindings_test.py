#!/usr/bin/env python3

from pathlib import Path
import sys
import tempfile
import time
import unittest

import pymujoco_ros

from mujoco_ros import MujocoEnv

# Test resources (two_link_robot.urdf/.srdf) live alongside this test's
# source tree, not under the installed package share directory -- mirrors
# how the C++ gtests reach them via TEST_RESOURCES_DIR, but for a Python
# launch test the script always runs from its source-tree location (see
# CMakeLists.txt's `test_script:=${CMAKE_CURRENT_SOURCE_DIR}/python/...`).
_TEST_RESOURCES_DIR = Path(__file__).resolve().parent.parent / "resources"


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


def shutdown_rclpy_if_needed():
    try:
        import rclpy
    except ImportError:
        return
    if rclpy.ok():
        rclpy.shutdown()


class DescriptionConverterBindingsTest(unittest.TestCase):
    def _resource(self, name):
        return str(_TEST_RESOURCES_DIR / name)

    def test_load_model_from_description_returns_model_and_data(self):
        require_python_mujoco()
        import mujoco

        urdf_path = self._resource("two_link_robot.urdf")
        srdf_path = self._resource("two_link_robot.srdf")

        model, data = pymujoco_ros.load_model_from_description(urdf_path, srdf_path)

        self.assertIsInstance(model, mujoco.MjModel)
        self.assertIsInstance(data, mujoco.MjData)
        # nullptr world composes into default_world + an unprefixed robot (default attach_prefix is "")
        self.assertNotEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link"), -1)
        self.assertNotEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground_plane"), -1)
        self.assertGreaterEqual(model.nlight, 1)
        self.assertIsNotNone(data)

    def test_load_model_from_description_generates_actuators_when_requested(self):
        require_python_mujoco()
        import mujoco

        model, data = pymujoco_ros.load_model_from_description(
            self._resource("ros2_control_robot.urdf"),
            "",
            generate_actuators=True,
        )

        self.assertIsNotNone(data)
        self.assertNotEqual(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "pos_joint_act_pos"),
            -1,
        )

    def test_load_model_from_description_applies_explicit_attach_prefix(self):
        require_python_mujoco()
        import mujoco

        model, data = pymujoco_ros.load_model_from_description(
            self._resource("two_link_robot.urdf"),
            self._resource("two_link_robot.srdf"),
            attach_prefix="py_",
        )

        self.assertIsNotNone(data)
        self.assertNotEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "py_base_link"), -1)

    def test_load_model_from_description_cleans_temp_model_when_mujoco_import_fails(self):
        class RejectMujocoImport:
            def find_spec(self, fullname, path=None, target=None):
                if fullname == "mujoco":
                    raise ImportError("forced mujoco import failure")
                return None

        temp_models_before = set(Path(tempfile.gettempdir()).glob("mujoco_ros_description_*.mjb"))
        import_hook = RejectMujocoImport()
        original_mujoco = sys.modules.pop("mujoco", None)
        sys.meta_path.insert(0, import_hook)
        try:
            with self.assertRaisesRegex(ImportError, "forced mujoco import failure"):
                pymujoco_ros.load_model_from_description(
                    self._resource("two_link_robot.urdf"),
                    self._resource("two_link_robot.srdf"),
                )
        finally:
            sys.meta_path.remove(import_hook)
            if original_mujoco is not None:
                sys.modules["mujoco"] = original_mujoco

        self.assertEqual(
            set(Path(tempfile.gettempdir()).glob("mujoco_ros_description_*.mjb")),
            temp_models_before,
        )

    def test_load_model_from_description_raises_on_missing_urdf(self):
        require_python_mujoco()

        urdf_path = self._resource("does_not_exist.urdf")
        srdf_path = self._resource("two_link_robot.srdf")

        with self.assertRaises(RuntimeError):
            pymujoco_ros.load_model_from_description(urdf_path, srdf_path)

    def test_mujoco_env_from_description_loads_a_valid_model(self):
        require_python_mujoco()

        urdf_path = self._resource("two_link_robot.urdf")
        srdf_path = self._resource("two_link_robot.srdf")

        with MujocoEnv.from_description(urdf_path, srdf_path) as env:
            wait_for_idle(env)
            self.assertTrue(env.model_valid)
            self.assertIn("two_link_robot.urdf", env.filename)
            self.assertIsNotNone(env.model)
            self.assertIsNotNone(env.data)

    def test_mujoco_env_from_description_raises_on_missing_urdf(self):
        require_python_mujoco()

        urdf_path = self._resource("does_not_exist.urdf")
        srdf_path = self._resource("two_link_robot.srdf")

        with self.assertRaises(RuntimeError):
            MujocoEnv.from_description(urdf_path, srdf_path)


if __name__ == "__main__":
    if is_ros1():
        import rostest

        rostest.rosrun(
            "mujoco_ros", "description_converter_bindings_test", DescriptionConverterBindingsTest
        )
    else:
        result = None
        try:
            result = unittest.main(argv=[sys.argv[0]], exit=False).result
        finally:
            shutdown_rclpy_if_needed()
        sys.exit(0 if result is not None and result.wasSuccessful() else 1)
