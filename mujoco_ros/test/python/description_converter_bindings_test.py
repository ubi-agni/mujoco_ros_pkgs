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

    def _latch_publish(self, node_or_none, topic, content):
        """Publish `content` once on `topic` with a latched publisher, returning
        whatever handle(s) must stay alive until the subscriber has read it."""
        if is_ros1():
            import rospy
            from std_msgs.msg import String

            if not rospy.core.is_initialized():
                rospy.init_node(
                    "description_converter_bindings_test_pub", anonymous=True, disable_signals=True
                )
            pub = rospy.Publisher(topic, String, queue_size=1, latch=True)
            time.sleep(0.2)  # let the publisher register before the subscriber connects
            pub.publish(String(data=content))
            return pub

        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        pub = node_or_none.create_publisher(String, topic, qos)
        pub.publish(String(data=content))
        return pub

    def test_mujoco_env_from_description_topic_loads_a_valid_model(self):
        require_python_mujoco()

        urdf_content = Path(self._resource("two_link_robot.urdf")).read_text(encoding="utf-8")
        srdf_content = Path(self._resource("two_link_robot.srdf")).read_text(encoding="utf-8")

        pub_node = None
        try:
            if is_ros1():
                self._latch_publish(None, "/description_topic_test/urdf", urdf_content)
                self._latch_publish(None, "/description_topic_test/srdf", srdf_content)
            else:
                import rclpy

                if not rclpy.ok():
                    rclpy.init()
                pub_node = rclpy.create_node("description_converter_bindings_test_pub")
                self._latch_publish(pub_node, "/description_topic_test/urdf", urdf_content)
                self._latch_publish(pub_node, "/description_topic_test/srdf", srdf_content)

            with MujocoEnv.from_description_topic(
                "/description_topic_test/urdf", "/description_topic_test/srdf"
            ) as env:
                wait_for_idle(env)
                self.assertTrue(env.model_valid)
                self.assertIsNotNone(env.model)
                self.assertIsNotNone(env.data)
        finally:
            if pub_node is not None:
                pub_node.destroy_node()

    def test_mujoco_env_from_description_topic_raises_on_timeout(self):
        require_python_mujoco()

        with self.assertRaisesRegex(RuntimeError, "Timed out"):
            MujocoEnv.from_description_topic(
                "/description_topic_test/never_published", timeout=0.2
            )

    def test_mujoco_env_from_description_topic_prefers_srdf_path_over_srdf_topic(self):
        """srdf_path is a plain filesystem backup for deployments where SRDF stays
        local-only; when given it must win even if srdf_topic is also set (and
        never published on, so a topic-read attempt would time out and fail)."""
        require_python_mujoco()

        urdf_content = Path(self._resource("two_link_robot.urdf")).read_text(encoding="utf-8")
        srdf_path = self._resource("two_link_robot.srdf")

        pub_node = None
        try:
            if is_ros1():
                self._latch_publish(None, "/description_topic_test/urdf_only", urdf_content)
            else:
                import rclpy

                if not rclpy.ok():
                    rclpy.init()
                pub_node = rclpy.create_node("description_converter_bindings_test_pub2")
                self._latch_publish(pub_node, "/description_topic_test/urdf_only", urdf_content)

            with MujocoEnv.from_description_topic(
                "/description_topic_test/urdf_only",
                srdf_topic="/description_topic_test/srdf_never_published",
                srdf_path=srdf_path,
                timeout=0.2,
            ) as env:
                wait_for_idle(env)
                self.assertTrue(env.model_valid)
        finally:
            if pub_node is not None:
                pub_node.destroy_node()

    def test_mujoco_env_from_description_forwards_ros_params(self):
        require_python_mujoco()

        urdf_path = self._resource("two_link_robot.urdf")
        srdf_path = self._resource("two_link_robot.srdf")

        with MujocoEnv.from_description(
            urdf_path, srdf_path, ros_params={"domain_id": 7}
        ) as env:
            wait_for_idle(env)
            self.assertTrue(env.model_valid)
            if not is_ros1():
                self.assertIsNotNone(env._temp_param_file)
                self.assertIn(
                    "domain_id: 7",
                    Path(env._temp_param_file).read_text(encoding="utf-8"),
                )

    def test_mujoco_env_from_description_topic_forwards_ros_params(self):
        require_python_mujoco()

        urdf_content = Path(self._resource("two_link_robot.urdf")).read_text(encoding="utf-8")

        pub_node = None
        try:
            if is_ros1():
                self._latch_publish(None, "/description_topic_test/urdf_params", urdf_content)
            else:
                import rclpy

                if not rclpy.ok():
                    rclpy.init()
                pub_node = rclpy.create_node("description_converter_bindings_test_pub3")
                self._latch_publish(pub_node, "/description_topic_test/urdf_params", urdf_content)

            with MujocoEnv.from_description_topic(
                "/description_topic_test/urdf_params",
                srdf_path=self._resource("two_link_robot.srdf"),
                ros_params={"domain_id": 3},
            ) as env:
                wait_for_idle(env)
                self.assertTrue(env.model_valid)
                if not is_ros1():
                    self.assertIsNotNone(env._temp_param_file)
                    self.assertIn(
                        "domain_id: 3",
                        Path(env._temp_param_file).read_text(encoding="utf-8"),
                    )
        finally:
            if pub_node is not None:
                pub_node.destroy_node()


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
