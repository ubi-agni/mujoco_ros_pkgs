#!/usr/bin/env python3

import unittest

import pymujoco_ros

from mujoco_ros import MujocoEnv

from python_bindings_test import get_package_share_directory
from python_bindings_test import require_python_mujoco
from python_bindings_test import wait_for_idle


class PythonBindingsReloadTest(unittest.TestCase):
    def test_camera_reload_rebinds_layout_metadata(self):
        if pymujoco_ros.__render_backend__ == "NONE":
            raise unittest.SkipTest("offscreen camera bindings require a render backend")
        require_python_mujoco()
        from mujoco_ros.rendering import OffcamManager

        model_path = (
            get_package_share_directory("mujoco_ros_testing_utils") / "assets" / "camera_world.xml"
        )
        with open(model_path, "r", encoding="utf-8") as stream:
            replacement_xml = stream.read().replace('name="test_cam"', 'name="reloaded_cam"')

        parameters = {
            "cam_config": {
                "reloaded_cam": {"stream_type": 1, "width": 9, "height": 5},
            }
        }
        with MujocoEnv(model_path=model_path, parameters=parameters) as env:
            wait_for_idle(env)
            if not env.settings.render_offscreen:
                raise unittest.SkipTest("offscreen rendering is unavailable at runtime")

            with OffcamManager(
                env.binding._camera_publication_transport, env.model, cam_buff_size=2
            ) as manager:
                cam = manager.cam(0)
                old_native_camera = cam.camera
                self.assertEqual("test_cam", cam.cam_name)
                self.assertGreater(cam.width, 0)
                self.assertGreater(cam.height, 0)
                self.assertTrue(env.pause())
                self.assertTrue(env.step(2))
                with cam.borrow_latest_rgb() as old_view:
                    old_generation = old_view.frame_generation
                    old_shape = old_view.shape

                self.assertTrue(env.load_from_string(replacement_xml))
                wait_for_idle(env)
                self.assertEqual("test_cam", old_native_camera.name)
                self.assertIsNot(old_native_camera, cam.camera)
                self.assertEqual((9, 5, "reloaded_cam"), (cam.width, cam.height, cam.cam_name))
                self.assertTrue(cam._has_plane(1))
                self.assertFalse(cam._has_plane(2))
                self.assertIs(manager.cam("reloaded_cam"), cam)
                self.assertIsNone(manager.cam("test_cam"))
                self.assertEqual(old_shape, old_view.shape)
                self.assertEqual(old_generation, old_view.frame_generation)

                self.assertTrue(env.step(2))
                rgb, depth, segment = cam.buffer(last_n=1)
                self.assertIsNotNone(rgb)
                self.assertIsNone(depth)
                self.assertIsNone(segment)
                self.assertEqual((1, 5, 9, 3), rgb.shape)
                with self.assertRaisesRegex(RuntimeError, "not configured"):
                    cam.borrow_latest_depth()
                with self.assertRaisesRegex(RuntimeError, "not configured"):
                    cam.copy_depth(1)


if __name__ == "__main__":
    result = unittest.main(argv=[__file__], exit=False).result
    try:
        from python_bindings_test import shutdown_rclpy_if_needed

        shutdown_rclpy_if_needed()
    finally:
        raise SystemExit(0 if result is not None and result.wasSuccessful() else 1)
