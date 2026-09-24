#!/usr/bin/env python3

import faulthandler
import sys
import unittest

from mujoco_ros import MujocoEnv


def is_ros1():
    try:
        import rosgraph  # noqa: F401
    except ImportError:
        return False
    return True


class PythonBindingsPolicyTest(unittest.TestCase):
    def test_render_backpressure_policy_startup_reader(self):
        with MujocoEnv(parameters={"render_backpressure_policy": "wait_for_slot"}) as env:
            self.assertEqual(env.render_backpressure_policy, "wait_for_slot")
            self.assertEqual(env.settings.render_backpressure_policy, "wait_for_slot")

    def test_immediate_shutdown_survives_executor_spin_race(self):
        """Construct then shutdown immediately must not wedge in executor join.

        ROS 2 MultiThreadedExecutor::spin() re-arms spinning after a cancel that
        raced ahead of spin entry. Without bounded cancel-retry, join hangs.
        faulthandler is required because the GIL stays held during the C++ join.
        """
        if is_ros1():
            self.skipTest("executor spin race is ROS 2 only")

        iterations = 50
        for i in range(iterations):
            env = MujocoEnv(parameters={"render_backpressure_policy": "wait_for_slot"})
            faulthandler.dump_traceback_later(10, exit=True)
            try:
                env.shutdown()
            finally:
                faulthandler.cancel_dump_traceback_later()


if __name__ == "__main__":
    if is_ros1():
        import rostest

        rostest.rosrun("mujoco_ros", "python_bindings_policy_test", PythonBindingsPolicyTest)
    else:
        unittest.main(argv=[sys.argv[0]])
