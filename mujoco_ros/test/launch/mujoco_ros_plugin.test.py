import unittest
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util

TEST_TIMEOUT = 600


def generate_test_description():
    test_binary = LaunchConfiguration('test_binary')
    timeout = LaunchConfiguration('timeout')

    params_file = str(Path(__file__).resolve().with_name('mujoco_ros_plugin.params.yaml'))

    test_proc = ExecuteProcess(
        cmd=[test_binary, '--ros-args', '--params-file', params_file],
        name='mujoco_ros_plugin_test',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('test_binary'),
        DeclareLaunchArgument('timeout', default_value=str(TEST_TIMEOUT)),
        SetEnvironmentVariable(
            'ROSCONSOLE_FORMAT',
            '[${severity}] [${time}] [${logger}] [${node}]: ${message}'
        ),
        test_proc,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), {
        'test_proc': test_proc,
        'timeout': timeout,
    }

class TestGTestWaitForCompletion(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, test_proc):
        proc_info.assertWaitForShutdown(test_proc, timeout=TEST_TIMEOUT)

@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    def test_gtest_pass(self, proc_info, test_proc):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_proc)
