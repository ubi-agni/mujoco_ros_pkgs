import unittest

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util


def generate_test_description():
    test_binary = LaunchConfiguration('test_binary')
    timeout = LaunchConfiguration('timeout')

    test_proc = ExecuteProcess(
        cmd=[test_binary],
        name='mocap_test',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('test_binary'),
        DeclareLaunchArgument('timeout', default_value='45'),
        SetEnvironmentVariable(
            'ROSCONSOLE_FORMAT',
            '[${severity}] [${time}] [${logger}] [${node}]: ${message}'
        ),
        SetParameter(name='use_sim_time', value=True),
        test_proc,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), {
        'test_proc': test_proc,
        'timeout': timeout,
    }


class TestGTestWaitForCompletion(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, test_proc):
        proc_info.assertWaitForShutdown(test_proc)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    def test_gtest_pass(self, proc_info, test_proc):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_proc)
