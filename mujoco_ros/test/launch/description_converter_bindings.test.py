import unittest
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util

TEST_TIMEOUT = 60


def generate_test_description():
    test_script = LaunchConfiguration('test_script')
    python_module_path = LaunchConfiguration('python_module_path')
    python_package_path = LaunchConfiguration('python_package_path')

    test_proc = ExecuteProcess(
        cmd=[sys.executable, test_script, '--ros-args'],
        name='description_converter_bindings_test',
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('test_script'),
            DeclareLaunchArgument('python_module_path'),
            DeclareLaunchArgument('python_package_path'),
            SetEnvironmentVariable(
                'ROSCONSOLE_FORMAT', '[${severity}] [${time}] [${logger}] [${node}]: ${message}'
            ),
            SetEnvironmentVariable(
                'PYTHONPATH',
                [
                    python_module_path,
                    ':',
                    python_package_path,
                    ':',
                    EnvironmentVariable('PYTHONPATH', default_value=''),
                ],
            ),
            SetParameter(name='use_sim_time', value=True),
            test_proc,
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {
        'test_proc': test_proc,
    }


class TestDescriptionConverterBindingsWaitForCompletion(unittest.TestCase):
    def test_description_converter_bindings_run_complete(self, proc_info, test_proc):
        proc_info.assertWaitForShutdown(test_proc, timeout=TEST_TIMEOUT)


@launch_testing.post_shutdown_test()
class TestDescriptionConverterBindingsProcessPostShutdown(unittest.TestCase):
    def test_description_converter_bindings_pass(self, proc_info, test_proc):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_proc)
