import unittest

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util

TEST_TIMEOUT = 180

_CASES = (
    ('completion', 4, 2, 1),
    ('attribution', 2, 3, 2),
    ('exit_deadline', 4, 10, 2),
    ('camera_1', 1, 2, 1),
    ('camera_2', 2, 2, 1),
    ('camera_4', 4, 2, 1),
)


def generate_test_description():
    test_binary = LaunchConfiguration('test_binary')
    benchmark_backend = LaunchConfiguration('benchmark_backend')
    output_dir = LaunchConfiguration('output_dir')

    test_procs = []
    actions = [
        DeclareLaunchArgument('test_binary'),
        DeclareLaunchArgument('benchmark_backend'),
        DeclareLaunchArgument('output_dir', default_value='.'),
        SetEnvironmentVariable(
            'ROSCONSOLE_FORMAT', '[${severity}] [${time}] [${logger}] [${node}]: ${message}'
        ),
    ]
    for name, camera_count, iterations, repeat in _CASES:
        test_proc = ExecuteProcess(
            cmd=[
                test_binary,
                '--backend',
                benchmark_backend,
                '--camera-count',
                str(camera_count),
                '--iterations',
                str(iterations),
                '--repeat',
                str(repeat),
                '--output',
                PathJoinSubstitution([output_dir, f'render_performance_{name}.json']),
            ],
            name=f'render_performance_{name}',
            output='screen',
        )
        test_procs.append(test_proc)

    for current_proc, next_proc in zip(test_procs, test_procs[1:]):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=current_proc,
                    on_exit=[next_proc],
                )
            )
        )

    actions.extend(
        [
            test_procs[0],
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ]
    )

    return LaunchDescription(actions), {
        'test_procs': test_procs,
    }


class TestRenderPerformanceCompletion(unittest.TestCase):
    def test_benchmarks_complete(self, proc_info, test_procs):
        for test_proc in test_procs:
            proc_info.assertWaitForShutdown(test_proc, timeout=TEST_TIMEOUT)


@launch_testing.post_shutdown_test()
class TestRenderPerformanceProcessExit(unittest.TestCase):
    def test_benchmarks_exit_cleanly(self, proc_info, test_procs):
        for test_proc in test_procs:
            launch_testing.asserts.assertExitCodes(proc_info, process=test_proc)
