import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_test_description():
    test_proc = ExecuteProcess(
        cmd=[LaunchConfiguration('test_binary')],
        name='description_converter_integration_test',
        output='screen',
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('test_binary'),
            test_proc,
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {'test_proc': test_proc}


class TestCompletion:
    def test_gtest_run_complete(self, proc_info, test_proc):
        proc_info.assertWaitForShutdown(test_proc, timeout=60)


@launch_testing.post_shutdown_test()
class TestExit:
    def test_gtest_pass(self, proc_info, test_proc):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_proc)
