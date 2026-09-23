"""ROS 2 Python launch for mujoco_server.

Mirrors ``launch_server.launch.xml``: same launch arguments, including the
Description-bundle overrides (``urdf_*`` / ``srdf_*`` /
``convert_ascii_stl``). Bundle activation requires
``urdf_source``; SRDF args are optional.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def _maybe_set(params, key, value):
    """Add a dotted param only when the launch arg is non-empty."""
    if value:
        params[key] = value


def _launch_setup(context, *args, **kwargs):
    verbose = LaunchConfiguration('verbose').perform(context)
    log_level = 'debug' if verbose.lower() in ('true', '1') else 'info'

    params = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'ns': LaunchConfiguration('ns'),
        'unpause': LaunchConfiguration('unpause'),
        'headless': LaunchConfiguration('headless'),
        'render_offscreen': LaunchConfiguration('render_offscreen'),
        'no_render': LaunchConfiguration('no_render'),
        'num_steps': LaunchConfiguration('num_sim_steps'),
        'eval_mode': LaunchConfiguration('eval_mode'),
        'modelfile': LaunchConfiguration('modelfile'),
        'wait_for_xml': LaunchConfiguration('wait_for_xml'),
        'realtime': LaunchConfiguration('realtime'),
        'num_mj_threads': LaunchConfiguration('mujoco_threads'),
        'log_level': log_level,
    }

    # Description bundle — only set keys the user actually passed.
    _maybe_set(params, 'urdf.source', LaunchConfiguration('urdf_source').perform(context))
    _maybe_set(params, 'urdf.path', LaunchConfiguration('urdf_path').perform(context))
    _maybe_set(params, 'urdf.topic', LaunchConfiguration('urdf_topic').perform(context))
    _maybe_set(params, 'srdf.source', LaunchConfiguration('srdf_source').perform(context))
    _maybe_set(params, 'srdf.path', LaunchConfiguration('srdf_path').perform(context))
    _maybe_set(params, 'srdf.topic', LaunchConfiguration('srdf_topic').perform(context))
    _maybe_set(
        params,
        'description.convert_ascii_stl',
        LaunchConfiguration('convert_ascii_stl').perform(context),
    )

    param_files = [
        ParameterFile(LaunchConfiguration('initial_joint_states'), allow_substs=True),
    ]
    plugin_config = LaunchConfiguration('mujoco_plugin_config').perform(context)
    if plugin_config:
        param_files.append(ParameterFile(plugin_config, allow_substs=True))

    node_args = [
        '--admin-hash',
        LaunchConfiguration('admin_hash'),
        '--ros-args',
        '--log-level',
        f'mujoco_server:={log_level}',
        '--log-level',
        f'mujoco_ros_plugin_loader:={log_level}',
        '--log-level',
        f'Viewer:={log_level}',
    ]

    gdb_term_cmd = LaunchConfiguration('gdb_term_cmd').perform(context)
    valgrind_args = LaunchConfiguration('valgrind_args').perform(context)
    valgrind_prefix = f'valgrind {valgrind_args}'.strip() if valgrind_args else 'valgrind'

    def make_node(launch_prefix=None):
        return Node(
            package='mujoco_ros',
            executable='mujoco_node',
            output='screen',
            arguments=node_args,
            parameters=[params, *param_files],
            prefix=launch_prefix,
        )

    return [
        GroupAction(
            condition=IfCondition(LaunchConfiguration('debug')),
            actions=[
                GroupAction(
                    condition=UnlessCondition(LaunchConfiguration('debug_server')),
                    actions=[make_node(launch_prefix=f'{gdb_term_cmd}gdb --args')],
                ),
                GroupAction(
                    condition=IfCondition(LaunchConfiguration('debug_server')),
                    actions=[make_node(launch_prefix='gdbserver localhost:1234')],
                ),
            ],
        ),
        GroupAction(
            condition=UnlessCondition(LaunchConfiguration('debug')),
            actions=[
                GroupAction(
                    condition=IfCondition(LaunchConfiguration('valgrind')),
                    actions=[make_node(launch_prefix=valgrind_prefix)],
                ),
                GroupAction(
                    condition=UnlessCondition(LaunchConfiguration('valgrind')),
                    actions=[
                        SetEnvironmentVariable(
                            name='CPUPROFILE',
                            value='/tmp/profile.out',
                            condition=IfCondition(LaunchConfiguration('profile')),
                        ),
                        make_node(),
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('mujoco_ros')

    return LaunchDescription(
        [
            DeclareLaunchArgument('ns', default_value='', description='namespace'),
            DeclareLaunchArgument(
                'verbose',
                default_value='false',
                description='Whether more debug output should be printed.',
            ),
            DeclareLaunchArgument(
                'unpause',
                default_value='false',
                description='Whether the simulation should be unpaused on start.',
            ),
            DeclareLaunchArgument('headless', default_value='false'),
            DeclareLaunchArgument(
                'render_offscreen',
                default_value='true',
                description='Whether offscreen rendering should be enabled.',
            ),
            DeclareLaunchArgument(
                'no_render',
                default_value='false',
                description='Shorthand for render_offscreen:=false headless:=true.',
            ),
            DeclareLaunchArgument(
                'eval_mode',
                default_value='false',
                description='Whether to run mujoco_ros in evaluation mode.',
            ),
            DeclareLaunchArgument(
                'admin_hash',
                default_value="''",
                description='Hash to verify critical operations in evaluation mode.',
            ),
            DeclareLaunchArgument(
                'debug', default_value='false', description='Whether to run with gdb.'
            ),
            DeclareLaunchArgument(
                'debug_server',
                default_value='false',
                description='Whether to run with gdbserver on port 1234.',
            ),
            DeclareLaunchArgument(
                'valgrind', default_value='false', description='Whether to run with valgrind.'
            ),
            DeclareLaunchArgument(
                'valgrind_args', default_value='', description='arguments for valgrind'
            ),
            DeclareLaunchArgument(
                'wait_for_xml',
                default_value='false',
                description='Whether mujoco_ros should wait for an xml in the parameter server.',
            ),
            DeclareLaunchArgument(
                'realtime',
                default_value='1.0',
                description='Fraction of desired realtime (0,1]. -1 to run as fast as possible.',
            ),
            DeclareLaunchArgument(
                'profile',
                default_value='false',
                description='Whether mujoco_ros should be profiled.',
            ),
            DeclareLaunchArgument('num_sim_steps', default_value='-1'),
            DeclareLaunchArgument(
                'mujoco_plugin_config',
                default_value='',
                description='Optionally provide the path to a yaml with plugin configurations to load.',
            ),
            DeclareLaunchArgument(
                'mujoco_threads',
                default_value='1',
                description='Deprecated. Number of MuJoCo simulation threads.',
            ),
            DeclareLaunchArgument(
                'gdb_term_cmd',
                default_value='gnome-terminal -- ',
                description="Command to open gdb in a new terminal. Common alternative: 'xterm -e '",
            ),
            DeclareLaunchArgument(
                'modelfile',
                default_value='',
                description='Full MuJoCo model when description bundle unused. '
                'With urdf_source set: world MJCF to compose into '
                '(empty = built-in default_world).',
            ),
            DeclareLaunchArgument(
                'initial_joint_states',
                default_value=os.path.join(pkg_share, 'config', 'initial_joint_states.yaml'),
                description='Provide a filepath containing initial joint states to load.',
            ),
            DeclareLaunchArgument(
                'console_config_file',
                default_value=os.path.join(pkg_share, 'config', 'rosconsole.config'),
                description='Path to ROS console config used when verbose logging is active.',
            ),
            DeclareLaunchArgument(
                'urdf_source',
                default_value='',
                description="URDF source kind: 'file' or 'topic'. Empty skips the description bundle.",
            ),
            DeclareLaunchArgument(
                'urdf_path', default_value='', description='URDF file path when urdf_source:=file.'
            ),
            DeclareLaunchArgument(
                'urdf_topic',
                default_value='',
                description='Topic publishing URDF as a latched std_msgs/String when urdf_source:=topic. '
                'Server defaults to robot_description if unset.',
            ),
            DeclareLaunchArgument(
                'srdf_source',
                default_value='',
                description="Optional. SRDF source kind: 'file' or 'topic'.",
            ),
            DeclareLaunchArgument(
                'srdf_path', default_value='', description='SRDF file path when srdf_source:=file.'
            ),
            DeclareLaunchArgument(
                'srdf_topic',
                default_value='',
                description='Topic publishing SRDF as a latched std_msgs/String when srdf_source:=topic. '
                'Server defaults to robot_description_semantic if unset.',
            ),
            DeclareLaunchArgument(
                'convert_ascii_stl',
                default_value='',
                description="Optional. 'true'/'false' for description.convert_ascii_stl "
                '(ASCII STL → temp binary cache). Empty = false (OBJ fallback).',
            ),
            DeclareLaunchArgument('use_sim_time'),
            SetEnvironmentVariable(
                name='RCUTILS_CONSOLE_OUTPUT_FORMAT',
                value='[{severity}] [{time}] [{name}] [{function_name}]: {message}',
            ),
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
