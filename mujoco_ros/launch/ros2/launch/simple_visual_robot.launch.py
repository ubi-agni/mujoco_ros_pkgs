import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('mujoco_ros')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('unpause', default_value='false'),
            DeclareLaunchArgument('headless', default_value='false'),
            DeclareLaunchArgument('no_render', default_value='false'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'launch_server.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'unpause': LaunchConfiguration('unpause'),
                    'headless': LaunchConfiguration('headless'),
                    'no_render': LaunchConfiguration('no_render'),
                    'urdf_source': 'file',
                    'urdf_path': os.path.join(
                        pkg_share, 'examples', 'robot_description', 'simple_visual_robot.urdf'
                    ),
                }.items(),
            ),
        ]
    )
