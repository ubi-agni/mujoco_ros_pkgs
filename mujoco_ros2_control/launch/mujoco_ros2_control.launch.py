from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import xacro

def generate_launch_description():
    mujoco_ros_path = get_package_share_directory('mujoco_ros')
    xml_path = os.path.join(mujoco_ros_path, 'assets', 'pendulum.xml')
    xacro_file = os.path.join(mujoco_ros_path, 'assets', 'pendulum.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # pendulum_config = os.path.join(get_package_share_directory('mujoco_ros2_base'), 'config','pendulum.yaml')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    node_joint_state_broadcaster = Node( # RVIZ dependency
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['pendulum/joint_states'],
                 'rate': 10}],
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(mujoco_ros_path + '/launch/launch_server.launch'),
            launch_arguments={
                'use_sim_time': "true",
                'modelfile': xml_path,
                'verbose': "true",
            }.items()
        ),
        node_robot_state_publisher,
        # Node( # RVIZ dependency
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['joint_state_broadcaster'],
        #     output='screen',
        # ),
        # node_joint_state_broadcaster
    ])
