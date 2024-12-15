from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import xacro

def generate_launch_description():
    xml_path = os.path.join(get_package_share_directory('mujoco_ros2_base'), 'sample_xml','pendulum.xml')
    mr2c_yaml_path = os.path.join(get_package_share_directory('mujoco_ros2_base'), 'config','mujoco_ros2_plugin_configs.yaml')
    xacro_file = os.path.join(get_package_share_directory('mujoco_ros2_base'),
                              'sample_xml',
                              'pendulum.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        Node(
            package="mujoco_ros2_base",
            executable="mujoco_ros2_main",
            # name="mujoco_ros2_main_node", // setting name here causes plugin's nodes to also be created with this name?
            output="screen",
            emulate_tty=True,
            parameters=[
                {"MujocoPlugins": ['MujocoRos2Control'],
                "MujocoXml": xml_path,
                'MujocoPluginConfigs': mr2c_yaml_path}
            ],
            arguments=['--ros-args', '--log-level', 'DEBUG'],
        ),
        node_robot_state_publisher
    ])
