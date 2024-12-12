from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    xml_path = os.path.join(get_package_share_directory('mujoco_ros2_base'), 'sample_xml','pendulum.xml')
    print(xml_path)
    return LaunchDescription([
        Node(
            package="mujoco_ros2_base",
            executable="mujoco_ros2_main",
            name="mujoco_ros2_main_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"MujocoPlugins": ["earth","wind","fire", "MujocoRos2Control"],
                 "MujocoXml": xml_path}
            ]
        )
    ])
