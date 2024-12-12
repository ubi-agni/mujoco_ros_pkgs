from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mujoco_ros2_base",
            executable="mujoco_ros2_main",
            name="mujoco_ros2_main_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"MujocoPlugins": ["earth","wind","fire", "MujocoRos2Control"]}
            ]
        )
    ])
