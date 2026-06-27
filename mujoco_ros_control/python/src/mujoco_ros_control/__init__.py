from pymujoco_ros_control import MujocoRosControlPlugin
from pymujoco_ros_control import bind

from mujoco_ros.plugins import register_plugin_binding

register_plugin_binding("mujoco_ros_control/MujocoRosControlPlugin", bind)
register_plugin_binding("MujocoRosControlPlugin", bind)

__all__ = ["MujocoRosControlPlugin", "bind"]
