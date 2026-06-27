from pymujoco_ros_mocap import MocapPlugin
from pymujoco_ros_mocap import bind

from mujoco_ros.plugins import register_plugin_binding

register_plugin_binding("mujoco_ros_mocap/MocapPlugin", bind)
register_plugin_binding("MocapPlugin", bind)

__all__ = ["MocapPlugin", "bind"]
