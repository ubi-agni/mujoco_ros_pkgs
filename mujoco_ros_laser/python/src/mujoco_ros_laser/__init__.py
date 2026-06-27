from pymujoco_ros_laser import LaserConfig
from pymujoco_ros_laser import LaserPlugin
from pymujoco_ros_laser import bind

from mujoco_ros.plugins import register_plugin_binding

register_plugin_binding("mujoco_ros_laser/LaserPlugin", bind)
register_plugin_binding("LaserPlugin", bind)

__all__ = ["LaserConfig", "LaserPlugin", "bind"]
