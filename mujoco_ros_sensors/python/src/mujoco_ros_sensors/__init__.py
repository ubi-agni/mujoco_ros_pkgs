from pymujoco_ros_sensors import MujocoRosSensorsPlugin
from pymujoco_ros_sensors import _SensorConfig
from pymujoco_ros_sensors import bind

from mujoco_ros.plugins import register_plugin_binding

register_plugin_binding("mujoco_ros_sensors/MujocoRosSensorsPlugin", bind)
register_plugin_binding("MujocoRosSensorsPlugin", bind)

__all__ = ["MujocoRosSensorsPlugin", "_SensorConfig", "bind"]
