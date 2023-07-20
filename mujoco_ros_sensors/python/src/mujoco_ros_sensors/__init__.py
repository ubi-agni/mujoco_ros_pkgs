import mujoco_ros

try:
    from _mujoco_ros_sensors_python import MujocoRosSensorsPlugin, _SensorConfig
except ImportError as e:
    print("Could not find mujoco_ros_sensors plugin python bindings!")
    raise e
