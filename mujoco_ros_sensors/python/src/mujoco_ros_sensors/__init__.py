try:
    from pymujoco_ros_sensors import MujocoRosSensorsPlugin, _SensorConfig
except ImportError as e:
    print("Could not find mujoco_ros_sensors plugin python bindings!")
    raise e
