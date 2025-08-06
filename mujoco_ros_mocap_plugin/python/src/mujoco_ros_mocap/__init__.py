try:
    from pymujoco_ros_mocap import MujocoRosMocapPlugin
except ImportError as e:
    print("Could not find mujoco_ros_mocap plugin python bindings!")
    raise e
