import mujoco_ros

try:
    from _mujoco_ros_mocap_python import MujocoRosMocapPlugin
except ImportError as e:
    print("Could not find mujoco_ros_mocap plugin python bindings!")
    raise e
