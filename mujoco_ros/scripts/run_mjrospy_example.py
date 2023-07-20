#!/usr/bin/env python3

import mujoco
import mujoco_ros

import matplotlib.pyplot as plt
import numpy as np
from copy import deepcopy
from pathlib import Path
import os
import rospkg
import rospy
import argparse

try:
    import _mujoco_ros_sensors_python
except ImportError:
    print("MuJoCo Sensor plugin bindings not built")
try:
    import _mujoco_ros_mocap_python
except ImportError:
    print("MuJoCo Mocap plugin bindings not built")

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("-m", "--model", required=False)
parser.add_argument("-c", "--load-configs", nargs="+", default=[])

args = parser.parse_args()

print(f"Mujoco version: {mujoco.__version__}")

frames = []

if args.model:
    model_path = Path(args.model)
else:
    model_path = Path(
        rospkg.RosPack().get_path("mujoco_ros"), "test", "camera_world.xml"
    )
print(f"Loading model from {model_path.absolute()}")

configs_to_load = []
if len(args.load_configs) > 0:
    [configs_to_load.append(Path(load_config)) for load_config in args.load_configs]
print(f"Loading config files: {[cfg.absolute() for cfg in configs_to_load]}")


mj_env = mujoco_ros.MujocoEnv(
    model_path,
    start_ros_core=False,
    unpause=False,
    configs_to_load=configs_to_load,
    cam_buff_size=20,
)

# Run for 10_000 steps to get the model in a usable state

if not args.headless:
    mj_env.attach_viewer(active=True)

if mj_env.offcam_manager.num_cams > 0:
    delta_t = 0.01
    fps = mj_env.offcam_manager.camera().fps
    step_size = int(fps / delta_t + 0.5) * 20

    rgb, _, _ = mj_env.offcam_manager.buffer(0)
    img = plt.imshow(np.zeros(rgb.shape[1:]))

    rgb_frames = []

    for i in range(10_000 // step_size):
        mj_env.step(step_size)
        rgb, _, _ = mj_env.offcam_manager.buffer(0)
        rgb_frames.append(deepcopy(rgb))

    rgb_frames = np.concatenate(rgb_frames, axis=0)
    print("Got {} frames".format(rgb_frames.shape[0]))
    for frame in rgb_frames:
        img.set_data(frame)
        plt.pause(0.1)
else:
    print("Modelfile has no cameras, skipping render example")

mj_env.step(200)

plugins = mj_env._env.get_plugins()
print(f"Got {len(plugins)} plugins:")
for plugin in plugins:
    if type(plugin).__name__ == "_MujocoPlugin":
        print(f"- Plugin with no availabe (loaded) python bindings found: {plugin}")
    else:
        print(
            f"- Plugin with specific python bindings found: {plugin} ({type(plugin).__name__})"
        )

mj_env.step(100)
