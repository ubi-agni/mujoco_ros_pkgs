# MuJoCo hand of god plugin
This plugin holds an object, defined in the config, in place.

## Usage

Define the desired hog-objects in config (hog_example_config.yaml).
Launch plugin with `roslaunch mujoco_ros_hog hog.launch`
Pause/unpause the plugin with with the servicecall `mujoco_ros_hog/set_active bool`


# Licensing

This work is licensed under the BSD 3-Clause License (see LICENSE).
The original work it is based on was released under a BSD 3-Clause License (see LICENSE-ORIGINAL).
