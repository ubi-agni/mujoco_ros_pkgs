# MuJoCo ROS Laser Plugin

Provides a CPU-based Laser sensor, using MuJoCo's `ray_collision` function.

Rays are cast serially with one `mj_multiRay` call per scan. The server's `mujoco_threads` option does not parallelize laser scans; it only sizes MuJoCo's internal engine threadpool, which exists only for values above 1.

## Configuration Example

https://github.com/ubi-agni/mujoco_ros/blob/noetic-devel/mujoco_ros_laser/config/laser_example_config.yaml?plain=1

## License

This repository is licensed under the [BSD 3-Clause License](../mujoco_ros/LICENSE).
