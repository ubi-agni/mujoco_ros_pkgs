# MuJoCo ROS Control

`mujoco_ros_control` provides the ROS 1 `ros_control` and ROS 2 `ros2_control`
integration for `mujoco_ros`.

The package exports:

- ROS 1 `mujoco_ros_control/MujocoRosControlPlugin`
- ROS 1 `mujoco_ros_control/DefaultRobotHWSim`
- ROS 2 `mujoco_ros_control/MujocoRosControlPlugin`
- ROS 2 `mujoco_ros_control/MujocoRosSystem`

The ROS 1 path follows the same role as `gazebo_ros_control`. The ROS 2 path
keeps the MuJoCo plugin and the default `ros2_control` system plugin in this
package, similar in spirit to `gz_ros2_control`.

## Actuator Semantics

ROS controllers always own the command value. The hardware interface decides how
that command is applied to MuJoCo.

The current actuator mapping uses joint-name suffixes:

- `<joint>_act_eff` for effort commands
- `<joint>_act_vel` for velocity commands
- `<joint>_act_pos` for position commands

If a matching MuJoCo actuator exists and `ignore_actuators` is `false`, the
command is written to `mjData.ctrl` for that actuator.

If the matching actuator is missing, or `ignore_actuators` is `true`, commands
fall back to generalized force application through `mjData.qfrc_applied`.
Effort commands are applied directly. Position and velocity fallback require
configured gains.

## Configuration

ROS 1 plugin configuration is loaded through `MujocoPlugins`:

```yaml
MujocoPlugins:
  - type: mujoco_ros_control/MujocoRosControlPlugin
    hardware:
      type: mujoco_ros_control/DefaultRobotHWSim
      control_period: 0.02
      robot_description: robot_description
      ignore_actuators: false
```

ROS 1 position/velocity fallback gains use the existing PID namespace:

```yaml
mujoco_ros_control:
  pid_gains:
    position_hinge:
      p: 4.0
      i: 0.0
      d: 0.1
```

ROS 2 uses `mujoco_ros_control/MujocoRosControlPlugin` in `MujocoPlugins` and
`mujoco_ros_control/MujocoRosSystem` in the URDF `<ros2_control>` block:

```xml
<ros2_control name="MujocoRosSystem" type="system">
  <hardware>
    <plugin>mujoco_ros_control/MujocoRosSystem</plugin>
    <param name="ignore_actuators">false</param>
  </hardware>
</ros2_control>
```

ROS 2 fallback gains are joint parameters inside the `<ros2_control>` block,
for example `kp`, `kv`, and optional `effort_limit`.

## Examples

Both examples use the same MuJoCo model file: `example/pendulum.xml`.

The model contains four independent pendulums:

- `fallback_hinge`: no MuJoCo actuator, effort command falls back to `qfrc_applied`
- `motor_hinge`: effort command is routed to `motor_hinge_act_eff`
- `velocity_hinge`: velocity command is routed to `velocity_hinge_act_vel`
- `position_hinge`: position command is routed to `position_hinge_act_pos`

### Normal Actuator Routing

ROS 1:

```bash
roslaunch mujoco_ros_control mujoco_ros_control.launch
rostopic pub /fallback_effort_controller/command std_msgs/Float64 "data: 0.2" -r 20
rostopic pub /motor_effort_controller/command std_msgs/Float64 "data: 0.2" -r 20
rostopic pub /velocity_controller/command std_msgs/Float64 "data: 0.5" -r 20
rostopic pub /position_controller/command std_msgs/Float64 "data: 0.5" -r 20
```

ROS 2:

```bash
ros2 launch mujoco_ros_control mujoco_ros_control.launch.py
ros2 topic pub /fallback_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.2]}" -r 20
ros2 topic pub /motor_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.2]}" -r 20
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5]}" -r 20
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5]}" -r 20
```

### Ignore Actuators

The ignore-actuators examples load the same MuJoCo model, disable actuator
routing, and force all command interfaces through the generalized-force
fallback path.

ROS 1:

```bash
roslaunch mujoco_ros_control mujoco_ros_control_ignore_actuators.launch
```

ROS 2:

```bash
ros2 launch mujoco_ros_control mujoco_ros_control_ignore_actuators.launch.py
```

Use the same command topics as in the normal example.

## Licensing

This work is licensed under the BSD 3-Clause License (see `LICENSE`).
The original work it is based on was released under a BSD 3-Clause License
(see `LICENSE-ORIGINAL`).
