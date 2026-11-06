/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#pragma once

#include <mujoco_ros_sensors/sensor_stream.hpp>

#include <mujoco/mujoco.h>

#include <memory>
#include <string>

namespace mujoco_ros::sensors {

/**
 * Builds the stream that publishes mjSENS_* sensor `sensor_id` through the
 * publishers already registered on `config`. Returns nullptr for sensor types
 * without a native stream. `config` and `noise` must outlive the stream.
 */
std::unique_ptr<ISensorStream> MakeNativeStream(const mjModel *model, int sensor_id, std::string name,
                                                const SensorConfig &config, NoiseSource &noise, bool eval_mode);

} // namespace mujoco_ros::sensors
