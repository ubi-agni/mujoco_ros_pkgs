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

/// Types that never get a stream, publisher or hub entry (no meaningful ROS message).
bool IsIgnoredSensorType(int sensor_type);

/**
 * Number of sensordata values a native stream reads for `sensor_type`, or -1
 * if the type has no native stream. Init compares this with the model's
 * `sensor_dim` so a differently-sized sensor is skipped instead of truncated.
 */
int ExpectedNativeDim(int sensor_type);

/**
 * Resolves the `frame_id` of the gap-closing sensor types (FRAMEANGVEL,
 * TENDONACTFRC, E_*, INSIDESITE, GEOM*, CAMPROJECTION). Returns false and sets
 * `error` if the object this frame depends on is missing or unnamed; the caller
 * must then skip the sensor rather than publish a wrong frame.
 */
bool ResolveGapFrameId(const mjModel *model, int sensor_id, std::string &frame_id, std::string &error);

/**
 * Builds the stream that publishes mjSENS_* sensor `sensor_id` through the
 * publishers already registered on `config`. Returns nullptr for sensor types
 * without a native stream. `config` and `noise` must outlive the stream.
 */
std::unique_ptr<ISensorStream> MakeNativeStream(const mjModel *model, int sensor_id, std::string name,
                                                const SensorConfig &config, NoiseSource &noise, bool eval_mode);

} // namespace mujoco_ros::sensors
