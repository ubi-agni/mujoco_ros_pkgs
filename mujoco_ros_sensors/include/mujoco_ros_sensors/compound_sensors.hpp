/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#pragma once

#include <mujoco/mujoco.h>

#include <string>
#include <vector>

namespace mujoco_ros::sensors {

/// Compound stream kinds; each is selected by its exact YAML `type` string.
enum class CompoundKind { kIMU, kTwist, kWrench, kPose, kAccel, kOdometry };

/// Accepts only the exact YAML type strings (`IMU`, `Twist`, `Wrench`, `Pose`, `Accel`, `Odometry`).
bool ParseCompoundKind(const std::string &type, CompoundKind &out);

/// YAML role keys a compound of `kind` must define, in message order.
const std::vector<std::string> &RequiredRoles(CompoundKind kind);

/// True if an MJCF sensor of `mj_sensor_type` may fill `role` of `kind`. Unknown roles are rejected.
bool RoleAcceptsMjType(CompoundKind kind, const std::string &role, int mj_sensor_type);

/**
 * Body a sensor measures: site -> parent body, body/xbody -> that body,
 * geom/camera -> owning body, joint (ballquat, ballangvel) -> joint's body.
 * Returns false and sets `error` if the sensor's object cannot be resolved.
 */
bool ResolveSensorBodyId(const mjModel *model, int sensor_id, int &body_id, std::string &error);

/// Object-identity check: both sensors must measure the same body. Sets `error` on mismatch or unresolvable sensor.
bool SameAnchorBody(const mjModel *model, int anchor_sensor_id, int orientation_sensor_id, std::string &error);

} // namespace mujoco_ros::sensors
