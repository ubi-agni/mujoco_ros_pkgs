/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#pragma once

#include <mujoco/mujoco.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mujoco_ros::sensors {

/// Compound stream kinds; each is selected by its exact YAML `type` string.
enum class CompoundKind
{
	kIMU,
	kTwist,
	kWrench,
	kPose,
	kAccel,
	kOdometry
};

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

/**
 * Body a position/orientation sensor is expressed relative to: framepos/framequat `reftype`/`refid` (site -> its body,
 * none -> world body 0), ballquat -> its joint's parent body. Returns false and sets `error` otherwise.
 */
bool ResolveSensorReferenceBody(const mjModel *model, int sensor_id, int &body_id, std::string &error);

/// Both sensors must be expressed relative to the same body; sets `error` on mismatch or unresolvable sensor.
bool SameReferenceBody(const mjModel *model, int position_sensor_id, int orientation_sensor_id, std::string &error);

/// One `compound_sensors` entry as written in the plugin config, before any validation.
struct CompoundSpec
{
	std::string name;
	std::string type;
	/// Every config key except `name`, `type` and `update_rate`, mapped to the MJCF sensor name it points at.
	std::map<std::string, std::string> roles;
	double update_rate = 0.0;
};

/// A validated compound: all roles resolved to existing sensors with consistent frames and bodies.
struct CompoundDescriptor
{
	std::string name;
	CompoundKind kind = CompoundKind::kIMU;
	/// Role key -> MJCF sensor name; holds exactly RequiredRoles(kind).
	std::map<std::string, std::string> roles;
	/// Message `header.frame_id`.
	std::string frame_id;
	/// Odometry `child_frame_id`; empty for every other kind.
	std::string child_frame_id;
	double update_rate = 0.0;
};

/**
 * Validates every spec against the model and the native sensors' frame ids (`part_frame_ids`: sensor name -> frame_id,
 * only sensors that got a native stream). Fails on the first problem with `error` set and `out` left empty: unknown
 * type, compound name clash, missing/unknown role key, unknown or unstreamed sensor, wrong MuJoCo type, a sensor
 * claimed twice, a name that is no valid topic token, frame mismatch (Wrench/Twist/Accel/IMU/Odometry twist pair),
 * roles on different bodies (IMU, Pose, Odometry), orientation relative to another reference body than the position
 * (Pose, Odometry), or an IMU ballquat whose joint parent is not the world body.
 */
bool ValidateCompounds(const mjModel *model, const std::vector<CompoundSpec> &specs,
                       const std::map<std::string, std::string> &part_frame_ids, std::vector<CompoundDescriptor> &out,
                       std::string &error);

class ISensorStream;
struct SensorConfig;

/**
 * Builds the public stream of a validated compound from its already built part streams (`parts`: role -> stream).
 * Value and ground truth are composed from the parts' own per-step cached Value()/GroundTruth(), so the parts must
 * outlive the compound and be invalidated before it publishes each step. `config` carries the compound's frame_id,
 * update_rate and publishers (msg type per kind: WrenchStamped, TwistStamped, PoseStamped, AccelStamped,
 * sensor_msgs/Imu, nav_msgs/Odometry; IMU/Odometry covariances are left unknown, first element -1).
 */
std::unique_ptr<ISensorStream> MakeCompoundStream(const CompoundDescriptor &desc, const SensorConfig &config,
                                                  const std::map<std::string, ISensorStream *> &parts, bool eval_mode);

} // namespace mujoco_ros::sensors
