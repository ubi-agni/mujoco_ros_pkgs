/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#include <mujoco_ros_sensors/compound_sensors.hpp>

#include <algorithm>
#include <initializer_list>
#include <utility>

namespace mujoco_ros::sensors {

bool ParseCompoundKind(const std::string &type, CompoundKind &out)
{
	static const std::pair<const char *, CompoundKind> kKinds[] = {
		{ "IMU", CompoundKind::kIMU },     { "Twist", CompoundKind::kTwist },
		{ "Wrench", CompoundKind::kWrench }, { "Pose", CompoundKind::kPose },
		{ "Accel", CompoundKind::kAccel }, { "Odometry", CompoundKind::kOdometry },
	};
	for (const auto &[name, kind] : kKinds) {
		if (type == name) {
			out = kind;
			return true;
		}
	}
	return false;
}

const std::vector<std::string> &RequiredRoles(CompoundKind kind)
{
	static const std::vector<std::string> kImu      = { "linear_acceleration", "angular_velocity", "orientation" };
	static const std::vector<std::string> kLinAng   = { "linear", "angular" };
	static const std::vector<std::string> kWrench   = { "force", "torque" };
	static const std::vector<std::string> kPose     = { "position", "orientation" };
	static const std::vector<std::string> kOdometry = { "position", "orientation", "linear", "angular" };
	switch (kind) {
		case CompoundKind::kIMU:
			return kImu;
		case CompoundKind::kWrench:
			return kWrench;
		case CompoundKind::kPose:
			return kPose;
		case CompoundKind::kOdometry:
			return kOdometry;
		case CompoundKind::kTwist:
		case CompoundKind::kAccel:
			return kLinAng;
	}
	// Unreachable for valid enum values; an empty catalog makes callers fail loudly on bad input.
	static const std::vector<std::string> kNone;
	return kNone;
}

bool RoleAcceptsMjType(CompoundKind kind, const std::string &role, int t)
{
	auto any_of = [t](std::initializer_list<int> allowed) { return std::find(allowed.begin(), allowed.end(), t) != allowed.end(); };
	const bool quat = any_of({ mjSENS_FRAMEQUAT, mjSENS_BALLQUAT });

	switch (kind) {
		case CompoundKind::kIMU:
			if (role == "linear_acceleration")
				return any_of({ mjSENS_ACCELEROMETER, mjSENS_FRAMELINACC });
			if (role == "angular_velocity")
				return any_of({ mjSENS_GYRO, mjSENS_FRAMEANGVEL, mjSENS_BALLANGVEL });
			return role == "orientation" && quat;
		case CompoundKind::kTwist:
			if (role == "linear")
				return t == mjSENS_FRAMELINVEL;
			return role == "angular" && t == mjSENS_FRAMEANGVEL;
		case CompoundKind::kWrench:
			if (role == "force")
				return t == mjSENS_FORCE;
			return role == "torque" && t == mjSENS_TORQUE;
		case CompoundKind::kPose:
			if (role == "position")
				return t == mjSENS_FRAMEPOS;
			return role == "orientation" && quat;
		case CompoundKind::kAccel:
			if (role == "linear")
				return t == mjSENS_FRAMELINACC;
			return role == "angular" && t == mjSENS_FRAMEANGACC;
		case CompoundKind::kOdometry:
			if (role == "position" || role == "orientation")
				return RoleAcceptsMjType(CompoundKind::kPose, role, t);
			return RoleAcceptsMjType(CompoundKind::kTwist, role, t);
	}
	return false;
}

bool ResolveSensorBodyId(const mjModel *model, int sensor_id, int &body_id, std::string &error)
{
	if (!model || sensor_id < 0 || sensor_id >= model->nsensor) {
		error = "sensor id " + std::to_string(sensor_id) + " out of range";
		return false;
	}
	const char *name = mj_id2name(model, mjOBJ_SENSOR, sensor_id);
	const std::string label = name ? name : "#" + std::to_string(sensor_id);

	const int objid = model->sensor_objid[sensor_id];
	int body        = -1;
	switch (model->sensor_objtype[sensor_id]) {
		case mjOBJ_BODY:
		case mjOBJ_XBODY:
			body = (objid >= 0 && objid < model->nbody) ? objid : -1;
			break;
		case mjOBJ_SITE:
			body = (objid >= 0 && objid < model->nsite) ? model->site_bodyid[objid] : -1;
			break;
		case mjOBJ_GEOM:
			body = (objid >= 0 && objid < model->ngeom) ? model->geom_bodyid[objid] : -1;
			break;
		case mjOBJ_CAMERA:
			body = (objid >= 0 && objid < model->ncam) ? model->cam_bodyid[objid] : -1;
			break;
		case mjOBJ_JOINT:
			body = (objid >= 0 && objid < model->njnt) ? model->jnt_bodyid[objid] : -1;
			break;
		default:
			break;
	}
	if (body < 0) {
		error = "sensor '" + label + "' does not measure an object with a resolvable body";
		return false;
	}
	body_id = body;
	return true;
}

bool SameAnchorBody(const mjModel *model, int anchor_sensor_id, int orientation_sensor_id, std::string &error)
{
	int anchor_body = -1, orientation_body = -1;
	if (!ResolveSensorBodyId(model, anchor_sensor_id, anchor_body, error) ||
	    !ResolveSensorBodyId(model, orientation_sensor_id, orientation_body, error)) {
		return false;
	}
	if (anchor_body != orientation_body) {
		const char *a = mj_id2name(model, mjOBJ_BODY, anchor_body);
		const char *o = mj_id2name(model, mjOBJ_BODY, orientation_body);
		error = std::string("orientation sensor measures body '") + (o ? o : "?") + "' but the anchor measures body '" +
		        (a ? a : "?") + "'";
		return false;
	}
	return true;
}

} // namespace mujoco_ros::sensors
