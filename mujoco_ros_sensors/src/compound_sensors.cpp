/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#include <mujoco_ros_sensors/compound_sensors.hpp>

#include <mujoco_ros_sensors/sensor_stream.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#else
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#endif

#include <mujoco_ros/util.hpp>

#include <algorithm>
#include <array>
#include <functional>
#include <initializer_list>
#include <set>
#include <utility>

namespace mujoco_ros::sensors {

#if MJR_ROS_VERSION == ROS_1
using AccelStamped      = geometry_msgs::AccelStamped;
using PointStamped      = geometry_msgs::PointStamped;
using PoseStamped       = geometry_msgs::PoseStamped;
using QuaternionStamped = geometry_msgs::QuaternionStamped;
using TwistStamped      = geometry_msgs::TwistStamped;
using Vector3Stamped    = geometry_msgs::Vector3Stamped;
using WrenchStamped     = geometry_msgs::WrenchStamped;
using Imu               = sensor_msgs::Imu;
using Odometry          = nav_msgs::Odometry;
#else
using AccelStamped      = geometry_msgs::msg::AccelStamped;
using PointStamped      = geometry_msgs::msg::PointStamped;
using PoseStamped       = geometry_msgs::msg::PoseStamped;
using QuaternionStamped = geometry_msgs::msg::QuaternionStamped;
using TwistStamped      = geometry_msgs::msg::TwistStamped;
using Vector3Stamped    = geometry_msgs::msg::Vector3Stamped;
using WrenchStamped     = geometry_msgs::msg::WrenchStamped;
using Imu               = sensor_msgs::msg::Imu;
using Odometry          = nav_msgs::msg::Odometry;
#endif

bool ParseCompoundKind(const std::string &type, CompoundKind &out)
{
	static const std::pair<const char *, CompoundKind> kKinds[] = {
		{ "IMU", CompoundKind::kIMU },   { "Twist", CompoundKind::kTwist }, { "Wrench", CompoundKind::kWrench },
		{ "Pose", CompoundKind::kPose }, { "Accel", CompoundKind::kAccel }, { "Odometry", CompoundKind::kOdometry },
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
	auto any_of = [t](std::initializer_list<int> allowed) {
		return std::find(allowed.begin(), allowed.end(), t) != allowed.end();
	};
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
	const char *name        = mj_id2name(model, mjOBJ_SENSOR, sensor_id);
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

namespace {

int FindSensor(const mjModel *model, const std::string &name)
{
	return mj_name2id(model, mjOBJ_SENSOR, name.c_str());
}

bool ValidateCompound(const mjModel *model, const CompoundSpec &spec, const std::map<std::string, std::string> &frames,
                      std::set<std::string> &claimed, CompoundDescriptor &desc, std::string &error)
{
	if (!ParseCompoundKind(spec.type, desc.kind)) {
		error = "unknown compound type '" + spec.type + "' (expected IMU, Twist, Wrench, Pose, Accel or Odometry)";
		return false;
	}
	desc.name        = spec.name;
	desc.update_rate = spec.update_rate;

	const auto &required = RequiredRoles(desc.kind);
	for (const auto &[role, sensor] : spec.roles) {
		if (std::find(required.begin(), required.end(), role) == required.end()) {
			error = "unknown role '" + role + "' for type " + spec.type;
			return false;
		}
	}

	std::map<std::string, int> ids;
	for (const auto &role : required) {
		const auto it = spec.roles.find(role);
		if (it == spec.roles.end()) {
			error = "missing required role '" + role + "'";
			return false;
		}
		const std::string &sensor = it->second;
		const int id              = FindSensor(model, sensor);
		if (id < 0) {
			error = "role '" + role + "' names unknown sensor '" + sensor + "'";
			return false;
		}
		if (!RoleAcceptsMjType(desc.kind, role, model->sensor_type[id])) {
			error = "sensor '" + sensor + "' has a MuJoCo type that role '" + role + "' of " + spec.type + " rejects";
			return false;
		}
		if (frames.find(sensor) == frames.end()) {
			error = "sensor '" + sensor + "' has no native stream (skipped at init), cannot be claimed";
			return false;
		}
		if (!claimed.insert(sensor).second) {
			error = "sensor '" + sensor + "' is already claimed by another role or compound";
			return false;
		}
		ids[role]        = id;
		desc.roles[role] = sensor;
	}

	auto frame_of     = [&](const std::string &role) { return frames.at(desc.roles.at(role)); };
	auto shared_frame = [&](const std::string &a, const std::string &b, std::string &frame) {
		if (frame_of(a) != frame_of(b)) {
			error = "roles '" + a + "' and '" + b + "' must share one frame_id but have '" + frame_of(a) + "' and '" +
			        frame_of(b) + "'";
			return false;
		}
		frame = frame_of(a);
		return true;
	};
	auto same_body = [&](const std::string &anchor_role) {
		return SameAnchorBody(model, ids.at(anchor_role), ids.at("orientation"), error);
	};

	switch (desc.kind) {
		case CompoundKind::kWrench:
			return shared_frame("force", "torque", desc.frame_id);
		case CompoundKind::kTwist:
		case CompoundKind::kAccel:
			return shared_frame("linear", "angular", desc.frame_id);
		case CompoundKind::kIMU:
			return shared_frame("linear_acceleration", "angular_velocity", desc.frame_id) &&
			       same_body("linear_acceleration");
		case CompoundKind::kPose:
			desc.frame_id = frame_of("position");
			return same_body("position");
		case CompoundKind::kOdometry:
			desc.frame_id = frame_of("position");
			return same_body("position") && shared_frame("linear", "angular", desc.child_frame_id);
	}
	return false;
}

} // namespace

bool ValidateCompounds(const mjModel *model, const std::vector<CompoundSpec> &specs,
                       const std::map<std::string, std::string> &part_frame_ids, std::vector<CompoundDescriptor> &out,
                       std::string &error)
{
	out.clear();
	std::set<std::string> names, claimed;
	for (const auto &spec : specs) {
		auto fail = [&](const std::string &why) {
			error = "compound_sensors entry '" + spec.name + "': " + why;
			out.clear();
			return false;
		};
		if (spec.name.empty()) {
			return fail("name must not be empty");
		}
		if (!names.insert(spec.name).second) {
			return fail("duplicate compound name");
		}
		if (FindSensor(model, spec.name) >= 0) {
			return fail("name collides with an MJCF sensor of the same name");
		}
		CompoundDescriptor desc;
		std::string why;
		if (!ValidateCompound(model, spec, part_frame_ids, claimed, desc, why)) {
			return fail(why);
		}
		out.push_back(std::move(desc));
	}
	return true;
}

namespace {

/// A compound message: header + every part's current (or ground-truth) message, in role order.
template <typename MsgT, typename... Parts>
class ComposedStream : public PublishingStream<MsgT>
{
public:
	using Fill = std::function<void(MsgT &, const Parts &...)>;

	ComposedStream(std::string name, const SensorConfig &config, bool eval_mode,
	               std::array<ISensorStream *, sizeof...(Parts)> parts, Fill fill)
	    : PublishingStream<MsgT>(std::move(name), config, eval_mode), parts_(parts), fill_(std::move(fill))
	{
	}

protected:
	// Parts cache per step, so value/GT built here share one noise sample set with any other reader of the part.
	MsgT Noisy(const mjModel *model, const mjData *data) override
	{
		return ComposeFrom(data, std::index_sequence_for<Parts...>{}, [&](ISensorStream &p) -> const std::any & {
			return p.Value(model, data);
		});
	}

	MsgT Truth(const mjModel *model, const mjData *data) override
	{
		return ComposeFrom(data, std::index_sequence_for<Parts...>{}, [&](ISensorStream &p) -> const std::any & {
			return p.GroundTruth(model, data);
		});
	}

private:
	template <std::size_t... I, typename Get>
	MsgT ComposeFrom(const mjData *data, std::index_sequence<I...>, Get get) const
	{
		MsgT msg;
		msg.header.frame_id = this->config().frame_id;
		msg.header.stamp    = util::toRosTime(util::simTimeToNanoseconds(data->time));
		fill_(msg, std::any_cast<const Parts &>(get(*parts_[I]))...);
		return msg;
	}

	std::array<ISensorStream *, sizeof...(Parts)> parts_;
	Fill fill_;
};

/// ROS "unknown covariance" marker: first element -1, rest 0. Never derived from noise sigma.
template <typename Cov>
void MarkCovarianceUnknown(Cov &covariance)
{
	covariance.fill(0.0);
	covariance[0] = -1.0;
}

} // namespace

bool HasCompoundStream(CompoundKind kind)
{
	return kind == CompoundKind::kWrench || kind == CompoundKind::kTwist || kind == CompoundKind::kPose ||
	       kind == CompoundKind::kAccel || kind == CompoundKind::kIMU || kind == CompoundKind::kOdometry;
}

std::unique_ptr<ISensorStream> MakeCompoundStream(const CompoundDescriptor &desc, const SensorConfig &config,
                                                  const std::map<std::string, ISensorStream *> &parts, bool eval_mode)
{
	auto part = [&](const char *role) { return parts.at(role); };
	switch (desc.kind) {
		case CompoundKind::kWrench:
			return std::make_unique<ComposedStream<WrenchStamped, Vector3Stamped, Vector3Stamped>>(
			    desc.name, config, eval_mode, std::array{ part("force"), part("torque") },
			    [](WrenchStamped &m, const Vector3Stamped &force, const Vector3Stamped &torque) {
				    m.wrench.force  = force.vector;
				    m.wrench.torque = torque.vector;
			    });
		case CompoundKind::kTwist:
			return std::make_unique<ComposedStream<TwistStamped, Vector3Stamped, Vector3Stamped>>(
			    desc.name, config, eval_mode, std::array{ part("linear"), part("angular") },
			    [](TwistStamped &m, const Vector3Stamped &linear, const Vector3Stamped &angular) {
				    m.twist.linear  = linear.vector;
				    m.twist.angular = angular.vector;
			    });
		case CompoundKind::kAccel:
			return std::make_unique<ComposedStream<AccelStamped, Vector3Stamped, Vector3Stamped>>(
			    desc.name, config, eval_mode, std::array{ part("linear"), part("angular") },
			    [](AccelStamped &m, const Vector3Stamped &linear, const Vector3Stamped &angular) {
				    m.accel.linear  = linear.vector;
				    m.accel.angular = angular.vector;
			    });
		case CompoundKind::kPose:
			return std::make_unique<ComposedStream<PoseStamped, PointStamped, QuaternionStamped>>(
			    desc.name, config, eval_mode, std::array{ part("position"), part("orientation") },
			    [](PoseStamped &m, const PointStamped &position, const QuaternionStamped &orientation) {
				    m.pose.position    = position.point;
				    m.pose.orientation = orientation.quaternion;
			    });
		case CompoundKind::kIMU:
			return std::make_unique<ComposedStream<Imu, Vector3Stamped, Vector3Stamped, QuaternionStamped>>(
			    desc.name, config, eval_mode,
			    std::array{ part("linear_acceleration"), part("angular_velocity"), part("orientation") },
			    [](Imu &m, const Vector3Stamped &acceleration, const Vector3Stamped &angular_velocity,
			       const QuaternionStamped &orientation) {
				    m.linear_acceleration = acceleration.vector;
				    m.angular_velocity    = angular_velocity.vector;
				    m.orientation         = orientation.quaternion;
				    MarkCovarianceUnknown(m.orientation_covariance);
				    MarkCovarianceUnknown(m.angular_velocity_covariance);
				    MarkCovarianceUnknown(m.linear_acceleration_covariance);
			    });
		case CompoundKind::kOdometry:
			return std::make_unique<
			    ComposedStream<Odometry, PointStamped, QuaternionStamped, Vector3Stamped, Vector3Stamped>>(
			    desc.name, config, eval_mode,
			    std::array{ part("position"), part("orientation"), part("linear"), part("angular") },
			    [child = desc.child_frame_id](Odometry &m, const PointStamped &position,
			                                  const QuaternionStamped &orientation, const Vector3Stamped &linear,
			                                  const Vector3Stamped &angular) {
				    m.child_frame_id         = child;
				    m.pose.pose.position     = position.point;
				    m.pose.pose.orientation  = orientation.quaternion;
				    m.twist.twist.linear     = linear.vector;
				    m.twist.twist.angular    = angular.vector;
				    MarkCovarianceUnknown(m.pose.covariance);
				    MarkCovarianceUnknown(m.twist.covariance);
			    });
	}
	return nullptr;
}

} // namespace mujoco_ros::sensors
