/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: David P. Leins */

#include <mujoco_ros_laser/laser.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <pluginlib/class_list_macros.h>
#else
#include <pluginlib/class_list_macros.hpp>
#endif

#include <mujoco_ros/logging.hpp>
#include <mujoco_ros/util.hpp>

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace mujoco_ros;

namespace mujoco_ros::sensors::laser {

namespace {
static constexpr std::size_t LASER_PUBLISHER_QUEUE_SIZE = 10;

#if MJR_ROS_VERSION == ROS_1
template <typename MessageT>
ros::Publisher CreatePublisher(ros::NodeHandle &node, const std::string &topic)
{
	return node.advertise<MessageT>(topic, LASER_PUBLISHER_QUEUE_SIZE);
}

template <typename MessageT>
void Publish(const mujoco_ros::sensors::SensorConfig &config, const MessageT &message)
{
	config.value_pub.publish(message);
}

double SecondsSince(const RosTime &now, const RosTime &then)
{
	return (now - then).toSec();
}

bool ReadOptionalBoolFromConfig(const XmlRpc::XmlRpcValue &config, const std::string &name, bool default_value)
{
	if (config.hasMember(name) && config[name].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
		return static_cast<bool>(config[name]);
	}
	return default_value;
}

std::string ReadOptionalStringFromConfig(const XmlRpc::XmlRpcValue &config, const std::string &name,
                                         const std::string &default_value)
{
	if (config.hasMember(name) && config[name].getType() == XmlRpc::XmlRpcValue::TypeString) {
		return static_cast<std::string>(config[name]);
	}
	return default_value;
}

double ReadOptionalDoubleFromConfig(const XmlRpc::XmlRpcValue &config, const std::string &name, double default_value)
{
	if (!config.hasMember(name)) {
		return default_value;
	}

	if (config[name].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
		return static_cast<double>(config[name]);
	}
	if (config[name].getType() == XmlRpc::XmlRpcValue::TypeInt) {
		return static_cast<int>(config[name]);
	}
	return default_value;
}
#else
template <typename MessageT>
typename rclcpp::Publisher<MessageT>::SharedPtr CreatePublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                                                                const std::string &topic)
{
	return node->create_publisher<MessageT>(topic, rclcpp::QoS(LASER_PUBLISHER_QUEUE_SIZE));
}

template <typename MessageT>
void Publish(const mujoco_ros::sensors::SensorConfig &config, const MessageT &message)
{
	std::any_cast<typename rclcpp::Publisher<MessageT>::SharedPtr>(config.value_pub)->publish(message);
}

double SecondsSince(const RosTime &now, const RosTime &then)
{
	return (now - then).seconds();
}

std::string SensorParamName(const std::string &plugin_name, const std::string &sensor_name,
                            const std::string &param_name)
{
	return "MujocoPlugins." + plugin_name + "." + sensor_name + "." + param_name;
}

bool GetBoolParam(const rclcpp::Node *node, const std::string &name, bool default_value)
{
	if (!node->has_parameter(name)) {
		return default_value;
	}
	return node->get_parameter(name).as_bool();
}

std::string GetStringParam(const rclcpp::Node *node, const std::string &name, const std::string &default_value)
{
	if (!node->has_parameter(name)) {
		return default_value;
	}
	return node->get_parameter(name).as_string();
}

double GetDoubleParam(const rclcpp::Node *node, const std::string &name, double default_value)
{
	if (!node->has_parameter(name)) {
		return default_value;
	}

	const auto parameter = node->get_parameter(name);
	if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
		return static_cast<double>(parameter.as_int());
	}
	return parameter.as_double();
}
#endif

void InitRays(LaserConfig &config)
{
	config.nrays = static_cast<uint>((config.max_angle - config.min_angle) / config.angular_resolution);
	config.rays  = new mjtNum[config.nrays * 3];
	for (uint i = 0; i < config.nrays; ++i) {
		mjtNum ray[3] = { mju_cos(config.min_angle + i * config.angular_resolution),
			               mju_sin(config.min_angle + i * config.angular_resolution), 0 };
		mju_copy3(config.rays + i * 3, ray);
	}
}
} // namespace

#if MJR_ROS_VERSION == ROS_1
LaserConfig::LaserConfig(const XmlRpc::XmlRpcValue &config, const std::string &frame_id, const std::string &name,
                         int site_attached)
    : SensorConfig(ReadOptionalStringFromConfig(config, "frame_id", frame_id))
    , name(name)
    , site_attached(site_attached)
    , visualize(ReadOptionalBoolFromConfig(config, "visualize", DEFAULT_VISUALIZE))
    , update_rate(ReadOptionalDoubleFromConfig(config, "update_rate", DEFAULT_UPDATE_RATE))
    , min_range(ReadOptionalDoubleFromConfig(config, "min_range", DEFAULT_MIN_RANGE))
    , max_range(ReadOptionalDoubleFromConfig(config, "max_range", DEFAULT_MAX_RANGE))
    , range_resolution(ReadOptionalDoubleFromConfig(config, "range_resolution", DEFAULT_RANGE_RESOLUTION))
    , angular_resolution(ReadOptionalDoubleFromConfig(config, "angular_resolution", DEFAULT_ANGULAR_RESOLUTION))
    , min_angle(ReadOptionalDoubleFromConfig(config, "min_angle", DEFAULT_MIN_ANGLE))
    , max_angle(ReadOptionalDoubleFromConfig(config, "max_angle", DEFAULT_MAX_ANGLE))
{
	this->sigma[0] = ReadOptionalDoubleFromConfig(config, "sensor_std", DEFAULT_SENSOR_STD);
	if (this->sigma[0] > 0) {
		this->is_set = 1;
	}
	InitRays(*this);
}
#endif

LaserConfig::LaserConfig(std::string frame_id, std::string name, int site_attached, bool visualize, double update_rate,
                         double min_range, double max_range, double range_resolution, double angular_resolution,
                         double min_angle, double max_angle, double sensor_std)
    : SensorConfig(std::move(frame_id))
    , name(std::move(name))
    , site_attached(site_attached)
    , visualize(visualize)
    , update_rate(update_rate)
    , min_range(min_range)
    , max_range(max_range)
    , range_resolution(range_resolution)
    , angular_resolution(angular_resolution)
    , min_angle(min_angle)
    , max_angle(max_angle)
{
	this->sigma[0] = sensor_std;
	if (this->sigma[0] > 0) {
		this->is_set = 1;
	}
	InitRays(*this);
}

namespace {
// Defined below next to the stream class; Load only needs to construct streams.
std::unique_ptr<mujoco_ros::sensors::ISensorStream> MakeLaserStream(const LaserConfig &config, mjvGeom *geoms,
                                                                    std::mt19937 &generator,
                                                                    std::normal_distribution<double> &noise_dist);
} // namespace

bool LaserPlugin::Load(const mjModel *model, mjData *data)
{
	MJR_INFO_NAMED("lasers", "Loading laser plugin ...");

#if MJR_ROS_VERSION == ROS_1
	std::string lasers_namespace;
	if (rosparam_config_.hasMember("namespace")) {
		lasers_namespace = static_cast<std::string>(rosparam_config_["namespace"]);
	}
	lasers_nh_ = ros::NodeHandle(lasers_namespace);
#else
	lasers_nh_ = get_node();
#endif

	noise_dist = std::normal_distribution<double>(0.0, 1.0);

#if MJR_ROS_VERSION == ROS_1
	if (!rosparam_config_.hasMember("sensors")) {
		MJR_ERROR_NAMED("lasers", "Laser plugin needs to configure at least one sensor!");
		return false;
	}

	if (rosparam_config_["sensors"].getType() != XmlRpc::XmlRpcValue::TypeArray) {
		MJR_ERROR_NAMED("lasers", "Sensors config is not an array!");
		return false;
	}

	for (uint i = 0; i < rosparam_config_["sensors"].size(); i++) {
		if (!InitSensor(model, rosparam_config_["sensors"][i])) {
			return false;
		}
	}
#else
	const std::string sensors_param = "MujocoPlugins." + get_name() + ".sensors";
	if (!env_ptr_->has_parameter(sensors_param)) {
		MJR_ERROR_STREAM_NAMED("lasers", "Laser plugin needs sensor names in parameter '" << sensors_param << "'");
		return false;
	}

	const auto sensor_names = env_ptr_->get_parameter(sensors_param).as_string_array();
	for (const auto &sensor_name : sensor_names) {
		if (!InitSensor(model, sensor_name)) {
			return false;
		}
	}
#endif

	// Two lasers of this plugin sharing a name would shadow each other on the hub (and share one topic).
	{
		std::set<std::string> seen_names;
		for (const auto &laser_config : laser_configs_) {
			if (!seen_names.insert(laser_config->name).second) {
				MJR_ERROR_STREAM_NAMED("lasers", "Duplicate laser name '" << laser_config->name
				                                                          << "' within the laser plugin config");
				return false;
			}
		}
	}

	ngeom_ = 0;
	for (const auto &laser_config : laser_configs_) {
		if (laser_config->visualize) {
			ngeom_ += laser_config->nrays;
		}
	}

	delete[] laser_geoms_;
	laser_geoms_ = new mjvGeom[ngeom_];

	int geom_offset = 0;
	for (const auto &laser_config : laser_configs_) {
		mjvGeom *geoms = nullptr;
		if (laser_config->visualize) {
			geoms = laser_geoms_ + geom_offset;
			geom_offset += static_cast<int>(laser_config->nrays);
		}
		streams_.push_back(MakeLaserStream(*laser_config, geoms, rand_generator, noise_dist));
	}

	m_ = model;
	d_ = data;

	// A public name already taken by a native stream (or another provider) must fail the load loudly: silently
	// shadowing a stream would make hub lookups ambiguous.
	try {
		env_ptr_->sensors().RegisterProvider(*this);
		hub_registered_ = true;
	} catch (const std::exception &e) {
		MJR_ERROR_STREAM_NAMED("lasers", "Could not register laser streams on the sensor hub: " << e.what());
		return false;
	}
	return true;
}

#if MJR_ROS_VERSION == ROS_1
bool LaserPlugin::InitSensor(const mjModel *model, const XmlRpc::XmlRpcValue &config)
{
	if (!config.hasMember("site_attached")) {
		MJR_ERROR_NAMED("lasers",
		                "Laser plugin needs to configure where to attach the laser with the `site_attached` parameter!");
		return false;
	}

	const std::string site_attached = static_cast<std::string>(config["site_attached"]);
	const std::string topic_name    = ReadOptionalStringFromConfig(config, "name", site_attached + "_laser");
#else
bool LaserPlugin::InitSensor(const mjModel *model, const std::string &sensor_name)
{
	const std::string site_param = SensorParamName(get_name(), sensor_name, "site_attached");
	if (!env_ptr_->has_parameter(site_param)) {
		MJR_ERROR_STREAM_NAMED("lasers", "Laser sensor '" << sensor_name << "' needs a 'site_attached' parameter!");
		return false;
	}

	const std::string site_attached = env_ptr_->get_parameter(site_param).as_string();
	const std::string topic_name =
	    GetStringParam(env_ptr_, SensorParamName(get_name(), sensor_name, "name"), sensor_name);
#endif

	int site_id = mj_name2id(const_cast<mjModel *>(model), mjOBJ_SITE, site_attached.c_str());
	if (site_id == -1) {
		MJR_ERROR_STREAM_NAMED("lasers", "Site with name '" << site_attached << "' could not be found in the model!");
		return false;
	}

	const int parent_id          = model->site_bodyid[site_id];
	const std::string body_frame = mj_id2name(const_cast<mjModel *>(model), mjOBJ_BODY, parent_id);
#if MJR_ROS_VERSION == ROS_1
	LaserConfig laser_config(config, body_frame, topic_name, site_id);
#else
	const std::string frame_id =
	    GetStringParam(env_ptr_, SensorParamName(get_name(), sensor_name, "frame_id"), body_frame);
	LaserConfig laser_config(
	    frame_id, topic_name, site_id,
	    GetBoolParam(env_ptr_, SensorParamName(get_name(), sensor_name, "visualize"), DEFAULT_VISUALIZE),
	    GetDoubleParam(env_ptr_, SensorParamName(get_name(), sensor_name, "update_rate"), DEFAULT_UPDATE_RATE),
	    GetDoubleParam(env_ptr_, SensorParamName(get_name(), sensor_name, "min_range"), DEFAULT_MIN_RANGE),
	    GetDoubleParam(env_ptr_, SensorParamName(get_name(), sensor_name, "max_range"), DEFAULT_MAX_RANGE),
	    GetDoubleParam(env_ptr_, SensorParamName(get_name(), sensor_name, "range_resolution"), DEFAULT_RANGE_RESOLUTION),
	    GetDoubleParam(env_ptr_, SensorParamName(get_name(), sensor_name, "angular_resolution"),
	                   DEFAULT_ANGULAR_RESOLUTION),
	    GetDoubleParam(env_ptr_, SensorParamName(get_name(), sensor_name, "min_angle"), DEFAULT_MIN_ANGLE),
	    GetDoubleParam(env_ptr_, SensorParamName(get_name(), sensor_name, "max_angle"), DEFAULT_MAX_ANGLE),
	    GetDoubleParam(env_ptr_, SensorParamName(get_name(), sensor_name, "sensor_std"), DEFAULT_SENSOR_STD));
#endif

	laser_config.RegisterPub(CreatePublisher<LaserScan>(lasers_nh_, topic_name));
	laser_configs_.push_back(std::make_unique<LaserConfig>(laser_config));
	return true;
}

void LaserPlugin::RenderCallback(const mjModel * /*model*/, mjData * /*data*/, mjvScene *scene)
{
	if (!HasRenderData()) {
		return;
	}

	for (uint i = 0; i < std::min(ngeom_, scene->maxgeom - scene->ngeom); ++i) {
		scene->geoms[scene->ngeom++] = laser_geoms_[i];
	}
}

void ProcessRay(std::mt19937 &rand_generator, std::normal_distribution<double> &noise_dist,
                const LaserConfig &laser_config, mjvGeom *geom, const mjtNum (&pos)[3], const mjtNum *dir, mjtNum dist,
                const float (&rgba)[4], LaserScan &scan_msg, int ray_idx)
{
	if (dist < 0) {
		dist = laser_config.max_range;
	}

	if (laser_config.is_set) {
		dist += (laser_config.sigma[0] * noise_dist(rand_generator)) * dist;
	}

	dist                     = std::min(std::max(laser_config.min_range, dist), laser_config.max_range);
	scan_msg.ranges[ray_idx] = dist;

	if (geom == nullptr) {
		return;
	}

	mjtNum target[3];
	mju_addScl3(target, pos, dir, dist);
	mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba);
	mjv_connector(geom, mjGEOM_LINE, 1., pos, target);
}

namespace {

/**
 * One laser = one public custom stream. The rate gate is per stream and runs against sim time, so a rewind
 * (sim reset) must clear it, otherwise scans would be skipped until sim time catches up with the old last publish.
 * Scans are built and published regardless of subscribers: visualization geoms come out of the same build.
 */
class LaserStream : public mujoco_ros::sensors::ISensorStream
{
public:
	LaserStream(const LaserConfig &config, mjvGeom *geoms, std::mt19937 &generator,
	            std::normal_distribution<double> &noise_dist)
	    : ISensorStream(config.name, config, true)
	    , laser_(config)
	    , geoms_(geoms)
	    , generator_(generator)
	    , noise_dist_(noise_dist)
	{
	}

	void MaybePublish(const mjModel *model, const mjData *data) override
	{
		computed_         = false;
		const RosTime now = util::toRosTime(util::simTimeToNanoseconds(data->time));
		if (has_last_ && now < last_) {
			MJR_DEBUG_NAMED("lasers", "Sim time moved backward, resetting laser rate gate");
			has_last_ = false;
		}
		if (has_last_ && SecondsSince(now, last_) < 1.0 / laser_.update_rate) {
			return;
		}

		computed_ = true;
		Publish<LaserScan>(laser_, std::any_cast<const LaserScan &>(Value(model, data)));
		last_     = now;
		has_last_ = true;
		value_publish_count_.fetch_add(1, std::memory_order_release);
	}

	std::any BuildValue(const mjModel *model, const mjData *data) override
	{
		static const float rgba[4]                   = { 0., 0., 1., 0.8 };
		static const mjtByte ignore_groups[mjNGROUP] = { 1, 0, 1, 1, 1, 1 };

		LaserScan scan_msg;
		scan_msg.header.stamp    = util::toRosTime(util::simTimeToNanoseconds(data->time));
		scan_msg.header.frame_id = laser_.frame_id;
		scan_msg.angle_min       = laser_.min_angle;
		scan_msg.angle_max       = laser_.max_angle;
		scan_msg.angle_increment = laser_.angular_resolution;
		scan_msg.range_min       = laser_.min_range;
		scan_msg.range_max       = laser_.max_range;
		scan_msg.ranges.resize(laser_.nrays);

		mjtNum rot[9];
		mju_copy(rot, data->site_xmat + 9 * laser_.site_attached, 9);

		// Rotate all rays into the world frame, then cast them in one serial mj_multiRay call.
		// Static geoms are included. The mount body is excluded. The cutoff is effectively unbounded.
		mjtNum pos[3];
		mju_copy3(pos, data->site_xpos + 3 * laser_.site_attached);
		std::vector<mjtNum> dirs(3 * laser_.nrays);
		std::vector<mjtNum> dists(laser_.nrays);
		std::vector<int> geomids(laser_.nrays);
		for (uint i = 0; i < laser_.nrays; ++i) {
			mju_mulMatVec3(dirs.data() + 3 * i, rot, laser_.rays + 3 * i);
		}
		// Exclude the mount body. mj_multiRay takes one body id, so the rest of the kinematic chain can still be hit.
		const int mount_body = model->site_bodyid[laser_.site_attached];
		// mj_multiRay only reads the kinematics but its C signature takes a mutable mjData.
		mj_multiRay(model, const_cast<mjData *>(data), pos, dirs.data(), ignore_groups, 1, mount_body, geomids.data(),
		            dists.data(), nullptr, static_cast<int>(laser_.nrays), mjMAXVAL);

		for (uint i = 0; i < laser_.nrays; ++i) {
			mjvGeom *g = geoms_ == nullptr ? nullptr : geoms_ + i;
			ProcessRay(generator_, noise_dist_, laser_, g, pos, dirs.data() + 3 * i, dists[i], rgba, scan_msg,
			           static_cast<int>(i));
		}
		has_render_data_ = has_render_data_ || laser_.visualize;
		return scan_msg;
	}

	void ResetGate()
	{
		has_last_ = false;
		computed_ = false;
	}
	bool ComputedLastStep() const { return computed_; }
	bool HasRenderData() const { return has_render_data_; }

private:
	const LaserConfig &laser_;
	mjvGeom *geoms_;
	std::mt19937 &generator_;
	std::normal_distribution<double> &noise_dist_;
	RosTime last_;
	bool has_last_        = false;
	bool computed_        = false;
	bool has_render_data_ = false;
};

std::unique_ptr<mujoco_ros::sensors::ISensorStream> MakeLaserStream(const LaserConfig &config, mjvGeom *geoms,
                                                                    std::mt19937 &generator,
                                                                    std::normal_distribution<double> &noise_dist)
{
	return std::make_unique<LaserStream>(config, geoms, generator, noise_dist);
}

} // namespace

LaserPlugin::LaserPlugin() = default;

void LaserPlugin::LastStageCallback(const mjModel *model, mjData *data)
{
	bool any_computed = false;
	for (auto &stream : streams_) {
		// New step: drop last step's cache so publish and reads share one build (one noise sample set) per step.
		stream->InvalidateCache();
		stream->MaybePublish(model, data);
		any_computed = any_computed || static_cast<LaserStream &>(*stream).ComputedLastStep();
	}
	// Steps where every laser was rate-gated are nearly free and would skew the callback-time average.
	skip_ema_ = !streams_.empty() && !any_computed;
}

void LaserPlugin::Reset()
{
	for (auto &stream : streams_) {
		stream->InvalidateCache();
		static_cast<LaserStream &>(*stream).ResetGate();
	}
}

std::vector<LaserConfig> LaserPlugin::GetLaserConfigs() const
{
	std::vector<LaserConfig> configs;
	configs.reserve(laser_configs_.size());
	for (const auto &config : laser_configs_) {
		configs.push_back(*config);
	}
	return configs;
}

bool LaserPlugin::HasRenderData() const
{
	return std::any_of(streams_.begin(), streams_.end(),
	                   [](const auto &stream) { return static_cast<LaserStream &>(*stream).HasRenderData(); });
}

mujoco_ros::SensorStreamBase *LaserPlugin::Find(const std::string &name) const
{
	for (const auto &stream : streams_) {
		if (stream->name() == name && stream->is_public()) {
			return stream.get();
		}
	}
	return nullptr;
}

std::vector<std::string> LaserPlugin::PublicNames() const
{
	std::vector<std::string> names;
	for (const auto &stream : streams_) {
		if (stream->is_public()) {
			names.push_back(stream->name());
		}
	}
	return names;
}

LaserPlugin::~LaserPlugin()
{
	if (hub_registered_) {
		env_ptr_->sensors().UnregisterProvider(*this);
	}
	streams_.clear();
	delete[] laser_geoms_;
}

} // namespace mujoco_ros::sensors::laser

PLUGINLIB_EXPORT_CLASS(mujoco_ros::sensors::laser::LaserPlugin, mujoco_ros::MujocoPlugin)
