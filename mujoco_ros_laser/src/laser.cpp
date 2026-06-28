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

#include <algorithm>
#include <string>

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

RosTime Now(ros::NodeHandle & /*node*/)
{
	return ros::Time::now();
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

RosTime Now(rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
{
	return node->now();
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

struct ProcessRayArgs
{
	const mjModel *model;
	mjData *data;
	std::mt19937 *rand_generator;
	std::normal_distribution<double> *noise_dist;
	const LaserConfig *laser_config;
	mjvGeom *geom;
	mjtNum (*rot)[9];
	mjtByte (*ignore_groups)[mjNGROUP];
	const float (*rgba)[4];
	LaserScan *scan_msg;
	int ray_idx;
};

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

	ngeom_ = 0;
	for (const auto &laser_config : laser_configs_) {
		if (laser_config.visualize) {
			ngeom_ += laser_config.nrays;
		}
	}

	delete[] laser_geoms_;
	laser_geoms_ = new mjvGeom[ngeom_];

	m_ = model;
	d_ = data;
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
	laser_configs_.push_back(laser_config);
	return true;
}

void LaserPlugin::RenderCallback(const mjModel * /*model*/, mjData * /*data*/, mjvScene *scene)
{
	if (!has_render_data_) {
		return;
	}

	for (uint i = 0; i < std::min(ngeom_, scene->maxgeom - scene->ngeom); ++i) {
		scene->geoms[scene->ngeom++] = laser_geoms_[i];
	}
}

void ProcessRay(const mjModel *model, mjData *data, std::mt19937 rand_generator,
                std::normal_distribution<double> &noise_dist, const LaserConfig &laser_config, mjvGeom *geom,
                mjtNum (&rot)[9], const mjtByte (&ignore_groups)[mjNGROUP], const float (&rgba)[4], LaserScan &scan_msg,
                int ray_idx)
{
	mjtNum pos[3], target[3];
	mju_copy(pos, laser_config.rays + 3 * ray_idx, 3);
	mju_mulMatVec3(target, rot, pos);

	mju_copy(pos, data->site_xpos + 3 * laser_config.site_attached, 3);

	int geomid  = -1;
	mjtNum dist = mj_ray(model, data, pos, target, ignore_groups, 1, -1, &geomid);
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

	mju_addScl3(target, pos, target, dist);
	mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba);
	mjv_connector(geom, mjGEOM_LINE, 1., pos, target);
}

void *ProcessRayThreaded(void *args)
{
	ProcessRayArgs *pargs = static_cast<ProcessRayArgs *>(args);
	ProcessRay(pargs->model, pargs->data, *pargs->rand_generator, *pargs->noise_dist, *pargs->laser_config, pargs->geom,
	           *pargs->rot, *pargs->ignore_groups, *pargs->rgba, *pargs->scan_msg, pargs->ray_idx);
	return nullptr;
}

void LaserPlugin::ComputeLasers(const mjModel *model, mjData *data)
{
	uint n_vGeom        = 0;
	const float rgba[4] = { 0., 0., 1., 0.8 };
	mjtNum rot[9];

	mjtByte ignore_groups[mjNGROUP] = { 0 };
	for (unsigned char &ignore_group : ignore_groups) {
		ignore_group = 1;
	}
	ignore_groups[1] = 0;

	for (const auto &laser_config : laser_configs_) {
		const auto now = Now(lasers_nh_);
		if (last_update_time_initialized_ && SecondsSince(now, last_update_time_) < 1.0 / laser_config.update_rate) {
			skip_ema_ = true;
			continue;
		}

		LaserScan scan_msg;
		scan_msg.header.stamp    = now;
		scan_msg.header.frame_id = laser_config.frame_id;
		scan_msg.angle_min       = laser_config.min_angle;
		scan_msg.angle_max       = laser_config.max_angle;
		scan_msg.angle_increment = laser_config.angular_resolution;
		scan_msg.range_min       = laser_config.min_range;
		scan_msg.range_max       = laser_config.max_range;
		scan_msg.ranges.resize(laser_config.nrays);

		mju_copy(rot, data->site_xmat + 9 * laser_config.site_attached, 9);

		for (uint i = 0; i < laser_config.nrays; ++i) {
			mjvGeom *g = nullptr;
			if (laser_config.visualize) {
				g = laser_geoms_ + n_vGeom;
				n_vGeom++;
			}
			ProcessRay(model, data, rand_generator, noise_dist, laser_config, g, rot, ignore_groups, rgba, scan_msg, i);
		}

		Publish<LaserScan>(laser_config, scan_msg);
		last_update_time_             = now;
		last_update_time_initialized_ = true;
		has_render_data_              = has_render_data_ || laser_config.visualize;
	}
}

void LaserPlugin::ComputeLasersMultithreaded(const mjModel *model, mjData *data)
{
	uint n_vGeom        = 0;
	const float rgba[4] = { 0., 0., 1., 0.8 };
	mjtNum rot[9];

	mjtByte ignore_groups[mjNGROUP] = { 0 };
	for (unsigned char &ignore_group : ignore_groups) {
		ignore_group = 1;
	}
	ignore_groups[1] = 0;

	for (const auto &laser_config : laser_configs_) {
		const auto now = Now(lasers_nh_);
		if (last_update_time_initialized_ && SecondsSince(now, last_update_time_) < 1.0 / laser_config.update_rate) {
			skip_ema_ = true;
			continue;
		}

		LaserScan scan_msg;
		scan_msg.header.stamp    = now;
		scan_msg.header.frame_id = laser_config.frame_id;
		scan_msg.angle_min       = laser_config.min_angle;
		scan_msg.angle_max       = laser_config.max_angle;
		scan_msg.angle_increment = laser_config.angular_resolution;
		scan_msg.range_min       = laser_config.min_range;
		scan_msg.range_max       = laser_config.max_range;
		scan_msg.ranges.resize(laser_config.nrays);

		mj_markStack(data);
		ProcessRayArgs *ray_args = static_cast<ProcessRayArgs *>(
		    mj_stackAllocByte(data, sizeof(ProcessRayArgs) * laser_config.nrays, alignof(ProcessRayArgs)));
		mjTask *tasks =
		    static_cast<mjTask *>(mj_stackAllocByte(data, sizeof(mjTask) * laser_config.nrays, alignof(mjTask)));

		mju_copy(rot, data->site_xmat + 9 * laser_config.site_attached, 9);

		for (uint i = 0; i < laser_config.nrays; ++i) {
			mjvGeom *g = nullptr;
			if (laser_config.visualize) {
				g = laser_geoms_ + n_vGeom;
				n_vGeom++;
			}
			ray_args[i].model          = model;
			ray_args[i].data           = data;
			ray_args[i].rand_generator = &rand_generator;
			ray_args[i].noise_dist     = &noise_dist;
			ray_args[i].laser_config   = &laser_config;
			ray_args[i].geom           = g;
			ray_args[i].rot            = &rot;
			ray_args[i].ignore_groups  = &ignore_groups;
			ray_args[i].rgba           = &rgba;
			ray_args[i].scan_msg       = &scan_msg;
			ray_args[i].ray_idx        = i;

			mju_defaultTask(&tasks[i]);
			tasks[i].func = ProcessRayThreaded;
			tasks[i].args = &ray_args[i];

			mju_threadPoolEnqueue(reinterpret_cast<mjThreadPool *>(data->threadpool), &tasks[i]);
		}

		for (uint i = 0; i < laser_config.nrays; ++i) {
			mju_taskJoin(&tasks[i]);
		}

		mj_freeStack(data);

		Publish<LaserScan>(laser_config, scan_msg);
		last_update_time_             = now;
		last_update_time_initialized_ = true;
		has_render_data_              = has_render_data_ || laser_config.visualize;
	}
}

void LaserPlugin::LastStageCallback(const mjModel *model, mjData *data)
{
	if (data->threadpool) {
		ComputeLasersMultithreaded(model, data);
		return;
	}
	ComputeLasers(model, data);
}

void LaserPlugin::Reset()
{
	last_update_time_initialized_ = false;
}

LaserPlugin::~LaserPlugin()
{
	delete[] laser_geoms_;
}

} // namespace mujoco_ros::sensors::laser

PLUGINLIB_EXPORT_CLASS(mujoco_ros::sensors::laser::LaserPlugin, mujoco_ros::MujocoPlugin)
