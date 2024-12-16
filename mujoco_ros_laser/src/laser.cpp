/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022-2024, Bielefeld University
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

#include <pluginlib/class_list_macros.h>

#include <mujoco_ros_laser/laser.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/LaserScan.h>

using namespace mujoco_ros;

namespace mujoco_ros::sensors::laser {

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
	sensor_msgs::LaserScan *scan_msg;
	int ray_idx;
};

void readOptionalDoubleFromConfig(const XmlRpc::XmlRpcValue &config, const std::string &name, double &target,
                                  double default_value)
{
	if (config.hasMember(name) and config[name].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
		target = static_cast<double>(config[name]);
		return;
	}

	target = default_value;
}

LaserConfig::LaserConfig(const XmlRpc::XmlRpcValue &config, const std::string &frame_id, const std::string &name,
                         int site_attached)
    : SensorConfig(frame_id), name(name), site_attached(site_attached)
{
	if (config.hasMember("visualize")) {
		this->visualize = static_cast<bool>(config["visualize"]);
	} else {
		this->visualize = DEFAULT_VISUALIZE;
	}

	readOptionalDoubleFromConfig(config, "update_rate", this->update_rate, DEFAULT_UPDATE_RATE);
	readOptionalDoubleFromConfig(config, "min_range", this->min_range, DEFAULT_MIN_RANGE);
	readOptionalDoubleFromConfig(config, "max_range", this->max_range, DEFAULT_MAX_RANGE);
	readOptionalDoubleFromConfig(config, "range_resolution", this->range_resolution, DEFAULT_RANGE_RESOLUTION);
	readOptionalDoubleFromConfig(config, "angular_resolution", this->angular_resolution, DEFAULT_ANGULAR_RESOLUTION);
	readOptionalDoubleFromConfig(config, "min_angle", this->min_angle, DEFAULT_MIN_ANGLE);
	readOptionalDoubleFromConfig(config, "max_angle", this->max_angle, DEFAULT_MAX_ANGLE);
	readOptionalDoubleFromConfig(config, "sensor_std", this->sigma[0], DEFAULT_SENSOR_STD);

	if (this->sigma[0] > 0) {
		this->is_set = 1;
	}

	this->nrays = (max_angle - min_angle) / angular_resolution;
	this->rays  = new mjtNum[this->nrays * 3];
	for (uint i = 0; i < this->nrays; ++i) {
		mjtNum ray[3] = { mju_cos(min_angle + i * angular_resolution), mju_sin(min_angle + i * angular_resolution), 0 };
		mju_copy3(this->rays + i * 3, ray);
	}
}

bool LaserPlugin::load(const mjModel *m, mjData *d)
{
	std::string lasers_namespace;
	if (rosparam_config_.hasMember("namespace")) {
		lasers_namespace = static_cast<std::string>(rosparam_config_["namespace"]);
	}
	lasers_nh_ = ros::NodeHandle(lasers_namespace);

	// init noise distribution
	noise_dist = std::normal_distribution<double>(0.0, 1.0);

	if (!rosparam_config_.hasMember("sensors")) {
		ROS_ERROR_NAMED("lasers", "Laser plugin needs to configure at least one sensor!");
		return false;
	}

	if (rosparam_config_["sensors"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
		// iterate through configs
		// NOLINTNEXTLINE(modernize-loop-convert) range-based for loop throws XmlRpcValue Exception
		for (uint i = 0; i < rosparam_config_["sensors"].size(); i++) {
			initSensor(m, rosparam_config_["sensors"][i]);
		}
	} else {
		ROS_ERROR_NAMED("lasers", "Sensors config is not an array!");
		return false;
	}

	ngeom_ = 0;
	for (const auto &laser_config : laser_configs_) {
		// register laser geoms
		if (laser_config.visualize) {
			ngeom_ += laser_config.nrays;
		}
	}

	laser_geoms_ = new mjvGeom[ngeom_];

	m_ = m;
	d_ = d;
	return true;
}

bool LaserPlugin::initSensor(const mjModel *model, const XmlRpc::XmlRpcValue &config)
{
	if (!config.hasMember("site_attached")) {
		ROS_ERROR_NAMED("lasers",
		                "Laser plugin needs to configure where to attach the laser with the `site_attached` parameter!");
		return false;
	}

	std::string site_attached = static_cast<std::string>(config["site_attached"]);
	int site_id               = mj_name2id(const_cast<mjModel *>(model), mjOBJ_SITE, site_attached.c_str());

	if (site_id == -1) {
		ROS_ERROR_STREAM_NAMED("lasers", "Site with name '" << site_attached << "' could not be found in the model!");
		return false;
	}

	std::string frame_id, name;
	int parent_id = model->site_bodyid[site_id];
	frame_id      = mj_id2name(const_cast<mjModel *>(model), mjOBJ_BODY, parent_id);

	if (config.hasMember("name")) {
		name = static_cast<std::string>(config["name"]);
	} else {
		name = site_attached + std::string("_laser");
	}

	LaserConfig laser_config = LaserConfig(config, frame_id, name, site_id);
	laser_config.registerPub(lasers_nh_.advertise<sensor_msgs::LaserScan>(name, 1));
	this->laser_configs_.push_back(laser_config);

	return true;
}

void LaserPlugin::renderCallback(const mjModel * /*model*/, mjData * /*data*/, mjvScene *scene)
{
	// add visual geoms to render to the scene

	if (not has_render_data_) {
		return;
	}

	for (uint i = 0; i < std::min(ngeom_, scene->maxgeom - scene->ngeom); ++i) {
		scene->geoms[scene->ngeom++] = laser_geoms_[i];
	}
}

void processRay(const mjModel *model, mjData *data, std::mt19937 rand_generator,
                std::normal_distribution<double> &noise_dist, const LaserConfig &laser_config, mjvGeom *geom,
                mjtNum (&rot)[9], const mjtByte (&ignore_groups)[mjNGROUP], const float (&rgba)[4],
                sensor_msgs::LaserScan &scan_msg, int ray_idx)
{
	mjtNum pos[3], target[3];
	mju_copy(pos, laser_config.rays + 3 * ray_idx, 3);
	mju_mulMatVec3(target, rot, pos);

	// start position
	mju_copy(pos, data->site_xpos + 3 * laser_config.site_attached, 3);

	// run raycast with clamping
	int geomid  = -1;
	mjtNum dist = mj_ray(model, data, pos, target, ignore_groups, 1, -1, &geomid);
	if (dist < 0) {
		dist = laser_config.max_range;
	}

	// add noise to distance:
	// noise is std per meter, so multiply by distance
	if (laser_config.is_set) {
		dist += (laser_config.sigma[0] * noise_dist(rand_generator)) * dist;
	}

	dist                     = std::min(std::max(laser_config.min_range, dist), laser_config.max_range);
	scan_msg.ranges[ray_idx] = dist;

	if (geom == nullptr) {
		return;
	}
	// add ray to start position
	mju_addScl3(target, pos, target, dist);
	mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, rgba);
	mjv_connector(geom, mjGEOM_LINE, 1., pos, target);
}

void *processRayThreaded(void *args)
{
	ProcessRayArgs *pargs = static_cast<ProcessRayArgs *>(args);
	processRay(pargs->model, pargs->data, *pargs->rand_generator, *pargs->noise_dist, *pargs->laser_config, pargs->geom,
	           *pargs->rot, *pargs->ignore_groups, *pargs->rgba, *pargs->scan_msg, pargs->ray_idx);
	return nullptr;
}

void LaserPlugin::computeLasers(const mjModel *model, mjData *data)
{
	// run last stage code here
	uint n_vGeom        = 0;
	const float rgba[4] = { 0., 0., 1., 0.8 };
	mjtNum rot[9];

	mjtByte ignore_groups[mjNGROUP] = { 0 };
	for (unsigned char &ignore_group : ignore_groups) {
		ignore_group = 1;
	}
	ignore_groups[1] = 0; // ignore group 1 (robot geom)

	for (const auto &laser_config : laser_configs_) {
		// check if laser should be computed
		if (ros::Time::now() - last_update_time_ < ros::Duration(1.0 / laser_config.update_rate)) {
			continue;
		}
		sensor_msgs::LaserScan scan_msg;
		scan_msg.header.stamp    = ros::Time::now();
		scan_msg.header.frame_id = laser_config.frame_id;
		scan_msg.angle_min       = laser_config.min_angle;
		scan_msg.angle_max       = laser_config.max_angle;
		scan_msg.angle_increment = laser_config.angular_resolution;
		scan_msg.range_min       = laser_config.min_range;
		scan_msg.range_max       = laser_config.max_range;
		scan_msg.ranges.resize(laser_config.nrays);

		// run laser scan
		mju_copy(rot, data->site_xmat + 9 * laser_config.site_attached, 9);

		for (uint i = 0; i < laser_config.nrays; ++i) {
			mjvGeom *g = nullptr;
			if (laser_config.visualize) {
				g = laser_geoms_ + n_vGeom;
				n_vGeom++;
			}
			processRay(model, data, rand_generator, noise_dist, laser_config, g, rot, ignore_groups, rgba, scan_msg, i);
		}

		// publish laser scan
		// laser_config.gt_pub.publish(scan_msg);
		laser_config.value_pub.publish(scan_msg);
		has_render_data_ = has_render_data_ || laser_config.visualize;
	}
}

void LaserPlugin::computeLasersMultithreaded(const mjModel *model, mjData *data)
{
	// run last stage code here
	uint n_vGeom        = 0;
	const float rgba[4] = { 0., 0., 1., 0.8 };
	mjtNum rot[9];

	mjtByte ignore_groups[mjNGROUP] = { 0 };
	for (unsigned char &ignore_group : ignore_groups) {
		ignore_group = 1;
	}
	ignore_groups[1] = 0; // ignore group 1 (robot geom)

	for (const auto &laser_config : laser_configs_) {
		// check if laser should be computed
		if (ros::Time::now() - last_update_time_ < ros::Duration(1.0 / laser_config.update_rate)) {
			skip_ema_ = true;
			continue;
		}

		sensor_msgs::LaserScan scan_msg;
		scan_msg.header.stamp    = ros::Time::now();
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
		// ProcessRayArgs ray_args[laser_config.nrays];
		// mjTask tasks[laser_config.nrays];

		// run laser scan
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
			tasks[i].func = processRayThreaded;
			tasks[i].args = &ray_args[i];

			// NOLINTNEXTLINE(performance-no-int-to-ptr)
			mju_threadPoolEnqueue(reinterpret_cast<mjThreadPool *>(data->threadpool), &tasks[i]);
		}

		for (uint i = 0; i < laser_config.nrays; ++i) {
			mju_taskJoin(&tasks[i]);
		}

		mj_freeStack(data);

		// publish laser scan
		// laser_config.gt_pub.publish(scan_msg);
		laser_config.value_pub.publish(scan_msg);
		has_render_data_ = has_render_data_ || laser_config.visualize;
	}
}

void LaserPlugin::lastStageCallback(const mjModel *model, mjData *data)
{
	if (!data->threadpool) {
		computeLasers(model, data);
		return;
	} else {
		computeLasersMultithreaded(model, data);
	}
}

// Needs to be defined
void LaserPlugin::reset() {}

LaserPlugin::~LaserPlugin()
{
	delete[] laser_geoms_;
}

} // namespace mujoco_ros::sensors::laser

PLUGINLIB_EXPORT_CLASS(mujoco_ros::sensors::laser::LaserPlugin, mujoco_ros::MujocoPlugin)
