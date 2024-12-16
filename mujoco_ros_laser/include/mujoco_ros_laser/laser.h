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

#pragma once

#include <mujoco_ros/plugin_utils.h>
#include <mujoco_ros/common_types.h>
#include <mujoco_ros/mujoco_env.h>

#include <mujoco_ros_sensors/mujoco_sensor_handler_plugin.h>

#include <ros/ros.h>

#include <random>

using namespace mujoco_ros;
using namespace mujoco_ros::sensors;

namespace mujoco_ros::sensors::laser {

// Defaults
static bool DEFAULT_VISUALIZE            = false;
static double DEFAULT_UPDATE_RATE        = 10.;
static double DEFAULT_MIN_RANGE          = 0.1;
static double DEFAULT_MAX_RANGE          = 30.;
static double DEFAULT_RANGE_RESOLUTION   = 0.01;
static double DEFAULT_ANGULAR_RESOLUTION = 0.02;
static double DEFAULT_MIN_ANGLE          = -1.57;
static double DEFAULT_MAX_ANGLE          = 1.57;
static double DEFAULT_SENSOR_STD         = 0.005;

struct LaserConfig : public SensorConfig
{
public:
	LaserConfig(const XmlRpc::XmlRpcValue &config, const std::string &frame_id, const std::string &name,
	            int site_attached); // : SensorConfig(std::move(frame_id)), name(std::move(name)),
	                                // site_attached(site_attached);
	LaserConfig(std::string frame_id, std::string name, int site_attached, bool visualize, double update_rate,
	            double min_range, double max_range, double range_resolution, double angular_resolution, double min_angle,
	            double max_angle)
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
	    , max_angle(max_angle){};

	// Sensor name
	std::string name;
	// Site in the model to attach the laser to
	int site_attached;
	// Whether to visualize the laser scan in the simulation
	bool visualize;
	// The update rate of the laser scan in Hz
	double update_rate;
	// The minimum range of the laser scan
	mjtNum min_range;
	// The maximum range of the laser scan
	mjtNum max_range;
	// The range resolution of the laser scan in meters
	double range_resolution;
	// The angular resolution of the laser scan in radians
	double angular_resolution;
	// The minimum and maximum angle of the laser scan in radians
	double min_angle;
	double max_angle;

	uint nrays;
	mjtNum *rays;

	mjtNum cur_xpos[3];
};

class LaserPlugin : public MujocoPlugin
{
public:
	~LaserPlugin() override;

	bool load(const mjModel *m, mjData *d) override;
	void reset() override;

	void renderCallback(const mjModel *model, mjData *data, mjvScene *scene) override;
	void lastStageCallback(const mjModel *model, mjData *data) override;

private:
	// The env_ptr_ (shared_ptr) in the parent class ensures mjModel and mjData are not destroyed
	const mjModel *m_;
	mjData *d_;

	// Laser sensor configurations
	std::vector<LaserConfig> laser_configs_;
	// Handle for publishers of laser scan messages
	ros::NodeHandle lasers_nh_;

	// Last sim time the laser was computed
	ros::Time last_update_time_;

	// Initialize the laser sensor configuration
	bool initSensor(const mjModel *model, const XmlRpc::XmlRpcValue &config);

	// Laser computation
	void computeLasers(const mjModel *model, mjData *data);
	// Multi-threaded computation of laser rays
	void computeLasersMultithreaded(const mjModel *model, mjData *data);

	// Laser visualization geoms
	mjvGeom *laser_geoms_;
	// Number of laser geoms
	int ngeom_ = 0;

	// Whether at least one sensor is to be rendered
	bool has_render_data_ = false;

	// Random number generator for sensor noise
	std::mt19937 rand_generator = std::mt19937(std::random_device{}());
	// Normal distribution for sensor noise
	std::normal_distribution<double> noise_dist;
};
} // namespace mujoco_ros::sensors::laser
