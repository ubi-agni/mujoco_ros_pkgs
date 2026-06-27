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

#pragma once

#include <mujoco_ros/ros_version.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <mujoco_ros/ros_one/plugin_utils.hpp>
#include <sensor_msgs/LaserScan.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <mujoco_ros/ros_two/plugin_utils.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#endif

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros_sensors/mujoco_sensor_handler_plugin.hpp>

#include <memory>
#include <random>
#include <string>
#include <vector>

namespace mujoco_ros::sensors::laser {

#if MJR_ROS_VERSION == ROS_1
using LaserScan = sensor_msgs::LaserScan;
using RosTime   = ros::Time;
#else
using LaserScan = sensor_msgs::msg::LaserScan;
using RosTime   = rclcpp::Time;
#endif

// Defaults
static constexpr bool DEFAULT_VISUALIZE            = false;
static constexpr double DEFAULT_UPDATE_RATE        = 10.;
static constexpr double DEFAULT_MIN_RANGE          = 0.1;
static constexpr double DEFAULT_MAX_RANGE          = 30.;
static constexpr double DEFAULT_RANGE_RESOLUTION   = 0.01;
static constexpr double DEFAULT_ANGULAR_RESOLUTION = 0.02;
static constexpr double DEFAULT_MIN_ANGLE          = -1.57;
static constexpr double DEFAULT_MAX_ANGLE          = 1.57;
static constexpr double DEFAULT_SENSOR_STD         = 0.005;

struct LaserConfig : public mujoco_ros::sensors::SensorConfig
{
public:
#if MJR_ROS_VERSION == ROS_1
	LaserConfig(const XmlRpc::XmlRpcValue &config, const std::string &frame_id, const std::string &name,
	            int site_attached);
#endif
	LaserConfig(std::string frame_id, std::string name, int site_attached, bool visualize, double update_rate,
	            double min_range, double max_range, double range_resolution, double angular_resolution, double min_angle,
	            double max_angle, double sensor_std);

	std::string name;
	int site_attached;
	bool visualize;
	double update_rate;
	mjtNum min_range;
	mjtNum max_range;
	double range_resolution;
	double angular_resolution;
	double min_angle;
	double max_angle;

	uint nrays;
	mjtNum *rays;

	mjtNum cur_xpos[3];
};

class LaserPlugin : public mujoco_ros::MujocoPlugin
{
public:
	~LaserPlugin() override;

	bool Load(const mjModel *m, mjData *d) override;
	void Reset() override;

	void RenderCallback(const mjModel *model, mjData *data, mjvScene *scene) override;
	void LastStageCallback(const mjModel *model, mjData *data) override;

private:
	const mjModel *m_{ nullptr };
	mjData *d_{ nullptr };

	std::vector<LaserConfig> laser_configs_;

#if MJR_ROS_VERSION == ROS_1
	ros::NodeHandle lasers_nh_;
#else
	rclcpp_lifecycle::LifecycleNode::SharedPtr lasers_nh_;
#endif

	RosTime last_update_time_;
	bool last_update_time_initialized_ = false;

#if MJR_ROS_VERSION == ROS_1
	bool InitSensor(const mjModel *model, const XmlRpc::XmlRpcValue &config);
#else
	bool InitSensor(const mjModel *model, const std::string &sensor_name);
#endif

	void ComputeLasers(const mjModel *model, mjData *data);
	void ComputeLasersMultithreaded(const mjModel *model, mjData *data);

	mjvGeom *laser_geoms_{ nullptr };
	int ngeom_ = 0;

	bool has_render_data_ = false;

	std::mt19937 rand_generator = std::mt19937(std::random_device{}());
	std::normal_distribution<double> noise_dist;
};
} // namespace mujoco_ros::sensors::laser
