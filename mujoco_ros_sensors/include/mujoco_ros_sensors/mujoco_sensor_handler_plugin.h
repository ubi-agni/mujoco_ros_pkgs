/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2023, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
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
 */

/* Authors: David P. Leins */

#pragma once

#include <ros/ros.h>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/plugin_utils.h>
#include <mujoco_ros/mujoco_env.h>

#include <mujoco_ros_msgs/RegisterSensorNoiseModels.h>

#include <random>

namespace mujoco_ros::sensors {

struct SensorConfig
{
public:
	SensorConfig() : frame_id(""){};
	SensorConfig(std::string frame_id) : frame_id(std::move(frame_id)){};

	void setFrameId(const std::string &frame_id) { this->frame_id = frame_id; };

	void registerPub(const ros::Publisher &pub) { value_pub = pub; };

	void registerGTPub(const ros::Publisher &pub) { gt_pub = pub; };

	std::string frame_id;

	ros::Publisher gt_pub;
	ros::Publisher value_pub;

	// Noise params
	double mean[3];
	double sigma[3];

	uint8_t is_set = 0; // 0 for unset, otherwise binary code for combination of dims
};

using SensorConfigPtr = std::unique_ptr<SensorConfig>;

class MujocoRosSensorsPlugin : public mujoco_ros::MujocoPlugin
{
public:
	~MujocoRosSensorsPlugin() override;

	// Overload entry point
	bool load(const mjModel *m, mjData *d) override;

	void reset() override;

	void lastStageCallback(const mjModel *model, mjData *data) override;

private:
	ros::NodeHandle sensors_nh_;
	void initSensors(const mjModel *model, mjData *data);
	std::mt19937 rand_generator = std::mt19937(std::random_device{}());
	std::normal_distribution<double> noise_dist;

	std::map<std::string, SensorConfigPtr> sensor_map_;

	ros::ServiceServer register_noise_model_server_;

	bool registerNoiseModelsCB(mujoco_ros_msgs::RegisterSensorNoiseModels::Request &req,
	                           mujoco_ros_msgs::RegisterSensorNoiseModels::Response &rep);
};

const char *SENSOR_STRING[36];

} // namespace mujoco_ros::sensors
