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
	SensorConfig(const std::string frame_id) : frame_id(frame_id){};

	void setFrameId(const std::string frame_id) { this->frame_id = frame_id; };

	void registerPub(ros::Publisher pub) { value_pub = pub; };

	void registerGTPub(ros::Publisher pub) { gt_pub = pub; };

	std::string frame_id;

	ros::Publisher gt_pub;
	ros::Publisher value_pub;

	// Noise params
	double mean[3];
	double sigma[3];

	uint8_t is_set = 0; // 0 for unset, otherwise binary code for combination of dims
};

typedef std::shared_ptr<SensorConfig> SensorConfigPtr;

struct WrenchSensorConfig
{
public:
	WrenchSensorConfig() : force_config(), torque_config(){};
	WrenchSensorConfig(const int force_sensor_id, const SensorConfigPtr force_config, const int torque_sensor_id,
	                   const SensorConfigPtr torque_config)
	    : force_sensor_id(force_sensor_id)
	    , force_config(force_config)
	    , torque_sensor_id(torque_sensor_id)
	    , torque_config(torque_config){};

	void setFrameId(const std::string frame_id)
	{
		this->force_config->setFrameId(frame_id);
		this->torque_config->setFrameId(frame_id);
	};

	void registerPub(ros::Publisher pub) { value_pub = pub; };

	void registerGTPub(ros::Publisher pub) { gt_pub = pub; };

	ros::Publisher gt_pub;
	ros::Publisher value_pub;

	SensorConfigPtr force_config;
	SensorConfigPtr torque_config;

	int force_sensor_id;
	int torque_sensor_id;

	uint8_t is_set = 0; // 0 for unset, otherwise binary code for combination of dims
};

typedef std::shared_ptr<WrenchSensorConfig> WrenchSensorConfigPtr;

class MujocoRosSensorsPlugin : public mujoco_ros::MujocoPlugin
{
public:
	virtual ~MujocoRosSensorsPlugin();

	// Overload entry point
	virtual bool load(mujoco_ros::mjModelPtr m, mujoco_ros::mjDataPtr d);

	virtual void reset();

	void lastStageCallback(mujoco_ros::mjModelPtr model, mujoco_ros::mjDataPtr data);

private:
	ros::NodeHandlePtr sensors_nh_;
	void initSensors(mujoco_ros::mjModelPtr model, mujoco_ros::mjDataPtr data);
	std::mt19937 rand_generator = std::mt19937(std::random_device{}());
	std::normal_distribution<double> noise_dist;

	std::map<std::string, SensorConfigPtr> sensor_map_;

	std::vector<std::string> wrench_sensor_names_;
	std::map<std::string, WrenchSensorConfigPtr> wrench_sensor_map_;

	ros::ServiceServer register_noise_model_server_;

	bool registerNoiseModelsCB(mujoco_ros_msgs::RegisterSensorNoiseModels::Request &req,
	                           mujoco_ros_msgs::RegisterSensorNoiseModels::Response &rep);
};

const char *const SENSOR_STRING[] = { [mjSENS_TOUCH]          = "touch",
	                                   [mjSENS_ACCELEROMETER]  = "accelerometer",
	                                   [mjSENS_VELOCIMETER]    = "velocimeter",
	                                   [mjSENS_GYRO]           = "gyro",
	                                   [mjSENS_FORCE]          = "force",
	                                   [mjSENS_TORQUE]         = "torque",
	                                   [mjSENS_MAGNETOMETER]   = "magnetometer",
	                                   [mjSENS_RANGEFINDER]    = "rangefinder",
	                                   [mjSENS_JOINTPOS]       = "jointpos",
	                                   [mjSENS_JOINTVEL]       = "jointvel",
	                                   [mjSENS_TENDONPOS]      = "tendonpos",
	                                   [mjSENS_TENDONVEL]      = "tendonvel",
	                                   [mjSENS_ACTUATORPOS]    = "actuatorpos",
	                                   [mjSENS_ACTUATORVEL]    = "actuatorvel",
	                                   [mjSENS_ACTUATORFRC]    = "actuatorfrc",
	                                   [mjSENS_BALLQUAT]       = "ballquat",
	                                   [mjSENS_BALLANGVEL]     = "ballangvel",
	                                   [mjSENS_JOINTLIMITPOS]  = "jointlimitpos",
	                                   [mjSENS_JOINTLIMITVEL]  = "jointlimitvel",
	                                   [mjSENS_JOINTLIMITFRC]  = "jointlimitfrc",
	                                   [mjSENS_TENDONLIMITPOS] = "tendonlimitpos",
	                                   [mjSENS_TENDONLIMITVEL] = "tendonlimitvel",
	                                   [mjSENS_TENDONLIMITFRC] = "tendonlimitfrc",
	                                   [mjSENS_FRAMEPOS]       = "framepos",
	                                   [mjSENS_FRAMEQUAT]      = "framequat",
	                                   [mjSENS_FRAMEXAXIS]     = "framexaxis",
	                                   [mjSENS_FRAMEYAXIS]     = "frameyaxis",
	                                   [mjSENS_FRAMEZAXIS]     = "framezaxis",
	                                   [mjSENS_FRAMELINVEL]    = "framelinvel",
	                                   [mjSENS_FRAMEANGVEL]    = "frameangvel",
	                                   [mjSENS_FRAMELINACC]    = "framelinacc",
	                                   [mjSENS_FRAMEANGACC]    = "frameangacc",
	                                   [mjSENS_SUBTREECOM]     = "subtreecom",
	                                   [mjSENS_SUBTREELINVEL]  = "subtreelinvel",
	                                   [mjSENS_SUBTREEANGMOM]  = "subtreeangmom" };

} // namespace mujoco_ros::sensors
