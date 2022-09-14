/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022, Bielefeld University
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

#include <tf2_ros/transform_listener.h>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/plugin_utils.h>

namespace mujoco_ros_sensors {

class MujocoRosSensorsPlugin : public MujocoSim::MujocoPlugin
{
public:
	virtual ~MujocoRosSensorsPlugin();

	// Overload entry point
	virtual bool load(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d);

	virtual void reset();

	void lastStageCallback(MujocoSim::mjModelPtr model, MujocoSim::mjDataPtr data);

private:
	void initSensors(MujocoSim::mjModelPtr model, MujocoSim::mjDataPtr data);

	std::map<std::string, std::pair<ros::Publisher, std::string>> sensor_map_;
	bool got_transform_ = false;

	// deferred load in case ros is blocking
	boost::thread deferred_load_thread_;
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

} // namespace mujoco_ros_sensors
