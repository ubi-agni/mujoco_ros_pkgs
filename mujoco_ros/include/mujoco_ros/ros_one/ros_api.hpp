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

#include <ros/ros.h>

#include <mujoco_ros/common_types.hpp>

#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include <actionlib/server/simple_action_server.h>

#include <mujoco_ros_msgs/StepAction.h>
#include <mujoco_ros_msgs/StepGoal.h>

#include <mujoco_ros_msgs/SetPause.h>
#include <mujoco_ros_msgs/Reload.h>
#include <mujoco_ros_msgs/SetBodyState.h>
#include <mujoco_ros_msgs/GetBodyState.h>
#include <mujoco_ros_msgs/SetGeomProperties.h>
#include <mujoco_ros_msgs/GetGeomProperties.h>
#include <mujoco_ros_msgs/EqualityConstraintParameters.h>
#include <mujoco_ros_msgs/GetEqualityConstraintParameters.h>
#include <mujoco_ros_msgs/SetEqualityConstraintParameters.h>
#include <mujoco_ros_msgs/SetGravity.h>
#include <mujoco_ros_msgs/GetGravity.h>
#include <mujoco_ros_msgs/GetStateUint.h>
#include <mujoco_ros_msgs/GetSimInfo.h>
#include <mujoco_ros_msgs/SetFloat.h>
#include <mujoco_ros_msgs/PluginStats.h>
#include <mujoco_ros_msgs/GetPluginStats.h>

namespace mujoco_ros {

class RosAPI
{
public:
	RosAPI(std::shared_ptr<ros::NodeHandle> &nh, MujocoEnvPtr env_ptr);

	void SetupServices();
	void SetupClockPublisher();
	void PublishSimTime(mjtNum sim_time);

private:
	std::shared_ptr<ros::NodeHandle> nh_;
	MujocoEnvPtr env_ptr_;
	ros::Publisher clock_pub_;

	bool SetPauseCB(mujoco_ros_msgs::SetPause::Request &req, mujoco_ros_msgs::SetPause::Response &res);
	bool ShutdownCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool ReloadCB(mujoco_ros_msgs::Reload::Request &req, mujoco_ros_msgs::Reload::Response &res);
	bool ResetCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool SetBodyStateCB(mujoco_ros_msgs::SetBodyState::Request &req, mujoco_ros_msgs::SetBodyState::Response &res);
	bool GetBodyStateCB(mujoco_ros_msgs::GetBodyState::Request &req, mujoco_ros_msgs::GetBodyState::Response &res);
	bool SetGeomPropertiesCB(mujoco_ros_msgs::SetGeomProperties::Request &req,
	                         mujoco_ros_msgs::SetGeomProperties::Response &res);
	bool GetGeomPropertiesCB(mujoco_ros_msgs::GetGeomProperties::Request &req,
	                         mujoco_ros_msgs::GetGeomProperties::Response &res);
	bool SetEqualityConstraintParameters(const mujoco_ros_msgs::EqualityConstraintParameters &parameters,
	                                     const std::string &admin_hash, char *error_msg = nullptr,
	                                     const int error_sz = 0);
	bool GetEqualityConstraintParameters(mujoco_ros_msgs::EqualityConstraintParameters &parameters,
	                                     const std::string &admin_hash, char *error_msg = nullptr,
	                                     const int error_sz = 0);
	bool SetEqualityConstraintParametersArrayCB(mujoco_ros_msgs::SetEqualityConstraintParameters::Request &req,
	                                            mujoco_ros_msgs::SetEqualityConstraintParameters::Response &res);
	bool GetEqualityConstraintParametersArrayCB(mujoco_ros_msgs::GetEqualityConstraintParameters::Request &req,
	                                            mujoco_ros_msgs::GetEqualityConstraintParameters::Response &res);
	bool GetStateUintCB(mujoco_ros_msgs::GetStateUint::Request &req, mujoco_ros_msgs::GetStateUint::Response &res);
	bool GetSimInfoCB(mujoco_ros_msgs::GetSimInfo::Request &req, mujoco_ros_msgs::GetSimInfo::Response &res);
	bool SetRTFactorCB(mujoco_ros_msgs::SetFloat::Request &req, mujoco_ros_msgs::SetFloat::Response &res);
	bool GetPluginStatsCB(mujoco_ros_msgs::GetPluginStats::Request &req, mujoco_ros_msgs::GetPluginStats::Response &res);
	bool SetGravityCB(mujoco_ros_msgs::SetGravity::Request &req, mujoco_ros_msgs::SetGravity::Response &res);
	bool GetGravityCB(mujoco_ros_msgs::GetGravity::Request &req, mujoco_ros_msgs::GetGravity::Response &res);
	bool LoadInitialJointStatesCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	void OnStepGoal(const mujoco_ros_msgs::StepGoalConstPtr &goal);

	// std::recursive_mutex sim_params_mutex_;
	// std::unique_ptr<dynamic_reconfigure::Server<mujoco_ros::SimParamsConfig>> param_server_;
	// mujoco_ros::SimParamsConfig sim_params_;
	// void dynparamCallback(mujoco_ros::SimParamsConfig &config, uint32_t level);
	// void updateDynamicParams();

	std::vector<ros::ServiceServer> service_servers_;
	std::unique_ptr<actionlib::SimpleActionServer<mujoco_ros_msgs::StepAction>> action_step_;
};

} // namespace mujoco_ros
