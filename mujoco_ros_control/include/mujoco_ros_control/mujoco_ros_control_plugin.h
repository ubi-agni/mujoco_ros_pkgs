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

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <string>
#include <memory> // std::unique_ptr

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/plugin_utils.h>

#include <mujoco_ros_control/robot_hw_sim.h>

#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

namespace mujoco_ros::control {

class MujocoRosControlPlugin : public mujoco_ros::MujocoPlugin
{
public:
	~MujocoRosControlPlugin() override;

	// Overload entry point
	bool load(const mjModel *m, mjData *d) override;

	// Called on reset
	void reset() override;

	// Get URDF from parameter server
	std::string getURDF(const std::string &param_name) const;

	// Get transmissions from URDF
	bool parseTransmissionsFromURDF(const std::string &urdf_string);

	void controlCallback(const mjModel *model, mjData *data) override;

protected:
	void eStopCB(const std_msgs::BoolConstPtr &e_stop_active);

	// Interface loader
	std::unique_ptr<pluginlib::ClassLoader<mujoco_ros::control::RobotHWSim>> robot_hw_sim_loader_;

	std::string robot_description_;
	std::string robot_namespace_;

	// Transmissions in this plugin's scope
	std::vector<transmission_interface::TransmissionInfo> transmissions_;

	std::string robot_hw_sim_type_str_;
	std::unique_ptr<mujoco_ros::control::RobotHWSim> robot_hw_sim_;

	// Controller manager
	std::unique_ptr<controller_manager::ControllerManager> controller_manager_;

	// Timing
	ros::Duration control_period_;
	ros::Time last_update_sim_time_ros_;
	ros::Time last_write_sim_time_ros_;

	bool e_stop_active_, last_e_stop_active_;
	ros::Subscriber e_stop_sub_;

	// Nodehandle in robot namespace
	ros::NodeHandle robot_nh_;
};

} // namespace mujoco_ros::control
