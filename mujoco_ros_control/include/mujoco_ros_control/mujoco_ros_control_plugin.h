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

/* Authors: David P. Leins*/

#pragma once

#include <mujoco_ros_control/mujoco_ros_control_plugin.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <string>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/mujoco_sim.h>
#include <mujoco_ros/plugin_utils.h>

#include <mujoco_ros_control/robot_hw_sim.h>

#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

namespace mujoco_ros_control {

class MujocoRosControlPlugin : public MujocoSim::MujocoPlugin
{
public:
	virtual ~MujocoRosControlPlugin();

	// Overlead entry point
	virtual bool load(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d);

	// Called on reset
	virtual void reset();

	// Get URDF from parameter server
	std::string getURDF(std::string param_name) const;

	// Get transmissions from URDF
	bool parseTransmissionsFromURDF(const std::string &urdf_string);

	void controlCallback(MujocoSim::mjModelPtr model, MujocoSim::mjDataPtr data);

protected:
	void eStopCB(const std_msgs::BoolConstPtr &e_stop_active);

	// Node Handles
	ros::NodeHandle model_nh_;

	// deferred load in case ros is blocking
	boost::thread deffered_load_thread_;

	// Interface loader
	boost::shared_ptr<pluginlib::ClassLoader<mujoco_ros_control::RobotHWSim>> robot_hw_sim_loader_;

	std::string robot_namespace_;
	std::string robot_description_;

	// Transmissions in this plugin's scope
	std::vector<transmission_interface::TransmissionInfo> transmissions_;

	std::string robot_hw_sim_type_str_;
	boost::shared_ptr<mujoco_ros_control::RobotHWSim> robot_hw_sim_;

	// Mujoco model and data pointers
	MujocoSim::mjModelPtr m_;
	MujocoSim::mjDataPtr d_;

	// Controller manager
	boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

	// Timing
	ros::Duration control_period_;
	ros::Time last_update_sim_time_ros_;
	ros::Time last_write_sim_time_ros_;

	bool e_stop_active_, last_e_stop_active_;
	ros::Subscriber e_stop_sub_;
};

} // namespace mujoco_ros_control
