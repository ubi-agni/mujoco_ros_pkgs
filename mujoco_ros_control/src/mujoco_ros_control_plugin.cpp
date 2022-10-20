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

#include <mujoco_ros_control/mujoco_ros_control_plugin.h>

#include <pluginlib/class_list_macros.h>

#include <boost/bind.hpp>
#include <urdf/model.h>
#include <chrono>
#include <thread>

namespace mujoco_ros_control {

MujocoRosControlPlugin::~MujocoRosControlPlugin() {}

bool MujocoRosControlPlugin::load(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d)
{
	ROS_INFO_STREAM_NAMED("mujoco_ros_control", "Loading mujoco_ros_control plugin ...");

	// Check that ROS has been initialized
	if (!ros::isInitialized()) {
		ROS_FATAL_STREAM_NAMED("mujoco_ros_control",
		                       "A ROS node for Mujoco has not been initialized, unable to load plugin.");
		return false;
	}

	robot_namespace_ = node_handle_->getNamespace();

	ROS_ASSERT(rosparam_config_.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	// Check rosparam sanity
	if (!rosparam_config_.hasMember("hardware")) {
		ROS_ERROR_NAMED("mujoco_ros_control", "MujocoRosControlPlugin expects a 'hardware' rosparam specifying at least "
		                                      "the 'type' of hardware interface and a 'control_period'");
		return false;
	}
	if (rosparam_config_["hardware"].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
		if (rosparam_config_["hardware"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
			ROS_ERROR_NAMED("mujoco_ros_control",
			                "The 'hardware' param for MujocoRosControlPlugin must define a struct, but got an array");
		} else {
			ROS_ERROR_NAMED("mujoco_ros_control", "MujocoRosControlPlugin expects a 'hardware' rosparam specifying at "
			                                      "least the 'type' of hardware interface and a 'control_period'");
		}
		return false;
	}
	if (!rosparam_config_["hardware"].hasMember("type")) {
		ROS_ERROR_NAMED("mujoco_ros_control",
		                "MujocoRosControlPlugin can not find which hardware simulation to load on the parameter server.");
		return false;
	}
	if (!rosparam_config_["hardware"].hasMember("control_period")) {
		ROS_ERROR_NAMED("mujoco_ros_control",
		                "MujocoRosControlPlugin can not find a control_period to set on the parameter server.");
		return false;
	}

	robot_hw_sim_type_str_ = (std::string)rosparam_config_["hardware"]["type"];
	control_period_        = ros::Duration((double)rosparam_config_["hardware"]["control_period"]);

	if (control_period_.toSec() < m->opt.timestep) {
		ROS_WARN_STREAM_NAMED("mujoco_ros_control", "Desired controller update period ("
		                                                << control_period_
		                                                << " s) is faster than the mujoco simulation timestep ("
		                                                << m->opt.timestep << " s).");
	}

	robot_description_ = rosparam_config_["hardware"].hasMember("robot_description") ?
	                         (std::string)rosparam_config_["hardware"]["robot_description"] :
	                         "robot_description";

	e_stop_active_      = false;
	last_e_stop_active_ = false;

	if (rosparam_config_["hardware"].hasMember("eStopTopic")) {
		const std::string e_stop_topic = (std::string)rosparam_config_["hardware"]["eStopTopic"];
		e_stop_sub_                    = node_handle_->subscribe(e_stop_topic, 1, &MujocoRosControlPlugin::eStopCB, this);
	}

	std::string urdf_string = getURDF(robot_description_);
	if (!parseTransmissionsFromURDF(urdf_string)) {
		ROS_ERROR_NAMED("mujoco_ros_control",
		                "Error parsing URDF for transmissions in mujoco_ros_control plugin, plugin not active.");
		return false;
	}

	try {
		robot_hw_sim_loader_.reset(new pluginlib::ClassLoader<mujoco_ros_control::RobotHWSim>(
		    "mujoco_ros_control", "mujoco_ros_control::RobotHWSim"));

		robot_hw_sim_ = robot_hw_sim_loader_->createInstance(robot_hw_sim_type_str_);
		urdf::Model urdf_model;
		const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

		ROS_DEBUG_STREAM_NAMED("mujoco_ros_control",
		                       "Trying to initialize robot hw sim of type '" << robot_hw_sim_type_str_ << "'");
		if (!robot_hw_sim_->initSim(m, d, robot_namespace_, *node_handle_, urdf_model_ptr, transmissions_)) {
			ROS_FATAL_NAMED("mujoco_ros_control", "Could not initialize robot simulation interface");
			return false;
		}

		ROS_DEBUG_STREAM_NAMED("mujoco_ros_control", "Loading controller manager");
		controller_manager_.reset(new controller_manager::ControllerManager(robot_hw_sim_.get(), *node_handle_));
	} catch (pluginlib::LibraryLoadException &ex) {
		ROS_FATAL_STREAM_NAMED("mujoco_ros_control", "Failed to create robot simulation interface loader: " << ex.what());
		return false;
	}

	ROS_INFO("Loaded mujoco_ros_control");
	return true;
}

void MujocoRosControlPlugin::controlCallback(MujocoSim::mjModelPtr /*model*/, MujocoSim::mjDataPtr /*data*/)
{
	ros::Time sim_time_ros   = ros::Time::now();
	ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;
	bool reset_ctrls         = last_update_sim_time_ros_.isZero();

	robot_hw_sim_->eStopActive(e_stop_active_);

	if (sim_period >= control_period_ || (reset_ctrls && !sim_period.isZero())) {
		last_update_sim_time_ros_ = sim_time_ros;
		robot_hw_sim_->readSim(sim_time_ros, sim_period);

		if (e_stop_active_) {
			last_e_stop_active_ = true;
		} else if (last_e_stop_active_) {
			reset_ctrls         = true;
			last_e_stop_active_ = false;
		}

		controller_manager_->update(sim_time_ros, sim_period, reset_ctrls);
	}

	if (last_update_sim_time_ros_.toNSec() > 0.0 && (sim_time_ros - last_write_sim_time_ros_).toNSec() > 0.0)
		robot_hw_sim_->writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);

	last_write_sim_time_ros_ = sim_time_ros;
}

void MujocoRosControlPlugin::reset()
{
	ROS_INFO("Resetting mujoco_ros_control");
	last_update_sim_time_ros_ = ros::Time();
	last_write_sim_time_ros_  = ros::Time();
}

std::string MujocoRosControlPlugin::getURDF(std::string param_name) const
{
	std::string urdf_string;

	// search and wait for robot_description on param server
	while (urdf_string.empty()) {
		std::string search_param_name;
		if (node_handle_->searchParam(param_name, search_param_name)) {
			ROS_INFO_ONCE_NAMED("mujoco_ros_control",
			                    "mujoco_ros_control plugin is waiting for model"
			                    " URDF in parameter [%s] on the ROS param server.",
			                    search_param_name.c_str());

			node_handle_->getParam(search_param_name, urdf_string);
		} else {
			ROS_INFO_ONCE_NAMED("mujoco_ros_control",
			                    "mujoco_ros_control plugin is waiting for model"
			                    " URDF in parameter [%s] on the ROS param server.",
			                    robot_description_.c_str());

			node_handle_->getParam(param_name, urdf_string);
		}

		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}
	ROS_DEBUG_STREAM_NAMED("mujoco_ros_control", "Recieved urdf from param server, parsing...");

	return urdf_string;
}

bool MujocoRosControlPlugin::parseTransmissionsFromURDF(const std::string &urdf_string)
{
	transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
	return true;
}

void MujocoRosControlPlugin::eStopCB(const std_msgs::BoolConstPtr &e_stop_active)
{
	e_stop_active_ = e_stop_active->data;
}
} // namespace mujoco_ros_control

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::MujocoRosControlPlugin, MujocoSim::MujocoPlugin)
