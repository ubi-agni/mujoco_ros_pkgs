/*********************************************************************
 * Software License Agreement (BSD License)
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

#include <mujoco_ros/mujoco_env.h>

#include <mujoco_ros_msgs/GetStateUint.h>

namespace mujoco_ros {
namespace mju = ::mujoco::sample_util;

void MujocoEnv::setupServices()
{
	service_servers_.push_back(nh_->advertiseService("set_pause", &MujocoEnv::setPauseCB, this));
	service_servers_.push_back(nh_->advertiseService("shutdown", &MujocoEnv::shutdownCB, this));
	service_servers_.push_back(nh_->advertiseService("reload", &MujocoEnv::reloadCB, this));
	service_servers_.push_back(nh_->advertiseService("reset", &MujocoEnv::resetCB, this));

	service_servers_.push_back(nh_->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
	    "load_initial_joint_states", [&](auto /*&req*/, auto /*&res*/) {
		    std::lock_guard<std::recursive_mutex> lock(physics_thread_mutex_);
		    loadInitialJointStates();
		    return true;
	    }));
	service_servers_.push_back(
	    nh_->advertiseService<mujoco_ros_msgs::GetStateUint::Request, mujoco_ros_msgs::GetStateUint::Response>(
	        "get_loading_request_state", [&](auto /*&req*/, auto &res) {
		        uint8_t status  = getOperationalStatus();
		        res.state.value = status;

		        std::string description;
		        if (status == 0)
			        description = "Sim ready";
		        else if (status == 1)
			        description = "Loading in progress";
		        else if (status >= 2)
			        description = "Loading issued";
		        res.state.description = description;
		        return true;
	        }));
}

void MujocoEnv::runControlCbs()
{
	for (const auto &plugin : this->cb_ready_plugins_) {
		plugin->controlCallback(this->model_, this->data_);
	}
}

void MujocoEnv::runPassiveCbs()
{
	for (const auto &plugin : this->cb_ready_plugins_) {
		plugin->passiveCallback(this->model_, this->data_);
	}
}

void MujocoEnv::runRenderCbs(mjModelPtr model, mjDataPtr data, mjvScene *scene)
{
	for (const auto &plugin : this->cb_ready_plugins_) {
		plugin->renderCallback(model, data, scene);
	}
}

void MujocoEnv::runLastStageCbs()
{
	for (const auto &plugin : this->cb_ready_plugins_) {
		plugin->lastStageCallback(this->model_, this->data_);
	}
}

bool MujocoEnv::setPauseCB(mujoco_ros_msgs::SetPause::Request &req, mujoco_ros_msgs::SetPause::Response &res)
{
	ROS_DEBUG_STREAM("Set pause to " << static_cast<bool>(req.paused));

	if (settings_.eval_mode && req.paused) {
		ROS_DEBUG("Evaluation mode is active. Checking request validity");
		if (settings_.admin_hash != req.admin_hash) {
			ROS_ERROR("Unauthorized (un)pause request detected. Ignoring request");
			res.success = false;
			return true;
		}
		ROS_DEBUG("Request valid. Handling request");
	}
	settings_.run.store(!req.paused);
	if (settings_.run.load())
		settings_.env_steps_request.store(0);
	res.success = true;
	return true;
}

bool MujocoEnv::shutdownCB([[maybe_unused]] std_srvs::Empty::Request &req,
                           [[maybe_unused]] std_srvs::Empty::Response &res)
{
	ROS_DEBUG("Shutdown requested");
	settings_.exit_request.store(1);
	return true;
}

bool MujocoEnv::reloadCB(mujoco_ros_msgs::Reload::Request &req, mujoco_ros_msgs::Reload::Response &res)
{
	ROS_DEBUG("Requested reload via ROS service");

	if (req.model.size() > kMaxFilenameLength) {
		ROS_ERROR_STREAM("Model string too long. Max length: "
		                 << kMaxFilenameLength << " (got " << req.model.size()
		                 << "); Consider compiling with a larger value for kMaxFilenameLength");
		res.success        = false;
		res.status_message = "Model string too long (max: " + std::to_string(kMaxFilenameLength) + ")";
		return true;
	}
	mju::strcpy_arr(queued_filename_, req.model.c_str());

	settings_.load_request.store(3);

	while (getOperationalStatus() > 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	res.success        = sim_state_.model_valid;
	res.status_message = load_error_;

	return true;
}

bool MujocoEnv::resetCB([[maybe_unused]] std_srvs::Empty::Request &req, [[maybe_unused]] std_srvs::Empty::Response &res)
{
	ROS_DEBUG("Reset requested");
	settings_.reset_request.store(1);
	return true;
}

} // namespace mujoco_ros
