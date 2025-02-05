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

#include <mujoco_ros/ros_one/ros_api.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <mujoco_ros/array_safety.h>
#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/util.hpp>

namespace mujoco_ros {
namespace mju = ::mujoco::sample_util;

RosAPI::RosAPI(std::shared_ptr<ros::NodeHandle> &nh, MujocoEnvPtr env_ptr) : nh_(nh), env_ptr_(env_ptr) {}

void RosAPI::SetupServices()
{
	service_servers_.emplace_back(nh_->advertiseService("set_pause", &RosAPI::SetPauseCB, this));
	service_servers_.emplace_back(nh_->advertiseService("shutdown", &RosAPI::ShutdownCB, this));
	service_servers_.emplace_back(nh_->advertiseService("reload", &RosAPI::ReloadCB, this));
	service_servers_.emplace_back(nh_->advertiseService("reset", &RosAPI::ResetCB, this));
	service_servers_.emplace_back(nh_->advertiseService("set_body_state", &RosAPI::SetBodyStateCB, this));
	service_servers_.emplace_back(nh_->advertiseService("get_body_state", &RosAPI::GetBodyStateCB, this));
	service_servers_.emplace_back(nh_->advertiseService("set_geom_properties", &RosAPI::SetGeomPropertiesCB, this));
	service_servers_.emplace_back(nh_->advertiseService("get_geom_properties", &RosAPI::GetGeomPropertiesCB, this));
	service_servers_.emplace_back(
	    nh_->advertiseService("set_eq_constraint_parameters", &RosAPI::SetEqualityConstraintParametersArrayCB, this));
	service_servers_.emplace_back(
	    nh_->advertiseService("get_eq_constraint_parameters", &RosAPI::GetEqualityConstraintParametersArrayCB, this));
	service_servers_.emplace_back(
	    nh_->advertiseService("load_initial_joint_states", &RosAPI::LoadInitialJointStatesCB, this));
	service_servers_.emplace_back(nh_->advertiseService("get_loading_request_state", &RosAPI::GetStateUintCB, this));
	service_servers_.emplace_back(nh_->advertiseService("get_sim_info", &RosAPI::GetSimInfoCB, this));
	service_servers_.emplace_back(nh_->advertiseService("set_rt_factor", &RosAPI::SetRTFactorCB, this));
	service_servers_.emplace_back(nh_->advertiseService("get_plugin_stats", &RosAPI::GetPluginStatsCB, this));
	service_servers_.emplace_back(nh_->advertiseService("set_gravity", &RosAPI::SetGravityCB, this));
	service_servers_.emplace_back(nh_->advertiseService("get_gravity", &RosAPI::GetGravityCB, this));

	action_step_ = std::make_unique<actionlib::SimpleActionServer<mujoco_ros_msgs::StepAction>>(
	    *nh_, "step", boost::bind(&RosAPI::OnStepGoal, this, boost::placeholders::_1), false);
	action_step_->start();

	// param_server_ = std::make_unique<dynamic_reconfigure::Server<mujoco_ros::SimParamsConfig>>(sim_params_mutex_,
	// *nh_); param_server_->setCallback(
	//     std::bind(&RosAPI::dynparamCallback, this, std::placeholders::_1, std::placeholders::_2));
}

// Helper function to convert a double array to a space-delimited string
void arr_to_string(const mjtNum *arr, int size, std::string &str)
{
	str.clear();
	for (int i = 0; i < size; ++i) {
		str += std::to_string(arr[i]);
		if (i < size - 1) {
			str += " ";
		}
	}
}

// TODO(dleins): once changes from python bindings are merged, use RosAPISettings struct instead of passing all these
// arguments
// void ReadSimParams(SimParamsConfig &config, mjModel *model_, bool is_running, const std::string &admin_hash)
// {
// 	if (model_ != nullptr) {
// 		config.running    = is_running;
// 		config.admin_hash = admin_hash;

// 		config.integrator = model_->opt.integrator;
// 		config.cone       = model_->opt.cone;
// 		config.jacobian   = model_->opt.jacobian;
// 		config.solver     = model_->opt.solver;

// 		config.timestep    = model_->opt.timestep;
// 		config.iterations  = model_->opt.iterations;
// 		config.tolerance   = model_->opt.tolerance;
// 		config.ls_iter     = model_->opt.ls_iterations;
// 		config.ls_tol      = model_->opt.ls_tolerance;
// 		config.noslip_iter = model_->opt.noslip_iterations;
// 		config.noslip_tol  = model_->opt.noslip_tolerance;
// 		config.mpr_iter    = model_->opt.mpr_iterations;
// 		config.mpr_tol     = model_->opt.mpr_tolerance;
// 		config.sdf_iter    = model_->opt.sdf_iterations;
// 		config.sdf_init    = model_->opt.sdf_initpoints;

// 		std::string gravity;
// 		arr_to_string(model_->opt.gravity, 3, gravity);
// 		config.gravity = gravity;

// 		std::string wind;
// 		arr_to_string(model_->opt.wind, 3, wind);
// 		config.wind = wind;

// 		std::string magnetic;
// 		arr_to_string(model_->opt.magnetic, 3, magnetic);
// 		config.magnetic = magnetic;

// 		config.density   = model_->opt.density;
// 		config.viscosity = model_->opt.viscosity;
// 		config.impratio  = model_->opt.impratio;

// 		config.constraint_disabled   = ((model_->opt.disableflags & (1 << 0)) != 0);
// 		config.equality_disabled     = ((model_->opt.disableflags & (1 << 1)) != 0);
// 		config.frictionloss_disabled = ((model_->opt.disableflags & (1 << 2)) != 0);
// 		config.limit_disabled        = ((model_->opt.disableflags & (1 << 3)) != 0);
// 		config.contact_disabled      = ((model_->opt.disableflags & (1 << 4)) != 0);
// 		config.passive_disabled      = ((model_->opt.disableflags & (1 << 5)) != 0);
// 		config.gravity_disabled      = ((model_->opt.disableflags & (1 << 6)) != 0);
// 		config.clampctrl_disabled    = ((model_->opt.disableflags & (1 << 7)) != 0);
// 		config.warmstart_disabled    = ((model_->opt.disableflags & (1 << 8)) != 0);
// 		config.filterparent_disabled = ((model_->opt.disableflags & (1 << 9)) != 0);
// 		config.actuation_disabled    = ((model_->opt.disableflags & (1 << 10)) != 0);
// 		config.refsafe_disabled      = ((model_->opt.disableflags & (1 << 11)) != 0);
// 		config.sensor_disabled       = ((model_->opt.disableflags & (1 << 12)) != 0);
// 		config.midphase_disabled     = ((model_->opt.disableflags & (1 << 13)) != 0);
// 		config.eulerdamp_disabled    = ((model_->opt.disableflags & (1 << 14)) != 0);

// 		config.override_contacts = ((model_->opt.enableflags & (1 << 0)) != 0);
// 		config.energy            = ((model_->opt.enableflags & (1 << 1)) != 0);
// 		config.fwd_inv           = ((model_->opt.enableflags & (1 << 2)) != 0);
// 		config.inv_discrete      = ((model_->opt.enableflags & (1 << 3)) != 0);
// 		config.multiccd          = ((model_->opt.enableflags & (1 << 4)) != 0);
// 		config.island            = ((model_->opt.enableflags & (1 << 5)) != 0);

// 		config.margin = model_->opt.o_margin;

// 		std::string solimp;
// 		arr_to_string(model_->opt.o_solimp, mjNIMP, solimp);
// 		config.solimp = solimp;

// 		std::string solref;
// 		arr_to_string(model_->opt.o_solref, mjNREF, solref);
// 		config.solref = solref;

// 		std::string friction;
// 		arr_to_string(model_->opt.o_friction, 5, friction);
// 		config.friction = friction;
// 	}
// }

// void RosAPI::UpdateDynamicParams()
// {
// 	boost::recursive_mutex::scoped_lock lk(sim_params_mutex_);
// 	SimParamsConfig config;
// 	ReadSimParams(config, model_.get(), settings_.run.load(), std::string(settings_.admin_hash));
// 	param_server_->updateConfig(config);
// }

// void RosAPI::DynparamCallback(mujoco_ros::SimParamsConfig &config, uint32_t level)
// {
// 	boost::recursive_mutex::scoped_lock lk(sim_params_mutex_);
// 	if (level == 0xFFFFFFFF) {
// 		// First call on init -> Set params from model
// 		readSimParams(config, model_.get(), settings_.run.load(), std::string(settings_.admin_hash));
// 		return;
// 	}
// 	settings_.run.store(config.running);
// 	mju::strcpy_arr(settings_.admin_hash, config.admin_hash.c_str());

// 	model_->opt.integrator = config.integrator;
// 	model_->opt.cone       = config.cone;
// 	model_->opt.jacobian   = config.jacobian;
// 	model_->opt.solver     = config.solver;

// 	model_->opt.timestep          = config.timestep;
// 	model_->opt.iterations        = config.iterations;
// 	model_->opt.tolerance         = config.tolerance;
// 	model_->opt.ls_iterations     = config.ls_iter;
// 	model_->opt.ls_tolerance      = config.ls_tol;
// 	model_->opt.noslip_iterations = config.noslip_iter;
// 	model_->opt.noslip_tolerance  = config.noslip_tol;
// 	model_->opt.mpr_iterations    = config.mpr_iter;
// 	model_->opt.mpr_tolerance     = config.mpr_tol;
// 	model_->opt.sdf_iterations    = config.sdf_iter;
// 	model_->opt.sdf_initpoints    = config.sdf_init;

// 	set_from_string(model_->opt.gravity, config.gravity, 3);
// 	set_from_string(model_->opt.wind, config.wind, 3);
// 	set_from_string(model_->opt.magnetic, config.magnetic, 3);
// 	model_->opt.density   = config.density;
// 	model_->opt.viscosity = config.viscosity;
// 	model_->opt.impratio  = config.impratio;

// 	bit_set_to(model_->opt.disableflags, 0, config.constraint_disabled);
// 	bit_set_to(model_->opt.disableflags, 1, config.equality_disabled);
// 	bit_set_to(model_->opt.disableflags, 2, config.frictionloss_disabled);
// 	bit_set_to(model_->opt.disableflags, 3, config.limit_disabled);
// 	bit_set_to(model_->opt.disableflags, 4, config.contact_disabled);
// 	bit_set_to(model_->opt.disableflags, 5, config.passive_disabled);
// 	bit_set_to(model_->opt.disableflags, 6, config.gravity_disabled);
// 	bit_set_to(model_->opt.disableflags, 7, config.clampctrl_disabled);
// 	bit_set_to(model_->opt.disableflags, 8, config.warmstart_disabled);
// 	bit_set_to(model_->opt.disableflags, 9, config.filterparent_disabled);
// 	bit_set_to(model_->opt.disableflags, 10, config.actuation_disabled);
// 	bit_set_to(model_->opt.disableflags, 11, config.refsafe_disabled);
// 	bit_set_to(model_->opt.disableflags, 12, config.sensor_disabled);
// 	bit_set_to(model_->opt.disableflags, 13, config.midphase_disabled);
// 	bit_set_to(model_->opt.disableflags, 14, config.eulerdamp_disabled);

// 	bit_set_to(model_->opt.enableflags, 0, config.override_contacts);
// 	bit_set_to(model_->opt.enableflags, 1, config.energy);
// 	bit_set_to(model_->opt.enableflags, 2, config.fwd_inv);
// 	bit_set_to(model_->opt.enableflags, 3, config.inv_discrete);
// 	bit_set_to(model_->opt.enableflags, 4, config.multiccd);
// 	bit_set_to(model_->opt.enableflags, 5, config.island);

// 	model_->opt.o_margin = config.margin;
// 	set_from_string(model_->opt.o_solimp, config.solimp, mjNIMP);
// 	set_from_string(model_->opt.o_solref, config.solref, mjNREF);
// 	set_from_string(model_->opt.o_friction, config.friction, 5);
// }

void RosAPI::OnStepGoal(const mujoco_ros_msgs::StepGoalConstPtr &goal)
{
	mujoco_ros_msgs::StepResult result;

	if (env_ptr_->settings_.env_steps_request.load() > 0 || env_ptr_->settings_.run.load()) {
		ROS_WARN("Simulation is currently unpaused. Stepping makes no sense right now.");
		result.success = false;
		action_step_->setPreempted(result);
		return;
	}

	mujoco_ros_msgs::StepFeedback feedback;

	feedback.steps_left = goal->num_steps;
	env_ptr_->settings_.env_steps_request.store(goal->num_steps);

	result.success = true;
	while (env_ptr_->settings_.env_steps_request.load() > 0) {
		if (action_step_->isPreemptRequested() || !ros::ok() || env_ptr_->settings_.exit_request.load() > 0 ||
		    env_ptr_->settings_.load_request.load() > 0 || env_ptr_->settings_.reset_request.load() > 0) {
			ROS_WARN_STREAM("Simulation step action preempted");
			feedback.steps_left = util::as_unsigned(env_ptr_->settings_.env_steps_request.load());
			action_step_->publishFeedback(feedback);
			result.success = false;
			action_step_->setPreempted(result);
			env_ptr_->settings_.env_steps_request.store(0);
			return;
		}

		feedback.steps_left = util::as_unsigned(env_ptr_->settings_.env_steps_request.load());
		action_step_->publishFeedback(feedback);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	feedback.steps_left = util::as_unsigned(env_ptr_->settings_.env_steps_request.load());
	action_step_->publishFeedback(feedback);
	action_step_->setSucceeded(result);
}

bool RosAPI::SetPauseCB(mujoco_ros_msgs::SetPause::Request &req, mujoco_ros_msgs::SetPause::Response &res)
{
	if (req.paused) {
		ROS_DEBUG("Requested pause via ROS service");
	} else {
		ROS_DEBUG("Requested unpause via ROS service");
	}
	res.success = env_ptr_->TogglePaused(req.paused, req.admin_hash);
	return true;
}

bool RosAPI::ShutdownCB(std_srvs::Empty::Request & /*req*/, std_srvs::Empty::Response & /*res*/)
{
	env_ptr_->Shutdown();
	return true;
}

// TODO: Check why message has admin_hash (not used in LoadModelFromString)
bool RosAPI::ReloadCB(mujoco_ros_msgs::Reload::Request &req, mujoco_ros_msgs::Reload::Response &res)
{
	char load_error[MujocoEnv::kErrorLength];
	res.success        = env_ptr_->LoadModelFromString(req.model, load_error);
	res.status_message = load_error;
	return true;
}

bool RosAPI::ResetCB(std_srvs::Empty::Request & /*req*/, std_srvs::Empty::Response & /*res*/)
{
	env_ptr_->Reset();
	return true;
}

bool RosAPI::SetBodyStateCB(mujoco_ros_msgs::SetBodyState::Request &req, mujoco_ros_msgs::SetBodyState::Response &res)
{
	geometry_msgs::PoseStamped target_pose;

	mjtNum state[15] = { 0 };
	mjtNum *pose     = state;
	mjtNum *quat     = pose + 3;
	mjtNum *twist    = pose + 7;
	mjtNum *mass     = twist + 6;

	std::string full_error_msg = "";

	bool resp_failure_override = false;

	if (req.set_pose) {
		geometry_msgs::PoseStamped init_pose = req.state.pose;
		bool valid_pose                      = true;
		if (!init_pose.header.frame_id.empty() && init_pose.header.frame_id != "world") {
			try {
				env_ptr_->tf_buffer_->transform<geometry_msgs::PoseStamped>(init_pose, target_pose, "world");
			} catch (tf2::TransformException &ex) {
				MJR_WARN_STREAM(ex.what());
				full_error_msg +=
				    "Could not transform frame '" + req.state.pose.header.frame_id + "' to frame world" + '\n';
				resp_failure_override = true;
				valid_pose            = false;
			}
		} else {
			target_pose = req.state.pose;
		}

		if (valid_pose) {
			pose[0] = target_pose.pose.position.x;
			pose[1] = target_pose.pose.position.y;
			pose[2] = target_pose.pose.position.z;
			pose[3] = target_pose.pose.orientation.w;
			pose[4] = target_pose.pose.orientation.x;
			pose[5] = target_pose.pose.orientation.y;
			pose[6] = target_pose.pose.orientation.z;

			mju_normalize4(quat);
		} else {
			MJR_WARN("Invalid pose, not setting pose");
			req.set_pose = false;
		}
	}

	if (req.set_twist) {
		// Only pose can be transformed. Twist will be ignored!
		if (!req.state.twist.header.frame_id.empty() && req.state.twist.header.frame_id != "world") {
			std::string error_msg("Transforming twists from other frames is not supported! Not setting twist.");
			ROS_WARN_STREAM(error_msg);
			full_error_msg += error_msg + '\n';
			resp_failure_override = true;
		} else {
			twist[0] = req.state.twist.twist.linear.x;
			twist[1] = req.state.twist.twist.linear.y;
			twist[2] = req.state.twist.twist.linear.z;
			twist[3] = req.state.twist.twist.angular.x;
			twist[4] = req.state.twist.twist.angular.y;
			twist[5] = req.state.twist.twist.angular.z;
		}
	}

	char status_msg[MujocoEnv::kErrorLength] = { 0 };

	*mass       = static_cast<mjtNum>(req.state.mass);
	res.success = env_ptr_->SetBodyState(req.state.name, pose, twist, *mass, req.set_pose, req.set_twist, req.set_mass,
	                                     req.reset_qpos, req.admin_hash, status_msg, MujocoEnv::kErrorLength);

	if (resp_failure_override) {
		res.success        = false;
		res.status_message = full_error_msg + "\n" + std::string(status_msg);
	}
	return true;
}

bool RosAPI::GetBodyStateCB(mujoco_ros_msgs::GetBodyState::Request &req, mujoco_ros_msgs::GetBodyState::Response &res)
{
	std::string body_name = req.name;
	mjtNum state[14]      = { 0 };
	mjtNum *pose          = state;
	mjtNum *twist         = pose + 7;
	mjtNum *mass          = twist + 6;

	char status_msg[MujocoEnv::kErrorLength] = { 0 };

	res.success =
	    env_ptr_->GetBodyState(body_name, pose, twist, mass, req.admin_hash, status_msg, MujocoEnv::kErrorLength);

	res.status_message = std::string(status_msg);

	res.state.mass = static_cast<decltype(res.state.mass)>(*mass);

	res.state.pose.header             = std_msgs::Header();
	res.state.pose.header.frame_id    = "world";
	res.state.pose.pose.position.x    = pose[0];
	res.state.pose.pose.position.y    = pose[1];
	res.state.pose.pose.position.z    = pose[2];
	res.state.pose.pose.orientation.w = pose[3];
	res.state.pose.pose.orientation.x = pose[4];
	res.state.pose.pose.orientation.y = pose[5];
	res.state.pose.pose.orientation.z = pose[6];

	res.state.twist.header          = std_msgs::Header();
	res.state.twist.header.frame_id = "world";
	res.state.twist.twist.linear.x  = twist[0];
	res.state.twist.twist.linear.y  = twist[1];
	res.state.twist.twist.linear.z  = twist[2];
	res.state.twist.twist.angular.x = twist[3];
	res.state.twist.twist.angular.y = twist[4];
	res.state.twist.twist.angular.z = twist[5];

	return true;
}

bool RosAPI::SetGravityCB(mujoco_ros_msgs::SetGravity::Request &req, mujoco_ros_msgs::SetGravity::Response &res)
{
	char status_msg[MujocoEnv::kErrorLength] = { 0 };
	res.success = env_ptr_->SetGravity(req.gravity.c_array(), req.admin_hash, status_msg, MujocoEnv::kErrorLength);
	res.status_message = std::string(status_msg);
	return true;
}

bool RosAPI::GetGravityCB(mujoco_ros_msgs::GetGravity::Request &req, mujoco_ros_msgs::GetGravity::Response &res)
{
	char status_msg[MujocoEnv::kErrorLength] = { 0 };
	res.success = env_ptr_->GetGravity(res.gravity.c_array(), req.admin_hash, status_msg, MujocoEnv::kErrorLength);
	res.status_message = std::string(status_msg);
	return true;
}

bool RosAPI::SetGeomPropertiesCB(mujoco_ros_msgs::SetGeomProperties::Request &req,
                                 mujoco_ros_msgs::SetGeomProperties::Response &res)
{
	char status_msg[MujocoEnv::kErrorLength] = { 0 };
	res.success                              = env_ptr_->SetGeomProperties(
	                                 req.properties.name, req.properties.body_mass, req.properties.friction_slide, req.properties.friction_spin,
	                                 req.properties.friction_roll, req.properties.size_0, req.properties.size_1, req.properties.size_2,
	                                 req.properties.type.value, req.set_mass, req.set_friction, req.set_type, req.set_size, req.admin_hash,
	                                 status_msg, MujocoEnv::kErrorLength);
	res.status_message = std::string(status_msg);
	return true;
}

bool RosAPI::GetGeomPropertiesCB(mujoco_ros_msgs::GetGeomProperties::Request &req,
                                 mujoco_ros_msgs::GetGeomProperties::Response &res)
{
	mjtNum properties[8]                     = { 0 };
	char status_msg[MujocoEnv::kErrorLength] = { 0 };

	res.success        = env_ptr_->GetGeomProperties(req.geom_name,
	                                                 properties[0], // body_mass
	                                                 properties[1], // friction_slide
	                                                 properties[2], // friction_spin
	                                                 properties[3], // friction_roll
	                                                 properties[4], // size_x
	                                                 properties[5], // size_y
	                                                 properties[6], // size_z
	                                                 properties[7], // type
	                                                 req.admin_hash, status_msg, MujocoEnv::kErrorLength);
	res.status_message = std::string(status_msg);

	res.properties.name           = req.geom_name;
	res.properties.body_mass      = static_cast<decltype(res.properties.body_mass)>(properties[0]);
	res.properties.friction_slide = static_cast<decltype(res.properties.friction_slide)>(properties[1]);
	res.properties.friction_spin  = static_cast<decltype(res.properties.friction_spin)>(properties[2]);
	res.properties.friction_roll  = static_cast<decltype(res.properties.friction_roll)>(properties[3]);
	res.properties.size_0         = static_cast<decltype(res.properties.size_0)>(properties[4]);
	res.properties.size_1         = static_cast<decltype(res.properties.size_1)>(properties[5]);
	res.properties.size_2         = static_cast<decltype(res.properties.size_2)>(properties[6]);
	res.properties.type.value     = static_cast<decltype(res.properties.type.value)>(properties[7]);

	return true;
}

bool RosAPI::SetEqualityConstraintParameters(const mujoco_ros_msgs::EqualityConstraintParameters &parameters,
                                             const std::string &admin_hash, char *status_message, const int status_sz)
{
	// TODO: add set_bools to message to not require all parameters to be set
	//  3 for anchor, 7 for relpose, mjNEQDATA for polycoef, 1 for torquescale, mjNIMP + mjNREF for solimp and solref
	//  (solver_params)
	mjtNum params[3 + 7 + mjNEQDATA + 1 + mjNIMP + mjNREF] = { 0 };
	mjtNum *anchor                                         = params;
	mjtNum *relpose                                        = anchor + 3;
	mjtNum *polycoef                                       = relpose + 7;
	mjtNum *torquescale                                    = polycoef + mjNEQDATA;
	mjtNum *solver_params                                  = torquescale + 1;

	anchor[0] = parameters.anchor.x;
	anchor[1] = parameters.anchor.y;
	anchor[2] = parameters.anchor.z;

	relpose[0] = parameters.relpose.position.x;
	relpose[1] = parameters.relpose.position.y;
	relpose[2] = parameters.relpose.position.z;
	relpose[3] = parameters.relpose.orientation.w;
	relpose[4] = parameters.relpose.orientation.x;
	relpose[5] = parameters.relpose.orientation.y;
	relpose[6] = parameters.relpose.orientation.z;

	mju_copy(polycoef, parameters.polycoef.data(), mjNEQDATA);
	torquescale[0] = parameters.torquescale;

	return env_ptr_->SetEqualityConstraintParameters(
	    parameters.name, parameters.type.value, solver_params, parameters.active, parameters.element1,
	    parameters.element2, *torquescale, anchor, relpose, polycoef, admin_hash, status_message, status_sz);
}

bool RosAPI::SetEqualityConstraintParametersArrayCB(mujoco_ros_msgs::SetEqualityConstraintParameters::Request &req,
                                                    mujoco_ros_msgs::SetEqualityConstraintParameters::Response &res)
{
	res.success = true;

	bool failed_any    = false;
	bool succeeded_any = false;
	char error_msg[MujocoEnv::kErrorLength];
	std::string status_message;

	for (const auto &parameters : req.parameters) {
		status_message[0] = '\0';
		bool success  = SetEqualityConstraintParameters(parameters, req.admin_hash, error_msg, MujocoEnv::kErrorLength);
		failed_any    = (failed_any || !success);
		succeeded_any = (succeeded_any || success);
		if (!success) {
			status_message += std::string(error_msg) + '\n';
		}
	}

	if (succeeded_any && failed_any) {
		status_message += "Not all constraints could be set";
		res.status_message = status_message;
		res.success        = false;
	} else if (failed_any) {
		status_message += "Could not set any constraints";
		res.status_message = status_message;
		res.success        = false;
	}

	return true;
}

bool RosAPI::GetEqualityConstraintParameters(mujoco_ros_msgs::EqualityConstraintParameters &parameters,
                                             const std::string &admin_hash, char *status_message, const int status_sz)
{
	// 3 for anchor, 7 for relpose, mjNEQDATA for polycoef, 1 for torquescale, mjNIMP + mjNREF for solimp and solref
	// (solver_params)
	mjtNum params[3 + 7 + mjNEQDATA + 1 + mjNIMP + mjNREF] = { 0 };
	mjtNum *anchor                                         = params;
	mjtNum *relpose                                        = anchor + 3;
	mjtNum *polycoef                                       = relpose + 7;
	mjtNum *torquescale                                    = polycoef + mjNEQDATA;
	mjtNum *solver_params                                  = torquescale + 1;
	int type;
	bool active, success;

	success = env_ptr_->GetEqualityConstraintParameters(parameters.name, type, solver_params, active,
	                                                    parameters.element1, parameters.element2, *torquescale, anchor,
	                                                    relpose, polycoef, admin_hash, status_message, status_sz);

	parameters.type.value = static_cast<decltype(parameters.type.value)>(type);
	parameters.active     = active;
	return success;
}

bool RosAPI::GetEqualityConstraintParametersArrayCB(mujoco_ros_msgs::GetEqualityConstraintParameters::Request &req,
                                                    mujoco_ros_msgs::GetEqualityConstraintParameters::Response &res)
{
	res.success = true;

	bool failed_any    = false;
	bool succeeded_any = false;
	char error_msg[MujocoEnv::kErrorLength];
	std::string status_message;
	for (const auto &name : req.names) {
		status_message[0] = '\0';
		mujoco_ros_msgs::EqualityConstraintParameters eqc;
		eqc.name     = name;
		bool success = GetEqualityConstraintParameters(eqc, req.admin_hash, error_msg, MujocoEnv::kErrorLength);

		failed_any    = (failed_any || !success);
		succeeded_any = (succeeded_any || success);
		if (success) {
			res.parameters.emplace_back(eqc);
		} else {
			status_message += std::string(error_msg) + '\n';
		}
	}

	if (succeeded_any && failed_any) {
		status_message += "Result: Not all constraints could be fetched";
		res.status_message = status_message;
		res.success        = false;
	} else if (failed_any) {
		status_message += "Result: Could not fetch any constraints";
		res.status_message = status_message;
		res.success        = false;
	}

	return true;
}

bool RosAPI::GetStateUintCB(mujoco_ros_msgs::GetStateUint::Request & /*req*/,
                            mujoco_ros_msgs::GetStateUint::Response &res)
{
	int status;
	env_ptr_->GetSimulationStatus(status, res.state.description);
	res.state.value = static_cast<decltype(res.state.value)>(status);
	return true;
}

bool RosAPI::GetSimInfoCB(mujoco_ros_msgs::GetSimInfo::Request & /*req*/, mujoco_ros_msgs::GetSimInfo::Response &res)
{
	bool valid, paused;
	int load_count, loading_state, pending_sim_steps;
	env_ptr_->GetSimInfo(res.state.model_path, valid, load_count, loading_state, res.state.loading_state.description,
	                     paused, pending_sim_steps, res.state.rt_measured, res.state.rt_setting);
	res.state.model_valid         = valid;
	res.state.load_count          = static_cast<decltype(res.state.load_count)>(load_count);
	res.state.loading_state.value = static_cast<decltype(res.state.loading_state.value)>(loading_state);
	res.state.paused              = paused;
	res.state.pending_sim_steps   = static_cast<decltype(res.state.pending_sim_steps)>(pending_sim_steps);
	return true;
}

bool RosAPI::SetRTFactorCB(mujoco_ros_msgs::SetFloat::Request &req, mujoco_ros_msgs::SetFloat::Response &res)
{
	res.success = env_ptr_->SetRealTimeFactor(static_cast<float>(req.value), req.admin_hash);
	return true;
}

bool RosAPI::GetPluginStatsCB(mujoco_ros_msgs::GetPluginStats::Request & /*req*/,
                              mujoco_ros_msgs::GetPluginStats::Response &res)
{
	std::vector<std::string> plugin_names;
	std::vector<std::string> types;
	std::vector<double> load_times;
	std::vector<double> reset_times;
	std::vector<double> ema_steptimes_control;
	std::vector<double> ema_steptimes_passive;
	std::vector<double> ema_steptimes_render;
	std::vector<double> ema_steptimes_last_stage;

	int num_plugins = env_ptr_->GetPluginStats(plugin_names, types, load_times, reset_times, ema_steptimes_control,
	                                           ema_steptimes_passive, ema_steptimes_render, ema_steptimes_last_stage);

	for (int i = 0; i < num_plugins; ++i) {
		mujoco_ros_msgs::PluginStats stats;
		// stats.plugin_name             = plugin_names[i]; // TODO: add plugin name to message
		stats.plugin_type             = types[i];
		stats.load_time               = load_times[i];
		stats.reset_time              = reset_times[i];
		stats.ema_steptime_control    = ema_steptimes_control[i];
		stats.ema_steptime_passive    = ema_steptimes_passive[i];
		stats.ema_steptime_render     = ema_steptimes_render[i];
		stats.ema_steptime_last_stage = ema_steptimes_last_stage[i];
		res.stats.emplace_back(stats);
	}
	return true;
}

bool RosAPI::LoadInitialJointStatesCB(std_srvs::Empty::Request & /*req*/, std_srvs::Empty::Response & /*res*/)
{
	env_ptr_->LoadInitialJointStates();
	return true;
}

void RosAPI::SetupClockPublisher()
{
	if (env_ptr_->settings_.use_sim_time) {
		clock_pub_ = nh_->advertise<rosgraph_msgs::Clock>("/clock", 1);
		PublishSimTime(mjtNum(0));
	}
}

void RosAPI::PublishSimTime(mjtNum time)
{
	if (!env_ptr_->settings_.use_sim_time) {
		return;
	}
	// This is the fastes option for intra-node time updates
	// however, together with non-blocking publish it breaks stuff
	// ros::Time::setNow(ros::Time(time));
	rosgraph_msgs::ClockPtr ros_time(new rosgraph_msgs::Clock);
	ros_time->clock.fromSec(time);
	clock_pub_.publish(ros_time);
	// Current workaround to simulate a blocking time update
	while (ros::Time::now() < ros::Time(time)) {
		std::this_thread::yield();
	}
}

} // namespace mujoco_ros
