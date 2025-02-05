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

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/logging.hpp>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/ros_two/plugin_utils.hpp>
#include <mujoco_ros/ros_two/ros_api.hpp>
#include <mujoco_ros/util.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mujoco_ros {

RosAPI::RosAPI(MujocoEnvPtr env_ptr) : env_ptr_(env_ptr)
{
	RCLCPP_DEBUG(env_ptr_->get_logger(), "Declaring default ros parameters");
	declare_parameter_if_not_declared(env_ptr_, "eval_mode", rclcpp::ParameterValue(false));
	declare_parameter_if_not_declared(env_ptr_, "no_render", rclcpp::ParameterValue(false));
	declare_parameter_if_not_declared(env_ptr_, "render_offscreen", rclcpp::ParameterValue(false));
	declare_parameter_if_not_declared(env_ptr_, "headless", rclcpp::ParameterValue(false));
	declare_parameter_if_not_declared(env_ptr_, "no_x", rclcpp::ParameterValue(false));
	declare_parameter_if_not_declared(env_ptr_, "unpause", rclcpp::ParameterValue(true));
	declare_parameter_if_not_declared(env_ptr_, "num_steps", rclcpp::ParameterValue(-1));
	declare_parameter_if_not_declared(env_ptr_, "modelfile", rclcpp::ParameterValue(std::string("")));
	declare_parameter_if_not_declared(env_ptr_, "realtime", rclcpp::ParameterValue(1.0));
	declare_parameter_if_not_declared(env_ptr_, plugin_utils::MUJOCO_PLUGIN_PARAM_NAME + ".names",
	                                  rclcpp::ParameterValue(std::vector<std::string>{}));
	declare_parameter_if_not_declared(env_ptr_, "wait_for_xml", rclcpp::ParameterValue(false));
	declare_parameter_if_not_declared(env_ptr_, "mujoco_xml", rclcpp::ParameterValue(std::string("")));
	declare_parameter_if_not_declared(env_ptr_, "use_sim_time", rclcpp::ParameterValue(true));
}

void RosAPI::SetupServices()
{
	MJR_ERROR_STREAM("Effective namespace: " << env_ptr_->get_namespace());
	std::string ns = env_ptr_->get_namespace();
	set_pause_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::SetPause>(
	    ns + "set_pause", std::bind(&RosAPI::SetPauseCB, this, std::placeholders::_1, std::placeholders::_2));
	shutdown_srv_ = env_ptr_->create_service<std_srvs::srv::Empty>(
	    ns + "shutdown", std::bind(&RosAPI::ShutdownCB, this, std::placeholders::_1, std::placeholders::_2));
	reload_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::Reload>(
	    ns + "reload", std::bind(&RosAPI::ReloadCB, this, std::placeholders::_1, std::placeholders::_2));
	reset_srv_ = env_ptr_->create_service<std_srvs::srv::Empty>(
	    ns + "reset", std::bind(&RosAPI::ResetCB, this, std::placeholders::_1, std::placeholders::_2));
	set_body_state_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::SetBodyState>(
	    ns + "set_body_state", std::bind(&RosAPI::SetBodyStateCB, this, std::placeholders::_1, std::placeholders::_2));
	get_body_state_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetBodyState>(
	    ns + "get_body_state", std::bind(&RosAPI::GetBodyStateCB, this, std::placeholders::_1, std::placeholders::_2));
	set_geom_properties_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::SetGeomProperties>(
	    ns + "set_geom_properties",
	    std::bind(&RosAPI::SetGeomPropertiesCB, this, std::placeholders::_1, std::placeholders::_2));
	get_geom_properties_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetGeomProperties>(
	    ns + "get_geom_properties",
	    std::bind(&RosAPI::GetGeomPropertiesCB, this, std::placeholders::_1, std::placeholders::_2));
	set_eq_constraint_parameters_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::SetEqualityConstraintParameters>(
	    ns + "set_eq_constraint_parameters",
	    std::bind(&RosAPI::SetEqualityConstraintParametersArrayCB, this, std::placeholders::_1, std::placeholders::_2));
	get_eq_constraint_parameters_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetEqualityConstraintParameters>(
	    ns + "get_eq_constraint_parameters",
	    std::bind(&RosAPI::GetEqualityConstraintParametersArrayCB, this, std::placeholders::_1, std::placeholders::_2));
	load_initial_joint_states_srv_ = env_ptr_->create_service<std_srvs::srv::Empty>(
	    ns + "load_initial_joint_states",
	    std::bind(&RosAPI::LoadInitialJointStatesCB, this, std::placeholders::_1, std::placeholders::_2));
	get_state_uint_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetStateUint>(
	    ns + "get_loading_request_state",
	    std::bind(&RosAPI::GetStateUintCB, this, std::placeholders::_1, std::placeholders::_2));
	get_sim_info_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetSimInfo>(
	    ns + "get_sim_info", std::bind(&RosAPI::GetSimInfoCB, this, std::placeholders::_1, std::placeholders::_2));
	set_rt_factor_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::SetFloat>(
	    ns + "set_rt_factor", std::bind(&RosAPI::SetRTFactorCB, this, std::placeholders::_1, std::placeholders::_2));
	get_plugin_stats_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetPluginStats>(
	    ns + "get_plugin_stats",
	    std::bind(&RosAPI::GetPluginStatsCB, this, std::placeholders::_1, std::placeholders::_2));
	set_gravity_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::SetGravity>(
	    ns + "set_gravity", std::bind(&RosAPI::SetGravityCB, this, std::placeholders::_1, std::placeholders::_2));
	get_gravity_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetGravity>(
	    ns + "get_gravity", std::bind(&RosAPI::GetGravityCB, this, std::placeholders::_1, std::placeholders::_2));

	action_step_ = rclcpp_action::create_server<mujoco_ros_msgs::action::Step>(
	    env_ptr_, ns + "step", std::bind(&RosAPI::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
	    std::bind(&RosAPI::HandleCancel, this, std::placeholders::_1),
	    std::bind(&RosAPI::OnStepGoal, this, std::placeholders::_1));
}

void RosAPI::OnStepGoal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mujoco_ros_msgs::action::Step>> goal_handle)
{
	const auto goal  = goal_handle->get_goal();
	auto feedback    = std::make_shared<mujoco_ros_msgs::action::Step::Feedback>();
	auto &steps_left = feedback->steps_left;
	auto result      = std::make_shared<mujoco_ros_msgs::action::Step::Result>();

	if (env_ptr_->settings_.env_steps_request.load() > 0 || env_ptr_->settings_.run.load()) {
		MJR_WARN("Simulation is currently unpaused. Stepping makes no sense right now.");
		result->success = false;
		goal_handle->abort(result);
		MJR_DEBUG("Aborted step goal");
		return;
	}

	steps_left = goal->num_steps;
	env_ptr_->settings_.env_steps_request.store(goal->num_steps);

	result->success = true;
	while (env_ptr_->settings_.env_steps_request.load() > 0) {
		if (goal_handle->is_canceling() || !rclcpp::ok() || env_ptr_->settings_.exit_request.load() > 0 ||
		    env_ptr_->settings_.load_request.load() > 0 || env_ptr_->settings_.reset_request.load() > 0) {
			MJR_WARN("Simulation step action preempted");
			steps_left = util::as_unsigned(env_ptr_->settings_.env_steps_request.load());
			goal_handle->publish_feedback(feedback);
			result->success = false;
			goal_handle->canceled(result);
			env_ptr_->settings_.env_steps_request.store(0);
			return;
		}

		steps_left = util::as_unsigned(env_ptr_->settings_.env_steps_request.load());
		goal_handle->publish_feedback(feedback);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	steps_left = util::as_unsigned(env_ptr_->settings_.env_steps_request.load());
	goal_handle->publish_feedback(feedback);
	goal_handle->succeed(result);
}

void RosAPI::SetPauseCB(const mujoco_ros_msgs::srv::SetPause::Request::SharedPtr req,
                        mujoco_ros_msgs::srv::SetPause::Response::SharedPtr res)
{
	res->success = env_ptr_->TogglePaused(req->paused, req->admin_hash);
}

void RosAPI::ShutdownCB(const std_srvs::srv::Empty::Request::SharedPtr /*req*/,
                        std_srvs::srv::Empty::Response::SharedPtr /*res*/)
{
	env_ptr_->Shutdown();
}

void RosAPI::ReloadCB(const mujoco_ros_msgs::srv::Reload::Request::SharedPtr req,
                      mujoco_ros_msgs::srv::Reload::Response::SharedPtr res)
{
	char load_error[MujocoEnv::kErrorLength];
	res->success        = env_ptr_->LoadModelFromString(req->model, load_error);
	res->status_message = load_error;
}

void RosAPI::ResetCB(const std_srvs::srv::Empty::Request::SharedPtr /*req*/,
                     std_srvs::srv::Empty::Response::SharedPtr /*res*/)
{
	env_ptr_->Reset();
}

void RosAPI::SetBodyStateCB(const mujoco_ros_msgs::srv::SetBodyState::Request::SharedPtr req,
                            mujoco_ros_msgs::srv::SetBodyState::Response::SharedPtr res)
{
	geometry_msgs::msg::PoseStamped target_pose;

	mjtNum state[15] = { 0 };
	mjtNum *pose     = state;
	mjtNum *quat     = pose + 3;
	mjtNum *twist    = pose + 7;
	mjtNum *mass     = twist + 6;

	std::string full_error_msg;

	bool resp_failure_override = false;

	if (req->set_pose) {
		geometry_msgs::msg::PoseStamped init_pose = req->state.pose;
		bool valid_pose                           = true;
		if (!init_pose.header.frame_id.empty() && init_pose.header.frame_id != "world") {
			try {
				env_ptr_->tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(init_pose, target_pose, "world");
			} catch (tf2::TransformException &ex) {
				MJR_WARN_STREAM(ex.what());
				full_error_msg +=
				    "Could not transform frame '" + req->state.pose.header.frame_id + "' to frame world" + '\n';
				resp_failure_override = true;
				valid_pose            = false;
			}
		} else {
			target_pose = req->state.pose;
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
			req->set_pose = false;
		}
	}

	if (req->set_twist) {
		// Only pose can be transformed. Twist will be ignored!
		if (!req->state.twist.header.frame_id.empty() && req->state.twist.header.frame_id != "world") {
			std::string error_msg("Transforming twists from other frames is not supported! Not setting twist.");
			MJR_WARN_STREAM(error_msg);
			full_error_msg += error_msg + '\n';
			resp_failure_override = true;
		} else {
			twist[0] = req->state.twist.twist.linear.x;
			twist[1] = req->state.twist.twist.linear.y;
			twist[2] = req->state.twist.twist.linear.z;
			twist[3] = req->state.twist.twist.angular.x;
			twist[4] = req->state.twist.twist.angular.y;
			twist[5] = req->state.twist.twist.angular.z;
		}
	}

	char status_msg[MujocoEnv::kErrorLength] = { 0 };
	*mass                                    = static_cast<mjtNum>(req->state.mass);
	res->success =
	    env_ptr_->SetBodyState(req->state.name, pose, twist, *mass, req->set_pose, req->set_twist, req->set_mass,
	                           req->reset_qpos, req->admin_hash, status_msg, MujocoEnv::kErrorLength);

	if (resp_failure_override) {
		res->success        = false;
		res->status_message = full_error_msg + "\n" + std::string(status_msg);
	}
}

void RosAPI::GetBodyStateCB(const mujoco_ros_msgs::srv::GetBodyState::Request::SharedPtr req,
                            mujoco_ros_msgs::srv::GetBodyState::Response::SharedPtr res)
{
	std::string body_name = req->name;
	mjtNum state[15]      = { 0 };
	mjtNum *pose          = state;
	mjtNum *twist         = pose + 7;
	mjtNum *mass          = twist + 6;

	char status_msg[MujocoEnv::kErrorLength] = { 0 };

	res->success =
	    env_ptr_->GetBodyState(body_name, pose, twist, mass, req->admin_hash, status_msg, MujocoEnv::kErrorLength);

	res->status_message = std::string(status_msg);
	res->state.mass     = static_cast<decltype(res->state.mass)>(*mass);

	res->state.pose.header             = std_msgs::msg::Header();
	res->state.pose.header.frame_id    = "world";
	res->state.pose.pose.position.x    = pose[0];
	res->state.pose.pose.position.y    = pose[1];
	res->state.pose.pose.position.z    = pose[2];
	res->state.pose.pose.orientation.w = pose[3];
	res->state.pose.pose.orientation.x = pose[4];
	res->state.pose.pose.orientation.y = pose[5];
	res->state.pose.pose.orientation.z = pose[6];

	res->state.twist.header          = std_msgs::msg::Header();
	res->state.twist.header.frame_id = "world";
	res->state.twist.twist.linear.x  = twist[0];
	res->state.twist.twist.linear.y  = twist[1];
	res->state.twist.twist.linear.z  = twist[2];
	res->state.twist.twist.angular.x = twist[3];
	res->state.twist.twist.angular.y = twist[4];
	res->state.twist.twist.angular.z = twist[5];
}

void RosAPI::SetGravityCB(const mujoco_ros_msgs::srv::SetGravity::Request::SharedPtr req,
                          mujoco_ros_msgs::srv::SetGravity::Response::SharedPtr res)
{
	char status_msg[MujocoEnv::kErrorLength] = { 0 };
	res->success = env_ptr_->SetGravity(req->gravity.data(), req->admin_hash, status_msg, MujocoEnv::kErrorLength);
	res->status_message = std::string(status_msg);
}

void RosAPI::GetGravityCB(const mujoco_ros_msgs::srv::GetGravity::Request::SharedPtr req,
                          mujoco_ros_msgs::srv::GetGravity::Response::SharedPtr res)
{
	char status_msg[MujocoEnv::kErrorLength] = { 0 };

	res->success = env_ptr_->GetGravity(res->gravity.data(), req->admin_hash, status_msg, MujocoEnv::kErrorLength);

	res->status_message = std::string(status_msg);
}

void RosAPI::SetGeomPropertiesCB(const mujoco_ros_msgs::srv::SetGeomProperties::Request::SharedPtr req,
                                 mujoco_ros_msgs::srv::SetGeomProperties::Response::SharedPtr res)
{
	char status_msg[MujocoEnv::kErrorLength] = { 0 };
	res->success                             = env_ptr_->SetGeomProperties(
	                                req->properties.name, req->properties.body_mass, req->properties.friction_slide, req->properties.friction_spin,
	                                req->properties.friction_roll, req->properties.size_0, req->properties.size_1, req->properties.size_2,
	                                req->properties.type.value, req->set_mass, req->set_friction, req->set_type, req->set_size, req->admin_hash,
	                                status_msg, MujocoEnv::kErrorLength);
	res->status_message = std::string(status_msg);
}

void RosAPI::GetGeomPropertiesCB(const mujoco_ros_msgs::srv::GetGeomProperties::Request::SharedPtr req,
                                 mujoco_ros_msgs::srv::GetGeomProperties::Response::SharedPtr res)
{
	mjtNum properties[8]                     = { 0 };
	char status_msg[MujocoEnv::kErrorLength] = { 0 };

	res->success        = env_ptr_->GetGeomProperties(req->geom_name,
	                                                  properties[0], // body_mass
	                                                  properties[1], // friction_slide
	                                                  properties[2], // friction_spin
	                                                  properties[3], // friction_roll
	                                                  properties[4], // size_x
	                                                  properties[5], // size_y
	                                                  properties[6], // size_z
	                                                  properties[7], // type
	                                                  req->admin_hash, status_msg, MujocoEnv::kErrorLength);
	res->status_message = std::string(status_msg);

	res->properties.name           = req->geom_name;
	res->properties.body_mass      = static_cast<decltype(res->properties.body_mass)>(properties[0]);
	res->properties.friction_slide = static_cast<decltype(res->properties.friction_slide)>(properties[1]);
	res->properties.friction_spin  = static_cast<decltype(res->properties.friction_spin)>(properties[2]);
	res->properties.friction_roll  = static_cast<decltype(res->properties.friction_roll)>(properties[3]);
	res->properties.size_0         = static_cast<decltype(res->properties.size_0)>(properties[4]);
	res->properties.size_1         = static_cast<decltype(res->properties.size_1)>(properties[5]);
	res->properties.size_2         = static_cast<decltype(res->properties.size_2)>(properties[6]);
	res->properties.type.value     = static_cast<decltype(res->properties.type.value)>(properties[7]);
}

bool RosAPI::SetEqualityConstraintParameters(const mujoco_ros_msgs::msg::EqualityConstraintParameters &parameters,
                                             const std::string &admin_hash, char *status_message, const int status_sz)
{
	// TODO: add set_bools to message to not require all parameters to be set
	// 3 for anchor, 7 for relpose, mjNEQDATA for polycoef, 1 for torquescale, mjNIMP + mjNREF for solimp and solref
	// (solver_params)
	mjtNum params[3 + 7 + mjNEQDATA + 1 + mjNIMP + mjNREF] = { 0 };
	mjtNum *anchor                                         = params;
	mjtNum *relpose                                        = anchor + 3;
	mjtNum *polycoef                                       = relpose + 7;
	mjtNum *torquescale                                    = polycoef + mjNEQDATA;
	mjtNum *solver_params                                  = torquescale + 1;

	return env_ptr_->SetEqualityConstraintParameters(
	    parameters.name, parameters.type.value, solver_params, parameters.active, parameters.element1,
	    parameters.element2, *torquescale, anchor, relpose, polycoef, admin_hash, status_message, status_sz);
}

void RosAPI::SetEqualityConstraintParametersArrayCB(
    const mujoco_ros_msgs::srv::SetEqualityConstraintParameters::Request::SharedPtr req,
    mujoco_ros_msgs::srv::SetEqualityConstraintParameters::Response::SharedPtr res)
{
	res->success = true;

	bool failed_any    = false;
	bool succeeded_any = false;
	char error_msg[MujocoEnv::kErrorLength];
	std::string status_message;

	for (const auto &parameters : req->parameters) {
		status_message[0] = '\0';
		bool success  = SetEqualityConstraintParameters(parameters, req->admin_hash, error_msg, MujocoEnv::kErrorLength);
		failed_any    = (failed_any || !success);
		succeeded_any = (succeeded_any || success);
		if (!success) {
			status_message += std::string(error_msg) + '\n';
		}
	}

	if (succeeded_any && failed_any) {
		status_message += "Not all constraints could be set";
		res->status_message = status_message;
		res->success        = false;
	} else if (failed_any) {
		status_message += "Could not set any constraints";
		res->status_message = status_message;
		res->success        = false;
	}
}

bool RosAPI::GetEqualityConstraintParameters(mujoco_ros_msgs::msg::EqualityConstraintParameters &parameters,
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

void RosAPI::GetEqualityConstraintParametersArrayCB(
    const mujoco_ros_msgs::srv::GetEqualityConstraintParameters::Request::SharedPtr req,
    mujoco_ros_msgs::srv::GetEqualityConstraintParameters::Response::SharedPtr res)
{
	res->success = true;

	bool failed_any    = false;
	bool succeeded_any = false;
	char error_msg[MujocoEnv::kErrorLength];
	std::string status_message;
	for (const auto &name : req->names) {
		status_message[0] = '\0';
		mujoco_ros_msgs::msg::EqualityConstraintParameters eqc;
		eqc.name     = name;
		bool success = GetEqualityConstraintParameters(eqc, req->admin_hash, error_msg, MujocoEnv::kErrorLength);

		failed_any    = (failed_any || !success);
		succeeded_any = (succeeded_any || success);
		if (success) {
			res->parameters.emplace_back(eqc);
		} else {
			status_message += std::string(error_msg) + '\n';
		}
	}

	if (succeeded_any && failed_any) {
		status_message += "Result: Not all constraints could be fetched";
		res->status_message = status_message;
		res->success        = false;
	} else if (failed_any) {
		status_message += "Result: Could not fetch any constraints";
		res->status_message = status_message;
		res->success        = false;
	}
}

void RosAPI::GetStateUintCB(const mujoco_ros_msgs::srv::GetStateUint::Request::SharedPtr /*req*/,
                            mujoco_ros_msgs::srv::GetStateUint::Response::SharedPtr res)
{
	int status;
	env_ptr_->GetSimulationStatus(status, res->state.description);
	res->state.value = static_cast<decltype(res->state.value)>(status);
}

void RosAPI::GetSimInfoCB(const mujoco_ros_msgs::srv::GetSimInfo::Request::SharedPtr /*req*/,
                          mujoco_ros_msgs::srv::GetSimInfo::Response::SharedPtr res)
{
	bool valid, paused;
	int load_count, loading_state, pending_sim_steps;
	env_ptr_->GetSimInfo(res->state.model_path, valid, load_count, loading_state, res->state.loading_state.description,
	                     paused, pending_sim_steps, res->state.rt_measured, res->state.rt_setting);
	res->state.model_valid         = valid;
	res->state.load_count          = static_cast<decltype(res->state.load_count)>(load_count);
	res->state.loading_state.value = static_cast<decltype(res->state.loading_state.value)>(loading_state);
	res->state.paused              = paused;
	res->state.pending_sim_steps   = static_cast<decltype(res->state.pending_sim_steps)>(pending_sim_steps);
}

void RosAPI::SetRTFactorCB(const mujoco_ros_msgs::srv::SetFloat::Request::SharedPtr req,
                           mujoco_ros_msgs::srv::SetFloat::Response::SharedPtr res)
{
	res->success = env_ptr_->SetRealTimeFactor(static_cast<float>(req->value), req->admin_hash);
}

void RosAPI::GetPluginStatsCB(const mujoco_ros_msgs::srv::GetPluginStats::Request::SharedPtr /*req*/,
                              mujoco_ros_msgs::srv::GetPluginStats::Response::SharedPtr res)
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
		mujoco_ros_msgs::msg::PluginStats stats;
		// plugin.name = plugin_names[i]; // TODO: add plugin name to message
		stats.plugin_type             = types[i];
		stats.load_time               = load_times[i];
		stats.reset_time              = reset_times[i];
		stats.ema_steptime_control    = ema_steptimes_control[i];
		stats.ema_steptime_passive    = ema_steptimes_passive[i];
		stats.ema_steptime_render     = ema_steptimes_render[i];
		stats.ema_steptime_last_stage = ema_steptimes_last_stage[i];
		res->stats.emplace_back(stats);
	}
}

void RosAPI::LoadInitialJointStatesCB(const std_srvs::srv::Empty::Request::SharedPtr /*req*/,
                                      std_srvs::srv::Empty::Response::SharedPtr /*res*/)
{
	env_ptr_->LoadInitialJointStates();
}

void RosAPI::SetupClockPublisher()
{
	if (env_ptr_->settings_.use_sim_time) {
		MJR_DEBUG("Setting up clock publisher");
		rclcpp::QoS clock_qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
		clock_pub_            = env_ptr_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", clock_qos);
		PublishSimTime(mjtNum(0));
	}
}

void RosAPI::PublishSimTime(mjtNum time)
{
	if (!env_ptr_->settings_.use_sim_time) {
		return;
	}
	rosgraph_msgs::msg::Clock::UniquePtr ros_time = std::make_unique<rosgraph_msgs::msg::Clock>();
	ros_time->clock = rclcpp::Time(static_cast<uint64_t>(time * 1e9)); // convert to nanoseconds
	clock_pub_->publish(std::move(ros_time));
}

} // namespace mujoco_ros
