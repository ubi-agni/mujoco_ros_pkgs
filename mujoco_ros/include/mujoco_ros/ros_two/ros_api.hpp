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

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/logging.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <mujoco_ros/common_types.hpp>

// #include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/empty.hpp>

// #include <actionlib/server/simple_action_server.h>

#include <rosgraph_msgs/msg/clock.hpp>

#include <mujoco_ros_msgs/action/step.hpp>

#include <mujoco_ros_msgs/msg/equality_constraint_parameters.hpp>
#include <mujoco_ros_msgs/msg/plugin_stats.hpp>

#include <mujoco_ros_msgs/srv/set_pause.hpp>
#include <mujoco_ros_msgs/srv/reload.hpp>
#include <mujoco_ros_msgs/srv/set_body_state.hpp>
#include <mujoco_ros_msgs/srv/get_body_state.hpp>
#include <mujoco_ros_msgs/srv/set_geom_properties.hpp>
#include <mujoco_ros_msgs/srv/get_geom_properties.hpp>
#include <mujoco_ros_msgs/srv/get_equality_constraint_parameters.hpp>
#include <mujoco_ros_msgs/srv/set_equality_constraint_parameters.hpp>
#include <mujoco_ros_msgs/srv/set_gravity.hpp>
#include <mujoco_ros_msgs/srv/get_gravity.hpp>
#include <mujoco_ros_msgs/srv/get_state_uint.hpp>
#include <mujoco_ros_msgs/srv/get_sim_info.hpp>
#include <mujoco_ros_msgs/srv/set_float.hpp>
#include <mujoco_ros_msgs/srv/get_plugin_stats.hpp>

namespace mujoco_ros {

/**
 * Declares static ROS2 parameter and sets it to a given value if it was not already declared.
 * @param[in] node Pointer to the node to declare the parameter in.
 * @param[in] name Name of the parameter to declare.
 * @param[in] default_value Default value to initialize with.
 * @param[in] parameter_descriptor Optional parameter descriptor.
 */
template <typename NodeType>
void declare_parameter_if_not_declared(
    NodeType node, const std::string &name, const rclcpp::ParameterValue &default_value,
    const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor())
{
	if (!node->has_parameter(name)) {
		node->declare_parameter(name, default_value, parameter_descriptor);
	}
}

/**
 * Fetches a ROS2 parameter and sets it to a given value if it was not already declared.
 * @param[in] node Pointer to the node to fetch/declare the parameter from/in.
 * @param[in] name Name of the parameter.
 * @param[in] default_value Default value to maybe initialize with.
 */
template <typename T>
auto get_maybe_undeclared_param(rclcpp::Node *node, const std::string &param_name, const T default_val)
{
	if (!node->has_parameter(param_name)) {
		return node->declare_parameter<T>(param_name, default_val);
	}
	return node->get_parameter(param_name).get_value<T>();
}

class RosAPI
{
public:
	RosAPI(MujocoEnvPtr env_ptr);

	void SetupServices();
	void SetupClockPublisher();
	void PublishSimTime(mjtNum sim_time);

private:
	MujocoEnvPtr env_ptr_;
	rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

	void SetPauseCB(const mujoco_ros_msgs::srv::SetPause::Request::SharedPtr req,
	                mujoco_ros_msgs::srv::SetPause::Response::SharedPtr res);
	void ShutdownCB(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res);
	void ReloadCB(const mujoco_ros_msgs::srv::Reload::Request::SharedPtr req,
	              mujoco_ros_msgs::srv::Reload::Response::SharedPtr res);
	void ResetCB(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res);
	void SetBodyStateCB(const mujoco_ros_msgs::srv::SetBodyState::Request::SharedPtr req,
	                    mujoco_ros_msgs::srv::SetBodyState::Response::SharedPtr res);
	void GetBodyStateCB(const mujoco_ros_msgs::srv::GetBodyState::Request::SharedPtr req,
	                    mujoco_ros_msgs::srv::GetBodyState::Response::SharedPtr res);
	void SetGeomPropertiesCB(const mujoco_ros_msgs::srv::SetGeomProperties::Request::SharedPtr req,
	                         mujoco_ros_msgs::srv::SetGeomProperties::Response::SharedPtr res);
	void GetGeomPropertiesCB(const mujoco_ros_msgs::srv::GetGeomProperties::Request::SharedPtr req,
	                         mujoco_ros_msgs::srv::GetGeomProperties::Response::SharedPtr res);
	void SetEqualityConstraintParametersArrayCB(
	    const mujoco_ros_msgs::srv::SetEqualityConstraintParameters::Request::SharedPtr req,
	    mujoco_ros_msgs::srv::SetEqualityConstraintParameters::Response::SharedPtr res);
	void GetEqualityConstraintParametersArrayCB(
	    const mujoco_ros_msgs::srv::GetEqualityConstraintParameters::Request::SharedPtr req,
	    mujoco_ros_msgs::srv::GetEqualityConstraintParameters::Response::SharedPtr res);
	void GetStateUintCB(const mujoco_ros_msgs::srv::GetStateUint::Request::SharedPtr req,
	                    mujoco_ros_msgs::srv::GetStateUint::Response::SharedPtr res);
	void GetSimInfoCB(const mujoco_ros_msgs::srv::GetSimInfo::Request::SharedPtr req,
	                  mujoco_ros_msgs::srv::GetSimInfo::Response::SharedPtr res);
	void SetRTFactorCB(const mujoco_ros_msgs::srv::SetFloat::Request::SharedPtr req,
	                   mujoco_ros_msgs::srv::SetFloat::Response::SharedPtr res);
	void GetPluginStatsCB(const mujoco_ros_msgs::srv::GetPluginStats::Request::SharedPtr req,
	                      mujoco_ros_msgs::srv::GetPluginStats::Response::SharedPtr res);
	void SetGravityCB(const mujoco_ros_msgs::srv::SetGravity::Request::SharedPtr req,
	                  mujoco_ros_msgs::srv::SetGravity::Response::SharedPtr res);
	void GetGravityCB(const mujoco_ros_msgs::srv::GetGravity::Request::SharedPtr req,
	                  mujoco_ros_msgs::srv::GetGravity::Response::SharedPtr res);
	void LoadInitialJointStatesCB(const std_srvs::srv::Empty::Request::SharedPtr req,
	                              std_srvs::srv::Empty::Response::SharedPtr res);

	bool SetEqualityConstraintParameters(const mujoco_ros_msgs::msg::EqualityConstraintParameters &parameters,
	                                     const std::string &admin_hash, char *error_msg = nullptr,
	                                     const int error_sz = 0);
	bool GetEqualityConstraintParameters(mujoco_ros_msgs::msg::EqualityConstraintParameters &parameters,
	                                     const std::string &admin_hash, char *error_msg = nullptr,
	                                     const int error_sz = 0);

	rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID & /*uuid*/,
	                                       std::shared_ptr<const mujoco_ros_msgs::action::Step::Goal> goal)
	{
		MJR_DEBUG_STREAM("Received step request for " << goal->num_steps << " steps");
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	};
	rclcpp_action::CancelResponse
	HandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mujoco_ros_msgs::action::Step>> /*goal_handle*/)
	{
		MJR_DEBUG_STREAM("Received step goal cancel request");
		return rclcpp_action::CancelResponse::ACCEPT;
	};
	// void HandleStepGoal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mujoco_ros_msgs::action::Step>>
	// goal_handle) {
	//     MJR_DEBUG("Spinning up new thread to handle step goal");
	//     std::thread{std::bind(&RosAPI::OnStepGoal, this, std::placeholders::_1), goal_handle}.detach();
	// }
	void OnStepGoal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mujoco_ros_msgs::action::Step>> goal_handle);

	// boost::recursive_mutex sim_params_mutex_;
	// dynamic_reconfigure::Server<mujoco_ros::SimParamsConfig> *param_server_;
	// mujoco_ros::SimParamsConfig sim_params_;
	// void dynparamCallback(mujoco_ros::SimParamsConfig &config, uint32_t level);
	// void updateDynamicParams();

	rclcpp_action::Server<mujoco_ros_msgs::action::Step>::SharedPtr action_step_;

	rclcpp::Service<mujoco_ros_msgs::srv::SetPause>::SharedPtr set_pause_srv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr shutdown_srv_;
	rclcpp::Service<mujoco_ros_msgs::srv::Reload>::SharedPtr reload_srv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
	rclcpp::Service<mujoco_ros_msgs::srv::SetBodyState>::SharedPtr set_body_state_srv_;
	rclcpp::Service<mujoco_ros_msgs::srv::GetBodyState>::SharedPtr get_body_state_srv_;
	rclcpp::Service<mujoco_ros_msgs::srv::SetGeomProperties>::SharedPtr set_geom_properties_srv_;
	rclcpp::Service<mujoco_ros_msgs::srv::GetGeomProperties>::SharedPtr get_geom_properties_srv_;
	rclcpp::Service<mujoco_ros_msgs::srv::GetEqualityConstraintParameters>::SharedPtr get_eq_constraint_parameters_srv_;
	rclcpp::Service<mujoco_ros_msgs::srv::SetEqualityConstraintParameters>::SharedPtr set_eq_constraint_parameters_srv_;
	rclcpp::Service<mujoco_ros_msgs::srv::GetStateUint>::SharedPtr get_state_uint_srv_;
	rclcpp::Service<mujoco_ros_msgs::srv::GetSimInfo>::SharedPtr get_sim_info_srv_;
	rclcpp::Service<mujoco_ros_msgs::srv::SetFloat>::SharedPtr set_rt_factor_srv_;
	rclcpp::Service<mujoco_ros_msgs::srv::GetPluginStats>::SharedPtr get_plugin_stats_srv_;
	rclcpp::Service<mujoco_ros_msgs::srv::SetGravity>::SharedPtr set_gravity_srv_;
	rclcpp::Service<mujoco_ros_msgs::srv::GetGravity>::SharedPtr get_gravity_srv_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr load_initial_joint_states_srv_;
};

} // namespace mujoco_ros
