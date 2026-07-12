/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
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
#include <mujoco_ros/render_backend.hpp>
#include <mujoco_ros/logging.hpp>
#include <mujoco_ros/util.hpp>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/ros_two/plugin_utils.hpp>

#include <mujoco/mujoco.h>
#include <mujoco_ros/array_safety.h>

#include <rosgraph_msgs/msg/clock.hpp>

namespace mju = ::mujoco::sample_util;

namespace mujoco_ros {

void MujocoEnv::FetchRosConfiguration()
{
	RCLCPP_DEBUG(this->get_logger(), "Fetching configuration");

	rclcpp::Parameter eval_mode_param = this->get_parameter("eval_mode");
	settings_.eval_mode               = eval_mode_param.as_bool();

	rclcpp::Parameter use_sim_time_param;
	if (!this->get_parameter("use_sim_time", use_sim_time_param)) {
		RCLCPP_FATAL(this->get_logger(),
		             "/use_sim_time ROS param is unset. This node requires you to explicitly set it to true "
		             "or false. Also Make sure it is set before starting any node, "
		             "otherwise nodes might behave unexpectedly.");
		throw std::runtime_error("/use_sim_time ROS param is unset.");
	}

	rclcpp::Parameter no_render_param = this->get_parameter("no_render");

	if (no_render_param.as_bool()) {
		RCLCPP_INFO(this->get_logger(), "no_render is set. Disabling rendering and setting headless to true");
		this->set_parameters_atomically(
		    { rclcpp::Parameter("headless", true), rclcpp::Parameter("render_offscreen", false) });
	}

	rclcpp::Parameter render_offscreen_param = this->get_parameter("render_offscreen");
	rclcpp::Parameter headless_param         = this->get_parameter("headless");
	rclcpp::Parameter unpause_param          = this->get_parameter("unpause");
	rclcpp::Parameter num_steps_param        = this->get_parameter("num_steps");
	rclcpp::Parameter num_mj_threads_param   = this->get_parameter("num_mj_threads");

	settings_.render_offscreen = render_offscreen_param.as_bool();
	settings_.headless         = headless_param.as_bool();
	ApplyPauseState(!unpause_param.as_bool(), false);
	num_steps_until_exit_    = num_steps_param.as_int();
	settings_.num_mj_threads = num_mj_threads_param.as_int();

	std::string filename = "";

	this->get_parameter("modelfile", filename);

	rclcpp::Parameter wait_for_xml_param = this->get_parameter("wait_for_xml");
	if (wait_for_xml_param.as_bool()) {
		RCLCPP_INFO(this->get_logger(), "Waiting for mujoco_xml content parameter...");
		bool wait_for_xml = true;
		std::string xml_content;
		rclcpp::Parameter xml_content_param;
		while (wait_for_xml) {
			if (this->get_parameter("mujoco_xml", xml_content_param)) {
				RCLCPP_INFO(this->get_logger(), "Got xml content from ros parameter");
				if (!xml_content_param.get_value<std::string>().empty()) {
					filename = "rosparam_content";
				} else {
					RCLCPP_WARN(this->get_logger(), "Empty xml content received from ros parameter");
				}
				wait_for_xml = false;
			}
		}
	}

	if (!filename.empty()) {
		RCLCPP_INFO_STREAM(this->get_logger(), "Using modelfile " << filename);
		mju::strcpy_arr(queued_filename_, filename.c_str());
		RequestLoad(2);
	} else {
		RCLCPP_WARN(this->get_logger(), "No modelfile was provided, launching empty simulation!");
	}
}

void MujocoEnv::InitTFBroadcasting()
{
	static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
	tf_buffer_          = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_buffer_->setUsingDedicatedThread(true);
	tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

void MujocoEnv::GetCameraConfiguration(const std::string &cam_name, rendering::StreamType &stream_type,
                                       float &pub_frequency, bool &use_segid, int &width, int &height,
                                       std::string &base_topic, std::string &rgb_topic, std::string &depth_topic,
                                       std::string &segment_topic)
{
	int stream_type_int;
	stream_type_int = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".stream_type",
	                                             static_cast<int>(rendering::kDEFAULT_CAM_STREAM_TYPE));
	stream_type     = rendering::StreamType(stream_type_int);
	pub_frequency =
	    get_maybe_undeclared_param(this, "cam_config." + cam_name + ".frequency", rendering::kDEFAULT_CAM_PUB_FREQ);
	use_segid =
	    get_maybe_undeclared_param(this, "cam_config." + cam_name + ".use_segid", rendering::kDEFAULT_CAM_USE_SEGID);
	width       = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".width", rendering::kDEFAULT_CAM_WIDTH);
	height      = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".height", rendering::kDEFAULT_CAM_HEIGHT);
	base_topic  = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".topic", "cameras/" + cam_name);
	rgb_topic   = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".name_rgb",
	                                         std::string(rendering::kDEFAULT_CAM_RGB_TOPIC));
	depth_topic = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".name_depth",
	                                         std::string(rendering::kDEFAULT_CAM_DEPTH_TOPIC));
	segment_topic = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".name_segment",
	                                           std::string(rendering::kDEFAULT_CAM_SEGMENT_TOPIC));
}

void MujocoEnv::GetInitialJointPositions(std::map<std::string, std::vector<double>> &joint_pos_map)
{
	std::string param_name = "initial_joint_states";
	auto result            = this->list_parameters({ param_name }, 2);

	if (result.names.empty()) {
		RCLCPP_WARN(this->get_logger(),
		            "No initial joint position specified (failed to get 'initial_joint_states' parameter).");
		return;
	}
	std::vector<std::string> joint_names;

	for (const auto &joint_name : result.names) {
		if (joint_name.rfind(param_name + ".", 0) == 0) {
			joint_names.push_back(joint_name);
		}
	}

	auto parameters = this->get_parameters(joint_names);

	for (const auto &joint : parameters) {
		std::string joint_name = joint.get_name().substr(param_name.length() + 1); // Strip "initial_joint_states."
		if (joint.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
			joint_pos_map[joint_name] = joint.as_double_array();
			RCLCPP_INFO_STREAM(this->get_logger(), "Joint " << joint_name << " has initial values: ["
			                                                << mujoco_ros::util::vector_to_string(joint.as_double_array())
			                                                << "]");
		} else {
			RCLCPP_WARN(this->get_logger(), "Joint %s is not a double array", joint.get_name().c_str());
		}
	}
}

void MujocoEnv::GetInitialJointVelocities(std::map<std::string, std::vector<double>> &joint_vel_map)
{
	std::string param_name = "initial_joint_velocities";
	auto result            = this->list_parameters({ param_name }, 2);

	if (result.names.empty()) {
		RCLCPP_WARN(this->get_logger(),
		            "No initial joint velocity specified (failed to get 'initial_joint_velocities' parameter).");
		return;
	}
	std::vector<std::string> joint_names;

	for (const auto &joint_name : result.names) {
		if (joint_name.rfind(param_name + ".", 0) == 0) {
			joint_names.push_back(joint_name);
		}
	}

	auto parameters = this->get_parameters(joint_names);

	for (const auto &joint : parameters) {
		std::string joint_name = joint.get_name().substr(param_name.length() + 1); // Strip "initial_joint_velocities."
		if (joint.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
			joint_vel_map[joint_name] = joint.as_double_array();
			RCLCPP_INFO_STREAM(this->get_logger(), "Joint " << joint_name << " has initial velocity values: ["
			                                                << mujoco_ros::util::vector_to_string(joint.as_double_array())
			                                                << "]");
		} else {
			RCLCPP_WARN(this->get_logger(), "Joint %s is not a double array", joint.get_name().c_str());
		}
	}
}

// TODO:
// setupServices
// callbacks

} // namespace mujoco_ros
