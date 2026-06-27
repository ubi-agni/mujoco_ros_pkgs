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

#include "test_plugin.hpp"

#include <algorithm>

#if MJR_ROS_VERSION == ROS_1
#include <pluginlib/class_list_macros.h>
#else // MJR_ROS_VERSION == ROS_2
#include <pluginlib/class_list_macros.hpp>
#endif

using namespace mujoco_ros;
namespace mujoco_ros {

bool TestPlugin::Load(const mjModel *m, mjData *d)
{
#if MJR_ROS_VERSION == ROS_1
	if (rosparam_config_.hasMember("example_param")) {
		got_config_param.store(true);
	}

	if (rosparam_config_.hasMember("nested_array_param_1")) {
		if (rosparam_config_["nested_array_param_1"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
			got_lvl1_nested_array.store(true);
			if (rosparam_config_["nested_array_param_1"][0].hasMember("nested_array_param_2")) {
				got_lvl2_nested_array.store(true);
			}
		}
	}

	if (rosparam_config_.hasMember("nested_struct_param_1")) {
		if (rosparam_config_["nested_struct_param_1"].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
			got_lvl1_nested_struct.store(true);
			if (rosparam_config_["nested_struct_param_1"].hasMember("nested_struct_param_2")) {
				got_lvl2_nested_struct.store(true);
			}
		}
	}

#else // MJR_ROS_VERSION == ROS_2
	const auto parameter_names = get_node()->list_parameters({}, 100).names;
	auto has_parameter_name    = [&parameter_names](const std::string &name) {
      return std::find(parameter_names.begin(), parameter_names.end(), name) != parameter_names.end();
	};
	auto has_parameter_prefix = [&parameter_names](const std::string &prefix) {
		const std::string prefix_with_separator = prefix + ".";
		return std::any_of(
		    parameter_names.begin(), parameter_names.end(),
		    [&prefix_with_separator](const std::string &name) { return name.rfind(prefix_with_separator, 0) == 0; });
	};
	auto has_nested_parameter = [&parameter_names](const std::string &prefix, const std::string &leaf_name) {
		const std::string prefix_with_separator = prefix + ".";
		return std::any_of(parameter_names.begin(), parameter_names.end(),
		                   [&prefix_with_separator, &leaf_name](const std::string &name) {
			                   return name.rfind(prefix_with_separator, 0) == 0 &&
			                          name.find(leaf_name) != std::string::npos;
		                   });
	};

	if (has_parameter_name("example_param")) {
		got_config_param.store(true);
	}

	if (has_parameter_prefix("nested_array_param_1")) {
		got_lvl1_nested_array.store(true);
		if (has_nested_parameter("nested_array_param_1", "nested_array_param_2")) {
			got_lvl2_nested_array.store(true);
		}
	}

	if (has_parameter_prefix("nested_struct_param_1")) {
		got_lvl1_nested_struct.store(true);
		if (has_nested_parameter("nested_struct_param_1", "nested_struct_param_2")) {
			got_lvl2_nested_struct.store(true);
		}
	}
#endif

	bool tmp_fail = false;

#if MJR_ROS_VERSION == ROS_1
	node_handle_.param<bool>("should_fail", tmp_fail, false);

#else // MJR_ROS_VERSION == ROS_2
	if (get_node()->has_parameter("should_fail")) {
		tmp_fail = get_node()->get_parameter("should_fail").as_bool();
	} else if (env_ptr_ != nullptr && env_ptr_->has_parameter("should_fail")) {
		tmp_fail = env_ptr_->get_parameter("should_fail").as_bool();
	}
#endif

	should_fail.store(tmp_fail);
	const bool loaded_ok = !tmp_fail;
	if (loaded_ok) {
		m_ = m;
		d_ = d;
	}

	return loaded_ok;
}

void TestPlugin::Reset()
{
	ran_reset.store(true);
}

void TestPlugin::ControlCallback(const mjModel * /*model*/, mjData * /*data*/)
{
	ran_control_cb.store(true);
}

void TestPlugin::PassiveCallback(const mjModel * /*model*/, mjData * /*data*/)
{
	ran_passive_cb.store(true);
}

void TestPlugin::RenderCallback(const mjModel * /*model*/, mjData * /*data*/, mjvScene * /*scene*/)
{
	ran_render_cb.store(true);
}

void TestPlugin::LastStageCallback(const mjModel * /*model*/, mjData * /*data*/)
{
	ran_last_cb.store(true);
}

void TestPlugin::OnGeomChanged(const mjModel * /*model*/, mjData * /*data*/, const int /*geom_id*/)
{
	ran_on_geom_changed_cb.store(true);
}
} // namespace mujoco_ros

PLUGINLIB_EXPORT_CLASS(mujoco_ros::TestPlugin, mujoco_ros::MujocoPlugin)
