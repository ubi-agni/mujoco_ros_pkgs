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

/* Authors: David P. Leins*/

#include <mujoco_ros/ros_two/plugin_utils.hpp>

#include <mujoco_ros/logging.hpp>
#include <mujoco_ros/mujoco_env.hpp>

namespace mujoco_ros::plugin_utils {

bool ParsePlugins(MujocoEnv *env_ptr, std::vector<std::string> &plugin_names)
{
	plugin_names = env_ptr->get_parameter(MUJOCO_PLUGIN_PARAM_NAME + ".names").as_string_array();
	if (plugin_names.empty()) {
		MJR_INFO_NAMED("mujoco_ros_pluginloader", "No plugins to load listed in parameter server!");
		return false;
	}
	MJR_INFO_STREAM_NAMED("mujoco_ros_plugin_loader", "Found " << plugin_names.size() << " plugins to load.");
	MJR_DEBUG_NAMED("mujoco_ros_plugin_loader", "Initializing plugin loader ... ");

	return true;
}

void RegisterPlugins(const std::vector<std::string> &plugin_names, std::vector<MujocoPluginPtr> &plugins,
                     MujocoEnv *env_ptr)
{
	for (const auto &plugin_name : plugin_names) {
		MJR_DEBUG_STREAM_NAMED("mujoco_ros_plugin_loader", "Processing plugin with name " << plugin_name);

		MJR_DEBUG_STREAM_NAMED("mujoco_ros_plugin_loader", "Checking for 'type' member in plugin config at "
		                                                       << MUJOCO_PLUGIN_PARAM_NAME + "." + plugin_name + ".type");

		std::string type;
		if (env_ptr->has_parameter(MUJOCO_PLUGIN_PARAM_NAME + "." + plugin_name + ".type")) {
			type = env_ptr->get_parameter(MUJOCO_PLUGIN_PARAM_NAME + "." + plugin_name + ".type").as_string();
		}

		if (type.empty()) {
			MJR_ERROR_STREAM_NAMED("mujoco_ros_plugin_loader", "Error while parsing MujocoPlugins rosparam: Every listed "
			                                                   "plugin should provide a 'type' member!");
			continue;
		}

		MJR_DEBUG_STREAM_NAMED("mujoco_ros_plugin_loader", "Registering plugin of type " << type);

		try {
			MujocoPlugin *mjplugin_ptr = plugin_loader_ptr_->createUnmanagedInstance(type);
			mjplugin_ptr->Init(plugin_name, env_ptr, type);
			env_ptr->AddNodeToExecutor(mjplugin_ptr->get_node()->get_node_base_interface());
			mjplugin_ptr->get_node()->configure();
			plugins.emplace_back(std::unique_ptr<MujocoPlugin>(mjplugin_ptr));
			MJR_DEBUG_STREAM_NAMED("mujoco_ros_plugin_loader",
			                       "Added " << type << " to the list of loaded plugins in namespace '" << plugin_name
			                                << "'. List now contains " << plugins.size() << " plugin(s)");
		} catch (const pluginlib::PluginlibException &ex) {
			MJR_ERROR_STREAM_NAMED("mujoco_ros_plugin_loader",
			                       "Plugin " << plugin_name << " of type " << type << " failed to load: " << ex.what());
		}
	}
}

void InitPluginLoader()
{
	// NOLINTBEGIN(clang-analyzer-optin.cplusplus.VirtualCall)
	plugin_loader_ptr_ =
	    std::make_unique<pluginlib::ClassLoader<mujoco_ros::MujocoPlugin>>("mujoco_ros", "mujoco_ros::MujocoPlugin");
	// NOLINTEND(clang-analyzer-optin.cplusplus.VirtualCall)
}

void UnloadPluginloader()
{
	plugin_loader_ptr_.reset();
}

} // namespace mujoco_ros::plugin_utils
