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

/* Authors: David P. Leins*/

#include <mujoco_ros/ros_two/plugin_utils.hpp>

#include <mujoco_ros/logging.hpp>
#include <mujoco_ros/mujoco_env.hpp>

#include <utility>

namespace mujoco_ros::plugin_utils {

namespace {

std::vector<std::string> GetConfiguredPluginNames(MujocoEnv *env_ptr)
{
	const std::string names_param = MUJOCO_PLUGIN_PARAM_NAME + ".names";
	if (!env_ptr->has_parameter(names_param)) {
		return {};
	}
	return env_ptr->get_parameter(names_param).as_string_array();
}

std::string GetPluginType(MujocoEnv *env_ptr, const std::string &plugin_name)
{
	const std::string type_param = MUJOCO_PLUGIN_PARAM_NAME + "." + plugin_name + ".type";
	if (!env_ptr->has_parameter(type_param)) {
		return {};
	}
	return env_ptr->get_parameter(type_param).as_string();
}

} // namespace

namespace {

class FailedPluginAdapter final : public IPluginAdapter
{
public:
	FailedPluginAdapter(std::string name, std::string type, std::string error)
	    : name_(std::move(name)), type_(std::move(type)), error_(std::move(error))
	{
	}

	const std::string &Name() const override { return name_; }
	const std::string &Type() const override { return type_; }
	bool Load(const mjModel *, mjData *, std::string &error) override
	{
		error = error_;
		return false;
	}
	void Control(const mjModel *, mjData *) override {}
	void Passive(const mjModel *, mjData *) override {}
	void Render(const mjModel *, mjData *, mjvScene *) override {}
	void LastStage(const mjModel *, mjData *) override {}
	void Reset() override {}
	void GeometryChanged(const mjModel *, mjData *, int) override {}
	PluginStat Statistics() const override { return { name_, type_ }; }

private:
	std::string name_;
	std::string type_;
	std::string error_;
};

} // namespace

bool ParsePlugins(MujocoEnv *env_ptr, std::vector<std::string> &plugin_names)
{
	plugin_names = GetConfiguredPluginNames(env_ptr);
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

		std::string type = GetPluginType(env_ptr, plugin_name);

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

RosPluginAdapterFactory::~RosPluginAdapterFactory()
{
	UnloadPluginloader();
}

std::vector<std::unique_ptr<IPluginAdapter>> RosPluginAdapterFactory::CreateAdapters()
{
	std::vector<std::unique_ptr<IPluginAdapter>> adapters;
	std::vector<std::string> plugin_names;
	if (!ParsePlugins(env_, plugin_names)) {
		return adapters;
	}

	for (const auto &plugin_name : plugin_names) {
		const std::string type = GetPluginType(env_, plugin_name);
		if (type.empty()) {
			adapters.emplace_back(std::make_unique<FailedPluginAdapter>(
			    plugin_name, "", "ROS 2 plugin configuration entry is missing type"));
			continue;
		}

		try {
			auto plugin = MujocoPluginPtr(plugin_loader_ptr_->createUnmanagedInstance(type));
			plugin->Init(plugin_name, env_, type);
			env_->AddNodeToExecutor(plugin->get_node()->get_node_base_interface());
			plugin->get_node()->configure();
			adapters.emplace_back(std::make_unique<RosPluginAdapter>(std::move(plugin)));
		} catch (const pluginlib::PluginlibException &ex) {
			adapters.emplace_back(std::make_unique<FailedPluginAdapter>(plugin_name, type, ex.what()));
		}
	}
	return adapters;
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
