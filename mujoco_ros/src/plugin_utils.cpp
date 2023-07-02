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

/* Authors: David P. Leins*/

#include <mujoco_ros/plugin_utils.h>

// clude <mujoco_ros/mujoco_env.h>

namespace mujoco_ros::plugin_utils {

bool parsePlugins(const ros::NodeHandlePtr nh, XmlRpc::XmlRpcValue &plugin_config_rpc)
{
	std::string param_path;
	if (nh->searchParam(MUJOCO_PLUGIN_PARAM_NAME, param_path) ||
	    ros::param::search(MUJOCO_PLUGIN_PARAM_NAME, param_path)) {
		ROS_DEBUG_STREAM_NAMED("mujoco_ros_pluginloader", "Found MujocoPlugins param under " << param_path);
	} else {
		ROS_INFO_NAMED("mujoco_ros_pluginloader", "No plugins to load listed in parameter server!");
		return false;
	}

	ROS_DEBUG_NAMED("mujoco_ros_plugin_loader", "Initializing plugin loader ... ");

	nh->getParam(param_path, plugin_config_rpc);

	if (plugin_config_rpc.getType() != XmlRpc::XmlRpcValue::TypeArray) {
		ROS_ERROR_STREAM_NAMED("mujoco_ros_plugin_loader", "Error while parsing MujocoPlugins rosparam: wrong type.");
		ROS_DEBUG_STREAM_NAMED("mujoco_ros_plugin_loader", "MujocoPlugins rosparam should be of type '"
		                                                       << XmlRpc::XmlRpcValue::TypeArray << "', but got type '"
		                                                       << plugin_config_rpc.getType() << "' (yaml array)!");
		return false;
	}
	return true;
}

void registerPlugins(ros::NodeHandlePtr nh, XmlRpc::XmlRpcValue &config_rpc, std::vector<MujocoPluginPtr> &plugins,
                     MujocoEnvPtr env)
{
	for (int8_t i = 0; i < config_rpc.size(); i++) {
		if (config_rpc[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
			ROS_ERROR_STREAM_NAMED("mujoco_ros_plugin_loader", "Error while parsing MujocoPlugins rosparam: wrong type.");
			ROS_DEBUG_STREAM_NAMED("mujoco_ros_plugin_loader", "Children of 'MujocoPlugins' should be of type '"
			                                                       << XmlRpc::XmlRpcValue::TypeStruct
			                                                       << "', but got type '" << config_rpc.getType()
			                                                       << "'. Skipping " << config_rpc[i]);
			continue;
		}
		// TODO: handle failed registration somehow?
		registerPlugin(nh, config_rpc[i], plugins, env);
	}
}

bool registerPlugin(ros::NodeHandlePtr nh, XmlRpc::XmlRpcValue &config, std::vector<MujocoPluginPtr> &plugins,
                    MujocoEnvPtr env)
{
	ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	std::string type;

	if (!config.hasMember("type")) {
		ROS_ERROR_NAMED("mujoco_ros_plugin_loader", "Error while parsing MujocoPlugins rosparam: Every listed plugin "
		                                            "should provide a 'type' member!");
		return false;
	}
	type = static_cast<std::string>(config["type"]);

	ROS_DEBUG_STREAM_NAMED("mujoco_ros_plugin_loader", "Registering plugin of type " << type);

	try {
		MujocoPluginPtr mjplugin_ptr = plugin_loader_ptr_->createInstance(type);
		mjplugin_ptr->init(config, nh, env);
		plugins.push_back(mjplugin_ptr);
		ROS_DEBUG_STREAM_NAMED("mujoco_ros_plugin_loader",
		                       "Added " << type << " to the list of loaded plugins in namespace '" << nh->getNamespace()
		                                << "'. List now contains " << plugins.size() << " plugin(s)");
	} catch (const pluginlib::PluginlibException &ex) {
		ROS_ERROR_STREAM_NAMED("mujoco_ros_plugin_loader",
		                       "The plugin failed to load (for namespace " << nh->getNamespace() << " ): " << ex.what());
		return false;
	}

	return true;
}

void initPluginLoader()
{
	plugin_loader_ptr_ =
	    std::make_unique<pluginlib::ClassLoader<mujoco_ros::MujocoPlugin>>("mujoco_ros", "mujoco_ros::MujocoPlugin");
}

void unloadPluginloader()
{
	plugin_loader_ptr_.reset();
}

} // namespace mujoco_ros::plugin_utils
