/*********************************************************************
 * Software License Agreement (BSD License)
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
#include <mujoco_ros/mujoco_sim.h>

namespace MujocoSim::plugin_utils {

bool parsePlugins(const ros::NodeHandle &nh)
{
	if (!nh.hasParam(MUJOCO_PLUGIN_PARAM_PATH)) {
		ROS_DEBUG_NAMED("mujoco_ros_pluginloader", "No plugins to load listed in parameter server!");
		return false;
	}

	ROS_DEBUG_NAMED("mujoco_ros_plugin_loader", "Initializing plugin loader ... ");

	plugin_loader_ptr_.reset(new pluginlib::ClassLoader<MujocoPlugin>("mujoco_ros", "MujocoSim::MujocoPlugin"));

	std::vector<std::string> plugin_list;
	XmlRpc::XmlRpcValue plugin_list_rpc;
	nh.getParam(MUJOCO_PLUGIN_PARAM_PATH, plugin_list_rpc);

	if (plugin_list_rpc.getType() != XmlRpc::XmlRpcValue::TypeArray) {
		ROS_ERROR_STREAM_NAMED("mujoco_ros_plugin_loader", "Error while parsing MujocoPlugins rosparam: wrong type.");
		ROS_DEBUG_STREAM_NAMED("mujoco_ros_plugin_loader", "MujocoPlugins rosparam should be of type '"
		                                                       << XmlRpc::XmlRpcValue::TypeArray << "', but got type '"
		                                                       << plugin_list_rpc.getType() << "' (yaml array)!");
		return false;
	}

	for (uint i = 0; i < plugin_list_rpc.size(); i++) {
		if (plugin_list_rpc[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
			ROS_ERROR_STREAM_NAMED("mujoco_ros_plugin_loader", "Error while parsing MujocoPlugins rosparam: wrong type.");
			ROS_DEBUG_STREAM_NAMED("mujoco_ros_plugin_loader", "Children of 'MujocoPlugins' should be of type '"
			                                                       << XmlRpc::XmlRpcValue::TypeStruct
			                                                       << "', but got type '" << plugin_list_rpc.getType()
			                                                       << "'. Skipping " << plugin_list_rpc[i]);
			continue;
		}
		registerPlugin(nh, plugin_list_rpc[i]);
	}

	return true;
}

bool registerPlugin(const ros::NodeHandle &nh, const XmlRpc::XmlRpcValue &config)
{
	ROS_ASSERT(config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	std::string type;

	if (!config.hasMember("type")) {
		ROS_ERROR_NAMED("mujoco_ros_plugin_loader", "Error while parsing MujocoPlugins rosparam: Every listed plugin "
		                                            "should provide a 'type' member!");
		return false;
	}
	type = (std::string)config["type"];

	ROS_DEBUG_STREAM_NAMED("mujoco_ros_plugin_loader", "Loading plugin of type " << type);

	try {
		MujocoPluginPtr mjplugin_ptr = plugin_loader_ptr_->createInstance(type);
		mjplugin_ptr->init(config);
		mujoco_plugins_.push_back(mjplugin_ptr);
		ROS_DEBUG_STREAM_NAMED("mujoco_ros_plugin_loader",
		                       "Added " << type << " to the list of loaded plugins. List now contains "
		                                << mujoco_plugins_.size() << " plugin(s)");
	} catch (const pluginlib::PluginlibException &ex) {
		ROS_ERROR_NAMED("mujoco_ros_plugin_loader", "The plugin failed to load: %s", ex.what());
	}

	return true;
}

void loadRegisteredPlugins(mjModelPtr m, mjDataPtr d)
{
	if (!mujoco_plugins_.empty()) {
		for (uint i = 0; i < mujoco_plugins_.size(); i++) {
			mujoco_plugins_[i]->safe_load(m, d);
		}
	}
}

void resetRegisteredPlugins()
{
	if (!mujoco_plugins_.empty()) {
		for (uint i = 0; i < mujoco_plugins_.size(); i++) {
			mujoco_plugins_[i]->safe_reset();
		}
	}
}

void triggerUpdate()
{
	if (!mujoco_plugins_.empty()) {
		for (uint i = 0; i < mujoco_plugins_.size(); i++) {
			mujoco_plugins_[i]->safe_update();
		}
	}
}

void unloadRegisteredPlugins()
{
	mujoco_plugins_.clear();
	plugin_loader_ptr_.reset();
}

std::vector<MujocoPluginPtr> *getRegisteredPluginPtrs()
{
	return &mujoco_plugins_;
}

} // namespace MujocoSim::plugin_utils
