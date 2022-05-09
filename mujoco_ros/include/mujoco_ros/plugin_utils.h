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

#pragma once
#include <ros/ros.h>
#include <mujoco_ros/mujoco_sim.h>

#include <pluginlib/class_loader.h>

namespace MujocoSim {

class MujocoPlugin
{
public:
	virtual ~MujocoPlugin() {}

	// Called directly afte plugin creation
	void init(const XmlRpc::XmlRpcValue &config) { rosparam_config_ = config; };

	// Wrapper method that evaluates if loading the plugin is successful
	void safe_load(mjModelPtr m, mjDataPtr d)
	{
		loading_successful_ = load(m, d);
		if (!loading_successful_)
			ROS_WARN_STREAM_NAMED("mujoco_ros_plugin",
			                      "Plugin of type '"
			                          << rosparam_config_["type"] << "' and full config '" << rosparam_config_
			                          << "' failed to load. It will be ignored until the next load attempt.");
	}

	// Wrapper method that only calls update if loading the plugin was successful
	void safe_update()
	{
		if (loading_successful_)
			update();
	}

	// Wrapper method that only calls reset if loading the plugin was successful
	void safe_reset()
	{
		if (loading_successful_)
			reset();
	}

	// Called once the world is loaded
	virtual bool load(mjModelPtr m, mjDataPtr d) = 0;

	// Called on reset
	virtual void reset() = 0;

	// Called after every world update
	virtual void update() = 0;

private:
	bool loading_successful_ = false;

protected:
	MujocoPlugin() {}
	XmlRpc::XmlRpcValue rosparam_config_;
};
typedef boost::shared_ptr<MujocoPlugin> MujocoPluginPtr;

namespace plugin_utils {

/**
 * @brief Searches for plugins to load in the ros parameter server and tries to load them.
 */
bool parsePlugins(const ros::NodeHandle &nh);
/**
 * @brief Loads a MujocoPlugin via pluginlib and registers them for further usage.
 */
bool registerPlugin(const ros::NodeHandle &nh, const XmlRpc::XmlRpcValue &config);

/**
 * @brief (Re)set the registered plugins.
 */
void loadRegisteredPlugins(mjModelPtr m, mjDataPtr d);

/**
 * @brief Calls the reset function of each registered plugin.
 */
void resetRegisteredPlugins();

/**
 * @brief Calls the update function of each registered plugin.
 */
void triggerUpdate();

void unloadRegisteredPlugins();

/**
 * @brief Get the vector containing Ptrs to all registered plugin instances.
 */
std::vector<MujocoPluginPtr> *getRegisteredPluginPtrs();

static std::vector<MujocoPluginPtr> mujoco_plugins_;
static boost::shared_ptr<pluginlib::ClassLoader<MujocoPlugin>> plugin_loader_ptr_;

const static std::string MUJOCO_PLUGIN_PARAM_PATH = "/MujocoPlugins/";

} // end namespace plugin_utils
} // namespace MujocoSim
