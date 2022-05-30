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
#include <mujoco_ros/common_types.h>

#include <pluginlib/class_loader.h>

namespace MujocoSim {

class MujocoPlugin
{
public:
	virtual ~MujocoPlugin() {}

	// Called directly after plugin creation
	void init(const XmlRpc::XmlRpcValue &config, ros::NodeHandlePtr nh) { rosparam_config_ = config; };

	// Wrapper method that evaluates if loading the plugin is successful
	bool safe_load(mjModelPtr m, mjDataPtr d)
	{
		loading_successful_ = load(m, d);
		if (!loading_successful_)
			ROS_WARN_STREAM_NAMED("mujoco_ros_plugin",
			                      "Plugin of type '"
			                          << rosparam_config_["type"] << "' and full config '" << rosparam_config_
			                          << "' failed to load. It will be ignored until the next load attempt.");
		return loading_successful_;
	}

	/**
	 * @brief Wrapper method that only calls reset if loading the plugin was successful.
	 */
	void safe_reset()
	{
		if (loading_successful_)
			reset();
	}

	/**
	 * @brief Override this function to implement custom control laws.
	 * To apply control, write into \c mjData.ctrl, \c mjData.qfrc_applied and/or \c mjData.xfrc_applied.
	 * If defined, this function will be called by the mujoco step function at the appropriate time.
	 *
	 * @param model pointer to mjModel.
	 * @param data pointer to mjData.
	 */
	virtual void controlCallback(mjModelPtr model, mjDataPtr data){};

	/**
	 * @brief Override this function to compute and apply custom passive (i.e. non-controlled) forces.
	 * This callback should add to the vector \c mjData.qfrc_passive instead of overwriting it, otherwise
	 * the standard passive forces will be lost.
	 *
	 * @param model pointer to mjModel.
	 * @param data pointer to mjData.
	 */
	virtual void passiveCallback(mjModelPtr model, mjDataPtr data){};

	/**
	 * @brief Override this callback to add custum visualisations to the scene.
	 *
	 * @param model pointer to mjModel.
	 * @param data pointer to mjData.
	 */
	virtual void renderCallback(mjModelPtr model, mjDataPtr data, mjvScene *scene){};

protected:
	/**
	 * @brief Called once the world is loaded.
	 *
	 * @param m shared pointer to mujoco model.
	 * @param d shared pointer to mujoco data.
	 * @return true on succesful load.
	 * @return false if load was not successful.
	 */
	virtual bool load(mjModelPtr m, mjDataPtr d) = 0;

	/**
	 * @brief Called on reset.
	 */
	virtual void reset() = 0;

private:
	bool loading_successful_ = false;

protected:
	MujocoPlugin() {}
	XmlRpc::XmlRpcValue rosparam_config_;
};

namespace plugin_utils {

/**
 * @brief Searches for plugins to load in the ros parameter server and stores a the configuration in \c
 * plugin_config_rpc.
 */
bool parsePlugins(const ros::NodeHandle &nh, XmlRpc::XmlRpcValue &plugin_config_rpc);

/**
 * @brief Calls registerPlugin for each plugin defined in \c config_rpc.
 */
void registerPlugins(const ros::NodeHandle &nh, const XmlRpc::XmlRpcValue &config_rpc, MujocoEnvPtr &env);

/**
 * @brief Loads a MujocoPlugin defined in \c config_rpc via pluginlib and registers them in the passed MujocoEnv for
 * further usage.
 */
bool registerPlugin(const ros::NodeHandle &nh, const XmlRpc::XmlRpcValue &config_rpc, MujocoEnvPtr &env);

/**
 * @brief Empties the list of registered plugins. Since they are stored as shared pointers, the plugins will be
 * destroyed if no other object is referencing them, i.e. they aren't used by any other MujocoEnv.
 */
void unloadRegisteredPlugins();

static boost::shared_ptr<pluginlib::ClassLoader<MujocoPlugin>> plugin_loader_ptr_;

const static std::string MUJOCO_PLUGIN_PARAM_NAME = "MujocoPlugins";

} // end namespace plugin_utils
} // namespace MujocoSim
