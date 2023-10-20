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

/* Authors: David P. Leins */

#pragma once
#include <ros/ros.h>
#include <mujoco_ros/common_types.h>

#include <pluginlib/class_loader.h>

namespace mujoco_ros {

class MujocoPlugin
{
public:
	virtual ~MujocoPlugin() { ROS_DEBUG_STREAM("Deleted plugin of type " << rosparam_config_["type"]); }

	// Called directly after plugin creation
	void init(const XmlRpc::XmlRpcValue &config, ros::NodeHandlePtr nh, MujocoEnvPtr env_ptr)
	{
		rosparam_config_ = config;
		node_handle_     = std::move(nh);
		env_ptr_         = env_ptr;
		type_            = static_cast<std::string>(rosparam_config_["type"]);
	};

	std::string type_;

	/**
	 * @brief Wrapper method that evaluates if loading the plugin is successful
	 *
	 * @param[in] m
	 * @param[out] d
	 * @return true if plugin could be loaded without errors.
	 * @return false if errors occurred during loading.
	 */
	bool safe_load(const mjModel *m, mjData *d)
	{
		loading_successful_ = load(m, d);
		if (!loading_successful_)
			ROS_WARN_STREAM_NAMED("mujoco_ros_plugin",
			                      "Plugin of type '"
			                          << rosparam_config_["type"] << "' with full config '" << rosparam_config_
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
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 */
	virtual void controlCallback(const mjModel * /*model*/, mjData * /*data*/){};

	/**
	 * @brief Override this function to compute and apply custom passive (i.e. non-controlled) forces.
	 * This callback should add to the vector \c mjData.qfrc_passive instead of overwriting it, otherwise
	 * the standard passive forces will be lost.
	 *
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 */
	virtual void passiveCallback(const mjModel * /*model*/, mjData * /*data*/){};

	/**
	 * @brief Override this callback to add custom visualisations to the scene.
	 *
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 * @param[in] scene pointer to mjvScene.
	 */
	virtual void renderCallback(const mjModel * /*model*/, mjData * /*data*/, mjvScene * /*scene*/){};

	/**
	 * @brief Override this callback to add custom behavior at the end of a mujoco_ros simulation step.
	 * Note that unlike `controlCallback` and `passiveCallback` this callback will not be called when mujoco runs
	 * individual sub-steps.
	 *
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 */
	virtual void lastStageCallback(const mjModel * /*model*/, mjData * /*data*/){};

	/**
	 * @brief Override this callback to add custom behavior when a geom has been changed in the model.
	 *
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 * @param[in] geom_id id of the geom thas has been changed.
	 */
	virtual void onGeomChanged(const mjModel * /*model*/, mjData * /*data*/, const int /*geom_id*/){};

protected:
	/**
	 * @brief Called once the world is loaded.
	 *
	 * @param[in] m shared pointer to mujoco model.
	 * @param[in] d shared pointer to mujoco data.
	 * @return true on succesful load.
	 * @return false if load was not successful.
	 */
	virtual bool load(const mjModel *m, mjData *d) = 0;

	/**
	 * @brief Called on reset.
	 */
	virtual void reset() = 0;

private:
	bool loading_successful_ = false;

protected:
	MujocoPlugin() = default;
	XmlRpc::XmlRpcValue rosparam_config_;
	ros::NodeHandlePtr node_handle_;
	MujocoEnvPtr env_ptr_;
};

namespace plugin_utils {

/**
 * @brief Searches for plugins to load in the ros parameter server and stores a the configuration in \c
 * plugin_config_rpc.
 * @param[in] nh Pointer to nodehandle where to first look for the plugin config.
 * @param[inout] plugin_config_rpc If any configuration is found, it is stored in this variable.
 */
bool parsePlugins(const ros::NodeHandle *nh, XmlRpc::XmlRpcValue &plugin_config_rpc);

/**
 * @brief Calls registerPlugin for each plugin defined in \c config_rpc.
 *
 * @param[in] nh pointer to nodehandle in correct namespace.
 * @param[in] config_rpc config of at least one plugin to load.
 * @param[inout] plugins vector of plugins. If successfully initialized, the plugins are appended to the vector.
 */
void registerPlugins(ros::NodeHandlePtr &nh, XmlRpc::XmlRpcValue &config_rpc, std::vector<MujocoPluginPtr> &plugins,
                     MujocoEnv *env);

/**
 * @brief Loads a MujocoPlugin defined in \c config_rpc via pluginlib and registers it in the passed plugin vector for
 * further usage.
 *
 * @param[in] nh pointer to nodehandle in correct namespace.
 * @param[in] config_rpc config of the plugin to load.
 * @param[inout] plugins vector of plugins. If successfully initialized, the plugin is appended to the vector.
 * @return true if initializing the plugin was successful, false otherwise.
 */
bool registerPlugin(ros::NodeHandlePtr nh, XmlRpc::XmlRpcValue &config_rpc, std::vector<MujocoPluginPtr> &plugins,
                    MujocoEnv *env);

void unloadPluginloader();
void initPluginLoader();

static std::unique_ptr<pluginlib::ClassLoader<MujocoPlugin>> plugin_loader_ptr_;

/**
 * @brief Defines under which path the plugin configuration is stored in the ros parameter server.
 */
const static std::string MUJOCO_PLUGIN_PARAM_NAME = "MujocoPlugins";

} // end namespace plugin_utils
} // namespace mujoco_ros
