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

/* Authors: David P. Leins */

#pragma once

#include <thread>
#include <ros/ros.h>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/plugin_utils.h>

namespace MujocoSim {

namespace environments {

static std::map<mjData *, MujocoEnvPtr> env_map_;

/**
 * @brief Adds a new entry to the env_map_, assigning a MujocoEnvPtr to a unique mjData pointer.
 *
 * @param[in] data mjData pointer used as key element in the map.
 * @param[in] env MujocoEnvPtr assigned to the mjData pointer.
 */
void assignData(mjData *data, MujocoEnvPtr env);

/**
 * @brief Retrieve the MujocoEnvPtr assigned to the given mjData pointer from the env_map_.
 *
 * @param[in] data mjData pointer used as key.
 * @return MujocoEnvPtr if `data` is in the map, otherwise nullptr is returned.
 */
MujocoEnvPtr getEnv(mjData *data);

/**
 * @brief Retreive a MujocoEnvPtr by id.
 *
 * @param[in] id id to search for.
 * @return MujocoEnvPtr if an env with that id exists, otherwise nullptr is returned.
 */
MujocoEnvPtr getEnvById(uint id);

/**
 * @brief Unregisters an mjData/MujocoEnvPtr entry in the static map resolved by mjData pointer.
 * This is necessary to deallocate the environment and its plugins, once it is not needed anymore, to greacefully shut
 * down.
 *
 * @param data mjData pointer which is used as key in the map to retrieve the correct MujocoEnvPtr.
 */
void unregisterEnv(mjData *data);

/**
 * @brief Unregisters an mjData/MujocoEnvPtr entry in the static map resolved by environment id.
 * This is necessary to deallocate the environment and its plugins, once it is not needed anymore, to greacefully shut
 * down.
 *
 * @param id id of the MujocoEnv that should be unregistered.
 */
void unregisterEnv(uint id);

} // end namespace environments

struct MujocoEnv
{
public:
	/**
	 * @brief Construct a new Mujoco Env object in a given namespace.
	 *
	 * @param[in] name namespace of the environment.
	 */
	MujocoEnv(std::string name) : name(name)
	{
		nh.reset(new ros::NodeHandle(name));
		ROS_DEBUG_STREAM_NAMED("mujoco_env", "New env created with namespace: " << name);
	};

	~MujocoEnv();

	MujocoEnv(const MujocoEnv &) = delete;

	void initializeRenderResources(void);

	/// Pointer to mjModel
	mjModelPtr model;
	/// Pointer to mjData
	mjDataPtr data;
	/// Noise to apply to control signal
	mjtNum *ctrlnoise = nullptr;
	/// Pointer to ros nodehandle in env namespace
	ros::NodeHandlePtr nh;
	/// Env namespace
	std::string name;
	/// struct holding objects needed for rendering
	render_utils::VisualStruct vis;
	/// CamStream objects for other cameras
	std::vector<render_utils::CamStreamPtr> cam_streams;

	/**
	 * @brief Calls reload functions of all members depeding on mjData.
	 * This function is called when a new mjData object is assigned to the environment.
	 */
	void reload();

	/**
	 * @brief Calls reset functions of all members depending on mjData.
	 * This function is called on a reset request by the user. mjModel and mjData are not reinitialized.
	 */
	void reset();

	const std::vector<MujocoPluginPtr> getPlugins();

	void runControlCbs();
	void runPassiveCbs();
	void runRenderCbs();
	void runLastStageCbs();

	void notifyGeomChanged(const int geom_id);

protected:
	XmlRpc::XmlRpcValue rpc_plugin_config;
	std::vector<MujocoPluginPtr> plugins;

private:
	std::vector<MujocoPluginPtr> cb_ready_plugins;
};

struct MujocoEnvParallel : MujocoEnv
{
public:
	/**
	 * @brief Construct a new Mujoco Env object
	 *
	 * @param[in] ros_ns namespace of the environment used in ros.
	 * @param[in] launchfile (optional) launchfile that will be started to bootstrap a ros environment for the
	 * namespace.
	 * @param[in] launch_args (optional) arguments to start the supplied launchfile with.
	 */
	MujocoEnvParallel(const std::string &ros_ns, const std::string &launchfile = "",
	                  const std::vector<std::string> &launch_args = std::vector<std::string>());
	MujocoEnvParallel(const MujocoEnvParallel &) = delete;
	~MujocoEnvParallel();

	// Env loop stepping thread
	std::thread *loop_thread = nullptr;
	// Bool to notify loop thread it should be stopped.
	std::atomic_bool stop_loop;

	std::string launchfile;
	std::vector<std::string> launch_args;

	/**
	 * @brief Runs the launchfile with the supplied list of arguments, if any was given.
	 */
	void bootstrapNamespace();
};

} // end namespace MujocoSim
