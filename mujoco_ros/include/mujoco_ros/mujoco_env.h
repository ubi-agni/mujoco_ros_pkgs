#pragma once

#include <ros/ros.h>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/plugin_utils.h>

namespace MujocoSim {

namespace environments {
static std::map<mjData *, MujocoEnvPtr> env_map_;
void assignData(mjData *data, MujocoEnvPtr env);
MujocoEnvPtr getEnv(mjData *data);
} // end namespace environments

struct MujocoEnv
{
public:
	MujocoEnv(std::string name) : name(name) { ROS_DEBUG_STREAM("created env with name: " << name); };

	MujocoEnv(const MujocoEnv &) = delete;

	mjModelPtr model;
	mjDataPtr data;
	mjtNum *ctrlnoise = nullptr;
	ros::NodeHandlePtr nh;

	std::string name;

	void resetPlugins();
	void loadPlugins();
	void addPlugin(MujocoPluginPtr plugin);
	const std::vector<MujocoPluginPtr> getPlugins();

	void runControlCbs();
	void runPassiveCbs();
	void runContactFilterCbs();
	void runRenderCbs(mjvScene *scene);

protected:
	XmlRpc::XmlRpcValue rpc_plugin_config;
	std::vector<MujocoPluginPtr> plugins;

private:
	std::vector<int> tests;
	std::vector<MujocoPluginPtr> cb_ready_plugins;
};

struct MujocoEnvParallel : MujocoEnv
{
public:
	/**
	 * @brief Construct a new Mujoco Env object
	 *
	 * @param ros_ns namespace of the environment used in ros.
	 * @param bootstrap_launchfile (optional) launchfile that will be started to bootstrap a ros environment for the
	 * namespace.
	 */
	MujocoEnvParallel(const std::string &ros_ns, const std::string &launchfile = "",
	                  const std::vector<std::string> &launch_args = std::vector<std::string>());
	MujocoEnvParallel(const MujocoEnvParallel &) = delete;
	~MujocoEnvParallel();

	std::string launchfile;
	std::vector<std::string> launch_args;

	/**
	 * @brief Runs the launchfile, if any was given.
	 */
	void bootstrapNamespace();
};

} // end namespace MujocoSim
