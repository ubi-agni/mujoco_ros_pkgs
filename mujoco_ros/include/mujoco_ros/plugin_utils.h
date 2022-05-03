#pragma once
#include <mjmodel.h>
#include <mjdata.h>
#include <ros/ros.h>

#include <pluginlib/class_loader.h>

namespace MujocoSim {

class MujocoPlugin
{
public:
	virtual ~MujocoPlugin() {}

	// Called once the world is loaded
	virtual void load(mjModel m, mjData d) = 0;

	// Called on reset
	virtual void reset() = 0;

	// Called after every world update
	virtual void update() = 0;

protected:
	MujocoPlugin() {}
};
typedef boost::shared_ptr<MujocoPlugin> MujocoPluginPtr;

namespace plugin_util {

/**
 * @brief Searches for plugins to load in the ros parameter server and tries to load them.
 */
bool loadRegisteredPlugins(const ros::NodeHandle &nh);
/**
 * @brief Loads a MujocoPlugin via pluginlib.
 */
bool loadPlugin(const ros::NodeHandle &nh, const std::string &name);

static std::vector<MujocoPluginPtr> mujoco_plugins_;
static boost::shared_ptr<pluginlib::ClassLoader<MujocoPlugin>> plugin_loader_ptr_;

const static std::string MUJOCO_PLUGIN_PARAM_PATH = "/MujocoPlugins/";

} // end namespace plugin_util
} // namespace MujocoSim
