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
