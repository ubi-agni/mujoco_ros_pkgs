#include <mujoco_ros/plugin_utils.h>

namespace MujocoSim::plugin_util {

bool loadRegisteredPlugins(const ros::NodeHandle &nh)
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

	ROS_ASSERT(plugin_list_rpc.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	for (const auto child : plugin_list_rpc) {
		ROS_DEBUG_STREAM("Requesting plugin: " << child.first);
		plugin_list.push_back(child.first);

		loadPlugin(nh, child.first);
	}

	return true;
}

bool loadPlugin(const ros::NodeHandle &nh, const std::string &name)
{
	std::string type, args_path;

	if (!nh.hasParam(MUJOCO_PLUGIN_PARAM_PATH + name + "/type")) {
		ROS_ERROR_STREAM_NAMED("mujoco_ros_plugin_loader",
		                       "Plugin '" << name << "' needs to define a type to be loaded!");
		return false;
	}

	nh.getParam(MUJOCO_PLUGIN_PARAM_PATH + name + "/type", type);
	nh.param<std::string>(MUJOCO_PLUGIN_PARAM_PATH + name + "/args", args_path, "");

	ROS_DEBUG_STREAM_NAMED("mujoco_ros_plugin_loader",
	                       "Loading plugin " << name << " of type " << type << " and args_path " << args_path);

	try {
		MujocoPluginPtr mjplugin_ptr = plugin_loader_ptr_->createInstance(type);
		mujoco_plugins_.push_back(mjplugin_ptr);
	} catch (const pluginlib::PluginlibException &ex) {
		ROS_ERROR_NAMED("mujoco_ros_plugin_loader", "The plugin failed to load: %s", ex.what());
	}

	return true;
}

} // namespace MujocoSim::plugin_util
