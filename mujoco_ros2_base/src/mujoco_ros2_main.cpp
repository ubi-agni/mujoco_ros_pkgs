#include <mujoco_ros2_base/plugin_utils.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include "mujoco/mujoco.h"
#include <yaml-cpp/yaml.h>
#include <algorithm>

// Main mujoco ros2 node
using namespace std::chrono_literals;

namespace mujoco_ros2 {
class MujocoRos2Node : public rclcpp::Node
{
public:
	MujocoRos2Node() : Node("mujoco_ros2_node")
	{
		rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);

		// Declare the three parameters:
		// list of plugins,
		// path to the mujoco xml file,
		// and a config yaml file that contains information about the
		//  individual plugin's configs, if necessary
		this->declare_parameter("MujocoPlugins", rclcpp::PARAMETER_STRING_ARRAY);
		this->declare_parameter("MujocoXml", rclcpp::PARAMETER_STRING);
		this->declare_parameter("MujocoPluginConfigs", rclcpp::PARAMETER_STRING);

		this->get_parameter("MujocoPlugins", mj_ros2_plugins_);
		this->get_parameter("MujocoXml", mj_xml_file_);
		this->get_parameter("MujocoPluginConfigs", plugin_yaml_path_);

		if (mj_xml_file_ != "") {
			m_.reset(mj_loadXML(mj_xml_file_.c_str(), 0, mj_error_, 1000));
			if (!m_.get()) {
				RCLCPP_ERROR_STREAM(get_logger(), "Failed to load xml: " << mj_error_);
				throw std::exception();
			}
		}
		d_.reset(mj_makeData(m_.get()));

		// check if applying a force to the pendulum then reading actually steps the simulation properly
		d_->ctrl[0] = 0.1;
		mj_step(m_.get(), d_.get());
		// for(auto i = 0; i < 3000; i++){
		//     mj_step(m_.get(), d_.get());
		//     RCLCPP_INFO(get_logger(), "Pendulum jnt: %f", d_->qpos[0]);
		// }
		// seems to be working!

		// And from here, check for the presence of "mujoco_ros2_control", and load the plugin
		if (std::find(mj_ros2_plugins_.begin(), mj_ros2_plugins_.end(), std::string("MujocoRos2Control")) !=
		    mj_ros2_plugins_.end()) {
			RCLCPP_INFO(get_logger(), "Found MujocoRos2Control Plugin!");
			RCLCPP_INFO(get_logger(), "Loaded xml: %s", mj_xml_file_.c_str());

			// Unfortunately, can't figure out an easy way to use ROS2 param for getting nested parameters
			//    and pack it into a map. Need to manually get the path to a config file,
			//    and read it using yaml-cpp or a similar library.

			// Get the corresponding parameters for the plugin, if it exists
			YAML::Node mj_yaml_node = YAML::LoadFile(plugin_yaml_path_);
			mj_yaml_node            = mj_yaml_node["MujocoRos2Control"];
			if (!mj_yaml_node.size()) {
				RCLCPP_WARN(get_logger(), "The plugin seems to have no corresponding configuration.");
			}
			// mj_yaml_node = mj_yaml_node["MujocoRos2Control"];
			
			// auto configs = mj_yaml_node["MujocoRos2Control"].as<std::map<std::string,std::string>>();
			// RCLCPP_INFO(get_logger(), "Yaml: %s", configs["controller_manager_name"].c_str());
			plugin_loader_.reset(
			    new pluginlib::ClassLoader<mujoco_ros2::MujocoPlugin>("mujoco_ros2_base", "mujoco_ros2::MujocoPlugin"));
			std::shared_ptr<mujoco_ros2::MujocoPlugin> mjros2control =
			    plugin_loader_->createSharedInstance("mujoco_ros2_control::MujocoRos2ControlPlugin");
			int env_ptr = 99;
			mjros2control->init(env_ptr, mj_yaml_node);
			mjros2control->safe_load(m_.get(), d_.get());
			for(auto i = 0; i < 10; i++){
				mj_step(m_.get(), d_.get());
				d_->ctrl[0] = 0.1;
				mjros2control->wrappedControlCallback(m_.get(), d_.get());
				mjros2control->wrappedPassiveCallback(m_.get(), d_.get());
				// mjros2control->wrappedRenderCallback(m_.get(), d_.get(), scene);
				// mjros2control->wrappedLastStageCallback(m_.get(), d_.get());
				RCLCPP_INFO(get_logger(), "Pendulum jnt: %f", d_->qpos[0]);
			}
		}
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	std::vector<std::string> mj_ros2_plugins_;
	std::string plugin_yaml_path_;
	std::string mj_xml_file_;
	std::unique_ptr<pluginlib::ClassLoader<mujoco_ros2::MujocoPlugin>> plugin_loader_;
	std::shared_ptr<mjModel> m_;
	std::shared_ptr<mjData> d_;
	char mj_error_[1000] = { "" };
};
} // namespace mujoco_ros2

int main(int argc, char **argv)
{
	// To avoid unused parameter warnings
	(void)argc;
	(void)argv;

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<mujoco_ros2::MujocoRos2Node>());
	rclcpp::shutdown();
	return 0;
	//   pluginlib::ClassLoader<mujoco_ros2::MujocoPlugin> mj_loader("mujoco_plugin_base", "mujoco_ros2::MujocoPlugin");

	//   try
	//   {
	//     // Read a parameter called MujocoPlugins, which needs to be defined in the launch file

	//     // If defined, load up the mujoco_ros2_plugin

	//     // else, just exit
	//     std::shared_ptr<polygon_base::RegularPolygon> triangle =
	//     poly_loader.createSharedInstance("polygon_plugins::Triangle"); triangle->initialize(10.0);

	//     std::shared_ptr<polygon_base::RegularPolygon> square =
	//     poly_loader.createSharedInstance("polygon_plugins::Square"); square->initialize(10.0);

	//     printf("Triangle area: %.2f\n", triangle->area());
	//     printf("Square area: %.2f\n", square->area());
	//   }
	//   catch(pluginlib::PluginlibException& ex)
	//   {
	//     printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
	//   }

	return 0;
}
