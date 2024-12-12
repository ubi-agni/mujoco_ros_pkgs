#include <mujoco_ros2_base/plugin_utils.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

// Main mujoco ros2 node
using namespace std::chrono_literals;

namespace mujoco_ros2 {
class MujocoRos2Node : public rclcpp::Node
{
public:
	MujocoRos2Node() : Node("mujoco_ros2_node")
	{
		// Declare parameters like so,
		this->declare_parameter("MujocoPlugins", rclcpp::PARAMETER_STRING_ARRAY);

		// timer_ = this->create_wall_timer(
		//   5000ms, std::bind(&MujocoRos2Node::timer_callback, this));

		// And read parameters like so, for parameters that were declared only by this node
		this->get_parameter("MujocoPlugins", mj_ros2_plugins_);

		for (uint i = 0; i < mj_ros2_plugins_.size(); i++) {
			RCLCPP_INFO(get_logger(), "Constructor - Plugin %d: %s", i + 1, mj_ros2_plugins_[i].c_str());
		}
		// And from here, check for the presence of "mujoco_ros2_control", and load the plugin
		if (std::find(mj_ros2_plugins_.begin(), mj_ros2_plugins_.end(), std::string("MujocoRos2Control")) !=
		    mj_ros2_plugins_.end()) {
			RCLCPP_INFO(get_logger(), "Found MujocoRos2Control Plugin!");

			plugin_loader_.reset(
			    new pluginlib::ClassLoader<mujoco_ros2::MujocoPlugin>("mujoco_ros2_base", "mujoco_ros2::MujocoPlugin"));
			std::shared_ptr<mujoco_ros2::MujocoPlugin> mjros2control =
			    plugin_loader_->createSharedInstance("mujoco_ros2::MujocoRos2ControlPlugin");
			int env_ptr = 99;
			int model   = 1;
			int data    = 2;
			int scene   = 3;
			mjros2control->init(env_ptr);
			mjros2control->Configure();
			mjros2control->wrappedControlCallback(model, data);
			mjros2control->wrappedPassiveCallback(model, data);
			mjros2control->wrappedRenderCallback(model, data, scene);
			mjros2control->wrappedLastStageCallback(model, data);
		}
	}

	// void timer_callback()
	// {
	//   this->get_parameter("MujocoPlugins", mj_ros2_plugins_);

	//   for(uint i = 0; i < mj_ros2_plugins_.size(); i++){
	//       RCLCPP_INFO(get_logger(), "Plugin %d: %s", i+1, mj_ros2_plugins_[i].c_str());
	//   }

	//   // std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
	//   // this->set_parameters(all_new_parameters);
	// }

private:
	rclcpp::TimerBase::SharedPtr timer_;
	std::vector<std::string> mj_ros2_plugins_;
	std::unique_ptr<pluginlib::ClassLoader<mujoco_ros2::MujocoPlugin>> plugin_loader_;
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
