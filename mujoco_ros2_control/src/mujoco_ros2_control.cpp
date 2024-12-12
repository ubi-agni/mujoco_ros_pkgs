#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

namespace mujoco_ros2 {

std::string MujocoRos2ControlPluginPrivate::getURDF() const
{
	std::string urdf_string;

	using namespace std::chrono_literals;
	auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(node_, robot_description_node_);
	while (!parameters_client->wait_for_service(0.5s)) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for %s service. Exiting.",
			             robot_description_node_.c_str());
			return 0;
		}
		RCLCPP_ERROR(node_->get_logger(), "%s service not available, waiting again...", robot_description_node_.c_str());
	}

	RCLCPP_INFO(node_->get_logger(), "connected to service!! %s asking for %s", robot_description_node_.c_str(),
	            this->robot_description_.c_str());

	// search and wait for robot_description on param server
	while (urdf_string.empty()) {
		RCLCPP_DEBUG(node_->get_logger(), "param_name %s", this->robot_description_.c_str());

		try {
			auto f = parameters_client->get_parameters({ this->robot_description_ });
			f.wait();
			std::vector<rclcpp::Parameter> values = f.get();
			urdf_string                           = values[0].as_string();
		} catch (const std::exception &e) {
			RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
		}

		if (!urdf_string.empty()) {
			break;
		} else {
			RCLCPP_ERROR(node_->get_logger(),
			             "ign_ros2_control plugin is waiting for model"
			             " URDF in parameter [%s] on the ROS param server.",
			             this->robot_description_.c_str());
		}
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}
	RCLCPP_INFO(node_->get_logger(), "Received URDF from param server");

	return urdf_string;
}

// MujocoRos2ControlPlugin::MujocoRos2ControlPlugin()
// {

// }

MujocoRos2ControlPlugin::~MujocoRos2ControlPlugin()
{
	// Stop controller manager thread
	if (!this->dataPtr_->controller_manager_) {
		return;
	}
	this->dataPtr_->executor_->remove_node(this->dataPtr_->controller_manager_);
	this->dataPtr_->executor_->cancel();
	this->dataPtr_->thread_executor_spin_.join();
}

void MujocoRos2ControlPlugin::controlCallback(int model, int data)
{
	RCLCPP_INFO_STREAM(get_my_logger(), "controlCallback w/ " << model << " " << data);
};
void MujocoRos2ControlPlugin::passiveCallback(int model, int data)
{
	RCLCPP_INFO_STREAM(get_my_logger(), "passiveCallback w/ " << model << " " << data);
};
void MujocoRos2ControlPlugin::renderCallback(int model, int data, int scene)
{
	RCLCPP_INFO_STREAM(get_my_logger(), "renderCallback w/ " << model << " " << data << " " << scene);
};
void MujocoRos2ControlPlugin::lastStageCallback(int model, int data)
{
	RCLCPP_INFO_STREAM(get_my_logger(), "lastStageCallback w/ " << model << " " << data);
};
void MujocoRos2ControlPlugin::onGeomChanged(int model, int data, const int geom_id)
{
	RCLCPP_INFO_STREAM(get_my_logger(), "onGeomChanged w/ " << model << " " << data << " " << geom_id);
};

bool MujocoRos2ControlPlugin::load(const mjModel *model, mjData *data)
{
	dataPtr_ = std::make_unique<MujocoRos2ControlPluginPrivate>();

	RCLCPP_INFO_STREAM(get_my_logger(), "loading with given model and data");
	// for (auto it = yaml_node_.begin(); it != yaml_node_.end(); ++it) {
	// 	YAML::Node key = it->first;
	// 	YAML::Node value = it->second;
	// 	if (key.Type() == YAML::NodeType::Scalar) {
	// 		// This should be true; do something here with the scalar key.
	// 		RCLCPP_INFO(get_my_logger(), "Scalar");
	// 	}
	// 	if (value.Type() == YAML::NodeType::Map) {
	// 		RCLCPP_INFO(get_my_logger(), "Map");
	// 		// This should be true; do something here with the map.
	// 	}
	// }

	// RCLCPP_INFO_STREAM(get_my_logger(), "controller_manager_name: " << yaml_node_["controller_manager_name"]);
	return true;
}

void MujocoRos2ControlPlugin::reset()
{
	RCLCPP_INFO_STREAM(get_my_logger(), "reset");
}

} // namespace mujoco_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mujoco_ros2::MujocoRos2ControlPlugin, mujoco_ros2::MujocoPlugin)
