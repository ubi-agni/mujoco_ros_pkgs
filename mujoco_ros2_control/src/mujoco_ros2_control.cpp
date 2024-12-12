#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

namespace mujoco_ros2 {

// MujocoRos2ControlPlugin::MujocoRos2ControlPlugin()
// {

// }

// MujocoRos2ControlPlugin::~MujocoRos2ControlPlugin()
// {
// }

void MujocoRos2ControlPlugin::Configure()
{
	dataPtr_ = std::make_unique<MujocoRos2ControlPluginPrivate>();
	int a    = 10;
	int b    = 20;
	dataPtr_->set_data(a, b);
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

bool MujocoRos2ControlPlugin::load(int model, int data)
{
	RCLCPP_INFO_STREAM(get_my_logger(), "lastStageCallback w/ " << model << " " << data);
	return true;
}

void MujocoRos2ControlPlugin::reset()
{
	RCLCPP_INFO_STREAM(get_my_logger(), "reset");
}

} // namespace mujoco_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mujoco_ros2::MujocoRos2ControlPlugin, mujoco_ros2::MujocoPlugin)
