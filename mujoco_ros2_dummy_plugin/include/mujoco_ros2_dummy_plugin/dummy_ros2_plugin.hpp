#ifndef MUJOCO_ROS2_DUMMY_PLUGIN__DUMMY_ROS2_PLUGIN_HPP_
#define MUJOCO_ROS2_DUMMY_PLUGIN__DUMMY_ROS2_PLUGIN_HPP_

#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
// #include <controller_manager/controller_manager.hpp>

// #include <hardware_interface/resource_manager.hpp>
// #include <hardware_interface/component_parser.hpp>
// #include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "mujoco_ros/ros_two/plugin_utils.hpp"
#include "mujoco_ros/common_types.hpp"
// #include "mujoco_ros2_control/visibility_control.h"
// #include "mujoco_ros2_control/mujoco_ros2_control_system_interface.hpp"

namespace mujoco_ros {
/**
 * @def MujocoRos2ControlPluginPrivate
 * @brief Data structure class for storing ros2, ros2_control objects. Taken from ign_ros2_control_plugin
 */

class DummyRos2Plugin : public mujoco_ros::MujocoPlugin
{
public:
	// MujocoRos2ControlPlugin();

	mujoco_ros::CallbackReturn on_configure(const rclcpp_lifecycle::State &/*previous_state*/) override;

	~DummyRos2Plugin() override;
	void ControlCallback(const mjModel* model, mjData* data) override;
	

    // These don't need to be implemented if they are not used
	// void PassiveCallback(const mjModel* model, mjData* data) override;
	// void RenderCallback(const mjModel* model, mjData* data, mjvScene* scene) override;
	// void LastStageCallback(const mjModel* model, mjData* data) override;
	// void OnGeomChanged(const mjModel* model, mjData* data, const int geom_id) override;

protected:
	/**
	 * @brief Called once the world is loaded. Similar to Configure() in ign_ros2_control_plugin.
	 *
	 * @param[in] m shared pointer to mujoco model.
	 * @param[in] d shared pointer to mujoco data.
	 * @return true on succesful load.
	 * @return false if load was not successful.
	 */
	bool Load(const mjModel *m, mjData *d) override;

	/**
	 * @brief Called on reset.
	 */
	void Reset() override;

private:
	// std::unique_ptr<MujocoRos2ControlPluginPrivate> dataPtr_;
	rclcpp::Logger get_my_logger() { return rclcpp::get_logger("DummyRos2Plugin"); };
	std::string plugin_name = "dummy_ros2_plugin";
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

	rclcpp::Node::SharedPtr child_node_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr child_publisher_;

	rclcpp::Node::SharedPtr parallel_node_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr parallel_publisher_;

    mjData* d_;
    size_t count_;
    const mjModel* m_;
};

} // namespace mujoco_ros

#endif // MUJOCO_ROS2_DUMMY_PLUGIN__DUMMY_ROS2_PLUGIN_HPP_
