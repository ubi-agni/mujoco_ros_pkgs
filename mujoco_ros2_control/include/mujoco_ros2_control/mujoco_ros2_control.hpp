#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_

#include <thread>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "mujoco_ros/ros_two/plugin_utils.hpp"
#include "mujoco_ros/common_types.hpp"
#include "mujoco_ros2_control/visibility_control.h"
#include "mujoco_ros2_control/mujoco_ros2_control_system_interface.hpp"

namespace mujoco_ros2_control {
/**
 * @def MujocoRos2ControlPluginPrivate
 * @brief Data structure class for storing ros2, ros2_control objects. Taken from ign_ros2_control_plugin
 */
class MujocoRos2ControlPluginPrivate
{
public:
	/// \brief Get the URDF XML from the parameter server
	std::string getURDF() const;

	/// \brief Node Handles
	std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_{ nullptr };

	/// \brief Thread where the executor will spin
	std::thread thread_executor_spin_;

	/// \brief Executor to spin the controller
	rclcpp::Executor::SharedPtr executor_;

	/// \brief Timing
	rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);

	/// \brief Controller manager
	std::shared_ptr<controller_manager::ControllerManager> controller_manager_{ nullptr };

	/// \brief String with the robot description param_name
	std::string robot_description_ = "robot_description";

	/// \brief String with the name of the node that contains the robot_description
	std::string robot_description_node_ = "robot_state_publisher";

	/// \brief Last time the update method was called
	rclcpp::Time last_update_sim_time_mj_ = rclcpp::Time((int64_t)0, RCL_STEADY_TIME);

	/// \brief controller update rate
	unsigned int update_rate;

	/// \brief Interface loader
	std::shared_ptr<pluginlib::ClassLoader<mujoco_ros2_control::MujocoRos2SystemInterface>>
		robot_hw_sim_loader_{nullptr};
};

class MujocoRos2ControlPlugin : public mujoco_ros::MujocoPlugin
{
public:
	// MujocoRos2ControlPlugin();

	~MujocoRos2ControlPlugin() override;
	mujoco_ros::CallbackReturn on_configure(const rclcpp_lifecycle::State &/*previous_state*/) override;
	
	void ControlCallback(const mjModel* model, mjData* data) override;
	void PassiveCallback(const mjModel* model, mjData* data) override;
	void RenderCallback(const mjModel* model, mjData* data, mjvScene* scene) override;
	void LastStageCallback(const mjModel* model, mjData* data) override;
	void OnGeomChanged(const mjModel* model, mjData* data, const int geom_id) override;
	

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
	std::unique_ptr<MujocoRos2ControlPluginPrivate> dataPtr_;
	rclcpp::Logger get_my_logger() { return rclcpp::get_logger("MujocoRos2ControlPlugin"); };
	std::string MUJOCO_PLUGIN_PARAM_NAME = "MujocoPlugins";
	std::string plugin_name = "mujoco_ros2_control";
};

} // namespace mujoco_ros2

#endif // MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_
