#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "mujoco_ros2_base/plugin_utils.h"
#include "mujoco_ros2_control/visibility_control.h"

namespace mujoco_ros2 {

class MujocoRos2ControlPluginPrivate
{
public:
	int data1;
	int data2;

	void set_data(int a, int b)
	{
		data1 = a;
		data2 = b;
	}
};

class MujocoRos2ControlPlugin : public MujocoPlugin
{
public:
	// MujocoRos2ControlPlugin();

	// virtual ~MujocoRos2ControlPlugin();
	void Configure() override;
	void controlCallback(int model, int data) override;
	void passiveCallback(int model, int data) override;
	void renderCallback(int model, int data, int scene) override;
	void lastStageCallback(int model, int data) override;
	void onGeomChanged(int model, int data, const int geom_id) override;

protected:
	/**
	 * @brief Called once the world is loaded.
	 *
	 * @param[in] m shared pointer to mujoco model.
	 * @param[in] d shared pointer to mujoco data.
	 * @return true on succesful load.
	 * @return false if load was not successful.
	 */
	bool load(int m, int d) override;

	/**
	 * @brief Called on reset.
	 */
	void reset() override;

private:
	std::unique_ptr<MujocoRos2ControlPluginPrivate> dataPtr_;
	rclcpp::Logger get_my_logger() { return rclcpp::get_logger("MujocoRos2ControlPlugin"); };
};

} // namespace mujoco_ros2

#endif // MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_
