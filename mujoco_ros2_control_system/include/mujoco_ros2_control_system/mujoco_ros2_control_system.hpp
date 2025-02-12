#ifndef MUJOCO_ROS2_CONTROL_SYSTEM__MUJOCO_ROS2_CONTROL_SYSTEM_HPP_
#define MUJOCO_ROS2_CONTROL_SYSTEM__MUJOCO_ROS2_CONTROL_SYSTEM_HPP_

#include "mujoco_ros2_control_system/visibility_control.h"


#include <map>
#include <memory>
#include <string>
#include <vector>
#include <mujoco/mujoco.h>

#include "mujoco_ros2_control/mujoco_ros2_control_system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace mujoco_ros2_control
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Forward declaration
class MujocoRos2SystemPrivate;

// These class must inherit `ign_ros2_control::IgnitionSystemInterface` which implements a
// simulated `ros2_control` `hardware_interface::SystemInterface`.

class MujocoRos2System : public MujocoRos2SystemInterface
{
public:
  // Documentation Inherited
  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info)
  override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Documentation Inherited
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Documentation Inherited
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  
  // Documentation Inherited
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // Documentation Inherited
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // Documentation Inherited
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // Documentation Inherited
  bool initSim(
    rclcpp_lifecycle::LifecycleNode::SharedPtr & model_nh,
    const hardware_interface::HardwareInfo & hardware_info,
    const mjModel* m,
    mjData* d,
    unsigned int & update_rate) override;

private:
  // Register a sensor (for now just IMUs)
  // \param[in] hardware_info hardware information where the data of
  // the sensors is extract.
  // void registerSensors(
  //   const hardware_interface::HardwareInfo & hardware_info);

  /// \brief Private data class
  std::unique_ptr<MujocoRos2SystemPrivate> dataPtr_;
};

struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Joint's mujoco qpos id
  int joint_qposadr;

  /// \brief Joint's mujoco qvel id, also used for qfrc_applied
  int joint_dofadr;

  /// @brief Corresponding joint's effort (torque) actuator index
  int act_effidx = -1;
  /// @brief Corresponding joint's position actuator index
  int act_posidx = -1;
  /// @brief Corresponding joint's velocity actuator index
  int act_velidx = -1;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint position
  double joint_position_cmd;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd;

  /// \brief Current cmd joint effort
  double joint_effort_cmd;

  /// \brief flag if joint is actuated (has command interfaces) or passive
  bool is_actuated;

  /// \brief value to be used for setting velocity actuator's gain, if it exists
  double kv;

  /// \brief value to be used for setting position actuator's gain, if it exists
  double kp;
  
  /// \brief Control method defined in the URDF for each joint.
  mujoco_ros2_control::MujocoRos2SystemInterface::ControlMethod joint_control_method;
};

class MujocoRos2SystemPrivate
{
public:
  MujocoRos2SystemPrivate() = default;

  ~MujocoRos2SystemPrivate() = default;
  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_mj_;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  // /// \brief vector with the imus .
  // std::vector<std::shared_ptr<ImuData>> imus_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Mujoco data pointer
  mjData* d_; 

  /// \brief Mujoco model pointer
  const mjModel* m_; 

  /// \brief controller update rate
  unsigned int * update_rate;

  std::string robot_name_;

};

}  // namespace ign_ros2_control

#endif  // MUJOCO_ROS2_CONTROL_SYSTEM__MUJOCO_ROS2_CONTROL_SYSTEM_HPP_
