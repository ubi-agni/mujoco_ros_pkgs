#include <mujoco_ros2_control_system/mujoco_ros2_control_system.hpp>

namespace mujoco_ros2_control{

bool MujocoRos2System::initSim(
    rclcpp::Node::SharedPtr & model_nh,
    const hardware_interface::HardwareInfo & hardware_info,
    // const mjModel* m,
    // mjData* d,
    int & update_rate){
    
    // Lesson: Using an uninitialized nh_ will cause the node to simply crash without warning,
    // probably because the output is not piped to stdout.
    // this was the main reason for the crashing, ironically the logging itself was set up wrongly.
    this->nh_ = model_nh;
    RCLCPP_INFO(model_nh->get_logger(), "initSim start");
    // this->dataPtr_ = std::make_unique<MujocoRos2SystemPrivate>();
    // this->dataPtr_->last_update_sim_time_ros_ = rclcpp::Time();
    // this->dataPtr_->m_ = m;
    // this->dataPtr_->d_ = d;
    // this->dataPtr_->update_rate = &update_rate;
    // this->dataPtr_->n_dof_ = hardware_info.joints.size();
    RCLCPP_INFO(model_nh->get_logger(), "initSim done");

    return true;
  }


CallbackReturn MujocoRos2System::on_init(const hardware_interface::HardwareInfo & system_info){
  RCLCPP_INFO(this->nh_->get_logger(), "on_init");
  return CallbackReturn::SUCCESS;
};
CallbackReturn MujocoRos2System::on_configure(const rclcpp_lifecycle::State & previous_state){
  RCLCPP_INFO(this->nh_->get_logger(), "on_configure");
  return CallbackReturn::SUCCESS;
};
std::vector<hardware_interface::StateInterface> MujocoRos2System::export_state_interfaces(){
  // return std::move(this->dataPtr_->state_interfaces_);
  return std::vector<hardware_interface::StateInterface>{};
};
std::vector<hardware_interface::CommandInterface> MujocoRos2System::export_command_interfaces(){
  // return std::move(this->dataPtr_->command_interfaces_);
  return std::vector<hardware_interface::CommandInterface>{};
};
CallbackReturn MujocoRos2System::on_activate(const rclcpp_lifecycle::State & previous_state){
  RCLCPP_INFO(this->nh_->get_logger(), "on_activate");
  return CallbackReturn::SUCCESS;
};
CallbackReturn MujocoRos2System::on_deactivate(const rclcpp_lifecycle::State & previous_state){
  RCLCPP_INFO(this->nh_->get_logger(), "on_deactivate");
  return CallbackReturn::SUCCESS;
};

// Documentation Inherited
hardware_interface::return_type MujocoRos2System::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces){

    RCLCPP_INFO(this->nh_->get_logger(), "perform_command_mode_switch");
    return hardware_interface::return_type::OK;
  };

// Documentation Inherited
hardware_interface::return_type MujocoRos2System::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period){

    RCLCPP_INFO(this->nh_->get_logger(), "read");
    return hardware_interface::return_type::OK;
  };

// Documentation Inherited
hardware_interface::return_type MujocoRos2System::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period){

    RCLCPP_INFO(this->nh_->get_logger(), "write");
    return hardware_interface::return_type::OK;
  };

} // namespace mujoco_ros2

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  mujoco_ros2_control::MujocoRos2System, mujoco_ros2_control::MujocoRos2SystemInterface)
