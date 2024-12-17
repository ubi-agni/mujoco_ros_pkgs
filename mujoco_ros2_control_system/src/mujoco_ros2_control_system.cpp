#include <mujoco_ros2_control_system/mujoco_ros2_control_system.hpp>

namespace mujoco_ros2_control{

bool MujocoRos2System::initSim(
    rclcpp::Node::SharedPtr & model_nh,
    const hardware_interface::HardwareInfo & hardware_info,
    const mjModel* m,
    mjData* d,
    int & update_rate){
    
    // Lesson: Using an uninitialized nh_ will cause the node to simply crash without warning,
    // probably because the output is not piped to stdout.
    // this was the main reason for the crashing, ironically the logging itself was set up wrongly.
    this->nh_ = model_nh;
    this->dataPtr_ = std::make_unique<MujocoRos2SystemPrivate>();
    this->dataPtr_->last_update_sim_time_mj_ = rclcpp::Time(0L, RCL_STEADY_TIME);
    // this->dataPtr_->m_ = m;
    this->dataPtr_->d_ = d;
    this->dataPtr_->update_rate = &update_rate;
    this->dataPtr_->n_dof_ = hardware_info.joints.size();
    this->dataPtr_->joints_.resize(this->dataPtr_->n_dof_);

    RCLCPP_DEBUG(this->nh_->get_logger(), "initSim assignments done, update rate: %d", *this->dataPtr_->update_rate);
    RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "Joint size: " << this->dataPtr_->n_dof_);
    for (uint i = 0; i<this->dataPtr_->n_dof_; i++){
      auto & joint_info = hardware_info.joints[i];
      std::string joint_name = this->dataPtr_->joints_[i].name = joint_info.name;
      RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "Processing joint #" << i << " named " << joint_name);

      int jnt_idx = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
      if(jnt_idx == -1){ // basic check to see if URDF and MuJoCo joint names match
        RCLCPP_FATAL(this->nh_->get_logger(), "The requested joint %s cannot be found in the MuJoCo model!", joint_name.c_str());
        return false;
      }

      // if the check passes, we populate the joint struct with all the data
      this->dataPtr_->joints_[i].joint_qposadr = m->jnt_qposadr[jnt_idx];
      this->dataPtr_->joints_[i].joint_dofadr = m->jnt_dofadr[jnt_idx];


      // Populate the STATE INTERFACES
      for (uint j = 0; j < joint_info.state_interfaces.size(); ++j) {
        RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "\tState name: " << joint_info.state_interfaces[j].name);
        // position = qpos
        if(joint_info.state_interfaces[j].name == "position"){
          this->dataPtr_->state_interfaces_.emplace_back(joint_name, 
                                                          hardware_interface::HW_IF_POSITION, 
                                                          &this->dataPtr_->joints_[i].joint_position);
          // initialize the data with the current value in the sim
          this->dataPtr_->joints_[j].joint_position = d->qpos[this->dataPtr_->joints_[i].joint_qposadr];
        }
        // velocity = qvel
        if(joint_info.state_interfaces[j].name == "velocity"){
          this->dataPtr_->state_interfaces_.emplace_back(joint_name, 
                                                          hardware_interface::HW_IF_VELOCITY, 
                                                          &this->dataPtr_->joints_[i].joint_velocity);
          // initialize the data with the current value in the sim
          this->dataPtr_->joints_[j].joint_velocity = d->qvel[this->dataPtr_->joints_[i].joint_dofadr];
        }

        // effort = qfrc, but multipanda has actuator_force + qfrc_gravcomp... need to check
        if(joint_info.state_interfaces[j].name == "effort"){
          this->dataPtr_->state_interfaces_.emplace_back(joint_name, 
                                                          hardware_interface::HW_IF_EFFORT, 
                                                          &this->dataPtr_->joints_[i].joint_effort);
          // initialize the data with the current value in the sim
          this->dataPtr_->joints_[j].joint_effort = d->qfrc_applied[this->dataPtr_->joints_[i].joint_dofadr] + d->qfrc_actuator[this->dataPtr_->joints_[i].joint_dofadr];
          // this would work, if every joint has an associated torque controller... but since that's not always true,
          // just using applied force might be better in the end
          // d->actuator_force[act_trq_indices_[i]] + d->qfrc_gravcomp[act_trq_indices_[i]]; 
        }
      }

      // Populate the COMMAND INTERFACES
      /*
      We need a mapping from, 
      the current joint's name -> associated actuator's name/adr in mujoco
      seems like trnid is the key to that, well, trnid * 2, since we don't consider actuators acting on tendons 
      But then again, maybe overthinking? Just document that it should be jointname_act_type
      */
      for (uint j = 0; j < joint_info.command_interfaces.size(); ++j) {
        RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "\tCommand name: " << joint_info.command_interfaces[j].name);
        if(joint_info.command_interfaces[j].name == "position"){
          std::string act_name = joint_name + "_act_pos";
          int act_idx = mj_name2id(m, mjOBJ_ACTUATOR, act_name.c_str());
          if(act_idx == -1){
            RCLCPP_WARN(this->nh_->get_logger(), "Could not find a position actuator with name %s in the mujoco xml file.", act_name.c_str());
            this->dataPtr_->joints_[i].act_posidx = -1;  
            continue;
          }
          this->dataPtr_->joints_[i].act_posidx = act_idx;
          this->dataPtr_->command_interfaces_.emplace_back(joint_name, 
                                                          hardware_interface::HW_IF_POSITION, 
                                                          &this->dataPtr_->joints_[i].joint_position_cmd);
          this->dataPtr_->joints_[i].joint_position_cmd = d->qvel[this->dataPtr_->joints_[i].act_posidx];
        }
        if(joint_info.command_interfaces[j].name == "velocity"){
          std::string act_name = joint_name + "_act_vel";
          int act_idx = mj_name2id(m, mjOBJ_ACTUATOR, act_name.c_str());
          if(act_idx == -1){
            RCLCPP_WARN(this->nh_->get_logger(), "Could not find a position actuator with name %s in the mujoco xml file.", act_name.c_str());
            this->dataPtr_->joints_[i].act_velidx = -1;
            continue;
          }
          this->dataPtr_->joints_[i].act_velidx = act_idx;
          this->dataPtr_->command_interfaces_.emplace_back(joint_name, 
                                                          hardware_interface::HW_IF_VELOCITY, 
                                                          &this->dataPtr_->joints_[i].joint_velocity_cmd);
          this->dataPtr_->joints_[i].joint_velocity_cmd = d->qvel[this->dataPtr_->joints_[i].act_velidx];
        }
        if(joint_info.command_interfaces[j].name == "effort"){
          std::string act_name = joint_name + "_act_eff";
          int act_idx = mj_name2id(m, mjOBJ_ACTUATOR, act_name.c_str());
          if(act_idx == -1){
            RCLCPP_WARN(this->nh_->get_logger(), "Could not find a position actuator with name %s in the mujoco xml file.", act_name.c_str());
            this->dataPtr_->joints_[i].act_effidx = -1;
            continue;
          }
          this->dataPtr_->joints_[i].act_effidx = act_idx;
          this->dataPtr_->command_interfaces_.emplace_back(joint_name, 
                                                          hardware_interface::HW_IF_POSITION, 
                                                          &this->dataPtr_->joints_[i].joint_effort_cmd);
          this->dataPtr_->joints_[i].joint_effort_cmd = d->qvel[this->dataPtr_->joints_[i].act_effidx];                                                          
        }
      }
      this->dataPtr_->joints_[i].is_actuated = (joint_info.command_interfaces.size() > 0);

    }
    return true;
  }


CallbackReturn MujocoRos2System::on_init(const hardware_interface::HardwareInfo & system_info){
  RCLCPP_WARN(this->nh_->get_logger(), "On init...");
  // This needs to be called for ResourceManager to be able to find it.
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
};
CallbackReturn MujocoRos2System::on_configure(const rclcpp_lifecycle::State & previous_state){
  RCLCPP_DEBUG(this->nh_->get_logger(), "on_configure");
  return CallbackReturn::SUCCESS;
};
std::vector<hardware_interface::StateInterface> MujocoRos2System::export_state_interfaces(){
  RCLCPP_DEBUG(this->nh_->get_logger(), "export_state_interfaces");
  return std::move(this->dataPtr_->state_interfaces_);
};
std::vector<hardware_interface::CommandInterface> MujocoRos2System::export_command_interfaces(){
  RCLCPP_DEBUG(this->nh_->get_logger(), "export_command_interfaces");
  return std::move(this->dataPtr_->command_interfaces_);
};
CallbackReturn MujocoRos2System::on_activate(const rclcpp_lifecycle::State & previous_state){
  RCLCPP_DEBUG(this->nh_->get_logger(), "on_activate");
  return CallbackReturn::SUCCESS;
};
CallbackReturn MujocoRos2System::on_deactivate(const rclcpp_lifecycle::State & previous_state){
  RCLCPP_DEBUG(this->nh_->get_logger(), "on_deactivate");
  return CallbackReturn::SUCCESS;
};

// Documentation Inherited
hardware_interface::return_type MujocoRos2System::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces){

    RCLCPP_DEBUG(this->nh_->get_logger(), "perform_command_mode_switch");
    return hardware_interface::return_type::OK;
  };

// Documentation Inherited
hardware_interface::return_type MujocoRos2System::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & /* period */){

    if(epsilonComp(time, this->dataPtr_->last_update_sim_time_mj_, static_cast<double>(*this->dataPtr_->update_rate))){
      RCLCPP_DEBUG(this->nh_->get_logger(), "read, curr time: %f, last update: %f", time.seconds(), this->dataPtr_->last_update_sim_time_mj_.seconds());
      for(uint i = 0; i < this->dataPtr_->joints_.size(); i++){
        // qposadr and dofadr are always populated, if the system was initialized successfully.
        this->dataPtr_->joints_[i].joint_position = this->dataPtr_->d_->qpos[this->dataPtr_->joints_[i].joint_qposadr];
        this->dataPtr_->joints_[i].joint_velocity = this->dataPtr_->d_->qvel[this->dataPtr_->joints_[i].joint_dofadr];
        this->dataPtr_->joints_[i].joint_effort   = this->dataPtr_->d_->qfrc_applied[this->dataPtr_->joints_[i].joint_dofadr] +
                                                    this->dataPtr_->d_->qfrc_actuator[this->dataPtr_->joints_[i].joint_dofadr];
      }
      this->dataPtr_->last_update_sim_time_mj_ = time;
    }
    return hardware_interface::return_type::OK;
  };

// Documentation Inherited
hardware_interface::return_type MujocoRos2System::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period){
    
    if(epsilonComp(time, this->dataPtr_->last_update_sim_time_mj_, static_cast<double>(*this->dataPtr_->update_rate))){
      RCLCPP_DEBUG(this->nh_->get_logger(), "write");
      for(uint i = 0; i < this->dataPtr_->joints_.size(); i++){
        // Simply check the joint's control method, and whether the corresponding mj actuator index is populated or not
        if(this->dataPtr_->joints_[i].joint_control_method & ControlMethod_::POSITION &&
          this->dataPtr_->joints_[i].act_posidx != -1){
          this->dataPtr_->d_->ctrl[this->dataPtr_->joints_[i].act_posidx]= this->dataPtr_->joints_[i].joint_position_cmd;
        }
        if(this->dataPtr_->joints_[i].joint_control_method & ControlMethod_::VELOCITY &&
          this->dataPtr_->joints_[i].act_velidx != -1){
        this->dataPtr_->d_->ctrl[this->dataPtr_->joints_[i].act_velidx]= this->dataPtr_->joints_[i].joint_velocity_cmd;
        }
        if(this->dataPtr_->joints_[i].joint_control_method & ControlMethod_::EFFORT &&
          this->dataPtr_->joints_[i].act_effidx != -1){
        this->dataPtr_->d_->ctrl[this->dataPtr_->joints_[i].act_effidx]= this->dataPtr_->joints_[i].joint_effort_cmd;
        }
      }
    }
    return hardware_interface::return_type::OK;
  };

} // namespace mujoco_ros2

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  mujoco_ros2_control::MujocoRos2System, mujoco_ros2_control::MujocoRos2SystemInterface)
