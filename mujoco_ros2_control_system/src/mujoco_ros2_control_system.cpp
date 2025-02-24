#include <mujoco_ros2_control_system/mujoco_ros2_control_system.hpp>

namespace mujoco_ros2_control{

bool MujocoRos2System::initSim(
    rclcpp_lifecycle::LifecycleNode::SharedPtr & model_nh,
    const hardware_interface::HardwareInfo & hardware_info,
    const mjModel* m,
    mjData* d,
    unsigned int & update_rate){
    
    // Lesson: Using an uninitialized nh_ will cause the node to simply crash without warning,
    // probably because the output is not piped to stdout.
    // this was the main reason for the crashing, ironically the logging itself was set up wrongly.
    // const std::string node_name = hardware_info.name + "_node";
    // const std::string ns = std::string(model_nh->get_name());
    // const auto node_options = rclcpp::NodeOptions().arguments({"--ros-args", "--remap", node_name + ":__node:=" + node_name});
    // this->nh_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, ns, node_options);

    this->nh_ = model_nh;
    this->dataPtr_ = std::make_unique<MujocoRos2SystemPrivate>();
    this->dataPtr_->last_update_sim_time_mj_ = rclcpp::Time(0L, RCL_STEADY_TIME);
    this->dataPtr_->m_ = m;
    this->dataPtr_->d_ = d;
    this->dataPtr_->update_rate = &update_rate;
    this->dataPtr_->n_dof_ = hardware_info.joints.size();
    this->dataPtr_->joints_.resize(this->dataPtr_->n_dof_);
    this->dataPtr_->robot_name_ = hardware_info.name;
    RCLCPP_INFO(this->nh_->get_logger(), "InitSim Robot name: %s", this->dataPtr_->robot_name_.c_str());
    RCLCPP_DEBUG(this->nh_->get_logger(), "initSim start, update rate: %d", *this->dataPtr_->update_rate);
    RCLCPP_DEBUG(this->nh_->get_logger(), "initSim assignments done, update rate: %d", *this->dataPtr_->update_rate);
    RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "Joint size: " << this->dataPtr_->n_dof_);
    for (uint i = 0; i<this->dataPtr_->n_dof_; i++){
      auto & joint_info = hardware_info.joints[i];
      std::string joint_name = this->dataPtr_->joints_[i].name = joint_info.name;
      RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "Processing joint #" << i << " named " << joint_name);

      // Handle the default kv and kp parameters to set the joint by, for velocity and position actuators respectively      
      this->dataPtr_->joints_[i].kv = 0.0;
      this->dataPtr_->joints_[i].kp = 0.0;
      this->dataPtr_->joints_[i].is_actuated = false;
      if(joint_info.parameters.find("kv") != joint_info.parameters.end()){
        this->dataPtr_->joints_[i].kv = stod(joint_info.parameters.at("kv"));
        this->dataPtr_->joints_[i].is_actuated = true;
      }
      if(joint_info.parameters.find("kp") != joint_info.parameters.end()){
        this->dataPtr_->joints_[i].kp = stod(joint_info.parameters.at("kp"));
        this->dataPtr_->joints_[i].is_actuated = true;
      }

      bool has_value = joint_info.parameters.find("test") != joint_info.parameters.end();
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
        else if(joint_info.state_interfaces[j].name == "velocity"){
          this->dataPtr_->state_interfaces_.emplace_back(joint_name, 
                                                          hardware_interface::HW_IF_VELOCITY, 
                                                          &this->dataPtr_->joints_[i].joint_velocity);
          // initialize the data with the current value in the sim
          this->dataPtr_->joints_[j].joint_velocity = d->qvel[this->dataPtr_->joints_[i].joint_dofadr];
        }

        // effort = qfrc, but multipanda has actuator_force + qfrc_gravcomp... need to check
        else if(joint_info.state_interfaces[j].name == "effort"){
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
        // for some reason, this part crashes with realloc(): invalid pointer, but only sometimes?
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
        else if(joint_info.command_interfaces[j].name == "velocity"){
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
        else if(joint_info.command_interfaces[j].name == "effort"){
          std::string act_name = joint_name + "_act_eff";
          int act_idx = mj_name2id(m, mjOBJ_ACTUATOR, act_name.c_str());
          if(act_idx == -1){
            RCLCPP_WARN(this->nh_->get_logger(), "Could not find a position actuator with name %s in the mujoco xml file.", act_name.c_str());
            this->dataPtr_->joints_[i].act_effidx = -1;
            continue;
          }
          this->dataPtr_->joints_[i].act_effidx = act_idx;
          this->dataPtr_->command_interfaces_.emplace_back(joint_name, 
                                                          hardware_interface::HW_IF_EFFORT, 
                                                          &this->dataPtr_->joints_[i].joint_effort_cmd);
          this->dataPtr_->joints_[i].joint_effort_cmd = d->qvel[this->dataPtr_->joints_[i].act_effidx];                                                          
        }
        RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "\tFinished processing: " << joint_info.command_interfaces[j].name);
      }
      this->dataPtr_->joints_[i].is_actuated = (joint_info.command_interfaces.size() > 0);
      RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "\tJoint processing done: " << joint_name);
    }

    RCLCPP_DEBUG(this->nh_->get_logger(), "initSim end, update rate: %d", *this->dataPtr_->update_rate);
    return true;
  }

bool startsWith(const std::string& mainStr, const std::string& toMatch) {
    if (mainStr.size() < toMatch.size()) {
        return false;
    }
    return mainStr.compare(0, toMatch.size(), toMatch) == 0;
  };
bool stringExistsInVector(const std::vector<std::string>& vec, const std::string& element) {
    return std::find(vec.begin(), vec.end(), element) != vec.end();
}
std::string getLastElement(const std::string& str, char delimiter) {
    std::stringstream ss(str);
    std::string item;
    std::vector<std::string> elements;

    while (std::getline(ss, item, delimiter)) {
        elements.push_back(item);
    }

    if (!elements.empty()) {
        return elements.back();
    }

    return "";
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

hardware_interface::return_type MujocoRos2System::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces){

    /*
    For the interface,
    first we get a collection of interfaces that match the joint names in this system.
    Then we check that the interfaces are unique on the joint level, i.e. 
    we should avoid
    joint1/velocity
    joint1/effort -> throw error
    
    ending up with filtered_interfaces

    then we enforce that the no. of unique joints are leq to the joints in this system,
    filtered_interfaces.size() <= this->dataPtr_->joints_.size()

    then finally we change the joints' control modes,
    which will then be used in perform_command_mode_switch

    */
    RCLCPP_DEBUG(this->nh_->get_logger(), "prepare_command_mode_switch");

    ///////////////////////////////////
    //            Start case         //
    ///////////////////////////////////
    std::vector<std::string> filtered_starts;
    std::vector<std::string> started_joints;
    for(auto& start : start_interfaces){
      for(auto& joint : this->dataPtr_->joints_){
        if(startsWith(start, joint.name)){
          if(!stringExistsInVector(started_joints,joint.name)){
            filtered_starts.push_back(start);
            break;
          }
          else{
            RCLCPP_ERROR(this->nh_->get_logger(), "The joint %s has already been marked to be"
                                            " started! Check that you are not trying to start"
                                            " two command interfaces on the same joint together."
                                            " Given: %s, target joint: %s", 
                                            start.c_str(), joint.name.c_str());
            return hardware_interface::return_type::ERROR;
          }
        }
      }
    }
    // Make sure that the filtered interfaces are leq to the number of joints in the system
    if(filtered_starts.size() > this->dataPtr_->joints_.size()){
      RCLCPP_ERROR(this->nh_->get_logger(), "The number of start interfaces is"
                                            " greater than the number of joints in this system!"
                                            " start interfaces: %d, system joint size: %d", 
                                            filtered_starts.size(), this->dataPtr_->joints_.size());
      return hardware_interface::return_type::ERROR;
    }
    // If things have progressed to this point, we can safely set the joint's command mode
    for(auto& start : filtered_starts){
      for(auto& joint : this->dataPtr_->joints_){
        if(startsWith(start, joint.name)){
          if(!joint.is_actuated){
            RCLCPP_WARN(this->nh_->get_logger(), "Joint %s has received a start interface, but"
                                                 " it is not actuated! Skipping the joint.", joint.name.c_str());
            break;
          }
          // extract the command mode
          auto interface = getLastElement(start, '/');
          joint.joint_control_method &= NONE; // mask to none
          if(interface == "position"){
            joint.joint_control_method |= POSITION;
          }
          if(interface == "velocity"){
            joint.joint_control_method |= VELOCITY;
          }
          if(interface == "effort"){
            joint.joint_control_method |= EFFORT;
          }
          RCLCPP_INFO(this->nh_->get_logger(), "Joint %s has been set to %s", joint.name.c_str(), interface.c_str());
          break;
        }
      }
    }


    ///////////////////////////////////
    //            Stop case          //
    ///////////////////////////////////
    std::vector<std::string> filtered_stops;
    std::vector<std::string> stopped_joints;
    for(auto& stop : stop_interfaces){
      for(auto& joint : this->dataPtr_->joints_){
        if(startsWith(stop, joint.name)){
          if(!stringExistsInVector(stopped_joints,joint.name)){
            filtered_stops.push_back(stop);
            break;
          }
          else{
            RCLCPP_ERROR(this->nh_->get_logger(), "The joint %s has already been marked to be"
                                            " stopped! Check that you are not trying to stop"
                                            " two command interfaces on the same joint together."
                                            " Given: %s, target joint: %s", 
                                            stop.c_str(), joint.name.c_str());
            return hardware_interface::return_type::ERROR;
          }
        }
        else{
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), stop << " doesn't start with " << joint.name);
        }
      }
    }
    // Make sure that the filtered interfaces are leq to the number of joints in the system
    if(filtered_stops.size() > this->dataPtr_->joints_.size()){
      RCLCPP_ERROR(this->nh_->get_logger(), "The number of stop interfaces is"
                                            " greater than the number of joints in this system!"
                                            " stop interfaces: %d, system joint size: %d", 
                                            filtered_stops.size(), this->dataPtr_->joints_.size());
      return hardware_interface::return_type::ERROR;
    }
    // If things have progressed to this point, we can safely set the joint's command mode
    for(auto& stop : filtered_stops){
      for(auto& joint : this->dataPtr_->joints_){
        if(startsWith(stop, joint.name)){
          if(!joint.is_actuated){
            RCLCPP_WARN(this->nh_->get_logger(), "Joint %s has received a stop interface, but"
                                                 " it is not actuated! Skipping the joint.", joint.name.c_str());
            break;
          }
          // extract the command mode
          auto interface = getLastElement(stop, '/');
          joint.joint_control_method &= NONE; // mask to none
          RCLCPP_INFO(this->nh_->get_logger(), "Joint %s has been set to STOP", joint.name.c_str(), interface.c_str());
          break;
        }
      }
    }

    return hardware_interface::return_type::OK;
  };

// Documentation Inherited
hardware_interface::return_type MujocoRos2System::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces){

    RCLCPP_DEBUG(this->nh_->get_logger(), "perform_command_mode_switch");
    // This is a simple for loop over joints with a switch case over the control mode enum.
    // TODO: Set the appropriate gains for each of the command modes 
    for(auto &joint : this->dataPtr_->joints_){
      if(joint.joint_control_method & EFFORT){
        RCLCPP_INFO(this->nh_->get_logger(), "Joint %s has EFFORT", joint.name.c_str());
        setTorqueControl(this->dataPtr_->m_, joint.act_effidx, 1);
        setPositionServo(this->dataPtr_->m_, joint.act_posidx, 0);
        setVelocityServo(this->dataPtr_->m_, joint.act_velidx, 0);
      }
      else if(joint.joint_control_method & POSITION){
        RCLCPP_INFO(this->nh_->get_logger(), "Joint %s has POSITION", joint.name.c_str());
        setTorqueControl(this->dataPtr_->m_, joint.act_effidx, 0);
        setPositionServo(this->dataPtr_->m_, joint.act_posidx, joint.kp);
        setVelocityServo(this->dataPtr_->m_, joint.act_velidx, 0);
      }
      else if(joint.joint_control_method & VELOCITY){
        RCLCPP_INFO(this->nh_->get_logger(), "Joint %s has VELOCITY", joint.name.c_str());
        setTorqueControl(this->dataPtr_->m_, joint.act_effidx, 0);
        setPositionServo(this->dataPtr_->m_, joint.act_posidx, 0);
        setVelocityServo(this->dataPtr_->m_, joint.act_velidx, joint.kv);
      }
      else{
        RCLCPP_INFO(this->nh_->get_logger(), "Joint %s has NONE", joint.name.c_str());
        setTorqueControl(this->dataPtr_->m_, joint.act_effidx, 0);
        setPositionServo(this->dataPtr_->m_, joint.act_posidx, 0);
        setVelocityServo(this->dataPtr_->m_, joint.act_velidx, 0);
      }

    }
    return hardware_interface::return_type::OK;
  };

// Documentation Inherited
hardware_interface::return_type MujocoRos2System::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & /* period */){
    if(epsilonComp(time, this->dataPtr_->last_update_sim_time_mj_, static_cast<double>(*this->dataPtr_->update_rate))){
      for(uint i = 0; i < this->dataPtr_->joints_.size(); i++){
        // qposadr and dofadr are always populated, if the system was initialized successfully.
        this->dataPtr_->joints_[i].joint_position = this->dataPtr_->d_->qpos[this->dataPtr_->joints_[i].joint_qposadr];
        this->dataPtr_->joints_[i].joint_velocity = this->dataPtr_->d_->qvel[this->dataPtr_->joints_[i].joint_dofadr];
        this->dataPtr_->joints_[i].joint_effort   = this->dataPtr_->d_->qfrc_applied[this->dataPtr_->joints_[i].joint_dofadr] +
                                                    this->dataPtr_->d_->qfrc_actuator[this->dataPtr_->joints_[i].joint_dofadr];
      }
      // last sim time should be updated in the write function, not read,
      // since read is always called before write.
      // this->dataPtr_->last_update_sim_time_mj_ = time;
    }
    return hardware_interface::return_type::OK;
  };

// Documentation Inherited
hardware_interface::return_type MujocoRos2System::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period){
    
    if(epsilonComp(time, this->dataPtr_->last_update_sim_time_mj_, static_cast<double>(*this->dataPtr_->update_rate))){
      for(uint i = 0; i < this->dataPtr_->joints_.size(); i++){
        // Simply check the joint's control method, and whether the corresponding mj actuator index is populated or not
        if(this->dataPtr_->joints_[i].act_posidx != -1)
        {
          if(this->dataPtr_->joints_[i].joint_control_method & ControlMethod_::POSITION){
            this->dataPtr_->d_->ctrl[this->dataPtr_->joints_[i].act_posidx] = this->dataPtr_->joints_[i].joint_position_cmd;
          }
          else{
            this->dataPtr_->d_->ctrl[this->dataPtr_->joints_[i].act_posidx] = 0.0;
          }
        }
        if(this->dataPtr_->joints_[i].act_velidx != -1){
          if(this->dataPtr_->joints_[i].joint_control_method & ControlMethod_::VELOCITY){
            this->dataPtr_->d_->ctrl[this->dataPtr_->joints_[i].act_velidx] = this->dataPtr_->joints_[i].joint_velocity_cmd;
          }
          else{
            this->dataPtr_->d_->ctrl[this->dataPtr_->joints_[i].act_velidx] = 0.0;
          }

        }
        if(this->dataPtr_->joints_[i].act_effidx != -1){
          if(this->dataPtr_->joints_[i].joint_control_method & ControlMethod_::EFFORT){
            this->dataPtr_->d_->ctrl[this->dataPtr_->joints_[i].act_effidx] = this->dataPtr_->joints_[i].joint_effort_cmd;
          }
          else{
            this->dataPtr_->d_->ctrl[this->dataPtr_->joints_[i].act_effidx] = 0.0;
          }
        }
      }
      this->dataPtr_->last_update_sim_time_mj_ = time;
    }
    return hardware_interface::return_type::OK;
  };

} // namespace mujoco_ros2

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  mujoco_ros2_control::MujocoRos2System, mujoco_ros2_control::MujocoRos2SystemInterface)
