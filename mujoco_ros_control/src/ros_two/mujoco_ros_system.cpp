/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: David P. Leins */

#include <mujoco_ros_control/ros_two/mujoco_ros_system.hpp>

#include <algorithm>
#include <sstream>

namespace mujoco_ros {
namespace control {

namespace {
constexpr int kActuatorParamWidth = 10;

int GetActuatorId(const mjModel *model, const std::string &joint_name, const std::string &suffix)
{
	return mj_name2id(model, mjOBJ_ACTUATOR, (joint_name + suffix).c_str());
}

void DisableActuator(const mjModel *model, mjData *data, int actuator_id)
{
	if (actuator_id < 0) {
		return;
	}

	data->ctrl[actuator_id] = 0.;
	for (int i = 0; i < kActuatorParamWidth; ++i) {
		model->actuator_gainprm[kActuatorParamWidth * actuator_id + i] = 0.;
	}
	for (int i = 0; i < kActuatorParamWidth; ++i) {
		model->actuator_biasprm[kActuatorParamWidth * actuator_id + i] = 0.;
	}
}

void ZeroActuator(mjData *data, int actuator_id)
{
	if (actuator_id >= 0) {
		data->ctrl[actuator_id] = 0.;
	}
}

double Clamp(const double value, const double limit)
{
	return std::clamp(value, -limit, limit);
}
} // namespace

bool MujocoRosSystem::initSim(rclcpp_lifecycle::LifecycleNode::SharedPtr &model_nh,
                              const hardware_interface::HardwareInfo &hardware_info, const mjModel *m, mjData *d,
                              unsigned int &update_rate)
{
	// Lesson: Using an uninitialized nh_ will cause the node to simply crash without warning,
	// probably because the output is not piped to stdout.
	// this was the main reason for the crashing, ironically the logging itself was set up wrongly.
	// const std::string node_name = hardware_info.name + "_node";
	// const std::string ns = std::string(model_nh->get_name());
	// const auto node_options = rclcpp::NodeOptions().arguments({"--ros-args", "--remap", node_name + ":__node:=" +
	// node_name}); this->nh_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, ns, node_options);

	this->nh_                                = model_nh;
	this->dataPtr_                           = std::make_unique<MujocoRosSystemPrivate>();
	this->dataPtr_->last_update_sim_time_mj_ = rclcpp::Time(0L, RCL_STEADY_TIME);
	this->dataPtr_->m_                       = m;
	this->dataPtr_->d_                       = d;
	this->dataPtr_->update_rate              = &update_rate;
	this->dataPtr_->n_dof_                   = hardware_info.joints.size();
	this->dataPtr_->joints_.resize(this->dataPtr_->n_dof_);
	this->dataPtr_->robot_name_       = hardware_info.name;
	this->dataPtr_->ignore_actuators_ = false;
	auto ignore_it                    = hardware_info.hardware_parameters.find("ignore_actuators");
	if (ignore_it != hardware_info.hardware_parameters.end()) {
		this->dataPtr_->ignore_actuators_ = ignore_it->second == "true" || ignore_it->second == "1";
	}
	RCLCPP_INFO(this->nh_->get_logger(), "InitSim Robot name: %s", this->dataPtr_->robot_name_.c_str());
	RCLCPP_DEBUG(this->nh_->get_logger(), "initSim start, update rate: %d", *this->dataPtr_->update_rate);
	RCLCPP_DEBUG(this->nh_->get_logger(), "initSim assignments done, update rate: %d", *this->dataPtr_->update_rate);
	RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "Joint size: " << this->dataPtr_->n_dof_);
	for (uint i = 0; i < this->dataPtr_->n_dof_; i++) {
		auto &joint_info       = hardware_info.joints[i];
		std::string joint_name = this->dataPtr_->joints_[i].name = joint_info.name;
		RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "Processing joint #" << i << " named " << joint_name);

		// Handle the default kv and kp parameters to set the joint by, for velocity and position actuators respectively
		this->dataPtr_->joints_[i].kv               = 0.0;
		this->dataPtr_->joints_[i].kp               = 0.0;
		this->dataPtr_->joints_[i].effort_limit     = std::numeric_limits<double>::max();
		this->dataPtr_->joints_[i].ignore_actuators = this->dataPtr_->ignore_actuators_;
		this->dataPtr_->joints_[i].is_actuated      = false;
		if (joint_info.parameters.find("kv") != joint_info.parameters.end()) {
			this->dataPtr_->joints_[i].kv          = std::stod(joint_info.parameters.at("kv"));
			this->dataPtr_->joints_[i].is_actuated = true;
		}
		if (joint_info.parameters.find("kp") != joint_info.parameters.end()) {
			this->dataPtr_->joints_[i].kp          = std::stod(joint_info.parameters.at("kp"));
			this->dataPtr_->joints_[i].is_actuated = true;
		}
		if (joint_info.parameters.find("effort_limit") != joint_info.parameters.end()) {
			this->dataPtr_->joints_[i].effort_limit = std::abs(std::stod(joint_info.parameters.at("effort_limit")));
		}

		int jnt_idx = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
		if (jnt_idx == -1) { // basic check to see if URDF and MuJoCo joint names match
			RCLCPP_FATAL(this->nh_->get_logger(), "The requested joint %s cannot be found in the MuJoCo model!",
			             joint_name.c_str());
			return false;
		}

		// if the check passes, we populate the joint struct with all the data
		this->dataPtr_->joints_[i].joint_qposadr = m->jnt_qposadr[jnt_idx];
		this->dataPtr_->joints_[i].joint_dofadr  = m->jnt_dofadr[jnt_idx];
		this->dataPtr_->joints_[i].act_posidx    = GetActuatorId(m, joint_name, "_act_pos");
		this->dataPtr_->joints_[i].act_velidx    = GetActuatorId(m, joint_name, "_act_vel");
		this->dataPtr_->joints_[i].act_effidx    = GetActuatorId(m, joint_name, "_act_eff");

		if (this->dataPtr_->joints_[i].ignore_actuators) {
			DisableActuator(m, d, this->dataPtr_->joints_[i].act_posidx);
			DisableActuator(m, d, this->dataPtr_->joints_[i].act_velidx);
			DisableActuator(m, d, this->dataPtr_->joints_[i].act_effidx);
		}

		// Populate the STATE INTERFACES
		for (const auto &state_interface : joint_info.state_interfaces) {
			RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "\tState name: " << state_interface.name);
			// position = qpos
			if (state_interface.name == "position") {
				this->dataPtr_->state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_POSITION,
				                                               &this->dataPtr_->joints_[i].joint_position);
				// initialize the data with the current value in the sim
				this->dataPtr_->joints_[i].joint_position = d->qpos[this->dataPtr_->joints_[i].joint_qposadr];
			}
			// velocity = qvel
			else if (state_interface.name == "velocity") {
				this->dataPtr_->state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY,
				                                               &this->dataPtr_->joints_[i].joint_velocity);
				// initialize the data with the current value in the sim
				this->dataPtr_->joints_[i].joint_velocity = d->qvel[this->dataPtr_->joints_[i].joint_dofadr];
			}

			// effort = qfrc, but multipanda has actuator_force + qfrc_gravcomp... need to check
			else if (state_interface.name == "effort") {
				this->dataPtr_->state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT,
				                                               &this->dataPtr_->joints_[i].joint_effort);
				// initialize the data with the current value in the sim
				this->dataPtr_->joints_[i].joint_effort = d->qfrc_applied[this->dataPtr_->joints_[i].joint_dofadr] +
				                                          d->qfrc_actuator[this->dataPtr_->joints_[i].joint_dofadr];
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
		for (const auto &command_interface : joint_info.command_interfaces) {
			RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "\tCommand name: " << command_interface.name);
			// for some reason, this part crashes with realloc(): invalid pointer, but only sometimes?
			if (command_interface.name == "position") {
				if (this->dataPtr_->joints_[i].act_posidx == -1 || this->dataPtr_->joints_[i].ignore_actuators) {
					if (this->dataPtr_->joints_[i].kp <= 0.0) {
						RCLCPP_ERROR(this->nh_->get_logger(),
						             "Joint %s needs a positive kp parameter for position fallback control.",
						             joint_name.c_str());
						return false;
					}
				}
				this->dataPtr_->command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_POSITION,
				                                                 &this->dataPtr_->joints_[i].joint_position_cmd);
				this->dataPtr_->joints_[i].joint_position_cmd = d->qpos[this->dataPtr_->joints_[i].joint_qposadr];
			} else if (command_interface.name == "velocity") {
				if (this->dataPtr_->joints_[i].act_velidx == -1 || this->dataPtr_->joints_[i].ignore_actuators) {
					if (this->dataPtr_->joints_[i].kv <= 0.0) {
						RCLCPP_ERROR(this->nh_->get_logger(),
						             "Joint %s needs a positive kv parameter for velocity fallback control.",
						             joint_name.c_str());
						return false;
					}
				}
				this->dataPtr_->command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY,
				                                                 &this->dataPtr_->joints_[i].joint_velocity_cmd);
				this->dataPtr_->joints_[i].joint_velocity_cmd = d->qvel[this->dataPtr_->joints_[i].joint_dofadr];
			} else if (command_interface.name == "effort") {
				this->dataPtr_->command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT,
				                                                 &this->dataPtr_->joints_[i].joint_effort_cmd);
				this->dataPtr_->joints_[i].joint_effort_cmd = 0.0;
			}
			RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "\tFinished processing: " << command_interface.name);
		}
		this->dataPtr_->joints_[i].is_actuated = (!joint_info.command_interfaces.empty());
		RCLCPP_DEBUG_STREAM(this->nh_->get_logger(), "\tJoint processing done: " << joint_name);
	}

	RCLCPP_DEBUG(this->nh_->get_logger(), "initSim end, update rate: %d", *this->dataPtr_->update_rate);
	return true;
}

bool startsWith(const std::string &mainStr, const std::string &toMatch)
{
	if (mainStr.size() < toMatch.size()) {
		return false;
	}
	return mainStr.compare(0, toMatch.size(), toMatch) == 0;
};
bool stringExistsInVector(const std::vector<std::string> &vec, const std::string &element)
{
	return std::find(vec.begin(), vec.end(), element) != vec.end();
}
std::string getLastElement(const std::string &str, char delimiter)
{
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

CallbackReturn MujocoRosSystem::on_init(const hardware_interface::HardwareInfo &system_info)
{
	RCLCPP_WARN(this->nh_->get_logger(), "On init...");
	// This needs to be called for ResourceManager to be able to find it.
	if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
		return CallbackReturn::ERROR;
	}
	return CallbackReturn::SUCCESS;
};
CallbackReturn MujocoRosSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
	RCLCPP_DEBUG(this->nh_->get_logger(), "on_configure");
	return CallbackReturn::SUCCESS;
};
std::vector<hardware_interface::StateInterface> MujocoRosSystem::export_state_interfaces()
{
	RCLCPP_DEBUG(this->nh_->get_logger(), "export_state_interfaces");
	return std::move(this->dataPtr_->state_interfaces_);
};
std::vector<hardware_interface::CommandInterface> MujocoRosSystem::export_command_interfaces()
{
	RCLCPP_DEBUG(this->nh_->get_logger(), "export_command_interfaces");
	return std::move(this->dataPtr_->command_interfaces_);
};
CallbackReturn MujocoRosSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	RCLCPP_DEBUG(this->nh_->get_logger(), "on_activate");
	return CallbackReturn::SUCCESS;
};
CallbackReturn MujocoRosSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
	RCLCPP_DEBUG(this->nh_->get_logger(), "on_deactivate");
	return CallbackReturn::SUCCESS;
};

hardware_interface::return_type
MujocoRosSystem::prepare_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                             const std::vector<std::string> &stop_interfaces)
{
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
	for (auto &start : start_interfaces) {
		for (auto &joint : this->dataPtr_->joints_) {
			if (startsWith(start, joint.name)) {
				if (!stringExistsInVector(started_joints, joint.name)) {
					filtered_starts.push_back(start);
					break;
				} else {
					RCLCPP_ERROR(this->nh_->get_logger(),
					             "The joint has already been marked to be"
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
	if (filtered_starts.size() > this->dataPtr_->joints_.size()) {
		RCLCPP_ERROR(this->nh_->get_logger(),
		             "The number of start interfaces is"
		             " greater than the number of joints in this system!"
		             " start interfaces: %ld, system joint size: %ld",
		             filtered_starts.size(), this->dataPtr_->joints_.size());
		return hardware_interface::return_type::ERROR;
	}
	// If things have progressed to this point, we can safely set the joint's command mode
	for (auto &start : filtered_starts) {
		for (auto &joint : this->dataPtr_->joints_) {
			if (startsWith(start, joint.name)) {
				if (!joint.is_actuated) {
					RCLCPP_WARN(this->nh_->get_logger(),
					            "Joint %s has received a start interface, but"
					            " it is not actuated! Skipping the joint.",
					            joint.name.c_str());
					break;
				}
				// extract the command mode
				auto interface = getLastElement(start, '/');
				joint.joint_control_method &= NONE; // mask to none
				if (interface == "position") {
					joint.joint_control_method |= POSITION;
				}
				if (interface == "velocity") {
					joint.joint_control_method |= VELOCITY;
				}
				if (interface == "effort") {
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
	for (auto &stop : stop_interfaces) {
		for (auto &joint : this->dataPtr_->joints_) {
			if (startsWith(stop, joint.name)) {
				if (!stringExistsInVector(stopped_joints, joint.name)) {
					filtered_stops.push_back(stop);
					break;
				} else {
					RCLCPP_ERROR(this->nh_->get_logger(),
					             "The joint has already been marked to be"
					             " stopped! Check that you are not trying to stop"
					             " two command interfaces on the same joint together."
					             " Given: %s, target joint: %s",
					             stop.c_str(), joint.name.c_str());
					return hardware_interface::return_type::ERROR;
				}
			} else {
				RCLCPP_INFO_STREAM(this->nh_->get_logger(), stop << " doesn't start with " << joint.name);
			}
		}
	}
	// Make sure that the filtered interfaces are leq to the number of joints in the system
	if (filtered_stops.size() > this->dataPtr_->joints_.size()) {
		RCLCPP_ERROR(this->nh_->get_logger(),
		             "The number of stop interfaces is"
		             " greater than the number of joints in this system!"
		             " stop interfaces: %ld, system joint size: %ld",
		             filtered_stops.size(), this->dataPtr_->joints_.size());
		return hardware_interface::return_type::ERROR;
	}
	// If things have progressed to this point, we can safely set the joint's command mode
	for (auto &stop : filtered_stops) {
		for (auto &joint : this->dataPtr_->joints_) {
			if (startsWith(stop, joint.name)) {
				if (!joint.is_actuated) {
					RCLCPP_WARN(this->nh_->get_logger(),
					            "Joint %s has received a stop interface, but"
					            " it is not actuated! Skipping the joint.",
					            joint.name.c_str());
					break;
				}
				// extract the command mode
				auto interface = getLastElement(stop, '/');
				joint.joint_control_method &= NONE; // mask to none
				RCLCPP_INFO(this->nh_->get_logger(), "Joint %s has been set to STOP", joint.name.c_str());
				break;
			}
		}
	}

	return hardware_interface::return_type::OK;
};

// Documentation Inherited
hardware_interface::return_type
MujocoRosSystem::perform_command_mode_switch(const std::vector<std::string> & /*start_interfaces*/,
                                             const std::vector<std::string> & /*stop_interfaces*/)
{
	RCLCPP_DEBUG(this->nh_->get_logger(), "perform_command_mode_switch");
	for (auto &joint : this->dataPtr_->joints_) {
		if (joint.joint_control_method & EFFORT) {
			RCLCPP_INFO(this->nh_->get_logger(), "Joint %s has EFFORT", joint.name.c_str());
		} else if (joint.joint_control_method & POSITION) {
			RCLCPP_INFO(this->nh_->get_logger(), "Joint %s has POSITION", joint.name.c_str());
		} else if (joint.joint_control_method & VELOCITY) {
			RCLCPP_INFO(this->nh_->get_logger(), "Joint %s has VELOCITY", joint.name.c_str());
		} else {
			RCLCPP_INFO(this->nh_->get_logger(), "Joint %s has NONE", joint.name.c_str());
		}
	}
	return hardware_interface::return_type::OK;
};

// Documentation Inherited
hardware_interface::return_type MujocoRosSystem::read(const rclcpp::Time &time, const rclcpp::Duration & /* period */)
{
	if (epsilonComp(time, this->dataPtr_->last_update_sim_time_mj_, static_cast<double>(*this->dataPtr_->update_rate))) {
		for (uint i = 0; i < this->dataPtr_->joints_.size(); i++) {
			// qposadr and dofadr are always populated, if the system was initialized successfully.
			this->dataPtr_->joints_[i].joint_position = this->dataPtr_->d_->qpos[this->dataPtr_->joints_[i].joint_qposadr];
			this->dataPtr_->joints_[i].joint_velocity = this->dataPtr_->d_->qvel[this->dataPtr_->joints_[i].joint_dofadr];
			this->dataPtr_->joints_[i].joint_effort =
			    this->dataPtr_->d_->qfrc_applied[this->dataPtr_->joints_[i].joint_dofadr] +
			    this->dataPtr_->d_->qfrc_actuator[this->dataPtr_->joints_[i].joint_dofadr];
		}
		// last sim time should be updated in the write function, not read,
		// since read is always called before write.
		// this->dataPtr_->last_update_sim_time_mj_ = time;
	}
	return hardware_interface::return_type::OK;
};

// Documentation Inherited
hardware_interface::return_type MujocoRosSystem::write(const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
{
	if (epsilonComp(time, this->dataPtr_->last_update_sim_time_mj_, static_cast<double>(*this->dataPtr_->update_rate))) {
		for (uint i = 0; i < this->dataPtr_->joints_.size(); i++) {
			auto &joint = this->dataPtr_->joints_[i];
			ZeroActuator(this->dataPtr_->d_, joint.act_posidx);
			ZeroActuator(this->dataPtr_->d_, joint.act_velidx);
			ZeroActuator(this->dataPtr_->d_, joint.act_effidx);

			double effort           = 0.0;
			bool use_force_fallback = false;

			if (joint.joint_control_method & ControlMethod_::POSITION) {
				if (!joint.ignore_actuators && joint.act_posidx != -1) {
					this->dataPtr_->d_->ctrl[joint.act_posidx] = joint.joint_position_cmd;
				} else {
					effort = Clamp(joint.kp * (joint.joint_position_cmd - joint.joint_position), joint.effort_limit);
					use_force_fallback = true;
				}
			} else if (joint.joint_control_method & ControlMethod_::VELOCITY) {
				if (!joint.ignore_actuators && joint.act_velidx != -1) {
					this->dataPtr_->d_->ctrl[joint.act_velidx] = joint.joint_velocity_cmd;
				} else {
					effort = Clamp(joint.kv * (joint.joint_velocity_cmd - joint.joint_velocity), joint.effort_limit);
					use_force_fallback = true;
				}
			} else if (joint.joint_control_method & ControlMethod_::EFFORT) {
				if (!joint.ignore_actuators && joint.act_effidx != -1) {
					this->dataPtr_->d_->ctrl[joint.act_effidx] = joint.joint_effort_cmd;
				} else {
					effort             = Clamp(joint.joint_effort_cmd, joint.effort_limit);
					use_force_fallback = true;
				}
			}

			if (use_force_fallback) {
				this->dataPtr_->d_->qfrc_applied[joint.joint_dofadr] = effort;
			} else {
				this->dataPtr_->d_->qfrc_applied[joint.joint_dofadr] = 0.0;
			}
		}
		this->dataPtr_->last_update_sim_time_mj_ = time;
	}
	return hardware_interface::return_type::OK;
};

} // namespace control
} // namespace mujoco_ros

#include "pluginlib/class_list_macros.hpp" // NOLINT
PLUGINLIB_EXPORT_CLASS(mujoco_ros::control::MujocoRosSystem, mujoco_ros::control::MujocoRosSystemInterface)
