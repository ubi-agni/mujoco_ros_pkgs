/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2023, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
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
 */

/* Authors: David P. Leins*/

#include <mujoco_ros_control/default_robot_hw_sim.h>
#include <mujoco_ros/util.h>

namespace {
double clamp(const double val, const double min_val, const double max_val)
{
	return std::min(std::max(val, min_val), max_val);
}
} // namespace

namespace mujoco_ros::control {

bool DefaultRobotHWSim::initSim(mujoco_ros::mjModelPtr m_ptr, mujoco_ros::mjDataPtr d_ptr,
                                MujocoEnvPtr /*mujoco_env_ptr*/, const std::string &robot_namespace,
                                ros::NodeHandle model_nh, const urdf::Model *const urdf_model,
                                std::vector<transmission_interface::TransmissionInfo> transmissions)
{
	// getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
	// parameters name is "joint_limits/<joint name>" an example is "joint_limits/axle_joint"
	const ros::NodeHandle joint_limit_nh(model_nh);

	m_ptr_ = m_ptr;
	d_ptr_ = d_ptr;

	// Resize vectors to our DOF
	n_dof_ = transmissions.size();
	joint_names_.resize(n_dof_);
	joint_types_.resize(n_dof_);
	joint_lower_limits_.resize(n_dof_);
	joint_upper_limits_.resize(n_dof_);
	joint_effort_limits_.resize(n_dof_);
	joint_control_methods_.resize(n_dof_);
	pid_controllers_.resize(n_dof_);
	joint_position_.resize(n_dof_);
	joint_velocity_.resize(n_dof_);
	joint_effort_.resize(n_dof_);
	joint_effort_command_.resize(n_dof_);
	joint_position_command_.resize(n_dof_);
	joint_velocity_command_.resize(n_dof_);

	mujoco_joint_ids_.resize(n_dof_);

	ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim", "Got " << n_dof_ << " transmissions to process ...");

	// Initialize values
	for (unsigned int j = 0; j < n_dof_; j++) {
		ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim", "\tparsing transmission " << j);
		// Check that this transmission has one joint
		if (transmissions[j].joints_.size() == 0) {
			ROS_WARN_STREAM_NAMED("default_robot_hw_sim",
			                      "Transmission " << transmissions[j].name_ << " has no associated joints.");
			mujoco_joint_ids_[j] = -1;
			continue;
		} else if (transmissions[j].joints_.size() > 1) {
			ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Transmission "
			                                                  << transmissions[j].name_
			                                                  << " has more than one joint. Currently the default robot "
			                                                     "hardare simulation interface only supports one.");
			mujoco_joint_ids_[j] = -1;
			continue;
		}

		std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
		if (joint_interfaces.empty() && !(transmissions[j].actuators_.empty()) &&
		    !(transmissions[j].actuators_[0].hardware_interfaces_.empty())) {
			std::vector<std::string> joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
			ROS_WARN_STREAM_NAMED("default_robot_hw_sim",
			                      "The <hardware_interface> element of transmission "
			                          << transmissions[j].name_
			                          << " should be nested inside the <joint> element, not <actuator>. The transmission "
			                             "will be properly loaded, but please update your robot model to remain "
			                             "compatible with future versions of the plugin.");
		}

		if (joint_interfaces.empty()) {
			ROS_WARN_STREAM_NAMED(
			    "default_robot_hw_sim",
			    "Joint " << transmissions[j].joints_[0].name_ << " of transmission " << transmissions[j].name_
			             << " does not specify any hardware interface. Not adding it to the robot hardware simulation.");
			mujoco_joint_ids_[j] = -1;
			continue;
		} else if (joint_interfaces.size() > 1) {
			ROS_WARN_STREAM_NAMED("default_robot_hw_sim",
			                      "Joint " << transmissions[j].joints_[0].name_ << " of transmission "
			                               << transmissions[j].name_
			                               << " specifies multiple hardware interfaces. Currently the default robot "
			                                  "hardware simulation only supports one. Using the first entry");
		}

		joint_names_[j]            = transmissions[j].joints_[0].name_;
		joint_position_[j]         = 1.0;
		joint_velocity_[j]         = 0.0;
		joint_effort_[j]           = 1.0; // N/m for continous joints
		joint_effort_command_[j]   = 0.0;
		joint_position_command_[j] = 0.0;
		joint_velocity_command_[j] = 0.0;

		const std::string &hardware_interface = joint_interfaces.front();

		ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim",
		                       "Loading joint " << joint_names_[j] << " of type '" << hardware_interface << "'");

		// Create joint state interface for all joints
		js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[j], &joint_position_[j],
		                                                                  &joint_velocity_[j], &joint_effort_[j]));

		// Decide what kind of command interface this actuator/joint has
		hardware_interface::JointHandle joint_handle;
		if (hardware_interface == "EffortJointInterface" ||
		    hardware_interface == "hardware_interface/EffortJointInterface") {
			// Create effort joint interface
			joint_control_methods_[j] = EFFORT;
			joint_handle =
			    hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]), &joint_effort_command_[j]);
			ej_interface_.registerHandle(joint_handle);
		} else if (hardware_interface == "PositionJointInterface" ||
		           hardware_interface == "hardware_interface/PositionJointInterface") {
			// Create position joint interface
			joint_control_methods_[j] = POSITION;
			joint_handle =
			    hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]), &joint_position_command_[j]);
			pj_interface_.registerHandle(joint_handle);
		} else if (hardware_interface == "VelocityJointInterface" ||
		           hardware_interface == "hardware_interface/VelocityJointInterface") {
			// Create velocity interface
			joint_control_methods_[j] = VELOCITY;
			joint_handle =
			    hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]), &joint_velocity_command_[j]);
			vj_interface_.registerHandle(joint_handle);
		} else {
			ROS_FATAL_STREAM_NAMED("default_robot_hw_sim", "No matching hardware interface found for '"
			                                                   << hardware_interface << "' while loading interfaces for "
			                                                   << joint_names_[j]);
			return false;
		}

		if (hardware_interface == "EffortJointInterface" || hardware_interface == "PositionJointInterface" ||
		    hardware_interface == "VelocityJointInterface") {
			ROS_WARN_STREAM_NAMED("defautl_robot_hw_sim", "Deprecated syntax, please prepend 'hardware_interface/' to '"
			                                                  << hardware_interface
			                                                  << "' within the <hardwareInterface> tag in joint"
			                                                  << joint_names_[j]);
		}

		int joint_id = mujoco_ros::util::jointName2id(m_ptr.get(), joint_names_[j]);
		if (joint_id < 0) {
			ROS_ERROR_STREAM_NAMED("default_robot_hw_sim", "This robot has a joint named '"
			                                                   << joint_names_[j]
			                                                   << "' which is not in the mujoco model!");
			return false;
		}
		mujoco_joint_ids_[j] = (uint)joint_id;

		registerJointLimits(joint_names_[j], joint_handle, joint_control_methods_[j], joint_limit_nh, urdf_model,
		                    &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j], &joint_effort_limits_[j]);
		if (joint_control_methods_[j] != EFFORT) {
			// Initialize the PID controller
			const ros::NodeHandle nh(robot_namespace + "/mujoco_ros_control/pid_gains/" + joint_names_[j]);
			if (pid_controllers_[j].init(nh)) {
				switch (joint_control_methods_[j]) {
					case POSITION:
						joint_control_methods_[j] = POSITION_PID;
						break;
					case VELOCITY:
						joint_control_methods_[j] = VELOCITY_PID;
						break;
				}
			}

			else {
				// TODO: Set joint effort limits in joint struct?
				// setting limits directly on the mujoco joint apparently is not possible
			}
		}
	}

	// Register interface
	registerInterface(&js_interface_);
	registerInterface(&ej_interface_);
	registerInterface(&pj_interface_);
	registerInterface(&vj_interface_);

	// Initialize emergency stop code.
	e_stop_active_      = false;
	last_e_stop_active_ = false;

	return true;
}

void DefaultRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
	for (unsigned int j = 0; j < n_dof_; j++) {
		if (mujoco_joint_ids_[j] == -1)
			continue;

		double position, velocity, effort;
		getJointData(mujoco_joint_ids_[j], position, velocity, effort);
		if (joint_types_[j] == urdf::Joint::PRISMATIC) {
			joint_position_[j] = position;
		} else {
			joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], position);
		}
		joint_velocity_[j] = velocity;
		joint_effort_[j]   = effort;
	}
}

void DefaultRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
	// If the E-stop is active, joints controlled by position commands will maintain their positions.
	if (e_stop_active_) {
		if (!last_e_stop_active_) {
			last_joint_position_command_ = joint_position_command_;
			last_e_stop_active_          = true;
		}

		joint_position_command_ = last_joint_position_command_;
	} else {
		last_e_stop_active_ = false;
	}

	ej_sat_interface_.enforceLimits(period);
	ej_limits_interface_.enforceLimits(period);
	pj_sat_interface_.enforceLimits(period);
	pj_limits_interface_.enforceLimits(period);
	vj_sat_interface_.enforceLimits(period);
	vj_limits_interface_.enforceLimits(period);

	for (unsigned int j = 0; j < n_dof_; j++) {
		switch (joint_control_methods_[j]) {
			case EFFORT: {
				const double effort                         = e_stop_active_ ? 0 : joint_effort_command_[j];
				d_ptr_->qfrc_applied[m_ptr_->jnt_dofadr[j]] = effort;
				break;
			}

			case POSITION: {
				d_ptr_->qpos[m_ptr_->jnt_dofadr[j]]         = joint_position_command_[j];
				d_ptr_->qvel[m_ptr_->jnt_dofadr[j]]         = 0.;
				d_ptr_->qfrc_applied[m_ptr_->jnt_dofadr[j]] = 0.;
				break;
			}

			case POSITION_PID: {
				double error;
				switch (joint_types_[j]) {
					case urdf::Joint::REVOLUTE:
						angles::shortest_angular_distance_with_limits(joint_position_[j], joint_position_command_[j],
						                                              joint_lower_limits_[j], joint_upper_limits_[j], error);
						break;

					case urdf::Joint::CONTINUOUS:
						error = angles::shortest_angular_distance(joint_position_[j], joint_position_command_[j]);
						break;

					default:
						error = joint_position_command_[j] - joint_position_[j];
				}

				const double effort_limit = joint_effort_limits_[j];
				const double effort = clamp(pid_controllers_[j].computeCommand(error, period), -effort_limit, effort_limit);
				d_ptr_->qfrc_applied[m_ptr_->jnt_dofadr[j]] = effort;
				break;
			}

			case VELOCITY: {
				d_ptr_->qvel[m_ptr_->jnt_dofadr[j]]         = e_stop_active_ ? 0. : joint_velocity_command_[j];
				d_ptr_->qfrc_applied[m_ptr_->jnt_dofadr[j]] = 0.;
				break;
			}

			case VELOCITY_PID: {
				double error;

				if (e_stop_active_)
					error = -joint_velocity_[j];
				else
					error = joint_velocity_command_[j] - joint_velocity_[j];
				const double effort_limit = joint_effort_limits_[j];
				const double effort = clamp(pid_controllers_[j].computeCommand(error, period), -effort_limit, effort_limit);
				d_ptr_->qfrc_applied[m_ptr_->jnt_dofadr[j]] = effort;
				break;
			}
		}
	}
}

void DefaultRobotHWSim::eStopActive(const bool active)
{
	e_stop_active_ = active;
}

void DefaultRobotHWSim::getJointData(const int &joint_id, double &position, double &velocity, double &effort)
{
	position = d_ptr_->qpos[m_ptr_->jnt_qposadr[joint_id]];
	velocity = d_ptr_->qvel[m_ptr_->jnt_dofadr[joint_id]];
	effort   = d_ptr_->qfrc_applied[m_ptr_->jnt_dofadr[joint_id]];
}

void DefaultRobotHWSim::registerJointLimits(const std::string &joint_name,
                                            const hardware_interface::JointHandle &joint_handle,
                                            const ControlMethod ctrl_method, const ros::NodeHandle &joint_limit_nh,
                                            const urdf::Model *const urdf_model, int *const joint_type,
                                            double *const lower_limit, double *const upper_limit,
                                            double *const effort_limit)
{
	*joint_type   = urdf::Joint::UNKNOWN;
	*lower_limit  = -std::numeric_limits<double>::max();
	*upper_limit  = std::numeric_limits<double>::max();
	*effort_limit = std::numeric_limits<double>::max();

	joint_limits_interface::JointLimits limits;
	bool has_limits = false;
	joint_limits_interface::SoftJointLimits soft_limits;
	bool has_soft_limits = false;

	if (urdf_model != nullptr) {
		const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
		if (urdf_joint != nullptr) {
			*joint_type = urdf_joint->type;
			// Get limits from the URDF file
			if (joint_limits_interface::getJointLimits(urdf_joint, limits))
				has_limits = true;
			if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
				has_soft_limits = true;
		}
	}

	// Get limits from the parameter server
	if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
		has_limits = true;

	if (!has_limits)
		return;

	if (*joint_type == urdf::Joint::UNKNOWN) {
		// Infer joint type

		if (limits.has_position_limits) {
			*joint_type = urdf::Joint::REVOLUTE;
		} else {
			if (limits.angle_wraparound)
				*joint_type = urdf::Joint::CONTINUOUS;
			else
				*joint_type = urdf::Joint::PRISMATIC;
		}
	}

	if (limits.has_position_limits) {
		*lower_limit = limits.min_position;
		*upper_limit = limits.max_position;
	}
	if (limits.has_effort_limits) {
		*effort_limit = limits.max_effort;
	}

	if (has_soft_limits) {
		switch (ctrl_method) {
			case EFFORT: {
				const joint_limits_interface::EffortJointSoftLimitsHandle limits_handle(joint_handle, limits, soft_limits);
				ej_limits_interface_.registerHandle(limits_handle);
				break;
			}

			case POSITION: {
				const joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(joint_handle, limits,
				                                                                          soft_limits);
				pj_limits_interface_.registerHandle(limits_handle);
				break;
			}

			case VELOCITY: {
				const joint_limits_interface::VelocityJointSoftLimitsHandle limits_handle(joint_handle, limits,
				                                                                          soft_limits);
				vj_limits_interface_.registerHandle(limits_handle);
				break;
			}
		}
	} else {
		switch (ctrl_method) {
			case EFFORT: {
				const joint_limits_interface::EffortJointSaturationHandle sat_handle(joint_handle, limits);
				ej_sat_interface_.registerHandle(sat_handle);
				break;
			}

			case POSITION: {
				const joint_limits_interface::PositionJointSaturationHandle sat_handle(joint_handle, limits);
				pj_sat_interface_.registerHandle(sat_handle);
				break;
			}

			case VELOCITY: {
				const joint_limits_interface::VelocityJointSaturationHandle sat_handle(joint_handle, limits);
				vj_sat_interface_.registerHandle(sat_handle);
				break;
			}
		}
	}
}

} // namespace mujoco_ros::control

PLUGINLIB_EXPORT_CLASS(mujoco_ros::control::DefaultRobotHWSim, mujoco_ros::control::RobotHWSim)
