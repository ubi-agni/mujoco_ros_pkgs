/*********************************************************************
 * Software License Agreement (BSD License)
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

/* Authors: David P. Leins*/

#pragma once

// ros control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// ros
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// mujoco_ros_control
#include <mujoco_ros_control/robot_hw_sim.h>

// URDF
#include <urdf/model.h>

namespace mujoco_ros::control {
class DefaultRobotHWSim : public mujoco_ros::control::RobotHWSim
{
public:
	bool initSim(const mjModel *m_ptr, mjData *d_ptr, mujoco_ros::MujocoEnv *mujoco_env_ptr,
	             const std::string &robot_namespace, ros::NodeHandle model_nh, const urdf::Model *const urdf_model,
	             std::vector<transmission_interface::TransmissionInfo> transmissions) override;

	void readSim(ros::Time time, ros::Duration period) override;

	void writeSim(ros::Time time, ros::Duration period) override;

	void eStopActive(const bool Active) override;

protected:
	// Methods used to control a joint.
	enum ControlMethod
	{
		EFFORT,
		POSITION,
		POSITION_PID,
		VELOCITY,
		VELOCITY_PID
	};

	void getJointData(const int &joint_id, double &position, double &velocity, double &effort);

	/**
	 * Register the limits of the joint specified by joint_name and joint_handle. The limits are
	 * retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
	 * Return the joints type, lower position limit, upper position limit, and effort limit.
	 */
	void registerJointLimits(const std::string &joint_name, const hardware_interface::JointHandle &joint_handle,
	                         const ControlMethod ctrl_method, const ros::NodeHandle &joint_limit_nh,
	                         const urdf::Model *const urdf_model, int *const joint_type, double *const lower_limit,
	                         double *const upper_limit, double *const effort_limit);

	unsigned int n_dof_;

	hardware_interface::JointStateInterface js_interface_;
	hardware_interface::EffortJointInterface ej_interface_;
	hardware_interface::PositionJointInterface pj_interface_;
	hardware_interface::VelocityJointInterface vj_interface_;

	joint_limits_interface::EffortJointSaturationInterface ej_sat_interface_;
	joint_limits_interface::EffortJointSoftLimitsInterface ej_limits_interface_;
	joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
	joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
	joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
	joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

	std::vector<std::string> joint_names_;
	std::vector<int> joint_types_;
	std::vector<double> joint_lower_limits_;
	std::vector<double> joint_upper_limits_;
	std::vector<double> joint_effort_limits_;

	std::vector<ControlMethod> joint_control_methods_;
	std::vector<control_toolbox::Pid> pid_controllers_;
	std::vector<double> joint_position_;
	std::vector<double> joint_velocity_;
	std::vector<double> joint_effort_;
	std::vector<double> joint_effort_command_;
	std::vector<double> joint_position_command_;
	std::vector<double> last_joint_position_command_;
	std::vector<double> joint_velocity_command_;

	std::vector<uint> mujoco_joint_ids_;

	bool e_stop_active_, last_e_stop_active_;
};

using DefaultRobotHWSimPtr = std::unique_ptr<DefaultRobotHWSim>;
} // namespace mujoco_ros::control
