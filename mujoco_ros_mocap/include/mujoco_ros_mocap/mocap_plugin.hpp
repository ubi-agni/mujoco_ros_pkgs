/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2025, Bielefeld University
 *  All rights reserved.
 */

/* Authors: David Leins, Julian Leichert */

#pragma once

#include <mujoco_ros/ros_version.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>

#include <mujoco_ros/ros_one/plugin_utils.hpp>
#include <mujoco_ros_msgs/MocapState.h>
#include <mujoco_ros_msgs/SetMocapState.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <mujoco_ros/ros_two/plugin_utils.hpp>
#include <mujoco_ros_msgs/msg/mocap_state.hpp>
#include <mujoco_ros_msgs/srv/set_mocap_state.hpp>
#endif

#include <mujoco_ros/common_types.hpp>
#include <mujoco_ros/mujoco_env.hpp>

#include <memory>

namespace mujoco_ros::mocap {

#if MJR_ROS_VERSION == ROS_1
using MocapState         = mujoco_ros_msgs::MocapState;
using MocapStateConstPtr = mujoco_ros_msgs::MocapState::ConstPtr;
using SetMocapState      = mujoco_ros_msgs::SetMocapState;
#else
using MocapState         = mujoco_ros_msgs::msg::MocapState;
using MocapStateConstPtr = mujoco_ros_msgs::msg::MocapState::ConstSharedPtr;
using SetMocapState      = mujoco_ros_msgs::srv::SetMocapState;
#endif

class MocapPlugin : public mujoco_ros::MujocoPlugin
{
public:
	~MocapPlugin() override = default;

	bool Load(const mjModel *m, mjData *d) override;
	void Reset() override;

	void ControlCallback(const mjModel *m, mjData *d) override;

private:
	void MocapStateCallback(const MocapStateConstPtr &msg);

#if MJR_ROS_VERSION == ROS_1
	bool MocapServiceCallback(SetMocapState::Request &req, SetMocapState::Response &resp);
#else
	void MocapServiceCallback(const std::shared_ptr<SetMocapState::Request> req,
	                          std::shared_ptr<SetMocapState::Response> resp);
#endif

	const mjModel *m_{ nullptr };
	mjData *d_{ nullptr };

#if MJR_ROS_VERSION == ROS_1
	ros::Subscriber pose_subscriber_;
	ros::ServiceServer pose_service_;
#else
	rclcpp::Subscription<MocapState>::SharedPtr pose_subscriber_;
	rclcpp::Service<SetMocapState>::SharedPtr pose_service_;
#endif

	MocapState last_mocap_state_;
};

} // namespace mujoco_ros::mocap
