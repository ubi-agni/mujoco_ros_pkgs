/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2025, Bielefeld University
 *  All rights reserved.
 */

/* Authors: David Leins, Julian Leichert */

#include <mujoco_ros_mocap/mocap_plugin.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <pluginlib/class_list_macros.h>
#else
#include <pluginlib/class_list_macros.hpp>
#endif

#include <mujoco_ros/logging.hpp>

#include <functional>
#include <string>

namespace mujoco_ros::mocap {

namespace {
bool ValidateMocapMsg(const MocapState &msg, const mjModel *model)
{
	if (msg.name.size() != msg.pose.size()) {
		MJR_ERROR_STREAM("mocap plugin expected the same number of names and poses, but got "
		                 << msg.name.size() << " names and " << msg.pose.size() << " poses");
		return false;
	}

	for (std::size_t idx = 0; idx < msg.pose.size(); ++idx) {
		if (msg.pose[idx].header.frame_id != "world" && !msg.pose[idx].header.frame_id.empty()) {
			MJR_ERROR_STREAM("mocap plugin expects poses in world frame, but got pose in frame "
			                 << msg.pose[idx].header.frame_id);
			return false;
		}

		const int body_id = mj_name2id(model, mjOBJ_BODY, msg.name[idx].c_str());
		if (body_id == -1) {
			MJR_ERROR_STREAM("mocap plugin got pose for unknown body " << msg.name[idx]);
			return false;
		}
		if (model->body_mocapid[body_id] == -1) {
			MJR_ERROR_STREAM("mocap plugin got pose for body " << msg.name[idx] << " which is not a mocap body");
			return false;
		}
	}

	return true;
}
} // namespace

void MocapPlugin::MocapStateCallback(const MocapStateConstPtr &msg)
{
	MJR_DEBUG("Got target poses");
	if (!ValidateMocapMsg(*msg, m_)) {
		return;
	}
	last_mocap_state_ = *msg;
}

void MocapPlugin::ControlCallback(const mjModel *model, mjData *data)
{
	for (std::size_t idx = 0; idx < last_mocap_state_.pose.size(); ++idx) {
		const int bodyid = mj_name2id(model, mjOBJ_BODY, last_mocap_state_.name[idx].c_str());

		if (bodyid == -1) {
			return;
		}
		const int mocap_data_id = model->body_mocapid[bodyid];
		if (mocap_data_id == -1) {
			return;
		}

		mjtNum mocap_pose[7] = {
			last_mocap_state_.pose[idx].pose.position.x,    last_mocap_state_.pose[idx].pose.position.y,
			last_mocap_state_.pose[idx].pose.position.z,    last_mocap_state_.pose[idx].pose.orientation.w,
			last_mocap_state_.pose[idx].pose.orientation.x, last_mocap_state_.pose[idx].pose.orientation.y,
			last_mocap_state_.pose[idx].pose.orientation.z
		};

		mju_normalize4(mocap_pose + 3);

		mju_copy3(data->mocap_pos + mocap_data_id * 3, mocap_pose);
		mju_copy4(data->mocap_quat + mocap_data_id * 4, mocap_pose + 3);
	}
}

#if MJR_ROS_VERSION == ROS_1
bool MocapPlugin::MocapServiceCallback(SetMocapState::Request &req, SetMocapState::Response &resp)
{
	if (!ValidateMocapMsg(req.mocap_state, m_)) {
		resp.success = false;
		return true;
	}
	last_mocap_state_ = req.mocap_state;
	resp.success      = true;
	return true;
}
#else
void MocapPlugin::MocapServiceCallback(const std::shared_ptr<SetMocapState::Request> req,
                                       std::shared_ptr<SetMocapState::Response> resp)
{
	if (!ValidateMocapMsg(req->mocap_state, m_)) {
		resp->success = false;
		return;
	}
	last_mocap_state_ = req->mocap_state;
	resp->success     = true;
}
#endif

bool MocapPlugin::Load(const mjModel *model, mjData *data)
{
#if MJR_ROS_VERSION == ROS_1
	if (!ros::isInitialized()) {
		MJR_FATAL("A ROS node for Mujoco has not been initialized, unable to load plugin.");
		return false;
	}
#endif

	m_ = model;
	d_ = data;

#if MJR_ROS_VERSION == ROS_1
	pose_subscriber_ = node_handle_.subscribe("mocap_poses", 1, &MocapPlugin::MocapStateCallback, this);
	pose_service_    = node_handle_.advertiseService("set_mocap_state", &MocapPlugin::MocapServiceCallback, this);
#else
	auto node        = get_node();
	pose_subscriber_ = node->create_subscription<MocapState>(
	    "mocap_poses", rclcpp::QoS(1), std::bind(&MocapPlugin::MocapStateCallback, this, std::placeholders::_1));
	pose_service_ =
	    node->create_service<SetMocapState>("set_mocap_state", std::bind(&MocapPlugin::MocapServiceCallback, this,
	                                                                     std::placeholders::_1, std::placeholders::_2));
#endif

	MJR_INFO("Mocap plugin initialized");
	return true;
}

void MocapPlugin::Reset()
{
	last_mocap_state_ = MocapState();
}

} // namespace mujoco_ros::mocap

PLUGINLIB_EXPORT_CLASS(mujoco_ros::mocap::MocapPlugin, mujoco_ros::MujocoPlugin)
