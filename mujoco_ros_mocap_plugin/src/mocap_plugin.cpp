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

/* Authors: David Leins */

#include <geometry_msgs/PoseStamped.h>
#include <mujoco/mujoco.h>

#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <vector>

#include <mujoco_ros_mocap/mocap_plugin.h>

namespace mujoco_ros::mocap {

bool validateMocapMsg(const mujoco_ros_msgs::MocapState &msg, const mjModel *m)
{
	for (int idx = 0; idx < msg.pose.size(); idx++) {
		if (msg.pose[idx].header.frame_id != "world" && !msg.pose[idx].header.frame_id.empty()) {
			ROS_ERROR_STREAM("mocap plugin expects poses in world frame, but got pose in frame "
			                 << msg.pose[idx].header.frame_id);
			return false;
		} else {
			int body_id = mj_name2id(m, mjOBJ_BODY, msg.name[idx].c_str());
			if (body_id == -1) {
				ROS_ERROR_STREAM("mocap plugin got pose for unknown body " << msg.name[idx]);
				return false;
			}
			if (m->body_mocapid[body_id] == -1) {
				ROS_ERROR_STREAM("mocap plugin got pose for body " << msg.name[idx] << " which is not a mocap body");
			}
		}
	}

	return true;
}

void MocapPlugin::mocapStateCallback(const mujoco_ros_msgs::MocapState::ConstPtr &msg)
{
	ROS_DEBUG("Got target poses");
	if (!validateMocapMsg(*msg, m_))
		return;
	last_mocap_state_ = mujoco_ros_msgs::MocapState(*msg);
}

void MocapPlugin::controlCallback(const mjModel *m, mjData *d)
{
	for (int idx = 0; idx < last_mocap_state_.pose.size(); idx++) {
		int bodyid = mj_name2id(m, mjOBJ_BODY, last_mocap_state_.name[idx].c_str());

		if (bodyid == -1)
			return;
		int mocap_data_id = m->body_mocapid[bodyid];
		if (mocap_data_id == -1)
			return;

		mjtNum mocap_pose[9] = {
			last_mocap_state_.pose[idx].pose.position.x,    last_mocap_state_.pose[idx].pose.position.y,
			last_mocap_state_.pose[idx].pose.position.z,    last_mocap_state_.pose[idx].pose.orientation.w,
			last_mocap_state_.pose[idx].pose.orientation.x, last_mocap_state_.pose[idx].pose.orientation.y,
			last_mocap_state_.pose[idx].pose.orientation.z
		};

		// normalize quaternion to be sure
		mju_normalize4(mocap_pose + 3);

		// set position and quaternion
		mju_copy3(d->mocap_pos + mocap_data_id * 3, mocap_pose);
		mju_copy4(d->mocap_quat + mocap_data_id * 4, mocap_pose + 3);
	}
}

bool MocapPlugin::mocapServiceCallback(mujoco_ros_msgs::SetMocapState::Request &req,
                                       mujoco_ros_msgs::SetMocapState::Response &resp)
{
	if (!validateMocapMsg(req.mocap_state, m_)) {
		resp.success = false;
		return true;
	}
	last_mocap_state_ = req.mocap_state;
	resp.success      = true;
	return true;
}

bool MocapPlugin::load(const mjModel *m, mjData *d)
{
	// Check that ROS has been initialized
	if (!ros::isInitialized()) {
		ROS_FATAL("A ROS node for Mujoco has not been initialized, unable to load plugin.");
		return false;
	}

	m_ = m;
	d_ = d;

	pose_subscriber_ = node_handle_.subscribe("mocap_poses", 1, &MocapPlugin::mocapStateCallback, this);
	pose_service_    = node_handle_.advertiseService("set_mocap_state", &MocapPlugin::mocapServiceCallback, this);
	ROS_INFO("Mocap plugin initialized");
	return true;
}

void MocapPlugin::reset(){};
} // namespace mujoco_ros::mocap

PLUGINLIB_EXPORT_CLASS(mujoco_ros::mocap::MocapPlugin, mujoco_ros::MujocoPlugin)
