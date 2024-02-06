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

/* Authors: David Leins, Julian Leichert*/

#pragma once

#include <string>

#include <mujoco_ros/plugin_utils.h>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/mujoco_env.h>

#include <mujoco_ros_msgs/MocapState.h>
#include <mujoco_ros_msgs/SetMocapState.h>

namespace mujoco_ros::mocap {

class MocapPlugin : public mujoco_ros::MujocoPlugin
{
public:
	~MocapPlugin() override = default;

	// Overload entry point
	bool load(const mjModel *m, mjData *d) override;
	// Called on reset
	void reset() override;

	void controlCallback(const mjModel *m, mjData *d) override;

private:
	void mocapStateCallback(const mujoco_ros_msgs::MocapState::ConstPtr &msg);
	bool mocapServiceCallback(mujoco_ros_msgs::SetMocapState::Request &req,
	                          mujoco_ros_msgs::SetMocapState::Response &resp);

	const mjModel *m_;
	mjData *d_;

	ros::Subscriber pose_subscriber_;
	ros::ServiceServer pose_service_;

	mujoco_ros_msgs::MocapState last_mocap_state_;
};

} // namespace mujoco_ros::mocap
