/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2025, Bielefeld University
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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <py_binding_tools/ros_msg_typecasters.h>

#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/plugin_utils.h>
#include <mujoco_ros_msgs/MocapState.h>
#include <geometry_msgs/PoseStamped.h>

#include <mujoco_ros_mocap/mocap_plugin.h>

namespace py = pybind11;

namespace mujoco_ros::mocap {

class MocapPluginAccessor
{
public:
	static mujoco_ros_msgs::MocapState getLastMocapState(mujoco_ros::mocap::MocapPlugin &plugin)
	{
		return plugin.last_mocap_state_;
	}

	static mujoco_ros_msgs::MocapState *getLastMocapStatePtr(mujoco_ros::mocap::MocapPlugin &plugin)
	{
		return &plugin.last_mocap_state_;
	}

	static void setLastMocapState(mujoco_ros::mocap::MocapPlugin &plugin, const mujoco_ros_msgs::MocapState &state)
	{
		if (ValidateMocapMsg(state, plugin.m_)) {
			plugin.last_mocap_state_ = state;
		} else {
			ROS_ERROR("Invalid MocapState provided");
		}
	}

	static mujoco_ros_msgs::MocapState getCurrentMocapsAsMsg(mujoco_ros::mocap::MocapPlugin &plugin)
	{
		mujoco_ros_msgs::MocapState mocap_state;
		for (int i = 0; i < plugin.m_->nbody; ++i) {
			if (plugin.m_->body_mocapid[i] != -1) {
				geometry_msgs::PoseStamped pose_stamped;
				pose_stamped.header.frame_id    = "world"; // Assuming all mocap bodies are in the world frame
				pose_stamped.pose.position.x    = plugin.d_->xpos[3 * i];
				pose_stamped.pose.position.y    = plugin.d_->xpos[3 * i + 1];
				pose_stamped.pose.position.z    = plugin.d_->xpos[3 * i + 2];
				pose_stamped.pose.orientation.w = plugin.d_->xquat[4 * i];
				pose_stamped.pose.orientation.x = plugin.d_->xquat[4 * i + 1];
				pose_stamped.pose.orientation.y = plugin.d_->xquat[4 * i + 2];
				pose_stamped.pose.orientation.z = plugin.d_->xquat[4 * i + 3];
				mocap_state.name.push_back(mj_id2name(plugin.m_, mjOBJ_BODY, i));
				mocap_state.pose.push_back(pose_stamped);
			}
		}
		return mocap_state;
	}
};

} // namespace mujoco_ros::mocap

namespace mujoco_ros::python::mocap {
PYBIND11_MODULE(pymujoco_ros_mocap, m)
{
	py::module::import("mujoco_ros"); // Import mujoco_ros to ensure MujocoPlugin is registered

	py::class_<mujoco_ros::mocap::MocapPlugin, mujoco_ros::MujocoPlugin,
	           std::shared_ptr<mujoco_ros::mocap::MocapPlugin>>(m, "MujocoRosMocapPlugin")
	    .def(py::init<>())
	    .def("get_current_mocaps_as_msg", &mujoco_ros::mocap::MocapPluginAccessor::getCurrentMocapsAsMsg)
	    .def_property("mocap_state", &mujoco_ros::mocap::MocapPluginAccessor::getLastMocapStatePtr,
	                  &mujoco_ros::mocap::MocapPluginAccessor::setLastMocapState,
	                  py::return_value_policy::reference_internal);
}

} // namespace mujoco_ros::python::mocap
