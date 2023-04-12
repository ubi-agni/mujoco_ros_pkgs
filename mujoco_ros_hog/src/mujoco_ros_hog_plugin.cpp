/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022, Bielefeld University
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

/* Authors: Julian Leichert*/
#include <mujoco_ros_hog/mujoco_ros_hog_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <vector>


namespace mujoco_ros_hog {


bool MujocoHogPlugin::load(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d)
{
	ROS_INFO_NAMED("HoG", "Hog initialized");

	// Check that ROS has been initialized
	if (!ros::isInitialized()) {
		ROS_FATAL_STREAM_NAMED("mujoco_ros_control",
		                       "A ROS node for Mujoco has not been initialized, unable to load plugin.");
		return false;
	}

	ROS_ASSERT(rosparam_config_.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	if (!rosparam_config_.hasMember("hogBodies")) {
		ROS_ERROR_NAMED("mujoco_ros_hog", "MujocoRosHogPlugin expects a 'hogBodies' rosparam specifying at least "
		                                      "one body");
		return false;
	}
	ROS_ASSERT(rosparam_config_["hogBodies"].getType() == XmlRpc::XmlRpcValue::TypeStruct);

	XmlRpc::XmlRpcValue::iterator itr;

	for (itr = rosparam_config_["hogBodies"].begin(); itr != rosparam_config_["hogBodies"].end(); ++itr) {

		registerHog(m,itr->first);
    }

	return true; 
	
}

void MujocoHogPlugin::controlCallback(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d)
{
}


bool MujocoHogPlugin::registerHog(MujocoSim::mjModelPtr m, std::string bodyname)
{
    int id = mj_name2id(m.get(), mjOBJ_XBODY, bodyname.c_str());
    // check if the body exists
    if (id == -1) {
        ROS_ERROR_STREAM_NAMED("mujoco_ros_hog","Could not find hog body with the name:"<<bodyname);
        return false;
    } else {
        ROS_INFO_STREAM_NAMED("mujoco_ros_hog","Registered hog body with the name:"<<bodyname);
        hog_bodies_.push_back(bodyname);
    }
    return true;
}

void MujocoHogPlugin::reset() {};
}//namespace mujoco_ros_hog

PLUGINLIB_EXPORT_CLASS(mujoco_ros_hog::MujocoHogPlugin, MujocoSim::MujocoPlugin)
