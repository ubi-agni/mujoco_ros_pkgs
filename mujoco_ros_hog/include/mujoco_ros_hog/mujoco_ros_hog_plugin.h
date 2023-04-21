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

#pragma once


#include <string>

#include <mujoco_ros/plugin_utils.h>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/mujoco_sim.h>
#include <mujoco_ros/plugin_utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>



namespace mujoco_ros_hog{

class MujocoHogPlugin: public MujocoSim::MujocoPlugin
{
public:
	~MujocoHogPlugin() = default;

	// Overload entry point
	bool load(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d) override;

	// Called on reset
	void reset() override;


	void controlCallback(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d);

	bool registerHog(MujocoSim::mjModelPtr m, std::string bodyname,std::vector<double> desiredPose={});

	void updateHog(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d);

protected:


	mjtNum last_time;
	std::map<std::string,std::vector<double>> hog_bodies_;
	boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;

};

} // namespace mujoco_ros_hog
