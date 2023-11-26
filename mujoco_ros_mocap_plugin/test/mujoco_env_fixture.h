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

/* Authors: David P. Leins */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <mujoco_ros/mujoco_env.h>

using namespace mujoco_ros;
namespace mju = ::mujoco::sample_util;

class MujocoEnvTestWrapper : public MujocoEnv
{
public:
	MujocoEnvTestWrapper(const std::string &admin_hash = std::string()) : MujocoEnv(admin_hash) {}
	mjModelPtr getModelPtr() { return model_; }
	mjDataPtr getDataPtr() { return data_; }
	int getPendingSteps() { return num_steps_until_exit_; }

	std::string getFilename() { return std::string(filename_); }
	int isPhysicsRunning() { return is_physics_running_; }
	int isEventRunning() { return is_event_running_; }
	int isRenderingRunning() { return is_rendering_running_; }

	int getNumCBReadyPlugins() { return cb_ready_plugins_.size(); }
	void notifyGeomChange() { notifyGeomChanged(0); }

	void shutdown()
	{
		settings_.exit_request = 1;
		waitForPhysicsJoin();
		waitForEventsJoin();
	}

	const std::string &getHandleNamespace() { return nh_->getNamespace(); }

	void startWithXML(const std::string &xml_path)
	{
		mju::strcpy_arr(queued_filename_, xml_path.c_str());
		settings_.load_request = 2;
		startPhysicsLoop();
		startEventLoop();
	}
};

class BaseEnvFixture : public ::testing::Test
{
protected:
	boost::shared_ptr<ros::NodeHandle> nh;

	void SetUp() override
	{
		nh.reset(new ros::NodeHandle("~"));
		nh->setParam("unpause", true);
		nh->setParam("no_x", true);
		nh->setParam("use_sim_time", true);
	}

	void TearDown() override {}
};
