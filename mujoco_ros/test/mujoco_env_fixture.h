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
#include "test_util.h"

using namespace mujoco_ros;
namespace mju = ::mujoco::sample_util;

class MujocoEnvTestWrapper : public MujocoEnv
{
public:
	MujocoEnvTestWrapper(const std::string &admin_hash = std::string()) : MujocoEnv(admin_hash) {}
	mjModel *getModelPtr() { return model_.get(); }
	mjData *getDataPtr() { return data_.get(); }
	int getPendingSteps() { return num_steps_until_exit_; }

	std::string getFilename() { return { filename_ }; }
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
	std::unique_ptr<ros::NodeHandle> nh;

	void SetUp() override
	{
		nh = std::make_unique<ros::NodeHandle>("~");
		nh->setParam("unpause", true);
		nh->setParam("no_x", true);
		nh->setParam("use_sim_time", true);
	}

	void TearDown() override {}
};

class PendulumEnvFixture : public ::testing::Test
{
protected:
	std::unique_ptr<ros::NodeHandle> nh;
	MujocoEnvTestWrapper *env_ptr;

	void SetUp() override
	{
		nh = std::make_unique<ros::NodeHandle>("~");
		nh->setParam("unpause", false);
		nh->setParam("no_x", true);
		nh->setParam("use_sim_time", true);
		nh->setParam("sim_steps", -1);

		env_ptr = new MujocoEnvTestWrapper();

		std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/pendulum_world.xml";
		env_ptr->startWithXML(xml_path);

		float seconds = 0;
		while (env_ptr->getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";
	}

	void TearDown() override
	{
		env_ptr->shutdown();
		delete env_ptr;
	}
};

class EqualityEnvFixture : public ::testing::Test
{
protected:
	boost::shared_ptr<ros::NodeHandle> nh;
	MujocoEnvTestWrapper *env_ptr;

	virtual void SetUp()
	{
		nh.reset(new ros::NodeHandle("~"));
		nh->setParam("unpause", false);
		nh->setParam("no_x", true);
		nh->setParam("use_sim_time", true);
		nh->setParam("sim_steps", -1);

		// verify expected parameter array sizes
		EXPECT_EQ(mjNEQDATA, 11) << "This versions expects the maximum equality contraint parameters to be 11";
		EXPECT_EQ(mjNIMP, 5) << "This version expects the number of solimp parameters to be 5";
		EXPECT_EQ(mjNREF, 2) << "This version expects the number of solref parameters to be 2";

		// verify enum consistency
		EXPECT_EQ(mjEQ_CONNECT, mujoco_ros_msgs::EqualityConstraintType::CONNECT)
		    << "Mismatch between connect constraint types";
		EXPECT_EQ(mjEQ_WELD, mujoco_ros_msgs::EqualityConstraintType::WELD) << "Mismatch between weld constraint types";
		EXPECT_EQ(mjEQ_JOINT, mujoco_ros_msgs::EqualityConstraintType::JOINT)
		    << "Mismatch between joint constraint types";
		EXPECT_EQ(mjEQ_TENDON, mujoco_ros_msgs::EqualityConstraintType::TENDON)
		    << "Mismatch between tendon constraint types";

		env_ptr = new MujocoEnvTestWrapper();

		std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/equality_world.xml";
		env_ptr->startWithXML(xml_path);

		float seconds = 0;
		while (env_ptr->getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";
	}

	virtual void TearDown()
	{
		env_ptr->shutdown();
		delete env_ptr;
	}
};
