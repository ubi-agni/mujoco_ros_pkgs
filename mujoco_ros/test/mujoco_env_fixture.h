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
#include <dynamic_reconfigure/server.h>
#include <mujoco_ros/SimParamsConfig.h>

using namespace mujoco_ros;
namespace mju = ::mujoco::sample_util;

class MujocoEnvTestWrapper : public MujocoEnv
{
public:
	MujocoEnvTestWrapper(const std::string &admin_hash = std::string()) : MujocoEnv(admin_hash) {}
	mjModel *getModelPtr() { return model_.get(); }
	mjData *getDataPtr() { return data_.get(); }
	MujocoEnvMutex *getMutexPtr() { return &physics_thread_mutex_; }
	dynamic_reconfigure::Server<mujoco_ros::SimParamsConfig> *getParamServer() { return param_server_; }
	int getPendingSteps() { return num_steps_until_exit_; }

	void setEvalMode(bool eval_mode) { settings_.eval_mode = eval_mode; }
	void setAdminHash(const std::string &hash) { mju::strcpy_arr(settings_.admin_hash, hash.c_str()); }

	std::string getFilename() { return { filename_ }; }
	int isPhysicsRunning() { return is_physics_running_; }
	int isEventRunning() { return is_event_running_; }
	int isRenderingRunning() { return is_rendering_running_; }

	OffscreenRenderContext *getOffscreenContext() { return &offscreen_; }

	int getNumCBReadyPlugins() { return cb_ready_plugins_.size(); }
	void notifyGeomChange() { notifyGeomChanged(0); }

	void load_queued_model()
	{
		settings_.load_request = 2;
		float seconds          = 0;
		while (getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		EXPECT_LT(seconds, 2) << "Model could not be loaded in time, ran into 2 second timeout!";
	}

	void load_filename(const std::string &filename)
	{
		mju::strcpy_arr(queued_filename_, filename.c_str());
		load_queued_model();
	}

	void shutdown()
	{
		settings_.exit_request = 1;
		waitForPhysicsJoin();
		waitForEventsJoin();
	}

	const std::string &getHandleNamespace() { return nh_->getNamespace(); }

	void startWithXML(const std::string &xml_path, bool wait = true, float timeout_secs = 2.)
	{
		mju::strcpy_arr(queued_filename_, xml_path.c_str());
		settings_.load_request = 2;
		startPhysicsLoop();
		startEventLoop();

		if (not wait)
			return;

		// Wait for model to be loaded
		float seconds = 0;
		while (getOperationalStatus() != 0 && seconds < timeout_secs) { // wait for model to be loaded or timeout
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
	}
};

class BaseEnvFixture : public ::testing::Test
{
protected:
	std::unique_ptr<ros::NodeHandle> nh;
	std::unique_ptr<MujocoEnvTestWrapper> env_ptr = nullptr;

	void SetUp() override
	{
		nh = std::make_unique<ros::NodeHandle>("~");
		nh->setParam("unpause", true);
		nh->setParam("no_render", true);
		nh->setParam("use_sim_time", true);
	}

	void TearDown() override
	{
		if (env_ptr != nullptr) {
			env_ptr->shutdown();
		}
		// clean up all parameters
		ros::param::del(nh->getNamespace());
	}
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
		nh->setParam("no_render", true);
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
		ASSERT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

		// Make sure forward has been run at least once
		{
			std::lock_guard<MujocoEnvMutex> lock(*env_ptr->getMutexPtr());
			mj_forward(env_ptr->getModelPtr(), env_ptr->getDataPtr());
		}
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
	mjModel *m;
	mjData *d;

	void SetUp() override
	{
		nh.reset(new ros::NodeHandle("~"));
		nh->setParam("unpause", false);
		nh->setParam("no_render", true);
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
		m = env_ptr->getModelPtr();
		d = env_ptr->getDataPtr();

		EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
		EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
		EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/set_eq_constraint_parameters", true))
		    << "Set eq constraints service should be available!";
	}

	void TearDown() override
	{
		env_ptr->shutdown();
		delete env_ptr;
	}
};
