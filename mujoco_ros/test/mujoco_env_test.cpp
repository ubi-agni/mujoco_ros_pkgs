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

#include "mujoco_env_fixture.h"

#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/common_types.h>
#include <mujoco_ros/util.h>

#include <ros/ros.h>
#include <chrono>

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_env_test");
	return RUN_ALL_TESTS();
}

using namespace mujoco_ros;
namespace mju = ::mujoco::sample_util;

// This needs to be listed first, otherwise the throw is not detected
TEST_F(BaseEnvFixture, EvalModeWithoutHashThrow)
{
	nh->setParam("eval_mode", true);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	EXPECT_THROW(MujocoEnvTestWrapper env, std::runtime_error);
	nh->setParam("eval_mode", false);
}

TEST_F(BaseEnvFixture, RunEvalMode)
{
	nh->setParam("eval_mode", true);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env("some_hash");

	env.startWithXML(xml_path);

	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_EQ(env.getFilename(), xml_path) << "Model was not loaded correctly!";
	EXPECT_FALSE(env.settings_.exit_request) << "Exit request is set before shutdown!";

	env.shutdown();
	nh->setParam("eval_mode", false);
}

TEST_F(BaseEnvFixture, EvalPauseWithHash)
{
	nh->setParam("eval_mode", true);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env("some_hash");

	env.startWithXML(xml_path);
	env.settings_.run = 1;

	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_EQ(env.getFilename(), xml_path) << "Model was not loaded correctly!";

	env.togglePaused(true, "some_hash");
	EXPECT_FALSE(env.settings_.run) << "Model should not be running!";

	env.shutdown();
	nh->setParam("eval_mode", false);
}

TEST_F(BaseEnvFixture, EvalPauseWithoutHashForbidden)
{
	nh->setParam("eval_mode", true);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env("some_hash");

	env.startWithXML(xml_path);
	env.settings_.run = 1;

	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_EQ(env.getFilename(), xml_path) << "Model was not loaded correctly!";

	env.togglePaused(true);
	EXPECT_TRUE(env.settings_.run) << "Model should still be running!";

	env.shutdown();
	nh->setParam("eval_mode", false);
}

TEST_F(BaseEnvFixture, EvalUnpauseWithHash)
{
	nh->setParam("eval_mode", true);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env("some_hash");

	env.startWithXML(xml_path);

	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_EQ(env.getFilename(), xml_path) << "Model was not loaded correctly!";

	env.togglePaused(false, "some_hash");
	EXPECT_TRUE(env.settings_.run) << "Model should be running!";

	env.shutdown();
	nh->setParam("eval_mode", false);
}

TEST_F(BaseEnvFixture, StepBeforeLoad)
{
	MujocoEnvTestWrapper env;
	EXPECT_FALSE(env.step(1));
}

TEST_F(BaseEnvFixture, StepAfterShutdown)
{
	MujocoEnvTestWrapper env;
	env.shutdown();
	EXPECT_FALSE(env.step(1));
}

TEST_F(BaseEnvFixture, StepWhileUnpaused)
{
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env;

	env.startWithXML(xml_path);
	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Model was not loaded correctly!";
	EXPECT_FALSE(env.step(1));

	env.shutdown();
}

TEST_F(BaseEnvFixture, StepSingleWhilePaused)
{
	nh->setParam("unpause", false);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env;

	env.startWithXML(xml_path);
	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Model was not loaded correctly!";
	EXPECT_DOUBLE_EQ(env.getDataPtr()->time, 0.0);
	EXPECT_TRUE(env.step(1));
	EXPECT_DOUBLE_EQ(env.getDataPtr()->time, env.getModelPtr()->opt.timestep);

	env.shutdown();
	nh->setParam("unpause", true);
}

TEST_F(BaseEnvFixture, StepMultiWhilePaused)
{
	nh->setParam("unpause", false);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env;

	env.startWithXML(xml_path);
	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Model was not loaded correctly!";
	EXPECT_DOUBLE_EQ(env.getDataPtr()->time, 0.0);
	EXPECT_TRUE(env.step(100));
	EXPECT_NEAR(env.getDataPtr()->time, 100 * env.getModelPtr()->opt.timestep, 1e-6);

	env.shutdown();
	nh->setParam("unpause", true);
}

TEST_F(BaseEnvFixture, StepUnblocked)
{
	nh->setParam("unpause", false);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env;

	env.startWithXML(xml_path);
	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Model was not loaded correctly!";
	EXPECT_DOUBLE_EQ(env.getDataPtr()->time, 0.0);
	EXPECT_TRUE(env.step(100, false));
	EXPECT_GT(env.settings_.env_steps_request, 0);

	seconds = 0;
	while (env.getDataPtr()->time < 100 * env.getModelPtr()->opt.timestep && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Time should have passed but ran into 2 seconds timeout!";

	env.shutdown();
	nh->setParam("unpause", true);
}

TEST_F(BaseEnvFixture, StepNegativeFail)
{
	nh->setParam("unpause", false);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env;

	env.startWithXML(xml_path);
	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Model was not loaded correctly!";
	EXPECT_DOUBLE_EQ(env.getDataPtr()->time, 0.0);
	EXPECT_FALSE(env.step(-10)) << "Stepping with negative steps should not succeed!";
	EXPECT_EQ(env.settings_.env_steps_request, 0);
	EXPECT_DOUBLE_EQ(env.getDataPtr()->time, 0.0);

	env.shutdown();
	nh->setParam("unpause", true);
}

TEST_F(BaseEnvFixture, Shutdown)
{
	MujocoEnvTestWrapper env;

	EXPECT_FALSE(env.isPhysicsRunning()) << "Physics thread should not be running yet!";
	EXPECT_FALSE(env.isEventRunning()) << "Event thread should not be running yet!";

	env.startPhysicsLoop();
	env.startEventLoop();

	EXPECT_FALSE(env.settings_.exit_request) << "Exit request is set before shutdown!";

	// Make sure the threads are running
	float seconds = 0;
	while (seconds < 2 && (!env.isPhysicsRunning() || !env.isEventRunning())) { // wait for threads to start
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
		seconds += 0.003;
	}
	EXPECT_TRUE(env.isPhysicsRunning()) << "Physics thread should have started by now!";
	EXPECT_TRUE(env.isEventRunning()) << "Event thread should have started by now!";

	env.settings_.exit_request = 1;

	seconds = 0;
	while (seconds < 2 && (env.isPhysicsRunning() || env.isEventRunning())) { // wait for threads to exit
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
		seconds += 0.003;
	}
	EXPECT_FALSE(env.isPhysicsRunning()) << "Physics thread is still running after shutdown!";
	EXPECT_FALSE(env.isEventRunning()) << "Event thread is still running after shutdown!";

	env.waitForEventsJoin();
	env.waitForPhysicsJoin();
}

TEST_F(BaseEnvFixture, InitWithModel)
{
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/pendulum_world.xml";
	MujocoEnvTestWrapper env;

	env.startWithXML(xml_path);

	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_EQ(env.getFilename(), xml_path) << "Model was not loaded correctly!";

	seconds = 0;
	while (env.getDataPtr()->time == 0 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005;
	}
	EXPECT_LT(seconds, 2) << "Time did not pass in simulation, ran into 2 second timeout!";

	env.shutdown();
}

TEST_F(BaseEnvFixture, EvalUnpauseWithoutHash)
{
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env("some_hash");

	env.startWithXML(xml_path);

	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_EQ(env.getFilename(), xml_path) << "Model was not loaded correctly!";

	env.togglePaused(false);
	EXPECT_TRUE(env.settings_.run) << "Model should be running!";

	env.shutdown();
}

TEST_F(BaseEnvFixture, PauseUnpause)
{
	nh->setParam("unpause", false);
	MujocoEnvTestWrapper env;

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	env.startWithXML(xml_path);

	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}

	EXPECT_FALSE(env.settings_.run) << "Model should not be running!";

	mjtNum time = env.getDataPtr()->time;

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	EXPECT_EQ(env.getDataPtr()->time, time) << "Time should not have changed in paused mode!";

	env.settings_.run.store(1);

	seconds = 0;
	while (env.getDataPtr()->time == time && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005;
	}
	EXPECT_LT(seconds, 2) << "Time should have been moving forward in unpaused state, ran into 2 seconds timeout!";

	env.shutdown();
	nh->setParam("unpause", true);
}

TEST_F(BaseEnvFixture, StepsTerminate)
{
	nh->setParam("num_steps", 100);

	MujocoEnvTestWrapper env;
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/pendulum_world.xml";
	env.startWithXML(xml_path);

	float seconds = 0;
	while (env.getOperationalStatus() > 1 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}

	seconds     = 0;
	int last    = env.getPendingSteps();
	int current = last;
	while (env.getPendingSteps() > 0) {
		current = env.getPendingSteps();
		if (current == last) { // wait for model to be loaded or timeout
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
			seconds += 0.002;
		} else {
			last = current;
		}
		if (seconds >= 2)
			break;
		seconds = 0.;
	}
	EXPECT_LT(seconds, 2) << "Pending steps should have decreased but ran into 2 seconds timeout";

	EXPECT_NEAR(env.getDataPtr()->time, env.getModelPtr()->opt.timestep * 100, env.getModelPtr()->opt.timestep * 0.1)
	    << "Time should have stopped after 100 steps";
	nh->deleteParam("num_steps");

	env.shutdown();
}

TEST_F(BaseEnvFixture, ManualSteps)
{
	nh->setParam("unpause", false);

	MujocoEnvTestWrapper env;

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/pendulum_world.xml";
	env.startWithXML(xml_path);

	while (env.getOperationalStatus() != 0) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
	}

	EXPECT_FALSE(env.settings_.env_steps_request) << "pending manual steps should be 0 after initialization!";
	EXPECT_FALSE(env.settings_.run) << "Model should not be running!";
	EXPECT_EQ(env.getDataPtr()->time, 0) << "Time should be 0 after initialization!";

	env.settings_.env_steps_request.store(1);

	float seconds = 0;
	while (env.settings_.env_steps_request != 0 && seconds < 1) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
		seconds += 0.002;
	}
	EXPECT_LT(seconds, 1) << "Manual step should have been executed but ran into 1 second timeout!";
	EXPECT_EQ(env.getDataPtr()->time, env.getModelPtr()->opt.timestep) << "Time should have been increased by one step!";

	env.settings_.run.store(1);
	env.settings_.env_steps_request.store(100);

	// Wait for time to pass
	std::this_thread::sleep_for(std::chrono::milliseconds(2));

	EXPECT_EQ(env.settings_.env_steps_request, 100) << "pending manual steps should not change in unpaused mode!";
	env.settings_.env_steps_request.store(0);
	env.settings_.run.store(0);

	mjtNum time = env.getDataPtr()->time;

	env.settings_.env_steps_request.store(100);

	seconds = 0;
	while (env.settings_.env_steps_request != 0 && seconds < 2) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005;
	}
	EXPECT_LT(seconds, 2) << "Manual step should have been executed but ran into 1 second timeout!";
	EXPECT_NEAR(env.getDataPtr()->time, time + 100 * env.getModelPtr()->opt.timestep,
	            env.getModelPtr()->opt.timestep * 0.1)
	    << "Time should have been increased by 100*timestep!";

	env.shutdown();
	nh->setParam("unpause", true);
}

TEST_F(BaseEnvFixture, Reset)
{
	nh->setParam("unpause", false);
	MujocoEnvTestWrapper env;
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/pendulum_world.xml";
	env.startWithXML(xml_path);
	while (env.getOperationalStatus() != 0) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
	}

	EXPECT_TRUE(env.step(100)) << "Stepping failed!";

	env.settings_.run = 0;
	EXPECT_NEAR(env.getDataPtr()->time, 100 * env.getModelPtr()->opt.timestep, 1e-6) << "Time should have been running!";

	env.settings_.reset_request.store(1);

	float seconds = 0;
	while (env.settings_.reset_request != 0 && seconds < 2) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
		seconds += 0.002;
	}
	EXPECT_LT(seconds, 2) << "Reset should have been executed but ran into 2 seconds timeout!";
	EXPECT_FALSE(env.settings_.run) << "Model should stay paused after reset!";
	EXPECT_NEAR(env.getDataPtr()->time, 0, 1e-6) << "Time should have been reset to 0!";

	env.settings_.run = 1;
	env.settings_.reset_request.store(1);
	while (env.settings_.reset_request != 0) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}
	EXPECT_TRUE(env.settings_.run) << "Model should keep running after reset!";

	env.settings_.run = 0;
	int id2           = mujoco_ros::util::jointName2id(env.getModelPtr(), "joint2");
	EXPECT_NE(id2, -1) << "joint2 should exist in model!";
	env.getDataPtr()->qpos[env.getModelPtr()->jnt_qposadr[id2]] = 0.5;
	env.getDataPtr()->qvel[env.getModelPtr()->jnt_dofadr[id2]]  = 0.1;
	env.settings_.reset_request.store(1);
	while (env.settings_.reset_request != 0) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}
	EXPECT_NE(env.getDataPtr()->qpos[id2], 0.5) << "joint2 position should have been reset!";
	EXPECT_NE(env.getDataPtr()->qvel[id2], 0.1) << "joint2 velocity should have been reset!";

	env.shutdown();
}

// Test reloading
TEST_F(BaseEnvFixture, Reload)
{
	nh->setParam("unpause", false);

	MujocoEnvTestWrapper env;
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	env.startWithXML(xml_path);

	while (env.getOperationalStatus() != 0) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
	}

	// Load same model again in unpaused state
	load_queued_model(env);
	EXPECT_FALSE(env.settings_.run) << "Model should stay paused on init!";
	EXPECT_EQ(env.getFilename(), xml_path) << "Wrong content in filename_!";
	EXPECT_EQ(env.getDataPtr()->time, 0) << "Time should have been reset to 0!";
	EXPECT_EQ(env.settings_.run, 0) << "Model should stay paused after reset!";

	// Load new model in paused state
	std::string xml_path2 = ros::package::getPath("mujoco_ros") + "/test/pendulum_world.xml";
	mju::strcpy_arr(env.queued_filename_, xml_path2.c_str());

	load_queued_model(env);
	EXPECT_EQ(env.getFilename(), xml_path2) << "Wrong content in filename_!";

	env.settings_.run.store(1);

	// Let some time pass
	while (env.getDataPtr()->time < 0.01) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	// Load same model in unpaused state
	load_queued_model(env);
	EXPECT_EQ(env.getFilename(), xml_path2) << "Wrong content in filename_!";

	// Let some time pass
	while (env.getDataPtr()->time < 0.01) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	env.shutdown();
	nh->setParam("unpause", false);
}
