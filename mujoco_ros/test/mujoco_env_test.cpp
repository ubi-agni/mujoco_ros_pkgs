/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
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

#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/common_types.hpp>
#include <mujoco_ros/simulation_control_state.hpp>
#include <mujoco_ros/util.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>
#else // MJR_ROS_VERSION == ROS_2
#include <rclcpp/rclcpp.hpp>
#endif

int main(int argc, char **argv)
{
#if MJR_ROS_VERSION == ROS_1
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_env_test");

	// Uncomment to enable debug output (useful for debugging failing tests)
	// ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	// ros::console::notifyLoggerLevelsChanged();

	// Create spinner to communicate with ROS
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle nh;
	int ret = RUN_ALL_TESTS();

	// Stop spinner and shutdown ROS before returning
	spinner.stop();
	ros::shutdown();
#else // MJR_ROS_VERSION == ROS_2
	rclcpp::init(argc, argv);
	::testing::InitGoogleTest(&argc, argv);
	int ret = RUN_ALL_TESTS();
	rclcpp::shutdown();
#endif
	return ret;
}

using namespace mujoco_ros;
namespace mju = ::mujoco::sample_util;

class ControlStateTestWrapper : public MujocoEnvTestWrapper
{
public:
	using MujocoEnvTestWrapper::MujocoEnvTestWrapper;
	using MujocoEnvTestWrapper::RecordCompletedManualStep;
	using MujocoEnvTestWrapper::RequestReload;
	using MujocoEnvTestWrapper::RequestViewerReset;
	using MujocoEnvTestWrapper::RequestViewerShutdown;
	using MujocoEnvTestWrapper::SetPaused;
	using MujocoEnvTestWrapper::SetViewerRealTimeIndex;
};

TEST(SimulationControlStateTest, PauseAndRunSnapshot)
{
	SimulationControlState state;

	EXPECT_FALSE(state.Snapshot().running);
	state.SetPaused(false);
	EXPECT_TRUE(state.Snapshot().running);
	state.SetPaused(true);
	EXPECT_FALSE(state.Snapshot().running);
}

TEST(SimulationControlStateTest, RejectsInvalidManualStepRequests)
{
	SimulationControlState state;

	EXPECT_FALSE(state.RequestSteps(0));
	EXPECT_FALSE(state.RequestSteps(-1));

	state.SetPaused(false);
	EXPECT_FALSE(state.RequestSteps(1));
}

TEST(SimulationControlStateTest, RejectsOverlappingManualStepRequests)
{
	SimulationControlState state;

	ASSERT_TRUE(state.RequestSteps(3));
	EXPECT_FALSE(state.RequestSteps(1));
	EXPECT_EQ(state.Snapshot().pending_steps, 3);
}

TEST(SimulationControlStateTest, ManualStepProgressIsSnapshotBased)
{
	SimulationControlState state;

	ASSERT_TRUE(state.RequestSteps(2));
	EXPECT_EQ(state.Snapshot().pending_steps, 2);
	EXPECT_TRUE(state.RecordCompletedStep());
	EXPECT_EQ(state.Snapshot().pending_steps, 1);
	EXPECT_TRUE(state.RecordCompletedStep());
	EXPECT_EQ(state.Snapshot().pending_steps, 0);
	EXPECT_FALSE(state.RecordCompletedStep());
}

TEST(SimulationControlStateTest, UnpauseClearsPendingManualSteps)
{
	SimulationControlState state;

	ASSERT_TRUE(state.RequestSteps(3));
	state.SetPaused(false);

	const auto snapshot = state.Snapshot();
	EXPECT_TRUE(snapshot.running);
	EXPECT_EQ(snapshot.pending_steps, 0);
}

TEST(SimulationControlStateTest, LifecycleRequestsClearPendingManualSteps)
{
	SimulationControlState state;

	ASSERT_TRUE(state.RequestSteps(3));
	state.RequestReset();
	EXPECT_TRUE(state.Snapshot().reset_requested);
	EXPECT_EQ(state.Snapshot().pending_steps, 0);

	ASSERT_TRUE(state.RequestSteps(2));
	state.SetLoadRequest(2);
	EXPECT_EQ(state.Snapshot().load_request, 2);
	EXPECT_EQ(state.Snapshot().pending_steps, 0);

	ASSERT_TRUE(state.RequestSteps(1));
	state.RequestShutdown();
	EXPECT_TRUE(state.Snapshot().shutdown_requested);
	EXPECT_EQ(state.Snapshot().pending_steps, 0);
	EXPECT_FALSE(state.RequestSteps(1));
}

TEST(SimulationControlStateTest, ClearsLifecycleAndConsumesSpeedChange)
{
	SimulationControlState state;

	state.RequestReset();
	state.ClearResetRequest();
	EXPECT_FALSE(state.Snapshot().reset_requested);

	state.SetLoadRequest(2);
	state.SetLoadRequest(1);
	EXPECT_EQ(state.Snapshot().load_request, 1);
	state.SetLoadRequest(0);
	EXPECT_EQ(state.Snapshot().load_request, 0);

	state.MarkSpeedChanged();
	EXPECT_TRUE(state.Snapshot().speed_changed);
	EXPECT_TRUE(state.ConsumeSpeedChange());
	EXPECT_FALSE(state.Snapshot().speed_changed);
	EXPECT_FALSE(state.ConsumeSpeedChange());
}

TEST_F(BaseEnvFixture, ControlRequestsMirrorInternalLoadTransitions)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());

	sync_env->requestLoad(2);
	EXPECT_EQ(sync_env->GetControlSnapshot().load_request, 2);
	EXPECT_EQ(sync_env->settings_.load_request.load(), 2);

	sync_env->requestLoad(1);
	EXPECT_EQ(sync_env->GetControlSnapshot().load_request, 1);
	EXPECT_EQ(sync_env->settings_.load_request.load(), 1);

	sync_env->requestLoad(3);
	EXPECT_EQ(sync_env->GetControlSnapshot().load_request, 3);
	EXPECT_EQ(sync_env->settings_.load_request.load(), 3);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, ManualStepProgressDoesNotReapplyCompletedStepCount)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());

	sync_env->SetPaused(true);
	ASSERT_FALSE(sync_env->GetControlSnapshot().running);

	ASSERT_TRUE(sync_env->RequestManualSteps(2));
	sync_env->RecordCompletedManualStep();
	ASSERT_EQ(sync_env->GetControlSnapshot().pending_steps, 1);

	EXPECT_EQ(sync_env->GetControlSnapshot().pending_steps, 1);
	EXPECT_EQ(sync_env->settings_.env_steps_request.load(), 1);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, ViewerControlRequestsRouteThroughControlState)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());

	sync_env->SetPaused(false);
	EXPECT_TRUE(sync_env->GetControlSnapshot().running);
	EXPECT_TRUE(sync_env->settings_.run.load());

	sync_env->SetPaused(true);
	EXPECT_FALSE(sync_env->GetControlSnapshot().running);
	EXPECT_FALSE(sync_env->settings_.run.load());

	EXPECT_TRUE(sync_env->RequestManualSteps(2));
	EXPECT_EQ(sync_env->GetControlSnapshot().pending_steps, 2);
	EXPECT_EQ(sync_env->settings_.env_steps_request.load(), 2);

	sync_env->RequestViewerReset();
	EXPECT_TRUE(sync_env->GetControlSnapshot().reset_requested);
	EXPECT_TRUE(sync_env->settings_.reset_request.load());
	EXPECT_EQ(sync_env->GetControlSnapshot().pending_steps, 0);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, ViewerLoadAndShutdownRequestsRouteThroughControlState)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());

	sync_env->RequestReload();
	EXPECT_EQ(sync_env->GetControlSnapshot().load_request, 3);
	EXPECT_EQ(sync_env->settings_.load_request.load(), 3);

	sync_env->RequestViewerShutdown();
	EXPECT_TRUE(sync_env->GetControlSnapshot().shutdown_requested);
	EXPECT_TRUE(sync_env->settings_.exit_request.load());

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, ViewerSpeedChangesRouteThroughControlState)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());

	sync_env->SetViewerRealTimeIndex(3);
	EXPECT_EQ(sync_env->settings_.real_time_index, 3);
	EXPECT_TRUE(sync_env->GetControlSnapshot().speed_changed);
	EXPECT_TRUE(sync_env->settings_.speed_changed.load());

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, EvalModeWithoutHashThrow)
{
	MJR_WARN("###### [START] EvalModeWithoutHashThrow ######");
	nh->setParam("eval_mode", true);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	EXPECT_THROW(env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get()), std::runtime_error);
	MJR_WARN("###### [END] EvalModeWithoutHashThrow ######");
}

TEST_F(BaseEnvFixture, RunEvalMode)
{
	nh->setParam("eval_mode", true);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("some_hash", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";
	EXPECT_FALSE(env_ptr->GetControlSnapshot().shutdown_requested) << "Exit request is set before shutdown!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, EvalPauseWithHash)
{
	nh->setParam("eval_mode", true);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("some_hash", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

	env_ptr->togglePaused(true, "some_hash");
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should not be running!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, EvalPauseWithoutHashFails)
{
	nh->setParam("eval_mode", true);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("some_hash", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";
	EXPECT_TRUE(env_ptr->GetControlSnapshot().running) << "Model should start running!";

	EXPECT_FALSE(env_ptr->togglePaused(true)) << "Pause without admin hash should fail in eval mode!";
	EXPECT_TRUE(env_ptr->GetControlSnapshot().running) << "Model should keep running!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, EvalUnpauseWithHash)
{
	nh->setParam("eval_mode", true);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("some_hash", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

	env_ptr->togglePaused(false, "some_hash");
	EXPECT_TRUE(env_ptr->GetControlSnapshot().running) << "Model should be running!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, EvalUnpauseWithoutHash)
{
	nh->setParam("eval_mode", true);
	nh->setParam("unpause", false);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("some_hash", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should start paused!";

	EXPECT_TRUE(env_ptr->togglePaused(false)) << "Unpause without admin hash should succeed in eval mode!";
	EXPECT_TRUE(env_ptr->GetControlSnapshot().running) << "Model should be running!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, StepBeforeLoad)
{
	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	EXPECT_FALSE(env_ptr->step(1));
}

TEST_F(BaseEnvFixture, StepAfterShutdown)
{
	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	env_ptr->shutdown();
	EXPECT_FALSE(env_ptr->step(1));
}

TEST_F(BaseEnvFixture, StepWhileUnpaused)
{
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);
	EXPECT_FALSE(env_ptr->step(1));

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, StepSingleWhilePaused)
{
	nh->setParam("unpause", false);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);
	EXPECT_DOUBLE_EQ(env_ptr->getDataPtr()->time, 0.0);
	EXPECT_TRUE(env_ptr->step(1));
	EXPECT_DOUBLE_EQ(env_ptr->getDataPtr()->time, env_ptr->getModelPtr()->opt.timestep);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, StepMultiWhilePaused)
{
	nh->setParam("unpause", false);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);
	EXPECT_DOUBLE_EQ(env_ptr->getDataPtr()->time, 0.0);
	EXPECT_TRUE(env_ptr->step(100));
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 100 * env_ptr->getModelPtr()->opt.timestep, 1e-6);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, StepUnblocked)
{
	nh->setParam("unpause", false);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);
	EXPECT_DOUBLE_EQ(env_ptr->getDataPtr()->time, 0.0);
	EXPECT_TRUE(env_ptr->step(100, false));
	EXPECT_GT(env_ptr->GetControlSnapshot().pending_steps, 0);

	float seconds = 0;
	while (env_ptr->getDataPtr()->time < 100 * env_ptr->getModelPtr()->opt.timestep && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Time should have passed but ran into 2 seconds timeout!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, StepNegativeFail)
{
	nh->setParam("unpause", false);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);
	EXPECT_DOUBLE_EQ(env_ptr->getDataPtr()->time, 0.0);
	EXPECT_FALSE(env_ptr->step(-10)) << "Stepping with negative steps should not succeed!";
	EXPECT_EQ(env_ptr->GetControlSnapshot().pending_steps, 0);
	EXPECT_DOUBLE_EQ(env_ptr->getDataPtr()->time, 0.0);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Shutdown)
{
	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	EXPECT_FALSE(env_ptr->isPhysicsRunning()) << "Physics thread should not be running yet!";
	EXPECT_FALSE(env_ptr->isEventRunning()) << "Event thread should not be running yet!";

	env_ptr->StartPhysicsLoop();
	env_ptr->StartEventLoop();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().shutdown_requested) << "Exit request is set before shutdown!";

	// Make sure the threads are running
	float seconds = 0;
	while (seconds < 2 && (!env_ptr->isPhysicsRunning() || !env_ptr->isEventRunning())) { // wait for threads to start
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
		seconds += 0.003;
	}
	EXPECT_TRUE(env_ptr->isPhysicsRunning()) << "Physics thread should have started by now!";
	EXPECT_TRUE(env_ptr->isEventRunning()) << "Event thread should have started by now!";

	env_ptr->Shutdown();

	seconds = 0;
	while (seconds < 2 && (env_ptr->isPhysicsRunning() || env_ptr->isEventRunning())) { // wait for threads to exit
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
		seconds += 0.003;
	}
	EXPECT_FALSE(env_ptr->isPhysicsRunning()) << "Physics thread is still running after shutdown!";
	EXPECT_FALSE(env_ptr->isEventRunning()) << "Event thread is still running after shutdown!";

	env_ptr->WaitForEventsJoin();
	env_ptr->WaitForPhysicsJoin();
}

TEST_F(BaseEnvFixture, InitWithModel)
{
	std::string xml_path = testing::get_test_model_path("pendulum_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

	seconds = 0;
	while (env_ptr->getDataPtr()->time == 0 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005;
	}
	EXPECT_LT(seconds, 2) << "Time did not pass in simulation, ran into 2 second timeout!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, PauseUnpause)
{
	nh->setParam("unpause", false);
	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr->StartWithXML(xml_path);

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should not be running!";

	mjtNum time = env_ptr->getDataPtr()->time;

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	EXPECT_EQ(env_ptr->getDataPtr()->time, time) << "Time should not have changed in paused mode!";

	ASSERT_TRUE(env_ptr->togglePaused(false));

	float seconds = 0;
	while (env_ptr->getDataPtr()->time == time && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005;
	}
	EXPECT_LT(seconds, 2) << "Time should have been moving forward in unpaused state, ran into 2 seconds timeout!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, StepsTerminate)
{
	nh->setParam("num_steps", 100);

	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	std::string xml_path = testing::get_test_model_path("pendulum_world.xml");
	env_ptr->StartWithXML(xml_path);

	float seconds = 0;
	int current;
	int last = env_ptr->getPendingSteps();
	while (env_ptr->getPendingSteps() > 0) {
		current = env_ptr->getPendingSteps();
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

	EXPECT_NEAR(env_ptr->getDataPtr()->time, env_ptr->getModelPtr()->opt.timestep * 100,
	            env_ptr->getModelPtr()->opt.timestep * 0.1)
	    << "Time should have stopped after 100 steps";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, ManualSteps)
{
	nh->setParam("unpause", false);

	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	std::string xml_path = testing::get_test_model_path("pendulum_world.xml");
	env_ptr->StartWithXML(xml_path);

	EXPECT_FALSE(env_ptr->GetControlSnapshot().pending_steps)
	    << "pending manual steps should be 0 after initialization!";
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should not be running!";
	EXPECT_EQ(env_ptr->getDataPtr()->time, 0) << "Time should be 0 after initialization!";

	EXPECT_TRUE(env_ptr->Step(1, false));

	float seconds = 0;
	while (env_ptr->GetControlSnapshot().pending_steps != 0 && seconds < 1) { // wait for step completion
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
		seconds += 0.002;
	}
	EXPECT_LT(seconds, 1) << "Manual step should have been executed but ran into 1 second timeout!";
	EXPECT_EQ(env_ptr->getDataPtr()->time, env_ptr->getModelPtr()->opt.timestep)
	    << "Time should have been increased by one step!";

	EXPECT_TRUE(env_ptr->TogglePaused(false));
	EXPECT_FALSE(env_ptr->Step(100, false));

	// Wait for time to pass
	std::this_thread::sleep_for(std::chrono::milliseconds(2));

	EXPECT_EQ(env_ptr->GetControlSnapshot().pending_steps, 0)
	    << "pending manual steps should stay clear in unpaused mode!";
	EXPECT_TRUE(env_ptr->TogglePaused(true));

	mjtNum time = env_ptr->getDataPtr()->time;

	EXPECT_TRUE(env_ptr->Step(100, false));

	seconds = 0;
	while (env_ptr->GetControlSnapshot().pending_steps != 0 && seconds < 2) { // wait for step completion
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005;
	}
	EXPECT_LT(seconds, 2) << "Manual step should have been executed but ran into 1 second timeout!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, time + 100 * env_ptr->getModelPtr()->opt.timestep,
	            env_ptr->getModelPtr()->opt.timestep * 0.1)
	    << "Time should have been increased by 100*timestep!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Reset)
{
	nh->setParam("unpause", false);
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	std::string xml_path = testing::get_test_model_path("pendulum_world.xml");
	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->step(100)) << "Stepping failed!";

	ASSERT_TRUE(env_ptr->togglePaused(true));
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 100 * env_ptr->getModelPtr()->opt.timestep, 1e-6)
	    << "Time should have been running!";

	env_ptr->Reset();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should stay paused after reset!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Time should have been reset to 0!";

	ASSERT_TRUE(env_ptr->togglePaused(false));
	env_ptr->Reset();
	EXPECT_TRUE(env_ptr->GetControlSnapshot().running) << "Model should keep running after reset!";

	ASSERT_TRUE(env_ptr->togglePaused(true));
	int id2 = mujoco_ros::util::jointName2id(env_ptr->getModelPtr(), "joint2");
	EXPECT_NE(id2, -1) << "joint2 should exist in model!";
	env_ptr->getDataPtr()->qpos[env_ptr->getModelPtr()->jnt_qposadr[id2]] = 0.5;
	env_ptr->getDataPtr()->qvel[env_ptr->getModelPtr()->jnt_dofadr[id2]]  = 0.1;
	env_ptr->Reset();
	EXPECT_NE(env_ptr->getDataPtr()->qpos[id2], 0.5) << "joint2 position should have been reset!";
	EXPECT_NE(env_ptr->getDataPtr()->qvel[id2], 0.1) << "joint2 velocity should have been reset!";

	env_ptr->shutdown();
}

// Test reloading
TEST_F(BaseEnvFixture, Reload)
{
	nh->setParam("unpause", false);

	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr->StartWithXML(xml_path);

	// Load same model again in unpaused state
	env_ptr->load_queued_model();
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should stay paused on init!";
	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Wrong content in filename_!";
	EXPECT_EQ(env_ptr->getDataPtr()->time, 0) << "Time should have been reset to 0!";
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should stay paused after reset!";

	// Load new model in paused state
	std::string xml_path2 = testing::get_test_model_path("pendulum_world.xml");
	env_ptr->load_filename(xml_path2);
	EXPECT_EQ(env_ptr->getFilename(), xml_path2) << "Wrong content in filename_!";

	ASSERT_TRUE(env_ptr->togglePaused(false));

	// Let some time pass
	float seconds = 0;
	while (env_ptr->getDataPtr()->time < 0.01 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Simulation time did not advance before timeout!";

	// Load same model in unpaused state
	env_ptr->load_queued_model();
	EXPECT_EQ(env_ptr->getFilename(), xml_path2) << "Wrong content in filename_!";

	// Let some time pass
	seconds = 0;
	while (env_ptr->getDataPtr()->time < 0.01 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Simulation time did not advance before timeout!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, InitModelFromQueuedBuffer)
{
	// Create a MujocoEnv object
	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	// Set the queued model buffer
	std::string queuedFilename = "<mujoco/>";

	// Call the initModelFromQueue function
	env_ptr->StartWithXML(queuedFilename);

	// Check the result
	ASSERT_TRUE(env_ptr->getModelPtr());
	ASSERT_TRUE(env_ptr->getDataPtr());
	ASSERT_STREQ(env_ptr->getFilename().c_str(), queuedFilename.c_str());
	ASSERT_TRUE(env_ptr->sim_state_.model_valid);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, InitModelFromInvalidQueuedBuffer)
{
	// Create a MujocoEnv object
	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	// Set the queued model buffer
	std::string valid = "<mujoco/>";

	// Call the initModelFromQueue function
	env_ptr->StartWithXML(valid);

	std::string invalid = "<mujoco>";
	env_ptr->load_filename(invalid);

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) { // wait for model load attempt
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
		seconds += 0.003;
	}
	EXPECT_LT(seconds, 2) << "Invalid model load did not finish before timeout!";

	// Check the result
	ASSERT_TRUE(env_ptr->getModelPtr());
	ASSERT_TRUE(env_ptr->getDataPtr());
	ASSERT_STREQ(env_ptr->getFilename().c_str(), valid.c_str());
	ASSERT_FALSE(env_ptr->sim_state_.model_valid);

	env_ptr->shutdown();
}
