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

#include <cmath>
#include <chrono>
#include <thread>

#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>
#include <mujoco_ros_testing_utils/test_util.hpp>
#include <mujoco_ros/util.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mujoco_ros/SimParamsConfig.h>
#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/util.hpp>
#include <mujoco_ros_msgs/GetBodyState.h>
#include <mujoco_ros_msgs/GeomType.h>
#include <mujoco_ros_msgs/SetBodyState.h>
#include <mujoco_ros_msgs/SetGeomProperties.h>
#include <mujoco_ros_msgs/SetPause.h>
#include <mujoco_ros_msgs/StepAction.h>
#include <mujoco_ros_msgs/StepGoal.h>
#include <ros/ros.h>
#else // MJR_ROS_VERSION == ROS_2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/empty.hpp>

#include <mujoco_ros_msgs/action/step.hpp>

#include <mujoco_ros_msgs/msg/equality_constraint_parameters.hpp>
#include <mujoco_ros_msgs/msg/equality_constraint_type.hpp>
#include <mujoco_ros_msgs/msg/geom_type.hpp>

#include <mujoco_ros_msgs/srv/get_body_state.hpp>
#include <mujoco_ros_msgs/srv/get_equality_constraint_parameters.hpp>
#include <mujoco_ros_msgs/srv/get_geom_properties.hpp>
#include <mujoco_ros_msgs/srv/get_gravity.hpp>
#include <mujoco_ros_msgs/srv/get_sim_info.hpp>
#include <mujoco_ros_msgs/srv/get_state_uint.hpp>
#include <mujoco_ros_msgs/srv/reload.hpp>
#include <mujoco_ros_msgs/srv/set_body_state.hpp>
#include <mujoco_ros_msgs/srv/set_equality_constraint_parameters.hpp>
#include <mujoco_ros_msgs/srv/set_float.hpp>
#include <mujoco_ros_msgs/srv/set_geom_properties.hpp>
#include <mujoco_ros_msgs/srv/set_gravity.hpp>
#include <mujoco_ros_msgs/srv/set_pause.hpp>

namespace std_srvs {
using Empty = testing::ServiceCall<std_srvs::srv::Empty>;
}

namespace mujoco_ros_msgs {
using SetPause                        = testing::ServiceCall<mujoco_ros_msgs::srv::SetPause>;
using Reload                          = testing::ServiceCall<mujoco_ros_msgs::srv::Reload>;
using SetBodyState                    = testing::ServiceCall<mujoco_ros_msgs::srv::SetBodyState>;
using GetBodyState                    = testing::ServiceCall<mujoco_ros_msgs::srv::GetBodyState>;
using SetGeomProperties               = testing::ServiceCall<mujoco_ros_msgs::srv::SetGeomProperties>;
using GetGeomProperties               = testing::ServiceCall<mujoco_ros_msgs::srv::GetGeomProperties>;
using SetEqualityConstraintParameters = testing::ServiceCall<mujoco_ros_msgs::srv::SetEqualityConstraintParameters>;
using GetEqualityConstraintParameters = testing::ServiceCall<mujoco_ros_msgs::srv::GetEqualityConstraintParameters>;
using SetGravity                      = testing::ServiceCall<mujoco_ros_msgs::srv::SetGravity>;
using GetGravity                      = testing::ServiceCall<mujoco_ros_msgs::srv::GetGravity>;
using GetStateUint                    = testing::ServiceCall<mujoco_ros_msgs::srv::GetStateUint>;
using SetFloat                        = testing::ServiceCall<mujoco_ros_msgs::srv::SetFloat>;
using GetSimInfo                      = testing::ServiceCall<mujoco_ros_msgs::srv::GetSimInfo>;
using GeomType                        = mujoco_ros_msgs::msg::GeomType;
using EqualityConstraintType          = mujoco_ros_msgs::msg::EqualityConstraintType;
using EqualityConstraintParameters    = mujoco_ros_msgs::msg::EqualityConstraintParameters;
} // namespace mujoco_ros_msgs
#endif

int main(int argc, char **argv)
{
#if MJR_ROS_VERSION == ROS_1
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_ros_interface_test");

	// Uncomment to enable debug output (useful for debugging failing tests)
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	ros::console::notifyLoggerLevelsChanged();

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

#if MJR_ROS_VERSION == ROS_1

double current_ros_time(MujocoEnvTestWrapper * /*env_ptr*/ = nullptr)
{
	return ros::Time::now().toSec();
}

#else // MJR_ROS_VERSION == ROS_2

double current_ros_time(MujocoEnvTestWrapper *env_ptr = nullptr)
{
	return env_ptr->get_clock()->now().seconds();
}

#endif

void compare_current_ros_time(double expected, const std::string &msg, MujocoEnvTestWrapper *env_ptr = nullptr,
                              double tol = 1e-6)
{
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
	double actual       = current_ros_time(env_ptr);
	while (std::abs(actual - expected) > tol && std::chrono::steady_clock::now() < deadline) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		actual = current_ros_time(env_ptr);
	}
	EXPECT_NEAR(actual, expected, tol) << msg;
}

TEST_F(PendulumEnvFixture, Clock)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0., 1e-6) << "Simulation time should be 0.0!";
	int total_steps = 0;

	EXPECT_TRUE(env_ptr->step()) << "Step did not succeed!";
	total_steps = 1;
	compare_current_ros_time(env_ptr->getModelPtr()->opt.timestep,
	                         std::string("Simulation time should have increased by 1 step!"), env_ptr.get());

	EXPECT_TRUE(env_ptr->step(10)) << "Stepping did not succeed!";
	total_steps += 10;
	compare_current_ros_time(env_ptr->getModelPtr()->opt.timestep * total_steps,
	                         std::string("Simulation time should have increased by 10 steps!"), env_ptr.get());

	EXPECT_FALSE(env_ptr->step(-10)) << "Stepping with negative steps should not succeed!";
	compare_current_ros_time(env_ptr->getModelPtr()->opt.timestep * total_steps,
	                         std::string("Simulation time should have increased by 10 steps!"), env_ptr.get());
}

TEST_F(PendulumEnvFixture, ShutdownCallback)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/shutdown"))
	    << "Shutdown service should be available!";

	std_srvs::Empty srv;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/shutdown", srv))
	    << "Shutdown service call failed!";
	MJR_WARN_STREAM("Called shutdown service on topic "
	                << env_ptr->GetHandleNamespace() + "/shutdown, waiting for shutdown to complete...");
	float seconds = 0;
	while ((env_ptr->isEventRunning() || env_ptr->isPhysicsRunning()) && seconds < 2) { // wait for shutdown
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_FALSE(env_ptr->isEventRunning()) << "Event loop should be stopped!";
	EXPECT_FALSE(env_ptr->isPhysicsRunning()) << "Physics loop should be stopped!";
}

TEST_F(PendulumEnvFixture, PauseCallback)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_pause"))
	    << "Pause service should be available!";
	mujoco_ros_msgs::SetPause srv;

	srv.request.paused = false;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_pause", srv))
	    << "unpause service call failed!";

	float seconds = 0;
	while (seconds < 2 && !env_ptr->GetControlSnapshot().running) { // wait for unpause
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_TRUE(env_ptr->GetControlSnapshot().running) << "Simulation should be running!";

	srv.request.paused = true;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_pause", srv))
	    << "pause service call failed!";
	seconds = 0;
	while (seconds < 2 && env_ptr->GetControlSnapshot().running) { // wait for pause
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
}

TEST_F(PendulumEnvFixture, ReloadStringTooLong)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/reload"))
	    << "Reload service should be available!";

	mujoco_ros_msgs::Reload srv;
	srv.request.model = std::string(env_ptr->kMaxFilenameLength + 1, 'a');
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/reload", srv))
	    << "Reload service call failed!";
	EXPECT_FALSE(srv.response.success) << "Reload service should fail!";
}

TEST_F(PendulumEnvFixture, ReloadSameModelCallback)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/reload"))
	    << "Reload service should be available!";

	mujoco_ros_msgs::Reload srv;

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/reload", srv))
	    << "Reload service call failed!";
	EXPECT_TRUE(srv.response.success) << "Reload service should succeed!";
	float seconds = 0;
	while (seconds < 2 && env_ptr->GetOperationalStatus() > 0) { // wait for reload to finish
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_EQ(env_ptr->GetOperationalStatus(), 0) << "Operational status should be 0!";
}

TEST_F(PendulumEnvFixture, ReloadNewModelCallback)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/reload"))
	    << "Reload service should be available!";

	mujoco_ros_msgs::Reload srv;
	std::string model_xml = testing::get_test_model_path("empty_world.xml");
	srv.request.model     = model_xml;

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/reload", srv))
	    << "Reload service call failed!";
	float seconds = 0;
	while (seconds < 2 && env_ptr->GetOperationalStatus() > 0) { // wait for reload to finish
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_EQ(env_ptr->GetOperationalStatus(), 0) << "Operational status should be 0!";
	EXPECT_EQ(env_ptr->getFilename(), model_xml) << "New model should be loaded!";
}

TEST_F(PendulumEnvFixture, ResetCallback)
{
	ASSERT_TRUE(env_ptr->togglePaused(false));

	env_ptr->step(100);

	ASSERT_TRUE(env_ptr->togglePaused(true));

	// Make sure reset service is available
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/reset"))
	    << "Reset service should be available!";
	std_srvs::Empty srv;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/reset", srv))
	    << "Reset service call failed!";

	float seconds = 0;
	while (seconds < 2 && env_ptr->GetOperationalStatus() > 0) { // wait for reset to finish
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Reset callback took too long!";

	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0.0, 1e-6) << "Simulation time should be reset to 0.0!";
}

TEST_F(PendulumEnvFixture, StepGoalSingle)
{
#if MJR_ROS_VERSION == ROS_1
	auto topics = testing::get_available_topics_for_test(env_ptr);

	const bool found = testing::has_topic(topics, env_ptr->GetHandleNamespace() + "/step/result");
	// Workaround to connect to action server, this is only needed in cpp
	ASSERT_TRUE(env_ptr->togglePaused(false));
	EXPECT_TRUE(found) << "Step action should be available!";
	ros::spinOnce();
	actionlib::SimpleActionClient<mujoco_ros_msgs::StepAction> ac(env_ptr->GetHandleNamespace() + "/step", true);
	ASSERT_TRUE(env_ptr->togglePaused(true));

	// Wait for paused state to be applied
	std::this_thread::sleep_for(std::chrono::milliseconds(5));

	mjtNum time = env_ptr->getDataPtr()->time;

	mujoco_ros_msgs::StepGoal goal;
	goal.num_steps = 1;

	EXPECT_EQ(env_ptr->GetControlSnapshot().pending_steps, 0) << "No steps should be requested yet!";
	ac.sendGoal(goal);

	EXPECT_TRUE(ac.waitForResult(ros::Duration(1.0))) << "Step action did not finish in time!";
	EXPECT_EQ(ac.getState(), actionlib::SimpleClientGoalState::SUCCEEDED) << "Step action did not succeed!";
	EXPECT_TRUE(ac.getResult()->success) << "Step action did not succeed!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, time + env_ptr->getModelPtr()->opt.timestep, 1e-6)
	    << "Simulation time should have changed by timestep!";
#else // MJR_ROS_VERSION == ROS_2

	auto ac = rclcpp_action::create_client<mujoco_ros_msgs::action::Step>(env_ptr.get(),
	                                                                      env_ptr->GetHandleNamespace() + "/step");

	EXPECT_TRUE(ac->wait_for_action_server(std::chrono::seconds(1))) << "Step action server should be available!";
#endif
}

TEST_F(PendulumEnvFixture, StepGoalMultiple)
{
	auto topics = testing::get_available_topics_for_test(env_ptr);

#if MJR_ROS_VERSION == ROS_1
	const bool found = testing::has_topic(topics, env_ptr->GetHandleNamespace() + "/step/result");
#else // MJR_ROS_VERSION == ROS_2
	auto ac          = rclcpp_action::create_client<mujoco_ros_msgs::action::Step>(env_ptr.get(),
	                                                                               env_ptr->GetHandleNamespace() + "/step");
	const bool found = ac->wait_for_action_server(std::chrono::seconds(1));
#endif
	// Workaround to connect to action server, this is only needed in cpp
	ASSERT_TRUE(env_ptr->togglePaused(false));
	EXPECT_TRUE(found) << "Step action should be available!";

#if MJR_ROS_VERSION == ROS_1
	ros::spinOnce();
	actionlib::SimpleActionClient<mujoco_ros_msgs::StepAction> ac(env_ptr->GetHandleNamespace() + "/step", true);
	mujoco_ros_msgs::StepGoal goal;
#else // MJR_ROS_VERSION == ROS_2
	mujoco_ros_msgs::action::Step::Goal goal;
#endif
	ASSERT_TRUE(env_ptr->togglePaused(true));

	// Wait for paused state to be applied
	std::this_thread::sleep_for(std::chrono::milliseconds(5));

	mjtNum time = env_ptr->getDataPtr()->time;

	goal.num_steps = 100;
#if MJR_ROS_VERSION == ROS_1
	ac.sendGoal(goal);
	EXPECT_TRUE(ac.waitForResult(ros::Duration(1.0))) << "Step action did not finish in time!";
	EXPECT_EQ(ac.getState(), actionlib::SimpleClientGoalState::SUCCEEDED) << "Step action did not succeed!";
	EXPECT_TRUE(ac.getResult()->success) << "Step action did not succeed!";
#else // MJR_ROS_VERSION == ROS_2
	auto goal_future   = ac->async_send_goal(goal);
	auto result_future = ac->async_get_result(goal_future.get());
	EXPECT_NE(result_future.wait_for(std::chrono::seconds(1)), std::future_status::timeout)
	    << "Step action did not finish in time!";
	auto result = result_future.get();
	EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED) << "Step action did not succeed!";
	EXPECT_TRUE(result.result->success) << "Step action did not succeed!";
#endif

	EXPECT_NEAR(env_ptr->getDataPtr()->time, time + env_ptr->getModelPtr()->opt.timestep * 100, 1e-6)
	    << "Simulation time should have changed by timestep * 100!";
}

TEST_F(PendulumEnvFixture, StepGoalPreemptUnpaused)
{
#if MJR_ROS_VERSION == ROS_1
	auto topics = testing::get_available_topics_for_test(env_ptr);

	const bool found = testing::has_topic(topics, env_ptr->GetHandleNamespace() + "/step/result");
#else // MJR_ROS_VERSION == ROS_2
	auto ac          = rclcpp_action::create_client<mujoco_ros_msgs::action::Step>(env_ptr.get(),
	                                                                               env_ptr->GetHandleNamespace() + "/step");
	const bool found = ac->wait_for_action_server(std::chrono::seconds(1));
#endif
	// Workaround to connect to action server, this is only needed in cpp
	ASSERT_TRUE(env_ptr->togglePaused(false));
	EXPECT_TRUE(found) << "Step action should be available!";
#if MJR_ROS_VERSION == ROS_1
	ros::spinOnce();
	actionlib::SimpleActionClient<mujoco_ros_msgs::StepAction> ac(env_ptr->GetHandleNamespace() + "/step", true);

	mujoco_ros_msgs::StepGoal goal;
	ac.sendGoal(goal);

	EXPECT_TRUE(ac.waitForResult(ros::Duration(1.0))) << "Step action did not finish in time!";
	EXPECT_EQ(ac.getState(), actionlib::SimpleClientGoalState::PREEMPTED)
	    << "Step action must be preempted when unpaused!";
	EXPECT_FALSE(ac.getResult()->success) << "Step action should have failed!";
#else // MJR_ROS_VERSION == ROS_2
	auto goal          = mujoco_ros_msgs::action::Step::Goal();
	auto goal_future   = ac->async_send_goal(goal);
	auto result_future = ac->async_get_result(goal_future.get());
	EXPECT_NE(result_future.wait_for(std::chrono::seconds(1)), std::future_status::timeout)
	    << "Step action did not finish in time!";
	auto result = result_future.get();
	EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED) << "Step action should be aborted when unpaused!";
	EXPECT_FALSE(result.result->success) << "Step action should have failed!";
#endif
}

TEST_F(PendulumEnvFixture, StepGoalCancelPreempt)
{
#if MJR_ROS_VERSION == ROS_1
	auto topics = testing::get_available_topics_for_test(env_ptr);

	const bool found = testing::has_topic(topics, env_ptr->GetHandleNamespace() + "/step/result");
#else // MJR_ROS_VERSION == ROS_2
	auto ac    = rclcpp_action::create_client<mujoco_ros_msgs::action::Step>(env_ptr.get(),
	                                                                         env_ptr->GetHandleNamespace() + "/step");
	bool found = ac->wait_for_action_server(std::chrono::seconds(1));
#endif
	EXPECT_TRUE(found) << "Step action should be available!";

#if MJR_ROS_VERSION == ROS_1
	// Workaround to connect to action server, this is only needed in cpp
	ASSERT_TRUE(env_ptr->togglePaused(false));
	ros::spinOnce();
	actionlib::SimpleActionClient<mujoco_ros_msgs::StepAction> ac(env_ptr->GetHandleNamespace() + "/step", true);

	ASSERT_TRUE(env_ptr->togglePaused(true));

	// Wait for paused state to be applied
	std::this_thread::sleep_for(std::chrono::milliseconds(5));

	mujoco_ros_msgs::StepGoal goal;
	goal.num_steps = 10000;
	ac.sendGoal(goal);
	ac.cancelGoal();
	ac.waitForResult();
	EXPECT_EQ(ac.getState(), actionlib::SimpleClientGoalState::PREEMPTED);
	EXPECT_FALSE(ac.getResult()->success) << "Step action should have failed!";
#else // MJR_ROS_VERSION == ROS_2
	ASSERT_FALSE(env_ptr->GetControlSnapshot().running) << "Step cancel test requires a paused simulation!";

	auto goal               = mujoco_ros_msgs::action::Step::Goal();
	goal.num_steps          = 10000;
	auto goal_handle_future = ac->async_send_goal(goal);
	ASSERT_NE(goal_handle_future.wait_for(std::chrono::seconds(1)), std::future_status::timeout)
	    << "Step action did not start in time!";

	auto goal_handle = goal_handle_future.get();
	ASSERT_TRUE(goal_handle) << "Goal was not accepted!";

	float seconds = 0;
	while (env_ptr->GetControlSnapshot().pending_steps == 0 && seconds < 1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	ASSERT_LT(seconds, 1) << "Step action did not create a pending manual-step request!";

	auto cancel_future = ac->async_cancel_goal(goal_handle);
	ASSERT_NE(cancel_future.wait_for(std::chrono::seconds(1)), std::future_status::timeout)
	    << "Step action cancel did not finish in time!";

	auto cancel_response = cancel_future.get();
	EXPECT_TRUE(cancel_response) << "Goal cancel was not accepted!";
	EXPECT_FALSE(cancel_response->goals_canceling.empty()) << "No goals were canceled!";

	auto result_future = ac->async_get_result(goal_handle);
	EXPECT_NE(result_future.wait_for(std::chrono::seconds(1)), std::future_status::timeout)
	    << "Step action did not finish in time!";
	auto result = result_future.get();
	EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED) << "Step action should be canceled!";
	EXPECT_FALSE(result.result->success) << "Step action should have failed!";
#endif
}

TEST_F(PendulumEnvFixture, DefaultInitialJointStates)
{
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	std::map<std::string, std::string> pos_map, vel_map;
	testing::get_initial_joint_state_maps(env_ptr, nh.get(), pos_map, vel_map);

	EXPECT_EQ(pos_map.size(), 0) << "No initial joint positions should be set!";
	EXPECT_EQ(vel_map.size(), 0) << "No initial joint velocities should be set!";

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should not be running yet!";
	EXPECT_EQ(env_ptr->getPendingSteps(), -1) << "Simulation should have pending steps!";
	EXPECT_NEAR(d->time, 0.0, 1e-6) << "Simulation time should be 0.0!";

	int id_balljoint, id1, id2, id_free;
	id_balljoint = mujoco_ros::util::jointName2id(m, "balljoint");
	id1          = mujoco_ros::util::jointName2id(m, "joint1");
	id2          = mujoco_ros::util::jointName2id(m, "joint2");
	id_free      = mujoco_ros::util::jointName2id(m, "ball_freejoint");

	EXPECT_NE(id_balljoint, -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(id1, -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(id2, -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(id_free, -1) << "'ball_freejoint' should be found as joint in model!";

	compare_qpos(d, m->jnt_qposadr[id_balljoint], "balljoint", { 1.0, 0.0, 0.0, 0.0 });
	compare_qpos(d, m->jnt_qposadr[id1], "joint1", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[id2], "joint2", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 1.0, 0.0, 0.06, 1.0, 0.0, 0.0, 0.0 });

	compare_qvel(d, m->jnt_dofadr[id_balljoint], "balljoint", { 0.0, 0.0, 0.0 });
	compare_qvel(d, m->jnt_dofadr[id1], "joint1", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[id2], "joint2", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
}

TEST_F(BaseEnvFixture, CustomInitialJointStates)
{
	nh->setParam("unpause", false);

	std::string xml_path = testing::get_test_model_path("pendulum_world.xml");
	std::map<std::string, std::string> pos_map, vel_map;

	pos_map.insert({ "balljoint", "0 0.707 0 0.707" });
	pos_map.insert({ "joint1", "-1.57" });
	pos_map.insert({ "joint2", "-0.66" });
	pos_map.insert({ "ball_freejoint", "2.0 1.0 1.06 0.0 0.707 0.0 0.707" });

	vel_map.insert({ "balljoint", "5 5 10" });
	vel_map.insert({ "joint2", "1.05" });
	vel_map.insert({ "ball_freejoint", "1.0 2.0 3.0 10 20 30" });

	testing::set_initial_joint_state_maps(nh.get(), pos_map, vel_map);

	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	env_ptr->StartWithXML(xml_path);

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() > 0 && seconds < 2) { // wait for reset to be done
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	ASSERT_LT(seconds, 2) << "Initial joint state reset did not finish before timeout!";

	mjData *d  = env_ptr->getDataPtr();
	mjModel *m = env_ptr->getModelPtr();

	int id_balljoint, id1, id2, id_free;

	id_balljoint = mujoco_ros::util::jointName2id(m, "balljoint");
	id1          = mujoco_ros::util::jointName2id(m, "joint1");
	id2          = mujoco_ros::util::jointName2id(m, "joint2");
	id_free      = mujoco_ros::util::jointName2id(m, "ball_freejoint");

	EXPECT_NE(id_balljoint, -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(id1, -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(id2, -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(id_free, -1) << "'ball_freejoint' should be found as joint in model!";

	compare_qpos(d, m->jnt_qposadr[id_balljoint], "balljoint", { 0.0, 0.707, 0.0, 0.707 }, { 0.0, 9e-4, 0.0, 9e-4 });
	compare_qpos(d, m->jnt_qposadr[id1], "joint1", { -1.57 });
	compare_qpos(d, m->jnt_qposadr[id2], "joint2", { -0.66 });
	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 2.0, 1.0, 1.06, 0.0, 0.707, 0.0, 0.707 },
	             { 0.0, 0.0, 0.0, 0.0, 9e-4, 0.0, 9e-4 });

	compare_qvel(d, m->jnt_dofadr[id_balljoint], "balljoint", { 5.0, 5.0, 10.0 });
	compare_qvel(d, m->jnt_dofadr[id1], "joint1", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[id2], "joint2", { 1.05 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 1.0, 2.0, 3.0, 10.0, 20.0, 30.0 });

	env_ptr->shutdown();
}

TEST_F(PendulumEnvFixture, CustomInitialJointStatesOnReset)
{
	std::map<std::string, std::string> pos_map, vel_map;

	pos_map.insert({ "balljoint", "0 0.707 0 0.707" });
	pos_map.insert({ "joint1", "-1.57" });
	pos_map.insert({ "joint2", "-0.66" });
	pos_map.insert({ "ball_freejoint", "2.0 1.0 1.06 0.0 0.707 0.0 0.707" });

	vel_map.insert({ "balljoint", "5 5 10" });
	vel_map.insert({ "joint2", "1.05" });
	vel_map.insert({ "ball_freejoint", "1.0 2.0 3.0 10 20 30" });

	mjData *d  = env_ptr->getDataPtr();
	mjModel *m = env_ptr->getModelPtr();

	int id_balljoint, id1, id2, id_free;

	id_balljoint = mujoco_ros::util::jointName2id(m, "balljoint");
	id1          = mujoco_ros::util::jointName2id(m, "joint1");
	id2          = mujoco_ros::util::jointName2id(m, "joint2");
	id_free      = mujoco_ros::util::jointName2id(m, "ball_freejoint");

	EXPECT_NE(id_balljoint, -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(id1, -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(id2, -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(id_free, -1) << "'ball_freejoint' should be found as joint in model!";

	compare_qpos(d, m->jnt_qposadr[id_balljoint], "balljoint", { 1.0, 0.0, 0.0, 0.0 });
	compare_qpos(d, m->jnt_qposadr[id1], "joint1", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[id2], "joint2", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 1.0, 0.0, 0.06, 1.0, 0.0, 0.0, 0.0 });

	compare_qvel(d, m->jnt_dofadr[id_balljoint], "balljoint", { 0.0, 0.0, 0.0 });
	compare_qvel(d, m->jnt_dofadr[id1], "joint1", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[id2], "joint2", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

	testing::set_initial_joint_state_maps(nh.get(), pos_map, vel_map);

	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/reset"))
	    << "Reset service should be available!";

	std_srvs::Empty srv;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/reset", srv))
	    << "Reset service call failed!";

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() > 0 && seconds < 2) { // wait for reset to be done
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	ASSERT_LT(seconds, 2) << "Initial joint state reset did not finish before timeout!";

	compare_qpos(d, m->jnt_qposadr[id_balljoint], "balljoint", { 0.0, 0.707, 0.0, 0.707 }, { 0.0, 9e-4, 0.0, 9e-4 });
	compare_qpos(d, m->jnt_qposadr[id1], "joint1", { -1.57 });
	compare_qpos(d, m->jnt_qposadr[id2], "joint2", { -0.66 });
	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 2.0, 1.0, 1.06, 0.0, 0.707, 0.0, 0.707 },
	             { 0.0, 0.0, 0.0, 0.0, 9e-4, 0.0, 9e-4 });

	compare_qvel(d, m->jnt_dofadr[id_balljoint], "balljoint", { 5.0, 5.0, 10.0 });
	compare_qvel(d, m->jnt_dofadr[id1], "joint1", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[id2], "joint2", { 1.05 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 1.0, 2.0, 3.0, 10.0, 20.0, 30.0 });

#if MJR_ROS_VERSION == ROS_1
	nh->deleteParam("initial_joint_positions/joint_map");
	nh->deleteParam("initial_joint_velocities/joint_map");
#endif
}

TEST_F(PendulumEnvFixture, SetBodyStateNotAllowed)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";

	// Set eval mode and hash
	env_ptr->setEvalMode(true);
	env_ptr->setAdminHash("right_hash");

	mujoco_ros_msgs::SetBodyState srv;
	srv.request.admin_hash = "wrong_hash";
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(PendulumEnvFixture, SetBodyStateEmptyBodyName)
{
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";

	mujoco_ros_msgs::SetBodyState srv;

	// Invalid body_name
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(PendulumEnvFixture, SetBodyStateInvalidBodyName)
{
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";

	mujoco_ros_msgs::SetBodyState srv;
	srv.request.state.name = "unknown";

	// Invalid body_name
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(PendulumEnvFixture, SetBodyStateResolveBody)
{
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";

	mujoco_ros_msgs::SetBodyState srv;

	// Resolve body
	srv.request.state.name = "middle_link";
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_TRUE(srv.response.success);
}

TEST_F(PendulumEnvFixture, SetBodyStateResolveBodyFromGeom)
{
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";

	mujoco_ros_msgs::SetBodyState srv;

	// Resolve body from child geom
	srv.request.state.name = "EE";
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_TRUE(srv.response.success);
}

TEST_F(PendulumEnvFixture, SetBodyStatePosNonFreejointError)
{
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";

	mujoco_ros_msgs::SetBodyState srv;
	srv.request.set_pose = true;

	//   Not a freejoint
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(PendulumEnvFixture, SetBodyStatePosNoJointError)
{
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";

	mujoco_ros_msgs::SetBodyState srv;
	srv.request.set_pose = true;

	srv.request.state.name = "immovable";
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(PendulumEnvFixture, SetBodyStatePosUnknownFrameID)
{
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";

	mujoco_ros_msgs::SetBodyState srv;
	srv.request.set_pose = true;

	srv.request.state.name                 = "ball";
	srv.request.state.pose.header.frame_id = "unknown";
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(PendulumEnvFixture, SetBodyStateTwistNotWorldFrame)
{
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";

	mujoco_ros_msgs::SetBodyState srv;
	srv.request.set_twist = true;

	//   other frame_id than world
	srv.request.state.twist.header.frame_id = "not-world";
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(PendulumEnvFixture, SetBodyStatePosAndTwist)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";

	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	int id_free = mujoco_ros::util::jointName2id(m, "ball_freejoint");

	mujoco_ros_msgs::SetBodyState srv;
	srv.request.state.name = "ball";
	// New twist and pose
	srv.request.set_twist = true;
	srv.request.set_pose  = true;

	srv.request.state.pose.header.frame_id    = "world";
	srv.request.state.pose.pose.position.x    = 2;
	srv.request.state.pose.pose.position.y    = 2;
	srv.request.state.pose.pose.position.z    = 2;
	srv.request.state.pose.pose.orientation.x = 0.707;
	srv.request.state.pose.pose.orientation.y = 0.0;
	srv.request.state.pose.pose.orientation.z = 0.707;
	srv.request.state.pose.pose.orientation.w = 0.0;

	srv.request.state.twist.header.frame_id = "world";
	srv.request.state.twist.twist.linear.x  = 0.1;
	srv.request.state.twist.twist.linear.y  = 0.1;
	srv.request.state.twist.twist.linear.z  = -0.1;
	srv.request.state.twist.twist.angular.x = 0.1;
	srv.request.state.twist.twist.angular.y = 0;
	srv.request.state.twist.twist.angular.z = 0;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_TRUE(srv.response.success);

	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 2.0, 2.0, 2.0, 0.0, 0.707, 0.0, 0.707 },
	             { 0.0, 0.0, 0.0, 0.0, 9e-4, 0.0, 9e-4 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 0.1, 0.1, -0.1, 0.1, 0.0, 0.0 });
}

TEST_F(PendulumEnvFixture, SetBodyStateMass)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";

	mjModel *m = env_ptr->getModelPtr();

	mujoco_ros_msgs::SetBodyState srv;
	srv.request.state.name = "ball";
	// New mass
	srv.request.set_mass   = true;
	srv.request.state.mass = 0.299f;
	EXPECT_NE(m->body_mass[mj_name2id(m, mjOBJ_BODY, "body_ball")], srv.request.state.mass)
	    << "Mass already has the requested value!"; // Check that mass is different beforehand
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_TRUE(srv.response.success);

	EXPECT_EQ(m->body_mass[mj_name2id(m, mjOBJ_BODY, "body_ball")], srv.request.state.mass)
	    << "Mass did not change to the requested value";
}

TEST_F(PendulumEnvFixture, SetBodyStateResetQPos)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";

	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	int id_free = mujoco_ros::util::jointName2id(m, "ball_freejoint");

	mujoco_ros_msgs::SetBodyState srv;
	srv.request.state.name = "ball";
	// reset
	srv.request.reset_qpos = true;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_TRUE(srv.response.success);

	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 1.0, 0.0, 0.06, 1.0, 0.0, 0.0, 0.0 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
}

TEST_F(PendulumEnvFixture, GetBodyStateNotAllowed)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state"))
	    << "Get body state service should be available!";

	// set eval mode and hash
	env_ptr->setEvalMode(true);
	env_ptr->setAdminHash("right_hash");

	mujoco_ros_msgs::GetBodyState g_srv;
	g_srv.request.admin_hash = "wrong_hash";

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state", g_srv))
	    << "get body state service call failed!";
	EXPECT_FALSE(g_srv.response.success);
}

TEST_F(PendulumEnvFixture, GetBodyStateNameEmpty)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state"))
	    << "Get body state service should be available!";

	mujoco_ros_msgs::GetBodyState g_srv;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state", g_srv))
	    << "get body state service call failed!";
	EXPECT_FALSE(g_srv.response.success);
}

TEST_F(PendulumEnvFixture, GetBodyStateInvalidName)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state"))
	    << "Get body state service should be available!";

	mujoco_ros_msgs::GetBodyState g_srv;
	// wrong body name
	g_srv.request.name = "unknown";
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state", g_srv))
	    << "get body state service call failed!";
	EXPECT_FALSE(g_srv.response.success);
}

TEST_F(PendulumEnvFixture, GetBodyStateResolveBodyFromGeom)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state"))
	    << "Get body state service should be available!";

	mujoco_ros_msgs::GetBodyState g_srv;
	g_srv.request.name = "EE";
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state", g_srv))
	    << "get body state service call failed!";
	EXPECT_TRUE(g_srv.response.success);
	EXPECT_EQ(g_srv.response.state.name, "end_link");
}

TEST_F(PendulumEnvFixture, GetBodyStateStaticBody)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state"))
	    << "Get body state service should be available!";

	mujoco_ros_msgs::GetBodyState g_srv;
	g_srv.request.name = "immovable";
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state", g_srv))
	    << "get body state service call failed!";
	EXPECT_TRUE(g_srv.response.success);
	EXPECT_EQ(g_srv.response.state.name, "immovable");
	EXPECT_DOUBLE_EQ(g_srv.response.state.mass, 0.10239999741315842);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.position.x, 0.56428);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.position.y, 0.221972);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.position.z, 0.6);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.orientation.w, 1.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.orientation.x, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.orientation.y, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.orientation.z, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.twist.twist.linear.x, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.twist.twist.linear.y, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.twist.twist.linear.z, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.twist.twist.angular.x, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.twist.twist.angular.y, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.twist.twist.angular.z, 0.0);
}

TEST_F(PendulumEnvFixture, GetBodyStateMultijoint)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state"))
	    << "Get body state service should be available!";

	mujoco_ros_msgs::GetBodyState g_srv;
	g_srv.request.name = "multijoint";
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state", g_srv))
	    << "get body state service call failed!";
	EXPECT_TRUE(g_srv.response.success);
	EXPECT_EQ(g_srv.response.state.name, "multijoint");
	EXPECT_DOUBLE_EQ(g_srv.response.state.mass, 5.4286723136901855);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.position.x, 2.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.position.y, 1.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.position.z, 0.5);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.orientation.w, 1.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.orientation.x, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.orientation.y, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.pose.pose.orientation.z, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.twist.twist.linear.x, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.twist.twist.linear.y, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.twist.twist.linear.z, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.twist.twist.angular.x, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.twist.twist.angular.y, 0.0);
	EXPECT_DOUBLE_EQ(g_srv.response.state.twist.twist.angular.z, 0.0);
}

TEST_F(PendulumEnvFixture, GetBodyStateFreejoint)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state"))
	    << "Set body state service should be available!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state"))
	    << "Get body state service should be available!";

	mujoco_ros_msgs::SetBodyState srv;
	srv.request.set_pose                      = true;
	srv.request.set_twist                     = true;
	srv.request.set_mass                      = true;
	srv.request.set_pose                      = true;
	srv.request.state.name                    = "body_ball";
	srv.request.state.mass                    = 0.299f;
	srv.request.state.pose.pose.position.x    = 1.0;
	srv.request.state.pose.pose.position.y    = 1.0;
	srv.request.state.pose.pose.position.z    = 1.0;
	srv.request.state.pose.pose.orientation.w = 1.0;
	srv.request.state.pose.pose.orientation.x = 0.0;
	srv.request.state.pose.pose.orientation.y = 0.0;
	srv.request.state.pose.pose.orientation.z = 0.0;
	srv.request.state.twist.twist.linear.x    = 0.1;
	srv.request.state.twist.twist.linear.y    = 0.1;
	srv.request.state.twist.twist.linear.z    = -0.1;
	srv.request.state.twist.twist.angular.x   = 0.1;
	srv.request.state.twist.twist.angular.y   = 0.1;
	srv.request.state.twist.twist.angular.z   = -0.1;

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_TRUE(srv.response.success);

	mujoco_ros_msgs::GetBodyState g_srv;

	// correct request
	g_srv.request.name = "body_ball";
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_body_state", g_srv))
	    << "get body state service call failed!";
	EXPECT_TRUE(g_srv.response.success);
	EXPECT_EQ(g_srv.response.state.mass, srv.request.state.mass);
	EXPECT_EQ(g_srv.response.state.name, srv.request.state.name);
	EXPECT_EQ(g_srv.response.state.pose.pose.position, srv.request.state.pose.pose.position);
	EXPECT_EQ(g_srv.response.state.pose.pose.orientation, srv.request.state.pose.pose.orientation);
	EXPECT_EQ(g_srv.response.state.twist.twist, srv.request.state.twist.twist);
}

TEST_F(PendulumEnvFixture, SetGeomPropertiesNameEmpty)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties"))
	    << "Set geom properties service should be available!";

	mujoco_ros_msgs::SetGeomProperties srv;

	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(PendulumEnvFixture, SetGeomPropertiesInvalidGeomName)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties"))
	    << "Set geom properties service should be available!";

	mujoco_ros_msgs::SetGeomProperties srv;

	// Invalid geom_name
	srv.request.properties.name = "unknown";
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(PendulumEnvFixture, SetGeomPropertiesValidGeomName)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties"))
	    << "Set geom properties service should be available!";

	mujoco_ros_msgs::SetGeomProperties srv;

	// Resolve geom
	srv.request.properties.name = "ball";
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
}

TEST_F(PendulumEnvFixture, SetGeomPropertiesMass)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties"))
	    << "Set geom properties service should be available!";

	mjModel *m = env_ptr->getModelPtr();

	int ball_body_id = mj_name2id(m, mjOBJ_BODY, "body_ball");
	EXPECT_NE(ball_body_id, -1) << "'body_ball' should be found as body in model!";

	mujoco_ros_msgs::SetGeomProperties srv;
	srv.request.properties.name = "ball";
	// set mass
	srv.request.set_mass             = true;
	srv.request.properties.body_mass = 0.299f;
	EXPECT_NE(m->body_mass[ball_body_id], srv.request.properties.body_mass) << "Mass already has requested value!";
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(m->body_mass[ball_body_id], srv.request.properties.body_mass)
	    << "Mass did not change to the requested value";
}

TEST_F(PendulumEnvFixture, SetGeomPropertiesFriction)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties"))
	    << "Set geom properties service should be available!";

	mjModel *m = env_ptr->getModelPtr();

	int ball_geom_id = mj_name2id(m, mjOBJ_GEOM, "ball");
	EXPECT_NE(ball_geom_id, -1) << "'ball' should be found as geom in model!";

	mujoco_ros_msgs::SetGeomProperties srv;
	srv.request.properties.name = "ball";
	// set friction
	srv.request.set_friction              = true;
	srv.request.properties.friction_slide = 0;
	srv.request.properties.friction_spin  = 0;
	srv.request.properties.friction_roll  = 0;
	EXPECT_TRUE(m->geom_friction[ball_geom_id * 3] != 0 && m->geom_friction[ball_geom_id * 3 + 1] != 0 &&
	            m->geom_friction[ball_geom_id * 3 + 2] != 0)
	    << "Some friction values already at 0!";
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_TRUE(m->geom_friction[ball_geom_id * 3] == 0) << "Slide friction unchanged!";
	EXPECT_TRUE(m->geom_friction[ball_geom_id * 3 + 1] == 0) << "Spin friction unchanged!";
	EXPECT_TRUE(m->geom_friction[ball_geom_id * 3 + 2] == 0) << "Roll friction uncahnged!";
}

// set type (not checking PLANE, HFIELD, MESH, and rendering types)
TEST_F(PendulumEnvFixture, SetGeomPropertiesTypeBox)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties"))
	    << "Set geom properties service should be available!";

	mjModel *m = env_ptr->getModelPtr();

	int ball_geom_id = mj_name2id(m, mjOBJ_GEOM, "ball");
	EXPECT_NE(ball_geom_id, -1) << "'ball' should be found as geom in model!";

	mujoco_ros_msgs::SetGeomProperties srv;
	srv.request.properties.name = "ball";
	srv.request.set_type        = true;
	//   BOX
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::BOX;
	EXPECT_NE(m->geom_type[ball_geom_id], mjGEOM_BOX) << "Geom already is of type BOX";
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(m->geom_type[ball_geom_id], mjGEOM_BOX) << "Geom unchanged";
}

TEST_F(PendulumEnvFixture, SetGeomPropertiesTypeCylinder)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties"))
	    << "Set geom properties service should be available!";

	mjModel *m = env_ptr->getModelPtr();

	int ball_geom_id = mj_name2id(m, mjOBJ_GEOM, "ball");
	EXPECT_NE(ball_geom_id, -1) << "'ball' should be found as geom in model!";

	mujoco_ros_msgs::SetGeomProperties srv;
	srv.request.properties.name = "ball";
	srv.request.set_type        = true;
	//   CYLINDER
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::CYLINDER;
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(m->geom_type[ball_geom_id], mjGEOM_CYLINDER) << "Geom unchanged";
}

TEST_F(PendulumEnvFixture, SetGeomPropertiesTypeEllipsoid)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties"))
	    << "Set geom properties service should be available!";

	mjModel *m = env_ptr->getModelPtr();

	int ball_geom_id = mj_name2id(m, mjOBJ_GEOM, "ball");
	EXPECT_NE(ball_geom_id, -1) << "'ball' should be found as geom in model!";

	mujoco_ros_msgs::SetGeomProperties srv;
	srv.request.properties.name = "ball";
	srv.request.set_type        = true;
	//  ELLIPSOID
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::ELLIPSOID;
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(m->geom_type[ball_geom_id], mjGEOM_ELLIPSOID) << "Geom unchanged";
}

TEST_F(PendulumEnvFixture, SetGeomPropertiesTypeCapsule)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties"))
	    << "Set geom properties service should be available!";

	mjModel *m = env_ptr->getModelPtr();

	int ball_geom_id = mj_name2id(m, mjOBJ_GEOM, "ball");
	EXPECT_NE(ball_geom_id, -1) << "'ball' should be found as geom in model!";

	mujoco_ros_msgs::SetGeomProperties srv;
	srv.request.set_type        = true;
	srv.request.properties.name = "ball";
	//  CAPSULE
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::CAPSULE;
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(m->geom_type[ball_geom_id], mjGEOM_CAPSULE) << "Geom unchanged";
}

TEST_F(PendulumEnvFixture, SetGeomPropertiesTypeSphere)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties"))
	    << "Set geom properties service should be available!";

	mjModel *m = env_ptr->getModelPtr();

	int ball_geom_id = mj_name2id(m, mjOBJ_GEOM, "ball");
	EXPECT_NE(ball_geom_id, -1) << "'ball' should be found as geom in model!";

	mujoco_ros_msgs::SetGeomProperties srv;
	srv.request.properties.name = "ball";
	srv.request.set_type        = true;
	//  SPHERE
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::SPHERE;
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(m->geom_type[ball_geom_id], mjGEOM_SPHERE) << "Geom unchanged";
}

TEST_F(PendulumEnvFixture, SetGeomPropertiesSize)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties"))
	    << "Set geom properties service should be available!";

	mjModel *m = env_ptr->getModelPtr();

	int ball_geom_id = mj_name2id(m, mjOBJ_GEOM, "ball");
	int ball_body_id = mj_name2id(m, mjOBJ_BODY, "body_ball");

	EXPECT_NE(ball_geom_id, -1) << "'ball' should be found as geom in model!";
	EXPECT_NE(ball_body_id, -1) << "'body_ball' should be found as body in model!";

	mujoco_ros_msgs::SetGeomProperties srv;
	srv.request.properties.name   = "ball";
	srv.request.set_size          = true;
	srv.request.properties.size_0 = 0.01f;
	srv.request.properties.size_1 = 0.01f;
	srv.request.properties.size_2 = 0.01f;
	EXPECT_TRUE(m->geom_size[ball_geom_id * 3] != 0.01 && m->geom_size[ball_geom_id * 3 + 1] != 0.01 &&
	            m->geom_size[ball_geom_id * 3 + 2] != 0.01)
	    << "Geom size is already 0.01 0.01 0.01!";
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_NEAR(m->geom_size[ball_geom_id * 3], 0.01, 9e-4) << "Size 0 unchanged";
	EXPECT_NEAR(m->geom_size[ball_geom_id * 3 + 1], 0.01, 9e-4) << "Size 1 unchanged";
	EXPECT_NEAR(m->geom_size[ball_geom_id * 3 + 2], 0.01, 9e-4) << "Size 2 unchanged";
}

TEST_F(PendulumEnvFixture, GetGeomPropertiesNotAllowed)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_geom_properties"))
	    << "Get geom properties service should be available!";

	// set eval mode and hash
	env_ptr->setEvalMode(true);
	env_ptr->setAdminHash("right_hash");

	mujoco_ros_msgs::GetGeomProperties g_srv;
	g_srv.request.admin_hash = "wrong_hash";

	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_geom_properties", g_srv))
	    << "Get geom properties service call failed!";
	EXPECT_FALSE(g_srv.response.success);
}

TEST_F(PendulumEnvFixture, GetGeomPropertiesNameEmpty)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_geom_properties"))
	    << "Get geom properties service should be available!";

	mujoco_ros_msgs::GetGeomProperties g_srv;

	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_geom_properties", g_srv))
	    << "Get geom properties service call failed!";
	EXPECT_FALSE(g_srv.response.success);
}

TEST_F(PendulumEnvFixture, GetGeomPropertiesInvalidGeomName)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_geom_properties"))
	    << "Get geom properties service should be available!";

	mujoco_ros_msgs::GetGeomProperties g_srv;
	g_srv.request.geom_name = "unknown";

	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_geom_properties", g_srv))
	    << "Get geom properties service call failed!";
	EXPECT_FALSE(g_srv.response.success);
}

TEST_F(PendulumEnvFixture, GetGeomPropertiesValidGeomName)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties"))
	    << "Set geom properties service should be available!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_geom_properties"))
	    << "Get geom properties service should be available!";

	mujoco_ros_msgs::SetGeomProperties srv;
	srv.request.set_type                  = true;
	srv.request.set_mass                  = true;
	srv.request.set_size                  = true;
	srv.request.set_friction              = true;
	srv.request.properties.name           = "ball";
	srv.request.properties.type.value     = mujoco_ros_msgs::GeomType::BOX;
	srv.request.properties.body_mass      = 0.299f;
	srv.request.properties.size_0         = 0.01f;
	srv.request.properties.size_1         = 0.01f;
	srv.request.properties.size_2         = 0.01f;
	srv.request.properties.friction_slide = 1.;
	srv.request.properties.friction_spin  = 1.;
	srv.request.properties.friction_roll  = 1.;
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);

	mujoco_ros_msgs::GetGeomProperties g_srv;

	g_srv.request.geom_name = "ball";
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_geom_properties", g_srv))
	    << "Get geom properties service call failed!";
	EXPECT_TRUE(g_srv.response.success);

	EXPECT_EQ(srv.request.properties.name, g_srv.response.properties.name);
	EXPECT_EQ(srv.request.properties.type.value, g_srv.response.properties.type.value);
	EXPECT_EQ(srv.request.properties.body_mass, g_srv.response.properties.body_mass);
	EXPECT_EQ(srv.request.properties.size_0, g_srv.response.properties.size_0);
	EXPECT_EQ(srv.request.properties.size_1, g_srv.response.properties.size_1);
	EXPECT_EQ(srv.request.properties.size_2, g_srv.response.properties.size_2);
	EXPECT_EQ(srv.request.properties.friction_slide, g_srv.response.properties.friction_slide);
	EXPECT_EQ(srv.request.properties.friction_spin, g_srv.response.properties.friction_spin);
	EXPECT_EQ(srv.request.properties.friction_roll, g_srv.response.properties.friction_roll);
}

TEST_F(EqualityEnvFixture, InitialEqualityConstraintValues)
{
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	int weld_eq_id    = mj_name2id(m, mjOBJ_EQUALITY, "weld_eq");
	int joint_eq_id   = mj_name2id(m, mjOBJ_EQUALITY, "joint_eq");
	int connect_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_eq");
	int tendon_eq_id  = mj_name2id(m, mjOBJ_EQUALITY, "tendon_eq");

	EXPECT_NE(weld_eq_id, -1) << "weld_eq is not defined in loaded model!";
	EXPECT_NE(joint_eq_id, -1) << "joint_eq is not defined in loaded model!";
	EXPECT_NE(connect_eq_id, -1) << "weld_eq is not defined in loaded model!";
	EXPECT_NE(tendon_eq_id, -1) << "joint_eq is not defined in loaded model!";

	EXPECT_EQ(m->eq_type[weld_eq_id], mjEQ_WELD) << "weld_eq has incorrect type";
	EXPECT_EQ(m->eq_type[joint_eq_id], mjEQ_JOINT) << "joint_eq has incorrect type";
	EXPECT_EQ(m->eq_type[tendon_eq_id], mjEQ_TENDON) << "weld_eq has incorrect type";
	EXPECT_EQ(m->eq_type[connect_eq_id], mjEQ_CONNECT) << "connect_eq has incorrect type";
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";

	// test joint constraint data
	EXPECT_TRUE(d->eq_active[joint_eq_id]) << "Joint constraint mismatch of active state";
	EXPECT_DOUBLE_EQ(m->eq_data[joint_eq_id * mjNEQDATA], 0.5) << "Joint constraint polycoef mismatch at index 0";
	EXPECT_DOUBLE_EQ(m->eq_data[joint_eq_id * mjNEQDATA + 1], 0.25) << "Joint constraint polycoef mismatch at index 1";
	EXPECT_DOUBLE_EQ(m->eq_data[joint_eq_id * mjNEQDATA + 2], 0.76) << "Joint constraint polycoef mismatch at index 2";
	EXPECT_DOUBLE_EQ(m->eq_data[joint_eq_id * mjNEQDATA + 3], 0.66) << "Joint constraint polycoef mismatch at index 3";
	EXPECT_DOUBLE_EQ(m->eq_data[joint_eq_id * mjNEQDATA + 4], 1) << "Joint constraint polycoef mismatch at index 4";
	EXPECT_STREQ("joint_eq_element1", mj_id2name(m, mjOBJ_JOINT, m->eq_obj1id[joint_eq_id]))
	    << "Joint constraint joint1 mismatch";
	EXPECT_STREQ("joint_eq_element2", mj_id2name(m, mjOBJ_JOINT, m->eq_obj2id[joint_eq_id]))
	    << "Joint constraint joint2 mismatch";

	// test connect
	EXPECT_TRUE(d->eq_active[connect_eq_id]) << "Joint constraint mismatch of active state";
	EXPECT_DOUBLE_EQ(m->eq_data[connect_eq_id * mjNEQDATA + 0], 0) << "Anchor mismatch at index 0";
	EXPECT_DOUBLE_EQ(m->eq_data[connect_eq_id * mjNEQDATA + 1], 0) << "Anchor mismatch at index 1";
	EXPECT_DOUBLE_EQ(m->eq_data[connect_eq_id * mjNEQDATA + 2], 0) << "Anchor mismatch at index 2";
	EXPECT_STREQ("immovable", mj_id2name(m, mjOBJ_XBODY, m->eq_obj1id[connect_eq_id]))
	    << "connect constraint element1 mismatch";

	// test tendon
	EXPECT_TRUE(d->eq_active[tendon_eq_id]) << "Joint constraint mismatch of active state";
	EXPECT_DOUBLE_EQ(m->eq_data[tendon_eq_id * mjNEQDATA], 0.5) << "Joint constraint polycoef mismatch at index 0";
	EXPECT_DOUBLE_EQ(m->eq_data[tendon_eq_id * mjNEQDATA + 1], 0.25) << "Joint constraint polycoef mismatch at index 1";
	EXPECT_DOUBLE_EQ(m->eq_data[tendon_eq_id * mjNEQDATA + 2], 0.76) << "Joint constraint polycoef mismatch at index 2";
	EXPECT_DOUBLE_EQ(m->eq_data[tendon_eq_id * mjNEQDATA + 3], 0.66) << "Joint constraint polycoef mismatch at index 3";
	EXPECT_DOUBLE_EQ(m->eq_data[tendon_eq_id * mjNEQDATA + 4], 1) << "Joint constraint polycoef mismatch at index 4";
	EXPECT_STREQ("tendon_eq_element1", mj_id2name(m, mjOBJ_TENDON, m->eq_obj1id[tendon_eq_id]))
	    << "tendon constraint element1 mismatch";
	EXPECT_STREQ("tendon_eq_element2", mj_id2name(m, mjOBJ_TENDON, m->eq_obj2id[tendon_eq_id]))
	    << "tendon constraint element2 mismatch";
	// test weld
	double solimp[5] = { 0.8, 0.95, 0.002, 0.4, 2.0 };
	EXPECT_DOUBLE_EQ(m->eq_solimp[weld_eq_id], solimp[0]) << "Weld constraint solimp dmin mismatch";
	EXPECT_DOUBLE_EQ(m->eq_solimp[weld_eq_id + 1], solimp[1]) << "Weld constraint solimp dmax mismatch";
	EXPECT_DOUBLE_EQ(m->eq_solimp[weld_eq_id + 2], solimp[2]) << "Weld constraint solimp width mismatch";
	EXPECT_DOUBLE_EQ(m->eq_solimp[weld_eq_id + 3], solimp[3]) << "Weld constraint solimp midpoint mismatch";
	EXPECT_DOUBLE_EQ(m->eq_solimp[weld_eq_id + 4], solimp[4]) << "Weld constraint solimp power mismatch";

	EXPECT_DOUBLE_EQ(m->eq_solref[weld_eq_id], 0.3) << "Weld constraint solref timeconst mismatch";
	EXPECT_DOUBLE_EQ(m->eq_solref[weld_eq_id + 1], 0.9) << "Weld constraint solref dampratio mismatch";

	EXPECT_STREQ("immovable", mj_id2name(m, mjOBJ_BODY, m->eq_obj1id[weld_eq_id])) << "Weld constraint body1 mismatch";
	EXPECT_EQ(0, m->eq_obj2id[weld_eq_id]) << "Weld constraint body2 mismatch (worldbody)";

	// data[0-2] anchor data[3-9] relpose, data[10] torque
	mjtNum data[11] = { 0., 0., 0., 1.1, 1.2, 1.3, 0.358, -0.003, -0.886, 0.295, 0.9 };
	mju_normalize4(data + 6); // Normalize quaternion

	for (int i = 0; i < 3; i++) {
		EXPECT_DOUBLE_EQ(m->eq_data[weld_eq_id * mjNEQDATA + i], data[i]) << "Weld constraint data mismatch in anchor";
	}
	for (int i = 3; i < 6; i++) {
		EXPECT_DOUBLE_EQ(m->eq_data[weld_eq_id * mjNEQDATA + i], data[i])
		    << "Weld constraint data mismatch in relpose position";
	}
	for (int i = 6; i < 10; i++) {
		EXPECT_NEAR(m->eq_data[weld_eq_id * mjNEQDATA + i], data[i], 1e-3)
		    << "Weld constraint data mismatch in relpose orientation";
	}
	EXPECT_DOUBLE_EQ(m->eq_data[weld_eq_id * mjNEQDATA + 10], data[10])
	    << "Weld constraint data mismatch in torquescale";
}

// helper function to verify inequality of mujoco equality constraint and message type parameters
void compare_eqc_values_with_msg_inequal(mjModel *m, mjData *d, int eq_id,
                                         const mujoco_ros_msgs::EqualityConstraintParameters &eqc)
{
	EXPECT_NE(d->eq_active[eq_id], eqc.active) << eqc.name << " constraint active parameter would not change";

	EXPECT_NE(m->eq_solref[eq_id * mjNREF], eqc.solver_parameters.timeconst)
	    << eqc.name << " solref timeconst would not change";
	EXPECT_NE(m->eq_solref[eq_id * mjNREF + 1], eqc.solver_parameters.dampratio)
	    << eqc.name << " solref dampratio would not change";

	EXPECT_NE(m->eq_solimp[eq_id * mjNIMP], eqc.solver_parameters.dmin) << eqc.name << " solimp dmin would not change";
	EXPECT_NE(m->eq_solimp[eq_id * mjNIMP + 1], eqc.solver_parameters.dmax)
	    << eqc.name << " solimp dmax would not change";
	EXPECT_NE(m->eq_solimp[eq_id * mjNIMP + 2], eqc.solver_parameters.width)
	    << eqc.name << " solimp width would not change";
	EXPECT_NE(m->eq_solimp[eq_id * mjNIMP + 3], eqc.solver_parameters.midpoint)
	    << eqc.name << " solimp midpoint would not change";
	EXPECT_NE(m->eq_solimp[eq_id * mjNIMP + 4], eqc.solver_parameters.power)
	    << eqc.name << " solimp power would not change";

	// TODO: replace NE with something reproducing not EXPECT_DOUBLE_EQ (EXPECT_DOUBLE_NE does not exist!)
	if (eqc.type.value == mujoco_ros_msgs::EqualityConstraintType::WELD) {
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA], eqc.anchor.x) << eqc.name << " anchor[0] would not change";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 1], eqc.anchor.y) << eqc.name << " anchor[1] would not change";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 2], eqc.anchor.z) << eqc.name << " anchor[2] would not change";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 3], eqc.relpose.position.x)
		    << eqc.name << " relpose[0] would not change";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 4], eqc.relpose.position.y)
		    << eqc.name << " relpose[1] would not change";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 5], eqc.relpose.position.z)
		    << eqc.name << " relpose[2] would not change";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 6], eqc.relpose.orientation.w)
		    << eqc.name << " relpose[3] would not change";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 7], eqc.relpose.orientation.x)
		    << eqc.name << " relpose[4] would not change";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 8], eqc.relpose.orientation.y)
		    << eqc.name << " relpose[5] would not change";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 9], eqc.relpose.orientation.z)
		    << eqc.name << " relpose[6] would not change";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 10], eqc.torquescale) << eqc.name << " torquescale would not change";

	} else if (eqc.type.value == mujoco_ros_msgs::EqualityConstraintType::CONNECT) {
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA], eqc.anchor.x) << eqc.name << " anchor[0] would not change";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 1], eqc.anchor.y) << eqc.name << " anchor[1] would not change";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 2], eqc.anchor.z) << eqc.name << " anchor[2] would not change";

	} else if (eqc.type.value == mujoco_ros_msgs::EqualityConstraintType::JOINT ||
	           eqc.type.value == mujoco_ros_msgs::EqualityConstraintType::TENDON) {
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA], eqc.polycoef[0]) << eqc.name << " polyceof[0] was not set correctly";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 1], eqc.polycoef[1]) << eqc.name << " polycoef[1] was not set correctly";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 2], eqc.polycoef[2]) << eqc.name << " polycoef[2] was not set correctly";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 3], eqc.polycoef[3]) << eqc.name << " polycoef[3] was not set correctly";
		EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 4], eqc.polycoef[4]) << eqc.name << " polycoef[3] was not set correctly";
	}
}

// helper function to verify equality of mujoco equality constraint and message type parameters
void compare_eqc_values_with_msg(mjModel *m, mjData *d, int eq_id,
                                 const mujoco_ros_msgs::EqualityConstraintParameters &eqc)
{
	EXPECT_EQ(d->eq_active[eq_id], eqc.active);
	EXPECT_DOUBLE_EQ(m->eq_solref[eq_id * mjNREF], eqc.solver_parameters.timeconst)
	    << eqc.name << " solref timeconst was not set correctly";
	EXPECT_DOUBLE_EQ(m->eq_solref[eq_id * mjNREF + 1], eqc.solver_parameters.dampratio)
	    << eqc.name << " solref dampratio was not set correctly";

	EXPECT_DOUBLE_EQ(m->eq_solimp[eq_id * mjNIMP], eqc.solver_parameters.dmin)
	    << eqc.name << " solimp dmin was not set correctly";
	EXPECT_DOUBLE_EQ(m->eq_solimp[eq_id * mjNIMP + 1], eqc.solver_parameters.dmax)
	    << eqc.name << " solimp dmax was not set correctly";
	EXPECT_DOUBLE_EQ(m->eq_solimp[eq_id * mjNIMP + 2], eqc.solver_parameters.width)
	    << eqc.name << " solimp width was not set correctly";
	EXPECT_DOUBLE_EQ(m->eq_solimp[eq_id * mjNIMP + 3], eqc.solver_parameters.midpoint)
	    << eqc.name << " solimp midpoint was not set correctly";
	EXPECT_DOUBLE_EQ(m->eq_solimp[eq_id * mjNIMP + 4], eqc.solver_parameters.power)
	    << eqc.name << " solimp power was not set correctly";
	if (eqc.type.value == mujoco_ros_msgs::EqualityConstraintType::WELD) {
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA], eqc.anchor.x) << eqc.name << " anchor[0] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 1], eqc.anchor.y)
		    << eqc.name << " anchor[1] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 2], eqc.anchor.z)
		    << eqc.name << " anchor[2] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 3], eqc.relpose.position.x)
		    << eqc.name << " relpose[0] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 4], eqc.relpose.position.y)
		    << eqc.name << " relpose[1] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 5], eqc.relpose.position.z)
		    << eqc.name << " relpose[2] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 6], eqc.relpose.orientation.w)
		    << eqc.name << " relpose[3] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 7], eqc.relpose.orientation.x)
		    << eqc.name << " relpose[4] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 8], eqc.relpose.orientation.y)
		    << eqc.name << " relpose[5] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 9], eqc.relpose.orientation.z)
		    << eqc.name << " relpose[6] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 10], eqc.torquescale)
		    << eqc.name << " torquescale was not set correctly";

	} else if (eqc.type.value == mujoco_ros_msgs::EqualityConstraintType::CONNECT) {
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA], eqc.anchor.x) << eqc.name << " anchor[0] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 1], eqc.anchor.y)
		    << eqc.name << " anchor[1] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 2], eqc.anchor.z)
		    << eqc.name << " anchor[2] was not set correctly";

	} else if (eqc.type.value == mujoco_ros_msgs::EqualityConstraintType::JOINT ||
	           eqc.type.value == mujoco_ros_msgs::EqualityConstraintType::TENDON) {
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA], eqc.polycoef[0])
		    << eqc.name << " polyceof[0] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 1], eqc.polycoef[1])
		    << eqc.name << " polycoef[1] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 2], eqc.polycoef[2])
		    << eqc.name << " polycoef[2] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 3], eqc.polycoef[3])
		    << eqc.name << " polycoef[3] was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 4], eqc.polycoef[4])
		    << eqc.name << " polycoef[3] was not set correctly";
	}
}

// TODO: Should we test changing element1 and/or element2?
// are changes like that valid?
TEST_F(EqualityEnvFixture, SetEqConstraintNotAllowed)
{
	// set eval mode and admin hash
	env_ptr->setEvalMode(true);
	env_ptr->setAdminHash("right_hash");

	mujoco_ros_msgs::SetEqualityConstraintParameters srv;
	srv.request.admin_hash = "wrong_hash";
	mujoco_ros_msgs::EqualityConstraintParameters some_eqc;
	srv.request.parameters = { some_eqc };

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/set_eq_constraint_parameters", srv))
	    << "Set eq constraint service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(EqualityEnvFixture, SetEqConstraintNotProvided)
{
	mujoco_ros_msgs::SetEqualityConstraintParameters srv;

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/set_eq_constraint_parameters", srv))
	    << "Set eq constraint service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(EqualityEnvFixture, SetEqConstraintInvalidName)
{
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(),
	                                               env_ptr->GetHandleNamespace() + "/set_eq_constraint_parameters"))
	    << "Set eq constraint service should be available!";

	mujoco_ros_msgs::EqualityConstraintParameters unknown_eqc;
	unknown_eqc.name = "unknown_eqc";
	mujoco_ros_msgs::SetEqualityConstraintParameters srv;
	srv.request.parameters = { unknown_eqc };

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/set_eq_constraint_parameters", srv))
	    << "Set eq constraint service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(EqualityEnvFixture, SetEqConstraintConnect)
{
	mujoco_ros_msgs::EqualityConstraintParameters connect_eqc;
	connect_eqc.name  = "connect_eq";
	int connect_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "connect_eq");
	EXPECT_NE(connect_eq_id, -1) << "connect eq constraint is not defined in loaded model!";

	mujoco_ros_msgs::SetEqualityConstraintParameters srv;

	connect_eqc.active                      = false;
	connect_eqc.type.value                  = mujoco_ros_msgs::EqualityConstraintType::CONNECT;
	connect_eqc.solver_parameters.dampratio = 0.8;
	connect_eqc.solver_parameters.timeconst = 0.2;
	connect_eqc.solver_parameters.dmin      = 0.7;
	connect_eqc.solver_parameters.dmax      = 0.9;
	connect_eqc.solver_parameters.width     = 0.001;
	connect_eqc.solver_parameters.midpoint  = 0.5;
	connect_eqc.solver_parameters.power     = 3.0;
	// constraint specific parameters
	connect_eqc.element1 = "immovable";
	connect_eqc.element2 = "";
	connect_eqc.anchor.x = 5.0;
	connect_eqc.anchor.y = 3.0;
	connect_eqc.anchor.z = 7.0;

	// Verify values differ to confirm values have changed later on
	compare_eqc_values_with_msg_inequal(m, d, connect_eq_id, connect_eqc);

	srv.request.parameters = { connect_eqc };

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/set_eq_constraint_parameters", srv))
	    << "Set eq constraint service call failed!";
	EXPECT_TRUE(srv.response.success);

	compare_eqc_values_with_msg(m, d, connect_eq_id, connect_eqc);
}

TEST_F(EqualityEnvFixture, SetEqConstraintWeld)
{
	mujoco_ros_msgs::EqualityConstraintParameters weld_eqc;
	weld_eqc.name  = "weld_eq";
	int weld_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "weld_eq");
	EXPECT_NE(weld_eq_id, -1) << "weld eq constraint is not defined in loaded model!";

	mjtNum quat[4] = { -0.634, -0.002, -0.733, 0.244 };
	mju_normalize4(quat);

	mujoco_ros_msgs::SetEqualityConstraintParameters srv;
	weld_eqc.active                      = false;
	weld_eqc.type.value                  = mujoco_ros_msgs::EqualityConstraintType::WELD;
	weld_eqc.solver_parameters.dampratio = 0.8;
	weld_eqc.solver_parameters.timeconst = 0.2;
	weld_eqc.solver_parameters.dmin      = 0.7;
	weld_eqc.solver_parameters.dmax      = 0.9;
	weld_eqc.solver_parameters.width     = 0.001;
	weld_eqc.solver_parameters.midpoint  = 0.5;
	weld_eqc.solver_parameters.power     = 3.0;
	// constraint specific parameters
	weld_eqc.element1              = "immovable";
	weld_eqc.element1              = "";
	weld_eqc.anchor.x              = 5.0;
	weld_eqc.anchor.y              = 3.0;
	weld_eqc.anchor.z              = 7.0;
	weld_eqc.relpose.position.x    = 1.2;
	weld_eqc.relpose.position.y    = 1.3;
	weld_eqc.relpose.position.z    = 1.4;
	weld_eqc.relpose.orientation.w = quat[0];
	weld_eqc.relpose.orientation.x = quat[1];
	weld_eqc.relpose.orientation.y = quat[2];
	weld_eqc.relpose.orientation.z = quat[3];

	// Verify values differ to confirm values have changed later on
	compare_eqc_values_with_msg_inequal(m, d, weld_eq_id, weld_eqc);

	srv.request.parameters = { weld_eqc };

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/set_eq_constraint_parameters", srv))
	    << "Set eq constraint service call failed!";
	EXPECT_TRUE(srv.response.success);

	compare_eqc_values_with_msg(m, d, weld_eq_id, weld_eqc);
}

TEST_F(EqualityEnvFixture, SetEqConstraintJoint)
{
	mujoco_ros_msgs::EqualityConstraintParameters joint_eqc;
	joint_eqc.name  = "joint_eq";
	int joint_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "joint_eq");
	EXPECT_NE(joint_eq_id, -1) << "joint eq constraint is not defined in loaded model!";

	mujoco_ros_msgs::SetEqualityConstraintParameters srv;
	joint_eqc.active                      = false;
	joint_eqc.type.value                  = mujoco_ros_msgs::EqualityConstraintType::JOINT;
	joint_eqc.solver_parameters.dampratio = 0.8;
	joint_eqc.solver_parameters.timeconst = 0.2;
	joint_eqc.solver_parameters.dmin      = 0.7;
	joint_eqc.solver_parameters.dmax      = 0.9;
	joint_eqc.solver_parameters.width     = 0.001;
	joint_eqc.solver_parameters.midpoint  = 0.5;
	joint_eqc.solver_parameters.power     = 3.0;
	// constraint specific parameters
	joint_eqc.element1 = "joint_eq_element1";
	joint_eqc.element2 = "";
	joint_eqc.polycoef = std::vector<double>{ 0.1, 1.0, 0.2, 0.3, 0.4 };

	// Verify values differ to confirm values have changed later on
	compare_eqc_values_with_msg_inequal(m, d, joint_eq_id, joint_eqc);

	srv.request.parameters = { joint_eqc };

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/set_eq_constraint_parameters", srv))
	    << "Set eq constraint service call failed!";
	EXPECT_TRUE(srv.response.success);

	compare_eqc_values_with_msg(m, d, joint_eq_id, joint_eqc);
}

TEST_F(EqualityEnvFixture, SetEqConstraintTendon)
{
	mujoco_ros_msgs::EqualityConstraintParameters tendon_eqc;
	tendon_eqc.name  = "tendon_eq";
	int tendon_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "tendon_eq");
	EXPECT_NE(tendon_eq_id, -1) << "tendon eq constraint is not defined in loaded model!";

	mujoco_ros_msgs::SetEqualityConstraintParameters srv;
	tendon_eqc.active                      = false;
	tendon_eqc.type.value                  = mujoco_ros_msgs::EqualityConstraintType::TENDON;
	tendon_eqc.solver_parameters.dampratio = 0.8;
	tendon_eqc.solver_parameters.timeconst = 0.2;
	tendon_eqc.solver_parameters.dmin      = 0.7;
	tendon_eqc.solver_parameters.dmax      = 0.9;
	tendon_eqc.solver_parameters.width     = 0.001;
	tendon_eqc.solver_parameters.midpoint  = 0.5;
	tendon_eqc.solver_parameters.power     = 3.0;
	// constraint specific parameters
	tendon_eqc.element1 = "tendon_eq_element1";
	tendon_eqc.element2 = "";
	tendon_eqc.polycoef = std::vector<double>{ 0.1, 1.0, 0.2, 0.3, 0.4 };

	// Verify values differ to confirm values have changed later on
	compare_eqc_values_with_msg_inequal(m, d, tendon_eq_id, tendon_eqc);

	srv.request.parameters = { tendon_eqc };

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/set_eq_constraint_parameters", srv))
	    << "Set eq constraint service call failed!";
	EXPECT_TRUE(srv.response.success);

	compare_eqc_values_with_msg(m, d, tendon_eq_id, tendon_eqc);
}

TEST_F(EqualityEnvFixture, SetEqConstraintPartialSuccess)
{
	mujoco_ros_msgs::EqualityConstraintParameters joint_eqc;
	joint_eqc.name  = "joint_eq";
	int joint_eq_id = mj_name2id(m, mjOBJ_EQUALITY, "joint_eq");
	EXPECT_NE(joint_eq_id, -1) << "joint eq constraint is not defined in loaded model!";

	mujoco_ros_msgs::SetEqualityConstraintParameters srv;
	joint_eqc.active                      = false;
	joint_eqc.type.value                  = mujoco_ros_msgs::EqualityConstraintType::JOINT;
	joint_eqc.solver_parameters.dampratio = 0.8;
	joint_eqc.solver_parameters.timeconst = 0.2;
	joint_eqc.solver_parameters.dmin      = 0.7;
	joint_eqc.solver_parameters.dmax      = 0.9;
	joint_eqc.solver_parameters.width     = 0.001;
	joint_eqc.solver_parameters.midpoint  = 0.5;
	joint_eqc.solver_parameters.power     = 3.0;
	// constraint specific parameters
	joint_eqc.element1 = "joint_eq_element1";
	joint_eqc.element2 = "";
	joint_eqc.polycoef = std::vector<double>{ 0.1, 1.0, 0.2, 0.3, 0.4 };

	// Verify values differ to confirm values have changed later on
	compare_eqc_values_with_msg_inequal(m, d, joint_eq_id, joint_eqc);

	mujoco_ros_msgs::EqualityConstraintParameters unknown_eqc;
	unknown_eqc.name       = "unknown_eqc";
	srv.request.parameters = { unknown_eqc, joint_eqc };

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/set_eq_constraint_parameters", srv))
	    << "Set eq constraint service call failed!";
	EXPECT_FALSE(srv.response.success);

	// Verify joint_eqc was set nonetheless
	compare_eqc_values_with_msg(m, d, joint_eq_id, joint_eqc);
}

TEST_F(EqualityEnvFixture, GetEqConstraint)
{
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(),
	                                               env_ptr->GetHandleNamespace() + "/set_eq_constraint_parameters"))
	    << "Set geom properties service should be available!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(),
	                                               env_ptr->GetHandleNamespace() + "/get_eq_constraint_parameters"))
	    << "Ret geom properties service should be available!";

	mujoco_ros_msgs::GetEqualityConstraintParameters srv;
	srv.request.names = { "weld_eq", "tendon_eq", "joint_eq", "connect_eq" };
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/get_eq_constraint_parameters", srv))
	    << "Get eq constraints service call failed!";
	EXPECT_TRUE(srv.response.success);

	for (const auto &eqc : srv.response.parameters) {
		int eq_id = mj_name2id(m, mjOBJ_EQUALITY, eqc.name.c_str());
		EXPECT_NE(eq_id, -1) << eqc.name << " eq constraint is not defined in loaded model!";

		MJR_WARN_STREAM("Comparing values of eq constraint: " << eqc.name);

		compare_eqc_values_with_msg(m, d, eq_id, eqc);
	}

	EXPECT_EQ(srv.response.parameters.size(), 4)
	    << "Expected to find one connect, weld, joint, and tendon constraint each!";
}

TEST_F(PendulumEnvFixture, SetGravityNotAllowed)
{
	mjData *d = env_ptr->getDataPtr();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_gravity"))
	    << "Set gravity service should be available!";

	// set eval mode and admin hash
	env_ptr->setEvalMode(true);
	env_ptr->setAdminHash("right_hash");

	mujoco_ros_msgs::SetGravity srv;
	srv.request.admin_hash = "wrong_hash";

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_gravity", srv))
	    << "Set gravity service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(PendulumEnvFixture, SetGravity)
{
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_gravity"))
	    << "Set gravity service should be available!";

	mujoco_ros_msgs::SetGravity srv;
	srv.request.gravity = { .0, -10.0, .0 };

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_gravity", srv))
	    << "Set gravity service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_DOUBLE_EQ(m->opt.gravity[0], srv.request.gravity[0]) << "Gravity x mismatch";
	EXPECT_DOUBLE_EQ(m->opt.gravity[1], srv.request.gravity[1]) << "Gravity y mismatch";
	EXPECT_DOUBLE_EQ(m->opt.gravity[2], srv.request.gravity[2]) << "Gravity z mismatch";
}

TEST_F(PendulumEnvFixture, GetGravityNotAllowed)
{
	mjData *d = env_ptr->getDataPtr();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_gravity"))
	    << "Get gravity service should be available!";

	// set eval mode and admin hash
	env_ptr->setEvalMode(true);
	env_ptr->setAdminHash("right_hash");

	mujoco_ros_msgs::GetGravity srv;
	srv.request.admin_hash = "wrong_hash";

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_gravity", srv))
	    << "Get gravity service call failed!";
	EXPECT_FALSE(srv.response.success);
}

TEST_F(PendulumEnvFixture, GetGravity)
{
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_gravity"))
	    << "Get gravity service should be available!";

	mujoco_ros_msgs::GetGravity srv;

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_gravity", srv))
	    << "Get gravity service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_DOUBLE_EQ(m->opt.gravity[0], srv.response.gravity[0]) << "Gravity x mismatch";
	EXPECT_DOUBLE_EQ(m->opt.gravity[1], srv.response.gravity[1]) << "Gravity y mismatch";
	EXPECT_DOUBLE_EQ(m->opt.gravity[2], srv.response.gravity[2]) << "Gravity z mismatch";
}

TEST_F(PendulumEnvFixture, GetStateUintReady)
{
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_loading_request_state"))
	    << "Get state service should be available!";

	mujoco_ros_msgs::GetStateUint srv;

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/get_loading_request_state", srv))
	    << "Get state service call failed!";
	EXPECT_EQ(srv.response.state.value, 0) << "State should be ready (0)!";
}

// Depends to much on speed. Might fail on slow machines
/*
TEST_F(PendulumEnvFixture, GetStateUintLoadIssued) {
   EXPECT_TRUE(ros::service::exists(env_ptr->GetHandleNamespace() + "/get_loading_request_state", true))
       << "Get state service should be available!";

   EXPECT_TRUE(ros::service::exists(env_ptr->GetHandleNamespace() + "/reload", true))
       << "Reload service should be available!";

   mujoco_ros_msgs::GetStateUint srv;
   mujoco_ros_msgs::Reload r_srv;

   EXPECT_TRUE(ros::service::call(env_ptr->GetHandleNamespace() + "/reload", r_srv)) << "Reload service call failed!";
   while (env_ptr->GetOperationalStatus() == 0) { // wait for reload to finish
      std::this_thread::yield();
   }

   EXPECT_TRUE(ros::service::call(env_ptr->GetHandleNamespace() + "/get_loading_request_state", srv))
       << "Get state service call failed!";
   EXPECT_GE(srv.response.state.value, 2) << "State should be load issued (2)!";
}

TEST_F(PendulumEnvFixture, GetStateUintLoadInProgress) {
   EXPECT_TRUE(ros::service::exists(env_ptr->GetHandleNamespace() + "/get_loading_request_state", true))
       << "Get state service should be available!";

   EXPECT_TRUE(ros::service::exists(env_ptr->GetHandleNamespace() + "/reload", true))
       << "Reload service should be available!";

   mujoco_ros_msgs::GetStateUint srv;
   mujoco_ros_msgs::Reload r_srv;

   EXPECT_TRUE(ros::service::call(env_ptr->GetHandleNamespace() + "/reload", r_srv)) << "Reload service call failed!";
   while (env_ptr->GetOperationalStatus() != 1) { // wait for reload to finish
      std::this_thread::yield();
   }

   EXPECT_TRUE(ros::service::call(env_ptr->GetHandleNamespace() + "/get_loading_request_state", srv))
       << "Get state service call failed!";
   EXPECT_EQ(srv.response.state.value, 1) << "State should be load issued (1)!";
}
*/

TEST_F(PendulumEnvFixture, LoadInitialJointPositions_Valid)
{
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/load_initial_joint_states"))
	    << "Load initial joint states service should be available!";

	std::map<std::string, std::string> joint_states;
	joint_states.insert({ "balljoint", "0. 1. 0. 0." });
	joint_states.insert({ "joint1", "0.3" });
	joint_states.insert({ "joint2", "0.5" });
	joint_states.insert({ "ball_freejoint", "1. 0.9 0.8 0.0 0.0 0.0 1.0" });

	// verify initial joint states are different
	int ids[4];
	ids[0] = mujoco_ros::util::jointName2id(m, "balljoint");
	ids[1] = mujoco_ros::util::jointName2id(m, "joint1");
	ids[2] = mujoco_ros::util::jointName2id(m, "joint2");
	ids[3] = mujoco_ros::util::jointName2id(m, "ball_freejoint");

	EXPECT_NE(ids[0], -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(ids[1], -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(ids[2], -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(ids[3], -1) << "'ball_freejoint' should be found as joint in model!";

	compare_qpos(d, m->jnt_qposadr[ids[0]], "balljoint", { 1.0, 0.0, 0.0, 0.0 });
	compare_qpos(d, m->jnt_qposadr[ids[1]], "joint1", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[ids[2]], "joint2", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[ids[3]], "ball_freejoint", { 1.0, 0.0, 0.06, 1.0, 0.0, 0.0, 0.0 });

#if MJR_ROS_VERSION == ROS_1
	nh->setParam(env_ptr->GetHandleNamespace() + "/initial_joint_positions/joint_map", joint_states);
#elif MJR_ROS_VERSION == ROS_2
	testing::set_initial_joint_state_maps(nh.get(), joint_states, {});
#endif

	std_srvs::Empty srv;

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/load_initial_joint_states", srv))
	    << "Load initial joint states service call failed!";

#if MJR_ROS_VERSION == ROS_1
	nh->deleteParam(env_ptr->GetHandleNamespace() + "/initial_joint_positions/joint_map");
#endif

	compare_qpos(d, m->jnt_qposadr[ids[0]], "balljoint", { 0.0, 1.0, 0.0, 0.0 });
	compare_qpos(d, m->jnt_qposadr[ids[1]], "joint1", { 0.3 });
	compare_qpos(d, m->jnt_qposadr[ids[2]], "joint2", { 0.5 });
	compare_qpos(d, m->jnt_qposadr[ids[3]], "ball_freejoint", { 1.0, 0.9, 0.8, 0.0, 0.0, 0.0, 1.0 });
}

TEST_F(PendulumEnvFixture, LoadInitialJointPositions_NoParams)
{
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/load_initial_joint_states"))
	    << "Load initial joint states service should be available!";

	std_srvs::Empty srv;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/load_initial_joint_states", srv))
	    << "Load initial joint states service call failed!";

	int ids[4];
	ids[0] = mujoco_ros::util::jointName2id(m, "balljoint");
	ids[1] = mujoco_ros::util::jointName2id(m, "joint1");
	ids[2] = mujoco_ros::util::jointName2id(m, "joint2");
	ids[3] = mujoco_ros::util::jointName2id(m, "ball_freejoint");

	EXPECT_NE(ids[0], -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(ids[1], -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(ids[2], -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(ids[3], -1) << "'ball_freejoint' should be found as joint in model!";

	compare_qpos(d, m->jnt_qposadr[ids[0]], "balljoint", { 1.0, 0.0, 0.0, 0.0 });
	compare_qpos(d, m->jnt_qposadr[ids[1]], "joint1", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[ids[2]], "joint2", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[ids[3]], "ball_freejoint", { 1.0, 0.0, 0.06, 1.0, 0.0, 0.0, 0.0 });
}

TEST_F(PendulumEnvFixture, LoadInitialJointPositions_InvalidJointName)
{
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/load_initial_joint_states"))
	    << "Load initial joint states service should be available!";

	int ids[4];
	ids[0] = mujoco_ros::util::jointName2id(m, "balljoint");
	ids[1] = mujoco_ros::util::jointName2id(m, "joint1");
	ids[2] = mujoco_ros::util::jointName2id(m, "joint2");
	ids[3] = mujoco_ros::util::jointName2id(m, "ball_freejoint");

	EXPECT_NE(ids[0], -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(ids[1], -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(ids[2], -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(ids[3], -1) << "'ball_freejoint' should be found as joint in model!";

	std::map<std::string, std::string> joint_states;
	joint_states.insert({ "invalid_joint", "0.3" });

#if MJR_ROS_VERSION == ROS_1
	nh->setParam(env_ptr->GetHandleNamespace() + "/initial_joint_positions/joint_map", joint_states);
#elif MJR_ROS_VERSION == ROS_2
	testing::set_initial_joint_state_maps(nh.get(), joint_states, {});
#endif

	std_srvs::Empty srv;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/load_initial_joint_states", srv))
	    << "Load initial joint states service call failed!";
#if MJR_ROS_VERSION == ROS_1
	nh->deleteParam(env_ptr->GetHandleNamespace() + "/initial_joint_positions/joint_map");
#elif MJR_ROS_VERSION == ROS_2
	testing::clear_initial_joint_state_maps(nh.get(), true, false);
#endif

	compare_qpos(d, m->jnt_qposadr[ids[0]], "balljoint", { 1.0, 0.0, 0.0, 0.0 });
	compare_qpos(d, m->jnt_qposadr[ids[1]], "joint1", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[ids[2]], "joint2", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[ids[3]], "ball_freejoint", { 1.0, 0.0, 0.06, 1.0, 0.0, 0.0, 0.0 });
}

TEST_F(PendulumEnvFixture, LoadInitialJointPositions_InvalidDOFs)
{
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/load_initial_joint_states"))
	    << "Load initial joint states service should be available!";

	int ids[4];
	ids[0] = mujoco_ros::util::jointName2id(m, "balljoint");
	ids[1] = mujoco_ros::util::jointName2id(m, "joint1");
	ids[2] = mujoco_ros::util::jointName2id(m, "joint2");
	ids[3] = mujoco_ros::util::jointName2id(m, "ball_freejoint");

	EXPECT_NE(ids[0], -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(ids[1], -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(ids[2], -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(ids[3], -1) << "'ball_freejoint' should be found as joint in model!";

	std::map<std::string, std::string> joint_states;
	joint_states.insert({ "balljoint", "0. 1. 0." }); // Invalid DOFs for balljoint
	joint_states.insert({ "joint1", "0.3 0.4" }); // Invalid DOFs for hinge joint
	joint_states.insert({ "ball_freejoint", "0.6" }); // Invalid DOFs for freejoint

#if MJR_ROS_VERSION == ROS_1
	nh->setParam(env_ptr->GetHandleNamespace() + "/initial_joint_positions/joint_map", joint_states);
#elif MJR_ROS_VERSION == ROS_2
	testing::set_initial_joint_state_maps(nh.get(), joint_states, {});
#endif

	std_srvs::Empty srv;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/load_initial_joint_states", srv))
	    << "Load initial joint states service call failed!";
#if MJR_ROS_VERSION == ROS_1
	nh->deleteParam(env_ptr->GetHandleNamespace() + "/initial_joint_positions/joint_map");
#elif MJR_ROS_VERSION == ROS_2
	testing::clear_initial_joint_state_maps(nh.get(), true, false);
#endif

	compare_qpos(d, m->jnt_qposadr[ids[0]], "balljoint", { 1.0, 0.0, 0.0, 0.0 });
	compare_qpos(d, m->jnt_qposadr[ids[1]], "joint1", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[ids[2]], "joint2", { 0.0 });
	compare_qpos(d, m->jnt_qposadr[ids[3]], "ball_freejoint", { 1.0, 0.0, 0.06, 1.0, 0.0, 0.0, 0.0 });
}

TEST_F(PendulumEnvFixture, LoadInitialJointVels_Valid)
{
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/load_initial_joint_states"))
	    << "Load initial joint states service should be available!";

	std::map<std::string, std::string> joint_states;
	joint_states.insert({ "balljoint", "0.2 1. 0.1" });
	joint_states.insert({ "joint1", "0.3" });
	joint_states.insert({ "joint2", "0.5" });
	joint_states.insert({ "ball_freejoint", "1. 0.9 0.8 0.2 0.3 1.0" });

	int ids[4];
	ids[0] = mujoco_ros::util::jointName2id(m, "balljoint");
	ids[1] = mujoco_ros::util::jointName2id(m, "joint1");
	ids[2] = mujoco_ros::util::jointName2id(m, "joint2");
	ids[3] = mujoco_ros::util::jointName2id(m, "ball_freejoint");

	EXPECT_NE(ids[0], -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(ids[1], -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(ids[2], -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(ids[3], -1) << "'ball_freejoint' should be found as joint in model!";

	compare_qvel(d, m->jnt_dofadr[ids[0]], "balljoint", { 0.0, 0.0, 0.0 });
	compare_qvel(d, m->jnt_dofadr[ids[1]], "joint1", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[ids[2]], "joint2", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[ids[3]], "ball_freejoint", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });

#if MJR_ROS_VERSION == ROS_1
	nh->setParam(env_ptr->GetHandleNamespace() + "/initial_joint_velocities/joint_map", joint_states);
#elif MJR_ROS_VERSION == ROS_2
	testing::set_initial_joint_state_maps(nh.get(), {}, joint_states);
#endif

	std_srvs::Empty srv;

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/load_initial_joint_states", srv))
	    << "Load initial joint states service call failed!";
#if MJR_ROS_VERSION == ROS_1
	nh->deleteParam(env_ptr->GetHandleNamespace() + "/initial_joint_velocities/joint_map");
#elif MJR_ROS_VERSION == ROS_2
	testing::clear_initial_joint_state_maps(nh.get(), false, true);
#endif

	compare_qvel(d, m->jnt_dofadr[ids[0]], "balljoint", { 0.2, 1.0, 0.1 });
	compare_qvel(d, m->jnt_dofadr[ids[1]], "joint1", { 0.3 });
	compare_qvel(d, m->jnt_dofadr[ids[2]], "joint2", { 0.5 });
	compare_qvel(d, m->jnt_dofadr[ids[3]], "ball_freejoint", { 1.0, 0.9, 0.8, 0.2, 0.3, 1.0 });
}

TEST_F(PendulumEnvFixture, LoadInitialJointVels_NoParams)
{
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/load_initial_joint_states"))
	    << "Load initial joint states service should be available!";

	std_srvs::Empty srv;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/load_initial_joint_states", srv))
	    << "Load initial joint states service call failed!";

	int ids[4];
	ids[0] = mujoco_ros::util::jointName2id(m, "balljoint");
	ids[1] = mujoco_ros::util::jointName2id(m, "joint1");
	ids[2] = mujoco_ros::util::jointName2id(m, "joint2");
	ids[3] = mujoco_ros::util::jointName2id(m, "ball_freejoint");

	EXPECT_NE(ids[0], -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(ids[1], -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(ids[2], -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(ids[3], -1) << "'ball_freejoint' should be found as joint in model!";

	compare_qvel(d, m->jnt_dofadr[ids[0]], "balljoint", { 0.0, 0.0, 0.0 });
	compare_qvel(d, m->jnt_dofadr[ids[1]], "joint1", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[ids[2]], "joint2", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[ids[3]], "ball_freejoint", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
}

TEST_F(PendulumEnvFixture, LoadInitialJointVels_InvalidJointName)
{
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/load_initial_joint_states"))
	    << "Load initial joint states service should be available!";

	int ids[4];
	ids[0] = mujoco_ros::util::jointName2id(m, "balljoint");
	ids[1] = mujoco_ros::util::jointName2id(m, "joint1");
	ids[2] = mujoco_ros::util::jointName2id(m, "joint2");
	ids[3] = mujoco_ros::util::jointName2id(m, "ball_freejoint");

	EXPECT_NE(ids[0], -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(ids[1], -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(ids[2], -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(ids[3], -1) << "'ball_freejoint' should be found as joint in model!";

	std::map<std::string, std::string> joint_states;
	joint_states.insert({ "invalid_joint", "0.3" });

#if MJR_ROS_VERSION == ROS_1
	nh->setParam(env_ptr->GetHandleNamespace() + "/initial_joint_velocities/joint_map", joint_states);
#elif MJR_ROS_VERSION == ROS_2
	testing::set_initial_joint_state_maps(nh.get(), {}, joint_states);
#endif

	std_srvs::Empty srv;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/load_initial_joint_states", srv))
	    << "Load initial joint states service call failed!";
#if MJR_ROS_VERSION == ROS_1
	nh->deleteParam(env_ptr->GetHandleNamespace() + "/initial_joint_velocities/joint_map");
#elif MJR_ROS_VERSION == ROS_2
	testing::clear_initial_joint_state_maps(nh.get(), false, true);
#endif

	compare_qvel(d, m->jnt_dofadr[ids[0]], "balljoint", { 0.0, 0.0, 0.0 });
	compare_qvel(d, m->jnt_dofadr[ids[1]], "joint1", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[ids[2]], "joint2", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[ids[3]], "ball_freejoint", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
}

TEST_F(PendulumEnvFixture, LoadInitialJointVels_InvalidDOFs)
{
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
	EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(
	    ::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/load_initial_joint_states"))
	    << "Load initial joint states service should be available!";

	int ids[4];
	ids[0] = mujoco_ros::util::jointName2id(m, "balljoint");
	ids[1] = mujoco_ros::util::jointName2id(m, "joint1");
	ids[2] = mujoco_ros::util::jointName2id(m, "joint2");
	ids[3] = mujoco_ros::util::jointName2id(m, "ball_freejoint");

	EXPECT_NE(ids[0], -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(ids[1], -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(ids[2], -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(ids[3], -1) << "'ball_freejoint' should be found as joint in model!";

	std::map<std::string, std::string> joint_states;
	joint_states.insert({ "balljoint", "0. 1." }); // Invalid DOFs for balljoint
	joint_states.insert({ "joint1", "0.3 0.4" }); // Invalid DOFs for hinge joint
	joint_states.insert({ "ball_freejoint", "0.6" }); // Invalid DOFs for freejoint

#if MJR_ROS_VERSION == ROS_1
	nh->setParam(env_ptr->GetHandleNamespace() + "/initial_joint_velocities/joint_map", joint_states);
#elif MJR_ROS_VERSION == ROS_2
	testing::set_initial_joint_state_maps(nh.get(), {}, joint_states);
#endif

	std_srvs::Empty srv;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(),
	                                             env_ptr->GetHandleNamespace() + "/load_initial_joint_states", srv))
	    << "Load initial joint states service call failed!";
#if MJR_ROS_VERSION == ROS_1
	nh->deleteParam(env_ptr->GetHandleNamespace() + "/initial_joint_velocities/joint_map");
#elif MJR_ROS_VERSION == ROS_2
	testing::clear_initial_joint_state_maps(nh.get(), false, true);
#endif

	compare_qvel(d, m->jnt_dofadr[ids[0]], "balljoint", { 0.0, 0.0, 0.0 });
	compare_qvel(d, m->jnt_dofadr[ids[1]], "joint1", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[ids[2]], "joint2", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[ids[3]], "ball_freejoint", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
}

TEST_F(PendulumEnvFixture, SetRTFactor_NotAllowed)
{
	// Set eval mode and admin hash
	env_ptr->setEvalMode(true);
	env_ptr->setAdminHash("right_hash");

	int initial_rt_index = env_ptr->settings_.real_time_index;

	mujoco_ros_msgs::SetFloat srv;
	srv.request.value      = 1.5; // Increase real-time factor
	srv.request.admin_hash = "wrong_hash";

	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_rt_factor"))
	    << "Set RT factor service call failed!";

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_rt_factor", srv))
	    << "Set RT factor service call failed!";
	EXPECT_FALSE(srv.response.success) << "Service call should not be successful!";

	EXPECT_EQ(env_ptr->settings_.real_time_index, initial_rt_index) << "Real-time factor should not have changed!";
}

TEST_F(PendulumEnvFixture, SetRTFactor_Increase)
{
	mujoco_ros_msgs::SetFloat srv;
	srv.request.value = 1.5; // Increase real-time factor
	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_rt_factor"))
	    << "Set RT factor service call failed!";

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_rt_factor", srv))
	    << "Set RT factor service call failed!";
	EXPECT_TRUE(srv.response.success) << "Service call was not successful!";

	EXPECT_FLOAT_EQ(env_ptr->percentRealTime[env_ptr->settings_.real_time_index], 150.f)
	    << "Real-time factor should be set to 1.5!";
}

TEST_F(PendulumEnvFixture, SetRTFactor_Decrease)
{
	mujoco_ros_msgs::SetFloat srv;
	srv.request.value = 0.5; // Decrease real-time factor

	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_rt_factor"))
	    << "Set RT factor service call failed!";
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_rt_factor", srv))
	    << "Set RT factor service call failed!";
	EXPECT_TRUE(srv.response.success) << "Service call was not successful!";
	EXPECT_FLOAT_EQ(env_ptr->percentRealTime[env_ptr->settings_.real_time_index], 50.f)
	    << "Real-time factor should be set to 0.5!";
}

TEST_F(PendulumEnvFixture, SetRTFactor_UnboundMode)
{
	mujoco_ros_msgs::SetFloat srv;
	srv.request.value = -1; // Set to unbound mode

	EXPECT_TRUE(::testing::service_exists_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_rt_factor"))
	    << "Set RT factor service call failed!";
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_rt_factor", srv))
	    << "Set RT factor service call failed!";
	EXPECT_TRUE(srv.response.success) << "Service call was not successful!";
	EXPECT_EQ(env_ptr->settings_.real_time_index, 0) << "Real-time factor should be set to unbound mode!";
}

TEST_F(PendulumEnvFixture, SetRTFactor_OutOfBounds)
{
	mujoco_ros_msgs::SetFloat srv;
	srv.request.value = 1000; // Set to a value outside the boundaries

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_rt_factor", srv))
	    << "Set RT factor service call failed!";
	EXPECT_TRUE(srv.response.success) << "Service call was not successful!";
	EXPECT_FLOAT_EQ(env_ptr->percentRealTime[env_ptr->settings_.real_time_index], 2000.0f)
	    << "Real-time factor should be clipped to the maximum boundary value!";
}

TEST_F(PendulumEnvFixture, SetRTFactor_RoundUpClosest)
{
	mujoco_ros_msgs::SetFloat srv;
	srv.request.value = 0.45; // Set to a value outside the boundaries

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_rt_factor", srv))
	    << "Set RT factor service call failed!";
	EXPECT_TRUE(srv.response.success) << "Service call was not successful!";
	EXPECT_FLOAT_EQ(env_ptr->percentRealTime[env_ptr->settings_.real_time_index], 50.0f)
	    << "Real-time factor should be clipped to the maximum boundary value!";
}

TEST_F(PendulumEnvFixture, SetRTFactor_RoundDownClosest)
{
	mujoco_ros_msgs::SetFloat srv;
	srv.request.value = 0.44; // Set to a value outside the boundaries

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_rt_factor", srv))
	    << "Set RT factor service call failed!";
	EXPECT_TRUE(srv.response.success) << "Service call was not successful!";
	EXPECT_FLOAT_EQ(env_ptr->percentRealTime[env_ptr->settings_.real_time_index], 40.0f)
	    << "Real-time factor should be clipped to the maximum boundary value!";
}

TEST_F(PendulumEnvFixture, GetSimInfo_ModelPath)
{
	mujoco_ros_msgs::GetSimInfo srv;

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_sim_info", srv))
	    << "Get sim info service call failed!";
	EXPECT_TRUE(srv.response.state.model_path.find("pendulum_world.xml") != std::string::npos)
	    << "Model path should contain 'pendulum_world.xml'!";
	EXPECT_TRUE(srv.response.state.model_valid) << "Model should be valid!";
}

TEST_F(PendulumEnvFixture, GetSimInfo_LoadCountIncreases)
{
	mujoco_ros_msgs::Reload reload_srv;
	mujoco_ros_msgs::GetSimInfo sim_info_srv;

	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_sim_info", sim_info_srv))
	    << "Get sim info service call failed!";
	int initial_load_count = sim_info_srv.response.state.load_count;

	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/reload", reload_srv))
	    << "Reload service call failed!";
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_sim_info", sim_info_srv))
	    << "Get sim info service call failed!";
	EXPECT_GT(sim_info_srv.response.state.load_count, initial_load_count) << "Load count should increase after reload!";
}

TEST_F(PendulumEnvFixture, GetSimInfo_PauseStateChanges)
{
	mujoco_ros_msgs::SetPause pause_srv;
	mujoco_ros_msgs::GetSimInfo sim_info_srv;

	pause_srv.request.paused = true;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_pause", pause_srv))
	    << "Set pause service call failed!";
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_sim_info", sim_info_srv))
	    << "Get sim info service call failed!";
	EXPECT_TRUE(sim_info_srv.response.state.paused) << "Simulation should be paused!";

	pause_srv.request.paused = false;
	EXPECT_TRUE(::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_pause", pause_srv))
	    << "Set pause service call failed!";
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_sim_info", sim_info_srv))
	    << "Get sim info service call failed!";
	EXPECT_FALSE(sim_info_srv.response.state.paused) << "Simulation should be unpaused!";
}

TEST_F(PendulumEnvFixture, GetSimInfo_RTSettingChanges)
{
	mujoco_ros_msgs::SetFloat rt_factor_srv;
	rt_factor_srv.request.value = 1.5; // Change real-time factor

	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/set_rt_factor", rt_factor_srv))
	    << "Set RT factor service call failed!";
	EXPECT_TRUE(rt_factor_srv.response.success) << "Service call was not successful!";

	mujoco_ros_msgs::GetSimInfo sim_info_srv;
	EXPECT_TRUE(
	    ::testing::service_call_for_test(env_ptr.get(), env_ptr->GetHandleNamespace() + "/get_sim_info", sim_info_srv))
	    << "Get sim info service call failed!";
	EXPECT_FLOAT_EQ(sim_info_srv.response.state.rt_setting, 1.5f)
	    << "RT setting should change when RT factor is changed!";
}

// ROS 1 uses dynamic reconfigure; ROS 2 mirrors the same runtime knobs as node parameters.
#if MJR_ROS_VERSION == ROS_1
TEST_F(BaseEnvFixture, DynParamEnumsMatchMJEnums)
{
	// Integrators
	ASSERT_EQ(mujoco_ros::SimParams_Euler, mjINT_EULER);
	ASSERT_EQ(mujoco_ros::SimParams_RK4, mjINT_RK4);
	ASSERT_EQ(mujoco_ros::SimParams_Implicit, mjINT_IMPLICIT);
	ASSERT_EQ(mujoco_ros::SimParams_Implicitfast, mjINT_IMPLICITFAST);

	// Cones
	ASSERT_EQ(mujoco_ros::SimParams_Elliptic, mjCONE_ELLIPTIC);
	ASSERT_EQ(mujoco_ros::SimParams_Pyramidal, mjCONE_PYRAMIDAL);

	// Jacobians
	ASSERT_EQ(mujoco_ros::SimParams_Dense, mjJAC_DENSE);
	ASSERT_EQ(mujoco_ros::SimParams_Sparse, mjJAC_SPARSE);
	ASSERT_EQ(mujoco_ros::SimParams_Auto, mjJAC_AUTO);

	// Solvers
	ASSERT_EQ(mujoco_ros::SimParams_PGS, mjSOL_PGS);
	ASSERT_EQ(mujoco_ros::SimParams_CG, mjSOL_CG);
	ASSERT_EQ(mujoco_ros::SimParams_Newton, mjSOL_NEWTON);
}

TEST_F(PendulumEnvFixture, DynamicReconfigureServiceExists)
{
	EXPECT_NE(env_ptr->GetParamServer(), nullptr) << "Parameter server should be initialized!";
	EXPECT_TRUE(ros::service::waitForService(env_ptr->GetHandleNamespace() + "/set_parameters", 1000))
	    << "Service should be available!";
}

TEST_F(PendulumEnvFixture, DynamicReconfigureSingleParam)
{
	// Test dynamic reconfigure
	dynamic_reconfigure::ReconfigureRequest req;
	dynamic_reconfigure::ReconfigureResponse res;
	dynamic_reconfigure::DoubleParameter param;
	dynamic_reconfigure::Config conf;

	param.name  = "timestep";
	param.value = 0.002;
	conf.doubles.emplace_back(param);
	req.config = conf;

	EXPECT_TRUE(ros::service::call(env_ptr->GetHandleNamespace() + "/set_parameters", req, res))
	    << "Service call should not fail!";
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.timestep, 0.002) << "Timestep should have been updated!";
}

TEST_F(PendulumEnvFixture, DynamicReconfigureAllParams)
{
	// Test dynamic reconfigure with all parameters
	dynamic_reconfigure::ReconfigureRequest req;
	dynamic_reconfigure::ReconfigureResponse res;
	dynamic_reconfigure::StrParameter string_param;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::IntParameter int_param;
	dynamic_reconfigure::BoolParameter bool_param;
	dynamic_reconfigure::Config conf;

	string_param.name  = "admin_hash";
	string_param.value = "new_hash";
	conf.strs.emplace_back(string_param);

	// Physics
	int_param.name  = "integrator";
	int_param.value = 2;
	conf.ints.emplace_back(int_param);

	int_param.name  = "cone";
	int_param.value = 0;
	conf.ints.emplace_back(int_param);

	int_param.name  = "jacobian";
	int_param.value = 1;
	conf.ints.emplace_back(int_param);

	int_param.name  = "solver";
	int_param.value = 1;
	conf.ints.emplace_back(int_param);

	// alg_params
	double_param.name  = "timestep";
	double_param.value = 0.002;
	conf.doubles.emplace_back(double_param);

	int_param.name  = "iterations";
	int_param.value = 200;
	conf.ints.emplace_back(int_param);

	double_param.name  = "tolerance";
	double_param.value = 1e-7;
	conf.doubles.emplace_back(double_param);

	int_param.name  = "ls_iter";
	int_param.value = 60;
	conf.ints.emplace_back(int_param);

	double_param.name  = "ls_tol";
	double_param.value = 0.02;
	conf.doubles.emplace_back(double_param);

	int_param.name  = "noslip_iter";
	int_param.value = 10;
	conf.ints.emplace_back(int_param);

	double_param.name  = "noslip_tol";
	double_param.value = 1e-5;
	conf.doubles.emplace_back(double_param);

	int_param.name  = "ccd_iter";
	int_param.value = 60;
	conf.ints.emplace_back(int_param);

	double_param.name  = "ccd_tol";
	double_param.value = 1e-5;
	conf.doubles.emplace_back(double_param);

	int_param.name  = "sdf_iter";
	int_param.value = 15;
	conf.ints.emplace_back(int_param);

	int_param.name  = "sdf_init";
	int_param.value = 50;
	conf.ints.emplace_back(int_param);

	// phy_params
	string_param.name  = "gravity";
	string_param.value = "9.81 1 2";
	conf.strs.emplace_back(string_param);

	string_param.name  = "wind";
	string_param.value = "1 1 1";
	conf.strs.emplace_back(string_param);

	string_param.name  = "magnetic";
	string_param.value = "0.1 0.1 0.1";
	conf.strs.emplace_back(string_param);

	double_param.name  = "density";
	double_param.value = 1.2;
	conf.doubles.emplace_back(double_param);

	double_param.name  = "viscosity";
	double_param.value = 0.00003;
	conf.doubles.emplace_back(double_param);

	double_param.name  = "impratio";
	double_param.value = 1.5;
	conf.doubles.emplace_back(double_param);

	// disable flags
	bool_param.name  = "constraint_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "equality_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "frictionloss_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "limit_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "contact_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "passive_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "gravity_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "clampctrl_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "warmstart_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "filterparent_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "actuation_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "refsafe_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "sensor_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "midphase_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "eulerdamp_disabled";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	// enbale flags
	bool_param.name  = "override_contacts";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "energy";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "fwd_inv";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "inv_discrete";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "multiccd";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	bool_param.name  = "island";
	bool_param.value = true;
	conf.bools.emplace_back(bool_param);

	// contact override
	double_param.name  = "margin";
	double_param.value = 0.5;
	conf.doubles.emplace_back(double_param);

	string_param.name  = "solimp";
	string_param.value = "0.8 0.9 0.1";
	conf.strs.emplace_back(string_param);

	string_param.name  = "solref";
	string_param.value = "0.03 1.1";
	conf.strs.emplace_back(string_param);

	string_param.name  = "friction";
	string_param.value = "0.5 0.5 0.1 0.1";
	conf.strs.emplace_back(string_param);

	req.config = conf;

	EXPECT_NE(env_ptr->getModelPtr()->opt.integrator, 2) << "Integrator is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.cone, 0) << "Cone is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.jacobian, 1) << "Jacobian is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.solver, 1) << "Solver is already set to the target value!";

	EXPECT_NE(env_ptr->getModelPtr()->opt.timestep, 0.002) << "Timestep is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.iterations, 200) << "Iterations are already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.tolerance, 1e-7) << "Tolerance is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.ls_iterations, 60) << "LS iterations are already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.ls_tolerance, 0.02) << "LS tolerance is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.noslip_iterations, 10)
	    << "No-slip iterations are already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.noslip_tolerance, 1e-5)
	    << "No-slip tolerance is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.ccd_iterations, 60) << "CCD iterations are already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.ccd_tolerance, 1e-5) << "CCD tolerance is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.sdf_iterations, 15) << "SDF iterations are already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.sdf_initpoints, 50) << "SDF init is already set to the target value!";

	EXPECT_NE(env_ptr->getModelPtr()->opt.gravity[0], 9.81) << "Gravity is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.gravity[1], 1) << "Gravity is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.gravity[2], 2) << "Gravity is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.wind[0], 1) << "Wind is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.wind[1], 1) << "Wind is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.wind[2], 1) << "Wind is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.magnetic[0], 0.1) << "Magnetic is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.magnetic[1], 0.1) << "Magnetic is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.magnetic[2], 0.1) << "Magnetic is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.density, 1.2) << "Density is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.viscosity, 0.00003) << "Viscosity is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.impratio, 1.5) << "Imp ratio is already set to the target value!";

	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_CONSTRAINT) << "Constraint is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_EQUALITY) << "Equality is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_FRICTIONLOSS) << "Friction loss is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_LIMIT) << "Limit is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_CONTACT) << "Contact is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_PASSIVE) << "Passive is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_GRAVITY) << "Gravity is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_CLAMPCTRL) << "Clamp control is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_WARMSTART) << "Warm start is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_FILTERPARENT) << "Filter parent is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_ACTUATION) << "Actuation is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_REFSAFE) << "Ref safe is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_SENSOR) << "Sensor is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_MIDPHASE) << "Midphase is already disabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_EULERDAMP) << "Euler damp is already disabled!";

	EXPECT_FALSE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_OVERRIDE) << "Override contacts are already enabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_ENERGY) << "Energy is already enabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_FWDINV) << "Forward inverse is already enabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_INVDISCRETE) << "Inverse discrete is already enabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_MULTICCD) << "Multi CCD is already enabled!";
	EXPECT_FALSE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_ISLAND) << "Island is already enabled!";

	EXPECT_NE(env_ptr->getModelPtr()->opt.o_margin, 0.5) << "Margin is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.o_solimp[0], 0.8) << "Solimp is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.o_solimp[1], 0.9) << "Solimp is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.o_solimp[2], 0.1) << "Solimp is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.o_solref[0], 0.03) << "Solref is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.o_solref[1], 1.1) << "Solref is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.o_friction[0], 0.5) << "Friction is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.o_friction[1], 0.5) << "Friction is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.o_friction[2], 0.1) << "Friction is already set to the target value!";
	EXPECT_NE(env_ptr->getModelPtr()->opt.o_friction[3], 0.1) << "Friction is already set to the target value!";

	EXPECT_TRUE(ros::service::call(env_ptr->GetHandleNamespace() + "/set_parameters", req, res));

	// Wait for physics thread to update
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	std::lock_guard<std::recursive_mutex> lock(env_ptr->physics_thread_mutex_);

	EXPECT_STREQ(env_ptr->settings_.admin_hash, std::string("new_hash").c_str())
	    << "Eval mode should have been updated!";

	EXPECT_EQ(env_ptr->getModelPtr()->opt.integrator, 2) << "Integrator should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.cone, 0) << "Cone should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.jacobian, 1) << "Jacobian should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.solver, 1) << "Solver should have been updated!";

	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.timestep, 0.002) << "Timestep should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.iterations, 200) << "Iterations should have been updated!";
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.tolerance, 1e-7) << "Tolerance should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.ls_iterations, 60) << "LS iterations should have been updated!";
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.ls_tolerance, 0.02) << "LS tolerance should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.noslip_iterations, 10) << "No-slip iterations should have been updated!";
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.noslip_tolerance, 1e-5)
	    << "No-slip tolerance should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.ccd_iterations, 60) << "CCD iterations should have been updated!";
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.ccd_tolerance, 1e-5) << "CCD tolerance should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.sdf_iterations, 15) << "SDF iterations should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.sdf_initpoints, 50) << "SDF init should have been updated!";

	EXPECT_EQ(env_ptr->getModelPtr()->opt.gravity[0], 9.81) << "Gravity should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.gravity[1], 1) << "Gravity should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.gravity[2], 2) << "Gravity should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.wind[0], 1) << "Wind should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.wind[1], 1) << "Wind should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.wind[2], 1) << "Wind should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.magnetic[0], 0.1) << "Magnetic should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.magnetic[1], 0.1) << "Magnetic should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.magnetic[2], 0.1) << "Magnetic should have been updated!";
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.density, 1.2) << "Density should have been updated!";
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.viscosity, 0.00003) << "Viscosity should have been updated!";
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.impratio, 1.5) << "Imp ratio should have been updated!";

	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_CONSTRAINT) << "Constraint should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_EQUALITY) << "Equality should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_FRICTIONLOSS)
	    << "Friction loss should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_LIMIT) << "Limit should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_CONTACT) << "Contact should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_PASSIVE) << "Passive should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_GRAVITY) << "Gravity should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_CLAMPCTRL)
	    << "Clamp control should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_WARMSTART) << "Warm start should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_FILTERPARENT)
	    << "Filter parent should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_ACTUATION) << "Actuation should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_REFSAFE) << "Ref safe should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_SENSOR) << "Sensor should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_MIDPHASE) << "Midphase should have been disabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_EULERDAMP) << "Euler damp should have been disabled!";

	EXPECT_TRUE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_OVERRIDE)
	    << "Override contacts should have been enabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_ENERGY) << "Energy should have been enabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_FWDINV) << "Forward inverse should have been enabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_INVDISCRETE)
	    << "Inverse discrete should have been enabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_MULTICCD) << "Multi CCD should have been enabled!";
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_ISLAND) << "Island should have been enabled!";

	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_margin, 0.5) << "Margin should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_solimp[0], 0.8) << "Solimp should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_solimp[1], 0.9) << "Solimp should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_solimp[2], 0.1) << "Solimp should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_solref[0], 0.03) << "Solref should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_solref[1], 1.1) << "Solref should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_friction[0], 0.5) << "Friction should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_friction[1], 0.5) << "Friction should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_friction[2], 0.1) << "Friction should have been updated!";
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_friction[3], 0.1) << "Friction should have been updated!";
}
#else // MJR_ROS_VERSION == ROS_2
TEST_F(PendulumEnvFixture, RuntimeParametersExistAndSyncFromModel)
{
	ASSERT_TRUE(env_ptr->has_parameter("running"));
	ASSERT_TRUE(env_ptr->has_parameter("admin_hash"));
	ASSERT_TRUE(env_ptr->has_parameter("timestep"));
	ASSERT_TRUE(env_ptr->has_parameter("gravity"));
	ASSERT_TRUE(env_ptr->has_parameter("constraint_disabled"));
	ASSERT_TRUE(env_ptr->has_parameter("override_contacts"));
	ASSERT_TRUE(env_ptr->has_parameter("solimp"));

	EXPECT_EQ(env_ptr->get_parameter("running").as_bool(), env_ptr->GetControlSnapshot().running);
	EXPECT_DOUBLE_EQ(env_ptr->get_parameter("timestep").as_double(), env_ptr->getModelPtr()->opt.timestep);
	EXPECT_EQ(env_ptr->get_parameter("integrator").as_int(), env_ptr->getModelPtr()->opt.integrator);
}

TEST_F(PendulumEnvFixture, RuntimeParameterDescriptors)
{
	const auto integrator_descriptor = env_ptr->describe_parameter("integrator");
	ASSERT_EQ(integrator_descriptor.integer_range.size(), 1u);
	EXPECT_EQ(integrator_descriptor.integer_range.front().from_value, 0);
	EXPECT_EQ(integrator_descriptor.integer_range.front().to_value, 3);
	EXPECT_NE(integrator_descriptor.additional_constraints.find("Euler"), std::string::npos);
	EXPECT_NE(integrator_descriptor.additional_constraints.find("Implicitfast"), std::string::npos);
	EXPECT_FALSE(integrator_descriptor.read_only);

	const auto cone_descriptor = env_ptr->describe_parameter("cone");
	ASSERT_EQ(cone_descriptor.integer_range.size(), 1u);
	EXPECT_EQ(cone_descriptor.integer_range.front().from_value, 0);
	EXPECT_EQ(cone_descriptor.integer_range.front().to_value, 1);
	EXPECT_NE(cone_descriptor.additional_constraints.find("Pyramidal"), std::string::npos);
	EXPECT_NE(cone_descriptor.additional_constraints.find("Elliptic"), std::string::npos);

	const auto jacobian_descriptor = env_ptr->describe_parameter("jacobian");
	ASSERT_EQ(jacobian_descriptor.integer_range.size(), 1u);
	EXPECT_EQ(jacobian_descriptor.integer_range.front().from_value, 0);
	EXPECT_EQ(jacobian_descriptor.integer_range.front().to_value, 2);
	EXPECT_NE(jacobian_descriptor.additional_constraints.find("Dense"), std::string::npos);
	EXPECT_NE(jacobian_descriptor.additional_constraints.find("Auto"), std::string::npos);

	const auto solver_descriptor = env_ptr->describe_parameter("solver");
	ASSERT_EQ(solver_descriptor.integer_range.size(), 1u);
	EXPECT_EQ(solver_descriptor.integer_range.front().from_value, 0);
	EXPECT_EQ(solver_descriptor.integer_range.front().to_value, 2);
	EXPECT_NE(solver_descriptor.additional_constraints.find("PGS"), std::string::npos);
	EXPECT_NE(solver_descriptor.additional_constraints.find("Newton"), std::string::npos);
}

TEST_F(PendulumEnvFixture, RuntimeParameterSingleParam)
{
	const auto result = env_ptr->set_parameters_atomically({ rclcpp::Parameter("timestep", 0.002) });

	ASSERT_TRUE(result.successful) << result.reason;
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.timestep, 0.002) << "Timestep should have been updated!";
}

TEST_F(PendulumEnvFixture, RuntimeParameterSyncsModelSideChanges)
{
	mjtNum gravity[3] = { 1.0, 2.0, 3.0 };
	ASSERT_TRUE(env_ptr->SetGravity(gravity));

	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
	while (env_ptr->get_parameter("gravity").as_string() != "1.000000 2.000000 3.000000" &&
	       std::chrono::steady_clock::now() < deadline) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	EXPECT_EQ(env_ptr->get_parameter("gravity").as_string(), "1.000000 2.000000 3.000000");
}

TEST_F(PendulumEnvFixture, RuntimeEnumParametersUpdateAndRejectInvalidValues)
{
	auto result =
	    env_ptr->set_parameters_atomically({ rclcpp::Parameter("integrator", 1), rclcpp::Parameter("cone", 0),
	                                         rclcpp::Parameter("jacobian", 1), rclcpp::Parameter("solver", 1) });
	ASSERT_TRUE(result.successful) << result.reason;
	EXPECT_EQ(env_ptr->getModelPtr()->opt.integrator, 1);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.cone, 0);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.jacobian, 1);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.solver, 1);

	const auto old_integrator = env_ptr->getModelPtr()->opt.integrator;
	result                    = env_ptr->set_parameters_atomically({ rclcpp::Parameter("integrator", 99) });
	EXPECT_FALSE(result.successful);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.integrator, old_integrator);

	const auto old_solver = env_ptr->getModelPtr()->opt.solver;
	result                = env_ptr->set_parameters_atomically({ rclcpp::Parameter("solver", -1) });
	EXPECT_FALSE(result.successful);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.solver, old_solver);
}

TEST_F(PendulumEnvFixture, RuntimeParameterAllParams)
{
	const auto result = env_ptr->set_parameters_atomically({
	    rclcpp::Parameter("admin_hash", std::string("new_hash")),
	    rclcpp::Parameter("running", false),
	    rclcpp::Parameter("integrator", 2),
	    rclcpp::Parameter("cone", 0),
	    rclcpp::Parameter("jacobian", 1),
	    rclcpp::Parameter("solver", 1),
	    rclcpp::Parameter("timestep", 0.002),
	    rclcpp::Parameter("iterations", 200),
	    rclcpp::Parameter("tolerance", 1e-7),
	    rclcpp::Parameter("ls_iter", 60),
	    rclcpp::Parameter("ls_tol", 0.02),
	    rclcpp::Parameter("noslip_iter", 10),
	    rclcpp::Parameter("noslip_tol", 1e-5),
	    rclcpp::Parameter("ccd_iter", 60),
	    rclcpp::Parameter("ccd_tol", 1e-5),
	    rclcpp::Parameter("sdf_iter", 15),
	    rclcpp::Parameter("sdf_init", 50),
	    rclcpp::Parameter("gravity", std::string("9.81 1 2")),
	    rclcpp::Parameter("wind", std::string("1 1 1")),
	    rclcpp::Parameter("magnetic", std::string("0.1 0.1 0.1")),
	    rclcpp::Parameter("density", 1.2),
	    rclcpp::Parameter("viscosity", 0.00003),
	    rclcpp::Parameter("impratio", 1.5),
	    rclcpp::Parameter("constraint_disabled", true),
	    rclcpp::Parameter("equality_disabled", true),
	    rclcpp::Parameter("frictionloss_disabled", true),
	    rclcpp::Parameter("limit_disabled", true),
	    rclcpp::Parameter("contact_disabled", true),
	    rclcpp::Parameter("passive_disabled", true),
	    rclcpp::Parameter("gravity_disabled", true),
	    rclcpp::Parameter("clampctrl_disabled", true),
	    rclcpp::Parameter("warmstart_disabled", true),
	    rclcpp::Parameter("filterparent_disabled", true),
	    rclcpp::Parameter("actuation_disabled", true),
	    rclcpp::Parameter("refsafe_disabled", true),
	    rclcpp::Parameter("sensor_disabled", true),
	    rclcpp::Parameter("midphase_disabled", true),
	    rclcpp::Parameter("eulerdamp_disabled", true),
	    rclcpp::Parameter("override_contacts", true),
	    rclcpp::Parameter("energy", true),
	    rclcpp::Parameter("fwd_inv", true),
	    rclcpp::Parameter("inv_discrete", true),
	    rclcpp::Parameter("multiccd", true),
	    rclcpp::Parameter("island", true),
	    rclcpp::Parameter("margin", 0.5),
	    rclcpp::Parameter("solimp", std::string("0.8 0.9 0.1")),
	    rclcpp::Parameter("solref", std::string("0.03 1.1")),
	    rclcpp::Parameter("friction", std::string("0.5 0.5 0.1 0.1")),
	});

	ASSERT_TRUE(result.successful) << result.reason;
	std::lock_guard<std::recursive_mutex> lock(env_ptr->physics_thread_mutex_);

	EXPECT_STREQ(env_ptr->settings_.admin_hash, std::string("new_hash").c_str());
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running);

	EXPECT_EQ(env_ptr->getModelPtr()->opt.integrator, 2);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.cone, 0);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.jacobian, 1);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.solver, 1);
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.timestep, 0.002);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.iterations, 200);
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.tolerance, 1e-7);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.ls_iterations, 60);
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.ls_tolerance, 0.02);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.noslip_iterations, 10);
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.noslip_tolerance, 1e-5);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.ccd_iterations, 60);
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.ccd_tolerance, 1e-5);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.sdf_iterations, 15);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.sdf_initpoints, 50);

	EXPECT_EQ(env_ptr->getModelPtr()->opt.gravity[0], 9.81);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.gravity[1], 1);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.gravity[2], 2);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.wind[0], 1);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.wind[1], 1);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.wind[2], 1);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.magnetic[0], 0.1);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.magnetic[1], 0.1);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.magnetic[2], 0.1);
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.density, 1.2);
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.viscosity, 0.00003);
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.impratio, 1.5);

	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_CONSTRAINT);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_EQUALITY);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_FRICTIONLOSS);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_LIMIT);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_CONTACT);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_PASSIVE);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_GRAVITY);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_CLAMPCTRL);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_WARMSTART);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_FILTERPARENT);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_ACTUATION);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_REFSAFE);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_SENSOR);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_MIDPHASE);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.disableflags & mjDSBL_EULERDAMP);

	EXPECT_TRUE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_OVERRIDE);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_ENERGY);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_FWDINV);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_INVDISCRETE);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_MULTICCD);
	EXPECT_TRUE(env_ptr->getModelPtr()->opt.enableflags & mjENBL_ISLAND);

	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_margin, 0.5);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_solimp[0], 0.8);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_solimp[1], 0.9);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_solimp[2], 0.1);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_solref[0], 0.03);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_solref[1], 1.1);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_friction[0], 0.5);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_friction[1], 0.5);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_friction[2], 0.1);
	EXPECT_EQ(env_ptr->getModelPtr()->opt.o_friction[3], 0.1);
}

TEST_F(PendulumEnvFixture, RuntimeParameterRejectsInvalidValues)
{
	const double old_gravity = env_ptr->getModelPtr()->opt.gravity[0];

	auto result = env_ptr->set_parameters_atomically({ rclcpp::Parameter("gravity", std::string("1 invalid 3")) });
	EXPECT_FALSE(result.successful);
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->opt.gravity[0], old_gravity);

	result = env_ptr->set_parameters_atomically({ rclcpp::Parameter("timestep", std::string("not_a_double")) });
	EXPECT_FALSE(result.successful);
}
#endif
