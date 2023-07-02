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

#include <actionlib/client/simple_action_client.h>
#include <mujoco_ros_msgs/StepAction.h>
#include <mujoco_ros_msgs/StepGoal.h>

#include <mujoco_ros_msgs/SetPause.h>
#include <mujoco_ros_msgs/SetBodyState.h>
#include <mujoco_ros_msgs/GetBodyState.h>
#include <mujoco_ros_msgs/SetGeomProperties.h>
#include <mujoco_ros_msgs/GeomType.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "mujoco_env_fixture.h"

#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/util.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_ros_interface_test");

	// Create spinner to communicate with ROS
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle nh;
	int ret = RUN_ALL_TESTS();

	// Stop spinner and shutdown ROS before returning
	spinner.stop();
	ros::shutdown();
	return ret;
}

using namespace mujoco_ros;
namespace mju = ::mujoco::sample_util;

TEST_F(PendulumEnvFixture, Clock)
{
	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
	EXPECT_NEAR(ros::Time::now().toSec(), 0, 1e-6) << "Simulation time should be 0.0!";
	int total_steps = 0;

	EXPECT_TRUE(env_ptr->step()) << "Step did not succeed!";
	total_steps = 1;
	EXPECT_NEAR(ros::Time::now().toSec(), env_ptr->getModelPtr()->opt.timestep, 1e-6)
	    << "Simulation time should have increased by 1 step!";

	EXPECT_TRUE(env_ptr->step(10)) << "Stepping did not succeed!";
	total_steps += 10;
	std::this_thread::sleep_for(std::chrono::milliseconds(100)); // wait for time messages to be sent
	EXPECT_NEAR(ros::Time::now().toSec(), env_ptr->getModelPtr()->opt.timestep * total_steps, 1e-6)
	    << "Simulation time should have increased by 10 steps!";

	EXPECT_FALSE(env_ptr->step(-10)) << "Stepping with negative steps should not succeed!";
	EXPECT_NEAR(ros::Time::now().toSec(), env_ptr->getModelPtr()->opt.timestep * total_steps, 1e-6)
	    << "Simulation time should have increased by 10 steps!";
}

TEST_F(PendulumEnvFixture, ShutdownCallback)
{
	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/shutdown", true))
	    << "Shutdown service should be available!";

	std_srvs::Empty srv;
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/shutdown", srv)) << "Shutdown service call failed!";
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
	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/set_pause", true))
	    << "Reset service should be available!";
	mujoco_ros_msgs::SetPause srv;

	srv.request.paused = false;
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_pause", srv)) << "unpause service call failed!";

	float seconds = 0;
	while (seconds < 2 && !env_ptr->settings_.run) { // wait for unpause
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_TRUE(env_ptr->settings_.run) << "Simulation should be running!";

	srv.request.paused = true;
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_pause", srv)) << "pause service call failed!";
	seconds = 0;
	while (seconds < 2 && env_ptr->settings_.run) { // wait for pause
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
}

TEST_F(PendulumEnvFixture, ReloadSameModelCallback)
{
	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/reload", true))
	    << "Reload service should be available!";

	mujoco_ros_msgs::Reload srv;

	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/reload", srv)) << "Reload service call failed!";
	float seconds = 0;
	while (seconds < 2 && env_ptr->getOperationalStatus() > 0) { // wait for reload to finish
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_EQ(env_ptr->getOperationalStatus(), 0) << "Operational status should be 0!";
}

TEST_F(PendulumEnvFixture, ReloadNewModelCallback)
{
	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/reload", true))
	    << "Reload service should be available!";

	mujoco_ros_msgs::Reload srv;
	std::string model_xml = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	srv.request.model     = model_xml;

	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/reload", srv)) << "Reload service call failed!";
	float seconds = 0;
	while (seconds < 2 && env_ptr->getOperationalStatus() > 0) { // wait for reload to finish
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_EQ(env_ptr->getOperationalStatus(), 0) << "Operational status should be 0!";
	EXPECT_EQ(env_ptr->getFilename(), model_xml) << "New model should be loaded!";
}

TEST_F(PendulumEnvFixture, ResetCallback)
{
	env_ptr->settings_.run = 1;

	env_ptr->step(100);

	env_ptr->settings_.run = 0;

	// Make sure reset service is available
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/reset", true))
	    << "Reset service should be available!";
	std_srvs::Empty srv;
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/reset", srv)) << "Reset service call failed!";

	float seconds = 0;
	while (seconds < 2 && env_ptr->getOperationalStatus() > 0) { // wait for reset to finish
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Reset callback took too long!";

	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0.0, 1e-6) << "Simulation time should be reset to 0.0!";
}

TEST_F(PendulumEnvFixture, StepGoal)
{
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	bool found = false;
	for (const auto &t : master_topics) {
		if (t.name == env_ptr->getHandleNamespace() + "/step/result") {
			found = true;
			break;
		}
	}
	// Workaround to connect to action server, this is only needed in cpp
	env_ptr->settings_.run = 1;
	EXPECT_TRUE(found) << "Step action should be available!";
	ros::spinOnce();
	actionlib::SimpleActionClient<mujoco_ros_msgs::StepAction> ac(env_ptr->getHandleNamespace() + "/step", true);
	env_ptr->settings_.run = 0;

	// Wait for paused state to be applied
	std::this_thread::sleep_for(std::chrono::milliseconds(5));

	mjtNum time = env_ptr->getDataPtr()->time;

	mujoco_ros_msgs::StepGoal goal;
	goal.num_steps = 1;

	EXPECT_EQ(env_ptr->settings_.env_steps_request, 0) << "No steps should be requested yet!";
	ac.sendGoal(goal);

	EXPECT_TRUE(ac.waitForResult(ros::Duration(1.0))) << "Step action did not finish in time!";
	EXPECT_TRUE(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) << "Step action did not succeed!";
	EXPECT_EQ(ac.getResult()->success, true) << "Step action did not succeed!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, time + env_ptr->getModelPtr()->opt.timestep, 1e-6)
	    << "Simulation time should have changed by timestep!";

	goal.num_steps = 100;
	ac.sendGoal(goal);

	EXPECT_TRUE(ac.waitForResult(ros::Duration(1.0))) << "Step action did not finish in time!";
	EXPECT_TRUE(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) << "Step action did not succeed!";
	EXPECT_EQ(ac.getResult()->success, true) << "Step action did not succeed!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, time + env_ptr->getModelPtr()->opt.timestep * 101, 1e-6)
	    << "Simulation time should have changed by timestep * 101!";

	env_ptr->settings_.run = 1;
	ac.sendGoal(goal);

	EXPECT_TRUE(ac.waitForResult(ros::Duration(1.0))) << "Step action did not finish in time!";
	EXPECT_TRUE(ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
	    << "Step action must be preempted when unpaused!";
	EXPECT_EQ(ac.getResult()->success, false) << "Step action should have failed!";
}

TEST_F(PendulumEnvFixture, DefaultInitialJointStates)
{
	mjModelPtr m = env_ptr->getModelPtr();
	mjDataPtr d  = env_ptr->getDataPtr();

	std::map<std::string, std::string> pos_map, vel_map;
	nh->getParam("initial_joint_positions/joint_map", pos_map);
	nh->getParam("initial_joint_velocities/joint_map", vel_map);

	EXPECT_EQ(pos_map.size(), 0) << "No initial joint positions should be set!";
	EXPECT_EQ(vel_map.size(), 0) << "No initial joint velocities should be set!";

	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should not be running yet!";
	EXPECT_EQ(env_ptr->getPendingSteps(), -1) << "Simulation should have pending steps!";
	EXPECT_NEAR(d->time, 0.0, 1e-6) << "Simulation time should be 0.0!";

	int id_balljoint, id1, id2, id_free;
	id_balljoint = mujoco_ros::util::jointName2id(m.get(), "balljoint");
	id1          = mujoco_ros::util::jointName2id(m.get(), "joint1");
	id2          = mujoco_ros::util::jointName2id(m.get(), "joint2");
	id_free      = mujoco_ros::util::jointName2id(m.get(), "ball_freejoint");

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

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/pendulum_world.xml";
	std::map<std::string, std::string> pos_map, vel_map;

	pos_map.insert({ "balljoint", "0 0.707 0 0.707" });
	pos_map.insert({ "joint1", "-1.57" });
	pos_map.insert({ "joint2", "-0.66" });
	pos_map.insert({ "ball_freejoint", "2.0 1.0 1.06 0.0 0.707 0.0 0.707" });

	vel_map.insert({ "balljoint", "5 5 10" });
	vel_map.insert({ "joint2", "1.05" });
	vel_map.insert({ "ball_freejoint", "1.0 2.0 3.0 10 20 30" });

	nh->setParam("initial_joint_positions/joint_map", pos_map);
	nh->setParam("initial_joint_velocities/joint_map", vel_map);

	MujocoEnvTestWrapper env;
	env.startWithXML(xml_path);

	while (env.getOperationalStatus() > 0) { // wait for reset to be done
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	mjDataPtr d  = env.getDataPtr();
	mjModelPtr m = env.getModelPtr();

	int id_balljoint, id1, id2, id_free;

	id_balljoint = mujoco_ros::util::jointName2id(m.get(), "balljoint");
	id1          = mujoco_ros::util::jointName2id(m.get(), "joint1");
	id2          = mujoco_ros::util::jointName2id(m.get(), "joint2");
	id_free      = mujoco_ros::util::jointName2id(m.get(), "ball_freejoint");

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

	env.shutdown();

	nh->deleteParam("initial_joint_positions/joint_map");
	nh->deleteParam("initial_joint_velocities/joint_map");
	nh->setParam("unpause", true);
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

	mjDataPtr d  = env_ptr->getDataPtr();
	mjModelPtr m = env_ptr->getModelPtr();

	int id_balljoint, id1, id2, id_free;

	id_balljoint = mujoco_ros::util::jointName2id(m.get(), "balljoint");
	id1          = mujoco_ros::util::jointName2id(m.get(), "joint1");
	id2          = mujoco_ros::util::jointName2id(m.get(), "joint2");
	id_free      = mujoco_ros::util::jointName2id(m.get(), "ball_freejoint");

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

	nh->setParam("initial_joint_positions/joint_map", pos_map);
	nh->setParam("initial_joint_velocities/joint_map", vel_map);

	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/reset", true))
	    << "Reset service should be available!";

	std_srvs::Empty srv;
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/reset", srv)) << "Reset service call failed!";

	while (env_ptr->getOperationalStatus() > 0) { // wait for reset to be done
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	compare_qpos(d, m->jnt_qposadr[id_balljoint], "balljoint", { 0.0, 0.707, 0.0, 0.707 }, { 0.0, 9e-4, 0.0, 9e-4 });
	compare_qpos(d, m->jnt_qposadr[id1], "joint1", { -1.57 });
	compare_qpos(d, m->jnt_qposadr[id2], "joint2", { -0.66 });
	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 2.0, 1.0, 1.06, 0.0, 0.707, 0.0, 0.707 },
	             { 0.0, 0.0, 0.0, 0.0, 9e-4, 0.0, 9e-4 });

	compare_qvel(d, m->jnt_dofadr[id_balljoint], "balljoint", { 5.0, 5.0, 10.0 });
	compare_qvel(d, m->jnt_dofadr[id1], "joint1", { 0.0 });
	compare_qvel(d, m->jnt_dofadr[id2], "joint2", { 1.05 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 1.0, 2.0, 3.0, 10.0, 20.0, 30.0 });

	nh->deleteParam("initial_joint_positions/joint_map");
	nh->deleteParam("initial_joint_velocities/joint_map");
}

TEST_F(PendulumEnvFixture, SetBodyStateCallback)
{
	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/set_body_state", true))
	    << "Set body state service should be available!";

	int id_free;

	mjModelPtr m = env_ptr->getModelPtr();
	mjDataPtr d  = env_ptr->getDataPtr();

	id_free = mujoco_ros::util::jointName2id(m.get(), "ball_freejoint");

	mujoco_ros_msgs::SetBodyState srv;

	// Invalid body_name
	srv.request.state.name = "unknown";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_FALSE(srv.response.success);

	// Resolve body
	srv.request.state.name = "middle_link";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_TRUE(srv.response.success);

	// Resolve body from child geom
	srv.request.state.name = "EE";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_TRUE(srv.response.success);

	// Position change errors
	srv.request.set_pose = true;

	//   Not a freejoint
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_FALSE(srv.response.success);

	//   No joint
	srv.request.state.name = "immovable";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_FALSE(srv.response.success);

	//   unknown frame_id
	srv.request.state.name                 = "ball";
	srv.request.state.pose.header.frame_id = "unknown";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_FALSE(srv.response.success);

	// Twist change errors
	srv.request.set_pose  = false;
	srv.request.set_twist = true;

	//   other frame_id than world
	srv.request.state.twist.header.frame_id = "not-world";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_FALSE(srv.response.success);

	// New twist and pose
	srv.request.set_pose                      = true;
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
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_TRUE(srv.response.success);

	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 2.0, 2.0, 2.0, 0.0, 0.707, 0.0, 0.707 },
	             { 0.0, 0.0, 0.0, 0.0, 9e-4, 0.0, 9e-4 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 0.1, 0.1, -0.1, 0.1, 0.0, 0.0 });

	// New mass
	srv.request.set_pose   = false;
	srv.request.set_twist  = false;
	srv.request.set_mass   = true;
	srv.request.state.mass = 0.299f;
	EXPECT_NE(m->body_mass[mj_name2id(m.get(), mjOBJ_BODY, "body_ball")], srv.request.state.mass)
	    << "Mass already has the requested value!"; // Check that mass is different beforehand
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_TRUE(srv.response.success);

	EXPECT_EQ(m->body_mass[mj_name2id(m.get(), mjOBJ_BODY, "body_ball")], srv.request.state.mass)
	    << "Mass did not change to the requested value";
	// reset
	srv.request.set_mass   = false;
	srv.request.reset_qpos = true;
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_TRUE(srv.response.success);

	compare_qpos(d, m->jnt_qposadr[id_free], "ball_freejoint", { 1.0, 0.0, 0.06, 1.0, 0.0, 0.0, 0.0 });
	compare_qvel(d, m->jnt_dofadr[id_free], "ball_freejoint", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
}

TEST_F(PendulumEnvFixture, GetBodyStateCallback)
{
	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/set_body_state", true))
	    << "Set body state service should be available!";
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/get_body_state", true))
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
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_TRUE(srv.response.success);

	mujoco_ros_msgs::GetBodyState g_srv;
	// wrong body name
	g_srv.request.name = "unknown";

	// correct request
	g_srv.request.name = "body_ball";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/get_body_state", g_srv))
	    << "get body state service call failed!";
	EXPECT_TRUE(g_srv.response.success);
	EXPECT_EQ(g_srv.response.state.mass, srv.request.state.mass);
	EXPECT_EQ(g_srv.response.state.name, srv.request.state.name);
	EXPECT_EQ(g_srv.response.state.pose.pose, srv.request.state.pose.pose);
	EXPECT_EQ(g_srv.response.state.twist.twist, srv.request.state.twist.twist);

	// TODO(dleins): tests for bodies with a non-freejoint, no joint, and multiple joints (cannot set position but read!)
}

TEST_F(PendulumEnvFixture, SetGeomPropertiesCallback)
{
	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/set_geom_properties", true))
	    << "Set geom properties service should be available!";

	mjModelPtr m = env_ptr->getModelPtr();
	mjDataPtr d  = env_ptr->getDataPtr();

	int ball_geom_id = mj_name2id(m.get(), mjOBJ_GEOM, "ball");
	int ball_body_id = mj_name2id(m.get(), mjOBJ_BODY, "body_ball");

	EXPECT_NE(ball_geom_id, -1) << "'ball' should be found as geom in model!";
	EXPECT_NE(ball_body_id, -1) << "'body_ball' should be found as body in model!";

	mujoco_ros_msgs::SetGeomProperties srv;

	// Invalid geom_name
	srv.request.properties.name = "unknown";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_FALSE(srv.response.success);

	// Resolve geom
	srv.request.properties.name = "ball";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);

	// set mass
	srv.request.set_mass             = true;
	srv.request.properties.body_mass = 0.299f;
	EXPECT_NE(m->body_mass[ball_body_id], srv.request.properties.body_mass) << "Mass already has requested value!";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(m->body_mass[ball_body_id], srv.request.properties.body_mass)
	    << "Mass did not change to the requested value";

	// set friction
	srv.request.set_mass                  = false;
	srv.request.set_friction              = true;
	srv.request.properties.friction_slide = 0;
	srv.request.properties.friction_spin  = 0;
	srv.request.properties.friction_roll  = 0;
	EXPECT_TRUE(m->geom_friction[ball_geom_id * 3] != 0 && m->geom_friction[ball_geom_id * 3 + 1] != 0 &&
	            m->geom_friction[ball_geom_id * 3 + 2] != 0)
	    << "Some friction values already at 0!";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_TRUE(m->geom_friction[ball_geom_id * 3] == 0) << "Slide friction unchanged!";
	EXPECT_TRUE(m->geom_friction[ball_geom_id * 3 + 1] == 0) << "Spin friction unchanged!";
	EXPECT_TRUE(m->geom_friction[ball_geom_id * 3 + 2] == 0) << "Roll friction uncahnged!";

	// set type (not checking PLANE, HFIELD, MESH, and rendering types)
	srv.request.set_friction = false;
	srv.request.set_type     = true;
	//   BOX
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::BOX;
	EXPECT_NE(m->geom_type[ball_geom_id], mjGEOM_BOX) << "Geom already is of type BOX";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(m->geom_type[ball_geom_id], mjGEOM_BOX) << "Geom unchanged";
	//   CYLINDER
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::CYLINDER;
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(m->geom_type[ball_geom_id], mjGEOM_CYLINDER) << "Geom unchanged";
	//  ELLIPSOID
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::ELLIPSOID;
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(m->geom_type[ball_geom_id], mjGEOM_ELLIPSOID) << "Geom unchanged";
	//  CAPSULE
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::CAPSULE;
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(m->geom_type[ball_geom_id], mjGEOM_CAPSULE) << "Geom unchanged";
	//  SPHERE
	srv.request.properties.type.value = mujoco_ros_msgs::GeomType::SPHERE;
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);
	EXPECT_EQ(m->geom_type[ball_geom_id], mjGEOM_SPHERE) << "Geom unchanged";

	// set size
	srv.request.set_type          = false;
	srv.request.set_size          = true;
	srv.request.properties.size_0 = 0.01f;
	srv.request.properties.size_1 = 0.01f;
	srv.request.properties.size_2 = 0.01f;
	EXPECT_TRUE(m->geom_size[ball_geom_id * 3] != 0.01 && m->geom_size[ball_geom_id * 3 + 1] != 0.01 &&
	            m->geom_size[ball_geom_id * 3 + 2] != 0.01)
	    << "Geom size is already 0.01 0.01 0.01!";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_NEAR(m->geom_size[ball_geom_id * 3], 0.01, 9e-4) << "Size 0 unchanged";
	EXPECT_NEAR(m->geom_size[ball_geom_id * 3 + 1], 0.01, 9e-4) << "Size 1 unchanged";
	EXPECT_NEAR(m->geom_size[ball_geom_id * 3 + 2], 0.01, 9e-4) << "Size 2 unchanged";
}

TEST_F(PendulumEnvFixture, GetGeomPropertiesCallback)
{
	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/set_geom_properties", true))
	    << "Set geom properties service should be available!";
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/get_geom_properties", true))
	    << "Set geom properties service should be available!";

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
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_geom_properties", srv))
	    << "Set geom properties service call failed!";
	EXPECT_TRUE(srv.response.success);

	mujoco_ros_msgs::GetGeomProperties g_srv;
	// wrong geom name
	g_srv.request.geom_name = "unknown";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/get_geom_properties", g_srv))
	    << "Get geom properties service call failed!";
	EXPECT_FALSE(g_srv.response.success);

	g_srv.request.geom_name = "ball";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/get_geom_properties", g_srv))
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
