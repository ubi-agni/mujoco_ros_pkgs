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
	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	std::map<std::string, std::string> pos_map, vel_map;
	nh->getParam("initial_joint_positions/joint_map", pos_map);
	nh->getParam("initial_joint_velocities/joint_map", vel_map);

	EXPECT_EQ(pos_map.size(), 0) << "No initial joint positions should be set!";
	EXPECT_EQ(vel_map.size(), 0) << "No initial joint velocities should be set!";

	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should not be running yet!";
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

	mjData *d  = env.getDataPtr();
	mjModel *m = env.getModelPtr();

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

	mjModel *m = env_ptr->getModelPtr();
	mjData *d  = env_ptr->getDataPtr();

	id_free = mujoco_ros::util::jointName2id(m, "ball_freejoint");

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
	EXPECT_NE(m->body_mass[mj_name2id(m, mjOBJ_BODY, "body_ball")], srv.request.state.mass)
	    << "Mass already has the requested value!"; // Check that mass is different beforehand
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_body_state", srv))
	    << "set body state service call failed!";
	EXPECT_TRUE(srv.response.success);

	EXPECT_EQ(m->body_mass[mj_name2id(m, mjOBJ_BODY, "body_ball")], srv.request.state.mass)
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

	mjModel *m = env_ptr->getModelPtr();

	int ball_geom_id = mj_name2id(m, mjOBJ_GEOM, "ball");
	int ball_body_id = mj_name2id(m, mjOBJ_BODY, "body_ball");

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

TEST_F(EqualityEnvFixture, InitialEqualityConstraintValues)
{
	mjModel *m = env_ptr->getModelPtr();

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
	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";

	// test joint constraint data
	EXPECT_TRUE(m->eq_active[joint_eq_id]) << "Joint constraint mismatch of active state";
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
	EXPECT_TRUE(m->eq_active[connect_eq_id]) << "Joint constraint mismatch of active state";
	EXPECT_DOUBLE_EQ(m->eq_data[connect_eq_id * mjNEQDATA + 0], 0) << "Anchor mismatch at index 0";
	EXPECT_DOUBLE_EQ(m->eq_data[connect_eq_id * mjNEQDATA + 1], 0) << "Anchor mismatch at index 1";
	EXPECT_DOUBLE_EQ(m->eq_data[connect_eq_id * mjNEQDATA + 2], 0) << "Anchor mismatch at index 2";
	EXPECT_STREQ("immovable", mj_id2name(m, mjOBJ_XBODY, m->eq_obj1id[connect_eq_id]))
	    << "connect constraint element1 mismatch";

	// test tendon
	EXPECT_TRUE(m->eq_active[tendon_eq_id]) << "Joint constraint mismatch of active state";
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

// TODO: Should we test changing element1 and/or element2?
// are changes like that valid?
TEST_F(EqualityEnvFixture, SetEqConstraint)
{
	mjModel *m = env_ptr->getModelPtr();

	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/set_eq_constraint_parameters", true))
	    << "Set eq constraints service should be available!";

	const char *eq_types[4] = { "connect_eq", "weld_eq", "joint_eq", "tendon_eq" };
	int index               = 0;
	for (const char *eq_constraint : eq_types) {
		int eq_id = mj_name2id(m, mjOBJ_EQUALITY, eq_constraint);
		EXPECT_NE(eq_id, -1) << "eq constraint is not defined in loaded model!";

		mjtNum quat[4] = { -0.634, -0.002, -0.733, 0.244 };
		mju_normalize4(quat);

		mujoco_ros_msgs::SetEqualityConstraintParameters srv;
		srv.request.parameters.active                     = false;
		srv.request.parameters.name                       = eq_constraint;
		srv.request.parameters.type.typevalue             = index;
		srv.request.parameters.solverParameters.dampratio = 0.8;
		srv.request.parameters.solverParameters.timeconst = 0.2;
		srv.request.parameters.solverParameters.dmin      = 0.7;
		srv.request.parameters.solverParameters.dmax      = 0.9;
		srv.request.parameters.solverParameters.width     = 0.001;
		srv.request.parameters.solverParameters.midpoint  = 0.5;
		srv.request.parameters.solverParameters.power     = 3.0;

		// constraint specific parameters
		if (strcmp(eq_constraint, "weld_eq") == 0) {
			srv.request.parameters.element1              = "immovable";
			srv.request.parameters.element1              = "";
			srv.request.parameters.anchor.x              = 5.0;
			srv.request.parameters.anchor.y              = 3.0;
			srv.request.parameters.anchor.z              = 7.0;
			srv.request.parameters.relpose.position.x    = 1.2;
			srv.request.parameters.relpose.position.y    = 1.3;
			srv.request.parameters.relpose.position.z    = 1.4;
			srv.request.parameters.relpose.orientation.w = quat[0];
			srv.request.parameters.relpose.orientation.x = quat[1];
			srv.request.parameters.relpose.orientation.y = quat[2];
			srv.request.parameters.relpose.orientation.z = quat[3];
			srv.request.parameters.type.typevalue        = mujoco_ros_msgs::EqualityConstraintType::WELD;
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA], srv.request.parameters.anchor.x) << "Anchor[0] would not change";
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 1], srv.request.parameters.anchor.y) << "Anchor[1] would not change";
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 2], srv.request.parameters.anchor.z) << "Anchor[2] would not change";
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 3], srv.request.parameters.relpose.position.x)
			    << "Relpose[0] would not change";
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 4], srv.request.parameters.relpose.position.y)
			    << "Relpose[1] would not change";
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 5], srv.request.parameters.relpose.position.z)
			    << "Relpose[2] would not change";
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 6], srv.request.parameters.relpose.orientation.w)
			    << "Relpose[3] would not change";
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 7], srv.request.parameters.relpose.orientation.x)
			    << "Relpose[4] would not change";
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 8], srv.request.parameters.relpose.orientation.y)
			    << "Relpose[5] would not change";
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 9], srv.request.parameters.relpose.orientation.z)
			    << "Relpose[6] would not change";
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 10], srv.request.parameters.torquescale)
			    << "Torquescale would not change";

		} else if (strcmp(eq_constraint, "connect_eq") == 0) {
			srv.request.parameters.element1 = "immovable";
			srv.request.parameters.element2 = "";
			srv.request.parameters.anchor.x = 5.0;
			srv.request.parameters.anchor.y = 3.0;
			srv.request.parameters.anchor.z = 7.0;
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA], srv.request.parameters.anchor.x) << "Anchor[0] would not change";
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 1], srv.request.parameters.anchor.y) << "Anchor[1] would not change";
			EXPECT_NE(m->eq_data[eq_id * mjNEQDATA + 2], srv.request.parameters.anchor.z) << "Anchor[2] would not change";

		} else if (strcmp(eq_constraint, "joint_eq") == 0 || strcmp(eq_constraint, "tendon_eq") == 0) {
			std::string constraint_name     = eq_constraint;
			srv.request.parameters.element1 = constraint_name + "_element1";
			srv.request.parameters.element1 = "";
			srv.request.parameters.polycoef = std::vector<double>{ 0.1, 1.0, 0.2, 0.3, 0.4 };
		}

		// Verify values differ to confirm values have changed later on
		EXPECT_NE(m->eq_active[eq_id], srv.request.parameters.active) << "Constraint is already inactive";

		EXPECT_NE(m->eq_solref[eq_id * mjNREF], srv.request.parameters.solverParameters.timeconst)
		    << "Solref timeconst would not change";
		EXPECT_NE(m->eq_solref[eq_id * mjNREF + 1], srv.request.parameters.solverParameters.dampratio)
		    << "Solref dampratio would not change";

		EXPECT_NE(m->eq_solimp[eq_id * mjNIMP], srv.request.parameters.solverParameters.dmin)
		    << "Solimp dmin would not change";
		EXPECT_NE(m->eq_solimp[eq_id * mjNIMP + 1], srv.request.parameters.solverParameters.dmax)
		    << "Solimp dmax would not change";
		EXPECT_NE(m->eq_solimp[eq_id * mjNIMP + 2], srv.request.parameters.solverParameters.width)
		    << "Solimp width would not change";
		EXPECT_NE(m->eq_solimp[eq_id * mjNIMP + 3], srv.request.parameters.solverParameters.midpoint)
		    << "Solimp midpoint would not change";
		EXPECT_NE(m->eq_solimp[eq_id * mjNIMP + 4], srv.request.parameters.solverParameters.power)
		    << "Solimp power would not change";

		EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/set_eq_constraint_parameters", srv))
		    << "Set eq constraint service call failed!";
		EXPECT_TRUE(srv.response.success);

		EXPECT_DOUBLE_EQ(m->eq_solref[eq_id * mjNREF], srv.request.parameters.solverParameters.timeconst)
		    << "Solref timeconst was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_solref[eq_id * mjNREF + 1], srv.request.parameters.solverParameters.dampratio)
		    << "Solref dampration was not set correctly";

		EXPECT_DOUBLE_EQ(m->eq_solimp[eq_id * mjNIMP], srv.request.parameters.solverParameters.dmin)
		    << "Solimp dmin was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_solimp[eq_id * mjNIMP + 1], srv.request.parameters.solverParameters.dmax)
		    << "Solimp dmax was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_solimp[eq_id * mjNIMP + 2], srv.request.parameters.solverParameters.width)
		    << "Solimp width was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_solimp[eq_id * mjNIMP + 3], srv.request.parameters.solverParameters.midpoint)
		    << "Solimp midpoint was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_solimp[eq_id * mjNIMP + 4], srv.request.parameters.solverParameters.power)
		    << "Solimp power was not set correctly";
		if (strcmp(eq_constraint, "weld_eq") == 0) {
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA], srv.request.parameters.anchor.x)
			    << "Anchor[0] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 1], srv.request.parameters.anchor.y)
			    << "Anchor[1] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 2], srv.request.parameters.anchor.z)
			    << "Anchor[2] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 3], srv.request.parameters.relpose.position.x)
			    << "Relpose[0] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 4], srv.request.parameters.relpose.position.y)
			    << "Relpose[1] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 5], srv.request.parameters.relpose.position.z)
			    << "Relpose[2] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 6], srv.request.parameters.relpose.orientation.w)
			    << "Relpose[3] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 7], srv.request.parameters.relpose.orientation.x)
			    << "Relpose[4] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 8], srv.request.parameters.relpose.orientation.y)
			    << "Relpose[5] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 9], srv.request.parameters.relpose.orientation.z)
			    << "Relpose[6] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 10], srv.request.parameters.torquescale)
			    << "Torquescale was not set correctly";

		} else if (strcmp(eq_constraint, "connect_eq") == 0) {
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA], srv.request.parameters.anchor.x)
			    << "Anchor[0] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 1], srv.request.parameters.anchor.y)
			    << "Anchor[1] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 2], srv.request.parameters.anchor.z)
			    << "Anchor[2] was not set correctly";

		} else if (strcmp(eq_constraint, "joint_eq") == 0 || strcmp(eq_constraint, "tendon_eq") == 0) {
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA], srv.request.parameters.polycoef[0])
			    << "Polyceof[0] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 1], srv.request.parameters.polycoef[1])
			    << "Polycoef[1] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 2], srv.request.parameters.polycoef[2])
			    << "Polycoef[2] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 3], srv.request.parameters.polycoef[3])
			    << "Polycoef[3] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 4], srv.request.parameters.polycoef[4])
			    << "Polycoef[3] was not set correctly";
		}
		index++;
	}
}

TEST_F(EqualityEnvFixture, GetEqConstraint)
{
	mjModel *m = env_ptr->getModelPtr();

	EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Simulation time should be 0.0!";
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/set_eq_constraint_parameters", true))
	    << "Set geom properties service should be available!";
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/get_eq_constraint_parameters", true))
	    << "Ret geom properties service should be available!";

	const char *eq_types[4] = { "weld_eq", "tendon_eq", "joint_eq", "connect_eq" };

	for (const char *eq_constraint : eq_types) {
		int eq_id = mj_name2id(m, mjOBJ_EQUALITY, eq_constraint);
		EXPECT_NE(eq_id, -1) << "eq constraint is not defined in loaded model!";

		mujoco_ros_msgs::GetEqualityConstraintParameters srv;
		srv.request.name = eq_constraint;
		EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/get_eq_constraint_parameters", srv))
		    << "Get eq constraints service call failed!";
		EXPECT_TRUE(srv.response.success);

		EXPECT_NEAR(m->eq_solref[eq_id * mjNREF], srv.response.parameters.solverParameters.timeconst, 1e-5)
		    << "Solref timeconst was not set correctly";
		EXPECT_NEAR(m->eq_solref[eq_id * mjNREF + 1], srv.response.parameters.solverParameters.dampratio, 1e-5)
		    << "Solref dampration was not set correctly";

		EXPECT_NEAR(m->eq_solimp[eq_id * mjNIMP], srv.response.parameters.solverParameters.dmin, 1e-5)
		    << "Solimp dmin was not set correctly";
		EXPECT_NEAR(m->eq_solimp[eq_id * mjNIMP + 1], srv.response.parameters.solverParameters.dmax, 1e-5)
		    << "Solimp dmax was not set correctly";
		EXPECT_NEAR(m->eq_solimp[eq_id * mjNIMP + 2], srv.response.parameters.solverParameters.width, 1e-5)
		    << "Solimp width was not set correctly";
		EXPECT_NEAR(m->eq_solimp[eq_id * mjNIMP + 3], srv.response.parameters.solverParameters.midpoint, 1e-5)
		    << "Solimp midpoint was not set correctly";
		EXPECT_DOUBLE_EQ(m->eq_solimp[eq_id * mjNIMP + 4], srv.response.parameters.solverParameters.power)
		    << "Solimp power was not set correctly";

		if (strcmp(eq_constraint, "weld_eq") == 0) {
			int elem1_id = mj_name2id(m, mjOBJ_BODY, "immovable");
			EXPECT_NE(elem1_id, -1) << "Body with name 'immovable' is not defined in loaded model!";
			EXPECT_EQ(m->eq_obj1id[eq_id], elem1_id) << "Weld constraint body1 was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA], srv.response.parameters.anchor.x)
			    << "Anchor[0] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 1], srv.response.parameters.anchor.y)
			    << "Anchor[1] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 2], srv.response.parameters.anchor.z)
			    << "Anchor[2] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 3], srv.response.parameters.relpose.position.x)
			    << "Relpose[0] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 4], srv.response.parameters.relpose.position.y)
			    << "Relpose[1] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 5], srv.response.parameters.relpose.position.z)
			    << "Relpose[2] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 6], srv.response.parameters.relpose.orientation.w)
			    << "Relpose[3] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 7], srv.response.parameters.relpose.orientation.x)
			    << "Relpose[4] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 8], srv.response.parameters.relpose.orientation.y)
			    << "Relpose[5] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 9], srv.response.parameters.relpose.orientation.z)
			    << "Relpose[6] was not set correctly";
			EXPECT_NEAR(m->eq_data[eq_id * mjNEQDATA + 10], srv.response.parameters.torquescale, 1e-5)
			    << "Torquescale was not set correctly";
		} else if (strcmp(eq_constraint, "connect_eq") == 0) {
			int elem1_id = mj_name2id(m, mjOBJ_BODY, "immovable");
			EXPECT_NE(elem1_id, -1) << "Body with name 'immovable' is not defined in loaded model!";
			EXPECT_EQ(m->eq_obj1id[eq_id], elem1_id) << "Weld constraint body1 was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA], srv.response.parameters.anchor.x)
			    << "Anchor[0] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 1], srv.response.parameters.anchor.y)
			    << "Anchor[1] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 2], srv.response.parameters.anchor.z)
			    << "Anchor[2] was not set correctly";
		} else if (strcmp(eq_constraint, "joint_eq") == 0 || strcmp(eq_constraint, "tendon_eq") == 0) {
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA], srv.response.parameters.polycoef[0])
			    << "polycoef[]0] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 1], srv.response.parameters.polycoef[1])
			    << "polycoef[]1] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 2], srv.response.parameters.polycoef[2])
			    << "polycoef[]2] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 3], srv.response.parameters.polycoef[3])
			    << "polycoef[3] was not set correctly";
			EXPECT_DOUBLE_EQ(m->eq_data[eq_id * mjNEQDATA + 4], srv.response.parameters.polycoef[4])
			    << "polycoef[4] was not set correctly";
		}
	}
}
