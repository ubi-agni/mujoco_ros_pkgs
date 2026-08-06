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

// Verifies that when urdf.source/srdf.source params are set on the
// mujoco_server node, FetchRosConfiguration() drives the description-bundle
// description-bundle path instead of -- or, when absent, in addition to --
// the pre-existing modelfile path, ending in a valid loaded model. This covers
// both startup-path variants exercised by the integration test.

#include <gtest/gtest.h>

#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>

#include <mujoco_ros/mujoco_env.hpp>

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>
#include <std_msgs/String.h>
#else // MJR_ROS_VERSION == ROS_2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

int main(int argc, char **argv)
{
#if MJR_ROS_VERSION == ROS_1
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "description_bundle_integration_test");

	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle nh;
	int ret = RUN_ALL_TESTS();

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

namespace {

std::string ReadFileToString(const std::string &path)
{
	std::ifstream file_stream(path);
	std::stringstream buffer;
	buffer << file_stream.rdbuf();
	return buffer.str();
}

std::string ResourcePath(const std::string &name)
{
	return std::string(TEST_RESOURCES_DIR) + "/" + name;
}

#if MJR_ROS_VERSION == ROS_2
// A minimal, separately-spinning node that latches a single std_msgs/String
// on the given topic -- stands in for a urdf publisher so
// ResolveTopicSource's throwaway-node + rclcpp::spin_until_future_complete
// wait has an actual, independently-spinning publisher to receive from.
class FakeDescriptionTopicPublisher
{
public:
	FakeDescriptionTopicPublisher(const std::string &node_name, const std::string &topic_name,
	                              const std::string &content)
	    : node_(std::make_shared<rclcpp::Node>(node_name))
	{
		auto qos = rclcpp::QoS(1).transient_local().reliable();
		pub_     = node_->create_publisher<std_msgs::msg::String>(topic_name, qos);
		std_msgs::msg::String msg;
		msg.data = content;
		pub_->publish(msg);
		executor_.add_node(node_);
		spin_thread_ = std::thread([this]() { executor_.spin(); });
	}

	~FakeDescriptionTopicPublisher()
	{
		executor_.cancel();
		if (spin_thread_.joinable()) {
			spin_thread_.join();
		}
	}

	// Owns a spinning thread bound to `this` -- copying or moving would leave
	// a dangling `this` capture in the spin lambda, so both are disabled.
	FakeDescriptionTopicPublisher(const FakeDescriptionTopicPublisher &)            = delete;
	FakeDescriptionTopicPublisher &operator=(const FakeDescriptionTopicPublisher &) = delete;
	FakeDescriptionTopicPublisher(FakeDescriptionTopicPublisher &&)                 = delete;
	FakeDescriptionTopicPublisher &operator=(FakeDescriptionTopicPublisher &&)      = delete;

private:
	rclcpp::Node::SharedPtr node_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
	rclcpp::executors::SingleThreadedExecutor executor_;
	std::thread spin_thread_;
};
#endif

} // namespace

TEST_F(BaseEnvFixture, FileSourcedBundleProducesARunningEnv)
{
	nh->setParam("urdf.source", "file");
	nh->setParam("urdf.path", ResourcePath("two_link_robot.urdf"));
	nh->setParam("srdf.source", "file");
	nh->setParam("srdf.path", ResourcePath("two_link_robot.srdf"));

	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	// Must be asserted BEFORE StartEventLoop(): LoadWithModelAndData() (mujoco_ros/src/loading.cpp)
	// consumes and resets is_python_request to 0 unconditionally once it runs, regardless of what
	// FetchRosConfiguration() set it to -- so checking it after the load has completed can never
	// distinguish the bug (set to 1) from the fix (set to 0). It tells LoadWithModelAndData()
	// whether the model/data buffers are Python-owned (no-op deleter) or C++-owned
	// (mj_deleteModel/mj_deleteData). load_model_from_description() allocates via plain
	// mj_loadModel/mj_makeData -- genuinely C++-owned -- so leaving this at 1 (as an earlier
	// version of this code did) would silently leak the model forever instead of freeing it on the
	// next reload or on ~MujocoEnv(). Confirmed this catches the regression: re-introducing
	// is_python_request.store(1) in FetchRosConfiguration() makes this EXPECT_EQ fail while every
	// other assertion in this test still passes.
	EXPECT_EQ(env_ptr->settings_.is_python_request.load(), 0)
	    << "is_python_request must be 0 for a C++-owned model, or it will never be freed!";

	env_ptr->StartPhysicsLoop();
	env_ptr->StartEventLoop();

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 5) { // wait for queued bundle load or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005f;
	}
	EXPECT_LT(seconds, 5) << "File-sourced description bundle did not finish loading before timeout!";
	EXPECT_TRUE(env_ptr->sim_state_.model_valid) << "Model loaded from the file-sourced bundle should be valid!";
	ASSERT_TRUE(env_ptr->getModelPtr());
	// Empty modelfile + active bundle -> composed into the default world.
	EXPECT_NE(mj_name2id(env_ptr->getModelPtr(), mjOBJ_BODY, "base_link"), -1);
	EXPECT_NE(mj_name2id(env_ptr->getModelPtr(), mjOBJ_GEOM, "ground_plane"), -1);
	EXPECT_EQ(env_ptr->getFilename(), ResourcePath("two_link_robot.urdf"))
	    << "filename_ should reflect the bundle's resolved URDF path, not a stale value!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, CustomSrdfTagsDoNotBreakNormalConversionWithoutHandlers)
{
	nh->setParam("urdf.source", "file");
	nh->setParam("urdf.path", ResourcePath("two_link_robot.urdf"));
	nh->setParam("srdf.source", "file");
	nh->setParam("srdf.path", ResourcePath("srdf_extended_params_with_custom_tags.srdf"));

	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	env_ptr->StartPhysicsLoop();
	env_ptr->StartEventLoop();

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 5) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005f;
	}
	EXPECT_LT(seconds, 5);
	EXPECT_TRUE(env_ptr->sim_state_.model_valid);
	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, SrdfActuatorOverrideAppliesThroughRosParamBundle)
{
	nh->setParam("urdf.source", "file");
	nh->setParam("urdf.path", ResourcePath("ros2_control_two_link_robot.urdf"));
	nh->setParam("srdf.source", "file");
	nh->setParam("srdf.path", ResourcePath("srdf_extended_params_actuator_override.srdf"));
	nh->setParam("description.generate_actuators", "true");

	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	env_ptr->StartPhysicsLoop();
	env_ptr->StartEventLoop();

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 5) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005f;
	}
	EXPECT_LT(seconds, 5);
	EXPECT_TRUE(env_ptr->sim_state_.model_valid);
	ASSERT_TRUE(env_ptr->getModelPtr());
	const int actuator = mj_name2id(env_ptr->getModelPtr(), mjOBJ_ACTUATOR, "joint_1_act_pos");
	ASSERT_NE(actuator, -1);
	EXPECT_DOUBLE_EQ(env_ptr->getModelPtr()->actuator_gainprm[mjNGAIN * actuator], 500.0);
	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, TopicSourcedUrdfProducesARunningEnv)
{
	std::string urdf_content = ReadFileToString(ResourcePath("two_link_robot.urdf"));
	ASSERT_FALSE(urdf_content.empty()) << "Failed to read fixture URDF from disk!";

	std::string topic_name = "topic_sourced_urdf_test/robot_description";
#if MJR_ROS_VERSION == ROS_1
	// ROS 1 topics are latched automatically for any subscriber that connects
	// after the publish, as long as the publisher set latch=true -- no second
	// node/spinner is required.
	ros::Publisher pub = nh->advertise<std_msgs::String>(topic_name, 1, /*latch=*/true);
	std_msgs::String msg;
	msg.data = urdf_content;
	pub.publish(msg);
#else // MJR_ROS_VERSION == ROS_2
	// This is the case that exercises the throwaway-node + transient_local
	// wait: a real, independently spinning second node latching the message
	// via transient_local QoS, read via ResolveTopicSource's own throwaway
	// node + rclcpp::spin_until_future_complete.
	FakeDescriptionTopicPublisher topic_publisher("fake_rdf_publisher", topic_name, urdf_content);
#endif

	nh->setParam("urdf.source", "topic");
	nh->setParam("urdf.topic", topic_name);
	nh->setParam("srdf.source", "file");
	nh->setParam("srdf.path", ResourcePath("two_link_robot.srdf"));

	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	// See FileSourcedBundleProducesARunningEnv for why this must be checked here (before
	// StartEventLoop() consumes and resets it) rather than after the load completes.
	EXPECT_EQ(env_ptr->settings_.is_python_request.load(), 0)
	    << "is_python_request must be 0 for a C++-owned model, or it will never be freed!";

	env_ptr->StartPhysicsLoop();
	env_ptr->StartEventLoop();

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 5) { // wait for queued bundle load or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005f;
	}
	EXPECT_LT(seconds, 5) << "Topic-sourced description bundle did not finish loading before timeout! (if this "
	                         "is consistently the full 5s, the topic subscription is likely never receiving)";
	EXPECT_TRUE(env_ptr->sim_state_.model_valid) << "Model loaded from the topic-sourced bundle should be valid!";
	ASSERT_TRUE(env_ptr->getModelPtr());
	EXPECT_EQ(env_ptr->getModelPtr()->nbody, 3); // world + base_link + link_1

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, TopicSourceTimesOutLoudlyWhenNothingIsPublished)
{
	nh->setParam("urdf.source", "topic");
	nh->setParam("urdf.topic", "topic_sourced_urdf_test/nobody_publishes_here");

	EXPECT_THROW(std::make_unique<MujocoEnvTestWrapper>("", nh.get()), std::runtime_error);
}

TEST_F(BaseEnvFixture, AbsentBundleParamsLeavesPlainModelfilePathWorking)
{
	// Regression coverage: no urdf.source/srdf.source set at all -- must fall
	// through to the pre-existing modelfile-driven load exactly as before this
	// task's change (TryParseDescriptionBundleFromMap returns std::nullopt for
	// an empty flat-params map, so InitModelFromQueue()'s existing path is the
	// only one that ever runs here).
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	nh->setParam("modelfile", xml_path);

	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	env_ptr->StartPhysicsLoop();
	env_ptr->StartEventLoop();

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
		seconds += 0.003f;
	}
	EXPECT_LT(seconds, 2) << "Plain modelfile load did not finish before timeout!";
	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly via the modelfile path!";
	EXPECT_TRUE(env_ptr->sim_state_.model_valid);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, UrdfOnlyBundleUsesDefaultWorld)
{
	nh->setParam("urdf.source", "file");
	nh->setParam("urdf.path", ResourcePath("two_link_robot.urdf"));
	// no srdf.*; leave modelfile unset/empty

	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	env_ptr->StartPhysicsLoop();
	env_ptr->StartEventLoop();

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 5) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005f;
	}
	EXPECT_LT(seconds, 5);
	ASSERT_TRUE(env_ptr->getModelPtr());
	EXPECT_NE(mj_name2id(env_ptr->getModelPtr(), mjOBJ_BODY, "base_link"), -1);
	EXPECT_NE(mj_name2id(env_ptr->getModelPtr(), mjOBJ_GEOM, "ground_plane"), -1);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, BundleWithModelfileUsesThatFileAsWorld)
{
	nh->setParam("urdf.source", "file");
	nh->setParam("urdf.path", ResourcePath("two_link_robot.urdf"));
	// minimal_world.xml is already under TEST_RESOURCES_DIR (converter tests).
	nh->setParam("modelfile", std::string(TEST_RESOURCES_DIR) + "/minimal_world.xml");

	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	env_ptr->StartPhysicsLoop();
	env_ptr->StartEventLoop();

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 5) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005f;
	}
	EXPECT_LT(seconds, 5);
	ASSERT_TRUE(env_ptr->getModelPtr());
	EXPECT_NE(mj_name2id(env_ptr->getModelPtr(), mjOBJ_BODY, "base_link"), -1);
	// minimal_world has no ground_plane -- absence proves we did not fall back to default_world
	EXPECT_EQ(mj_name2id(env_ptr->getModelPtr(), mjOBJ_GEOM, "ground_plane"), -1);

	env_ptr->shutdown();
}
