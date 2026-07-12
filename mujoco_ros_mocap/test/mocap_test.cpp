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

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros_mocap/mocap_plugin.hpp>
#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <mujoco_ros_msgs/MocapState.h>
#include <mujoco_ros_msgs/SetMocapState.h>
#include <ros/master.h>
#include <ros/package.h>
#include <ros/ros.h>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco_ros_msgs/msg/mocap_state.hpp>
#include <mujoco_ros_msgs/srv/set_mocap_state.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

#if MJR_ROS_VERSION == ROS_1
using MocapState    = mujoco_ros_msgs::MocapState;
using SetMocapState = mujoco_ros_msgs::SetMocapState;
#else
using MocapState    = mujoco_ros_msgs::msg::MocapState;
using SetMocapState = mujoco_ros_msgs::srv::SetMocapState;
#endif

std::string get_mocap_model_path()
{
#if MJR_ROS_VERSION == ROS_1
	return ros::package::getPath("mujoco_ros_mocap") + "/assets/mocap_world.xml";
#else
	return ament_index_cpp::get_package_share_directory("mujoco_ros_mocap") + "/assets/mocap_world.xml";
#endif
}

void configure_plugin_params(testing::TestNodeHandle &nh)
{
#if MJR_ROS_VERSION == ROS_2
	nh.setParam("MujocoPlugins.names", std::vector<std::string>{ "mujoco_ros_mocap" });
	nh.setParam("MujocoPlugins.mujoco_ros_mocap.type", "mujoco_ros_mocap/MocapPlugin");
#else
	(void)nh;
#endif
}

void clear_test_params(testing::TestNodeHandle &nh)
{
#if MJR_ROS_VERSION == ROS_1
	ros::param::del(nh.getNamespace());
#else
	nh.clearNode();
#endif
}

void init_ros(int argc, char **argv)
{
#if MJR_ROS_VERSION == ROS_1
	ros::init(argc, argv, "mujoco_ros_mocap_test");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	ros::console::notifyLoggerLevelsChanged();
#else
	rclcpp::init(argc, argv);
#endif
}

int run_all_tests()
{
#if MJR_ROS_VERSION == ROS_1
	ros::AsyncSpinner spinner(1);
	spinner.start();
	const int ret = RUN_ALL_TESTS();
	spinner.stop();
	ros::shutdown();
	return ret;
#else
	const int ret = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return ret;
#endif
}

std::string mocap_topic_name(MujocoEnvTestWrapper *env_ptr)
{
	return env_ptr->GetHandleNamespace() + "/mocap_poses";
}

std::string mocap_service_name(MujocoEnvTestWrapper *env_ptr)
{
	return env_ptr->GetHandleNamespace() + "/set_mocap_state";
}

MocapState make_mocap_state(const std::string &body_name, const std::string &frame_id, double x, double y, double z,
                            double qw, double qx = 0.0, double qy = 0.0, double qz = 0.0)
{
	MocapState state;
	state.name.push_back(body_name);
	state.pose.emplace_back();
	state.pose.back().header.frame_id    = frame_id;
	state.pose.back().pose.position.x    = x;
	state.pose.back().pose.position.y    = y;
	state.pose.back().pose.position.z    = z;
	state.pose.back().pose.orientation.w = qw;
	state.pose.back().pose.orientation.x = qx;
	state.pose.back().pose.orientation.y = qy;
	state.pose.back().pose.orientation.z = qz;
	return state;
}

int mocap_id_for_body(mjModel *model, const std::string &body_name)
{
	const int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
	if (body_id == -1) {
		return -1;
	}
	return model->body_mocapid[body_id];
}

void expect_mocap_pose(mjData *data, int mocap_id, double x, double y, double z, double qw, double qx, double qy,
                       double qz)
{
	ASSERT_GE(mocap_id, 0);
	EXPECT_NEAR(data->mocap_pos[mocap_id * 3], x, 1e-9);
	EXPECT_NEAR(data->mocap_pos[mocap_id * 3 + 1], y, 1e-9);
	EXPECT_NEAR(data->mocap_pos[mocap_id * 3 + 2], z, 1e-9);
	EXPECT_NEAR(data->mocap_quat[mocap_id * 4], qw, 1e-9);
	EXPECT_NEAR(data->mocap_quat[mocap_id * 4 + 1], qx, 1e-9);
	EXPECT_NEAR(data->mocap_quat[mocap_id * 4 + 2], qy, 1e-9);
	EXPECT_NEAR(data->mocap_quat[mocap_id * 4 + 3], qz, 1e-9);
}

template <typename MessageT>
auto create_publisher(MujocoEnvTestWrapper *env_ptr, testing::TestNodeHandle *nh, const std::string &topic,
                      int queue_size)
{
#if MJR_ROS_VERSION == ROS_1
	(void)env_ptr;
	return nh->template advertise<MessageT>(topic, queue_size);
#else
	(void)nh;
	return env_ptr->template create_publisher<MessageT>(topic, rclcpp::QoS(queue_size));
#endif
}

void publish_mocap_state(MujocoEnvTestWrapper *env_ptr, testing::TestNodeHandle *nh, const MocapState &state)
{
	auto publisher      = create_publisher<MocapState>(env_ptr, nh, mocap_topic_name(env_ptr), 1);
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
	while (std::chrono::steady_clock::now() < deadline) {
#if MJR_ROS_VERSION == ROS_1
		if (publisher.getNumSubscribers() > 0) {
			break;
		}
#else
		if (publisher->get_subscription_count() > 0) {
			break;
		}
#endif
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
#if MJR_ROS_VERSION == ROS_1
	publisher.publish(state);
#else
	publisher->publish(state);
#endif
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

bool call_set_mocap_state(MujocoEnvTestWrapper *env_ptr, testing::ServiceCall<SetMocapState> &srv)
{
	return testing::service_call_for_test(env_ptr, mocap_service_name(env_ptr), srv);
}

std::vector<testing::TopicInfo> wait_for_mocap_topic(MujocoEnvTestWrapper *env_ptr)
{
	std::vector<testing::TopicInfo> topics;
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
	while (std::chrono::steady_clock::now() < deadline) {
#if MJR_ROS_VERSION == ROS_1
		XmlRpc::XmlRpcValue request;
		XmlRpc::XmlRpcValue response;
		XmlRpc::XmlRpcValue payload;
		request[0] = ros::this_node::getName();
		ros::master::execute("getSystemState", request, response, payload, true);
		topics.clear();
		if (payload.getType() == XmlRpc::XmlRpcValue::TypeArray && payload.size() >= 2 &&
		    payload[1].getType() == XmlRpc::XmlRpcValue::TypeArray) {
			for (int i = 0; i < payload[1].size(); ++i) {
				if (payload[1][i].getType() == XmlRpc::XmlRpcValue::TypeArray && payload[1][i].size() >= 1) {
					topics.push_back({ static_cast<std::string>(payload[1][i][0]) });
				}
			}
		}
#else
		topics = testing::get_available_topics_for_test(env_ptr);
#endif
		if (testing::has_topic(topics, mocap_topic_name(env_ptr))) {
			return topics;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return topics;
}

bool wait_for_mocap_service(MujocoEnvTestWrapper *env_ptr)
{
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
	while (std::chrono::steady_clock::now() < deadline) {
		if (testing::service_exists(env_ptr, mocap_service_name(env_ptr), true)) {
			return true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return false;
}

} // namespace

int main(int argc, char **argv)
{
#if MJR_ROS_VERSION == ROS_1
	::testing::InitGoogleTest(&argc, argv);
	init_ros(argc, argv);
#else
	init_ros(argc, argv);
	::testing::InitGoogleTest(&argc, argv);
#endif
	return run_all_tests();
}

class MocapPluginFixture : public ::testing::Test
{
protected:
	std::unique_ptr<testing::TestNodeHandle> nh;
	std::unique_ptr<MujocoEnvTestWrapper> env_ptr;
	mujoco_ros::mocap::MocapPlugin *mocap_plugin = nullptr;
	mjModel *m                                   = nullptr;
	mjData *d                                    = nullptr;

	void SetUp() override
	{
		nh = std::make_unique<testing::TestNodeHandle>("~");
		nh->setParam("unpause", false);
		nh->setParam("no_render", true);
		nh->setParam("use_sim_time", true);
		configure_plugin_params(*nh);

		env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

		const std::string xml_path = get_mocap_model_path();
		env_ptr->StartWithXML(xml_path, false);

		float seconds = 0;
		while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001f;
		}
		ASSERT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

		m = env_ptr->getModelPtr();
		d = env_ptr->getDataPtr();

		for (const auto &plugin : env_ptr->GetPlugins()) {
			mocap_plugin = dynamic_cast<mujoco_ros::mocap::MocapPlugin *>(plugin.get());
			if (mocap_plugin != nullptr) {
				break;
			}
		}
	}

	void TearDown() override
	{
		mocap_plugin = nullptr;
		if (env_ptr != nullptr) {
			env_ptr->shutdown();
		}
		if (nh != nullptr) {
			clear_test_params(*nh);
		}
	}
};

TEST_F(MocapPluginFixture, PluginLoaded)
{
	EXPECT_NE(mocap_plugin, nullptr);
}

TEST_F(MocapPluginFixture, TopicAndServiceCreated)
{
	const auto topics = wait_for_mocap_topic(env_ptr.get());
	EXPECT_TRUE(testing::has_topic(topics, mocap_topic_name(env_ptr.get())));
	EXPECT_TRUE(wait_for_mocap_service(env_ptr.get()));
}

TEST_F(MocapPluginFixture, ServiceUpdatesMocapPose)
{
	testing::ServiceCall<SetMocapState> srv;
	srv.request.mocap_state = make_mocap_state("mocap", "world", 1.0, 2.0, 3.0, 2.0);

	ASSERT_TRUE(call_set_mocap_state(env_ptr.get(), srv));
	EXPECT_TRUE(srv.response.success);

	env_ptr->togglePaused(true);
	ASSERT_TRUE(env_ptr->step());

	const int mocap_id = mocap_id_for_body(m, "mocap");
	expect_mocap_pose(d, mocap_id, 1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0);
}

TEST_F(MocapPluginFixture, TopicUpdatesMocapPose)
{
	const MocapState state = make_mocap_state("mocap2", "world", 0.4, 0.5, 0.6, 1.0);
	publish_mocap_state(env_ptr.get(), nh.get(), state);

	env_ptr->togglePaused(true);
	for (int i = 0; i < 10; ++i) {
		ASSERT_TRUE(env_ptr->step());
	}

	const int mocap_id = mocap_id_for_body(m, "mocap2");
	expect_mocap_pose(d, mocap_id, 0.4, 0.5, 0.6, 1.0, 0.0, 0.0, 0.0);
}

TEST_F(MocapPluginFixture, RejectsUnknownBody)
{
	testing::ServiceCall<SetMocapState> srv;
	srv.request.mocap_state = make_mocap_state("unknown_body", "world", 1.0, 2.0, 3.0, 1.0);

	ASSERT_TRUE(call_set_mocap_state(env_ptr.get(), srv));
	EXPECT_FALSE(srv.response.success);
}

TEST_F(MocapPluginFixture, RejectsNonWorldFrame)
{
	testing::ServiceCall<SetMocapState> srv;
	srv.request.mocap_state = make_mocap_state("mocap", "map", 1.0, 2.0, 3.0, 1.0);

	ASSERT_TRUE(call_set_mocap_state(env_ptr.get(), srv));
	EXPECT_FALSE(srv.response.success);
}

TEST_F(MocapPluginFixture, RejectsNonMocapBody)
{
	testing::ServiceCall<SetMocapState> srv;
	srv.request.mocap_state = make_mocap_state("box", "world", 1.0, 2.0, 3.0, 1.0);

	ASSERT_TRUE(call_set_mocap_state(env_ptr.get(), srv));
	EXPECT_FALSE(srv.response.success);
}
