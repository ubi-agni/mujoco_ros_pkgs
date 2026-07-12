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
#include <mujoco_ros_laser/laser.hpp>
#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#endif

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace mujoco_ros;
using namespace mujoco_ros::sensors::laser;

namespace {

#if MJR_ROS_VERSION == ROS_1
using LaserScanMsg = sensor_msgs::LaserScan;

template <typename MessageT>
using MessageConstPtr = typename MessageT::ConstPtr;
#else
using LaserScanMsg = sensor_msgs::msg::LaserScan;

template <typename MessageT>
using MessageConstPtr = typename MessageT::ConstSharedPtr;
#endif

std::string get_laser_model_path()
{
#if MJR_ROS_VERSION == ROS_1
	return ros::package::getPath("mujoco_ros_laser") + "/assets/laser_world.xml";
#else
	return ament_index_cpp::get_package_share_directory("mujoco_ros_laser") + "/assets/laser_world.xml";
#endif
}

void configure_plugin_params(testing::TestNodeHandle &nh)
{
#if MJR_ROS_VERSION == ROS_2
	nh.setParam("MujocoPlugins.names", std::vector<std::string>{ "mujoco_ros_laser" });
	nh.setParam("MujocoPlugins.mujoco_ros_laser.type", "mujoco_ros_laser/LaserPlugin");
	nh.setParam("MujocoPlugins.mujoco_ros_laser.sensors", std::vector<std::string>{ "scan" });
	nh.setParam("MujocoPlugins.mujoco_ros_laser.scan.site_attached", "laser_site");
	nh.setParam("MujocoPlugins.mujoco_ros_laser.scan.frame_id", "scan");
	nh.setParam("MujocoPlugins.mujoco_ros_laser.scan.visualize", true);
	nh.setParam("MujocoPlugins.mujoco_ros_laser.scan.update_rate", 10.0);
	nh.setParam("MujocoPlugins.mujoco_ros_laser.scan.min_range", 0.1);
	nh.setParam("MujocoPlugins.mujoco_ros_laser.scan.max_range", 30.0);
	nh.setParam("MujocoPlugins.mujoco_ros_laser.scan.range_resolution", 0.01);
	nh.setParam("MujocoPlugins.mujoco_ros_laser.scan.angular_resolution", 0.02);
	nh.setParam("MujocoPlugins.mujoco_ros_laser.scan.min_angle", -1.57);
	nh.setParam("MujocoPlugins.mujoco_ros_laser.scan.max_angle", 1.57);
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
	ros::init(argc, argv, "mujoco_ros_laser_test");
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

std::string scan_topic_name(MujocoEnvTestWrapper *env_ptr)
{
#if MJR_ROS_VERSION == ROS_1
	(void)env_ptr;
	return "/scan";
#else
	return env_ptr->GetHandleNamespace() + "/scan";
#endif
}

template <typename MessageT, typename CallbackT>
auto subscribe_to(MujocoEnvTestWrapper *env_ptr, testing::TestNodeHandle *nh, const std::string &topic, int queue_size,
                  CallbackT &&callback)
{
#if MJR_ROS_VERSION == ROS_1
	(void)env_ptr;
	return nh->template subscribe<MessageT>(topic, queue_size, std::forward<CallbackT>(callback));
#else
	(void)nh;
	return env_ptr->template create_subscription<MessageT>(topic, rclcpp::QoS(queue_size),
	                                                       std::forward<CallbackT>(callback));
#endif
}

std::vector<testing::TopicInfo> wait_for_scan_topic(MujocoEnvTestWrapper *env_ptr)
{
	std::vector<testing::TopicInfo> topics;
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
	while (std::chrono::steady_clock::now() < deadline) {
		topics = testing::get_available_topics_for_test(env_ptr);
		if (testing::has_topic(topics, scan_topic_name(env_ptr))) {
			return topics;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return topics;
}

MessageConstPtr<LaserScanMsg> wait_for_scan_message(MujocoEnvTestWrapper *env_ptr, testing::TestNodeHandle *nh)
{
	MessageConstPtr<LaserScanMsg> message;
	auto sub = subscribe_to<LaserScanMsg>(env_ptr, nh, scan_topic_name(env_ptr), 10,
	                                      [&message](const MessageConstPtr<LaserScanMsg> &msg) { message = msg; });

	env_ptr->togglePaused(true);
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
	while (message == nullptr && std::chrono::steady_clock::now() < deadline) {
		env_ptr->step(25);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	return message;
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

class LoadedPluginFixture : public ::testing::TestWithParam<int>
{
protected:
	std::unique_ptr<testing::TestNodeHandle> nh;
	std::unique_ptr<MujocoEnvTestWrapper> env_ptr;
	LaserPlugin *laser_plugin = nullptr;

	void SetUp() override
	{
		nh = std::make_unique<testing::TestNodeHandle>("~");
		nh->setParam("unpause", false);
		nh->setParam("no_render", true);
		nh->setParam("use_sim_time", true);
		nh->setParam("num_mj_threads", GetParam());
		configure_plugin_params(*nh);

		env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

		const std::string xml_path = get_laser_model_path();
		env_ptr->StartWithXML(xml_path, false);

		float seconds = 0;
		while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001f;
		}
		ASSERT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

		for (const auto &plugin : env_ptr->GetPlugins()) {
			laser_plugin = dynamic_cast<LaserPlugin *>(plugin.get());
			if (laser_plugin != nullptr) {
				break;
			}
		}
	}

	void TearDown() override
	{
		laser_plugin = nullptr;
		if (env_ptr != nullptr) {
			env_ptr->shutdown();
		}
		if (nh != nullptr) {
			clear_test_params(*nh);
		}
	}
};

TEST_P(LoadedPluginFixture, PluginLoaded)
{
	EXPECT_NE(laser_plugin, nullptr) << "Plugin loading failed!";
}

TEST_P(LoadedPluginFixture, ScanTopicCreated)
{
	const auto topics = wait_for_scan_topic(env_ptr.get());
	EXPECT_TRUE(testing::has_topic(topics, scan_topic_name(env_ptr.get())))
	    << "Laser scan topic should have been generated";
}

TEST_P(LoadedPluginFixture, ThreadpoolModeMatchesConfiguration)
{
	const bool has_threadpool = static_cast<bool>(env_ptr->getDataPtr()->threadpool);
	if (GetParam() > 1) {
		EXPECT_TRUE(has_threadpool) << "Expected MuJoCo threadpool for multithreaded laser execution";
	} else {
		EXPECT_FALSE(has_threadpool) << "Expected single-threaded laser execution";
	}
}

TEST_P(LoadedPluginFixture, PublishesLaserScan)
{
	const auto msg = wait_for_scan_message(env_ptr.get(), nh.get());
	ASSERT_TRUE(msg != nullptr) << "Could not get message on " << scan_topic_name(env_ptr.get());

	EXPECT_EQ(msg->header.frame_id, "scan");
	EXPECT_NEAR(msg->range_min, 0.1, 1e-6);
	EXPECT_NEAR(msg->range_max, 30.0, 1e-6);
	EXPECT_NEAR(msg->angle_min, -1.57, 1e-6);
	EXPECT_NEAR(msg->angle_max, 1.57, 1e-6);
	EXPECT_NEAR(msg->angle_increment, 0.02, 1e-6);
	EXPECT_FALSE(msg->ranges.empty());
}

INSTANTIATE_TEST_SUITE_P(LaserThreading, LoadedPluginFixture, ::testing::Values(1, 2),
                         [](const ::testing::TestParamInfo<int> &info) {
	                         return info.param == 1 ? "SingleThreaded" : "MultiThreaded";
                         });
