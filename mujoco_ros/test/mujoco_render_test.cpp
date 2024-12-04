/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Bielefeld University
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
#include <mujoco_ros/offscreen_camera.h>
#include <mujoco_ros/util.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <chrono>
#include <cmath>

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_render_test");

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
	return ret;
}

using namespace mujoco_ros;
namespace mju = ::mujoco::sample_util;

TEST_F(BaseEnvFixture, Not_Headless_Warn)
{
	nh->setParam("no_render", false);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	while (env_ptr->getOperationalStatus() != 0) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
	}

	env_ptr->shutdown();
}

#if RENDER_BACKEND == GLFW_BACKEND || RENDER_BACKEND == EGL_BACKEND || \
    RENDER_BACKEND == OSMESA_BACKEND // i.e. any render backend available
TEST_F(BaseEnvFixture, NoRender_Params_Correct)
{
	nh->setParam("no_render", true);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	bool offscreen = true, headless = false;
	nh->getParam("render_offscreen", offscreen);
	nh->getParam("headless", headless);
	EXPECT_TRUE(headless);
	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_FALSE(offscreen);
	EXPECT_FALSE(env_ptr->settings_.render_offscreen);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Headless_params_correct)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.render_offscreen);
	EXPECT_TRUE(env_ptr->settings_.headless);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RGB_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::RGB);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::streamType::RGB);

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	bool img = false, info = false;
	for (const auto &t : master_topics) {
		if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/rgb/image_raw") {
			img = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/rgb/camera_info") {
			info = true;
		}
		if (img && info)
			break;
	}
	EXPECT_TRUE(img && info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, DEPTH_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::DEPTH);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::streamType::DEPTH);

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	bool img = false, info = false;
	for (const auto &t : master_topics) {
		if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/depth/image_raw") {
			img = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/depth/camera_info") {
			info = true;
		}
		if (img && info)
			break;
	}
	EXPECT_TRUE(img && info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, SEGMENTATION_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::SEGMENTED);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::streamType::SEGMENTED);

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	bool img = false, info = false;
	for (const auto &t : master_topics) {
		if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/segmented/image_raw") {
			img = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/segmented/camera_info") {
			info = true;
		}
		if (img && info)
			break;
	}
	EXPECT_TRUE(img && info);
	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RGB_DEPTH_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::RGB_D);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::streamType::RGB_D);

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	bool found_rgb = false, found_depth = false;
	bool found_rgb_info = false, found_depth_info = false;
	for (const auto &t : master_topics) {
		if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/rgb/image_raw") {
			found_rgb = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/depth/image_raw") {
			found_depth = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/rgb/camera_info") {
			found_rgb_info = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/depth/camera_info") {
			found_depth_info = true;
		}
		if (found_rgb && found_depth && found_rgb_info && found_depth_info)
			break;
	}
	EXPECT_TRUE(found_rgb && found_depth && found_rgb_info && found_depth_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RGB_SEGMENTATION_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::RGB_S);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::streamType::RGB_S);

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	bool found_rgb = false, found_seg = false;
	bool found_rgb_info = false, found_seg_info = false;
	for (const auto &t : master_topics) {
		if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/rgb/image_raw") {
			found_rgb = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/segmented/image_raw") {
			found_seg = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/rgb/camera_info") {
			found_rgb_info = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/segmented/camera_info") {
			found_seg_info = true;
		}
		if (found_rgb && found_seg && found_rgb_info && found_seg_info)
			break;
	}
	EXPECT_TRUE(found_rgb && found_seg && found_rgb_info && found_seg_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, DEPTH_SEGMENTATION_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::DEPTH_S);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::streamType::DEPTH_S);

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	bool found_depth = false, found_seg = false;
	bool found_depth_info = false, found_seg_info = false;
	for (const auto &t : master_topics) {
		if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/depth/image_raw") {
			found_depth = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/segmented/image_raw") {
			found_seg = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/depth/camera_info") {
			found_depth_info = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/segmented/camera_info") {
			found_seg_info = true;
		}
		if (found_depth && found_seg && found_depth_info && found_seg_info)
			break;
	}
	EXPECT_TRUE(found_depth && found_seg && found_depth_info && found_seg_info);
	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RGB_DEPTH_SEGMENTATION_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::RGB_D_S);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::streamType::RGB_D_S);

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	bool found_rgb = false, found_depth = false, found_seg = false;
	bool found_rgb_info = false, found_depth_info = false, found_seg_info = false;
	for (const auto &t : master_topics) {
		if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/rgb/image_raw") {
			found_rgb = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/depth/image_raw") {
			found_depth = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/segmented/image_raw") {
			found_seg = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/rgb/camera_info") {
			found_rgb_info = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/depth/camera_info") {
			found_depth_info = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/segmented/camera_info") {
			found_seg_info = true;
		}
		if (found_rgb && found_depth && found_seg && found_rgb_info && found_depth_info && found_seg_info)
			break;
	}
	EXPECT_TRUE(found_rgb && found_depth && found_seg && found_rgb_info && found_depth_info && found_seg_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Default_Cam_Settings)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);

	// Check default camera settings
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::streamType::RGB);
	EXPECT_EQ(offscreen->cams[0]->pub_freq_, 15);
	EXPECT_EQ(offscreen->cams[0]->width_, 720);
	EXPECT_EQ(offscreen->cams[0]->height_, 480);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Resolution_Settings)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/width", 640);
	nh->setParam("cam_config/test_cam/height", 480);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);

	// Check camera settings
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::streamType::RGB);
	EXPECT_EQ(offscreen->cams[0]->pub_freq_, 15);
	EXPECT_EQ(offscreen->cams[0]->width_, 640);
	EXPECT_EQ(offscreen->cams[0]->height_, 480);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Stream_BaseTopic_Relative)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::RGB_D_S);
	nh->setParam("cam_config/test_cam/topic", "alt_topic");

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_STREQ(offscreen->cams[0]->topic_.c_str(), "alt_topic");

	bool found_rgb = false, found_depth = false, found_seg = false;
	bool found_rgb_info = false, found_depth_info = false, found_seg_info = false;

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	for (const auto &t : master_topics) {
		if (t.name == env_ptr->getHandleNamespace() + "/alt_topic/rgb/image_raw") {
			found_rgb = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/alt_topic/depth/image_raw") {
			found_depth = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/alt_topic/segmented/image_raw") {
			found_seg = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/alt_topic/rgb/camera_info") {
			found_rgb_info = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/alt_topic/depth/camera_info") {
			found_depth_info = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/alt_topic/segmented/camera_info") {
			found_seg_info = true;
		}
		if (found_rgb && found_depth && found_seg && found_rgb_info && found_depth_info && found_seg_info)
			break;
	}
	EXPECT_TRUE(found_rgb && found_depth && found_seg && found_rgb_info && found_depth_info && found_seg_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Stream_BaseTopic_Absolute)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::RGB_D_S);
	nh->setParam("cam_config/test_cam/topic", "/alt_topic");

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_STREQ(offscreen->cams[0]->topic_.c_str(), "/alt_topic");

	bool found_rgb = false, found_depth = false, found_seg = false;
	bool found_rgb_info = false, found_depth_info = false, found_seg_info = false;

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	for (const auto &t : master_topics) {
		if (t.name == "/alt_topic/rgb/image_raw") {
			found_rgb = true;
		} else if (t.name == "/alt_topic/depth/image_raw") {
			found_depth = true;
		} else if (t.name == "/alt_topic/segmented/image_raw") {
			found_seg = true;
		} else if (t.name == "/alt_topic/rgb/camera_info") {
			found_rgb_info = true;
		} else if (t.name == "/alt_topic/depth/camera_info") {
			found_depth_info = true;
		} else if (t.name == "/alt_topic/segmented/camera_info") {
			found_seg_info = true;
		}
		if (found_rgb && found_depth && found_seg && found_rgb_info && found_depth_info && found_seg_info)
			break;
	}
	EXPECT_TRUE(found_rgb && found_depth && found_seg && found_rgb_info && found_depth_info && found_seg_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RGB_Alternative_StreamName)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::RGB);
	nh->setParam("cam_config/test_cam/name_rgb", "alt_rgb");

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");

	bool img = false, found_info = false;

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	for (const auto &t : master_topics) {
		if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/alt_rgb/image_raw") {
			img = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/alt_rgb/camera_info") {
			found_info = true;
		}
		if (img && found_info)
			break;
	}
	EXPECT_TRUE(img && found_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, DEPTH_Alternative_StreamName)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::DEPTH);
	nh->setParam("cam_config/test_cam/name_depth", "alt_depth");

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");

	bool img = false, found_info = false;

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	for (const auto &t : master_topics) {
		if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/alt_depth/image_raw") {
			img = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/alt_depth/camera_info") {
			found_info = true;
		}
		if (img && found_info)
			break;
	}
	EXPECT_TRUE(img && found_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, SEGMENT_Alternative_StreamName)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::SEGMENTED);
	nh->setParam("cam_config/test_cam/name_segment", "alt_seg");

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");

	bool img = false, found_info = false;

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	for (const auto &t : master_topics) {
		if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/alt_seg/image_raw") {
			img = true;
		} else if (t.name == env_ptr->getHandleNamespace() + "/cameras/test_cam/alt_seg/camera_info") {
			found_info = true;
		}
		if (img && found_info)
			break;
	}
	EXPECT_TRUE(img && found_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RGB_Published_Correctly)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("unpause", false);
	nh->setParam("cam_config/test_cam/frequency", 30);
	nh->setParam("cam_config/test_cam/width", 7);
	nh->setParam("cam_config/test_cam/height", 4);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	std::vector<sensor_msgs::Image> rgb_images;
	std::vector<sensor_msgs::CameraInfo> rgb_infos;

	// Subscribe to topic
	ros::Subscriber rgb_sub = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&rgb_images](const sensor_msgs::Image::ConstPtr &msg) { rgb_images.emplace_back(*msg); });
	ros::Subscriber info_sub = nh->subscribe<sensor_msgs::CameraInfo>(
	    "cameras/test_cam/rgb/camera_info", 1,
	    [&rgb_infos](const sensor_msgs::CameraInfo::ConstPtr &msg) { rgb_infos.emplace_back(*msg); });

	env_ptr->startWithXML(xml_path);
	env_ptr->step(1);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();

	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->rgb_pub_.getNumSubscribers(), 1);

	// Wait for image to be published with 1s timeout
	float seconds = 0.f;
	while ((rgb_images.empty() || rgb_infos.empty()) && seconds < 1.f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001f;
	}
	EXPECT_LT(seconds, 1.f) << "RGB image not published within 1s";

	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::streamType::RGB);
	EXPECT_EQ(offscreen->cams[0]->pub_freq_, 30);

	ASSERT_EQ(rgb_images.size(), 1);
	ASSERT_EQ(rgb_infos.size(), 1);

	ros::Time t1 = ros::Time::now();
	EXPECT_EQ(rgb_images[0].header.stamp, t1);
	EXPECT_STREQ(rgb_images[0].header.frame_id.c_str(), "test_cam_optical_frame");
	EXPECT_EQ(rgb_images[0].width, 7);
	EXPECT_EQ(rgb_images[0].height, 4);
	EXPECT_EQ(rgb_images[0].encoding, sensor_msgs::image_encodings::RGB8);

	EXPECT_EQ(rgb_infos[0].header.stamp, t1);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Cam_Timing_Correct)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("unpause", false);
	nh->setParam("cam_config/test_cam/frequency", 30);
	nh->setParam("cam_config/test_cam/width", 7);
	nh->setParam("cam_config/test_cam/height", 4);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	std::vector<sensor_msgs::Image> rgb_images;
	std::vector<sensor_msgs::CameraInfo> rgb_infos;

	// Subscribe to topic
	ros::Subscriber rgb_sub = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&rgb_images](const sensor_msgs::Image::ConstPtr &msg) { rgb_images.emplace_back(*msg); });
	ros::Subscriber info_sub = nh->subscribe<sensor_msgs::CameraInfo>(
	    "cameras/test_cam/rgb/camera_info", 1,
	    [&rgb_infos](const sensor_msgs::CameraInfo::ConstPtr &msg) { rgb_infos.emplace_back(*msg); });

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	env_ptr->step(1);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);

	// Check default camera settings
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::streamType::RGB);
	EXPECT_EQ(offscreen->cams[0]->pub_freq_, 30);

	// Wait for image to be published with 400ms timeout
	float seconds = 0.f;
	while ((rgb_images.empty() || rgb_infos.empty()) && seconds < .4f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001f;
	}
	EXPECT_LT(seconds, .4f) << "RGB image not published within 400ms";

	ASSERT_EQ(rgb_infos.size(), 1);
	ASSERT_EQ(rgb_images.size(), 1);

	ros::Time t1 = ros::Time::now();
	// Step the simulation to as to trigger the camera rendering
	mjModel *m  = env_ptr->getModelPtr();
	int n_steps = std::ceil((1.0 / 30.0) / env_ptr->getModelPtr()->opt.timestep);

	env_ptr->step(n_steps - 1);
	// wait a little
	std::this_thread::sleep_for(std::chrono::milliseconds(5));

	// should not have received a new image yet
	ASSERT_EQ(rgb_infos.size(), 1);
	ASSERT_EQ(rgb_images.size(), 1);

	env_ptr->step(1);
	// Wait for image to be published with 400ms timeout
	seconds = 0.f;
	while ((rgb_images.size() < 2 || rgb_infos.size() < 2) && seconds < .4f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001f;
	}
	// should now have received new image
	EXPECT_LT(seconds, .4f) << "RGB image not published within 400ms";

	ASSERT_EQ(rgb_infos.size(), 2);
	ASSERT_EQ(rgb_images.size(), 2);
	ros::Time t2 = ros::Time::now();

	ASSERT_EQ(rgb_images[0].header.stamp, t1);
	ASSERT_EQ(rgb_images[1].header.stamp, t2);

	ASSERT_EQ(rgb_infos[0].header.stamp, t1);
	ASSERT_EQ(rgb_infos[1].header.stamp, t2);

	// int n_steps = std::ceil((1.0 / 30.0) / env_ptr->getModelPtr()->opt.timestep);
	ros::Time t3 = t2 + ros::Duration((std::ceil((1.0 / 30.0) / m->opt.timestep)) * m->opt.timestep);
	// ros::Time t3 = t2 + (t2 - t1);
	// Step over next image trigger but before trigger after that
	env_ptr->step(2 * n_steps - 1);

	ASSERT_EQ(rgb_infos.size(), 3);
	ASSERT_EQ(rgb_images.size(), 3);

	// Check that the timestamps are as expected
	EXPECT_EQ(rgb_images[2].header.stamp, rgb_infos[2].header.stamp);
	EXPECT_EQ(rgb_images[2].header.stamp, t3);
	EXPECT_EQ(rgb_infos[2].header.stamp, t3);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RGB_Image_Dtype)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("unpause", false);
	nh->setParam("cam_config/test_cam/width", 7);
	nh->setParam("cam_config/test_cam/height", 4);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	std::vector<sensor_msgs::Image> rgb_images;
	// Subscribe to topic
	ros::Subscriber rgb_sub = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&rgb_images](const sensor_msgs::Image::ConstPtr &msg) { rgb_images.emplace_back(*msg); });

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	env_ptr->step(1);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::streamType::RGB);
	EXPECT_EQ(offscreen->cams[0]->width_, 7);
	EXPECT_EQ(offscreen->cams[0]->height_, 4);

	// Wait for image to be published with 400ms timeout
	float seconds = 0.f;
	while (rgb_images.empty() && seconds < .4f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001f;
	}
	EXPECT_LT(seconds, .4f) << "RGB image not published within 400ms";

	ASSERT_EQ(rgb_images.size(), 1);
	EXPECT_EQ(rgb_images[0].data.size(), 7 * 4 * 3);
	EXPECT_EQ(rgb_images[0].encoding, sensor_msgs::image_encodings::RGB8);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, DEPTH_Image_Dtype)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("unpause", false);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::DEPTH);
	nh->setParam("cam_config/test_cam/width", 7);
	nh->setParam("cam_config/test_cam/height", 4);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	std::vector<sensor_msgs::Image> depth_images;

	// Subscribe to topic
	ros::Subscriber depth_sub = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/depth/image_raw", 1,
	    [&depth_images](const sensor_msgs::Image::ConstPtr &msg) { depth_images.emplace_back(*msg); });

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	env_ptr->step(1);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::streamType::DEPTH);
	EXPECT_EQ(offscreen->cams[0]->width_, 7);
	EXPECT_EQ(offscreen->cams[0]->height_, 4);

	// Wait for image to be published with 200ms timeout
	float seconds = 0.f;
	while (depth_images.empty() && seconds < .2f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001f;
	}
	EXPECT_LT(seconds, .2f) << "Depth image not published within 200ms";

	ASSERT_EQ(depth_images.size(), 1);
	EXPECT_EQ(depth_images[0].width, 7);
	EXPECT_EQ(depth_images[0].height, 4);
	EXPECT_EQ(depth_images[0].encoding, sensor_msgs::image_encodings::TYPE_32FC1);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, SEGMENTED_Image_Dtype)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("unpause", false);
	nh->setParam("cam_config/test_cam/stream_type", rendering::streamType::SEGMENTED);
	nh->setParam("cam_config/test_cam/width", 7);
	nh->setParam("cam_config/test_cam/height", 4);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	std::vector<sensor_msgs::Image> seg_images;

	// Subscribe to topic
	ros::Subscriber seg_sub = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/segmented/image_raw", 1,
	    [&seg_images](const sensor_msgs::Image::ConstPtr &msg) { seg_images.emplace_back(*msg); });

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	env_ptr->step(1);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::streamType::SEGMENTED);
	EXPECT_EQ(offscreen->cams[0]->width_, 7);
	EXPECT_EQ(offscreen->cams[0]->height_, 4);

	// Wait for image to be published with 200ms timeout
	float seconds = 0.f;
	while (seg_images.empty() && seconds < .2f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001f;
	}
	EXPECT_LT(seconds, .2f) << "Segmentation image not published within 200ms";

	ASSERT_EQ(seg_images.size(), 1);
	EXPECT_EQ(seg_images[0].data.size(), 7 * 4 * 3);
	EXPECT_EQ(seg_images[0].encoding, sensor_msgs::image_encodings::RGB8);

	env_ptr->shutdown();
}

#endif // RENDER_BACKEND == GLFW_BACKEND || RENDER_BACKEND == EGL_BACKEND || RENDER_BACKEND == OSMESA_BACKEND // i.e.
       // any render backend available

#if RENDER_BACKEND == NO_BACKEND // i.e. no render backend available
TEST_F(BaseEnvFixture, No_Render_Backend_Headless_Warn)
{
	nh->setParam("headless", true);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	while (env_ptr->getOperationalStatus() != 0) { // wait for model to be loaded
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
	}

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_FALSE(env_ptr->settings_.render_offscreen);

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	EXPECT_TRUE(offscreen->cams.empty());

	env_ptr->shutdown();
}
#endif // RENDER_BACKEND == NO_BACKEND // i.e. no render backend available
