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

#include <ros/package.h>

#include "mujoco_env_fixture.h"
#include "test_plugin/test_plugin.h"

#include <mujoco_ros/render_backend.h>
#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/plugin_utils.h>
#include <string>

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_ros_plugin_test");

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

class LoadedPluginFixture : public ::testing::Test
{
protected:
	std::unique_ptr<ros::NodeHandle> nh;
	TestPlugin *test_plugin = nullptr;
	MujocoEnvTestWrapper *env_ptr;

	void SetUp() override
	{
		nh = std::make_unique<ros::NodeHandle>("~");
		nh->setParam("unpause", false);
		nh->setParam("no_render", true);
		nh->setParam("headless", true);
		nh->setParam("use_sim_time", true);

		env_ptr              = new MujocoEnvTestWrapper();
		std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
		env_ptr->startWithXML(xml_path);

		float seconds = 0;
		while (env_ptr->getOperationalStatus() != 0 && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";

		auto &plugins = env_ptr->getPlugins();
		for (const auto &p : plugins) {
			test_plugin = dynamic_cast<TestPlugin *>(p.get());
			if (test_plugin != nullptr) {
				break;
			}
		}

		ASSERT_NE(test_plugin, nullptr) << "TestPlugin not found!";
	}

	void TearDown() override
	{
		// cleanup all parameters
		ros::param::del(nh->getNamespace());
		test_plugin = nullptr;
		env_ptr->shutdown();
		delete env_ptr;
	}
};

TEST_F(LoadedPluginFixture, ControlCallback)
{
	// mjcb_control is called in mj_forward, which is also called when paused
	// we can't guarantee that the control callback has not been called yet, if the test's
	// timing is too slow
	// EXPECT_FALSE(test_plugin->ran_control_cb.load());
	EXPECT_TRUE(env_ptr->step());
	EXPECT_TRUE(test_plugin->ran_control_cb.load());
}

TEST_F(LoadedPluginFixture, PassiveCallback)
{
	// mjcb_passive is called in mj_forward, which is also called when paused
	// we can't guarantee that the passive callback has not been called yet, if the test's
	// timing is too slow
	// EXPECT_FALSE(test_plugin->ran_passive_cb.load());
	EXPECT_TRUE(env_ptr->step());
	EXPECT_TRUE(test_plugin->ran_passive_cb.load());
}

#if RENDER_BACKEND == GLFW_BACKEND || RENDER_BACKEND == EGL_BACKEND || RENDER_BACKEND == OSMESA_BACKEND
TEST_F(BaseEnvFixture, RenderCallback)
{
	nh->setParam("no_render", false);
	nh->setParam("unpause", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/width", 7);
	nh->setParam("cam_config/test_cam/height", 4);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	// NOP subscriber to trigger render callback
	ros::Subscriber rgb_sub = nh->subscribe<sensor_msgs::Image>("cameras/test_cam/rgb/image_raw", 1,
	                                                            [&](const sensor_msgs::Image::ConstPtr & /*msg*/) {});

	env_ptr->startWithXML(xml_path);

	EXPECT_TRUE(env_ptr->step());

	OffscreenRenderContext *offscreen = env_ptr->getOffscreenContext();
	EXPECT_TRUE(offscreen->cams.size() == 1);

	TestPlugin *test_plugin = nullptr;
	auto &plugins           = env_ptr->getPlugins();
	for (const auto &p : plugins) {
		test_plugin = dynamic_cast<TestPlugin *>(p.get());
		if (test_plugin != nullptr) {
			break;
		}
	}

	ASSERT_NE(test_plugin, nullptr) << "TestPlugin not found!";

	// wait for render callback to be called
	float seconds = 0;
	while (!test_plugin->ran_render_cb.load() && seconds < 1.) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 1) << "Render callback was not called within 1 second!";

	env_ptr->shutdown();
}
#endif

#if RENDER_BACKEND == NO_BACKEND
TEST_F(BaseEnvFixture, RenderCallback_NoRender)
{
	nh->setParam("no_render", false);
	nh->setParam("unpause", false);
	nh->setParam("headless", true);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/camera_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	// NOP subscriber to trigger render callback
	ros::Subscriber rgb_sub = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&](const sensor_msgs::Image::ConstPtr & /*msg*/) { ROS_ERROR("Got image!"); });

	env_ptr->startWithXML(xml_path);
	EXPECT_TRUE(env_ptr->step(5));

	TestPlugin *test_plugin = nullptr;
	auto &plugins           = env_ptr->getPlugins();
	for (const auto &p : plugins) {
		test_plugin = dynamic_cast<TestPlugin *>(p.get());
		if (test_plugin != nullptr) {
			break;
		}
	}

	// wait for render callback to be called
	float seconds = 0;
	while (!test_plugin->ran_render_cb.load() && seconds < .1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_FALSE(test_plugin->ran_render_cb.load()) << "Render callback was called!";

	env_ptr->shutdown();
}
#endif

TEST_F(LoadedPluginFixture, LastCallback)
{
	EXPECT_FALSE(test_plugin->ran_last_cb.load());
	EXPECT_TRUE(env_ptr->step());
	EXPECT_TRUE(test_plugin->ran_last_cb.load());
}

TEST_F(LoadedPluginFixture, OnGeomChangedCallback)
{
	EXPECT_FALSE(test_plugin->ran_on_geom_changed_cb.load());
	env_ptr->notifyGeomChange();
	EXPECT_TRUE(test_plugin->ran_on_geom_changed_cb.load());
}

TEST_F(BaseEnvFixture, LoadPlugin)
{
	nh->setParam("no_render", true);
	nh->setParam("unpause", false);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	float seconds = 0;
	while (env_ptr->getOperationalStatus() != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";
	EXPECT_EQ(env_ptr->getPlugins().size(), 1) << "Env should have 1 plugin registered!";
	EXPECT_EQ(env_ptr->getNumCBReadyPlugins(), 1) << "Env should have 1 plugin loaded!";

	env_ptr->shutdown();
}

TEST_F(LoadedPluginFixture, ResetPlugin)
{
	env_ptr->settings_.reset_request = 1;
	float seconds                    = 0;
	while (env_ptr->settings_.reset_request != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env reset ran into 2 seconds timeout!";
	env_ptr->step(10);

	EXPECT_TRUE(test_plugin->ran_reset.load()) << "Dummy plugin reset was not called!";
}

TEST_F(LoadedPluginFixture, GetConfigToplevel)
{
	EXPECT_TRUE(test_plugin->got_config_param.load());
}

TEST_F(LoadedPluginFixture, GetConfigArray)
{
	EXPECT_TRUE(test_plugin->got_lvl1_nested_array.load());
	EXPECT_TRUE(test_plugin->got_lvl2_nested_array.load());
}

TEST_F(LoadedPluginFixture, GetConfigStruct)
{
	EXPECT_TRUE(test_plugin->got_lvl1_nested_struct.load());
	EXPECT_TRUE(test_plugin->got_lvl2_nested_struct.load());
}

TEST_F(BaseEnvFixture, FailedLoad)
{
	nh->setParam("unpause", false);
	nh->setParam("should_fail", true);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	float seconds = 0;
	while (env_ptr->getOperationalStatus() != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";

	EXPECT_EQ(env_ptr->getPlugins().size(), 1) << "Env should have 1 plugin registered!";
	EXPECT_EQ(env_ptr->getNumCBReadyPlugins(), 0) << "Env should have 0 plugins loaded!";

	{
		TestPlugin *test_plugin = nullptr;

		auto &plugins = env_ptr->getPlugins();
		for (const auto &p : plugins) {
			test_plugin = dynamic_cast<TestPlugin *>(p.get());
			if (test_plugin != nullptr) {
				break;
			}
		}

		ASSERT_NE(test_plugin, nullptr) << "Dummy plugin was not loaded!";

		EXPECT_FALSE(test_plugin->ran_control_cb.load());
		EXPECT_FALSE(test_plugin->ran_passive_cb.load());
		EXPECT_FALSE(test_plugin->ran_render_cb.load());
		EXPECT_FALSE(test_plugin->ran_last_cb.load());
		EXPECT_FALSE(test_plugin->ran_on_geom_changed_cb.load());
	}

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, FailedLoadRecoverReload)
{
	nh->setParam("should_fail", true);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	float seconds = 0;
	while (env_ptr->getOperationalStatus() != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";

	EXPECT_EQ(env_ptr->getPlugins().size(), 1) << "Env should have 1 plugin registered!";
	EXPECT_EQ(env_ptr->getNumCBReadyPlugins(), 0) << "Env should have 0 plugins loaded!";

	{
		TestPlugin *test_plugin = nullptr;

		auto &plugins = env_ptr->getPlugins();
		for (const auto &p : plugins) {
			test_plugin = dynamic_cast<TestPlugin *>(p.get());
			if (test_plugin != nullptr) {
				break;
			}
		}

		ASSERT_NE(test_plugin, nullptr) << "Dummy plugin was not loaded!";

		nh->setParam("should_fail", false);

		env_ptr->settings_.load_request = 2;
		float seconds                   = 0;
		while (env_ptr->getOperationalStatus() != 0 && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		EXPECT_LT(seconds, 2) << "Env reset ran into 2 seconds timeout!";
		EXPECT_EQ(env_ptr->getPlugins().size(), 1) << "Env should have 1 plugin registered!";
		EXPECT_EQ(env_ptr->getNumCBReadyPlugins(), 1) << "Env should have 1 plugin loaded!";
	}

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, FailedLoadReset)
{
	nh->setParam("should_fail", true);
	nh->setParam("unpause", false);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("");

	env_ptr->startWithXML(xml_path);

	float seconds = 0;
	while (env_ptr->getOperationalStatus() != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";

	EXPECT_EQ(env_ptr->getPlugins().size(), 1) << "Env should have 1 plugin registered!";
	EXPECT_EQ(env_ptr->getNumCBReadyPlugins(), 0) << "Env should have 0 plugins loaded!";

	{
		TestPlugin *test_plugin = nullptr;

		auto &plugins = env_ptr->getPlugins();
		for (const auto &p : plugins) {
			test_plugin = dynamic_cast<TestPlugin *>(p.get());
			if (test_plugin != nullptr) {
				break;
			}
		}

		ASSERT_NE(test_plugin, nullptr) << "Dummy plugin was not loaded!";

		env_ptr->settings_.reset_request = 1;
		float seconds                    = 0;
		while (env_ptr->settings_.reset_request != 0 && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		EXPECT_LT(seconds, 2) << "Env reset ran into 2 seconds timeout!";
		env_ptr->step(10);

		EXPECT_FALSE(test_plugin->ran_reset.load()) << "Dummy plugin should not have beeon reset!";
	}

	env_ptr->shutdown();
}

TEST_F(LoadedPluginFixture, PluginStats_InitialPaused)
{
	EXPECT_EQ(env_ptr->settings_.run, 0) << "Env should be paused!";

	mujoco_ros_msgs::GetPluginStats srv;
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/get_plugin_stats", true))
	    << "Plugin stats service should exist!";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/get_plugin_stats", srv))
	    << "Get plugin stats service call failed!";
	EXPECT_EQ(srv.response.stats.size(), 1) << "Should have 1 plugin stats!";
	EXPECT_EQ(srv.response.stats[0].plugin_type, "mujoco_ros/TestPlugin") << "Should be TestPlugin!";
	EXPECT_GT(srv.response.stats[0].load_time, -1) << "Load time should be set!";
	EXPECT_EQ(srv.response.stats[0].reset_time, -1) << "Reset time should be unset!";
	// passive and control are also run when paused
	EXPECT_NEAR(srv.response.stats[0].ema_steptime_control, 0, 1e-7) << "Control time should be unset!";
	EXPECT_NEAR(srv.response.stats[0].ema_steptime_passive, 0, 1e-7) << "Passive time should be unset!";
	EXPECT_NEAR(srv.response.stats[0].ema_steptime_render, 0, 1e-8) << "Render time should be unset!";
	EXPECT_NEAR(srv.response.stats[0].ema_steptime_last_stage, 0, 1e-8) << "Last stage time should be unset!";
}

TEST_F(LoadedPluginFixture, PluginStats_SetTimesOnStep)
{
	EXPECT_EQ(env_ptr->settings_.run, 0) << "Env should be paused!";

	env_ptr->step(1);
	// sleep for a bit to ensure the plugin callbacks have been called
	std::this_thread::sleep_for(std::chrono::milliseconds(1));

	mujoco_ros_msgs::GetPluginStats srv;
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/get_plugin_stats", true))
	    << "Plugin stats service should exist!";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/get_plugin_stats", srv))
	    << "Get plugin stats service call failed!";

	EXPECT_EQ(srv.response.stats.size(), 1) << "Should have 1 plugin stats!";
	EXPECT_EQ(srv.response.stats[0].plugin_type, "mujoco_ros/TestPlugin") << "Should be TestPlugin!";
	EXPECT_GT(srv.response.stats[0].load_time, -1) << "Load time should be set!";
	EXPECT_EQ(srv.response.stats[0].reset_time, -1) << "Reset time should be unset!";
	EXPECT_GT(srv.response.stats[0].ema_steptime_control, -1) << "Control time should be unset!";
	EXPECT_GT(srv.response.stats[0].ema_steptime_passive, -1) << "Passive time should be unset!";
	// EXPECT_GT(srv.response.stats[0].ema_steptime_render, -1) << "Render time should be unset!"; // TODO: add when
	// rendering is enabled in tests
	EXPECT_GT(srv.response.stats[0].ema_steptime_last_stage, -1) << "Last stage time should be unset!";
}

TEST_F(LoadedPluginFixture, PluginStats_ResetTimeOnReset)
{
	env_ptr->settings_.reset_request = 1;
	float seconds                    = 0;
	while (env_ptr->settings_.reset_request != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env reset ran into 2 seconds timeout!";

	mujoco_ros_msgs::GetPluginStats srv;
	EXPECT_TRUE(ros::service::exists(env_ptr->getHandleNamespace() + "/get_plugin_stats", true))
	    << "Plugin stats service should exist!";
	EXPECT_TRUE(ros::service::call(env_ptr->getHandleNamespace() + "/get_plugin_stats", srv))
	    << "Get plugin stats service call failed!";

	EXPECT_EQ(srv.response.stats.size(), 1) << "Should have 1 plugin stats!";
	EXPECT_EQ(srv.response.stats[0].plugin_type, "mujoco_ros/TestPlugin") << "Should be TestPlugin!";
	EXPECT_GT(srv.response.stats[0].reset_time, -1) << "Reset time should be unset!";
}
