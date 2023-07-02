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

#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/plugin_utils.h>
#include <string>

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_ros_plugin_test");
	return RUN_ALL_TESTS();
}

class LoadedPluginFixture : public ::testing::Test
{
protected:
	boost::shared_ptr<ros::NodeHandle> nh;
	TestPlugin *test_plugin;
	MujocoEnvTestWrapper *env_ptr;

	virtual void SetUp()
	{
		nh.reset(new ros::NodeHandle("~"));
		nh->setParam("unpause", false);
		nh->setParam("no_x", true);
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

		auto plugins = env_ptr->getPlugins();
		for (auto p : plugins) {
			test_plugin = dynamic_cast<TestPlugin *>(p.get());
			if (test_plugin != nullptr) {
				break;
			}
		}
	}

	virtual void TearDown()
	{
		test_plugin = nullptr;
		env_ptr->shutdown();
		delete env_ptr;
	}
};

TEST_F(LoadedPluginFixture, ControlCallback)
{
	EXPECT_FALSE(test_plugin->ran_control_cb);
	EXPECT_TRUE(env_ptr->step());
	EXPECT_TRUE(test_plugin->ran_control_cb);
}

TEST_F(LoadedPluginFixture, PassiveCallback)
{
	// EXPECT_FALSE(test_plugin->ran_passive_cb);
	EXPECT_TRUE(env_ptr->step());
	EXPECT_TRUE(test_plugin->ran_passive_cb);
}

// TODO: Involves offscreen rendering, can we do this in tests?
// TEST_F(LoadedPluginFixture, RenderCallback) {
// 	EXPECT_TRUE(test_plugin->ran_render_cb);
// }

TEST_F(LoadedPluginFixture, LastCallback)
{
	EXPECT_FALSE(test_plugin->ran_last_cb);
	EXPECT_TRUE(env_ptr->step());
	EXPECT_TRUE(test_plugin->ran_last_cb);
}

TEST_F(LoadedPluginFixture, OnGeomChangedCallback)
{
	EXPECT_FALSE(test_plugin->ran_on_geom_changed_cb);
	env_ptr->notifyGeomChange();
	EXPECT_TRUE(test_plugin->ran_on_geom_changed_cb);
}

TEST_F(BaseEnvFixture, LoadPlugin)
{
	nh->setParam("unpause", false);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env;

	env.startWithXML(xml_path);

	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";
	EXPECT_EQ(env.getPlugins().size(), 1) << "Env should have 1 plugin registered!";
	EXPECT_EQ(env.getNumCBReadyPlugins(), 1) << "Env should have 1 plugin loaded!";

	env.shutdown();
	nh->setParam("unpause", true);
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

	EXPECT_TRUE(test_plugin->ran_reset) << "Dummy plugin reset was not called!";
}

TEST_F(LoadedPluginFixture, GetConfigToplevel)
{
	EXPECT_TRUE(test_plugin->got_config_param);
}

TEST_F(LoadedPluginFixture, GetConfigArray)
{
	EXPECT_TRUE(test_plugin->got_lvl1_nested_array);
	EXPECT_TRUE(test_plugin->got_lvl2_nested_array);
}

TEST_F(LoadedPluginFixture, GetConfigStruct)
{
	EXPECT_TRUE(test_plugin->got_lvl1_nested_struct);
	EXPECT_TRUE(test_plugin->got_lvl2_nested_struct);
}

TEST_F(BaseEnvFixture, FailedLoad)
{
	nh->setParam("unpause", false);
	nh->setParam("should_fail", true);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env;

	env.startWithXML(xml_path);

	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";

	EXPECT_EQ(env.getPlugins().size(), 1) << "Env should have 1 plugin registered!";
	EXPECT_EQ(env.getNumCBReadyPlugins(), 0) << "Env should have 0 plugins loaded!";

	{
		TestPlugin *test_plugin = nullptr;

		auto plugins = env.getPlugins();
		for (auto p : plugins) {
			test_plugin = dynamic_cast<TestPlugin *>(p.get());
			if (test_plugin != nullptr) {
				break;
			}
		}

		EXPECT_NE(test_plugin, nullptr) << "Dummy plugin was not loaded!";

		EXPECT_FALSE(test_plugin->ran_control_cb);
		EXPECT_FALSE(test_plugin->ran_passive_cb);
		EXPECT_FALSE(test_plugin->ran_render_cb);
		EXPECT_FALSE(test_plugin->ran_last_cb);
		EXPECT_FALSE(test_plugin->ran_on_geom_changed_cb);
	}

	env.shutdown();
	nh->setParam("should_fail", false);
	nh->setParam("unpause", true);
}

TEST_F(BaseEnvFixture, FailedLoadRecoverReload)
{
	nh->setParam("should_fail", true);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env;

	env.startWithXML(xml_path);

	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";

	EXPECT_EQ(env.getPlugins().size(), 1) << "Env should have 1 plugin registered!";
	EXPECT_EQ(env.getNumCBReadyPlugins(), 0) << "Env should have 0 plugins loaded!";

	{
		TestPlugin *test_plugin = nullptr;

		auto plugins = env.getPlugins();
		for (auto p : plugins) {
			test_plugin = dynamic_cast<TestPlugin *>(p.get());
			if (test_plugin != nullptr) {
				break;
			}
		}

		nh->setParam("should_fail", false);

		env.settings_.load_request = 2;
		float seconds              = 0;
		while (env.getOperationalStatus() != 0 && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		EXPECT_LT(seconds, 2) << "Env reset ran into 2 seconds timeout!";
		EXPECT_EQ(env.getPlugins().size(), 1) << "Env should have 1 plugin registered!";
		EXPECT_EQ(env.getNumCBReadyPlugins(), 1) << "Env should have 1 plugin loaded!";
	}

	env.shutdown();
}

TEST_F(BaseEnvFixture, FailedLoadReset)
{
	nh->setParam("should_fail", true);
	nh->setParam("unpause", false);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	MujocoEnvTestWrapper env;

	env.startWithXML(xml_path);

	float seconds = 0;
	while (env.getOperationalStatus() != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";

	EXPECT_EQ(env.getPlugins().size(), 1) << "Env should have 1 plugin registered!";
	EXPECT_EQ(env.getNumCBReadyPlugins(), 0) << "Env should have 0 plugins loaded!";

	{
		TestPlugin *test_plugin = nullptr;

		auto plugins = env.getPlugins();
		for (auto p : plugins) {
			test_plugin = dynamic_cast<TestPlugin *>(p.get());
			if (test_plugin != nullptr) {
				break;
			}
		}

		env.settings_.reset_request = 1;
		float seconds               = 0;
		while (env.settings_.reset_request != 0 && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		EXPECT_LT(seconds, 2) << "Env reset ran into 2 seconds timeout!";
		env.step(10);

		EXPECT_FALSE(test_plugin->ran_reset) << "Dummy plugin should not have beeon reset!";
	}

	env.shutdown();
	nh->setParam("should_fail", false);
	nh->setParam("unpause", true);
}
