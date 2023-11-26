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

#include <mujoco_ros_mocap/mocap_plugin.h>
#include "mujoco_env_fixture.h"

#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/plugin_utils.h>
#include <string>

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_ros_mocap_test");
	return RUN_ALL_TESTS();
}

class MocapPluginFixture : public ::testing::Test
{
protected:
	boost::shared_ptr<ros::NodeHandle> nh;
	mujoco_ros::mocap::MocapPlugin *mocap_plugin;
	MujocoEnvTestWrapper *env_ptr;

	void SetUp() override
	{
		nh.reset(new ros::NodeHandle("~"));
		nh->setParam("unpause", false);
		nh->setParam("no_x", true);
		nh->setParam("use_sim_time", true);

		env_ptr              = new MujocoEnvTestWrapper();
		std::string xml_path = ros::package::getPath("mujoco_ros_mocap") + "/assets/mocap_world.xml";
		env_ptr->startWithXML(xml_path);

		float seconds = 0;
		while (env_ptr->getOperationalStatus() != 0 && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";
		auto &plugins = env_ptr->getPlugins();
		for (const auto &p : plugins) {
			mocap_plugin = dynamic_cast<mujoco_ros::mocap::MocapPlugin *>(p.get());
			if (mocap_plugin != nullptr) {
				break;
			}
		}
	}

	void TearDown() override
	{
		mocap_plugin = nullptr;
		env_ptr->shutdown();
		delete env_ptr;
	}
};

TEST_F(MocapPluginFixture, PluginExists)
{
	EXPECT_TRUE(mocap_plugin != nullptr);
}

TEST_F(MocapPluginFixture, ControlCallback)
{
	EXPECT_TRUE(env_ptr->step());
}
