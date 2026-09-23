/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2026, Neura Robotics
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
 *   * Neither the name of Neura Robotics nor the names of its
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

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <mujoco_ros/ros_one/plugin_utils.hpp>

namespace {

class LifetimeTestPlugin final : public mujoco_ros::MujocoPlugin
{
public:
	bool Load(const mjModel *, mjData *) override
	{
		type_ = "mujoco_ros/TypeChangedDuringLoad";
		return true;
	}
	void Reset() override {}
};

class RosInitializationEnvironment final : public ::testing::Environment
{
public:
	void SetUp() override
	{
		int argc          = 1;
		char executable[] = "ros1_plugin_adapter_test";
		char *argv[]      = { executable, nullptr };
		ros::init(argc, argv, executable, ros::init_options::NoSigintHandler);
	}

	void TearDown() override
	{
		if (ros::isStarted()) {
			ros::shutdown();
		}
	}
};

} // namespace

testing::Environment *const ros_initialization_environment =
    testing::AddGlobalTestEnvironment(new RosInitializationEnvironment);

TEST(RosPluginAdapter, TypeRemainsStableAcrossPluginLoad)
{
	constexpr char kConfiguredType[] = "mujoco_ros/LifetimeTestPlugin";
	constexpr char kTypeAfterLoad[]  = "mujoco_ros/TypeChangedDuringLoad";

	XmlRpc::XmlRpcValue config;
	config["type"] = kConfiguredType;
	auto plugin    = std::make_unique<LifetimeTestPlugin>();
	plugin->Init(config, "~", nullptr);
	mujoco_ros::plugin_utils::RosPluginAdapter adapter(std::move(plugin));

	std::string error;
	ASSERT_TRUE(adapter.Load(nullptr, nullptr, error));
	EXPECT_TRUE(error.empty());

	EXPECT_EQ(adapter.Statistics().type, kTypeAfterLoad);
	EXPECT_EQ(adapter.Type(), kConfiguredType);
}
