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

#include <mujoco_ros/ros_version.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <boost/function.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#else // MJR_ROS_VERSION == ROS_2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>
#include "test_plugin/test_plugin.hpp"

#include <mujoco_ros/render_backend.hpp>
#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/offscreen_camera.hpp>
#include <mujoco_ros/util.hpp>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#if MJR_ROS_VERSION == ROS_1
class LifetimeTestPlugin final : public mujoco_ros::MujocoPlugin
{
public:
	bool Load(const mjModel *, mjData *) override { return true; }
	void Reset() override {}
};
#endif

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
#if MJR_ROS_VERSION == ROS_1
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
#else // MJR_ROS_VERSION == ROS_2
	rclcpp::init(argc, argv);
	int ret = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return ret;
#endif
}

template <typename Func>
decltype(auto) WithTestPlugin(MujocoEnvTestWrapper *env, Func &&func)
{
#if MJR_ROS_VERSION == ROS_1
	return env->WithBackendPlugin<TestPlugin>("mujoco_ros/TestPlugin", "mujoco_ros/TestPlugin",
	                                          std::forward<Func>(func));
#else
	return env->WithBackendPlugin<TestPlugin>("test_plugin", "mujoco_ros/TestPlugin", std::forward<Func>(func));
#endif
}

class LoadedPluginFixture : public ::testing::Test
{
protected:
	std::unique_ptr<testing::TestNodeHandle> nh;
	MujocoEnvTestWrapper *env_ptr;

	void SetUp() override
	{
		nh = std::make_unique<testing::TestNodeHandle>("~");
		nh->setParam("unpause", false);
		nh->setParam("no_render", true);
		nh->setParam("headless", true);
		nh->setParam("use_sim_time", true);

		env_ptr              = new MujocoEnvTestWrapper(nh.get());
		std::string xml_path = testing::get_test_model_path("empty_world.xml");
		env_ptr->StartWithXML(xml_path);

		float seconds = 0;
		while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";

		ASSERT_NO_THROW(WithTestPlugin(env_ptr, [](TestPlugin *) {}));
	}

	void TearDown() override
	{
		// cleanup all parameters
		nh->deleteParam(nh->getNamespace());
		env_ptr->shutdown();
		delete env_ptr;
	}
};

#if MJR_ROS_VERSION == ROS_1
TEST(RosPluginAdapter, TypeReferenceRemainsStableAfterGetterTemporaryExpires)
{
	XmlRpc::XmlRpcValue config;
	config["type"] = "mujoco_ros/LifetimeTestPlugin";
	auto plugin    = std::make_unique<LifetimeTestPlugin>();
	plugin->Init(config, "~", nullptr);
	mujoco_ros::plugin_utils::RosPluginAdapter adapter(std::move(plugin));

	const std::string &type = adapter.Type();
	std::string allocation_churn(4096, 'x');
	EXPECT_EQ(type, "mujoco_ros/LifetimeTestPlugin");
	EXPECT_EQ(&type, &adapter.Type());
	EXPECT_FALSE(allocation_churn.empty());
}
#endif

TEST_F(LoadedPluginFixture, ControlCallback)
{
	// mjcb_control is called in mj_forward, which is also called when paused
	// we can't guarantee that the control callback has not been called yet, if the test's
	// timing is too slow
	// EXPECT_FALSE(test_plugin->ran_control_cb.load());
	EXPECT_TRUE(env_ptr->step());
	EXPECT_TRUE(WithTestPlugin(env_ptr, [](TestPlugin *plugin) { return plugin->ran_control_cb.load(); }));
}

TEST_F(LoadedPluginFixture, PassiveCallback)
{
	// mjcb_passive is called in mj_forward, which is also called when paused
	// we can't guarantee that the passive callback has not been called yet, if the test's
	// timing is too slow
	// EXPECT_FALSE(test_plugin->ran_passive_cb.load());
	EXPECT_TRUE(env_ptr->step());
	EXPECT_TRUE(WithTestPlugin(env_ptr, [](TestPlugin *plugin) { return plugin->ran_passive_cb.load(); }));
}

#if MJR_ROS_VERSION == ROS_1
#if OFFSCREEN_RENDER_BACKEND == EGL_BACKEND || OFFSCREEN_RENDER_BACKEND == OSMESA_BACKEND
TEST_F(BaseEnvFixture, RenderCallback)
{
	nh->setParam("no_render", false);
	nh->setParam("unpause", false);
	nh->setParam("headless", true);
	nh->setParam("cam_config/test_cam/width", 7);
	nh->setParam("cam_config/test_cam/height", 4);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>(nh.get());

	// NOP subscriber to trigger render callback
	ros::Subscriber rgb_sub = nh->subscribe<sensor_msgs::Image>("cameras/test_cam/rgb/image_raw", 1,
	                                                            [&](const sensor_msgs::Image::ConstPtr & /*msg*/) {});

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->step());

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	EXPECT_TRUE(offscreen->cams.size() == 1);

	// wait for render callback to be called
	float seconds = 0;
	while (!WithTestPlugin(env_ptr.get(), [](TestPlugin *plugin) { return plugin->ran_render_cb.load(); }) &&
	       seconds < 1.) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 1) << "Render callback was not called within 1 second!";

	env_ptr->shutdown();
}
#endif

#if OFFSCREEN_RENDER_BACKEND == NO_BACKEND
TEST_F(BaseEnvFixture, RenderCallback_NoRender)
{
	nh->setParam("no_render", false);
	nh->setParam("unpause", false);
	nh->setParam("headless", true);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>(nh.get());

	// NOP subscriber to trigger render callback
	ros::Subscriber rgb_sub = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&](const sensor_msgs::Image::ConstPtr & /*msg*/) { ROS_ERROR("Got image!"); });

	env_ptr->StartWithXML(xml_path);
	EXPECT_TRUE(env_ptr->step(5));

	// wait for render callback to be called
	float seconds = 0;
	while (!WithTestPlugin(env_ptr.get(), [](TestPlugin *plugin) { return plugin->ran_render_cb.load(); }) &&
	       seconds < .1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_FALSE(WithTestPlugin(env_ptr.get(), [](TestPlugin *plugin) { return plugin->ran_render_cb.load(); }))
	    << "Render callback was called!";

	env_ptr->shutdown();
}
#endif
#endif // MJR_ROS_VERSION == ROS_1

TEST_F(BaseEnvFixture, NoOffscreenDemandSkipsPluginRenderCallback)
{
	nh->setParam("no_render", false);
	nh->setParam("unpause", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);

	env_ptr = std::make_unique<MujocoEnvTestWrapper>(nh.get());
	env_ptr->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(env_ptr->GetOperationalStatus(), 0);
	auto *offscreen = env_ptr->getCameraPublicationTransport();
	if (!offscreen->ActiveRenderCore()) {
		GTEST_SKIP() << "offscreen RenderCore is unavailable for this backend";
	}
	ASSERT_EQ(offscreen->cams.size(), 1U);

	ASSERT_TRUE(WithTestPlugin(env_ptr.get(), [](TestPlugin *plugin) {
		plugin->ran_render_cb.store(false);
		return true;
	}));
	ASSERT_TRUE(env_ptr->step());
	EXPECT_FALSE(WithTestPlugin(env_ptr.get(), [](TestPlugin *plugin) { return plugin->ran_render_cb.load(); }));
}

TEST_F(BaseEnvFixture, DueRosDemandRunsPluginRenderCallbackAndPublishesCapture)
{
	nh->setParam("no_render", false);
	nh->setParam("unpause", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);

#if MJR_ROS_VERSION == ROS_1
	boost::function<void(const sensor_msgs::Image::ConstPtr &)> image_callback =
	    [](const sensor_msgs::Image::ConstPtr &) {};
	auto image_subscriber = nh->subscribe<sensor_msgs::Image>("cameras/test_cam/rgb/image_raw", 1, image_callback);
#endif
	env_ptr = std::make_unique<MujocoEnvTestWrapper>(nh.get());
#if MJR_ROS_VERSION == ROS_2
	auto observer_node = std::make_shared<rclcpp::Node>("render_demand_observer");
	env_ptr->AddNodeToExecutor(observer_node->get_node_base_interface());
	auto image_subscriber = observer_node->create_subscription<sensor_msgs::msg::Image>(
	    env_ptr->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw", rclcpp::SensorDataQoS(),
	    [](const sensor_msgs::msg::Image::ConstSharedPtr) {});
#endif
	env_ptr->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(env_ptr->GetOperationalStatus(), 0);
	auto *offscreen = env_ptr->getCameraPublicationTransport();
	if (!offscreen->ActiveRenderCore()) {
		GTEST_SKIP() << "offscreen RenderCore is unavailable for this backend";
	}
	ASSERT_EQ(offscreen->cams.size(), 1U);

	{
		std::lock_guard<MujocoEnvMutex> lock(*env_ptr->getMutexPtr());
		env_ptr->getModelPtr()->opt.timestep = 2.094;
	}
	const auto subscriber_deadline = Clock::now() + std::chrono::seconds(2);
	while (offscreen->cams.front()->rgb_pub_.getNumSubscribers() == 0 && Clock::now() < subscriber_deadline) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	ASSERT_EQ(offscreen->cams.front()->rgb_pub_.getNumSubscribers(), 1U);
	const auto capture_id_before = offscreen->cams.front()->last_published_capture_id();
	ASSERT_EQ(capture_id_before, 0U);
	ASSERT_TRUE(WithTestPlugin(env_ptr.get(), [](TestPlugin *plugin) {
		plugin->ran_render_cb.store(false);
		return true;
	}));
	ASSERT_TRUE(env_ptr->step());
	EXPECT_TRUE(WithTestPlugin(env_ptr.get(), [](TestPlugin *plugin) { return plugin->ran_render_cb.load(); }));
	EXPECT_EQ(offscreen->cams.front()->last_published_capture_id(), capture_id_before + 1U);
	const auto expected_nanoseconds = util::simTimeToNanoseconds(env_ptr->getDataPtr()->time);
	mjtNum simulation_time_seconds  = static_cast<mjtNum>(expected_nanoseconds) / 1e9;
	const auto float_roundtrip_ns   = static_cast<std::int64_t>(simulation_time_seconds * 1e9);
	ASSERT_NE(float_roundtrip_ns, expected_nanoseconds)
	    << "test timestep must not round-trip through mjtNum without nanosecond loss";
#if MJR_ROS_VERSION == ROS_1
	const auto published_ns = static_cast<std::int64_t>(offscreen->cams.front()->last_pub_.toNSec());
	EXPECT_EQ(published_ns, util::toRosTime(expected_nanoseconds).toNSec());
	EXPECT_NE(published_ns, float_roundtrip_ns);
#else
	const auto published_ns = offscreen->cams.front()->last_pub_.nanoseconds();
	EXPECT_EQ(published_ns, util::toRosTime(expected_nanoseconds).nanoseconds());
	EXPECT_NE(published_ns, float_roundtrip_ns);
#endif
}

TEST_F(LoadedPluginFixture, LastCallback)
{
	EXPECT_FALSE(WithTestPlugin(env_ptr, [](TestPlugin *plugin) { return plugin->ran_last_cb.load(); }));
	EXPECT_TRUE(env_ptr->step());
	EXPECT_TRUE(WithTestPlugin(env_ptr, [](TestPlugin *plugin) { return plugin->ran_last_cb.load(); }));
}

TEST_F(LoadedPluginFixture, OnGeomChangedCallback)
{
	EXPECT_FALSE(WithTestPlugin(env_ptr, [](TestPlugin *plugin) { return plugin->ran_on_geom_changed_cb.load(); }));
	env_ptr->NotifyGeomChange();
	EXPECT_TRUE(WithTestPlugin(env_ptr, [](TestPlugin *plugin) { return plugin->ran_on_geom_changed_cb.load(); }));
}

TEST_F(LoadedPluginFixture, ReloadObserverThrowQuiescesPluginHostBeforeModelCleanup)
{
	std::atomic_bool observer_called{ false };
	env_ptr->SetReloadObserver([&](MujocoEnv::ReloadPhase phase) {
		if (phase == MujocoEnv::ReloadPhase::kRenderReconfigureStarted) {
			observer_called.store(true);
			throw std::runtime_error("deterministic plugin reload observer failure");
		}
	});

	env_ptr->load_queued_model();
	ASSERT_TRUE(env_ptr->WaitForOperationalStatusIdle(std::chrono::seconds(2)))
	    << "observer-throw reload did not return to an idle lifecycle";
	EXPECT_TRUE(observer_called.load());
	EXPECT_TRUE(env_ptr->isEventRunning()) << "observer exception terminated the event loop";
	EXPECT_EQ(env_ptr->GetNumCBReadyPlugins(), 0) << "PluginHost retained adapters after model/data cleanup";
	EXPECT_THROW(WithTestPlugin(env_ptr, [](TestPlugin *) {}), std::runtime_error)
	    << "an old Plugin Generation remained active after reload failure";

	env_ptr->requestShutdown();
	env_ptr->WaitForEventsJoin();
	EXPECT_FALSE(env_ptr->isEventRunning());
}

TEST_F(BaseEnvFixture, LoadPlugin)
{
	nh->setParam("no_render", true);
	nh->setParam("unpause", false);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>(nh.get());

	env_ptr->StartWithXML(xml_path);

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";
	EXPECT_EQ(env_ptr->GetPluginStats().size(), 1) << "Env should have 1 plugin registered!";
	EXPECT_EQ(env_ptr->GetNumCBReadyPlugins(), 1) << "Env should have 1 plugin loaded!";

	env_ptr->shutdown();
}

TEST_F(LoadedPluginFixture, ResetPlugin)
{
	env_ptr->RequestReset();
	float seconds = 0;
	while (!WithTestPlugin(env_ptr, [](TestPlugin *plugin) { return plugin->ran_reset.load(); }) && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	env_ptr->step(10);

	EXPECT_LT(seconds, 2) << "Env reset ran into 2 seconds timeout!";
	EXPECT_TRUE(WithTestPlugin(env_ptr, [](TestPlugin *plugin) { return plugin->ran_reset.load(); }))
	    << "Dummy plugin reset was not called!";
}

TEST_F(LoadedPluginFixture, GetConfigToplevel)
{
	EXPECT_TRUE(WithTestPlugin(env_ptr, [](TestPlugin *plugin) { return plugin->got_config_param.load(); }));
}

TEST_F(LoadedPluginFixture, GetConfigArray)
{
	EXPECT_TRUE(WithTestPlugin(env_ptr, [](TestPlugin *plugin) { return plugin->got_lvl1_nested_array.load(); }));
	EXPECT_TRUE(WithTestPlugin(env_ptr, [](TestPlugin *plugin) { return plugin->got_lvl2_nested_array.load(); }));
}

TEST_F(LoadedPluginFixture, GetConfigStruct)
{
	EXPECT_TRUE(WithTestPlugin(env_ptr, [](TestPlugin *plugin) { return plugin->got_lvl1_nested_struct.load(); }));
	EXPECT_TRUE(WithTestPlugin(env_ptr, [](TestPlugin *plugin) { return plugin->got_lvl2_nested_struct.load(); }));
}

TEST_F(BaseEnvFixture, FailedLoad)
{
	nh->setParam("unpause", false);
	nh->setParam("should_fail", true);

	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>(nh.get());

	env_ptr->StartWithXML(xml_path);

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";

	EXPECT_EQ(env_ptr->GetPluginStats().size(), 1) << "Env should have 1 plugin registered!";
	EXPECT_EQ(env_ptr->GetNumCBReadyPlugins(), 0) << "Env should have 0 plugins loaded!";
	EXPECT_EQ(env_ptr->GetPluginStats().front().type, "mujoco_ros/TestPlugin");

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, FailedLoadRecoverReload)
{
	nh->setParam("should_fail", true);

	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>(nh.get());

	env_ptr->StartWithXML(xml_path);

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";

	EXPECT_EQ(env_ptr->GetPluginStats().size(), 1) << "Env should have 1 plugin registered!";
	EXPECT_EQ(env_ptr->GetNumCBReadyPlugins(), 0) << "Env should have 0 plugins loaded!";

	{
		nh->setParam("should_fail", false);
#if MJR_ROS_VERSION == ROS_2
		env_ptr->set_parameters({ rclcpp::Parameter("should_fail", false) });
#endif

		env_ptr->load_queued_model();
		float seconds = 0;
		while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		EXPECT_LT(seconds, 2) << "Env reset ran into 2 seconds timeout!";
		EXPECT_EQ(env_ptr->GetPluginStats().size(), 1) << "Env should have 1 plugin registered!";
		EXPECT_EQ(env_ptr->GetNumCBReadyPlugins(), 1) << "Env should have 1 plugin loaded!";
	}

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, ReloadRejectsOldPluginGenerationAndActivatesOnlyNewCallbacks)
{
	nh->setParam("unpause", false);
	const std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr                    = std::make_unique<MujocoEnvTestWrapper>(nh.get());
	env_ptr->StartWithXML(xml_path);

	PluginGeneration old_generation;
	env_ptr->WithPluginAccess(
	    [&old_generation](const ScopedPluginAccess &access) { old_generation = access.Generation(); });

	env_ptr->load_queued_model();
	EXPECT_THROW(env_ptr->WithPluginAccess(old_generation, [](const ScopedPluginAccess &) {}), std::runtime_error);
	EXPECT_TRUE(WithTestPlugin(env_ptr.get(), [](TestPlugin *plugin) {
		plugin->ran_control_cb.store(false);
		return true;
	}));
	env_ptr->step();
	EXPECT_TRUE(WithTestPlugin(env_ptr.get(), [](TestPlugin *plugin) { return plugin->ran_control_cb.load(); }));

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, ReloadHasDeterministicGenerationBoundaryAndCallbackOrder)
{
	nh->setParam("unpause", false);
	const std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr                    = std::make_unique<MujocoEnvTestWrapper>(nh.get());
	env_ptr->StartWithXML(xml_path);

	ASSERT_TRUE(env_ptr->step());
	ASSERT_TRUE(WithTestPlugin(env_ptr.get(), [](TestPlugin *plugin) { return plugin->ran_control_cb.load(); }));

	PluginGeneration old_generation;
	env_ptr->WithPluginAccess(
	    [&old_generation](const ScopedPluginAccess &access) { old_generation = access.Generation(); });
	const int previous_loads        = TestPlugin::load_count.load();
	const int previous_destructions = TestPlugin::destruction_count.load();
	std::mutex mutex;
	std::condition_variable condition;
	std::vector<std::string> events{ "old_callback" };
	int ready_at_forward = -1;
	int ready_after_load = -1;
	PluginGeneration new_generation;
	bool old_generation_rejected = false;

	env_ptr->SetReloadObserver([&](MujocoEnv::ReloadPhase phase) {
		std::lock_guard<std::mutex> lock(mutex);
		switch (phase) {
			case MujocoEnv::ReloadPhase::kRenderQuiescenceStarted:
				break;
			case MujocoEnv::ReloadPhase::kRenderTurnsIdle:
				break;
			case MujocoEnv::ReloadPhase::kRenderReconfigureStarted:
				break;
			case MujocoEnv::ReloadPhase::kOldGenerationQuiesced:
				ASSERT_GT(TestPlugin::destruction_count.load(), previous_destructions);
				EXPECT_THROW(env_ptr->WithPluginAccess(old_generation, [](const ScopedPluginAccess &) {}),
				             std::runtime_error);
				old_generation_rejected = true;
				events.emplace_back("old_destroyed");
				break;
			case MujocoEnv::ReloadPhase::kModelSwapped:
				events.emplace_back("model_swapped");
				break;
			case MujocoEnv::ReloadPhase::kForwarded:
				ready_at_forward = env_ptr->GetNumCBReadyPlugins();
				events.emplace_back("forwarded");
				break;
			case MujocoEnv::ReloadPhase::kNewGenerationLoaded:
				ready_after_load = env_ptr->GetNumCBReadyPlugins();
				env_ptr->WithPluginAccess(
				    [&new_generation](const ScopedPluginAccess &access) { new_generation = access.Generation(); });
				events.emplace_back("new_loaded");
				break;
			case MujocoEnv::ReloadPhase::kReloadFailed:
				break;
		}
		condition.notify_all();
	});

	env_ptr->requestLoad(2);
	std::unique_lock<std::mutex> lock(mutex);
	ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5),
	                               [&]() { return !events.empty() && events.back() == "new_loaded"; }));
	lock.unlock();
	env_ptr->SetReloadObserver({});

	ASSERT_GT(TestPlugin::load_count.load(), previous_loads);
	ASSERT_TRUE(old_generation_rejected);
	ASSERT_NE(new_generation, old_generation);
	ASSERT_EQ(ready_at_forward, 0);
	ASSERT_EQ(ready_after_load, 1);
	ASSERT_EQ(events,
	          (std::vector<std::string>{ "old_callback", "old_destroyed", "model_swapped", "forwarded", "new_loaded" }));

	ASSERT_TRUE(WithTestPlugin(env_ptr.get(), [](TestPlugin *plugin) {
		plugin->ran_control_cb.store(false);
		return true;
	}));
	ASSERT_TRUE(env_ptr->step());
	EXPECT_TRUE(WithTestPlugin(env_ptr.get(), [](TestPlugin *plugin) { return plugin->ran_control_cb.load(); }));

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, FailedLoadReset)
{
	nh->setParam("should_fail", true);
	nh->setParam("unpause", false);

	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>(nh.get());

	env_ptr->StartWithXML(xml_path);

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Env loading ran into 2 seconds timeout!";

	EXPECT_EQ(env_ptr->GetPluginStats().size(), 1) << "Env should have 1 plugin registered!";
	EXPECT_EQ(env_ptr->GetNumCBReadyPlugins(), 0) << "Env should have 0 plugins loaded!";

	{
		env_ptr->RequestReset();
		float seconds = 0;
		while (env_ptr->GetControlSnapshot().reset_requested && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		env_ptr->step(10);

		EXPECT_LT(seconds, 2) << "Env reset ran into 2 seconds timeout!";
		EXPECT_EQ(env_ptr->GetNumCBReadyPlugins(), 0) << "Failed plugin must remain inactive";
	}

	env_ptr->shutdown();
}

TEST_F(LoadedPluginFixture, PluginStats_InitialPaused)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Env should be paused!";

// TODO: once this service is added to ROS 2, refactor to hybrid test fixtures, too
// Tests the GetPluginStats service
#if MJR_ROS_VERSION == ROS_1
	mujoco_ros_msgs::GetPluginStats srv;
	EXPECT_TRUE(ros::service::exists(env_ptr->GetHandleNamespace() + "/get_plugin_stats", true))
	    << "Plugin stats service should exist!";
	EXPECT_TRUE(ros::service::call(env_ptr->GetHandleNamespace() + "/get_plugin_stats", srv))
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
#endif

	// Tests the non-service version of GetPluginStats
	std::vector<std::string> plugin_names, types;
	std::vector<double> load_times, reset_times, ema_steptimes_control, ema_steptimes_passive, ema_steptimes_render,
	    ema_steptimes_last_stage;
	env_ptr->GetPluginStats(plugin_names, types, load_times, reset_times, ema_steptimes_control, ema_steptimes_passive,
	                        ema_steptimes_render, ema_steptimes_last_stage);
	EXPECT_EQ(plugin_names.size(), 1) << "Should have 1 plugin stats!";
	EXPECT_EQ(types[0], "mujoco_ros/TestPlugin") << "Should be TestPlugin!";
	EXPECT_GT(load_times[0], -1) << "Load time should be set!";
	EXPECT_EQ(reset_times[0], -1) << "Reset time should be unset!";
	// passive and control are also run when paused
	EXPECT_NEAR(ema_steptimes_control[0], 0, 1e-7) << "Control time should be unset!";
	EXPECT_NEAR(ema_steptimes_passive[0], 0, 1e-7) << "Passive time should be unset!";
	EXPECT_NEAR(ema_steptimes_render[0], 0, 1e-8) << "Render time should be unset!";
	EXPECT_NEAR(ema_steptimes_last_stage[0], 0, 1e-8) << "Last stage time should be unset!";
}

TEST_F(LoadedPluginFixture, PluginStats_SetTimesOnStep)
{
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Env should be paused!";

	env_ptr->step(1);
	// sleep for a bit to ensure the plugin callbacks have been called
	std::this_thread::sleep_for(std::chrono::milliseconds(1));

// TODO: once this service is added to ROS 2, refactor to hybrid test fixtures, too
// Tests the GetPluginStats service
#if MJR_ROS_VERSION == ROS_1
	mujoco_ros_msgs::GetPluginStats srv;
	EXPECT_TRUE(ros::service::exists(env_ptr->GetHandleNamespace() + "/get_plugin_stats", true))
	    << "Plugin stats service should exist!";
	EXPECT_TRUE(ros::service::call(env_ptr->GetHandleNamespace() + "/get_plugin_stats", srv))
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
#endif

	// Tests the non-service version of GetPluginStats
	std::vector<std::string> plugin_names, types;
	std::vector<double> load_times, reset_times, ema_steptimes_control, ema_steptimes_passive, ema_steptimes_render,
	    ema_steptimes_last_stage;
	env_ptr->GetPluginStats(plugin_names, types, load_times, reset_times, ema_steptimes_control, ema_steptimes_passive,
	                        ema_steptimes_render, ema_steptimes_last_stage);
	EXPECT_EQ(plugin_names.size(), 1) << "Should have 1 plugin stats!";
	EXPECT_EQ(types[0], "mujoco_ros/TestPlugin") << "Should be TestPlugin!";
	EXPECT_GT(load_times[0], -1) << "Load time should be set!";
	EXPECT_EQ(reset_times[0], -1) << "Reset time should be unset!";
	EXPECT_GT(ema_steptimes_control[0], -1) << "Control time should be unset!";
	EXPECT_GT(ema_steptimes_passive[0], -1) << "Passive time should be unset!";
	// EXPECT_GT(ema_steptimes_render[0], -1) << "Render time should be unset!"; // TODO: add when rendering is enabled
	// in tests
	EXPECT_GT(ema_steptimes_last_stage[0], -1) << "Last stage time should be unset!";
}

TEST_F(LoadedPluginFixture, PluginStats_ResetTimeOnReset)
{
	env_ptr->RequestReset();

	float seconds = 0;
	while (env_ptr->GetPluginStats().front().reset_time <= -1 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Plugin reset stats were not recorded before timeout!";

// TODO: once this service is added to ROS 2, refactor to hybrid test fixtures, too
// Tests the GetPluginStats service
#if MJR_ROS_VERSION == ROS_1
	mujoco_ros_msgs::GetPluginStats srv;
	EXPECT_TRUE(ros::service::exists(env_ptr->GetHandleNamespace() + "/get_plugin_stats", true))
	    << "Plugin stats service should exist!";
	EXPECT_TRUE(ros::service::call(env_ptr->GetHandleNamespace() + "/get_plugin_stats", srv))
	    << "Get plugin stats service call failed!";

	EXPECT_EQ(srv.response.stats.size(), 1) << "Should have 1 plugin stats!";
	EXPECT_EQ(srv.response.stats[0].plugin_type, "mujoco_ros/TestPlugin") << "Should be TestPlugin!";
	EXPECT_GT(srv.response.stats[0].reset_time, -1) << "Reset time should be unset!";
#endif

	// Tests the non-service version of GetPluginStats
	std::vector<std::string> plugin_names, types;
	std::vector<double> load_times, reset_times, ema_steptimes_control, ema_steptimes_passive, ema_steptimes_render,
	    ema_steptimes_last_stage;
	env_ptr->GetPluginStats(plugin_names, types, load_times, reset_times, ema_steptimes_control, ema_steptimes_passive,
	                        ema_steptimes_render, ema_steptimes_last_stage);
	EXPECT_EQ(plugin_names.size(), 1) << "Should have 1 plugin stats!";
	EXPECT_EQ(types[0], "mujoco_ros/TestPlugin") << "Should be TestPlugin!";
	EXPECT_GT(reset_times[0], -1) << "Reset time should be unset!";
}
