/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
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
 *   * Neither the name of Bielefeld University nor Neura Robotics nor the names of their
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
#include <mujoco_ros/util.hpp>
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

#include <algorithm>
#include <chrono>
#include <cmath>
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

std::string get_laser_model_path(const char *asset = "laser_world.xml")
{
#if MJR_ROS_VERSION == ROS_1
	return ros::package::getPath("mujoco_ros_laser") + "/assets/" + asset;
#else
	return ament_index_cpp::get_package_share_directory("mujoco_ros_laser") + "/assets/" + asset;
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

template <typename StampT>
int64_t stamp_to_ns(const StampT &stamp)
{
#if MJR_ROS_VERSION == ROS_1
	return static_cast<int64_t>(stamp.toNSec());
#else
	return rclcpp::Time(stamp).nanoseconds();
#endif
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

	virtual void ConfigureExtraParams() {}
	virtual std::string ModelPath() const { return get_laser_model_path(); }

	void SetUp() override
	{
		nh = std::make_unique<testing::TestNodeHandle>("~");
		nh->setParam("unpause", false);
		nh->setParam("no_render", true);
#if MJR_ROS_VERSION == ROS_1
		nh->setParam("use_sim_time", true);
#endif
		nh->setParam("num_mj_threads", GetParam());
		configure_plugin_params(*nh);
		ConfigureExtraParams();

		env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

		const std::string xml_path = ModelPath();
		env_ptr->StartWithXML(xml_path);

		float seconds = 0;
		while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001f;
		}
		ASSERT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

		ASSERT_EQ(env_ptr->GetPluginStats().size(), 1u);
		ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	}

	void TearDown() override
	{
		if (env_ptr != nullptr) {
			env_ptr->shutdown();
			// Destroy the environment before clearing its parameters.  shutdown()
			// joins the simulation threads, but the ROS 2 executor and MuJoCo
			// threadpool are owned by the environment until its destructor runs.
			// Keeping it alive while removing parameters lets a late callback from
			// the previous fixture race with the next parameterized instance.
			env_ptr.reset();
		}
		if (nh != nullptr) {
			clear_test_params(*nh);
		}
	}
};

TEST_P(LoadedPluginFixture, PluginLoaded)
{
	EXPECT_EQ(env_ptr->GetPluginStats().front().type, "mujoco_ros_laser/LaserPlugin") << "Plugin loading failed!";
}

TEST_P(LoadedPluginFixture, ScanTopicCreated)
{
	const auto topics = wait_for_scan_topic(env_ptr.get());
	EXPECT_TRUE(testing::has_topic(topics, scan_topic_name(env_ptr.get())))
	    << "Laser scan topic should have been generated";
}

// The engine threadpool (mju_threadpool) follows num_mj_threads: enabled iff the effective thread count is >= 2. The
// laser plugin does not use it for ray casting; it always casts serially.
TEST_P(LoadedPluginFixture, ThreadpoolModeMatchesConfiguration)
{
	const bool has_threadpool           = static_cast<bool>(env_ptr->getDataPtr()->threadpool);
	const unsigned int hardware_threads = std::thread::hardware_concurrency();
	const int available_threads         = hardware_threads > 1 ? static_cast<int>(hardware_threads - 1) : 0;
	const int effective_threads         = std::min(GetParam(), available_threads);
	EXPECT_EQ(has_threadpool, effective_threads >= 2)
	    << "Unexpected MuJoCo threadpool state for requested " << GetParam() << " and effective " << effective_threads
	    << " threads";
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

	// Stamp is sim time (data->time): never ahead of the sim, and within one step batch of it (wait_for_scan_message
	// stops stepping once a scan arrives).
	const auto stamp_ns = stamp_to_ns(msg->header.stamp);
	const auto time_ns  = mujoco_ros::util::simTimeToNanoseconds(env_ptr->getDataPtr()->time);
	const auto step_ns  = mujoco_ros::util::simTimeToNanoseconds(env_ptr->getModelPtr()->opt.timestep);
	EXPECT_EQ(stamp_ns % step_ns, 0);
	EXPECT_LE(stamp_ns, time_ns);
	EXPECT_GE(stamp_ns, time_ns - 30 * step_ns) << "Stamp lags data->time by more than one 25-step batch";
}

TEST_P(LoadedPluginFixture, ScanStampRestartsAfterReset)
{
	std::atomic<int64_t> last_stamp_ns{ -1 };
	auto sub = subscribe_to<LaserScanMsg>(
	    env_ptr.get(), nh.get(), scan_topic_name(env_ptr.get()), 10,
	    [&last_stamp_ns](const MessageConstPtr<LaserScanMsg> &msg) { last_stamp_ns = stamp_to_ns(msg->header.stamp); });

	env_ptr->togglePaused(true);
	const auto deadline_run = std::chrono::steady_clock::now() + std::chrono::seconds(5);
	while (last_stamp_ns.load() < 200'000'000 && std::chrono::steady_clock::now() < deadline_run) {
		env_ptr->step(25);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	const int64_t before_reset_ns = last_stamp_ns.load();
	ASSERT_GE(before_reset_ns, 200'000'000) << "Sim never advanced to 0.2 s before the reset";

	env_ptr->RequestReset();
	const auto deadline_reset = std::chrono::steady_clock::now() + std::chrono::seconds(3);
	bool restarted            = false;
	while (!restarted && std::chrono::steady_clock::now() < deadline_reset) {
		env_ptr->step(25);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		const int64_t latest = last_stamp_ns.load();
		restarted            = latest >= 0 && latest < before_reset_ns;
	}
	EXPECT_TRUE(restarted) << "No scan with a stamp below " << before_reset_ns << " ns after reset (last "
	                       << last_stamp_ns.load() << " ns): rate gate muted scans or stamp is not sim time";
}

#if MJR_ROS_VERSION == ROS_2
class NoiseFreeLaserFixture : public LoadedPluginFixture
{
protected:
	void ConfigureExtraParams() override { nh->setParam("MujocoPlugins.mujoco_ros_laser.scan.sensor_std", 0.0); }
};

// Published ranges must equal a serial per-ray mj_ray loop with the laser's ray set and arguments, i.e. the
// mj_multiRay hot path is output-identical to the former serial path (noise disabled).
TEST_P(NoiseFreeLaserFixture, RangesMatchSerialMjRay)
{
	const auto msg = wait_for_scan_message(env_ptr.get(), nh.get());
	ASSERT_TRUE(msg != nullptr) << "Could not get message on " << scan_topic_name(env_ptr.get());
	ASSERT_GT(msg->ranges.size(), 100u);

	const mjModel *m = env_ptr->getModelPtr();
	const mjData *d  = env_ptr->getDataPtr();
	const int site   = mj_name2id(m, mjOBJ_SITE, "laser_site");
	ASSERT_GE(site, 0);

	mjtByte groups[mjNGROUP];
	std::fill(std::begin(groups), std::end(groups), 1);
	groups[1] = 0;

	const int mocap_body = mj_name2id(m, mjOBJ_BODY, "mocap_obs");
	ASSERT_GE(mocap_body, 0);

	int hits = 0; // hits on anything but the mocap box, so obs0..2 stay exercised
	for (size_t i = 0; i < msg->ranges.size(); ++i) {
		const double angle    = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
		const mjtNum local[3] = { mju_cos(angle), mju_sin(angle), 0 };
		mjtNum dir[3];
		mju_mulMatVec3(dir, d->site_xmat + 9 * site, local);
		int geomid      = -1;
		mjtNum expected = mj_ray(m, d, d->site_xpos + 3 * site, dir, groups, 1, -1, &geomid, nullptr);
		hits += expected >= 0 && expected < msg->range_max && m->geom_bodyid[geomid] != mocap_body;
		if (expected < 0) {
			expected = msg->range_max;
		}
		expected = std::min<mjtNum>(std::max<mjtNum>(msg->range_min, expected), msg->range_max);
		// Boxes rest on the plane and may creep by micrometres between the scan step and this check.
		EXPECT_NEAR(msg->ranges[i], expected, 1e-4) << "ray " << i;
	}
	EXPECT_GT(hits, 5) << "Scan should hit the free obstacles in the laser world";
}

// A mocap box in the beam must register in the scan and the range must follow mocap_pos, and a static world geom
// must still be hit. Guards MuJoCo >= 3.12 ray semantics: flg_static=1 hits mocap, dynamic and static geoms, while
// flg_static=0 would drop the static wall (and the ground plane).
TEST_P(NoiseFreeLaserFixture, RangesFollowMocapObstacle)
{
	mjModel *m     = env_ptr->getModelPtr();
	mjData *d      = env_ptr->getDataPtr();
	const int site = mj_name2id(m, mjOBJ_SITE, "laser_site");
	const int body = mj_name2id(m, mjOBJ_BODY, "mocap_obs");
	const int wall = mj_name2id(m, mjOBJ_GEOM, "static_wall");
	ASSERT_GE(site, 0);
	ASSERT_GE(body, 0);
	ASSERT_GE(wall, 0);
	const int mocap_id = m->body_mocapid[body];
	ASSERT_GE(mocap_id, 0) << "mocap_obs must be a mocap body";
	ASSERT_EQ(m->body_weldid[m->geom_bodyid[wall]], 0) << "static_wall must be a world geom";

	const double half_extent = m->geom_size[3 * m->body_geomadr[body]];
	const double sx          = d->site_xpos[3 * site];
	const double sy          = d->site_xpos[3 * site + 1];

	// Ray index closest to the bearing of (x, y) from the laser site; the laser frame is axis-aligned with the world.
	auto ray_towards = [](const LaserScanMsg &msg, double x, double y, double &angle) {
		const size_t idx = static_cast<size_t>(std::lround((std::atan2(y, x) - msg.angle_min) / msg.angle_increment));
		angle            = msg.angle_min + static_cast<double>(idx) * msg.angle_increment;
		return idx;
	};

	// Range to the mocap box's -x face along the ray aimed at the box centre.
	auto mocap_range = [&](double box_x) -> double {
		d->mocap_pos[3 * mocap_id] = box_x;
		const auto msg             = wait_for_scan_message(env_ptr.get(), nh.get());
		EXPECT_TRUE(msg != nullptr);
		if (msg == nullptr) {
			return -1;
		}
		const double box_y = d->mocap_pos[3 * mocap_id + 1];
		double angle;
		const size_t idx      = ray_towards(*msg, box_x - sx, box_y - sy, angle);
		const double expected = (box_x - half_extent - sx) / std::cos(angle);
		EXPECT_NEAR(msg->ranges[idx], expected, 1e-3) << "mocap box at x=" << box_x;
		return msg->ranges[idx];
	};

	env_ptr->togglePaused(true);
	const double near_range = mocap_range(3.0);
	const double far_range  = mocap_range(4.0);
	// Both ranges are checked analytically above; the oblique ray makes the delta slightly differ from 1.0.
	EXPECT_GT(far_range - near_range, 0.9) << "Range must follow the moved mocap box";

	// Static wall at +y, not shadowed by the mocap box (at -y): ray hits its -y face.
	const auto msg = wait_for_scan_message(env_ptr.get(), nh.get());
	ASSERT_TRUE(msg != nullptr);
	const double *wc = d->geom_xpos + 3 * wall;
	double angle;
	const size_t idx    = ray_towards(*msg, wc[0] - sx, wc[1] - sy, angle);
	const double face_y = wc[1] - m->geom_size[3 * wall + 1];
	const double hit_x  = sx + (face_y - sy) / std::tan(angle);
	ASSERT_LT(std::abs(hit_x - wc[0]), m->geom_size[3 * wall]) << "test ray must land on the wall face";
	EXPECT_NEAR(msg->ranges[idx], (face_y - sy) / std::sin(angle), 1e-3) << "static world geom must be hit";
}

// Every ray of a scan must advance the shared noise generator. Rays with identical true range then get distinct noise.
TEST(LaserNoise, RaysDrawDistinctNoiseSamples)
{
	const LaserConfig config("scan", "scan", 0, false, 10.0, 0.1, 30.0, 0.01, 0.02, -0.1, 0.1, 0.05);
	ASSERT_TRUE(config.is_set);
	ASSERT_GE(config.nrays, 3u);

	std::mt19937 generator(42);
	std::normal_distribution<double> noise_dist(0.0, 1.0);
	const mjtNum pos[3]     = { 0, 0, 0 };
	const mjtNum dir[3]     = { 1, 0, 0 };
	const float rgba[4]     = { 0, 0, 1, 1 };
	const mjtNum true_range = 5.0;

	LaserScan scan;
	scan.ranges.resize(config.nrays);
	for (uint i = 0; i < config.nrays; ++i) {
		ProcessRay(generator, noise_dist, config, nullptr, pos, dir, true_range, rgba, scan, static_cast<int>(i));
	}

	std::vector<float> distinct(scan.ranges.begin(), scan.ranges.end());
	std::sort(distinct.begin(), distinct.end());
	distinct.erase(std::unique(distinct.begin(), distinct.end()), distinct.end());
	EXPECT_GT(distinct.size(), config.nrays / 2) << "Rays of one scan reuse the same noise draw";

	// Same seed, same scan: reproducible.
	std::mt19937 generator2(42);
	std::normal_distribution<double> noise_dist2(0.0, 1.0);
	LaserScan scan2;
	scan2.ranges.resize(config.nrays);
	for (uint i = 0; i < config.nrays; ++i) {
		ProcessRay(generator2, noise_dist2, config, nullptr, pos, dir, true_range, rgba, scan2, static_cast<int>(i));
	}
	EXPECT_EQ(scan.ranges, scan2.ranges);
}

// The laser's own mount body carries a geom in the beam path. The scan must ignore it and report the real target.
class SelfHitLaserFixture : public NoiseFreeLaserFixture
{
protected:
	std::string ModelPath() const override { return get_laser_model_path("laser_self_hit_world.xml"); }
};

TEST_P(SelfHitLaserFixture, MountBodyGeomIsNotHit)
{
	const auto msg = wait_for_scan_message(env_ptr.get(), nh.get());
	ASSERT_TRUE(msg != nullptr) << "Could not get message on " << scan_topic_name(env_ptr.get());

	const mjModel *m  = env_ptr->getModelPtr();
	const mjData *d   = env_ptr->getDataPtr();
	const int site    = mj_name2id(m, mjOBJ_SITE, "laser_site");
	const int target  = mj_name2id(m, mjOBJ_GEOM, "target_wall");
	const int bracket = mj_name2id(m, mjOBJ_GEOM, "mount_bracket");
	ASSERT_GE(site, 0);
	ASSERT_GE(target, 0);
	ASSERT_GE(bracket, 0);
	ASSERT_EQ(m->geom_bodyid[bracket], m->site_bodyid[site]) << "bracket must belong to the laser's mount body";

	// Ray along the site x axis: expected range is the distance to the target wall's -x face.
	const size_t idx      = static_cast<size_t>(std::lround(-msg->angle_min / msg->angle_increment));
	const double angle    = msg->angle_min + static_cast<double>(idx) * msg->angle_increment;
	const double face_x   = d->geom_xpos[3 * target] - m->geom_size[3 * target];
	const double expected = (face_x - d->site_xpos[3 * site]) / std::cos(angle);
	EXPECT_GT(expected, 2.0);
	EXPECT_NEAR(msg->ranges[idx], expected, 1e-3) << "Ray reports the mount body's own bracket instead of the target";
}

INSTANTIATE_TEST_SUITE_P(LaserSelfHit, SelfHitLaserFixture, ::testing::Values(1),
                         [](const ::testing::TestParamInfo<int> &) { return "SingleThreaded"; });

INSTANTIATE_TEST_SUITE_P(LaserRanges, NoiseFreeLaserFixture, ::testing::Values(1, 2),
                         [](const ::testing::TestParamInfo<int> &info) {
	                         return info.param == 1 ? "SingleThreaded" : "MultiThreaded";
                         });
#endif

INSTANTIATE_TEST_SUITE_P(LaserThreading, LoadedPluginFixture, ::testing::Values(1, 2),
                         [](const ::testing::TestParamInfo<int> &info) {
	                         return info.param == 1 ? "SingleThreaded" : "MultiThreaded";
                         });
