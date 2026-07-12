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
#include <mujoco_ros_sensors/mujoco_sensor_handler_plugin.hpp>
#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mujoco_ros_msgs/RegisterSensorNoiseModels.h>
#include <mujoco_ros_msgs/ScalarStamped.h>
#include <mujoco_ros_msgs/SensorNoiseModel.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mujoco_ros_msgs/msg/scalar_stamped.hpp>
#include <mujoco_ros_msgs/msg/sensor_noise_model.hpp>
#include <mujoco_ros_msgs/srv/register_sensor_noise_models.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <Eigen/Dense>
#include <tf2/LinearMath/Matrix3x3.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace mujoco_ros;

namespace {

static constexpr int NUM_SAMPLES                      = 5000;
static constexpr double FIRST_MESSAGE_TIMEOUT_SECONDS = 2.0;
static constexpr double NOISY_MEAN_TOLERANCE          = 2e-3;
static constexpr double NOISY_VARIANCE_TOLERANCE      = 1e-4;

#if MJR_ROS_VERSION == ROS_1
using PointStamped              = geometry_msgs::PointStamped;
using QuaternionStamped         = geometry_msgs::QuaternionStamped;
using RegisterSensorNoiseModels = mujoco_ros_msgs::RegisterSensorNoiseModels;
using ScalarStamped             = mujoco_ros_msgs::ScalarStamped;
using SensorNoiseModel          = mujoco_ros_msgs::SensorNoiseModel;
using Vector3Stamped            = geometry_msgs::Vector3Stamped;

template <typename MessageT>
using MessageConstPtr = typename MessageT::ConstPtr;
#else
using PointStamped              = geometry_msgs::msg::PointStamped;
using QuaternionStamped         = geometry_msgs::msg::QuaternionStamped;
using RegisterSensorNoiseModels = mujoco_ros_msgs::srv::RegisterSensorNoiseModels;
using ScalarStamped             = mujoco_ros_msgs::msg::ScalarStamped;
using SensorNoiseModel          = mujoco_ros_msgs::msg::SensorNoiseModel;
using Vector3Stamped            = geometry_msgs::msg::Vector3Stamped;

template <typename MessageT>
using MessageConstPtr = typename MessageT::ConstSharedPtr;
#endif

std::string get_sensors_model_path()
{
#if MJR_ROS_VERSION == ROS_1
	return ros::package::getPath("mujoco_ros_sensors") + "/test/sensors_world.xml";
#else
	return ament_index_cpp::get_package_share_directory("mujoco_ros_sensors") + "/test/sensors_world.xml";
#endif
}

void configure_plugin_params(testing::TestNodeHandle &nh)
{
#if MJR_ROS_VERSION == ROS_2
	nh.setParam("MujocoPlugins.names", std::vector<std::string>{ "mujoco_ros_sensors" });
	nh.setParam("MujocoPlugins.mujoco_ros_sensors.type", "mujoco_ros_sensors/MujocoRosSensorsPlugin");
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
	ros::init(argc, argv, "mujoco_ros_sensors_test");
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
	return ret;
#else
	const int ret = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return ret;
#endif
}

std::string noise_model_service_name(MujocoEnvTestWrapper *env_ptr)
{
#if MJR_ROS_VERSION == ROS_1
	(void)env_ptr;
	return "/sensors/register_noise_models";
#else
	return env_ptr->GetHandleNamespace() + "/mujoco_ros_sensors/sensors/register_noise_models";
#endif
}

std::string sensor_topic_name(MujocoEnvTestWrapper *env_ptr, const std::string &sensor_name)
{
#if MJR_ROS_VERSION == ROS_1
	(void)env_ptr;
	return "/" + sensor_name;
#else
	return env_ptr->GetHandleNamespace() + "/" + sensor_name;
#endif
}

template <typename MessageT>
MessageConstPtr<MessageT> wait_for_message(MujocoEnvTestWrapper *env_ptr, const std::string &topic,
                                           double timeout_seconds)
{
#if MJR_ROS_VERSION == ROS_1
	(void)env_ptr;
	return ros::topic::waitForMessage<MessageT>(topic, ros::Duration(timeout_seconds));
#else
	MessageConstPtr<MessageT> message;
	auto sub = env_ptr->template create_subscription<MessageT>(
	    topic, rclcpp::QoS(10), [&message](const MessageConstPtr<MessageT> msg) { message = msg; });

	const auto deadline =
	    std::chrono::steady_clock::now() +
	    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_seconds));
	while (message == nullptr && std::chrono::steady_clock::now() < deadline) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	return message;
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

void wait_for_subscription_discovery()
{
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

bool call_register_noise_models(MujocoEnvTestWrapper *env_ptr, testing::ServiceCall<RegisterSensorNoiseModels> &srv)
{
	env_ptr->togglePaused(true);

	const auto service_name = noise_model_service_name(env_ptr);
	const auto deadline     = std::chrono::steady_clock::now() + std::chrono::seconds(2);
	while (!testing::service_exists_for_test(env_ptr, service_name) && std::chrono::steady_clock::now() < deadline) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return testing::service_call_for_test(env_ptr, service_name, srv);
}

int getSensorByName(const std::string &sensor_name, mjModel *model)
{
	for (int n = 0; n < model->nsensor; n++) {
		if (!model->names[model->name_sensoradr[n]]) {
			continue;
		}

		if (sensor_name == mj_id2name(model, mjOBJ_SENSOR, n)) {
			return n;
		}
	}
	ADD_FAILURE() << "Could not find sensor `" << sensor_name << "' in model";
	return -1;
}

void compare_vectors(const std::vector<double> &a, const std::vector<double> &b, double tol, bool same)
{
	EXPECT_EQ(a.size(), b.size()) << "Size of compared vectors must be equal!";
	for (std::size_t i = 0; i < a.size(); i++) {
		if (same) {
			EXPECT_NEAR(a[i], b[i], tol) << "Vectors are not equal at index " << i;
		} else {
			EXPECT_NE(a[i], b[i]) << "Vectors are equal at index " << i;
		}
	}
}

int collected_samples(const std::atomic<int> &samples)
{
	return samples.load(std::memory_order_acquire);
}

int min_collected_samples(const std::atomic<int> &n, const std::atomic<int> &n_gt)
{
	return std::min(collected_samples(n), collected_samples(n_gt));
}

template <typename SampleWriterT>
void store_next_sample(std::atomic<int> &sample_count, SampleWriterT &&sample_writer)
{
	const int sample = sample_count.load(std::memory_order_relaxed);
	if (sample >= NUM_SAMPLES) {
		return;
	}

	sample_writer(sample);
	sample_count.store(sample + 1, std::memory_order_release);
}

bool collect_samples(MujocoEnvTestWrapper *env_ptr, std::atomic<int> &n, std::atomic<int> &n_gt)
{
	static constexpr int STEP_BATCH_SIZE = 100;
	static constexpr int WARMUP_SAMPLES  = 5;

	wait_for_subscription_discovery();
	env_ptr->togglePaused(true);

	env_ptr->step(WARMUP_SAMPLES);
	const auto warmup_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
	while (min_collected_samples(n, n_gt) < WARMUP_SAMPLES && std::chrono::steady_clock::now() < warmup_deadline) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	n.store(0, std::memory_order_release);
	n_gt.store(0, std::memory_order_release);

	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
	while ((collected_samples(n) < NUM_SAMPLES || collected_samples(n_gt) < NUM_SAMPLES) &&
	       std::chrono::steady_clock::now() < deadline) {
		const int missing_samples = NUM_SAMPLES - min_collected_samples(n, n_gt);
		const int steps_to_run    = std::min(STEP_BATCH_SIZE, std::max(1, missing_samples));
		const int target_samples  = std::min(NUM_SAMPLES, min_collected_samples(n, n_gt) + steps_to_run);
		env_ptr->step(steps_to_run);

		const auto drain_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
		while (min_collected_samples(n, n_gt) < target_samples && std::chrono::steady_clock::now() < drain_deadline) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}

	return collected_samples(n) >= NUM_SAMPLES && collected_samples(n_gt) >= NUM_SAMPLES;
}

std::vector<testing::TopicInfo> wait_for_sensor_topics(MujocoEnvTestWrapper *env_ptr, mjModel *model,
                                                       bool expect_gt_topics)
{
	std::vector<testing::TopicInfo> topics;
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);

	while (std::chrono::steady_clock::now() < deadline) {
		topics = testing::get_available_topics_for_test(env_ptr);

		bool found_all_topics = true;
		for (int n = 0; n < model->nsensor; n++) {
			if (!model->names[model->name_sensoradr[n]]) {
				continue;
			}

			const std::string sensor_name = mj_id2name(model, mjOBJ_SENSOR, n);
			found_all_topics &= testing::has_topic(topics, sensor_topic_name(env_ptr, sensor_name));
			if (expect_gt_topics) {
				found_all_topics &= testing::has_topic(topics, sensor_topic_name(env_ptr, sensor_name + "_GT"));
			}
		}

		if (found_all_topics) {
			return topics;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	return topics;
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

class SensorsFixture : public ::testing::Test
{
protected:
	std::unique_ptr<testing::TestNodeHandle> nh;
	std::unique_ptr<MujocoEnvTestWrapper> env_ptr;
	mjModel *m = nullptr;
	mjData *d  = nullptr;

	void setup(bool eval_mode)
	{
		nh = std::make_unique<testing::TestNodeHandle>("~");
		nh->setParam("eval_mode", eval_mode);
		nh->setParam("unpause", true);
		nh->setParam("no_render", true);
		nh->setParam("use_sim_time", true);
		configure_plugin_params(*nh);

		env_ptr = std::make_unique<MujocoEnvTestWrapper>(eval_mode ? "some_hash" : "", nh.get());

		const std::string xml_path = get_sensors_model_path();
		env_ptr->StartWithXML(xml_path, false);

		float seconds = 0;
		while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001f;
		}
		ASSERT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

		m = env_ptr->getModelPtr();
		d = env_ptr->getDataPtr();
	}

	void TearDown() override
	{
		if (env_ptr != nullptr) {
			env_ptr->shutdown();
		}
		if (nh != nullptr) {
			clear_test_params(*nh);
		}
	}
};

class TrainEnvFixture : public SensorsFixture
{
protected:
	void SetUp() override { setup(false); }
};

class EvalEnvFixture : public SensorsFixture
{
protected:
	void SetUp() override { setup(true); }
};

TEST_F(TrainEnvFixture, PluginLoaded)
{
	ASSERT_EQ(env_ptr->GetPlugins().size(), 1u);
	EXPECT_NE(dynamic_cast<mujoco_ros::sensors::MujocoRosSensorsPlugin *>(env_ptr->GetPlugins().front().get()), nullptr);
}

TEST_F(TrainEnvFixture, SensorCreatedTrain)
{
	const auto topics = wait_for_sensor_topics(env_ptr.get(), m, true);

	for (int n = 0; n < m->nsensor; n++) {
		if (!m->names[m->name_sensoradr[n]]) {
			continue;
		}

		const std::string sensor_name = mj_id2name(m, mjOBJ_SENSOR, n);
		EXPECT_TRUE(testing::has_topic(topics, sensor_topic_name(env_ptr.get(), sensor_name)))
		    << "Value topic should have been generated for sensor " << sensor_name;
		EXPECT_TRUE(testing::has_topic(topics, sensor_topic_name(env_ptr.get(), sensor_name + "_GT")))
		    << "GT topic should have been generated for sensor " << sensor_name;
	}
}

TEST_F(EvalEnvFixture, SensorCreatedEval)
{
	const auto topics = wait_for_sensor_topics(env_ptr.get(), m, false);

	for (int n = 0; n < m->nsensor; n++) {
		if (!m->names[m->name_sensoradr[n]]) {
			continue;
		}

		const std::string sensor_name = mj_id2name(m, mjOBJ_SENSOR, n);
		EXPECT_TRUE(testing::has_topic(topics, sensor_topic_name(env_ptr.get(), sensor_name)))
		    << "Value topic should have been generated for sensor " << sensor_name;
		EXPECT_FALSE(testing::has_topic(topics, sensor_topic_name(env_ptr.get(), sensor_name + "_GT")))
		    << "GT topic should not have been generated for sensor " << sensor_name;
	}
}

TEST_F(TrainEnvFixture, Sensor3DOF)
{
	const int n_sensor = getSensorByName("vel_EE", m);
	ASSERT_GE(n_sensor, 0);

	const int adr    = m->sensor_adr[n_sensor];
	const int cutoff = (m->sensor_cutoff[n_sensor] > 0 ? m->sensor_cutoff[n_sensor] : 1);

	const auto msgPtr_GT = wait_for_message<Vector3Stamped>(env_ptr.get(), sensor_topic_name(env_ptr.get(), "vel_EE_GT"),
	                                                        FIRST_MESSAGE_TIMEOUT_SECONDS);
	const auto msgPtr    = wait_for_message<Vector3Stamped>(env_ptr.get(), sensor_topic_name(env_ptr.get(), "vel_EE"),
	                                                        FIRST_MESSAGE_TIMEOUT_SECONDS);

	ASSERT_TRUE(msgPtr != nullptr) << "Could not get message on /vel_EE topic!";
	ASSERT_TRUE(msgPtr_GT != nullptr) << "Could not get message on /vel_EE_GT topic!";

	compare_vectors({ msgPtr_GT->vector.x, msgPtr_GT->vector.y, msgPtr_GT->vector.z },
	                { msgPtr->vector.x, msgPtr->vector.y, msgPtr->vector.z }, 1e-4, true);

	SensorNoiseModel noise_model;
	noise_model.mean        = { 0.0, 1.0, 0.0 };
	noise_model.std         = { 0.025, 0.0, 0.0 };
	noise_model.set_flag    = 7;
	noise_model.sensor_name = "vel_EE";

	testing::ServiceCall<RegisterSensorNoiseModels> srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "example_hash";
	EXPECT_TRUE(call_register_noise_models(env_ptr.get(), srv)) << "Service call failed!";

	std::array<double, 3 * NUM_SAMPLES> values{};
	std::array<double, 3 * NUM_SAMPLES> values_gt{};
	std::atomic<int> n{ 0 };
	std::atomic<int> n_gt{ 0 };

	auto sub = subscribe_to<Vector3Stamped>(env_ptr.get(), nh.get(), sensor_topic_name(env_ptr.get(), "vel_EE"),
	                                        NUM_SAMPLES, [&values, &n](const MessageConstPtr<Vector3Stamped> &msg) {
		                                        store_next_sample(n, [&values, &msg](int sample) {
			                                        values[sample * 3]     = msg->vector.x;
			                                        values[sample * 3 + 1] = msg->vector.y;
			                                        values[sample * 3 + 2] = msg->vector.z;
		                                        });
	                                        });

	auto sub_gt =
	    subscribe_to<Vector3Stamped>(env_ptr.get(), nh.get(), sensor_topic_name(env_ptr.get(), "vel_EE_GT"), NUM_SAMPLES,
	                                 [&values_gt, &n_gt](const MessageConstPtr<Vector3Stamped> &msg) {
		                                 store_next_sample(n_gt, [&values_gt, &msg](int sample) {
			                                 values_gt[sample * 3]     = msg->vector.x;
			                                 values_gt[sample * 3 + 1] = msg->vector.y;
			                                 values_gt[sample * 3 + 2] = msg->vector.z;
		                                 });
	                                 });

	ASSERT_TRUE(collect_samples(env_ptr.get(), n, n_gt))
	    << "Messages not received within 15s (got " << collected_samples(n) << " and " << collected_samples(n_gt)
	    << " of " << NUM_SAMPLES << " expected, respectively)";

	compare_vectors({ values_gt[0], values_gt[1], values_gt[2] },
	                { d->sensordata[adr] / cutoff, d->sensordata[adr + 1] / cutoff, d->sensordata[adr + 2] / cutoff },
	                0.0001, true);

	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 3, Eigen::RowMajor>> values_map(values.data());
	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 3, Eigen::RowMajor>> values_gt_map(values_gt.data());

	auto diff                             = values_map - values_gt_map;
	Eigen::Matrix<double, 1, 3> means     = diff.colwise().mean();
	Eigen::Matrix<double, 1, 3> variances = (diff.rowwise() - diff.colwise().mean()).array().square().colwise().mean();

	EXPECT_NEAR(means(0), 0, NOISY_MEAN_TOLERANCE);
	EXPECT_NEAR(means(1), 1, NOISY_MEAN_TOLERANCE);
	EXPECT_NEAR(means(2), 0, NOISY_MEAN_TOLERANCE);

	EXPECT_NEAR(variances(0), 0.000625, NOISY_VARIANCE_TOLERANCE);
	EXPECT_NEAR(variances(1), 0., 5e-5);
	EXPECT_NEAR(variances(2), 0., 5e-5);

	Eigen::Matrix<double, 1, 3> means_gt = values_gt_map.colwise().mean();
	Eigen::Matrix<double, 1, 3> variances_gt =
	    (values_gt_map.rowwise() - values_gt_map.colwise().mean()).array().square().colwise().mean();

	EXPECT_NEAR(means_gt(0), d->sensordata[adr] / cutoff, 1e-3);
	EXPECT_NEAR(means_gt(1), d->sensordata[adr + 1] / cutoff, 1e-3);
	EXPECT_NEAR(means_gt(2), d->sensordata[adr + 2] / cutoff, 1e-3);

	EXPECT_EQ(variances_gt(0), 0);
	EXPECT_EQ(variances_gt(1), 0);
	EXPECT_EQ(variances_gt(2), 0);
}

TEST_F(TrainEnvFixture, Framepos)
{
	const int n_sensor = getSensorByName("immovable_pos", m);
	ASSERT_GE(n_sensor, 0);

	const int adr    = m->sensor_adr[n_sensor];
	const int cutoff = (m->sensor_cutoff[n_sensor] > 0 ? m->sensor_cutoff[n_sensor] : 1);

	const auto msgPtr_GT = wait_for_message<PointStamped>(
	    env_ptr.get(), sensor_topic_name(env_ptr.get(), "immovable_pos_GT"), FIRST_MESSAGE_TIMEOUT_SECONDS);
	const auto msgPtr = wait_for_message<PointStamped>(env_ptr.get(), sensor_topic_name(env_ptr.get(), "immovable_pos"),
	                                                   FIRST_MESSAGE_TIMEOUT_SECONDS);

	ASSERT_TRUE(msgPtr != nullptr) << "Could not get message on /immovable_pos topic!";
	ASSERT_TRUE(msgPtr_GT != nullptr) << "Could not get message on /immovable_pos_GT topic!";

	compare_vectors({ msgPtr_GT->point.x, msgPtr_GT->point.y, msgPtr_GT->point.z },
	                { msgPtr->point.x, msgPtr->point.y, msgPtr->point.z }, 0.0001, true);

	SensorNoiseModel noise_model;
	noise_model.mean        = { 0.0, 1.0, 0.0 };
	noise_model.std         = { 0.025, 0.0, 0.0 };
	noise_model.set_flag    = 7;
	noise_model.sensor_name = "immovable_pos";

	testing::ServiceCall<RegisterSensorNoiseModels> srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "example_hash";
	EXPECT_TRUE(call_register_noise_models(env_ptr.get(), srv)) << "Service call failed!";

	std::array<double, 3 * NUM_SAMPLES> values{};
	std::array<double, 3 * NUM_SAMPLES> values_gt{};
	std::atomic<int> n{ 0 };
	std::atomic<int> n_gt{ 0 };

	auto sub = subscribe_to<PointStamped>(env_ptr.get(), nh.get(), sensor_topic_name(env_ptr.get(), "immovable_pos"),
	                                      NUM_SAMPLES, [&values, &n](const MessageConstPtr<PointStamped> &msg) {
		                                      store_next_sample(n, [&values, &msg](int sample) {
			                                      values[sample * 3]     = msg->point.x;
			                                      values[sample * 3 + 1] = msg->point.y;
			                                      values[sample * 3 + 2] = msg->point.z;
		                                      });
	                                      });

	auto sub_gt =
	    subscribe_to<PointStamped>(env_ptr.get(), nh.get(), sensor_topic_name(env_ptr.get(), "immovable_pos_GT"),
	                               NUM_SAMPLES, [&values_gt, &n_gt](const MessageConstPtr<PointStamped> &msg) {
		                               store_next_sample(n_gt, [&values_gt, &msg](int sample) {
			                               values_gt[sample * 3]     = msg->point.x;
			                               values_gt[sample * 3 + 1] = msg->point.y;
			                               values_gt[sample * 3 + 2] = msg->point.z;
		                               });
	                               });

	ASSERT_TRUE(collect_samples(env_ptr.get(), n, n_gt))
	    << "Messages not received within 15s (got " << collected_samples(n) << " and " << collected_samples(n_gt)
	    << " of " << NUM_SAMPLES << " expected, respectively)";

	compare_vectors({ values_gt[0], values_gt[1], values_gt[2] },
	                { d->sensordata[adr] / cutoff, d->sensordata[adr + 1] / cutoff, d->sensordata[adr + 2] / cutoff },
	                0.0001, true);

	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 3, Eigen::RowMajor>> values_map(values.data());
	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 3, Eigen::RowMajor>> values_gt_map(values_gt.data());

	auto diff                             = values_map - values_gt_map;
	Eigen::Matrix<double, 1, 3> means     = diff.colwise().mean();
	Eigen::Matrix<double, 1, 3> variances = (diff.rowwise() - diff.colwise().mean()).array().square().colwise().mean();

	EXPECT_NEAR(means(0), 0, NOISY_MEAN_TOLERANCE);
	EXPECT_NEAR(means(1), 1., NOISY_MEAN_TOLERANCE);
	EXPECT_NEAR(means(2), 0., NOISY_MEAN_TOLERANCE);

	EXPECT_NEAR(variances(0), 0.000625, NOISY_VARIANCE_TOLERANCE);
	EXPECT_NEAR(variances(1), 0., 5e-5);
	EXPECT_NEAR(variances(2), 0., 5e-5);

	Eigen::Matrix<double, 1, 3> means_gt = values_gt_map.colwise().mean();
	Eigen::Matrix<double, 1, 3> variances_gt =
	    (values_gt_map.rowwise() - values_gt_map.colwise().mean()).array().square().colwise().mean();

	EXPECT_NEAR(means_gt[0], d->sensordata[adr] / cutoff, 1e-3);
	EXPECT_NEAR(means_gt[1], d->sensordata[adr + 1] / cutoff, 1e-3);
	EXPECT_NEAR(means_gt[2], d->sensordata[adr + 2] / cutoff, 1e-3);

	EXPECT_EQ(variances_gt[0], 0);
	EXPECT_EQ(variances_gt[1], 0);
	EXPECT_EQ(variances_gt[2], 0);
}

TEST_F(TrainEnvFixture, ScalarStamped)
{
	const int n_sensor = getSensorByName("vel_joint2", m);
	ASSERT_GE(n_sensor, 0);

	const int adr    = m->sensor_adr[n_sensor];
	const int cutoff = (m->sensor_cutoff[n_sensor] > 0 ? m->sensor_cutoff[n_sensor] : 1);

	const auto msgPtr_GT = wait_for_message<ScalarStamped>(
	    env_ptr.get(), sensor_topic_name(env_ptr.get(), "vel_joint2_GT"), FIRST_MESSAGE_TIMEOUT_SECONDS);
	const auto msgPtr = wait_for_message<ScalarStamped>(env_ptr.get(), sensor_topic_name(env_ptr.get(), "vel_joint2"),
	                                                    FIRST_MESSAGE_TIMEOUT_SECONDS);

	ASSERT_TRUE(msgPtr != nullptr) << "Could not get message on /vel_joint2 topic!";
	ASSERT_TRUE(msgPtr_GT != nullptr) << "Could not get message on /vel_joint2_GT topic!";

	EXPECT_NEAR(msgPtr_GT->value, msgPtr->value, 0.0001) << "Without noise sensor value should equal GT";

	SensorNoiseModel noise_model;
	noise_model.mean        = { 1.0 };
	noise_model.std         = { 0.025 };
	noise_model.set_flag    = 1;
	noise_model.sensor_name = "vel_joint2";

	testing::ServiceCall<RegisterSensorNoiseModels> srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "example_hash";
	EXPECT_TRUE(call_register_noise_models(env_ptr.get(), srv)) << "Service call failed!";

	std::array<double, NUM_SAMPLES> values{};
	std::array<double, NUM_SAMPLES> values_gt{};
	std::atomic<int> n{ 0 };
	std::atomic<int> n_gt{ 0 };

	auto sub = subscribe_to<ScalarStamped>(env_ptr.get(), nh.get(), sensor_topic_name(env_ptr.get(), "vel_joint2"),
	                                       NUM_SAMPLES, [&values, &n](const MessageConstPtr<ScalarStamped> &msg) {
		                                       store_next_sample(
		                                           n, [&values, &msg](int sample) { values[sample] = msg->value; });
	                                       });

	auto sub_gt = subscribe_to<ScalarStamped>(
	    env_ptr.get(), nh.get(), sensor_topic_name(env_ptr.get(), "vel_joint2_GT"), NUM_SAMPLES,
	    [&values_gt, &n_gt](const MessageConstPtr<ScalarStamped> &msg) {
		    store_next_sample(n_gt, [&values_gt, &msg](int sample) { values_gt[sample] = msg->value; });
	    });

	ASSERT_TRUE(collect_samples(env_ptr.get(), n, n_gt))
	    << "Messages not received within 15s (got " << collected_samples(n) << " and " << collected_samples(n_gt)
	    << " of " << NUM_SAMPLES << " expected, respectively)";

	EXPECT_NEAR(values_gt[0], d->sensordata[adr] / cutoff, 0.0001) << "GT differs from actual sensor value";

	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 1>> values_map(values.data());
	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 1>> values_gt_map(values_gt.data());

	auto diff       = values_map - values_gt_map;
	double mean     = diff.colwise().mean()(0);
	double variance = (diff.rowwise() - diff.colwise().mean()).array().square().colwise().mean()(0);

	EXPECT_NEAR(mean, 1, 0.01);
	EXPECT_NEAR(variance, 0.000625, 0.0001);

	double mean_gt     = values_gt_map.colwise().mean()(0);
	double variance_gt = (values_gt_map.rowwise() - values_gt_map.colwise().mean()).array().square().colwise().mean()(0);

	EXPECT_NEAR(mean_gt, d->sensordata[adr] / cutoff, 0.0001);
	EXPECT_EQ(variance_gt, 0);
}

TEST_F(TrainEnvFixture, Quaternion)
{
	const int n_sensor = getSensorByName("immovable_quat", m);
	ASSERT_GE(n_sensor, 0);

	const int adr    = m->sensor_adr[n_sensor];
	const int cutoff = (m->sensor_cutoff[n_sensor] > 0 ? m->sensor_cutoff[n_sensor] : 1);

	const auto msgPtr_GT = wait_for_message<QuaternionStamped>(
	    env_ptr.get(), sensor_topic_name(env_ptr.get(), "immovable_quat_GT"), FIRST_MESSAGE_TIMEOUT_SECONDS);
	const auto msgPtr = wait_for_message<QuaternionStamped>(
	    env_ptr.get(), sensor_topic_name(env_ptr.get(), "immovable_quat"), FIRST_MESSAGE_TIMEOUT_SECONDS);

	ASSERT_TRUE(msgPtr != nullptr) << "Could not get message on /immovable_quat topic!";
	ASSERT_TRUE(msgPtr_GT != nullptr) << "Could not get message on /immovable_quat_GT topic!";

	compare_vectors(
	    { msgPtr_GT->quaternion.w, msgPtr_GT->quaternion.x, msgPtr_GT->quaternion.y, msgPtr_GT->quaternion.z },
	    { msgPtr->quaternion.w, msgPtr->quaternion.x, msgPtr->quaternion.y, msgPtr->quaternion.z }, 0.0001, true);

	SensorNoiseModel noise_model;
	noise_model.mean        = { 0.0, 0.0, 1.0 };
	noise_model.std         = { 0.0, 0.0, 0.025 };
	noise_model.set_flag    = 7;
	noise_model.sensor_name = "immovable_quat";

	testing::ServiceCall<RegisterSensorNoiseModels> srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "example_hash";
	EXPECT_TRUE(call_register_noise_models(env_ptr.get(), srv)) << "Service call failed!";

	std::array<double, 3 * NUM_SAMPLES> values{};
	std::array<double, 3 * NUM_SAMPLES> values_gt{};
	std::vector<double> gt_quat;
	std::atomic<int> n{ 0 };
	std::atomic<int> n_gt{ 0 };

	auto sub = subscribe_to<QuaternionStamped>(
	    env_ptr.get(), nh.get(), sensor_topic_name(env_ptr.get(), "immovable_quat"), NUM_SAMPLES,
	    [&values, &n](const MessageConstPtr<QuaternionStamped> &msg) {
		    store_next_sample(n, [&values, &msg](int sample) {
			    tf2::Quaternion q;
			    tf2::fromMsg(msg->quaternion, q);
			    auto m = tf2::Matrix3x3(q);
			    m.getRPY(values[sample * 3], values[sample * 3 + 1], values[sample * 3 + 2]);
		    });
	    });

	auto sub_gt = subscribe_to<QuaternionStamped>(
	    env_ptr.get(), nh.get(), sensor_topic_name(env_ptr.get(), "immovable_quat_GT"), NUM_SAMPLES,
	    [&values_gt, &gt_quat, &n_gt](const MessageConstPtr<QuaternionStamped> &msg) {
		    store_next_sample(n_gt, [&values_gt, &gt_quat, &msg](int sample) {
			    if (sample == 0) {
				    gt_quat = { msg->quaternion.w, msg->quaternion.x, msg->quaternion.y, msg->quaternion.z };
			    }
			    tf2::Quaternion q;
			    tf2::fromMsg(msg->quaternion, q);
			    auto m = tf2::Matrix3x3(q);
			    m.getRPY(values_gt[sample * 3], values_gt[sample * 3 + 1], values_gt[sample * 3 + 2]);
		    });
	    });

	ASSERT_TRUE(collect_samples(env_ptr.get(), n, n_gt))
	    << "Messages not received within 15s (got " << collected_samples(n) << " and " << collected_samples(n_gt)
	    << " of " << NUM_SAMPLES << " expected, respectively)";

	compare_vectors(gt_quat,
	                { d->sensordata[adr] / cutoff, d->sensordata[adr + 1] / cutoff, d->sensordata[adr + 2] / cutoff,
	                  d->sensordata[adr + 3] / cutoff },
	                0.0001, true);

	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 3, Eigen::RowMajor>> values_map(values.data());
	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 3, Eigen::RowMajor>> values_gt_map(values_gt.data());

	auto diff                             = values_map - values_gt_map;
	Eigen::Matrix<double, 1, 3> means     = diff.colwise().mean();
	Eigen::Matrix<double, 1, 3> variances = (diff.rowwise() - diff.colwise().mean()).array().square().colwise().mean();

	EXPECT_EQ(means(0), 0);
	EXPECT_EQ(means(1), 0);
	EXPECT_NEAR(means(2), 1, 0.01);

	EXPECT_EQ(variances(0), 0);
	EXPECT_EQ(variances(1), 0);
	EXPECT_NEAR(variances(2), 0.000625, 0.0001);

	Eigen::Matrix<double, 1, 3> means_gt = values_gt_map.colwise().mean();
	Eigen::Matrix<double, 1, 3> variances_gt =
	    (values_gt_map.rowwise() - values_gt_map.colwise().mean()).array().square().colwise().mean();

	double R, P, Y;
	tf2::Quaternion q;
	tf2::fromMsg(msgPtr_GT->quaternion, q);
	auto m = tf2::Matrix3x3(q);
	m.getRPY(R, P, Y);

	EXPECT_NEAR(means_gt(0), R, 0.0001);
	EXPECT_NEAR(means_gt(1), P, 0.0001);
	EXPECT_NEAR(means_gt(2), Y, 0.0001);

	EXPECT_EQ(variances_gt(0), 0);
	EXPECT_EQ(variances_gt(1), 0);
	EXPECT_EQ(variances_gt(2), 0);
}

TEST_F(EvalEnvFixture, NoAdmEvalNoiseModelChange)
{
	SensorNoiseModel noise_model;
	noise_model.mean        = { 0.0, 1.0 };
	noise_model.std         = { 0.025, 0.0 };
	noise_model.set_flag    = 3;
	noise_model.sensor_name = "vel_EE";

	testing::ServiceCall<RegisterSensorNoiseModels> srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "some_wrong_hash";

	EXPECT_TRUE(call_register_noise_models(env_ptr.get(), srv)) << "Service call failed!";
	EXPECT_FALSE(srv.response.success) << "Service call should have failed!";
}

TEST_F(EvalEnvFixture, AllowedEvalNoiseModelChange)
{
	SensorNoiseModel noise_model;
	noise_model.mean        = { 0.0, 1.0 };
	noise_model.std         = { 0.025, 0.0 };
	noise_model.set_flag    = 3;
	noise_model.sensor_name = "vel_EE";

	testing::ServiceCall<RegisterSensorNoiseModels> srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "some_hash";

	EXPECT_TRUE(call_register_noise_models(env_ptr.get(), srv)) << "Service call failed!";
	EXPECT_TRUE(srv.response.success) << "Service call should have succeeded!";
}

TEST_F(TrainEnvFixture, UnknownSensorAddNoise)
{
	SensorNoiseModel noise_model;
	noise_model.mean        = { 0.0, 1.0 };
	noise_model.std         = { 0.025, 0.0 };
	noise_model.set_flag    = 3;
	noise_model.sensor_name = "unknown_sensor";

	testing::ServiceCall<RegisterSensorNoiseModels> srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "example_hash";

	EXPECT_TRUE(call_register_noise_models(env_ptr.get(), srv)) << "Service call failed!";
	EXPECT_TRUE(srv.response.success) << "Service call should have succeeded!";
}
