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

#include <mujoco_ros/render_backend.h>
#include "mujoco_env_fixture.h"
#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/common_types.h>
#include <mujoco_ros_sensors/mujoco_sensor_handler_plugin.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <mujoco_ros_msgs/SensorNoiseModel.h>
#include <mujoco_ros_msgs/RegisterSensorNoiseModels.h>
#include <mujoco_ros_msgs/ScalarStamped.h>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/package.h>
#include <chrono>
#include <thread>

using namespace mujoco_ros;
namespace mju = ::mujoco::sample_util;

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_ros_sensors_test");

	// Uncomment to enable debug output (useful for debugging failing tests)
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	ros::console::notifyLoggerLevelsChanged();

	ros::AsyncSpinner spinner(1);
	spinner.start();
	int ret = RUN_ALL_TESTS();

	spinner.stop();
	return ret;
}

static constexpr int NUM_SAMPLES = 20000;

class TrainEnvFixture : public ::testing::Test
{
protected:
	std::unique_ptr<ros::NodeHandle> nh;
	MujocoEnvTestWrapper *env_ptr;
	mjModel *m;
	mjData *d;

	void SetUp() override
	{
		nh = std::make_unique<ros::NodeHandle>("~");
		nh->setParam("eval_mode", false);
		nh->setParam("unpause", true);
		nh->setParam("no_render", true);
		nh->setParam("use_sim_time", true);

		env_ptr = new MujocoEnvTestWrapper();

		std::string xml_path = ros::package::getPath("mujoco_ros_sensors") + "/test/sensors_world.xml";
		env_ptr->startWithXML(xml_path);

		float seconds = 0;
		while (env_ptr->getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		ASSERT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

		m = env_ptr->getModelPtr();
		d = env_ptr->getDataPtr();
	}

	void TearDown() override
	{
		env_ptr->shutdown();
		delete env_ptr;

		// clear all parameters
		ros::param::del(nh->getNamespace());
	}
};

class EvalEnvFixture : public ::testing::Test
{
protected:
	std::unique_ptr<ros::NodeHandle> nh;
	MujocoEnvTestWrapper *env_ptr;
	mjModel *m;
	mjData *d;

	void SetUp() override
	{
		nh = std::make_unique<ros::NodeHandle>("~");
		nh->setParam("eval_mode", true);
		nh->setParam("unpause", true);
		nh->setParam("no_render", true);
		nh->setParam("use_sim_time", true);

		env_ptr = new MujocoEnvTestWrapper("some_hash");

		std::string xml_path = ros::package::getPath("mujoco_ros_sensors") + "/test/sensors_world.xml";
		env_ptr->startWithXML(xml_path);

		float seconds = 0;
		while (env_ptr->getOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		ASSERT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

		m = env_ptr->getModelPtr();
		d = env_ptr->getDataPtr();
	}

	void TearDown() override
	{
		env_ptr->shutdown();
		delete env_ptr;

		// clear all parameters
		ros::param::del(nh->getNamespace());
	}
};

TEST_F(TrainEnvFixture, SensorCreatedTrain)
{
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	std::string sensor_name;
	bool found_val_top, found_gt_top;

	for (int n = 0; n < m->nsensor; n++) {
		found_gt_top  = false;
		found_val_top = false;

		if (m->names[m->name_sensoradr[n]]) {
			sensor_name = mj_id2name(m, mjOBJ_SENSOR, n);
		} else {
			continue;
		}

		for (const auto &info : master_topics) {
			// ROS_INFO_STREAM("Found topic " << info.name);
			if (info.name.find(sensor_name + "_GT") != std::string::npos) {
				found_gt_top = true;
				ROS_DEBUG_STREAM("Found GT topic for " << sensor_name << " at " << info.name);
			} else if (info.name.find(sensor_name) != std::string::npos) {
				found_val_top = true;
				ROS_DEBUG_STREAM("Found val topic for " << sensor_name << " at " << info.name);
			}
		}
		EXPECT_TRUE(found_gt_top && found_val_top)
		    << "GT and value topic should have been generated for sensor " << sensor_name;
	}
}

TEST_F(EvalEnvFixture, SensorCreatedEval)
{
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	std::string sensor_name;

	bool found_val_top, found_gt_top;

	for (int n = 0; n < m->nsensor; n++) {
		found_gt_top  = false;
		found_val_top = false;

		if (m->names[m->name_sensoradr[n]]) {
			sensor_name = mj_id2name(m, mjOBJ_SENSOR, n);
		} else {
			continue;
		}

		for (const auto &info : master_topics) {
			if (info.name.find(sensor_name + "_GT") != std::string::npos) {
				found_gt_top = true;
				ROS_DEBUG_STREAM("Found GT topic for " << sensor_name << " at " << info.name);
			} else if (info.name.find(sensor_name) != std::string::npos) {
				found_val_top = true;
				ROS_DEBUG_STREAM("Found val topic for " << sensor_name << " at " << info.name);
			}
		}
		EXPECT_TRUE(found_val_top) << "Value topic should have been generated for sensor " << sensor_name;
		EXPECT_FALSE(found_gt_top) << "GT topic should not have been generated for sensor " << sensor_name;
	}
}

void getSensorByName(const std::string &sensor_name, mjModel *m, int &n_sensor)
{
	std::string name;

	for (int n = 0; n < m->nsensor; n++) {
		if (m->names[m->name_sensoradr[n]]) {
			name = mj_id2name(m, mjOBJ_SENSOR, n);
		} else {
			continue;
		}
		if (name == sensor_name) {
			n_sensor = n;
			return;
		}
	}
	EXPECT_TRUE(false) << "Could not find sensor `" << sensor_name << "' in model";
	return;
}

void compare_vectors(std::vector<double> a, std::vector<double> b, double tol, bool same)
{
	EXPECT_EQ(a.size(), b.size()) << "Size of compared vectors must be equal!";
	for (int i = 0; i < a.size(); i++) {
		if (same) {
			EXPECT_NEAR(a[i], b[i], tol) << "Vectors are not equal at index " << i;
		} else {
			EXPECT_NE(a[i], b[i]) << "Vectors are equal at index " << i;
		}
	}
}

TEST_F(TrainEnvFixture, Sensor3DOF)
{
	int n_sensor;
	getSensorByName("vel_EE", m, n_sensor);

	int adr    = m->sensor_adr[n_sensor];
	int cutoff = (m->sensor_cutoff[n_sensor] > 0 ? m->sensor_cutoff[n_sensor] : 1);

	// Check if topics are published
	geometry_msgs::Vector3StampedConstPtr msgPtr_GT;
	msgPtr_GT = ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("/vel_EE_GT", ros::Duration(0.1));

	geometry_msgs::Vector3StampedConstPtr msgPtr;
	msgPtr = ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("/vel_EE", ros::Duration(0.1));

	EXPECT_TRUE(msgPtr != nullptr) << "Could not get message on /vel_EE topic!";
	EXPECT_TRUE(msgPtr_GT != nullptr) << "Could not get message on /vel_EE_GT topic!";

	// Without noise should be the same
	compare_vectors({ msgPtr_GT->vector.x, msgPtr_GT->vector.y, msgPtr_GT->vector.z },
	                { msgPtr->vector.x, msgPtr->vector.y, msgPtr->vector.z }, 1e-4, true);

	mujoco_ros_msgs::SensorNoiseModel noise_model;
	noise_model.mean.emplace_back(0.0);
	noise_model.mean.emplace_back(1.0);
	noise_model.mean.emplace_back(0.0);
	noise_model.std.emplace_back(0.025);
	noise_model.std.emplace_back(0.0);
	noise_model.std.emplace_back(0.0);
	noise_model.set_flag    = 7;
	noise_model.sensor_name = "vel_EE";

	mujoco_ros_msgs::RegisterSensorNoiseModels srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "example_hash";

	ros::ServiceClient client =
	    nh->serviceClient<mujoco_ros_msgs::RegisterSensorNoiseModels>("/sensors/register_noise_models");
	EXPECT_TRUE(client.call(srv)) << "Service call failed!";

	// Pause sim for synchronous message
	env_ptr->settings_.run.store(0);

	int n                             = 0;
	double values[3 * NUM_SAMPLES]    = { 0 };
	int n_gt                          = 0;
	double values_gt[3 * NUM_SAMPLES] = { 0 };

	ros::Subscriber sub = nh->subscribe<geometry_msgs::Vector3Stamped>(
	    "/vel_EE", NUM_SAMPLES, [&values, &n](const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
		    values[n * 3]     = msg->vector.x;
		    values[n * 3 + 1] = msg->vector.y;
		    values[n * 3 + 2] = msg->vector.z;
		    n += 1;
	    });

	ros::Subscriber sub_gt = nh->subscribe<geometry_msgs::Vector3Stamped>(
	    "/vel_EE_GT", NUM_SAMPLES, [&values_gt, &n_gt](const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
		    values_gt[n_gt * 3]     = msg->vector.x;
		    values_gt[n_gt * 3 + 1] = msg->vector.y;
		    values_gt[n_gt * 3 + 2] = msg->vector.z;
		    n_gt += 1;
	    });

	// Wait for latched messages to be received
	float secs = 0.f;
	while ((n < 1 || n_gt < 1) && secs < 1.f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		secs += 0.001f;
	}
	ASSERT_LT(secs, 1.f) << "Messages not received within 1s";

	// GT == Sensor reading
	compare_vectors({ values_gt[0], values_gt[1], values_gt[2] },
	                { d->sensordata[adr] / cutoff, d->sensordata[adr + 1] / cutoff, d->sensordata[adr + 2] / cutoff },
	                0.0001, true);

	env_ptr->step(NUM_SAMPLES - 1);

	secs = 0.f;
	while ((n < NUM_SAMPLES || n_gt < NUM_SAMPLES) && secs < 15.f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		secs += 0.001f;
	}
	ASSERT_LT(secs, 15.f) << "Messages not received within 15s (got " << n << " and " << n_gt << " of " << NUM_SAMPLES
	                      << " expected, respectively)";

	// Map values to Eigen matrix for fast metrics computation
	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 3, Eigen::RowMajor>> values_map(values);
	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 3, Eigen::RowMajor>> values_gt_map(values_gt);

	// Compute means and variances
	auto diff                             = values_map - values_gt_map;
	Eigen::Matrix<double, 1, 3> means     = diff.colwise().mean();
	Eigen::Matrix<double, 1, 3> variances = (diff.rowwise() - diff.colwise().mean()).array().square().colwise().mean();

	EXPECT_NEAR(means(0), 0, 1e-3);
	EXPECT_NEAR(means(1), 1, 1e-3);
	EXPECT_NEAR(means(2), 0, 1e-3);

	EXPECT_NEAR(variances(0), 0.000625, 2e-5);
	EXPECT_NEAR(variances(1), 0., 5e-5);
	EXPECT_NEAR(variances(2), 0., 2e-5);

	// Map values to Eigen matrix for fast metrics computation
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
	int n_sensor;
	getSensorByName("immovable_pos", m, n_sensor);

	int adr    = m->sensor_adr[n_sensor];
	int cutoff = (m->sensor_cutoff[n_sensor] > 0 ? m->sensor_cutoff[n_sensor] : 1);

	// Check if topics are published
	geometry_msgs::PointStampedConstPtr msgPtr_GT;
	msgPtr_GT = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/immovable_pos_GT", ros::Duration(0.1));

	geometry_msgs::PointStampedConstPtr msgPtr;
	msgPtr = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/immovable_pos", ros::Duration(0.1));

	EXPECT_TRUE(msgPtr != nullptr) << "Could not get message on /immovable_pos topic!";
	EXPECT_TRUE(msgPtr_GT != nullptr) << "Could not get message on /immovable_pos_GT topic!";

	// Without noise should be the same
	compare_vectors({ msgPtr_GT->point.x, msgPtr_GT->point.y, msgPtr_GT->point.z },
	                { msgPtr->point.x, msgPtr->point.y, msgPtr->point.z }, 0.0001, true);

	mujoco_ros_msgs::SensorNoiseModel noise_model;
	noise_model.mean.emplace_back(0.0);
	noise_model.mean.emplace_back(1.0);
	noise_model.mean.emplace_back(0.0);
	noise_model.std.emplace_back(0.025);
	noise_model.std.emplace_back(0.0);
	noise_model.std.emplace_back(0.0);
	noise_model.set_flag    = 7;
	noise_model.sensor_name = "immovable_pos";

	mujoco_ros_msgs::RegisterSensorNoiseModels srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "example_hash";

	ros::ServiceClient client =
	    nh->serviceClient<mujoco_ros_msgs::RegisterSensorNoiseModels>("/sensors/register_noise_models");
	EXPECT_TRUE(client.call(srv)) << "Service call failed!";

	// Pause sim for synchronous message
	env_ptr->settings_.run.store(0);

	double values[3 * NUM_SAMPLES]    = { 0 };
	int n                             = 0;
	double values_gt[3 * NUM_SAMPLES] = { 0 };
	int n_gt                          = 0;

	ros::Subscriber sub = nh->subscribe<geometry_msgs::PointStamped>(
	    "/immovable_pos", NUM_SAMPLES, [&values, &n](const geometry_msgs::PointStamped::ConstPtr &msg) {
		    values[n * 3]     = msg->point.x;
		    values[n * 3 + 1] = msg->point.y;
		    values[n * 3 + 2] = msg->point.z;
		    n += 1;
	    });

	ros::Subscriber sub_gt = nh->subscribe<geometry_msgs::PointStamped>(
	    "/immovable_pos_GT", NUM_SAMPLES, [&values_gt, &n_gt](const geometry_msgs::PointStamped::ConstPtr &msg) {
		    values_gt[n_gt * 3]     = msg->point.x;
		    values_gt[n_gt * 3 + 1] = msg->point.y;
		    values_gt[n_gt * 3 + 2] = msg->point.z;
		    n_gt += 1;
	    });

	// Wait for latched messages to be received
	float secs = 0.f;
	while ((n < 1 || n_gt < 1) && secs < 1.f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		secs += 0.001f;
	}
	ASSERT_LT(secs, 1.f) << "Messages not received within 1s";

	compare_vectors({ values_gt[0], values_gt[1], values_gt[2] },
	                { d->sensordata[adr] / cutoff, d->sensordata[adr + 1] / cutoff, d->sensordata[adr + 2] / cutoff },
	                0.0001, true);

	env_ptr->step(NUM_SAMPLES - 1);

	secs = 0.f;
	while ((n < NUM_SAMPLES || n_gt < NUM_SAMPLES) && secs < 15.f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		secs += 0.001f;
	}
	ASSERT_LT(secs, 15.f) << "Messages not received within 15s (got " << n << " and " << n_gt << " of " << NUM_SAMPLES
	                      << " expected, respectively)";

	// Map values to Eigen matrix for fast metrics computation
	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 3, Eigen::RowMajor>> values_map(values);
	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 3, Eigen::RowMajor>> values_gt_map(values_gt);
	// Compute means and variances
	auto diff                             = values_map - values_gt_map;
	Eigen::Matrix<double, 1, 3> means     = diff.colwise().mean();
	Eigen::Matrix<double, 1, 3> variances = (diff.rowwise() - diff.colwise().mean()).array().square().colwise().mean();

	EXPECT_NEAR(means(0), 0, 1e-3);
	EXPECT_NEAR(means(1), 1., 1e-3);
	EXPECT_NEAR(means(2), 0., 1e-3);

	EXPECT_NEAR(variances(0), 0.000625, 2e-5);
	EXPECT_NEAR(variances(1), 0., 5e-5);
	EXPECT_NEAR(variances(2), 0., 2e-5);

	// Compute means and variances
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

TEST_F(TrainEnvFixture, scalar_stamped)
{
	int n_sensor;
	getSensorByName("vel_joint2", m, n_sensor);

	int adr    = m->sensor_adr[n_sensor];
	int cutoff = (m->sensor_cutoff[n_sensor] > 0 ? m->sensor_cutoff[n_sensor] : 1);

	// Check if topics are published
	mujoco_ros_msgs::ScalarStampedConstPtr msgPtr_GT;
	msgPtr_GT = ros::topic::waitForMessage<mujoco_ros_msgs::ScalarStamped>("/vel_joint2_GT", ros::Duration(0.1));

	mujoco_ros_msgs::ScalarStampedConstPtr msgPtr;
	msgPtr = ros::topic::waitForMessage<mujoco_ros_msgs::ScalarStamped>("/vel_joint2", ros::Duration(0.1));

	EXPECT_TRUE(msgPtr != nullptr) << "Could not get message on /vel_joint2 topic!";
	EXPECT_TRUE(msgPtr_GT != nullptr) << "Could not get message on /vel_joint2_GT topic!";

	// Without noise should be the same
	EXPECT_NEAR(msgPtr_GT->value, msgPtr->value, 0.0001) << "Without noise sensor value should equal GT";

	mujoco_ros_msgs::SensorNoiseModel noise_model;
	noise_model.mean.emplace_back(1.0);
	noise_model.std.emplace_back(0.025);
	noise_model.set_flag    = 1;
	noise_model.sensor_name = "vel_joint2";

	mujoco_ros_msgs::RegisterSensorNoiseModels srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "example_hash";

	ros::ServiceClient client =
	    nh->serviceClient<mujoco_ros_msgs::RegisterSensorNoiseModels>("/sensors/register_noise_models");
	EXPECT_TRUE(client.call(srv)) << "Service call failed!";

	// Pause sim for synchronous message
	env_ptr->settings_.run.store(0);

	double values[NUM_SAMPLES]    = { 0 };
	int n                         = 0;
	double values_gt[NUM_SAMPLES] = { 0 };
	int n_gt                      = 0;

	ros::Subscriber sub = nh->subscribe<mujoco_ros_msgs::ScalarStamped>(
	    "/vel_joint2", NUM_SAMPLES, [&](const mujoco_ros_msgs::ScalarStamped::ConstPtr &msg) {
		    values[n] = msg->value;
		    n += 1;
	    });

	ros::Subscriber sub_gt = nh->subscribe<mujoco_ros_msgs::ScalarStamped>(
	    "/vel_joint2_GT", NUM_SAMPLES, [&](const mujoco_ros_msgs::ScalarStamped::ConstPtr &msg) {
		    values_gt[n_gt] = msg->value;
		    n_gt += 1;
	    });

	// Wait for latched messages to be received
	float secs = 0.f;
	while ((n < 1 || n_gt < 1) && secs < 1.f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		secs += 0.001f;
	}
	ASSERT_LT(secs, 1.f) << "Messages not received within 1s";

	// GT == Sensor reading
	EXPECT_NEAR(values_gt[0], d->sensordata[adr] / cutoff, 0.0001) << "GT differs from actual sensor value";

	env_ptr->step(NUM_SAMPLES - 1);

	secs = 0.f;
	while ((n < NUM_SAMPLES || n_gt < NUM_SAMPLES) && secs < 15.f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		secs += 0.001f;
	}
	ASSERT_LT(secs, 15.f) << "Messages not received within 15s (got " << n << " and " << n_gt << " of " << NUM_SAMPLES
	                      << " expected, respectively)";

	// Map values to Eigen matrix for fast metrics computation
	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 1>> values_map(values);
	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 1>> values_gt_map(values_gt);

	// Compute means and variances
	auto diff       = values_map - values_gt_map;
	double mean     = diff.colwise().mean()(0);
	double variance = (diff.rowwise() - diff.colwise().mean()).array().square().colwise().mean()(0);

	EXPECT_NEAR(mean, 1, 0.01);
	EXPECT_NEAR(variance, 0.000625, 0.0001);

	// Map values to Eigen matrix for fast metrics computation
	double mean_gt     = values_gt_map.colwise().mean()(0);
	double variance_gt = (values_gt_map.rowwise() - values_gt_map.colwise().mean()).array().square().colwise().mean()(0);

	EXPECT_NEAR(mean_gt, d->sensordata[adr] / cutoff, 0.0001);
	EXPECT_EQ(variance_gt, 0);
}

TEST_F(TrainEnvFixture, quaternion)
{
	int n_sensor;
	getSensorByName("immovable_quat", m, n_sensor);

	int adr    = m->sensor_adr[n_sensor];
	int cutoff = (m->sensor_cutoff[n_sensor] > 0 ? m->sensor_cutoff[n_sensor] : 1);

	// Check if topics are published
	geometry_msgs::QuaternionStampedConstPtr msgPtr_GT;
	msgPtr_GT = ros::topic::waitForMessage<geometry_msgs::QuaternionStamped>("/immovable_quat_GT", ros::Duration(0.1));

	geometry_msgs::QuaternionStampedConstPtr msgPtr;
	msgPtr = ros::topic::waitForMessage<geometry_msgs::QuaternionStamped>("/immovable_quat", ros::Duration(0.1));

	EXPECT_TRUE(msgPtr != nullptr) << "Could not get message on /immovable_pos topic!";
	EXPECT_TRUE(msgPtr_GT != nullptr) << "Could not get message on /immovable_pos_GT topic!";

	// Without noise should be the same
	compare_vectors(
	    { msgPtr_GT->quaternion.w, msgPtr_GT->quaternion.x, msgPtr_GT->quaternion.y, msgPtr_GT->quaternion.z },
	    { msgPtr->quaternion.w, msgPtr->quaternion.x, msgPtr->quaternion.y, msgPtr->quaternion.z }, 0.0001, true);

	mujoco_ros_msgs::SensorNoiseModel noise_model;
	noise_model.mean.emplace_back(0.0);
	noise_model.mean.emplace_back(0.0);
	noise_model.mean.emplace_back(1.0);
	noise_model.std.emplace_back(0.0);
	noise_model.std.emplace_back(0.0);
	noise_model.std.emplace_back(0.025);
	noise_model.set_flag    = 7;
	noise_model.sensor_name = "immovable_quat";

	mujoco_ros_msgs::RegisterSensorNoiseModels srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "example_hash";

	ros::ServiceClient client =
	    nh->serviceClient<mujoco_ros_msgs::RegisterSensorNoiseModels>("/sensors/register_noise_models");
	EXPECT_TRUE(client.call(srv)) << "Service call failed!";

	// Pause sim for synchronous message
	env_ptr->settings_.run.store(0);

	double values[3 * NUM_SAMPLES]    = { 0 };
	int n                             = 0;
	double values_gt[3 * NUM_SAMPLES] = { 0 };
	int n_gt                          = 0;

	std::vector<double> gt_quat;

	ros::Subscriber sub = nh->subscribe<geometry_msgs::QuaternionStamped>(
	    "/immovable_quat", NUM_SAMPLES, [&values, &n](const geometry_msgs::QuaternionStamped::ConstPtr &msg) {
		    tf2::Quaternion q;
		    tf2::fromMsg(msg->quaternion, q);
		    auto m = tf2::Matrix3x3(q);
		    m.getRPY(values[n * 3], values[n * 3 + 1], values[n * 3 + 2]);
		    n += 1;
	    });

	ros::Subscriber sub_gt = nh->subscribe<geometry_msgs::QuaternionStamped>(
	    "/immovable_quat_GT", NUM_SAMPLES,
	    [&values_gt, &gt_quat, &n_gt](const geometry_msgs::QuaternionStamped::ConstPtr &msg) {
		    if (n_gt == 0) {
			    gt_quat = { msg->quaternion.w, msg->quaternion.x, msg->quaternion.y, msg->quaternion.z };
		    }
		    tf2::Quaternion q;
		    tf2::fromMsg(msg->quaternion, q);
		    auto m = tf2::Matrix3x3(q);
		    m.getRPY(values_gt[n_gt * 3], values_gt[n_gt * 3 + 1], values_gt[n_gt * 3 + 2]);
		    n_gt += 1;
	    });

	// Wait for latched messages to be received
	float secs = 0.f;
	while ((n < 1 || n_gt < 1) && secs < 1.f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		secs += 0.001f;
	}
	ASSERT_LT(secs, 1.f) << "Messages not received within 1s";

	// GT == Sensor reading
	compare_vectors(gt_quat,
	                { d->sensordata[adr] / cutoff, d->sensordata[adr + 1] / cutoff, d->sensordata[adr + 2] / cutoff,
	                  d->sensordata[adr + 3] / cutoff },
	                0.0001, true);

	env_ptr->step(NUM_SAMPLES - 1);

	secs = 0.f;
	while ((n < NUM_SAMPLES || n_gt < NUM_SAMPLES) && secs < 15.f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		secs += 0.001f;
	}
	ASSERT_LT(secs, 15.f) << "Messages not received within 15s (got " << n << " and " << n_gt << " of " << NUM_SAMPLES
	                      << " expected, respectively)";

	// Map values to Eigen matrix for fast metrics computation
	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 3, Eigen::RowMajor>> values_map(values);
	Eigen::Map<Eigen::Matrix<double, NUM_SAMPLES, 3, Eigen::RowMajor>> values_gt_map(values_gt);

	// Compute means and variances
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
	mujoco_ros_msgs::SensorNoiseModel noise_model;
	noise_model.mean.emplace_back(0.0);
	noise_model.mean.emplace_back(1.0);
	noise_model.std.emplace_back(0.025);
	noise_model.std.emplace_back(0.0);
	noise_model.set_flag    = 3;
	noise_model.sensor_name = "vel_EE";

	mujoco_ros_msgs::RegisterSensorNoiseModels srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "some_wrong_hash";

	ros::ServiceClient client =
	    nh->serviceClient<mujoco_ros_msgs::RegisterSensorNoiseModels>("/sensors/register_noise_models");
	EXPECT_TRUE(client.call(srv)) << "Service call failed!";

	EXPECT_EQ(srv.response.success, false) << "Service call should have failed!";
}

TEST_F(EvalEnvFixture, AllowedEvalNoiseModelChange)
{
	mujoco_ros_msgs::SensorNoiseModel noise_model;
	noise_model.mean.emplace_back(0.0);
	noise_model.mean.emplace_back(1.0);
	noise_model.std.emplace_back(0.025);
	noise_model.std.emplace_back(0.0);
	noise_model.set_flag    = 3;
	noise_model.sensor_name = "vel_EE";

	mujoco_ros_msgs::RegisterSensorNoiseModels srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "some_hash";

	ros::ServiceClient client =
	    nh->serviceClient<mujoco_ros_msgs::RegisterSensorNoiseModels>("/sensors/register_noise_models");
	EXPECT_TRUE(client.call(srv)) << "Service call failed!";

	EXPECT_EQ(srv.response.success, true) << "Service call should have succeeded!";
}

TEST_F(TrainEnvFixture, UnknownSensorAddNoise)
{
	mujoco_ros_msgs::SensorNoiseModel noise_model;
	noise_model.mean.emplace_back(0.0);
	noise_model.mean.emplace_back(1.0);
	noise_model.std.emplace_back(0.025);
	noise_model.std.emplace_back(0.0);
	noise_model.set_flag    = 3;
	noise_model.sensor_name = "unknown_sensor";

	mujoco_ros_msgs::RegisterSensorNoiseModels srv;
	srv.request.noise_models.emplace_back(noise_model);
	srv.request.admin_hash = "example_hash";

	ros::ServiceClient client =
	    nh->serviceClient<mujoco_ros_msgs::RegisterSensorNoiseModels>("/sensors/register_noise_models");
	EXPECT_TRUE(client.call(srv)) << "Service call failed!";

	EXPECT_EQ(srv.response.success, true) << "Service call should have succeeded!";
}
