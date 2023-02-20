/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Bielefeld University
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

#include <mujoco_ros/mujoco_sim.h>
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

#include <ros/ros.h>
#include <ros/package.h>
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_ros_test_node");

	ros::AsyncSpinner spinner(4);
	spinner.start();
	ROS_DEBUG("Started spinner");
	int ret = RUN_ALL_TESTS();
	spinner.stop();
	ROS_DEBUG("Stopped spinner");
	return ret;
}

namespace unit_testing {

class MujocoRosTrainFixture : public ::testing::Test
{
protected:
	std::shared_ptr<ros::NodeHandle> nh;
	MujocoSim::MujocoEnvPtr env;
	MujocoSim::mjDataPtr d;
	MujocoSim::mjModelPtr m;
	std::unique_ptr<std::thread> mj_thread;

	virtual void SetUp()
	{
		nh.reset(new ros::NodeHandle("~"));
		nh->setParam("eval_mode", false);
		nh->setParam("unpause", true);
		nh->setParam("no_x", true);
		nh->setParam("use_sim_time", true);

		std::string xml_path = ros::package::getPath("mujoco_ros_sensors") + "/test/sensors_world.xml";

		mj_thread = std::unique_ptr<std::thread>(new std::thread(MujocoSim::init, xml_path, ""));
		while (MujocoSim::detail::settings_.loadrequest.load() == 0) { // wait for request to be made
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		while (MujocoSim::detail::settings_.loadrequest.load() > 0) { // wait for model to be loaded
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		env = MujocoSim::detail::main_env_;
		d   = env->data;
		m   = env->model;
	}

	virtual void TearDown()
	{
		MujocoSim::requestExternalShutdown();
		mj_thread->join();
	}
};

class MujocoRosEvalFixture : public ::testing::Test
{
protected:
	std::shared_ptr<ros::NodeHandle> nh;
	MujocoSim::MujocoEnvPtr env;
	MujocoSim::mjDataPtr d;
	MujocoSim::mjModelPtr m;
	std::unique_ptr<std::thread> mj_thread;

	virtual void SetUp()
	{
		nh.reset(new ros::NodeHandle("~"));
		nh->setParam("eval_mode", true);
		nh->setParam("unpause", true);
		nh->setParam("no_x", true);
		nh->setParam("use_sim_time", true);

		std::string xml_path = ros::package::getPath("mujoco_ros_sensors") + "/test/sensors_world.xml";

		mj_thread = std::unique_ptr<std::thread>(new std::thread(MujocoSim::init, xml_path, "example_hash"));
		while (MujocoSim::detail::settings_.loadrequest.load() == 0) { // wait for request to be made
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		while (MujocoSim::detail::settings_.loadrequest.load() > 0) { // wait for model to be loaded
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		env = MujocoSim::detail::main_env_;
		d   = env->data;
		m   = env->model;
	}

	virtual void TearDown()
	{
		MujocoSim::requestExternalShutdown();
		mj_thread->join();
	}
};

TEST_F(MujocoRosTrainFixture, sensor_created_train)
{
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	std::string sensor_name;
	bool found_val_top, found_gt_top;

	for (int n = 0; n < m->nsensor; n++) {
		found_gt_top  = false;
		found_val_top = false;

		if (m->names[m->name_sensoradr[n]]) {
			sensor_name = mj_id2name(m.get(), mjOBJ_SENSOR, n);
		} else {
			continue;
		}

		for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++) {
			const ros::master::TopicInfo &info = *it;
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

TEST_F(MujocoRosEvalFixture, sensor_created_eval)
{
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	std::string sensor_name;

	int adr, type;
	mjtNum cutoff;
	bool found_val_top, found_gt_top;

	for (int n = 0; n < m->nsensor; n++) {
		adr           = m->sensor_adr[n];
		type          = m->sensor_type[n];
		cutoff        = (m->sensor_cutoff[n] > 0 ? m->sensor_cutoff[n] : 1);
		found_gt_top  = false;
		found_val_top = false;

		if (m->names[m->name_sensoradr[n]]) {
			sensor_name = mj_id2name(m.get(), mjOBJ_SENSOR, n);
		} else {
			continue;
		}

		for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++) {
			const ros::master::TopicInfo &info = *it;
			// ROS_INFO_STREAM("Found topic " << info.name);
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

void getSensorByName(const std::string sensor_name, MujocoSim::mjModelPtr m, int &n_sensor)
{
	std::string name;

	for (int n = 0; n < m->nsensor; n++) {
		if (m->names[m->name_sensoradr[n]]) {
			name = mj_id2name(m.get(), mjOBJ_SENSOR, n);
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

TEST_F(MujocoRosTrainFixture, sensor_3DOF)
{
	int n_sensor;
	getSensorByName("vel_EE", m, n_sensor);

	int adr    = m->sensor_adr[n_sensor];
	int type   = m->sensor_type[n_sensor];
	int cutoff = (m->sensor_cutoff[n_sensor] > 0 ? m->sensor_cutoff[n_sensor] : 1);

	// Check if topics are published
	geometry_msgs::Vector3StampedConstPtr msgPtr_GT;
	msgPtr_GT = ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("/vel_EE_GT", ros::Duration(0.1));

	geometry_msgs::Vector3StampedConstPtr msgPtr;
	msgPtr = ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("/vel_EE", ros::Duration(0.1));

	EXPECT_TRUE(msgPtr != NULL) << "Could not get message on /vel_EE topic!";
	EXPECT_TRUE(msgPtr_GT != NULL) << "Could not get message on /vel_EE_GT topic!";

	// Without noise should be the same
	compare_vectors({ msgPtr_GT->vector.x, msgPtr_GT->vector.y, msgPtr_GT->vector.z },
	                { msgPtr->vector.x, msgPtr->vector.y, msgPtr->vector.z }, 0.0001, true);

	mujoco_ros_msgs::SensorNoiseModel noise_model;
	noise_model.mean.push_back(0.0);
	noise_model.mean.push_back(1.0);
	noise_model.std.push_back(0.25);
	noise_model.std.push_back(0.0);
	noise_model.set_flag    = 3;
	noise_model.sensor_name = "vel_EE";

	mujoco_ros_msgs::RegisterSensorNoiseModels srv;
	srv.request.noise_models.push_back(noise_model);
	srv.request.admin_hash = "example_hash";

	ros::ServiceClient client =
	    nh->serviceClient<mujoco_ros_msgs::RegisterSensorNoiseModels>("/sensors/register_noise_models");
	EXPECT_TRUE(client.call(srv)) << "Service call failed!";

	// Pause sim for synchronous message
	MujocoSim::detail::settings_.run.store(0);

	msgPtr    = ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("/vel_EE");
	msgPtr_GT = ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("/vel_EE_GT");

	// GT == Sensor reading
	compare_vectors({ msgPtr_GT->vector.x, msgPtr_GT->vector.y, msgPtr_GT->vector.z },
	                { d->sensordata[adr] / cutoff, d->sensordata[adr + 1] / cutoff, d->sensordata[adr + 2] / cutoff },
	                0.0001, true);

	int n               = 0;
	double means[6]     = { 0.0 };
	double deltas[6]    = { 0.0 };
	double variances[6] = { 0.0 };

	for (int i = 0; i <= 1000; i++) {
		MujocoSim::detail::settings_.manual_env_steps.store(1);

		while (MujocoSim::detail::settings_.manual_env_steps.load() != 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		msgPtr    = ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("/vel_EE");
		msgPtr_GT = ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("/vel_EE_GT");

		n += 1;
		deltas[0] = (msgPtr->vector.x - msgPtr_GT->vector.x) - means[0];
		deltas[1] = (msgPtr->vector.y - msgPtr_GT->vector.y) - means[1];
		deltas[2] = (msgPtr->vector.z - msgPtr_GT->vector.z) - means[2];

		deltas[3] = msgPtr_GT->vector.x - means[3];
		deltas[4] = msgPtr_GT->vector.y - means[4];
		deltas[5] = msgPtr_GT->vector.z - means[5];

		means[0] += deltas[0] / n;
		means[1] += deltas[1] / n;
		means[2] += deltas[2] / n;

		means[3] += deltas[3] / n;
		means[4] += deltas[4] / n;
		means[5] += deltas[5] / n;

		variances[0] += deltas[0] * ((msgPtr->vector.x - msgPtr_GT->vector.x) - means[0]);
		variances[1] += deltas[1] * ((msgPtr->vector.y - msgPtr_GT->vector.y) - means[1]);
		variances[2] += deltas[2] * ((msgPtr->vector.z - msgPtr_GT->vector.z) - means[2]);

		variances[3] += deltas[3] * (msgPtr_GT->vector.x - means[3]);
		variances[4] += deltas[4] * (msgPtr_GT->vector.y - means[4]);
		variances[5] += deltas[5] * (msgPtr_GT->vector.z - means[5]);
	}

	variances[0] /= n - 1;
	variances[1] /= n - 1;
	variances[2] /= n - 1;
	variances[3] /= n - 1;
	variances[4] /= n - 1;
	variances[5] /= n - 1;

	EXPECT_NEAR(means[0], 0, 0.02);
	EXPECT_EQ(means[1], 1);
	EXPECT_EQ(means[2], 0);

	EXPECT_NEAR(means[3], d->sensordata[adr] / cutoff, 0.0001);
	EXPECT_NEAR(means[4], d->sensordata[adr + 1] / cutoff, 0.0001);
	EXPECT_NEAR(means[5], d->sensordata[adr + 2] / cutoff, 0.0001);

	EXPECT_NEAR(variances[0], 0.0625, 0.01);
	EXPECT_EQ(variances[1], 0);
	EXPECT_EQ(variances[2], 0);

	EXPECT_EQ(variances[3], 0);
	EXPECT_EQ(variances[4], 0);
	EXPECT_EQ(variances[5], 0);
}

TEST_F(MujocoRosTrainFixture, framepos)
{
	int n_sensor;
	getSensorByName("immovable_pos", m, n_sensor);

	int adr    = m->sensor_adr[n_sensor];
	int type   = m->sensor_type[n_sensor];
	int cutoff = (m->sensor_cutoff[n_sensor] > 0 ? m->sensor_cutoff[n_sensor] : 1);

	// Check if topics are published
	geometry_msgs::PointStampedConstPtr msgPtr_GT;
	msgPtr_GT = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/immovable_pos_GT", ros::Duration(0.1));

	geometry_msgs::PointStampedConstPtr msgPtr;
	msgPtr = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/immovable_pos", ros::Duration(0.1));

	EXPECT_TRUE(msgPtr != NULL) << "Could not get message on /immovable_pos topic!";
	EXPECT_TRUE(msgPtr_GT != NULL) << "Could not get message on /immovable_pos_GT topic!";

	// Without noise should be the same
	compare_vectors({ msgPtr_GT->point.x, msgPtr_GT->point.y, msgPtr_GT->point.z },
	                { msgPtr->point.x, msgPtr->point.y, msgPtr->point.z }, 0.0001, true);

	mujoco_ros_msgs::SensorNoiseModel noise_model;
	noise_model.mean.push_back(0.0);
	noise_model.mean.push_back(1.0);
	noise_model.std.push_back(0.25);
	noise_model.std.push_back(0.0);
	noise_model.set_flag    = 3;
	noise_model.sensor_name = "immovable_pos";

	mujoco_ros_msgs::RegisterSensorNoiseModels srv;
	srv.request.noise_models.push_back(noise_model);
	srv.request.admin_hash = "example_hash";

	ros::ServiceClient client =
	    nh->serviceClient<mujoco_ros_msgs::RegisterSensorNoiseModels>("/sensors/register_noise_models");
	EXPECT_TRUE(client.call(srv)) << "Service call failed!";
	// sensor_plugin->registerNoiseModelsCB(srv.request, srv.response);

	// Pause sim for synchronous message
	MujocoSim::detail::settings_.run.store(0);

	msgPtr    = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/immovable_pos");
	msgPtr_GT = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/immovable_pos_GT");

	// GT == Sensor reading
	compare_vectors({ msgPtr_GT->point.x, msgPtr_GT->point.y, msgPtr_GT->point.z },
	                { d->sensordata[adr] / cutoff, d->sensordata[adr + 1] / cutoff, d->sensordata[adr + 2] / cutoff },
	                0.0001, true);

	int n               = 0;
	double means[6]     = { 0.0 };
	double deltas[6]    = { 0.0 };
	double variances[6] = { 0.0 };

	for (int i = 0; i <= 1000; i++) {
		MujocoSim::detail::settings_.manual_env_steps.store(1);

		while (MujocoSim::detail::settings_.manual_env_steps.load() != 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		msgPtr    = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/immovable_pos");
		msgPtr_GT = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/immovable_pos_GT");

		n += 1;
		deltas[0] = (msgPtr->point.x - msgPtr_GT->point.x) - means[0];
		deltas[1] = (msgPtr->point.y - msgPtr_GT->point.y) - means[1];
		deltas[2] = (msgPtr->point.z - msgPtr_GT->point.z) - means[2];

		deltas[3] = msgPtr_GT->point.x - means[3];
		deltas[4] = msgPtr_GT->point.y - means[4];
		deltas[5] = msgPtr_GT->point.z - means[5];

		means[0] += deltas[0] / n;
		means[1] += deltas[1] / n;
		means[2] += deltas[2] / n;

		means[3] += deltas[3] / n;
		means[4] += deltas[4] / n;
		means[5] += deltas[5] / n;

		variances[0] += deltas[0] * ((msgPtr->point.x - msgPtr_GT->point.x) - means[0]);
		variances[1] += deltas[1] * ((msgPtr->point.y - msgPtr_GT->point.y) - means[1]);
		variances[2] += deltas[2] * ((msgPtr->point.z - msgPtr_GT->point.z) - means[2]);

		variances[3] += deltas[3] * (msgPtr_GT->point.x - means[3]);
		variances[4] += deltas[4] * (msgPtr_GT->point.y - means[4]);
		variances[5] += deltas[5] * (msgPtr_GT->point.z - means[5]);
	}

	variances[0] /= n - 1;
	variances[1] /= n - 1;
	variances[2] /= n - 1;
	variances[3] /= n - 1;
	variances[4] /= n - 1;
	variances[5] /= n - 1;

	EXPECT_NEAR(means[0], 0, 0.02);
	EXPECT_NEAR(means[1], 1., 0.0001);
	EXPECT_EQ(means[2], 0);

	EXPECT_NEAR(means[3], d->sensordata[adr] / cutoff, 0.0001);
	EXPECT_NEAR(means[4], d->sensordata[adr + 1] / cutoff, 0.0001);
	EXPECT_NEAR(means[5], d->sensordata[adr + 2] / cutoff, 0.0001);

	EXPECT_NEAR(variances[0], 0.0625, 0.01);
	EXPECT_EQ(variances[1], 0);
	EXPECT_EQ(variances[2], 0);

	EXPECT_EQ(variances[3], 0);
	EXPECT_EQ(variances[4], 0);
	EXPECT_EQ(variances[5], 0);
}

TEST_F(MujocoRosTrainFixture, scalar_stamped)
{
	int n_sensor;
	getSensorByName("vel_joint2", m, n_sensor);

	int adr    = m->sensor_adr[n_sensor];
	int type   = m->sensor_type[n_sensor];
	int cutoff = (m->sensor_cutoff[n_sensor] > 0 ? m->sensor_cutoff[n_sensor] : 1);

	// Check if topics are published
	mujoco_ros_msgs::ScalarStampedConstPtr msgPtr_GT;
	msgPtr_GT = ros::topic::waitForMessage<mujoco_ros_msgs::ScalarStamped>("/vel_joint2_GT", ros::Duration(0.1));

	mujoco_ros_msgs::ScalarStampedConstPtr msgPtr;
	msgPtr = ros::topic::waitForMessage<mujoco_ros_msgs::ScalarStamped>("/vel_joint2", ros::Duration(0.1));

	EXPECT_TRUE(msgPtr != NULL) << "Could not get message on /vel_joint2 topic!";
	EXPECT_TRUE(msgPtr_GT != NULL) << "Could not get message on /vel_joint2_GT topic!";

	// Without noise should be the same
	EXPECT_NEAR(msgPtr_GT->value, msgPtr->value, 0.0001) << "Without noise sensor value should equal GT";

	mujoco_ros_msgs::SensorNoiseModel noise_model;
	noise_model.mean.push_back(1.0);
	noise_model.std.push_back(0.25);
	noise_model.set_flag    = 1;
	noise_model.sensor_name = "vel_joint2";

	mujoco_ros_msgs::RegisterSensorNoiseModels srv;
	srv.request.noise_models.push_back(noise_model);
	srv.request.admin_hash = "example_hash";

	ros::ServiceClient client =
	    nh->serviceClient<mujoco_ros_msgs::RegisterSensorNoiseModels>("/sensors/register_noise_models");
	EXPECT_TRUE(client.call(srv)) << "Service call failed!";

	// Pause sim for synchronous message
	MujocoSim::detail::settings_.run.store(0);

	msgPtr_GT = ros::topic::waitForMessage<mujoco_ros_msgs::ScalarStamped>("/vel_joint2_GT");
	msgPtr    = ros::topic::waitForMessage<mujoco_ros_msgs::ScalarStamped>("/vel_joint2");

	// GT == Sensor reading
	EXPECT_NEAR(msgPtr_GT->value, d->sensordata[adr] / cutoff, 0.0001) << "GT differs from actual sensor value";

	int n               = 0;
	double means[2]     = { 0.0 };
	double deltas[2]    = { 0.0 };
	double variances[2] = { 0.0 };

	for (int i = 0; i <= 1000; i++) {
		MujocoSim::detail::settings_.manual_env_steps.store(1);

		while (MujocoSim::detail::settings_.manual_env_steps.load() != 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		msgPtr_GT = ros::topic::waitForMessage<mujoco_ros_msgs::ScalarStamped>("/vel_joint2_GT");
		msgPtr    = ros::topic::waitForMessage<mujoco_ros_msgs::ScalarStamped>("/vel_joint2");

		n += 1;
		deltas[0] = (msgPtr->value - msgPtr_GT->value) - means[0];
		deltas[1] = msgPtr_GT->value - means[1];

		means[0] += deltas[0] / n;
		means[1] += deltas[1] / n;

		variances[0] += deltas[0] * ((msgPtr->value - msgPtr_GT->value) - means[0]);
		variances[1] += deltas[1] * (msgPtr_GT->value - means[1]);
	}

	variances[0] /= n - 1;
	variances[1] /= n - 1;

	EXPECT_NEAR(means[0], 1, 0.01);
	EXPECT_NEAR(means[1], d->sensordata[adr] / cutoff, 0.0001);

	EXPECT_NEAR(variances[0], 0.0625, 0.01);
	EXPECT_EQ(variances[1], 0);
}

TEST_F(MujocoRosTrainFixture, quaternion)
{
	int n_sensor;
	getSensorByName("immovable_quat", m, n_sensor);

	int adr    = m->sensor_adr[n_sensor];
	int type   = m->sensor_type[n_sensor];
	int cutoff = (m->sensor_cutoff[n_sensor] > 0 ? m->sensor_cutoff[n_sensor] : 1);

	// Check if topics are published
	geometry_msgs::QuaternionStampedConstPtr msgPtr_GT;
	msgPtr_GT = ros::topic::waitForMessage<geometry_msgs::QuaternionStamped>("/immovable_quat_GT", ros::Duration(0.1));

	geometry_msgs::QuaternionStampedConstPtr msgPtr;
	msgPtr = ros::topic::waitForMessage<geometry_msgs::QuaternionStamped>("/immovable_quat", ros::Duration(0.1));

	EXPECT_TRUE(msgPtr != NULL) << "Could not get message on /immovable_pos topic!";
	EXPECT_TRUE(msgPtr_GT != NULL) << "Could not get message on /immovable_pos_GT topic!";

	// Without noise should be the same
	compare_vectors(
	    { msgPtr_GT->quaternion.w, msgPtr_GT->quaternion.x, msgPtr_GT->quaternion.y, msgPtr_GT->quaternion.z },
	    { msgPtr->quaternion.w, msgPtr->quaternion.x, msgPtr->quaternion.y, msgPtr->quaternion.z }, 0.0001, true);

	mujoco_ros_msgs::SensorNoiseModel noise_model;
	noise_model.mean.push_back(1.0);
	noise_model.std.push_back(0.25);
	noise_model.set_flag    = 4;
	noise_model.sensor_name = "immovable_quat";

	mujoco_ros_msgs::RegisterSensorNoiseModels srv;
	srv.request.noise_models.push_back(noise_model);
	srv.request.admin_hash = "example_hash";

	ros::ServiceClient client =
	    nh->serviceClient<mujoco_ros_msgs::RegisterSensorNoiseModels>("/sensors/register_noise_models");
	EXPECT_TRUE(client.call(srv)) << "Service call failed!";

	// Pause sim for synchronous message
	MujocoSim::detail::settings_.run.store(0);

	msgPtr    = ros::topic::waitForMessage<geometry_msgs::QuaternionStamped>("/immovable_quat");
	msgPtr_GT = ros::topic::waitForMessage<geometry_msgs::QuaternionStamped>("/immovable_quat_GT");

	// GT == Sensor reading
	compare_vectors(
	    { msgPtr_GT->quaternion.w, msgPtr_GT->quaternion.x, msgPtr_GT->quaternion.y, msgPtr_GT->quaternion.z },
	    { d->sensordata[adr] / cutoff, d->sensordata[adr + 1] / cutoff, d->sensordata[adr + 2] / cutoff,
	      d->sensordata[adr + 3] / cutoff },
	    0.0001, true);

	int n               = 0;
	double means[6]     = { 0.0 };
	double deltas[6]    = { 0.0 };
	double variances[6] = { 0.0 };

	tf2::Quaternion q, q_GT;
	tf2::Matrix3x3 m;
	double r, p, y, R, P, Y;

	for (int i = 0; i <= 1000; i++) {
		MujocoSim::detail::settings_.manual_env_steps.store(1);

		while (MujocoSim::detail::settings_.manual_env_steps.load() != 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		msgPtr    = ros::topic::waitForMessage<geometry_msgs::QuaternionStamped>("/immovable_quat");
		msgPtr_GT = ros::topic::waitForMessage<geometry_msgs::QuaternionStamped>("/immovable_quat_GT");

		n += 1;

		tf2::fromMsg(msgPtr->quaternion, q);
		tf2::fromMsg(msgPtr_GT->quaternion, q_GT);

		m = tf2::Matrix3x3(q);
		m.getRPY(r, p, y);
		m = tf2::Matrix3x3(q_GT);
		m.getRPY(R, P, Y);

		deltas[0] = (r - R) - means[0];
		deltas[1] = (p - P) - means[1];
		deltas[2] = (y - Y) - means[2];

		means[0] += deltas[0] / n;
		means[1] += deltas[1] / n;
		means[2] += deltas[2] / n;

		variances[0] += deltas[0] * ((r - R) - means[0]);
		variances[1] += deltas[1] * ((p - P) - means[1]);
		variances[2] += deltas[2] * ((y - Y) - means[2]);

		deltas[3] = R - means[3];
		deltas[4] = P - means[4];
		deltas[5] = Y - means[5];

		means[3] += deltas[3] / n;
		means[4] += deltas[4] / n;
		means[5] += deltas[5] / n;

		variances[3] += deltas[3] * (R - means[3]);
		variances[4] += deltas[4] * (P - means[4]);
		variances[5] += deltas[5] * (Y - means[5]);
	}

	variances[0] /= n - 1;
	variances[1] /= n - 1;
	variances[2] /= n - 1;
	variances[3] /= n - 1;
	variances[4] /= n - 1;
	variances[5] /= n - 1;

	EXPECT_EQ(means[0], 0);
	EXPECT_EQ(means[1], 0);
	EXPECT_NEAR(means[2], 1, 0.01);

	EXPECT_NEAR(means[3], R, 0.0001);
	EXPECT_NEAR(means[4], P, 0.0001);
	EXPECT_NEAR(means[5], Y, 0.0001);

	EXPECT_EQ(variances[0], 0);
	EXPECT_EQ(variances[1], 0);
	EXPECT_NEAR(variances[2], 0.0625, 0.01);

	EXPECT_EQ(variances[3], 0);
	EXPECT_EQ(variances[4], 0);
	EXPECT_EQ(variances[5], 0);
}

} // namespace unit_testing
