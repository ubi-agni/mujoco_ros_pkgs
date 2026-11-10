/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#include <gtest/gtest.h>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros_sensors/sensor_stream.hpp>
#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/package.h>
#include <ros/ros.h>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <mujoco_ros_msgs/srv/register_sensor_noise_models.hpp>
#include <mujoco_ros_msgs/msg/sensor_noise_model.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace mujoco_ros;

namespace {

struct CompoundParams
{
	std::string name;
	std::string type;
	std::map<std::string, std::string> roles;
};

std::string compound_world_path()
{
#if MJR_ROS_VERSION == ROS_1
	return ros::package::getPath("mujoco_ros_sensors") + "/test/sensors_compound_world.xml";
#else
	return ament_index_cpp::get_package_share_directory("mujoco_ros_sensors") + "/test/sensors_compound_world.xml";
#endif
}

class CompoundLoadFixture : public ::testing::Test
{
protected:
	std::unique_ptr<testing::TestNodeHandle> nh;
	std::unique_ptr<MujocoEnvTestWrapper> env_ptr;

	// compound_sensors cannot be nested in ROS 2 params from this fixture; ROS 1 supplies them via launch rosparam.
	void Start(const std::vector<CompoundParams> &compounds)
	{
		nh = std::make_unique<testing::TestNodeHandle>("~");
		nh->setParam("eval_mode", false);
		nh->setParam("unpause", true);
		nh->setParam("no_render", true);
#if MJR_ROS_VERSION == ROS_1
		nh->setParam("use_sim_time", true);
		ASSERT_TRUE(compounds.empty()) << "ROS 1 fixture cannot inject compound_sensors";
#else
		nh->setParam("MujocoPlugins.names", std::vector<std::string>{ "mujoco_ros_sensors" });
		nh->setParam("MujocoPlugins.mujoco_ros_sensors.type", "mujoco_ros_sensors/MujocoRosSensorsPlugin");
		if (!compounds.empty()) {
			const std::string base = "MujocoPlugins.mujoco_ros_sensors.compound_sensors";
			std::vector<std::string> names;
			for (const auto &compound : compounds) {
				names.push_back(compound.name);
				// "base..type" is not a valid ROS 2 parameter name; an empty name can only appear in the array.
				if (compound.name.empty()) {
					continue;
				}
				nh->setParam(base + "." + compound.name + ".type", compound.type);
				for (const auto &[role, sensor] : compound.roles) {
					nh->setParam(base + "." + compound.name + "." + role, sensor);
				}
			}
			nh->setParam(base, names);
		}
#endif
		env_ptr                    = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
		const std::string xml_path = compound_world_path();
		env_ptr->StartWithXML(xml_path);

		float seconds = 0;
		while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001f;
		}
		ASSERT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";
	}

	void TearDown() override
	{
		if (env_ptr != nullptr) {
			env_ptr->shutdown();
		}
		if (nh != nullptr) {
#if MJR_ROS_VERSION == ROS_1
			ros::param::del(nh->getNamespace());
#else
			nh->clearNode();
#endif
		}
	}

	void ExpectLoadFails(const std::vector<CompoundParams> &compounds)
	{
		Start(compounds);
		EXPECT_EQ(env_ptr->GetNumCBReadyPlugins(), 0) << "plugin must not come up with an invalid compound config";
		EXPECT_TRUE(env_ptr->sensors().Native().Names().empty())
		    << "failed Load must not leave the provider registered on the hub";
	}

	void ExpectLoads(const std::vector<CompoundParams> &compounds)
	{
		Start(compounds);
		EXPECT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
		EXPECT_FALSE(env_ptr->sensors().Native().Names().empty());
	}
};

} // namespace

int main(int argc, char **argv)
{
#if MJR_ROS_VERSION == ROS_1
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_compound_sensors_test");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	const int ret = RUN_ALL_TESTS();
	spinner.stop();
	return ret;
#else
	rclcpp::init(argc, argv);
	::testing::InitGoogleTest(&argc, argv);
	const int ret = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return ret;
#endif
}

TEST_F(CompoundLoadFixture, AbsentCompoundSensorsLoads)
{
	ExpectLoads({});
}

#if MJR_ROS_VERSION == ROS_2
TEST_F(CompoundLoadFixture, ValidCompoundsLoad)
{
	ExpectLoads({
	    { "wrench", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } },
	    { "twist", "Twist", { { "linear", "linvel_a" }, { "angular", "angvel_a" } } },
	    { "accel", "Accel", { { "linear", "linacc_a" }, { "angular", "angacc_a" } } },
	    // framequat keeps frame_id world; object identity, not the frame string, ties it to the anchor
	    { "imu",
	      "IMU",
	      { { "linear_acceleration", "acc_a" }, { "angular_velocity", "gyro_a" }, { "orientation", "quat_a" } } },
	});
}

TEST_F(CompoundLoadFixture, ValidPoseAndOdometryLoad)
{
	ExpectLoads({
	    { "pose", "Pose", { { "position", "pos_a" }, { "orientation", "quat_a" } } },
	    { "imu_b",
	      "IMU",
	      { { "linear_acceleration", "acc_b" }, { "angular_velocity", "gyro_b" }, { "orientation", "ballquat_b" } } },
	});
}

TEST_F(CompoundLoadFixture, UnknownTypeFailsLoad)
{
	ExpectLoadFails({ { "c", "IMUSensor", { { "force", "force_a" }, { "torque", "torque_a" } } } });
}

TEST_F(CompoundLoadFixture, MissingRoleFailsLoad)
{
	ExpectLoadFails({ { "c", "Wrench", { { "force", "force_a" } } } });
}

TEST_F(CompoundLoadFixture, UnknownRoleKeyFailsLoad)
{
	ExpectLoadFails({ { "c", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" }, { "bogus", "acc_a" } } } });
}

TEST_F(CompoundLoadFixture, UnknownSensorFailsLoad)
{
	ExpectLoadFails({ { "c", "Wrench", { { "force", "force_a" }, { "torque", "no_such_sensor" } } } });
}

TEST_F(CompoundLoadFixture, WrongMujocoTypeFailsLoad)
{
	// force sensor in the torque role
	ExpectLoadFails({ { "c", "Wrench", { { "force", "force_a" }, { "torque", "force_a" } } } });
}

TEST_F(CompoundLoadFixture, WrenchFrameMismatchFailsLoad)
{
	ExpectLoadFails({ { "c", "Wrench", { { "force", "force_a" }, { "torque", "torque_b" } } } });
}

TEST_F(CompoundLoadFixture, TwistFrameMismatchFailsLoad)
{
	ExpectLoadFails({ { "c", "Twist", { { "linear", "linvel_a" }, { "angular", "angvel_world" } } } });
}

TEST_F(CompoundLoadFixture, ImuAccelGyroFrameMismatchFailsLoad)
{
	ExpectLoadFails(
	    { { "c",
	        "IMU",
	        { { "linear_acceleration", "acc_a" }, { "angular_velocity", "gyro_b" }, { "orientation", "quat_a" } } } });
}

TEST_F(CompoundLoadFixture, ImuOrientationOnOtherBodyFailsLoad)
{
	ExpectLoadFails(
	    { { "c",
	        "IMU",
	        { { "linear_acceleration", "acc_a" }, { "angular_velocity", "gyro_a" }, { "orientation", "quat_b" } } } });
}

TEST_F(CompoundLoadFixture, ImuBallquatOnOtherBodyFailsLoad)
{
	ExpectLoadFails({ { "c",
	                    "IMU",
	                    { { "linear_acceleration", "acc_a" },
	                      { "angular_velocity", "gyro_a" },
	                      { "orientation", "ballquat_b" } } } });
}

TEST_F(CompoundLoadFixture, PoseOrientationOnOtherBodyFailsLoad)
{
	ExpectLoadFails({ { "c", "Pose", { { "position", "pos_a" }, { "orientation", "quat_b" } } } });
}

TEST_F(CompoundLoadFixture, OdometryOrientationOnOtherBodyFailsLoad)
{
	ExpectLoadFails({ { "c",
	                    "Odometry",
	                    { { "position", "pos_a" },
	                      { "orientation", "quat_b" },
	                      { "linear", "linvel_a" },
	                      { "angular", "angvel_a" } } } });
}

TEST_F(CompoundLoadFixture, DuplicateCompoundNameFailsLoad)
{
	ExpectLoadFails({ { "same", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } },
	                  { "same", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } } });
}

TEST_F(CompoundLoadFixture, EmptyCompoundNameFailsLoad)
{
	ExpectLoadFails({ { "", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } } });
}

TEST_F(CompoundLoadFixture, DoubleClaimFailsLoad)
{
	ExpectLoadFails({ { "first", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } },
	                  { "second", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } } });
}

TEST_F(CompoundLoadFixture, CompoundNameCollidingWithSensorFailsLoad)
{
	ExpectLoadFails({ { "acc_b", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } } });
}

namespace {

// The published sample comes from the last step; the model is re-forwarded afterwards, so reads drift by ~1 step.
constexpr double kRelTol = 2e-2;

#define EXPECT_SAMPLE(actual, expected) EXPECT_NEAR(actual, expected, kRelTol *std::max(1.0, std::abs(expected)))

class CompoundStreamFixture : public CompoundLoadFixture
{
protected:
	std::string TopicOf(const std::string &sensor) { return env_ptr->GetHandleNamespace() + "/" + sensor; }

	/// Sensor value as the native streams publish it (float cast, divided by the sensor cutoff).
	double Sensor(const std::string &name, int i = 0)
	{
		const mjModel *m = env_ptr->getModelPtr();
		const int id     = mj_name2id(m, mjOBJ_SENSOR, name.c_str());
		EXPECT_GE(id, 0) << name;
		const double cutoff = m->sensor_cutoff[id] > 0 ? m->sensor_cutoff[id] : 1;
		return static_cast<float>(env_ptr->getDataPtr()->sensordata[m->sensor_adr[id] + i] / cutoff);
	}

	template <typename MsgT>
	struct Latest
	{
		std::mutex mutex;
		MsgT msg;
		bool got = false;
	};

	/// Subscribes to the compound's value and GT topics while paused, runs a few steps, returns the messages of the
	/// final step. Sensors read velocities/accelerations of the pre-step state, so one step would only see zeros.
	template <typename MsgT>
	void ReceiveValueAndGt(const std::string &compound, MsgT &value, MsgT &gt, int steps = 20)
	{
		Latest<MsgT> v, g;
		auto make_cb = [](Latest<MsgT> &slot) {
			return [&slot](typename MsgT::ConstSharedPtr msg) {
				std::lock_guard<std::mutex> lock(slot.mutex);
				slot.msg = *msg;
				slot.got = true;
			};
		};
		env_ptr->togglePaused(true);
		auto value_sub = env_ptr->template create_subscription<MsgT>(TopicOf(compound), rclcpp::QoS(10), make_cb(v));
		auto gt_sub = env_ptr->template create_subscription<MsgT>(TopicOf(compound + "_GT"), rclcpp::QoS(10), make_cb(g));
		std::this_thread::sleep_for(std::chrono::milliseconds(250));
		env_ptr->step(steps);
		const double final_time = env_ptr->getDataPtr()->time;

		auto at_final = [&](Latest<MsgT> &slot) {
			std::lock_guard<std::mutex> lock(slot.mutex);
			return slot.got && std::abs(rclcpp::Time(slot.msg.header.stamp).seconds() - final_time) < 5e-4;
		};
		const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
		while (!(at_final(v) && at_final(g)) && std::chrono::steady_clock::now() < deadline) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		ASSERT_TRUE(at_final(v) && at_final(g))
		    << "no value/GT sample of the final step on compound '" << compound << "'";
		std::scoped_lock lock(v.mutex, g.mutex);
		value = v.msg;
		gt    = g.msg;
	}

	/// Compound public in the hub, claimed parts gone from hub and topic graph.
	void ExpectDemoted(const std::string &compound, const std::vector<std::string> &parts)
	{
		const auto native = env_ptr->sensors().Native();
		EXPECT_NE(native.Get(compound), nullptr);
		const auto names = native.Names();
		EXPECT_NE(std::find(names.begin(), names.end(), compound), names.end());
		const auto topics = testing::get_available_topics_for_test(env_ptr.get());
		EXPECT_TRUE(testing::has_topic(topics, TopicOf(compound))) << compound;
		for (const auto &part : parts) {
			EXPECT_EQ(native.Get(part), nullptr) << part << " must not be a public stream";
			EXPECT_EQ(std::find(names.begin(), names.end(), part), names.end()) << part;
			// Immediate check on purpose: claimed parts are never advertised, so nothing has to be torn down first.
			EXPECT_FALSE(testing::has_topic(topics, TopicOf(part))) << part << " must not be advertised";
			EXPECT_FALSE(testing::has_topic(topics, TopicOf(part + "_GT"))) << part << "_GT must not be advertised";
		}
	}
};

} // namespace

TEST_F(CompoundStreamFixture, WrenchCompoundPublishesAndDemotesParts)
{
	Start({ { "ee_wrench", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	ExpectDemoted("ee_wrench", { "force_a", "torque_a" });

	geometry_msgs::msg::WrenchStamped value, gt;
	ReceiveValueAndGt("ee_wrench", value, gt);
	EXPECT_EQ(value.header.frame_id, "link_a");
	EXPECT_EQ(gt.header.frame_id, "link_a");
	EXPECT_SAMPLE(gt.wrench.force.x, Sensor("force_a", 0));
	EXPECT_SAMPLE(gt.wrench.force.y, Sensor("force_a", 1));
	EXPECT_SAMPLE(gt.wrench.force.z, Sensor("force_a", 2));
	EXPECT_SAMPLE(gt.wrench.torque.x, Sensor("torque_a", 0));
	EXPECT_SAMPLE(gt.wrench.torque.y, Sensor("torque_a", 1));
	EXPECT_SAMPLE(gt.wrench.torque.z, Sensor("torque_a", 2));
	EXPECT_NE(gt.wrench.force.x, 0.0) << "force role must carry data, not a default message";
	// no noise registered: value equals GT
	EXPECT_EQ(value.wrench.force, gt.wrench.force);
	EXPECT_EQ(value.wrench.torque, gt.wrench.torque);
	EXPECT_NE(gt.wrench.torque.y, 0.0);
}

TEST_F(CompoundStreamFixture, TwistCompoundPublishesAndDemotesParts)
{
	// World-frame pair: a velocity measured relative to its own body would be 0 and prove nothing
	Start({ { "ee_twist", "Twist", { { "linear", "linvel_world" }, { "angular", "angvel_world" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	ExpectDemoted("ee_twist", { "linvel_world", "angvel_world" });

	geometry_msgs::msg::TwistStamped value, gt;
	ReceiveValueAndGt("ee_twist", value, gt);
	EXPECT_EQ(value.header.frame_id, "world");
	EXPECT_SAMPLE(gt.twist.linear.x, Sensor("linvel_world", 0));
	EXPECT_SAMPLE(gt.twist.linear.z, Sensor("linvel_world", 2));
	EXPECT_SAMPLE(gt.twist.angular.x, Sensor("angvel_world", 0));
	EXPECT_SAMPLE(gt.twist.angular.y, Sensor("angvel_world", 1));
	EXPECT_SAMPLE(gt.twist.angular.z, Sensor("angvel_world", 2));
	EXPECT_NE(gt.twist.angular.y, 0.0) << "angular role must carry data, not a default message";
	EXPECT_NE(gt.twist.linear.x, 0.0) << "linear role must carry data, not a default message";
	// no noise registered: the value channel must carry the same mapped fields as GT
	EXPECT_EQ(value.twist.linear, gt.twist.linear);
	EXPECT_EQ(value.twist.angular, gt.twist.angular);
}

TEST_F(CompoundStreamFixture, AccelCompoundPublishesAndDemotesParts)
{
	Start({ { "ee_accel", "Accel", { { "linear", "linacc_tip" }, { "angular", "angacc_a" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	ExpectDemoted("ee_accel", { "linacc_tip", "angacc_a" });

	geometry_msgs::msg::AccelStamped value, gt;
	ReceiveValueAndGt("ee_accel", value, gt);
	EXPECT_EQ(value.header.frame_id, "world");
	EXPECT_SAMPLE(gt.accel.linear.x, Sensor("linacc_tip", 0));
	EXPECT_SAMPLE(gt.accel.linear.z, Sensor("linacc_tip", 2));
	EXPECT_SAMPLE(gt.accel.angular.y, Sensor("angacc_a", 1));
	EXPECT_NE(gt.accel.angular.y, 0.0) << "angular role must carry data, not a default message";
	EXPECT_NE(gt.accel.linear.x, 0.0) << "linear role must carry data, not a default message";
	EXPECT_EQ(value.accel.linear, gt.accel.linear);
	EXPECT_EQ(value.accel.angular, gt.accel.angular);
}

TEST_F(CompoundStreamFixture, PoseCompoundUsesPositionFrameAndWorldQuaternion)
{
	// pos_b_in_a is body-relative (frame link_a); quat_b has frame_id world but measures the same body link_b
	Start({ { "ee_pose", "Pose", { { "position", "pos_b_in_a" }, { "orientation", "quat_b" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	ExpectDemoted("ee_pose", { "pos_b_in_a", "quat_b" });

	geometry_msgs::msg::PoseStamped value, gt;
	ReceiveValueAndGt("ee_pose", value, gt);
	EXPECT_EQ(value.header.frame_id, "link_a");
	EXPECT_SAMPLE(gt.pose.position.x, Sensor("pos_b_in_a", 0));
	EXPECT_SAMPLE(gt.pose.position.z, Sensor("pos_b_in_a", 2));
	EXPECT_SAMPLE(gt.pose.orientation.w, Sensor("quat_b", 0));
	EXPECT_SAMPLE(gt.pose.orientation.x, Sensor("quat_b", 1));
	EXPECT_SAMPLE(gt.pose.orientation.y, Sensor("quat_b", 2));
	EXPECT_SAMPLE(gt.pose.orientation.z, Sensor("quat_b", 3));
	EXPECT_NE(gt.pose.position.z, 0.0);
	EXPECT_EQ(value.pose.position, gt.pose.position);
	EXPECT_EQ(value.pose.orientation, gt.pose.orientation);
}

TEST_F(CompoundStreamFixture, ImuCompoundPublishesAndDemotesParts)
{
	// quat_a is a world-frame framequat on link_a: object identity ties it to the acc_a/gyro_a anchor
	Start({ { "ee_imu",
	          "IMU",
	          { { "linear_acceleration", "acc_a" }, { "angular_velocity", "gyro_a" }, { "orientation", "quat_a" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	ExpectDemoted("ee_imu", { "acc_a", "gyro_a", "quat_a" });

	sensor_msgs::msg::Imu value, gt;
	ReceiveValueAndGt("ee_imu", value, gt);
	EXPECT_EQ(value.header.frame_id, "link_a");
	EXPECT_EQ(gt.header.frame_id, "link_a");
	EXPECT_SAMPLE(gt.linear_acceleration.x, Sensor("acc_a", 0));
	EXPECT_SAMPLE(gt.linear_acceleration.z, Sensor("acc_a", 2));
	EXPECT_SAMPLE(gt.angular_velocity.x, Sensor("gyro_a", 0));
	EXPECT_SAMPLE(gt.angular_velocity.y, Sensor("gyro_a", 1));
	EXPECT_SAMPLE(gt.angular_velocity.z, Sensor("gyro_a", 2));
	EXPECT_SAMPLE(gt.orientation.w, Sensor("quat_a", 0));
	EXPECT_SAMPLE(gt.orientation.x, Sensor("quat_a", 1));
	EXPECT_SAMPLE(gt.orientation.y, Sensor("quat_a", 2));
	EXPECT_SAMPLE(gt.orientation.z, Sensor("quat_a", 3));
	EXPECT_NE(gt.linear_acceleration.z, 0.0) << "acceleration role must carry data, not a default message";
	EXPECT_NE(gt.angular_velocity.y, 0.0) << "gyro role must carry data, not a default message";
	EXPECT_NE(gt.orientation.y, 0.0) << "orientation role must carry data, not a default message";
	// no noise registered: value equals GT
	EXPECT_EQ(value.linear_acceleration, gt.linear_acceleration);
	EXPECT_EQ(value.angular_velocity, gt.angular_velocity);
	EXPECT_EQ(value.orientation, gt.orientation);
	// covariances unknown (first element -1), never filled from sigma
	for (const auto *msg : { &value, &gt }) {
		EXPECT_EQ(msg->orientation_covariance[0], -1.0);
		EXPECT_EQ(msg->angular_velocity_covariance[0], -1.0);
		EXPECT_EQ(msg->linear_acceleration_covariance[0], -1.0);
		for (std::size_t i = 1; i < 9; ++i) {
			EXPECT_EQ(msg->orientation_covariance[i], 0.0);
			EXPECT_EQ(msg->angular_velocity_covariance[i], 0.0);
			EXPECT_EQ(msg->linear_acceleration_covariance[i], 0.0);
		}
	}
}

TEST_F(CompoundStreamFixture, ImuBallquatOrientationPublishes)
{
	// ballquat_b measures joint ball_b of link_b, the same body as acc_b/gyro_b's site_b
	Start({ { "imu_b",
	          "IMU",
	          { { "linear_acceleration", "acc_b" },
	            { "angular_velocity", "gyro_b" },
	            { "orientation", "ballquat_b" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	ExpectDemoted("imu_b", { "acc_b", "gyro_b", "ballquat_b" });

	sensor_msgs::msg::Imu value, gt;
	ReceiveValueAndGt("imu_b", value, gt);
	EXPECT_EQ(value.header.frame_id, "link_b");
	EXPECT_SAMPLE(gt.orientation.w, Sensor("ballquat_b", 0));
	EXPECT_SAMPLE(gt.orientation.x, Sensor("ballquat_b", 1));
	EXPECT_SAMPLE(gt.orientation.y, Sensor("ballquat_b", 2));
	EXPECT_SAMPLE(gt.orientation.z, Sensor("ballquat_b", 3));
	EXPECT_SAMPLE(gt.linear_acceleration.z, Sensor("acc_b", 2));
}

TEST_F(CompoundStreamFixture, OdometryCompoundUsesPoseParentAndTwistChildFrame)
{
	// Pose pair in link_a (pos_b_in_a + quat_b, same body link_b); twist pair in world: parent and child differ
	Start({ { "ee_odom",
	          "Odometry",
	          { { "position", "pos_b_in_a" },
	            { "orientation", "quat_b" },
	            { "linear", "linvel_world" },
	            { "angular", "angvel_world" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	ExpectDemoted("ee_odom", { "pos_b_in_a", "quat_b", "linvel_world", "angvel_world" });

	nav_msgs::msg::Odometry value, gt;
	ReceiveValueAndGt("ee_odom", value, gt);
	EXPECT_EQ(value.header.frame_id, "link_a");
	EXPECT_EQ(gt.header.frame_id, "link_a");
	EXPECT_EQ(value.child_frame_id, "world");
	EXPECT_EQ(gt.child_frame_id, "world");
	EXPECT_SAMPLE(gt.pose.pose.position.x, Sensor("pos_b_in_a", 0));
	EXPECT_SAMPLE(gt.pose.pose.position.z, Sensor("pos_b_in_a", 2));
	EXPECT_SAMPLE(gt.pose.pose.orientation.w, Sensor("quat_b", 0));
	EXPECT_SAMPLE(gt.pose.pose.orientation.x, Sensor("quat_b", 1));
	EXPECT_SAMPLE(gt.pose.pose.orientation.y, Sensor("quat_b", 2));
	EXPECT_SAMPLE(gt.pose.pose.orientation.z, Sensor("quat_b", 3));
	EXPECT_SAMPLE(gt.twist.twist.linear.x, Sensor("linvel_world", 0));
	EXPECT_SAMPLE(gt.twist.twist.linear.z, Sensor("linvel_world", 2));
	EXPECT_SAMPLE(gt.twist.twist.angular.y, Sensor("angvel_world", 1));
	EXPECT_NE(gt.pose.pose.position.z, 0.0);
	EXPECT_NE(gt.twist.twist.linear.x, 0.0);
	EXPECT_NE(gt.twist.twist.angular.y, 0.0);
	EXPECT_EQ(value.pose.pose.position, gt.pose.pose.position);
	EXPECT_EQ(value.pose.pose.orientation, gt.pose.pose.orientation);
	EXPECT_EQ(value.twist.twist.linear, gt.twist.twist.linear);
	EXPECT_EQ(value.twist.twist.angular, gt.twist.twist.angular);
	for (const auto *msg : { &value, &gt }) {
		EXPECT_EQ(msg->pose.covariance[0], -1.0);
		EXPECT_EQ(msg->twist.covariance[0], -1.0);
		EXPECT_EQ(msg->pose.covariance[1], 0.0);
		EXPECT_EQ(msg->twist.covariance[35], 0.0);
	}
}

TEST_F(CompoundStreamFixture, InProcessReadDoesNotLeaveStalePartValuesForNextStep)
{
	// "a_wrench" sorts before its parts "force_a"/"torque_a". An in-process read after a step fills the parts' caches;
	// if LastStage did not invalidate every cache before publishing, the next step would compose that stale part
	// state and republish the previous sample.
	Start({ { "a_wrench", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	env_ptr->togglePaused(true);
	env_ptr->step(1);

	auto *stream = static_cast<mujoco_ros::sensors::ISensorStream *>(env_ptr->sensors().Native().Get("a_wrench"));
	ASSERT_NE(stream, nullptr);
	const auto read = std::any_cast<const geometry_msgs::msg::WrenchStamped &>(
	    stream->Value(env_ptr->getModelPtr(), env_ptr->getDataPtr()));

	geometry_msgs::msg::WrenchStamped next, next_gt;
	ReceiveValueAndGt("a_wrench", next, next_gt, 1);
	EXPECT_NE(next.wrench.force.z, read.wrench.force.z) << "value of the next step is the previous step's sample";
	EXPECT_NE(next_gt.wrench.force.z, read.wrench.force.z) << "GT of the next step is the previous step's sample";
}

namespace {

constexpr double kNoiseShift = 100.0;

class CompoundNoiseFixture : public CompoundStreamFixture
{
protected:
	/// Registers a zero-sigma mean shift of `kNoiseShift` on all three slots of `sensor_name`; returns the response.
	bool RegisterShift(const std::string &sensor_name) { return RegisterShifts({ sensor_name }); }

	/// One request carrying a shift entry per name, in order.
	bool RegisterShifts(const std::vector<std::string> &sensor_names)
	{
		testing::ServiceCall<mujoco_ros_msgs::srv::RegisterSensorNoiseModels> srv;
		for (const auto &sensor_name : sensor_names) {
			mujoco_ros_msgs::msg::SensorNoiseModel model;
			model.sensor_name = sensor_name;
			model.mean        = { kNoiseShift, kNoiseShift, kNoiseShift };
			model.std         = { 0.0, 0.0, 0.0 };
			model.set_flag    = 7;
			srv.request.noise_models.push_back(model);
		}
		srv.request.admin_hash = "";
		env_ptr->togglePaused(true);
		const std::string name = env_ptr->GetHandleNamespace() + "/mujoco_ros_sensors/sensors/register_noise_models";
		const auto deadline    = std::chrono::steady_clock::now() + std::chrono::seconds(2);
		while (!testing::service_exists_for_test(env_ptr.get(), name) && std::chrono::steady_clock::now() < deadline) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		EXPECT_TRUE(testing::service_call_for_test(env_ptr.get(), name, srv)) << "service call itself failed";
		return srv.response.success;
	}
};

} // namespace

TEST_F(CompoundNoiseFixture, WrenchRoleNoiseShiftsOnlyThatRoleAndNotGt)
{
	Start({ { "ee_wrench", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	EXPECT_TRUE(RegisterShift("ee_wrench.force"));

	geometry_msgs::msg::WrenchStamped value, gt;
	ReceiveValueAndGt("ee_wrench", value, gt);
	EXPECT_SAMPLE(gt.wrench.force.x, Sensor("force_a", 0));
	EXPECT_SAMPLE(gt.wrench.force.z, Sensor("force_a", 2));
	EXPECT_NEAR(value.wrench.force.x - gt.wrench.force.x, kNoiseShift, 1e-3);
	EXPECT_NEAR(value.wrench.force.y - gt.wrench.force.y, kNoiseShift, 1e-3);
	EXPECT_NEAR(value.wrench.force.z - gt.wrench.force.z, kNoiseShift, 1e-3);
	EXPECT_EQ(value.wrench.torque, gt.wrench.torque) << "torque role has no noise";
}

TEST_F(CompoundNoiseFixture, ImuRoleNoiseShiftsValueKeepsCovarianceUnknown)
{
	Start({ { "ee_imu",
	          "IMU",
	          { { "linear_acceleration", "acc_a" }, { "angular_velocity", "gyro_a" }, { "orientation", "quat_a" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	EXPECT_TRUE(RegisterShift("ee_imu.linear_acceleration"));

	sensor_msgs::msg::Imu value, gt;
	ReceiveValueAndGt("ee_imu", value, gt);
	EXPECT_NEAR(value.linear_acceleration.z - gt.linear_acceleration.z, kNoiseShift, 1e-3);
	EXPECT_SAMPLE(gt.linear_acceleration.z, Sensor("acc_a", 2));
	EXPECT_EQ(value.angular_velocity, gt.angular_velocity);
	EXPECT_EQ(value.orientation, gt.orientation);
	// noise must never leak into covariances
	for (const auto *msg : { &value, &gt }) {
		EXPECT_EQ(msg->linear_acceleration_covariance[0], -1.0);
		EXPECT_EQ(msg->angular_velocity_covariance[0], -1.0);
		EXPECT_EQ(msg->orientation_covariance[0], -1.0);
		for (std::size_t i = 1; i < 9; ++i) {
			EXPECT_EQ(msg->linear_acceleration_covariance[i], 0.0);
			EXPECT_EQ(msg->angular_velocity_covariance[i], 0.0);
			EXPECT_EQ(msg->orientation_covariance[i], 0.0);
		}
	}
}

TEST_F(CompoundNoiseFixture, UnknownCompoundOrRoleIsRejected)
{
	Start({ { "ee_wrench", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	EXPECT_FALSE(RegisterShift("ee_wrench.bogus"));
	EXPECT_FALSE(RegisterShift("ee_wrench."));
	EXPECT_FALSE(RegisterShift("ee_wrench.force.extra"));

	geometry_msgs::msg::WrenchStamped value, gt;
	ReceiveValueAndGt("ee_wrench", value, gt);
	EXPECT_EQ(value.wrench.force, gt.wrench.force);
	EXPECT_EQ(value.wrench.torque, gt.wrench.torque);
}

TEST_F(CompoundNoiseFixture, NoiseOnDemotedPartNameAloneDoesNotChangeCompound)
{
	Start({ { "ee_wrench", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	EXPECT_FALSE(RegisterShift("force_a")) << "part is no public noise target";

	geometry_msgs::msg::WrenchStamped value, gt;
	ReceiveValueAndGt("ee_wrench", value, gt);
	EXPECT_EQ(value.wrench.force, gt.wrench.force);
}

TEST_F(CompoundNoiseFixture, DottedNameWithoutCompoundPrefixTakesNativePath)
{
	// ROS topic names cannot contain '.', so a dotted MJCF sensor never gets a native stream and cannot be applied to
	// here. What is testable: with no compound "ft", "ft.force" is routed as a plain name (legacy unknown-sensor skip,
	// success) and not rejected as compound.role; "ee_wrench.bogus" has a known compound prefix and is rejected.
	Start({ { "ee_wrench", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	EXPECT_TRUE(RegisterShift("ft.force"));
	EXPECT_TRUE(RegisterShift("no_such.force"));
	EXPECT_FALSE(RegisterShift("ee_wrench.bogus"));

	geometry_msgs::msg::WrenchStamped value, gt;
	ReceiveValueAndGt("ee_wrench", value, gt);
	EXPECT_EQ(value.wrench.force, gt.wrench.force);
}

TEST_F(CompoundNoiseFixture, RejectedEntryAppliesNothingFromTheSameRequest)
{
	// all-or-nothing on rejection: the valid first entry must not apply when a later entry is rejected
	Start({ { "ee_wrench", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	EXPECT_FALSE(RegisterShifts({ "ee_wrench.force", "ee_wrench.bogus" }));

	geometry_msgs::msg::WrenchStamped value, gt;
	ReceiveValueAndGt("ee_wrench", value, gt);
	EXPECT_EQ(value.wrench.force, gt.wrench.force);
	EXPECT_EQ(value.wrench.torque, gt.wrench.torque);

	// the same valid entry alone is accepted afterwards
	EXPECT_TRUE(RegisterShifts({ "ee_wrench.force", "no_such_plain_sensor" })) << "unknown plain sensor stays a skip";
	ReceiveValueAndGt("ee_wrench", value, gt);
	EXPECT_NEAR(value.wrench.force.x - gt.wrench.force.x, kNoiseShift, 1e-3);
}

TEST_F(CompoundStreamFixture, UnclaimedSensorsStayPublic)
{
	Start({ { "ee_wrench", "Wrench", { { "force", "force_a" }, { "torque", "torque_a" } } } });
	ASSERT_EQ(env_ptr->GetNumCBReadyPlugins(), 1);
	const auto native = env_ptr->sensors().Native();
	EXPECT_NE(native.Get("acc_a"), nullptr);
	EXPECT_NE(native.Get("quat_b"), nullptr);
	const auto topics = testing::get_available_topics_for_test(env_ptr.get());
	EXPECT_TRUE(testing::has_topic(topics, TopicOf("acc_a")));
}
#endif
