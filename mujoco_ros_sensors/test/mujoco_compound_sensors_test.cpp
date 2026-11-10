/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#include <gtest/gtest.h>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/package.h>
#include <ros/ros.h>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

#include <chrono>
#include <map>
#include <memory>
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
		env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
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
	ExpectLoadFails({ { "c",
	                    "IMU",
	                    { { "linear_acceleration", "acc_a" }, { "angular_velocity", "gyro_b" }, { "orientation", "quat_a" } } } });
}

TEST_F(CompoundLoadFixture, ImuOrientationOnOtherBodyFailsLoad)
{
	ExpectLoadFails({ { "c",
	                    "IMU",
	                    { { "linear_acceleration", "acc_a" }, { "angular_velocity", "gyro_a" }, { "orientation", "quat_b" } } } });
}

TEST_F(CompoundLoadFixture, ImuBallquatOnOtherBodyFailsLoad)
{
	ExpectLoadFails({ { "c",
	                    "IMU",
	                    { { "linear_acceleration", "acc_a" }, { "angular_velocity", "gyro_a" }, { "orientation", "ballquat_b" } } } });
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
#endif
