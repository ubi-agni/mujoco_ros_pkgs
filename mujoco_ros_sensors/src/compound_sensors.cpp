/**
 * Software License Agreement (BSD 3-Clause License)
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
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
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
 */

/* Authors: David P. Leins */

#include <mujoco_ros_sensors/sensors.h>
#include <mujoco_ros_sensors/serialization.h>

#include <mujoco/mujoco.h>

namespace mujoco_ros::sensors {

void RosSensorInterfaceBase::unregisterPublishers(bool eval_mode)
{
	value_pub_.shutdown();
	if (!eval_mode) {
		gt_pub_.shutdown();
	}
}

/////////// Wrench ///////////

WrenchSensor::WrenchSensor(const std::string &sensor_name, std::vector<RosSensorInterfaceBase *> &wrench_sensors,
                           bool eval_mode, ros::NodeHandle *sensors_nh)
    : RosSensorInterfaceBase("world", sensor_name, 0)
{
	value_pub_ = sensors_nh->advertise<geometry_msgs::WrenchStamped>(sensor_name, 1);
	if (!eval_mode) {
		gt_pub_ = sensors_nh->advertise<geometry_msgs::WrenchStamped>(sensor_name + "_GT", 1);
	}

	frame_id_ = wrench_sensors[0]->getFrameId();
	for (const auto &sensor : wrench_sensors) {
		if (frame_id_ != sensor->getFrameId()) {
			ROS_ERROR_STREAM("Wrench sensor " << sensor_name << " has sensors with different frame ids. Expected "
			                                  << frame_id_ << ", but got " << sensor->getFrameId() << " instead");
			throw std::runtime_error("Invalid wrench sensor configuration.");
		}
		if (sensor->getType() == mjtSensor::mjSENS_FORCE) {
			if (force_ != nullptr) {
				ROS_ERROR_STREAM("Wrench sensor " << sensor_name << " has multiple force sensors.");
				throw std::runtime_error("Invalid wrench sensor configuration.");
			}
			force_ = dynamic_cast<RosSensorInterface<geometry_msgs::Vector3Stamped> *>(sensor);
		} else if (sensor->getType() == mjtSensor::mjSENS_TORQUE) {
			if (torque_ != nullptr) {
				ROS_ERROR_STREAM("Wrench sensor " << sensor_name << " has multiple torque sensors.");
				throw std::runtime_error("Invalid wrench sensor configuration.");
			}
			torque_ = dynamic_cast<RosSensorInterface<geometry_msgs::Vector3Stamped> *>(sensor);
		} else {
			ROS_ERROR_STREAM("Wrench sensor " << sensor_name << " has an invalid sensor type ("
			                                  << SENSOR_STRING[sensor->getType()] << ").");
			throw std::runtime_error("Invalid wrench sensor configuration.");
		}
	}
}

void WrenchSensor::publish(const bool publish_gt, const mjData *data, std::normal_distribution<double> &dist,
                           std::mt19937 &gen)
{
	geometry_msgs::WrenchStamped msg;
	msg.header.stamp    = ros::Time::now();
	msg.header.frame_id = frame_id_;

	auto f = force_->serialize(data);
	auto t = torque_->serialize(data);

	msg.wrench.force  = f.vector;
	msg.wrench.torque = t.vector;

	if (publish_gt) {
		gt_pub_.publish(msg);
	}

	force_->addNoise(f, dist, gen);
	torque_->addNoise(t, dist, gen);

	value_pub_.publish(msg);
}

/////////// Twist ///////////

TwistSensor::TwistSensor(const std::string &sensor_name, std::vector<RosSensorInterfaceBase *> &twist_sensors,
                         bool eval_mode, ros::NodeHandle *sensors_nh)
    : RosSensorInterfaceBase("world", sensor_name, 0)
{
	value_pub_ = sensors_nh->advertise<geometry_msgs::TwistStamped>(sensor_name, 1);
	if (!eval_mode) {
		gt_pub_ = sensors_nh->advertise<geometry_msgs::TwistStamped>(sensor_name + "_GT", 1);
	}

	frame_id_ = twist_sensors[0]->getFrameId();
	for (const auto &sensor : twist_sensors) {
		if (frame_id_ != sensor->getFrameId()) {
			ROS_ERROR_STREAM("Twist sensor " << sensor_name << " has sensors with different frame ids. Expected "
			                                 << frame_id_ << ", but got " << sensor->getFrameId() << " instead");
			throw std::runtime_error("Invalid twist sensor configuration.");
		}
		if (sensor->getType() == mjtSensor::mjSENS_FRAMELINVEL) {
			if (linear_velocity_ != nullptr) {
				ROS_ERROR_STREAM("Twist sensor " << sensor_name << " has multiple linear velocity sensors.");
				throw std::runtime_error("Invalid twist sensor configuration.");
			}
			linear_velocity_ = dynamic_cast<RosSensorInterface<geometry_msgs::Vector3Stamped> *>(sensor);
		} else if (sensor->getType() == mjtSensor::mjSENS_FRAMEANGVEL) {
			if (angular_velocity_ != nullptr) {
				ROS_ERROR_STREAM("Twist sensor " << sensor_name << " has multiple angular velocity sensors.");
				throw std::runtime_error("Invalid twist sensor configuration.");
			}
			angular_velocity_ = dynamic_cast<RosSensorInterface<geometry_msgs::Vector3Stamped> *>(sensor);
		} else {
			ROS_ERROR_STREAM("Twist sensor " << sensor_name << " has an invalid sensor type ("
			                                 << SENSOR_STRING[sensor->getType()] << ").");
			throw std::runtime_error("Invalid twist sensor configuration.");
		}
	}
}

void TwistSensor::publish(const bool publish_gt, const mjData *data, std::normal_distribution<double> &dist,
                          std::mt19937 &gen)
{
	geometry_msgs::TwistStamped msg;
	msg.header.stamp    = ros::Time::now();
	msg.header.frame_id = frame_id_;

	auto lv = linear_velocity_->serialize(data);
	auto av = angular_velocity_->serialize(data);

	msg.twist.linear  = lv.vector;
	msg.twist.angular = av.vector;

	if (publish_gt) {
		gt_pub_.publish(msg);
	}

	linear_velocity_->addNoise(lv, dist, gen);
	angular_velocity_->addNoise(av, dist, gen);

	value_pub_.publish(msg);
}

/////////// Pose ///////////

PoseSensor::PoseSensor(const std::string &sensor_name, std::vector<RosSensorInterfaceBase *> &pose_sensors,
                       bool eval_mode, ros::NodeHandle *sensors_nh)
    : RosSensorInterfaceBase("world", sensor_name, 0)
{
	value_pub_ = sensors_nh->advertise<geometry_msgs::PoseStamped>(sensor_name, 1);
	if (!eval_mode) {
		gt_pub_ = sensors_nh->advertise<geometry_msgs::PoseStamped>(sensor_name + "_GT", 1);
	}

	frame_id_ = pose_sensors[0]->getFrameId();
	for (const auto &sensor : pose_sensors) {
		if (frame_id_ != sensor->getFrameId()) {
			ROS_ERROR_STREAM("Pose sensor " << sensor_name << " has sensors with different frame ids. Expected "
			                                << frame_id_ << ", but got " << sensor->getFrameId() << " instead");
			throw std::runtime_error("Invalid pose sensor configuration.");
		}
		if (sensor->getType() == mjtSensor::mjSENS_FRAMEPOS) {
			if (position_ != nullptr) {
				ROS_ERROR_STREAM("Pose sensor " << sensor_name << " has multiple position sensors.");
				throw std::runtime_error("Invalid pose sensor configuration.");
			}
			position_ = dynamic_cast<RosSensorInterface<geometry_msgs::PointStamped> *>(sensor);
		} else if (sensor->getType() == mjtSensor::mjSENS_FRAMEQUAT) {
			if (orientation_ != nullptr) {
				ROS_ERROR_STREAM("Pose sensor " << sensor_name << " has multiple orientation sensors.");
				throw std::runtime_error("Invalid pose sensor configuration.");
			}
			orientation_ = dynamic_cast<RosSensorInterface<geometry_msgs::QuaternionStamped> *>(sensor);
		} else {
			ROS_ERROR_STREAM("Pose sensor " << sensor_name << " has an invalid sensor type ("
			                                << SENSOR_STRING[sensor->getType()] << ").");
			throw std::runtime_error("Invalid pose sensor configuration.");
		}
	}
}

void PoseSensor::publish(const bool publish_gt, const mjData *data, std::normal_distribution<double> &dist,
                         std::mt19937 &gen)
{
	geometry_msgs::PoseStamped msg;
	msg.header.stamp    = ros::Time::now();
	msg.header.frame_id = frame_id_;

	auto p = position_->serialize(data);
	auto o = orientation_->serialize(data);

	msg.pose.position    = p.point;
	msg.pose.orientation = o.quaternion;

	if (publish_gt) {
		gt_pub_.publish(msg);
	}

	position_->addNoise(p, dist, gen);
	orientation_->addNoise(o, dist, gen);

	value_pub_.publish(msg);
}

/////////// IMU ///////////

IMUSensor::IMUSensor(const std::string &sensor_name, std::vector<RosSensorInterfaceBase *> &imu_sensors, bool eval_mode,
                     ros::NodeHandle *sensors_nh)
    : RosSensorInterfaceBase("world", sensor_name, 0)
{
	value_pub_ = sensors_nh->advertise<sensor_msgs::Imu>(sensor_name, 1);
	if (!eval_mode) {
		gt_pub_ = sensors_nh->advertise<sensor_msgs::Imu>(sensor_name + "_GT", 1);
	}

	if (imu_sensors.size() != 3) {
		ROS_ERROR_STREAM("IMU sensor " << sensor_name << " has " << imu_sensors.size()
		                               << " sensors, but exactly 3 are required.");
		throw std::runtime_error("Invalid IMU sensor configuration.");
	}

	frame_id_ = imu_sensors[0]->getFrameId();
	for (const auto &sensor : imu_sensors) {
		if (frame_id_ != sensor->getFrameId()) {
			ROS_ERROR_STREAM("IMU sensor " << sensor_name << " has sensors with different frame ids. Expected "
			                               << frame_id_ << ", but got " << sensor->getFrameId() << " instead");
			throw std::runtime_error("Invalid IMU sensor configuration.");
		}
		if (sensor->getType() == mjtSensor::mjSENS_FRAMELINACC || sensor->getType() == mjtSensor::mjSENS_ACCELEROMETER) {
			if (linear_acceleration_ != nullptr) {
				ROS_ERROR_STREAM("IMU sensor " << sensor_name << " has multiple linear acceleration sensors.");
				throw std::runtime_error("Invalid IMU sensor configuration.");
			}
			linear_acceleration_ = dynamic_cast<RosSensorInterface<geometry_msgs::Vector3Stamped> *>(sensor);
		} else if (sensor->getType() == mjtSensor::mjSENS_FRAMEANGACC || sensor->getType() == mjtSensor::mjSENS_GYRO ||
		           sensor->getType() == mjtSensor::mjSENS_BALLANGVEL) {
			if (angular_acceleration_ != nullptr) {
				ROS_ERROR_STREAM("IMU sensor " << sensor_name << " has multiple angular acceleration sensors.");
				throw std::runtime_error("Invalid IMU sensor configuration.");
			}
			angular_acceleration_ = dynamic_cast<RosSensorInterface<geometry_msgs::Vector3Stamped> *>(sensor);
		} else if (sensor->getType() == mjtSensor::mjSENS_FRAMEQUAT || sensor->getType() == mjtSensor::mjSENS_BALLQUAT) {
			if (orientation_ != nullptr) {
				ROS_ERROR_STREAM("IMU sensor " << sensor_name << " has multiple orientation sensors.");
				throw std::runtime_error("Invalid IMU sensor configuration.");
			}
			orientation_ = dynamic_cast<RosSensorInterface<geometry_msgs::QuaternionStamped> *>(sensor);
		} else {
			ROS_ERROR_STREAM("IMU sensor " << sensor_name << " has an invalid sensor type ("
			                               << SENSOR_STRING[sensor->getType()] << ").");
			throw std::runtime_error("Invalid IMU sensor configuration.");
		}
	}
}

void IMUSensor::publish(const bool publish_gt, const mjData *data, std::normal_distribution<double> &dist,
                        std::mt19937 &gen)
{
	sensor_msgs::Imu msg;
	msg.header.stamp    = ros::Time::now();
	msg.header.frame_id = frame_id_;

	auto la = linear_acceleration_->serialize(data);
	auto aa = angular_acceleration_->serialize(data);
	auto o  = orientation_->serialize(data);

	msg.linear_acceleration = la.vector;
	msg.angular_velocity    = aa.vector;
	msg.orientation         = o.quaternion;

	if (publish_gt) {
		gt_pub_.publish(msg);
	}

	linear_acceleration_->addNoise(la, dist, gen);
	angular_acceleration_->addNoise(aa, dist, gen);
	orientation_->addNoise(o, dist, gen);

	value_pub_.publish(msg);
}

} // namespace mujoco_ros::sensors
