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

#pragma once

#include <mujoco/mujoco.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

#include <mujoco_ros_sensors/serialization.h>
#include <mujoco_ros_msgs/ScalarStamped.h>

#include <ros/publisher.h>
#include <ros/node_handle.h>

#include <random>

namespace mujoco_ros::sensors {

class RosSensorInterfaceBase
{
public:
	RosSensorInterfaceBase(const std::string &frame_id, const std::string &sensor_name, const int dofs)
	    : frame_id_(frame_id), sensor_name_(sensor_name), dofs_(dofs)
	{
		if (dofs_ == 0)
			return;

		mu_noise_    = new mjtNum[dofs];
		sigma_noise_ = new mjtNum[dofs];
	};

	void setNoise(const mjtNum *mu_noise, const mjtNum *sigma_noise, uint8_t noise_set);

	~RosSensorInterfaceBase()
	{
		if (dofs_ == 0)
			return;

		delete[] mu_noise_;
		delete[] sigma_noise_;
	};

	virtual void publish(const bool publish_gt, const mjData *data, std::normal_distribution<double> &dist,
	                     std::mt19937 &gen) = 0;

	void unregisterPublishers(bool eval_mode);

	int getType() const { return type_; }

	std::string getFrameId() const { return frame_id_; }

protected:
	std::string frame_id_;
	std::string sensor_name_;

	ros::Time last_pub_;
	ros::Publisher value_pub_;
	ros::Publisher gt_pub_;

	mjtNum *mu_noise_;
	mjtNum *sigma_noise_;

	uint dofs_;
	int type_ = -1;

	uint8_t noise_set_ = 0; // 0 for unset, otherwise binary code for set dimensions
};

template <typename T>
class RosSensorInterface : public RosSensorInterfaceBase
{
public:
	RosSensorInterface(const std::string &frame_id, const std::string &sensor_name, const int dofs, const int type,
	                   const int adr, const mjtNum cutoff, bool eval_mode, ros::NodeHandle *sensors_nh);

	void publish(const bool publish_gt, const mjData *data, std::normal_distribution<double> &dist,
	             std::mt19937 &gen) override;
	T serialize(const mjData *data);
	void addNoise(T &msg, std::normal_distribution<double> &dist, std::mt19937 &gen);

private:
	int adr_;
	mjtNum cutoff_;
};

class WrenchSensor : public RosSensorInterfaceBase
{
public:
	WrenchSensor(const std::string &sensor_name, std::vector<RosSensorInterfaceBase *> &wrench_sensors, bool eval_mode,
	             ros::NodeHandle *sensors_nh);

	void publish(const bool publish_gt, const mjData *data, std::normal_distribution<double> &dist,
	             std::mt19937 &gen) override;

private:
	RosSensorInterface<geometry_msgs::Vector3Stamped> *force_  = nullptr;
	RosSensorInterface<geometry_msgs::Vector3Stamped> *torque_ = nullptr;
};

class TwistSensor : public RosSensorInterfaceBase
{
public:
	TwistSensor(const std::string &sensor_name, std::vector<RosSensorInterfaceBase *> &twist_sensors, bool eval_mode,
	            ros::NodeHandle *sensors_nh);

	void publish(const bool publish_gt, const mjData *data, std::normal_distribution<double> &dist,
	             std::mt19937 &gen) override;

private:
	RosSensorInterface<geometry_msgs::Vector3Stamped> *linear_velocity_  = nullptr;
	RosSensorInterface<geometry_msgs::Vector3Stamped> *angular_velocity_ = nullptr;
};

class PoseSensor : public RosSensorInterfaceBase
{
public:
	PoseSensor(const std::string &sensor_name, std::vector<RosSensorInterfaceBase *> &pose_sensors, bool eval_mode,
	           ros::NodeHandle *sensors_nh);

	void publish(const bool publish_gt, const mjData *data, std::normal_distribution<double> &dist,
	             std::mt19937 &gen) override;

private:
	RosSensorInterface<geometry_msgs::PointStamped> *position_         = nullptr;
	RosSensorInterface<geometry_msgs::QuaternionStamped> *orientation_ = nullptr;
};

class IMUSensor : public RosSensorInterfaceBase
{
public:
	IMUSensor(const std::string &sensor_name, std::vector<RosSensorInterfaceBase *> &imu_sensors, bool eval_mode,
	          ros::NodeHandle *sensors_nh);

	void publish(const bool publish_gt, const mjData *data, std::normal_distribution<double> &dist,
	             std::mt19937 &gen) override;

private:
	RosSensorInterface<geometry_msgs::Vector3Stamped> *linear_acceleration_  = nullptr;
	RosSensorInterface<geometry_msgs::Vector3Stamped> *angular_acceleration_ = nullptr;
	RosSensorInterface<geometry_msgs::QuaternionStamped> *orientation_       = nullptr;

	mjtNum lin_cov_[9];
	mjtNum ang_cov_[9];
	mjtNum ori_cov_[9];
};

// template impl needs to be in header
template <typename T>
void RosSensorInterface<T>::publish(const bool publish_gt, const mjData *data, std::normal_distribution<double> &dist,
                                    std::mt19937 &gen)
{
	T msg = serialize(data);
	if (publish_gt) {
		gt_pub_.publish(msg);
	}
	addNoise(msg, dist, gen);
	value_pub_.publish(msg);
}

template <typename T>
RosSensorInterface<T>::RosSensorInterface(const std::string &frame_id, const std::string &sensor_name, const int dofs,
                                          const int type, const int adr, const mjtNum cutoff, bool eval_mode,
                                          ros::NodeHandle *sensors_nh)
    : RosSensorInterfaceBase(frame_id, sensor_name, dofs), adr_(adr), cutoff_(cutoff)
{
	value_pub_ = sensors_nh->advertise<T>(sensor_name, 1);
	if (!eval_mode) {
		gt_pub_ = sensors_nh->advertise<T>(sensor_name + "_GT", 1);
	}
	type_ = type;
};

template <typename T>
T RosSensorInterface<T>::serialize(const mjData *data)
{
	T msg;
	mujoco_ros::sensors::serialize(msg, data, frame_id_, adr_, cutoff_);
	return msg;
}

template <typename T>
void RosSensorInterface<T>::addNoise(T &msg, std::normal_distribution<double> &dist, std::mt19937 &gen)
{
	mujoco_ros::sensors::addNoise(msg, mu_noise_, sigma_noise_, noise_set_, dist, gen);
}

extern const char *SENSOR_STRING[35];

} // namespace mujoco_ros::sensors
