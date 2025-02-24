/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2024, Bielefeld University
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

#pragma once

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <mujoco_ros/common_types.hpp>
#include <mujoco_ros/mujoco_env.hpp>
// #include <mujoco_ros/logging.hpp>

// #include <mujoco_ros_msgs/RegisterSensorNoiseModels.h>
#include <mujoco_ros_msgs/srv/register_sensor_noise_models.hpp>
#include <mujoco_ros_msgs/msg/scalar_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
// #include <mujoco_ros/ros_one/plugin_utils.h>
#include <mujoco_ros/ros_two/plugin_utils.hpp>

// #include <mujoco_ros_msgs/srv/register_sensor_noise_models.hpp>
// #include <geometry_msgs/msg/vector3_stamped.hpp>
// #include <geometry_msgs/msg/quaternion_stamped.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>
// #include <mujoco_ros_msgs/msg/scalar_stamped.hpp>

#include <random>

namespace mujoco_ros::sensors {

struct SensorConfig
{
public:
	SensorConfig() : frame_id(""){};
	SensorConfig(std::string frame_id) : frame_id(std::move(frame_id)){};

	void setFrameId(const std::string &frame_id) { this->frame_id = frame_id; };

	// Not sure if having it as a & would break it once it goes out of the loop...
	// need to check this after compiling

	// void registerPub(const ros::Publisher &pub) { value_pub = pub; };
	// void registerPub(rclcpp::GenericPublisher::SharedPtr &pub) { value_pub = pub; };

	// void registerGTPub(const ros::Publisher &pub) { gt_pub = pub; };
	// void registerGTPub(rclcpp::GenericPublisher::SharedPtr &pub) { gt_pub = pub; };

	template<typename M> rclcpp::SerializedMessage serializeMessage(const M &msg){
		rclcpp::Serialization<M> serializer;
		rclcpp::SerializedMessage serialized_msg;
		serializer.serialize_message(&msg, &serialized_msg);
		return serialized_msg;
	};

	std::string frame_id;

	// ros::Publisher gt_pub;
	// ros::Publisher value_pub;
	rclcpp::GenericPublisher::SharedPtr gt_pub;
	rclcpp::GenericPublisher::SharedPtr value_pub;

	// Noise params
	double mean[3];
	double sigma[3];

	uint8_t is_set = 0; // 0 for unset, otherwise binary code for combination of dims
};

struct LidarConfig{
	sensor_msgs::msg::LaserScan msg_;
	double max_;
	double min_;
	int freq_;
	int rf_count_;
	double angle_;
	double angle_to_rotate_;
	double accumulated_angle_;
	std::vector<int> sensor_ids_;
	int body_id_;
	bool valid_ = false;
};

using SensorConfigPtr = std::unique_ptr<SensorConfig>;
using LidarConfigPtr = std::unique_ptr<LidarConfig>;

class MujocoRos2SensorsPlugin : public mujoco_ros::MujocoPlugin
{
public:
	~MujocoRos2SensorsPlugin() override;

	// Overload entry point
	bool Load(const mjModel *m, mjData *d) override; // load -> Load

	void Reset() override; // reset -> Reset

	void LastStageCallback(const mjModel *model, mjData *data) override; // last -> Last

	mujoco_ros::CallbackReturn on_configure(const rclcpp_lifecycle::State &/*previous_state*/){
        // RCLCPP_INFO_STREAM(get_my_logger(), "Configuring DummyRos2Plugin");

        declare_parameter_if_not_declared(
            this->get_node()->get_node_parameters_interface(),
            "test_name",
            rclcpp::ParameterValue("parallel_node_name")
        );

		if (!this->get_node()->has_parameter("lidars")) {
			this->get_node()->declare_parameter<std::vector<std::string>>("lidars", std::vector<std::string>());
		}

        return mujoco_ros::CallbackReturn::SUCCESS;
    }

private:
	// replaced via env_ptr_
	rclcpp_lifecycle::LifecycleNode::SharedPtr sensors_nh_;


	void initSensors(const mjModel *model, mjData *data);
	std::mt19937 rand_generator = std::mt19937(std::random_device{}());
	std::normal_distribution<double> noise_dist;

	std::map<std::string, SensorConfigPtr> sensor_map_;
	std::map<std::string, LidarConfigPtr> lidar_map_;

	// ros::ServiceServer register_noise_model_server_;
	rclcpp::Service<mujoco_ros_msgs::srv::RegisterSensorNoiseModels>::SharedPtr register_noise_model_server_;

	// bool registerNoiseModelsCB(mujoco_ros_msgs::RegisterSensorNoiseModels::Request &req,
	//                            mujoco_ros_msgs::RegisterSensorNoiseModels::Response &rep);
	void registerNoiseModelsCB(const mujoco_ros_msgs::srv::RegisterSensorNoiseModels::Request::SharedPtr &req,
							   const mujoco_ros_msgs::srv::RegisterSensorNoiseModels::Response::SharedPtr &rep);
	void configureLidarMap(const mjModel *model, mjData *data);
	void publishLidarData(const mjModel* model, mjData *data);

	static rclcpp::Logger getLogger(){
		return rclcpp::get_logger("MujocoRos2SensorPlugin");
	};
};

const char *SENSOR_STRING[37];

} // namespace mujoco_ros::sensors
