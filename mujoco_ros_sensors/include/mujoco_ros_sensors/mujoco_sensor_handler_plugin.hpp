/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2025, Bielefeld University
 *  All rights reserved.
 */

#pragma once

#include <mujoco_ros/ros_version.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>

#include <mujoco_ros/ros_one/plugin_utils.hpp>
#include <mujoco_ros_msgs/RegisterSensorNoiseModels.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <mujoco_ros/ros_two/plugin_utils.hpp>
#include <mujoco_ros_msgs/msg/scalar_stamped.hpp>
#include <mujoco_ros_msgs/srv/register_sensor_noise_models.hpp>
#endif

#include <mujoco_ros/mujoco_env.hpp>

#include <any>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>

namespace mujoco_ros::sensors {

struct SensorConfig
{
public:
	SensorConfig() : frame_id("") {}
	explicit SensorConfig(std::string frame_id) : frame_id(std::move(frame_id)) {}

	void SetFrameId(const std::string &frame_id) { this->frame_id = frame_id; }

#if MJR_ROS_VERSION == ROS_1
	void RegisterPub(const ros::Publisher &pub) { value_pub = pub; }
	void RegisterGTPub(const ros::Publisher &pub) { gt_pub = pub; }
#else
	template <typename MessageT, typename AllocatorT>
	void RegisterPub(const std::shared_ptr<rclcpp::Publisher<MessageT, AllocatorT> > &pub)
	{
		value_pub = pub;
	}
	template <typename MessageT, typename AllocatorT>
	void RegisterGTPub(const std::shared_ptr<rclcpp::Publisher<MessageT, AllocatorT> > &pub)
	{
		gt_pub = pub;
	}
#endif

	std::string frame_id;

#if MJR_ROS_VERSION == ROS_1
	ros::Publisher gt_pub;
	ros::Publisher value_pub;
#else
	std::any gt_pub;
	std::any value_pub;
#endif

	double mean[3];
	double sigma[3];

	uint8_t is_set = 0;
};

using SensorConfigPtr = std::unique_ptr<SensorConfig>;

class MujocoRosSensorsPlugin : public mujoco_ros::MujocoPlugin
{
public:
	MujocoRosSensorsPlugin()                                          = default;
	MujocoRosSensorsPlugin(const MujocoRosSensorsPlugin &)            = delete;
	MujocoRosSensorsPlugin &operator=(const MujocoRosSensorsPlugin &) = delete;
	~MujocoRosSensorsPlugin() override;

	bool Load(const mjModel *m, mjData *d) override;
	void Reset() override;
	void LastStageCallback(const mjModel *model, mjData *data) override;
	const std::map<std::string, SensorConfigPtr> &GetSensorConfigs() const { return sensor_map_; }

#if MJR_ROS_VERSION == ROS_2
	mujoco_ros::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
	{
		declare_parameter_if_not_declared(this->get_node()->get_node_parameters_interface(), "test_name",
		                                  rclcpp::ParameterValue("parallel_node_name"));
		return mujoco_ros::CallbackReturn::SUCCESS;
	}
#endif

protected:
#if MJR_ROS_VERSION == ROS_1
	ros::NodeHandle sensors_nh_;
	void InitSensors(const mjModel *model, mjData *data);
	ros::ServiceServer register_noise_model_server_;
	bool RegisterNoiseModelsCB(mujoco_ros_msgs::RegisterSensorNoiseModels::Request &req,
	                           mujoco_ros_msgs::RegisterSensorNoiseModels::Response &rep);
#else
	rclcpp_lifecycle::LifecycleNode::SharedPtr sensors_nh_;
	void InitSensors(const mjModel *model, mjData *data);
	rclcpp::Service<mujoco_ros_msgs::srv::RegisterSensorNoiseModels>::SharedPtr register_noise_model_server_;
	void RegisterNoiseModelsCB(const mujoco_ros_msgs::srv::RegisterSensorNoiseModels::Request::SharedPtr &req,
	                           const mujoco_ros_msgs::srv::RegisterSensorNoiseModels::Response::SharedPtr &rep);
#endif

	std::mt19937 rand_generator = std::mt19937(std::random_device{}());
	std::normal_distribution<double> noise_dist;
	std::map<std::string, SensorConfigPtr> sensor_map_;
};

inline const char *SENSOR_STRING[49] = {};

} // namespace mujoco_ros::sensors
