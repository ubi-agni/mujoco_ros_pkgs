/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023 - 2024, Bielefeld University
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

#include <cmath>

#include <mujoco_ros/ros_version.hpp>

#include <mujoco_ros/common_types.hpp>
#include <mujoco_ros/offscreen_camera.hpp>

#if MJR_ROS_VERSION == ROS_1

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

using CameraInfo       = sensor_msgs::CameraInfo;
using TransformStamped = geometry_msgs::TransformStamped;

namespace roscpp = ros;

#else // MJR_ROS_VERSION == ROS_2

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

using CameraInfo       = sensor_msgs::msg::CameraInfo;
using TransformStamped = geometry_msgs::msg::TransformStamped;

namespace roscpp = rclcpp;

#endif

#include <tf2_ros/transform_listener.h>

namespace mujoco_ros::rendering {

class OffscreenCamera
{
public:
	OffscreenCamera(const uint8_t cam_id, const std::string &base_topic, const std::string &cam_name, const int width,
	                const int height, const StreamType stream_type, const bool use_segid, const float pub_freq,
	                const mjModel *model, mjData *data, mujoco_ros::MujocoEnv *env_ptr);

#if MJR_ROS_VERSION == ROS_1
	void InitializeTransport(const std::shared_ptr<ros::NodeHandle> &parent_nh, const mjModel *model, mjData *data,
	                         std::string &rgb_topic, std::string &depth_topic, std::string &segment_topic);
#else // MJR_ROS_VERSION == ROS_2
	void InitializeTransport(const mjModel *model, mjData *data, std::string &rgb_topic, std::string &depth_topic,
	                         std::string &segment_topic);
#endif

	~OffscreenCamera()
	{
		MJR_DEBUG("Freeing offscreen scene state");
		mjv_freeSceneState(&scn_state_);

#if MJR_ROS_VERSION == ROS_1
		if (rgb_pub_ != nullptr) {
			rgb_pub_.shutdown();
		}
		if (depth_pub_ != nullptr) {
			depth_pub_.shutdown();
		}
		if (segment_pub_ != nullptr) {
			segment_pub_.shutdown();
		}
		if (rgb_camera_info_pub_.get() != nullptr) {
			rgb_camera_info_pub_->shutdown();
		}
		if (depth_camera_info_pub_.get() != nullptr) {
			depth_camera_info_pub_->shutdown();
		}
		if (segment_camera_info_pub_.get() != nullptr) {
			segment_camera_info_pub_->shutdown();
		}
#endif
	};

	uint8_t cam_id_;
	std::string cam_name_;
	std::string topic_;
	int width_, height_;
	StreamType stream_type_ = StreamType::RGB;
	bool use_segid_         = true;
	float pub_freq_         = 15.f;

	bool initial_published_ = false;

	mjvOption vopt_ = {};
	mjvSceneState scn_state_;

	roscpp::Time last_pub_;
#if MJR_ROS_VERSION == ROS_1
	std::shared_ptr<ros::NodeHandle> nh_;
	std::unique_ptr<ros::Publisher> rgb_camera_info_pub_;
	std::unique_ptr<ros::Publisher> depth_camera_info_pub_;
	std::unique_ptr<ros::Publisher> segment_camera_info_pub_;
#else // MJR_ROS_VERSION == ROS_2
	rclcpp::Node::SharedPtr nh_;
	rclcpp::Publisher<CameraInfo>::SharedPtr rgb_camera_info_pub_;
	rclcpp::Publisher<CameraInfo>::SharedPtr depth_camera_info_pub_;
	rclcpp::Publisher<CameraInfo>::SharedPtr segment_camera_info_pub_;
#endif
	std::shared_ptr<image_transport::ImageTransport> it_;
	image_transport::Publisher rgb_pub_;
	image_transport::Publisher depth_pub_;
	image_transport::Publisher segment_pub_;

	void RenderAndPublish(mujoco_ros::OffscreenRenderContext *offscreen);

	/**
	 * @brief Check if the camera should render a new image at time t.
	 * @param[in] t The time to check.
	 */
	bool ShouldRender(const roscpp::Time &t);

private:
	std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

	bool RenderAndPubIfNecessary(mujoco_ros::OffscreenRenderContext *offscreen, const bool rgb, const bool depth,
	                             const bool segment);
};

} // end namespace mujoco_ros::rendering
