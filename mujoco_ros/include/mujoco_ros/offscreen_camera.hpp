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

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>

#include <mujoco_ros/ros_version.hpp>

#include <mujoco_ros/common_types.hpp>
#include <mujoco_ros/logging.hpp>
#include <mujoco_ros/rendering/bounded_publication_queue.hpp>
#include <mujoco_ros/rendering/render_core.hpp>

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

#include <atomic>

namespace mujoco_ros::rendering {

struct CameraTransportBootstrap
{
	roscpp::Time static_transform_stamp{};
	std::string parent_frame;
	std::array<mjtNum, 3> body_to_cam_position{};
	std::array<mjtNum, 4> body_to_cam_orientation{};
	float vertical_field_of_view_deg = 0.0F;
};

class OffscreenCamera
{
public:
	struct PublishResult
	{
		rendering::FrameStatus status = rendering::FrameStatus::Ok();
		bool accepted                 = false;

		bool ok() const { return status.ok(); }
	};

	OffscreenCamera(const uint8_t cam_id, const std::string &base_topic, const std::string &cam_name, const int width,
	                const int height, const StreamType stream_type, const bool use_segid, const float pub_freq,
	                const CameraTransportBootstrap &bootstrap, mujoco_ros::MujocoEnv *env_ptr);

#if MJR_ROS_VERSION == ROS_1
	void InitializeTransport(const std::shared_ptr<ros::NodeHandle> &parent_nh, float vertical_field_of_view_deg,
	                         std::string &rgb_topic, std::string &depth_topic, std::string &segment_topic);
#else // MJR_ROS_VERSION == ROS_2
	void InitializeTransport(float vertical_field_of_view_deg, std::string &rgb_topic, std::string &depth_topic,
	                         std::string &segment_topic);
#endif

	~OffscreenCamera();

	uint8_t cam_id_;
	std::string cam_name_;
	std::string topic_;
	int width_, height_;
	StreamType stream_type_ = StreamType::RGB;
	bool use_segid_         = true;
	float pub_freq_         = 15.f;

	std::atomic_uint64_t publication_sequence_{ 0 };
	std::atomic_bool reset_pending_{ false };

	mjvOption vopt_ = {}; // Options remain transport-visible camera settings.
	rendering::CameraDescriptor descriptor_;
	rendering::ConsumerId ros_consumer_;
	bool ros_consumer_registered_             = false;
	rendering::RenderCore *ros_consumer_core_ = nullptr;
	std::weak_ptr<rendering::RenderCore> ros_consumer_owner_;
	rendering::ConsumerId python_consumer_;
	bool python_consumer_registered_             = false;
	rendering::RenderCore *python_consumer_core_ = nullptr;
	std::weak_ptr<rendering::RenderCore> python_consumer_owner_;
	bool demand_enabled_ = false;

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

	rendering::CameraDescriptor descriptor() const;
	void RegisterConsumer(rendering::RenderCore &core);
	rendering::ConsumerId RegisterPythonConsumer(const std::shared_ptr<rendering::RenderCore> &core,
	                                             std::size_t history_depth = 1);
	rendering::ConsumerId RegisterPythonConsumer(rendering::RenderCore &core, std::size_t history_depth = 1);
	void UnregisterPythonConsumer(rendering::ConsumerId consumer);
	void UnregisterPythonConsumer();
	void SetActiveRenderCore(const std::shared_ptr<rendering::RenderCore> &core);
	void SetVisualFlag(int flag_idx, bool enable);
	void ToggleVisualFlag(int flag_idx);
	int GetVisualFlag(int flag_idx) const;
	std::uint64_t UpdateDemand(rendering::RenderCore &core, const roscpp::Time &time);
	bool HasSubscribers() const;
	PublishResult PublishLatest(rendering::RenderCore &core, const roscpp::Time &accepted_time,
	                            std::uint64_t accepted_publication_sequence, std::uint64_t expected_capture_id);
	std::uint64_t last_published_capture_id() const
	{
		std::lock_guard<std::mutex> lock(publication_mutex_);
		return last_published_capture_id_;
	}
	std::size_t publication_drops() const
	{
		if (!publication_queue_) {
			return 0;
		}
		return publication_queue_->dropped_count();
	}
	std::size_t publication_cancelled() const
	{
		if (!publication_queue_) {
			return 0;
		}
		return publication_queue_->cancelled_count();
	}
	rendering::FrameStatus last_publication_status() const
	{
		std::lock_guard<std::mutex> lock(publication_mutex_);
		return last_publication_status_;
	}
	void Reset();

	/**
	 * @brief Check if the camera should publish at time t.
	 */
	bool ShouldPublishAtTime(const roscpp::Time &t);

private:
	friend class OffscreenCameraTestAccess;

	struct TransportEnvelope
	{
		roscpp::Time stamp;
		std::uint64_t capture_id = 0;
	};

	struct PublicationItem
	{
		std::uint64_t capture_id                     = 0;
		std::uint64_t committed_publication_sequence = 0;
		std::optional<rendering::FrameLease> rgb;
		std::optional<rendering::FrameLease> depth;
		std::optional<rendering::FrameLease> segment;
		TransportEnvelope envelope;
		rendering::PlaneMask published_planes = rendering::PlaneMask::kNone;
		std::optional<rendering::FrameStatus> missing_plane;
		std::function<void()> worker_hook;
		std::function<void()> publication_end_hook;
	};

	std::unique_ptr<BoundedPublicationQueue<PublicationItem>> publication_queue_;
	void EnsurePublicationWorker();
	void PublishQueuedItem(PublicationItem &item);
	bool PublicationItemIsStale(const PublicationItem &item) const;

	std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
	mutable std::mutex descriptor_mutex_;
	mutable std::mutex publication_mutex_;
	mutable std::mutex test_hook_mutex_;
	std::weak_ptr<rendering::RenderCore> active_render_core_;
	std::function<roscpp::Time(const rendering::FrameLease &)> timestamp_resolver_;
	bool ros_request_accepted_               = false;
	std::uint64_t last_published_capture_id_ = 0;
	rendering::FrameStatus last_publication_status_;
	std::function<void()> publication_test_hook_;
	std::function<void()> publication_enqueue_test_hook_;
	std::function<void()> worker_publication_hook_;
	std::function<void()> publication_publish_entry_test_hook_;
	std::function<void()> publication_end_test_hook_;
	std::function<void()> reset_test_hook_;
	std::atomic_bool retirement_failure_once_{ false };

	void UnregisterRosConsumer();
	bool ShouldPublishAtTimeLocked(const roscpp::Time &time) const;
	void PublishRgb(const rendering::FrameLease &lease, const TransportEnvelope &envelope);
	void PublishDepth(const rendering::FrameLease &lease, const TransportEnvelope &envelope);
	void PublishSegment(const rendering::FrameLease &lease, const TransportEnvelope &envelope);
	void PublishCameraInfo(const roscpp::Time &time, rendering::PlaneMask planes);
	void UnregisterConsumers();

	struct PythonConsumerRegistration
	{
		rendering::RenderCore *core = nullptr;
		std::weak_ptr<rendering::RenderCore> owner;
	};
	std::unordered_map<std::uint64_t, PythonConsumerRegistration> python_consumers_;
};

} // end namespace mujoco_ros::rendering
