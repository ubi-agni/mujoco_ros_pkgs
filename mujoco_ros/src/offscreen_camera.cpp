/**
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

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/render_backend.hpp>
#include <mujoco_ros/logging.hpp>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/offscreen_camera.hpp>
#include <mujoco_ros/rendering/frame_capacity.hpp>
#include <mujoco_ros/util.hpp>
#include <mujoco_ros/viewer.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using TransformStamped = geometry_msgs::TransformStamped;
using PoseStamped      = geometry_msgs::PoseStamped;
using Image            = sensor_msgs::Image;
using ImagePtr         = sensor_msgs::ImagePtr;
using CameraInfo       = sensor_msgs::CameraInfo;

namespace roscpp = ros;
using namespace boost;
#else // MJR_ROS_VERSION == ROS_2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

using TransformStamped = geometry_msgs::msg::TransformStamped;
using PoseStamped      = geometry_msgs::msg::PoseStamped;
using Image            = sensor_msgs::msg::Image;
using ImagePtr         = sensor_msgs::msg::Image::SharedPtr;
using CameraInfo       = sensor_msgs::msg::CameraInfo;

namespace roscpp = rclcpp;
using namespace std;
#endif

namespace mujoco_ros::rendering {

OffscreenCamera::OffscreenCamera(const uint8_t cam_id, const std::string &base_topic, const std::string &cam_name,
                                 const int width, const int height, const StreamType stream_type, const bool use_segid,
                                 const float pub_freq, const CameraTransportBootstrap &bootstrap,
                                 mujoco_ros::MujocoEnv *env_ptr)
    : cam_id_(cam_id)
    , cam_name_(cam_name)
    , topic_(base_topic)
    , width_(width)
    , height_(height)
    , stream_type_(stream_type)
    , use_segid_(use_segid)
    , pub_freq_(pub_freq)
    , descriptor_{ static_cast<CameraId>(cam_id + 1),
	                cam_name,
	                width,
	                height,
	                static_cast<PlaneMask>(static_cast<std::uint8_t>(stream_type)),
	                CameraMetadata{ static_cast<CameraId>(cam_id + 1), cam_name, width, height, 0.0F, 0.0F, 0.0F,
	                                0.0F } }
{
	if (bootstrap.vertical_field_of_view_deg <= 0.0F) {
		throw std::invalid_argument("camera transport bootstrap requires positive vertical field of view for " +
		                            cam_name);
	}
	if (bootstrap.parent_frame.empty()) {
		throw std::invalid_argument("camera transport bootstrap requires parent frame for " + cam_name);
	}
#if MJR_ROS_VERSION == ROS_1
	last_pub_ = ros::Time::now();
#else
	last_pub_ = env_ptr->now();
#endif

	mjv_defaultOption(&vopt_);
	timestamp_resolver_        = [](const FrameLease &lease) { return util::toRosTime(lease.simulation_time_ns()); };
	descriptor_.visual_options = vopt_;
	descriptor_.planes         = PlaneMask::kNone;
	if (stream_type_ & StreamType::RGB)
		descriptor_.planes = descriptor_.planes | PlaneMask::kRgb;
	if (stream_type_ & StreamType::DEPTH)
		descriptor_.planes = descriptor_.planes | PlaneMask::kDepth;
	if (stream_type_ & StreamType::SEGMENTED)
		descriptor_.planes = descriptor_.planes | PlaneMask::kSegmentation;
	descriptor_.metadata = CameraMetadata{ descriptor_.id, cam_name_, width_, height_, 0.0F, 0.0F, 0.0F, 0.0F };

#if MJR_ROS_VERSION == ROS_2
	auto options = rclcpp::NodeOptions().arguments({ "--ros-args", "--remap", cam_name + ":__node:=" + cam_name });
	const bool base_topic_is_absolute = !base_topic.empty() && base_topic.front() == '/';
	const std::string camera_namespace =
	    base_topic_is_absolute ? base_topic : std::string(env_ptr->get_name()) + "/" + base_topic;
	nh_ = std::make_shared<rclcpp::Node>(cam_name, camera_namespace, options);
	it_ = std::make_unique<image_transport::ImageTransport>(nh_);
#endif

	TransformStamped cam_transform;
	cam_transform.header.stamp            = bootstrap.static_transform_stamp;
	cam_transform.header.frame_id         = bootstrap.parent_frame;
	cam_transform.child_frame_id          = cam_name + "_link";
	cam_transform.transform.translation.x = bootstrap.body_to_cam_position[0];
	cam_transform.transform.translation.y = bootstrap.body_to_cam_position[1];
	cam_transform.transform.translation.z = bootstrap.body_to_cam_position[2];
	cam_transform.transform.rotation.w    = bootstrap.body_to_cam_orientation[0];
	cam_transform.transform.rotation.x    = bootstrap.body_to_cam_orientation[1];
	cam_transform.transform.rotation.y    = bootstrap.body_to_cam_orientation[2];
	cam_transform.transform.rotation.z    = bootstrap.body_to_cam_orientation[3];
	env_ptr->RegisterStaticTransform(cam_transform);

	cam_transform.header.frame_id         = cam_name + "_link";
	cam_transform.child_frame_id          = cam_name + "_optical_frame";
	cam_transform.transform.translation.x = 0;
	cam_transform.transform.translation.y = 0;
	cam_transform.transform.translation.z = 0;
	cam_transform.transform.rotation.w    = 0;
	cam_transform.transform.rotation.x    = -1.0;
	cam_transform.transform.rotation.y    = 0;
	cam_transform.transform.rotation.z    = 0;
	env_ptr->RegisterStaticTransform(cam_transform);
}

OffscreenCamera::~OffscreenCamera()
{
	if (publication_queue_) {
		publication_queue_->Shutdown();
	}
	UnregisterConsumers();
#if MJR_ROS_VERSION == ROS_1
	if (rgb_pub_ != nullptr)
		rgb_pub_.shutdown();
	if (depth_pub_ != nullptr)
		depth_pub_.shutdown();
	if (segment_pub_ != nullptr)
		segment_pub_.shutdown();
#endif
}

CameraDescriptor OffscreenCamera::descriptor() const
{
	std::lock_guard<std::mutex> lock(descriptor_mutex_);
	return descriptor_;
}

#if MJR_ROS_VERSION == ROS_1
std::unique_ptr<ros::Publisher> advertiseInfoPublisher(const std::string &topic,
                                                       const std::shared_ptr<ros::NodeHandle> &nh)
{
	return std::make_unique<ros::Publisher>(nh->advertise<CameraInfo>(topic, 1, true));
}
#else // MJR_ROS_VERSION == ROS_2
rclcpp::Publisher<CameraInfo>::SharedPtr advertiseInfoPublisher(const std::string &topic,
                                                                const rclcpp::Node::SharedPtr &nh)
{
	return nh->create_publisher<CameraInfo>(topic, 1);
}
#endif

#if MJR_ROS_VERSION == ROS_1
void OffscreenCamera::InitializeTransport(const std::shared_ptr<ros::NodeHandle> &parent_nh,
                                          const float vertical_field_of_view_deg, std::string &rgb_topic,
                                          std::string &depth_topic, std::string &segment_topic)
{
	// Initialize transport
	nh_ = std::make_shared<ros::NodeHandle>(*parent_nh.get(), topic_);
	it_ = std::make_unique<image_transport::ImageTransport>(*nh_.get());
	// init camera info manager
	camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(*nh_.get(), cam_name_);

#else // MJR_ROS_VERSION == ROS_2
void OffscreenCamera::InitializeTransport(const float vertical_field_of_view_deg, std::string &rgb_topic,
                                          std::string &depth_topic, std::string &segment_topic)
{
	// In ROS 2 image_transport expects fully namespaced topics.
	rgb_topic     = nh_->get_effective_namespace() + "/" + rgb_topic;
	depth_topic   = nh_->get_effective_namespace() + "/" + depth_topic;
	segment_topic = nh_->get_effective_namespace() + "/" + segment_topic;
	// init camera info manager
	camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(nh_.get(), cam_name_);
#endif

	if (stream_type_ & StreamType::RGB) {
		MJR_DEBUG_NAMED("mujoco_env", "\tCreating rgb publisher");
		rgb_pub_             = it_->advertise(rgb_topic + "/image_raw", 1);
		rgb_camera_info_pub_ = advertiseInfoPublisher(rgb_topic + "/camera_info", nh_);
	}
	if (stream_type_ & StreamType::DEPTH) {
		MJR_DEBUG_NAMED("mujoco_env", "\tCreating depth publisher");
		depth_pub_             = it_->advertise(depth_topic + "/image_raw", 1);
		depth_camera_info_pub_ = advertiseInfoPublisher(depth_topic + "/camera_info", nh_);
	}
	if (stream_type_ & StreamType::SEGMENTED) {
		MJR_DEBUG_NAMED("mujoco_env", "\tCreating segmentation publisher");
		segment_pub_             = it_->advertise(segment_topic + "/image_raw", 1);
		segment_camera_info_pub_ = advertiseInfoPublisher(segment_topic + "/camera_info", nh_);
	}

	MJR_DEBUG_STREAM_NAMED("mujoco_env", "\tSetting up camera stream(s) of type '"
	                                         << stream_type_ << "' with a publish rate of " << pub_freq_
	                                         << " Hz for camera named " << cam_name_ << " with resolution " << width_
	                                         << "x" << height_);

	// Camera info uses bootstrap-owned vertical field of view; live mjModel/mjData are not required here.
	mjtNum fovy_rad      = (M_PI / 180 * vertical_field_of_view_deg);
	mjtNum focal_scaling = (1. / mju_tan(fovy_rad / 2.0)) * height_ / 2.0;

	std::string camera_frame_name(cam_name_ + "_optical_frame");

	CameraInfo ci      = camera_info_manager_->getCameraInfo();
	ci.header.frame_id = camera_frame_name;
	ci.width           = static_cast<decltype(ci.width)>(width_);
	ci.height          = static_cast<decltype(ci.height)>(height_);

	// clang-format off
	mjtNum extrinsic[12] = {
		focal_scaling, 	0.0, 			(width_ - 1) / 2.0,   0.0,
		0.0, 			focal_scaling, 	(height_ - 1) / 2.0,  0.0,
		0.0, 			0.0, 			1.0, 				  0.0
	};
	// clang-format on

	// Copy extrinsic camera matrix to camera info
#if MJR_ROS_VERSION == ROS_1 // ROS 1 CameraInfo Projection Matrix is P, ROS 2 CameraInfo Projection Matrix is p
	mju_copy(ci.P.c_array(), extrinsic, 12);
#else // MJR_ROS_VERSION == ROS_2
	std::copy(extrinsic, extrinsic + 12, ci.p.begin());
#endif

	// Copy intrinsic Camera matrix to camera info
#if MJR_ROS_VERSION == ROS_1 // ROS 1 CameraInfo Intrinisic Matrix is K, ROS 2 CameraInfo Intrinisic Matrix is k
	mju_copy(ci.K.c_array(), extrinsic, 3);
	mju_copy(ci.K.c_array() + 3, extrinsic + 4, 3);
	mju_copy(ci.K.c_array() + 6, extrinsic + 8, 3);
#else // MJR_ROS_VERSION == ROS_2
	std::copy(extrinsic, extrinsic + 3, ci.k.begin());
	std::copy(extrinsic + 4, extrinsic + 7, ci.k.begin() + 3);
	std::copy(extrinsic + 8, extrinsic + 11, ci.k.begin() + 6);
#endif

	camera_info_manager_->setCameraInfo(ci);
	EnsurePublicationWorker();
}

void OffscreenCamera::EnsurePublicationWorker()
{
	if (publication_queue_) {
		return;
	}
	publication_queue_ = std::make_unique<BoundedPublicationQueue<PublicationItem>>(
	    kRosPublicationQueueDepth, [this](PublicationItem &item) { PublishQueuedItem(item); });
}

void OffscreenCamera::PublishQueuedItem(PublicationItem &item)
{
	if (reset_pending_.load(std::memory_order_acquire)) {
		MJR_WARN_STREAM("ROS camera publication worker dropped capture_id="
		                << item.capture_id << " for camera " << cam_name_ << " because Reset() is pending");
		return;
	}
	if (PublicationItemIsStale(item)) {
		MJR_WARN_STREAM("ROS camera publication worker dropped stale capture_id=" << item.capture_id << " for camera "
		                                                                          << cam_name_);
		return;
	}
	if (item.worker_hook) {
		item.worker_hook();
	}
	if (PublicationItemIsStale(item)) {
		MJR_WARN_STREAM("ROS camera publication worker dropped stale capture_id=" << item.capture_id << " for camera "
		                                                                          << cam_name_);
		return;
	}
	{
		std::function<void()> publish_entry_test_hook;
		{
			std::lock_guard<std::mutex> lock(test_hook_mutex_);
			publish_entry_test_hook = publication_publish_entry_test_hook_;
		}
		std::lock_guard<std::mutex> lock(publication_mutex_);
		if (reset_pending_.load(std::memory_order_acquire)) {
			MJR_WARN_STREAM("ROS camera publication worker dropped capture_id="
			                << item.capture_id << " for camera " << cam_name_ << " because Reset() is pending");
			return;
		}
		if (PublicationItemIsStale(item)) {
			MJR_WARN_STREAM("ROS camera publication worker dropped stale capture_id=" << item.capture_id << " for camera "
			                                                                          << cam_name_);
			return;
		}
		if (publish_entry_test_hook) {
			publish_entry_test_hook();
		}
		if (item.rgb) {
			PublishRgb(*item.rgb, item.envelope);
		}
		if (item.depth) {
			PublishDepth(*item.depth, item.envelope);
		}
		if (item.segment) {
			PublishSegment(*item.segment, item.envelope);
		}
		PublishCameraInfo(item.envelope.stamp, item.published_planes);
	}
	if (item.publication_end_hook) {
		item.publication_end_hook();
	}
}

bool OffscreenCamera::PublicationItemIsStale(const PublicationItem &item) const
{
	const auto current_sequence = publication_sequence_.load(std::memory_order_acquire);
	return (current_sequence & ~std::uint64_t{ 1 }) != (item.committed_publication_sequence & ~std::uint64_t{ 1 });
}

bool OffscreenCamera::ShouldPublishAtTimeLocked(const roscpp::Time &time) const
{
#if MJR_ROS_VERSION == ROS_1
	return !(publication_sequence_.load(std::memory_order_acquire) & 1U) ||
	       (last_pub_ != time && roscpp::Duration(1.0 / static_cast<double>(pub_freq_)) <= time - last_pub_);
#else
	return !(publication_sequence_.load(std::memory_order_acquire) & 1U) ||
	       (last_pub_ != time &&
	        rclcpp::Duration::from_seconds(1.0 / static_cast<double>(pub_freq_)) <= time - last_pub_);
#endif
}

bool OffscreenCamera::ShouldPublishAtTime(const roscpp::Time &time)
{
	std::lock_guard<std::mutex> lock(publication_mutex_);
	return ShouldPublishAtTimeLocked(time);
}

void OffscreenCamera::RegisterConsumer(RenderCore &core)
{
	{
		std::lock_guard<std::mutex> lock(descriptor_mutex_);
		if (ros_consumer_registered_ && ros_consumer_core_ == &core)
			return;
	}
	UnregisterRosConsumer();
	std::lock_guard<std::mutex> lock(descriptor_mutex_);
	ros_consumer_ = core.RegisterOneShotConsumer("ros/" + cam_name_, descriptor_.id);
	core.SetConsumerEnabled(ros_consumer_, false);
	ros_consumer_registered_ = true;
	ros_consumer_core_       = &core;
	ros_consumer_owner_      = active_render_core_;
}

ConsumerId OffscreenCamera::RegisterPythonConsumer(const std::shared_ptr<RenderCore> &core,
                                                   const std::size_t history_depth)
{
	if (!core) {
		throw std::invalid_argument("Python render consumer requires an active RenderCore");
	}
	ValidatePythonHistoryDepth(history_depth);
	const auto consumer = history_depth > 1 ? core->RegisterCadencedConsumer("python/" + cam_name_,
	                                                                         kPythonAsyncBufferCadence, descriptor_.id) :
	                                          core->RegisterOneShotConsumer("python/" + cam_name_, descriptor_.id);
	{
		std::lock_guard<std::mutex> lock(descriptor_mutex_);
		python_consumers_.emplace(consumer.value(), PythonConsumerRegistration{ &*core, core });
		python_consumer_registered_ = true;
		python_consumer_core_       = &*core;
		python_consumer_owner_      = core;
	}
	return consumer;
}

ConsumerId OffscreenCamera::RegisterPythonConsumer(RenderCore &core, const std::size_t history_depth)
{
	ValidatePythonHistoryDepth(history_depth);
	const auto consumer = history_depth > 1 ? core.RegisterCadencedConsumer("python/" + cam_name_,
	                                                                        kPythonAsyncBufferCadence, descriptor_.id) :
	                                          core.RegisterOneShotConsumer("python/" + cam_name_, descriptor_.id);
	{
		std::lock_guard<std::mutex> lock(descriptor_mutex_);
		python_consumers_.emplace(consumer.value(), PythonConsumerRegistration{ &core, {} });
		python_consumer_registered_ = true;
		python_consumer_core_       = &core;
		python_consumer_owner_.reset();
	}
	return consumer;
}

void OffscreenCamera::UnregisterPythonConsumer(ConsumerId consumer)
{
	PythonConsumerRegistration registration;
	{
		std::lock_guard<std::mutex> lock(descriptor_mutex_);
		const auto it = python_consumers_.find(consumer.value());
		if (it == python_consumers_.end()) {
			return;
		}
		registration = it->second;
		python_consumers_.erase(it);
		python_consumer_registered_ = !python_consumers_.empty();
		if (python_consumers_.empty()) {
			python_consumer_core_ = nullptr;
			python_consumer_owner_.reset();
		} else {
			const auto &remaining  = python_consumers_.begin()->second;
			python_consumer_core_  = remaining.core;
			python_consumer_owner_ = remaining.owner;
		}
	}
	if (registration.core != nullptr) {
		registration.core->UnregisterConsumer(consumer);
	}
}

void OffscreenCamera::UnregisterRosConsumer()
{
	std::shared_ptr<RenderCore> ros_core;
	ConsumerId ros_consumer;
	{
		std::lock_guard<std::mutex> lock(descriptor_mutex_);
		if (!ros_consumer_registered_)
			return;
		ros_core                 = ros_consumer_owner_.lock();
		ros_consumer             = ros_consumer_;
		ros_consumer_registered_ = false;
		ros_consumer_core_       = nullptr;
		ros_consumer_owner_.reset();
	}
	{
		std::lock_guard<std::mutex> lock(publication_mutex_);
		ros_request_accepted_ = false;
	}
	if (ros_core != nullptr) {
		ros_core->UnregisterConsumer(ros_consumer);
	}
}

void OffscreenCamera::UnregisterPythonConsumer()
{
	std::vector<std::pair<ConsumerId, PythonConsumerRegistration>> registrations;
	{
		std::lock_guard<std::mutex> lock(descriptor_mutex_);
		registrations.reserve(python_consumers_.size());
		for (const auto &[value, registration] : python_consumers_) {
			registrations.emplace_back(ConsumerId(value), registration);
		}
		python_consumers_.clear();
		python_consumer_registered_ = false;
		python_consumer_core_       = nullptr;
		python_consumer_owner_.reset();
	}
	for (const auto &[consumer, registration] : registrations) {
		if (registration.core != nullptr) {
			registration.core->UnregisterConsumer(consumer);
		}
	}
}

void OffscreenCamera::UnregisterConsumers()
{
	UnregisterRosConsumer();
	UnregisterPythonConsumer();
}

void OffscreenCamera::SetActiveRenderCore(const std::shared_ptr<RenderCore> &core)
{
	if (!core && retirement_failure_once_.exchange(false)) {
		throw std::runtime_error("deterministic camera retirement failure");
	}
	if (!core) {
		UnregisterConsumers();
	}
	std::lock_guard<std::mutex> lock(descriptor_mutex_);
	active_render_core_ = core;
}

void OffscreenCamera::SetVisualFlag(int flag_idx, bool enable)
{
	if (flag_idx < 0 || flag_idx >= mjNVISFLAG) {
		throw std::out_of_range("invalid MuJoCo visual flag index");
	}
	std::shared_ptr<RenderCore> core;
	mjvOption options;
	{
		std::lock_guard<std::mutex> lock(descriptor_mutex_);
		vopt_.flags[flag_idx]                      = enable ? 1 : 0;
		descriptor_.visual_options.flags[flag_idx] = vopt_.flags[flag_idx];
		options                                    = descriptor_.visual_options;
		core                                       = active_render_core_.lock();
	}
	if (core) {
		const auto status = core->UpdateCameraVisualOptions(descriptor_.id, options);
		if (!status.ok()) {
			throw std::runtime_error("failed to update camera visual options: " + status.message);
		}
	}
}

void OffscreenCamera::ToggleVisualFlag(int flag_idx)
{
	if (flag_idx < 0 || flag_idx >= mjNVISFLAG) {
		throw std::out_of_range("invalid MuJoCo visual flag index");
	}
	const bool enable = [&]() {
		std::lock_guard<std::mutex> lock(descriptor_mutex_);
		return vopt_.flags[flag_idx] == 0;
	}();
	SetVisualFlag(flag_idx, enable);
}

int OffscreenCamera::GetVisualFlag(int flag_idx) const
{
	if (flag_idx < 0 || flag_idx >= mjNVISFLAG) {
		throw std::out_of_range("invalid MuJoCo visual flag index");
	}
	std::lock_guard<std::mutex> lock(descriptor_mutex_);
	return vopt_.flags[flag_idx];
}

std::uint64_t OffscreenCamera::UpdateDemand(RenderCore &core, const roscpp::Time &time)
{
	RegisterConsumer(core);
	std::lock_guard<std::mutex> lock(publication_mutex_);
	const bool enabled = HasSubscribers();
	core.SetConsumerEnabled(ros_consumer_, enabled);
	ros_request_accepted_ = enabled && ShouldPublishAtTimeLocked(time);
	if (ros_request_accepted_) {
		core.RequestOneShot(ros_consumer_);
	}
	demand_enabled_ = enabled;
	return publication_sequence_.load(std::memory_order_acquire);
}

bool OffscreenCamera::HasSubscribers() const
{
#if MJR_ROS_VERSION == ROS_1
	return ((stream_type_ & StreamType::RGB) &&
	        (rgb_pub_.getNumSubscribers() > 0 || rgb_camera_info_pub_->getNumSubscribers() > 0)) ||
	       ((stream_type_ & StreamType::DEPTH) &&
	        (depth_pub_.getNumSubscribers() > 0 || depth_camera_info_pub_->getNumSubscribers() > 0)) ||
	       ((stream_type_ & StreamType::SEGMENTED) &&
	        (segment_pub_.getNumSubscribers() > 0 || segment_camera_info_pub_->getNumSubscribers() > 0));
#else
	return ((stream_type_ & StreamType::RGB) &&
	        (rgb_pub_.getNumSubscribers() > 0 || rgb_camera_info_pub_->get_subscription_count() > 0)) ||
	       ((stream_type_ & StreamType::DEPTH) &&
	        (depth_pub_.getNumSubscribers() > 0 || depth_camera_info_pub_->get_subscription_count() > 0)) ||
	       ((stream_type_ & StreamType::SEGMENTED) &&
	        (segment_pub_.getNumSubscribers() > 0 || segment_camera_info_pub_->get_subscription_count() > 0));
#endif
}

void OffscreenCamera::PublishCameraInfo(const roscpp::Time &time, const PlaneMask planes)
{
	CameraInfo info   = camera_info_manager_->getCameraInfo();
	info.header.stamp = time;
	if (HasPlane(planes, PlaneKind::kRgb) && rgb_camera_info_pub_)
		rgb_camera_info_pub_->publish(info);
	if (HasPlane(planes, PlaneKind::kDepth) && depth_camera_info_pub_)
		depth_camera_info_pub_->publish(info);
	if (HasPlane(planes, PlaneKind::kSegmentation) && segment_camera_info_pub_)
		segment_camera_info_pub_->publish(info);
}

void OffscreenCamera::PublishRgb(const FrameLease &lease, const TransportEnvelope &envelope)
{
	ImagePtr msg         = make_shared<Image>();
	msg->header.frame_id = cam_name_ + "_optical_frame";
	msg->header.stamp    = envelope.stamp;
	msg->width           = static_cast<decltype(msg->width)>(lease.layout().width);
	msg->height          = static_cast<decltype(msg->height)>(lease.layout().height);
	msg->encoding        = sensor_msgs::image_encodings::RGB8;
	msg->step            = static_cast<decltype(msg->step)>(lease.layout().stride_bytes);
	msg->data.resize(lease.bytes().size());
	std::memcpy(msg->data.data(), lease.bytes().data(), lease.bytes().size());
	rgb_pub_.publish(msg);
}

void OffscreenCamera::PublishDepth(const FrameLease &lease, const TransportEnvelope &envelope)
{
	ImagePtr msg         = make_shared<Image>();
	msg->header.frame_id = cam_name_ + "_optical_frame";
	msg->header.stamp    = envelope.stamp;
	msg->width           = static_cast<decltype(msg->width)>(lease.layout().width);
	msg->height          = static_cast<decltype(msg->height)>(lease.layout().height);
	msg->encoding        = sensor_msgs::image_encodings::TYPE_32FC1;
	msg->step            = static_cast<decltype(msg->step)>(lease.layout().stride_bytes);
	msg->data.resize(lease.bytes().size());
	std::memcpy(msg->data.data(), lease.bytes().data(), lease.bytes().size());
	depth_pub_.publish(msg);
}

void OffscreenCamera::PublishSegment(const FrameLease &lease, const TransportEnvelope &envelope)
{
	ImagePtr msg         = make_shared<Image>();
	msg->header.frame_id = cam_name_ + "_optical_frame";
	msg->header.stamp    = envelope.stamp;
	msg->width           = static_cast<decltype(msg->width)>(lease.layout().width);
	msg->height          = static_cast<decltype(msg->height)>(lease.layout().height);
	msg->encoding        = sensor_msgs::image_encodings::RGB8;
	msg->step            = static_cast<decltype(msg->step)>(lease.layout().stride_bytes);
	msg->data.resize(lease.bytes().size());
	std::memcpy(msg->data.data(), lease.bytes().data(), lease.bytes().size());
	segment_pub_.publish(msg);
}

OffscreenCamera::PublishResult OffscreenCamera::PublishLatest(RenderCore &core, const roscpp::Time &accepted_time,
                                                              std::uint64_t accepted_publication_sequence,
                                                              std::uint64_t expected_capture_id)
{
	std::function<void()> publication_test_hook;
	{
		std::lock_guard<std::mutex> lock(test_hook_mutex_);
		publication_test_hook = publication_test_hook_;
	}
	if (publication_test_hook)
		publication_test_hook();

	{
		std::lock_guard<std::mutex> lock(publication_mutex_);
		const auto current_publication_sequence = publication_sequence_.load(std::memory_order_acquire);
		if ((current_publication_sequence & ~std::uint64_t{ 1 }) !=
		    (accepted_publication_sequence & ~std::uint64_t{ 1 })) {
			ros_request_accepted_ = false;
			return PublishResult{};
		}
		if (!demand_enabled_ || !ros_request_accepted_ || !HasSubscribers()) {
			return PublishResult{};
		}
		ros_request_accepted_ = false;
		if (!ShouldPublishAtTimeLocked(accepted_time)) {
			return PublishResult{};
		}
	}
	const auto camera = descriptor();
	if (expected_capture_id == 0U) {
		std::lock_guard<std::mutex> lock(publication_mutex_);
		last_publication_status_ =
		    rendering::FrameStatus{ rendering::FrameStatusCode::kNoFrame, 0, std::nullopt, core.frames().generation(),
			                         "ROS camera accepted demand but current capture has no identity" };
		return PublishResult{ last_publication_status_, false };
	}
	std::optional<FrameLease> first;
	for (const auto plane : { PlaneKind::kRgb, PlaneKind::kDepth, PlaneKind::kSegmentation }) {
		if (HasPlane(camera.planes, plane)) {
			first = core.AcquireLatest(expected_capture_id, camera.id, plane);
			if (first)
				break;
		}
	}
	if (!first) {
		std::lock_guard<std::mutex> lock(publication_mutex_);
		last_publication_status_ =
		    rendering::FrameStatus{ rendering::FrameStatusCode::kNoFrame, expected_capture_id, std::nullopt,
			                         core.frames().generation(),
			                         "ROS camera accepted demand but current capture committed no planes" };
		return PublishResult{ last_publication_status_, false };
	}
	const auto capture_id = first->capture_id();
	std::optional<FrameLease> rgb;
	std::optional<FrameLease> depth;
	std::optional<FrameLease> segment;
	PlaneMask published_planes = PlaneMask::kNone;
	std::optional<rendering::FrameStatus> missing_plane;
	if (stream_type_ & StreamType::RGB) {
		rgb = core.AcquireLatest(capture_id, camera.id, PlaneKind::kRgb);
		if (!rgb) {
			missing_plane =
			    rendering::FrameStatus{ rendering::FrameStatusCode::kFrameUnavailable, capture_id, PlaneKind::kRgb,
				                         first->generation(), "RGB plane missing from accepted frame capture" };
		} else {
			published_planes = published_planes | PlaneMask::kRgb;
		}
	}
	if (stream_type_ & StreamType::DEPTH) {
		depth = core.AcquireLatest(capture_id, camera.id, PlaneKind::kDepth);
		if (!depth) {
			if (!missing_plane) {
				missing_plane =
				    rendering::FrameStatus{ rendering::FrameStatusCode::kFrameUnavailable, capture_id, PlaneKind::kDepth,
					                         first->generation(), "depth plane missing from accepted frame capture" };
			}
		} else {
			published_planes = published_planes | PlaneMask::kDepth;
		}
	}
	if (stream_type_ & StreamType::SEGMENTED) {
		segment = core.AcquireLatest(capture_id, camera.id, PlaneKind::kSegmentation);
		if (!segment) {
			if (!missing_plane) {
				missing_plane = rendering::FrameStatus{ rendering::FrameStatusCode::kFrameUnavailable, capture_id,
					                                     PlaneKind::kSegmentation, first->generation(),
					                                     "segmentation plane missing from accepted frame capture" };
			}
		} else {
			published_planes = published_planes | PlaneMask::kSegmentation;
		}
	}
	const TransportEnvelope envelope{ timestamp_resolver_(*first), capture_id };
	const auto committed_stamp = envelope.stamp;
	EnsurePublicationWorker();
	PublicationItem item;
	item.capture_id       = capture_id;
	item.rgb              = std::move(rgb);
	item.depth            = std::move(depth);
	item.segment          = std::move(segment);
	item.envelope         = envelope;
	item.published_planes = published_planes;
	item.missing_plane    = missing_plane;
	std::function<void()> enqueue_test_hook;
	{
		std::lock_guard<std::mutex> lock(test_hook_mutex_);
		item.worker_hook          = worker_publication_hook_;
		item.publication_end_hook = publication_end_test_hook_;
		enqueue_test_hook         = publication_enqueue_test_hook_;
	}
	{
		std::lock_guard<std::mutex> lock(publication_mutex_);
		auto expected_sequence = accepted_publication_sequence;
		const bool committed   = publication_sequence_.compare_exchange_strong(
          expected_sequence, accepted_publication_sequence | 1U, std::memory_order_acq_rel, std::memory_order_acquire);
		if (!committed) {
			last_publication_status_ =
			    rendering::FrameStatus{ rendering::FrameStatusCode::kFrameUnavailable, capture_id, std::nullopt,
				                         first->generation(),
				                         "ROS camera publication sequence advanced before enqueue; frame dropped" };
			return PublishResult{ last_publication_status_, false };
		}
		item.committed_publication_sequence = accepted_publication_sequence | 1U;
		if (enqueue_test_hook) {
			enqueue_test_hook();
		}
		const auto enqueue = publication_queue_->Enqueue(std::move(item), false);
		if (!enqueue.enqueued) {
			last_publication_status_ =
			    rendering::FrameStatus{ rendering::FrameStatusCode::kTerminalError, capture_id, std::nullopt,
				                         first->generation(),
				                         "ROS camera publication queue rejected frame; queue may be shutting down" };
			return PublishResult{ last_publication_status_, false };
		}
		last_pub_                  = committed_stamp;
		last_published_capture_id_ = capture_id;
		if (missing_plane) {
			last_publication_status_ = *missing_plane;
		} else {
			last_publication_status_ = rendering::FrameStatus{ rendering::FrameStatusCode::kOk, capture_id, std::nullopt,
				                                                first->generation(), "" };
		}
		if (enqueue.dropped_older) {
			MJR_WARN_STREAM("ROS camera publication queue dropped capture_id=" << enqueue.dropped_capture_id
			                                                                   << " for camera " << cam_name_);
		}
	}
	publication_queue_->NotifyWorker();
	return PublishResult{ last_publication_status_, true };
}

void OffscreenCamera::Reset()
{
	std::function<void()> reset_test_hook;
	{
		std::lock_guard<std::mutex> lock(test_hook_mutex_);
		reset_test_hook = reset_test_hook_;
	}
	reset_pending_.store(true, std::memory_order_release);
	std::lock_guard<std::mutex> lock(publication_mutex_);
	reset_pending_.store(false, std::memory_order_release);
	ros_request_accepted_ = false;
	if (publication_queue_) {
		publication_queue_->CancelPending();
	}
	auto expected_sequence = publication_sequence_.load(std::memory_order_acquire);
	while (!publication_sequence_.compare_exchange_weak(expected_sequence, (expected_sequence & ~std::uint64_t{ 1 }) + 2,
	                                                    std::memory_order_acq_rel, std::memory_order_acquire)) {
	}
	if (reset_test_hook)
		reset_test_hook();
}

} // namespace mujoco_ros::rendering
