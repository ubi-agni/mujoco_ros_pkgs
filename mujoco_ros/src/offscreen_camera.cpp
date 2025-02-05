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

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/render_backend.hpp>
#include <mujoco_ros/logging.hpp>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/offscreen_camera.hpp>
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
                                 const float pub_freq, const mjModel *model, mjData *data,
                                 mujoco_ros::MujocoEnv *env_ptr)
    : cam_id_(cam_id)
    , cam_name_(cam_name)
    , topic_(base_topic)
    , width_(width)
    , height_(height)
    , stream_type_(stream_type)
    , use_segid_(use_segid)
    , pub_freq_(pub_freq)
{
#if MJR_ROS_VERSION == ROS_1
	last_pub_ = ros::Time::now();
#else // MJR_ROS_VERSION == ROS_2
	last_pub_ = env_ptr->now();
#endif

	mjv_defaultOption(&vopt_);
	mjv_defaultSceneState(&scn_state_);
	mjv_makeSceneState(const_cast<mjModel *>(model), data, &scn_state_, Viewer::kMaxGeom);

#if MJR_ROS_VERSION == ROS_2
	auto options = rclcpp::NodeOptions().arguments({ "--ros-args", "--remap", cam_name + ":__node:=" + cam_name });
	nh_ = std::make_shared<rclcpp::Node>(cam_name, std::string(env_ptr->get_name()) + "/" + base_topic, options);
	// nh_ = env_ptr->create_sub_node(base_topic);
	// nh_ = std::shared_ptr<rclcpp::Node>(new rclcpp::Node(*env_ptr, base_topic));
	MJR_ERROR_STREAM("Created cam node with name " << nh_->get_name() << " and namespace " << nh_->get_namespace());
	// Initialize transport
	it_ = std::make_unique<image_transport::ImageTransport>(nh_);
#endif

	// Create static transform to camera
	// The camera is looking along the -Z axis of its frame. The +X axis points to the right, and the +Y axis points up.
	// https://mujoco.readthedocs.io/en/latest/XMLreference.html#body-camera

	int body_id              = model->cam_bodyid[cam_id];
	std::string parent_frame = mj_id2name(const_cast<mjModel *>(model), mjOBJ_BODY, body_id);
	MJR_DEBUG_STREAM("Creating camera frames for cam '" << cam_name << "' with parent link " << parent_frame);

	roscpp::Time time;
#if MJR_ROS_VERSION == ROS_1
	time = ros::Time(data->time);
#else // MJR_ROS_VERSION == ROS_2
	time      = rclcpp::Time(static_cast<uint64_t>(data->time * 1e9)); // convert to nanoseconds
#endif

	TransformStamped cam_transform;
	cam_transform.header.stamp            = time;
	cam_transform.header.frame_id         = parent_frame;
	cam_transform.child_frame_id          = cam_name + "_link";
	cam_transform.transform.translation.x = model->cam_pos[cam_id * 3];
	cam_transform.transform.translation.y = model->cam_pos[cam_id * 3 + 1];
	cam_transform.transform.translation.z = model->cam_pos[cam_id * 3 + 2];

	cam_transform.transform.rotation.w = model->cam_quat[cam_id * 4];
	cam_transform.transform.rotation.x = model->cam_quat[cam_id * 4 + 1];
	cam_transform.transform.rotation.y = model->cam_quat[cam_id * 4 + 2];
	cam_transform.transform.rotation.z = model->cam_quat[cam_id * 4 + 3];
	env_ptr->RegisterStaticTransform(cam_transform);

	cam_transform.header.frame_id = cam_name + "_link";
	cam_transform.child_frame_id  = cam_name + "_optical_frame";

	cam_transform.transform.translation.x = 0;
	cam_transform.transform.translation.y = 0;
	cam_transform.transform.translation.z = 0;

	cam_transform.transform.rotation.w = 0;
	cam_transform.transform.rotation.x = -1.0;
	cam_transform.transform.rotation.y = 0;
	cam_transform.transform.rotation.z = 0;
	env_ptr->RegisterStaticTransform(cam_transform);
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
void OffscreenCamera::InitializeTransport(const std::shared_ptr<ros::NodeHandle> &parent_nh, const mjModel *model,
                                          mjData *data, std::string &rgb_topic, std::string &depth_topic,
                                          std::string &segment_topic)
{
	// Initialize transport
	nh_ = std::make_shared<ros::NodeHandle>(*parent_nh.get(), topic_);
	it_ = std::make_unique<image_transport::ImageTransport>(*nh_.get());
	// init camera info manager
	camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(*nh_.get(), cam_name_);

#else // MJR_ROS_VERSION == ROS_2
void OffscreenCamera::InitializeTransport(const mjModel *model, mjData *data, std::string &rgb_topic,
                                          std::string &depth_topic, std::string &segment_topic)
{
	// Because of flawed ROS 2 namespace handling, we need to prepend the effective namespace to the topic
	// otherwise image transport segfaults
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

	// Get camera info
	mjtNum cam_pos[3];
	mju_copy(cam_pos, const_cast<const mjtNum *>(data->cam_xpos + cam_id_ * 3), 3);

	mjtNum fovy_rad      = (M_PI / 180 * model->cam_fovy[cam_id_]);
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
}

bool OffscreenCamera::ShouldRender(const roscpp::Time &t)
{
#if MJR_ROS_VERSION == ROS_1
	return (!initial_published_ ||
	        (last_pub_ != t && roscpp::Duration(1.0 / static_cast<double>(pub_freq_)) < t - last_pub_));
#else // MJR_ROS_VERSION == ROS_2
	return (!initial_published_ ||
	        (last_pub_ != t && rclcpp::Duration::from_seconds(1.0 / static_cast<double>(pub_freq_)) < t - last_pub_));
#endif
}

bool OffscreenCamera::RenderAndPubIfNecessary(mujoco_ros::OffscreenRenderContext *offscreen, const bool rgb,
                                              const bool depth, const bool segment)
{
	bool rgb_or_s = rgb || segment;

	if (!rgb_or_s && !depth) { // Nothing to render
		return false;
	}

	// TODO(dleins): Add option to have differing resolutions for rgb and depth (which is common in real cameras)?

	// Resize according to the camera resolution
	offscreen->con.offWidth  = width_;
	offscreen->con.offHeight = height_;
	mjrRect viewport         = mjr_maxViewport(&offscreen->con);

	// Update from scn_state_
	mjv_updateSceneFromState(&scn_state_, &vopt_, nullptr, &offscreen->cam, mjCAT_ALL, &offscreen->scn);
	// Render to buffer
	mjr_render(viewport, &offscreen->scn, &offscreen->con);
	// read buffers
	if (rgb_or_s && depth) {
		mjr_readPixels(offscreen->rgb.get(), offscreen->depth.get(), viewport, &offscreen->con);
	} else if (rgb_or_s) {
		mjr_readPixels(offscreen->rgb.get(), nullptr, viewport, &offscreen->con);
	} else { // depth only
		mjr_readPixels(nullptr, offscreen->depth.get(), viewport, &offscreen->con);
	}
#if RENDER_BACKEND == GLFW_BACKEND
	glfwSwapBuffers(offscreen->window.get());
#endif

#if MJR_ROS_VERSION == ROS_1
	auto ros_time = ros::Time(scn_state_.data.time);
#else // MJR_ROS_VERSION == ROS_2
	auto ros_time = rclcpp::Time(static_cast<uint64_t>(scn_state_.data.time * 1e9)); // convert to nanoseconds
#endif
	CameraInfo camera_info_msg   = camera_info_manager_->getCameraInfo();
	camera_info_msg.header.stamp = ros_time;

	if (rgb_or_s) {
		// Publish RGB image
		ImagePtr rgb_msg         = make_shared<Image>();
		rgb_msg->header.frame_id = cam_name_ + "_optical_frame";
		rgb_msg->header.stamp    = ros_time;
		rgb_msg->width           = static_cast<decltype(rgb_msg->width)>(viewport.width);
		rgb_msg->height          = static_cast<decltype(rgb_msg->height)>(viewport.height);
		rgb_msg->encoding        = sensor_msgs::image_encodings::RGB8;
		rgb_msg->step            = static_cast<decltype(rgb_msg->step)>(viewport.width) * 3u * sizeof(unsigned char);
		size_t size              = rgb_msg->step * static_cast<uint>(viewport.height);
		rgb_msg->data.resize(size);

		memcpy(reinterpret_cast<char *>(&rgb_msg->data[0]), offscreen->rgb.get(), size);

		for (uint r = 0; r < rgb_msg->height / 2; ++r) {
			unsigned char *top_row    = &rgb_msg->data[3 * rgb_msg->width * r];
			unsigned char *bottom_row = &rgb_msg->data[3 * rgb_msg->width * (rgb_msg->height - 1 - r)];
			std::swap_ranges(top_row, top_row + 3 * rgb_msg->width, bottom_row);
		}

		if (segment) {
			segment_pub_.publish(rgb_msg);
			segment_camera_info_pub_->publish(camera_info_msg);
		} else {
			rgb_pub_.publish(rgb_msg);
			rgb_camera_info_pub_->publish(camera_info_msg);
		}
	}

	if (depth) {
		// Publish DEPTH image
		ImagePtr depth_msg         = make_shared<Image>();
		depth_msg->header.frame_id = cam_name_ + "_optical_frame";
		depth_msg->header.stamp    = ros_time;
		depth_msg->width           = util::as_unsigned(viewport.width);
		depth_msg->height          = util::as_unsigned(viewport.height);
		depth_msg->encoding        = sensor_msgs::image_encodings::TYPE_32FC1;
		depth_msg->step = static_cast<decltype(depth_msg->step)>(util::as_unsigned(viewport.width) * sizeof(float));
		size_t size     = depth_msg->step * util::as_unsigned(viewport.height);
		depth_msg->data.resize(size);

		auto *dest_float = reinterpret_cast<float *>(&depth_msg->data[0]);
		uint index       = 0;

		auto e  = static_cast<float>(scn_state_.model.stat.extent);
		float f = e * scn_state_.model.vis.map.zfar;
		float n = e * scn_state_.model.vis.map.znear;

		for (uint32_t j = depth_msg->height; j > 0; j--) {
			for (uint32_t i = 0; i < depth_msg->width; i++) {
				float depth_val = offscreen->depth[index];
				index += 1u;
				dest_float[i + (j - 1u) * depth_msg->width] = -f * n / (depth_val * (f - n) - f);
			}
		}

		depth_pub_.publish(depth_msg);
		depth_camera_info_pub_->publish(camera_info_msg);
	}

	return true;
}

void OffscreenCamera::RenderAndPublish(mujoco_ros::OffscreenRenderContext *offscreen)
{
	{
#if MJR_ROS_VERSION == ROS_1
		roscpp::Time t = roscpp::Time(scn_state_.data.time);
#else // MJR_ROS_VERSION == ROS_2
		rclcpp::Time t = rclcpp::Time(static_cast<uint64_t>(scn_state_.data.time * 1e9)); // convert to nanoseconds
#endif
		if (!ShouldRender(t)) {
			return;
		}
	}

	initial_published_ = true;

#if MJR_ROS_VERSION == ROS_1
	last_pub_ = ros::Time(scn_state_.data.time);

	bool segment = (stream_type_ & StreamType::SEGMENTED) &&
	               (segment_pub_.getNumSubscribers() > 0 || segment_camera_info_pub_->getNumSubscribers() > 0);
	bool rgb = (stream_type_ & StreamType::RGB) &&
	           (rgb_pub_.getNumSubscribers() > 0 || rgb_camera_info_pub_->getNumSubscribers() > 0);
	bool depth = (stream_type_ & StreamType::DEPTH) &&
	             (depth_pub_.getNumSubscribers() > 0 || depth_camera_info_pub_->getNumSubscribers() > 0);

#else // MJR_ROS_VERSION == ROS_2
	last_pub_ = rclcpp::Time(static_cast<uint64_t>(scn_state_.data.time * 1e9)); // convert to nanoseconds

	bool segment = (stream_type_ & StreamType::SEGMENTED) &&
	               (segment_pub_.getNumSubscribers() > 0 || segment_camera_info_pub_->get_subscription_count() > 0);
	bool rgb = (stream_type_ & StreamType::RGB) &&
	           (rgb_pub_.getNumSubscribers() > 0 || rgb_camera_info_pub_->get_subscription_count() > 0);
	bool depth = (stream_type_ & StreamType::DEPTH) &&
	             (depth_pub_.getNumSubscribers() > 0 || depth_camera_info_pub_->get_subscription_count() > 0);

#endif

	offscreen->cam.fixedcamid = cam_id_;

	// If rgb and segment are requested, we need to run two low-level render passes (segment is rgb with a different
	// flag).
	if (rgb && segment) { // first render RGB and maybe DEPTH, then SEGMENTED
		offscreen->scn.flags[mjRND_SEGMENT] = 0;
		RenderAndPubIfNecessary(offscreen, true, depth, false);

		offscreen->scn.flags[mjRND_SEGMENT] = 1;
		RenderAndPubIfNecessary(offscreen, false, false, true);
	} else { // render maybe RGB/SEGMENTED and maybe DEPTH
		offscreen->scn.flags[mjRND_SEGMENT] = segment;
		RenderAndPubIfNecessary(offscreen, rgb, depth, segment);
	}
}

} // namespace mujoco_ros::rendering
