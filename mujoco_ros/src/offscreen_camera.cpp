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

#include <mujoco_ros/offscreen_camera.h>
#include <geometry_msgs/PoseStamped.h>

#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/util.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

namespace mujoco_ros::rendering {

OffscreenCamera::OffscreenCamera(const uint8_t cam_id, const std::string &base_topic, const std::string &rgb_topic,
                                 const std::string &depth_topic, const std::string &segment_topic,
                                 const std::string &cam_name, const int width, const int height,
                                 const streamType stream_type, const bool use_segid, const float pub_freq,
                                 const ros::NodeHandle &parent_nh, const mjModel *model, mjData *data,
                                 mujoco_ros::MujocoEnv *env_ptr)
    : cam_id_(cam_id)
    , cam_name_(cam_name)
    , topic_(base_topic)
    , width_(width)
    , height_(height)
    , stream_type_(stream_type)
    , use_segid_(use_segid)
    , pub_freq_(pub_freq)
    , nh_(parent_nh, base_topic)
    , it_(nh_)
{
	last_pub_ = ros::Time::now();

	mjv_defaultOption(&vopt_);
	mjv_defaultSceneState(&scn_state_);
	mjv_makeSceneState(const_cast<mjModel *>(model), data, &scn_state_, Viewer::kMaxGeom);

	if (stream_type & streamType::RGB) {
		ROS_DEBUG_NAMED("mujoco_env", "\tCreating rgb publisher");
		rgb_pub_             = it_.advertise(rgb_topic + "/image_raw", 1);
		rgb_camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(rgb_topic + "/camera_info", 1, true);
	}
	if (stream_type & streamType::DEPTH) {
		ROS_DEBUG_NAMED("mujoco_env", "\tCreating depth publisher");
		depth_pub_             = it_.advertise(depth_topic + "/image_raw", 1);
		depth_camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(depth_topic + "/camera_info", 1, true);
	}
	if (stream_type & streamType::SEGMENTED) {
		ROS_DEBUG_NAMED("mujoco_env", "\tCreating segmentation publisher");
		segment_pub_             = it_.advertise(segment_topic + "/image_raw", 1);
		segment_camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(segment_topic + "/camera_info", 1, true);
	}

	ROS_DEBUG_STREAM_NAMED("mujoco_env", "\tSetting up camera stream(s) of type '"
	                                         << stream_type << "' with a publish rate of " << pub_freq
	                                         << " Hz for camera named " << cam_name << " with resolution " << width
	                                         << "x" << height);

	// Create static transform to camera
	// The camera is looking along the -Z axis of its frame. The +X axis points to the right, and the +Y axis points up.
	// https://mujoco.readthedocs.io/en/latest/XMLreference.html#body-camera

	int body_id              = model->cam_bodyid[cam_id];
	std::string parent_frame = mj_id2name(const_cast<mjModel *>(model), mjOBJ_BODY, body_id);
	ROS_DEBUG_STREAM("Creating camera frames for cam '" << cam_name << "' with parent link " << parent_frame);

	geometry_msgs::TransformStamped cam_transform;
	cam_transform.header.stamp            = ros::Time::now();
	cam_transform.header.frame_id         = parent_frame;
	cam_transform.child_frame_id          = cam_name + "_link";
	cam_transform.transform.translation.x = model->cam_pos[cam_id * 3];
	cam_transform.transform.translation.y = model->cam_pos[cam_id * 3 + 1];
	cam_transform.transform.translation.z = model->cam_pos[cam_id * 3 + 2];

	cam_transform.transform.rotation.w = model->cam_quat[cam_id * 4];
	cam_transform.transform.rotation.x = model->cam_quat[cam_id * 4 + 1];
	cam_transform.transform.rotation.y = model->cam_quat[cam_id * 4 + 2];
	cam_transform.transform.rotation.z = model->cam_quat[cam_id * 4 + 3];
	env_ptr->registerStaticTransform(cam_transform);

	cam_transform.header.frame_id = cam_name + "_link";
	cam_transform.child_frame_id  = cam_name + "_optical_frame";

	cam_transform.transform.translation.x = 0;
	cam_transform.transform.translation.y = 0;
	cam_transform.transform.translation.z = 0;

	cam_transform.transform.rotation.w = 0;
	cam_transform.transform.rotation.x = -1.0;
	cam_transform.transform.rotation.y = 0;
	cam_transform.transform.rotation.z = 0;
	env_ptr->registerStaticTransform(cam_transform);

	// init camera info manager
	camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(nh_, cam_name);

	// Get camera info
	mjtNum cam_pos[3];
	mju_copy(cam_pos, const_cast<const mjtNum *>(data->cam_xpos + cam_id * 3), 3);

	mjtNum fovy_rad      = (M_PI / 180 * model->cam_fovy[cam_id]);
	mjtNum focal_scaling = (1. / mju_tan(fovy_rad / 2.0)) * height / 2.0;

	std::string camera_frame_name(cam_name + "_optical_frame");

	sensor_msgs::CameraInfo ci;
	ci.header.frame_id = camera_frame_name;
	ci.width           = static_cast<decltype(ci.width)>(width);
	ci.height          = static_cast<decltype(ci.height)>(height);

	// clang-format off
	mjtNum extrinsic[12] = {
		focal_scaling, 	0.0, 			(width - 1) / 2.0, 	0.0,
		0.0, 			focal_scaling, 	(height - 1) / 2.0, 0.0,
		0.0, 			0.0, 			1.0, 				0.0
	};
	// clang-format on

	// Copy extrinsic camera matrix to camera info
	mju_copy(ci.P.c_array(), extrinsic, 12);

	// Copy intrinsic Camera matrix to camera info
	mju_copy(ci.K.c_array(), extrinsic, 3);
	mju_copy(ci.K.c_array() + 3, extrinsic + 4, 3);
	mju_copy(ci.K.c_array() + 6, extrinsic + 8, 3);

	camera_info_manager_->setCameraInfo(ci);
}

bool OffscreenCamera::shouldRender(const ros::Time &t)
{
	return (!initial_published_ ||
	        (last_pub_ != t && ros::Duration(1.0 / static_cast<double>(pub_freq_)) < t - last_pub_));
}

bool OffscreenCamera::renderAndPubIfNecessary(mujoco_ros::OffscreenRenderContext *offscreen, const bool rgb,
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

	// create info msg
	auto ros_time                           = ros::Time(scn_state_.data.time);
	sensor_msgs::CameraInfo camera_info_msg = camera_info_manager_->getCameraInfo();
	camera_info_msg.header.stamp            = ros_time;

	if (rgb_or_s) {
		// Publish RGB image
		sensor_msgs::ImagePtr rgb_msg = boost::make_shared<sensor_msgs::Image>();
		rgb_msg->header.frame_id      = cam_name_ + "_optical_frame";
		rgb_msg->header.stamp         = ros_time;
		rgb_msg->width                = static_cast<decltype(rgb_msg->width)>(viewport.width);
		rgb_msg->height               = static_cast<decltype(rgb_msg->height)>(viewport.height);
		rgb_msg->encoding             = sensor_msgs::image_encodings::RGB8;
		rgb_msg->step                 = static_cast<decltype(rgb_msg->step)>(viewport.width) * 3u * sizeof(unsigned char);
		size_t size                   = rgb_msg->step * static_cast<uint>(viewport.height);
		rgb_msg->data.resize(size);

		memcpy(reinterpret_cast<char *>(&rgb_msg->data[0]), offscreen->rgb.get(), size);

		for (uint r = 0; r < rgb_msg->height / 2; ++r) {
			unsigned char *top_row    = &rgb_msg->data[3 * rgb_msg->width * r];
			unsigned char *bottom_row = &rgb_msg->data[3 * rgb_msg->width * (rgb_msg->height - 1 - r)];
			std::swap_ranges(top_row, top_row + 3 * rgb_msg->width, bottom_row);
		}

		if (segment) {
			segment_pub_.publish(rgb_msg);
			segment_camera_info_pub_.publish(camera_info_msg);
		} else {
			rgb_pub_.publish(rgb_msg);
			rgb_camera_info_pub_.publish(camera_info_msg);
		}
	}

	if (depth) {
		// Publish DEPTH image
		sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
		depth_msg->header.frame_id      = cam_name_ + "_optical_frame";
		depth_msg->header.stamp         = ros_time;
		depth_msg->width                = util::as_unsigned(viewport.width);
		depth_msg->height               = util::as_unsigned(viewport.height);
		depth_msg->encoding             = sensor_msgs::image_encodings::TYPE_32FC1;
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
		depth_camera_info_pub_.publish(camera_info_msg);
	}

	return true;
}

void OffscreenCamera::renderAndPublish(mujoco_ros::OffscreenRenderContext *offscreen)
{
	if (!shouldRender(ros::Time(scn_state_.data.time))) {
		return;
	}

	initial_published_ = true;

	last_pub_ = ros::Time(scn_state_.data.time);

	bool segment = (stream_type_ & streamType::SEGMENTED) &&
	               (segment_pub_.getNumSubscribers() > 0 || segment_camera_info_pub_.getNumSubscribers() > 0);
	bool rgb = (stream_type_ & streamType::RGB) &&
	           (rgb_pub_.getNumSubscribers() > 0 || rgb_camera_info_pub_.getNumSubscribers() > 0);
	bool depth = (stream_type_ & streamType::DEPTH) &&
	             (depth_pub_.getNumSubscribers() > 0 || depth_camera_info_pub_.getNumSubscribers() > 0);

	offscreen->cam.fixedcamid = cam_id_;

	// If rgb and segment are requested, we need to run two low-level render passes (segment is rgb with a different
	// flag).
	if (rgb && segment) { // first render RGB and maybe DEPTH, then SEGMENTED
		offscreen->scn.flags[mjRND_SEGMENT] = 0;
		renderAndPubIfNecessary(offscreen, true, depth, false);

		offscreen->scn.flags[mjRND_SEGMENT] = 1;
		renderAndPubIfNecessary(offscreen, false, false, true);
	} else { // render maybe RGB/SEGMENTED and maybe DEPTH
		offscreen->scn.flags[mjRND_SEGMENT] = segment;
		renderAndPubIfNecessary(offscreen, rgb, depth, segment);
	}
}

} // namespace mujoco_ros::rendering
