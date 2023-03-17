/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022, Bielefeld University
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

#include <mujoco_ros/rendering/camera.h>
#include <geometry_msgs/PoseStamped.h>

#include <mujoco_ros/mujoco_sim.h>

namespace MujocoSim::rendering {

CameraStream::CameraStream(const uint8_t cam_id, const std::string cam_name, const int width, const int height,
                           const streamType stream_type, const bool use_segid, const float pub_freq,
                           image_transport::ImageTransport *it, ros::NodeHandlePtr parent_nh,
                           const MujocoSim::mjModelPtr model, MujocoSim::mjDataPtr data)
    : cam_id(cam_id)
    , cam_name(cam_name)
    , width(width)
    , height(height)
    , stream_type(stream_type)
    , use_segid(use_segid)
    , pub_freq(pub_freq)
{
	last_pub = ros::Time::now();
	ros::NodeHandlePtr nh(new ros::NodeHandle(*(parent_nh.get()), "cameras/" + cam_name));

	// pub_rel    = nh->advertise<geometry_msgs::PoseStamped>("campose_rel", 1, true);
	// pub_rel_tf = nh->advertise<geometry_msgs::PoseStamped>("campose_rel_tf", 1, true);
	// pub_cart   = nh->advertise<geometry_msgs::PoseStamped>("campose_cart", 1, true);

	if (stream_type & streamType::RGB) {
		ROS_DEBUG_NAMED("mujoco_env", "\tCreating rgb publisher");
		rgb_pub = it->advertise("cameras/" + cam_name + "/rgb", 1);
	}
	if (stream_type & streamType::DEPTH) {
		ROS_DEBUG_NAMED("mujoco_env", "\tCreating depth publisher");
		depth_pub = it->advertise("cameras/" + cam_name + "/depth", 1);
	}
	if (stream_type & streamType::SEGMENTED) {
		ROS_DEBUG_NAMED("mujoco_env", "\tCreating segmentation publisher");
		segment_pub = it->advertise("cameras/" + cam_name + "/segmented", 1);
	}

	ROS_DEBUG_STREAM_NAMED("mujoco_env", "\tSetting up camera stream(s) of type '"
	                                         << stream_type << "' with a publish rate of " << pub_freq
	                                         << " Hz for camera named " << cam_name << " with resolution " << width
	                                         << "x" << height);

	// Create static transform to camera
	// The camera is looking along the -Z axis of its frame. The +X axis points to the right, and the +Y axis points up.
	// https://mujoco.readthedocs.io/en/latest/XMLreference.html#body-camera

	int body_id              = model->cam_bodyid[cam_id];
	std::string parent_frame = mj_id2name(model.get(), mjOBJ_BODY, body_id);
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
	MujocoSim::registerStaticTransform(cam_transform);

	cam_transform.header.frame_id = cam_name + "_link";
	cam_transform.child_frame_id  = cam_name + "_optical_frame";

	cam_transform.transform.translation.x = 0;
	cam_transform.transform.translation.y = 0;
	cam_transform.transform.translation.z = 0;

	cam_transform.transform.rotation.w = 0;
	cam_transform.transform.rotation.x = -1.0;
	cam_transform.transform.rotation.y = 0;
	cam_transform.transform.rotation.z = 0;
	MujocoSim::registerStaticTransform(cam_transform);

	// init camera info manager
	camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(*(nh.get()), cam_name));

	// Get camera info
	mjtNum cam_pos[3];
	mju_copy(cam_pos, const_cast<const mjtNum *>(data->cam_xpos + cam_id * 3), 3);

	mjtNum fovy_rad      = (M_PI / 180 * model->cam_fovy[cam_id]);
	mjtNum focal_scaling = (1. / mju_tan(fovy_rad / 2.0)) * height / 2.0;

	std::string camera_frame_name(cam_name + "_optical_frame");

	sensor_msgs::CameraInfo ci;
	ci.header.frame_id = camera_frame_name;
	ci.width           = width;
	ci.height          = height;

	mjtNum extrinsic[12] = { focal_scaling,
		                      0.0,
		                      (width - 1) / 2.0,
		                      0, // cam_pos[0],
		                      0.0,
		                      focal_scaling,
		                      (height - 1) / 2.0,
		                      0, // cam_pos[1],
		                      0.0,
		                      0.0,
		                      1.0,
		                      0 }; // cam_pos[2]};

	// Copy extrinsic camera matrix to camera info
	mju_copy(ci.P.c_array(), extrinsic, 12);

	// Copy intrinsic Camera matrix to camera info
	mju_copy(ci.K.c_array(), extrinsic, 3);
	mju_copy(ci.K.c_array() + 3, extrinsic + 4, 3);
	mju_copy(ci.K.c_array() + 6, extrinsic + 8, 3);

	camera_info_manager_->setCameraInfo(ci);
	camera_info_pub_ = nh->advertise<sensor_msgs::CameraInfo>("camera_info", 1, true);

	// quatserver = nh->advertiseService("get_cam_info", &CameraStream::setCamQuatCB, this);
	// quatserver = nh->advertiseService("get_cam_info", &CameraStream::getCamTransform, this);
}

// bool CameraStream::getCamTransform(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {

//     mjtNum xpos[3];
//     mju_copy(xpos, MujocoSim::detail::main_env_->data->cam_xpos+cam_id*3, 3);
//     mjtNum rel_pos[3];
//     mju_copy(rel_pos, MujocoSim::detail::main_env_->model->cam_pos+cam_id*3, 3);

//     mjtNum xquat[4];
//     mjtNum xmat[9];
//     mju_copy(xmat, MujocoSim::detail::main_env_->data->cam_xmat+cam_id*9, 9);
//     mju_mat2Quat(xquat, xmat);
//     mjtNum rel_quat[4];
//     mju_copy(rel_quat, MujocoSim::detail::main_env_->model->cam_quat+cam_id*4, 4);

//     geometry_msgs::PoseStamped p_rel, p_rel_tf, p_cart;
//     p_cart.header.stamp = ros::Time::now();
//     p_cart.header.frame_id = "world";
//     p_cart.pose.position.x = xpos[0];
//     p_cart.pose.position.y = xpos[1];
//     p_cart.pose.position.z = xpos[2];
//     p_cart.pose.orientation.w = xquat[0];
//     p_cart.pose.orientation.x = xquat[1];
//     p_cart.pose.orientation.y = xquat[2];
//     p_cart.pose.orientation.z = xquat[3];

//     pub_cart.publish(p_cart);

//     p_rel.header.stamp = ros::Time::now();
//     p_rel.header.frame_id = cam_name+"_optical_frame";
//     p_rel.pose.position.x = 0;//rel_pos[0];
//     p_rel.pose.position.y = 0;//rel_pos[1];
//     p_rel.pose.position.z = 0;//rel_pos[2];
//     p_rel.pose.orientation.w = 1;//rel_quat[0];
//     p_rel.pose.orientation.x = 0;//rel_quat[1];
//     p_rel.pose.orientation.y = 0;//rel_quat[2];
//     p_rel.pose.orientation.z = 0;//rel_quat[3];

//     pub_rel.publish(p_rel);

//     p_rel_tf = MujocoSim::detail::tf_bufferPtr_->transform(p_rel, "world");

//     pub_rel_tf.publish(p_rel_tf);

//     ROS_INFO_STREAM(p_cart << "\n" << p_rel << "\n" << p_rel_tf);

//     return true;

// }

void CameraStream::publishCameraInfo()
{
	sensor_msgs::CameraInfo camera_info_msg = camera_info_manager_->getCameraInfo();
	camera_info_msg.header.stamp            = ros::Time::now();

	camera_info_pub_.publish(camera_info_msg);
}

} // namespace MujocoSim::rendering
