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

/* Authors: David P. Leins*/

#include "pymujoco_ros.hpp"

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/offscreen_camera.hpp>

#include <pybind11/numpy.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <thread>

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
using ImageConstSharedPtr = sensor_msgs::ImageConstPtr;
#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
using ImageConstSharedPtr = sensor_msgs::msg::Image::ConstSharedPtr;
#endif

namespace mujoco_ros::python {
namespace {

std::string ImageTopic(const mujoco_ros::rendering::OffscreenCamera &cam, const char *image_type)
{
#if MJR_ROS_VERSION == ROS_1
	return cam.topic_ + "/" + image_type + "/image_raw";
#else
	return std::string(cam.nh_->get_effective_namespace()) + "/" + image_type + "/image_raw";
#endif
}

class OffscreenCameraBuffer
{
public:
	OffscreenCameraBuffer(mujoco_ros::rendering::OffscreenCamera *cam, std::uint8_t buffer_size)
	    : cam_(cam), buffer_size_(std::max<std::uint8_t>(buffer_size, 1))
	{
		const auto pixel_count =
		    static_cast<size_t>(buffer_size_) * static_cast<size_t>(cam_->width_) * static_cast<size_t>(cam_->height_);
		if (cam_->stream_type_ & mujoco_ros::rendering::StreamType::RGB) {
			rgb_buffer_ = std::make_unique<std::uint8_t[]>(pixel_count * 3);
			SubscribeRgb();
		}
		if (cam_->stream_type_ & mujoco_ros::rendering::StreamType::DEPTH) {
			depth_buffer_ = std::make_unique<float[]>(pixel_count);
			SubscribeDepth();
		}
		if (cam_->stream_type_ & mujoco_ros::rendering::StreamType::SEGMENTED) {
			segment_buffer_ = std::make_unique<std::uint8_t[]>(pixel_count * 3);
			SubscribeSegment();
		}
#if MJR_ROS_VERSION == ROS_2
		if (node_) {
			executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
			executor_->add_node(node_);
			executor_thread_ = std::thread([this]() { executor_->spin(); });
		}
#endif
	}

	~OffscreenCameraBuffer()
	{
#if MJR_ROS_VERSION == ROS_2
		if (executor_) {
			try {
				executor_->cancel();
			} catch (...) {
			}
		}
		if (executor_thread_.joinable()) {
			executor_thread_.join();
		}
		if (executor_ && node_) {
			try {
				executor_->remove_node(node_);
			} catch (...) {
			}
		}
#endif
	}

	void SetFlag(int flag_idx, bool enable) { cam_->vopt_.flags[flag_idx] = enable ? 1 : 0; }

	void ToggleFlag(int flag_idx) { cam_->vopt_.flags[flag_idx] ^= 1; }

	int GetFlag(int flag_idx) const { return cam_->vopt_.flags[flag_idx]; }

	py::tuple GetBufferHandles()
	{
		const py::ssize_t buffer_size = static_cast<py::ssize_t>(buffer_size_);
		const py::ssize_t height      = static_cast<py::ssize_t>(cam_->height_);
		const py::ssize_t width       = static_cast<py::ssize_t>(cam_->width_);
		const py::ssize_t channels    = 3;

		py::object rgb     = py::none();
		py::object depth   = py::none();
		py::object segment = py::none();

		if (rgb_buffer_) {
			py::capsule capsule(rgb_buffer_.get(), [](void *) {});
			rgb = py::array_t<std::uint8_t>({ buffer_size, height, width, channels }, rgb_buffer_.get(), capsule);
			rgb.attr("flags").attr("writeable") = false;
		}
		if (depth_buffer_) {
			py::capsule capsule(depth_buffer_.get(), [](void *) {});
			depth = py::array_t<float>({ buffer_size, height, width }, depth_buffer_.get(), capsule);
			depth.attr("flags").attr("writeable") = false;
		}
		if (segment_buffer_) {
			py::capsule capsule(segment_buffer_.get(), [](void *) {});
			segment = py::array_t<std::uint8_t>({ buffer_size, height, width, channels }, segment_buffer_.get(), capsule);
			segment.attr("flags").attr("writeable") = false;
		}
		return py::make_tuple(rgb, depth, segment);
	}

	void ResetFrameCounts()
	{
		rgb_frame_count_.store(0);
		depth_frame_count_.store(0);
		segment_frame_count_.store(0);
	}

	void Lock() { buffer_mutex_.lock(); }

	void Unlock() { buffer_mutex_.unlock(); }

	int RgbIndex() const { return current_rgb_index_.load(); }

	int DepthIndex() const { return current_depth_index_.load(); }

	int SegmentIndex() const { return current_segment_index_.load(); }

	int RgbFrameCount() const { return rgb_frame_count_.load(); }

	void SetRgbFrameCount(int count) { rgb_frame_count_.store(count); }

	int DepthFrameCount() const { return depth_frame_count_.load(); }

	void SetDepthFrameCount(int count) { depth_frame_count_.store(count); }

	int SegmentFrameCount() const { return segment_frame_count_.load(); }

	void SetSegmentFrameCount(int count) { segment_frame_count_.store(count); }

private:
	void CopyRgb(const ImageConstSharedPtr &msg)
	{
		CopyBytes(msg, rgb_buffer_.get(), current_rgb_index_, rgb_frame_count_, 3 * sizeof(std::uint8_t));
	}

	void CopyDepth(const ImageConstSharedPtr &msg)
	{
		CopyBytes(msg, depth_buffer_.get(), current_depth_index_, depth_frame_count_, sizeof(float));
	}

	void CopySegment(const ImageConstSharedPtr &msg)
	{
		CopyBytes(msg, segment_buffer_.get(), current_segment_index_, segment_frame_count_, 3 * sizeof(std::uint8_t));
	}

	template <typename T>
	void CopyBytes(const ImageConstSharedPtr &msg, T *target, std::atomic<int> &index, std::atomic<int> &frame_count,
	               size_t bytes_per_pixel)
	{
		if (target == nullptr) {
			return;
		}
		const size_t frame_bytes =
		    static_cast<size_t>(cam_->width_) * static_cast<size_t>(cam_->height_) * bytes_per_pixel;
		if (msg->data.size() < frame_bytes) {
			return;
		}

		std::unique_lock lock(buffer_mutex_);
		std::memcpy(reinterpret_cast<std::uint8_t *>(target) + static_cast<size_t>(index.load()) * frame_bytes,
		            msg->data.data(), frame_bytes);
		if (index.load() < static_cast<int>(buffer_size_) - 1) {
			++index;
		} else {
			index.store(0);
		}
		frame_count.store(std::min(frame_count.load() + 1, static_cast<int>(buffer_size_)));
	}

	void SubscribeRgb()
	{
#if MJR_ROS_VERSION == ROS_1
		rgb_sub_ =
		    ros::NodeHandle("~").subscribe(ImageTopic(*cam_, "rgb"), buffer_size_, &OffscreenCameraBuffer::CopyRgb, this);
#else
		EnsureNode();
		rgb_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
		    ImageTopic(*cam_, "rgb"), static_cast<size_t>(buffer_size_),
		    [this](const ImageConstSharedPtr msg) { CopyRgb(msg); });
#endif
	}

	void SubscribeDepth()
	{
#if MJR_ROS_VERSION == ROS_1
		depth_sub_ = ros::NodeHandle("~").subscribe(ImageTopic(*cam_, "depth"), buffer_size_,
		                                            &OffscreenCameraBuffer::CopyDepth, this);
#else
		EnsureNode();
		depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
		    ImageTopic(*cam_, "depth"), static_cast<size_t>(buffer_size_),
		    [this](const ImageConstSharedPtr msg) { CopyDepth(msg); });
#endif
	}

	void SubscribeSegment()
	{
#if MJR_ROS_VERSION == ROS_1
		segment_sub_ = ros::NodeHandle("~").subscribe(ImageTopic(*cam_, "segmented"), buffer_size_,
		                                              &OffscreenCameraBuffer::CopySegment, this);
#else
		EnsureNode();
		segment_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
		    ImageTopic(*cam_, "segmented"), static_cast<size_t>(buffer_size_),
		    [this](const ImageConstSharedPtr msg) { CopySegment(msg); });
#endif
	}

#if MJR_ROS_VERSION == ROS_2
	void EnsureNode()
	{
		if (!node_) {
			node_ = std::make_shared<rclcpp::Node>("pymujoco_ros_offscreen_buffer_" + std::to_string(cam_->cam_id_));
		}
	}
#endif

	mujoco_ros::rendering::OffscreenCamera *cam_;
	std::shared_mutex buffer_mutex_;

	std::atomic<int> current_rgb_index_     = { 0 };
	std::atomic<int> rgb_frame_count_       = { 0 };
	std::atomic<int> current_depth_index_   = { 0 };
	std::atomic<int> depth_frame_count_     = { 0 };
	std::atomic<int> current_segment_index_ = { 0 };
	std::atomic<int> segment_frame_count_   = { 0 };

	std::unique_ptr<std::uint8_t[]> rgb_buffer_;
	std::unique_ptr<float[]> depth_buffer_;
	std::unique_ptr<std::uint8_t[]> segment_buffer_;
	std::uint8_t buffer_size_;

#if MJR_ROS_VERSION == ROS_1
	ros::Subscriber rgb_sub_;
	ros::Subscriber depth_sub_;
	ros::Subscriber segment_sub_;
#else
	rclcpp::Node::SharedPtr node_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr segment_sub_;
	rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
	std::thread executor_thread_;
#endif
};

} // namespace

void InitRendering(py::module_ &module)
{
	py::enum_<mujoco_ros::rendering::StreamType>(module, "StreamType")
	    .value("RGB", mujoco_ros::rendering::StreamType::RGB)
	    .value("DEPTH", mujoco_ros::rendering::StreamType::DEPTH)
	    .value("SEGMENTED", mujoco_ros::rendering::StreamType::SEGMENTED)
	    .value("RGB_D", mujoco_ros::rendering::StreamType::RGB_D)
	    .value("RGB_S", mujoco_ros::rendering::StreamType::RGB_S)
	    .value("DEPTH_S", mujoco_ros::rendering::StreamType::DEPTH_S)
	    .value("RGB_D_S", mujoco_ros::rendering::StreamType::RGB_D_S);

	py::class_<OffscreenCameraBuffer>(module, "_OffscreenCameraBuffer")
	    .def(py::init<mujoco_ros::rendering::OffscreenCamera *, std::uint8_t>(), py::arg("camera"),
	         py::arg("buffer_size") = 1)
	    .def("getBufferHandles", &OffscreenCameraBuffer::GetBufferHandles, py::return_value_policy::reference_internal)
	    .def("_toggle_flag", &OffscreenCameraBuffer::ToggleFlag, py::arg("flag_idx"))
	    .def("_set_flag", &OffscreenCameraBuffer::SetFlag, py::arg("flag_idx"), py::arg("enable") = true)
	    .def("_get_flag", &OffscreenCameraBuffer::GetFlag, py::arg("flag_idx"))
	    .def_property_readonly("_rgb_buf_idx", &OffscreenCameraBuffer::RgbIndex)
	    .def_property_readonly("_depth_buf_idx", &OffscreenCameraBuffer::DepthIndex)
	    .def_property_readonly("_segment_buf_idx", &OffscreenCameraBuffer::SegmentIndex)
	    .def_property("_rgb_frame_count", &OffscreenCameraBuffer::RgbFrameCount,
	                  &OffscreenCameraBuffer::SetRgbFrameCount)
	    .def_property("_depth_frame_count", &OffscreenCameraBuffer::DepthFrameCount,
	                  &OffscreenCameraBuffer::SetDepthFrameCount)
	    .def_property("_segment_frame_count", &OffscreenCameraBuffer::SegmentFrameCount,
	                  &OffscreenCameraBuffer::SetSegmentFrameCount)
	    .def("set_buffers_read", &OffscreenCameraBuffer::ResetFrameCounts)
	    .def("__enter__",
	         [](OffscreenCameraBuffer &self) -> OffscreenCameraBuffer & {
		         self.Lock();
		         return self;
	         })
	    .def("__exit__", [](OffscreenCameraBuffer &self, py::object, py::object, py::object) { self.Unlock(); });

	py::class_<mujoco_ros::rendering::OffscreenCamera>(module, "_OffscreenCamera")
	    .def_readonly("id", &mujoco_ros::rendering::OffscreenCamera::cam_id_)
	    .def_readonly("name", &mujoco_ros::rendering::OffscreenCamera::cam_name_)
	    .def_readonly("topic", &mujoco_ros::rendering::OffscreenCamera::topic_)
	    .def_readonly("width", &mujoco_ros::rendering::OffscreenCamera::width_)
	    .def_readonly("height", &mujoco_ros::rendering::OffscreenCamera::height_)
	    .def_readonly("stream_type", &mujoco_ros::rendering::OffscreenCamera::stream_type_)
	    .def_readonly("use_segid", &mujoco_ros::rendering::OffscreenCamera::use_segid_)
	    .def_readwrite("pub_freq", &mujoco_ros::rendering::OffscreenCamera::pub_freq_)
	    .def("__repr__", [](const mujoco_ros::rendering::OffscreenCamera &camera) {
		    return "<OffscreenCamera name='" + camera.cam_name_ + "'>";
	    });

	py::class_<mujoco_ros::OffscreenRenderContext>(module, "_OffscreenRenderContext")
	    .def("trigger_render_request",
	         [](mujoco_ros::OffscreenRenderContext &self) { self.cond_render_request.notify_one(); })
	    .def("__enter__",
	         [](mujoco_ros::OffscreenRenderContext &self) -> mujoco_ros::OffscreenRenderContext & {
		         self.render_mutex.lock();
		         return self;
	         })
	    .def("__exit__", [](mujoco_ros::OffscreenRenderContext &self, py::object, py::object,
	                        py::object) { self.render_mutex.unlock(); })
	    .def(
	        "camera",
	        [](mujoco_ros::OffscreenRenderContext &self,
	           std::uint8_t cam_id) -> mujoco_ros::rendering::OffscreenCamera & {
		        for (auto &cam : self.cams) {
			        if (cam->cam_id_ == cam_id) {
				        return *cam;
			        }
		        }
		        throw std::out_of_range("Invalid camera id " + std::to_string(cam_id));
	        },
	        py::arg("id"), py::return_value_policy::reference_internal)
	    .def(
	        "camera",
	        [](mujoco_ros::OffscreenRenderContext &self,
	           const std::string &cam_name) -> mujoco_ros::rendering::OffscreenCamera & {
		        for (auto &cam : self.cams) {
			        if (cam->cam_name_ == cam_name) {
				        return *cam;
			        }
		        }
		        throw std::out_of_range("Invalid camera name " + cam_name);
	        },
	        py::arg("name"), py::return_value_policy::reference_internal)
	    .def_property_readonly("num_cams",
	                           [](const mujoco_ros::OffscreenRenderContext &self) { return self.cams.size(); });
}

} // namespace mujoco_ros::python
