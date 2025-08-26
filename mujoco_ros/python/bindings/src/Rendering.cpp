/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2025, Bielefeld University
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

#include "pymujoco_ros.h"

#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/offscreen_camera.h>

#include <atomic>
#include <shared_mutex>

#include <math.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

namespace mujoco_ros::python::rendering {

class OffscreenCameraBuffer
{
public:
	mujoco_ros::rendering::OffscreenCamera *cam_;

	// Mutex to protect access to the buffers
	std::shared_mutex buffer_mutex_;

	std::atomic<int> current_rgb_index_ = { 0 };
	std::atomic<int> rgb_frame_count_   = { 0 };

	std::atomic<int> current_depth_index_ = { 0 };
	std::atomic<int> depth_frame_count_   = { 0 };

	std::atomic<int> current_segment_index_ = { 0 };
	std::atomic<int> segment_frame_count_   = { 0 };

private:
	// Buffers for RGB, depth and segmentation images
	uint8_t *rgb_buffer_;
	float *depth_buffer_;
	uint8_t *segment_buffer_;
	uint8_t buffer_size_;

	ros::Subscriber rgb_sub_;
	ros::Subscriber depth_sub_;
	ros::Subscriber segment_sub_;

public:
	OffscreenCameraBuffer(mujoco_ros::rendering::OffscreenCamera *cam, uint8_t buffer_size)
	    : cam_(cam), buffer_size_(buffer_size)
	{
		// Allocate buffers for RGB, depth and segmentation images based on the camera's stream type
		// and initialize subscribers for each.
		int width  = cam->width_;
		int height = cam->height_;

		if (cam->stream_type_ & mujoco_ros::rendering::streamType::RGB) {
			rgb_buffer_ = new uint8_t[buffer_size * width * height * 3];
			rgb_sub_    = ros::NodeHandle("~").subscribe(cam->topic_ + "/" + cam->rgb_topic_ + "/image_raw", buffer_size,
			                                             &OffscreenCameraBuffer::rgbCallback, this);
		}
		if (cam->stream_type_ & mujoco_ros::rendering::streamType::DEPTH) {
			depth_buffer_ = new float[buffer_size * width * height];
			depth_sub_ = ros::NodeHandle("~").subscribe(cam->topic_ + "/" + cam->depth_topic_ + "/image_raw", buffer_size,
			                                            &OffscreenCameraBuffer::depthCallback, this);
		}
		if (cam->stream_type_ & mujoco_ros::rendering::streamType::SEGMENTED) {
			segment_buffer_ = new uint8_t[buffer_size * width * height * 3];
			segment_sub_    = ros::NodeHandle("~").subscribe(cam->topic_ + "/" + cam->segment_topic_ + "/image_raw",
			                                                 buffer_size, &OffscreenCameraBuffer::segmentCallback, this);
		}
	}

	~OffscreenCameraBuffer()
	{
		if (cam_->stream_type_ & mujoco_ros::rendering::streamType::RGB) {
			delete[] rgb_buffer_;
		}
		if (cam_->stream_type_ & mujoco_ros::rendering::streamType::DEPTH) {
			delete[] depth_buffer_;
		}
		if (cam_->stream_type_ & mujoco_ros::rendering::streamType::SEGMENTED) {
			delete[] segment_buffer_;
		}
	}

	// Callbacks to handle incoming messages
	void rgbCallback(const sensor_msgs::ImageConstPtr &msg)
	{
		std::shared_lock lock(buffer_mutex_);
		std::memcpy(rgb_buffer_ + current_rgb_index_ * cam_->width_ * cam_->height_ * 3, msg->data.data(),
		            cam_->width_ * cam_->height_ * 3 * sizeof(uint8_t));
		// ROS_INFO_STREAM("RGB buffer updated, current index: " << current_rgb_index_.load() <<
		//     ", bytes: " << msg->data.size() <<
		//     ", buffer size: " << cam_->width_ * cam_->height_ * 3 * sizeof(uint8_t) <<
		//     ", in width: " << msg->width <<
		//     ", in height: " << msg->height <<
		//     ", starting at: " << (current_rgb_index_.load() * cam_->width_ * cam_->height_ * 3)
		//  );
		if (current_rgb_index_ < buffer_size_ - 1) {
			current_rgb_index_++;
		} else {
			current_rgb_index_.store(0);
		}
		rgb_frame_count_.store(std::min(rgb_frame_count_.load() + 1, (int)buffer_size_));
	}

	void depthCallback(const sensor_msgs::ImageConstPtr &msg)
	{
		std::shared_lock lock(buffer_mutex_);
		// Copy depth data into the buffer
		std::memcpy(depth_buffer_ + current_depth_index_ * cam_->width_ * cam_->height_, msg->data.data(),
		            cam_->width_ * cam_->height_ * sizeof(float));
		if (current_depth_index_ < buffer_size_ - 1) {
			current_depth_index_++;
		} else {
			current_depth_index_.store(0);
		}
		depth_frame_count_.store(std::min(depth_frame_count_.load() + 1, (int)buffer_size_));
	}

	void segmentCallback(const sensor_msgs::ImageConstPtr &msg)
	{
		std::shared_lock lock(buffer_mutex_);
		// Copy segmentation data into the buffer
		std::memcpy(segment_buffer_ + current_segment_index_ * cam_->width_ * cam_->height_ * 3, msg->data.data(),
		            cam_->width_ * cam_->height_ * 3 * sizeof(uint8_t));
		if (current_segment_index_ < buffer_size_ - 1) {
			current_segment_index_++;
		} else {
			current_segment_index_.store(0);
		}
		segment_frame_count_.store(std::min(segment_frame_count_.load() + 1, (int)buffer_size_));
	}

	void setFlag(int flag_idx, bool enable) { cam_->vopt_.flags[flag_idx] = enable ? 1 : 0; }

	void toggleFlag(int flag_idx)
	{
		cam_->vopt_.flags[flag_idx] ^= 1; // Toggle the specified flag in vopt
	}

	int getFlag(int flag_idx) { return cam_->vopt_.flags[flag_idx]; }

	py::tuple getBufferHandles()
	{
		py::array_t<uint8_t> rgb_array;
		py::array_t<float> depth_array;
		py::array_t<uint8_t> segment_array;

		if (cam_->stream_type_ & mujoco_ros::rendering::streamType::RGB) {
			py::capsule rgb_capsule(rgb_buffer_, [](void *p) { /* no action needed */ });
			rgb_array =
			    py::array_t<uint8_t>({ (ssize_t)buffer_size_, (ssize_t)cam_->height_, (ssize_t)cam_->width_, (ssize_t)3 },
			                         rgb_buffer_, rgb_capsule);
			rgb_array.attr("flags").attr("writeable") = false; // Make the array read-only
		}
		if (cam_->stream_type_ & mujoco_ros::rendering::streamType::DEPTH) {
			py::capsule depth_capsule(depth_buffer_, [](void *p) { /* no action needed */ });
			depth_array = py::array_t<float>({ (ssize_t)buffer_size_, (ssize_t)cam_->width_, (ssize_t)cam_->height_ },
			                                 depth_buffer_, depth_capsule);
			depth_array.attr("flags").attr("writeable") = false; // Make the array read-only
		}
		if (cam_->stream_type_ & mujoco_ros::rendering::streamType::SEGMENTED) {
			py::capsule segment_capsule(segment_buffer_, [](void *p) { /* no action needed */ });
			segment_array =
			    py::array_t<uint8_t>({ (ssize_t)buffer_size_, (ssize_t)cam_->width_, (ssize_t)cam_->height_, (ssize_t)3 },
			                         segment_buffer_, segment_capsule);
			segment_array.attr("flags").attr("writeable") = false; // Make the array read-only
		}

		return py::make_tuple(rgb_array, depth_array, segment_array);
	}
};

} // namespace mujoco_ros::python::rendering

namespace mujoco_ros::python {
using namespace mujoco_ros;

void InitRenderingPy(py::module &m)
{
	py::enum_<mujoco_ros::rendering::streamType>(m, "streamType")
	    .value("RGB", mujoco_ros::rendering::streamType::RGB)
	    .value("DEPTH", mujoco_ros::rendering::streamType::DEPTH)
	    .value("SEGMENTED", mujoco_ros::rendering::streamType::SEGMENTED);

	py::class_<mujoco_ros::python::rendering::OffscreenCameraBuffer>(m, "_OffscreenCameraBuffer")
	    .def(py::init<mujoco_ros::rendering::OffscreenCamera *, uint8_t>())
	    .def("getBufferHandles", &mujoco_ros::python::rendering::OffscreenCameraBuffer::getBufferHandles,
	         py::return_value_policy::reference_internal)
	    .def("_toggle_flag", &mujoco_ros::python::rendering::OffscreenCameraBuffer::toggleFlag, py::arg("flag_idx"))
	    .def("_set_flag", &mujoco_ros::python::rendering::OffscreenCameraBuffer::setFlag, py::arg("flag_idx"),
	         py::arg("enable") = true)
	    .def("_get_flag", &mujoco_ros::python::rendering::OffscreenCameraBuffer::getFlag, py::arg("flag_idx"))
	    .def_property_readonly(
	        "_rgb_buf_idx",
	        [](mujoco_ros::python::rendering::OffscreenCameraBuffer &self) { return self.current_rgb_index_.load(); })
	    .def_property_readonly(
	        "_depth_buf_idx",
	        [](mujoco_ros::python::rendering::OffscreenCameraBuffer &self) { return self.current_depth_index_.load(); })
	    .def_property_readonly("_segment_buf_idx",
	                           [](mujoco_ros::python::rendering::OffscreenCameraBuffer &self) {
		                           return self.current_segment_index_.load();
	                           })
	    .def_property(
	        "_rgb_frame_count",
	        [](mujoco_ros::python::rendering::OffscreenCameraBuffer &self) { return self.rgb_frame_count_.load(); },
	        [](mujoco_ros::python::rendering::OffscreenCameraBuffer &self, int count) {
		        self.rgb_frame_count_.store(count);
	        })
	    .def_property(
	        "_depth_frame_count",
	        [](mujoco_ros::python::rendering::OffscreenCameraBuffer &self) { return self.depth_frame_count_.load(); },
	        [](mujoco_ros::python::rendering::OffscreenCameraBuffer &self, int count) {
		        self.depth_frame_count_.store(count);
	        })
	    .def_property(
	        "_segment_frame_count",
	        [](mujoco_ros::python::rendering::OffscreenCameraBuffer &self) { return self.segment_frame_count_.load(); },
	        [](mujoco_ros::python::rendering::OffscreenCameraBuffer &self, int count) {
		        self.segment_frame_count_.store(count);
	        })
	    .def("set_buffers_read",
	         [](mujoco_ros::python::rendering::OffscreenCameraBuffer &self) {
		         // Lock the buffer mutex to ensure thread safety
		         self.rgb_frame_count_.store(0);
		         self.depth_frame_count_.store(0);
		         self.segment_frame_count_.store(0);
	         })
	    .def("__enter__",
	         [](mujoco_ros::python::rendering::OffscreenCameraBuffer &self) {
		         // Lock the buffer mutex to ensure thread safety;
		         self.buffer_mutex_.lock();
	         })
	    .def("__exit__", [](mujoco_ros::python::rendering::OffscreenCameraBuffer &self, py::object exc_type,
	                        py::object exc_value, py::object traceback) {
		    // Unlock the buffer mutex when done
		    self.buffer_mutex_.unlock();
	    });

	py::class_<mujoco_ros::rendering::OffscreenCamera>(m, "_OffscreenCamera")
	    .def_readonly("id", &mujoco_ros::rendering::OffscreenCamera::cam_id_)
	    .def_readonly("name", &mujoco_ros::rendering::OffscreenCamera::cam_name_)
	    .def_readonly("topic", &mujoco_ros::rendering::OffscreenCamera::topic_)
	    .def_readonly("width", &mujoco_ros::rendering::OffscreenCamera::width_)
	    .def_readonly("height", &mujoco_ros::rendering::OffscreenCamera::height_)
	    .def_readonly("stream_type", &mujoco_ros::rendering::OffscreenCamera::stream_type_)
	    .def_readonly("use_segid", &mujoco_ros::rendering::OffscreenCamera::use_segid_)
	    .def_readwrite("pub_freq", &mujoco_ros::rendering::OffscreenCamera::pub_freq_)
	    .def("__repr__", [](const mujoco_ros::rendering::OffscreenCamera &self) {
		    return "<OffscreenCamera id='" + self.cam_name_ + "'>";
	    });

	py::class_<mujoco_ros::OffscreenRenderContext>(m, "_OffscreenRenderContext")
	    .def("trigger_render_request",
	         [](mujoco_ros::OffscreenRenderContext &self) { return self.cond_render_request.notify_one(); })
	    .def("__enter__",
	         [](mujoco_ros::OffscreenRenderContext &self) {
		         self.render_mutex.lock();
		         return py::cast(self);
	         })
	    .def("__exit__", [](mujoco_ros::OffscreenRenderContext &self, py::object exc_type, py::object exc_value,
	                        py::object traceback) { self.render_mutex.unlock(); })
	    .def(
	        "camera",
	        [](mujoco_ros::OffscreenRenderContext &self, uint8_t cam_id) -> mujoco_ros::rendering::OffscreenCamera & {
		        uint8_t imax = 0;
		        for (auto &cam : self.cams) {
			        if (cam->cam_id_ == cam_id) {
				        return *cam.get();
			        }
			        imax = std::max(imax, cam->cam_id_);
		        }
		        throw std::out_of_range("Invalid index " + std::to_string(cam_id) + ". Valid indices from 0 to " +
		                                std::to_string(imax));
	        },
	        py::arg("id"), py::return_value_policy::reference_internal)
	    .def(
	        "camera",
	        [](mujoco_ros::OffscreenRenderContext &self,
	           std::string cam_name) -> mujoco_ros::rendering::OffscreenCamera & {
		        for (auto &cam : self.cams) {
			        if (cam->cam_name_ == cam_name) {
				        return *cam.get();
			        }
		        }
		        throw std::out_of_range("Invalid camera name " + cam_name);
	        },
	        py::arg("name"), py::return_value_policy::reference_internal);
}

} // namespace mujoco_ros::python
