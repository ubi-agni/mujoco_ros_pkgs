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
#include <mujoco_ros/offscreen_camera_config.hpp>
#include <mujoco_ros/offscreen_transport.hpp>
#include <mujoco_ros/rendering/frame_capacity.hpp>

#include <pybind11/numpy.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace mujoco_ros::python {
namespace {

using namespace mujoco_ros::rendering;

py::array MakeCopiedArray(const std::vector<FrameLease> &leases, PlaneKind plane)
{
	if (leases.empty()) {
		return py::array();
	}
	const auto &layout = leases.front().layout();
	const auto count   = static_cast<py::ssize_t>(leases.size());
	py::array result;
	if (plane == PlaneKind::kDepth) {
		py::array::ShapeContainer shape(std::vector<py::ssize_t>{ count, static_cast<py::ssize_t>(layout.height),
		                                                          static_cast<py::ssize_t>(layout.width) });
		result = py::array_t<float>(shape);
	} else {
		py::array::ShapeContainer shape(std::vector<py::ssize_t>{ count, static_cast<py::ssize_t>(layout.height),
		                                                          static_cast<py::ssize_t>(layout.width), 3 });
		result = py::array_t<std::uint8_t>(shape);
	}
	for (py::ssize_t index = 0; index < count; ++index) {
		const auto &source = leases[static_cast<std::size_t>(index)];
		if (source.layout().byte_length != layout.byte_length) {
			throw std::runtime_error("frame snapshot contains inconsistent plane layouts");
		}
		std::memcpy(static_cast<std::uint8_t *>(result.mutable_data()) +
		                static_cast<std::size_t>(index) * layout.byte_length,
		            source.bytes().data(), layout.byte_length);
	}
	result.attr("setflags")(false);
	return result;
}

class BorrowedFrame
{
public:
	BorrowedFrame(std::optional<FrameLease> lease, PlaneKind plane)
	{
		if (!lease) {
			throw std::runtime_error("requested render plane has no committed frame");
		}
		lease_             = std::make_shared<FrameLease>(std::move(*lease));
		const auto &layout = lease_->layout();
		auto holder        = new std::shared_ptr<FrameLease>(lease_);
		py::capsule capsule(holder, [](void *value) { delete static_cast<std::shared_ptr<FrameLease> *>(value); });
		if (plane == PlaneKind::kDepth) {
			py::array::ShapeContainer shape(std::vector<py::ssize_t>{ static_cast<py::ssize_t>(layout.height),
			                                                          static_cast<py::ssize_t>(layout.width) });
			py::array::StridesContainer strides(std::vector<py::ssize_t>{ static_cast<py::ssize_t>(layout.stride_bytes),
			                                                              static_cast<py::ssize_t>(sizeof(float)) });
			view_ =
			    py::array_t<float>(shape, strides, reinterpret_cast<const float *>((*holder)->bytes().data()), capsule);
		} else {
			py::array::ShapeContainer shape(std::vector<py::ssize_t>{ static_cast<py::ssize_t>(layout.height),
			                                                          static_cast<py::ssize_t>(layout.width), 3 });
			py::array::StridesContainer strides(
			    std::vector<py::ssize_t>{ static_cast<py::ssize_t>(layout.stride_bytes), 3, 1 });
			view_ = py::array_t<std::uint8_t>(shape, strides,
			                                  reinterpret_cast<const std::uint8_t *>((*holder)->bytes().data()), capsule);
		}
		view_.attr("setflags")(false);
	}

	py::array Enter() const { return view_; }
	void Exit() { view_ = py::array(); }
	std::uint64_t CaptureId() const { return lease_->capture_id(); }
	std::uint64_t FrameGeneration() const { return lease_->generation().value(); }
	std::size_t Size() const { return lease_->bytes().size(); }

private:
	std::shared_ptr<FrameLease> lease_;
	py::array view_;
};

class OffscreenCameraBuffer
{
public:
	OffscreenCameraBuffer(mujoco_ros::MujocoEnv *env, mujoco_ros::CameraPublicationTransport *transport,
	                      const std::string &camera_name, std::uint8_t buffer_size)
	    : env_(env)
	    , transport_(transport)
	    , camera_name_(camera_name)
	    , buffer_size_(std::max<std::uint8_t>(buffer_size, 1))
	{
		if (env_ == nullptr || transport_ == nullptr) {
			throw std::runtime_error("Python rendering context is unavailable");
		}
		mujoco_ros::rendering::ValidatePythonHistoryDepth(buffer_size_);
		config_ = env_->OpenOffscreenCamera(camera_name_);
		config_->EnableFrames(buffer_size_);
	}

	~OffscreenCameraBuffer() { Close(); }

	void Close()
	{
		if (closed_)
			return;
		closed_ = true;
		config_.reset();
	}

	void SetGeomGroup(int group, bool enable)
	{
		EnsureOpen();
		config_->SetGeomGroup(group, enable);
	}
	void SetFlag(int flag_idx, bool enable)
	{
		EnsureOpen();
		config_->SetVisualFlag(flag_idx, enable);
	}
	void ToggleFlag(int flag_idx)
	{
		EnsureOpen();
		const bool enabled = config_->VisualFlagEnabled(flag_idx);
		config_->SetVisualFlag(flag_idx, !enabled);
	}
	int GetFlag(int flag_idx) const
	{
		EnsureOpen();
		return config_->VisualFlagEnabled(flag_idx) ? 1 : 0;
	}

	BorrowedFrame Borrow(PlaneKind plane) { return BorrowedFrame(AcquireLatest(plane), plane); }

	py::object Copy(PlaneKind plane, std::size_t count)
	{
		const auto leases = AcquireRecent(plane, count);
		if (leases.empty()) {
			return py::none();
		}
		return MakeCopiedArray(leases, plane);
	}

	py::tuple GetBufferHandles()
	{
		EnsureOpen();
		return py::make_tuple(
		    config_->PlaneConfigured(PlaneKind::kRgb) ? Copy(PlaneKind::kRgb, buffer_size_) : py::none(),
		    config_->PlaneConfigured(PlaneKind::kDepth) ? Copy(PlaneKind::kDepth, buffer_size_) : py::none(),
		    config_->PlaneConfigured(PlaneKind::kSegmentation) ? Copy(PlaneKind::kSegmentation, buffer_size_) :
		                                                         py::none());
	}

	int RgbFrameCount() { return FrameCount(PlaneKind::kRgb); }
	int DepthFrameCount() { return FrameCount(PlaneKind::kDepth); }
	int SegmentFrameCount() { return FrameCount(PlaneKind::kSegmentation); }

private:
	void EnsureOpen() const
	{
		if (closed_) {
			throw std::runtime_error("Python camera buffer is closed");
		}
		if (!config_.has_value()) {
			throw std::runtime_error("Python camera buffer is closed");
		}
	}

	int FrameCount(PlaneKind plane)
	{
		EnsureOpen();
		if (!config_->PlaneConfigured(plane)) {
			return 0;
		}
		return static_cast<int>(config_->TakeRecent(plane, buffer_size_).size());
	}

	std::optional<FrameLease> AcquireLatest(PlaneKind plane)
	{
		EnsureOpen();
		return config_->TakeLatest(plane);
	}

	std::vector<FrameLease> AcquireRecent(PlaneKind plane, std::size_t count)
	{
		EnsureOpen();
		return config_->TakeRecent(plane, count);
	}

	mujoco_ros::MujocoEnv *env_;
	mujoco_ros::CameraPublicationTransport *transport_;
	std::string camera_name_;
	std::uint8_t buffer_size_;
	std::optional<mujoco_ros::OffscreenCameraConfig> config_;
	bool closed_ = false;
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

	py::class_<BorrowedFrame>(module, "_BorrowedFrame")
	    .def("__enter__", &BorrowedFrame::Enter)
	    .def("__exit__", [](BorrowedFrame &self, py::object, py::object, py::object) { self.Exit(); })
	    .def_property_readonly("capture_id", &BorrowedFrame::CaptureId)
	    .def_property_readonly("frame_generation", &BorrowedFrame::FrameGeneration)
	    .def_property_readonly("size", &BorrowedFrame::Size);

	py::class_<OffscreenCameraBuffer>(module, "_OffscreenCameraBuffer")
	    .def(py::init([](const py::object &env_binding, mujoco_ros::CameraPublicationTransport *transport,
	                     const std::string &camera_name, const std::uint8_t buffer_size) {
		         return new OffscreenCameraBuffer(MujocoEnvPtrFromPythonBinding(env_binding), transport, camera_name,
		                                          buffer_size);
	         }),
	         py::arg("env"), py::arg("transport"), py::arg("camera_name"), py::arg("buffer_size") = 1)
	    .def("getBufferHandles", &OffscreenCameraBuffer::GetBufferHandles, py::return_value_policy::reference_internal)
	    .def("close", &OffscreenCameraBuffer::Close)
	    .def("borrow_latest_rgb", [](OffscreenCameraBuffer &self) { return self.Borrow(PlaneKind::kRgb); })
	    .def("borrow_latest_depth", [](OffscreenCameraBuffer &self) { return self.Borrow(PlaneKind::kDepth); })
	    .def("borrow_latest_segment", [](OffscreenCameraBuffer &self) { return self.Borrow(PlaneKind::kSegmentation); })
	    .def("copy_rgb",
	         [](OffscreenCameraBuffer &self, std::size_t count) { return self.Copy(PlaneKind::kRgb, count); })
	    .def("copy_depth",
	         [](OffscreenCameraBuffer &self, std::size_t count) { return self.Copy(PlaneKind::kDepth, count); })
	    .def("copy_segment",
	         [](OffscreenCameraBuffer &self, std::size_t count) { return self.Copy(PlaneKind::kSegmentation, count); })
	    .def("_set_geom_group", &OffscreenCameraBuffer::SetGeomGroup, py::arg("group"), py::arg("enable") = true)
	    .def("_toggle_flag", &OffscreenCameraBuffer::ToggleFlag, py::arg("flag_idx"))
	    .def("_set_flag", &OffscreenCameraBuffer::SetFlag, py::arg("flag_idx"), py::arg("enable") = true)
	    .def("_get_flag", &OffscreenCameraBuffer::GetFlag, py::arg("flag_idx"))
	    .def_property_readonly("_rgb_frame_count", &OffscreenCameraBuffer::RgbFrameCount)
	    .def_property_readonly("_depth_frame_count", &OffscreenCameraBuffer::DepthFrameCount)
	    .def_property_readonly("_segment_frame_count", &OffscreenCameraBuffer::SegmentFrameCount)
	    .def("__enter__", [](OffscreenCameraBuffer &self) -> OffscreenCameraBuffer & { return self; })
	    .def("__exit__", [](OffscreenCameraBuffer &, py::object, py::object, py::object) {});

	py::class_<mujoco_ros::rendering::OffscreenCamera, std::shared_ptr<mujoco_ros::rendering::OffscreenCamera>>(
	    module, "_OffscreenCamera")
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

	py::class_<mujoco_ros::CameraPublicationTransport>(module, "_CameraPublicationTransport")
#ifdef MJR_BUILD_TESTING
	    .def(
	        "_set_retirement_pending_for_test",
	        [](mujoco_ros::CameraPublicationTransport &self, bool pending) {
		        self.SetRetirementPendingForTest(pending);
	        },
	        py::arg("pending"))
#endif
	    .def("__enter__",
	         [](mujoco_ros::CameraPublicationTransport &self) -> mujoco_ros::CameraPublicationTransport & {
		         self.render_mutex.lock();
		         return self;
	         })
	    .def("__exit__", [](mujoco_ros::CameraPublicationTransport &self, py::object, py::object,
	                        py::object) { self.render_mutex.unlock(); })
	    .def(
	        "camera",
	        [](mujoco_ros::CameraPublicationTransport &self,
	           std::uint8_t cam_id) -> std::shared_ptr<mujoco_ros::rendering::OffscreenCamera> {
		        std::lock_guard<std::mutex> lock(self.lifecycle_mutex);
		        for (auto &cam : self.cams) {
			        if (cam->cam_id_ == cam_id) {
				        return cam;
			        }
		        }
		        throw std::out_of_range("Invalid camera id " + std::to_string(cam_id));
	        },
	        py::arg("id"))
	    .def(
	        "camera",
	        [](mujoco_ros::CameraPublicationTransport &self,
	           const std::string &cam_name) -> std::shared_ptr<mujoco_ros::rendering::OffscreenCamera> {
		        std::lock_guard<std::mutex> lock(self.lifecycle_mutex);
		        for (auto &cam : self.cams) {
			        if (cam->cam_name_ == cam_name) {
				        return cam;
			        }
		        }
		        throw std::out_of_range("Invalid camera name " + cam_name);
	        },
	        py::arg("name"))
	    .def_property_readonly("num_cams", [](const mujoco_ros::CameraPublicationTransport &self) {
		    std::lock_guard<std::mutex> lock(self.lifecycle_mutex);
		    return self.cams.size();
	    });
}

} // namespace mujoco_ros::python
