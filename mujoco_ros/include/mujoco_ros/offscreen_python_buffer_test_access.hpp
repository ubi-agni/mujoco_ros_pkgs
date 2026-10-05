#pragma once

#ifdef MJR_BUILD_TESTING

#include <cstdint>
#include <optional>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/rendering/frame_boundary.hpp>

namespace mujoco_ros::python_buffer_test_access {

// Mirrors _OffscreenCameraBuffer registration (RegisterPythonConsumer with buffer_size).
std::uint64_t RegisterBufferConsumer(CameraPublicationTransport &state, std::uint8_t camera_id,
                                     std::size_t buffer_size);

// Mirrors borrow_latest_* read path (plane validation + AcquirePythonLatest).
std::optional<rendering::FrameLease>
BorrowLatestThroughBuffer(CameraPublicationTransport &state, std::uint64_t registration_id, rendering::PlaneKind plane);

} // namespace mujoco_ros::python_buffer_test_access

#endif
