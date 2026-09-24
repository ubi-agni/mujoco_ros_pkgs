/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 * Copyright (c) 2022-2026, Bielefeld University
 *********************************************************************/

#pragma once

#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mujoco_ros/generation.hpp>
#include <mujoco_ros/rendering/frame_boundary.hpp>
#include <mujoco_ros/rendering/frame_capacity.hpp>
#include <mujoco_ros/rendering/render_core.hpp>

namespace mujoco_ros::rendering {

class OffscreenCamera;
using OffscreenCameraPtr = std::shared_ptr<OffscreenCamera>;

} // namespace mujoco_ros::rendering

namespace mujoco_ros {

class MujocoEnv;

// ROS transport state for configured camera publication adapters. RenderCore owns graphics state.
struct CameraPublicationTransport
{
	CameraPublicationTransport()                                              = default;
	CameraPublicationTransport(const CameraPublicationTransport &)            = delete;
	CameraPublicationTransport &operator=(const CameraPublicationTransport &) = delete;
	~CameraPublicationTransport();

	void BindRenderOwner(MujocoEnv *owner) noexcept { render_owner_ = owner; }
	std::shared_ptr<rendering::RenderCore> ActiveRenderCore() const;
	FrameGeneration ActiveFrameGeneration() const;

	// This preserves the production validation order without loading the ROS runtime.
	static void ValidatePythonConsumerRequest(const std::size_t history_depth, const bool has_active_render_core)
	{
		rendering::ValidatePythonHistoryDepth(history_depth);
		if (!has_active_render_core) {
			throw std::runtime_error("Python rendering context has no active RenderCore");
		}
	}

	// Transport-side mutex for body-mass service paths that touch render resources.
	std::mutex render_mutex;
	mutable std::mutex render_status_mutex;
	rendering::FrameStatus last_render_status = rendering::FrameStatus::Ok();

	void RecordRenderStatus(rendering::FrameStatus status)
	{
		std::lock_guard<std::mutex> lock(render_status_mutex);
		last_render_status = std::move(status);
	}

	rendering::FrameStatus LastRenderStatus() const
	{
		std::lock_guard<std::mutex> lock(render_status_mutex);
		return last_render_status;
	}

	// Protects the active camera vector and Python consumer registrations.
	mutable std::mutex lifecycle_mutex;
	std::vector<rendering::OffscreenCameraPtr> cams;
	// Set before old-camera deactivation and cleared only after all old
	// consumer associations and camera/Core links are retired successfully.
	bool retirement_pending = false;

	std::uint64_t RegisterPythonConsumer(std::uint8_t camera_id, std::size_t history_depth = 1);
	void UnregisterPythonConsumer(std::uint64_t registration_id);
	void RebindPythonConsumersLocked();
	void UnregisterPythonConsumersLocked();
	std::optional<rendering::FrameLease> AcquirePythonLatest(std::uint64_t registration_id, rendering::PlaneKind plane);
	std::vector<rendering::FrameLease> AcquirePythonRecent(std::uint64_t registration_id, rendering::PlaneKind plane,
	                                                       std::size_t count);
#ifdef MJR_BUILD_TESTING
	void SetRetirementPendingForTest(bool pending);
#endif

	struct PythonDemandRegistration
	{
		std::uint8_t camera_id    = 0;
		std::size_t history_depth = 1;
		std::shared_ptr<rendering::RenderCore> core;
		rendering::OffscreenCameraPtr camera;
		rendering::ConsumerId consumer;
	};
	std::uint64_t next_python_registration_id = 1;
	std::unordered_map<std::uint64_t, PythonDemandRegistration> python_consumers;
	// Caller must already hold lifecycle_mutex.
	FrameGeneration ActiveFrameGenerationLocked() const;

private:
	MujocoEnv *render_owner_ = nullptr;
};

} // namespace mujoco_ros
