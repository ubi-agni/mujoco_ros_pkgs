/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  Copyright (c) 2026, Neura Robotics
 *  All rights reserved.
 *********************************************************************/

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <type_traits>
#include <utility>

#include <mujoco/mujoco.h>

#include <mujoco_ros/generation.hpp>
#include <mujoco_ros/rendering/frame_boundary.hpp>
#include <mujoco_ros/rendering/render_demand.hpp>

namespace mujoco_ros {

class MujocoEnv;
struct CameraPublicationTransport;

namespace rendering {
class OffscreenCamera;
class RenderCore;
} // namespace rendering

class OffscreenCameraConfig
{
public:
	OffscreenCameraConfig(const OffscreenCameraConfig &)            = delete;
	OffscreenCameraConfig &operator=(const OffscreenCameraConfig &) = delete;
	OffscreenCameraConfig(OffscreenCameraConfig &&other) noexcept;
	OffscreenCameraConfig &operator=(OffscreenCameraConfig &&other) noexcept;
	~OffscreenCameraConfig();

	void SetGeomGroup(int group, bool enable);
	void SetVisualFlag(int flag, bool enable);
	bool GeomGroupEnabled(int group) const;
	bool VisualFlagEnabled(int flag) const;
	const std::string &camera_name() const { return camera_name_; }

	void EnableFrames(std::size_t history_depth);
	std::optional<rendering::FrameLease> TakeLatest(rendering::PlaneKind plane);
	std::vector<rendering::FrameLease> TakeRecent(rendering::PlaneKind plane, std::size_t count);
	bool PlaneConfigured(rendering::PlaneKind plane) const;

private:
	friend class MujocoEnv;
	OffscreenCameraConfig(MujocoEnv *env, std::string camera_name);

	rendering::OffscreenCamera &ResolveCameraLocked(CameraPublicationTransport &transport, bool require_live_camera);
	void ApplyPendingConfigurationLocked(rendering::OffscreenCamera &camera, CameraPublicationTransport &transport);
	rendering::ConsumerId RegisterFrameConsumer(rendering::OffscreenCamera &camera,
	                                            const std::shared_ptr<rendering::RenderCore> &core,
	                                            std::size_t history_depth) const;
	void RegisterFrameConsumerLocked(rendering::OffscreenCamera &camera, CameraPublicationTransport &transport,
	                                 std::size_t history_depth);
	void UnregisterFrameConsumer() noexcept;
	bool FramesAlreadyRequested() const;
	void SynchronizeFrameConsumerLocked(CameraPublicationTransport &transport, rendering::OffscreenCamera &camera);

	MujocoEnv *env_ = nullptr;
	std::string camera_name_;
	std::array<std::optional<bool>, mjNGROUP> pending_geom_groups_{};
	std::array<std::optional<bool>, mjNVISFLAG> pending_visual_flags_{};
	std::optional<std::size_t> pending_enable_frames_depth_;
	bool camera_ever_resolved_ = false;
	std::optional<rendering::ConsumerId> frame_consumer_;
	std::weak_ptr<rendering::RenderCore> registered_core_;
	FrameGeneration registered_frame_generation_{ 0 };
	std::size_t history_depth_     = 0;
	std::uint64_t last_capture_id_ = 0;
};

} // namespace mujoco_ros
