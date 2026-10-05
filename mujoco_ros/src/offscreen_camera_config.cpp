/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  Copyright (c) 2026, Neura Robotics
 *  All rights reserved.
 *********************************************************************/

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/offscreen_camera.hpp>
#include <mujoco_ros/offscreen_camera_config.hpp>
#include <mujoco_ros/offscreen_transport.hpp>
#include <mujoco_ros/rendering/camera_descriptor.hpp>
#include <mujoco_ros/rendering/frame_capacity.hpp>
#include <mujoco_ros/rendering/render_core.hpp>

#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace mujoco_ros {

namespace {

[[noreturn]] void ThrowCameraUnavailableDuringReload()
{
	throw std::runtime_error("camera is unavailable during reload");
}

} // namespace

OffscreenCameraConfig::OffscreenCameraConfig(MujocoEnv *env, std::string camera_name)
    : env_(env), camera_name_(std::move(camera_name))
{
}

OffscreenCameraConfig::OffscreenCameraConfig(OffscreenCameraConfig &&other) noexcept
    : env_(other.env_)
    , camera_name_(std::move(other.camera_name_))
    , pending_geom_groups_(other.pending_geom_groups_)
    , pending_visual_flags_(other.pending_visual_flags_)
    , pending_enable_frames_depth_(other.pending_enable_frames_depth_)
    , camera_ever_resolved_(other.camera_ever_resolved_)
    , frame_consumer_(other.frame_consumer_)
    , registered_core_(std::move(other.registered_core_))
    , registered_frame_generation_(other.registered_frame_generation_)
    , history_depth_(other.history_depth_)
    , last_capture_id_(other.last_capture_id_)
{
	other.env_ = nullptr;
	other.pending_enable_frames_depth_.reset();
	other.camera_ever_resolved_ = false;
	other.frame_consumer_.reset();
	other.registered_core_.reset();
	other.registered_frame_generation_ = FrameGeneration(0);
	other.history_depth_               = 0;
	other.last_capture_id_             = 0;
}

OffscreenCameraConfig &OffscreenCameraConfig::operator=(OffscreenCameraConfig &&other) noexcept
{
	if (this == &other) {
		return *this;
	}
	UnregisterFrameConsumer();
	env_                         = other.env_;
	camera_name_                 = std::move(other.camera_name_);
	pending_geom_groups_         = other.pending_geom_groups_;
	pending_visual_flags_        = other.pending_visual_flags_;
	pending_enable_frames_depth_ = other.pending_enable_frames_depth_;
	camera_ever_resolved_        = other.camera_ever_resolved_;
	frame_consumer_              = other.frame_consumer_;
	registered_core_             = std::move(other.registered_core_);
	registered_frame_generation_ = other.registered_frame_generation_;
	history_depth_               = other.history_depth_;
	last_capture_id_             = other.last_capture_id_;
	other.env_                   = nullptr;
	other.pending_enable_frames_depth_.reset();
	other.camera_ever_resolved_ = false;
	other.frame_consumer_.reset();
	other.registered_core_.reset();
	other.registered_frame_generation_ = FrameGeneration(0);
	other.history_depth_               = 0;
	other.last_capture_id_             = 0;
	return *this;
}

OffscreenCameraConfig::~OffscreenCameraConfig()
{
	UnregisterFrameConsumer();
}

bool OffscreenCameraConfig::FramesAlreadyRequested() const
{
	return frame_consumer_.has_value() || pending_enable_frames_depth_.has_value();
}

void OffscreenCameraConfig::RegisterFrameConsumerLocked(rendering::OffscreenCamera &camera,
                                                        CameraPublicationTransport &transport,
                                                        const std::size_t history_depth)
{
	if (frame_consumer_.has_value()) {
		throw std::logic_error("frames are already enabled for this camera");
	}
	const auto core = transport.ActiveRenderCore();
	if (!core) {
		throw std::runtime_error("offscreen rendering context has no active RenderCore");
	}
	frame_consumer_              = RegisterFrameConsumer(camera, core, history_depth);
	registered_core_             = core;
	registered_frame_generation_ = transport.ActiveFrameGenerationLocked();
	history_depth_               = history_depth;
	last_capture_id_             = 0;
	if (history_depth == 1) {
		core->RequestOneShot(*frame_consumer_);
	}
}

void OffscreenCameraConfig::ApplyPendingConfigurationLocked(rendering::OffscreenCamera &camera,
                                                            CameraPublicationTransport &transport)
{
	for (int group = 0; group < mjNGROUP; ++group) {
		if (pending_geom_groups_[static_cast<std::size_t>(group)].has_value()) {
			camera.SetGeomGroup(group, *pending_geom_groups_[static_cast<std::size_t>(group)]);
			pending_geom_groups_[static_cast<std::size_t>(group)].reset();
		}
	}
	for (int flag = 0; flag < mjNVISFLAG; ++flag) {
		if (pending_visual_flags_[static_cast<std::size_t>(flag)].has_value()) {
			camera.SetVisualFlag(flag, *pending_visual_flags_[static_cast<std::size_t>(flag)]);
			pending_visual_flags_[static_cast<std::size_t>(flag)].reset();
		}
	}
	if (pending_enable_frames_depth_.has_value()) {
		const auto depth = *pending_enable_frames_depth_;
		pending_enable_frames_depth_.reset();
		RegisterFrameConsumerLocked(camera, transport, depth);
	}
}

rendering::OffscreenCamera &OffscreenCameraConfig::ResolveCameraLocked(CameraPublicationTransport &transport,
                                                                       const bool require_live_camera)
{
	if (transport.retirement_pending) {
		ThrowCameraUnavailableDuringReload();
	}
	if (require_live_camera && env_->reload_in_progress_.load(std::memory_order_acquire)) {
		ThrowCameraUnavailableDuringReload();
	}

	std::size_t matches                  = 0;
	rendering::OffscreenCamera *resolved = nullptr;
	for (const auto &camera : transport.cams) {
		if (camera->cam_name_ != camera_name_) {
			continue;
		}
		++matches;
		resolved = camera.get();
	}

	if (matches > 1) {
		throw std::runtime_error("offscreen camera '" + camera_name_ + "' is ambiguous");
	}
	if (matches == 0) {
		if (!camera_ever_resolved_) {
			throw std::runtime_error("offscreen camera '" + camera_name_ + "' was not found");
		}
		throw std::runtime_error("camera '" + camera_name_ + "' does not exist anymore after reload");
	}

	camera_ever_resolved_ = true;
	ApplyPendingConfigurationLocked(*resolved, transport);
	return *resolved;
}

rendering::ConsumerId OffscreenCameraConfig::RegisterFrameConsumer(rendering::OffscreenCamera &camera,
                                                                   const std::shared_ptr<rendering::RenderCore> &core,
                                                                   const std::size_t history_depth) const
{
	rendering::ValidatePythonHistoryDepth(history_depth);
	const auto consumer_name = "camera/" + camera_name_;
	if (history_depth > 1) {
		return core->RegisterCadencedConsumer(consumer_name, rendering::kPythonAsyncBufferCadence,
		                                      camera.descriptor().id);
	}
	return core->RegisterOneShotConsumer(consumer_name, camera.descriptor().id);
}

void OffscreenCameraConfig::UnregisterFrameConsumer() noexcept
{
	if (!frame_consumer_.has_value()) {
		return;
	}
	if (const auto core = registered_core_.lock()) {
		try {
			core->UnregisterConsumer(*frame_consumer_);
		} catch (...) {
		}
	}
	frame_consumer_.reset();
	registered_core_.reset();
	registered_frame_generation_ = FrameGeneration(0);
	history_depth_               = 0;
}

void OffscreenCameraConfig::SynchronizeFrameConsumerLocked(CameraPublicationTransport &transport,
                                                           rendering::OffscreenCamera &camera)
{
	if (!frame_consumer_.has_value()) {
		throw std::logic_error("EnableFrames must be called before reading frames");
	}

	const auto active_core      = transport.ActiveRenderCore();
	const auto frame_generation = transport.ActiveFrameGenerationLocked();
	if (!active_core) {
		throw std::runtime_error("offscreen rendering context has no active RenderCore");
	}

	const auto registered_core = registered_core_.lock();
	if (registered_core != active_core || registered_frame_generation_ != frame_generation) {
		const rendering::ConsumerId old_consumer            = *frame_consumer_;
		const std::weak_ptr<rendering::RenderCore> old_core = registered_core_;

		const rendering::ConsumerId new_consumer = RegisterFrameConsumer(camera, active_core, history_depth_);

		frame_consumer_              = new_consumer;
		registered_core_             = active_core;
		registered_frame_generation_ = frame_generation;
		last_capture_id_             = 0;
		if (history_depth_ == 1) {
			active_core->RequestOneShot(new_consumer);
		}

		if (const auto old_core_locked = old_core.lock()) {
			old_core_locked->UnregisterConsumer(old_consumer);
		}
	}
}

void OffscreenCameraConfig::EnableFrames(const std::size_t history_depth)
{
	rendering::ValidatePythonHistoryDepth(history_depth);
	if (FramesAlreadyRequested()) {
		throw std::logic_error("frames are already enabled for this camera");
	}

	auto &transport = env_->camera_publication_transport_;
	std::lock_guard<std::mutex> lock(transport.lifecycle_mutex);
	if (transport.retirement_pending) {
		ThrowCameraUnavailableDuringReload();
	}
	if (env_->reload_in_progress_.load(std::memory_order_acquire) && !transport.retirement_pending) {
		pending_enable_frames_depth_ = history_depth;
		return;
	}

	auto &camera = ResolveCameraLocked(transport, true);
	RegisterFrameConsumerLocked(camera, transport, history_depth);
}

std::optional<rendering::FrameLease> OffscreenCameraConfig::TakeLatest(const rendering::PlaneKind plane)
{
	if (!FramesAlreadyRequested()) {
		throw std::logic_error("EnableFrames must be called before TakeLatest");
	}

	auto &transport = env_->camera_publication_transport_;
	std::lock_guard<std::mutex> lock(transport.lifecycle_mutex);
	auto &camera = ResolveCameraLocked(transport, true);
	SynchronizeFrameConsumerLocked(transport, camera);

	const auto active_core = transport.ActiveRenderCore();
	if (!active_core) {
		throw std::runtime_error("offscreen rendering context has no active RenderCore");
	}

	if (!rendering::HasPlane(camera.descriptor().planes, plane)) {
		throw std::runtime_error("requested render plane is not configured for this camera");
	}

	if (history_depth_ <= 1) {
		active_core->RequestOneShot(*frame_consumer_);
	}

	const auto camera_id = camera.descriptor().id;
	auto lease           = active_core->AcquireLatest(camera_id, plane);
	if (!lease) {
		return std::nullopt;
	}
	const auto capture_id = lease->capture_id();
	if (capture_id == 0 || capture_id == last_capture_id_) {
		return std::nullopt;
	}
	last_capture_id_ = capture_id;
	return lease;
}

std::vector<rendering::FrameLease> OffscreenCameraConfig::TakeRecent(const rendering::PlaneKind plane,
                                                                     const std::size_t count)
{
	if (!FramesAlreadyRequested()) {
		throw std::logic_error("EnableFrames must be called before TakeRecent");
	}

	auto &transport = env_->camera_publication_transport_;
	std::lock_guard<std::mutex> lock(transport.lifecycle_mutex);
	auto &camera = ResolveCameraLocked(transport, true);
	SynchronizeFrameConsumerLocked(transport, camera);

	const auto active_core = transport.ActiveRenderCore();
	if (!active_core) {
		throw std::runtime_error("offscreen rendering context has no active RenderCore");
	}

	if (!rendering::HasPlane(camera.descriptor().planes, plane)) {
		throw std::runtime_error("requested render plane is not configured for this camera");
	}

	return active_core->AcquireRecent(camera.descriptor().id, plane, count);
}

bool OffscreenCameraConfig::PlaneConfigured(const rendering::PlaneKind plane) const
{
	auto &transport = env_->camera_publication_transport_;
	std::lock_guard<std::mutex> lock(transport.lifecycle_mutex);
	auto &camera = const_cast<OffscreenCameraConfig *>(this)->ResolveCameraLocked(transport, true);
	return rendering::HasPlane(camera.descriptor().planes, plane);
}

void OffscreenCameraConfig::SetGeomGroup(const int group, const bool enable)
{
	if (group < 0 || group >= mjNGROUP) {
		throw std::out_of_range("geom group index out of range");
	}

	auto &transport = env_->camera_publication_transport_;
	std::lock_guard<std::mutex> lock(transport.lifecycle_mutex);
	if (transport.retirement_pending) {
		ThrowCameraUnavailableDuringReload();
	}
	if (env_->reload_in_progress_.load(std::memory_order_acquire) && !transport.retirement_pending) {
		pending_geom_groups_[static_cast<std::size_t>(group)] = enable;
		return;
	}

	auto &camera = ResolveCameraLocked(transport, true);
	camera.SetGeomGroup(group, enable);
}

void OffscreenCameraConfig::SetVisualFlag(const int flag, const bool enable)
{
	if (flag < 0 || flag >= mjNVISFLAG) {
		throw std::out_of_range("visual flag index out of range");
	}

	auto &transport = env_->camera_publication_transport_;
	std::lock_guard<std::mutex> lock(transport.lifecycle_mutex);
	if (transport.retirement_pending) {
		ThrowCameraUnavailableDuringReload();
	}
	if (env_->reload_in_progress_.load(std::memory_order_acquire) && !transport.retirement_pending) {
		pending_visual_flags_[static_cast<std::size_t>(flag)] = enable;
		return;
	}

	auto &camera = ResolveCameraLocked(transport, true);
	camera.SetVisualFlag(flag, enable);
}

bool OffscreenCameraConfig::GeomGroupEnabled(const int group) const
{
	auto &transport = env_->camera_publication_transport_;
	std::lock_guard<std::mutex> lock(transport.lifecycle_mutex);
	auto &camera = const_cast<OffscreenCameraConfig *>(this)->ResolveCameraLocked(transport, true);
	return camera.GetGeomGroup(group) != 0;
}

bool OffscreenCameraConfig::VisualFlagEnabled(const int flag) const
{
	auto &transport = env_->camera_publication_transport_;
	std::lock_guard<std::mutex> lock(transport.lifecycle_mutex);
	auto &camera = const_cast<OffscreenCameraConfig *>(this)->ResolveCameraLocked(transport, true);
	return camera.GetVisualFlag(flag) != 0;
}

OffscreenCameraConfig MujocoEnv::OpenOffscreenCamera(const std::string &camera_name)
{
	if (camera_name.empty()) {
		throw std::runtime_error("offscreen camera '' was not found");
	}
	return OffscreenCameraConfig(this, camera_name);
}

} // namespace mujoco_ros
