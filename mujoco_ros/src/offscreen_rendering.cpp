/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 * Copyright (c) 2022-2026, Bielefeld University
 * Copyright (c) 2026, Neura Robotics
 *********************************************************************/

#include <algorithm>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include <mujoco_ros/logging.hpp>
#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/offscreen_camera.hpp>
#include <mujoco_ros/util.hpp>
#include <mujoco_ros/rendering/frame_capacity.hpp>

namespace mujoco_ros {

CameraPublicationTransport::~CameraPublicationTransport() = default;

std::shared_ptr<rendering::RenderCore> CameraPublicationTransport::ActiveRenderCore() const
{
	return render_owner_ != nullptr ? render_owner_->ActiveRenderCore() : nullptr;
}

FrameGeneration CameraPublicationTransport::ActiveFrameGeneration() const
{
	std::lock_guard<std::mutex> lock(lifecycle_mutex);
	return ActiveFrameGenerationLocked();
}

FrameGeneration CameraPublicationTransport::ActiveFrameGenerationLocked() const
{
	if (render_owner_ == nullptr || cams.empty()) {
		return FrameGeneration(0);
	}
	return render_owner_->PublishedFrameGeneration();
}

#ifdef MJR_BUILD_TESTING
void CameraPublicationTransport::SetRetirementPendingForTest(const bool pending)
{
	std::lock_guard<std::mutex> lock(lifecycle_mutex);
	retirement_pending = pending;
}
#endif

std::uint64_t CameraPublicationTransport::RegisterPythonConsumer(const std::uint8_t camera_id,
                                                                 const std::size_t history_depth)
{
	rendering::ValidatePythonHistoryDepth(history_depth);
	std::lock_guard<std::mutex> lock(lifecycle_mutex);
	if (retirement_pending) {
		throw std::runtime_error("Python rendering registration rejected while camera retirement is pending");
	}
	ValidatePythonConsumerRequest(history_depth, ActiveRenderCore() != nullptr);
	for (const auto &camera : cams) {
		if (camera->cam_id_ != camera_id) {
			continue;
		}
		const auto registration_id = next_python_registration_id++;
		const auto core            = ActiveRenderCore();
		const auto consumer        = camera->RegisterPythonConsumer(core, history_depth);
		python_consumers.emplace(registration_id,
		                         PythonDemandRegistration{ camera_id, history_depth, core, camera, consumer });
		return registration_id;
	}
	throw std::out_of_range("Invalid camera id " + std::to_string(camera_id));
}

void CameraPublicationTransport::UnregisterPythonConsumer(const std::uint64_t registration_id)
{
	std::lock_guard<std::mutex> lock(lifecycle_mutex);
	const auto it = python_consumers.find(registration_id);
	if (it == python_consumers.end()) {
		return;
	}
	if (it->second.camera) {
		it->second.camera->UnregisterPythonConsumer(it->second.consumer);
	}
	python_consumers.erase(it);
}

void CameraPublicationTransport::RebindPythonConsumersLocked()
{
	if (retirement_pending) {
		throw std::logic_error("Python rendering consumers cannot rebind during camera retirement");
	}
	for (auto &[registration_id, registration] : python_consumers) {
		(void)registration_id;
		registration.core.reset();
		registration.camera.reset();
		for (const auto &camera : cams) {
			if (camera->cam_id_ != registration.camera_id) {
				continue;
			}
			if (!ActiveRenderCore()) {
				break;
			}
			registration.core     = ActiveRenderCore();
			registration.camera   = camera;
			registration.consumer = camera->RegisterPythonConsumer(ActiveRenderCore(), registration.history_depth);
			break;
		}
	}
}

void CameraPublicationTransport::UnregisterPythonConsumersLocked()
{
	std::exception_ptr first_error;
	for (auto &[registration_id, registration] : python_consumers) {
		(void)registration_id;
		if (!registration.camera) {
			registration.core.reset();
			continue;
		}
		try {
			registration.camera->UnregisterPythonConsumer(registration.consumer);
			registration.core.reset();
			registration.camera.reset();
		} catch (...) {
			if (!first_error) {
				first_error = std::current_exception();
			}
		}
	}
	if (first_error) {
		std::rethrow_exception(first_error);
	}
}

std::optional<rendering::FrameLease>
CameraPublicationTransport::AcquirePythonLatest(const std::uint64_t registration_id, const rendering::PlaneKind plane)
{
	std::lock_guard<std::mutex> lock(lifecycle_mutex);
	const auto registration_it = python_consumers.find(registration_id);
	if (registration_it == python_consumers.end()) {
		throw std::runtime_error("unknown Python rendering registration");
	}
	const auto &registration = registration_it->second;
	if (retirement_pending) {
		throw std::runtime_error("Python frame acquisition rejected while camera retirement is pending");
	}
	if (!ActiveRenderCore()) {
		throw std::runtime_error("Python camera handle is stale after RenderCore reload");
	}
	const auto render_core = ActiveRenderCore();
	if (render_core != registration.core) {
		throw std::runtime_error("Python camera handle is stale after RenderCore reload");
	}
	const auto frame_generation = ActiveFrameGenerationLocked();
	if (render_core->frames().generation() != frame_generation) {
		throw std::runtime_error("Python camera handle is stale after frame-generation change");
	}
	for (const auto &camera : cams) {
		if (camera->cam_id_ != registration.camera_id) {
			continue;
		}
		if (registration.history_depth <= 1) {
			render_core->RequestOneShot(registration.consumer);
		}
		auto lease = render_core->AcquireLatest(camera->descriptor().id, plane);
		if (lease && lease->generation() != frame_generation) {
			throw std::runtime_error("Python acquisition crossed a frame-generation boundary");
		}
		return lease;
	}
	throw std::runtime_error("Python camera handle is stale after camera-layout change");
}

std::vector<rendering::FrameLease> CameraPublicationTransport::AcquirePythonRecent(const std::uint64_t registration_id,
                                                                                   const rendering::PlaneKind plane,
                                                                                   const std::size_t count)
{
	std::lock_guard<std::mutex> lock(lifecycle_mutex);
	const auto registration_it = python_consumers.find(registration_id);
	if (registration_it == python_consumers.end()) {
		throw std::runtime_error("unknown Python rendering registration");
	}
	const auto &registration = registration_it->second;
	if (retirement_pending) {
		throw std::runtime_error("Python frame acquisition rejected while camera retirement is pending");
	}
	if (!ActiveRenderCore()) {
		throw std::runtime_error("Python camera handle is stale after RenderCore reload");
	}
	const auto render_core = ActiveRenderCore();
	if (render_core != registration.core) {
		throw std::runtime_error("Python camera handle is stale after RenderCore reload");
	}
	const auto frame_generation = ActiveFrameGenerationLocked();
	if (render_core->frames().generation() != frame_generation) {
		throw std::runtime_error("Python camera handle is stale after frame-generation change");
	}
	for (const auto &camera : cams) {
		if (camera->cam_id_ != registration.camera_id) {
			continue;
		}
		if (registration.history_depth <= 1) {
			render_core->RequestOneShot(registration.consumer);
		}
		auto leases = render_core->AcquireRecent(camera->descriptor().id, plane, count);
		if (std::any_of(leases.begin(), leases.end(),
		                [frame_generation](const auto &lease) { return lease.generation() != frame_generation; })) {
			throw std::runtime_error("Python acquisition crossed a frame-generation boundary");
		}
		return leases;
	}
	throw std::runtime_error("Python camera handle is stale after camera-layout change");
}

namespace {

rendering::CameraTransportBootstrap BuildCameraTransportBootstrap(const mjModel &model, const mjData &data,
                                                                  const std::uint8_t cam_id)
{
	if (cam_id >= static_cast<std::uint8_t>(model.ncam)) {
		throw std::invalid_argument("camera id out of range for transport bootstrap");
	}
	const int body_id     = model.cam_bodyid[cam_id];
	const char *body_name = mj_id2name(const_cast<mjModel *>(&model), mjOBJ_BODY, body_id);
	if (body_name == nullptr) {
		throw std::invalid_argument("camera transport bootstrap requires named parent body for camera id " +
		                            std::to_string(cam_id));
	}
	rendering::CameraTransportBootstrap bootstrap;
	bootstrap.static_transform_stamp = util::toRosTime(data.time);
	bootstrap.parent_frame           = std::string(body_name);
	mju_copy(bootstrap.body_to_cam_position.data(), model.cam_pos + cam_id * 3, 3);
	mju_copy(bootstrap.body_to_cam_orientation.data(), model.cam_quat + cam_id * 4, 4);
	bootstrap.vertical_field_of_view_deg = model.cam_fovy[cam_id];
	return bootstrap;
}

} // namespace

std::vector<rendering::OffscreenCameraPtr> MujocoEnv::InitializeRenderResources()
{
	std::vector<rendering::OffscreenCameraPtr> cameras;
	if (!settings_.render_offscreen || !model_ || model_->ncam == 0) {
		return cameras;
	}

	int res_h          = 0;
	int res_w          = 0;
	int unnamed_cam_id = 0;
	for (int cam_id = 0; cam_id < model_->ncam; ++cam_id) {
		const char *name = mj_id2name(model_.get(), mjOBJ_CAMERA, cam_id);
		const std::string cam_name =
		    name != nullptr ? std::string(name) : "unnamed_cam_" + std::to_string(unnamed_cam_id++);
		rendering::StreamType stream_type;
		bool use_segid;
		std::string base_topic, rgb, depth, segment;
		float pub_freq;
		GetCameraConfiguration(cam_name, stream_type, pub_freq, use_segid, res_w, res_h, base_topic, rgb, depth, segment);
		const auto bootstrap = BuildCameraTransportBootstrap(*model_, *data_, static_cast<std::uint8_t>(cam_id));
		auto camera =
		    std::make_shared<rendering::OffscreenCamera>(static_cast<std::uint8_t>(cam_id), base_topic, cam_name, res_w,
		                                                 res_h, stream_type, use_segid, pub_freq, bootstrap, this);
#if MJR_ROS_VERSION == ROS_1
		camera->InitializeTransport(nh_, bootstrap.vertical_field_of_view_deg, rgb, depth, segment);
#else
		camera->InitializeTransport(bootstrap.vertical_field_of_view_deg, rgb, depth, segment);
#endif
		cameras.emplace_back(std::move(camera));
	}
	return cameras;
}

#ifdef MJR_BUILD_TESTING
namespace python_buffer_test_access {

std::uint64_t RegisterBufferConsumer(CameraPublicationTransport &state, const std::uint8_t camera_id,
                                     const std::size_t buffer_size)
{
	rendering::ValidatePythonHistoryDepth(buffer_size);
	return state.RegisterPythonConsumer(camera_id, buffer_size);
}

std::optional<rendering::FrameLease> BorrowLatestThroughBuffer(CameraPublicationTransport &state,
                                                               const std::uint64_t registration_id,
                                                               const rendering::PlaneKind plane)
{
	return state.AcquirePythonLatest(registration_id, plane);
}

} // namespace python_buffer_test_access
#endif

} // namespace mujoco_ros
