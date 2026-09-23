#pragma once

#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include <mujoco_ros/rendering/camera_descriptor.hpp>
#include <mujoco_ros/rendering/frame_boundary.hpp>
#include <mujoco_ros/rendering/render_snapshot.hpp>

namespace mujoco_ros::rendering {

enum class RenderStatusCode
{
	kOk,
	kNotInitialized,
	kStopped,
	kStaleModelGeneration,
	kBackendFailure,
	kFrameUnavailable,
	kFrameSlotsExhausted,
	kGenerationCapacityExhausted,
	kBackendUnavailable,
	kTerminalError,
};

struct RenderStatus
{
	RenderStatusCode code = RenderStatusCode::kOk;
	std::string message;

	bool ok() const { return code == RenderStatusCode::kOk; }
	static RenderStatus Ok() { return {}; }
	static RenderStatus Failure(RenderStatusCode code, std::string message)
	{
		return RenderStatus{ code, std::move(message) };
	}
};

inline bool IsContextIntegrityFailure(RenderStatusCode code)
{
	switch (code) {
		case RenderStatusCode::kBackendFailure:
		case RenderStatusCode::kBackendUnavailable:
		case RenderStatusCode::kTerminalError:
			return true;
		default:
			return false;
	}
}

inline bool IsContextIntegrityFailure(const RenderStatus &status)
{
	return IsContextIntegrityFailure(status.code);
}

class ContextIntegrityError : public std::runtime_error
{
public:
	using std::runtime_error::runtime_error;
};

inline RenderStatus ClassifyCaptureException(const std::exception &error)
{
	if (dynamic_cast<const ContextIntegrityError *>(&error) != nullptr) {
		return RenderStatus::Failure(RenderStatusCode::kBackendFailure, error.what());
	}
	return RenderStatus::Failure(RenderStatusCode::kFrameUnavailable, error.what());
}

struct RenderConfiguration
{
	FrameGeneration generation;
	FrameLayout frame_layout;
	std::string backend;
};

class IRenderBackend
{
public:
	virtual ~IRenderBackend()                                                                                 = default;
	virtual RenderStatus Initialize(const mjModel &, const RenderConfiguration &)                             = 0;
	virtual RenderStatus Resize(const RenderConfiguration &)                                                  = 0;
	virtual RenderStatus Render(const RenderSnapshot &, const CameraDescriptor &, PlaneMask, FrameBoundary &) = 0;
	virtual void ShutdownOnRenderThread()                                                                     = 0;
};

std::unique_ptr<IRenderBackend> CreateDisabledRenderBackend();
std::unique_ptr<IRenderBackend> CreateRenderBackend();
const char *CompiledRenderBackendName() noexcept;

} // namespace mujoco_ros::rendering
