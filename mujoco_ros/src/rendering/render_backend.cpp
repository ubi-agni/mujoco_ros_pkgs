#include <mujoco_ros/rendering/render_backend_interface.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#if defined(MJR_RENDER_BACKEND_EGL)
#include <EGL/egl.h>
#elif defined(MJR_RENDER_BACKEND_OSMESA)
#include <GL/osmesa.h>
#endif

namespace mujoco_ros::rendering {
namespace {

void FlipRows(std::vector<std::byte> &bytes, std::size_t row_bytes, int height)
{
	std::vector<std::byte> row(row_bytes);
	for (int y = 0; y < height / 2; ++y) {
		auto *top    = bytes.data() + static_cast<std::size_t>(y) * row_bytes;
		auto *bottom = bytes.data() + static_cast<std::size_t>(height - y - 1) * row_bytes;
		std::memcpy(row.data(), top, row_bytes);
		std::memcpy(top, bottom, row_bytes);
		std::memcpy(bottom, row.data(), row_bytes);
	}
}

void ConvertDepthBufferToMeters(std::vector<std::byte> &bytes, float znear, float zfar)
{
	if (!(znear > 0.0F) || !(zfar > znear)) {
		throw std::invalid_argument("MuJoCo depth range is invalid");
	}
	const auto count = bytes.size() / sizeof(float);
	auto *depth      = reinterpret_cast<float *>(bytes.data());
	for (std::size_t index = 0; index < count; ++index) {
		const float buffer_value = std::clamp(depth[index], 0.0F, 1.0F);
		const float denominator  = zfar - buffer_value * (zfar - znear);
		depth[index] = denominator > std::numeric_limits<float>::epsilon() ? znear * zfar / denominator : zfar;
	}
}

RenderStatus FromFrameStatus(const FrameStatus &status)
{
	const auto code =
	    status.code == FrameStatusCode::kStaleGeneration             ? RenderStatusCode::kStaleModelGeneration :
	    status.code == FrameStatusCode::kFrameSlotsExhausted         ? RenderStatusCode::kFrameSlotsExhausted :
	    status.code == FrameStatusCode::kGenerationCapacityExhausted ? RenderStatusCode::kGenerationCapacityExhausted :
	    status.code == FrameStatusCode::kBackendUnavailable          ? RenderStatusCode::kBackendUnavailable :
	                                                                   RenderStatusCode::kFrameUnavailable;
	return RenderStatus::Failure(code, status.message);
}

class MuJoCoRenderBackend final : public IRenderBackend
{
public:
	~MuJoCoRenderBackend() override { ShutdownOnRenderThread(); }

	RenderStatus Initialize(const mjModel &model, const RenderConfiguration &configuration) override
	{
		if (initialized_) {
			return RenderStatus::Failure(RenderStatusCode::kBackendFailure, "backend initialized twice");
		}
		if (!InitializeContext(configuration.frame_layout.width, configuration.frame_layout.height)) {
			ShutdownOnRenderThread();
			return RenderStatus::Failure(RenderStatusCode::kBackendFailure, "graphics context initialization failed");
		}
		mjr_defaultContext(&context_);
		mjv_defaultScene(&scene_);
		mjv_defaultCamera(&camera_);
		mjv_defaultOption(&option_);
		mjr_makeContext(&model, &context_, 50);
		context_initialized_ = true;
		mjv_makeScene(&model, &scene_, 20000);
		scene_initialized_ = true;
		mjr_setBuffer(mjFB_OFFSCREEN, &context_);
		initialized_ = true;
		return RenderStatus::Ok();
	}

	RenderStatus Resize(const RenderConfiguration &configuration) override
	{
		if (!initialized_) {
			return RenderStatus::Failure(RenderStatusCode::kNotInitialized, "backend is not initialized");
		}
#if defined(MJR_RENDER_BACKEND_OSMESA)
		osmesa_buffer_.resize(static_cast<std::size_t>(configuration.frame_layout.width) *
		                      static_cast<std::size_t>(configuration.frame_layout.height) * 4U);
		if (!OSMesaMakeCurrent(osmesa_context_, osmesa_buffer_.data(), GL_UNSIGNED_BYTE, configuration.frame_layout.width,
		                       configuration.frame_layout.height)) {
			return RenderStatus::Failure(RenderStatusCode::kBackendFailure, "OSMesa resize failed");
		}
#endif
		mjr_setBuffer(mjFB_OFFSCREEN, &context_);
		return RenderStatus::Ok();
	}

	RenderStatus Render(const RenderSnapshot &snapshot, const CameraDescriptor &camera, PlaneMask planes,
	                    FrameBoundary &boundary) override
	{
		if (!initialized_) {
			return RenderStatus::Failure(RenderStatusCode::kNotInitialized, "backend is not initialized");
		}
		if (!snapshot.valid()) {
			return RenderStatus::Failure(RenderStatusCode::kFrameUnavailable, "render snapshot is incomplete");
		}
		camera_.type       = mjCAMERA_FIXED;
		camera_.fixedcamid = static_cast<int>(camera.id) - 1;
		option_            = camera.visual_options;
		mjv_updateScene(snapshot.model.get(), snapshot.data.get(), &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
		const auto available_geometry   = static_cast<std::size_t>(scene_.maxgeom - scene_.ngeom);
		const auto *plugin_geometry     = snapshot.plugin_geometry.get();
		const auto plugin_geometry_size = plugin_geometry != nullptr ? plugin_geometry->size() : 0U;
		if (plugin_geometry_size > available_geometry) {
			return RenderStatus::Failure(
			    RenderStatusCode::kFrameUnavailable,
			    "plugin geometry capacity exhausted (model_geometry=" + std::to_string(scene_.ngeom) +
			        ", plugin_geometry=" + std::to_string(plugin_geometry_size) +
			        ", max_geometry=" + std::to_string(scene_.maxgeom) + ")");
		}
		if (plugin_geometry_size != 0U) {
			std::memcpy(scene_.geoms + scene_.ngeom, plugin_geometry->data(), plugin_geometry_size * sizeof(mjvGeom));
			scene_.ngeom += static_cast<int>(plugin_geometry_size);
		}
		const mjrRect viewport{ 0, 0, camera.width, camera.height };
		mjr_setBuffer(mjFB_OFFSCREEN, &context_);

		RenderStatus last_failure = RenderStatus::Ok();
		if (HasPlane(planes, PlaneKind::kRgb)) {
			if (auto result = ReadColor(snapshot, camera, viewport, boundary, false);
			    !result.ok() && IsContextIntegrityFailure(result)) {
				return result;
			} else if (!result.ok()) {
				last_failure = result;
			}
		}
		if (HasPlane(planes, PlaneKind::kSegmentation)) {
			if (auto result = ReadColor(snapshot, camera, viewport, boundary, true);
			    !result.ok() && IsContextIntegrityFailure(result)) {
				return result;
			} else if (!result.ok()) {
				last_failure = result;
			}
		}
		if (HasPlane(planes, PlaneKind::kDepth)) {
			if (auto result = ReadDepth(snapshot, viewport, boundary, camera);
			    !result.ok() && IsContextIntegrityFailure(result)) {
				return result;
			} else if (!result.ok()) {
				last_failure = result;
			}
		}
		return last_failure.ok() ? RenderStatus::Ok() : last_failure;
	}

	void ShutdownOnRenderThread() override
	{
		if (scene_initialized_) {
			mjv_freeScene(&scene_);
			scene_initialized_ = false;
		}
		if (context_initialized_) {
			mjr_freeContext(&context_);
			context_initialized_ = false;
		}
#if defined(MJR_RENDER_BACKEND_EGL)
		if (egl_display_ != EGL_NO_DISPLAY) {
			eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
			if (egl_surface_ != EGL_NO_SURFACE)
				eglDestroySurface(egl_display_, egl_surface_);
			if (egl_context_ != EGL_NO_CONTEXT)
				eglDestroyContext(egl_display_, egl_context_);
			eglTerminate(egl_display_);
			egl_display_ = EGL_NO_DISPLAY;
		}
#elif defined(MJR_RENDER_BACKEND_OSMESA)
		if (osmesa_context_ != nullptr) {
			OSMesaDestroyContext(osmesa_context_);
			osmesa_context_ = nullptr;
		}
		osmesa_buffer_.clear();
#endif
		initialized_ = false;
	}

private:
	bool InitializeContext(int width, int height)
	{
#if defined(MJR_RENDER_BACKEND_EGL)
		egl_display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
		if (egl_display_ == EGL_NO_DISPLAY)
			return false;
		EGLint major = 0, minor = 0;
		if (!eglInitialize(egl_display_, &major, &minor))
			return false;
		if (!eglBindAPI(EGL_OPENGL_API))
			return false;
		const EGLint attributes[] = { EGL_SURFACE_TYPE,
			                           EGL_PBUFFER_BIT,
			                           EGL_RENDERABLE_TYPE,
			                           EGL_OPENGL_BIT,
			                           EGL_RED_SIZE,
			                           8,
			                           EGL_GREEN_SIZE,
			                           8,
			                           EGL_BLUE_SIZE,
			                           8,
			                           EGL_DEPTH_SIZE,
			                           24,
			                           EGL_NONE };
		EGLConfig config          = nullptr;
		EGLint count              = 0;
		if (!eglChooseConfig(egl_display_, attributes, &config, 1, &count) || count != 1)
			return false;
		const EGLint surface_attributes[] = { EGL_WIDTH, width, EGL_HEIGHT, height, EGL_NONE };
		egl_surface_                      = eglCreatePbufferSurface(egl_display_, config, surface_attributes);
		egl_context_                      = eglCreateContext(egl_display_, config, EGL_NO_CONTEXT, nullptr);
		return egl_surface_ != EGL_NO_SURFACE && egl_context_ != EGL_NO_CONTEXT &&
		       eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_);
#elif defined(MJR_RENDER_BACKEND_OSMESA)
		osmesa_context_ = OSMesaCreateContextExt(OSMESA_RGBA, 24, 8, 8, nullptr);
		osmesa_buffer_.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4U);
		return osmesa_context_ != nullptr &&
		       OSMesaMakeCurrent(osmesa_context_, osmesa_buffer_.data(), GL_UNSIGNED_BYTE, width, height);
#else
		(void)width;
		(void)height;
		return false;
#endif
	}

	RenderStatus ReadColor(const RenderSnapshot &, const CameraDescriptor &camera, mjrRect viewport,
	                       FrameBoundary &boundary, bool segmentation)
	{
		if (segmentation) {
			scene_.flags[mjRND_SEGMENT] = 1;
		}
		const auto clear_segmentation = [this, segmentation]() {
			if (segmentation) {
				scene_.flags[mjRND_SEGMENT] = 0;
			}
		};
		mjr_render(viewport, &scene_, &context_);
		auto writer =
		    boundary.TryAcquireWriter(boundary.generation(), segmentation ? PlaneKind::kSegmentation : PlaneKind::kRgb,
		                              camera.layout(segmentation ? PlaneKind::kSegmentation : PlaneKind::kRgb));
		if (!writer.status().ok()) {
			clear_segmentation();
			return FromFrameStatus(writer.status());
		}
		mjr_readPixels(reinterpret_cast<unsigned char *>(writer.bytes().data()), nullptr, viewport, &context_);
		FlipRows(writer.bytes(), camera.layout(segmentation ? PlaneKind::kSegmentation : PlaneKind::kRgb).stride_bytes,
		         camera.height);
		const auto status = writer.Commit();
		clear_segmentation();
		return status.ok() ? RenderStatus::Ok() : FromFrameStatus(status);
	}

	RenderStatus ReadDepth(const RenderSnapshot &snapshot, mjrRect viewport, FrameBoundary &boundary,
	                       const CameraDescriptor &camera)
	{
		mjr_render(viewport, &scene_, &context_);
		auto writer =
		    boundary.TryAcquireWriter(boundary.generation(), PlaneKind::kDepth, camera.layout(PlaneKind::kDepth));
		if (!writer.status().ok()) {
			return FromFrameStatus(writer.status());
		}
		mjr_readPixels(nullptr, reinterpret_cast<float *>(writer.bytes().data()), viewport, &context_);
		FlipRows(writer.bytes(), camera.layout(PlaneKind::kDepth).stride_bytes, camera.height);
		try {
			const float znear = snapshot.model->vis.map.znear * snapshot.model->stat.extent;
			const float zfar  = snapshot.model->vis.map.zfar * snapshot.model->stat.extent;
			ConvertDepthBufferToMeters(writer.bytes(), znear, zfar);
		} catch (const std::exception &error) {
			return RenderStatus::Failure(RenderStatusCode::kFrameUnavailable, error.what());
		}
		const auto status = writer.Commit();
		return status.ok() ? RenderStatus::Ok() : FromFrameStatus(status);
	}

	mjrContext context_{};
	mjvScene scene_{};
	mjvCamera camera_{};
	mjvOption option_{};
	bool initialized_         = false;
	bool context_initialized_ = false;
	bool scene_initialized_   = false;
#if defined(MJR_RENDER_BACKEND_EGL)
	EGLDisplay egl_display_ = EGL_NO_DISPLAY;
	EGLSurface egl_surface_ = EGL_NO_SURFACE;
	EGLContext egl_context_ = EGL_NO_CONTEXT;
#elif defined(MJR_RENDER_BACKEND_OSMESA)
	OSMesaContext osmesa_context_ = nullptr;
	std::vector<unsigned char> osmesa_buffer_;
#endif
};

class DisabledRenderBackend final : public IRenderBackend
{
public:
	RenderStatus Initialize(const mjModel &, const RenderConfiguration &) override
	{
		return RenderStatus::Failure(RenderStatusCode::kBackendUnavailable, "render backend is disabled");
	}
	RenderStatus Resize(const RenderConfiguration &) override
	{
		return RenderStatus::Failure(RenderStatusCode::kBackendUnavailable, "render backend is disabled");
	}
	RenderStatus Render(const RenderSnapshot &, const CameraDescriptor &, PlaneMask, FrameBoundary &) override
	{
		return RenderStatus::Failure(RenderStatusCode::kBackendUnavailable, "render backend is disabled");
	}
	void ShutdownOnRenderThread() override {}
};

} // namespace

std::unique_ptr<IRenderBackend> CreateDisabledRenderBackend()
{
	return std::make_unique<DisabledRenderBackend>();
}

std::unique_ptr<IRenderBackend> CreateRenderBackend()
{
#if defined(MJR_RENDER_BACKEND_EGL) || defined(MJR_RENDER_BACKEND_OSMESA)
	return std::make_unique<MuJoCoRenderBackend>();
#else
	return CreateDisabledRenderBackend();
#endif
}

const char *CompiledRenderBackendName() noexcept
{
#if defined(MJR_RENDER_BACKEND_EGL)
	return "EGL";
#elif defined(MJR_RENDER_BACKEND_OSMESA)
	return "OSMESA";
#else
	return "NONE";
#endif
}

} // namespace mujoco_ros::rendering
