#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <mujoco_ros/glfw_adapter.h>
#include <mujoco_ros/rendering/render_backend_interface.hpp>

namespace mujoco_ros {
namespace {

std::mutex glfw_error_mutex;
std::string glfw_errors;

void RecordGlfwError(int code, const char *description)
{
	std::lock_guard<std::mutex> lock(glfw_error_mutex);
	glfw_errors += "GLFW error " + std::to_string(code) + ": " + (description != nullptr ? description : "") + "\n";
}

std::shared_ptr<mjModel> MinimalModel()
{
	const char *xml = "<mujoco><visual><global offwidth=\"32\" offheight=\"32\"/></visual>"
	                  "<worldbody><geom type=\"sphere\" size=\"0.1\"/>"
	                  "<camera name=\"camera\" pos=\"0 -2 0\" euler=\"90 0 0\"/></worldbody></mujoco>";
	mjVFS vfs;
	mj_defaultVFS(&vfs);
	mj_addBufferVFS(&vfs, "race.xml", xml, std::strlen(xml));
	char error[1024] = {};
	auto *model      = mj_loadXML("race.xml", &vfs, error, sizeof(error));
	mj_deleteVFS(&vfs);
	if (model == nullptr) {
		throw std::runtime_error(error);
	}
	return std::shared_ptr<mjModel>(model, mj_deleteModel);
}

// Hidden GLFW offscreen Initialize/Render/Shutdown races a viewer that PollEvents and SwapBuffers
// on another thread. Window lifecycle is serialized by the GLFW mutex; Render GL work is not.
TEST(GlfwOffscreenViewerRace, ConcurrentBackendAndViewerWindowLifecycle)
{
	if (std::getenv("DISPLAY") == nullptr && std::getenv("WAYLAND_DISPLAY") == nullptr) {
		FAIL() << "this test is built only for GLFW offscreen rendering; set DISPLAY or run under Xvfb";
	}
	ASSERT_STREQ(rendering::CompiledRenderBackendName(), "GLFW");

	glfw_errors.clear();
	glfwSetErrorCallback(&RecordGlfwError);
	const auto model = MinimalModel();

	constexpr int kIterations = 40;
	std::atomic<int> failures{ 0 };
	std::atomic<bool> offscreen_done{ false };
	std::mutex failure_mutex;
	std::string failure_message;
	const auto fail = [&](const std::string &message) {
		std::lock_guard<std::mutex> lock(failure_mutex);
		++failures;
		failure_message += message + "\n";
	};

	auto offscreen      = std::async(std::launch::async, [&]() {
      struct MarkDone
      {
         std::atomic<bool> &flag;
         ~MarkDone() { flag.store(true); }
      } mark_done{ offscreen_done };
      for (int i = 0; i < kIterations && failures == 0; ++i) {
         auto backend = rendering::CreateRenderBackend();
         rendering::RenderConfiguration configuration;
         configuration.generation   = FrameGeneration(1);
         configuration.frame_layout = rendering::FrameLayout(32, 32);
         const auto status          = backend->Initialize(*model, configuration);
         if (!status.ok()) {
            fail("offscreen Initialize failed: " + status.message);
            return;
         }
         rendering::FrameBoundary boundary(1, configuration.frame_layout.slot_byte_length);
         const auto reconfigured = boundary.Reconfigure(FrameGeneration(1), configuration.frame_layout, 1);
         if (!reconfigured.ok()) {
            fail("offscreen FrameBoundary Reconfigure failed: " + reconfigured.message);
            return;
         }
         rendering::RenderSnapshot snapshot;
         snapshot.model_generation = ModelGeneration(1);
         snapshot.model            = model;
         snapshot.data             = std::shared_ptr<mjData>(mj_makeData(model.get()), mj_deleteData);
         mj_forward(model.get(), snapshot.data.get());
         rendering::CameraDescriptor camera{ rendering::CameraId(1), "camera", 32, 32, rendering::PlaneMask::kRgb };
         const auto rendered = backend->Render(snapshot, camera, rendering::PlaneMask::kRgb, boundary);
         if (!rendered.ok()) {
            fail("offscreen Render failed: " + rendered.message);
            return;
         }
         backend->ShutdownOnRenderThread();
      }
   });
	auto viewer         = std::async(std::launch::async, [&]() {
      try {
         GlfwAdapter adapter(false);
         adapter.SetVSync(false);
         while (!offscreen_done.load() && failures == 0) {
            adapter.PollEvents();
            adapter.SwapBuffers();
            std::this_thread::yield();
         }
      } catch (const std::exception &error) {
         fail(std::string("viewer GlfwAdapter failed: ") + error.what());
      }
   });
	const auto deadline = std::chrono::seconds(20);
	if (offscreen.wait_for(deadline) != std::future_status::ready) {
		ADD_FAILURE() << "offscreen create/destroy deadlocked against viewer PollEvents/SwapBuffers";
		std::_Exit(1);
	}
	if (viewer.wait_for(deadline) != std::future_status::ready) {
		ADD_FAILURE() << "viewer PollEvents/SwapBuffers deadlocked against offscreen create/destroy";
		std::_Exit(1);
	}
	offscreen.get();
	viewer.get();

	EXPECT_EQ(failures.load(), 0) << failure_message;
	{
		std::lock_guard<std::mutex> lock(glfw_error_mutex);
		EXPECT_TRUE(glfw_errors.empty()) << glfw_errors;
	}
	glfwSetErrorCallback(nullptr);
}

} // namespace
} // namespace mujoco_ros
