#include <mujoco_ros/rendering/glfw_library.hpp>

#include <cstddef>

namespace mujoco_ros::rendering {
namespace {
std::mutex glfw_library_mutex;
std::size_t glfw_library_users                      = 0;
bool glfw_library_terminate_on_last_release         = true;
thread_local bool callbacks_expected_on_this_thread = false;
} // namespace

GlfwCallbackScope::GlfwCallbackScope() : previous_(callbacks_expected_on_this_thread)
{
	callbacks_expected_on_this_thread = true;
}

GlfwCallbackScope::~GlfwCallbackScope()
{
	callbacks_expected_on_this_thread = previous_;
}

std::unique_lock<std::mutex> LockGlfwLibraryUnlessInCallback()
{
	if (callbacks_expected_on_this_thread) {
		return std::unique_lock<std::mutex>(glfw_library_mutex, std::defer_lock);
	}
	return std::unique_lock<std::mutex>(glfw_library_mutex);
}

std::mutex &GlfwLibraryMutex()
{
	return glfw_library_mutex;
}

bool AcquireGlfwLibrary(int (*init)())
{
	if (glfw_library_users == 0 && !init()) {
		return false;
	}
	++glfw_library_users;
	return true;
}

void ReleaseGlfwLibrary(void (*terminate)())
{
	if (glfw_library_users == 0) {
		return;
	}
	--glfw_library_users;
	if (glfw_library_users == 0 && glfw_library_terminate_on_last_release) {
		terminate();
	}
}

void SetGlfwLibraryTerminateOnLastRelease(bool enabled)
{
	glfw_library_terminate_on_last_release = enabled;
}

} // namespace mujoco_ros::rendering
