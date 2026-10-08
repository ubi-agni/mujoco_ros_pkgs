#include <gtest/gtest.h>

#include <cstddef>
#include <cstring>
#include <new>
#include <stdexcept>

#include <mujoco_ros/glfw_adapter.h>
#include <mujoco_ros/glfw_dispatch.h>

namespace mujoco_ros {
namespace {

struct GlfwCalls
{
	int initialize               = 0;
	int terminate                = 0;
	int destroy                  = 0;
	GLFWwindow *destroyed_window = nullptr;
};

GlfwCalls calls;

int Initialize()
{
	++calls.initialize;
	return GLFW_TRUE;
}

void Terminate()
{
	++calls.terminate;
}

void WindowHint(int, int) {}

void WindowHintString(int, const char *) {}

GLFWmonitor *NoPrimaryMonitor()
{
	return nullptr;
}

void DestroyWindow(GLFWwindow *window)
{
	++calls.destroy;
	calls.destroyed_window = window;
}

} // namespace

const struct Glfw &Glfw(void *)
{
	static const struct Glfw dispatch = []() {
		struct Glfw result
		{};
		result.glfwInit              = &Initialize;
		result.glfwTerminate         = &Terminate;
		result.glfwWindowHint        = &WindowHint;
		result.glfwWindowHintString  = &WindowHintString;
		result.glfwGetPrimaryMonitor = &NoPrimaryMonitor;
		result.glfwDestroyWindow     = &DestroyWindow;
		return result;
	}();
	return dispatch;
}

namespace {

TEST(GlfwAdapterFailure, MissingMonitorNeverDestroysAnUninitializedWindow)
{
	calls = {};
	alignas(GlfwAdapter) std::byte storage[sizeof(GlfwAdapter)];
	std::memset(storage, 0xA5, sizeof(storage));

	EXPECT_THROW(new (storage) GlfwAdapter(false), std::runtime_error);
	EXPECT_EQ(calls.initialize, 1);
	EXPECT_EQ(calls.destroy, 0) << "constructor cleanup read the poisoned, uninitialized window member";
	EXPECT_EQ(calls.destroyed_window, nullptr);
	EXPECT_EQ(calls.terminate, 1);
}

} // namespace
} // namespace mujoco_ros
