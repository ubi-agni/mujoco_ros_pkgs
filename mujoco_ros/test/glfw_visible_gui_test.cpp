#include <gtest/gtest.h>

#include <cstdlib>

#include <mujoco_ros/glfw_adapter.h>

namespace mujoco_ros {
namespace {

bool NativeDisplayIsAdvertised()
{
	return std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
}

TEST(GlfwVisibleGui, OwnsInitializationAndWindowOnGuiThread)
{
	if (!NativeDisplayIsAdvertised()) {
		GTEST_SKIP() << "native display unavailable: DISPLAY and WAYLAND_DISPLAY are unset";
	}

	GlfwAdapter gui(false);
	const auto [width, height] = gui.GetWindowSize();
	ASSERT_GT(width, 0);
	ASSERT_GT(height, 0);
	EXPECT_TRUE(gui.IsGPUAccelerated());
}

} // namespace
} // namespace mujoco_ros
