// Copyright 2023 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* slight modifications by David P. Leins */

#include <mujoco_ros/glfw_adapter.h>

#include <cstdlib>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <GLFW/glfw3.h>
#include <mujoco_ros/rendering/glfw_library.hpp>
#include <mujoco/mjui.h>
#include <mujoco/mujoco.h>
#include <mujoco_ros/glfw_dispatch.h>
#include <mujoco_ros/viewer_branding.hpp>

namespace mujoco_ros {
namespace {
GlfwAdapter &GlfwAdapterFromWindow(GLFWwindow *window)
{
	return *static_cast<GlfwAdapter *>(Glfw().glfwGetWindowUserPointer(window));
}

GLFWmonitor *PrimaryMonitorOrThrow()
{
	GLFWmonitor *monitor = Glfw().glfwGetPrimaryMonitor();
	if (monitor == nullptr) {
		throw std::runtime_error("GLFW primary monitor is unavailable; viewer requires an active display");
	}
	return monitor;
}

} // namespace

GlfwAdapter::GlfwAdapter(bool visible)
{
	owner_thread_ = std::this_thread::get_id();
	std::unique_lock<std::mutex> glfw_lock(rendering::GlfwLibraryMutex());
	if (!rendering::AcquireGlfwLibrary(Glfw().glfwInit)) {
		throw std::runtime_error("Failed to initialize GLFW on the visible GUI owner thread");
	}
	glfw_initialized_ = true;

	try {
		Glfw().glfwWindowHint(GLFW_SAMPLES, 4);
		Glfw().glfwWindowHint(GLFW_VISIBLE, visible ? GLFW_TRUE : GLFW_FALSE);
		Glfw().glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
#if defined(GLFW_X11_CLASS_NAME)
		Glfw().glfwWindowHintString(GLFW_X11_CLASS_NAME, "mujoco_ros");
		Glfw().glfwWindowHintString(GLFW_X11_INSTANCE_NAME, "mujoco_ros");
#endif

		GLFWmonitor *monitor          = PrimaryMonitorOrThrow();
		const GLFWvidmode *video_mode = Glfw().glfwGetVideoMode(monitor);
		if (video_mode == nullptr) {
			throw std::runtime_error("GLFW primary monitor has no video mode; viewer cannot determine window size");
		}
		vidmode_ = *video_mode;
		window_ =
		    Glfw().glfwCreateWindow((2 * vidmode_.width) / 3, (2 * vidmode_.height) / 3, "Mujoco ROS", nullptr, nullptr);

		if (!window_) {
			throw std::runtime_error("Failed to create GLFW window");
		}
		glfw_lock.unlock();
		window_visible_ = visible;

		{
			std::lock_guard<std::mutex> post_create_lock(rendering::GlfwLibraryMutex());
			auto icon = LoadViewerBrandingAsset("mj_ros_icon.png", ViewerBrandingPixelFormat::kRgba);
			const GLFWimage icon_image{ static_cast<int>(icon.width), static_cast<int>(icon.height), icon.pixels.data() };
			Glfw().glfwSetWindowIcon(window_, 1, &icon_image);

			// save window position and size
			Glfw().glfwGetWindowPos(window_, &window_pos_.first, &window_pos_.second);
			Glfw().glfwGetWindowSize(window_, &window_size_.first, &window_size_.second);

			// set callbacks
			Glfw().glfwSetWindowUserPointer(window_, this);
			Glfw().glfwSetDropCallback(
			    window_, +[](GLFWwindow *window, int count, const char **paths) {
				    GlfwAdapterFromWindow(window).OnFilesDrop(count, paths);
			    });
			Glfw().glfwSetKeyCallback(
			    window_, +[](GLFWwindow *window, int key, int scancode, int act, int /*mods*/) {
				    GlfwAdapterFromWindow(window).OnKey(key, scancode, act);
			    });
			Glfw().glfwSetMouseButtonCallback(
			    window_, +[](GLFWwindow *window, int button, int act, int /*mods*/) {
				    GlfwAdapterFromWindow(window).OnMouseButton(button, act);
			    });
			Glfw().glfwSetCursorPosCallback(
			    window_, +[](GLFWwindow *window, double x, double y) { GlfwAdapterFromWindow(window).OnMouseMove(x, y); });
			Glfw().glfwSetScrollCallback(
			    window_, +[](GLFWwindow *window, double xoffset, double yoffset) {
				    GlfwAdapterFromWindow(window).OnScroll(xoffset, yoffset);
			    });
			Glfw().glfwSetWindowRefreshCallback(
			    window_, +[](GLFWwindow *window) { GlfwAdapterFromWindow(window).OnWindowRefresh(); });
			Glfw().glfwSetWindowSizeCallback(
			    window_, +[](GLFWwindow *window, int width, int height) {
				    GlfwAdapterFromWindow(window).OnWindowResize(width, height);
			    });

			// make context current
			Glfw().glfwMakeContextCurrent(window_);
		}
	} catch (...) {
		if (!glfw_lock.owns_lock()) {
			glfw_lock.lock();
		}
		if (window_ != nullptr) {
			Glfw().glfwDestroyWindow(window_);
			window_ = nullptr;
		}
		if (glfw_initialized_) {
			rendering::ReleaseGlfwLibrary(Glfw().glfwTerminate);
			glfw_initialized_ = false;
		}
		throw;
	}
}

void GlfwAdapter::RevealWindowUnderLock()
{
	if (window_visible_) {
		return;
	}
	Glfw().glfwMakeContextCurrent(window_);
	Glfw().glfwShowWindow(window_);
	window_visible_ = true;
}

void GlfwAdapter::ShowWindow()
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	rendering::GlfwCallbackScope scope;
	RevealWindowUnderLock();
}

GlfwAdapter::~GlfwAdapter()
{
	if (std::this_thread::get_id() != owner_thread_) {
		std::terminate();
	}
	std::lock_guard<std::mutex> glfw_lock(rendering::GlfwLibraryMutex());
	if (window_ != nullptr) {
		Glfw().glfwMakeContextCurrent(nullptr);
		Glfw().glfwDestroyWindow(window_);
		window_ = nullptr;
	}
	if (glfw_initialized_) {
		rendering::ReleaseGlfwLibrary(Glfw().glfwTerminate);
		glfw_initialized_ = false;
	}
}

bool GlfwAdapter::RefreshMjrContext(const mjModel *m, int fontscale)
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	Glfw().glfwMakeContextCurrent(window_);
	return PlatformUIAdapter::RefreshMjrContext(m, fontscale);
}

std::pair<double, double> GlfwAdapter::GetCursorPosition() const
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	double x, y;
	Glfw().glfwGetCursorPos(window_, &x, &y);
	return { x, y };
}

double GlfwAdapter::GetDisplayPixelsPerInch() const
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	int width_mm, height_mm;
	Glfw().glfwGetMonitorPhysicalSize(PrimaryMonitorOrThrow(), &width_mm, &height_mm);
	return 25.4 * vidmode_.width / width_mm;
}

std::pair<int, int> GlfwAdapter::GetFramebufferSize() const
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	int width, height;
	Glfw().glfwGetFramebufferSize(window_, &width, &height);
	return { width, height };
}

std::pair<int, int> GlfwAdapter::GetWindowSize() const
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	int width, height;
	Glfw().glfwGetWindowSize(window_, &width, &height);
	return { width, height };
}

bool GlfwAdapter::IsGPUAccelerated() const
{
	return true;
}

void GlfwAdapter::PollEvents()
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	rendering::GlfwCallbackScope scope;
	Glfw().glfwPollEvents();
}

void GlfwAdapter::SetClipboardString(const char *text)
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	Glfw().glfwSetClipboardString(window_, text);
}

void GlfwAdapter::SetVSync(bool enabled)
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	Glfw().glfwSwapInterval(enabled);
}

void GlfwAdapter::SetWindowTitle(const char *title)
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	Glfw().glfwSetWindowTitle(window_, title);
}

bool GlfwAdapter::ShouldCloseWindow() const
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	return Glfw().glfwWindowShouldClose(window_);
}

void GlfwAdapter::SwapBuffers()
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	{
		rendering::GlfwCallbackScope scope;
		RevealWindowUnderLock();
	}
	if (glfw_lock.owns_lock()) {
		glfw_lock.unlock();
	}
	Glfw().glfwSwapBuffers(window_);
}

void GlfwAdapter::ToggleFullscreen()
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	rendering::GlfwCallbackScope scope;
	// currently fullscreen: switch to windowed
	if (Glfw().glfwGetWindowMonitor(window_)) {
		// restore window size and position
		Glfw().glfwSetWindowMonitor(window_, nullptr, window_pos_.first, window_pos_.second, window_size_.first,
		                            window_size_.second, 0);
		// currently windowed: switch to fullscreen
	} else {
		// save window data
		Glfw().glfwGetWindowPos(window_, &window_pos_.first, &window_pos_.second);
		Glfw().glfwGetWindowSize(window_, &window_size_.first, &window_size_.second);

		// switch
		Glfw().glfwSetWindowMonitor(window_, PrimaryMonitorOrThrow(), 0, 0, vidmode_.width, vidmode_.height,
		                            vidmode_.refreshRate);
	}
}

bool GlfwAdapter::IsLeftMouseButtonPressed() const
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	return Glfw().glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
}

bool GlfwAdapter::IsMiddleMouseButtonPressed() const
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	return Glfw().glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
}

bool GlfwAdapter::IsRightMouseButtonPressed() const
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	return Glfw().glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
}

bool GlfwAdapter::IsAltKeyPressed() const
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	return Glfw().glfwGetKey(window_, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
	       Glfw().glfwGetKey(window_, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
}

bool GlfwAdapter::IsCtrlKeyPressed() const
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	return Glfw().glfwGetKey(window_, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
	       Glfw().glfwGetKey(window_, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
}

bool GlfwAdapter::IsShiftKeyPressed() const
{
	auto glfw_lock = rendering::LockGlfwLibraryUnlessInCallback();
	return Glfw().glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
	       Glfw().glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
}

bool GlfwAdapter::IsMouseButtonDownEvent(int act) const
{
	return act == GLFW_PRESS;
}

bool GlfwAdapter::IsKeyDownEvent(int act) const
{
	return act == GLFW_PRESS;
}

int GlfwAdapter::TranslateKeyCode(int key) const
{
	return key;
}

mjtButton GlfwAdapter::TranslateMouseButton(int button) const
{
	if (button == GLFW_MOUSE_BUTTON_LEFT) {
		return mjBUTTON_LEFT;
	} else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
		return mjBUTTON_MIDDLE;
	} else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
		return mjBUTTON_RIGHT;
	}
	return mjBUTTON_NONE;
}
} // namespace mujoco_ros
