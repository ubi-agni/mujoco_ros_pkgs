#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <future>
#include <mutex>
#include <new>
#include <stdexcept>
#include <thread>

#include <mujoco_ros/glfw_adapter.h>
#include <mujoco_ros/glfw_dispatch.h>
#include <mujoco_ros/rendering/glfw_library.hpp>

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

bool monitor_available = false;

GLFWmonitor *PrimaryMonitor()
{
	static int fake_monitor;
	return monitor_available ? reinterpret_cast<GLFWmonitor *>(&fake_monitor) : nullptr;
}

const GLFWvidmode *VideoMode(GLFWmonitor *)
{
	static const GLFWvidmode mode{ 300, 300, 8, 8, 8, 60 };
	return &mode;
}

struct FakeWindow
{
	void *user                   = nullptr;
	GLFWcursorposfun cursor_cb   = nullptr;
	GLFWmousebuttonfun button_cb = nullptr;
	GLFWscrollfun scroll_cb      = nullptr;
	GLFWwindowsizefun size_cb    = nullptr;
};

FakeWindow fake_windows[8];
std::atomic<int> next_window{ 0 };
std::atomic<int> callbacks_run{ 0 };

FakeWindow *Fake(GLFWwindow *w)
{
	return reinterpret_cast<FakeWindow *>(w);
}

GLFWwindow *CreateWindow(int, int, const char *, GLFWmonitor *, GLFWwindow *)
{
	return reinterpret_cast<GLFWwindow *>(&fake_windows[next_window++ % 8]);
}

void SetWindowIcon(GLFWwindow *, int, const GLFWimage *) {}
void GetWindowPos(GLFWwindow *, int *x, int *y)
{
	*x = *y = 0;
}
void GetWindowSize(GLFWwindow *, int *x, int *y)
{
	*x = *y = 200;
}
void GetFramebufferSize(GLFWwindow *, int *x, int *y)
{
	*x = *y = 200;
}
void GetCursorPos(GLFWwindow *, double *x, double *y)
{
	*x = *y = 1.0;
}
int GetMouseButton(GLFWwindow *, int)
{
	return GLFW_PRESS;
}
int GetKey(GLFWwindow *, int)
{
	return GLFW_PRESS;
}
void SetWindowUserPointer(GLFWwindow *w, void *p)
{
	Fake(w)->user = p;
}
void *GetWindowUserPointer(GLFWwindow *w)
{
	return Fake(w)->user;
}
void MakeContextCurrent(GLFWwindow *) {}

GLFWcursorposfun SetCursorPosCallback(GLFWwindow *w, GLFWcursorposfun cb)
{
	Fake(w)->cursor_cb = cb;
	return nullptr;
}
GLFWmousebuttonfun SetMouseButtonCallback(GLFWwindow *w, GLFWmousebuttonfun cb)
{
	Fake(w)->button_cb = cb;
	return nullptr;
}
GLFWscrollfun SetScrollCallback(GLFWwindow *w, GLFWscrollfun cb)
{
	Fake(w)->scroll_cb = cb;
	return nullptr;
}
GLFWwindowsizefun SetWindowSizeCallback(GLFWwindow *w, GLFWwindowsizefun cb)
{
	Fake(w)->size_cb = cb;
	return nullptr;
}
GLFWdropfun SetDropCallback(GLFWwindow *, GLFWdropfun)
{
	return nullptr;
}
GLFWkeyfun SetKeyCallback(GLFWwindow *, GLFWkeyfun)
{
	return nullptr;
}
GLFWwindowrefreshfun SetWindowRefreshCallback(GLFWwindow *, GLFWwindowrefreshfun)
{
	return nullptr;
}

void GetMonitorPhysicalSizeFn(GLFWmonitor *, int *w, int *h)
{
	*w = *h = 300;
}
int WindowShouldCloseFn(GLFWwindow *)
{
	return 0;
}
std::atomic<int> clipboard_calls{ 0 };
std::atomic<int> swap_interval_calls{ 0 };
std::atomic<int> title_calls{ 0 };
void SetClipboardStringFn(GLFWwindow *, const char *)
{
	++clipboard_calls;
}
void SwapIntervalFn(int)
{
	++swap_interval_calls;
}
void SetWindowTitleFn(GLFWwindow *, const char *)
{
	++title_calls;
}

// Test hook run inside the mock glfwPollEvents, i.e. as if from a viewer UiEvent reached by a GLFW callback.
std::function<void()> poll_hook;

// Like real glfwPollEvents: runs the callbacks of the first window synchronously on the polling thread.
void PollEvents()
{
	FakeWindow &w      = fake_windows[0];
	GLFWwindow *handle = reinterpret_cast<GLFWwindow *>(&w);
	w.cursor_cb(handle, 1.0, 1.0);
	w.button_cb(handle, GLFW_MOUSE_BUTTON_LEFT, GLFW_PRESS, 0); // IsLeft/Ctrl/..., GetCursor/Window/Framebuffer
	w.scroll_cb(handle, 0.0, 1.0);
	w.size_cb(handle, 200, 200);
	if (poll_hook) {
		poll_hook();
	}
	++callbacks_run;
}

// Real glfwShowWindow/glfwSwapBuffers/glfwSetWindowMonitor can fire resize/refresh callbacks synchronously.
std::atomic<int> sync_callbacks_run{ 0 };
void FireSyncCallbacks(GLFWwindow *w)
{
	FakeWindow &f = *Fake(w);
	if (f.size_cb) {
		f.size_cb(w, 200, 200); // OnWindowResize -> getters
		++sync_callbacks_run;
	}
}
void ShowWindowFn(GLFWwindow *w)
{
	FireSyncCallbacks(w);
}
void SwapBuffersFn(GLFWwindow *w)
{
	FireSyncCallbacks(w);
}
GLFWmonitor *GetWindowMonitor(GLFWwindow *)
{
	return nullptr;
}
void SetWindowMonitor(GLFWwindow *w, GLFWmonitor *, int, int, int, int, int)
{
	FireSyncCallbacks(w);
}

GLFWwindow *GetCurrentContext()
{
	return nullptr;
}
int GetError(const char **description)
{
	if (description != nullptr) {
		*description = nullptr;
	}
	return GLFW_NO_ERROR;
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
		result.glfwInit                     = &Initialize;
		result.glfwTerminate                = &Terminate;
		result.glfwWindowHint               = &WindowHint;
		result.glfwWindowHintString         = &WindowHintString;
		result.glfwGetPrimaryMonitor        = &PrimaryMonitor;
		result.glfwDestroyWindow            = &DestroyWindow;
		result.glfwGetCurrentContext        = &GetCurrentContext;
		result.glfwGetError                 = &GetError;
		result.glfwGetVideoMode             = &VideoMode;
		result.glfwCreateWindow             = &CreateWindow;
		result.glfwSetWindowIcon            = &SetWindowIcon;
		result.glfwGetWindowPos             = &GetWindowPos;
		result.glfwGetWindowSize            = &GetWindowSize;
		result.glfwGetFramebufferSize       = &GetFramebufferSize;
		result.glfwGetCursorPos             = &GetCursorPos;
		result.glfwGetMouseButton           = &GetMouseButton;
		result.glfwGetKey                   = &GetKey;
		result.glfwSetWindowUserPointer     = &SetWindowUserPointer;
		result.glfwGetWindowUserPointer     = &GetWindowUserPointer;
		result.glfwMakeContextCurrent       = &MakeContextCurrent;
		result.glfwSetCursorPosCallback     = &SetCursorPosCallback;
		result.glfwSetMouseButtonCallback   = &SetMouseButtonCallback;
		result.glfwSetScrollCallback        = &SetScrollCallback;
		result.glfwSetWindowSizeCallback    = &SetWindowSizeCallback;
		result.glfwSetDropCallback          = &SetDropCallback;
		result.glfwSetKeyCallback           = &SetKeyCallback;
		result.glfwSetWindowRefreshCallback = &SetWindowRefreshCallback;
		result.glfwPollEvents               = &PollEvents;
		result.glfwShowWindow               = &ShowWindowFn;
		result.glfwSwapBuffers              = &SwapBuffersFn;
		result.glfwGetWindowMonitor         = &GetWindowMonitor;
		result.glfwSetWindowMonitor         = &SetWindowMonitor;
		result.glfwSetClipboardString       = &SetClipboardStringFn;
		result.glfwSwapInterval             = &SwapIntervalFn;
		result.glfwGetMonitorPhysicalSize   = &GetMonitorPhysicalSizeFn;
		result.glfwWindowShouldClose        = &WindowShouldCloseFn;
		result.glfwSetWindowTitle           = &SetWindowTitleFn;
		return result;
	}();
	return dispatch;
}

namespace {

TEST(GlfwAdapterFailure, MissingMonitorNeverDestroysAnUninitializedWindow)
{
	calls             = {};
	monitor_available = false;
	alignas(GlfwAdapter) std::byte storage[sizeof(GlfwAdapter)];
	std::memset(storage, 0xA5, sizeof(storage));

	EXPECT_THROW(new (storage) GlfwAdapter(false), std::runtime_error);
	EXPECT_EQ(calls.initialize, 1);
	EXPECT_EQ(calls.destroy, 0) << "constructor cleanup read the poisoned, uninitialized window member";
	EXPECT_EQ(calls.destroyed_window, nullptr);
	EXPECT_EQ(calls.terminate, 1);
}

TEST(GlfwAdapterFailure, SkipsTerminateWhenDisabledForEglCoexistence)
{
	calls             = {};
	monitor_available = true;
	next_window       = 0;
	rendering::SetGlfwLibraryTerminateOnLastRelease(false);
	{
		GlfwAdapter adapter(false);
	}
	EXPECT_EQ(calls.terminate, 0) << "EGL coexistence keeps GLFW initialized after last viewer window";
	rendering::SetGlfwLibraryTerminateOnLastRelease(true);
	{
		GlfwAdapter adapter(false);
	}
	EXPECT_EQ(calls.terminate, 1);
}

struct LibraryHookCalls
{
	int initialize = 0;
	int terminate  = 0;
};

LibraryHookCalls library_hooks;

int LibraryInitFail()
{
	++library_hooks.initialize;
	return 0;
}

int LibraryInitOk()
{
	++library_hooks.initialize;
	return GLFW_TRUE;
}

void LibraryTerminateHook()
{
	++library_hooks.terminate;
}

TEST(GlfwLibrary, InitFailureDoesNotAcquireAndExtraReleaseIsNoOp)
{
	library_hooks = {};
	rendering::SetGlfwLibraryTerminateOnLastRelease(true);
	std::lock_guard<std::mutex> lock(rendering::GlfwLibraryMutex());

	EXPECT_FALSE(rendering::AcquireGlfwLibrary(&LibraryInitFail));
	EXPECT_EQ(library_hooks.initialize, 1);

	// users==0: Release must not call terminate (covers the early return).
	rendering::ReleaseGlfwLibrary(&LibraryTerminateHook);
	EXPECT_EQ(library_hooks.terminate, 0);

	ASSERT_TRUE(rendering::AcquireGlfwLibrary(&LibraryInitOk));
	rendering::ReleaseGlfwLibrary(&LibraryTerminateHook);
	EXPECT_EQ(library_hooks.terminate, 1);

	// Second release with users already 0 stays a no-op.
	rendering::ReleaseGlfwLibrary(&LibraryTerminateHook);
	EXPECT_EQ(library_hooks.terminate, 1);
}

// PollEvents holds the GLFW library mutex while callbacks run; those callbacks query the adapter's getters,
// which must not re-lock it. Meanwhile another backend is created/destroyed on this thread, which must still
// serialize with PollEvents through the same mutex.
TEST(GlfwAdapterFailure, PollEventsCallbacksDoNotDeadlockAndSerializeWithBackendLifecycle)
{
	calls             = {};
	monitor_available = true;
	next_window       = 0;
	callbacks_run     = 0;
	for (auto &w : fake_windows) {
		w = FakeWindow{};
	}

	GlfwAdapter polled(false); // gets fake_windows[0]
	std::atomic<bool> stop{ false };

	auto poller = std::async(std::launch::async, [&]() {
		while (!stop) {
			polled.PollEvents();
		}
	});

	auto lifecycle = std::async(std::launch::async, [&]() {
		for (int i = 0; i < 200; ++i) {
			next_window = 1 + (i % 7);
			GlfwAdapter other(false);
		}
	});

	const auto deadline = std::chrono::seconds(20);
	if (lifecycle.wait_for(deadline) != std::future_status::ready) {
		ADD_FAILURE() << "backend create/destroy deadlocked against PollEvents";
		std::_Exit(1);
	}
	lifecycle.get();
	stop = true;
	if (poller.wait_for(deadline) != std::future_status::ready) {
		ADD_FAILURE() << "PollEvents deadlocked calling a getter from a GLFW callback";
		std::_Exit(1);
	}
	poller.get();
	EXPECT_GT(callbacks_run.load(), 0);
	monitor_available = false;
}

// ShowWindow, SwapBuffers and ToggleFullscreen fire callbacks synchronously on the calling thread.
// Getters called from those callbacks must not deadlock (CallbackScope, not a nested lock).
TEST(GlfwAdapterFailure, SyncGlfwCallsFiringCallbacksDoNotDeadlock)
{
	calls              = {};
	monitor_available  = true;
	next_window        = 0;
	sync_callbacks_run = 0;
	for (auto &w : fake_windows) {
		w = FakeWindow{};
	}

	auto run = std::async(std::launch::async, [&]() {
		GlfwAdapter a(false);
		a.ShowWindow();
		a.SwapBuffers();
		a.ToggleFullscreen();
	});
	if (run.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
		ADD_FAILURE() << "ShowWindow/SwapBuffers/ToggleFullscreen deadlocked on a getter in a callback";
		std::_Exit(1);
	}
	run.get();
	EXPECT_GE(sync_callbacks_run.load(), 3);
	monitor_available = false;
}

void ResetFakes()
{
	calls               = {};
	monitor_available   = true;
	next_window         = 0;
	callbacks_run       = 0;
	sync_callbacks_run  = 0;
	clipboard_calls     = 0;
	swap_interval_calls = 0;
	title_calls         = 0;
	poll_hook           = nullptr;
	for (auto &w : fake_windows) {
		w = FakeWindow{};
	}
}

// Fails (and exits, since a deadlocked thread cannot be joined) instead of hanging the test run.
template <typename F>
void RunOrFailOnTimeout(F &&f, const char *what)
{
	auto run = std::async(std::launch::async, std::forward<F>(f));
	if (run.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
		ADD_FAILURE() << what;
		std::_Exit(1);
	}
	run.get();
}

// The viewer's UiEvent (reached from OnKey/OnMouse inside PollEvents) calls these mutators; they must not
// re-lock the non-recursive library mutex that PollEvents already holds on this thread.
TEST(GlfwAdapterFailure, PollEventsCallbackMayCallMutators)
{
	ResetFakes();
	RunOrFailOnTimeout(
	    [&]() {
		    GlfwAdapter a(false);
		    poll_hook = [&]() {
			    a.ToggleFullscreen();
			    a.SetVSync(true);
			    a.SetClipboardString("x");
			    a.SetWindowTitle("t");
			    a.RefreshMjrContext(nullptr, -1);
			    a.GetDisplayPixelsPerInch();
			    a.ShouldCloseWindow();
			    a.SwapBuffers();
			    a.ShowWindow();
		    };
		    a.PollEvents();
		    poll_hook = nullptr;
	    },
	    "PollEvents deadlocked when a callback called ToggleFullscreen/SetVSync/SetClipboardString/...");
	EXPECT_EQ(swap_interval_calls.load(), 1);
	EXPECT_EQ(clipboard_calls.load(), 1);
	EXPECT_EQ(title_calls.load(), 1);
	monitor_available = false;
}

// glfwPollEvents can run another window's callbacks: the skip flag must be per-thread, not per-adapter.
TEST(GlfwAdapterFailure, PollOnOneAdapterRunsOtherAdapterCallbacksAndMutators)
{
	ResetFakes();
	RunOrFailOnTimeout(
	    [&]() {
		    GlfwAdapter polled(false); // fake_windows[0]
		    GlfwAdapter other(false); // fake_windows[1]
		    poll_hook = [&]() {
			    FakeWindow &w      = fake_windows[1];
			    GLFWwindow *handle = reinterpret_cast<GLFWwindow *>(&w);
			    w.cursor_cb(handle, 2.0, 2.0);
			    w.button_cb(handle, GLFW_MOUSE_BUTTON_LEFT, GLFW_PRESS, 0);
			    w.size_cb(handle, 200, 200);
			    other.ToggleFullscreen();
			    other.SetVSync(false);
			    other.SetClipboardString("y");
			    other.SetWindowTitle("u");
			    other.RefreshMjrContext(nullptr, -1);
		    };
		    polled.PollEvents();
		    poll_hook = nullptr;
		    // Outside any callback the lock is taken again normally.
		    other.SetVSync(true);
	    },
	    "polling one adapter deadlocked on the other adapter's callbacks/mutators");
	EXPECT_EQ(swap_interval_calls.load(), 2);
	EXPECT_EQ(clipboard_calls.load(), 1);
	EXPECT_EQ(title_calls.load(), 1);
	monitor_available = false;
}

} // namespace
} // namespace mujoco_ros
