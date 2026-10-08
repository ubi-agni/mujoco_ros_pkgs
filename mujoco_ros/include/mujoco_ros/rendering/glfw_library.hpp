#pragma once

#include <mutex>

namespace mujoco_ros::rendering {

// Process-wide GLFW init/terminate refcount. The visible viewer and the hidden offscreen context
// share one GLFW instance; terminating while either still owns a window would invalidate it.
// Callers pass their own glfwInit/glfwTerminate (GLFW_TRUE-returning init) so tests can inject a
// dispatch table without this unit depending on GLFW.
//
// All GLFW library use (acquire, release, hints, create, destroy, poll, input queries, etc.)
// must be serialized with GlfwLibraryMutex(). AcquireGlfwLibrary and ReleaseGlfwLibrary must only be
// called while that mutex is held. glfwSwapBuffers runs after RevealWindow without holding the mutex
// so vsync cannot stall other GLFW owners; swap callbacks take the mutex normally.
std::mutex &GlfwLibraryMutex();

// RAII: marks this thread as inside a callback-firing GLFW call. Construct only while holding
// GlfwLibraryMutex() (or when already inside a scope). Nests; restores the previous state.
class GlfwCallbackScope
{
public:
	GlfwCallbackScope();
	~GlfwCallbackScope();
	GlfwCallbackScope(const GlfwCallbackScope &)            = delete;
	GlfwCallbackScope &operator=(const GlfwCallbackScope &) = delete;

private:
	bool previous_;
};

// Locks GlfwLibraryMutex() unless this thread is already inside a GlfwCallbackScope (then the returned
// lock is empty because the mutex is already held by this thread).
std::unique_lock<std::mutex> LockGlfwLibraryUnlessInCallback();

bool AcquireGlfwLibrary(int (*init)());
void ReleaseGlfwLibrary(void (*terminate)());

// When false, the last ReleaseGlfwLibrary skips terminate(). Used for GLFW viewer + EGL
// offscreen: glfwTerminate poisons process GL state that EGL RenderCore still needs.
// Default true. Call while holding GlfwLibraryMutex() or before any Acquire/Release.
void SetGlfwLibraryTerminateOnLastRelease(bool enabled);

} // namespace mujoco_ros::rendering
