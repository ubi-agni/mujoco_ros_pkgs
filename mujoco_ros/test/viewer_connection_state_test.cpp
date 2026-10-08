#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include <mujoco_ros/detail/viewer_connection_state.hpp>

namespace mujoco_ros {
namespace {

// Opaque stand-ins: ViewerConnectionState only stores these pointers for lease APIs.
struct DummyEnv
{};
struct DummyViewer
{};

TEST(ViewerConnectionStateTest, ClosedEnvironmentRejectsLeasesAndAdmission)
{
	DummyEnv env;
	DummyViewer viewer;
	auto state = std::make_shared<ViewerConnectionState>(reinterpret_cast<MujocoEnv *>(&env),
	                                                     reinterpret_cast<Viewer *>(&viewer));

	EXPECT_TRUE(state->AdmissionOpen());
	auto open_lease = state->TryAcquireEnvironment();
	ASSERT_TRUE(static_cast<bool>(open_lease));
	open_lease = EnvironmentLease();

	state->CloseEnvironment();
	EXPECT_FALSE(state->AdmissionOpen());
	EXPECT_TRUE(state->EnvironmentClosed());
	EXPECT_FALSE(static_cast<bool>(state->TryAcquireEnvironment()));
}

TEST(ViewerConnectionStateTest, RenderLoopActivateRejectsSecondOwner)
{
	DummyEnv env;
	DummyViewer viewer;
	auto state = std::make_shared<ViewerConnectionState>(reinterpret_cast<MujocoEnv *>(&env),
	                                                     reinterpret_cast<Viewer *>(&viewer));

	ASSERT_TRUE(state->TryActivateRenderLoop(std::this_thread::get_id()));
	EXPECT_FALSE(state->TryActivateRenderLoop(std::this_thread::get_id()));
	state->FinishRenderLoop();
	EXPECT_TRUE(state->TryActivateRenderLoop(std::this_thread::get_id()));
	state->FinishRenderLoop();
}

TEST(ViewerConnectionStateTest, OperationLeaseMoveAssignReleasesPrevious)
{
	DummyEnv env;
	DummyViewer viewer;
	auto state = std::make_shared<ViewerConnectionState>(reinterpret_cast<MujocoEnv *>(&env),
	                                                     reinterpret_cast<Viewer *>(&viewer));

	auto first  = state->TryAcquireViewerOperation();
	auto second = state->TryAcquireViewerOperation();
	ASSERT_TRUE(static_cast<bool>(first));
	ASSERT_TRUE(static_cast<bool>(second));

	first = std::move(second);
	EXPECT_TRUE(static_cast<bool>(first));

	// One lease remains; WaitForLeases must return once it is dropped.
	auto drained = std::async(std::launch::async, [&]() { state->WaitForLeases(); });
	EXPECT_EQ(drained.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
	first = ViewerOperationLease();
	EXPECT_EQ(drained.wait_for(std::chrono::seconds(1)), std::future_status::ready);
}

TEST(ViewerConnectionStateTest, AcquireViewerOperationLeasePairsWithWaitForLeases)
{
	DummyEnv env;
	DummyViewer viewer;
	auto state = std::make_shared<ViewerConnectionState>(reinterpret_cast<MujocoEnv *>(&env),
	                                                     reinterpret_cast<Viewer *>(&viewer));

	state->AcquireViewerOperationLease();
	auto drained = std::async(std::launch::async, [&]() { state->WaitForLeases(); });
	EXPECT_EQ(drained.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
	state->ReleaseViewerOperationLease();
	EXPECT_EQ(drained.wait_for(std::chrono::seconds(1)), std::future_status::ready);
}

TEST(ViewerConnectionStateTest, DetachedViewerYieldsEmptyOperationLease)
{
	DummyEnv env;
	DummyViewer viewer;
	auto state = std::make_shared<ViewerConnectionState>(reinterpret_cast<MujocoEnv *>(&env),
	                                                     reinterpret_cast<Viewer *>(&viewer));
	state->DetachViewer();
	EXPECT_FALSE(static_cast<bool>(state->TryAcquireViewerOperation()));
}

} // namespace
} // namespace mujoco_ros
