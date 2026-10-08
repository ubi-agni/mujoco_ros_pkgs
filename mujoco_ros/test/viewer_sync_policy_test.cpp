#include <gtest/gtest.h>

#include <mujoco_ros/viewer_sync_policy.hpp>

namespace mujoco_ros {
namespace {

TEST(ViewerSyncPolicy, ManagedViewerSyncsWhenEnvironmentIsIdle)
{
	EXPECT_TRUE(ShouldSyncViewerEachFrame(false, false, 0));
	EXPECT_TRUE(ShouldSyncViewerEachFrame(false, true, 0));
}

TEST(ViewerSyncPolicy, ManualPassiveViewerNeverSyncsAutomatically)
{
	EXPECT_FALSE(ShouldSyncViewerEachFrame(true, false, 0));
}

TEST(ViewerSyncPolicy, AutomaticPassiveViewerSyncsWhenEnvironmentIsIdle)
{
	EXPECT_TRUE(ShouldSyncViewerEachFrame(true, true, 0));
}

TEST(ViewerSyncPolicy, NoViewerSyncRunsDuringLoadOrReset)
{
	EXPECT_FALSE(ShouldSyncViewerEachFrame(false, false, 1));
	EXPECT_FALSE(ShouldSyncViewerEachFrame(true, true, 1));
	EXPECT_FALSE(ShouldSyncViewerEachFrame(true, true, -1));
}

TEST(ViewerSyncPolicy, OnlyManagedViewerExitShutsDownEnvironment)
{
	EXPECT_TRUE(ShouldShutdownEnvironmentOnViewerExit(false));
	EXPECT_FALSE(ShouldShutdownEnvironmentOnViewerExit(true));
}

TEST(ViewerSyncPolicy, PendingUiExitStopsManualPassiveViewerLocally)
{
	const auto action = ComputePendingViewerExitAction(true, true, 0);
	EXPECT_TRUE(action.set_exit_request);
	EXPECT_FALSE(action.request_environment_shutdown);
}

TEST(ViewerSyncPolicy, PendingUiExitStopsManagedViewerAndEnvironment)
{
	const auto action = ComputePendingViewerExitAction(true, false, 0);
	EXPECT_TRUE(action.set_exit_request);
	EXPECT_TRUE(action.request_environment_shutdown);
}

TEST(ViewerSyncPolicy, PendingUiExitIgnoredAfterExitAlreadyRequested)
{
	const auto action = ComputePendingViewerExitAction(true, true, 1);
	EXPECT_FALSE(action.set_exit_request);
	EXPECT_FALSE(action.request_environment_shutdown);
}

TEST(ViewerSyncPolicy, PendingUiExitIgnoredWhenNotPending)
{
	const auto action = ComputePendingViewerExitAction(false, true, 0);
	EXPECT_FALSE(action.set_exit_request);
	EXPECT_FALSE(action.request_environment_shutdown);
}

TEST(ViewerSyncPolicy, ManualPassiveExitFallbackAllowedDuringLoad)
{
	EXPECT_FALSE(ShouldSyncViewerEachFrame(true, false, 1));
	EXPECT_TRUE(ShouldProcessPendingViewerExitWithoutSync(true, false, 1));
}

TEST(ViewerSyncPolicy, ManagedViewerExitFallbackBlockedDuringLoad)
{
	EXPECT_FALSE(ShouldSyncViewerEachFrame(false, false, 1));
	EXPECT_FALSE(ShouldProcessPendingViewerExitWithoutSync(false, false, 1));
}

TEST(ViewerSyncPolicy, AutomaticPassiveViewerExitFallbackAllowedDuringLoad)
{
	EXPECT_FALSE(ShouldSyncViewerEachFrame(true, true, 1));
	EXPECT_TRUE(ShouldProcessPendingViewerExitWithoutSync(true, true, 1));
}

} // namespace
} // namespace mujoco_ros
