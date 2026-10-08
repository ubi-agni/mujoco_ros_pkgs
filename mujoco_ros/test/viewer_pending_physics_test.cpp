#include <gtest/gtest.h>

#include <mujoco_ros/viewer_pending_physics.hpp>

namespace mujoco_ros {
namespace {

struct Pending
{
	int manual_steps               = 0;
	bool history_after_manual_step = false;
	bool apply_model_flags         = false;
};

TEST(ViewerPendingPhysics, DrainRequestsStepsWhenPausedAndClearsQueue)
{
	Pending pending;
	pending.manual_steps              = 100;
	pending.history_after_manual_step = true;
	const auto result                 = DrainDeferredPhysicsUi(pending, false, true);
	EXPECT_EQ(result.request_manual_steps, 100);
	EXPECT_TRUE(result.add_to_history);
	EXPECT_EQ(pending.manual_steps, 0);
	EXPECT_FALSE(pending.history_after_manual_step);
}

TEST(ViewerPendingPhysics, DrainDropsStepsWhileRunning)
{
	Pending pending;
	pending.manual_steps = 1;
	const auto result    = DrainDeferredPhysicsUi(pending, true, true);
	EXPECT_EQ(result.request_manual_steps, 0);
	EXPECT_EQ(pending.manual_steps, 0);
}

TEST(ViewerPendingPhysics, DrainAppliesFlagsOnlyWhenModelExists)
{
	Pending with_model;
	with_model.apply_model_flags = true;
	const auto applied           = DrainDeferredPhysicsUi(with_model, false, true);
	EXPECT_TRUE(applied.apply_model_flags);
	EXPECT_FALSE(with_model.apply_model_flags);

	Pending without_model;
	without_model.apply_model_flags = true;
	const auto skipped              = DrainDeferredPhysicsUi(without_model, false, false);
	EXPECT_FALSE(skipped.apply_model_flags);
	EXPECT_FALSE(without_model.apply_model_flags);
}

TEST(ViewerPendingPhysics, LoadClearDropsStaleStepsAndFlags)
{
	Pending pending;
	pending.manual_steps              = 5;
	pending.history_after_manual_step = true;
	pending.apply_model_flags         = true;
	ClearDeferredPhysicsUi(pending);
	EXPECT_EQ(pending.manual_steps, 0);
	EXPECT_FALSE(pending.history_after_manual_step);
	EXPECT_FALSE(pending.apply_model_flags);
}

} // namespace
} // namespace mujoco_ros
