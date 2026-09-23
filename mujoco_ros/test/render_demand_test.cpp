#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include <mujoco_ros/rendering/render_demand.hpp>
#include <mujoco_ros/rendering/frame_capacity.hpp>

namespace mujoco_ros::rendering {

TEST(RenderDemand, PythonOneShotDoesNotAdvanceRosCadence)
{
	DemandScheduler scheduler;
	auto ros    = scheduler.RegisterCadencedConsumer("ros", std::chrono::milliseconds(100));
	auto python = scheduler.RegisterOneShotConsumer("python");

	scheduler.RequestOneShot(python);
	auto plan = scheduler.Evaluate(std::chrono::milliseconds(50), CameraId(1));
	ASSERT_EQ(plan.consumers.size(), 1U);
	EXPECT_EQ(plan.consumers.front(), python);
	scheduler.MarkDelivered(plan, python);

	auto due = scheduler.Evaluate(std::chrono::milliseconds(100), CameraId(1));
	ASSERT_EQ(due.consumers.size(), 1U);
	EXPECT_EQ(due.consumers.front(), ros);
}

TEST(RenderDemand, NoDemandProducesEmptyPlan)
{
	DemandScheduler scheduler;
	const auto plan = scheduler.Evaluate(std::chrono::milliseconds(1), CameraId(1));
	EXPECT_TRUE(plan.consumers.empty());
}

TEST(RenderDemand, OneCaptureSelectsAllDueConsumers)
{
	DemandScheduler scheduler;
	auto first  = scheduler.RegisterContinuousConsumer("first");
	auto second = scheduler.RegisterContinuousConsumer("second");

	const auto plan = scheduler.Evaluate(std::chrono::milliseconds(1), CameraId(2));
	ASSERT_EQ(plan.consumers.size(), 2U);
	EXPECT_EQ(plan.consumers[0], first);
	EXPECT_EQ(plan.consumers[1], second);
}

TEST(RenderDemand, ConsumerDemandIsBoundToItsCamera)
{
	DemandScheduler scheduler;
	auto ros_camera_one =
	    scheduler.RegisterCadencedConsumer("ros-camera-one", std::chrono::milliseconds(1), CameraId(1));
	auto python_camera_two = scheduler.RegisterOneShotConsumer("python-camera-two", CameraId(2));

	auto camera_one_plan = scheduler.Evaluate(std::chrono::milliseconds(2), CameraId(1));
	ASSERT_EQ(camera_one_plan.consumers.size(), 1U);
	EXPECT_EQ(camera_one_plan.consumers.front(), ros_camera_one);

	auto camera_two_plan = scheduler.Evaluate(std::chrono::milliseconds(2), CameraId(2));
	EXPECT_TRUE(camera_two_plan.consumers.empty());

	scheduler.RequestOneShot(python_camera_two);
	camera_two_plan = scheduler.Evaluate(std::chrono::milliseconds(2), CameraId(2));
	ASSERT_EQ(camera_two_plan.consumers.size(), 1U);
	EXPECT_EQ(camera_two_plan.consumers.front(), python_camera_two);
}

TEST(RenderDemand, IdleOneShotConsumerProducesNoDemandUntilRequested)
{
	DemandScheduler scheduler;
	(void)scheduler.RegisterOneShotConsumer("python");

	const auto idle_plan = scheduler.Evaluate(std::chrono::milliseconds(1), CameraId(1));
	EXPECT_TRUE(idle_plan.consumers.empty());
}

TEST(RenderDemand, CadencedPythonBufferIsDueEverySimulationStep)
{
	DemandScheduler scheduler;
	const auto python = scheduler.RegisterCadencedConsumer("python-buffer", kPythonAsyncBufferCadence, CameraId(1));

	const auto first_plan = scheduler.Evaluate(std::chrono::nanoseconds(1), CameraId(1));
	ASSERT_EQ(first_plan.consumers.size(), 1U);
	EXPECT_EQ(first_plan.consumers.front(), python);
	scheduler.MarkDelivered(first_plan, python);

	const auto between_plan = scheduler.Evaluate(std::chrono::nanoseconds(1), CameraId(1));
	EXPECT_TRUE(between_plan.consumers.empty());

	const auto second_plan = scheduler.Evaluate(std::chrono::nanoseconds(2), CameraId(1));
	ASSERT_EQ(second_plan.consumers.size(), 1U);
	EXPECT_EQ(second_plan.consumers.front(), python);
}

TEST(RenderDemand, UnregisterRemovesOnlyTheSelectedConsumer)
{
	DemandScheduler scheduler;
	const auto ros    = scheduler.RegisterContinuousConsumer("ros");
	const auto python = scheduler.RegisterContinuousConsumer("python");

	scheduler.UnregisterConsumer(python);
	const auto plan = scheduler.Evaluate(std::chrono::milliseconds(1), CameraId(1));

	ASSERT_EQ(plan.consumers.size(), 1U);
	EXPECT_EQ(plan.consumers.front(), ros);
	EXPECT_THROW(scheduler.SetEnabled(python, false), std::invalid_argument);
}

} // namespace mujoco_ros::rendering
