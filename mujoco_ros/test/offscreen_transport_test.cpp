#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mujoco_ros/generation.hpp>
#include <mujoco_ros/offscreen_transport.hpp>
#include <mujoco_ros/rendering/bounded_publication_queue.hpp>
#include <mujoco_ros/rendering/frame_boundary.hpp>
#include <mujoco_ros/rendering/frame_capacity.hpp>

namespace mujoco_ros {
namespace {

void ExpectHistoryDepthRejection(const std::function<void()> &action)
{
	try {
		action();
		FAIL() << "expected Python history-depth rejection";
	} catch (const std::runtime_error &error) {
		const std::string message = error.what();
		EXPECT_NE(message.find("history depth"), std::string::npos);
		EXPECT_NE(message.find("exceeds supported cap"), std::string::npos);
		EXPECT_EQ(message.find("RenderCore"), std::string::npos);
	}
}

void ExpectMissingCoreRejection(const std::function<void()> &action)
{
	try {
		action();
		FAIL() << "expected missing RenderCore rejection";
	} catch (const std::runtime_error &error) {
		const std::string message = error.what();
		EXPECT_NE(message.find("RenderCore"), std::string::npos);
		EXPECT_EQ(message.find("history depth"), std::string::npos);
	}
}

struct TestPublicationItem
{
	std::uint64_t capture_id = 0;
};

TEST(OffscreenTransport, HistoryValidationPrecedesMissingCoreCheck)
{
	ExpectHistoryDepthRejection([&] {
		CameraPublicationTransport::ValidatePythonConsumerRequest(rendering::kMaxPythonHistoryDepth + 1, false);
	});
	ExpectMissingCoreRejection([&] { CameraPublicationTransport::ValidatePythonConsumerRequest(1, false); });
}

TEST(BoundedPublicationQueue, ShutdownRejectsEnqueueWithEnqueuedFalse)
{
	rendering::BoundedPublicationQueue<TestPublicationItem> queue(1, [](TestPublicationItem &) {});
	queue.Shutdown();
	const auto result = queue.Enqueue(TestPublicationItem{ 1 });
	EXPECT_FALSE(result.enqueued);
	EXPECT_FALSE(result.dropped_older);
}

TEST(BoundedPublicationQueue, DeferredNotifyWakesWorkerAfterExplicitSignal)
{
	std::atomic_bool worker_started{ false };
	rendering::BoundedPublicationQueue<TestPublicationItem> queue(
	    1, [&](TestPublicationItem &) { worker_started.store(true, std::memory_order_release); });

	EXPECT_TRUE(queue.Enqueue(TestPublicationItem{ 42 }, false).enqueued);
	EXPECT_FALSE(worker_started.load(std::memory_order_acquire));
	queue.NotifyWorker();
	queue.WaitUntilWorkerIdle();
	queue.Shutdown();
	EXPECT_TRUE(worker_started.load(std::memory_order_acquire));
}

TEST(BoundedPublicationQueue, LatestWinsReplacesOldestCapture)
{
	std::vector<std::uint64_t> published_capture_ids;
	rendering::BoundedPublicationQueue<TestPublicationItem> queue(
	    rendering::kRosPublicationQueueDepth,
	    [&](TestPublicationItem &item) { published_capture_ids.push_back(item.capture_id); });

	queue.SetBlockDequeue(true);
	EXPECT_EQ(queue.Enqueue(TestPublicationItem{ 1 }).dropped_older, false);
	EXPECT_EQ(queue.Enqueue(TestPublicationItem{ 2 }).dropped_older, false);
	const auto third = queue.Enqueue(TestPublicationItem{ 3 });
	EXPECT_TRUE(third.enqueued);
	EXPECT_TRUE(third.dropped_older);
	EXPECT_EQ(third.dropped_capture_id, 1U);
	EXPECT_EQ(queue.dropped_count(), 1U);
	queue.SetBlockDequeue(false);
	queue.WaitUntilWorkerIdle();
	queue.Shutdown();

	ASSERT_EQ(published_capture_ids.size(), 2U);
	EXPECT_EQ(published_capture_ids[0], 2U);
	EXPECT_EQ(published_capture_ids[1], 3U);
}

TEST(BoundedPublicationQueue, CallerDoesNotWaitOnBlockedPublisher)
{
	std::mutex worker_mutex;
	std::condition_variable worker_condition;
	bool worker_entered = false;
	bool release_worker = false;
	std::atomic_bool caller_finished{ false };

	rendering::BoundedPublicationQueue<TestPublicationItem> queue(1, [&](TestPublicationItem &) {
		std::unique_lock<std::mutex> lock(worker_mutex);
		worker_entered = true;
		worker_condition.notify_all();
		worker_condition.wait(lock, [&]() { return release_worker; });
	});

	std::thread caller([&]() {
		(void)queue.Enqueue(TestPublicationItem{ 42 });
		caller_finished.store(true);
	});

	{
		std::unique_lock<std::mutex> lock(worker_mutex);
		ASSERT_TRUE(worker_condition.wait_for(lock, std::chrono::seconds(2), [&]() { return worker_entered; }));
	}
	EXPECT_TRUE(caller_finished.load());

	{
		std::lock_guard<std::mutex> lock(worker_mutex);
		release_worker = true;
	}
	worker_condition.notify_all();
	caller.join();
	queue.Shutdown();
}

TEST(BoundedPublicationQueue, CancelPendingWithoutSleepOrdering)
{
	std::mutex worker_mutex;
	std::condition_variable worker_condition;
	bool worker_entered = false;
	bool release_worker = false;
	std::atomic_int publish_count{ 0 };

	rendering::BoundedPublicationQueue<TestPublicationItem> queue(
	    rendering::kRosPublicationQueueDepth, [&](TestPublicationItem &) {
		    std::unique_lock<std::mutex> lock(worker_mutex);
		    worker_entered = true;
		    worker_condition.notify_all();
		    worker_condition.wait(lock, [&]() { return release_worker; });
		    publish_count.fetch_add(1, std::memory_order_relaxed);
	    });

	ASSERT_TRUE(queue.Enqueue(TestPublicationItem{ 1 }).enqueued);
	{
		std::unique_lock<std::mutex> lock(worker_mutex);
		ASSERT_TRUE(worker_condition.wait_for(lock, std::chrono::seconds(2), [&]() { return worker_entered; }));
	}
	ASSERT_TRUE(queue.Enqueue(TestPublicationItem{ 2 }).enqueued);
	queue.CancelPending();
	EXPECT_EQ(queue.cancelled_count(), 1U);
	EXPECT_EQ(queue.pending_count(), 1U) << "active item remains while pending is cleared";

	{
		std::lock_guard<std::mutex> lock(worker_mutex);
		release_worker = true;
	}
	worker_condition.notify_all();
	queue.WaitUntilWorkerIdle();
	queue.Shutdown();
	EXPECT_EQ(publish_count.load(), 1);
}

TEST(OffscreenTransport, ShouldRecordPublicationStatusPreservesPartialRenderFailure)
{
	const rendering::FrameStatus completed =
	    rendering::FrameStatus{ rendering::FrameStatusCode::kFrameSlotsExhausted, 2, std::nullopt, FrameGeneration(1),
		                         "slot exhausted" };
	const rendering::FrameStatus published_ok{ rendering::FrameStatusCode::kOk, 2, std::nullopt, FrameGeneration(1),
		                                        "" };
	const rendering::FrameStatus published_missing =
	    rendering::FrameStatus{ rendering::FrameStatusCode::kFrameUnavailable, 2, rendering::PlaneKind::kDepth,
		                         FrameGeneration(1), "missing depth" };
	EXPECT_FALSE(rendering::ShouldRecordPublicationStatus(completed, published_ok));
	EXPECT_TRUE(rendering::ShouldRecordPublicationStatus(completed, published_missing));
	EXPECT_TRUE(rendering::ShouldRecordPublicationStatus(rendering::FrameStatus::Ok(), published_ok));
}

TEST(BoundedPublicationQueue, ShutdownUnblocksDequeueWait)
{
	rendering::BoundedPublicationQueue<TestPublicationItem> queue(
	    1, [](TestPublicationItem &) { FAIL() << "blocked dequeue item should not publish"; });
	queue.SetBlockDequeue(true);
	queue.Shutdown();
	SUCCEED();
}

} // namespace
} // namespace mujoco_ros
