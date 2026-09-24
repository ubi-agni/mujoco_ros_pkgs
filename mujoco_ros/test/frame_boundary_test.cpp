#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <limits>
#include <thread>

#include <mujoco_ros/rendering/frame_boundary.hpp>
#include <mujoco_ros/rendering/frame_capacity.hpp>

namespace mujoco_ros::rendering {
namespace {

FrameLayout Layout(int width, int height)
{
	return FrameLayout{ width, height };
}

FrameLease CommitRgb(FrameBoundary &boundary, FrameGeneration generation, int width, int height, std::byte value)
{
	auto writer = boundary.TryAcquireWriter(generation, PlaneKind::kRgb, PlaneLayout::Rgb8(width, height));
	EXPECT_TRUE(writer.status().ok()) << writer.status().message;
	std::fill(writer.bytes().begin(), writer.bytes().end(), value);
	const auto committed = writer.Commit();
	EXPECT_TRUE(committed.ok()) << committed.message;
	auto lease = writer.Acquire();
	EXPECT_TRUE(lease.has_value());
	if (!lease.has_value()) {
		return FrameLease{};
	}
	return std::move(*lease);
}

} // namespace

TEST(FrameBoundary, FirstCommitUsesPreallocatedPoolSlot)
{
	FrameBoundary boundary(2);
	EXPECT_EQ(boundary.storage_allocation_count(), 2U);
	auto writer = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kRgb, PlaneLayout::Rgb8(2, 2));
	ASSERT_TRUE(writer.status().ok()) << writer.status().message;
	EXPECT_EQ(writer.bytes().size(), PlaneLayout::Rgb8(2, 2).byte_length);
	const auto committed = writer.Commit();
	ASSERT_TRUE(committed.ok()) << committed.message;
	EXPECT_EQ(boundary.storage_allocation_count(), 2U);
}

TEST(FrameBoundary, LargerPlaneLayoutCommitsWithinByteBudget)
{
	FrameBoundary boundary(2);
	const auto layout = PlaneLayout::Rgb8(4, 3);
	auto writer       = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kRgb, layout);
	ASSERT_TRUE(writer.status().ok()) << writer.status().message;
	EXPECT_EQ(writer.bytes().size(), layout.byte_length);
	const auto committed = writer.Commit();
	ASSERT_TRUE(committed.ok()) << committed.message;
}

TEST(FrameBoundary, ResizePreservesOldLeaseAndMovesNewAcquisitions)
{
	FrameBoundary boundary(2);
	const auto layout = PlaneLayout::Rgb8(4, 3);
	auto writer       = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kRgb, layout);
	ASSERT_TRUE(writer.status().ok()) << writer.status().message;
	std::fill(writer.bytes().begin(), writer.bytes().end(), std::byte{ 0x11 });
	const auto committed = writer.Commit();
	ASSERT_TRUE(committed.ok()) << committed.message;
	auto lease = writer.Acquire();
	ASSERT_TRUE(lease.has_value());
	auto old = std::move(*lease);

	auto reconfigure = boundary.Reconfigure(FrameGeneration(2), Layout(8, 6));
	ASSERT_TRUE(reconfigure.ok()) << reconfigure.message;

	auto current = CommitRgb(boundary, FrameGeneration(2), 8, 6, std::byte{ 0x22 });

	EXPECT_EQ(old.generation(), FrameGeneration(1));
	EXPECT_EQ(old.bytes()[0], std::byte{ 0x11 });
	EXPECT_EQ(current.generation(), FrameGeneration(2));
	EXPECT_EQ(current.bytes()[0], std::byte{ 0x22 });
}

TEST(FrameBoundary, SlotExhaustionIsExplicitAndLeasesAreMoveOnly)
{
	FrameBoundary boundary(1);
	auto first        = CommitRgb(boundary, FrameGeneration(1), 2, 2, std::byte{ 0x01 });
	const auto second = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kDepth, PlaneLayout::Depth32F(2, 2));
	EXPECT_EQ(second.status().code, FrameStatusCode::kFrameSlotsExhausted);

	FrameLease moved = std::move(first);
	EXPECT_TRUE(moved.valid());
}

TEST(FrameBoundary, RenderBackpressurePolicyUsesCanonicalValues)
{
	EXPECT_EQ(RenderBackpressurePolicyFromString("drop"), RenderBackpressurePolicy::kDrop);
	EXPECT_EQ(RenderBackpressurePolicyFromString("wait_for_slot"), RenderBackpressurePolicy::kWaitForSlot);
	EXPECT_FALSE(RenderBackpressurePolicyFromString("wait"));
	EXPECT_EQ(ToString(RenderBackpressurePolicy::kDrop), "drop");
	EXPECT_EQ(ToString(RenderBackpressurePolicy::kWaitForSlot), "wait_for_slot");
}

TEST(FrameBoundary, CapacityWaitUnblocksAfterLeaseRelease)
{
	FrameBoundary boundary(1);
	auto retained = CommitRgb(boundary, FrameGeneration(1), 2, 2, std::byte{ 0x01 });
	std::promise<void> entered;
	auto entered_future = entered.get_future();
	boundary.SetCapacityWaitObserverForTesting([&entered]() { entered.set_value(); });
	std::promise<FrameStatus> result;
	auto result_future = result.get_future();
	std::thread waiter(
	    [&] { result.set_value(boundary.WaitForCapacity(FrameGeneration(1), 1, PlaneLayout::Rgb8(2, 2).byte_length)); });

	ASSERT_EQ(entered_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	std::promise<void> released;
	auto released_future = released.get_future();
	retained             = FrameLease{};
	released.set_value();
	ASSERT_EQ(released_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	ASSERT_EQ(result_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	EXPECT_TRUE(result_future.get().ok());
	waiter.join();
}

TEST(FrameBoundary, CapacityWaitReturnsExplicitCancellation)
{
	FrameBoundary boundary(1);
	auto retained = CommitRgb(boundary, FrameGeneration(1), 2, 2, std::byte{ 0x01 });
	std::atomic_bool cancelled{ false };
	std::promise<void> entered;
	auto entered_future = entered.get_future();
	boundary.SetCapacityWaitObserverForTesting([&entered]() { entered.set_value(); });
	std::promise<FrameStatus> result;
	auto result_future = result.get_future();
	std::thread waiter([&] {
		result.set_value(boundary.WaitForCapacity(FrameGeneration(1), 1, PlaneLayout::Rgb8(2, 2).byte_length,
		                                          [&cancelled]() { return cancelled.load(std::memory_order_acquire); }));
	});

	ASSERT_EQ(entered_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	cancelled.store(true, std::memory_order_release);
	boundary.NotifyCapacityWaiters();
	ASSERT_EQ(result_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	EXPECT_EQ(result_future.get().code, FrameStatusCode::kStopped);
	waiter.join();
}

TEST(FrameBoundary, DestructionStopsWaitUsingRetainedState)
{
	auto boundary = std::make_shared<FrameBoundary>(1);
	auto retained = CommitRgb(*boundary, FrameGeneration(1), 2, 2, std::byte{ 0x01 });
	std::promise<void> entered;
	auto entered_future = entered.get_future();
	boundary->SetCapacityWaitObserverForTesting([&entered]() { entered.set_value(); });
	auto *raw_boundary = boundary.get();
	std::promise<FrameStatus> result;
	auto result_future = result.get_future();
	std::thread waiter([raw_boundary, &result] {
		result.set_value(raw_boundary->WaitForCapacity(FrameGeneration(1), 1, PlaneLayout::Rgb8(2, 2).byte_length));
	});

	ASSERT_EQ(entered_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	boundary.reset();
	ASSERT_EQ(result_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	EXPECT_EQ(result_future.get().code, FrameStatusCode::kStopped);
	waiter.join();
}

TEST(FrameBoundary, CapacityWaitDoesNotEvictUnleasedRecords)
{
	FrameBoundary boundary(1);
	auto committed = CommitRgb(boundary, FrameGeneration(1), 2, 2, std::byte{ 0x01 });
	committed      = FrameLease{};

	const auto waited = boundary.WaitForCapacity(FrameGeneration(1), 1, PlaneLayout::Rgb8(2, 2).byte_length);
	EXPECT_TRUE(waited.ok());
	auto latest = boundary.AcquireLatest(PlaneKind::kRgb);
	ASSERT_TRUE(latest.has_value());
	EXPECT_EQ(latest->bytes().front(), std::byte{ 0x01 });
}

TEST(FrameBoundary, CapacityWaitRejectsImpossibleRequirements)
{
	FrameBoundary boundary(1, 16);

	const auto waited = boundary.WaitForCapacity(FrameGeneration(1), 2, 1);
	EXPECT_EQ(waited.code, FrameStatusCode::kFrameSlotsExhausted);
	const auto oversized = boundary.WaitForCapacity(FrameGeneration(1), 1, 17);
	EXPECT_EQ(oversized.code, FrameStatusCode::kFrameSlotsExhausted);
}

TEST(FrameBoundary, CapacityWaitAccountsForOutstandingReservations)
{
	FrameBoundary boundary(1, 16);
	auto writer = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kRgb, PlaneLayout::Rgb8(2, 2));
	ASSERT_TRUE(writer.status().ok());

	std::promise<void> entered;
	auto entered_future = entered.get_future();
	boundary.SetCapacityWaitObserverForTesting([&entered]() { entered.set_value(); });
	std::promise<FrameStatus> result;
	auto result_future = result.get_future();
	std::thread waiter(
	    [&] { result.set_value(boundary.WaitForCapacity(FrameGeneration(1), 1, PlaneLayout::Rgb8(2, 2).byte_length)); });

	ASSERT_EQ(entered_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	writer = FrameWriter{};
	boundary.NotifyCapacityWaiters();
	ASSERT_EQ(result_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	EXPECT_TRUE(result_future.get().ok());
	waiter.join();
}

TEST(FrameBoundary, ByteExhaustionIncludesCommittedStorageHeldByLease)
{
	FrameBoundary boundary(2, 100);
	const PlaneLayout ninety_bytes{ 30, 1, 90, 90 };
	auto first = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kRgb, ninety_bytes);
	ASSERT_TRUE(first.status().ok()) << first.status().message;
	ASSERT_TRUE(first.Commit().ok());
	auto lease = first.Acquire();
	ASSERT_TRUE(lease.has_value());

	const auto second = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kRgb, ninety_bytes);

	EXPECT_EQ(second.status().code, FrameStatusCode::kFrameSlotsExhausted);
	EXPECT_EQ(second.status().generation, FrameGeneration(1));
	ASSERT_TRUE(second.status().plane.has_value());
	EXPECT_EQ(*second.status().plane, PlaneKind::kRgb);
	EXPECT_NE(second.status().message.find("byte budget"), std::string::npos);
}

TEST(FrameBoundary, IndependentPlanesHaveIndependentSequences)
{
	FrameBoundary boundary(3);
	ASSERT_TRUE(boundary.Reconfigure(FrameGeneration(1), Layout(2, 2)).ok());
	boundary.BeginCapture(ModelGeneration(1), 0);
	auto rgb_writer   = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kRgb, PlaneLayout::Rgb8(2, 2));
	auto depth_writer = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kDepth, PlaneLayout::Depth32F(2, 2));
	ASSERT_TRUE(rgb_writer.Commit().ok());
	ASSERT_TRUE(depth_writer.Commit().ok());

	auto rgb   = boundary.AcquireLatest(PlaneKind::kRgb);
	auto depth = boundary.AcquireLatest(PlaneKind::kDepth);
	ASSERT_TRUE(rgb.has_value());
	ASSERT_TRUE(depth.has_value());
	EXPECT_EQ(rgb->plane_sequence(), 1U);
	EXPECT_EQ(depth->plane_sequence(), 1U);
	EXPECT_EQ(rgb->capture_id(), depth->capture_id());

	boundary.BeginCapture(ModelGeneration(1), 1);
	auto next_rgb = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kRgb, PlaneLayout::Rgb8(2, 2));
	ASSERT_TRUE(next_rgb.Commit().ok());
	const auto next_rgb_lease = next_rgb.Acquire();
	ASSERT_TRUE(next_rgb_lease.has_value());
	EXPECT_EQ(next_rgb_lease->plane_sequence(), 2U);
}

TEST(FrameBoundary, RecentCameraLeasesAreReturnedChronologically)
{
	FrameBoundary boundary(6);
	ASSERT_TRUE(boundary.Reconfigure(FrameGeneration(1), Layout(2, 2)).ok());

	for (std::byte value : { std::byte{ 0x01 }, std::byte{ 0x02 }, std::byte{ 0x03 } }) {
		const auto stamp = boundary.BeginCapture(ModelGeneration(1), static_cast<int>(value), CameraId(7));
		auto writer      = boundary.TryAcquireWriter(FrameGeneration(1), stamp, PlaneKind::kRgb, PlaneLayout::Rgb8(2, 2));
		ASSERT_TRUE(writer.status().ok());
		std::fill(writer.bytes().begin(), writer.bytes().end(), value);
		ASSERT_TRUE(writer.Commit().ok());
	}

	const auto recent = boundary.AcquireRecent(CameraId(7), PlaneKind::kRgb, 2);
	ASSERT_EQ(recent.size(), 2U);
	EXPECT_EQ(recent[0].bytes().front(), std::byte{ 0x02 });
	EXPECT_EQ(recent[1].bytes().front(), std::byte{ 0x03 });
}

TEST(FrameBoundary, BoundedHistoryEvictsOldestUnleasedRecord)
{
	FrameBoundary boundary(2);
	ASSERT_TRUE(boundary.Reconfigure(FrameGeneration(1), Layout(2, 2)).ok());

	for (std::byte value : { std::byte{ 0x11 }, std::byte{ 0x22 } }) {
		auto writer = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kRgb, PlaneLayout::Rgb8(2, 2));
		ASSERT_TRUE(writer.status().ok());
		std::fill(writer.bytes().begin(), writer.bytes().end(), value);
		ASSERT_TRUE(writer.Commit().ok());
	}

	auto newest_writer = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kRgb, PlaneLayout::Rgb8(2, 2));
	ASSERT_TRUE(newest_writer.status().ok());
	std::fill(newest_writer.bytes().begin(), newest_writer.bytes().end(), std::byte{ 0x33 });
	ASSERT_TRUE(newest_writer.Commit().ok());

	const auto recent = boundary.AcquireRecent(CameraId(0), PlaneKind::kRgb, 3);
	ASSERT_EQ(recent.size(), 2U);
	EXPECT_EQ(recent[0].bytes().front(), std::byte{ 0x22 });
	EXPECT_EQ(recent[1].bytes().front(), std::byte{ 0x33 });
}

TEST(FrameBoundary, ReconfigureRejectsCapacityNeededByRetiredLease)
{
	const auto old_layout = PlaneLayout::Rgb8(4, 4);
	FrameBoundary boundary(2, old_layout.byte_length + PlaneLayout::Rgb8(8, 8).byte_length - 1);
	auto old = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kRgb, old_layout);
	ASSERT_TRUE(old.Commit().ok());
	auto lease = old.Acquire();
	ASSERT_TRUE(lease.has_value());

	const auto result = boundary.Reconfigure(FrameGeneration(2), Layout(8, 8));
	EXPECT_EQ(result.code, FrameStatusCode::kGenerationCapacityExhausted);
}

TEST(FrameBoundary, WriterCannotCommitAfterGenerationRollover)
{
	FrameBoundary boundary(2);
	ASSERT_TRUE(boundary.Reconfigure(FrameGeneration(1), Layout(2, 2)).ok());
	auto writer = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kRgb, PlaneLayout::Rgb8(2, 2));
	ASSERT_TRUE(writer.status().ok());
	ASSERT_TRUE(boundary.Reconfigure(FrameGeneration(2), Layout(2, 2)).ok());

	EXPECT_EQ(writer.Commit().code, FrameStatusCode::kStaleGeneration);
	EXPECT_FALSE(boundary.AcquireLatest(PlaneKind::kRgb).has_value());
}

TEST(FrameBoundary, NewAcquisitionsIgnoreRetiredGenerationRecords)
{
	FrameBoundary boundary(3);
	ASSERT_TRUE(boundary.Reconfigure(FrameGeneration(1), Layout(2, 2)).ok());
	const auto old = CommitRgb(boundary, FrameGeneration(1), 2, 2, std::byte{ 0x11 });

	ASSERT_TRUE(boundary.Reconfigure(FrameGeneration(2), Layout(2, 2)).ok());
	EXPECT_FALSE(boundary.AcquireLatest(PlaneKind::kRgb).has_value());
	EXPECT_TRUE(old.valid());
}

TEST(FrameBoundary, NoHotPathAllocationAfterWarmup)
{
	FrameBoundary boundary(8);
	EXPECT_EQ(boundary.storage_allocation_count(), 8U);
	const auto layout = Layout(4, 4);
	ASSERT_TRUE(boundary.Reconfigure(FrameGeneration(1), layout, 8).ok());
	const auto storage_before = boundary.storage_allocation_count();
	const auto growth_before  = boundary.byte_vector_growth_count();
	for (int capture = 0; capture < 16; ++capture) {
		for (const auto plane : { PlaneKind::kRgb, PlaneKind::kDepth }) {
			const auto plane_layout = plane == PlaneKind::kDepth ? PlaneLayout::Depth32F(4, 4) : PlaneLayout::Rgb8(4, 4);
			auto writer             = boundary.TryAcquireWriter(FrameGeneration(1), plane, plane_layout);
			ASSERT_TRUE(writer.status().ok()) << writer.status().message;
			ASSERT_TRUE(writer.Commit().ok());
		}
	}
	EXPECT_EQ(boundary.storage_allocation_count(), storage_before);
	EXPECT_EQ(boundary.byte_vector_growth_count(), growth_before);
}

TEST(FrameBoundary, WarmupRespectsMaxBytesBudget)
{
	constexpr int width    = 100;
	constexpr int height   = 100;
	const auto plane_bytes = FrameLayout::LargestPlaneByteLength(width, height);
	const auto max_bytes   = plane_bytes * 4 * kWarmupByteBuffersPerSlot;
	FrameBoundary boundary(384, max_bytes);
	const FrameLayout layout(width, height, plane_bytes);
	ASSERT_TRUE(boundary.Reconfigure(FrameGeneration(1), layout, 4).ok());
	EXPECT_EQ(boundary.warmed_slot_count(), 4U);
	EXPECT_LE(boundary.warmed_reserved_bytes(), max_bytes);
}

TEST(FrameCapacity, ComputeFrameByteCapacityRejectsOverflow)
{
	CameraDescriptor camera;
	camera.id     = CameraId(1);
	camera.width  = 65535;
	camera.height = 65535;
	camera.planes = PlaneMask::kRgb;
	const std::vector<CameraHistoryDepth> histories{ { camera.id, std::numeric_limits<std::size_t>::max() } };
	EXPECT_THROW((void)ComputeFrameByteCapacity({ camera }, histories), FrameCapacityOverflow);
}

TEST(FrameCapacity, RejectsPythonHistoryAboveFixedCap)
{
	EXPECT_NO_THROW(ValidatePythonHistoryDepth(kMaxPythonHistoryDepth));
	EXPECT_THROW(ValidatePythonHistoryDepth(kMaxPythonHistoryDepth + 1), std::runtime_error);
	EXPECT_THROW(ValidatePythonHistoryDepth(0), std::runtime_error);
}

TEST(FrameBoundary, RgbDepthSegmentationHistoryWithinCapacity)
{
	constexpr std::size_t history = 3;
	FrameBoundary boundary(history * 3);
	const FrameLayout layout(2, 2);
	ASSERT_TRUE(boundary.Reconfigure(FrameGeneration(1), layout, history * 3).ok());
	for (std::size_t frame = 0; frame < history; ++frame) {
		for (const auto plane : { PlaneKind::kRgb, PlaneKind::kDepth, PlaneKind::kSegmentation }) {
			const auto layout = plane == PlaneKind::kDepth ? PlaneLayout::Depth32F(2, 2) : PlaneLayout::Rgb8(2, 2);
			auto writer       = boundary.TryAcquireWriter(FrameGeneration(1), plane, layout);
			ASSERT_TRUE(writer.status().ok()) << writer.status().message;
			ASSERT_TRUE(writer.Commit().ok());
		}
	}
	for (const auto plane : { PlaneKind::kRgb, PlaneKind::kDepth, PlaneKind::kSegmentation }) {
		EXPECT_EQ(boundary.AcquireRecent(CameraId(0), plane, history).size(), history);
	}
}

TEST(FrameBoundary, BootstrapCapacitySupportsFourCameraLatestRgb)
{
	std::vector<CameraDescriptor> cameras;
	for (std::uint8_t id = 1; id <= 4; ++id) {
		CameraDescriptor camera;
		camera.id     = CameraId(id);
		camera.width  = 2;
		camera.height = 2;
		camera.planes = PlaneMask::kRgb;
		cameras.push_back(camera);
	}
	EXPECT_GE(ComputeFrameSlotCapacity(cameras), 4U);
	EXPECT_GE(BootstrapFrameSlotCapacity(), ComputeFrameSlotCapacity(cameras));
}

} // namespace mujoco_ros::rendering
