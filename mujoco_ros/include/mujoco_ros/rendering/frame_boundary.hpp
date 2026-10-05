#pragma once

#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <mujoco_ros/generation.hpp>
#include <mujoco_ros/rendering/camera_descriptor.hpp>

namespace mujoco_ros::rendering {

enum class FrameStatusCode
{
	kOk,
	kFrameSlotsExhausted,
	kBusy,
	kGenerationCapacityExhausted,
	kInvalidLayout,
	kNoFrame,
	kStaleGeneration,
	kFrameUnavailable,
	kSnapshotPoolExhausted,
	kBackendUnavailable,
	kNotInitialized,
	kBackendFailure,
	kStopped,
	kInvalidPolicy,
	kTerminalError,
};

struct FrameStamp
{
	std::uint64_t capture_id     = 0;
	CameraId camera_id           = 0;
	std::uint64_t plane_sequence = 0;
	ModelGeneration model_generation;
	FrameGeneration generation;
	std::int64_t simulation_time_ns = 0;
};

struct FrameStatus
{
	FrameStatusCode code     = FrameStatusCode::kOk;
	std::uint64_t capture_id = 0;
	std::optional<PlaneKind> plane;
	FrameGeneration generation;
	std::string message;

	bool ok() const { return code == FrameStatusCode::kOk; }
	static FrameStatus Ok() { return FrameStatus(); }
};

inline bool IsContextIntegrityFailure(FrameStatusCode code)
{
	switch (code) {
		case FrameStatusCode::kBackendFailure:
		case FrameStatusCode::kBackendUnavailable:
		case FrameStatusCode::kTerminalError:
			return true;
		default:
			return false;
	}
}

inline bool IsContextIntegrityFailure(const FrameStatus &status)
{
	return IsContextIntegrityFailure(status.code);
}

inline bool ShouldRecordPublicationStatus(const FrameStatus &completed, const FrameStatus &published)
{
	return completed.ok() || !published.ok();
}

enum class RenderBackpressurePolicy
{
	kDrop,
	kWaitForSlot,
};

std::optional<RenderBackpressurePolicy> RenderBackpressurePolicyFromString(const std::string &value);
std::string RenderBackpressurePolicyToString(RenderBackpressurePolicy policy);
inline std::string ToString(RenderBackpressurePolicy policy)
{
	return RenderBackpressurePolicyToString(policy);
}

class FrameBoundary;

class FrameLease
{
public:
	struct Storage;
	FrameLease() = default;
	FrameLease(FrameLease &&other) noexcept;
	FrameLease &operator=(FrameLease &&other) noexcept;
	~FrameLease();
	FrameLease(const FrameLease &)            = delete;
	FrameLease &operator=(const FrameLease &) = delete;

	bool valid() const { return storage_ != nullptr; }
	const std::vector<std::byte> &bytes() const;
	const PlaneLayout &layout() const;
	PlaneKind plane() const;
	FrameGeneration generation() const;
	ModelGeneration model_generation() const;
	std::uint64_t capture_id() const;
	CameraId camera_id() const;
	std::uint64_t plane_sequence() const;
	std::int64_t simulation_time_ns() const;

private:
	explicit FrameLease(std::shared_ptr<const Storage> storage, std::function<void()> notifier)
	    : storage_(std::move(storage)), notifier_(std::move(notifier))
	{
	}
	std::shared_ptr<const Storage> storage_;
	std::function<void()> notifier_;
	friend class FrameBoundary;
	friend class FrameWriter;
};

class FrameWriter
{
public:
	struct State;
	FrameWriter() = default;
	FrameWriter(FrameWriter &&other) noexcept;
	FrameWriter &operator=(FrameWriter &&other) noexcept;
	FrameWriter(const FrameWriter &)            = delete;
	FrameWriter &operator=(const FrameWriter &) = delete;
	~FrameWriter();

	const FrameStatus &status() const;
	std::vector<std::byte> &bytes();
	FrameStatus Commit();
	std::optional<FrameLease> Acquire() const;

private:
	explicit FrameWriter(std::shared_ptr<State> state) : state_(std::move(state)) {}
	std::shared_ptr<State> state_;
	friend class FrameBoundary;
};

class FrameBoundary
{
public:
	struct State;
	static constexpr std::size_t kDefaultMaxBytes = 64U * 1024U * 1024U;
	explicit FrameBoundary(std::size_t max_slots, std::size_t max_bytes = kDefaultMaxBytes);
	~FrameBoundary();

	FrameBoundary(const FrameBoundary &)            = delete;
	FrameBoundary &operator=(const FrameBoundary &) = delete;

	FrameStatus Reconfigure(FrameGeneration generation, const FrameLayout &layout, std::size_t warm_slot_count = 0);
	void WarmUpSlots(const FrameLayout &layout, std::size_t warm_slot_count = 0);
	FrameStamp BeginCapture(ModelGeneration model_generation, std::int64_t simulation_time_ns,
	                        CameraId camera = CameraId(0));
	FrameWriter TryAcquireWriter(FrameGeneration generation, PlaneKind plane, const PlaneLayout &layout);
	FrameWriter TryAcquireWriter(FrameGeneration generation, const FrameStamp &stamp, PlaneKind plane,
	                             const PlaneLayout &layout);
	FrameStatus WaitForCapacity(FrameGeneration generation, std::size_t required_slots, std::size_t required_bytes,
	                            std::function<bool()> cancelled = {});
	void NotifyCapacityWaiters();
#ifdef MJR_BUILD_TESTING
	void SetCapacityWaitObserverForTesting(std::function<void()> observer);
#endif
	std::optional<FrameLease> AcquireLatest(PlaneKind plane) const;
	std::optional<FrameLease> AcquireLatest(CameraId camera, PlaneKind plane) const;
	std::optional<FrameLease> AcquireLatest(std::uint64_t capture_id, PlaneKind plane) const;
	std::optional<FrameLease> AcquireLatest(std::uint64_t capture_id, CameraId camera, PlaneKind plane) const;
	std::vector<FrameLease> AcquireRecent(CameraId camera, PlaneKind plane, std::size_t max_count) const;
	FrameGeneration generation() const;
	std::size_t max_slots() const { return max_slots_; }
	std::size_t max_bytes() const { return max_bytes_; }
#ifdef MJR_BUILD_TESTING
	std::size_t storage_allocation_count() const;
	std::size_t byte_vector_growth_count() const;
	std::size_t warmed_slot_count() const;
	std::size_t warmed_reserved_bytes() const;
#endif

private:
	std::shared_ptr<State> state_;
	std::size_t max_slots_;
	std::size_t max_bytes_;
};

} // namespace mujoco_ros::rendering
