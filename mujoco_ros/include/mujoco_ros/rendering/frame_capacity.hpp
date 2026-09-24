#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <mujoco_ros/rendering/camera_descriptor.hpp>

namespace mujoco_ros::rendering {

constexpr std::size_t kMaxPlanesPerCamera    = 3U;
constexpr std::size_t kMaxPythonHistoryDepth = 8U;
// Async Python buffers (history_depth > 1) use an explicit per-step cadence independent from ROS.
inline constexpr std::chrono::nanoseconds kPythonAsyncBufferCadence{ 1 };
constexpr std::size_t kBootstrapCameraCapacity = 16U;
// Each warmed slot preallocates one pool-storage vector and one writer buffer.
constexpr std::size_t kWarmupByteBuffersPerSlot = 2U;

class FrameCapacityOverflow : public std::runtime_error
{
public:
	explicit FrameCapacityOverflow(const std::string &message) : std::runtime_error(message) {}
};

struct CameraHistoryDepth
{
	CameraId id       = 0;
	std::size_t depth = 1;
};

inline bool CheckedMultiply(std::size_t lhs, std::size_t rhs, std::size_t &product)
{
	if (lhs != 0 && rhs > std::numeric_limits<std::size_t>::max() / lhs) {
		return false;
	}
	product = lhs * rhs;
	return true;
}

inline bool CheckedAdd(std::size_t lhs, std::size_t rhs, std::size_t &sum)
{
	if (rhs > std::numeric_limits<std::size_t>::max() - lhs) {
		return false;
	}
	sum = lhs + rhs;
	return true;
}

inline std::size_t CountEnabledPlanes(PlaneMask planes)
{
	std::size_t count = 0;
	for (const auto plane : { PlaneKind::kRgb, PlaneKind::kDepth, PlaneKind::kSegmentation }) {
		if (HasPlane(planes, plane)) {
			++count;
		}
	}
	return count;
}

inline std::size_t HistoryDepthForCamera(CameraId camera, const std::vector<CameraHistoryDepth> &histories)
{
	std::size_t depth = 1;
	for (const auto &entry : histories) {
		if (entry.id == camera) {
			depth = std::max(depth, entry.depth);
		}
	}
	return depth;
}

inline std::size_t ComputeFrameSlotCapacity(const std::vector<CameraDescriptor> &cameras,
                                            const std::vector<CameraHistoryDepth> &histories = {})
{
	std::size_t total = 0;
	for (const auto &camera : cameras) {
		const std::size_t planes     = CountEnabledPlanes(camera.planes);
		const std::size_t history    = HistoryDepthForCamera(camera.id, histories);
		std::size_t slots_for_camera = 0;
		if (!CheckedMultiply(planes, history, slots_for_camera)) {
			throw FrameCapacityOverflow("frame slot capacity multiplication overflow");
		}
		std::size_t next_total = 0;
		if (!CheckedAdd(total, slots_for_camera, next_total)) {
			throw FrameCapacityOverflow("frame slot capacity addition overflow");
		}
		total = next_total;
	}
	return total;
}

inline std::size_t LargestEnabledPlaneBytes(const CameraDescriptor &camera)
{
	std::size_t largest = 0;
	for (const auto plane : { PlaneKind::kRgb, PlaneKind::kDepth, PlaneKind::kSegmentation }) {
		if (HasPlane(camera.planes, plane)) {
			largest = std::max(largest, camera.layout(plane).byte_length);
		}
	}
	return largest;
}

inline std::size_t ComputeFrameByteCapacity(const std::vector<CameraDescriptor> &cameras,
                                            const std::vector<CameraHistoryDepth> &histories = {})
{
	std::size_t total = 0;
	for (const auto &camera : cameras) {
		const std::size_t history = HistoryDepthForCamera(camera.id, histories);
		for (const auto plane : { PlaneKind::kRgb, PlaneKind::kDepth, PlaneKind::kSegmentation }) {
			if (!HasPlane(camera.planes, plane)) {
				continue;
			}
			std::size_t plane_bytes = 0;
			if (!CheckedMultiply(camera.layout(plane).byte_length, history, plane_bytes)) {
				throw FrameCapacityOverflow("frame byte capacity multiplication overflow");
			}
			std::size_t next_total = 0;
			if (!CheckedAdd(total, plane_bytes, next_total)) {
				throw FrameCapacityOverflow("frame byte capacity addition overflow");
			}
			total = next_total;
		}
	}
	return total;
}

inline std::size_t ComputeWarmSlotCount(std::size_t max_slots, std::size_t max_bytes, std::size_t warm_byte_length,
                                        std::size_t required_slots)
{
	if (warm_byte_length == 0 || required_slots == 0) {
		return 0;
	}
	std::size_t bytes_per_warmed_slot = 0;
	if (!CheckedMultiply(warm_byte_length, kWarmupByteBuffersPerSlot, bytes_per_warmed_slot)) {
		return 0;
	}
	const std::size_t byte_limited = max_bytes / bytes_per_warmed_slot;
	return std::min({ max_slots, required_slots, byte_limited });
}

inline void ValidatePythonHistoryDepth(std::size_t history_depth)
{
	if (history_depth == 0 || history_depth > kMaxPythonHistoryDepth) {
		throw std::runtime_error("Python frame history depth " + std::to_string(history_depth) +
		                         " exceeds supported cap of " + std::to_string(kMaxPythonHistoryDepth));
	}
}

constexpr std::size_t kBootstrapFrameSlotCapacity =
    kBootstrapCameraCapacity * kMaxPlanesPerCamera * kMaxPythonHistoryDepth;

inline std::size_t BootstrapFrameSlotCapacity()
{
	return kBootstrapFrameSlotCapacity;
}

} // namespace mujoco_ros::rendering
