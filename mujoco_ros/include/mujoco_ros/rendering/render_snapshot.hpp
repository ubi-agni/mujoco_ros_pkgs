#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include <mujoco_ros/generation.hpp>

namespace mujoco_ros::rendering {

struct RenderSnapshot
{
	ModelGeneration model_generation;
	std::int64_t simulation_time_ns = 0;
	std::shared_ptr<const mjModel> model;
	std::shared_ptr<mjData> data;
	std::shared_ptr<const std::vector<mjvGeom>> plugin_geometry;

	bool valid() const { return model != nullptr && data != nullptr; }
};

class SnapshotPool
{
public:
	// Two slots bound simultaneous owned captures without allowing unbounded allocator growth.
	static constexpr std::size_t kCapacity = 2;

	enum class AcquireCode
	{
		kOk,
		kInactiveGeneration,
		kExhausted,
	};

	struct AcquireResult
	{
		std::shared_ptr<mjData> data;
		AcquireCode code = AcquireCode::kInactiveGeneration;
		std::string message;

		bool ok() const { return code == AcquireCode::kOk && data != nullptr; }
	};

	SnapshotPool()                                = default;
	SnapshotPool(const SnapshotPool &)            = delete;
	SnapshotPool &operator=(const SnapshotPool &) = delete;

	void Activate(const mjModel &model, ModelGeneration generation);
	void Deactivate();
	AcquireResult Acquire(const mjModel &model, const mjData &source, ModelGeneration generation);

	std::size_t allocation_count() const;
	std::size_t copy_count() const;
	std::size_t active_lease_count() const;
	std::optional<ModelGeneration> active_generation() const;
	static std::size_t make_data_operations();
	static std::size_t delete_data_operations();
#ifdef MJR_BUILD_TESTING
	static void RecordMakeDataOperationForTest();
	static void RecordDeleteDataOperationForTest();
	static void ThrowAfterDataAllocationForTest();
#endif

private:
	struct GenerationState;

	mutable std::mutex mutex_;
	std::shared_ptr<GenerationState> active_;
};

std::shared_ptr<const mjModel> CopyModel(const mjModel &source);

namespace detail {
inline constexpr bool kExportsCopyDataHelper = false;
} // namespace detail

} // namespace mujoco_ros::rendering
