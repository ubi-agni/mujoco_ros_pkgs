#include <mujoco_ros/rendering/render_snapshot.hpp>

#include <stdexcept>
#include <utility>

namespace mujoco_ros::rendering {

namespace {
std::atomic<std::size_t> make_data_operations_counter{ 0 };
std::atomic<std::size_t> delete_data_operations_counter{ 0 };
#ifdef MJR_BUILD_TESTING
std::atomic_bool throw_after_data_allocation_for_test{ false };
#endif

struct SnapshotDataDeleter
{
	void operator()(mjData *data) const noexcept
	{
		if (data == nullptr) {
			return;
		}
		delete_data_operations_counter.fetch_add(1, std::memory_order_relaxed);
		mj_deleteData(data);
	}
};

using SnapshotData = std::unique_ptr<mjData, SnapshotDataDeleter>;
} // namespace

struct SnapshotPool::GenerationState
{
	GenerationState(const mjModel &model, ModelGeneration generation) : generation(generation)
	{
		slots.reserve(SnapshotPool::kCapacity);
		for (std::size_t index = 0; index < SnapshotPool::kCapacity; ++index) {
			make_data_operations_counter.fetch_add(1, std::memory_order_relaxed);
			SnapshotData data(mj_makeData(&model));
			if (data == nullptr) {
				throw std::runtime_error("MuJoCo snapshot pool allocation failed");
			}
#ifdef MJR_BUILD_TESTING
			if (throw_after_data_allocation_for_test.exchange(false, std::memory_order_relaxed)) {
				throw std::runtime_error("injected snapshot pool ownership insertion failure");
			}
#endif
			slots.push_back(std::move(data));
			leased.push_back(false);
		}
	}

	~GenerationState() = default;

	ModelGeneration generation;
	std::vector<SnapshotData> slots;
	std::vector<bool> leased;
	mutable std::mutex mutex;
	std::size_t copies = 0;
};

void SnapshotPool::Activate(const mjModel &model, ModelGeneration generation)
{
	auto next = std::make_shared<GenerationState>(model, generation);
	std::lock_guard<std::mutex> lock(mutex_);
	active_ = std::move(next);
}

void SnapshotPool::Deactivate()
{
	std::lock_guard<std::mutex> lock(mutex_);
	active_.reset();
}

SnapshotPool::AcquireResult SnapshotPool::Acquire(const mjModel &model, const mjData &source,
                                                  ModelGeneration generation)
{
	std::shared_ptr<GenerationState> state;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		state = active_;
	}
	if (!state || state->generation != generation) {
		return { nullptr, AcquireCode::kInactiveGeneration,
			      "render snapshot pool has no active slot for model generation " + std::to_string(generation.value()) };
	}

	std::lock_guard<std::mutex> lock(state->mutex);
	for (std::size_t index = 0; index < state->slots.size(); ++index) {
		if (state->leased[index]) {
			continue;
		}
		mj_copyData(state->slots[index].get(), &model, &source);
		++state->copies;
		const auto data_ptr  = state->slots[index].get();
		auto data            = std::shared_ptr<mjData>(data_ptr, [state, index](mjData *) {
         std::lock_guard<std::mutex> release_lock(state->mutex);
         state->leased[index] = false;
      });
		state->leased[index] = true;
		return { std::move(data), AcquireCode::kOk, "" };
	}
	return { nullptr, AcquireCode::kExhausted, "render snapshot pool exhausted; dropping capture without waiting" };
}

std::size_t SnapshotPool::allocation_count() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return active_ ? active_->slots.size() : 0;
}

std::size_t SnapshotPool::copy_count() const
{
	std::shared_ptr<GenerationState> state;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		state = active_;
	}
	if (!state) {
		return 0;
	}
	std::lock_guard<std::mutex> state_lock(state->mutex);
	return state->copies;
}

std::size_t SnapshotPool::active_lease_count() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!active_) {
		return 0;
	}
	std::lock_guard<std::mutex> state_lock(active_->mutex);
	std::size_t count = 0;
	for (const auto leased : active_->leased) {
		count += leased ? 1U : 0U;
	}
	return count;
}

std::optional<ModelGeneration> SnapshotPool::active_generation() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return active_ ? std::optional<ModelGeneration>(active_->generation) : std::nullopt;
}

std::size_t SnapshotPool::make_data_operations()
{
	return make_data_operations_counter.load(std::memory_order_relaxed);
}

std::size_t SnapshotPool::delete_data_operations()
{
	return delete_data_operations_counter.load(std::memory_order_relaxed);
}

#ifdef MJR_BUILD_TESTING
void SnapshotPool::RecordMakeDataOperationForTest()
{
	make_data_operations_counter.fetch_add(1, std::memory_order_relaxed);
}

void SnapshotPool::RecordDeleteDataOperationForTest()
{
	delete_data_operations_counter.fetch_add(1, std::memory_order_relaxed);
}

void SnapshotPool::ThrowAfterDataAllocationForTest()
{
	throw_after_data_allocation_for_test.store(true, std::memory_order_relaxed);
}
#endif

} // namespace mujoco_ros::rendering
