#include <mujoco_ros/rendering/frame_boundary.hpp>

#include <mujoco_ros/rendering/frame_capacity.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace mujoco_ros::rendering {

struct FrameLease::Storage
{
	PlaneLayout layout;
	PlaneKind plane = PlaneKind::kRgb;
	FrameStamp stamp;
	std::vector<std::byte> bytes;
	std::size_t pool_index = std::numeric_limits<std::size_t>::max();
};

struct FrameWriter::State
{
	std::weak_ptr<FrameBoundary::State> boundary;
	FrameStatus status;
	FrameStamp stamp;
	PlaneKind plane = PlaneKind::kRgb;
	PlaneLayout layout;
	std::vector<std::byte> bytes;
	std::shared_ptr<const FrameLease::Storage> storage;
	bool committed                  = false;
	bool reservation_released       = false;
	std::size_t writer_buffer_index = std::numeric_limits<std::size_t>::max();
};

struct FrameBoundary::State
{
	mutable std::mutex mutex;
	FrameGeneration generation;
	FrameLayout layout;
	std::uint64_t next_capture_id = 1;
	std::array<std::uint64_t, 3> next_plane_sequence{ 1, 1, 1 };
	FrameStamp active_capture;
	std::size_t reserved_slots  = 0;
	std::size_t reserved_bytes  = 0;
	std::size_t committed_bytes = 0;
	std::vector<std::shared_ptr<const FrameLease::Storage>> records;
	struct PooledSlot
	{
		std::shared_ptr<FrameLease::Storage> storage;
		bool in_records = false;
	};
	std::vector<PooledSlot> pool;
	std::vector<std::vector<std::byte>> writer_buffers;
	std::vector<bool> writer_buffer_in_use;
	std::size_t storage_allocations      = 0;
	std::size_t byte_vector_growth_count = 0;
	bool slots_warmed                    = false;
	std::size_t warmed_slot_count        = 0;
	std::size_t max_slots                = 0;
	std::size_t max_bytes                = 0;
	std::condition_variable capacity_condition;
	bool stopped = false;
#ifdef MJR_BUILD_TESTING
	std::function<void()> capacity_wait_observer;
#endif
};

namespace {

FrameStatus MakeStatus(FrameStatusCode code, FrameGeneration generation, PlaneKind plane, std::uint64_t capture_id,
                       std::string message)
{
	FrameStatus status;
	status.code       = code;
	status.generation = generation;
	status.plane      = plane;
	status.capture_id = capture_id;
	status.message    = std::move(message);
	return status;
}

void ReleaseReservation(const std::shared_ptr<FrameWriter::State> &writer)
{
	if (!writer || writer->reservation_released) {
		return;
	}
	if (auto boundary = writer->boundary.lock()) {
		std::lock_guard<std::mutex> lock(boundary->mutex);
		if (writer->writer_buffer_index < boundary->writer_buffers.size()) {
			boundary->writer_buffers[writer->writer_buffer_index]       = std::move(writer->bytes);
			boundary->writer_buffer_in_use[writer->writer_buffer_index] = false;
			writer->writer_buffer_index                                 = std::numeric_limits<std::size_t>::max();
		}
		if (boundary->reserved_slots > 0) {
			--boundary->reserved_slots;
		}
		if (boundary->reserved_bytes >= writer->layout.byte_length) {
			boundary->reserved_bytes -= writer->layout.byte_length;
		} else {
			boundary->reserved_bytes = 0;
		}
		boundary->capacity_condition.notify_all();
	}
	writer->reservation_released = true;
}

std::size_t PlaneIndex(PlaneKind plane)
{
	return static_cast<std::size_t>(plane);
}

bool HasCapacity(const FrameBoundary::State &state, std::size_t max_slots, std::size_t max_bytes,
                 std::size_t required_slots, std::size_t required_bytes)
{
	const bool slots_fit = required_slots <= max_slots && state.reserved_slots <= max_slots - required_slots &&
	                       state.records.size() <= max_slots - required_slots - state.reserved_slots;
	const bool bytes_fit = required_bytes <= max_bytes && state.reserved_bytes <= max_bytes - required_bytes &&
	                       state.committed_bytes <= max_bytes - required_bytes - state.reserved_bytes;
	return slots_fit && bytes_fit;
}

bool HasNonConsumingCapacity(const FrameBoundary::State &state, std::size_t max_slots, std::size_t max_bytes,
                             std::size_t required_slots, std::size_t required_bytes)
{
	const auto budget_fits = [](std::size_t budget, std::size_t required, std::size_t reserved, std::size_t used) {
		return required <= budget && reserved <= budget - required && used <= budget - required - reserved;
	};
	if (required_slots > max_slots || required_bytes > max_bytes || state.reserved_slots > max_slots - required_slots ||
	    state.reserved_bytes > max_bytes - required_bytes) {
		return false;
	}
	std::size_t reclaimable_slots = 0;
	std::size_t reclaimable_bytes = 0;
	for (const auto &record : state.records) {
		if (record.use_count() == 1) {
			++reclaimable_slots;
			reclaimable_bytes += record->bytes.size();
		}
	}
	if (reclaimable_bytes > state.committed_bytes) {
		throw std::logic_error("frame boundary committed byte accounting underflow");
	}
	const auto record_count    = state.records.size() - reclaimable_slots;
	const auto committed_bytes = state.committed_bytes - reclaimable_bytes;
	return budget_fits(max_slots, required_slots, state.reserved_slots, record_count) &&
	       budget_fits(max_bytes, required_bytes, state.reserved_bytes, committed_bytes);
}

bool EvictOldestUnleasedRecord(FrameBoundary::State &state)
{
	for (auto it = state.records.begin(); it != state.records.end(); ++it) {
		if ((*it).use_count() > 1) {
			continue;
		}
		if (state.committed_bytes < (*it)->bytes.size()) {
			throw std::logic_error("frame boundary committed byte accounting underflow");
		}
		state.committed_bytes -= (*it)->bytes.size();
		const auto pool_index = (*it)->pool_index;
		if (pool_index < state.pool.size()) {
			auto &slot      = state.pool[pool_index];
			slot.in_records = false;
			slot.storage    = std::const_pointer_cast<FrameLease::Storage>(*it);
		}
		state.records.erase(it);
		return true;
	}
	return false;
}

bool EnsureCapacity(FrameBoundary::State &state, std::size_t max_slots, std::size_t max_bytes,
                    std::size_t required_slots, std::size_t required_bytes)
{
	while (!HasCapacity(state, max_slots, max_bytes, required_slots, required_bytes)) {
		if (!EvictOldestUnleasedRecord(state)) {
#ifdef MJR_BUILD_TESTING
			// Capacity exhaustion is surfaced to callers; this trace helps unit tests.
			(void)max_slots;
			(void)max_bytes;
			(void)required_slots;
			(void)required_bytes;
#endif
			return false;
		}
	}
	return true;
}

std::shared_ptr<FrameLease::Storage> AcquireFreeStorage(FrameBoundary::State &state)
{
	for (auto &slot : state.pool) {
		if (!slot.in_records && slot.storage) {
			return slot.storage;
		}
	}
	return nullptr;
}

void ResizeByteVector(std::vector<std::byte> &bytes, std::size_t byte_length, std::size_t &growth_counter)
{
	if (bytes.capacity() < byte_length) {
		++growth_counter;
	}
	bytes.resize(byte_length);
}

void WarmUpSlotsLocked(FrameBoundary::State &state, std::size_t warm_slot_count, std::size_t warm_byte_length)
{
	if (warm_byte_length == 0 || warm_slot_count == 0) {
		return;
	}
	warm_slot_count = std::min(warm_slot_count, state.pool.size());
	state.writer_buffers.resize(warm_slot_count);
	state.writer_buffer_in_use.assign(warm_slot_count, false);
	for (std::size_t index = 0; index < warm_slot_count; ++index) {
		if (!state.pool[index].in_records && state.pool[index].storage) {
			ResizeByteVector(state.pool[index].storage->bytes, warm_byte_length, state.byte_vector_growth_count);
		}
		ResizeByteVector(state.writer_buffers[index], warm_byte_length, state.byte_vector_growth_count);
	}
	state.warmed_slot_count = warm_slot_count;
	state.slots_warmed      = true;
}

} // namespace

std::optional<RenderBackpressurePolicy> RenderBackpressurePolicyFromString(const std::string &value)
{
	if (value == "drop") {
		return RenderBackpressurePolicy::kDrop;
	}
	if (value == "wait_for_slot") {
		return RenderBackpressurePolicy::kWaitForSlot;
	}
	return std::nullopt;
}

std::string RenderBackpressurePolicyToString(RenderBackpressurePolicy policy)
{
	switch (policy) {
		case RenderBackpressurePolicy::kDrop:
			return "drop";
		case RenderBackpressurePolicy::kWaitForSlot:
			return "wait_for_slot";
	}
	throw std::invalid_argument("unknown render backpressure policy");
}

FrameLease::~FrameLease()
{
	auto notifier = std::move(notifier_);
	storage_.reset();
	if (notifier) {
		notifier();
	}
}

FrameLease::FrameLease(FrameLease &&other) noexcept
    : storage_(std::move(other.storage_)), notifier_(std::move(other.notifier_))
{
	other.notifier_ = {};
}

FrameLease &FrameLease::operator=(FrameLease &&other) noexcept
{
	if (this != &other) {
		auto notifier = std::move(notifier_);
		storage_.reset();
		if (notifier) {
			notifier();
		}
		storage_        = std::move(other.storage_);
		notifier_       = std::move(other.notifier_);
		other.notifier_ = {};
	}
	return *this;
}

const std::vector<std::byte> &FrameLease::bytes() const
{
	if (!storage_) {
		throw std::logic_error("invalid frame lease");
	}
	return storage_->bytes;
}

const PlaneLayout &FrameLease::layout() const
{
	if (!storage_) {
		throw std::logic_error("invalid frame lease");
	}
	return storage_->layout;
}

PlaneKind FrameLease::plane() const
{
	if (!storage_) {
		throw std::logic_error("invalid frame lease");
	}
	return storage_->plane;
}

FrameGeneration FrameLease::generation() const
{
	if (!storage_) {
		throw std::logic_error("invalid frame lease");
	}
	return storage_->stamp.generation;
}

ModelGeneration FrameLease::model_generation() const
{
	if (!storage_) {
		throw std::logic_error("invalid frame lease");
	}
	return storage_->stamp.model_generation;
}

std::uint64_t FrameLease::capture_id() const
{
	if (!storage_) {
		throw std::logic_error("invalid frame lease");
	}
	return storage_->stamp.capture_id;
}

CameraId FrameLease::camera_id() const
{
	if (!storage_) {
		throw std::logic_error("invalid frame lease");
	}
	return storage_->stamp.camera_id;
}

std::uint64_t FrameLease::plane_sequence() const
{
	if (!storage_) {
		throw std::logic_error("invalid frame lease");
	}
	return storage_->stamp.plane_sequence;
}

std::int64_t FrameLease::simulation_time_ns() const
{
	if (!storage_) {
		throw std::logic_error("invalid frame lease");
	}
	return storage_->stamp.simulation_time_ns;
}

FrameWriter::FrameWriter(FrameWriter &&other) noexcept : state_(std::move(other.state_)) {}

FrameWriter &FrameWriter::operator=(FrameWriter &&other) noexcept
{
	if (this != &other) {
		ReleaseReservation(state_);
		state_ = std::move(other.state_);
	}
	return *this;
}

FrameWriter::~FrameWriter()
{
	if (state_ && !state_->committed) {
		ReleaseReservation(state_);
	}
}

const FrameStatus &FrameWriter::status() const
{
	static const FrameStatus invalid{ FrameStatusCode::kFrameUnavailable, 0, std::nullopt, FrameGeneration(0),
		                               "invalid frame writer" };
	return state_ ? state_->status : invalid;
}

std::vector<std::byte> &FrameWriter::bytes()
{
	if (!state_ || !state_->status.ok() || state_->committed) {
		throw std::logic_error("frame writer is not writable");
	}
	return state_->bytes;
}

FrameStatus FrameWriter::Commit()
{
	if (!state_) {
		return status();
	}
	if (!state_->status.ok()) {
		return state_->status;
	}
	if (state_->committed) {
		return FrameStatus::Ok();
	}
	if (state_->bytes.size() != state_->layout.byte_length) {
		return MakeStatus(FrameStatusCode::kInvalidLayout, state_->stamp.generation, state_->plane,
		                  state_->stamp.capture_id, "frame writer byte length changed");
	}
	const auto boundary = state_->boundary.lock();
	if (!boundary) {
		state_->status = MakeStatus(FrameStatusCode::kStopped, state_->stamp.generation, state_->plane,
		                            state_->stamp.capture_id, "frame boundary was destroyed");
		return state_->status;
	}
	{
		std::lock_guard<std::mutex> lock(boundary->mutex);
		if (boundary->generation != state_->stamp.generation) {
			state_->status = MakeStatus(FrameStatusCode::kStaleGeneration, state_->stamp.generation, state_->plane,
			                            state_->stamp.capture_id, "writer generation is no longer active");
			if (boundary->reserved_slots > 0) {
				--boundary->reserved_slots;
			}
			if (boundary->reserved_bytes >= state_->layout.byte_length) {
				boundary->reserved_bytes -= state_->layout.byte_length;
			} else {
				boundary->reserved_bytes = 0;
			}
			state_->reservation_released = true;
			boundary->capacity_condition.notify_all();
			return state_->status;
		}
		if (!EnsureCapacity(*boundary, boundary->max_slots, boundary->max_bytes, 0, 0)) {
			state_->status = MakeStatus(FrameStatusCode::kFrameSlotsExhausted, state_->stamp.generation, state_->plane,
			                            state_->stamp.capture_id, "frame slot or byte budget exhausted");
			boundary->capacity_condition.notify_all();
			return state_->status;
		}
		auto storage = AcquireFreeStorage(*boundary);
		if (!storage) {
			if (!EvictOldestUnleasedRecord(*boundary)) {
				state_->status = MakeStatus(FrameStatusCode::kFrameSlotsExhausted, state_->stamp.generation, state_->plane,
				                            state_->stamp.capture_id, "frame slot or byte budget exhausted");
				boundary->capacity_condition.notify_all();
				return state_->status;
			}
			storage = AcquireFreeStorage(*boundary);
		}
		if (!storage) {
			state_->status = MakeStatus(FrameStatusCode::kFrameSlotsExhausted, state_->stamp.generation, state_->plane,
			                            state_->stamp.capture_id, "frame slot or byte budget exhausted");
			boundary->capacity_condition.notify_all();
			return state_->status;
		}
		FrameStamp stamp     = state_->stamp;
		stamp.plane_sequence = boundary->next_plane_sequence[PlaneIndex(state_->plane)]++;
		storage->layout      = state_->layout;
		storage->plane       = state_->plane;
		storage->stamp       = stamp;
		std::swap(storage->bytes, state_->bytes);
		for (auto &slot : boundary->pool) {
			if (slot.storage.get() == storage.get()) {
				slot.in_records = true;
				slot.storage.reset();
				break;
			}
		}
		boundary->records.push_back(storage);
		boundary->committed_bytes += storage->bytes.size();
		state_->storage = std::move(storage);
		if (boundary->reserved_slots > 0) {
			--boundary->reserved_slots;
		}
		if (boundary->reserved_bytes >= state_->layout.byte_length) {
			boundary->reserved_bytes -= state_->layout.byte_length;
		} else {
			boundary->reserved_bytes = 0;
		}
		if (state_->writer_buffer_index < boundary->writer_buffers.size()) {
			boundary->writer_buffers[state_->writer_buffer_index]       = std::move(state_->bytes);
			boundary->writer_buffer_in_use[state_->writer_buffer_index] = false;
			state_->writer_buffer_index                                 = std::numeric_limits<std::size_t>::max();
		}
		boundary->capacity_condition.notify_all();
	}
	state_->reservation_released = true;
	state_->committed            = true;
	return FrameStatus::Ok();
}

std::optional<FrameLease> FrameWriter::Acquire() const
{
	if (!state_ || !state_->committed || !state_->storage) {
		return std::nullopt;
	}
	const auto boundary = state_->boundary.lock();
	if (!boundary) {
		return std::nullopt;
	}
	return FrameLease(state_->storage, [boundary]() { boundary->capacity_condition.notify_all(); });
}

FrameBoundary::FrameBoundary(std::size_t max_slots, std::size_t max_bytes)
    : state_(std::make_shared<State>()), max_slots_(max_slots), max_bytes_(max_bytes)
{
	if (max_slots == 0 || max_bytes == 0) {
		throw std::invalid_argument("frame boundary capacity must be positive");
	}
	std::lock_guard<std::mutex> lock(state_->mutex);
	state_->max_slots = max_slots_;
	state_->max_bytes = max_bytes_;
	state_->pool.reserve(max_slots_);
	for (std::size_t index = 0; index < max_slots_; ++index) {
		auto storage        = std::make_shared<FrameLease::Storage>();
		storage->pool_index = index;
		++state_->storage_allocations;
		state_->pool.push_back({ storage, false });
	}
}

FrameBoundary::~FrameBoundary()
{
	if (!state_) {
		return;
	}
	{
		std::lock_guard<std::mutex> lock(state_->mutex);
		state_->stopped = true;
	}
	state_->capacity_condition.notify_all();
}

void FrameBoundary::WarmUpSlots(const FrameLayout &layout, std::size_t warm_slot_count)
{
	if (layout.width <= 0 || layout.height <= 0 || layout.slot_byte_length == 0) {
		throw std::invalid_argument("frame layout is invalid");
	}
	std::lock_guard<std::mutex> lock(state_->mutex);
	if (warm_slot_count == 0) {
		warm_slot_count = ComputeWarmSlotCount(max_slots_, max_bytes_, layout.slot_byte_length, max_slots_);
	}
	WarmUpSlotsLocked(*state_, warm_slot_count, layout.slot_byte_length);
	state_->capacity_condition.notify_all();
}

FrameStatus FrameBoundary::Reconfigure(FrameGeneration generation, const FrameLayout &layout,
                                       std::size_t warm_slot_count)
{
	if (layout.width <= 0 || layout.height <= 0 || layout.slot_byte_length == 0) {
		return MakeStatus(FrameStatusCode::kInvalidLayout, generation, PlaneKind::kRgb, 0, "frame layout is invalid");
	}
	std::lock_guard<std::mutex> lock(state_->mutex);
	if (!EnsureCapacity(*state_, max_slots_, max_bytes_, 1, layout.slot_byte_length)) {
		state_->capacity_condition.notify_all();
		return MakeStatus(FrameStatusCode::kGenerationCapacityExhausted, generation, PlaneKind::kRgb, 0,
		                  "retained frame leases leave insufficient generation capacity");
	}
	if (warm_slot_count == 0) {
		warm_slot_count = ComputeWarmSlotCount(max_slots_, max_bytes_, layout.slot_byte_length, max_slots_);
	}
	state_->generation = generation;
	state_->layout     = layout;
	WarmUpSlotsLocked(*state_, warm_slot_count, layout.slot_byte_length);
	state_->capacity_condition.notify_all();
	return FrameStatus::Ok();
}

FrameStamp FrameBoundary::BeginCapture(ModelGeneration model_generation, std::int64_t simulation_time_ns,
                                       CameraId camera)
{
	std::lock_guard<std::mutex> lock(state_->mutex);
	FrameStamp stamp;
	stamp.capture_id         = state_->next_capture_id++;
	stamp.model_generation   = model_generation;
	stamp.camera_id          = camera;
	stamp.generation         = state_->generation;
	stamp.simulation_time_ns = simulation_time_ns;
	state_->active_capture   = stamp;
	return stamp;
}

FrameWriter FrameBoundary::TryAcquireWriter(FrameGeneration generation, PlaneKind plane, const PlaneLayout &layout)
{
	FrameStamp stamp;
	stamp.generation = generation;
	return TryAcquireWriter(generation, stamp, plane, layout);
}

FrameWriter FrameBoundary::TryAcquireWriter(FrameGeneration generation, const FrameStamp &input_stamp, PlaneKind plane,
                                            const PlaneLayout &layout)
{
	FrameStamp stamp = input_stamp;
	stamp.generation = generation;
	std::lock_guard<std::mutex> lock(state_->mutex);
	if (state_->generation == FrameGeneration(0)) {
		state_->generation = generation;
	}
	if (state_->generation != generation) {
		return FrameWriter(std::make_shared<FrameWriter::State>(
		    FrameWriter::State{ state_,
		                        MakeStatus(FrameStatusCode::kStaleGeneration, generation, plane, stamp.capture_id,
		                                   "writer generation is not active"),
		                        stamp,
		                        plane,
		                        layout,
		                        {},
		                        nullptr,
		                        false,
		                        true,
		                        std::numeric_limits<std::size_t>::max() }));
	}
	if (!layout.valid()) {
		return FrameWriter(std::make_shared<FrameWriter::State>(FrameWriter::State{
		    state_,
		    MakeStatus(FrameStatusCode::kInvalidLayout, generation, plane, stamp.capture_id, "plane layout is invalid"),
		    stamp,
		    plane,
		    layout,
		    {},
		    nullptr,
		    false,
		    true,
		    std::numeric_limits<std::size_t>::max() }));
	}
	if (stamp.capture_id == 0) {
		stamp            = state_->active_capture;
		stamp.generation = generation;
		if (stamp.capture_id == 0) {
			stamp.capture_id = state_->next_capture_id++;
		}
	}
	if (!EnsureCapacity(*state_, max_slots_, max_bytes_, 1, layout.byte_length)) {
		return FrameWriter(std::make_shared<FrameWriter::State>(
		    FrameWriter::State{ state_,
		                        MakeStatus(FrameStatusCode::kFrameSlotsExhausted, generation, plane, stamp.capture_id,
		                                   "frame slot or byte budget exhausted"),
		                        stamp,
		                        plane,
		                        layout,
		                        {},
		                        nullptr,
		                        false,
		                        true,
		                        std::numeric_limits<std::size_t>::max() }));
	}
	std::size_t writer_buffer_index = std::numeric_limits<std::size_t>::max();
	for (std::size_t index = 0; index < state_->writer_buffer_in_use.size(); ++index) {
		if (!state_->writer_buffer_in_use[index]) {
			writer_buffer_index = index;
			break;
		}
	}
	std::vector<std::byte> writer_bytes;
	if (writer_buffer_index < state_->writer_buffers.size()) {
		state_->writer_buffer_in_use[writer_buffer_index] = true;
		writer_bytes                                      = std::move(state_->writer_buffers[writer_buffer_index]);
		if (writer_bytes.size() != layout.byte_length) {
			ResizeByteVector(writer_bytes, layout.byte_length, state_->byte_vector_growth_count);
		}
	} else {
		ResizeByteVector(writer_bytes, layout.byte_length, state_->byte_vector_growth_count);
	}
	++state_->reserved_slots;
	state_->reserved_bytes += layout.byte_length;
	return FrameWriter(std::make_shared<FrameWriter::State>(FrameWriter::State{ state_, FrameStatus::Ok(), stamp, plane,
	                                                                            layout, std::move(writer_bytes), nullptr,
	                                                                            false, false, writer_buffer_index }));
}

FrameStatus FrameBoundary::WaitForCapacity(FrameGeneration generation, std::size_t required_slots,
                                           std::size_t required_bytes, std::function<bool()> cancelled)
{
	const auto state     = state_;
	const auto max_slots = max_slots_;
	const auto max_bytes = max_bytes_;
	if (!state) {
		return MakeStatus(FrameStatusCode::kStopped, generation, PlaneKind::kRgb, 0, "frame boundary was stopped");
	}
	if (required_slots > max_slots || required_bytes > max_bytes) {
		return MakeStatus(FrameStatusCode::kFrameSlotsExhausted, generation, PlaneKind::kRgb, 0,
		                  "requested frame capacity exceeds the bounded frame boundary budget");
	}
	std::unique_lock<std::mutex> lock(state->mutex);
	for (;;) {
		if (state->stopped) {
			return MakeStatus(FrameStatusCode::kStopped, generation, PlaneKind::kRgb, 0, "frame boundary was stopped");
		}
		if (cancelled && cancelled()) {
			return MakeStatus(FrameStatusCode::kStopped, generation, PlaneKind::kRgb, 0,
			                  "frame capacity wait was cancelled");
		}
		if (state->generation != FrameGeneration(0) && state->generation != generation) {
			return MakeStatus(FrameStatusCode::kStaleGeneration, generation, PlaneKind::kRgb, 0,
			                  "frame capacity wait generation is no longer active");
		}
		if (HasNonConsumingCapacity(*state, max_slots, max_bytes, required_slots, required_bytes)) {
			return FrameStatus::Ok();
		}
#ifdef MJR_BUILD_TESTING
		auto capacity_wait_observer = std::move(state->capacity_wait_observer);
		if (capacity_wait_observer) {
			capacity_wait_observer();
		}
#endif
		state->capacity_condition.wait(lock, [&] {
			return state->stopped || (cancelled && cancelled()) ||
			       (state->generation != FrameGeneration(0) && state->generation != generation) ||
			       HasNonConsumingCapacity(*state, max_slots, max_bytes, required_slots, required_bytes);
		});
	}
}

void FrameBoundary::NotifyCapacityWaiters()
{
	state_->capacity_condition.notify_all();
}

#ifdef MJR_BUILD_TESTING
void FrameBoundary::SetCapacityWaitObserverForTesting(std::function<void()> observer)
{
	std::lock_guard<std::mutex> lock(state_->mutex);
	state_->capacity_wait_observer = std::move(observer);
}
#endif

std::optional<FrameLease> FrameBoundary::AcquireLatest(PlaneKind plane) const
{
	return AcquireLatest(0, CameraId(0), plane);
}

std::optional<FrameLease> FrameBoundary::AcquireLatest(CameraId camera, PlaneKind plane) const
{
	return AcquireLatest(0, camera, plane);
}

std::optional<FrameLease> FrameBoundary::AcquireLatest(std::uint64_t capture_id, PlaneKind plane) const
{
	return AcquireLatest(capture_id, CameraId(0), plane);
}

std::optional<FrameLease> FrameBoundary::AcquireLatest(std::uint64_t capture_id, CameraId camera, PlaneKind plane) const
{
	std::lock_guard<std::mutex> lock(state_->mutex);
	for (auto it = state_->records.rbegin(); it != state_->records.rend(); ++it) {
		if ((*it)->stamp.generation != state_->generation) {
			continue;
		}
		if ((*it)->plane == plane && (capture_id == 0 || (*it)->stamp.capture_id == capture_id) &&
		    (camera == CameraId(0) || (*it)->stamp.camera_id == camera)) {
			const auto boundary = state_;
			return FrameLease(*it, [boundary]() { boundary->capacity_condition.notify_all(); });
		}
	}
	return std::nullopt;
}

std::vector<FrameLease> FrameBoundary::AcquireRecent(CameraId camera, PlaneKind plane, std::size_t max_count) const
{
	std::vector<FrameLease> result;
	if (max_count == 0) {
		return result;
	}
	std::lock_guard<std::mutex> lock(state_->mutex);
	result.reserve(std::min(max_count, state_->records.size()));
	for (auto it = state_->records.rbegin(); it != state_->records.rend() && result.size() < max_count; ++it) {
		if ((*it)->stamp.generation == state_->generation && (*it)->plane == plane &&
		    (camera == CameraId(0) || (*it)->stamp.camera_id == camera)) {
			const auto boundary = state_;
			result.emplace_back(FrameLease(*it, [boundary]() { boundary->capacity_condition.notify_all(); }));
		}
	}
	std::reverse(result.begin(), result.end());
	return result;
}

FrameGeneration FrameBoundary::generation() const
{
	std::lock_guard<std::mutex> lock(state_->mutex);
	return state_->generation;
}

#ifdef MJR_BUILD_TESTING
std::size_t FrameBoundary::storage_allocation_count() const
{
	std::lock_guard<std::mutex> lock(state_->mutex);
	return state_->storage_allocations;
}

std::size_t FrameBoundary::byte_vector_growth_count() const
{
	std::lock_guard<std::mutex> lock(state_->mutex);
	return state_->byte_vector_growth_count;
}

std::size_t FrameBoundary::warmed_slot_count() const
{
	std::lock_guard<std::mutex> lock(state_->mutex);
	return state_->warmed_slot_count;
}

std::size_t FrameBoundary::warmed_reserved_bytes() const
{
	std::lock_guard<std::mutex> lock(state_->mutex);
	std::size_t total = 0;
	for (std::size_t index = 0; index < state_->warmed_slot_count; ++index) {
		if (index < state_->pool.size() && state_->pool[index].storage) {
			total += state_->pool[index].storage->bytes.capacity();
		}
		if (index < state_->writer_buffers.size()) {
			total += state_->writer_buffers[index].capacity();
		}
	}
	return total;
}
#endif

} // namespace mujoco_ros::rendering
