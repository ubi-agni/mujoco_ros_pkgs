#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

namespace mujoco_ros::rendering {

inline constexpr std::size_t kRosPublicationQueueDepth = 2;

template <typename Item>
class BoundedPublicationQueue
{
public:
	using PublishFn = std::function<void(Item &)>;

	struct EnqueueResult
	{
		bool enqueued                    = false;
		bool dropped_older               = false;
		std::uint64_t dropped_capture_id = 0;
	};

	explicit BoundedPublicationQueue(std::size_t depth, PublishFn publish)
	    : depth_(depth == 0 ? 1 : depth), publish_(std::move(publish))
	{
		worker_ = std::thread([this]() { WorkerLoop(); });
	}

	~BoundedPublicationQueue() { Shutdown(); }

	BoundedPublicationQueue(const BoundedPublicationQueue &)            = delete;
	BoundedPublicationQueue &operator=(const BoundedPublicationQueue &) = delete;

	EnqueueResult Enqueue(Item item, const bool notify_worker = true)
	{
		EnqueueResult result;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (shutdown_requested_) {
				return result;
			}
			if (pending_.size() >= depth_) {
				result.dropped_older      = true;
				result.dropped_capture_id = pending_.front().capture_id;
				pending_.pop_front();
				++dropped_count_;
			}
			pending_.push_back(Entry{ item.capture_id, std::move(item) });
			result.enqueued = true;
		}
		if (notify_worker) {
			NotifyWorker();
		}
		return result;
	}

	void NotifyWorker() { condition_.notify_one(); }

	void CancelPending()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		cancelled_count_ += pending_.size();
		pending_.clear();
	}

	void Shutdown()
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (shutdown_requested_) {
				return;
			}
			shutdown_requested_ = true;
			block_dequeue_      = false;
		}
		condition_.notify_all();
		if (worker_.joinable()) {
			worker_.join();
		}
	}

	std::size_t dropped_count() const { return dropped_count_.load(std::memory_order_acquire); }

	std::size_t cancelled_count() const { return cancelled_count_.load(std::memory_order_acquire); }

	std::size_t pending_count() const
	{
		std::lock_guard<std::mutex> lock(mutex_);
		return pending_.size() + (active_ ? 1U : 0U);
	}

	bool worker_idle() const
	{
		std::lock_guard<std::mutex> lock(mutex_);
		return pending_.empty() && !active_;
	}

	void WaitUntilWorkerIdle()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		condition_.wait(lock, [this]() { return pending_.empty() && !active_; });
	}

	void SetBlockDequeue(bool block)
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			block_dequeue_ = block;
		}
		condition_.notify_all();
	}

private:
	struct Entry
	{
		std::uint64_t capture_id = 0;
		Item item;
	};

	void WorkerLoop()
	{
		for (;;) {
			std::optional<Entry> entry;
			{
				std::unique_lock<std::mutex> lock(mutex_);
				condition_.wait(lock, [this]() { return (shutdown_requested_ || !pending_.empty()) && !block_dequeue_; });
				if (shutdown_requested_ && pending_.empty()) {
					return;
				}
				entry = std::move(pending_.front());
				pending_.pop_front();
				active_ = true;
			}
			if (publish_) {
				publish_(entry->item);
			}
			{
				std::lock_guard<std::mutex> lock(mutex_);
				active_ = false;
			}
			condition_.notify_all();
		}
	}

	const std::size_t depth_;
	PublishFn publish_;
	std::deque<Entry> pending_;
	mutable std::mutex mutex_;
	std::condition_variable condition_;
	std::thread worker_;
	bool shutdown_requested_ = false;
	bool active_             = false;
	bool block_dequeue_      = false;
	std::atomic<std::size_t> dropped_count_{ 0 };
	std::atomic<std::size_t> cancelled_count_{ 0 };
};

} // namespace mujoco_ros::rendering
