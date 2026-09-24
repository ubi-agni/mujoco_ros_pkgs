#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <atomic>
#include <optional>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <mujoco_ros/ros_version.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <boost/function.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

#include <mujoco_ros/offscreen_camera.hpp>
#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>

// Harness teardown invariant (docs/guardrails.md): flush JSON, shut down step worker,
// drain offscreen publication workers, then join physics/events before camera/queue destructors.

namespace mujoco_ros::rendering {
class OffscreenCameraTestAccess
{
public:
	static void SetHooks(OffscreenCamera &camera, std::function<void()> publication,
	                     std::function<void()> publication_end, std::function<void()> reset)
	{
		std::lock_guard<std::mutex> lock(camera.test_hook_mutex_);
		camera.publication_test_hook_     = std::move(publication);
		camera.publication_end_test_hook_ = std::move(publication_end);
		camera.reset_test_hook_           = std::move(reset);
	}

	static void SetEnqueueHook(OffscreenCamera &camera, std::function<void()> publication_enqueue)
	{
		std::lock_guard<std::mutex> lock(camera.test_hook_mutex_);
		camera.publication_enqueue_test_hook_ = std::move(publication_enqueue);
	}

	static void SetWorkerHook(OffscreenCamera &camera, std::function<void()> worker_publication)
	{
		std::lock_guard<std::mutex> lock(camera.test_hook_mutex_);
		camera.worker_publication_hook_ = std::move(worker_publication);
	}

	static void SetPublishEntryHook(OffscreenCamera &camera, std::function<void()> publication_publish_entry)
	{
		std::lock_guard<std::mutex> lock(camera.test_hook_mutex_);
		camera.publication_publish_entry_test_hook_ = std::move(publication_publish_entry);
	}

	static bool PublicationWorkerActive(const OffscreenCamera &camera)
	{
		if (!camera.publication_queue_) {
			return false;
		}
		return !camera.publication_queue_->worker_idle();
	}

	static void WaitUntilPublicationWorkerIdle(OffscreenCamera &camera)
	{
		if (camera.publication_queue_) {
			camera.publication_queue_->WaitUntilWorkerIdle();
		}
	}

	static void CancelPendingPublications(OffscreenCamera &camera)
	{
		if (camera.publication_queue_) {
			camera.publication_queue_->CancelPending();
		}
	}
};
} // namespace mujoco_ros::rendering

namespace {

using Clock = std::chrono::steady_clock;

mjtNum BenchmarkTime()
{
	return std::chrono::duration<mjtNum>(Clock::now().time_since_epoch()).count();
}

class TimerCallbackGuard
{
public:
	TimerCallbackGuard() : previous_(mjcb_time) { mjcb_time = BenchmarkTime; }
	~TimerCallbackGuard() { mjcb_time = previous_; }

	TimerCallbackGuard(const TimerCallbackGuard &)            = delete;
	TimerCallbackGuard &operator=(const TimerCallbackGuard &) = delete;

private:
	mjfTime previous_;
};

struct Options
{
	std::string backend = "GLFW";
	int camera_count    = 1;
	int iterations      = 1000;
	int repeat          = 1;
	std::string output;
	std::string revision;
	std::string interleave_letter;
	std::string gtest_output;
	int block_index = 0;
	int trial_index = 0;
};

struct PublicationStepTimestamps
{
	bool enqueue_valid        = false;
	bool worker_start_valid   = false;
	bool publish_entry_valid  = false;
	bool publish_return_valid = false;
	Clock::time_point enqueue;
	Clock::time_point worker_start;
	Clock::time_point publish_entry;
	Clock::time_point publish_return;
};

struct PublicationSpanSamples
{
	std::vector<double> enqueue_to_worker_ms;
	std::vector<double> worker_to_publish_entry_ms;
	std::vector<double> publish_entry_to_return_ms;
	std::vector<double> publish_return_to_receipt_ms;
	std::vector<double> enqueue_to_receipt_ms;
};

struct TrialMeasurements
{
	std::vector<double> frame_latency_ms;
	std::vector<double> physics_step_latency_ms;
	std::vector<double> wrapped_step_latency_ms;
	std::vector<double> post_step_to_receipt_latency_ms;
	PublicationSpanSamples publication_spans;
	double elapsed_wall_time_ms = 0.0;
	double renders              = 0.0;
};

struct Measurements
{
	std::vector<double> frame_latency_ms;
	std::vector<double> physics_step_latency_ms;
	std::vector<double> wrapped_step_latency_ms;
	std::vector<double> post_step_to_receipt_latency_ms;
	PublicationSpanSamples publication_spans;
	std::vector<TrialMeasurements> trials;
	double renders = 0.0;
};

struct PhysicsStepMeasurement
{
	double duration_ms;
	std::int64_t simulation_time_ns;
};

class BenchmarkEnv final : public MujocoEnvTestWrapper
{
public:
	using MujocoEnvTestWrapper::MujocoEnvTestWrapper;

	PhysicsStepMeasurement RunRenderStep()
	{
		mujoco_ros::RecursiveLock lock(physics_thread_mutex_);
		const auto timer_before = getDataPtr()->timer[mjTIMER_STEP];
		WrappedStep();
		const auto timer_after = getDataPtr()->timer[mjTIMER_STEP];
		if (timer_after.number - timer_before.number != 1) {
			throw std::runtime_error("MujocoEnv WrappedStep did not execute exactly one mj_step");
		}
		const double duration_ms = (timer_after.duration - timer_before.duration) * 1000.0;
		if (!(duration_ms > 0.0)) {
			throw std::runtime_error("MuJoCo mjTIMER_STEP produced a non-positive duration");
		}
		return { duration_ms, static_cast<std::int64_t>(getDataPtr()->time * 1e9) };
	}
};

class PublicationSpanCollector
{
public:
	explicit PublicationSpanCollector(int camera_count) : per_camera_(static_cast<std::size_t>(camera_count)) {}

	void InstallOnCameras(BenchmarkEnv &env)
	{
		auto *offscreen = env.getCameraPublicationTransport();
		for (std::size_t camera = 0; camera < offscreen->cams.size(); ++camera) {
			auto &offscreen_camera = *offscreen->cams[camera];
			mujoco_ros::rendering::OffscreenCameraTestAccess::SetHooks(
			    offscreen_camera, [this, camera] { RecordPublicationBegin(camera); },
			    [this, camera] { RecordPublishReturn(camera); }, {});
			mujoco_ros::rendering::OffscreenCameraTestAccess::SetEnqueueHook(offscreen_camera,
			                                                                 [this, camera] { RecordEnqueue(camera); });
			mujoco_ros::rendering::OffscreenCameraTestAccess::SetWorkerHook(offscreen_camera,
			                                                                [this, camera] { RecordWorkerStart(camera); });
			mujoco_ros::rendering::OffscreenCameraTestAccess::SetPublishEntryHook(
			    offscreen_camera, [this, camera] { RecordPublishEntry(camera); });
		}
	}

	void BeginStep() { ResetCurrentStep(); }

	PublicationSpanSamples FinalizeStep(const std::vector<Clock::time_point> &receipts)
	{
		PublicationSpanSamples samples;
		if (receipts.size() != per_camera_.size()) {
			throw std::runtime_error("publication span receipt count does not match configured camera count");
		}
		for (std::size_t camera = 0; camera < per_camera_.size(); ++camera) {
			AppendCameraSpans(samples, per_camera_[camera], receipts[camera]);
		}
		ResetCurrentStep();
		return samples;
	}

private:
	static void AppendSpan(std::vector<double> &target, bool from_valid, bool to_valid, Clock::time_point from,
	                       Clock::time_point to)
	{
		if (!from_valid || !to_valid) {
			return;
		}
		target.push_back(std::chrono::duration<double, std::milli>(to - from).count());
	}

	static void AppendCameraSpans(PublicationSpanSamples &samples, const PublicationStepTimestamps &timestamps,
	                              Clock::time_point receipt)
	{
		const auto publish_return       = timestamps.publish_return_valid ? timestamps.publish_return : receipt;
		const auto publish_return_valid = timestamps.publish_return_valid;
		const auto publish_entry        = timestamps.publish_entry_valid ?
		                                      timestamps.publish_entry :
		                                      (timestamps.worker_start_valid ? timestamps.worker_start : timestamps.enqueue);
		const auto publish_entry_valid =
		    timestamps.publish_entry_valid || timestamps.worker_start_valid || timestamps.enqueue_valid;
		const auto worker_start       = timestamps.worker_start_valid ? timestamps.worker_start : timestamps.enqueue;
		const auto worker_start_valid = timestamps.worker_start_valid || timestamps.enqueue_valid;

		AppendSpan(samples.enqueue_to_worker_ms, timestamps.enqueue_valid, worker_start_valid, timestamps.enqueue,
		           worker_start);
		AppendSpan(samples.worker_to_publish_entry_ms, worker_start_valid, publish_entry_valid, worker_start,
		           publish_entry);
		AppendSpan(samples.publish_entry_to_return_ms, publish_entry_valid, publish_return_valid, publish_entry,
		           publish_return);
		AppendSpan(samples.publish_return_to_receipt_ms, publish_return_valid, true, publish_return, receipt);
		AppendSpan(samples.enqueue_to_receipt_ms, timestamps.enqueue_valid, true, timestamps.enqueue, receipt);
	}

	void ResetCurrentStep()
	{
		for (auto &camera : per_camera_) {
			camera = PublicationStepTimestamps{};
		}
	}

	void RecordPublicationBegin(std::size_t camera)
	{
		if (!timestamps(camera).publish_entry_valid) {
			timestamps(camera).publish_entry       = Clock::now();
			timestamps(camera).publish_entry_valid = true;
		}
	}

	void RecordEnqueue(std::size_t camera)
	{
		timestamps(camera).enqueue       = Clock::now();
		timestamps(camera).enqueue_valid = true;
	}

	void RecordWorkerStart(std::size_t camera)
	{
		timestamps(camera).worker_start       = Clock::now();
		timestamps(camera).worker_start_valid = true;
	}

	void RecordPublishEntry(std::size_t camera)
	{
		timestamps(camera).publish_entry       = Clock::now();
		timestamps(camera).publish_entry_valid = true;
	}

	void RecordPublishReturn(std::size_t camera)
	{
		timestamps(camera).publish_return       = Clock::now();
		timestamps(camera).publish_return_valid = true;
	}

	PublicationStepTimestamps &timestamps(std::size_t camera) { return per_camera_.at(camera); }

	std::vector<PublicationStepTimestamps> per_camera_;
};

class StepWorker
{
public:
	explicit StepWorker(BenchmarkEnv &env) : env_(env), worker_([this] { WorkerLoop(); }) {}

	~StepWorker() { Shutdown(); }

	StepWorker(const StepWorker &)            = delete;
	StepWorker &operator=(const StepWorker &) = delete;

	void Shutdown()
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (shutdown_) {
				return;
			}
			shutdown_ = true;
			request_cv_.notify_one();
		}
		if (worker_.joinable()) {
			worker_.join();
		}
	}

	PhysicsStepMeasurement RunStep()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		if (shutdown_) {
			throw std::runtime_error("benchmark step worker is shut down");
		}
		pending_ = true;
		request_cv_.notify_one();
		done_cv_.wait(lock, [this] { return !pending_; });
		if (error_) {
			std::rethrow_exception(error_);
		}
		return result_;
	}

private:
	void WorkerLoop()
	{
		while (true) {
			std::unique_lock<std::mutex> lock(mutex_);
			request_cv_.wait(lock, [this] { return pending_ || shutdown_; });
			if (shutdown_ && !pending_) {
				return;
			}
			try {
				result_ = env_.RunRenderStep();
				error_  = nullptr;
			} catch (...) {
				error_ = std::current_exception();
			}
			pending_ = false;
			done_cv_.notify_one();
		}
	}

	BenchmarkEnv &env_;
	std::thread worker_;
	std::mutex mutex_;
	std::condition_variable request_cv_;
	std::condition_variable done_cv_;
	bool pending_  = false;
	bool shutdown_ = false;
	PhysicsStepMeasurement result_{};
	std::exception_ptr error_;
};

struct ImageReceipt
{
	Clock::time_point received_at;
	std::int64_t simulation_time_ns;
};

class ImageObserver
{
public:
	ImageObserver(BenchmarkEnv &env, int camera_count, int expected_iterations)
	    : env_(env), receipts_(static_cast<std::size_t>(camera_count))
	{
		const auto reserved_receipts = static_cast<std::size_t>(expected_iterations) + 2;
		for (auto &camera_receipts : receipts_) {
			camera_receipts.reserve(reserved_receipts);
		}
		const std::string topic_prefix = env.GetHandleNamespace() + "/cameras/camera_";
#if MJR_ROS_VERSION == ROS_1
		for (int camera = 0; camera < camera_count; ++camera) {
			const auto camera_index = static_cast<std::size_t>(camera);
			const boost::function<void(const sensor_msgs::Image::ConstPtr &)> callback =
			    [this, camera_index](const sensor_msgs::Image::ConstPtr &message) {
				    Record(camera_index, static_cast<std::int64_t>(message->header.stamp.toNSec()));
			    };
			subscriptions_.push_back(node_.subscribe<sensor_msgs::Image>(
			    topic_prefix + std::to_string(camera) + "/rgb/image_raw", 16, callback));
		}
#else
		node_ = std::make_shared<rclcpp::Node>("render_performance_observer");
		env.AddNodeToExecutor(node_->get_node_base_interface());
		for (int camera = 0; camera < camera_count; ++camera) {
			const auto camera_index = static_cast<std::size_t>(camera);
			subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::Image>(
			    topic_prefix + std::to_string(camera) + "/rgb/image_raw", rclcpp::SensorDataQoS(),
			    [this, camera_index](const sensor_msgs::msg::Image::ConstSharedPtr message) {
				    const auto &stamp = message->header.stamp;
				    Record(camera_index, static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec);
			    }));
		}
#endif
	}

	void Shutdown()
	{
		if (shutdown_) {
			return;
		}
		shutdown_ = true;
#if MJR_ROS_VERSION == ROS_1
		subscriptions_.clear();
#else
		subscriptions_.clear();
		if (node_) {
			env_.RemoveNodeFromExecutor(node_->get_node_base_interface());
		}
		node_.reset();
#endif
	}

	~ImageObserver() { Shutdown(); }

	std::vector<std::size_t> Counts() const
	{
		std::lock_guard<std::mutex> lock(mutex_);
		std::vector<std::size_t> counts;
		counts.reserve(receipts_.size());
		for (const auto &camera_receipts : receipts_) {
			counts.push_back(camera_receipts.size());
		}
		return counts;
	}

	void SetPendingReceiptCounts(std::vector<std::size_t> before)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		pending_receipt_counts_ = std::move(before);
	}

	void BeginAwaitingStep(std::int64_t expected_simulation_time_ns)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		awaiting_simulation_time_ns_ = expected_simulation_time_ns;
	}

	bool WaitForCurrentAfter(const std::vector<std::size_t> &before, std::int64_t expected_simulation_time_ns,
	                         std::chrono::milliseconds timeout) const
	{
		std::unique_lock<std::mutex> lock(mutex_);
		return receipt_cv_.wait_for(lock, timeout,
		                            [&] { return ReceivedCurrentAfterLocked(before, expected_simulation_time_ns); });
	}

	std::vector<Clock::time_point> CurrentReceiptsAfter(const std::vector<std::size_t> &before,
	                                                    std::int64_t expected_simulation_time_ns) const
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (before.size() != receipts_.size()) {
			throw std::runtime_error("RGB receipt baseline does not match configured camera count");
		}
		std::vector<Clock::time_point> result;
		result.reserve(receipts_.size());
		for (std::size_t camera = 0; camera < receipts_.size(); ++camera) {
			if (receipts_[camera].size() <= before[camera]) {
				throw std::runtime_error("RGB receipt missing for current simulation step");
			}
			const auto first_current =
			    std::find_if(receipts_[camera].begin() + static_cast<std::ptrdiff_t>(before[camera]),
			                 receipts_[camera].end(), [expected_simulation_time_ns](const ImageReceipt &receipt) {
				                 return receipt.simulation_time_ns == expected_simulation_time_ns;
			                 });
			if (first_current == receipts_[camera].end()) {
				throw std::runtime_error("RGB receipt missing for current simulation step");
			}
			result.push_back(first_current->received_at);
		}
		return result;
	}

private:
	bool ReceivedCurrentAfterLocked(const std::vector<std::size_t> &before,
	                                std::int64_t expected_simulation_time_ns) const
	{
		if (before.size() != receipts_.size()) {
			return false;
		}
		for (std::size_t camera = 0; camera < receipts_.size(); ++camera) {
			if (receipts_[camera].size() <= before[camera] ||
			    std::none_of(receipts_[camera].begin() + static_cast<std::ptrdiff_t>(before[camera]),
			                 receipts_[camera].end(), [expected_simulation_time_ns](const ImageReceipt &receipt) {
				                 return receipt.simulation_time_ns == expected_simulation_time_ns;
			                 })) {
				return false;
			}
		}
		return true;
	}

	void Record(std::size_t camera, std::int64_t simulation_time_ns)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		receipts_.at(camera).push_back({ Clock::now(), simulation_time_ns });
		if (awaiting_simulation_time_ns_ == simulation_time_ns && !pending_receipt_counts_.empty() &&
		    ReceivedCurrentAfterLocked(pending_receipt_counts_, simulation_time_ns)) {
			receipt_cv_.notify_all();
		}
	}

	BenchmarkEnv &env_;
	bool shutdown_ = false;
	mutable std::mutex mutex_;
	mutable std::condition_variable receipt_cv_;
	std::vector<std::vector<ImageReceipt>> receipts_;
	std::vector<std::size_t> pending_receipt_counts_;
	std::int64_t awaiting_simulation_time_ns_ = -1;
#if MJR_ROS_VERSION == ROS_1
	ros::NodeHandle node_;
	std::vector<ros::Subscriber> subscriptions_;
#else
	rclcpp::Node::SharedPtr node_;
	std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscriptions_;
#endif
};

class ImageObserverGuard
{
public:
	explicit ImageObserverGuard(ImageObserver &observer) : observer_(observer) {}
	~ImageObserverGuard() { observer_.Shutdown(); }

	ImageObserverGuard(const ImageObserverGuard &)            = delete;
	ImageObserverGuard &operator=(const ImageObserverGuard &) = delete;

private:
	ImageObserver &observer_;
};

Options Parse(int argc, char **argv)
{
	Options options;
	for (int i = 1; i < argc; ++i) {
		const std::string key(argv[i]);
		const std::string gtest_output_prefix = "--gtest_output=xml:";
		if (key.rfind(gtest_output_prefix, 0) == 0) {
			options.gtest_output = key.substr(gtest_output_prefix.size());
			continue;
		}
		if (key == "--backend" || key == "--camera-count" || key == "--iterations" || key == "--repeat" ||
		    key == "--output" || key == "--revision" || key == "--interleave-letter" || key == "--block-index" ||
		    key == "--trial-index") {
			if (i + 1 >= argc) {
				throw std::runtime_error("missing value for option: " + key);
			}
			const std::string value(argv[i + 1]);
			if (key == "--backend")
				options.backend = value;
			else if (key == "--camera-count")
				options.camera_count = std::stoi(value);
			else if (key == "--iterations")
				options.iterations = std::stoi(value);
			else if (key == "--repeat")
				options.repeat = std::stoi(value);
			else if (key == "--output")
				options.output = value;
			else if (key == "--revision")
				options.revision = value;
			else if (key == "--interleave-letter")
				options.interleave_letter = value;
			else if (key == "--block-index")
				options.block_index = std::stoi(value);
			else if (key == "--trial-index")
				options.trial_index = std::stoi(value);
			i += 1;
			continue;
		}
		throw std::runtime_error("unknown option: " + key);
	}
	if (options.output.empty() || options.iterations < 1 || options.repeat < 1) {
		throw std::runtime_error("--output, positive iterations, and repeat are required");
	}
	if (options.camera_count != 1 && options.camera_count != 2 && options.camera_count != 4) {
		throw std::runtime_error("camera count must be 1, 2, or 4 for the recovery protocol");
	}
	return options;
}

void WriteRostestResult(const Options &options)
{
	if (options.gtest_output.empty()) {
		return;
	}
	std::ofstream output(options.gtest_output);
	if (!output) {
		throw std::runtime_error("failed to open ROS 1 rostest result: " + options.gtest_output);
	}
	output << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
	       << "<testsuites tests=\"1\" failures=\"0\" errors=\"0\" time=\"0\">\n"
	       << "  <testsuite name=\"render_performance\" tests=\"1\" failures=\"0\" errors=\"0\">\n"
	       << "    <testcase name=\"benchmark_completed\"/>\n"
	       << "  </testsuite>\n"
	       << "</testsuites>\n";
	output.flush();
	if (!output) {
		throw std::runtime_error("failed to flush ROS 1 rostest result: " + options.gtest_output);
	}
}

std::string CompiledBackendName()
{
#if OFFSCREEN_RENDER_BACKEND == EGL_BACKEND
	return "EGL";
#elif OFFSCREEN_RENDER_BACKEND == OSMESA_BACKEND
	return "OSMESA";
#else
	return "NONE";
#endif
}

std::string CameraCountRole(int camera_count)
{
	switch (camera_count) {
		case 1:
		case 4:
			return "formal";
		case 2:
			return "diagnostic";
		default:
			throw std::runtime_error("camera count must be 1, 2, or 4 for the recovery protocol");
	}
}

std::string CameraWorld(int camera_count)
{
	std::string xml =
	    "<mujoco><option timestep='0.001'/><visual><global offwidth='64' offheight='64'/></visual><worldbody>"
	    "<body name='body'><geom type='box' size='.1 .1 .1'/>";
	for (int camera = 0; camera < camera_count; ++camera) {
		xml += "<camera name='camera_" + std::to_string(camera) + "' pos='0 -1 " +
		       std::to_string(0.2 + static_cast<double>(camera) * 0.02) + "' euler='90 0 0'/>";
	}
	return xml + "</body></worldbody></mujoco>";
}

template <typename Predicate, typename Rep, typename Period>
void WaitFor(Predicate predicate, std::chrono::duration<Rep, Period> timeout, const std::string &failure)
{
	const auto deadline = Clock::now() + timeout;
	while (!predicate() && Clock::now() < deadline) {
		std::this_thread::sleep_for(std::chrono::microseconds(100));
	}
	if (!predicate()) {
		throw std::runtime_error(failure);
	}
}

double Median(std::vector<double> values)
{
	if (values.empty())
		throw std::runtime_error("cannot compute a median without samples");
	std::sort(values.begin(), values.end());
	return values[values.size() / 2];
}

double RendersPerSecond(const TrialMeasurements &trial)
{
	if (trial.elapsed_wall_time_ms <= 0.0) {
		throw std::runtime_error("trial elapsed wall time must be positive");
	}
	return trial.renders / (trial.elapsed_wall_time_ms / 1000.0);
}

void ConfigureParameters(testing::TestNodeHandle &parameters, int camera_count)
{
	parameters.setParam("unpause", false);
	parameters.setParam("no_render", false);
	parameters.setParam("headless", true);
	parameters.setParam("render_offscreen", true);
	parameters.setParam("use_sim_time", true);
	for (int camera = 0; camera < camera_count; ++camera) {
		const std::string prefix = "cam_config/camera_" + std::to_string(camera);
		parameters.setParam(prefix + "/stream_type", mujoco_ros::rendering::StreamType::RGB);
		parameters.setParam(prefix + "/frequency", 2000.0);
		parameters.setParam(prefix + "/width", 64);
		parameters.setParam(prefix + "/height", 64);
	}
}

void ValidateFixture(BenchmarkEnv &env, int camera_count)
{
	if (env.GetOperationalStatus() != 0) {
		throw std::runtime_error("MujocoEnv did not become operational");
	}
	if (!env.settings_.render_offscreen || !env.isRenderingRunning()) {
		throw std::runtime_error("requested offscreen backend did not start");
	}
	auto *offscreen = env.getCameraPublicationTransport();
	if (offscreen->cams.size() != static_cast<std::size_t>(camera_count)) {
		throw std::runtime_error("configured camera count does not match loaded MujocoEnv camera count");
	}
	WaitFor(
	    [offscreen] {
		    return std::all_of(offscreen->cams.begin(), offscreen->cams.end(),
		                       [](const auto &camera) { return camera->rgb_pub_.getNumSubscribers() > 0; });
	    },
	    std::chrono::seconds(5), "RGB benchmark subscribers did not match every camera publisher");
}

struct CompletedRenderStep
{
	double physics_step_ms;
	double wrapped_step_ms;
	double post_step_to_receipt_ms;
	std::vector<double> frame_latency_ms;
	PublicationSpanSamples publication_spans;
};

void AppendPublicationSpans(PublicationSpanSamples &target, const PublicationSpanSamples &source)
{
	target.enqueue_to_worker_ms.insert(target.enqueue_to_worker_ms.end(), source.enqueue_to_worker_ms.begin(),
	                                   source.enqueue_to_worker_ms.end());
	target.worker_to_publish_entry_ms.insert(target.worker_to_publish_entry_ms.end(),
	                                         source.worker_to_publish_entry_ms.begin(),
	                                         source.worker_to_publish_entry_ms.end());
	target.publish_entry_to_return_ms.insert(target.publish_entry_to_return_ms.end(),
	                                         source.publish_entry_to_return_ms.begin(),
	                                         source.publish_entry_to_return_ms.end());
	target.publish_return_to_receipt_ms.insert(target.publish_return_to_receipt_ms.end(),
	                                           source.publish_return_to_receipt_ms.begin(),
	                                           source.publish_return_to_receipt_ms.end());
	target.enqueue_to_receipt_ms.insert(target.enqueue_to_receipt_ms.end(), source.enqueue_to_receipt_ms.begin(),
	                                    source.enqueue_to_receipt_ms.end());
}

std::optional<double> MedianOptional(const std::vector<double> &values)
{
	if (values.empty()) {
		return std::nullopt;
	}
	return Median(values);
}

void WriteOptionalMedian(std::ofstream &output, const char *key, const std::vector<double> &values)
{
	if (const auto median = MedianOptional(values)) {
		output << ", \"" << key << "\": " << *median;
	} else {
		output << ", \"" << key << "\": null";
	}
}

CompletedRenderStep RunCompletedRenderStep(StepWorker &worker, ImageObserver &observer,
                                           PublicationSpanCollector &span_collector)
{
	const auto receipt_counts = observer.Counts();
	observer.SetPendingReceiptCounts(receipt_counts);
	span_collector.BeginStep();
	const auto frame_start                 = Clock::now();
	const auto physics_step                = worker.RunStep();
	const auto physics_complete            = Clock::now();
	const auto expected_simulation_time_ns = physics_step.simulation_time_ns;
	observer.BeginAwaitingStep(expected_simulation_time_ns);
	if (!observer.WaitForCurrentAfter(receipt_counts, expected_simulation_time_ns, std::chrono::seconds(2))) {
		throw std::runtime_error("subscribed RGB completion timeout: no current-step image per configured camera");
	}
	CompletedRenderStep result;
	result.physics_step_ms = physics_step.duration_ms;
	result.wrapped_step_ms = std::chrono::duration<double, std::milli>(physics_complete - frame_start).count();
	const auto receipts    = observer.CurrentReceiptsAfter(receipt_counts, expected_simulation_time_ns);
	const auto last_receipt =
	    *std::max_element(receipts.begin(), receipts.end(),
	                      [](const Clock::time_point &lhs, const Clock::time_point &rhs) { return lhs < rhs; });
	result.post_step_to_receipt_ms = std::chrono::duration<double, std::milli>(last_receipt - physics_complete).count();
	for (const auto receipt : receipts) {
		result.frame_latency_ms.push_back(std::chrono::duration<double, std::milli>(receipt - frame_start).count());
	}
	result.publication_spans = span_collector.FinalizeStep(receipts);
	return result;
}

void WriteResults(const Options &options, const Measurements &measurements)
{
	std::ofstream output(options.output);
	if (!output) {
		throw std::runtime_error("could not open output");
	}
	TrialMeasurements aggregate;
	aggregate.frame_latency_ms                = measurements.frame_latency_ms;
	aggregate.physics_step_latency_ms         = measurements.physics_step_latency_ms;
	aggregate.wrapped_step_latency_ms         = measurements.wrapped_step_latency_ms;
	aggregate.post_step_to_receipt_latency_ms = measurements.post_step_to_receipt_latency_ms;
	aggregate.publication_spans               = measurements.publication_spans;
	aggregate.elapsed_wall_time_ms =
	    std::accumulate(measurements.trials.begin(), measurements.trials.end(), 0.0,
	                    [](double total, const TrialMeasurements &trial) { return total + trial.elapsed_wall_time_ms; });
	aggregate.renders = measurements.renders;

	std::vector<double> trial_throughputs;
	trial_throughputs.reserve(measurements.trials.size());
	for (const auto &trial : measurements.trials) {
		trial_throughputs.push_back(RendersPerSecond(trial));
	}

	output << std::setprecision(9) << "{\n"
	       << "  \"backend\": \"" << options.backend << "\",\n"
	       << "  \"completion_boundary\": \"subscribed RGB image received for each configured camera at current "
	          "simulation timestamp\",\n"
	       << "  \"frame_latency_boundary\": \"MujocoEnv WrappedStep entry to current-step subscribed RGB image "
	          "receipt per camera\",\n"
	       << "  \"physics_step_boundary\": \"MuJoCo mjTIMER_STEP duration delta for exactly one mj_step call; "
	          "excludes RenderCore and consumer delivery\",\n"
	       << "  \"post_step_to_receipt_boundary\": \"RunRenderStep return to last current-step subscribed RGB "
	          "image receipt; publication may occur inside RunRenderStep, so this is not publish-to-receipt\",\n"
	       << "  \"camera_count\": " << options.camera_count << ",\n"
	       << "  \"camera_count_role\": \"" << CameraCountRole(options.camera_count) << "\",\n"
	       << "  \"iterations_per_trial\": " << options.iterations << ",\n"
	       << "  \"trial_count\": " << options.repeat << ",\n"
	       << "  \"measurement_protocol\": {\n"
	       << "    \"step_worker\": \"persistent\",\n"
	       << "    \"completion_sync\": \"condition_variable\",\n"
	       << "    \"gate_throughput_statistic\": \"median_trial_renders_per_second\",\n"
	       << "    \"interleave_protocol\": \"B-C-C-B\"\n"
	       << "  },\n";
	if (!options.revision.empty() || options.block_index > 0 || options.trial_index > 0 ||
	    !options.interleave_letter.empty()) {
		output << "  \"run_metadata\": {\n";
		output << "    \"revision\": " << (options.revision.empty() ? "null" : "\"" + options.revision + "\"") << ",\n";
		output << "    \"block_index\": " << options.block_index << ",\n";
		output << "    \"trial_index\": " << options.trial_index << ",\n";
		output << "    \"interleave_letter\": "
		       << (options.interleave_letter.empty() ? "null" : "\"" + options.interleave_letter + "\"") << "\n"
		       << "  },\n";
	}
	output << "  \"frame_latency_samples\": " << measurements.frame_latency_ms.size() << ",\n"
	       << "  \"physics_step_samples\": " << measurements.physics_step_latency_ms.size() << ",\n"
	       << "  \"render_count\": " << aggregate.renders << ",\n"
	       << "  \"elapsed_wall_time_ms\": " << aggregate.elapsed_wall_time_ms << ",\n"
	       << "  \"renders_per_second\": " << RendersPerSecond(aggregate) << ",\n"
	       << "  \"median_trial_renders_per_second\": " << Median(trial_throughputs) << ",\n"
	       << "  \"median_frame_latency_ms\": " << Median(measurements.frame_latency_ms) << ",\n"
	       << "  \"median_physics_step_duration_ms\": " << Median(measurements.physics_step_latency_ms) << ",\n"
	       << "  \"attribution\": {\n"
	       << "    \"median_wrapped_step_ms\": " << Median(measurements.wrapped_step_latency_ms) << ",\n"
	       << "    \"median_post_step_to_receipt_ms\": " << Median(measurements.post_step_to_receipt_latency_ms) << ",\n"
	       << "    \"post_step_to_receipt_note\": "
	          "\"non-causal when publication completes inside WrappedStep; prefer publication span medians\",\n"
	       << "    \"median_enqueue_to_worker_ms\": ";
	if (const auto median = MedianOptional(measurements.publication_spans.enqueue_to_worker_ms)) {
		output << *median;
	} else {
		output << "null";
	}
	output << ",\n"
	       << "    \"median_worker_to_publish_entry_ms\": ";
	if (const auto median = MedianOptional(measurements.publication_spans.worker_to_publish_entry_ms)) {
		output << *median;
	} else {
		output << "null";
	}
	output << ",\n"
	       << "    \"median_publish_entry_to_return_ms\": ";
	if (const auto median = MedianOptional(measurements.publication_spans.publish_entry_to_return_ms)) {
		output << *median;
	} else {
		output << "null";
	}
	output << ",\n"
	       << "    \"median_publish_return_to_receipt_ms\": ";
	if (const auto median = MedianOptional(measurements.publication_spans.publish_return_to_receipt_ms)) {
		output << *median;
	} else {
		output << "null";
	}
	output << ",\n"
	       << "    \"median_enqueue_to_receipt_ms\": ";
	if (const auto median = MedianOptional(measurements.publication_spans.enqueue_to_receipt_ms)) {
		output << *median;
	} else {
		output << "null";
	}
	output << "\n"
	       << "  },\n"
	       << "  \"trials\": [\n";
	for (std::size_t index = 0; index < measurements.trials.size(); ++index) {
		const auto &trial = measurements.trials[index];
		output << "    {\"index\": " << (index + 1) << ", \"elapsed_wall_time_ms\": " << trial.elapsed_wall_time_ms
		       << ", \"render_count\": " << trial.renders
		       << ", \"frame_latency_samples\": " << trial.frame_latency_ms.size()
		       << ", \"physics_step_samples\": " << trial.physics_step_latency_ms.size()
		       << ", \"renders_per_second\": " << RendersPerSecond(trial)
		       << ", \"median_frame_latency_ms\": " << Median(trial.frame_latency_ms)
		       << ", \"median_physics_step_duration_ms\": " << Median(trial.physics_step_latency_ms)
		       << ", \"median_wrapped_step_ms\": " << Median(trial.wrapped_step_latency_ms)
		       << ", \"median_post_step_to_receipt_ms\": " << Median(trial.post_step_to_receipt_latency_ms);
		WriteOptionalMedian(output, "median_publish_return_to_receipt_ms",
		                    trial.publication_spans.publish_return_to_receipt_ms);
		WriteOptionalMedian(output, "median_enqueue_to_receipt_ms", trial.publication_spans.enqueue_to_receipt_ms);
		output << "}";
		output << (index + 1 == measurements.trials.size() ? "\n" : ",\n");
	}
	output << "  ]\n}\n";
	output.flush();
	if (!output) {
		throw std::runtime_error("failed to flush benchmark results");
	}
}

void WaitForPublicationWorkersIdle(BenchmarkEnv &env, std::chrono::milliseconds timeout)
{
	auto *offscreen         = env.getCameraPublicationTransport();
	const auto workers_idle = [offscreen] {
		return std::all_of(offscreen->cams.begin(), offscreen->cams.end(), [](const auto &camera) {
			return !mujoco_ros::rendering::OffscreenCameraTestAccess::PublicationWorkerActive(*camera);
		});
	};

	const auto deadline = Clock::now() + timeout;
	while (!workers_idle() && Clock::now() < deadline) {
		std::this_thread::sleep_for(std::chrono::microseconds(100));
	}

	if (!workers_idle()) {
		for (auto &camera : offscreen->cams) {
			mujoco_ros::rendering::OffscreenCameraTestAccess::CancelPendingPublications(*camera);
		}
		const auto recovery_deadline = Clock::now() + std::chrono::seconds(2);
		while (!workers_idle() && Clock::now() < recovery_deadline) {
			std::this_thread::sleep_for(std::chrono::microseconds(100));
		}
	}

	if (!workers_idle()) {
		std::cerr << "FATAL: offscreen publication workers did not become idle after cancel/recovery; exiting without "
		             "env "
		             "destructor to avoid unbounded queue join\n";
		std::_Exit(1);
	}

	for (auto &camera : offscreen->cams) {
		mujoco_ros::rendering::OffscreenCameraTestAccess::WaitUntilPublicationWorkerIdle(*camera);
	}
}

void ShutdownBenchmarkHarness(StepWorker &worker, ImageObserver &observer, BenchmarkEnv &env)
{
	worker.Shutdown();
	WaitForPublicationWorkersIdle(env, std::chrono::seconds(15));
	observer.Shutdown();
	env.shutdown();
}

class BenchmarkEnvTeardownGuard
{
public:
	BenchmarkEnvTeardownGuard(BenchmarkEnv &env, StepWorker &worker) : env_(env), worker_(worker) {}

	~BenchmarkEnvTeardownGuard()
	{
		if (teardown_done_) {
			return;
		}
		worker_.Shutdown();
		WaitForPublicationWorkersIdle(env_, std::chrono::seconds(5));
		env_.shutdown();
	}

	void MarkTeardownDone() { teardown_done_ = true; }

	BenchmarkEnvTeardownGuard(const BenchmarkEnvTeardownGuard &)            = delete;
	BenchmarkEnvTeardownGuard &operator=(const BenchmarkEnvTeardownGuard &) = delete;

private:
	BenchmarkEnv &env_;
	StepWorker &worker_;
	bool teardown_done_ = false;
};

int Run(const Options &options)
{
	TimerCallbackGuard timer_callback_guard;
	const std::string compiled_backend = CompiledBackendName();
	if (options.backend != compiled_backend) {
		throw std::runtime_error("requested backend '" + options.backend + "' does not match compiled backend '" +
		                         compiled_backend + "'");
	}

	auto parameters = std::make_unique<testing::TestNodeHandle>("~");
	ConfigureParameters(*parameters, options.camera_count);
	auto env                    = std::make_unique<BenchmarkEnv>("", parameters.get());
	const int expected_receipts = options.iterations * options.repeat + 2;
	ImageObserver observer(*env, options.camera_count, expected_receipts);
	const ImageObserverGuard observer_guard(observer);
	StepWorker worker(*env);
	BenchmarkEnvTeardownGuard env_teardown_guard(*env, worker);
	env->StartPhysicsLoop();
	env->StartEventLoop();
	char load_error[mujoco_ros::MujocoEnv::kErrorLength] = {};
	if (!env->LoadModelFromString(CameraWorld(options.camera_count), load_error, sizeof(load_error))) {
		throw std::runtime_error(std::string("MujocoEnv model load failed: ") + load_error);
	}
	ValidateFixture(*env, options.camera_count);
	PublicationSpanCollector span_collector(options.camera_count);
	span_collector.InstallOnCameras(*env);
	RunCompletedRenderStep(worker, observer, span_collector);

	Measurements measurements;
	for (int run = 0; run < options.repeat; ++run) {
		TrialMeasurements trial;
		const auto trial_start = Clock::now();
		for (int iteration = 0; iteration < options.iterations; ++iteration) {
			const auto completed_step = RunCompletedRenderStep(worker, observer, span_collector);
			trial.physics_step_latency_ms.push_back(completed_step.physics_step_ms);
			measurements.physics_step_latency_ms.push_back(completed_step.physics_step_ms);
			trial.wrapped_step_latency_ms.push_back(completed_step.wrapped_step_ms);
			measurements.wrapped_step_latency_ms.push_back(completed_step.wrapped_step_ms);
			trial.post_step_to_receipt_latency_ms.push_back(completed_step.post_step_to_receipt_ms);
			measurements.post_step_to_receipt_latency_ms.push_back(completed_step.post_step_to_receipt_ms);
			AppendPublicationSpans(trial.publication_spans, completed_step.publication_spans);
			AppendPublicationSpans(measurements.publication_spans, completed_step.publication_spans);
			trial.frame_latency_ms.insert(trial.frame_latency_ms.end(), completed_step.frame_latency_ms.begin(),
			                              completed_step.frame_latency_ms.end());
			measurements.frame_latency_ms.insert(measurements.frame_latency_ms.end(),
			                                     completed_step.frame_latency_ms.begin(),
			                                     completed_step.frame_latency_ms.end());
			trial.renders += static_cast<double>(completed_step.frame_latency_ms.size());
			measurements.renders += static_cast<double>(completed_step.frame_latency_ms.size());
		}
		trial.elapsed_wall_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - trial_start).count();
		measurements.trials.push_back(std::move(trial));
	}
	WriteResults(options, measurements);
	WriteRostestResult(options);
	ShutdownBenchmarkHarness(worker, observer, *env);
	env_teardown_guard.MarkTeardownDone();
	return 0;
}

} // namespace

int main(int argc, char **argv)
{
#if MJR_ROS_VERSION == ROS_1
	ros::init(argc, argv, "render_performance_test", ros::init_options::AnonymousName);
	ros::AsyncSpinner spinner(2);
	spinner.start();
#else
	rclcpp::init(argc, argv);
#endif
	int result = 1;
	try {
		result = Run(Parse(argc, argv));
	} catch (const std::exception &error) {
		std::cerr << error.what() << '\n';
	}
#if MJR_ROS_VERSION == ROS_1
	spinner.stop();
	ros::shutdown();
#else
	rclcpp::shutdown();
#endif
	return result;
}
