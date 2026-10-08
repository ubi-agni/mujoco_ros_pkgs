#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

#include <mujoco_ros/ros_version.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

#if RENDER_BACKEND == GLFW_BACKEND
#include <mujoco_ros/viewer.hpp>
#include <mujoco_ros/glfw_adapter.h>
#include <mujoco_ros/viewer_sync_policy.hpp>
#endif

#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>

using mujoco_ros::Viewer;
#if RENDER_BACKEND == GLFW_BACKEND
using mujoco_ros::GlfwAdapter;
#endif

namespace {

using Clock = std::chrono::steady_clock;

struct Stats
{
	double median   = 0.0;
	double variance = 0.0;
};

Stats ComputeStats(std::vector<double> samples)
{
	if (samples.empty()) {
		return {};
	}
	std::sort(samples.begin(), samples.end());
	Stats stats;
	const std::size_t mid = samples.size() / 2;
	stats.median          = samples.size() % 2 == 0 ? 0.5 * (samples[mid - 1] + samples[mid]) : samples[mid];
	const double mean     = std::accumulate(samples.begin(), samples.end(), 0.0) / static_cast<double>(samples.size());
	double sq_sum         = 0.0;
	for (const double value : samples) {
		const double delta = value - mean;
		sq_sum += delta * delta;
	}
	stats.variance = samples.size() > 1 ? sq_sum / static_cast<double>(samples.size() - 1) : 0.0;
	return stats;
}

template <typename Predicate>
bool WaitUntil(Predicate &&predicate, std::chrono::milliseconds timeout)
{
	const auto deadline = Clock::now() + timeout;
	while (Clock::now() < deadline) {
		if (predicate()) {
			return true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	return predicate();
}

#if RENDER_BACKEND == GLFW_BACKEND
std::vector<double> BenchmarkProductionRenderCalls(Viewer &viewer, int samples)
{
	std::vector<double> render_call_hz;
	render_call_hz.reserve(static_cast<std::size_t>(samples));
	for (int i = 0; i < samples; ++i) {
		auto lease = viewer.connection_state_->TryAcquireEnvironment();
		if (!lease) {
			break;
		}
		viewer.frame_lease_ = std::move(lease);
		MujocoEnv &env      = *viewer.FrameEnvironment();

		const auto start = Clock::now();
		viewer.platform_ui->PollEvents();
		const int operational_status = env.GetOperationalStatus();
		if (mujoco_ros::ShouldSyncViewerEachFrame(viewer.is_passive_, viewer.auto_sync_, operational_status)) {
			viewer.SyncInFrame(env, false);
		}
		viewer.Render();
		viewer.frame_lease_.reset();

		const auto end = Clock::now();
		render_call_hz.push_back(1.0 / std::chrono::duration<double>(end - start).count());
	}
	return render_call_hz;
}
#endif

void StopViewers(std::vector<std::shared_ptr<Viewer>> &viewers, std::vector<std::thread> &render_threads)
{
	for (auto &viewer : viewers) {
		if (viewer) {
			viewer->exit_request.store(1);
		}
	}
	for (auto &thread : render_threads) {
		if (thread.joinable()) {
			thread.join();
		}
	}
	viewers.clear();
	render_threads.clear();
}

} // namespace

int main(int argc, char **argv)
{
#if MJR_ROS_VERSION == ROS_1
	ros::init(argc, argv, "viewer_connection_benchmark", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
#else
	rclcpp::init(argc, argv);
	auto nh = std::make_shared<rclcpp::Node>("viewer_connection_benchmark");
#endif

#if RENDER_BACKEND != GLFW_BACKEND
	std::cerr << "viewer_connection_benchmark requires GLFW backend\n";
	return 1;
#endif
	if (std::getenv("DISPLAY") == nullptr && std::getenv("WAYLAND_DISPLAY") == nullptr) {
		std::cerr << "viewer_connection_benchmark requires a display\n";
		return 1;
	}

	constexpr int kSamples = 30;
	std::vector<double> physics_step_hz;
	std::vector<double> zero_viewer_step_hz;
	std::vector<double> one_viewer_step_hz;
	std::vector<double> two_viewer_step_hz;
	std::vector<double> zero_viewer_render_call_hz;
	std::vector<double> one_viewer_render_call_hz;
	std::vector<double> two_viewer_render_call_hz;

	try {
#if MJR_ROS_VERSION == ROS_1
		auto env = std::make_unique<MujocoEnvTestWrapper>("", &nh);
#else
		testing::TestNodeHandle test_nh;
		auto env = std::make_unique<MujocoEnvTestWrapper>("", &test_nh);
#endif
		env->StartPhysicsLoop();
		env->StartEventLoop();

		for (int i = 0; i < kSamples; ++i) {
			const auto start = Clock::now();
			env->step(1, false);
			const auto end = Clock::now();
			physics_step_hz.push_back(1.0 / std::chrono::duration<double>(end - start).count());
		}

		for (int viewer_count : { 0, 1, 2 }) {
			std::vector<std::shared_ptr<Viewer>> active_viewers;
			std::vector<std::thread> render_threads;
			std::vector<std::vector<double>> per_viewer_render_call_hz(static_cast<std::size_t>(viewer_count));
			std::atomic_int viewers_ready{ 0 };

			for (int index = 0; index < viewer_count; ++index) {
				auto viewer = std::make_shared<Viewer>(std::make_unique<GlfwAdapter>(), env.get(), true, true);
				active_viewers.push_back(viewer);
				render_threads.emplace_back([viewer, &per_viewer_render_call_hz, index, &viewers_ready]() {
					viewer->RenderLoop([viewer, &per_viewer_render_call_hz, index, &viewers_ready]() {
						per_viewer_render_call_hz[static_cast<std::size_t>(index)] =
						    BenchmarkProductionRenderCalls(*viewer, kSamples);
						viewers_ready.fetch_add(1, std::memory_order_release);
					});
				});
			}

			if (viewer_count > 0 &&
			    !WaitUntil([&viewers_ready,
			                viewer_count]() { return viewers_ready.load(std::memory_order_acquire) >= viewer_count; },
			               std::chrono::seconds(5))) {
				StopViewers(active_viewers, render_threads);
				std::cerr << "viewer_connection_benchmark timed out waiting for viewer readiness\n";
				return 1;
			}

			std::vector<double> &step_bucket = viewer_count == 0 ? zero_viewer_step_hz :
			                                   viewer_count == 1 ? one_viewer_step_hz :
			                                                       two_viewer_step_hz;
			for (int i = 0; i < kSamples; ++i) {
				const auto start = Clock::now();
				env->step(1, false);
				const auto end = Clock::now();
				step_bucket.push_back(1.0 / std::chrono::duration<double>(end - start).count());
			}

			std::vector<double> &render_call_bucket = viewer_count == 0 ? zero_viewer_render_call_hz :
			                                          viewer_count == 1 ? one_viewer_render_call_hz :
			                                                              two_viewer_render_call_hz;
			for (const auto &samples : per_viewer_render_call_hz) {
				render_call_bucket.insert(render_call_bucket.end(), samples.begin(), samples.end());
			}

			StopViewers(active_viewers, render_threads);
		}

		env->shutdown();
	} catch (const std::exception &error) {
		std::cerr << "viewer_connection_benchmark setup failed: " << error.what() << '\n';
		return 1;
	}

	const auto physics_stats     = ComputeStats(physics_step_hz);
	const auto zero_stats        = ComputeStats(zero_viewer_step_hz);
	const auto one_stats         = ComputeStats(one_viewer_step_hz);
	const auto two_stats         = ComputeStats(two_viewer_step_hz);
	const auto zero_render_stats = ComputeStats(zero_viewer_render_call_hz);
	const auto one_render_stats  = ComputeStats(one_viewer_render_call_hz);
	const auto two_render_stats  = ComputeStats(two_viewer_render_call_hz);

	std::cout << "viewer_connection_benchmark samples=" << kSamples << '\n';
	std::cout << "physics_step_hz median=" << physics_stats.median << " variance=" << physics_stats.variance << '\n';
	std::cout << "zero_viewer_step_hz median=" << zero_stats.median << " variance=" << zero_stats.variance << '\n';
	std::cout << "one_viewer_step_hz median=" << one_stats.median << " variance=" << one_stats.variance << '\n';
	std::cout << "two_viewer_step_hz median=" << two_stats.median << " variance=" << two_stats.variance << '\n';
	std::cout << "zero_viewer_render_call_hz median=" << zero_render_stats.median
	          << " variance=" << zero_render_stats.variance << '\n';
	std::cout << "one_viewer_render_call_hz median=" << one_render_stats.median
	          << " variance=" << one_render_stats.variance << '\n';
	std::cout << "two_viewer_render_call_hz median=" << two_render_stats.median
	          << " variance=" << two_render_stats.variance << '\n';

#if MJR_ROS_VERSION == ROS_2
	rclcpp::shutdown();
#else
	ros::shutdown();
#endif
	return 0;
}
