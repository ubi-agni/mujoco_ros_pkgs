#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstring>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include <mujoco_ros/rendering/render_core.hpp>
#include <mujoco_ros/rendering/frame_capacity.hpp>
#include <mujoco_ros/rendering/render_backend_interface.hpp>

namespace mujoco_ros::rendering {
namespace {

RenderStatus PreserveFrameStatus(const FrameStatus &status)
{
	const auto code = status.code == FrameStatusCode::kFrameSlotsExhausted ? RenderStatusCode::kFrameSlotsExhausted :
	                                                                         RenderStatusCode::kFrameUnavailable;
	return RenderStatus::Failure(code, status.message);
}

// Runs operation on a worker thread; returns nullopt on timeout after detaching the worker so
// gtest failure does not block on a hung std::async/future destructor. Shared heap state and
// operation captures (e.g. shared_ptr<RenderCore>) must outlive a timeout detach; a true hang may
// leave the detached worker running until process exit.
struct HangTimeoutState
{
	std::atomic_bool completed{ false };
	FrameStatus result{ FrameStatusCode::kBackendFailure, 0, std::nullopt, FrameGeneration(0), "not run" };
};

std::optional<FrameStatus> RunWithHangTimeout(std::chrono::milliseconds timeout, std::function<FrameStatus()> operation)
{
	auto state = std::make_shared<HangTimeoutState>();
	std::thread worker([state, operation = std::move(operation)]() {
		state->result = operation();
		state->completed.store(true, std::memory_order_release);
	});
	const auto deadline = std::chrono::steady_clock::now() + timeout;
	while (!state->completed.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	if (!state->completed.load()) {
		worker.detach();
		return std::nullopt;
	}
	worker.join();
	return state->result;
}

TEST(RenderBackend, GlfwIsRejectedAsAnOffscreenBackend)
{
	EXPECT_STRNE(CompiledRenderBackendName(), "GLFW");
}

class FakeBackend final : public IRenderBackend
{
public:
	RenderStatus Initialize(const mjModel &, const RenderConfiguration &) override
	{
		if (throw_on_initialize) {
			throw std::runtime_error("injected initialize exception");
		}
		initialize_thread = std::this_thread::get_id();
		++initialize_calls;
		return initialize_status;
	}

	RenderStatus Resize(const RenderConfiguration &) override
	{
		++resize_calls;
		return resize_status;
	}

	RenderStatus Render(const RenderSnapshot &snapshot, const CameraDescriptor &camera, PlaneMask planes,
	                    FrameBoundary &boundary) override
	{
		if (throw_on_render) {
			throw std::runtime_error("injected plane render exception");
		}
		if (throw_context_integrity_on_render) {
			throw ContextIntegrityError("injected context integrity exception");
		}
		{
			std::unique_lock<std::mutex> lock(barrier_mutex);
			if (block_render) {
				render_entered = true;
				barrier.notify_all();
				barrier.wait(lock, [this]() { return release_render; });
			}
		}
		++render_calls;
		rendered_simulation_times.push_back(snapshot.simulation_time_ns);
		rendered_snapshots.push_back(&snapshot);
		rendered_data.push_back(snapshot.data.get());
		rendered_plugin_geometry.push_back(snapshot.plugin_geometry.get());
		rendered_cameras.push_back(camera.id);
		last_planes               = planes;
		last_visual_flag          = camera.visual_options.flags[0];
		RenderStatus last_failure = RenderStatus::Ok();
		const auto stamp = boundary.BeginCapture(snapshot.model_generation, snapshot.simulation_time_ns, camera.id);
		for (const auto plane : { PlaneKind::kRgb, PlaneKind::kDepth, PlaneKind::kSegmentation }) {
			if (!HasPlane(planes, plane)) {
				continue;
			}
			attempted_planes.push_back(plane);
			auto writer = boundary.TryAcquireWriter(boundary.generation(), stamp, plane, camera.layout(plane));
			if (!writer.status().ok()) {
				const auto status = PreserveFrameStatus(writer.status());
				if (IsContextIntegrityFailure(status)) {
					return status;
				}
				last_failure = status;
				continue;
			}
			if (fail_plane && plane == failed_plane) {
				last_failure = RenderStatus::Failure(RenderStatusCode::kFrameUnavailable, "injected plane failure");
				continue;
			}
			if (!writer.Commit().ok()) {
				last_failure = RenderStatus::Failure(RenderStatusCode::kFrameUnavailable, "frame commit failed");
				continue;
			}
			committed_planes.push_back(plane);
		}
		if (!last_failure.ok()) {
			return last_failure;
		}
		return render_status;
	}

	void ShutdownOnRenderThread() override
	{
		shutdown_thread = std::this_thread::get_id();
		++shutdown_calls;
	}

	RenderStatus initialize_status;
	RenderStatus resize_status;
	RenderStatus render_status;
	int initialize_calls = 0;
	int resize_calls     = 0;
	int render_calls     = 0;
	int shutdown_calls   = 0;
	std::thread::id initialize_thread;
	std::thread::id shutdown_thread;
	std::vector<std::int64_t> rendered_simulation_times;
	std::vector<const RenderSnapshot *> rendered_snapshots;
	std::vector<const mjData *> rendered_data;
	std::vector<const std::vector<mjvGeom> *> rendered_plugin_geometry;
	std::vector<CameraId> rendered_cameras;
	PlaneMask last_planes = PlaneMask::kNone;
	std::mutex barrier_mutex;
	std::condition_variable barrier;
	bool block_render                      = false;
	bool render_entered                    = false;
	bool release_render                    = false;
	bool fail_plane                        = false;
	PlaneKind failed_plane                 = PlaneKind::kDepth;
	bool throw_on_initialize               = false;
	bool throw_on_render                   = false;
	bool throw_context_integrity_on_render = false;
	std::vector<PlaneKind> attempted_planes;
	std::vector<PlaneKind> committed_planes;
	int last_visual_flag = 0;
};

std::shared_ptr<const mjModel> EmptyModel()
{
	const char *xml = "<mujoco><worldbody><body/></worldbody></mujoco>";
	mjVFS vfs;
	mj_defaultVFS(&vfs);
	mj_addBufferVFS(&vfs, "inline.xml", xml, std::strlen(xml));
	char error[1024] = {};
	auto *model      = mj_loadXML("inline.xml", &vfs, error, sizeof(error));
	mj_deleteVFS(&vfs);
	if (model == nullptr) {
		throw std::runtime_error(error);
	}
	return std::shared_ptr<const mjModel>(model, mj_deleteModel);
}

TEST(RenderSnapshot, PoolReusesBoundedOwnedDataSlots)
{
	const auto model = EmptyModel();
	auto *source     = mj_makeData(model.get());
	ASSERT_NE(source, nullptr);
	source->time = 1.25;

	SnapshotPool pool;
	const auto make_before   = SnapshotPool::make_data_operations();
	const auto delete_before = SnapshotPool::delete_data_operations();
	pool.Activate(*model, ModelGeneration(1));
	ASSERT_EQ(pool.allocation_count(), SnapshotPool::kCapacity);
	EXPECT_EQ(SnapshotPool::make_data_operations() - make_before, SnapshotPool::kCapacity);

	const auto make_after_warmup   = SnapshotPool::make_data_operations();
	const auto delete_after_warmup = SnapshotPool::delete_data_operations();
	for (int capture = 0; capture < 4; ++capture) {
		const auto result = pool.Acquire(*model, *source, ModelGeneration(1));
		ASSERT_TRUE(result.ok()) << result.message;
		EXPECT_DOUBLE_EQ(result.data->time, source->time);
	}

	EXPECT_EQ(pool.allocation_count(), SnapshotPool::kCapacity);
	EXPECT_EQ(pool.copy_count(), 4U);
	EXPECT_EQ(pool.active_lease_count(), 0U);
	EXPECT_EQ(SnapshotPool::make_data_operations(), make_after_warmup);
	EXPECT_EQ(SnapshotPool::delete_data_operations(), delete_after_warmup);
	EXPECT_EQ(SnapshotPool::delete_data_operations(), delete_before);
	mj_deleteData(source);
}

TEST(RenderSnapshot, PoolDeletesAllocatedDataWhenOwnershipInsertionFails)
{
	const auto model = EmptyModel();
	SnapshotPool pool;
	const auto make_before   = SnapshotPool::make_data_operations();
	const auto delete_before = SnapshotPool::delete_data_operations();

	SnapshotPool::ThrowAfterDataAllocationForTest();
	EXPECT_THROW(pool.Activate(*model, ModelGeneration(1)), std::runtime_error);

	EXPECT_EQ(SnapshotPool::make_data_operations(), make_before + 1);
	EXPECT_EQ(SnapshotPool::delete_data_operations(), delete_before + 1);
	EXPECT_EQ(pool.allocation_count(), 0U);
}

TEST(RenderSnapshot, PoolExhaustionDropsWithoutWaiting)
{
	const auto model = EmptyModel();
	auto *source     = mj_makeData(model.get());
	ASSERT_NE(source, nullptr);
	SnapshotPool pool;
	pool.Activate(*model, ModelGeneration(1));

	auto first  = pool.Acquire(*model, *source, ModelGeneration(1));
	auto second = pool.Acquire(*model, *source, ModelGeneration(1));
	ASSERT_TRUE(first.ok());
	ASSERT_TRUE(second.ok());
	const auto exhausted = pool.Acquire(*model, *source, ModelGeneration(1));
	EXPECT_EQ(exhausted.code, SnapshotPool::AcquireCode::kExhausted);
	EXPECT_NE(exhausted.message.find("without waiting"), std::string::npos);

	first.data.reset();
	const auto reused = pool.Acquire(*model, *source, ModelGeneration(1));
	EXPECT_TRUE(reused.ok()) << reused.message;
	mj_deleteData(source);
}

TEST(RenderSnapshot, PoolRetiresGenerationsWithoutInvalidatingOldLease)
{
	const auto old_model = EmptyModel();
	const auto new_model = EmptyModel();
	auto *old_source     = mj_makeData(old_model.get());
	auto *new_source     = mj_makeData(new_model.get());
	ASSERT_NE(old_source, nullptr);
	ASSERT_NE(new_source, nullptr);
	old_source->time = 1.0;
	new_source->time = 2.0;

	SnapshotPool pool;
	pool.Activate(*old_model, ModelGeneration(1));
	auto old_lease = pool.Acquire(*old_model, *old_source, ModelGeneration(1));
	ASSERT_TRUE(old_lease.ok());
	const auto *old_data = old_lease.data.get();

	pool.Activate(*new_model, ModelGeneration(2));
	EXPECT_EQ(pool.active_generation(), std::optional<ModelGeneration>(ModelGeneration(2)));
	const auto new_lease = pool.Acquire(*new_model, *new_source, ModelGeneration(2));
	ASSERT_TRUE(new_lease.ok());
	EXPECT_NE(new_lease.data.get(), old_data);
	EXPECT_DOUBLE_EQ(old_lease.data->time, 1.0);
	EXPECT_DOUBLE_EQ(new_lease.data->time, 2.0);

	mj_deleteData(old_source);
	mj_deleteData(new_source);
}

TEST(RenderCore, BackendCreationAndDestructionStayOnRenderThread)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *backend_ptr = backend.get();
	RenderCore core(std::move(backend));
	const auto caller_thread = std::this_thread::get_id();
	const CameraDescriptor camera{ CameraId(1), "affinity", 8, 8, PlaneMask::kRgb };
	ASSERT_TRUE(core.Reconfigure(ModelGeneration(1), FrameGeneration(1), FrameLayout(8, 8), { camera }).ok());
	core.RegisterContinuousConsumer("affinity-test", camera.id);

	const auto model = EmptyModel();
	RenderSnapshot snapshot;
	snapshot.model_generation   = ModelGeneration(1);
	snapshot.simulation_time_ns = 1;
	snapshot.model              = model;
	snapshot.data               = std::shared_ptr<mjData>(mj_makeData(model.get()), mj_deleteData);
	ASSERT_NE(snapshot.data, nullptr);
	mj_forward(model.get(), snapshot.data.get());
	const auto plan = core.EvaluateDemand(std::chrono::nanoseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(std::move(snapshot), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	core.Shutdown();

	ASSERT_EQ(backend_ptr->initialize_calls, 1);
	ASSERT_EQ(backend_ptr->shutdown_calls, 1);
	EXPECT_EQ(backend_ptr->initialize_thread, backend_ptr->shutdown_thread);
	EXPECT_NE(backend_ptr->initialize_thread, caller_thread);
}

} // namespace

void ConfigureCore(RenderCore &core, CameraDescriptor camera, ModelGeneration model_generation = ModelGeneration(1),
                   FrameGeneration frame_generation = FrameGeneration(1))
{
	ASSERT_TRUE(
	    core.Reconfigure(model_generation, frame_generation, FrameLayout(camera.width, camera.height), { camera }).ok());
}

RenderSnapshot Snapshot(ModelGeneration generation, std::int64_t simulation_time_ns = 0)
{
	RenderSnapshot snapshot;
	snapshot.model_generation   = generation;
	snapshot.simulation_time_ns = simulation_time_ns;
	snapshot.model              = EmptyModel();
	snapshot.data               = std::shared_ptr<mjData>(mj_makeData(snapshot.model.get()), mj_deleteData);
	return snapshot;
}

std::shared_ptr<const mjModel> ColoredGeometryModel()
{
	const char *xml = R"(
<mujoco>
  <visual>
    <global offwidth="128" offheight="64"/>
    <headlight ambient="0.8 0.8 0.8" diffuse="0.8 0.8 0.8" specular="0 0 0"/>
  </visual>
  <worldbody>
    <geom type="sphere" pos="-0.5 0 0" size="0.25" rgba="1 0 0 1"/>
    <camera name="camera" pos="0 -3 0" euler="90 0 0" fovy="35"/>
  </worldbody>
</mujoco>)";
	mjVFS vfs;
	mj_defaultVFS(&vfs);
	mj_addBufferVFS(&vfs, "colored_geometry.xml", xml, std::strlen(xml));
	char error[1024] = {};
	auto *model      = mj_loadXML("colored_geometry.xml", &vfs, error, sizeof(error));
	mj_deleteVFS(&vfs);
	if (model == nullptr) {
		throw std::runtime_error(error);
	}
	return std::shared_ptr<const mjModel>(model, mj_deleteModel);
}

mjvGeom GreenPluginGeometry()
{
	mjvGeom geometry;
	const mjtNum size[3]        = { 0.25, 0.25, 0.25 };
	const mjtNum position[3]    = { 0.5, 0.0, 0.0 };
	const mjtNum orientation[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
	const float color[4]        = { 0.0F, 1.0F, 0.0F, 1.0F };
	mjv_initGeom(&geometry, mjGEOM_SPHERE, size, position, orientation, color);
	return geometry;
}

RenderStatus RenderWithRealBackend(const std::vector<mjvGeom> &plugin_geometry, FrameBoundary &boundary,
                                   std::shared_ptr<const mjModel> model  = {},
                                   std::shared_ptr<mjData> snapshot_data = {}, PlaneMask planes = PlaneMask::kRgb)
{
	auto backend = CreateRenderBackend();
	if (!model) {
		model = ColoredGeometryModel();
	}
	RenderConfiguration configuration;
	configuration.generation   = FrameGeneration(1);
	configuration.frame_layout = FrameLayout(128, 64);
	const auto initialized     = backend->Initialize(*model, configuration);
	if (!initialized.ok()) {
		return initialized;
	}
	RenderSnapshot snapshot;
	snapshot.model_generation = ModelGeneration(1);
	snapshot.model            = model;
	if (!snapshot_data) {
		snapshot_data = std::shared_ptr<mjData>(mj_makeData(model.get()), mj_deleteData);
		mj_forward(model.get(), snapshot_data.get());
	}
	snapshot.data            = std::move(snapshot_data);
	snapshot.plugin_geometry = std::make_shared<const std::vector<mjvGeom>>(plugin_geometry);
	CameraDescriptor camera{ CameraId(1), "camera", 128, 64, planes };
	const auto result = backend->Render(snapshot, camera, planes, boundary);
	backend->ShutdownOnRenderThread();
	return result;
}

TEST(RenderSnapshot, PooledDataRendersLikeFreshCopy)
{
	const auto model = ColoredGeometryModel();
	auto *source     = mj_makeData(model.get());
	ASSERT_NE(source, nullptr);
	mj_forward(model.get(), source);

	SnapshotPool pool;
	pool.Activate(*model, ModelGeneration(1));
	auto pooled = pool.Acquire(*model, *source, ModelGeneration(1));
	ASSERT_TRUE(pooled.ok()) << pooled.message;

	const auto planes      = PlaneMask::kRgb | PlaneMask::kDepth | PlaneMask::kSegmentation;
	const auto rgb_bytes   = PlaneLayout::Rgb8(128, 64).byte_length;
	const auto depth_bytes = PlaneLayout::Depth32F(128, 64).byte_length;
	const auto seg_bytes   = PlaneLayout::Segmentation8(128, 64).byte_length;
	const auto frame_bytes = rgb_bytes + depth_bytes + seg_bytes;
	// One capture commits three independent planes; reserve one slot per plane.
	FrameBoundary pooled_boundary(3, frame_bytes * 2U);
	FrameBoundary fresh_boundary(3, frame_bytes * 2U);
	ASSERT_TRUE(pooled_boundary.Reconfigure(FrameGeneration(1), FrameLayout(128, 64)).ok());
	ASSERT_TRUE(fresh_boundary.Reconfigure(FrameGeneration(1), FrameLayout(128, 64)).ok());
	const auto pooled_status = RenderWithRealBackend({}, pooled_boundary, model, pooled.data, planes);
	if (pooled_status.code == RenderStatusCode::kBackendUnavailable) {
		mj_deleteData(source);
		GTEST_SKIP() << pooled_status.message;
	}
	ASSERT_TRUE(pooled_status.ok()) << pooled_status.message;
	const auto fresh_status = RenderWithRealBackend({}, fresh_boundary, model, {}, planes);
	ASSERT_TRUE(fresh_status.ok()) << fresh_status.message;
	auto pooled_frame = pooled_boundary.AcquireLatest(PlaneKind::kRgb);
	auto fresh_frame  = fresh_boundary.AcquireLatest(PlaneKind::kRgb);
	ASSERT_TRUE(pooled_frame.has_value());
	ASSERT_TRUE(fresh_frame.has_value());
	EXPECT_EQ(pooled_frame->bytes(), fresh_frame->bytes());
	auto pooled_depth = pooled_boundary.AcquireLatest(PlaneKind::kDepth);
	auto fresh_depth  = fresh_boundary.AcquireLatest(PlaneKind::kDepth);
	ASSERT_TRUE(pooled_depth.has_value());
	ASSERT_TRUE(fresh_depth.has_value());
	EXPECT_EQ(pooled_depth->bytes(), fresh_depth->bytes());
	auto pooled_seg = pooled_boundary.AcquireLatest(PlaneKind::kSegmentation);
	auto fresh_seg  = fresh_boundary.AcquireLatest(PlaneKind::kSegmentation);
	ASSERT_TRUE(pooled_seg.has_value());
	ASSERT_TRUE(fresh_seg.has_value());
	EXPECT_EQ(pooled_seg->bytes(), fresh_seg->bytes());
	mj_deleteData(source);
}

TEST(RenderCore, OneRenderServesMultipleSelectedConsumers)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	auto first      = core.RegisterContinuousConsumer("first");
	auto second     = core.RegisterContinuousConsumer("second");
	const auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), CameraId(1));
	ASSERT_EQ(plan.consumers.size(), 2U);
	EXPECT_EQ(first, plan.consumers[0]);
	EXPECT_EQ(second, plan.consumers[1]);

	RenderSnapshot snapshot;
	snapshot.model_generation = ModelGeneration(1);
	snapshot.model            = EmptyModel();
	snapshot.data             = std::shared_ptr<mjData>(mj_makeData(snapshot.model.get()), mj_deleteData);
	ASSERT_TRUE(core.SubmitSnapshot(std::move(snapshot), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());

	EXPECT_EQ(raw_backend->render_calls, 1);
	auto ros_lease = core.AcquireLatest(camera.id, PlaneKind::kRgb);
	ASSERT_TRUE(ros_lease.has_value());
	EXPECT_EQ(ros_lease->camera_id(), camera.id);
	EXPECT_EQ(ros_lease->capture_id(), core.AcquireLatest(camera.id, PlaneKind::kRgb)->capture_id());
	core.Shutdown();
	EXPECT_EQ(raw_backend->shutdown_calls, 1);
}

TEST(RenderCore, FourCameraCaptureUsesSharedOwnedSnapshotStorage)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	RenderCore core(std::move(backend));
	std::vector<CameraDescriptor> cameras;
	for (std::uint8_t id = 1; id <= 4; ++id) {
		CameraDescriptor camera;
		camera.id     = CameraId(id);
		camera.name   = "camera_" + std::to_string(id);
		camera.width  = 2;
		camera.height = 2;
		camera.planes = PlaneMask::kRgb;
		cameras.push_back(std::move(camera));
	}
	ASSERT_TRUE(core.Reconfigure(ModelGeneration(1), FrameGeneration(1), FrameLayout(2, 2), cameras).ok());

	std::vector<ConsumerId> consumers;
	for (const auto &camera : cameras) {
		consumers.push_back(core.RegisterContinuousConsumer("consumer_" + std::to_string(camera.id), camera.id));
	}
	const auto model = EmptyModel();
	auto data        = std::shared_ptr<mjData>(mj_makeData(model.get()), mj_deleteData);
	ASSERT_NE(data, nullptr);
	auto plugin_geometry         = std::make_shared<const std::vector<mjvGeom>>(1);
	auto snapshot                = std::make_shared<RenderSnapshot>();
	snapshot->model_generation   = ModelGeneration(1);
	snapshot->simulation_time_ns = 1;
	snapshot->model              = model;
	snapshot->data               = data;
	snapshot->plugin_geometry    = plugin_geometry;

	for (const auto &camera : cameras) {
		const auto plan = core.EvaluateDemand(std::chrono::nanoseconds(1), camera.id);
		ASSERT_EQ(plan.consumers.size(), 1U);
		ASSERT_TRUE(core.SubmitSnapshot(snapshot, plan).ok());
		ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	}

	ASSERT_EQ(raw_backend->render_calls, 4);
	ASSERT_EQ(raw_backend->rendered_snapshots.size(), 4U);
	ASSERT_EQ(raw_backend->rendered_data.size(), 4U);
	ASSERT_EQ(raw_backend->rendered_plugin_geometry.size(), 4U);
	for (std::size_t index = 1; index < 4; ++index) {
		EXPECT_EQ(raw_backend->rendered_snapshots[index], raw_backend->rendered_snapshots.front());
		EXPECT_EQ(raw_backend->rendered_data[index], raw_backend->rendered_data.front());
		EXPECT_EQ(raw_backend->rendered_plugin_geometry[index], raw_backend->rendered_plugin_geometry.front());
		EXPECT_NE(raw_backend->rendered_cameras[index], raw_backend->rendered_cameras.front());
	}
	core.Shutdown();
}

TEST(RenderCore, FourCameraRgbLatestAvailableWithComputedCapacity)
{
	auto backend = std::make_unique<FakeBackend>();
	std::vector<CameraDescriptor> cameras;
	for (std::uint8_t id = 1; id <= 4; ++id) {
		CameraDescriptor camera;
		camera.id     = CameraId(id);
		camera.name   = "camera_" + std::to_string(id);
		camera.width  = 2;
		camera.height = 2;
		camera.planes = PlaneMask::kRgb;
		cameras.push_back(std::move(camera));
	}
	const auto capacity = ComputeFrameSlotCapacity(cameras);
	RenderCore core(std::move(backend), capacity);
	ASSERT_TRUE(core.Reconfigure(ModelGeneration(1), FrameGeneration(1), FrameLayout(2, 2), cameras).ok());

	for (const auto &camera : cameras) {
		core.RegisterContinuousConsumer("consumer_" + std::to_string(camera.id), camera.id);
	}
	const auto model = EmptyModel();
	auto data        = std::shared_ptr<mjData>(mj_makeData(model.get()), mj_deleteData);
	ASSERT_NE(data, nullptr);
	auto plugin_geometry         = std::make_shared<const std::vector<mjvGeom>>(1);
	auto snapshot                = std::make_shared<RenderSnapshot>();
	snapshot->model_generation   = ModelGeneration(1);
	snapshot->simulation_time_ns = 1;
	snapshot->model              = model;
	snapshot->data               = data;
	snapshot->plugin_geometry    = plugin_geometry;

	for (const auto &camera : cameras) {
		const auto plan = core.EvaluateDemand(std::chrono::nanoseconds(1), camera.id);
		ASSERT_TRUE(core.SubmitSnapshot(snapshot, plan).ok());
		ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	}

	for (const auto &camera : cameras) {
		EXPECT_TRUE(core.AcquireLatest(camera.id, PlaneKind::kRgb).has_value())
		    << "camera " << camera.id << " latest RGB frame must remain available under computed capacity";
	}
	core.Shutdown();
}

TEST(RenderCore, ThreeSlotBudgetCannotRetainFourCameraLatestFrames)
{
	auto backend = std::make_unique<FakeBackend>();
	std::vector<CameraDescriptor> cameras;
	for (std::uint8_t id = 1; id <= 4; ++id) {
		CameraDescriptor camera;
		camera.id     = CameraId(id);
		camera.name   = "camera_" + std::to_string(id);
		camera.width  = 2;
		camera.height = 2;
		camera.planes = PlaneMask::kRgb;
		cameras.push_back(std::move(camera));
	}
	RenderCore core(std::move(backend), 3);
	ASSERT_TRUE(core.Reconfigure(ModelGeneration(1), FrameGeneration(1), FrameLayout(2, 2), cameras).ok());

	for (const auto &camera : cameras) {
		core.RegisterContinuousConsumer("consumer_" + std::to_string(camera.id), camera.id);
	}
	const auto model = EmptyModel();
	auto data        = std::shared_ptr<mjData>(mj_makeData(model.get()), mj_deleteData);
	ASSERT_NE(data, nullptr);
	auto snapshot                = std::make_shared<RenderSnapshot>();
	snapshot->model_generation   = ModelGeneration(1);
	snapshot->simulation_time_ns = 1;
	snapshot->model              = model;
	snapshot->data               = data;
	snapshot->plugin_geometry    = std::make_shared<const std::vector<mjvGeom>>(1);

	for (const auto &camera : cameras) {
		const auto plan = core.EvaluateDemand(std::chrono::nanoseconds(1), camera.id);
		ASSERT_TRUE(core.SubmitSnapshot(snapshot, plan).ok());
		ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	}

	std::size_t available_latest = 0;
	for (const auto &camera : cameras) {
		if (core.AcquireLatest(camera.id, PlaneKind::kRgb).has_value()) {
			++available_latest;
		}
	}
	EXPECT_LT(available_latest, cameras.size())
	    << "hard-coded three-slot budget must not retain latest RGB for every camera";
	core.Shutdown();
}

TEST(RenderCore, RgbDepthSegmentationHistoryWithinComputedCapacity)
{
	auto backend                  = std::make_unique<FakeBackend>();
	constexpr std::size_t history = 3;
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2,
		                      PlaneMask::kRgb | PlaneMask::kDepth | PlaneMask::kSegmentation };
	const auto capacity = ComputeFrameSlotCapacity({ camera }, { { camera.id, history } });
	RenderCore core(std::move(backend), capacity);
	ASSERT_TRUE(core.Reconfigure(ModelGeneration(1), FrameGeneration(1), FrameLayout(2, 2), { camera }).ok());
	core.RegisterContinuousConsumer("python", camera.id);

	const auto model = EmptyModel();
	auto data        = std::shared_ptr<mjData>(mj_makeData(model.get()), mj_deleteData);
	ASSERT_NE(data, nullptr);
	auto snapshot              = std::make_shared<RenderSnapshot>();
	snapshot->model_generation = ModelGeneration(1);
	snapshot->model            = model;
	snapshot->data             = data;
	snapshot->plugin_geometry  = std::make_shared<const std::vector<mjvGeom>>(1);

	for (std::size_t frame = 0; frame < history; ++frame) {
		snapshot->simulation_time_ns = static_cast<std::int64_t>(frame + 1);
		const auto plan              = core.EvaluateDemand(std::chrono::nanoseconds(frame + 1), camera.id);
		ASSERT_TRUE(core.SubmitSnapshot(snapshot, plan).ok());
		ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	}

	for (const auto plane : { PlaneKind::kRgb, PlaneKind::kDepth, PlaneKind::kSegmentation }) {
		EXPECT_EQ(core.AcquireRecent(camera.id, plane, history).size(), history);
	}
	core.Shutdown();
}

TEST(RenderCore, RetiredLeaseConsumesCapacityAcrossReload)
{
	const auto old_layout = PlaneLayout::Rgb8(4, 4);
	FrameBoundary boundary(2, old_layout.byte_length + PlaneLayout::Rgb8(8, 8).byte_length - 1);
	ASSERT_TRUE(boundary.Reconfigure(FrameGeneration(1), FrameLayout(4, 4)).ok());
	auto writer = boundary.TryAcquireWriter(FrameGeneration(1), PlaneKind::kRgb, old_layout);
	ASSERT_TRUE(writer.Commit().ok());
	auto lease = writer.Acquire();
	ASSERT_TRUE(lease.has_value());

	const auto rejected = boundary.Reconfigure(FrameGeneration(2), FrameLayout(8, 8));
	EXPECT_EQ(rejected.code, FrameStatusCode::kGenerationCapacityExhausted);
	EXPECT_TRUE(lease->valid());
}

TEST(RenderCore, ForcedPythonDemandDoesNotAdvanceRosCadence)
{
	RenderCore core(std::make_unique<FakeBackend>());
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	const auto ros    = core.RegisterCadencedConsumer("ros/camera", std::chrono::milliseconds(10), camera.id);
	const auto python = core.RegisterOneShotConsumer("python/camera", camera.id);
	core.RequestOneShot(python);

	const auto forced_python = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_EQ(forced_python.consumers.size(), 1U);
	EXPECT_EQ(forced_python.consumers.front(), python);
	core.MarkDelivered(forced_python, python);

	const auto before_ros_cadence = core.EvaluateDemand(std::chrono::milliseconds(5), camera.id);
	EXPECT_TRUE(before_ros_cadence.consumers.empty());

	const auto ros_due = core.EvaluateDemand(std::chrono::milliseconds(10), camera.id);
	ASSERT_EQ(ros_due.consumers.size(), 1U);
	EXPECT_EQ(ros_due.consumers.front(), ros);
	core.MarkDelivered(ros_due, ros);
	core.Shutdown();
}

TEST(RenderCore, UnregisterKeepsIndependentConsumerDemand)
{
	RenderCore core(std::make_unique<FakeBackend>());
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	const auto ros    = core.RegisterContinuousConsumer("ros/camera", camera.id);
	const auto python = core.RegisterContinuousConsumer("python/camera", camera.id);

	core.UnregisterConsumer(python);
	const auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);

	ASSERT_EQ(plan.consumers.size(), 1U);
	EXPECT_EQ(plan.consumers.front(), ros);
	core.Shutdown();
}

TEST(RenderCore, CameraVisualOptionsUpdateReachesNextBackendRender)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	const auto consumer   = core.RegisterContinuousConsumer("python/camera", camera.id);
	const auto first_plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_EQ(first_plan.consumers.front(), consumer);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), first_plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	core.MarkDelivered(first_plan, consumer);

	auto options     = camera.visual_options;
	options.flags[0] = 1;
	ASSERT_TRUE(core.UpdateCameraVisualOptions(camera.id, options).ok());
	const auto second_plan = core.EvaluateDemand(std::chrono::milliseconds(2), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 2), second_plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	EXPECT_EQ(raw_backend->last_visual_flag, 1);
	core.Shutdown();
}

TEST(RenderCore, ReloadDuringRenderQuiescesBeforeReconfigure)
{
	auto backend              = std::make_unique<FakeBackend>();
	auto *raw_backend         = backend.get();
	raw_backend->block_render = true;
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	auto consumer = core.RegisterContinuousConsumer("consumer");
	auto plan     = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	{
		std::unique_lock<std::mutex> lock(raw_backend->barrier_mutex);
		ASSERT_TRUE(raw_backend->barrier.wait_for(lock, std::chrono::seconds(1),
		                                          [raw_backend]() { return raw_backend->render_entered; }));
	}
	core.StopAcceptingSnapshots();
	core.RequestCancelRenderTurn();
	{
		std::lock_guard<std::mutex> lock(raw_backend->barrier_mutex);
		raw_backend->release_render = true;
	}
	raw_backend->barrier.notify_all();
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	EXPECT_TRUE(core.Reconfigure(ModelGeneration(2), FrameGeneration(2), FrameLayout(2, 2), { camera }).ok());
	EXPECT_EQ(raw_backend->render_calls, 1);
	EXPECT_THROW(core.MarkDelivered(plan, consumer), std::invalid_argument);
}

TEST(RenderCore, FrameSlotExhaustionDropsOutputWithoutPoisoningCore)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	RenderCore core(std::move(backend), 1, FrameBoundary::kDefaultMaxBytes);
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");

	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 1), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	auto retained_lease = core.AcquireLatest(camera.id, PlaneKind::kRgb);
	ASSERT_TRUE(retained_lease.has_value());

	plan = core.EvaluateDemand(std::chrono::milliseconds(2), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 2), plan).ok());
	const auto dropped = core.FinishOrCancelRenderTurn();
	EXPECT_EQ(dropped.code, FrameStatusCode::kFrameSlotsExhausted);
	EXPECT_NE(dropped.message.find("frame slot"), std::string::npos);
	EXPECT_TRUE(core.Status().ok());

	retained_lease.reset();
	plan = core.EvaluateDemand(std::chrono::milliseconds(3), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 3), plan).ok());
	EXPECT_TRUE(core.FinishOrCancelRenderTurn().ok());
	EXPECT_EQ(raw_backend->render_calls, 3);
	EXPECT_TRUE(core.AcquireLatest(camera.id, PlaneKind::kRgb).has_value());
	core.Shutdown();
}

TEST(RenderCore, WaitForSlotPolicyBlocksUntilFrameLeaseIsReleased)
{
	RenderCore core(std::make_unique<FakeBackend>(), 1, FrameBoundary::kDefaultMaxBytes);
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 1), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	auto retained = core.AcquireLatest(camera.id, PlaneKind::kRgb);
	ASSERT_TRUE(retained.has_value());
	ASSERT_TRUE(core.SetRenderBackpressurePolicy(RenderBackpressurePolicy::kWaitForSlot).ok());

	std::promise<void> entered;
	auto entered_future = entered.get_future();
	core.SetCapacityWaitObserverForTesting([&entered]() { entered.set_value(); });
	std::promise<FrameStatus> result;
	auto result_future = result.get_future();
	std::thread waiter([&] { result.set_value(core.WaitForFrameCapacity(plan)); });
	ASSERT_EQ(entered_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	retained.reset();
	ASSERT_EQ(result_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	EXPECT_TRUE(result_future.get().ok());
	waiter.join();
	core.Shutdown();
}

TEST(RenderCore, InvalidEnumPolicyIsRejectedAtomically)
{
	RenderCore core(std::make_unique<FakeBackend>());

	EXPECT_EQ(core.GetRenderBackpressurePolicy(), RenderBackpressurePolicy::kDrop);
	const auto rejected = core.SetRenderBackpressurePolicy(static_cast<RenderBackpressurePolicy>(99));

	EXPECT_EQ(rejected.code, FrameStatusCode::kInvalidPolicy);
	EXPECT_EQ(core.GetRenderBackpressurePolicy(), RenderBackpressurePolicy::kDrop);
	core.Shutdown();
}

TEST(RenderCore, ChangingWaitPolicyToDropInObserverWindowCancelsCapacityWait)
{
	RenderCore core(std::make_unique<FakeBackend>(), 1, FrameBoundary::kDefaultMaxBytes);
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 1), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	auto retained = core.AcquireLatest(camera.id, PlaneKind::kRgb);
	ASSERT_TRUE(retained.has_value());
	ASSERT_TRUE(core.SetRenderBackpressurePolicy(RenderBackpressurePolicy::kWaitForSlot).ok());

	std::promise<void> entered;
	auto entered_future = entered.get_future();
	core.SetCapacityWaitObserverForTesting([&core, &entered]() {
		entered.set_value();
		ASSERT_TRUE(core.SetRenderBackpressurePolicy(RenderBackpressurePolicy::kDrop).ok());
	});
	std::promise<FrameStatus> result;
	auto result_future = result.get_future();
	std::thread waiter([&] { result.set_value(core.WaitForFrameCapacity(plan)); });
	ASSERT_EQ(entered_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	ASSERT_EQ(result_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	EXPECT_EQ(result_future.get().code, FrameStatusCode::kStopped);
	waiter.join();
	core.Shutdown();
}

TEST(RenderCore, ShutdownCancelsCapacityWait)
{
	RenderCore core(std::make_unique<FakeBackend>(), 1, FrameBoundary::kDefaultMaxBytes);
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 1), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	auto retained = core.AcquireLatest(camera.id, PlaneKind::kRgb);
	ASSERT_TRUE(retained.has_value());
	ASSERT_TRUE(core.SetRenderBackpressurePolicy(RenderBackpressurePolicy::kWaitForSlot).ok());

	std::promise<void> entered;
	auto entered_future = entered.get_future();
	core.SetCapacityWaitObserverForTesting([&entered]() { entered.set_value(); });
	std::promise<FrameStatus> result;
	auto result_future = result.get_future();
	std::thread waiter([&] { result.set_value(core.WaitForFrameCapacity(plan)); });
	ASSERT_EQ(entered_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	core.Shutdown();
	ASSERT_EQ(result_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	EXPECT_EQ(result_future.get().code, FrameStatusCode::kStopped);
	waiter.join();
}

TEST(RenderCore, RenderCancellationCancelsCapacityWait)
{
	RenderCore core(std::make_unique<FakeBackend>(), 1, FrameBoundary::kDefaultMaxBytes);
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 1), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	auto retained = core.AcquireLatest(camera.id, PlaneKind::kRgb);
	ASSERT_TRUE(retained.has_value());
	ASSERT_TRUE(core.SetRenderBackpressurePolicy(RenderBackpressurePolicy::kWaitForSlot).ok());

	std::promise<void> entered;
	auto entered_future = entered.get_future();
	core.SetCapacityWaitObserverForTesting([&entered]() { entered.set_value(); });
	std::promise<FrameStatus> result;
	auto result_future = result.get_future();
	std::thread waiter([&] { result.set_value(core.WaitForFrameCapacity(plan)); });
	ASSERT_EQ(entered_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	core.RequestCancelRenderTurn();
	ASSERT_EQ(result_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	EXPECT_EQ(result_future.get().code, FrameStatusCode::kStopped);
	waiter.join();
	core.Shutdown();
}

TEST(RenderCore, ReloadAdmissionClosureCancelsCapacityWait)
{
	RenderCore core(std::make_unique<FakeBackend>(), 1, FrameBoundary::kDefaultMaxBytes);
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 1), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	auto retained = core.AcquireLatest(camera.id, PlaneKind::kRgb);
	ASSERT_TRUE(retained.has_value());
	ASSERT_TRUE(core.SetRenderBackpressurePolicy(RenderBackpressurePolicy::kWaitForSlot).ok());

	std::promise<void> entered;
	auto entered_future = entered.get_future();
	core.SetCapacityWaitObserverForTesting([&entered]() { entered.set_value(); });
	std::promise<FrameStatus> result;
	auto result_future = result.get_future();
	std::thread waiter([&] { result.set_value(core.WaitForFrameCapacity(plan)); });
	ASSERT_EQ(entered_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	core.StopAcceptingSnapshots();
	ASSERT_EQ(result_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	EXPECT_EQ(result_future.get().code, FrameStatusCode::kStopped);
	waiter.join();
	core.Shutdown();
}

TEST(RenderCore, DirectReconfigureWaitsForInFlightRenderBeforeChangingGeneration)
{
	auto backend              = std::make_unique<FakeBackend>();
	auto *raw_backend         = backend.get();
	raw_backend->block_render = true;
	RenderCore core(std::move(backend));
	CameraDescriptor old_camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, old_camera);
	core.RegisterContinuousConsumer("consumer");
	const auto old_plan = core.EvaluateDemand(std::chrono::milliseconds(1), old_camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 1), old_plan).ok());
	{
		std::unique_lock<std::mutex> lock(raw_backend->barrier_mutex);
		ASSERT_TRUE(raw_backend->barrier.wait_for(lock, std::chrono::seconds(1),
		                                          [raw_backend]() { return raw_backend->render_entered; }));
	}

	CameraDescriptor new_camera{ CameraId(1), "camera", 4, 4, PlaneMask::kRgb };
	std::promise<FrameStatus> reconfigure_promise;
	auto reconfigure_future = reconfigure_promise.get_future();
	std::thread reconfigure_thread([&] {
		reconfigure_promise.set_value(
		    core.Reconfigure(ModelGeneration(1), FrameGeneration(2), FrameLayout(4, 4), { new_camera }));
	});
	EXPECT_EQ(reconfigure_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

	{
		std::lock_guard<std::mutex> lock(raw_backend->barrier_mutex);
		raw_backend->release_render = true;
	}
	raw_backend->barrier.notify_all();
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	ASSERT_EQ(reconfigure_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
	EXPECT_TRUE(reconfigure_future.get().ok());
	reconfigure_thread.join();

	EXPECT_EQ(core.frames().generation(), FrameGeneration(2));
	EXPECT_FALSE(core.AcquireLatest(new_camera.id, PlaneKind::kRgb).has_value());
	EXPECT_EQ(raw_backend->render_calls, 1);
	core.Shutdown();
}

TEST(RenderBackend, SegmentationWriterFailureRestoresVisualState)
{
	const FrameLayout frame_layout(128, 64);
	FrameBoundary boundary(1, frame_layout.slot_byte_length);
	ASSERT_TRUE(boundary.Reconfigure(FrameGeneration(1), frame_layout, 1).ok());
	const auto model = ColoredGeometryModel();
	RenderConfiguration configuration;
	configuration.generation   = FrameGeneration(1);
	configuration.frame_layout = FrameLayout(128, 64);
	auto backend               = CreateRenderBackend();
	const auto initialized     = backend->Initialize(*model, configuration);
	if (initialized.code == RenderStatusCode::kBackendUnavailable) {
		GTEST_SKIP() << initialized.message;
	}
	ASSERT_TRUE(initialized.ok()) << initialized.message;

	RenderSnapshot snapshot;
	snapshot.model_generation = ModelGeneration(1);
	snapshot.model            = model;
	snapshot.data             = std::shared_ptr<mjData>(mj_makeData(model.get()), mj_deleteData);
	mj_forward(model.get(), snapshot.data.get());
	CameraDescriptor camera{ CameraId(1), "camera", 128, 64, PlaneMask::kRgb | PlaneMask::kSegmentation };
	ASSERT_TRUE(backend->Render(snapshot, camera, PlaneMask::kRgb, boundary).ok());
	auto retained = boundary.AcquireLatest(PlaneKind::kRgb);
	ASSERT_TRUE(retained.has_value());

	const auto dropped = backend->Render(snapshot, camera, PlaneMask::kSegmentation, boundary);
	EXPECT_EQ(dropped.code, RenderStatusCode::kFrameSlotsExhausted);
	retained.reset();
	ASSERT_TRUE(backend->Render(snapshot, camera, PlaneMask::kRgb, boundary).ok());
	auto after_failure = boundary.AcquireLatest(PlaneKind::kRgb);
	ASSERT_TRUE(after_failure.has_value());

	FrameBoundary expected_boundary(1, frame_layout.slot_byte_length);
	ASSERT_TRUE(expected_boundary.Reconfigure(FrameGeneration(1), frame_layout, 1).ok());
	const auto expected = RenderWithRealBackend({}, expected_boundary);
	ASSERT_TRUE(expected.ok()) << expected.message;
	auto expected_lease = expected_boundary.AcquireLatest(PlaneKind::kRgb);
	ASSERT_TRUE(expected_lease.has_value());
	EXPECT_EQ(after_failure->bytes(), expected_lease->bytes());
	backend->ShutdownOnRenderThread();
}

TEST(RenderCore, QueuedSnapshotIsCancelledDuringShutdown)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	core.Shutdown();
	EXPECT_EQ(raw_backend->render_calls, 0);
}

TEST(RenderCore, QueuedTurnRejectsReplacementWithoutLosingAcceptedSnapshot)
{
	auto backend              = std::make_unique<FakeBackend>();
	auto *raw_backend         = backend.get();
	raw_backend->block_render = true;
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	const auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 1), plan).ok());
	{
		std::unique_lock<std::mutex> lock(raw_backend->barrier_mutex);
		ASSERT_TRUE(raw_backend->barrier.wait_for(lock, std::chrono::seconds(1),
		                                          [raw_backend]() { return raw_backend->render_entered; }));
	}
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 2), plan).ok());

	const auto rejected = core.SubmitSnapshot(Snapshot(ModelGeneration(1), 3), plan);

	EXPECT_EQ(rejected.code, FrameStatusCode::kBusy);
	EXPECT_EQ(rejected.generation, FrameGeneration(1));
	EXPECT_NE(rejected.message.find("queued render turn"), std::string::npos);
	{
		std::lock_guard<std::mutex> lock(raw_backend->barrier_mutex);
		raw_backend->release_render = true;
	}
	raw_backend->barrier.notify_all();
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	ASSERT_EQ(raw_backend->rendered_simulation_times.size(), 2U);
	EXPECT_EQ(raw_backend->rendered_simulation_times[0], 1);
	EXPECT_EQ(raw_backend->rendered_simulation_times[1], 2);
	core.Shutdown();
}

TEST(RenderBackend, PluginGeometryAppendsWithoutHidingModelGeometry)
{
	const auto rgb_layout = PlaneLayout::Rgb8(128, 64);
	FrameBoundary model_only_boundary(2, rgb_layout.byte_length * 2U);
	FrameBoundary plugin_boundary(2, rgb_layout.byte_length * 2U);
	ASSERT_TRUE(model_only_boundary.Reconfigure(FrameGeneration(1), FrameLayout(128, 64)).ok());
	ASSERT_TRUE(plugin_boundary.Reconfigure(FrameGeneration(1), FrameLayout(128, 64)).ok());
	const auto model_only_result = RenderWithRealBackend({}, model_only_boundary);
	const auto result            = RenderWithRealBackend({ GreenPluginGeometry() }, plugin_boundary);
	if (result.code == RenderStatusCode::kBackendUnavailable) {
		GTEST_SKIP() << result.message;
	}
	ASSERT_TRUE(model_only_result.ok()) << model_only_result.message;
	ASSERT_TRUE(result.ok()) << result.message;
	auto model_only  = model_only_boundary.AcquireLatest(PlaneKind::kRgb);
	auto with_plugin = plugin_boundary.AcquireLatest(PlaneKind::kRgb);
	ASSERT_TRUE(model_only.has_value());
	ASSERT_TRUE(with_plugin.has_value());
	const auto background_red          = std::to_integer<int>(model_only->bytes()[0]);
	const auto background_green        = std::to_integer<int>(model_only->bytes()[1]);
	const auto background_blue         = std::to_integer<int>(model_only->bytes()[2]);
	std::size_t model_pixels           = 0;
	std::size_t preserved_model_pixels = 0;
	for (std::size_t index = 0; index + 2 < model_only->bytes().size(); index += 3) {
		const auto red   = std::to_integer<int>(model_only->bytes()[index]);
		const auto green = std::to_integer<int>(model_only->bytes()[index + 1]);
		const auto blue  = std::to_integer<int>(model_only->bytes()[index + 2]);
		if (std::abs(red - background_red) + std::abs(green - background_green) + std::abs(blue - background_blue) <=
		    12) {
			continue;
		}
		++model_pixels;
		if (model_only->bytes()[index] == with_plugin->bytes()[index] &&
		    model_only->bytes()[index + 1] == with_plugin->bytes()[index + 1] &&
		    model_only->bytes()[index + 2] == with_plugin->bytes()[index + 2]) {
			++preserved_model_pixels;
		}
	}
	ASSERT_GT(model_pixels, 0U) << "model-only real-backend frame has no visible geometry";
	EXPECT_GT(preserved_model_pixels, model_pixels * 9U / 10U) << "plugin geometry hid model geometry";
	EXPECT_NE(model_only->bytes(), with_plugin->bytes()) << "plugin geometry was not rendered";
}

TEST(RenderBackend, PluginGeometryCapacityFailureIsExplicit)
{
	const auto rgb_layout = PlaneLayout::Rgb8(128, 64);
	FrameBoundary boundary(2, rgb_layout.byte_length * 2U);
	ASSERT_TRUE(boundary.Reconfigure(FrameGeneration(1), FrameLayout(128, 64)).ok());
	std::vector<mjvGeom> plugin_geometry(20000, GreenPluginGeometry());
	const auto result = RenderWithRealBackend(plugin_geometry, boundary);
	if (result.code == RenderStatusCode::kBackendUnavailable) {
		GTEST_SKIP() << result.message;
	}
	EXPECT_EQ(result.code, RenderStatusCode::kFrameUnavailable);
	EXPECT_NE(result.message.find("plugin geometry capacity"), std::string::npos);
}

TEST(RenderCore, ResizeAfterCurrentTurnUsesNewConfiguration)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	EXPECT_TRUE(core.Reconfigure(ModelGeneration(1), FrameGeneration(2), FrameLayout(4, 4), { camera }).ok());
	plan = core.EvaluateDemand(std::chrono::milliseconds(2), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	EXPECT_EQ(raw_backend->resize_calls, 1);
	core.Shutdown();
}

TEST(RenderCore, UnchangedFrameConfigurationDoesNotResizeBackend)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");

	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 1), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());

	plan = core.EvaluateDemand(std::chrono::milliseconds(2), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 2), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());

	EXPECT_EQ(raw_backend->resize_calls, 0);
	core.Shutdown();
}

TEST(RenderCore, InitializeFailureIsTerminal)
{
	auto backend               = std::make_unique<FakeBackend>();
	backend->initialize_status = RenderStatus::Failure(RenderStatusCode::kBackendFailure, "init failed");
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	auto consumer   = core.RegisterContinuousConsumer("consumer");
	const auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	EXPECT_EQ(core.FinishOrCancelRenderTurn().code, FrameStatusCode::kBackendFailure);
	EXPECT_EQ(core.Status().code, FrameStatusCode::kBackendFailure);
	core.Shutdown();
	(void)consumer;
}

TEST(RenderCore, ResizeFailureIsTerminal)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	raw_backend->resize_status = RenderStatus::Failure(RenderStatusCode::kBackendFailure, "resize failed");
	ASSERT_TRUE(core.Reconfigure(ModelGeneration(1), FrameGeneration(2), FrameLayout(4, 4), { camera }).ok());
	plan = core.EvaluateDemand(std::chrono::milliseconds(2), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	EXPECT_EQ(core.FinishOrCancelRenderTurn().code, FrameStatusCode::kBackendFailure);
	core.Shutdown();
}

TEST(RenderCore, RenderFailureIsTerminalAndCompletedFramesRemainReadable)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	ASSERT_TRUE(core.AcquireLatest(PlaneKind::kRgb).has_value());
	raw_backend->render_status = RenderStatus::Failure(RenderStatusCode::kBackendFailure, "render failed");
	plan                       = core.EvaluateDemand(std::chrono::milliseconds(2), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	EXPECT_EQ(core.FinishOrCancelRenderTurn().code, FrameStatusCode::kBackendFailure);
	EXPECT_TRUE(core.AcquireLatest(PlaneKind::kRgb).has_value());
	core.Shutdown();
}

TEST(RenderCore, ReconfigurePreservesTerminalBackendFailure)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan                  = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	raw_backend->render_status = RenderStatus::Failure(RenderStatusCode::kBackendFailure, "render failed");
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	ASSERT_EQ(core.FinishOrCancelRenderTurn().code, FrameStatusCode::kBackendFailure);

	ASSERT_TRUE(core.Reconfigure(ModelGeneration(2), FrameGeneration(2), FrameLayout(4, 4), { camera }).ok());
	EXPECT_EQ(core.Status().code, FrameStatusCode::kBackendFailure);
	plan = core.EvaluateDemand(std::chrono::milliseconds(2), camera.id);
	EXPECT_EQ(core.SubmitSnapshot(Snapshot(ModelGeneration(2)), plan).code, FrameStatusCode::kBackendFailure);
	core.Shutdown();
}

TEST(RenderCore, IndependentPlaneFailureIsObservable)
{
	auto backend          = std::make_unique<FakeBackend>();
	backend->fail_plane   = true;
	backend->failed_plane = PlaneKind::kDepth;
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb | PlaneMask::kDepth };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan   = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	plan.planes = camera.planes;
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	const auto completed = core.FinishOrCancelRenderTurn();
	EXPECT_EQ(completed.code, FrameStatusCode::kFrameUnavailable);
	EXPECT_FALSE(IsContextIntegrityFailure(completed));
	EXPECT_TRUE(core.Status().ok());
	EXPECT_TRUE(core.AcquireLatest(PlaneKind::kRgb).has_value());
	EXPECT_FALSE(core.AcquireLatest(PlaneKind::kDepth).has_value());
	core.Shutdown();
}

TEST(RenderCore, MiddlePlaneFailureDoesNotSuppressLaterPlane)
{
	auto backend          = std::make_unique<FakeBackend>();
	auto *raw_backend     = backend.get();
	backend->fail_plane   = true;
	backend->failed_plane = PlaneKind::kDepth;
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2,
		                      PlaneMask::kRgb | PlaneMask::kDepth | PlaneMask::kSegmentation };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan   = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	plan.planes = camera.planes;
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	const auto completed = core.FinishOrCancelRenderTurn();
	EXPECT_EQ(completed.code, FrameStatusCode::kFrameUnavailable);
	EXPECT_FALSE(IsContextIntegrityFailure(completed));
	ASSERT_EQ(raw_backend->attempted_planes.size(), 3U);
	ASSERT_EQ(raw_backend->committed_planes.size(), 2U);
	EXPECT_TRUE(core.AcquireLatest(PlaneKind::kRgb).has_value());
	EXPECT_FALSE(core.AcquireLatest(PlaneKind::kDepth).has_value());
	EXPECT_TRUE(core.AcquireLatest(PlaneKind::kSegmentation).has_value());
	core.Shutdown();
}

TEST(RenderCore, ContextIntegrityFailureRemainsTerminal)
{
	auto backend               = std::make_unique<FakeBackend>();
	backend->initialize_status = RenderStatus::Failure(RenderStatusCode::kBackendFailure, "context lost");
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	EXPECT_EQ(core.FinishOrCancelRenderTurn().code, FrameStatusCode::kBackendFailure);
	EXPECT_EQ(core.Status().code, FrameStatusCode::kBackendFailure);
	core.Shutdown();
}

TEST(RenderCore, PlaneLocalRenderFailureDoesNotTerminalizeCore)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	raw_backend->render_status =
	    RenderStatus::Failure(RenderStatusCode::kFrameUnavailable, "plane-local render failure");
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	const auto completed = core.FinishOrCancelRenderTurn();
	EXPECT_EQ(completed.code, FrameStatusCode::kFrameUnavailable);
	EXPECT_FALSE(IsContextIntegrityFailure(completed));
	EXPECT_TRUE(core.Status().ok());
	EXPECT_GT(core.LastTurnCaptureId(), 0U);
	core.Shutdown();
}

TEST(RenderCore, LastTurnCaptureIdDoesNotReuseHistoricalPlanes)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");

	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 1), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	const auto first_lease = core.AcquireLatest(camera.id, PlaneKind::kRgb);
	ASSERT_TRUE(first_lease.has_value());
	const auto first_capture_id = first_lease->capture_id();

	raw_backend->fail_plane   = true;
	raw_backend->failed_plane = PlaneKind::kRgb;
	plan                      = core.EvaluateDemand(std::chrono::milliseconds(2), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 2), plan).ok());
	const auto completed = core.FinishOrCancelRenderTurn();
	EXPECT_EQ(completed.code, FrameStatusCode::kFrameUnavailable);
	const auto current_capture_id = core.LastTurnCaptureId();
	EXPECT_GT(current_capture_id, first_capture_id);
	EXPECT_FALSE(core.AcquireLatest(current_capture_id, camera.id, PlaneKind::kRgb).has_value());
	EXPECT_TRUE(core.AcquireLatest(first_capture_id, camera.id, PlaneKind::kRgb).has_value());
	core.Shutdown();
}

TEST(RenderCore, PlaneRenderExceptionDoesNotTerminalizeCore)
{
	auto backend             = std::make_unique<FakeBackend>();
	backend->throw_on_render = true;
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	const auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	const auto completed = core.FinishOrCancelRenderTurn();
	EXPECT_EQ(completed.code, FrameStatusCode::kFrameUnavailable);
	EXPECT_FALSE(IsContextIntegrityFailure(completed));
	EXPECT_TRUE(core.Status().ok());
	EXPECT_GT(core.LastTurnCaptureId(), 0U);
	EXPECT_FALSE(core.AcquireLatest(core.LastTurnCaptureId(), camera.id, PlaneKind::kRgb).has_value());
	core.Shutdown();
}

TEST(RenderCore, CaptureContextIntegrityExceptionRemainsTerminal)
{
	auto backend                               = std::make_unique<FakeBackend>();
	backend->throw_context_integrity_on_render = true;
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	const auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	EXPECT_EQ(core.FinishOrCancelRenderTurn().code, FrameStatusCode::kBackendFailure);
	EXPECT_EQ(core.Status().code, FrameStatusCode::kBackendFailure);
	EXPECT_EQ(core.LastTurnCaptureId(), 0U);
	core.Shutdown();
}

TEST(RenderCore, InitializeExceptionRemainsTerminal)
{
	auto backend                 = std::make_unique<FakeBackend>();
	backend->throw_on_initialize = true;
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	const auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());
	EXPECT_EQ(core.FinishOrCancelRenderTurn().code, FrameStatusCode::kBackendFailure);
	EXPECT_EQ(core.Status().code, FrameStatusCode::kBackendFailure);
	EXPECT_EQ(core.LastTurnCaptureId(), 0U);
	core.Shutdown();
}

TEST(RenderCore, CancelledPendingTurnClearsLastTurnCaptureId)
{
	auto backend              = std::make_unique<FakeBackend>();
	auto *raw_backend         = backend.get();
	raw_backend->block_render = true;
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");

	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 1), plan).ok());
	{
		std::unique_lock<std::mutex> lock(raw_backend->barrier_mutex);
		ASSERT_TRUE(raw_backend->barrier.wait_for(lock, std::chrono::seconds(1),
		                                          [raw_backend]() { return raw_backend->render_entered; }));
	}

	auto second_plan = core.EvaluateDemand(std::chrono::milliseconds(2), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 2), second_plan).ok());
	core.RequestCancelRenderTurn();
	{
		std::lock_guard<std::mutex> lock(raw_backend->barrier_mutex);
		raw_backend->release_render = true;
	}
	raw_backend->barrier.notify_all();
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	EXPECT_EQ(core.LastTurnCaptureId(), 0U);
	core.Shutdown();
}

TEST(RenderCore, CancelWithoutPendingTurnClearsLastTurnCaptureId)
{
	auto backend = std::make_unique<FakeBackend>();
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");

	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 1), plan).ok());
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	const auto historical_lease = core.AcquireLatest(camera.id, PlaneKind::kRgb);
	ASSERT_TRUE(historical_lease.has_value());
	const auto historical_capture_id = historical_lease->capture_id();
	ASSERT_GT(core.LastTurnCaptureId(), 0U);

	core.RequestCancelRenderTurn();
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	EXPECT_EQ(core.LastTurnCaptureId(), 0U);
	EXPECT_TRUE(core.AcquireLatest(historical_capture_id, camera.id, PlaneKind::kRgb).has_value());
	core.Shutdown();
}

TEST(RenderCore, CancelDuringActiveTurnClearsLastTurnCaptureId)
{
	auto backend              = std::make_unique<FakeBackend>();
	auto *raw_backend         = backend.get();
	raw_backend->block_render = true;
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");

	auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1), 1), plan).ok());
	{
		std::unique_lock<std::mutex> lock(raw_backend->barrier_mutex);
		ASSERT_TRUE(raw_backend->barrier.wait_for(lock, std::chrono::seconds(1),
		                                          [raw_backend]() { return raw_backend->render_entered; }));
	}

	core.RequestCancelRenderTurn();
	{
		std::lock_guard<std::mutex> lock(raw_backend->barrier_mutex);
		raw_backend->release_render = true;
	}
	raw_backend->barrier.notify_all();
	ASSERT_TRUE(core.FinishOrCancelRenderTurn().ok());
	EXPECT_EQ(core.LastTurnCaptureId(), 0U);
	core.Shutdown();
}

TEST(RenderCore, IdleShutdownClearsCancelAndCaptureId)
{
	auto core = std::make_shared<RenderCore>(std::make_unique<FakeBackend>());
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(*core, camera);
	core->RegisterContinuousConsumer("consumer");

	auto plan = core->EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core->SubmitSnapshot(Snapshot(ModelGeneration(1), 1), plan).ok());
	ASSERT_TRUE(core->FinishOrCancelRenderTurn().ok());
	ASSERT_GT(core->LastTurnCaptureId(), 0U);

	core->Shutdown();

	const auto finish_status =
	    RunWithHangTimeout(std::chrono::milliseconds(500), [core]() { return core->FinishOrCancelRenderTurn(); });
	ASSERT_TRUE(finish_status.has_value()) << "FinishOrCancelRenderTurn hung after idle Shutdown";
	EXPECT_TRUE(finish_status->ok());
	EXPECT_EQ(core->LastTurnCaptureId(), 0U);
}

TEST(RenderCore, PostShutdownCancelQuiescenceDoesNotHang)
{
	auto core = std::make_shared<RenderCore>(std::make_unique<FakeBackend>());
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(*core, camera);

	core->Shutdown();
	core->RequestCancelRenderTurn();

	const auto finish_status =
	    RunWithHangTimeout(std::chrono::milliseconds(500), [core]() { return core->FinishOrCancelRenderTurn(); });
	ASSERT_TRUE(finish_status.has_value())
	    << "FinishOrCancelRenderTurn hung after Shutdown then RequestCancelRenderTurn";
	EXPECT_TRUE(finish_status->ok());
}

TEST(RenderCore, StaleModelSnapshotIsRejectedBeforeBackendWork)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ASSERT_TRUE(core.Reconfigure(ModelGeneration(2), FrameGeneration(1), FrameLayout(2, 2), { camera }).ok());

	RenderSnapshot snapshot;
	snapshot.model_generation = ModelGeneration(1);
	snapshot.model            = EmptyModel();
	snapshot.data             = std::shared_ptr<mjData>(mj_makeData(snapshot.model.get()), mj_deleteData);
	RenderPlan plan;
	plan.camera = camera;
	plan.consumers.push_back(ConsumerId(1));
	const auto result = core.SubmitSnapshot(std::move(snapshot), plan);

	EXPECT_EQ(result.code, FrameStatusCode::kStaleGeneration);
	EXPECT_EQ(raw_backend->render_calls, 0);
}

TEST(RenderCore, PlanEvaluatedBeforeFrameGenerationChangeIsRejected)
{
	auto backend      = std::make_unique<FakeBackend>();
	auto *raw_backend = backend.get();
	RenderCore core(std::move(backend));
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera, ModelGeneration(1), FrameGeneration(1));
	core.RegisterContinuousConsumer("consumer");
	const auto stale_plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);

	ASSERT_TRUE(core.Reconfigure(ModelGeneration(1), FrameGeneration(2), FrameLayout(4, 4), { camera }).ok());
	const auto result = core.SubmitSnapshot(Snapshot(ModelGeneration(1)), stale_plan);

	EXPECT_EQ(result.code, FrameStatusCode::kStaleGeneration);
	EXPECT_NE(result.message.find("frame generation"), std::string::npos);
	EXPECT_EQ(raw_backend->render_calls, 0);
	core.Shutdown();
}

TEST(RenderCore, PlanEvaluatedBeforeModelAndFrameChangeIsRejected)
{
	RenderCore core(std::make_unique<FakeBackend>());
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera, ModelGeneration(1), FrameGeneration(1));
	core.RegisterContinuousConsumer("consumer");
	const auto stale_plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);

	ASSERT_TRUE(core.Reconfigure(ModelGeneration(2), FrameGeneration(2), FrameLayout(4, 4), { camera }).ok());
	const auto result = core.SubmitSnapshot(Snapshot(ModelGeneration(2)), stale_plan);

	EXPECT_EQ(result.code, FrameStatusCode::kStaleGeneration);
	EXPECT_NE(result.message.find("frame generation"), std::string::npos);
	core.Shutdown();
}

TEST(RenderCore, DisabledBackendStatusRemainsObservable)
{
	RenderCore core(CreateDisabledRenderBackend());
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera);
	core.RegisterContinuousConsumer("consumer");
	const auto plan = core.EvaluateDemand(std::chrono::milliseconds(1), camera.id);
	ASSERT_TRUE(core.SubmitSnapshot(Snapshot(ModelGeneration(1)), plan).ok());

	EXPECT_EQ(core.FinishOrCancelRenderTurn().code, FrameStatusCode::kBackendUnavailable);
	EXPECT_EQ(core.Status().code, FrameStatusCode::kBackendUnavailable);
	core.Shutdown();
}

TEST(RenderCore, InvalidCameraReconfigureDoesNotMutateGeneration)
{
	RenderCore core(std::make_unique<FakeBackend>());
	CameraDescriptor camera{ CameraId(1), "camera", 2, 2, PlaneMask::kRgb };
	ConfigureCore(core, camera, ModelGeneration(1), FrameGeneration(3));
	CameraDescriptor invalid{ CameraId(0), "invalid", 2, 2, PlaneMask::kRgb };

	const auto result = core.Reconfigure(ModelGeneration(2), FrameGeneration(4), FrameLayout(4, 4), { invalid });

	EXPECT_EQ(result.code, FrameStatusCode::kInvalidLayout);
	EXPECT_EQ(core.frames().generation(), FrameGeneration(3));
	core.Shutdown();
}

} // namespace mujoco_ros::rendering
