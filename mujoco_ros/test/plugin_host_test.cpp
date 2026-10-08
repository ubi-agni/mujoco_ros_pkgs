#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <mujoco_ros/plugin_host.hpp>

namespace mujoco_ros {
namespace {

class EventLog
{
public:
	void Add(std::string event)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		events_.push_back(std::move(event));
		condition_.notify_all();
	}

	std::vector<std::string> Events() const
	{
		std::lock_guard<std::mutex> lock(mutex_);
		return events_;
	}

	void WaitForSize(std::size_t size)
	{
		std::unique_lock<std::mutex> lock(mutex_);
		condition_.wait(lock, [this, size]() { return events_.size() >= size; });
	}

private:
	mutable std::mutex mutex_;
	std::condition_variable condition_;
	std::vector<std::string> events_;
};

class RecordingAdapter : public IPluginAdapter
{
public:
	RecordingAdapter(EventLog &log, int id, bool load_successful, std::function<void()> control_hook = {})
	    : log_(log), id_(id), load_successful_(load_successful), control_hook_(std::move(control_hook))
	{
	}

	~RecordingAdapter() override { log_.Add("destroy:" + std::to_string(id_)); }

	const std::string &Name() const override { return name_; }
	const std::string &Type() const override { return type_; }

	bool Load(const mjModel *, mjData *, std::string &error) override
	{
		log_.Add("load:" + std::to_string(id_));
		if (!load_successful_) {
			error = "load failed";
		}
		return load_successful_;
	}

	void Activate() override { log_.Add("activate:" + std::to_string(id_)); }

	void Control(const mjModel *, mjData *) override
	{
		log_.Add("control:" + std::to_string(id_));
		if (control_hook_) {
			control_hook_();
			log_.Add("control-exit:" + std::to_string(id_));
		}
	}
	void Passive(const mjModel *, mjData *) override { log_.Add("passive:" + std::to_string(id_)); }
	void Render(const mjModel *, mjData *, mjvScene *) override { log_.Add("render:" + std::to_string(id_)); }
	void LastStage(const mjModel *, mjData *) override { log_.Add("last_stage:" + std::to_string(id_)); }
	void Reset() override { log_.Add("reset:" + std::to_string(id_)); }
	void GeometryChanged(const mjModel *, mjData *, int geom_id) override
	{
		log_.Add("geometry:" + std::to_string(id_) + ":" + std::to_string(geom_id));
	}

	PluginStat Statistics() const override
	{
		PluginStat stat;
		stat.name = name_;
		stat.type = type_;
		return stat;
	}

private:
	EventLog &log_;
	int id_;
	bool load_successful_;
	std::function<void()> control_hook_;
	std::string name_ = "plugin";
	std::string type_ = "test/RecordingAdapter";
};

class BlockingControl
{
public:
	void Enter()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		entered_ = true;
		condition_.notify_all();
		condition_.wait(lock, [this]() { return released_; });
	}

	void WaitForEntered()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		condition_.wait(lock, [this]() { return entered_; });
	}

	void Release()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		released_ = true;
		condition_.notify_all();
	}

private:
	std::mutex mutex_;
	std::condition_variable condition_;
	bool entered_  = false;
	bool released_ = false;
};

class BlockingFactory final : public IPluginAdapterFactory
{
public:
	explicit BlockingFactory(EventLog &log) : log_(log) {}

	std::vector<std::unique_ptr<IPluginAdapter>> CreateAdapters() override
	{
		std::vector<std::unique_ptr<IPluginAdapter>> adapters;
		adapters.emplace_back(std::make_unique<RecordingAdapter>(log_, 0, true, [this]() { control_.Enter(); }));
		return adapters;
	}

	void WaitForControlEntered() { control_.WaitForEntered(); }
	void ReleaseControl() { control_.Release(); }

private:
	EventLog &log_;
	BlockingControl control_;
};

class RecordingFactory final : public IPluginAdapterFactory
{
public:
	RecordingFactory(EventLog &log, std::vector<bool> load_results) : log_(log), load_results_(std::move(load_results))
	{
	}

	std::vector<std::unique_ptr<IPluginAdapter>> CreateAdapters() override
	{
		std::vector<std::unique_ptr<IPluginAdapter>> adapters;
		for (std::size_t index = 0; index < load_results_.size(); ++index) {
			adapters.emplace_back(std::make_unique<RecordingAdapter>(log_, static_cast<int>(index), load_results_[index]));
		}
		return adapters;
	}

private:
	EventLog &log_;
	std::vector<bool> load_results_;
};

class ThrowingAdapter final : public RecordingAdapter
{
public:
	ThrowingAdapter(EventLog &log) : RecordingAdapter(log, 0, true) {}
	bool Load(const mjModel *, mjData *, std::string &) override { throw std::runtime_error("load exploded"); }
};

class ThrowingFactory final : public IPluginAdapterFactory
{
public:
	explicit ThrowingFactory(EventLog &log) : log_(log) {}
	std::vector<std::unique_ptr<IPluginAdapter>> CreateAdapters() override
	{
		std::vector<std::unique_ptr<IPluginAdapter>> adapters;
		adapters.emplace_back(std::make_unique<ThrowingAdapter>(log_));
		return adapters;
	}

private:
	EventLog &log_;
};

enum class ThrowingLifecyclePhase
{
	Control,
	Passive,
	Render,
	LastStage,
	Reset,
	Geometry,
	Statistics,
};

class ThrowingLifecycleAdapter final : public RecordingAdapter
{
public:
	ThrowingLifecycleAdapter(EventLog &log, ThrowingLifecyclePhase phase) : RecordingAdapter(log, 0, true), phase_(phase)
	{
	}

	void Control(const mjModel *model, mjData *data) override
	{
		if (phase_ == ThrowingLifecyclePhase::Control)
			throw std::runtime_error("control exploded");
		RecordingAdapter::Control(model, data);
	}
	void Passive(const mjModel *model, mjData *data) override
	{
		if (phase_ == ThrowingLifecyclePhase::Passive)
			throw std::runtime_error("passive exploded");
		RecordingAdapter::Passive(model, data);
	}
	void Render(const mjModel *model, mjData *data, mjvScene *scene) override
	{
		if (phase_ == ThrowingLifecyclePhase::Render)
			throw std::runtime_error("render exploded");
		RecordingAdapter::Render(model, data, scene);
	}
	void LastStage(const mjModel *model, mjData *data) override
	{
		if (phase_ == ThrowingLifecyclePhase::LastStage)
			throw std::runtime_error("last stage exploded");
		RecordingAdapter::LastStage(model, data);
	}
	void Reset() override
	{
		if (phase_ == ThrowingLifecyclePhase::Reset)
			throw std::runtime_error("reset exploded");
		RecordingAdapter::Reset();
	}
	void GeometryChanged(const mjModel *model, mjData *data, int geom_id) override
	{
		if (phase_ == ThrowingLifecyclePhase::Geometry)
			throw std::runtime_error("geometry exploded");
		RecordingAdapter::GeometryChanged(model, data, geom_id);
	}
	PluginStat Statistics() const override
	{
		if (phase_ == ThrowingLifecyclePhase::Statistics)
			throw std::runtime_error("statistics exploded");
		return RecordingAdapter::Statistics();
	}

private:
	ThrowingLifecyclePhase phase_;
};

class ThrowingLifecycleFactory final : public IPluginAdapterFactory
{
public:
	ThrowingLifecycleFactory(EventLog &log, ThrowingLifecyclePhase phase) : log_(log), phase_(phase) {}

	std::vector<std::unique_ptr<IPluginAdapter>> CreateAdapters() override
	{
		std::vector<std::unique_ptr<IPluginAdapter>> adapters;
		adapters.emplace_back(std::make_unique<ThrowingLifecycleAdapter>(log_, phase_));
		return adapters;
	}

private:
	EventLog &log_;
	ThrowingLifecyclePhase phase_;
};

class ReentrantLoadAdapter final : public IPluginAdapter
{
public:
	ReentrantLoadAdapter(PluginHost &host, ModelGeneration model_generation)
	    : host_(host), model_generation_(model_generation)
	{
	}

	const std::string &Name() const override { return name_; }
	const std::string &Type() const override { return type_; }

	bool Load(const mjModel *, mjData *, std::string &) override
	{
		host_.NotifyGeometryChanged(model_generation_, nullptr, nullptr, 0);
		return true;
	}

	void Activate() override {}
	void Control(const mjModel *, mjData *) override {}
	void Passive(const mjModel *, mjData *) override {}
	void Render(const mjModel *, mjData *, mjvScene *) override {}
	void LastStage(const mjModel *, mjData *) override {}
	void Reset() override {}
	void GeometryChanged(const mjModel *, mjData *, int) override {}
	PluginStat Statistics() const override { return {}; }

private:
	PluginHost &host_;
	ModelGeneration model_generation_;
	std::string name_ = "reentrant";
	std::string type_ = "test/ReentrantLoadAdapter";
};

class ReentrantLoadFactory final : public IPluginAdapterFactory
{
public:
	void SetHost(PluginHost *host) { host_ = host; }
	void SetModelGeneration(ModelGeneration model_generation) { model_generation_ = model_generation; }

	std::vector<std::unique_ptr<IPluginAdapter>> CreateAdapters() override
	{
		std::vector<std::unique_ptr<IPluginAdapter>> adapters;
		adapters.emplace_back(std::make_unique<ReentrantLoadAdapter>(*host_, model_generation_));
		return adapters;
	}

private:
	PluginHost *host_ = nullptr;
	ModelGeneration model_generation_{ 1 };
};

TEST(PluginHost, LoadGenerationDoesNotDeadlockWhenLoadReentersPluginHost)
{
	ReentrantLoadFactory factory;
	auto host = std::make_unique<PluginHost>(factory);
	factory.SetHost(host.get());
	factory.SetModelGeneration(ModelGeneration(1));

	std::optional<PluginLoadReport> report;
	std::mutex report_mutex;
	std::condition_variable report_ready;
	bool load_finished = false;

	std::thread load_thread([&]() {
		const auto loaded = host->LoadGeneration(nullptr, nullptr, ModelGeneration(1), PluginGeneration(1));
		{
			std::lock_guard<std::mutex> lock(report_mutex);
			report        = loaded;
			load_finished = true;
		}
		report_ready.notify_one();
	});

	{
		std::unique_lock<std::mutex> lock(report_mutex);
		const bool finished = report_ready.wait_for(lock, std::chrono::seconds(2), [&]() { return load_finished; });
		if (!finished) {
			load_thread.detach();
			// The detached thread still holds PluginHost::mutex_; avoid running ~PluginHost().
			(void)host.release();
			FAIL() << "PluginHost::LoadGeneration deadlocked when a plugin's Load() re-entered PluginHost";
		}
	}
	load_thread.join();
	ASSERT_TRUE(report.has_value());
	EXPECT_TRUE(report->failures.empty());
	EXPECT_EQ(host->ReadyCount(), 1u);
}

TEST(PluginHost, ActivatesSuccessfulAdaptersOnlyAfterCompleteLoadPass)
{
	EventLog log;
	RecordingFactory factory(log, { true, false, true });
	PluginHost host(factory);

	const auto report = host.LoadGeneration(nullptr, nullptr, ModelGeneration(7), PluginGeneration(4));
	ASSERT_EQ(report.statistics.size(), 3u);
	EXPECT_EQ(log.Events(), (std::vector<std::string>{ "load:0", "load:1", "load:2", "activate:0", "activate:2" }));

	host.DispatchControl(ModelGeneration(7), nullptr, nullptr);
	const auto events = log.Events();
	EXPECT_EQ(std::vector<std::string>(events.end() - 2, events.end()),
	          (std::vector<std::string>{ "control:0", "control:2" }));
}

TEST(PluginHost, DispatchesEveryLifecycleOperationInConfiguredOrder)
{
	EventLog log;
	RecordingFactory factory(log, { true, true });
	PluginHost host(factory);
	host.LoadGeneration(nullptr, nullptr, ModelGeneration(1), PluginGeneration(2));

	host.DispatchPassive(ModelGeneration(1), nullptr, nullptr);
	host.DispatchRender(ModelGeneration(1), nullptr, nullptr, nullptr);
	host.DispatchLastStage(ModelGeneration(1), nullptr, nullptr);
	host.Reset(ModelGeneration(1));
	host.NotifyGeometryChanged(ModelGeneration(1), nullptr, nullptr, 9);

	const auto events = log.Events();
	EXPECT_EQ(std::vector<std::string>(events.end() - 10, events.end()),
	          (std::vector<std::string>{ "passive:0", "passive:1", "render:0", "render:1", "last_stage:0",
	                                     "last_stage:1", "reset:0", "reset:1", "geometry:0:9", "geometry:1:9" }));
}

TEST(PluginHost, ScopedAccessRejectsInactiveGenerationAndMissingAdapters)
{
	EventLog log;
	RecordingFactory factory(log, { true });
	PluginHost host(factory);
	host.LoadGeneration(nullptr, nullptr, ModelGeneration(1), PluginGeneration(8));

	{
		auto access = host.AcquireScopedAccess(PluginGeneration(8));
		EXPECT_EQ(access.Generation(), PluginGeneration(8));
		EXPECT_EQ(access.Adapter("plugin", "test/RecordingAdapter")->Name(), "plugin");
		EXPECT_THROW(access.Adapter("missing", "test/RecordingAdapter"), std::runtime_error);
	}

	host.QuiesceAndDestroy();
	EXPECT_THROW(host.AcquireScopedAccess(PluginGeneration(8)), std::runtime_error);
}

TEST(PluginHost, RejectsCallbacksForStaleModelGeneration)
{
	EventLog log;
	RecordingFactory factory(log, { true });
	PluginHost host(factory);
	host.LoadGeneration(nullptr, nullptr, ModelGeneration(7), PluginGeneration(8));

	host.DispatchControl(ModelGeneration(7), nullptr, nullptr);
	EXPECT_THROW(host.DispatchControl(ModelGeneration(6), nullptr, nullptr), std::runtime_error);
}

TEST(PluginHost, ReportsAdapterLoadExceptionsAsObservableFailures)
{
	EventLog log;
	ThrowingFactory factory(log);
	PluginHost host(factory);

	const auto report = host.LoadGeneration(nullptr, nullptr, ModelGeneration(1), PluginGeneration(1));
	ASSERT_EQ(report.failures.size(), 1u);
	EXPECT_EQ(report.failures.front().error, "load exploded");
	EXPECT_EQ(host.ReadyCount(), 0u);
}

TEST(PluginHost, ReportsLifecycleExceptionsWithoutSwallowingFailures)
{
	const std::vector<std::pair<ThrowingLifecyclePhase, std::string>> cases = {
		{ ThrowingLifecyclePhase::Control, "control exploded" },
		{ ThrowingLifecyclePhase::Passive, "passive exploded" },
		{ ThrowingLifecyclePhase::Render, "render exploded" },
		{ ThrowingLifecyclePhase::LastStage, "last stage exploded" },
		{ ThrowingLifecyclePhase::Reset, "reset exploded" },
		{ ThrowingLifecyclePhase::Geometry, "geometry exploded" },
		{ ThrowingLifecyclePhase::Statistics, "statistics exploded" },
	};

	for (const auto &[phase, error] : cases) {
		EventLog log;
		ThrowingLifecycleFactory factory(log, phase);
		PluginHost host(factory);
		const auto report = host.LoadGeneration(nullptr, nullptr, ModelGeneration(1), PluginGeneration(1));
		if (phase == ThrowingLifecyclePhase::Statistics) {
			ASSERT_EQ(report.failures.size(), 1u);
			EXPECT_EQ(report.failures.front().phase, "statistics");
			EXPECT_EQ(report.failures.front().error, error);
		} else {
			EXPECT_THROW(
			    [&]() {
				    switch (phase) {
					    case ThrowingLifecyclePhase::Control:
						    host.DispatchControl(ModelGeneration(1), nullptr, nullptr);
						    break;
					    case ThrowingLifecyclePhase::Passive:
						    host.DispatchPassive(ModelGeneration(1), nullptr, nullptr);
						    break;
					    case ThrowingLifecyclePhase::Render:
						    host.DispatchRender(ModelGeneration(1), nullptr, nullptr, nullptr);
						    break;
					    case ThrowingLifecyclePhase::LastStage:
						    host.DispatchLastStage(ModelGeneration(1), nullptr, nullptr);
						    break;
					    case ThrowingLifecyclePhase::Reset:
						    host.Reset(ModelGeneration(1));
						    break;
					    case ThrowingLifecyclePhase::Geometry:
						    host.NotifyGeometryChanged(ModelGeneration(1), nullptr, nullptr, 1);
						    break;
					    case ThrowingLifecyclePhase::Statistics:
						    break;
				    }
			    }(),
			    std::runtime_error);
			const auto diagnostics = host.Diagnostics();
			ASSERT_FALSE(diagnostics.empty());
			EXPECT_EQ(diagnostics.back().phase, phase == ThrowingLifecyclePhase::Control   ? "control" :
			                                    phase == ThrowingLifecyclePhase::Passive   ? "passive" :
			                                    phase == ThrowingLifecyclePhase::Render    ? "render" :
			                                    phase == ThrowingLifecyclePhase::LastStage ? "last_stage" :
			                                    phase == ThrowingLifecyclePhase::Reset     ? "reset" :
			                                                                                 "geometry");
			EXPECT_EQ(diagnostics.back().error, error);
		}
	}
}

TEST(PluginHost, ReloadOrderingExcludesOldGenerationCallbacks)
{
	EventLog log;
	RecordingFactory factory(log, { true });
	PluginHost host(factory);
	host.LoadGeneration(nullptr, nullptr, ModelGeneration(1), PluginGeneration(1));
	host.DispatchControl(ModelGeneration(1), nullptr, nullptr);
	host.LoadGeneration(nullptr, nullptr, ModelGeneration(2), PluginGeneration(2));
	host.DispatchControl(ModelGeneration(2), nullptr, nullptr);
	host.QuiesceAndDestroy();

	EXPECT_EQ(log.Events(), (std::vector<std::string>{ "load:0", "activate:0", "control:0", "destroy:0", "load:0",
	                                                   "activate:0", "control:0", "destroy:0" }));
}

TEST(PluginHost, QuiescenceWaitsForCallbackBeforeDestroyingGeneration)
{
	EventLog log;
	BlockingFactory factory(log);
	PluginHost host(factory);
	host.LoadGeneration(nullptr, nullptr, ModelGeneration(1), PluginGeneration(1));

	std::thread callback([&host]() { host.DispatchControl(ModelGeneration(1), nullptr, nullptr); });
	factory.WaitForControlEntered();
	std::thread quiesce([&host]() { host.QuiesceAndDestroy(); });

	// Release only after the callback has entered its critical section. If the
	// host drops its lock around callbacks, destruction would precede control-exit.
	factory.ReleaseControl();
	callback.join();
	quiesce.join();

	EXPECT_EQ(log.Events(),
	          (std::vector<std::string>{ "load:0", "activate:0", "control:0", "control-exit:0", "destroy:0" }));
}

} // namespace
} // namespace mujoco_ros
