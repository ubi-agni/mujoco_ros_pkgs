#include <gtest/gtest.h>

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
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

TEST(PluginHost, ActivatesSuccessfulAdaptersOnlyAfterCompleteLoadPass)
{
	EventLog log;
	RecordingFactory factory(log, { true, false, true });
	PluginHost host(factory);

	const auto report = host.LoadGeneration(nullptr, nullptr, ModelGeneration(7), PluginGeneration(4));
	ASSERT_EQ(report.statistics.size(), 3u);
	EXPECT_EQ(log.Events(), (std::vector<std::string>{ "load:0", "load:1", "load:2", "activate:0", "activate:2" }));

	host.DispatchControl(nullptr, nullptr);
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

	host.DispatchPassive(nullptr, nullptr);
	host.DispatchRender(nullptr, nullptr, nullptr);
	host.DispatchLastStage(nullptr, nullptr);
	host.Reset();
	host.NotifyGeometryChanged(nullptr, nullptr, 9);

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
						    host.DispatchControl(nullptr, nullptr);
						    break;
					    case ThrowingLifecyclePhase::Passive:
						    host.DispatchPassive(nullptr, nullptr);
						    break;
					    case ThrowingLifecyclePhase::Render:
						    host.DispatchRender(nullptr, nullptr, nullptr);
						    break;
					    case ThrowingLifecyclePhase::LastStage:
						    host.DispatchLastStage(nullptr, nullptr);
						    break;
					    case ThrowingLifecyclePhase::Reset:
						    host.Reset();
						    break;
					    case ThrowingLifecyclePhase::Geometry:
						    host.NotifyGeometryChanged(nullptr, nullptr, 1);
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

	std::thread callback([&host]() { host.DispatchControl(nullptr, nullptr); });
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
