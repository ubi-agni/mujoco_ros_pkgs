#pragma once

#include <mujoco_ros/generation.hpp>
#include <mujoco_ros/plugin_adapter.hpp>

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <functional>
#include <vector>

namespace mujoco_ros {

struct PluginLoadFailure
{
	std::string name;
	std::string type;
	std::string error;
	std::string phase;
};

struct PluginLoadReport
{
	ModelGeneration model_generation;
	PluginGeneration generation;
	std::vector<PluginStat> statistics;
	std::vector<PluginLoadFailure> failures;
};

class PluginHost;
class MujocoEnv;

class PluginHandle
{
public:
	PluginHandle() = default;

	const std::string &Name() const { return name_; }
	const std::string &Type() const { return type_; }
	PluginGeneration Generation() const { return generation_; }
	ModelGeneration Model() const { return model_generation_; }

	template <typename Func>
	decltype(auto) WithAccess(Func &&func) const;

	template <typename Backend, typename Func>
	decltype(auto) WithBackend(Func &&func) const;

private:
	friend class MujocoEnv;
	PluginHandle(MujocoEnv *env, std::weak_ptr<void> lifetime, ModelGeneration model_generation,
	             PluginGeneration generation, std::string name, std::string type)
	    : env_(env)
	    , lifetime_(std::move(lifetime))
	    , model_generation_(model_generation)
	    , generation_(generation)
	    , name_(std::move(name))
	    , type_(std::move(type))
	{
	}

	MujocoEnv *env_ = nullptr;
	std::weak_ptr<void> lifetime_;
	ModelGeneration model_generation_;
	PluginGeneration generation_;
	std::string name_;
	std::string type_;
};

class ScopedPluginAccess
{
public:
	ScopedPluginAccess(ScopedPluginAccess &&) noexcept            = default;
	ScopedPluginAccess &operator=(ScopedPluginAccess &&) noexcept = default;
	ScopedPluginAccess(const ScopedPluginAccess &)                = delete;
	ScopedPluginAccess &operator=(const ScopedPluginAccess &)     = delete;

	PluginGeneration Generation() const { return generation_; }
	std::vector<PluginStat> Statistics() const;
	IPluginAdapter *Adapter(const std::string &name, const std::string &type) const;

	template <typename Backend>
	Backend *BackendObject(const std::string &name, const std::string &type) const
	{
		auto *adapter = Adapter(name, type);
		void *object  = adapter->BackendObject();
		if (object == nullptr) {
			throw std::runtime_error("plugin adapter has no backend object for '" + name + "'");
		}
		return static_cast<Backend *>(object);
	}

private:
	friend class PluginHost;
	ScopedPluginAccess(PluginHost *host, std::unique_lock<std::mutex> lock, ModelGeneration model_generation,
	                   PluginGeneration generation)
	    : host_(host), lock_(std::move(lock)), model_generation_(model_generation), generation_(generation)
	{
	}

	PluginHost *host_ = nullptr;
	std::unique_lock<std::mutex> lock_;
	ModelGeneration model_generation_;
	PluginGeneration generation_;
};

class PluginHost
{
public:
	explicit PluginHost(IPluginAdapterFactory &factory) : factory_(factory) {}
	~PluginHost();

	PluginHost(const PluginHost &)            = delete;
	PluginHost &operator=(const PluginHost &) = delete;

	PluginLoadReport LoadGeneration(const mjModel *, mjData *, ModelGeneration, PluginGeneration);
	void QuiesceAndDestroy();

	void DispatchControl(ModelGeneration, const mjModel *, mjData *);
	void DispatchPassive(ModelGeneration, const mjModel *, mjData *);
	void DispatchRender(ModelGeneration, const mjModel *, mjData *, mjvScene *);
	void DispatchLastStage(ModelGeneration, const mjModel *, mjData *);
	void Reset(ModelGeneration);
	void NotifyGeometryChanged(ModelGeneration, const mjModel *, mjData *, int geom_id);
	void DispatchControl(const mjModel *model, mjData *data) { DispatchControl(model_generation_, model, data); }
	void DispatchPassive(const mjModel *model, mjData *data) { DispatchPassive(model_generation_, model, data); }
	void DispatchRender(const mjModel *model, mjData *data, mjvScene *scene)
	{
		DispatchRender(model_generation_, model, data, scene);
	}
	void DispatchLastStage(const mjModel *model, mjData *data) { DispatchLastStage(model_generation_, model, data); }
	void Reset() { Reset(model_generation_); }
	void NotifyGeometryChanged(const mjModel *model, mjData *data, int geom_id)
	{
		NotifyGeometryChanged(model_generation_, model, data, geom_id);
	}

	std::vector<PluginStat> Statistics() const;
	std::vector<PluginStat> ActiveStatistics() const;
	std::vector<PluginLoadFailure> Diagnostics() const;
	std::size_t ReadyCount() const;
	ModelGeneration ActiveModelGeneration() const;
	ScopedPluginAccess AcquireScopedAccess();
	ScopedPluginAccess AcquireScopedAccess(PluginGeneration expected_generation);
	ScopedPluginAccess AcquireScopedAccess(ModelGeneration expected_model_generation,
	                                       PluginGeneration expected_generation);

private:
	friend class ScopedPluginAccess;

	struct Entry
	{
		std::unique_ptr<IPluginAdapter> adapter;
		bool ready = false;
	};

	IPluginAdapter *FindAdapterLocked(const std::string &name, const std::string &type) const;
	bool ValidateModelGenerationLocked(ModelGeneration model_generation) const;
	void RecordFailureLocked(const IPluginAdapter &adapter, const char *phase, const std::string &error) const;

	IPluginAdapterFactory &factory_;
	mutable std::mutex mutex_;
	std::vector<Entry> entries_;
	ModelGeneration model_generation_;
	PluginGeneration generation_;
	bool generation_active_ = false;
	mutable std::vector<PluginLoadFailure> diagnostics_;
};

} // namespace mujoco_ros
