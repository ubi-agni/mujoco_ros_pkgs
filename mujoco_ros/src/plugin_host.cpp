#include <mujoco_ros/plugin_host.hpp>

#include <algorithm>
#include <exception>
#include <string>
#include <utility>

namespace mujoco_ros {

namespace {

PluginLoadFailure MakeFailure(const IPluginAdapter *adapter, const char *phase, std::string error)
{
	PluginLoadFailure failure;
	failure.phase = phase;
	failure.error = std::move(error);
	if (adapter == nullptr) {
		failure.name = "<unknown>";
		failure.type = "<unknown>";
		return failure;
	}
	try {
		failure.name = adapter->Name();
	} catch (...) {
		failure.name = "<unknown>";
	}
	try {
		failure.type = adapter->Type();
	} catch (...) {
		failure.type = "<unknown>";
	}
	return failure;
}

std::string ExceptionMessage(const char *operation)
{
	try {
		throw;
	} catch (const std::exception &exception) {
		return exception.what();
	} catch (...) {
		return std::string("plugin adapter ") + operation + " threw an unknown exception";
	}
}

} // namespace

PluginHost::~PluginHost()
{
	QuiesceAndDestroy();
}

PluginLoadReport PluginHost::LoadGeneration(const mjModel *model, mjData *data, ModelGeneration model_generation,
                                            PluginGeneration generation)
{
	QuiesceAndDestroy();

	std::unique_lock<std::mutex> lock(mutex_);
	PluginLoadReport report;
	report.model_generation = model_generation;
	report.generation       = generation;
	model_generation_       = model_generation;
	generation_             = generation;
	diagnostics_.clear();
	auto report_failure = [this, &report](PluginLoadFailure failure) {
		report.failures.push_back(failure);
		diagnostics_.push_back(std::move(failure));
	};

	std::vector<std::unique_ptr<IPluginAdapter>> adapters;
	try {
		adapters = factory_.CreateAdapters();
	} catch (const std::exception &exception) {
		report_failure({ "<factory>", "<factory>", exception.what(), "factory" });
		generation_active_ = true;
		return report;
	} catch (...) {
		report_failure({ "<factory>", "<factory>", "plugin adapter factory threw an unknown exception", "factory" });
		generation_active_ = true;
		return report;
	}

	entries_.clear();
	for (auto &adapter : adapters) {
		entries_.push_back(Entry{ std::move(adapter), false });
	}

	std::vector<bool> loaded_entries(entries_.size(), false);
	for (std::size_t index = 0; index < entries_.size(); ++index) {
		auto &entry = entries_[index];
		if (!entry.adapter) {
			report_failure({ "<null>", "<null>", "plugin adapter factory returned null", "factory" });
			continue;
		}
		const auto identity = [&, index]() {
			try {
				return std::pair<std::string, std::string>(entries_[index].adapter->Name(),
				                                           entries_[index].adapter->Type());
			} catch (...) {
				return std::pair<std::string, std::string>("<unknown>", "<unknown>");
			}
		}();
		std::string error;
		bool loaded = false;
		try {
			loaded = entry.adapter->Load(model, data, error);
		} catch (const std::exception &exception) {
			error = exception.what();
		} catch (...) {
			error = "plugin adapter Load threw an unknown exception";
		}
		if (!loaded) {
			if (error.empty()) {
				error = "plugin adapter Load returned false without a diagnostic";
			}
			report_failure({ identity.first, identity.second, std::move(error), "load" });
		} else {
			loaded_entries[index] = true;
		}
	}

	for (auto &entry : entries_) {
		if (!entry.adapter) {
			continue;
		}
		try {
			report.statistics.push_back(entry.adapter->Statistics());
		} catch (const std::exception &exception) {
			auto failure = MakeFailure(entry.adapter.get(), "statistics", exception.what());
			report_failure(failure);
			PluginStat fallback;
			fallback.name = failure.name;
			fallback.type = failure.type;
			report.statistics.push_back(std::move(fallback));
		} catch (...) {
			auto failure =
			    MakeFailure(entry.adapter.get(), "statistics", "plugin adapter Statistics threw an unknown exception");
			report_failure(failure);
			PluginStat fallback;
			fallback.name = failure.name;
			fallback.type = failure.type;
			report.statistics.push_back(std::move(fallback));
		}
	}

	for (std::size_t index = 0; index < entries_.size(); ++index) {
		if (loaded_entries[index]) {
			try {
				entries_[index].adapter->Activate();
				entries_[index].ready = true;
			} catch (const std::exception &exception) {
				report_failure(MakeFailure(entries_[index].adapter.get(), "activate", exception.what()));
			} catch (...) {
				report_failure(MakeFailure(entries_[index].adapter.get(), "activate",
				                           "plugin adapter Activate threw an unknown exception"));
			}
		}
	}

	generation_active_ = true;
	return report;
}

void PluginHost::QuiesceAndDestroy()
{
	std::lock_guard<std::mutex> lock(mutex_);
	generation_active_ = false;
	entries_.clear();
}

bool PluginHost::ValidateModelGenerationLocked(ModelGeneration model_generation) const
{
	if (!generation_active_) {
		return false;
	}
	if (model_generation_ != model_generation) {
		throw std::runtime_error("plugin callback belongs to an inactive Model Generation");
	}
	return true;
}

void PluginHost::RecordFailureLocked(const IPluginAdapter &adapter, const char *phase, const std::string &error) const
{
	diagnostics_.push_back(MakeFailure(&adapter, phase, error));
}

void PluginHost::DispatchControl(ModelGeneration model_generation, const mjModel *model, mjData *data)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!ValidateModelGenerationLocked(model_generation))
		return;
	for (auto &entry : entries_) {
		if (entry.ready) {
			try {
				entry.adapter->Control(model, data);
			} catch (...) {
				RecordFailureLocked(*entry.adapter, "control", ExceptionMessage("Control"));
				throw;
			}
		}
	}
}

void PluginHost::DispatchPassive(ModelGeneration model_generation, const mjModel *model, mjData *data)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!ValidateModelGenerationLocked(model_generation))
		return;
	for (auto &entry : entries_) {
		if (entry.ready) {
			try {
				entry.adapter->Passive(model, data);
			} catch (...) {
				RecordFailureLocked(*entry.adapter, "passive", ExceptionMessage("Passive"));
				throw;
			}
		}
	}
}

void PluginHost::DispatchRender(ModelGeneration model_generation, const mjModel *model, mjData *data, mjvScene *scene)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!ValidateModelGenerationLocked(model_generation))
		return;
	for (auto &entry : entries_) {
		if (entry.ready) {
			try {
				entry.adapter->Render(model, data, scene);
			} catch (...) {
				RecordFailureLocked(*entry.adapter, "render", ExceptionMessage("Render"));
				throw;
			}
		}
	}
}

void PluginHost::DispatchLastStage(ModelGeneration model_generation, const mjModel *model, mjData *data)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!ValidateModelGenerationLocked(model_generation))
		return;
	for (auto &entry : entries_) {
		if (entry.ready) {
			try {
				entry.adapter->LastStage(model, data);
			} catch (...) {
				RecordFailureLocked(*entry.adapter, "last_stage", ExceptionMessage("LastStage"));
				throw;
			}
		}
	}
}

void PluginHost::Reset(ModelGeneration model_generation)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!ValidateModelGenerationLocked(model_generation))
		return;
	for (auto &entry : entries_) {
		if (entry.ready) {
			try {
				entry.adapter->Reset();
			} catch (...) {
				RecordFailureLocked(*entry.adapter, "reset", ExceptionMessage("Reset"));
				throw;
			}
		}
	}
}

void PluginHost::NotifyGeometryChanged(ModelGeneration model_generation, const mjModel *model, mjData *data,
                                       int geom_id)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!ValidateModelGenerationLocked(model_generation))
		return;
	for (auto &entry : entries_) {
		if (entry.ready) {
			try {
				entry.adapter->GeometryChanged(model, data, geom_id);
			} catch (...) {
				RecordFailureLocked(*entry.adapter, "geometry", ExceptionMessage("GeometryChanged"));
				throw;
			}
		}
	}
}

std::vector<PluginStat> PluginHost::Statistics() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	std::vector<PluginStat> statistics;
	statistics.reserve(entries_.size());
	for (const auto &entry : entries_) {
		try {
			statistics.push_back(entry.adapter->Statistics());
		} catch (...) {
			RecordFailureLocked(*entry.adapter, "statistics", ExceptionMessage("Statistics"));
			throw;
		}
	}
	return statistics;
}

std::vector<PluginStat> PluginHost::ActiveStatistics() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	std::vector<PluginStat> statistics;
	for (const auto &entry : entries_) {
		if (entry.ready) {
			try {
				statistics.push_back(entry.adapter->Statistics());
			} catch (...) {
				RecordFailureLocked(*entry.adapter, "statistics", ExceptionMessage("Statistics"));
				throw;
			}
		}
	}
	return statistics;
}

std::vector<PluginLoadFailure> PluginHost::Diagnostics() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return diagnostics_;
}

std::size_t PluginHost::ReadyCount() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return static_cast<std::size_t>(
	    std::count_if(entries_.begin(), entries_.end(), [](const auto &entry) { return entry.ready; }));
}

ModelGeneration PluginHost::ActiveModelGeneration() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!generation_active_) {
		throw std::runtime_error("no active Model Generation");
	}
	return model_generation_;
}

ScopedPluginAccess PluginHost::AcquireScopedAccess()
{
	std::unique_lock<std::mutex> lock(mutex_);
	if (!generation_active_) {
		throw std::runtime_error("no active Plugin Generation");
	}
	return ScopedPluginAccess(this, std::move(lock), model_generation_, generation_);
}

ScopedPluginAccess PluginHost::AcquireScopedAccess(PluginGeneration expected_generation)
{
	std::unique_lock<std::mutex> lock(mutex_);
	if (!generation_active_ || generation_ != expected_generation) {
		throw std::runtime_error("plugin handle belongs to an inactive Plugin Generation");
	}
	return ScopedPluginAccess(this, std::move(lock), model_generation_, generation_);
}

ScopedPluginAccess PluginHost::AcquireScopedAccess(ModelGeneration expected_model_generation,
                                                   PluginGeneration expected_generation)
{
	std::unique_lock<std::mutex> lock(mutex_);
	if (!generation_active_ || model_generation_ != expected_model_generation || generation_ != expected_generation) {
		throw std::runtime_error("plugin handle belongs to an inactive Model or Plugin Generation");
	}
	return ScopedPluginAccess(this, std::move(lock), model_generation_, generation_);
}

IPluginAdapter *PluginHost::FindAdapterLocked(const std::string &name, const std::string &type) const
{
	for (const auto &entry : entries_) {
		if (entry.ready && entry.adapter->Name() == name && entry.adapter->Type() == type) {
			return entry.adapter.get();
		}
	}
	throw std::runtime_error("active plugin adapter not found: '" + name + "' of type '" + type + "'");
}

IPluginAdapter *ScopedPluginAccess::Adapter(const std::string &name, const std::string &type) const
{
	if (!lock_.owns_lock()) {
		throw std::runtime_error("plugin access scope is not active");
	}
	return host_->FindAdapterLocked(name, type);
}

std::vector<PluginStat> ScopedPluginAccess::Statistics() const
{
	if (!lock_.owns_lock()) {
		throw std::runtime_error("plugin access scope is not active");
	}
	std::vector<PluginStat> statistics;
	for (const auto &entry : host_->entries_) {
		if (entry.ready) {
			try {
				statistics.push_back(entry.adapter->Statistics());
			} catch (...) {
				host_->RecordFailureLocked(*entry.adapter, "statistics", ExceptionMessage("Statistics"));
				throw;
			}
		}
	}
	return statistics;
}

} // namespace mujoco_ros
