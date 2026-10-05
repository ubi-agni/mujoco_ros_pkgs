#pragma once

#include <mujoco/mujoco.h>

#include <memory>
#include <string>
#include <vector>

namespace mujoco_ros {

struct PluginStat
{
	std::string name;
	std::string type;
	double load_time               = 0.0;
	double reset_time              = 0.0;
	double ema_steptime_control    = 0.0;
	double ema_steptime_passive    = 0.0;
	double ema_steptime_render     = 0.0;
	double ema_steptime_last_stage = 0.0;
};

class IPluginAdapter
{
public:
	virtual ~IPluginAdapter() = default;

	virtual const std::string &Name() const                          = 0;
	virtual const std::string &Type() const                          = 0;
	virtual bool Load(const mjModel *, mjData *, std::string &error) = 0;
	virtual void Activate() {}
	virtual void Control(const mjModel *, mjData *)                      = 0;
	virtual void Passive(const mjModel *, mjData *)                      = 0;
	virtual void Render(const mjModel *, mjData *, mjvScene *)           = 0;
	virtual void LastStage(const mjModel *, mjData *)                    = 0;
	virtual void Reset()                                                 = 0;
	virtual void GeometryChanged(const mjModel *, mjData *, int geom_id) = 0;
	virtual PluginStat Statistics() const                                = 0;

	// Backend bindings use this only while a ScopedPluginAccess is held.
	virtual void *BackendObject() { return nullptr; }
};

class IPluginAdapterFactory
{
public:
	virtual ~IPluginAdapterFactory()                                      = default;
	virtual std::vector<std::unique_ptr<IPluginAdapter>> CreateAdapters() = 0;
};

} // namespace mujoco_ros
