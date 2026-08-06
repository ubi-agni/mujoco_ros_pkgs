#pragma once

#include <any>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <functional>
#include <string>
#include <vector>

#include <tinyxml2.h>

namespace mujoco_ros {

// Purely a pass-through bag: core never inspects user_data's contents.
// A caller that wants results back from a registered custom handler
// owns an accumulator, stashes a pointer to it here,
// and reads it back after ConvertDescription returns.
struct ConverterExtensionContext
{
	std::any user_data;
};

using ExtendedParamsCustomHandler = std::function<void(const std::string &, const std::string &,
                                                       const tinyxml2::XMLElement &, ConverterExtensionContext &)>;

// Register/Find are mutex-guarded so the global instance below can be
// safely populated from one thread (e.g. a handler-providing shared
// library's dlopen-time static initializer) while another thread concurrently
// calls Find (e.g. ConvertDescription running for a different robot).
class ExtendedParamsHandlerRegistry
{
public:
	void Register(const std::string &tag_name, ExtendedParamsCustomHandler handler);
	const ExtendedParamsCustomHandler *Find(const std::string &tag_name) const;

private:
	mutable std::mutex mutex_;
	std::map<std::string, ExtendedParamsCustomHandler> handlers_;
};

// Global registry populated by packages that want to interpret custom
// Extended Params tags. Backed by a function-local static singleton, so
// it is safe to populate from a file-scope static registrar object in any
// shared library -- static init for that library runs at dlopen/load time,
// strictly before any of that same library's own code can call into
// ConvertDescription. Concurrent Register/Find calls from any thread are
// safe: ExtendedParamsHandlerRegistry guards its internal state with a mutex.
ExtendedParamsHandlerRegistry &GetGlobalExtendedParamsRegistry();

// Registers `handler` for `tag_name` on the global registry. Throws
// std::runtime_error (via ExtendedParamsHandlerRegistry::Register) if
// `tag_name` is empty, `handler` is not callable, or `tag_name` already
// has a handler registered.
void RegisterExtendedParamsHandler(const std::string &tag_name, ExtendedParamsCustomHandler handler);

// Per-joint override for generated-actuator gains/dynamics. Each field is
// independently optional: callers may override one value and leave others at
// fixed or derived defaults. Force and control ranges stay URDF-derived.
struct ActuatorOverride
{
	std::optional<double> kp;
	std::optional<double> kv;
	std::optional<double> armature;
};

struct JointExtendedParams
{
	std::optional<ActuatorOverride> actuator;
};

using ExtendedParamsByJoint = std::map<std::string, JointExtendedParams>;

struct CollisionExclusion
{
	std::string link1;
	std::string link2;
};

struct GravcompOverride
{
	double value;
};

// An SRDF <extended_params name="..."> container after built-in children have
// been parsed. custom_xml owns only extension tags intended for future custom
// dispatch and remains valid for this entry's lifetime.
struct ExtendedParamsEntry
{
	std::optional<JointExtendedParams> joint_params;
	std::optional<GravcompOverride> gravcomp;
	std::unique_ptr<tinyxml2::XMLDocument> custom_doc;
	tinyxml2::XMLElement *custom_xml = nullptr;
};

struct ParsedSrdfExtensions
{
	std::map<std::string, ExtendedParamsEntry> entries;
	std::vector<CollisionExclusion> collision_exclusions;
};

// Parses canonical SRDF extensions and disable_collisions together. Built-in
// children are consumed before any custom XML is exposed.
ParsedSrdfExtensions ParseSrdfExtensions(const std::string &srdf_path);

void DispatchCustomExtendedParams(const std::string &joint_name, const ExtendedParamsEntry &entry,
                                  const ExtendedParamsHandlerRegistry &registry, ConverterExtensionContext &context);
void DispatchCustomExtendedParams(const std::string &joint_name, const ExtendedParamsEntry &entry,
                                  const ExtendedParamsHandlerRegistry &registry);

// Reads only <disable_collisions link1=... link2=.../> tags.
// <group>/<group_state> are intentionally never interpreted here,
// they stay MoveIt's concern.
std::vector<CollisionExclusion> ParseDisabledCollisions(const std::string &srdf_path);

} // namespace mujoco_ros
