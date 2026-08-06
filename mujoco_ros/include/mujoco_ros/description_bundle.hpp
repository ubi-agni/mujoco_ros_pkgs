#pragma once

#include <map>
#include <optional>
#include <string>

namespace mujoco_ros {

// Shared per-robot-instance description bundle. Public core fields only.
// value plus domain_id/attach_prefix as an *additive* read on the same
// top-level mujoco_server params, never a second parallel schema.
struct DescriptionSource
{
	enum class Kind
	{
		kFile,
		kTopic
	};
	Kind kind = Kind::kFile;
	std::string path; // when kind == kFile
	std::string topic; // when kind == kTopic, defaults applied by the parser
};

struct DescriptionBundle
{
	DescriptionSource urdf;
	std::optional<DescriptionSource> srdf; // absent => no disable_collisions
	bool convert_ascii_stl  = false;
	bool generate_actuators = false; // opt-in actuator generation
	std::string attach_prefix; // default ""; explicit multi-robot callers pass e.g. "r0_"/"r1_"
};

// Returns std::nullopt when neither "urdf.source" nor "srdf.source" is
// present in flat_params (the common case today: a plain `modelfile`-only
// launch) so callers can fall back to the pre-existing load path unchanged.
// Presence of only "srdf.source" is not a silent no-op: ParseDescriptionBundleFromMap
// throws because URDF configuration is required.
std::optional<DescriptionBundle>
TryParseDescriptionBundleFromMap(const std::map<std::string, std::string> &flat_params);

// Throws std::runtime_error if the bundle is absent or malformed (missing a
// required field for the declared source kind, or SRDF without URDF).
DescriptionBundle ParseDescriptionBundleFromMap(const std::map<std::string, std::string> &flat_params);

} // namespace mujoco_ros
