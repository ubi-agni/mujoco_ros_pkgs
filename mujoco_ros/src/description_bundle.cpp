#include <mujoco_ros/description_bundle.hpp>

#include <stdexcept>

namespace mujoco_ros {

namespace {

std::optional<std::string> Get(const std::map<std::string, std::string> &m, const std::string &key)
{
	auto it = m.find(key);
	return it == m.end() ? std::nullopt : std::optional<std::string>(it->second);
}

bool ParseOptionalBool(const std::map<std::string, std::string> &m, const std::string &key)
{
	auto val = Get(m, key);
	if (!val.has_value())
		return false;
	if (*val == "true")
		return true;
	if (*val == "false")
		return false;
	throw std::runtime_error("Description bundle: '" + key + "' must be 'true' or 'false', got '" + *val + "'");
}

DescriptionSource ParseSource(const std::map<std::string, std::string> &m, const std::string &prefix,
                              const std::string &default_param_name)
{
	DescriptionSource source;
	auto kind = Get(m, prefix + ".source");
	if (!kind.has_value())
		throw std::runtime_error("Description bundle: missing '" + prefix + ".source'");

	if (*kind == "file") {
		source.kind = DescriptionSource::Kind::kFile;
		auto path   = Get(m, prefix + ".path");
		if (!path.has_value())
			throw std::runtime_error("Description bundle: '" + prefix + ".source' is 'file' but '" + prefix +
			                         ".path' is missing");
		source.path = *path;
	} else if (*kind == "topic") {
		source.kind  = DescriptionSource::Kind::kTopic;
		source.topic = Get(m, prefix + ".topic").value_or(default_param_name);
	} else {
		throw std::runtime_error("Description bundle: '" + prefix + ".source' must be 'file' or 'topic', got '" + *kind +
		                         "'");
	}
	return source;
}

} // namespace

std::optional<DescriptionBundle> TryParseDescriptionBundleFromMap(const std::map<std::string, std::string> &flat_params)
{
	if (!Get(flat_params, "urdf.source").has_value()) {
		if (Get(flat_params, "srdf.source").has_value())
			return ParseDescriptionBundleFromMap(flat_params); // throws: URDF required
		return std::nullopt;
	}
	return ParseDescriptionBundleFromMap(flat_params);
}

DescriptionBundle ParseDescriptionBundleFromMap(const std::map<std::string, std::string> &flat_params)
{
	DescriptionBundle bundle;
	bundle.urdf = ParseSource(flat_params, "urdf", "robot_description");
	if (Get(flat_params, "srdf.source").has_value())
		bundle.srdf = ParseSource(flat_params, "srdf", "robot_description_semantic");
	bundle.convert_ascii_stl  = ParseOptionalBool(flat_params, "description.convert_ascii_stl");
	bundle.generate_actuators = ParseOptionalBool(flat_params, "description.generate_actuators");
	bundle.attach_prefix      = Get(flat_params, "description.attach_prefix").value_or("");
	return bundle;
}

} // namespace mujoco_ros
