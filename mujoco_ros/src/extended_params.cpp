#include <mujoco_ros/extended_params.hpp>
#include <mujoco_ros/logging.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace mujoco_ros {

namespace {

bool HasNonBlankAttribute(const char *value)
{
	return value != nullptr && std::string(value).find_first_not_of(" \t\n\r\f\v") != std::string::npos;
}

void LoadRequiredXml(tinyxml2::XMLDocument &doc, const std::string &path, const char *document_type)
{
	if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
		throw std::runtime_error(std::string(document_type) + ": failed to load or parse '" + path +
		                         "': " + std::string(doc.ErrorStr() != nullptr ? doc.ErrorStr() : "unknown error"));
	}
	if (doc.RootElement() == nullptr)
		throw std::runtime_error(std::string(document_type) + ": '" + path + "' has no root element");
}

double RequireDouble(const tinyxml2::XMLElement *elem, const char *attr, const std::string &joint_name)
{
	double value = 0.0;
	if (elem->QueryDoubleAttribute(attr, &value) != tinyxml2::XML_SUCCESS || !std::isfinite(value)) {
		throw std::runtime_error("Extended Params: joint '" + joint_name + "' element <" + elem->Name() +
		                         "> has invalid finite-number attribute '" + attr + "'");
	}
	return value;
}

std::optional<double> ParseOptionalFiniteDouble(const tinyxml2::XMLElement *elem, const char *attr,
                                                const std::string &joint_name)
{
	if (elem->Attribute(attr) == nullptr)
		return std::nullopt;

	double value = 0.0;
	if (elem->QueryDoubleAttribute(attr, &value) != tinyxml2::XML_SUCCESS || !std::isfinite(value)) {
		throw std::runtime_error("Extended Params: joint '" + joint_name +
		                         "' element <mujoco_actuator> has invalid finite-number attribute '" + attr + "'");
	}
	if (value < 0.0) {
		throw std::runtime_error("Extended Params: joint '" + joint_name + "' element <mujoco_actuator> attribute '" +
		                         attr + "' must be finite and >= 0");
	}
	return value;
}

std::optional<ActuatorOverride> ParseActuatorOverride(const tinyxml2::XMLElement *joint_elem,
                                                      const std::string &joint_name)
{
	const auto *elem = joint_elem->FirstChildElement("mujoco_actuator");
	if (elem == nullptr)
		return std::nullopt;
	if (elem->NextSiblingElement("mujoco_actuator") != nullptr) {
		throw std::runtime_error("Extended Params: joint '" + joint_name + "' has multiple <mujoco_actuator> elements");
	}

	ActuatorOverride ao;
	ao.kp       = ParseOptionalFiniteDouble(elem, "kp", joint_name);
	ao.kv       = ParseOptionalFiniteDouble(elem, "kv", joint_name);
	ao.armature = ParseOptionalFiniteDouble(elem, "armature", joint_name);
	return ao;
}

JointExtendedParams ParseJointParams(const tinyxml2::XMLElement *elem, const std::string &joint_name)
{
	JointExtendedParams params;
	params.actuator = ParseActuatorOverride(elem, joint_name);
	return params;
}

bool HasJointParams(const JointExtendedParams &params)
{
	return params.actuator.has_value();
}

std::optional<GravcompOverride> ParseGravcomp(const tinyxml2::XMLElement *elem, const std::string &joint_name)
{
	const auto *gravcomp = elem->FirstChildElement("mujoco_gravcomp");
	if (gravcomp == nullptr)
		return std::nullopt;
	if (gravcomp->NextSiblingElement("mujoco_gravcomp") != nullptr) {
		throw std::runtime_error("SRDF: extended_params for joint '" + joint_name +
		                         "' has multiple <mujoco_gravcomp> elements");
	}

	const double value = RequireDouble(gravcomp, "value", joint_name);
	if (!std::isfinite(value)) {
		throw std::runtime_error("SRDF: extended_params for joint '" + joint_name +
		                         "' has non-finite <mujoco_gravcomp> value");
	}
	return GravcompOverride{ value };
}

void RemoveBuiltInChildren(tinyxml2::XMLElement *elem)
{
	const char *builtin_tags[] = { "mujoco_actuator", "mujoco_gravcomp" };
	for (const char *tag : builtin_tags) {
		for (auto *child = elem->FirstChildElement(tag); child != nullptr;) {
			auto *next = child->NextSiblingElement(tag);
			elem->DeleteChild(child);
			child = next;
		}
	}
}

void PreserveCustomXml(const tinyxml2::XMLElement *source, ExtendedParamsEntry &entry)
{
	auto custom_doc  = std::make_unique<tinyxml2::XMLDocument>();
	auto *custom_xml = source->DeepClone(custom_doc.get())->ToElement();
	custom_doc->InsertEndChild(custom_xml);
	RemoveBuiltInChildren(custom_xml);
	if (custom_xml->FirstChildElement() != nullptr) {
		entry.custom_xml = custom_xml;
		entry.custom_doc = std::move(custom_doc);
	}
}

void ParseDisabledCollisions(const tinyxml2::XMLElement *root, const std::string &srdf_path,
                             std::vector<CollisionExclusion> &result)
{
	for (const auto *elem = root->FirstChildElement("disable_collisions"); elem != nullptr;
	     elem             = elem->NextSiblingElement("disable_collisions")) {
		const char *link1 = elem->Attribute("link1");
		const char *link2 = elem->Attribute("link2");
		if (!HasNonBlankAttribute(link1) || !HasNonBlankAttribute(link2)) {
			throw std::runtime_error("SRDF: '" + srdf_path + "' has a <disable_collisions> missing or blank link1/link2");
		}
		result.push_back({ link1, link2 });
	}
}

void ParseExtendedParamEntries(const tinyxml2::XMLElement *root, const std::string &srdf_path,
                               std::map<std::string, ExtendedParamsEntry> &entries)
{
	for (const auto *elem = root->FirstChildElement("extended_params"); elem != nullptr;
	     elem             = elem->NextSiblingElement("extended_params")) {
		const char *name = elem->Attribute("name");
		if (!HasNonBlankAttribute(name)) {
			throw std::runtime_error("SRDF: '" + srdf_path +
			                         "' has an <extended_params> with a missing or blank 'name' attribute");
		}

		ExtendedParamsEntry entry;
		auto joint_params = ParseJointParams(elem, name);
		if (HasJointParams(joint_params))
			entry.joint_params = std::move(joint_params);
		entry.gravcomp = ParseGravcomp(elem, name);
		PreserveCustomXml(elem, entry);
		if (!entries.emplace(name, std::move(entry)).second) {
			throw std::runtime_error("SRDF: '" + srdf_path + "' has duplicate <extended_params> for joint '" + name + "'");
		}
	}
}
} // namespace

void ExtendedParamsHandlerRegistry::Register(const std::string &tag_name, ExtendedParamsCustomHandler handler)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (tag_name.empty() || !handler)
		throw std::runtime_error("Extended Params: custom handler requires non-empty tag and callable handler");
	if (!handlers_.emplace(tag_name, std::move(handler)).second)
		throw std::runtime_error("Extended Params: duplicate custom handler for tag '" + tag_name + "'");
}

const ExtendedParamsCustomHandler *ExtendedParamsHandlerRegistry::Find(const std::string &tag_name) const
{
	std::lock_guard<std::mutex> lock(mutex_);
	auto it = handlers_.find(tag_name);
	return it == handlers_.end() ? nullptr : &it->second;
}

ExtendedParamsHandlerRegistry &GetGlobalExtendedParamsRegistry()
{
	static ExtendedParamsHandlerRegistry registry;
	return registry;
}

void RegisterExtendedParamsHandler(const std::string &tag_name, ExtendedParamsCustomHandler handler)
{
	GetGlobalExtendedParamsRegistry().Register(tag_name, std::move(handler));
}

void DispatchCustomExtendedParams(const std::string &joint_name, const ExtendedParamsEntry &entry,
                                  const ExtendedParamsHandlerRegistry &registry, ConverterExtensionContext &context)
{
	if (entry.custom_xml == nullptr)
		return;
	for (const auto *custom = entry.custom_xml->FirstChildElement(); custom != nullptr;
	     custom             = custom->NextSiblingElement()) {
		const auto *handler = registry.Find(custom->Name());
		if (handler == nullptr) {
			MJR_WARN_STREAM("Extended Params: joint '" << joint_name << "' has no registered handler for tag '"
			                                           << custom->Name() << "' -- skipping");
			continue;
		}
		(*handler)(joint_name, custom->Name(), *custom, context);
	}
}

void DispatchCustomExtendedParams(const std::string &joint_name, const ExtendedParamsEntry &entry,
                                  const ExtendedParamsHandlerRegistry &registry)
{
	ConverterExtensionContext context;
	DispatchCustomExtendedParams(joint_name, entry, registry, context);
}

ParsedSrdfExtensions ParseSrdfExtensions(const std::string &srdf_path)
{
	tinyxml2::XMLDocument doc;
	LoadRequiredXml(doc, srdf_path, "SRDF");
	if (std::string(doc.RootElement()->Name()) != "robot") {
		throw std::runtime_error("SRDF: '" + srdf_path + "' must have a <robot> root element");
	}

	ParsedSrdfExtensions result;
	ParseDisabledCollisions(doc.RootElement(), srdf_path, result.collision_exclusions);
	ParseExtendedParamEntries(doc.RootElement(), srdf_path, result.entries);
	return result;
}

std::vector<CollisionExclusion> ParseDisabledCollisions(const std::string &srdf_path)
{
	tinyxml2::XMLDocument doc;
	LoadRequiredXml(doc, srdf_path, "SRDF");
	if (std::string(doc.RootElement()->Name()) != "robot") {
		throw std::runtime_error("SRDF: '" + srdf_path + "' must have a <robot> root element");
	}

	std::vector<CollisionExclusion> result;
	ParseDisabledCollisions(doc.RootElement(), srdf_path, result);
	return result;
}

} // namespace mujoco_ros
