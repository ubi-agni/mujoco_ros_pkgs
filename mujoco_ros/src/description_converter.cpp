#include <mujoco_ros/description_converter.hpp>

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <set>
#include <stdexcept>

#include <mujoco_ros/extended_params.hpp>
#include <mujoco_ros/logging.hpp>
#include <mujoco_ros/mesh_uri_prep.hpp>
#include <mujoco_ros/ros_version.hpp>
#include <tinyxml2.h>
#if MJR_ROS_VERSION == ROS_1
#include <ros/package.h>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace mujoco_ros {

namespace {

bool HasNonBlankName(const char *name)
{
	if (name == nullptr)
		return false;
	for (const unsigned char *p = reinterpret_cast<const unsigned char *>(name); *p != '\0'; ++p) {
		if (!std::isspace(*p))
			return true;
	}
	return false;
}

std::string MujocoRosSharePath()
{
#if MJR_ROS_VERSION == ROS_1
	std::string share = ros::package::getPath("mujoco_ros");
	if (share.empty())
		throw std::runtime_error("Robot Description Converter: ros::package::getPath('mujoco_ros') returned empty");
	return share;
#else
	return ament_index_cpp::get_package_share_directory("mujoco_ros");
#endif
}

mjsBody *FindRobotRootBody(mjSpec *robot_spec, const std::string &robot_root_body)
{
	if (!robot_root_body.empty()) {
		mjsBody *named_root = mjs_findBody(robot_spec, robot_root_body.c_str());
		if (named_root == nullptr) {
			throw std::runtime_error("Robot Description Converter: robot root body '" + robot_root_body +
			                         "' does not exist");
		}
		return named_root;
	}

	mjsBody *world = mjs_findBody(robot_spec, "world");
	if (world == nullptr) {
		throw std::runtime_error("Robot Description Converter: robot spec has no world body");
	}

	mjsElement *first_child = mjs_firstChild(world, mjOBJ_BODY, 0);
	if (first_child == nullptr) {
		throw std::runtime_error("Robot Description Converter: robot spec has no root body to attach");
	}

	mjsElement *second_child = mjs_nextChild(world, first_child, 0);
	if (second_child != nullptr) {
		throw std::runtime_error("Robot Description Converter: robot spec has multiple root bodies; set "
		                         "ComposeOptions::robot_root_body explicitly");
	}

	return mjs_asBody(first_child);
}

std::string BodyName(mjsBody *body)
{
	const mjString *name = mjs_getName(body->element);
	const char *text     = (name != nullptr) ? mjs_getString(name) : nullptr;
	return text != nullptr ? text : "";
}

bool HasPrimitiveGeometry(const tinyxml2::XMLElement *container)
{
	const tinyxml2::XMLElement *geometry = container->FirstChildElement("geometry");
	if (geometry == nullptr) {
		return false;
	}
	return geometry->FirstChildElement("box") != nullptr || geometry->FirstChildElement("cylinder") != nullptr ||
	       geometry->FirstChildElement("sphere") != nullptr;
}

bool AddPrimitiveVisualCollisions(tinyxml2::XMLDocument &doc)
{
	tinyxml2::XMLElement *robot = doc.FirstChildElement("robot");
	if (robot == nullptr) {
		return false;
	}

	bool changed = false;
	for (tinyxml2::XMLElement *link = robot->FirstChildElement("link"); link != nullptr;
	     link                       = link->NextSiblingElement("link")) {
		if (link->FirstChildElement("collision") != nullptr) {
			continue;
		}

		for (const tinyxml2::XMLElement *visual = link->FirstChildElement("visual"); visual != nullptr;
		     visual                             = visual->NextSiblingElement("visual")) {
			if (!HasPrimitiveGeometry(visual)) {
				continue;
			}

			tinyxml2::XMLElement *collision = doc.NewElement("collision");
			if (const tinyxml2::XMLElement *origin = visual->FirstChildElement("origin"); origin != nullptr) {
				collision->InsertEndChild(origin->DeepClone(&doc));
			}
			if (const tinyxml2::XMLElement *geometry = visual->FirstChildElement("geometry"); geometry != nullptr) {
				collision->InsertEndChild(geometry->DeepClone(&doc));
			}
			link->InsertEndChild(collision);
			changed = true;
		}
	}
	return changed;
}

// discardvisual/fusestatic are URDF-import-time compiler settings applied
// while mj_parseXMLString() runs -- unlike strippath, setting them on the
// resulting mjSpec* afterwards (as ConvertDescription does for
// documentation/defense-in-depth) has no effect on an import that already
// happened. The only way to actually control them is via the <mujoco>
// extension tag inside the URDF itself, so force it here before parsing:
// never let a distinct <visual> mesh silently vanish because a <collision>
// counterpart exists (Robot Description Converter policy -- render exactly
// what the URDF describes).
void EnsureCompilerOverrides(tinyxml2::XMLDocument &doc)
{
	tinyxml2::XMLElement *robot = doc.FirstChildElement("robot");
	if (robot == nullptr) {
		return;
	}

	tinyxml2::XMLElement *mujoco_tag = robot->FirstChildElement("mujoco");
	if (mujoco_tag == nullptr) {
		mujoco_tag = doc.NewElement("mujoco");
		robot->InsertFirstChild(mujoco_tag);
	}

	tinyxml2::XMLElement *compiler_tag = mujoco_tag->FirstChildElement("compiler");
	if (compiler_tag == nullptr) {
		compiler_tag = doc.NewElement("compiler");
		mujoco_tag->InsertEndChild(compiler_tag);
	}

	compiler_tag->SetAttribute("fusestatic", "false");
	compiler_tag->SetAttribute("discardvisual", "false");
}

std::string ApplyPrimitiveVisualCollisionsToString(const std::string &urdf_xml)
{
	tinyxml2::XMLDocument doc;
	tinyxml2::XMLError rc = doc.Parse(urdf_xml.c_str(), urdf_xml.size());
	if (rc != tinyxml2::XML_SUCCESS) {
		throw std::runtime_error("Robot Description Converter: failed to parse working URDF while preparing "
		                         "primitive visuals: " +
		                         std::string(doc.ErrorStr()));
	}

	AddPrimitiveVisualCollisions(doc);
	EnsureCompilerOverrides(doc);

	tinyxml2::XMLPrinter printer;
	doc.Print(&printer);
	return printer.CStr();
}

mjVFS BuildMeshVfs(const std::map<std::string, std::string> &basename_to_dir)
{
	mjVFS vfs;
	mj_defaultVFS(&vfs);
	for (const auto &[base, dir] : basename_to_dir) {
		int rc = mj_addFileVFS(&vfs, dir.c_str(), base.c_str());
		if (rc != 0) {
			mj_deleteVFS(&vfs);
			throw std::runtime_error("Robot Description Converter: mj_addFileVFS failed for '" + base + "' in '" + dir +
			                         "' (rc=" + std::to_string(rc) + ")");
		}
	}
	return vfs;
}

mjsElement *FindWorldAttachParent(mjSpec *world_spec, const std::string &world_frame)
{
	auto as_frame_under_body = [](mjsBody *body) -> mjsElement * {
		mjsFrame *frame = mjs_addFrame(body, nullptr);
		if (frame == nullptr) {
			throw std::runtime_error("Robot Description Converter: failed to add an attach frame under body '" +
			                         BodyName(body) + "'");
		}
		return frame->element;
	};

	auto worldbody = [&]() -> mjsElement * {
		mjsBody *world = mjs_findBody(world_spec, "world");
		if (world == nullptr) {
			throw std::runtime_error("Robot Description Converter: world spec has no world body");
		}
		return as_frame_under_body(world);
	};

	if (!world_frame.empty()) {
		mjsFrame *frame = mjs_findFrame(world_spec, world_frame.c_str());
		if (frame != nullptr) {
			return frame->element;
		}
		mjsBody *body = mjs_findBody(world_spec, world_frame.c_str());
		if (body != nullptr) {
			return as_frame_under_body(body);
		}
		if (world_frame == "spawn_frame") {
			return worldbody();
		}
		throw std::runtime_error("Robot Description Converter: world attach frame/body '" + world_frame +
		                         "' does not exist");
	}

	return worldbody();
}

} // namespace

JointCommandInterfaces ParseRos2ControlCommandInterfaces(const std::string &urdf_text)
{
	tinyxml2::XMLDocument doc;
	if (doc.Parse(urdf_text.c_str(), urdf_text.size()) != tinyxml2::XML_SUCCESS) {
		throw std::runtime_error("Robot Description Converter: failed to parse URDF while reading <ros2_control>: " +
		                         std::string(doc.ErrorStr() != nullptr ? doc.ErrorStr() : "unknown error"));
	}

	JointCommandInterfaces result;
	const tinyxml2::XMLElement *robot = doc.FirstChildElement("robot");
	if (robot == nullptr)
		return result;

	for (const tinyxml2::XMLElement *ros2_control = robot->FirstChildElement("ros2_control"); ros2_control != nullptr;
	     ros2_control                             = ros2_control->NextSiblingElement("ros2_control")) {
		for (const tinyxml2::XMLElement *joint = ros2_control->FirstChildElement("joint"); joint != nullptr;
		     joint                             = joint->NextSiblingElement("joint")) {
			const char *joint_name = joint->Attribute("name");
			if (!HasNonBlankName(joint_name)) {
				throw std::runtime_error(
				    "Robot Description Converter: <ros2_control><joint> is missing its 'name' attribute");
			}
			for (const tinyxml2::XMLElement *ci = joint->FirstChildElement("command_interface"); ci != nullptr;
			     ci                             = ci->NextSiblingElement("command_interface")) {
				const char *interface_name = ci->Attribute("name");
				if (!HasNonBlankName(interface_name)) {
					throw std::runtime_error("Robot Description Converter: joint '" + std::string(joint_name) +
					                         "' has a <command_interface> missing its 'name' attribute");
				}
				auto &requested = result[joint_name];
				if (std::find(requested.begin(), requested.end(), interface_name) == requested.end())
					requested.push_back(interface_name);
			}
		}
	}
	return result;
}

std::map<std::string, std::string> ParseMimicFollowers(const std::string &urdf_text)
{
	tinyxml2::XMLDocument doc;
	if (doc.Parse(urdf_text.c_str(), urdf_text.size()) != tinyxml2::XML_SUCCESS) {
		throw std::runtime_error("Robot Description Converter: failed to parse URDF while reading <mimic>: " +
		                         std::string(doc.ErrorStr() != nullptr ? doc.ErrorStr() : "unknown error"));
	}

	std::map<std::string, std::string> result;
	const tinyxml2::XMLElement *robot = doc.FirstChildElement("robot");
	if (robot == nullptr)
		return result;

	for (const tinyxml2::XMLElement *joint = robot->FirstChildElement("joint"); joint != nullptr;
	     joint                             = joint->NextSiblingElement("joint")) {
		const tinyxml2::XMLElement *mimic = joint->FirstChildElement("mimic");
		if (mimic == nullptr)
			continue;
		const char *follower_name = joint->Attribute("name");
		if (!HasNonBlankName(follower_name)) {
			throw std::runtime_error(
			    "Robot Description Converter: <mimic> parent <joint> is missing a nonblank 'name' attribute");
		}

		bool first_mimic = true;
		for (; mimic != nullptr; mimic = mimic->NextSiblingElement("mimic")) {
			if (!first_mimic) {
				throw std::runtime_error("Robot Description Converter: mimic follower '" + std::string(follower_name) +
				                         "' has multiple <mimic> children");
			}
			first_mimic             = false;
			const char *driver_name = mimic->Attribute("joint");
			if (!HasNonBlankName(driver_name)) {
				throw std::runtime_error("Robot Description Converter: <mimic> for follower '" +
				                         std::string(follower_name) + "' is missing a nonblank 'joint' attribute");
			}
			if (!result.emplace(follower_name, driver_name).second) {
				throw std::runtime_error("Robot Description Converter: duplicate mimic follower joint '" +
				                         std::string(follower_name) + "'");
			}
		}
	}
	return result;
}

namespace {

bool GlbBindingMatchesGeomMesh(const GlbVisualBinding &binding, const char *geom_mesh_name)
{
	if (geom_mesh_name == nullptr || geom_mesh_name[0] == '\0')
		return false;
	if (binding.mesh_basename == geom_mesh_name)
		return true;
	return std::filesystem::path(binding.mesh_basename).stem().string() == geom_mesh_name;
}

} // namespace

void ApplyGlbVisualMaterials(mjSpec *robot_spec, const std::vector<GlbVisualBinding> &bindings)
{
	if (bindings.empty()) {
		return;
	}

	mjsBody *world = mjs_findBody(robot_spec, "world");
	if (world == nullptr) {
		return;
	}

	for (const auto &binding : bindings) {
		if (binding.texture_basename.empty()) {
			continue;
		}

		const std::string tex_name = binding.material_name + "_tex";

		mjsTexture *tex = mjs_addTexture(robot_spec);
		mjs_setName(tex->element, tex_name.c_str());
		tex->type = mjTEXTURE_2D;
		mjs_setString(tex->file, binding.texture_basename.c_str());

		mjsMaterial *mat = mjs_addMaterial(robot_spec, nullptr);
		mjs_setName(mat->element, binding.material_name.c_str());
		mjs_setInStringVec(mat->textures, mjTEXROLE_RGB, tex_name.c_str());
	}

	for (mjsElement *element = mjs_firstChild(world, mjOBJ_GEOM, 1); element != nullptr;
	     element             = mjs_nextChild(world, element, 1)) {
		mjsGeom *geom = mjs_asGeom(element);
		if (geom == nullptr) {
			continue;
		}
		if (geom->contype != 0 || geom->conaffinity != 0) {
			continue;
		}

		const char *mesh_name = (geom->meshname != nullptr) ? mjs_getString(geom->meshname) : nullptr;
		if (mesh_name == nullptr || mesh_name[0] == '\0') {
			continue;
		}

		for (const auto &binding : bindings) {
			if (!GlbBindingMatchesGeomMesh(binding, mesh_name)) {
				continue;
			}
			if (!binding.texture_basename.empty()) {
				mjs_setString(geom->material, binding.material_name.c_str());
			} else if (binding.has_rgba) {
				for (int i = 0; i < 4; ++i) {
					geom->rgba[i] = binding.rgba[i];
				}
			}
			break;
		}
	}
}
void AssignRobotGeomGroups(mjSpec *robot_spec)
{
	mjsBody *world = mjs_findBody(robot_spec, "world");
	if (world == nullptr) {
		return;
	}

	bool has_visual = false;
	for (mjsElement *element = mjs_firstChild(world, mjOBJ_GEOM, 1); element != nullptr;
	     element             = mjs_nextChild(world, element, 1)) {
		mjsGeom *geom = mjs_asGeom(element);
		if (geom != nullptr && geom->contype == 0 && geom->conaffinity == 0) {
			has_visual = true;
			break;
		}
	}

	for (mjsElement *element = mjs_firstChild(world, mjOBJ_GEOM, 1); element != nullptr;
	     element             = mjs_nextChild(world, element, 1)) {
		mjsGeom *geom = mjs_asGeom(element);
		if (geom == nullptr) {
			continue;
		}
		if (geom->contype == 0 && geom->conaffinity == 0) {
			geom->group       = 1;
			geom->contype     = 0;
			geom->conaffinity = 0;
		} else {
			geom->group = has_visual ? 2 : 1;
		}
	}
}

void AssignGeneratedGeomNames(mjSpec *robot_spec)
{
	mjsBody *world = mjs_findBody(robot_spec, "world");
	if (world == nullptr)
		return;
	std::set<std::string> used_names;
	for (mjsElement *element = mjs_firstChild(world, mjOBJ_GEOM, 1); element != nullptr;
	     element             = mjs_nextChild(world, element, 1)) {
		const mjString *name = mjs_getName(element);
		const char *text     = name != nullptr ? mjs_getString(name) : nullptr;
		if (HasNonBlankName(text))
			used_names.emplace(text);
	}
	std::map<std::string, int> next_suffix;
	for (mjsElement *element = mjs_firstChild(world, mjOBJ_GEOM, 1); element != nullptr;
	     element             = mjs_nextChild(world, element, 1)) {
		const mjString *name = mjs_getName(element);
		const char *text     = name != nullptr ? mjs_getString(name) : nullptr;
		if (HasNonBlankName(text))
			continue;
		mjsGeom *geom  = mjs_asGeom(element);
		mjsBody *owner = mjs_getParent(element);
		if (geom == nullptr || owner == nullptr || BodyName(owner).empty())
			throw std::runtime_error("Robot Description Converter: converter-managed geom has no owning body name");
		const bool is_collision = geom->contype != 0 || geom->conaffinity != 0;
		const std::string base  = BodyName(owner) + (is_collision ? "_collision" : "_visual");
		int &suffix             = next_suffix[base];
		std::string generated;
		do {
			const int index = suffix++;
			generated       = index == 0 ? base : base + "_" + std::to_string(index + 1);
		} while (used_names.find(generated) != used_names.end());
		mjs_setName(element, generated.c_str());
		used_names.emplace(std::move(generated));
	}
}

mjSpec *LoadDefaultWorldSpec()
{
	const std::string path = MujocoRosSharePath() + "/assets/default_world.xml";
	char error[1000]       = { 0 };
	mjSpec *spec           = mj_parseXML(path.c_str(), nullptr, error, sizeof(error));
	if (spec == nullptr)
		throw std::runtime_error("Robot Description Converter: failed to parse default world '" + path + "': " + error);
	return spec;
}

mjModel *CompileWithMeshVfs(mjSpec *spec, const std::map<std::string, std::string> &basename_to_dir)
{
	mjVFS vfs      = BuildMeshVfs(basename_to_dir);
	mjModel *model = mj_compile(spec, &vfs);
	mj_deleteVFS(&vfs);
	return model;
}

namespace {
constexpr double kDefaultPositionKp = 10.0;
constexpr double kDefaultPositionKv = 1.0;
constexpr double kDefaultVelocityKv = 5.0;
struct UrdfJointLimit
{
	bool has_lower = false, has_upper = false, has_effort = false, has_velocity = false;
	double lower = 0.0, upper = 0.0, effort = 0.0, velocity = 0.0;
};
std::optional<UrdfJointLimit> FindUrdfJointLimit(const tinyxml2::XMLDocument &doc, const std::string &joint_name)
{
	const auto *robot = doc.FirstChildElement("robot");
	if (robot == nullptr)
		return std::nullopt;
	for (const auto *joint = robot->FirstChildElement("joint"); joint != nullptr;
	     joint             = joint->NextSiblingElement("joint")) {
		const char *name = joint->Attribute("name");
		if (name == nullptr || joint_name != name)
			continue;
		const auto *limit = joint->FirstChildElement("limit");
		if (limit == nullptr)
			return UrdfJointLimit{};
		UrdfJointLimit result;
		result.has_lower    = limit->QueryDoubleAttribute("lower", &result.lower) == tinyxml2::XML_SUCCESS;
		result.has_upper    = limit->QueryDoubleAttribute("upper", &result.upper) == tinyxml2::XML_SUCCESS;
		result.has_effort   = limit->QueryDoubleAttribute("effort", &result.effort) == tinyxml2::XML_SUCCESS;
		result.has_velocity = limit->QueryDoubleAttribute("velocity", &result.velocity) == tinyxml2::XML_SUCCESS;
		return result;
	}
	return std::nullopt;
}
void ConfigureActuatorForInterface(mjsActuator *act, const std::string &interface, const UrdfJointLimit &limit,
                                   const ActuatorOverride *override_params)
{
	double kp = kDefaultPositionKp;
	double kv = interface == "velocity" ? kDefaultVelocityKv : kDefaultPositionKv;
	if (override_params != nullptr) {
		if (override_params->kp)
			kp = *override_params->kp;
		if (override_params->kv)
			kv = *override_params->kv;
	}
	if (interface == "position") {
		act->gaintype     = mjGAIN_FIXED;
		act->gainprm[0]   = kp;
		act->biastype     = mjBIAS_AFFINE;
		act->biasprm[0]   = 0;
		act->biasprm[1]   = -kp;
		act->biasprm[2]   = -kv;
		act->ctrllimited  = mjLIMITED_TRUE;
		act->ctrlrange[0] = limit.lower;
		act->ctrlrange[1] = limit.upper;
	} else if (interface == "velocity") {
		act->gaintype     = mjGAIN_FIXED;
		act->gainprm[0]   = kv;
		act->biastype     = mjBIAS_AFFINE;
		act->biasprm[0]   = 0;
		act->biasprm[1]   = 0;
		act->biasprm[2]   = -kv;
		act->ctrllimited  = mjLIMITED_TRUE;
		act->ctrlrange[0] = -limit.velocity;
		act->ctrlrange[1] = limit.velocity;
	} else {
		act->gaintype     = mjGAIN_FIXED;
		act->gainprm[0]   = 1.0;
		act->biastype     = mjBIAS_NONE;
		act->gear[0]      = 1.0;
		act->ctrllimited  = mjLIMITED_TRUE;
		act->ctrlrange[0] = -limit.effort;
		act->ctrlrange[1] = limit.effort;
	}
	act->forcelimited  = mjLIMITED_TRUE;
	act->forcerange[0] = -limit.effort;
	act->forcerange[1] = limit.effort;
}
std::string SpecElementName(mjsElement *element)
{
	const mjString *name = mjs_getName(element);
	const char *text     = name != nullptr ? mjs_getString(name) : nullptr;
	return text != nullptr ? text : "";
}

std::string ActuatorTarget(mjsActuator *actuator)
{
	const char *text = mjs_getString(actuator->target);
	return text != nullptr ? text : "";
}

void LogSpecActuatorInventory(const char *stage, mjSpec *spec, const std::string &prefix = "")
{
	std::size_t count = 0;
	for (mjsElement *element = mjs_firstElement(spec, mjOBJ_ACTUATOR); element != nullptr;
	     element             = mjs_nextElement(spec, element)) {
		mjsActuator *actuator = mjs_asActuator(element);
		MJR_DEBUG_STREAM("[Robot Description Converter]["
		                 << stage << "] actuator[" << count << "] name='" << SpecElementName(element) << "' target='"
		                 << ActuatorTarget(actuator) << "' expected_composed_name='" << prefix + SpecElementName(element)
		                 << "' expected_composed_target='" << prefix + ActuatorTarget(actuator) << "'");
		++count;
	}
	MJR_INFO_STREAM("[Robot Description Converter][" << stage << "] actuator_count=" << count);
}

void LogModelActuatorInventory(const char *stage, const mjModel *model)
{
	MJR_INFO_STREAM("[Robot Description Converter][" << stage << "] model_nu=" << model->nu);
	for (int id = 0; id < model->nu; ++id) {
		const char *name = mj_id2name(model, mjOBJ_ACTUATOR, id);
		MJR_DEBUG_STREAM("[Robot Description Converter][" << stage << "] actuator[" << id << "] name='"
		                                                  << (name != nullptr ? name : "<unnamed>") << "'");
	}
}
} // namespace

void GenerateActuatorsFromRos2Control(mjSpec *spec, const std::string &urdf_text,
                                      const ExtendedParamsByJoint &extended_params)
{
	const JointCommandInterfaces interfaces = ParseRos2ControlCommandInterfaces(urdf_text);
	std::size_t interface_count             = 0;
	for (const auto &[joint_name, command_interfaces] : interfaces) {
		std::ostringstream names;
		for (std::size_t index = 0; index < command_interfaces.size(); ++index) {
			if (index != 0)
				names << ",";
			names << command_interfaces[index];
		}
		interface_count += command_interfaces.size();
		MJR_DEBUG_STREAM("[Robot Description Converter][parse] joint='" << joint_name << "' deduped_command_interfaces=["
		                                                                << names.str()
		                                                                << "] count=" << command_interfaces.size());
	}
	MJR_INFO_STREAM("[Robot Description Converter][parse] ros2_control_joint_count="
	                << interfaces.size() << " deduped_command_interface_count=" << interface_count);
	if (interfaces.empty()) {
		MJR_INFO("[Robot Description Converter][generation] skipped: no ros2_control command interfaces");
		return;
	}
	MJR_INFO("[Robot Description Converter][generation] enabled");
	tinyxml2::XMLDocument doc;
	if (doc.Parse(urdf_text.c_str(), urdf_text.size()) != tinyxml2::XML_SUCCESS)
		throw std::runtime_error("Robot Description Converter: failed to parse URDF while generating actuators");
	const auto mimic_followers                        = ParseMimicFollowers(urdf_text);
	const std::map<std::string, std::string> suffixes = { { "position", "_act_pos" },
		                                                   { "velocity", "_act_vel" },
		                                                   { "effort", "_act_eff" } };
	struct PlannedJoint
	{
		mjsElement *joint_element;
		std::string name;
		UrdfJointLimit limit;
		const ActuatorOverride *override_params;
		std::vector<std::string> interfaces;
		double armature;
		bool set_armature;
	};
	std::vector<PlannedJoint> plan;
	std::set<std::string> actuator_names;
	for (const auto &[joint_name, command_interfaces] : interfaces) {
		mjsElement *joint_element = mjs_findElement(spec, mjOBJ_JOINT, joint_name.c_str());
		if (joint_element == nullptr)
			throw std::runtime_error("Robot Description Converter: <ros2_control> references joint '" + joint_name +
			                         "' which does not exist in the URDF");
		if (const auto mimic = mimic_followers.find(joint_name); mimic != mimic_followers.end())
			throw std::runtime_error("Robot Description Converter: joint '" + joint_name + "' is a <mimic> follower of '" +
			                         mimic->second +
			                         "' but is also claimed by <ros2_control>; remove follower command_interface entries "
			                         "or remove/exclude its <mimic> declaration");
		const auto limit = FindUrdfJointLimit(doc, joint_name);
		if (!limit)
			throw std::runtime_error("Robot Description Converter: joint '" + joint_name + "' has no URDF <limit>");
		const auto ep = extended_params.find(joint_name);
		const ActuatorOverride *override_params =
		    ep != extended_params.end() && ep->second.actuator ? &*ep->second.actuator : nullptr;
		if (override_params != nullptr) {
			for (const auto &[field, value] :
			     std::initializer_list<std::pair<const char *, const std::optional<double> *>>{
			         { "kp", &override_params->kp },
			         { "kv", &override_params->kv },
			         { "armature", &override_params->armature } })
				if (*value && (!std::isfinite(**value) || **value < 0.0))
					throw std::runtime_error("Robot Description Converter: joint '" + joint_name + "' actuator override '" +
					                         field + "' must be finite and >= 0");
		}
		double armature   = 0.0;
		bool set_armature = false;
		if (override_params != nullptr && override_params->armature) {
			armature     = *override_params->armature;
			set_armature = true;
		}
		if (set_armature && (!std::isfinite(armature) || armature < 0.0))
			throw std::runtime_error("Robot Description Converter: joint '" + joint_name +
			                         "' has invalid derived armature");
		for (const auto &interface : command_interfaces) {
			const auto suffix = suffixes.find(interface);
			if (suffix == suffixes.end())
				throw std::runtime_error("Robot Description Converter: joint '" + joint_name +
				                         "' has unsupported <ros2_control> command_interface '" + interface + "'");
			if (!limit->has_effort || !std::isfinite(limit->effort) || limit->effort <= 0.0)
				throw std::runtime_error("Robot Description Converter: joint '" + joint_name +
				                         "' URDF <limit> effort must be finite and > 0");
			if (interface == "position" && (!limit->has_lower || !limit->has_upper || !std::isfinite(limit->lower) ||
			                                !std::isfinite(limit->upper) || limit->lower >= limit->upper))
				throw std::runtime_error("Robot Description Converter: joint '" + joint_name +
				                         "' URDF <limit> lower/upper must be finite and lower < upper");
			if (interface == "velocity" &&
			    (!limit->has_velocity || !std::isfinite(limit->velocity) || limit->velocity <= 0.0))
				throw std::runtime_error("Robot Description Converter: joint '" + joint_name +
				                         "' URDF <limit> velocity must be finite and > 0");
			const std::string actuator_name = joint_name + suffix->second;
			if (!actuator_names.insert(actuator_name).second ||
			    mjs_findElement(spec, mjOBJ_ACTUATOR, actuator_name.c_str()) != nullptr)
				throw std::runtime_error("Robot Description Converter: duplicate actuator name '" + actuator_name + "'");
		}
		plan.push_back(
		    { joint_element, joint_name, *limit, override_params, command_interfaces, armature, set_armature });
	}
	for (const auto &joint : plan) {
		if (joint.set_armature)
			mjs_asJoint(joint.joint_element)->armature = joint.armature;
		for (const auto &interface : joint.interfaces) {
			const auto suffix = suffixes.find(interface);
			mjsActuator *act  = mjs_addActuator(spec, nullptr);
			mjs_setName(act->element, (joint.name + suffix->second).c_str());
			act->trntype = mjTRN_JOINT;
			mjs_setString(act->target, joint.name.c_str());
			ConfigureActuatorForInterface(act, interface, joint.limit, joint.override_params);
			MJR_DEBUG_STREAM("[Robot Description Converter][generation] actuator='"
			                 << joint.name + suffix->second << "' source_joint='" << joint.name << "' interface='"
			                 << interface << "' target_joint='" << ActuatorTarget(act) << "' gain=" << act->gainprm[0]
			                 << " bias_velocity=" << -act->biasprm[2] << " ctrlrange=[" << act->ctrlrange[0] << ","
			                 << act->ctrlrange[1] << "] forcerange=[" << act->forcerange[0] << "," << act->forcerange[1]
			                 << "]");
		}
	}
	MJR_INFO_STREAM("[Robot Description Converter][generation] generated_actuator_count=" << actuator_names.size());
}

void ApplyGravcompOverrides(mjSpec *spec, const ParsedSrdfExtensions &srdf)
{
	for (const auto &[joint_name, entry] : srdf.entries) {
		if (!entry.gravcomp)
			continue;
		mjsElement *joint = mjs_findElement(spec, mjOBJ_JOINT, joint_name.c_str());
		if (joint == nullptr)
			continue;
		mjsBody *body = mjs_getParent(joint);
		if (body != nullptr)
			body->gravcomp = entry.gravcomp->value;
	}
}

ConversionResult ConvertDescription(const std::string &urdf_path, const std::string &srdf_path,
                                    const MeshPrepOptions &mesh_options, bool generate_actuators,
                                    ConverterExtensionContext *extension_context)
{
	std::ifstream in(urdf_path);
	if (!in)
		throw std::runtime_error("Robot Description Converter: failed to read URDF '" + urdf_path + "'");
	std::ostringstream urdf_buf;
	urdf_buf << in.rdbuf();
	std::string urdf_text = urdf_buf.str();

	MeshPrepResult prep   = PrepareUrdfMeshes(urdf_text, std::filesystem::path(urdf_path).parent_path(), mesh_options);
	std::string ready_xml = ApplyPrimitiveVisualCollisionsToString(prep.working_urdf);

	mjVFS vfs = BuildMeshVfs(prep.basename_to_dir);

	char error[1000] = { 0 };
	mjSpec *spec     = mj_parseXMLString(ready_xml.c_str(), &vfs, error, sizeof(error));
	mj_deleteVFS(&vfs);
	if (spec == nullptr)
		throw std::runtime_error("Robot Description Converter: failed to parse URDF '" + urdf_path + "': " + error);

	// Binding decision 11: never rely on a version's URDF-strippath default.
	spec->strippath              = 1;
	spec->compiler.fusestatic    = 0;
	spec->compiler.discardvisual = 0;
	AssignRobotGeomGroups(spec);
	AssignGeneratedGeomNames(spec);
	ApplyGlbVisualMaterials(spec, prep.glb_visual_bindings);
	MJR_INFO_STREAM("[Robot Description Converter][generation] requested=" << std::boolalpha << generate_actuators);

	ParsedSrdfExtensions srdf_extensions;
	try {
		if (!srdf_path.empty()) {
			srdf_extensions = ParseSrdfExtensions(srdf_path);
			for (const auto &exclusion : srdf_extensions.collision_exclusions) {
				mjsExclude *excl = mjs_addExclude(spec);
				mjs_setString(excl->bodyname1, exclusion.link1.c_str());
				mjs_setString(excl->bodyname2, exclusion.link2.c_str());
			}
		}
		ConverterExtensionContext local_context;
		ConverterExtensionContext &context = extension_context != nullptr ? *extension_context : local_context;
		for (const auto &[joint_name, entry] : srdf_extensions.entries)
			DispatchCustomExtendedParams(joint_name, entry, GetGlobalExtendedParamsRegistry(), context);
		ApplyGravcompOverrides(spec, srdf_extensions);
	} catch (...) {
		mj_deleteSpec(spec);
		throw;
	}

	ConversionResult result;
	result.spec                = spec;
	result.basename_to_dir     = prep.basename_to_dir;
	result.glb_visual_bindings = prep.glb_visual_bindings;
	for (const auto &[joint_name, entry] : srdf_extensions.entries) {
		if (entry.joint_params.has_value())
			result.extended_params.emplace(joint_name, *entry.joint_params);
	}
	if (generate_actuators) {
		try {
			GenerateActuatorsFromRos2Control(spec, urdf_text, result.extended_params);
		} catch (...) {
			mj_deleteSpec(spec);
			throw;
		}
	} else {
		MJR_INFO("[Robot Description Converter][generation] skipped: description.generate_actuators=false");
	}
	LogSpecActuatorInventory("standalone", result.spec);
	return result;
}

mjsElement *ComposeIntoWorld(mjSpec *world_spec, mjSpec *robot_spec, const ComposeOptions &options)
{
	mjsBody *robot_root         = FindRobotRootBody(robot_spec, options.robot_root_body);
	const std::string root_name = BodyName(robot_root);
	const std::string prefix    = options.prefix;
	MJR_INFO_STREAM("[Robot Description Converter][compose-before] attach_prefix='" << prefix << "'");
	LogSpecActuatorInventory("compose-standalone", robot_spec, prefix);
	LogSpecActuatorInventory("compose-world-before", world_spec);

	// MuJoCo 3.3.5's mjs_attach does not reject duplicate prefixed names
	// (returns non-null; mj_compile later fails with "repeated name").
	// Detect the collision up front so ComposeIntoWorld fails loud without
	// mutating world_spec into an uncompilable state.
	const std::string prefixed_root = prefix + root_name;
	if (mjs_findBody(world_spec, prefixed_root.c_str()) != nullptr)
		throw std::runtime_error("Robot Description Converter: prefix '" + prefix +
		                         "' already used in world_spec (body '" + prefixed_root +
		                         "' exists) -- refuse attach, do not retry with same prefix");

	mjsElement *parent = FindWorldAttachParent(world_spec, options.world_frame);

	mjsElement *attached = mjs_attach(parent, robot_root->element, prefix.c_str(), "");
	if (attached == nullptr)
		throw std::runtime_error("Robot Description Converter: mjs_attach() rejected prefix '" + prefix +
		                         "' -- world_spec is now poisoned, discard and rebuild it, do not retry");

	LogSpecActuatorInventory("compose-world-after", world_spec);
	MJR_INFO_STREAM("[Robot Description Converter][compose-after] attach_prefix='"
	                << prefix << "' prefix_applies_to_actuator_names_and_joint_targets=true");

	if (options.pos != nullptr || options.quat != nullptr) {
		mjsBody *attached_body = mjs_asBody(attached);
		if (options.pos != nullptr)
			for (int i = 0; i < 3; ++i)
				attached_body->pos[i] = options.pos[i];
		if (options.quat != nullptr)
			for (int i = 0; i < 4; ++i)
				attached_body->quat[i] = options.quat[i];
	}
	return attached;
}

mjsElement *ComposeIntoWorld(mjSpec *world_spec, mjSpec *robot_spec, const std::string &prefix, const mjtNum pos[3],
                             const mjtNum quat[4])
{
	ComposeOptions options;
	options.prefix = prefix;
	options.pos    = pos;
	options.quat   = quat;
	return ComposeIntoWorld(world_spec, robot_spec, options);
}

std::string SaveDescriptionToTempMjb(const std::string &urdf_path, const std::string &srdf_path, mjSpec *world_spec,
                                     const MeshPrepOptions &mesh_options)
{
	return SaveDescriptionToTempMjb(urdf_path, srdf_path, world_spec, mesh_options, false, "");
}

std::string SaveDescriptionToTempMjb(const std::string &urdf_path, const std::string &srdf_path, mjSpec *world_spec,
                                     const MeshPrepOptions &mesh_options, bool generate_actuators,
                                     const std::string &attach_prefix, ConverterExtensionContext *extension_context)
{
	ConversionResult robot =
	    ConvertDescription(urdf_path, srdf_path, mesh_options, generate_actuators, extension_context);

	mjSpec *owned_default   = nullptr;
	mjSpec *effective_world = world_spec;
	if (effective_world == nullptr) {
		owned_default   = LoadDefaultWorldSpec();
		effective_world = owned_default;
	}

	mjSpec *compose_target = mj_copySpec(effective_world);
	try {
		ComposeIntoWorld(compose_target, robot.spec, attach_prefix);
	} catch (...) {
		// Same poison-spec leak policy as today: do not mj_deleteSpec(compose_target).
		mj_deleteSpec(robot.spec);
		if (owned_default != nullptr)
			mj_deleteSpec(owned_default);
		throw;
	}

	LogSpecActuatorInventory("compile-before", compose_target);
	mjModel *model = CompileWithMeshVfs(compose_target, robot.basename_to_dir);
	if (model == nullptr) {
		std::string err = mjs_getError(compose_target);
		mj_deleteSpec(compose_target);
		mj_deleteSpec(robot.spec);
		if (owned_default != nullptr)
			mj_deleteSpec(owned_default);
		throw std::runtime_error("Robot Description Converter: mj_compile failed: " + err);
	}

	LogModelActuatorInventory("compile-after", model);
	std::string tmp_path =
	    (std::filesystem::temp_directory_path() /
	     ("mujoco_ros_description_" + std::to_string(reinterpret_cast<std::uintptr_t>(model)) + ".mjb"))
	        .string();
	mj_saveModel(model, tmp_path.c_str(), nullptr, 0);

	mj_deleteModel(model);
	mj_deleteSpec(compose_target);
	mj_deleteSpec(robot.spec);
	if (owned_default != nullptr)
		mj_deleteSpec(owned_default);

	return tmp_path;
}

std::pair<mjModel *, mjData *> load_model_from_description(const std::string &urdf_path, const std::string &srdf_path,
                                                           mjSpec *world_spec, const MeshPrepOptions &mesh_options)
{
	return load_model_from_description(urdf_path, srdf_path, world_spec, mesh_options, false, "");
}

std::pair<mjModel *, mjData *> load_model_from_description(const std::string &urdf_path, const std::string &srdf_path,
                                                           mjSpec *world_spec, const MeshPrepOptions &mesh_options,
                                                           bool generate_actuators, const std::string &attach_prefix,
                                                           ConverterExtensionContext *extension_context)
{
	std::string tmp_path = SaveDescriptionToTempMjb(urdf_path, srdf_path, world_spec, mesh_options, generate_actuators,
	                                                attach_prefix, extension_context);

	mjModel *reloaded = mj_loadModel(tmp_path.c_str(), nullptr);
	std::remove(tmp_path.c_str());
	if (reloaded == nullptr) {
		throw std::runtime_error("Robot Description Converter: failed to reload compiled model from " + tmp_path);
	}
	LogModelActuatorInventory("reload-after", reloaded);
	mjData *data = mj_makeData(reloaded);

	return { reloaded, data };
}

} // namespace mujoco_ros
