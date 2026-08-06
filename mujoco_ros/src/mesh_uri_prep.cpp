#include <mujoco_ros/mesh_uri_prep.hpp>

#include <mujoco_ros/ascii_stl_convert.hpp>
#include <mujoco_ros/glb_visual_prep.hpp>
#include <mujoco_ros/ros_version.hpp>
#include <tinyxml2.h>

#include <cctype>
#include <functional>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#if MJR_ROS_VERSION == ROS_1
#include <ros/package.h>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace mujoco_ros {

namespace {

std::string ToLower(std::string s)
{
	for (char &c : s)
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	return s;
}

bool EqualsIgnoreCase(const std::string &a, const std::string &b)
{
	return ToLower(a) == ToLower(b);
}

bool IsGlbExtension(const std::filesystem::path &p)
{
	return EqualsIgnoreCase(p.extension().string(), ".glb");
}

bool IsStlExtension(const std::filesystem::path &p)
{
	return EqualsIgnoreCase(p.extension().string(), ".stl");
}

bool IsObjExtension(const std::filesystem::path &p)
{
	return EqualsIgnoreCase(p.extension().string(), ".obj");
}

bool IsSupportedMeshExt(const std::filesystem::path &p)
{
	const std::string ext = p.extension().string();
	return EqualsIgnoreCase(ext, ".stl") || EqualsIgnoreCase(ext, ".obj");
}

std::optional<std::filesystem::path> FindCaseInsensitiveStlInDir(const std::filesystem::path &dir,
                                                                 const std::string &stem)
{
	if (!std::filesystem::exists(dir))
		return std::nullopt;

	const std::string stem_lower = ToLower(stem);
	for (const auto &entry : std::filesystem::directory_iterator(dir)) {
		if (!entry.is_regular_file())
			continue;
		const std::filesystem::path &candidate = entry.path();
		if (ToLower(candidate.stem().string()) == stem_lower && IsStlExtension(candidate))
			return candidate;
	}
	return std::nullopt;
}

std::optional<std::filesystem::path> FindCaseInsensitiveObjInDir(const std::filesystem::path &dir,
                                                                 const std::string &stem)
{
	if (!std::filesystem::exists(dir))
		return std::nullopt;

	const std::string stem_lower = ToLower(stem);
	for (const auto &entry : std::filesystem::directory_iterator(dir)) {
		if (!entry.is_regular_file())
			continue;
		const std::filesystem::path &candidate = entry.path();
		if (ToLower(candidate.stem().string()) == stem_lower && IsObjExtension(candidate))
			return candidate;
	}
	return std::nullopt;
}

std::filesystem::path ResolveAsciiStlMesh(const std::filesystem::path &stl_abs, const MeshPrepOptions &options)
{
	// Prefer collision OBJ when present for ASCII or oversized binary STL.
	const auto TryCollisionObj = [&]() -> std::optional<std::filesystem::path> {
		const std::filesystem::path collision_dir = stl_abs.parent_path().parent_path() / "collision";
		const std::string stem                    = stl_abs.stem().string();
		if (auto direct = FindCaseInsensitiveObjInDir(collision_dir, stem))
			return *direct;
		if (auto hull = FindCaseInsensitiveObjInDir(collision_dir, stem + "_convex_hull"))
			return *hull;
		return std::nullopt;
	};

	if (const auto face_count = BinaryStlTriangleCount(stl_abs)) {
		if (*face_count <= kMujocoMaxStlFaces)
			return stl_abs;

		// Prefer full-mesh OBJ rewrite when convert is on (visual fidelity). Collision
		// hull is only a fallback when convert is off.
		if (options.convert_ascii_stl)
			return ConvertBinaryStlToCachedObj(stl_abs);
		if (auto obj = TryCollisionObj())
			return *obj;

		throw std::runtime_error("Mesh URI prep: binary STL '" + stl_abs.string() + "' has " +
		                         std::to_string(*face_count) + " faces; MuJoCo STL limit is " +
		                         std::to_string(kMujocoMaxStlFaces) +
		                         ". Provide a collision OBJ fallback or set description.convert_ascii_stl:=true "
		                         "to rewrite as cached OBJ");
	}

	if (!IsAsciiStlFile(stl_abs))
		return stl_abs;

	if (options.convert_ascii_stl)
		return ConvertAsciiStlToCachedBinary(stl_abs);

	if (auto obj = TryCollisionObj())
		return *obj;

	throw std::runtime_error("Mesh URI prep: ASCII STL '" + stl_abs.string() +
	                         "' has no collision OBJ fallback; set description.convert_ascii_stl:=true "
	                         "to convert in-tree");
}

std::filesystem::path ResolveStlForGlb(const std::filesystem::path &glb_abs)
{
	if (auto sibling = FindCaseInsensitiveStlInDir(glb_abs.parent_path(), glb_abs.stem().string()))
		return *sibling;

	const std::filesystem::path collision_dir = glb_abs.parent_path().parent_path() / "collision";
	if (auto collision = FindCaseInsensitiveStlInDir(collision_dir, glb_abs.stem().string()))
		return *collision;

	throw std::runtime_error("Mesh URI prep: no STL found for visual GLB '" + glb_abs.string() + "'");
}

enum class MeshContext
{
	Visual,
	Collision,
	Unknown,
};

MeshContext GetMeshContext(const tinyxml2::XMLElement *mesh)
{
	const tinyxml2::XMLElement *geometry = mesh->Parent()->ToElement();
	if (geometry == nullptr || std::string(geometry->Name()) != "geometry")
		return MeshContext::Unknown;

	const tinyxml2::XMLElement *container = geometry->Parent()->ToElement();
	if (container == nullptr)
		return MeshContext::Unknown;

	const std::string name = container->Name();
	if (name == "visual")
		return MeshContext::Visual;
	if (name == "collision")
		return MeshContext::Collision;
	return MeshContext::Unknown;
}

struct PendingGlbBinding
{
	GlbVisualBinding binding;
	std::optional<std::filesystem::path> png_abs;
};

constexpr const char *kBasenameCacheDir = "/tmp/mujoco_ros_mesh_basename_cache";

// mjVFS keys by basename only. On conflict, copy the new mesh to a unique cached name.
std::filesystem::path EnsureUniqueMeshPath(std::map<std::string, std::string> &basename_to_dir,
                                           const std::filesystem::path &abs)
{
	const std::string base = abs.filename().string();
	const std::string dir  = abs.parent_path().string() + "/";
	auto it                = basename_to_dir.find(base);
	if (it == basename_to_dir.end() || it->second == dir)
		return abs;

	const auto abs_canon = std::filesystem::absolute(abs);
	std::ostringstream key;
	key << abs_canon.string() << '\0' << it->second;
	std::ostringstream hex;
	hex << std::hex << std::hash<std::string>{}(key.str());
	const std::string unique = abs.stem().string() + "_" + hex.str() + abs.extension().string();
	const auto dest          = std::filesystem::path(kBasenameCacheDir) / unique;
	std::filesystem::create_directories(dest.parent_path());
	std::filesystem::copy_file(abs, dest, std::filesystem::copy_options::overwrite_existing);
	return dest;
}

void RegisterBasename(std::map<std::string, std::string> &basename_to_dir, const std::string &base,
                      const std::string &dir)
{
	auto it = basename_to_dir.find(base);
	if (it != basename_to_dir.end() && it->second != dir)
		throw std::runtime_error("Mesh URI prep: basename '" + base + "' maps to both '" + it->second + "' and '" + dir +
		                         "'");
	basename_to_dir[base] = dir;
}

void ProcessMeshElements(tinyxml2::XMLElement *element, const std::optional<std::filesystem::path> &urdf_file_parent,
                         const MeshPrepOptions &options, std::map<std::string, std::string> &basename_to_dir,
                         std::vector<GlbVisualBinding> &glb_visual_bindings)
{
	for (tinyxml2::XMLElement *child = element->FirstChildElement(); child != nullptr;
	     child                       = child->NextSiblingElement()) {
		if (std::string(child->Name()) == "mesh") {
			const char *filename_attr = child->Attribute("filename");
			if (filename_attr == nullptr)
				throw std::runtime_error("Mesh URI prep: mesh element missing filename attribute");

			const std::string original_uri = filename_attr;
			std::filesystem::path abs      = NormalizeMeshUri(original_uri, urdf_file_parent);
			const MeshContext context      = GetMeshContext(child);

			if (context == MeshContext::Collision && IsGlbExtension(abs)) {
				throw std::runtime_error("Mesh URI prep: GLB collision mesh not supported '" + original_uri + "'");
			}

			std::optional<PendingGlbBinding> pending_binding;
			if (context == MeshContext::Visual && IsGlbExtension(abs)) {
				const std::filesystem::path glb_abs = abs;
				const auto extracted                = ExtractGlbVisual(glb_abs);
				const std::string material_name     = "glb_mat_" + glb_abs.stem().string();

				if (extracted.kind == GlbVisualKind::TexturedObj) {
					abs = extracted.obj_path;
					PendingGlbBinding pending;
					pending.binding.material_name = material_name;
					pending.png_abs               = extracted.png_path;
					pending_binding               = std::move(pending);
				} else if (extracted.kind == GlbVisualKind::UntexturedObj) {
					abs = extracted.obj_path;
					PendingGlbBinding pending;
					pending.binding.material_name = material_name;
					if (extracted.has_rgba) {
						pending.binding.has_rgba = true;
						pending.binding.rgba     = extracted.rgba;
					}
					pending_binding = std::move(pending);
				} else {
					abs = ResolveStlForGlb(glb_abs);
				}
			} else if (!IsSupportedMeshExt(abs)) {
				throw std::runtime_error("Mesh URI prep: unsupported mesh extension '" + abs.extension().string() +
				                         "' in '" + original_uri + "'");
			}

			if (IsStlExtension(abs))
				abs = ResolveAsciiStlMesh(abs, options);

			if (!std::filesystem::is_regular_file(abs)) {
				throw std::runtime_error("Mesh URI prep: mesh file not found for '" + original_uri + "' (resolved '" +
				                         abs.string() + "')");
			}

			abs = EnsureUniqueMeshPath(basename_to_dir, abs);

			const std::string base = abs.filename().string();
			const std::string dir  = abs.parent_path().string() + "/";
			RegisterBasename(basename_to_dir, base, dir);
			child->SetAttribute("filename", base.c_str());

			if (pending_binding) {
				pending_binding->binding.mesh_basename = base;
				if (pending_binding->png_abs) {
					const std::string png_base = pending_binding->png_abs->filename().string();
					const std::string png_dir  = pending_binding->png_abs->parent_path().string() + "/";
					RegisterBasename(basename_to_dir, png_base, png_dir);
					pending_binding->binding.texture_basename = png_base;
				}
				glb_visual_bindings.push_back(std::move(pending_binding->binding));
			}
		} else {
			ProcessMeshElements(child, urdf_file_parent, options, basename_to_dir, glb_visual_bindings);
		}
	}
}

} // namespace

std::string ResolvePackageShare(const std::string &package_name)
{
#if MJR_ROS_VERSION == ROS_1
	std::string share = ros::package::getPath(package_name);
	if (share.empty())
		throw std::runtime_error("Mesh URI prep: cannot resolve package '" + package_name + "'");
	return share;
#else
	try {
		return ament_index_cpp::get_package_share_directory(package_name);
	} catch (const std::exception &e) {
		throw std::runtime_error(std::string("Mesh URI prep: cannot resolve package '") + package_name +
		                         "': " + e.what());
	}
#endif
}

std::filesystem::path NormalizeMeshUri(const std::string &uri,
                                       const std::optional<std::filesystem::path> &urdf_file_parent)
{
	if (uri.empty())
		throw std::runtime_error("Mesh URI prep: empty mesh filename");

	static const std::string kFile = "file://";
	static const std::string kPkg  = "package://";
	std::string path               = uri;
	if (path.rfind(kFile, 0) == 0)
		path = path.substr(kFile.size());

	if (path.rfind(kPkg, 0) == 0) {
		std::string rest = path.substr(kPkg.size());
		auto slash       = rest.find('/');
		if (slash == std::string::npos)
			throw std::runtime_error("Mesh URI prep: malformed package:// URI '" + uri + "'");
		std::string pkg = rest.substr(0, slash);
		std::string rel = rest.substr(slash + 1);
		return std::filesystem::path(ResolvePackageShare(pkg)) / rel;
	}

	std::filesystem::path p(path);
	if (p.is_absolute())
		return p;
	if (!urdf_file_parent.has_value())
		throw std::runtime_error(
		    "Mesh URI prep: relative mesh '" + uri +
		    "' requires a file-backed URDF (param/string URDF must use absolute, file://, or package://)");
	return *urdf_file_parent / p;
}

MeshPrepResult PrepareUrdfMeshes(const std::string &urdf_text,
                                 const std::optional<std::filesystem::path> &urdf_file_parent,
                                 const MeshPrepOptions &options)
{
	tinyxml2::XMLDocument doc;
	const tinyxml2::XMLError rc = doc.Parse(urdf_text.c_str(), urdf_text.size());
	if (rc != tinyxml2::XML_SUCCESS)
		throw std::runtime_error("Mesh URI prep: failed to parse URDF: " + std::string(doc.ErrorStr()));

	tinyxml2::XMLElement *robot = doc.FirstChildElement("robot");
	if (robot == nullptr)
		throw std::runtime_error("Mesh URI prep: URDF missing root <robot> element");

	MeshPrepResult result;
	ProcessMeshElements(robot, urdf_file_parent, options, result.basename_to_dir, result.glb_visual_bindings);

	tinyxml2::XMLPrinter printer;
	doc.Print(&printer);

	result.working_urdf = printer.CStr();
	return result;
}

} // namespace mujoco_ros
