#pragma once

#include <mujoco_ros/glb_visual_prep.hpp>

#include <filesystem>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace mujoco_ros {

struct MeshPrepResult
{
	std::string working_urdf;
	std::map<std::string, std::string> basename_to_dir;
	std::set<std::string> rewritten_collision_mesh_basenames;
	std::vector<GlbVisualBinding> glb_visual_bindings;
};

struct MeshPrepOptions
{
	bool convert_ascii_stl = false;
};

std::string ResolvePackageShare(const std::string &package_name);

std::filesystem::path NormalizeMeshUri(const std::string &uri,
                                       const std::optional<std::filesystem::path> &urdf_file_parent);

MeshPrepResult PrepareUrdfMeshes(const std::string &urdf_text,
                                 const std::optional<std::filesystem::path> &urdf_file_parent,
                                 const MeshPrepOptions &options = {});

} // namespace mujoco_ros
