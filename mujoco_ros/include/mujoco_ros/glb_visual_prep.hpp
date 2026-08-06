#pragma once
#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace mujoco_ros {

struct GlbVisualBinding
{
	std::string mesh_basename; // OBJ basename after rewrite (empty if STL path)
	std::string texture_basename; // PNG basename in VFS; empty if factor-only / none
	std::string material_name; // unique material name for post-parse
	bool has_rgba = false;
	std::array<float, 4> rgba{ { 1.f, 1.f, 1.f, 1.f } };
};

enum class GlbVisualKind
{
	TexturedObj,
	UntexturedObj,
	StlOnly
};

struct GlbVisualExtractResult
{
	GlbVisualKind kind = GlbVisualKind::StlOnly;
	std::filesystem::path obj_path;
	std::filesystem::path png_path;
	std::filesystem::path stl_path; // unused (legacy field)
	bool has_rgba = false;
	std::array<float, 4> rgba{ { 1.f, 1.f, 1.f, 1.f } };
};

// Attempt textured extraction; on soft failure return StlOnly
// with empty paths (caller runs ResolveStlForGlb).
GlbVisualExtractResult ExtractGlbVisual(const std::filesystem::path &glb_abs);

} // namespace mujoco_ros
