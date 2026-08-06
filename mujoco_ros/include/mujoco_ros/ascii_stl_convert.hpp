#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>

namespace mujoco_ros {

// MuJoCo 3.3.5 rejects STL meshes with more than this many faces (misleading
// "perhaps ASCII" error). OBJ has no such cap.
constexpr uint32_t kMujocoMaxStlFaces = 200000;

bool IsAsciiStlFile(const std::filesystem::path &path);

// Returns triangle count when file is a size-valid binary STL; else nullopt.
std::optional<uint32_t> BinaryStlTriangleCount(const std::filesystem::path &path);

std::filesystem::path ConvertAsciiStlToCachedBinary(const std::filesystem::path &ascii_path);

// Rewrite a binary STL to a cached OBJ (used when face count exceeds MuJoCo's STL limit).
std::filesystem::path ConvertBinaryStlToCachedObj(const std::filesystem::path &stl_path);

} // namespace mujoco_ros
