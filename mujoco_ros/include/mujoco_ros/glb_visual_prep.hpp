/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  Copyright (c) 2026, Neura Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor Neura Robotics nor
 *     the names of their contributors may be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

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
