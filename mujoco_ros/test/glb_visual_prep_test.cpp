/*********************************************************************
 * Software License Agreement (BSD License)
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

#include <gtest/gtest.h>
#include <fstream>
#include <string>

#include <mujoco_ros/glb_visual_prep.hpp>

namespace fs = std::filesystem;

TEST(GlbVisualPrep, TexturedGlbWritesObjAndPng)
{
	auto glb = fs::path(TEST_RESOURCES_DIR) / "glb_textured" / "textured_tri.glb";
	auto r   = mujoco_ros::ExtractGlbVisual(glb);
	ASSERT_EQ(r.kind, mujoco_ros::GlbVisualKind::TexturedObj);
	EXPECT_TRUE(fs::exists(r.obj_path));
	EXPECT_TRUE(fs::exists(r.png_path));
	EXPECT_EQ(r.obj_path.extension(), ".obj");

	// Fixture UVs include (0,0); OBJ must use OpenGL V (1-v) → "vt 0 1".
	std::ifstream in(r.obj_path);
	ASSERT_TRUE(in) << r.obj_path;
	bool saw_flipped = false;
	std::string line;
	while (std::getline(in, line)) {
		if (line == "vt 0 1" || line.rfind("vt 0 1 ", 0) == 0) {
			saw_flipped = true;
			break;
		}
	}
	EXPECT_TRUE(saw_flipped);
}

TEST(GlbVisualPrep, FactorOnlyWritesUntexturedObjAndRgba)
{
	auto glb = fs::path(TEST_RESOURCES_DIR) / "glb_textured" / "factor_only.glb";
	auto r   = mujoco_ros::ExtractGlbVisual(glb);
	ASSERT_EQ(r.kind, mujoco_ros::GlbVisualKind::UntexturedObj);
	EXPECT_TRUE(fs::exists(r.obj_path));
	EXPECT_EQ(r.obj_path.extension(), ".obj");
	EXPECT_TRUE(r.has_rgba);
	EXPECT_NEAR(r.rgba[0], 0.2f, 1e-3);
	EXPECT_NEAR(r.rgba[1], 0.4f, 1e-3);
	EXPECT_NEAR(r.rgba[2], 0.6f, 1e-3);
}

TEST(GlbVisualPrep, NoUvWritesUntexturedObj)
{
	auto glb = fs::path(TEST_RESOURCES_DIR) / "glb_textured" / "no_uv.glb";
	auto r   = mujoco_ros::ExtractGlbVisual(glb);
	ASSERT_EQ(r.kind, mujoco_ros::GlbVisualKind::UntexturedObj);
	EXPECT_TRUE(fs::exists(r.obj_path));
	EXPECT_EQ(r.obj_path.extension(), ".obj");
}
