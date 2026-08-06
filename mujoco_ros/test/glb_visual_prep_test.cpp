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
