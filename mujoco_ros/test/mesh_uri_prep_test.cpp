#include <gtest/gtest.h>
#include <cstdint>
#include <fstream>
#include <mujoco_ros/ascii_stl_convert.hpp>
#include <mujoco_ros/mesh_uri_prep.hpp>

namespace fs = std::filesystem;

namespace {

const char *kAsciiStl = "solid mesh\n  facet normal 0 0 1\n    outer loop\n"
                        "      vertex 0 0 0\n      vertex 1 0 0\n      vertex 0 1 0\n"
                        "    endloop\n  endfacet\nendsolid mesh\n";

const char *kMinimalObj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";

void WriteMinimalBinaryStl(const fs::path &path)
{
	std::ofstream out(path, std::ios::binary | std::ios::trunc);
	ASSERT_TRUE(out) << path;
	char header[80] = {};
	out.write(header, sizeof(header));
	const uint32_t count = 1;
	out.write(reinterpret_cast<const char *>(&count), sizeof(count));
	const float normal[3] = { 0.f, 0.f, 1.f };
	const float verts[9]  = { 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f, 0.f };
	out.write(reinterpret_cast<const char *>(normal), sizeof(normal));
	out.write(reinterpret_cast<const char *>(verts), sizeof(verts));
	const uint16_t attribute = 0;
	out.write(reinterpret_cast<const char *>(&attribute), sizeof(attribute));
	ASSERT_TRUE(out) << path;
}

} // namespace

TEST(MeshUriPrep, AbsolutePathUnchanged)
{
	auto p = mujoco_ros::NormalizeMeshUri("/tmp/foo.obj", std::nullopt);
	EXPECT_EQ(p, fs::path("/tmp/foo.obj"));
}

TEST(MeshUriPrep, StripsFileScheme)
{
	auto p = mujoco_ros::NormalizeMeshUri("file:///tmp/foo.obj", std::nullopt);
	EXPECT_EQ(p, fs::path("/tmp/foo.obj"));
}

TEST(MeshUriPrep, RelativeRequiresParent)
{
	EXPECT_THROW(mujoco_ros::NormalizeMeshUri("meshes/a.obj", std::nullopt), std::runtime_error);
}

TEST(MeshUriPrep, RelativeJoinsParent)
{
	auto p = mujoco_ros::NormalizeMeshUri("meshes/a.obj", fs::path("/robot/dir"));
	EXPECT_EQ(p, fs::path("/robot/dir/meshes/a.obj"));
}

TEST(MeshUriPrep, PackageShareResolvesMujocoRos)
{
	const std::string share = mujoco_ros::ResolvePackageShare("mujoco_ros");
	ASSERT_FALSE(share.empty());
	EXPECT_TRUE(fs::exists(fs::path(share) / "assets" / "default_world.xml"))
	    << "expected installed/share assets; build+install overlay first";
	auto p = mujoco_ros::NormalizeMeshUri("package://mujoco_ros/assets/default_world.xml", std::nullopt);
	EXPECT_TRUE(fs::exists(p));
}

TEST(MeshUriPrep, UnknownSchemeThrows)
{
	EXPECT_THROW(mujoco_ros::NormalizeMeshUri("http://x/a.obj", std::nullopt), std::runtime_error);
}

TEST(MeshUriPrep, GlbRewritesToSiblingStl)
{
	auto root = fs::temp_directory_path() / "mesh_uri_glb_sib";
	fs::create_directories(root / "meshes" / "visual");
	std::ofstream(root / "meshes" / "visual" / "Link0_body.glb") << "glb";
	WriteMinimalBinaryStl(root / "meshes" / "visual" / "Link0_body.STL");
	std::string urdf =
	    R"(<robot name="r"><link name="l"><visual><geometry><mesh filename="meshes/visual/Link0_body.glb"/></geometry></visual></link></robot>)";
	auto prep = mujoco_ros::PrepareUrdfMeshes(urdf, root);
	EXPECT_EQ(prep.working_urdf.find(".glb"), std::string::npos);
	EXPECT_NE(prep.working_urdf.find("filename=\"Link0_body.STL\""), std::string::npos);
	EXPECT_EQ(prep.working_urdf.find("meshes/visual"), std::string::npos);
	ASSERT_EQ(prep.basename_to_dir.count("Link0_body.STL"), 1u);
	EXPECT_EQ(prep.basename_to_dir.at("Link0_body.STL"), (root / "meshes" / "visual").string() + "/");
}

TEST(MeshUriPrep, BasenameMapForObj)
{
	auto root = fs::temp_directory_path() / "mesh_uri_base";
	fs::create_directories(root / "meshes" / "collision");
	std::ofstream(root / "meshes" / "collision" / "hull.obj") << "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
	std::string urdf =
	    R"(<robot name="r"><link name="l"><collision><geometry><mesh filename="meshes/collision/hull.obj"/></geometry></collision></link></robot>)";
	auto prep = mujoco_ros::PrepareUrdfMeshes(urdf, root);
	EXPECT_NE(prep.working_urdf.find("filename=\"hull.obj\""), std::string::npos);
	EXPECT_EQ(prep.working_urdf.find("meshes/collision"), std::string::npos);
	ASSERT_EQ(prep.basename_to_dir.count("hull.obj"), 1u);
	EXPECT_EQ(prep.basename_to_dir.at("hull.obj"), (root / "meshes" / "collision").string() + "/");
}

TEST(MeshUriPrep, DuplicateBasenameDifferentDirsUniquifies)
{
	auto root = fs::temp_directory_path() / "mesh_uri_dup";
	fs::create_directories(root / "a");
	fs::create_directories(root / "b");
	std::ofstream(root / "a" / "same.obj") << "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
	std::ofstream(root / "b" / "same.obj") << "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
	std::string urdf =
	    R"(<robot name="r"><link name="l">
         <visual><geometry><mesh filename="a/same.obj"/></geometry></visual>
         <collision><geometry><mesh filename="b/same.obj"/></geometry></collision>
       </link></robot>)";
	auto prep = mujoco_ros::PrepareUrdfMeshes(urdf, root);
	EXPECT_NE(prep.working_urdf.find("filename=\"same.obj\""), std::string::npos);
	// Second mesh gets a unique basename so mjVFS can load both.
	EXPECT_EQ(prep.basename_to_dir.size(), 2u);
	std::string second;
	for (const auto &[base, dir] : prep.basename_to_dir) {
		if (base != "same.obj")
			second = base;
	}
	ASSERT_FALSE(second.empty());
	EXPECT_NE(prep.working_urdf.find("filename=\"" + second + "\""), std::string::npos);
	EXPECT_TRUE(fs::exists(prep.basename_to_dir.at(second) + second));
}

TEST(MeshUriPrep, GlbMissingGeometryAndStlThrows)
{
	auto root = fs::temp_directory_path() / "mesh_uri_glb_miss";
	fs::create_directories(root / "meshes" / "visual");
	std::ofstream(root / "meshes" / "visual" / "only.glb") << "glb";
	std::string urdf =
	    R"(<robot name="r"><link name="l"><visual><geometry><mesh filename="meshes/visual/only.glb"/></geometry></visual></link></robot>)";
	EXPECT_THROW(mujoco_ros::PrepareUrdfMeshes(urdf, root), std::runtime_error);
}

TEST(MeshUriPrep, CollisionGlbThrows)
{
	auto root = fs::temp_directory_path() / "mesh_uri_col_glb";
	fs::create_directories(root / "meshes" / "collision");
	std::ofstream(root / "meshes" / "collision" / "c.glb") << "glb";
	std::string urdf =
	    R"(<robot name="r"><link name="l"><collision><geometry><mesh filename="meshes/collision/c.glb"/></geometry></collision></link></robot>)";
	EXPECT_THROW(mujoco_ros::PrepareUrdfMeshes(urdf, root), std::runtime_error);
}

TEST(MeshUriPrep, AsciiStlFallsBackToConvexHullObj)
{
	auto root = fs::temp_directory_path() / "mesh_uri_ascii_hull";
	fs::create_directories(root / "meshes" / "visual");
	fs::create_directories(root / "meshes" / "collision");
	std::ofstream(root / "meshes" / "visual" / "Link0_body.stl") << kAsciiStl;
	std::ofstream(root / "meshes" / "collision" / "Link0_body_convex_hull.OBJ") << kMinimalObj;
	std::string urdf =
	    R"(<robot name="r"><link name="l"><visual><geometry><mesh filename="meshes/visual/Link0_body.stl"/></geometry></visual></link></robot>)";
	auto prep = mujoco_ros::PrepareUrdfMeshes(urdf, root);
	EXPECT_NE(prep.working_urdf.find("filename=\"Link0_body_convex_hull.OBJ\""), std::string::npos);
	ASSERT_EQ(prep.basename_to_dir.count("Link0_body_convex_hull.OBJ"), 1u);
	EXPECT_EQ(prep.basename_to_dir.at("Link0_body_convex_hull.OBJ"), (root / "meshes" / "collision").string() + "/");
}

TEST(MeshUriPrep, AsciiStlPrefersDirectCollisionObj)
{
	auto root = fs::temp_directory_path() / "mesh_uri_ascii_direct";
	fs::create_directories(root / "meshes" / "visual");
	fs::create_directories(root / "meshes" / "collision");
	std::ofstream(root / "meshes" / "visual" / "part.stl") << kAsciiStl;
	std::ofstream(root / "meshes" / "collision" / "part.obj") << kMinimalObj;
	std::ofstream(root / "meshes" / "collision" / "part_convex_hull.obj") << kMinimalObj;
	std::string urdf =
	    R"(<robot name="r"><link name="l"><visual><geometry><mesh filename="meshes/visual/part.stl"/></geometry></visual></link></robot>)";
	auto prep = mujoco_ros::PrepareUrdfMeshes(urdf, root);
	EXPECT_NE(prep.working_urdf.find("filename=\"part.obj\""), std::string::npos);
	EXPECT_EQ(prep.working_urdf.find("convex_hull"), std::string::npos);
}

TEST(MeshUriPrep, AsciiStlConvertOptionUsesCachedBinary)
{
	auto root = fs::temp_directory_path() / "mesh_uri_ascii_conv";
	fs::create_directories(root / "meshes" / "collision");
	auto ascii = root / "meshes" / "collision" / "part.stl";
	std::ofstream(ascii) << kAsciiStl;
	std::string urdf =
	    R"(<robot name="r"><link name="l"><collision><geometry><mesh filename="meshes/collision/part.stl"/></geometry></collision></link></robot>)";
	mujoco_ros::MeshPrepOptions opts;
	opts.convert_ascii_stl = true;
	auto prep              = mujoco_ros::PrepareUrdfMeshes(urdf, root, opts);
	const auto cached      = mujoco_ros::ConvertAsciiStlToCachedBinary(ascii);
	ASSERT_EQ(prep.basename_to_dir.count(cached.filename().string()), 1u);
	EXPECT_EQ(prep.basename_to_dir.at(cached.filename().string()), cached.parent_path().string() + "/");
	EXPECT_FALSE(mujoco_ros::IsAsciiStlFile(cached));
}

TEST(MeshUriPrep, AsciiStlNoFallbackThrowsConvertHint)
{
	auto root = fs::temp_directory_path() / "mesh_uri_ascii_throw";
	fs::create_directories(root / "meshes" / "visual");
	std::ofstream(root / "meshes" / "visual" / "only.stl") << kAsciiStl;
	std::string urdf =
	    R"(<robot name="r"><link name="l"><visual><geometry><mesh filename="meshes/visual/only.stl"/></geometry></visual></link></robot>)";
	try {
		mujoco_ros::PrepareUrdfMeshes(urdf, root);
		FAIL() << "expected throw";
	} catch (const std::runtime_error &e) {
		EXPECT_NE(std::string(e.what()).find("description.convert_ascii_stl:=true"), std::string::npos);
	}
}

TEST(MeshUriPrep, TexturedGlbRewritesToCachedObj)
{
	const fs::path fixture = fs::path(TEST_RESOURCES_DIR) / "glb_textured" / "textured_tri.glb";
	ASSERT_TRUE(fs::exists(fixture)) << "missing glb_textured fixture; run gen_fixtures.py";
	auto root = fs::temp_directory_path() / "mesh_uri_glb_textured";
	fs::create_directories(root / "meshes" / "visual");
	fs::copy_file(fixture, root / "meshes" / "visual" / "textured_tri.glb", fs::copy_options::overwrite_existing);
	std::string urdf =
	    R"(<robot name="r"><link name="l"><visual><geometry><mesh filename="meshes/visual/textured_tri.glb"/></geometry></visual></link></robot>)";
	auto prep = mujoco_ros::PrepareUrdfMeshes(urdf, root);
	EXPECT_EQ(prep.working_urdf.find(".glb"), std::string::npos);
	EXPECT_EQ(prep.working_urdf.find("meshes/visual"), std::string::npos);
	ASSERT_FALSE(prep.glb_visual_bindings.empty());
	const auto &binding = prep.glb_visual_bindings.front();
	EXPECT_NE(binding.mesh_basename.find("textured_tri_"), std::string::npos);
	EXPECT_NE(binding.mesh_basename.find(".obj"), std::string::npos);
	EXPECT_NE(binding.texture_basename.find("textured_tri_"), std::string::npos);
	EXPECT_NE(binding.texture_basename.find(".png"), std::string::npos);
	EXPECT_NE(prep.working_urdf.find("filename=\"" + binding.mesh_basename + "\""), std::string::npos);
	ASSERT_EQ(prep.basename_to_dir.count(binding.mesh_basename), 1u);
	ASSERT_EQ(prep.basename_to_dir.count(binding.texture_basename), 1u);
	EXPECT_FALSE(binding.material_name.empty());
	EXPECT_TRUE(fs::exists(prep.basename_to_dir.at(binding.mesh_basename) + binding.mesh_basename));
	EXPECT_TRUE(fs::exists(prep.basename_to_dir.at(binding.texture_basename) + binding.texture_basename));
}

TEST(MeshUriPrep, MultipleTexturedGlbsGetUniqueBasenames)
{
	const fs::path fixture = fs::path(TEST_RESOURCES_DIR) / "glb_textured" / "textured_tri.glb";
	ASSERT_TRUE(fs::exists(fixture)) << "missing glb_textured fixture; run gen_fixtures.py";
	auto root = fs::temp_directory_path() / "mesh_uri_glb_multi";
	fs::create_directories(root / "meshes" / "visual");
	fs::copy_file(fixture, root / "meshes" / "visual" / "Link0_body.glb", fs::copy_options::overwrite_existing);
	fs::copy_file(fixture, root / "meshes" / "visual" / "Link1_body.glb", fs::copy_options::overwrite_existing);
	std::string urdf =
	    R"(<robot name="r"><link name="l0"><visual><geometry><mesh filename="meshes/visual/Link0_body.glb"/></geometry></visual></link>)"
	    R"(<link name="l1"><visual><geometry><mesh filename="meshes/visual/Link1_body.glb"/></geometry></visual></link></robot>)";
	auto prep = mujoco_ros::PrepareUrdfMeshes(urdf, root);
	ASSERT_EQ(prep.glb_visual_bindings.size(), 2u);
	EXPECT_NE(prep.glb_visual_bindings[0].mesh_basename, prep.glb_visual_bindings[1].mesh_basename);
	EXPECT_NE(prep.glb_visual_bindings[0].texture_basename, prep.glb_visual_bindings[1].texture_basename);
	EXPECT_EQ(prep.basename_to_dir.count(prep.glb_visual_bindings[0].mesh_basename), 1u);
	EXPECT_EQ(prep.basename_to_dir.count(prep.glb_visual_bindings[1].mesh_basename), 1u);
}

TEST(MeshUriPrep, NoUvGlbWritesUntexturedObjWithoutSiblingStl)
{
	const fs::path glb_fixture = fs::path(TEST_RESOURCES_DIR) / "glb_textured" / "no_uv.glb";
	ASSERT_TRUE(fs::exists(glb_fixture)) << "missing glb_textured fixture; run gen_fixtures.py";
	auto root = fs::temp_directory_path() / "mesh_uri_glb_no_uv";
	fs::create_directories(root / "meshes" / "visual");
	fs::copy_file(glb_fixture, root / "meshes" / "visual" / "no_uv.glb", fs::copy_options::overwrite_existing);
	std::string urdf =
	    R"(<robot name="r"><link name="l"><visual><geometry><mesh filename="meshes/visual/no_uv.glb"/></geometry></visual></link></robot>)";
	auto prep = mujoco_ros::PrepareUrdfMeshes(urdf, root);
	EXPECT_EQ(prep.working_urdf.find(".glb"), std::string::npos);
	EXPECT_EQ(prep.working_urdf.find(".STL"), std::string::npos);
	ASSERT_FALSE(prep.glb_visual_bindings.empty());
	const auto &binding = prep.glb_visual_bindings.front();
	EXPECT_NE(binding.mesh_basename.find(".obj"), std::string::npos);
	ASSERT_EQ(prep.basename_to_dir.count(binding.mesh_basename), 1u);
	EXPECT_TRUE(fs::exists(prep.basename_to_dir.at(binding.mesh_basename) + binding.mesh_basename));
}

TEST(MeshUriPrep, FactorOnlyGlbWithoutSiblingStlSucceeds)
{
	const fs::path glb_fixture = fs::path(TEST_RESOURCES_DIR) / "glb_textured" / "factor_only.glb";
	ASSERT_TRUE(fs::exists(glb_fixture)) << "missing glb_textured fixture; run gen_fixtures.py";
	auto root = fs::temp_directory_path() / "mesh_uri_glb_factor_nosib";
	fs::create_directories(root / "meshes" / "visual");
	fs::copy_file(glb_fixture, root / "meshes" / "visual" / "factor_only.glb", fs::copy_options::overwrite_existing);
	std::string urdf =
	    R"(<robot name="r"><link name="l"><visual><geometry><mesh filename="meshes/visual/factor_only.glb"/></geometry></visual></link></robot>)";
	auto prep = mujoco_ros::PrepareUrdfMeshes(urdf, root);
	ASSERT_FALSE(prep.glb_visual_bindings.empty());
	EXPECT_TRUE(prep.glb_visual_bindings.front().has_rgba);
	EXPECT_NE(prep.glb_visual_bindings.front().mesh_basename.find(".obj"), std::string::npos);
}
