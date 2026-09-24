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

#include <gtest/gtest.h>
#include <mujoco_ros/ascii_stl_convert.hpp>
#include <mujoco_ros/mesh_uri_prep.hpp>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

void WriteBinaryStlWithFaceCount(const fs::path &path, uint32_t n)
{
	std::ofstream out(path, std::ios::binary | std::ios::trunc);
	ASSERT_TRUE(out) << path;
	char header[80] = {};
	out.write(header, sizeof(header));
	out.write(reinterpret_cast<const char *>(&n), sizeof(n));
	std::vector<char> tri(50, 0);
	// Non-degenerate triangle so OBJ convert emits real verts.
	const float verts[9] = { 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f, 0.f };
	std::memcpy(tri.data() + 12, verts, sizeof(verts));
	for (uint32_t i = 0; i < n; ++i)
		out.write(tri.data(), static_cast<std::streamsize>(tri.size()));
	ASSERT_TRUE(out) << path;
}

} // namespace

TEST(AsciiStlConvert, DetectsAscii)
{
	auto dir = fs::temp_directory_path() / "ascii_stl_det";
	fs::create_directories(dir);
	auto p = dir / "a.stl";
	std::ofstream(p) << "solid mesh\n  facet normal 0 0 1\n    outer loop\n"
	                    "      vertex 0 0 0\n      vertex 1 0 0\n      vertex 0 1 0\n"
	                    "    endloop\n  endfacet\nendsolid mesh\n";
	EXPECT_TRUE(mujoco_ros::IsAsciiStlFile(p));
}

TEST(AsciiStlConvert, ConvertProducesBinaryReusableCache)
{
	auto dir = fs::temp_directory_path() / "ascii_stl_conv";
	fs::create_directories(dir);
	auto ascii = dir / "a.stl";
	std::ofstream(ascii) << "solid mesh\n  facet normal 0 0 1\n    outer loop\n"
	                        "      vertex 0 0 0\n      vertex 1 0 0\n      vertex 0 1 0\n"
	                        "    endloop\n  endfacet\nendsolid mesh\n";
	auto bin1 = mujoco_ros::ConvertAsciiStlToCachedBinary(ascii);
	ASSERT_TRUE(fs::exists(bin1));
	EXPECT_FALSE(mujoco_ros::IsAsciiStlFile(bin1));
	auto bin2 = mujoco_ros::ConvertAsciiStlToCachedBinary(ascii);
	EXPECT_EQ(bin1, bin2);
	std::ifstream in(ascii);
	std::string head(5, '\0');
	in.read(&head[0], 5);
	EXPECT_EQ(head, "solid");
}

TEST(AsciiStlConvert, BinaryTriangleCountAndObjConvert)
{
	auto dir = fs::temp_directory_path() / "bin_stl_count";
	fs::create_directories(dir);
	auto stl = dir / "b.stl";
	WriteBinaryStlWithFaceCount(stl, 3);
	auto count = mujoco_ros::BinaryStlTriangleCount(stl);
	ASSERT_TRUE(count.has_value());
	EXPECT_EQ(*count, 3u);
	EXPECT_FALSE(mujoco_ros::IsAsciiStlFile(stl));

	auto obj = mujoco_ros::ConvertBinaryStlToCachedObj(stl);
	ASSERT_TRUE(fs::exists(obj));
	EXPECT_EQ(obj.extension(), ".obj");
	std::ifstream in(obj);
	std::string line;
	bool saw_v = false;
	bool saw_f = false;
	while (std::getline(in, line)) {
		if (line.rfind("v ", 0) == 0)
			saw_v = true;
		if (line.rfind("f ", 0) == 0)
			saw_f = true;
	}
	EXPECT_TRUE(saw_v);
	EXPECT_TRUE(saw_f);
	EXPECT_EQ(mujoco_ros::ConvertBinaryStlToCachedObj(stl), obj);
}

TEST(AsciiStlConvert, CacheSurvivesMtimeChangeWithSameContent)
{
	// Git LFS's smudge filter rewrites mtime on every checkout even when
	// content is byte-identical; the cache key must not churn from that alone.
	auto dir = fs::temp_directory_path() / "bin_stl_mtime_churn";
	fs::create_directories(dir);
	auto stl = dir / "c.stl";
	WriteBinaryStlWithFaceCount(stl, 3);

	auto obj1 = mujoco_ros::ConvertBinaryStlToCachedObj(stl);
	ASSERT_TRUE(fs::exists(obj1));

	fs::last_write_time(stl, fs::last_write_time(stl) + std::chrono::hours(1));

	auto obj2 = mujoco_ros::ConvertBinaryStlToCachedObj(stl);
	EXPECT_EQ(obj2, obj1);
}

TEST(MeshUriPrep, OversizedBinaryStlConvertsToObjWhenEnabled)
{
	auto root = fs::temp_directory_path() / "mesh_uri_oversize_stl";
	fs::create_directories(root / "meshes" / "visual");
	const auto stl = root / "meshes" / "visual" / "big.STL";
	WriteBinaryStlWithFaceCount(stl, mujoco_ros::kMujocoMaxStlFaces + 1);
	std::string urdf =
	    R"(<robot name="r"><link name="l"><visual><geometry><mesh filename="meshes/visual/big.STL"/></geometry></visual></link></robot>)";

	EXPECT_THROW(mujoco_ros::PrepareUrdfMeshes(urdf, root), std::runtime_error);

	mujoco_ros::MeshPrepOptions opts;
	opts.convert_ascii_stl = true;
	auto prep              = mujoco_ros::PrepareUrdfMeshes(urdf, root, opts);
	EXPECT_EQ(prep.working_urdf.find(".STL"), std::string::npos);
	EXPECT_NE(prep.working_urdf.find(".obj"), std::string::npos);
	ASSERT_EQ(prep.basename_to_dir.size(), 1u);
	const auto &base = prep.basename_to_dir.begin()->first;
	EXPECT_NE(base.find(".obj"), std::string::npos);
	EXPECT_TRUE(fs::exists(prep.basename_to_dir.at(base) + base));
}
