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

#include <mujoco_ros/ascii_stl_convert.hpp>

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace mujoco_ros {

namespace {

constexpr const char *kCacheDir = "/tmp/mujoco_ros_stl_cache";

struct Triangle
{
	std::array<float, 3> normal{};
	std::array<float, 9> verts{};
};

bool StartsWithSolid(const std::string &head)
{
	if (head.size() < 5)
		return false;
	return head.compare(0, 5, "solid") == 0;
}

std::optional<uint32_t> ReadBinaryStlTriangleCount(const std::filesystem::path &path)
{
	const auto size = std::filesystem::file_size(path);
	if (size < 84)
		return std::nullopt;

	std::ifstream in(path, std::ios::binary);
	if (!in)
		return std::nullopt;

	in.seekg(80);
	uint32_t triangle_count = 0;
	in.read(reinterpret_cast<char *>(&triangle_count), sizeof(triangle_count));
	if (!in)
		return std::nullopt;

	const std::uintmax_t expected = 84ULL + static_cast<std::uintmax_t>(triangle_count) * 50ULL;
	if (size != expected)
		return std::nullopt;
	return triangle_count;
}

bool IsValidBinaryStl(const std::filesystem::path &path)
{
	return ReadBinaryStlTriangleCount(path).has_value();
}

std::string HashFileContents(const std::filesystem::path &path)
{
	std::ifstream in(path, std::ios::binary);
	std::string bytes(std::filesystem::file_size(path), '\0');
	in.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
	return std::to_string(std::hash<std::string>{}(bytes));
}

// Content-addressed, not mtime-addressed: these mesh files are Git LFS
// tracked, and LFS's smudge filter rewrites mtime on every checkout even
// when content is byte-identical -- an mtime-keyed cache would silently
// miss (and reconvert) on every fresh checkout of unchanged assets.
std::string CacheFileName(const std::filesystem::path &path, const char *ext)
{
	const auto abs  = std::filesystem::absolute(path);
	const auto size = std::filesystem::file_size(path);

	std::ostringstream key;
	key << abs.string() << '\0' << size << '\0' << HashFileContents(path);
	const auto digest = std::hash<std::string>{}(key.str());

	std::ostringstream hex;
	hex << std::hex << digest;
	return hex.str() + ext;
}

std::vector<Triangle> ParseAsciiStl(const std::filesystem::path &path)
{
	std::ifstream in(path);
	if (!in)
		throw std::runtime_error("ASCII STL convert: cannot read '" + path.string() + "'");

	std::vector<Triangle> triangles;
	Triangle current{};
	int vertex_count = 0;
	std::string token;

	while (in >> token) {
		if (token == "facet") {
			std::string normal_kw;
			in >> normal_kw;
			if (normal_kw != "normal")
				throw std::runtime_error("ASCII STL convert: expected 'normal' after 'facet' in '" + path.string() + "'");
			in >> current.normal[0] >> current.normal[1] >> current.normal[2];
			vertex_count = 0;
		} else if (token == "vertex") {
			if (vertex_count >= 3)
				throw std::runtime_error("ASCII STL convert: too many vertices in facet in '" + path.string() + "'");
			in >> current.verts[vertex_count * 3] >> current.verts[vertex_count * 3 + 1] >>
			    current.verts[vertex_count * 3 + 2];
			++vertex_count;
		} else if (token == "endfacet") {
			if (vertex_count != 3)
				throw std::runtime_error("ASCII STL convert: facet with " + std::to_string(vertex_count) +
				                         " vertices in '" + path.string() + "'");
			triangles.push_back(current);
		}
	}

	if (triangles.empty())
		throw std::runtime_error("ASCII STL convert: no triangles parsed from '" + path.string() + "'");

	return triangles;
}

void WriteBinaryStl(const std::filesystem::path &out_path, const std::vector<Triangle> &triangles)
{
	std::filesystem::create_directories(out_path.parent_path());

	std::ofstream out(out_path, std::ios::binary | std::ios::trunc);
	if (!out)
		throw std::runtime_error("ASCII STL convert: cannot write cache file '" + out_path.string() + "'");

	char header[80]   = {};
	const char *label = "mujoco_ros ASCII STL cache";
	std::snprintf(header, sizeof(header), "%s", label);
	out.write(header, sizeof(header));

	const uint32_t count = static_cast<uint32_t>(triangles.size());
	out.write(reinterpret_cast<const char *>(&count), sizeof(count));

	for (const Triangle &tri : triangles) {
		out.write(reinterpret_cast<const char *>(tri.normal.data()), sizeof(tri.normal));
		out.write(reinterpret_cast<const char *>(tri.verts.data()), sizeof(tri.verts));
		const uint16_t attribute = 0;
		out.write(reinterpret_cast<const char *>(&attribute), sizeof(attribute));
	}

	if (!out)
		throw std::runtime_error("ASCII STL convert: failed writing cache file '" + out_path.string() + "'");
}

} // namespace

std::optional<uint32_t> BinaryStlTriangleCount(const std::filesystem::path &path)
{
	if (!std::filesystem::exists(path))
		throw std::runtime_error("STL detect: file does not exist '" + path.string() + "'");
	return ReadBinaryStlTriangleCount(path);
}

bool IsAsciiStlFile(const std::filesystem::path &path)
{
	if (!std::filesystem::exists(path))
		throw std::runtime_error("ASCII STL detect: file does not exist '" + path.string() + "'");

	if (IsValidBinaryStl(path))
		return false;

	std::ifstream in(path);
	if (!in)
		throw std::runtime_error("ASCII STL detect: cannot read '" + path.string() + "'");

	std::string head(5, '\0');
	in.read(head.data(), static_cast<std::streamsize>(head.size()));
	if (!in && !in.eof())
		throw std::runtime_error("ASCII STL detect: cannot read '" + path.string() + "'");

	return StartsWithSolid(head);
}

std::filesystem::path ConvertAsciiStlToCachedBinary(const std::filesystem::path &ascii_path)
{
	if (!std::filesystem::exists(ascii_path))
		throw std::runtime_error("ASCII STL convert: file does not exist '" + ascii_path.string() + "'");

	const std::filesystem::path cache_path = std::filesystem::path(kCacheDir) / CacheFileName(ascii_path, ".stl");
	if (std::filesystem::exists(cache_path))
		return cache_path;

	const std::vector<Triangle> triangles = ParseAsciiStl(ascii_path);
	WriteBinaryStl(cache_path, triangles);
	return cache_path;
}

std::filesystem::path ConvertBinaryStlToCachedObj(const std::filesystem::path &stl_path)
{
	if (!std::filesystem::exists(stl_path))
		throw std::runtime_error("STL→OBJ convert: file does not exist '" + stl_path.string() + "'");

	const auto count = ReadBinaryStlTriangleCount(stl_path);
	if (!count.has_value())
		throw std::runtime_error("STL→OBJ convert: not a valid binary STL '" + stl_path.string() + "'");

	const std::filesystem::path cache_path = std::filesystem::path(kCacheDir) / CacheFileName(stl_path, ".obj");
	if (std::filesystem::exists(cache_path))
		return cache_path;

	std::ifstream in(stl_path, std::ios::binary);
	if (!in)
		throw std::runtime_error("STL→OBJ convert: cannot read '" + stl_path.string() + "'");
	in.seekg(84);

	std::filesystem::create_directories(cache_path.parent_path());
	std::ofstream out(cache_path);
	if (!out)
		throw std::runtime_error("STL→OBJ convert: cannot write '" + cache_path.string() + "'");

	int vert_index = 1;
	for (uint32_t i = 0; i < *count; ++i) {
		float normal[3] = {};
		float verts[9]  = {};
		uint16_t attr   = 0;
		in.read(reinterpret_cast<char *>(normal), sizeof(normal));
		in.read(reinterpret_cast<char *>(verts), sizeof(verts));
		in.read(reinterpret_cast<char *>(&attr), sizeof(attr));
		if (!in)
			throw std::runtime_error("STL→OBJ convert: truncated STL '" + stl_path.string() + "'");
		(void)normal;
		(void)attr;
		for (int k = 0; k < 3; ++k)
			out << "v " << verts[3 * k] << ' ' << verts[3 * k + 1] << ' ' << verts[3 * k + 2] << '\n';
		out << "f " << vert_index << ' ' << (vert_index + 1) << ' ' << (vert_index + 2) << '\n';
		vert_index += 3;
	}

	if (!out)
		throw std::runtime_error("STL→OBJ convert: failed writing '" + cache_path.string() + "'");
	return cache_path;
}

} // namespace mujoco_ros
