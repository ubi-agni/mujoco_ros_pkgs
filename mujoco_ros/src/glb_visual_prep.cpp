#include <mujoco_ros/glb_visual_prep.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

namespace mujoco_ros {

namespace {

constexpr const char *kCacheDir = "/tmp/mujoco_ros_glb_cache";

std::string CacheDirName(const std::filesystem::path &path)
{
	const auto abs   = std::filesystem::absolute(path);
	const auto mtime = std::filesystem::last_write_time(path);
	const auto size  = std::filesystem::file_size(path);

	// Salt bumps invalidate stale OBJ caches when extract rules change.
	std::ostringstream key;
	key << "uv_vflip1_untex_all1" << '\0' << abs.string() << '\0' << mtime.time_since_epoch().count() << '\0' << size;
	const auto digest = std::hash<std::string>{}(key.str());

	std::ostringstream hex;
	hex << std::hex << digest;
	return hex.str();
}

const cgltf_accessor *FindAttribute(const cgltf_primitive &prim, cgltf_attribute_type type, int set_index = 0)
{
	for (cgltf_size i = 0; i < prim.attributes_count; ++i) {
		if (prim.attributes[i].type == type && prim.attributes[i].index == set_index)
			return prim.attributes[i].data;
	}
	return nullptr;
}

bool HasNonDefaultBaseColorFactor(const cgltf_material *mat)
{
	if (!mat || !mat->has_pbr_metallic_roughness)
		return false;
	const auto &f       = mat->pbr_metallic_roughness.base_color_factor;
	constexpr float eps = 1e-5f;
	return std::abs(f[0] - 1.f) > eps || std::abs(f[1] - 1.f) > eps || std::abs(f[2] - 1.f) > eps ||
	       std::abs(f[3] - 1.f) > eps;
}

const cgltf_image *GetBaseColorImage(const cgltf_material *mat)
{
	if (!mat || !mat->has_pbr_metallic_roughness)
		return nullptr;
	const auto &tv = mat->pbr_metallic_roughness.base_color_texture;
	if (!tv.texture || !tv.texture->image)
		return nullptr;
	return tv.texture->image;
}

bool IsPngImage(const cgltf_image *image)
{
	if (!image)
		return false;
	if (image->mime_type && std::string(image->mime_type) == "image/png")
		return true;
	if (image->uri) {
		const std::string uri = image->uri;
		if (uri.size() >= 4 && (uri.rfind(".png") != std::string::npos || uri.rfind(".PNG") != std::string::npos))
			return true;
	}
	if (image->buffer_view && image->buffer_view->buffer && image->buffer_view->buffer->data &&
	    image->buffer_view->size >= 8) {
		const auto *data  = static_cast<const uint8_t *>(image->buffer_view->buffer->data);
		const auto offset = image->buffer_view->offset;
		return data[offset] == 0x89 && data[offset + 1] == 'P' && data[offset + 2] == 'N' && data[offset + 3] == 'G';
	}
	return false;
}

std::vector<uint8_t> ReadImageBytes(const cgltf_image *image)
{
	if (!image || !image->buffer_view || !image->buffer_view->buffer || !image->buffer_view->buffer->data)
		return {};
	const auto *base  = static_cast<const uint8_t *>(image->buffer_view->buffer->data);
	const auto offset = image->buffer_view->offset;
	const auto size   = image->buffer_view->size;
	std::vector<uint8_t> out(size);
	std::memcpy(out.data(), base + offset, size);
	return out;
}

// Append one TRIANGLES primitive. vertex_count = # of `v` lines already written.
// Faces use 1-based OBJ indices. emit_uv writes vt with OpenGL V flip.
void AppendObjFromPrimitive(const cgltf_primitive &prim, std::ostream &out, int &vertex_count, bool emit_uv)
{
	if (prim.type != cgltf_primitive_type_triangles)
		throw std::runtime_error("GLB extract: only TRIANGLES primitives are supported");

	const auto *pos_acc  = FindAttribute(prim, cgltf_attribute_type_position);
	const auto *uv_acc   = emit_uv ? FindAttribute(prim, cgltf_attribute_type_texcoord, 0) : nullptr;
	const auto *norm_acc = FindAttribute(prim, cgltf_attribute_type_normal);
	if (!pos_acc)
		throw std::runtime_error("GLB extract: primitive has no POSITION attribute");
	if (emit_uv && !uv_acc)
		throw std::runtime_error("GLB extract: textured path requires TEXCOORD_0");

	cgltf_size tri_count = 0;
	if (prim.indices)
		tri_count = prim.indices->count / 3;
	else
		tri_count = pos_acc->count / 3;

	for (cgltf_size t = 0; t < tri_count; ++t) {
		// Emit corners as 0,1,2 then face them. glTF is CCW; MuJoCo/OpenGL expect
		// that winding for front faces. UV V is flipped: glTF image origin is top-left,
		// OBJ/OpenGL UV origin is bottom-left.
		for (cgltf_size corner = 0; corner < 3; ++corner) {
			const cgltf_size idx      = t * 3 + corner;
			const cgltf_size vert_idx = prim.indices ? cgltf_accessor_read_index(prim.indices, idx) : idx;

			cgltf_float pos[3] = {};
			if (!cgltf_accessor_read_float(pos_acc, vert_idx, pos, 3))
				throw std::runtime_error("GLB extract: failed reading position");
			out << "v " << pos[0] << ' ' << pos[1] << ' ' << pos[2] << '\n';

			if (norm_acc) {
				cgltf_float n[3] = {};
				cgltf_accessor_read_float(norm_acc, vert_idx, n, 3);
				out << "vn " << n[0] << ' ' << n[1] << ' ' << n[2] << '\n';
			}

			if (uv_acc) {
				cgltf_float uv[2] = {};
				cgltf_accessor_read_float(uv_acc, vert_idx, uv, 2);
				out << "vt " << uv[0] << ' ' << (1.f - uv[1]) << '\n';
			}
		}

		const int base = vertex_count + static_cast<int>(t * 3) + 1;
		if (uv_acc && norm_acc)
			out << "f " << base << '/' << base << '/' << base << ' ' << (base + 1) << '/' << (base + 1) << '/'
			    << (base + 1) << ' ' << (base + 2) << '/' << (base + 2) << '/' << (base + 2) << '\n';
		else if (uv_acc)
			out << "f " << base << '/' << base << ' ' << (base + 1) << '/' << (base + 1) << ' ' << (base + 2) << '/'
			    << (base + 2) << '\n';
		else if (norm_acc)
			out << "f " << base << "//" << base << ' ' << (base + 1) << "//" << (base + 1) << ' ' << (base + 2) << "//"
			    << (base + 2) << '\n';
		else
			out << "f " << base << ' ' << (base + 1) << ' ' << (base + 2) << '\n';
	}

	vertex_count += static_cast<int>(tri_count * 3);
}

void WriteObjFromPrimitive(const cgltf_primitive &prim, const std::filesystem::path &obj_path, bool emit_uv)
{
	std::filesystem::create_directories(obj_path.parent_path());
	std::ofstream out(obj_path);
	if (!out)
		throw std::runtime_error("GLB extract: cannot write OBJ '" + obj_path.string() + "'");
	int vertex_count = 0;
	AppendObjFromPrimitive(prim, out, vertex_count, emit_uv);
}

bool WriteUntexturedObjFromData(const cgltf_data *data, const std::filesystem::path &obj_path)
{
	std::filesystem::create_directories(obj_path.parent_path());
	std::ofstream out(obj_path);
	if (!out)
		throw std::runtime_error("GLB extract: cannot write OBJ '" + obj_path.string() + "'");

	int vertex_count  = 0;
	int prims_written = 0;
	for (cgltf_size mi = 0; mi < data->meshes_count; ++mi) {
		const cgltf_mesh &mesh = data->meshes[mi];
		for (cgltf_size pi = 0; pi < mesh.primitives_count; ++pi) {
			const cgltf_primitive &prim = mesh.primitives[pi];
			if (prim.type != cgltf_primitive_type_triangles) {
				std::cerr << "GLB extract: skipping non-TRIANGLES primitive\n";
				continue;
			}
			if (!FindAttribute(prim, cgltf_attribute_type_position))
				continue;
			AppendObjFromPrimitive(prim, out, vertex_count, /*emit_uv=*/false);
			++prims_written;
		}
	}
	return prims_written > 0 && vertex_count > 0;
}

bool TryFillRgbaFromMaterials(const cgltf_data *data, GlbVisualExtractResult &result)
{
	for (cgltf_size i = 0; i < data->materials_count; ++i) {
		const cgltf_material *mat = &data->materials[i];
		if (!HasNonDefaultBaseColorFactor(mat))
			continue;
		const auto &f   = mat->pbr_metallic_roughness.base_color_factor;
		result.has_rgba = true;
		result.rgba     = { { f[0], f[1], f[2], f[3] } };
		return true;
	}
	return false;
}

void WritePng(const std::filesystem::path &png_path, const std::vector<uint8_t> &bytes)
{
	std::filesystem::create_directories(png_path.parent_path());
	std::ofstream out(png_path, std::ios::binary | std::ios::trunc);
	if (!out)
		throw std::runtime_error("GLB extract: cannot write PNG '" + png_path.string() + "'");
	out.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

struct CgltfDataDeleter
{
	void operator()(cgltf_data *data) const
	{
		if (data)
			cgltf_free(data);
	}
};

std::filesystem::path CachedUniqueObjPath(const std::filesystem::path &glb_abs)
{
	const std::string cache_key   = CacheDirName(glb_abs);
	const auto cache_dir          = std::filesystem::path(kCacheDir) / cache_key;
	const std::string unique_stem = glb_abs.stem().string() + "_" + cache_key;
	return cache_dir / (unique_stem + ".obj");
}

} // namespace

GlbVisualExtractResult ExtractGlbVisual(const std::filesystem::path &glb_abs)
{
	GlbVisualExtractResult result;

	if (!std::filesystem::exists(glb_abs))
		return result;

	cgltf_options options{};
	cgltf_data *raw_data = nullptr;
	if (cgltf_parse_file(&options, glb_abs.string().c_str(), &raw_data) != cgltf_result_success || !raw_data) {
		std::cerr << "GLB extract: failed to parse '" << glb_abs.string() << "'\n";
		return result;
	}
	std::unique_ptr<cgltf_data, CgltfDataDeleter> data(raw_data);

	if (cgltf_load_buffers(&options, data.get(), glb_abs.string().c_str()) != cgltf_result_success) {
		std::cerr << "GLB extract: failed to load buffers for '" << glb_abs.string() << "'\n";
		return result;
	}

	if (data->meshes_count == 0)
		return result;

	const cgltf_mesh &mesh0 = data->meshes[0];
	if (mesh0.primitives_count == 0)
		return result;

	cgltf_size total_prims = 0;
	for (cgltf_size mi = 0; mi < data->meshes_count; ++mi)
		total_prims += data->meshes[mi].primitives_count;
	if (data->materials_count > 1 || data->meshes_count > 1 || total_prims > 1) {
		std::cerr << "GLB extract: multi-part GLB '" << glb_abs.string() << "' (materials=" << data->materials_count
		          << ", meshes=" << data->meshes_count << ", primitives=" << total_prims << ")\n";
	}

	const cgltf_primitive &prim0 = mesh0.primitives[0];
	const cgltf_accessor *uv_acc = FindAttribute(prim0, cgltf_attribute_type_texcoord, 0);

	cgltf_material *mat = prim0.material;
	if (!mat && data->materials_count > 0)
		mat = data->materials;

	const cgltf_image *image = mat ? GetBaseColorImage(mat) : nullptr;
	const bool has_texture   = image && image->buffer_view && IsPngImage(image);

	if (has_texture && uv_acc) {
		// VFS keys meshes by basename only — filenames must be unique across all
		// GLBs (maira has many Link*_body.glb that would otherwise all become mesh.obj).
		const std::string cache_key   = CacheDirName(glb_abs);
		const auto cache_dir          = std::filesystem::path(kCacheDir) / cache_key;
		const std::string unique_stem = glb_abs.stem().string() + "_" + cache_key;
		const auto obj_path           = cache_dir / (unique_stem + ".obj");
		const auto png_path           = cache_dir / (unique_stem + ".png");

		if (std::filesystem::exists(obj_path) && std::filesystem::exists(png_path)) {
			result.kind     = GlbVisualKind::TexturedObj;
			result.obj_path = obj_path;
			result.png_path = png_path;
			return result;
		}

		const auto png_bytes = ReadImageBytes(image);
		if (png_bytes.empty()) {
			// Fall through to untextured geometry.
		} else {
			WritePng(png_path, png_bytes);
			WriteObjFromPrimitive(prim0, obj_path, /*emit_uv=*/true);

			result.kind     = GlbVisualKind::TexturedObj;
			result.obj_path = obj_path;
			result.png_path = png_path;
			return result;
		}
	}

	// No usable baseColor texture: merge all triangle prims into one untextured OBJ.
	const auto obj_path = CachedUniqueObjPath(glb_abs);
	if (std::filesystem::exists(obj_path)) {
		result.kind     = GlbVisualKind::UntexturedObj;
		result.obj_path = obj_path;
		TryFillRgbaFromMaterials(data.get(), result);
		return result;
	}

	if (!WriteUntexturedObjFromData(data.get(), obj_path))
		return result;

	result.kind     = GlbVisualKind::UntexturedObj;
	result.obj_path = obj_path;
	TryFillRgbaFromMaterials(data.get(), result);
	return result;
}

} // namespace mujoco_ros
