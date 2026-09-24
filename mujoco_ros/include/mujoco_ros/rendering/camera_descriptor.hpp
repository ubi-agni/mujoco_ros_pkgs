#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <mujoco/mujoco.h>

namespace mujoco_ros::rendering {

inline mjvOption DefaultCameraVisualOptions()
{
	mjvOption options;
	mjv_defaultOption(&options);
	return options;
}

using CameraId = std::uint32_t;

enum class PlaneKind : std::uint8_t
{
	kRgb = 0,
	kDepth,
	kSegmentation,
};

enum class PlaneMask : std::uint8_t
{
	kNone         = 0,
	kRgb          = 1U << 0U,
	kDepth        = 1U << 1U,
	kSegmentation = 1U << 2U,
	kAll          = 7,
};

constexpr PlaneMask operator|(PlaneMask lhs, PlaneMask rhs)
{
	return static_cast<PlaneMask>(static_cast<std::uint8_t>(lhs) | static_cast<std::uint8_t>(rhs));
}

constexpr PlaneMask operator&(PlaneMask lhs, PlaneMask rhs)
{
	return static_cast<PlaneMask>(static_cast<std::uint8_t>(lhs) & static_cast<std::uint8_t>(rhs));
}

constexpr bool HasPlane(PlaneMask mask, PlaneKind kind)
{
	const auto bit = kind == PlaneKind::kRgb   ? PlaneMask::kRgb :
	                 kind == PlaneKind::kDepth ? PlaneMask::kDepth :
	                                             PlaneMask::kSegmentation;
	return (mask & bit) != PlaneMask::kNone;
}

struct PlaneLayout
{
	int width                = 0;
	int height               = 0;
	std::size_t stride_bytes = 0;
	std::size_t byte_length  = 0;

	static PlaneLayout Rgb8(int width, int height) { return Interleaved(width, height, 3U); }

	static PlaneLayout Segmentation8(int width, int height) { return Interleaved(width, height, 3U); }

	static PlaneLayout Depth32F(int width, int height) { return Interleaved(width, height, sizeof(float)); }

	bool valid() const
	{
		return width > 0 && height > 0 && stride_bytes >= static_cast<std::size_t>(width) &&
		       byte_length == stride_bytes * static_cast<std::size_t>(height);
	}

private:
	static PlaneLayout Interleaved(int width, int height, std::size_t bytes_per_pixel)
	{
		if (width <= 0 || height <= 0 ||
		    static_cast<std::size_t>(width) > std::numeric_limits<std::size_t>::max() / bytes_per_pixel) {
			throw std::invalid_argument("render plane dimensions are invalid");
		}
		const auto stride = static_cast<std::size_t>(width) * bytes_per_pixel;
		if (static_cast<std::size_t>(height) > std::numeric_limits<std::size_t>::max() / stride) {
			throw std::invalid_argument("render plane byte length overflows");
		}
		return PlaneLayout{ width, height, stride, stride * static_cast<std::size_t>(height) };
	}
};

struct FrameLayout
{
	int width                    = 0;
	int height                   = 0;
	std::size_t slot_byte_length = 0;

	FrameLayout() = default;
	FrameLayout(int width_in, int height_in)
	    : width(width_in), height(height_in), slot_byte_length(LargestPlaneByteLength(width_in, height_in))
	{
	}
	FrameLayout(int width_in, int height_in, std::size_t warm_byte_length)
	    : width(width_in), height(height_in), slot_byte_length(warm_byte_length)
	{
	}

	static std::size_t LargestPlaneByteLength(int width_in, int height_in)
	{
		return std::max({ PlaneLayout::Rgb8(width_in, height_in).byte_length,
		                  PlaneLayout::Depth32F(width_in, height_in).byte_length,
		                  PlaneLayout::Segmentation8(width_in, height_in).byte_length });
	}
};

struct CameraMetadata
{
	CameraId id = 0;
	std::string name;
	int width               = 0;
	int height              = 0;
	float focal_length_x    = 0.0F;
	float focal_length_y    = 0.0F;
	float principal_point_x = 0.0F;
	float principal_point_y = 0.0F;
};

struct CameraDescriptor
{
	CameraId id = 0;
	std::string name;
	int width        = 0;
	int height       = 0;
	PlaneMask planes = PlaneMask::kRgb;
	CameraMetadata metadata;
	mjvOption visual_options = DefaultCameraVisualOptions();

	PlaneLayout layout(PlaneKind kind) const
	{
		if (!HasPlane(planes, kind)) {
			throw std::invalid_argument("requested plane is not configured for camera");
		}
		return kind == PlaneKind::kDepth        ? PlaneLayout::Depth32F(width, height) :
		       kind == PlaneKind::kSegmentation ? PlaneLayout::Segmentation8(width, height) :
		                                          PlaneLayout::Rgb8(width, height);
	}
};

} // namespace mujoco_ros::rendering
