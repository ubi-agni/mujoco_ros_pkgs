#pragma once

#include <cstdint>

namespace mujoco_ros {

template <typename Tag>
class GenerationId
{
public:
	explicit constexpr GenerationId(std::uint64_t value = 0) : value_(value) {}
	constexpr std::uint64_t value() const { return value_; }
	friend constexpr bool operator==(GenerationId lhs, GenerationId rhs) { return lhs.value_ == rhs.value_; }
	friend constexpr bool operator!=(GenerationId lhs, GenerationId rhs) { return !(lhs == rhs); }

private:
	std::uint64_t value_;
};

struct ModelGenerationTag;
struct PluginGenerationTag;
struct FrameGenerationTag;
struct OptionsEpochTag;

using ModelGeneration  = GenerationId<ModelGenerationTag>;
using PluginGeneration = GenerationId<PluginGenerationTag>;
using FrameGeneration  = GenerationId<FrameGenerationTag>;
using OptionsEpoch     = GenerationId<OptionsEpochTag>;

} // namespace mujoco_ros
