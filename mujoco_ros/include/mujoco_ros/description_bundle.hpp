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

#include <map>
#include <optional>
#include <string>

namespace mujoco_ros {

enum class BaseMode
{
	kAuto,
	kFixed,
	kFree
};

// Accepts exactly "auto", "fixed", and "free"; throws std::runtime_error otherwise.
BaseMode ParseBaseMode(const std::string &value);

// Shared per-robot-instance description bundle. Public core fields only.
// value plus domain_id/attach_prefix as an *additive* read on the same
// top-level mujoco_server params, never a second parallel schema.
struct DescriptionSource
{
	enum class Kind
	{
		kFile,
		kTopic
	};
	Kind kind = Kind::kFile;
	std::string path; // when kind == kFile
	std::string topic; // when kind == kTopic, defaults applied by the parser
};

struct DescriptionBundle
{
	DescriptionSource urdf;
	std::optional<DescriptionSource> srdf; // absent => no disable_collisions
	bool convert_ascii_stl  = false;
	bool generate_actuators = false; // opt-in actuator generation
	std::string attach_prefix; // default ""; explicit multi-robot callers pass e.g. "r0_"/"r1_"
	BaseMode base_mode = BaseMode::kAuto;
};

// Returns std::nullopt when neither "urdf.source" nor "srdf.source" is
// present in flat_params (the common case today: a plain `modelfile`-only
// launch) so callers can fall back to the pre-existing load path unchanged.
// Presence of only "srdf.source" is not a silent no-op: ParseDescriptionBundleFromMap
// throws because URDF configuration is required.
std::optional<DescriptionBundle>
TryParseDescriptionBundleFromMap(const std::map<std::string, std::string> &flat_params);

// Throws std::runtime_error if the bundle is absent or malformed (missing a
// required field for the declared source kind, or SRDF without URDF).
DescriptionBundle ParseDescriptionBundleFromMap(const std::map<std::string, std::string> &flat_params);

} // namespace mujoco_ros
