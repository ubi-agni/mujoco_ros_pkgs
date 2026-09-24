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

#include <mujoco_ros/description_bundle.hpp>

#include <stdexcept>

namespace mujoco_ros {

BaseMode ParseBaseMode(const std::string &value)
{
	if (value == "auto")
		return BaseMode::kAuto;
	if (value == "fixed")
		return BaseMode::kFixed;
	if (value == "free")
		return BaseMode::kFree;
	throw std::runtime_error("Description bundle: 'description.base_mode' must be 'auto', 'fixed', or 'free', got '" +
	                         value + "'");
}

namespace {

std::optional<std::string> Get(const std::map<std::string, std::string> &m, const std::string &key)
{
	auto it = m.find(key);
	return it == m.end() ? std::nullopt : std::optional<std::string>(it->second);
}

bool ParseOptionalBool(const std::map<std::string, std::string> &m, const std::string &key)
{
	auto val = Get(m, key);
	if (!val.has_value())
		return false;
	if (*val == "true")
		return true;
	if (*val == "false")
		return false;
	throw std::runtime_error("Description bundle: '" + key + "' must be 'true' or 'false', got '" + *val + "'");
}

DescriptionSource ParseSource(const std::map<std::string, std::string> &m, const std::string &prefix,
                              const std::string &default_param_name)
{
	DescriptionSource source;
	auto kind = Get(m, prefix + ".source");
	if (!kind.has_value())
		throw std::runtime_error("Description bundle: missing '" + prefix + ".source'");

	if (*kind == "file") {
		source.kind = DescriptionSource::Kind::kFile;
		auto path   = Get(m, prefix + ".path");
		if (!path.has_value())
			throw std::runtime_error("Description bundle: '" + prefix + ".source' is 'file' but '" + prefix +
			                         ".path' is missing");
		source.path = *path;
	} else if (*kind == "topic") {
		source.kind  = DescriptionSource::Kind::kTopic;
		source.topic = Get(m, prefix + ".topic").value_or(default_param_name);
	} else {
		throw std::runtime_error("Description bundle: '" + prefix + ".source' must be 'file' or 'topic', got '" + *kind +
		                         "'");
	}
	return source;
}

} // namespace

std::optional<DescriptionBundle> TryParseDescriptionBundleFromMap(const std::map<std::string, std::string> &flat_params)
{
	if (!Get(flat_params, "urdf.source").has_value()) {
		if (Get(flat_params, "srdf.source").has_value())
			return ParseDescriptionBundleFromMap(flat_params); // throws: URDF required
		return std::nullopt;
	}
	return ParseDescriptionBundleFromMap(flat_params);
}

DescriptionBundle ParseDescriptionBundleFromMap(const std::map<std::string, std::string> &flat_params)
{
	DescriptionBundle bundle;
	bundle.urdf = ParseSource(flat_params, "urdf", "robot_description");
	if (Get(flat_params, "srdf.source").has_value())
		bundle.srdf = ParseSource(flat_params, "srdf", "robot_description_semantic");
	bundle.convert_ascii_stl  = ParseOptionalBool(flat_params, "description.convert_ascii_stl");
	bundle.generate_actuators = ParseOptionalBool(flat_params, "description.generate_actuators");
	bundle.attach_prefix      = Get(flat_params, "description.attach_prefix").value_or("");
	if (auto base_mode = Get(flat_params, "description.base_mode"); base_mode.has_value())
		bundle.base_mode = ParseBaseMode(*base_mode);
	return bundle;
}

} // namespace mujoco_ros
