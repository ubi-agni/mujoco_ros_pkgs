/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Bielefeld University
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
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

/* Authors: David P. Leins */

#pragma once

#include <mujoco_ros/plugin_utils.h>
#include <mujoco_ros/common_types.h>
#include <mujoco_ros/mujoco_env.h>

using namespace mujoco_ros;
namespace mujoco_ros {
class TestPlugin : public MujocoPlugin
{
public:
	TestPlugin() {}
	virtual ~TestPlugin() {}
	virtual bool load(mjModelPtr m, mjDataPtr d) override;
	virtual void reset() override;
	virtual void controlCallback([[maybe_unused]] mjModelPtr model, [[maybe_unused]] mjDataPtr data) override;
	virtual void passiveCallback([[maybe_unused]] mjModelPtr model, [[maybe_unused]] mjDataPtr data) override;
	virtual void renderCallback([[maybe_unused]] mjModelPtr model, [[maybe_unused]] mjDataPtr data,
	                            [[maybe_unused]] mjvScene *scene) override;
	virtual void lastStageCallback([[maybe_unused]] mjModelPtr model, [[maybe_unused]] mjDataPtr data) override;
	virtual void onGeomChanged([[maybe_unused]] mjModelPtr model, [[maybe_unused]] mjDataPtr data,
	                           [[maybe_unused]] const int geom_id) override;

	mjModelPtr m_;
	mjDataPtr d_;
	std::atomic_int ran_reset              = { false };
	std::atomic_int ran_control_cb         = { false };
	std::atomic_int ran_passive_cb         = { false };
	std::atomic_int ran_render_cb          = { false };
	std::atomic_int ran_last_cb            = { false };
	std::atomic_int ran_on_geom_changed_cb = { false };
	std::atomic_int got_config_param       = { false };
	std::atomic_int got_lvl1_nested_array  = { false };
	std::atomic_int got_lvl1_nested_struct = { false };
	std::atomic_int got_lvl2_nested_array  = { false };
	std::atomic_int got_lvl2_nested_struct = { false };
	std::atomic_int should_fail            = { false };
};
} // namespace mujoco_ros
