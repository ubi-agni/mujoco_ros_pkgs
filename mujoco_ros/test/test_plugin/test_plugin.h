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
	virtual void controlCallback(mjModelPtr model, mjDataPtr data) override;
	virtual void passiveCallback(mjModelPtr model, mjDataPtr data) override;
	virtual void renderCallback(mjModelPtr model, mjDataPtr data, mjvScene *scene) override;
	virtual void lastStageCallback(mjModelPtr model, mjDataPtr data) override;
	virtual void onGeomChanged(mjModelPtr model, mjDataPtr data, const int geom_id) override;

	mjModelPtr m_;
	mjDataPtr d_;
	bool ran_reset              = false;
	bool ran_control_cb         = false;
	bool ran_passive_cb         = false;
	bool ran_render_cb          = false;
	bool ran_last_cb            = false;
	bool ran_on_geom_changed_cb = false;
	bool got_config_param       = false;
	bool got_lvl1_nested_array  = false;
	bool got_lvl1_nested_struct = false;
	bool got_lvl2_nested_array  = false;
	bool got_lvl2_nested_struct = false;
	bool should_fail            = false;
};
} // namespace mujoco_ros
