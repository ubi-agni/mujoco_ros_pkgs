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

#include <pluginlib/class_list_macros.h>

#include "test_plugin.h"

using namespace mujoco_ros;
namespace mujoco_ros {

bool TestPlugin::load(const mjModel *m, mjData *d)
{
	if (rosparam_config_.hasMember("example_param")) {
		got_config_param.store(true);
	}

	if (rosparam_config_.hasMember("nested_array_param_1")) {
		if (rosparam_config_["nested_array_param_1"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
			got_lvl1_nested_array.store(true);
			if (rosparam_config_["nested_array_param_1"][0].hasMember("nested_array_param_2")) {
				got_lvl2_nested_array.store(true);
			}
		}
	}

	if (rosparam_config_.hasMember("nested_struct_param_1")) {
		if (rosparam_config_["nested_struct_param_1"].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
			got_lvl1_nested_struct.store(true);
			if (rosparam_config_["nested_struct_param_1"].hasMember("nested_struct_param_2")) {
				got_lvl2_nested_struct.store(true);
			}
		}
	}

	bool tmp_fail = false;
	node_handle_.param<bool>("should_fail", tmp_fail, false);
	should_fail.store(tmp_fail);
	if (tmp_fail) {
		return false;
	}

	m_ = m;
	d_ = d;
	return true;
}

void TestPlugin::reset()
{
	ran_reset.store(true);
}

void TestPlugin::controlCallback(const mjModel * /*model*/, mjData * /*data*/)
{
	ran_control_cb.store(true);
}

void TestPlugin::passiveCallback(const mjModel * /*model*/, mjData * /*data*/)
{
	ran_passive_cb.store(true);
}

void TestPlugin::renderCallback(const mjModel * /*model*/, mjData * /*data*/, mjvScene * /*scene*/)
{
	ran_render_cb.store(true);
}

void TestPlugin::lastStageCallback(const mjModel * /*model*/, mjData * /*data*/)
{
	ran_last_cb.store(true);
}

void TestPlugin::onGeomChanged(const mjModel * /*model*/, mjData * /*data*/, const int /*geom_id*/)
{
	ran_on_geom_changed_cb.store(true);
}
} // namespace mujoco_ros

PLUGINLIB_EXPORT_CLASS(mujoco_ros::TestPlugin, mujoco_ros::MujocoPlugin)
