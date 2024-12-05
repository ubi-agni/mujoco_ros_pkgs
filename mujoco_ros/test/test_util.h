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

#include <mujoco_ros/common_types.h>
using namespace mujoco_ros;

void compare_qpos(mjData *d, int qpos_adr_int, const std::string &joint_name, const std::vector<double> &values,
                  const std::vector<double> &tolerances = {})
{
	uint qpos_adr = static_cast<uint>(qpos_adr_int);
	if (tolerances.empty()) {
		for (size_t i = 0; i < values.size(); i++) {
			EXPECT_EQ(d->qpos[qpos_adr + i], values[i]) << "qpos of joint '" << joint_name << "' at index " << i << " is "
			                                            << d->qpos[qpos_adr + i] << " instead of " << values[i] << "!";
		}
	} else {
		for (size_t i = 0; i < values.size(); i++) {
			EXPECT_NEAR(d->qpos[qpos_adr + i], values[i], tolerances[i])
			    << "qpos of joint '" << joint_name << "' at index " << i << " is " << d->qpos[qpos_adr + i]
			    << " instead of " << values[i] << " (tolerance: " << tolerances[i] << ")!";
		}
	}
}

void compare_qvel(mjData *d, int dof_adr_int, const std::string &joint_name, const std::vector<double> &values,
                  const std::vector<double> &tolerances = {})
{
	uint dof_adr = static_cast<uint>(dof_adr_int);
	if (tolerances.empty()) {
		for (size_t i = 0; i < values.size(); i++) {
			EXPECT_EQ(d->qvel[dof_adr + i], values[i]) << "qvel of joint '" << joint_name << "' at index " << i << " is "
			                                           << d->qvel[dof_adr + i] << " instead of " << values[i] << "!";
		}
	} else {
		for (size_t i = 0; i < values.size(); i++) {
			EXPECT_NEAR(d->qvel[dof_adr + i], values[i], tolerances[i])
			    << "qvel of joint '" << joint_name << "' at index " << i << " is " << d->qvel[dof_adr + i]
			    << " instead of " << values[i] << " (tolerance: " << tolerances[i] << ")!";
		}
	}
}
