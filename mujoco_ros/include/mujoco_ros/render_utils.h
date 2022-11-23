// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Bielefeld University
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

#include <cstdio>
#include <cstring>
#include <string>

#include <GLFW/glfw3.h>

#include <mujoco/mjxmacro.h>

#include <mujoco_ros/uitools.h>
#include <mujoco_ros/array_safety.h>
#include <mujoco_ros/common_types.h>
#include <mujoco_ros/mujoco_sim.h>

namespace mju = ::mujoco::sample_util;

using namespace MujocoSim::detail;

namespace MujocoSim::render_utils {

// Extern declaration for global access
extern mjvPerturb pert_;

// abstract visualization
static mjvFigure figconstraint_ = {};
static mjvFigure figcost_       = {};
static mjvFigure figtimer_      = {};
static mjvFigure figsize_       = {};
static mjvFigure figsensor_     = {};

// OpenGL renderin and UI
static GLFWvidmode vmode_       = {};
static int windowpos_[2]        = { 0 };
static int windowsize_[2]       = { 0 };
static GLFWwindow *main_window_ = nullptr;

static mjvScene free_scene_;
static mjrContext free_context_;
static mjvCamera free_camera_;
static mjvOption vopt_;

static mjuiState uistate_ = {};
static mjUI ui0_          = {};
static mjUI ui1_          = {};

// constants
const int maxgeom_                      = 5000; // preallocated geom array in mjvScene
const int max_slow_down_                = 128; // maximum slow-down quotient
const float render_ui_rate_lower_bound_ = 0.0333; // Minimum render freq at 30 fps
const float render_ui_rate_upper_bound_ = 0.0166; // Maximum render freq at 60 fps

// info strings
static char info_title[kBufSize];
static char info_content[kBufSize];

bool isWindowClosing(void);

void eventloop(void);

void initVisual(void);
void deinitVisual(void);

/**
 * Renders and publishes offscreen camera images of the provided MujocoEnv according to their configuration.
 *
 * @param env Environment to render.
 */
void offScreenRenderEnv(MujocoEnvPtr env);

/**
 * Update visual elements according to the model loaded by env.
 *
 * @param env Environment providing the model to update visual elements for.
 * @param align Whether to re-align the free camera.
 */
void onModelLoad(MujocoEnvPtr env, bool align);

// Profiler, Sensor, Info, Watch
void profilerInit(void);
void profilerUpdate(MujocoEnvPtr env);
void profilerShow(MujocoEnvPtr env, mjrRect rect);

void sensorInit(void);
void sensorUpdate(MujocoEnvPtr env);
void sensorShow(MujocoEnvPtr env, mjrRect rect);

void infotext(MujocoEnvPtr env, char (&title)[kBufSize], char (&content)[kBufSize], double interval);

void printField(char (&str)[mjMAXUINAME], void *ptr);

void watch(MujocoEnvPtr env);

// UI Elements
void makePhysics(MujocoEnvPtr env, int oldstate);
void makeRendering(MujocoEnvPtr env, int oldstate);
void makeGroup(MujocoEnvPtr env, int oldstate);
void makeJoint(MujocoEnvPtr env, int oldstate);
void makeControl(MujocoEnvPtr env, int oldstate);
void makeSections(MujocoEnvPtr env);

// Utility Functions
void alignScale(MujocoEnvPtr env);
void copyKey(MujocoEnvPtr env);
mjtNum timer(void);
void clearTimers(mjDataPtr data);
void printCamera(mjvGLCamera *camera);
void updateSettings(MujocoEnvPtr env);

// UI Hooks for uitools.h
int uiPredicate(int category, void *userdata);
void uiLayout(mjuiState *state);
void uiEvent(mjuiState *state);

// Rendering and Simulation
/**
 * Update scene and UI to be rendered on-screen later on. This function accesses data from the current main MujocoEnv
 * (read only), an thus should only be called in an access-safe context when it is ensured that the env is not
 * concurrently updated.
 *
 * @param interval Interval since the last update.
 */
void prepareOnScreen(const ros::WallDuration &r_interval);

/**
 * Render the current main MujocoEnv on-screen.
 */
void renderMain();

struct CamStream
{
public:
	CamStream(uint8_t cam_id, streamType stream_type, image_transport::Publisher rgb, image_transport::Publisher depth,
	          image_transport::Publisher segment, bool use_segid, float pub_freq)
	    : cam_id(cam_id)
	    , stream_type(stream_type)
	    , rgb_pub(rgb)
	    , depth_pub(depth)
	    , segment_pub(segment)
	    , use_segid(use_segid)
	    , pub_freq(pub_freq)
	{
		last_pub = ros::Time::now();
	}

	uint8_t cam_id;
	streamType stream_type = streamType::RGB;
	image_transport::Publisher rgb_pub;
	image_transport::Publisher depth_pub;
	image_transport::Publisher segment_pub;
	bool use_segid = true;
	float pub_freq = 15;
	ros::Time last_pub;
};

// section ids
enum
{
	// left ui
	SECT_FILE = 0,
	SECT_OPTION,
	SECT_SIMULATION,
	SECT_WATCH,
	SECT_PHYSICS,
	SECT_RENDERING,
	SECT_GROUP,
	NSECT0,

	// right ui
	SECT_JOINT = 0,
	SECT_CONTROL,
	NSECT1
};

// file section of UI
const mjuiDef defFile[] = { { mjITEM_SECTION, "File", 1, nullptr, "AF" },
	                         { mjITEM_BUTTON, "Save xml", 2, nullptr, "" },
	                         { mjITEM_BUTTON, "Save mjb", 2, nullptr, "" },
	                         { mjITEM_BUTTON, "Print model", 2, nullptr, "CM" },
	                         { mjITEM_BUTTON, "Print data", 2, nullptr, "CD" },
	                         { mjITEM_BUTTON, "Quit", 1, nullptr, "CQ" },
	                         { mjITEM_END } };

// option section of UI
const mjuiDef defOption[] = { { mjITEM_SECTION, "Option", 1, nullptr, "AO" },
	                           { mjITEM_SELECT, "Spacing", 1, &settings_.spacing, "Tight\nWide" },
	                           { mjITEM_SELECT, "Color", 1, &settings_.color, "Default\nOrange\nWhite\nBlack" },
	                           { mjITEM_SELECT, "Font", 1, &settings_.font, "50 %\n100 %\n150 %\n200 %\n250 %\n300 %" },
	                           { mjITEM_CHECKINT, "Left UI (Tab)", 1, &settings_.ui0, " #258" },
	                           { mjITEM_CHECKINT, "Right UI", 1, &settings_.ui1, "S#258" },
	                           { mjITEM_CHECKINT, "Help", 2, &settings_.help, " #290" },
	                           { mjITEM_CHECKINT, "Info", 2, &settings_.info, " #291" },
	                           { mjITEM_CHECKINT, "Profiler", 2, &settings_.profiler, " #292" },
	                           { mjITEM_CHECKINT, "Sensor", 2, &settings_.sensor, " #293" },
#ifdef __APPLE__
	                           { mjITEM_CHECKINT, "Fullscreen", 0, &settings_.fullscreen, " #294" },
#else
	                           { mjITEM_CHECKINT, "Fullscreen", 1, &settings_.fullscreen, " #294" },
#endif
	                           { mjITEM_CHECKINT, "Vertical Sync", 1, &settings_.vsync, " #295" },
	                           { mjITEM_CHECKINT, "Busy Wait", 1, &settings_.busywait, " #296" },
	                           { mjITEM_END } };

// simulation section of UI
const mjuiDef defSimulation[] = { { mjITEM_SECTION, "Simulation", 1, nullptr, "AS" },
	                               { mjITEM_RADIO, "", 2, &settings_.run, "Pause\nRun" },
	                               { mjITEM_BUTTON, "Reset", 2, nullptr, " #259" },
	                               { mjITEM_BUTTON, "Reload", 2, nullptr, "CL" },
	                               { mjITEM_BUTTON, "Align", 2, nullptr, "CA" },
	                               { mjITEM_BUTTON, "Copy pose", 2, nullptr, "CC" },
	                               { mjITEM_SLIDERINT, "Key", 3, &settings_.key, "0 0" },
	                               { mjITEM_BUTTON, "Load key", 3 },
	                               { mjITEM_BUTTON, "Set key", 3 },
	                               { mjITEM_SLIDERNUM, "Noise scale", 2, &settings_.ctrlnoisestd, "0 2" },
	                               { mjITEM_SLIDERNUM, "Noise rate", 2, &settings_.ctrlnoiserate, "0 2" },
	                               { mjITEM_END } };

// watch section of UI
const mjuiDef defWatch[] = { { mjITEM_SECTION, "Watch", 0, nullptr, "AW" },
	                          { mjITEM_EDITTXT, "Field", 2, &settings_.field, "qpos" },
	                          { mjITEM_EDITINT, "Index", 2, &settings_.index, "1" },
	                          { mjITEM_STATIC, "Value", 2, nullptr, " " },
	                          { mjITEM_END } };

// help strings
const char help_content[] = "Space\n"
                            "+  -\n"
                            "Right arrow\n"
                            "[  ]\n"
                            "Esc\n"
                            "Double-click\n"
                            "Page Up\n"
                            "Right double-click\n"
                            "Ctrl Right double-click\n"
                            "Scroll, middle drag\n"
                            "Left drag\n"
                            "[Shift] right drag\n"
                            "Ctrl [Shift] drag\n"
                            "Ctrl [Shift] right drag\n"
                            "F1\n"
                            "F2\n"
                            "F3\n"
                            "F4\n"
                            "F5\n"
                            "UI right hold\n"
                            "UI title double-click";

const char help_title[] = "Play / Pause\n"
                          "Speed up / down\n"
                          "Step\n"
                          "Cycle cameras\n"
                          "Free camera\n"
                          "Select\n"
                          "Select parent\n"
                          "Center\n"
                          "Tracking camera\n"
                          "Zoom\n"
                          "View rotate\n"
                          "View translate\n"
                          "Object rotate\n"
                          "Object translate\n"
                          "Help\n"
                          "Info\n"
                          "Profiler\n"
                          "Sensors\n"
                          "Full screen\n"
                          "Show UI shortcuts\n"
                          "Expand/collapse all";

} // namespace MujocoSim::render_utils
