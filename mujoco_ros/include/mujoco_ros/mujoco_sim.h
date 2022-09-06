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

#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <ros/ros.h>

#include "mjxmacro.h"

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/mujoco_env.h>

#include <mujoco_ros/uitools.h>
#include <mujoco_ros/array_safety.h>

#include <mujoco_ros_msgs/SetPause.h>
#include <std_srvs/Empty.h>

namespace mju = ::mujoco::sample_util;

namespace MujocoSim {

#define mjENABLED_ros(model, x) (model->opt.enableflags & (x))
#define mjDISABLED_ros(model, x) (model->opt.disableflags & (x))

void init(std::string modelfile);

void requestExternalShutdown(void);

// Threading and synchronization
static std::mutex sim_mtx;
static std::mutex render_mtx;

int jointName2id(mjModel *m, const std::string &joint_name);

void setJointPosition(mjModelPtr model, mjDataPtr data, const double &pos, const int &joint_id,
                      const int &jnt_axis = 0);
void setJointVelocity(mjModelPtr model, mjDataPtr data, const double &vel, const int &joint_id,
                      const int &jnt_axis = 0);

// Keep track of overriden collisions to throw warnings
static std::set<std::pair<int, int>> custom_collisions_;

/**
 * @brief Register a custom collision function for collisions between two geom types.
 *
 * @param [in] geom_type1 first geom type of the colliding geoms.
 * @param [in] geom_type2 second type of the colliding geoms.
 * @param [in] collision_cb collision function to call.
 */
void registerCollisionFunc(int geom_type1, int geom_type2, mjfCollision collision_cb);

/**
 * @brief Reset the simulation.
 *
 */
void resetSim();

void setupVFS(const std::string &filename, const std::string &content = std::string());

namespace detail {

// Env containing model and data
static MujocoEnvPtr mj_env_;

// Helper function for unit tests to access the env
namespace unit_testing {
/**
 * @brief Gets the current MujocoEnv. This function is meant for unit testing only and might return a nullpointer if
 * used wrongly.
 *
 * @return Pointer to the current MujocoEnv.
 */
MujocoEnvPtr getmjEnv();
} // namespace unit_testing

// MuJoCo virtual filesystem to store mjcf in-memory.
static mjVFS vfs_;

// filename strings
static char filename_[kBufSize]          = "";
static char previous_filename_[kBufSize] = "";

// info strings
static char info_title_[kBufSize];
static char info_content_[kBufSize];

// helpers
static bool vis_;
static double last_rendered_ = 0;

// ROS callback for sim time publisher on /clock
void publishSimTime(mjtNum time);
static ros::Publisher pub_clock_;
static boost::shared_ptr<ros::NodeHandle> nh_;

// Services
static std::vector<ros::ServiceServer> service_servers_;

// Keep track of time for resets to not mess up ros time
static mjtNum last_time_ = -1;

// constants
const int maxgeom_          = 5000; // preallocated geom array in mjvScene
const double syncmisalign_  = 0.1; // maximum time mis-alignment before re-sync
const double refreshfactor_ = 0.7; // fraction of refresh available for simulation
const int max_slow_down_    = 128; // maximum slow-down quotient

// abstract visualization
static mjvScene scn_;
static mjvCamera cam_;
static mjvOption vopt_;
static mjvPerturb pert_;
static mjvFigure figconstraint_;
static mjvFigure figcost_;
static mjvFigure figtimer_;
static mjvFigure figsize_;
static mjvFigure figsensor_;

// OpenGL renderin and UI
static GLFWvidmode vmode_;
static int windowpos_[2];
static int windowsize_[2];
static mjrContext con_;
static GLFWwindow *window_ = NULL;
static mjuiState uistate_;
static mjUI ui0_, ui1_;

void initVisual(void);

// Profiler, Sensor, Info, Watch
void profilerInit(void);
void profilerUpdate(mjModelPtr model, mjDataPtr data);
void profilerShow(mjrRect rect);

void sensorInit(void);
void sensorUpdate(mjModelPtr model, mjDataPtr data);
void sensorShow(mjrRect rect);

void infotext(mjModelPtr model, mjDataPtr data, char (&title)[kBufSize], char (&content)[kBufSize], double interval);

void printField(char (&str)[mjMAXUINAME], void *ptr);

void watch(mjModelPtr model, mjDataPtr data);

// UI Elements
void makePhysics(mjModelPtr model, mjDataPtr data, int oldstate);
void makeRendering(mjModelPtr model, int oldstate);
void makeGroup(int oldstate);
void makeJoint(mjModelPtr model, mjDataPtr data, int oldstate);
void makeControl(mjModelPtr model, mjDataPtr data, int oldstate);
void makeSections(void);

// Utility Functions
void alignScale(mjModelPtr model);
void copyKey(mjModelPtr model, mjDataPtr data);
mjtNum timer(void);
void clearTimers(mjDataPtr data);
void printCamera(mjvGLCamera *camera);
void updateSettings(mjModelPtr model);
void drop(GLFWwindow *window, int count, const char **paths);
void loadModel(void);
void loadInitialJointStates(mjModelPtr model, mjDataPtr data);

// UI Hooks for uitools.h
int uiPredicate(int category, void *userdata);
void uiLayout(mjuiState *state);
void uiEvent(mjuiState *state);

// Rendering and Simulation
void prepare(mjModelPtr model, mjDataPtr data);
void render(GLFWwindow *window);

// Threads
void simulate(void);
void eventloop(void);

// Service calls
void setupCallbacks();
bool shutdownCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
bool setPauseCB(mujoco_ros_msgs::SetPause::Request &req, mujoco_ros_msgs::SetPause::Response &resp);
bool resetCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

// UI settings not contained in MuJoCo structures
struct
{
	// file
	int exitrequest = 0;

	// option
	int spacing    = 0;
	int color      = 0;
	int font       = 0;
	int ui0        = 1;
	int ui1        = 1;
	int help       = 0;
	int info       = 0;
	int profiler   = 0;
	int sensor     = 0;
	int fullscreen = 0;
	int vsync      = 1;
	int busywait   = 0;

	// simulation
	int run              = 0;
	int key              = 0;
	int loadrequest      = 0;
	int slow_down        = 1;
	bool speed_changed   = true;
	double ctrlnoisestd  = 0.0;
	double ctrlnoiserate = 0.0;

	// watch
	char field[mjMAXUITEXT] = "qpos";
	int index               = 0;

	// physics: need sync
	int disable[mjNDISABLE];
	int enable[mjNENABLE];

	// rendering: need sync
	int camera = 0;
} settings_;

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
const mjuiDef defFile[] = { { mjITEM_SECTION, "File", 1, NULL, "AF" },
	                         { mjITEM_BUTTON, "Save xml", 2, NULL, "" },
	                         { mjITEM_BUTTON, "Save mjb", 2, NULL, "" },
	                         { mjITEM_BUTTON, "Print model", 2, NULL, "CM" },
	                         { mjITEM_BUTTON, "Print data", 2, NULL, "CD" },
	                         { mjITEM_BUTTON, "Quit", 1, NULL, "CQ" },
	                         { mjITEM_END } };

// option section of UI
const mjuiDef defOption[] = { { mjITEM_SECTION, "Option", 1, NULL, "AO" },
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
const mjuiDef defSimulation[] = { { mjITEM_SECTION, "Simulation", 1, NULL, "AS" },
	                               { mjITEM_RADIO, "", 2, &settings_.run, "Pause\nRun" },
	                               { mjITEM_BUTTON, "Reset", 2, NULL, " #259" },
	                               { mjITEM_BUTTON, "Reload", 2, NULL, "CL" },
	                               { mjITEM_BUTTON, "Align", 2, NULL, "CA" },
	                               { mjITEM_BUTTON, "Copy pose", 2, NULL, "CC" },
	                               { mjITEM_SLIDERINT, "Key", 3, &settings_.key, "0 0" },
	                               { mjITEM_BUTTON, "Reset to key", 3 },
	                               { mjITEM_BUTTON, "Set key", 3 },
	                               { mjITEM_SLIDERNUM, "Noise scale", 2, &settings_.ctrlnoisestd, "0 2" },
	                               { mjITEM_SLIDERNUM, "Noise rate", 2, &settings_.ctrlnoiserate, "0 2" },
	                               { mjITEM_END } };

// watch section of UI
const mjuiDef defWatch[] = { { mjITEM_SECTION, "Watch", 0, NULL, "AW" },
	                          { mjITEM_EDITTXT, "Field", 2, &settings_.field, "qpos" },
	                          { mjITEM_EDITINT, "Index", 2, &settings_.index, "1" },
	                          { mjITEM_STATIC, "Value", 2, NULL, " " },
	                          { mjITEM_END } };

// help strings
const char help_content_[] = "Alt mouse button\n"
                             "UI right hold\n"
                             "UI title double-click\n"
                             "Space\n"
                             "Esc\n"
                             "Right arrow\n"
                             "Left arrow\n"
                             "Down arrow\n"
                             "Up arrow\n"
                             "Page Up\n"
                             "Double-click\n"
                             "Right double-click\n"
                             "Ctrl Right double-click\n"
                             "Scroll, middle drag\n"
                             "Left drag\n"
                             "[Shift] right drag\n"
                             "Ctrl [Shift] drag\n"
                             "Ctrl [Shift] right drag";

const char help_title_[] = "Swap left-right\n"
                           "Show UI shortcuts\n"
                           "Expand/collapse all  \n"
                           "Pause\n"
                           "Free camera\n"
                           "Step forward\n"
                           "Step back\n"
                           "Step forward 100\n"
                           "Step back 100\n"
                           "Select parent\n"
                           "Select\n"
                           "Center\n"
                           "Track camera\n"
                           "Zoom\n"
                           "View rotate\n"
                           "View translate\n"
                           "Object rotate\n"
                           "Object translate";
} // end namespace detail
} // end namespace MujocoSim
