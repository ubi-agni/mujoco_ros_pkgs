/* Heavily inspired by MuJoCo's Simulate application with below license: */

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

#include <mujoco_ros/viewer.h>

#include <mujoco/mujoco.h>
#include <mujoco/mjxmacro.h>
#include <mujoco_ros/lodepng.h>
#include <mujoco_ros/util.h>

#include <climits>

static std::string GetSavePath(const char *filename)
{
	return filename;
}

namespace {
namespace mju = ::mujoco::sample_util;
using namespace mujoco_ros;

using Seconds      = std::chrono::duration<double>;
using Milliseconds = std::chrono::duration<double, std::milli>;

template <typename T>
inline bool IsDifferent(const T &a, const T &b)
{
	if constexpr (std::is_array_v<T>) {
		static_assert(std::rank_v<T> == 1);
		for (int i = 0; i < std::extent_v<T>; ++i) {
			if (a[i] != b[i]) {
				return true;
			}
		}
		return false;
	} else {
		return a != b;
	}
}

template <typename T>
inline void CopyScalar(T &dst, const T &src)
{
	dst = src;
}

template <typename T, int N>
inline void CopyArray(T (&dst)[N], const T (&src)[N])
{
	for (int i = 0; i < N; ++i) {
		dst[i] = src[i];
	}
}

template <typename T>
inline void Copy(T &dst, const T &src)
{
	if constexpr (std::is_array_v<T>) {
		CopyArray(dst, src);
	} else {
		CopyScalar(dst, src);
	}
}

// ratio of one click-wheel zoom increment to vertical extent
const double zoom_increment = 0.02;

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
	SECT_VISUALIZATION,
	SECT_GROUP,
	NSECT0,

	// right ui
	SECT_JOINT = 0,
	SECT_CONTROL,
	NSECT1
};

// file section of UI
const mjuiDef defFile[] = {
	{ mjITEM_SECTION, "File", 1, nullptr, "AF" },      { mjITEM_BUTTON, "Save xml", 2, nullptr, "" },
	{ mjITEM_BUTTON, "Save mjb", 2, nullptr, "" },     { mjITEM_BUTTON, "Print model", 2, nullptr, "CM" },
	{ mjITEM_BUTTON, "Print data", 2, nullptr, "CD" }, { mjITEM_BUTTON, "Quit", 1, nullptr, "CQ" },
	{ mjITEM_BUTTON, "Screenshot", 2, nullptr, "CP" }, { mjITEM_END }
};

// help string
const char help_content[] = "Space\n"
                            "+  -\n"
                            "Left / Right arrow\n"
                            "Tab / Shift-Tab\n"
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
                            "UI right-button hold\n"
                            "UI title double-click";

const char help_title[] = "Play / Pause\n"
                          "Speed Up / Down\n"
                          "Step Back / Forward\n"
                          "Toggle Left / Right UI\n"
                          "Cycle cameras\n"
                          "Free camera\n"
                          "Select\n"
                          "Select parent\n"
                          "Center camera\n"
                          "Tracking camera\n"
                          "Zoom\n"
                          "View Orbit\n"
                          "View Pan\n"
                          "Object Rotate\n"
                          "Object Translate\n"
                          "Help\n"
                          "Info\n"
                          "Profiler\n"
                          "Sensors\n"
                          "Full screen\n"
                          "Show UI shortcuts\n"
                          "Expand/collapse all";

// number of lines in the Constraint ("Counts") and Cost ("Convergence") figures
static constexpr int kConstraintNum = 5;
static constexpr int kCostNum       = 3;

// init profiler figures
void InitializeProfiler(mujoco_ros::Viewer *viewer)
{
	// set figures to default
	mjv_defaultFigure(&viewer->figconstraint);
	mjv_defaultFigure(&viewer->figcost);
	mjv_defaultFigure(&viewer->figtimer);
	mjv_defaultFigure(&viewer->figsize);

	// titles
	mju::strcpy_arr(viewer->figconstraint.title, "Counts");
	mju::strcpy_arr(viewer->figcost.title, "Convergence (log 10)");
	mju::strcpy_arr(viewer->figsize.title, "Dimensions");
	mju::strcpy_arr(viewer->figtimer.title, "CPU time (msec)");

	// x-labels
	mju::strcpy_arr(viewer->figconstraint.xlabel, "Solver iteration");
	mju::strcpy_arr(viewer->figcost.xlabel, "Solver iteration");
	mju::strcpy_arr(viewer->figsize.xlabel, "Video frame");
	mju::strcpy_arr(viewer->figtimer.xlabel, "Video frame");

	// y-tick number formats
	mju::strcpy_arr(viewer->figconstraint.yformat, "%.0f");
	mju::strcpy_arr(viewer->figcost.yformat, "%.1f");
	mju::strcpy_arr(viewer->figsize.yformat, "%.0f");
	mju::strcpy_arr(viewer->figtimer.yformat, "%.2f");

	// colors
	viewer->figconstraint.figurergba[0] = 0.1f;
	viewer->figcost.figurergba[2]       = 0.2f;
	viewer->figsize.figurergba[0]       = 0.1f;
	viewer->figtimer.figurergba[2]      = 0.2f;
	viewer->figconstraint.figurergba[3] = 0.5f;
	viewer->figcost.figurergba[3]       = 0.5f;
	viewer->figsize.figurergba[3]       = 0.5f;
	viewer->figtimer.figurergba[3]      = 0.5f;

	// repeat line colors for constraint and cost figures
	mjvFigure *fig = &viewer->figcost;
	for (int i = kCostNum; i < mjMAXLINE; i++) {
		fig->linergb[i][0] = fig->linergb[i - kCostNum][0];
		fig->linergb[i][1] = fig->linergb[i - kCostNum][1];
		fig->linergb[i][2] = fig->linergb[i - kCostNum][2];
	}
	fig = &viewer->figconstraint;
	for (int i = kConstraintNum; i < mjMAXLINE; i++) {
		fig->linergb[i][0] = fig->linergb[i - kConstraintNum][0];
		fig->linergb[i][1] = fig->linergb[i - kConstraintNum][1];
		fig->linergb[i][2] = fig->linergb[i - kConstraintNum][2];
	}

	// legends
	mju::strcpy_arr(viewer->figconstraint.linename[0], "total");
	mju::strcpy_arr(viewer->figconstraint.linename[1], "active");
	mju::strcpy_arr(viewer->figconstraint.linename[2], "changed");
	mju::strcpy_arr(viewer->figconstraint.linename[3], "evals");
	mju::strcpy_arr(viewer->figconstraint.linename[4], "updates");
	mju::strcpy_arr(viewer->figcost.linename[0], "improvement");
	mju::strcpy_arr(viewer->figcost.linename[1], "gradient");
	mju::strcpy_arr(viewer->figcost.linename[2], "lineslope");
	mju::strcpy_arr(viewer->figsize.linename[0], "dof");
	mju::strcpy_arr(viewer->figsize.linename[1], "body");
	mju::strcpy_arr(viewer->figsize.linename[2], "constraint");
	mju::strcpy_arr(viewer->figsize.linename[3], "sqrt(nnz)");
	mju::strcpy_arr(viewer->figsize.linename[4], "contact");
	mju::strcpy_arr(viewer->figsize.linename[5], "iteration");
	mju::strcpy_arr(viewer->figtimer.linename[0], "total");
	mju::strcpy_arr(viewer->figtimer.linename[1], "collision");
	mju::strcpy_arr(viewer->figtimer.linename[2], "prepare");
	mju::strcpy_arr(viewer->figtimer.linename[3], "solve");
	mju::strcpy_arr(viewer->figtimer.linename[4], "other");

	// grid sizes
	viewer->figconstraint.gridsize[0] = 5;
	viewer->figconstraint.gridsize[1] = 5;
	viewer->figcost.gridsize[0]       = 5;
	viewer->figcost.gridsize[1]       = 5;
	viewer->figsize.gridsize[0]       = 3;
	viewer->figsize.gridsize[1]       = 5;
	viewer->figtimer.gridsize[0]      = 3;
	viewer->figtimer.gridsize[1]      = 5;

	// minimum ranges
	viewer->figconstraint.range[0][0] = 0;
	viewer->figconstraint.range[0][1] = 20;
	viewer->figconstraint.range[1][0] = 0;
	viewer->figconstraint.range[1][1] = 80;
	viewer->figcost.range[0][0]       = 0;
	viewer->figcost.range[0][1]       = 20;
	viewer->figcost.range[1][0]       = -15;
	viewer->figcost.range[1][1]       = 5;
	viewer->figsize.range[0][0]       = -200;
	viewer->figsize.range[0][1]       = 0;
	viewer->figsize.range[1][0]       = 0;
	viewer->figsize.range[1][1]       = 100;
	viewer->figtimer.range[0][0]      = -200;
	viewer->figtimer.range[0][1]      = 0;
	viewer->figtimer.range[1][0]      = 0;
	viewer->figtimer.range[1][1]      = 0.4f;

	// init x axis on history figures (do not show yet)
	for (int n = 0; n < 6; n++) {
		for (int i = 0; i < mjMAXLINEPNT; i++) {
			viewer->figtimer.linedata[n][2 * i] = -i;
			viewer->figsize.linedata[n][2 * i]  = -i;
		}
	}
}

// update profiler figures
void UpdateProfiler(mujoco_ros::Viewer *viewer, const mjModel *m, const mjData *d)
{
	// reset lines in Constraint and Cost figures
	memset(viewer->figconstraint.linepnt, 0, mjMAXLINE * sizeof(int));
	memset(viewer->figcost.linepnt, 0, mjMAXLINE * sizeof(int));

	// number of islands that have diagnostics
	int nisland = mjMIN(d->solver_nisland, mjNISLAND);

	for (int k = 0; k < nisland; k++) {
		// === update Constarint ("Counts") figure

		// number of points to plot, starting line
		int npoints = mjMIN(mjMIN(d->solver_niter[k], mjNSOLVER), mjMAXLINEPNT);
		int start   = kConstraintNum * k;

		viewer->figconstraint.linepnt[start + 0] = npoints;
		for (int i = 1; i < kConstraintNum; i++) {
			viewer->figconstraint.linepnt[start + i] = npoints;
		}
		if (m->opt.solver == mjSOL_PGS) {
			viewer->figconstraint.linepnt[start + 3] = 0;
			viewer->figconstraint.linepnt[start + 4] = 0;
		}
		if (m->opt.solver == mjSOL_CG) {
			viewer->figconstraint.linepnt[start + 4] = 0;
		}
		for (int i = 0; i < npoints; i++) {
			// x
			viewer->figconstraint.linedata[start + 0][2 * i] = static_cast<float>(i);
			viewer->figconstraint.linedata[start + 1][2 * i] = static_cast<float>(i);
			viewer->figconstraint.linedata[start + 2][2 * i] = static_cast<float>(i);
			viewer->figconstraint.linedata[start + 3][2 * i] = static_cast<float>(i);
			viewer->figconstraint.linedata[start + 4][2 * i] = static_cast<float>(i);

			// y
			int nefc                                             = nisland == 1 ? d->nefc : d->island_efcnum[k];
			viewer->figconstraint.linedata[start + 0][2 * i + 1] = static_cast<float>(nefc);
			const mjSolverStat *stat                             = d->solver + k * mjNSOLVER + i;
			viewer->figconstraint.linedata[start + 1][2 * i + 1] = static_cast<float>(stat->nactive);
			viewer->figconstraint.linedata[start + 2][2 * i + 1] = static_cast<float>(stat->nchange);
			viewer->figconstraint.linedata[start + 3][2 * i + 1] = static_cast<float>(stat->neval);
			viewer->figconstraint.linedata[start + 4][2 * i + 1] = static_cast<float>(stat->nupdate);
		}

		// update cost figure
		start                              = kCostNum * k;
		viewer->figcost.linepnt[start + 0] = npoints;
		for (int i = 1; i < kCostNum; i++) {
			viewer->figcost.linepnt[start + i] = npoints;
		}
		if (m->opt.solver == mjSOL_PGS) {
			viewer->figcost.linepnt[start + 1] = 0;
			viewer->figcost.linepnt[start + 2] = 0;
		}

		for (size_t i = 0; i < viewer->figcost.linepnt[0]; i++) {
			// x
			viewer->figcost.linedata[start + 0][2 * i] = static_cast<float>(i);
			viewer->figcost.linedata[start + 1][2 * i] = static_cast<float>(i);
			viewer->figcost.linedata[start + 2][2 * i] = static_cast<float>(i);

			// y
			const mjSolverStat *stat                       = d->solver + k * mjNSOLVER + i;
			viewer->figcost.linedata[start + 0][2 * i + 1] = mju_log10(mju_max(mjMINVAL, stat->improvement));
			viewer->figcost.linedata[start + 1][2 * i + 1] = mju_log10(mju_max(mjMINVAL, stat->gradient));
			viewer->figcost.linedata[start + 2][2 * i + 1] = mju_log10(mju_max(mjMINVAL, stat->lineslope));
		}
	}

	// get timers: total, collision, prepare, solve, other
	mjtNum total = d->timer[mjTIMER_STEP].duration;
	int number   = d->timer[mjTIMER_STEP].number;
	if (!number) {
		total  = d->timer[mjTIMER_FORWARD].duration;
		number = d->timer[mjTIMER_FORWARD].number;
	}
	if (number) { // skip update if no measurements
		float tdata[5] = { static_cast<float>(total / number),
			                static_cast<float>(d->timer[mjTIMER_POS_COLLISION].duration / number),
			                static_cast<float>(d->timer[mjTIMER_POS_MAKE].duration / number) +
			                    static_cast<float>(d->timer[mjTIMER_POS_PROJECT].duration / number),
			                static_cast<float>(d->timer[mjTIMER_CONSTRAINT].duration / number), 0 };
		tdata[4]       = tdata[0] - tdata[1] - tdata[2] - tdata[3];

		// update figtimer
		int pnt = mjMIN(201, viewer->figtimer.linepnt[0] + 1);
		for (int n = 0; n < 5; n++) {
			// shift data
			for (int i = pnt - 1; i > 0; i--) {
				viewer->figtimer.linedata[n][2 * i + 1] = viewer->figtimer.linedata[n][2 * i - 1];
			}

			// assign new
			viewer->figtimer.linepnt[n]     = pnt;
			viewer->figtimer.linedata[n][1] = tdata[n];
		}
	}

	// get total number of iterations and nonzeros
	mjtNum sqrt_nnz  = 0;
	int solver_niter = 0;
	for (int island = 0; island < nisland; island++) {
		sqrt_nnz += mju_sqrt(d->solver_nnz[island]);
		solver_niter += d->solver_niter[island];
	}

	// get sizes: nv, nbody, nefc, sqrt(nnz), ncont, iter
	float sdata[6] = { static_cast<float>(m->nv),    static_cast<float>(m->nbody), static_cast<float>(d->nefc),
		                static_cast<float>(sqrt_nnz), static_cast<float>(d->ncon),  static_cast<float>(solver_niter) };

	// update figsize
	int pnt = mjMIN(201, viewer->figsize.linepnt[0] + 1);
	for (int n = 0; n < 6; n++) {
		// shift data
		for (int i = pnt - 1; i > 0; i--) {
			viewer->figsize.linedata[n][2 * i + 1] = viewer->figsize.linedata[n][2 * i - 1];
		}

		// assign new
		viewer->figsize.linepnt[n]     = pnt;
		viewer->figsize.linedata[n][1] = sdata[n];
	}
}

// show profiler figures
void ShowProfiler(mujoco_ros::Viewer *viewer, mjrRect rect)
{
	mjrRect viewport = { rect.left + rect.width - rect.width / 4, rect.bottom, rect.width / 4, rect.height / 4 };
	mjr_figure(viewport, &viewer->figtimer, &viewer->platform_ui->mjr_context());
	viewport.bottom += rect.height / 4;
	mjr_figure(viewport, &viewer->figsize, &viewer->platform_ui->mjr_context());
	viewport.bottom += rect.height / 4;
	mjr_figure(viewport, &viewer->figcost, &viewer->platform_ui->mjr_context());
	viewport.bottom += rect.height / 4;
	mjr_figure(viewport, &viewer->figconstraint, &viewer->platform_ui->mjr_context());
}

// init sensor figure
void InitializeSensor(mujoco_ros::Viewer *viewer)
{
	mjvFigure &figsensor = viewer->figsensor;

	// set figure to default
	mjv_defaultFigure(&figsensor);
	figsensor.figurergba[3] = 0.5f;

	// set flags
	figsensor.flg_extend    = 1;
	figsensor.flg_barplot   = 1;
	figsensor.flg_symmetric = 1;

	// title
	mju::strcpy_arr(figsensor.title, "Sensor data");

	// y-tick nubmer format
	mju::strcpy_arr(figsensor.yformat, "%.1f");

	// grid size
	figsensor.gridsize[0] = 2;
	figsensor.gridsize[1] = 3;

	// minimum range
	figsensor.range[0][0] = 0;
	figsensor.range[0][1] = 0;
	figsensor.range[1][0] = -1;
	figsensor.range[1][1] = 1;
}

// update sensor figure
void UpdateSensor(mujoco_ros::Viewer *viewer, const mjModel *m, const mjData *d)
{
	mjvFigure &figsensor        = viewer->figsensor;
	static const size_t maxline = 10;

	// clear linepnt
	for (size_t i = 0; i < maxline; i++) {
		figsensor.linepnt[i] = 0;
	}

	// start with line 0
	size_t lineid = 0;

	// loop over sensors
	for (size_t n = 0; n < m->nsensor; n++) {
		// go to next line if type is different
		if (n > 0 && m->sensor_type[n] != m->sensor_type[n - 1u]) {
			lineid = mjMIN(lineid + 1u, maxline - 1u);
		}

		// get info about this sensor
		mjtNum cutoff = (m->sensor_cutoff[n] > 0 ? m->sensor_cutoff[n] : 1);
		size_t adr    = util::as_unsigned(m->sensor_adr[n]);
		size_t dim    = util::as_unsigned(m->sensor_dim[n]);

		// data pointer in line
		size_t p = util::as_unsigned(figsensor.linepnt[lineid]);

		// fill in data for this sensor
		for (size_t i = 0; i < dim; i++) {
			// check size
			if ((p + 2 * i) >= mjMAXLINEPNT / 2) {
				break;
			}

			// x
			figsensor.linedata[lineid][2 * p + 4 * i]     = adr + i;
			figsensor.linedata[lineid][2 * p + 4 * i + 2] = adr + i;

			// y
			figsensor.linedata[lineid][2 * p + 4 * i + 1] = 0;
			figsensor.linedata[lineid][2 * p + 4 * i + 3] = d->sensordata[adr + i] / cutoff;
		}

		// update linepnt
		figsensor.linepnt[lineid] = mjMIN(mjMAXLINEPNT - 1, figsensor.linepnt[lineid] + 2 * dim);
	}
}

// show sensor figure
void ShowSensor(mujoco_ros::Viewer *viewer, mjrRect rect)
{
	// constant width with and without profiler
	int width = viewer->profiler ? rect.width / 3 : rect.width / 4;

	// render figure on the right
	mjrRect viewport = { rect.left + rect.width - width, rect.bottom, width, rect.height / 3 };
	mjr_figure(viewport, &viewer->figsensor, &viewer->platform_ui->mjr_context());
}

// load state from history buffer
static void LoadScrubState(mujoco_ros::Viewer * /*viewer*/)
{
	ROS_WARN("Stepping backwards currently not supported!");
	// // get index into circular buffer
	// int i = (viewer->scrub_index + viewer->history_cursor_) % viewer->nhistory_;
	// i     = (i + viewer->nhistory_) % viewer->nhistory_;

	// // load state
	// mjtNum *state = &viewer->history_[i * viewer->state_size_];
	// mj_setState(viewer->m_.get(), viewer->d_.get(), state, mjSTATE_INTEGRATION);

	// // call forward dynamics
	// mj_forward(viewer->m_.get(), viewer->d_.get());
}

// update an entire section of ui0
static void mjui0_update_section(mujoco_ros::Viewer *viewer, int section)
{
	mjui_update(section, -1, &viewer->ui0, &viewer->uistate, &viewer->platform_ui->mjr_context());
}

// prepare info text
void UpdateInfoText(mujoco_ros::Viewer *viewer, const mjModel *m, const mjData *d,
                    char (&title)[mujoco_ros::Viewer::kMaxFilenameLength],
                    char (&content)[mujoco_ros::Viewer::kMaxFilenameLength])
{
	char tmp[20];

	// number of islands with statistics
	int nisland = mjMIN(d->solver_nisland, mjNISLAND);

	// compute solver error (maximum over islands)
	mjtNum solerr = 0;
	for (int i = 0; i < nisland; i++) {
		mjtNum solerr_i = 0;
		if (d->solver_niter[i]) {
			int ind                  = mjMIN(d->solver_niter[i], mjNSOLVER) - 1;
			const mjSolverStat *stat = d->solver + i * mjNSOLVER + ind;
			solerr_i                 = mju_min(stat->improvement, stat->gradient);
			if (solerr_i == 0) {
				solerr_i = mju_max(stat->improvement, stat->gradient);
			}
		}
		solerr = mju_max(solerr, solerr_i);
	}
	solerr = mju_log10(mju_max(mjMINVAL, solerr));

	// format FPS text
	char fps[10];
	if (viewer->fps_ < 1) {
		mju::sprintf_arr(fps, "%0.1f ", viewer->fps_);
	} else {
		mju::sprintf_arr(fps, "%.0f ", viewer->fps_);
	}

	int solver_niter = 0;
	for (int i = 0; i < nisland; i++) {
		solver_niter += d->solver_niter[i];
	}

	// prepare info text
	mju::strcpy_arr(title, "Time\nSize\nCPU\nSolver   \nFPS\nMemory");
	mju::sprintf_arr(content, "%-9.3f\n%d  (%d con)\n%.3f\n%.1f  (%d it)\n%s\n%.2g of %s", d->time, d->nefc, d->ncon,
	                 viewer->run ? d->timer[mjTIMER_STEP].duration / mjMAX(1, d->timer[mjTIMER_STEP].number) :
	                               d->timer[mjTIMER_FORWARD].duration / mjMAX(1, d->timer[mjTIMER_FORWARD].number),
	                 solerr, solver_niter, fps, d->maxuse_arena / static_cast<double>(util::as_unsigned(d->narena)),
	                 mju_writeNumBytes(util::as_unsigned(d->narena)));

	// add Energy if enabled
	{
		if (mjENABLED(mjENBL_ENERGY)) {
			mju::sprintf_arr(tmp, "\n%.3f", d->energy[0] + d->energy[1]);
			mju::strcat_arr(content, tmp);
			mju::strcat_arr(title, "\nEnergy");
		}

		// add FwdInv if enabled
		if (mjENABLED(mjENBL_FWDINV)) {
			mju::sprintf_arr(tmp, "\n%.1f %.1f", mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[0])),
			                 mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[1])));
			mju::strcat_arr(content, tmp);
			mju::strcat_arr(title, "\nFwdInv");
		}

		// add islands if enabled
		if (mjENABLED(mjENBL_ISLAND)) {
			mju::sprintf_arr(tmp, "\n%d", d->nisland);
			mju::strcat_arr(content, tmp);
			mju::strcat_arr(title, "\nIslands");
		}
	}
}

// sprintf forwarding, to avoid compiler warning in x-macro
void PrintField(char (&str)[mjMAXUINAME], void *ptr)
{
	mju::sprintf_arr(str, "%g", *static_cast<mjtNum *>(ptr));
}

// update watch
void UpdateWatch(mujoco_ros::Viewer *viewer, const mjModel *m, const mjData *d)
{
	// clear
	viewer->ui0.sect[SECT_WATCH].item[2].multi.nelem = 1;
	mju::strcpy_arr(viewer->ui0.sect[SECT_WATCH].item[2].multi.name[0], "invalid field");

	// prepare symbols needed by xmacro
	MJDATA_POINTERS_PREAMBLE(m);

// find specified field in mjData arrays, update value
#define X(TYPE, NAME, NR, NC)                                                                     \
	if (!mju::strcmp_arr(#NAME, viewer->field) && !mju::strcmp_arr(#TYPE, "mjtNum")) {             \
		if (viewer->index >= 0 && viewer->index < m->NR * NC) {                                     \
			PrintField(viewer->ui0.sect[SECT_WATCH].item[2].multi.name[0], d->NAME + viewer->index); \
		} else {                                                                                    \
			mju::strcpy_arr(viewer->ui0.sect[SECT_WATCH].item[2].multi.name[0], "invalid index");    \
		}                                                                                           \
		return;                                                                                     \
	}

	MJDATA_POINTERS
#undef X
}

// make physics section of UI
void MakePhysicsSection(mujoco_ros::Viewer *viewer)
{
	mjOption *opt                = viewer->is_passive_ ? &viewer->scnstate_.model.opt : &viewer->m_->opt;
	mjuiDef defPhysics[]         = { { mjITEM_SECTION, "Physics", mjPRESERVE, nullptr, "AP" },
		                              { mjITEM_SELECT, "Integrator", 2, &(opt->integrator),
		                                "Euler\nRK4\nimplicit\nimplicitfast" },
		                              { mjITEM_SELECT, "Cone", 2, &(opt->cone), "Pyramidal\nElliptic" },
		                              { mjITEM_SELECT, "Jacobian", 2, &(opt->jacobian), "Dense\nSparse\nAuto" },
		                              { mjITEM_SELECT, "Solver", 2, &(opt->solver), "PGS\nCG\nNewton" },
		                              { mjITEM_SEPARATOR, "Algorithmic Parameters", mjPRESERVE },
		                              { mjITEM_EDITNUM, "Timestep", 2, &(opt->timestep), "1 0 1" },
		                              { mjITEM_EDITINT, "Iterations", 2, &(opt->iterations), "1 0 1000" },
		                              { mjITEM_EDITNUM, "Tolerance", 2, &(opt->tolerance), "1 0 1" },
		                              { mjITEM_EDITINT, "LS Iter", 2, &(opt->ls_iterations), "1 0 100" },
		                              { mjITEM_EDITNUM, "LS Tol", 2, &(opt->ls_tolerance), "1 0 0.1" },
		                              { mjITEM_EDITINT, "Noslip Iter", 2, &(opt->noslip_iterations), "1 0 1000" },
		                              { mjITEM_EDITNUM, "Noslip Tol", 2, &(opt->noslip_tolerance), "1 0 1" },
		                              { mjITEM_EDITINT, "MPR Iter", 2, &(opt->mpr_iterations), "1 0 1000" },
		                              { mjITEM_EDITNUM, "MPR Tol", 2, &(opt->mpr_tolerance), "1 0 1" },
		                              { mjITEM_EDITNUM, "API Rate", 2, &(opt->apirate), "1 0 1000" },
		                              { mjITEM_EDITINT, "SDF Iter", 2, &(opt->sdf_iterations), "1 1 20" },
		                              { mjITEM_EDITINT, "SDF Init", 2, &(opt->sdf_initpoints), "1 1 100" },
		                              { mjITEM_SEPARATOR, "Physical Parameters", mjPRESERVE },
		                              { mjITEM_EDITNUM, "Gravity", 2, opt->gravity, "3" },
		                              { mjITEM_EDITNUM, "Wind", 2, opt->wind, "3" },
		                              { mjITEM_EDITNUM, "Magnetic", 2, opt->magnetic, "3" },
		                              { mjITEM_EDITNUM, "Density", 2, &(opt->density), "1" },
		                              { mjITEM_EDITNUM, "Viscosity", 2, &(opt->viscosity), "1" },
		                              { mjITEM_EDITNUM, "Imp Ratio", 2, &(opt->impratio), "1" },
		                              { mjITEM_SEPARATOR, "Disable Flags", mjPRESERVE },
		                              { mjITEM_END } };
	mjuiDef defEnableFlags[]     = { { mjITEM_SEPARATOR, "Enable Flags", mjPRESERVE }, { mjITEM_END } };
	mjuiDef defOverride[]        = { { mjITEM_SEPARATOR, "Contact Override", mjPRESERVE },
		                              { mjITEM_EDITNUM, "Margin", 2, &(opt->o_margin), "1" },
		                              { mjITEM_EDITNUM, "Sol Imp", 2, &(opt->o_solimp), "5" },
		                              { mjITEM_EDITNUM, "Sol Ref", 2, &(opt->o_solref), "2" },
		                              { mjITEM_EDITNUM, "Friction", 2, &(opt->o_friction), "5" },
		                              { mjITEM_END } };
	mjuiDef defDisableActuator[] = { { mjITEM_SEPARATOR, "Actuator Group Enable", 1 },
		                              { mjITEM_CHECKBYTE, "Act Group 0", 2, viewer->enableactuator + 0, "" },
		                              { mjITEM_CHECKBYTE, "Act Group 1", 2, viewer->enableactuator + 1, "" },
		                              { mjITEM_CHECKBYTE, "Act Group 2", 2, viewer->enableactuator + 2, "" },
		                              { mjITEM_CHECKBYTE, "Act Group 3", 2, viewer->enableactuator + 3, "" },
		                              { mjITEM_CHECKBYTE, "Act Group 4", 2, viewer->enableactuator + 4, "" },
		                              { mjITEM_CHECKBYTE, "Act Group 5", 2, viewer->enableactuator + 5, "" },
		                              { mjITEM_END } };

	// add physics
	mjui_add(&viewer->ui0, defPhysics);

	// add flags programmatically
	mjuiDef defFlag[] = { { mjITEM_CHECKINT, "", 2, nullptr, "" }, { mjITEM_END } };
	for (int i = 0; i < mjNDISABLE; i++) {
		mju::strcpy_arr(defFlag[0].name, mjDISABLESTRING[i]);
		defFlag[0].pdata = viewer->disable + i;
		mjui_add(&viewer->ui0, defFlag);
	}
	mjui_add(&viewer->ui0, defEnableFlags);
	for (int i = 0; i < mjNENABLE; i++) {
		mju::strcpy_arr(defFlag[0].name, mjENABLESTRING[i]);
		defFlag[0].pdata = viewer->enable + i;
		mjui_add(&viewer->ui0, defFlag);
	}

	// add contact override
	mjui_add(&viewer->ui0, defOverride);

	// add actuator enable/disable
	mjui_add(&viewer->ui0, defDisableActuator);

	// make some subsections closed by default
	for (int i = 0; i < viewer->ui0.sect[SECT_PHYSICS].nitem; i++) {
		mjuiItem *item = viewer->ui0.sect[SECT_PHYSICS].item + i;

		// close less useful subsections
		if (item->type == mjITEM_SEPARATOR) {
			if (mju::strcmp_arr(item->name, "Actuator Group Enable") || mju::strcmp_arr(item->name, "Contact Override") ||
			    mju::strcmp_arr(item->name, "Physical Parameters")) {
				item->state = mjSEPCLOSED + 1;
			}
		}
	}
}

// make rendering section of UI
void MakeRenderingSection(mujoco_ros::Viewer *viewer, const mjModel *m)
{
	mjuiDef defRendering[] = { { mjITEM_SECTION, "Rendering", mjPRESERVE, nullptr, "AR" },
		                        { mjITEM_SELECT, "Camera", 2, &(viewer->camera), "Free\nTracking" },
		                        { mjITEM_SELECT, "Label", 2, &(viewer->opt.label),
		                          "None\nBody\nJoint\nGeom\nSite\nCamera\nLight\nTendon\n"
		                          "Actuator\nConstraint\nFlex\nSkin\nSelection\nSel Pnt\nContact\nForce\nIsland" },
		                        { mjITEM_SELECT, "Frame", 2, &(viewer->opt.frame),
		                          "None\nBody\nGeom\nSite\nCamera\nLight\nContact\nWorld" },
		                        { mjITEM_BUTTON, "Copy camera", 2, nullptr, "" },
		                        { mjITEM_SEPARATOR, "Model Elements", 1 },
		                        { mjITEM_END } };
	mjuiDef defOpenGL[]    = { { mjITEM_SEPARATOR, "OpenGL Effects", 1 }, { mjITEM_END } };

	// add model cameras, up to UI limit
	for (int i = 0; i < mjMIN(m->ncam, mjMAXUIMULTI - 2); i++) {
		// prepare name
		char camname[mjMAXUINAME] = "\n";
		if (m->names[m->name_camadr[i]]) {
			mju::strcat_arr(camname, m->names + m->name_camadr[i]);
		} else {
			mju::sprintf_arr(camname, "\nCamera %d", i);
		}

		// check string length
		if (mju::strlen_arr(camname) + mju::strlen_arr(defRendering[1].other) >= mjMAXUITEXT - 1) {
			break;
		}

		// add camera
		mju::strcat_arr(defRendering[1].other, camname);
	}

	// add rendering standard
	mjui_add(&viewer->ui0, defRendering);

	// add flags programmatically
	mjuiDef defFlag[] = { { mjITEM_CHECKBYTE, "", 2, nullptr, "" }, { mjITEM_END } };
	for (size_t i = 0; i < mjNVISFLAG; i++) {
		// set name
		mju::strcpy_arr(defFlag[0].name, mjVISSTRING[i][0]);

		// set shortcut and data
		if (mjVISSTRING[i][2][0]) {
			mju::sprintf_arr(defFlag[0].other, " %s", mjVISSTRING[i][2]);
		} else {
			defFlag[0].other[0] = 0;
		}
		defFlag[0].pdata = viewer->opt.flags + i;
		mjui_add(&viewer->ui0, defFlag);
	}

	// create tree slider
	mjuiDef defTree[] = { { mjITEM_SLIDERINT, "Tree depth", 2, &viewer->opt.bvh_depth, "0 20" },
		                   { mjITEM_SLIDERINT, "Flex layer", 2, &viewer->opt.flex_layer, "0 10" },
		                   { mjITEM_END } };
	mjui_add(&viewer->ui0, defTree);

	// add rendering flags
	mjui_add(&viewer->ui0, defOpenGL);
	for (int i = 0; i < mjNRNDFLAG; i++) {
		// set name
		mju::strcpy_arr(defFlag[0].name, mjRNDSTRING[i][0]);

		// set shortcut and data
		if (mjRNDSTRING[i][2][0]) {
			mju::sprintf_arr(defFlag[0].other, " %s", mjRNDSTRING[i][2]);
		} else {
			defFlag[0].other[0] = 0;
		}
		defFlag[0].pdata = viewer->scn.flags + i;
		mjui_add(&viewer->ui0, defFlag);
	}
}

// make visualization section of UI
void MakeVisualizationSection(mujoco_ros::Viewer *viewer, const mjModel * /*m*/)
{
	mjStatistic *stat = viewer->is_passive_ ? &viewer->scnstate_.model.stat : &viewer->m_->stat;
	mjVisual *vis     = viewer->is_passive_ ? &viewer->scnstate_.model.vis : &viewer->m_->vis;

	mjuiDef defVisualization[] = { { mjITEM_SECTION, "Visualization", mjPRESERVE, nullptr, "AV" },
		                            { mjITEM_SEPARATOR, "Headlight", 1 },
		                            { mjITEM_RADIO, "Active", 5, &(vis->headlight.active), "Off\nOn" },
		                            { mjITEM_EDITFLOAT, "Ambient", 2, &(vis->headlight.ambient), "3" },
		                            { mjITEM_EDITFLOAT, "Diffuse", 2, &(vis->headlight.diffuse), "3" },
		                            { mjITEM_EDITFLOAT, "Specular", 2, &(vis->headlight.specular), "3" },
		                            { mjITEM_SEPARATOR, "Free Camera", 1 },
		                            { mjITEM_RADIO, "Orthographic", 2, &(vis->global.orthographic), "No\nYes" },
		                            { mjITEM_EDITFLOAT, "Field of view", 2, &(vis->global.fovy), "1" },
		                            { mjITEM_EDITNUM, "Center", 2, &(stat->center), "3" },
		                            { mjITEM_EDITFLOAT, "Azimuth", 2, &(vis->global.azimuth), "1" },
		                            { mjITEM_EDITFLOAT, "Elevation", 2, &(vis->global.elevation), "1" },
		                            { mjITEM_BUTTON, "Align", 2, nullptr, "CA" },
		                            { mjITEM_SEPARATOR, "Global", 1 },
		                            { mjITEM_EDITNUM, "Extent", 2, &(stat->extent), "1" },
		                            { mjITEM_RADIO, "Inertia", 5, &(vis->global.ellipsoidinertia), "Box\nEllipsoid" },
		                            { mjITEM_RADIO, "BVH active", 5, &(vis->global.bvactive), "False\nTrue" },
		                            { mjITEM_SEPARATOR, "Map", 1 },
		                            { mjITEM_EDITFLOAT, "Stiffness", 2, &(vis->map.stiffness), "1" },
		                            { mjITEM_EDITFLOAT, "Rot stiffness", 2, &(vis->map.stiffnessrot), "1" },
		                            { mjITEM_EDITFLOAT, "Force", 2, &(vis->map.force), "1" },
		                            { mjITEM_EDITFLOAT, "Torque", 2, &(vis->map.torque), "1" },
		                            { mjITEM_EDITFLOAT, "Alpha", 2, &(vis->map.alpha), "1" },
		                            { mjITEM_EDITFLOAT, "Fog start", 2, &(vis->map.fogstart), "1" },
		                            { mjITEM_EDITFLOAT, "Fog end", 2, &(vis->map.fogend), "1" },
		                            { mjITEM_EDITFLOAT, "Z near", 2, &(vis->map.znear), "1" },
		                            { mjITEM_EDITFLOAT, "Z far", 2, &(vis->map.zfar), "1" },
		                            { mjITEM_EDITFLOAT, "Haze", 2, &(vis->map.haze), "1" },
		                            { mjITEM_EDITFLOAT, "Shadow clip", 2, &(vis->map.shadowclip), "1" },
		                            { mjITEM_EDITFLOAT, "Shadow scale", 2, &(vis->map.shadowscale), "1" },
		                            { mjITEM_SEPARATOR, "Scale", mjPRESERVE },
		                            { mjITEM_EDITNUM, "All (meansize)", 2, &(stat->meansize), "1" },
		                            { mjITEM_EDITFLOAT, "Force width", 2, &(vis->scale.forcewidth), "1" },
		                            { mjITEM_EDITFLOAT, "Contact width", 2, &(vis->scale.contactwidth), "1" },
		                            { mjITEM_EDITFLOAT, "Contact height", 2, &(vis->scale.contactheight), "1" },
		                            { mjITEM_EDITFLOAT, "Connect", 2, &(vis->scale.connect), "1" },
		                            { mjITEM_EDITFLOAT, "Com", 2, &(vis->scale.com), "1" },
		                            { mjITEM_EDITFLOAT, "Camera", 2, &(vis->scale.camera), "1" },
		                            { mjITEM_EDITFLOAT, "Light", 2, &(vis->scale.light), "1" },
		                            { mjITEM_EDITFLOAT, "Select point", 2, &(vis->scale.selectpoint), "1" },
		                            { mjITEM_EDITFLOAT, "Joint length", 2, &(vis->scale.jointlength), "1" },
		                            { mjITEM_EDITFLOAT, "Joint width", 2, &(vis->scale.jointwidth), "1" },
		                            { mjITEM_EDITFLOAT, "Actuator length", 2, &(vis->scale.actuatorlength), "1" },
		                            { mjITEM_EDITFLOAT, "Actuator width", 2, &(vis->scale.actuatorwidth), "1" },
		                            { mjITEM_EDITFLOAT, "Frame length", 2, &(vis->scale.framelength), "1" },
		                            { mjITEM_EDITFLOAT, "Frame width", 2, &(vis->scale.framewidth), "1" },
		                            { mjITEM_EDITFLOAT, "Constraint", 2, &(vis->scale.constraint), "1" },
		                            { mjITEM_EDITFLOAT, "Slider-crank", 2, &(vis->scale.slidercrank), "1" },
		                            { mjITEM_SEPARATOR, "RGBA", mjPRESERVE },
		                            { mjITEM_EDITFLOAT, "fog", 2, &(vis->rgba.fog), "4" },
		                            { mjITEM_EDITFLOAT, "haze", 2, &(vis->rgba.haze), "4" },
		                            { mjITEM_EDITFLOAT, "force", 2, &(vis->rgba.force), "4" },
		                            { mjITEM_EDITFLOAT, "inertia", 2, &(vis->rgba.inertia), "4" },
		                            { mjITEM_EDITFLOAT, "joint", 2, &(vis->rgba.joint), "4" },
		                            { mjITEM_EDITFLOAT, "actuator", 2, &(vis->rgba.actuator), "4" },
		                            { mjITEM_EDITFLOAT, "actnegative", 2, &(vis->rgba.actuatornegative), "4" },
		                            { mjITEM_EDITFLOAT, "actpositive", 2, &(vis->rgba.actuatorpositive), "4" },
		                            { mjITEM_EDITFLOAT, "com", 2, &(vis->rgba.com), "4" },
		                            { mjITEM_EDITFLOAT, "camera", 2, &(vis->rgba.camera), "4" },
		                            { mjITEM_EDITFLOAT, "light", 2, &(vis->rgba.light), "4" },
		                            { mjITEM_EDITFLOAT, "selectpoint", 2, &(vis->rgba.selectpoint), "4" },
		                            { mjITEM_EDITFLOAT, "connect", 2, &(vis->rgba.connect), "4" },
		                            { mjITEM_EDITFLOAT, "contactpoint", 2, &(vis->rgba.contactpoint), "4" },
		                            { mjITEM_EDITFLOAT, "contactforce", 2, &(vis->rgba.contactforce), "4" },
		                            { mjITEM_EDITFLOAT, "contactfriction", 2, &(vis->rgba.contactfriction), "4" },
		                            { mjITEM_EDITFLOAT, "contacttorque", 2, &(vis->rgba.contacttorque), "4" },
		                            { mjITEM_EDITFLOAT, "contactgap", 2, &(vis->rgba.contactgap), "4" },
		                            { mjITEM_EDITFLOAT, "rangefinder", 2, &(vis->rgba.rangefinder), "4" },
		                            { mjITEM_EDITFLOAT, "constraint", 2, &(vis->rgba.constraint), "4" },
		                            { mjITEM_EDITFLOAT, "slidercrank", 2, &(vis->rgba.slidercrank), "4" },
		                            { mjITEM_EDITFLOAT, "crankbroken", 2, &(vis->rgba.crankbroken), "4" },
		                            { mjITEM_EDITFLOAT, "frustum", 2, &(vis->rgba.frustum), "4" },
		                            { mjITEM_EDITFLOAT, "bv", 2, &(vis->rgba.bvactive), "4" },
		                            { mjITEM_END } };

	// add visualization section
	mjui_add(&viewer->ui0, defVisualization);
}

// make group section of UI
void MakeGroupSection(mujoco_ros::Viewer *viewer)
{
	mjuiDef defGroup[] = { { mjITEM_SECTION, "Group enable", mjPRESERVE, nullptr, "AG" },
		                    { mjITEM_SEPARATOR, "Geom groups", 1 },
		                    { mjITEM_CHECKBYTE, "Geom 0", 2, viewer->opt.geomgroup, " 0" },
		                    { mjITEM_CHECKBYTE, "Geom 1", 2, viewer->opt.geomgroup + 1, " 1" },
		                    { mjITEM_CHECKBYTE, "Geom 2", 2, viewer->opt.geomgroup + 2, " 2" },
		                    { mjITEM_CHECKBYTE, "Geom 3", 2, viewer->opt.geomgroup + 3, " 3" },
		                    { mjITEM_CHECKBYTE, "Geom 4", 2, viewer->opt.geomgroup + 4, " 4" },
		                    { mjITEM_CHECKBYTE, "Geom 5", 2, viewer->opt.geomgroup + 5, " 5" },
		                    { mjITEM_SEPARATOR, "Site groups", 1 },
		                    { mjITEM_CHECKBYTE, "Site 0", 2, viewer->opt.sitegroup, "S0" },
		                    { mjITEM_CHECKBYTE, "Site 1", 2, viewer->opt.sitegroup + 1, "S1" },
		                    { mjITEM_CHECKBYTE, "Site 2", 2, viewer->opt.sitegroup + 2, "S2" },
		                    { mjITEM_CHECKBYTE, "Site 3", 2, viewer->opt.sitegroup + 3, "S3" },
		                    { mjITEM_CHECKBYTE, "Site 4", 2, viewer->opt.sitegroup + 4, "S4" },
		                    { mjITEM_CHECKBYTE, "Site 5", 2, viewer->opt.sitegroup + 5, "S5" },
		                    { mjITEM_SEPARATOR, "Joint groups", 1 },
		                    { mjITEM_CHECKBYTE, "Joint 0", 2, viewer->opt.jointgroup, "" },
		                    { mjITEM_CHECKBYTE, "Joint 1", 2, viewer->opt.jointgroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Joint 2", 2, viewer->opt.jointgroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Joint 3", 2, viewer->opt.jointgroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Joint 4", 2, viewer->opt.jointgroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Joint 5", 2, viewer->opt.jointgroup + 5, "" },
		                    { mjITEM_SEPARATOR, "Tendon groups", 1 },
		                    { mjITEM_CHECKBYTE, "Tendon 0", 2, viewer->opt.tendongroup, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 1", 2, viewer->opt.tendongroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 2", 2, viewer->opt.tendongroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 3", 2, viewer->opt.tendongroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 4", 2, viewer->opt.tendongroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 5", 2, viewer->opt.tendongroup + 5, "" },
		                    { mjITEM_SEPARATOR, "Actuator groups", 1 },
		                    { mjITEM_CHECKBYTE, "Actuator 0", 2, viewer->opt.actuatorgroup, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 1", 2, viewer->opt.actuatorgroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 2", 2, viewer->opt.actuatorgroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 3", 2, viewer->opt.actuatorgroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 4", 2, viewer->opt.actuatorgroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 5", 2, viewer->opt.actuatorgroup + 5, "" },
		                    { mjITEM_SEPARATOR, "Flex groups", 1 },
		                    { mjITEM_CHECKBYTE, "Flex 0", 2, viewer->opt.flexgroup, "" },
		                    { mjITEM_CHECKBYTE, "Flex 1", 2, viewer->opt.flexgroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Flex 2", 2, viewer->opt.flexgroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Flex 3", 2, viewer->opt.flexgroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Flex 4", 2, viewer->opt.flexgroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Flex 5", 2, viewer->opt.flexgroup + 5, "" },
		                    { mjITEM_SEPARATOR, "Skin groups", 1 },
		                    { mjITEM_CHECKBYTE, "Skin 0", 2, viewer->opt.skingroup, "" },
		                    { mjITEM_CHECKBYTE, "Skin 1", 2, viewer->opt.skingroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Skin 2", 2, viewer->opt.skingroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Skin 3", 2, viewer->opt.skingroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Skin 4", 2, viewer->opt.skingroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Skin 5", 2, viewer->opt.skingroup + 5, "" },
		                    { mjITEM_END } };

	// add section
	mjui_add(&viewer->ui0, defGroup);
}

// make joint section of UI
void MakeJointSection(mujoco_ros::Viewer *viewer)
{
	mjuiDef defJoint[]  = { { mjITEM_SECTION, "Joint", mjPRESERVE, nullptr, "AJ" }, { mjITEM_END } };
	mjuiDef defSlider[] = { { mjITEM_SLIDERNUM, "", 2, nullptr, "0 1" }, { mjITEM_END } };

	// add section
	mjui_add(&viewer->ui1, defJoint);
	defSlider[0].state = 4;

	// add scalar joints, exit if UI limit reached
	int itemcnt = 0;
	for (size_t i = 0; i < viewer->jnt_type_.size() && itemcnt < mjMAXUIITEM; i++) {
		if ((viewer->jnt_type_[i] == mjJNT_HINGE || viewer->jnt_type_[i] == mjJNT_SLIDE)) {
			// skip if joint group is disabled
			if (!viewer->opt.jointgroup[mjMAX(0, mjMIN(mjNGROUP - 1, viewer->jnt_group_[i]))]) {
				continue;
			}

			// set data and name
			if (!viewer->is_passive_) {
				defSlider[0].pdata = &viewer->d_->qpos[util::as_unsigned(viewer->m_->jnt_qposadr[i])];
			} else {
				defSlider[0].pdata = &viewer->qpos_[util::as_unsigned(viewer->jnt_qposadr_[i])];
			}
			if (!viewer->jnt_names_[i].empty()) {
				mju::strcpy_arr(defSlider[0].name, viewer->jnt_names_[i].c_str());
			} else {
				mju::sprintf_arr(defSlider[0].name, "joint %d", i);
			}

			// set range
			if (viewer->jnt_range_[i].has_value())
				mju::sprintf_arr(defSlider[0].other, "%.4g %.4g", viewer->jnt_range_[i]->first,
				                 viewer->jnt_range_[i]->second);
			else if (viewer->jnt_type_[i] == mjJNT_SLIDE) {
				mju::strcpy_arr(defSlider[0].other, "-1 1");
			} else {
				mju::strcpy_arr(defSlider[0].other, "-3.1416 3.1416");
			}

			// add and count
			mjui_add(&viewer->ui1, defSlider);
			itemcnt++;
		}
	}
}

// make control section of UI
void MakeControlSection(mujoco_ros::Viewer *viewer)
{
	mjuiDef defControl[] = { { mjITEM_SECTION, "Control", mjPRESERVE, nullptr, "AC" },
		                      { mjITEM_BUTTON, "Clear all", 2 },
		                      { mjITEM_END } };
	mjuiDef defSlider[]  = { { mjITEM_SLIDERNUM, "", 2, nullptr, "0 1" }, { mjITEM_END } };

	// add section
	mjui_add(&viewer->ui1, defControl);

	// add controls, exit if UI limit reached (Clear button already added)
	int itemcnt = 1;
	for (size_t i = 0; i < viewer->actuator_ctrlrange_.size() && itemcnt < mjMAXUIITEM; i++) {
		// skip if actuator vis group is disabled
		int group = viewer->actuator_group_[i];
		if (!viewer->opt.actuatorgroup[mjMAX(0, mjMIN(mjNGROUP - 1, group))]) {
			continue;
		}
		// grey out if actuator group is disabled
		if (group >= 0 && group <= 30 && viewer->m_->opt.disableactuator & (1 << group)) {
			defSlider[0].state = 0;
		} else {
			defSlider[0].state = 2;
		}

		// set data and name
		if (!viewer->is_passive_) {
			defSlider[0].pdata = &viewer->d_->ctrl[i];
		} else {
			defSlider[0].pdata = &viewer->ctrl_[i];
		}
		if (!viewer->actuator_names_[i].empty()) {
			mju::strcpy_arr(defSlider[0].name, viewer->actuator_names_[i].c_str());
		} else {
			mju::sprintf_arr(defSlider[0].name, "control %d", i);
		}

		// set range
		if (viewer->actuator_ctrlrange_[i].has_value())
			mju::sprintf_arr(defSlider[0].other, "%.4g %.4g", viewer->actuator_ctrlrange_[i]->first,
			                 viewer->actuator_ctrlrange_[i]->second);
		else {
			mju::strcpy_arr(defSlider[0].other, "-1 1");
		}

		// add and count
		mjui_add(&viewer->ui1, defSlider);
		itemcnt++;
	}
}

// make model-dependent UI sections
void MakeUiSections(mujoco_ros::Viewer *viewer, const mjModel *m, const mjData * /*d*/)
{
	// clear model-dependent sections of UI
	viewer->ui0.nsect = SECT_PHYSICS;
	viewer->ui1.nsect = 0;

	// make
	MakePhysicsSection(viewer);
	MakeRenderingSection(viewer, m);
	MakeVisualizationSection(viewer, m);
	MakeGroupSection(viewer);
	MakeJointSection(viewer);
	MakeControlSection(viewer);
}

//---------------------------------- utility functions ---------------------------------------------

// align and scale view
void AlignAndScaleView(mujoco_ros::Viewer *viewer, const mjModel *m)
{
	// use default free camera parameters
	mjv_defaultFreeCamera(m, &viewer->cam);
}

// copy qpos to clipboard as key
void CopyPose(mujoco_ros::Viewer *viewer, const mjModel *m, const mjData *d)
{
	char clipboard[5000] = "<key qpos='";
	char buf[200];

	// prepare string
	for (int i = 0; i < m->nq; i++) {
		mju::sprintf_arr(buf, i == m->nq - 1 ? "%g" : "%g ", d->qpos[i]);
		mju::strcat_arr(clipboard, buf);
	}
	mju::strcat_arr(clipboard, "'/>");

	// copy to clipboard
	viewer->platform_ui->SetClipboardString(clipboard);
}

// millisecond timer, for MuJoCo built-in profiler
mjtNum Timer()
{
	static auto start = Clock::now();
	auto elapsed      = Milliseconds(Clock::now() - start);
	return elapsed.count();
}

// clear all times
void ClearTimers(mjData *d)
{
	for (auto &timer : d->timer) {
		timer.duration = 0;
		timer.number   = 0;
	}
}

// copy current camera to clipboard as MJCF specification
void CopyCamera(mujoco_ros::Viewer *viewer)
{
	mjvGLCamera *camera = viewer->scn.camera;

	char clipboard[500];
	mjtNum cam_right[3];
	mjtNum cam_forward[3];
	mjtNum cam_up[3];

	// get camera spec from the GLCamera
	mju_f2n(cam_forward, camera[0].forward, 3);
	mju_f2n(cam_up, camera[0].up, 3);
	mju_cross(cam_right, cam_forward, cam_up);

	// make MJCF camera spec
	mju::sprintf_arr(clipboard, "<camera pos=\"%.3f %.3f %.3f\" xyaxes=\"%.3f %.3f %.3f %.3f %.3f %.3f\"/>\n",
	                 (camera[0].pos[0] + camera[1].pos[0]) / 2, (camera[0].pos[1] + camera[1].pos[1]) / 2,
	                 (camera[0].pos[2] + camera[1].pos[2]) / 2, cam_right[0], cam_right[1], cam_right[2],
	                 camera[0].up[0], camera[0].up[1], camera[0].up[2]);

	// copy spec into clipboard
	viewer->platform_ui->SetClipboardString(clipboard);
}

// update UI 0 when MuJoCo structures change (except for joint sliders)
void UpdateSettings(mujoco_ros::Viewer *viewer, const mjModel *m)
{
	// physics flags
	for (int i = 0; i < mjNDISABLE; i++) {
		int new_value = ((m->opt.disableflags & (1 << i)) != 0);
		if (viewer->disable[i] != new_value) {
			viewer->disable[i]                 = new_value;
			viewer->pending_.ui_update_physics = true;
		}
	}
	for (int i = 0; i < mjNENABLE; i++) {
		int new_value = ((m->opt.enableflags & (1 << i)) != 0);
		if (viewer->enable[i] != new_value) {
			viewer->enable[i]                  = new_value;
			viewer->pending_.ui_update_physics = true;
		}
	}
	for (int i = 0; i < mjNGROUP; i++) {
		int enabled = ((m->opt.disableactuator & (1 << i)) == 0);
		if (viewer->enableactuator[i] != enabled) {
			viewer->enableactuator[i]          = enabled;
			viewer->pending_.ui_update_physics = true;
			viewer->pending_.ui_remake_ctrl    = true;
		}
	}

	// camera
	int old_camera = viewer->camera;
	if (viewer->cam.type == mjCAMERA_FIXED) {
		viewer->camera = 2 + viewer->cam.fixedcamid;
	} else if (viewer->cam.type == mjCAMERA_TRACKING) {
		viewer->camera = 1;
	} else {
		viewer->camera = 0;
	}
	if (old_camera != viewer->camera) {
		viewer->pending_.ui_update_rendering = true;
	}
}

// Compute suitable font scale.
int ComputeFontScale(const mujoco_ros::PlatformUIAdapter &platform_ui)
{
	// compute framebuffer-to-window ratio
	auto [buf_width, buf_height] = platform_ui.GetFramebufferSize();
	auto [win_width, win_height] = platform_ui.GetWindowSize();
	double b2w                   = static_cast<double>(buf_width) / win_width;

	// compute PPI
	double PPI = b2w * platform_ui.GetDisplayPixelsPerInch();

	// estimate font scaling, guard against unrealistic PPI
	int fs;
	if (buf_width > win_width) {
		fs = mju_round(b2w * 100);
	} else if (PPI > 50 && PPI < 350) {
		fs = mju_round(PPI);
	} else {
		fs = 150;
	}
	fs = mju_round(fs * 0.02) * 50;
	fs = mjMIN(250, mjMAX(100, fs));

	return fs;
}

//---------------------------------- UI handlers ---------------------------------------------------

// determine enable/disable item state given category
int UiPredicate(int category, void *userdata)
{
	auto *viewer = static_cast<mujoco_ros::Viewer *>(userdata);

	switch (category) {
		case 2: // require model
			return viewer->m_ || viewer->is_passive_;

		case 3: // require model and nkey
			return (viewer->m_ || viewer->is_passive_) && viewer->nkey_;

		case 4: // require model and paused
			return viewer->m_ && !viewer->run;

		case 5: // require model and fully managed mode
			return !viewer->is_passive_ && viewer->m_;

		default:
			return 1;
	}
}

// set window layout
void UiLayout(mjuiState *state)
{
	auto *viewer  = static_cast<mujoco_ros::Viewer *>(state->userdata);
	mjrRect *rect = state->rect;

	// set number of rectangles
	state->nrect = 4;

	// rect 1: UI 0
	rect[1].left   = 0;
	rect[1].width  = viewer->ui0_enable ? viewer->ui0.width : 0;
	rect[1].bottom = 0;
	rect[1].height = rect[0].height;

	// rect 2: UI 1
	rect[2].width  = viewer->ui1_enable ? viewer->ui1.width : 0;
	rect[2].left   = mjMAX(0, rect[0].width - rect[2].width);
	rect[2].bottom = 0;
	rect[2].height = rect[0].height;

	// rect 3: 3D plot (everything else is an overlay)
	rect[3].left   = rect[1].width;
	rect[3].width  = mjMAX(0, rect[0].width - rect[1].width - rect[2].width);
	rect[3].bottom = 0;
	rect[3].height = rect[0].height;
}

// modify UI
void UiModify(mjUI *ui, mjuiState *state, mjrContext *con)
{
	mjui_resize(ui, con);

	// remake aux buffer only if missing or different
	int id = ui->auxid;
	if (con->auxFBO[id] == 0 || con->auxFBO_r[id] == 0 || con->auxColor[id] == 0 || con->auxColor_r[id] == 0 ||
	    con->auxWidth[id] != ui->width || con->auxHeight[id] != ui->maxheight ||
	    con->auxSamples[id] != ui->spacing.samples) {
		mjr_addAux(ui->auxid, ui->width, ui->maxheight, ui->spacing.samples, con);
	}
	UiLayout(state);
	mjui_update(-1, -1, ui, state, con);
}

// handle UI event
void UiEvent(mjuiState *state)
{
	auto *viewer = static_cast<mujoco_ros::Viewer *>(state->userdata);

	// call UI 0 if event is directed to it
	if ((state->dragrect == viewer->ui0.rectid) || (state->dragrect == 0 && state->mouserect == viewer->ui0.rectid) ||
	    state->type == mjEVENT_KEY) {
		// process UI event
		mjuiItem *it = mjui_event(&viewer->ui0, state, &viewer->platform_ui->mjr_context());

		// file section
		if (it && it->sectionid == SECT_FILE) {
			switch (it->itemid) {
				case 0: // Save xml
					viewer->pending_.save_xml = GetSavePath("mjmodel.xml");
					break;

				case 1: // Save mjb
					viewer->pending_.save_mjb = GetSavePath("mjmodel.mjb");
					break;

				case 2: // Print model
					viewer->pending_.print_model = GetSavePath("MJMODEL.TXT");
					break;

				case 3: // Print data
					viewer->pending_.print_data = GetSavePath("MJDATA.TXT");
					break;

				case 4: // Quit
					viewer->pending_.ui_exit = true;
					break;

				case 5: // Screenshot
					viewer->screenshot_request.store(true);
					break;
			}
		}

		// option section
		else if (it && it->sectionid == SECT_OPTION) {
			if (it->pdata == &viewer->spacing) {
				viewer->ui0.spacing = mjui_themeSpacing(viewer->spacing);
				viewer->ui1.spacing = mjui_themeSpacing(viewer->spacing);
			} else if (it->pdata == &viewer->color) {
				viewer->ui0.color = mjui_themeColor(viewer->color);
				viewer->ui1.color = mjui_themeColor(viewer->color);
			} else if (it->pdata == &viewer->font) {
				mjr_changeFont(50 * (viewer->font + 1), &viewer->platform_ui->mjr_context());
			} else if (it->pdata == &viewer->fullscreen) {
				viewer->platform_ui->ToggleFullscreen();
			} else if (it->pdata == &viewer->vsync) {
				viewer->platform_ui->SetVSync(viewer->vsync);
			}

			// modify UI
			UiModify(&viewer->ui0, state, &viewer->platform_ui->mjr_context());
			UiModify(&viewer->ui1, state, &viewer->platform_ui->mjr_context());
		}

		// simulation section
		else if (it && it->sectionid == SECT_SIMULATION) {
			switch (it->itemid) {
				case 1: // Reset
					viewer->pending_.ui_reset = true;
					break;

				case 2: // Reload
					viewer->pending_.ui_reload = true;
					break;

				case 3: // Align
					viewer->pending_.align = true;
					break;

				case 4: // Copy pose
					viewer->pending_.copy_pose = true;
					break;

				case 5: // Adjust key
				case 6: // Load key
					viewer->pending_.load_key = true;
					break;

				case 7: // Save key
					viewer->pending_.save_key = true;
					break;

				case 11: // History scrubber
					viewer->run                        = 0;
					viewer->pending_.ui_update_run     = true;
					viewer->pending_.load_from_history = true;
					mjui0_update_section(viewer, SECT_SIMULATION);
					break;
			}
		}

		// physics section
		else if (it && it->sectionid == SECT_PHYSICS && viewer->m_) {
			mjOption *opt = !viewer->is_passive_ ? &viewer->scnstate_.model.opt : &viewer->m_->opt;

			// update disable flags in mjOption
			opt->disableflags = 0;
			for (int i = 0; i < mjNDISABLE; i++) {
				if (viewer->disable[i]) {
					opt->disableflags |= (1 << i);
				}
			}

			// update enable flags in mjOption
			opt->enableflags = 0;
			for (int i = 0; i < mjNENABLE; i++) {
				if (viewer->enable[i]) {
					opt->enableflags |= (1 << i);
				}
			}
			// update disableactuator bitflag in mjOption
			bool group_changed = false;
			for (int i = 0; i < mjNGROUP; i++) {
				if ((!viewer->enableactuator[i]) != (opt->disableactuator & (1 << i))) {
					group_changed = true;
					if (!viewer->enableactuator[i]) {
						// disable actuator group i
						opt->disableactuator |= (1 << i);
					} else {
						opt->disableactuator &= ~(1 << i);
					}
				}
			}

			if (group_changed) {
				viewer->pending_.ui_remake_ctrl = true;
			}

			// Update flags in env
			if (!viewer->is_passive_) {
				viewer->env_->UpdateModelFlags(opt);
				viewer->env_->settings_.settings_changed.store(true);
			}

		}

		// rendering section
		else if (it && it->sectionid == SECT_RENDERING) {
			// set camera in mjvCamera
			if (viewer->camera == 0) {
				viewer->cam.type = mjCAMERA_FREE;
			} else if (viewer->camera == 1) {
				if (viewer->pert.select > 0) {
					viewer->cam.type        = mjCAMERA_TRACKING;
					viewer->cam.trackbodyid = viewer->pert.select;
					viewer->cam.fixedcamid  = -1;
				} else {
					viewer->cam.type = mjCAMERA_FREE;
					viewer->camera   = 0;
					mjui0_update_section(viewer, SECT_RENDERING);
				}
			} else {
				viewer->cam.type       = mjCAMERA_FIXED;
				viewer->cam.fixedcamid = viewer->camera - 2;
			}
			// copy camera spec to clipboard (as MJCF element)
			if (it->itemid == 3) {
				CopyCamera(viewer);
			}
		}

		// visualization section
		else if (it && it->sectionid == SECT_VISUALIZATION) {
			if (!mju::strcmp_arr(it->name, "Align")) {
				viewer->pending_.align = true;
			}
		}

		// group section
		else if (it && it->sectionid == SECT_GROUP) {
			// remake joint section if joint group changed
			if (it->name[0] == 'J' && it->name[1] == 'o') {
				viewer->ui1.nsect = SECT_JOINT;
				MakeJointSection(viewer);
				viewer->ui1.nsect = NSECT1;
				UiModify(&viewer->ui1, state, &viewer->platform_ui->mjr_context());
			}

			// remake control section if actuator group changed
			if (it->name[0] == 'A' && it->name[1] == 'c') {
				viewer->pending_.ui_remake_ctrl = true;
			}
		}

		// stop if UI processed event
		if (it != nullptr || (state->type == mjEVENT_KEY && state->key == 0)) {
			return;
		}
	}

	// call UI 1 if event is directed to it
	if ((state->dragrect == viewer->ui1.rectid) || (state->dragrect == 0 && state->mouserect == viewer->ui1.rectid) ||
	    state->type == mjEVENT_KEY) {
		// process UI event
		mjuiItem *it = mjui_event(&viewer->ui1, state, &viewer->platform_ui->mjr_context());

		// control section
		if (it && it->sectionid == SECT_CONTROL) {
			// clear controls
			if (it->itemid == 0) {
				viewer->pending_.zero_ctrl = true;
			}
		}

		// stop if UI processed event
		if (it != nullptr || (state->type == mjEVENT_KEY && state->key == 0)) {
			return;
		}
	}

	// shortcut not handled by UI
	if (state->type == mjEVENT_KEY && state->key != 0) {
		switch (state->key) {
			case ' ': // Mode
				if (!viewer->is_passive_ && viewer->m_) {
					viewer->pending_.ui_update_run = true;
					viewer->pert.active            = 0;

					if (viewer->env_->settings_.run.load())
						viewer->scrub_index = 0; // reset scrubber
					mjui0_update_section(viewer, -1);
				}
				break;

			case mjKEY_RIGHT: // step forward
				if (!viewer->is_passive_ && !viewer->env_->settings_.run.load()) {
					ClearTimers(viewer->d_.get());

					// currently in scrubber: increment scrub, load state, update slider UI
					if (viewer->scrub_index < 0) {
						viewer->scrub_index++;
						viewer->pending_.load_from_history = true;
						mjui0_update_section(viewer, SECT_SIMULATION);
					}

					// not in scrubber: step, add to history buffer
					else {
						viewer->env_->settings_.env_steps_request.fetch_add(1);
						viewer->AddToHistory();
					}
				}
				break;

			case mjKEY_LEFT: // step backward
				if (!viewer->is_passive_ && viewer->m_) {
					viewer->pending_.ui_update_run = true;
					ClearTimers(viewer->d_.get());

					// decrement scrub, load state
					viewer->scrub_index                = mjMAX(viewer->scrub_index - 1, 1 - viewer->nhistory_);
					viewer->pending_.load_from_history = true;

					// Update slider UI, profiler, sensor
					mjui0_update_section(viewer, SECT_SIMULATION);
					UpdateProfiler(viewer, viewer->m_.get(), viewer->d_.get());
					UpdateSensor(viewer, viewer->m_.get(), viewer->d_.get());
				}
				break;

			case mjKEY_DOWN: // step forward 100
				if (!viewer->env_->settings_.run.load()) {
					ClearTimers(viewer->d_.get());
					viewer->env_->settings_.env_steps_request.fetch_add(100);
				}
				break;

			case mjKEY_PAGE_UP: // select parent body
				if ((viewer->m_ || viewer->is_passive_) && viewer->pert.select > 0) {
					viewer->pert.select     = viewer->body_parentid_[util::as_unsigned(viewer->pert.select)];
					viewer->pert.flexselect = -1;
					viewer->pert.skinselect = -1;

					// stop perturbation if world reached
					if (viewer->pert.select <= 0) {
						viewer->pert.active = 0;
					}
				}

				break;

			case ']': // cycle up fixed cameras
				if ((viewer->m_ || viewer->is_passive_) && viewer->ncam_) {
					viewer->cam.type = mjCAMERA_FIXED;
					// camera = {0 or 1} are reserved for the free and tracking cameras
					if (viewer->camera < 2 || viewer->camera == 2 + viewer->ncam_ - 1) {
						viewer->camera = 2;
					} else {
						viewer->camera += 1;
					}
					viewer->cam.fixedcamid = viewer->camera - 2;
					mjui0_update_section(viewer, SECT_RENDERING);
				}
				break;

			case '[': // cycle down fixed cameras
				if ((viewer->m_ || viewer->is_passive_) && viewer->ncam_) {
					viewer->cam.type = mjCAMERA_FIXED;
					// camera = {0 or 1} are reserved for the free and tracking cameras
					if (viewer->camera <= 2) {
						viewer->camera = 2 + viewer->ncam_ - 1;
					} else {
						viewer->camera -= 1;
					}
					viewer->cam.fixedcamid = viewer->camera - 2;
					mjui0_update_section(viewer, SECT_RENDERING);
				}
				break;

			case mjKEY_F6: // cycle frame visualisation
				if (viewer->m_ || viewer->is_passive_) {
					viewer->opt.frame = (viewer->opt.frame + 1) % mjNFRAME;
					mjui0_update_section(viewer, SECT_RENDERING);
				}
				break;

			case mjKEY_F7: // cycle label visualisation
				if (viewer->m_ || viewer->is_passive_) {
					viewer->opt.label = (viewer->opt.label + 1) % mjNLABEL;
					mjui0_update_section(viewer, SECT_RENDERING);
				}
				break;

			case mjKEY_ESCAPE: // free camera
				viewer->cam.type = mjCAMERA_FREE;
				viewer->camera   = 0;
				mjui0_update_section(viewer, SECT_RENDERING);
				break;

			case '-': // slow down
				if (!viewer->is_passive_) {
					int numclicks =
					    sizeof(mujoco_ros::MujocoEnv::percentRealTime) / sizeof(mujoco_ros::MujocoEnv::percentRealTime[0]);
					if (viewer->real_time_index < numclicks - 1 && !state->shift) {
						viewer->real_time_index++;
						viewer->pending_.ui_update_speed = true;
					}
				}
				break;

			case '=': // speed up
				if (!viewer->is_passive_ && viewer->real_time_index > 0 && !state->shift) {
					viewer->real_time_index--;
					viewer->pending_.ui_update_speed = true;
				}
				break;

			case mjKEY_TAB: // toggle left/right UI
				if (!state->shift) {
					// loggle left UI
					viewer->ui0_enable = !viewer->ui0_enable;
					UiModify(&viewer->ui0, state, &viewer->platform_ui->mjr_context());
				} else {
					// loggle right UI
					viewer->ui1_enable = !viewer->ui1_enable;
					UiModify(&viewer->ui1, state, &viewer->platform_ui->mjr_context());
				}
				break;
		}

		return;
	}

	// 3D scroll
	if (state->type == mjEVENT_SCROLL && state->mouserect == 3) {
		// emulate vertical mouse motion = 2% of window height
		if (viewer->m_ && !viewer->is_passive_) {
			mjv_moveCamera(viewer->m_.get(), mjMOUSE_ZOOM, 0, -zoom_increment * state->sy, &viewer->scn, &viewer->cam);
		} else {
			mjv_moveCameraFromState(&viewer->scnstate_, mjMOUSE_ZOOM, 0, -zoom_increment * state->sy, &viewer->scn,
			                        &viewer->cam);
		}
		return;
	}

	// 3D press
	if (state->type == mjEVENT_PRESS && state->mouserect == 3) {
		// set perturbation
		int newperturb = 0;
		if (state->control && viewer->pert.select > 0 && (viewer->m_ || viewer->is_passive_)) {
			// right: translate;  left: rotate
			if (state->right) {
				newperturb = mjPERT_TRANSLATE;
			} else if (state->left) {
				newperturb = mjPERT_ROTATE;
			}
			if (newperturb && !viewer->pert.active) {
				viewer->pending_.newperturb = newperturb;
			}
		}

		// handle double-click
		if (state->doubleclick && (viewer->m_ || viewer->is_passive_)) {
			viewer->pending_.select = true;
			std::memcpy(&viewer->pending_.select_state, state, sizeof(viewer->pending_.select_state));

			// stop perturbation on select
			viewer->pert.active         = 0;
			viewer->pending_.newperturb = 0;
		}

		return;
	}

	// 3D release
	if (state->type == mjEVENT_RELEASE && state->dragrect == 3 && (viewer->m_ || viewer->is_passive_)) {
		// stop perturbation
		viewer->pert.active         = 0;
		viewer->pending_.newperturb = 0;
		return;
	}

	// 3D move
	if (state->type == mjEVENT_MOVE && state->dragrect == 3 && (viewer->m_ || viewer->is_passive_)) {
		// determine action based on mouse button
		mjtMouse action;
		if (state->right) {
			action = state->shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
		} else if (state->left) {
			action = state->shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
		} else {
			action = mjMOUSE_ZOOM;
		}

		// move perturb or camera
		mjrRect r = state->rect[3];
		if (viewer->pert.active) {
			if (!viewer->is_passive_) {
				mjv_movePerturb(viewer->m_.get(), viewer->d_.get(), action, state->dx / r.height, -state->dy / r.height,
				                &viewer->scn, &viewer->pert);
			} else {
				mjv_movePerturbFromState(&viewer->scnstate_, action, state->dx / r.height, -state->dy / r.height,
				                         &viewer->scn, &viewer->pert);
			}
		} else {
			if (!viewer->is_passive_) {
				mjv_moveCamera(viewer->m_.get(), action, state->dx / r.height, -state->dy / r.height, &viewer->scn,
				               &viewer->cam);
			} else {
				mjv_moveCameraFromState(&viewer->scnstate_, action, state->dx / r.height, -state->dy / r.height,
				                        &viewer->scn, &viewer->cam);
			}
		}
		return;
	}

	// Dropped files
	if (state->type == mjEVENT_FILESDROP && state->dropcount > 0 && !viewer->is_passive_) {
		while (viewer->dropload_request.load()) {
		}
		mju::strcpy_arr(viewer->dropfilename, state->droppaths[0]);
		viewer->dropload_request.store(true);
		return;
	}

	// Redraw
	if (state->type == mjEVENT_REDRAW) {
		viewer->Render();
		return;
	}
}
} // anonymous namespace

namespace mujoco_ros {
namespace mju = ::mujoco::sample_util;

Viewer::Viewer(std::unique_ptr<PlatformUIAdapter> platform_ui_adapter, MujocoEnv *env, bool is_passive)
    : env_(env)
    , pert(env->pert_)
    , platform_ui(std::move(platform_ui_adapter))
    , uistate(this->platform_ui->state())
    , is_passive_(is_passive)
{
	mjv_defaultScene(&scn);
	mjv_defaultSceneState(&scnstate_);
	env_->connectViewer(this);
}

// synchronize model and data
// operations which require holding the mutex, prevents racing with physics thread
void Viewer::Sync()
{
	MutexLock lock(this->mtx);
	MutexLock lock_env(env_->physics_thread_mutex_);
	if (!m_ || !d_) {
		return;
	}

	if (!env_) {
		return;
	}

	if (this->exit_request.load()) {
		return;
	}

	// Avoid updating during load
	if (env_->settings_.load_request == 1) {
		return;
	}

	bool update_profiler = this->profiler && (this->pause_update || this->run);
	bool update_sensor   = this->sensor && (this->pause_update || this->run);

	for (size_t i = 0; i < m_->njnt; i++) {
		std::optional<std::pair<mjtNum, mjtNum>> range;
		if (m_->jnt_limited[i]) {
			range.emplace(m_->jnt_range[2 * i], m_->jnt_range[2 * i + 1]);
		}
		if (jnt_range_[i] != range) {
			pending_.ui_update_joint = true;
			jnt_range_[i].swap(range);
		}
	}

	for (size_t i = 0; i < m_->nu; i++) {
		std::optional<std::pair<mjtNum, mjtNum>> range;
		if (m_->actuator_ctrllimited[i]) {
			range.emplace(m_->actuator_ctrlrange[2 * i], m_->actuator_ctrlrange[2 * i + 1]);
		}
		if (actuator_ctrlrange_[i] != range) {
			pending_.ui_remake_ctrl = true;
			actuator_ctrlrange_[i].swap(range);
		}
	}

	for (size_t i = 0; i < m_->nq; i++) {
		if (qpos_[i] != qpos_prev_[i]) {
			d_->qpos[i] = qpos_[i];
		} else {
			qpos_[i] = d_->qpos[i];
		}
		if (qpos_prev_[i] != qpos_[i]) {
			pending_.ui_update_joint = true;
			qpos_prev_[i]            = qpos_[i];
		}
	}

	for (size_t i = 0; i < m_->nu; i++) {
		if (ctrl_[i] != ctrl_prev_[i]) {
			d_->ctrl[i] = ctrl_[i];
		} else {
			ctrl_[i] = d_->ctrl[i];
		}
		if (ctrl_prev_[i] != ctrl_[i]) {
			pending_.ui_update_ctrl = true;
			ctrl_prev_[i]           = ctrl_[i];
		}
	}

	if (is_passive_) {
		// synchronize m_->opt with changes made via the UI
#define X(name)                                                   \
	if (IsDifferent(scnstate_.model.opt.name, mjopt_prev_.name)) { \
		pending_.ui_update_physics = true;                          \
		Copy(m_->opt.name, scnstate_.model.opt.name);               \
	}

		X(timestep);
		X(apirate);
		X(impratio);
		X(tolerance);
		X(noslip_tolerance);
		X(mpr_tolerance);
		X(gravity);
		X(wind);
		X(magnetic);
		X(density);
		X(viscosity);
		X(o_margin);
		X(o_solref);
		X(o_solimp);
		X(o_friction);
		X(integrator);
		X(cone);
		X(jacobian);
		X(solver);
		X(iterations);
		X(noslip_iterations);
		X(mpr_iterations);
		X(disableflags);
		X(enableflags);
		X(disableactuator);
		X(sdf_initpoints);
		X(sdf_iterations);
#undef X

		// synchronize number of mjWARN_VGEOMFULL warnings
		if (scnstate_.data.warning[mjWARN_VGEOMFULL].number > warn_vgeomfull_prev_) {
			d_->warning[mjWARN_VGEOMFULL].number += scnstate_.data.warning[mjWARN_VGEOMFULL].number - warn_vgeomfull_prev_;
		}
	}

	this->run = env_->settings_.run.load();
	if (pending_.ui_update_run) {
		env_->settings_.run.store(1 - env_->settings_.run.load());
		env_->settings_.settings_changed.store(true);
		this->run              = env_->settings_.run.load();
		pending_.ui_update_run = false;
	}

	if (pending_.ui_update_speed) {
		env_->settings_.real_time_index = real_time_index;
		env_->settings_.speed_changed   = true;
		pending_.ui_update_speed        = false;
	} else {
		real_time_index = env_->settings_.real_time_index;
	}

	if (pending_.save_xml) {
		char err[200];
		if (!pending_.save_xml->empty() && !mj_saveLastXML(pending_.save_xml->c_str(), m_.get(), err, 200)) {
			std::printf("Save XML error: %s", err);
		}
		pending_.save_xml = std::nullopt;
	}

	if (pending_.save_mjb) {
		if (!pending_.save_mjb->empty()) {
			mj_saveModel(m_.get(), pending_.save_mjb->c_str(), nullptr, 0);
		}
		pending_.save_mjb = std::nullopt;
	}

	if (pending_.print_model) {
		if (!pending_.print_model->empty()) {
			mj_printModel(m_.get(), pending_.print_model->c_str());
		}
		pending_.print_model = std::nullopt;
	}

	if (pending_.print_data) {
		if (!pending_.print_data->empty()) {
			mj_printData(m_.get(), d_.get(), pending_.print_data->c_str());
		}
		pending_.print_data = std::nullopt;
	}

	if (pending_.ui_reset) {
		env_->settings_.reset_request.store(1);
		pending_.ui_reset             = false;
		update_profiler               = true;
		update_sensor                 = true;
		scrub_index                   = 0;
		pending_.ui_update_simulation = true;
	}

	if (pending_.ui_reload) {
		env_->settings_.load_request.store(3); // 3 triggers prepare reload
		pending_.ui_reload = false;
		update_profiler    = true;
		update_sensor      = true;
	}

	if (dropload_request.load()) {
		mju::strcpy_arr(env_->queued_filename_, dropfilename);
		dropload_request.store(0);
		env_->settings_.load_request.store(3);
		update_profiler = true;
		update_sensor   = true;
	}

	if (pending_.align) {
		AlignAndScaleView(this, m_.get());
		pending_.align = false;
	}

	if (pending_.copy_pose) {
		CopyPose(this, m_.get(), d_.get());
		pending_.copy_pose = false;
	}

	if (pending_.load_from_history) {
		LoadScrubState(this);
		update_profiler            = true;
		update_sensor              = true;
		pending_.load_from_history = false;
	}

	if (pending_.load_key) {
		int i = this->key;
		// Backup current time
		mjtNum tmp_time = d_->time;
		// TODO(dleins): Should resetting to a keypoint remain allowed?
		mj_resetDataKeyframe(m_.get(), d_.get(), i);
		// Set time back to the original value
		d_->time = tmp_time;
		mj_forward(m_.get(), d_.get());
		update_profiler   = true;
		update_sensor     = true;
		pending_.load_key = false;
	}

	if (pending_.save_key) {
		int i = this->key;
		mj_setKeyframe(m_.get(), d_.get(), i);
		pending_.save_key = false;
	}

	if (pending_.zero_ctrl) {
		mju_zero(d_->ctrl, m_->nu);
		pending_.zero_ctrl = false;
	}

	if (pending_.newperturb) {
		mjv_initPerturb(m_.get(), d_.get(), &this->scn, &this->pert);
		this->pert.active   = pending_.newperturb;
		pending_.newperturb = 0;
	}

	if (pending_.select) {
		// determine selection mode
		int selmode;
		if (pending_.select_state.button == mjBUTTON_LEFT) {
			selmode = 1;
		} else if (pending_.select_state.control) {
			selmode = 3;
		} else {
			selmode = 2;
		}

		// find geom and 3D click point, get corresponding body
		mjrRect r = pending_.select_state.rect[3];
		mjtNum selpnt[3];
		int selgeom, selflex, selskin;
		int selbody =
		    mjv_select(m_.get(), d_.get(), &this->opt, static_cast<mjtNum>(r.width) / r.height,
		               (pending_.select_state.x - r.left) / r.width, (pending_.select_state.y - r.bottom) / r.height,
		               &this->scn, selpnt, &selgeom, &selflex, &selskin);

		// set lookat point, start tracking is requested
		if (selmode == 2 || selmode == 3) {
			// copy selpnt if anything clicked
			if (selbody >= 0) {
				mju_copy3(this->cam.lookat, selpnt);
			}

			// switch to tracking camera if dynamic body clicked
			if (selmode == 3 && selbody > 0) {
				// mujoco camera
				this->cam.type        = mjCAMERA_TRACKING;
				this->cam.trackbodyid = selbody;
				this->cam.fixedcamid  = -1;

				// UI camera
				this->camera                 = 1;
				pending_.ui_update_rendering = true;
			}
		}

		// set body selection
		else {
			if (selbody >= 0) {
				// record selection
				this->pert.select     = selbody;
				this->pert.flexselect = selflex;
				this->pert.skinselect = selskin;

				// compute localpos
				mjtNum tmp[3];
				mju_sub3(tmp, selpnt, d_->xpos + 3 * this->pert.select);
				mju_mulMatTVec(this->pert.localpos, d_->xmat + 9 * this->pert.select, tmp, 3, 3);
			} else {
				this->pert.select     = 0;
				this->pert.flexselect = -1;
				this->pert.skinselect = -1;
			}
		}
		pending_.select = false;
	}

	if (!is_passive_) {
		mjv_updateScene(m_.get(), d_.get(), &this->opt, &this->pert, &this->cam, mjCAT_ALL, &this->scn);
	} else {
		mjv_updateSceneState(m_.get(), d_.get(), &this->opt, &scnstate_);

		// append geoms from user_scn to scnstate_ scratch space
		if (user_scn) {
			int ngeom   = user_scn->ngeom;
			int maxgeom = scnstate_.scratch.maxgeom - scnstate_.scratch.ngeom;
			if (ngeom > maxgeom) {
				mj_warning(d_.get(), mjWARN_VGEOMFULL, scnstate_.scratch.maxgeom);
				ngeom = maxgeom;
			}
			if (ngeom > 0) {
				std::memcpy(scnstate_.scratch.geoms + scnstate_.scratch.ngeom, user_scn->geoms, sizeof(mjvGeom) * ngeom);
				scnstate_.scratch.ngeom += ngeom;
			}
		}

		// pick up rendering flags changed via user_scn
		if (user_scn) {
			for (int i = 0; i < mjNRNDFLAG; ++i) {
				if (user_scn->flags[i] != user_scn_flags_prev_[i]) {
					scn.flags[i]                 = user_scn->flags[i];
					pending_.ui_update_rendering = true;
				}
			}
			Copy(user_scn->flags, scn.flags);
			Copy(user_scn_flags_prev_, user_scn->flags);
		}

		mjopt_prev_          = scnstate_.model.opt;
		warn_vgeomfull_prev_ = scnstate_.data.warning[mjWARN_VGEOMFULL].number;
	}

	// Run render cbs from plugins
	this->env_->runRenderCbs(&this->scn);

	// update settings
	UpdateSettings(this, m_.get());

	// update watch
	if (this->ui0_enable && this->ui0.sect[SECT_WATCH].state) {
		UpdateWatch(this, m_.get(), d_.get());
	}

	// update info text
	if (this->info) {
		UpdateInfoText(this, m_.get(), d_.get(), this->info_title, this->info_content);
	}
	if (update_profiler) {
		UpdateProfiler(this, m_.get(), d_.get());
	}
	if (update_sensor) {
		UpdateSensor(this, m_.get(), d_.get());
	}

	// clear timers once profiler info has been copied
	ClearTimers(d_.get());

	if (this->run || this->is_passive_) {
		// clear old perturbations, apply new
		mju_zero(d_->xfrc_applied, 6 * m_->nbody);
		mjv_applyPerturbPose(m_.get(), d_.get(), &this->pert, 0); // move mocap bodies only
		mjv_applyPerturbForce(m_.get(), d_.get(), &this->pert);
	} else {
		mjv_applyPerturbPose(m_.get(), d_.get(), &this->pert, 1); // mocap and dynamic bodies
	}

	if (pending_.ui_exit) {
		env_->settings_.exit_request.store(1);
		this->exit_request.store(1);
	}
}

void Viewer::LoadMessage(const char *displayed_filename)
{
	mju::strcpy_arr(this->filename, displayed_filename);

	{
		MutexLock lock(mtx);
		this->loadrequest = 3;
	}
}

void Viewer::LoadMessageClear()
{
	{
		MutexLock lock(mtx);
		this->loadrequest = 0;
	}
}

void Viewer::Load(mjModelPtr m, mjDataPtr d, const char *displayed_filename)
{
	this->mnew_ = std::move(m);
	this->dnew_ = std::move(d);
	mju::strcpy_arr(this->filename, displayed_filename);

	{
		MutexLock lock(mtx);
		this->loadrequest = 2;

		// Wait for the render thread to be done loading
		// so that we know the old model and data's memory can
		// be freed by the other thread (sometimes python)
		cond_loadrequest.wait(lock, [this]() { return this->loadrequest == 0; });
	}
}

void Viewer::LoadOnRenderThread()
{
	ROS_DEBUG("Loading model in render thread");
	this->m_ = this->mnew_;
	this->d_ = this->dnew_;

	ncam_ = this->m_->ncam;
	nkey_ = this->m_->nkey;
	body_parentid_.resize(util::as_unsigned(this->m_->nbody));
	std::memcpy(body_parentid_.data(), this->m_->body_parentid,
	            sizeof(this->m_->body_parentid[0]) * util::as_unsigned(this->m_->nbody));
	body_parentid_.shrink_to_fit();

	jnt_type_.resize(util::as_unsigned(this->m_->njnt));
	std::memcpy(jnt_type_.data(), this->m_->jnt_type, sizeof(this->m_->jnt_type[0]) * util::as_unsigned(this->m_->njnt));
	jnt_type_.shrink_to_fit();

	jnt_group_.resize(util::as_unsigned(this->m_->njnt));
	std::memcpy(jnt_group_.data(), this->m_->jnt_group,
	            sizeof(this->m_->jnt_group[0]) * util::as_unsigned(this->m_->njnt));
	jnt_group_.shrink_to_fit();

	jnt_qposadr_.resize(util::as_unsigned(this->m_->njnt));
	std::memcpy(jnt_qposadr_.data(), this->m_->jnt_qposadr,
	            sizeof(this->m_->jnt_qposadr[0]) * util::as_unsigned(this->m_->njnt));
	jnt_qposadr_.shrink_to_fit();

	jnt_range_.clear();
	jnt_range_.reserve(util::as_unsigned(this->m_->njnt));
	for (int i = 0; i < this->m_->njnt; ++i) {
		if (this->m_->jnt_limited[i]) {
			jnt_range_.emplace_back(std::make_pair(this->m_->jnt_range[2 * i], this->m_->jnt_range[2 * i + 1]));
		} else {
			jnt_range_.emplace_back(std::nullopt);
		}
	}
	jnt_range_.shrink_to_fit();

	jnt_names_.clear();
	jnt_names_.reserve(util::as_unsigned(this->m_->njnt));
	for (int i = 0; i < this->m_->njnt; ++i) {
		jnt_names_.emplace_back(this->m_->names + this->m_->name_jntadr[i]);
	}
	jnt_names_.shrink_to_fit();

	actuator_group_.resize(util::as_unsigned(this->m_->nu));
	std::memcpy(actuator_group_.data(), this->m_->actuator_group,
	            sizeof(this->m_->actuator_group[0]) * util::as_unsigned(this->m_->nu));
	actuator_group_.shrink_to_fit();

	actuator_ctrlrange_.clear();
	actuator_ctrlrange_.reserve(util::as_unsigned(this->m_->nu));
	for (int i = 0; i < this->m_->nu; ++i) {
		if (this->m_->actuator_ctrllimited[i]) {
			actuator_ctrlrange_.emplace_back(
			    std::make_pair(this->m_->actuator_ctrlrange[2 * i], this->m_->actuator_ctrlrange[2 * i + 1]));
		} else {
			actuator_ctrlrange_.emplace_back(std::nullopt);
		}
	}
	actuator_ctrlrange_.shrink_to_fit();

	actuator_names_.clear();
	actuator_names_.reserve(util::as_unsigned(this->m_->nu));
	for (int i = 0; i < this->m_->nu; ++i) {
		actuator_names_.emplace_back(this->m_->names + this->m_->name_actuatoradr[i]);
	}
	actuator_names_.shrink_to_fit();

	qpos_.resize(util::as_unsigned(this->m_->nq));
	std::memcpy(qpos_.data(), this->m_->qpos0, sizeof(this->m_->qpos0[0]) * util::as_unsigned(this->m_->nq));
	qpos_.shrink_to_fit();
	qpos_prev_ = qpos_;

	ctrl_.resize(util::as_unsigned(this->m_->nu));
	std::memcpy(ctrl_.data(), this->d_->ctrl, sizeof(this->d_->ctrl[0]) * util::as_unsigned(this->m_->nu));
	ctrl_.shrink_to_fit();
	ctrl_prev_ = ctrl_;

	// allocate history buffer: smaller of {2000 states, 100 MB}
	if (!this->is_passive_) {
		constexpr int kMaxHistoryBytes = 1e8;

		// get state size, size of history buffer
		state_size_        = mj_stateSize(this->m_.get(), mjSTATE_INTEGRATION);
		int state_bytes    = state_size_ * sizeof(mjtNum);
		int history_length = mjMIN(INT_MAX / state_bytes, 2000);
		int history_bytes  = mjMIN(state_bytes * history_length, kMaxHistoryBytes);
		nhistory_          = history_bytes / state_bytes;

		// allocate history buffer, reset cursor and UI slider
		history_.clear();
		history_.resize(nhistory_ * state_size_);
		history_cursor_ = 0;
		scrub_index     = 0;

		// fill buffer with initial state
		mj_getState(this->m_.get(), this->d_.get(), history_.data(), mjSTATE_INTEGRATION);
		for (int i = 1; i < nhistory_; ++i) {
			mju_copy(&history_[i * state_size_], history_.data(), state_size_);
		}
	}

	// re-create scene and context
	mjv_makeScene(this->m_.get(), &this->scn, this->kMaxGeom);
	if (this->is_passive_) {
		mjopt_prev_          = m_->opt;
		opt_prev_            = opt;
		cam_prev_            = cam;
		warn_vgeomfull_prev_ = d_->warning[mjWARN_VGEOMFULL].number;
		mjv_makeSceneState(this->m_.get(), this->d_.get(), &this->scnstate_, this->kMaxGeom);
	}

	this->platform_ui->RefreshMjrContext(this->m_.get(), 50 * (this->font + 1));
	UiModify(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
	UiModify(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());

	if (!this->platform_ui->IsGPUAccelerated()) {
		this->scn.flags[mjRND_SHADOW]     = 0;
		this->scn.flags[mjRND_REFLECTION] = 0;
	}

	if (this->user_scn) {
		Copy(this->user_scn->flags, this->scn.flags);
		Copy(this->user_scn_flags_prev_, this->scn.flags);
	}

	// clear perturbation state
	this->pert.active     = 0;
	this->pert.select     = 0;
	this->pert.flexselect = -1;
	this->pert.skinselect = -1;

	// align and scale view unless reloading the same file
	if (this->filename[0] && mju::strcmp_arr(this->filename, this->previous_filename)) {
		AlignAndScaleView(this, this->m_.get());
		mju::strcpy_arr(this->previous_filename, this->filename);
	}

	// update scene
	if (!is_passive_) {
		mjv_updateScene(this->m_.get(), this->d_.get(), &this->opt, &this->pert, &this->cam, mjCAT_ALL, &this->scn);
	} else {
		mjv_updateSceneState(this->m_.get(), this->d_.get(), &this->opt, &this->scnstate_);
	}

	// set window title to model name
	if (this->m_->names) {
		char title[200] = "MuJoCo ROS Viewer : ";
		mju::strcat_arr(title, this->m_->names);
		platform_ui->SetWindowTitle(title);
	}

	// set keyframe range and divisions
	this->ui0.sect[SECT_SIMULATION].item[5].slider.range[0]  = 0;
	this->ui0.sect[SECT_SIMULATION].item[5].slider.range[1]  = mjMAX(0, this->m_->nkey - 1);
	this->ui0.sect[SECT_SIMULATION].item[5].slider.divisions = mjMAX(1, this->m_->nkey - 1);

	// set scrubber range and divisions
	this->ui0.sect[SECT_SIMULATION].item[11].slider.range[0]  = 1 - nhistory_;
	this->ui0.sect[SECT_SIMULATION].item[11].slider.divisions = nhistory_;

	// rebuild UI sections
	MakeUiSections(this, this->m_.get(), this->d_.get());

	// full UI update
	UiModify(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
	UiModify(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());
	UpdateSettings(this, this->m_.get());

	// clear request
	ROS_DEBUG("Notifying load request complete");
	this->loadrequest = 0;
	cond_loadrequest.notify_all();

	// set real time index
	int numclicks   = sizeof(MujocoEnv::percentRealTime) / sizeof(MujocoEnv::percentRealTime[0]);
	float min_error = 1e6;
	float desired   = mju_log(100 * this->m_->vis.global.realtime);
	for (int click = 0; click < numclicks; click++) {
		float error = mju_abs(mju_log(MujocoEnv::percentRealTime[click]) - desired);
		if (error < min_error) {
			min_error             = error;
			this->real_time_index = click;
		}
	}

	this->mnew_ = nullptr;
	this->dnew_ = nullptr;
}

// render the UI to the window
void Viewer::Render()
{
	// Update rendering context buffer size if required
	if (this->platform_ui->EnsureContextSize()) {
		UiModify(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
		UiModify(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());
	}

	// Get 3D rectangle and reduced for profiler
	mjrRect rect      = this->uistate.rect[3];
	mjrRect smallrect = rect;
	if (this->profiler) {
		smallrect.width = rect.width / 4;
	}

	// No model
	if (!is_passive_ && !this->m_) {
		// blank screen
		mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);

		// label
		if (this->loadrequest) {
			mjr_overlay(mjFONT_BIG, mjGRID_TOP, smallrect, "LOADING...", nullptr, &this->platform_ui->mjr_context());
		} else {
			char intro_message[Viewer::kMaxFilenameLength];
			mju::sprintf_arr(intro_message, "MuJoCo ROS version %s\nLoad model to visualize", mj_versionString());
			mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, intro_message, nullptr, &this->platform_ui->mjr_context());
		}

		// Show last loading error
		if (this->load_error[0]) {
			mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOM, rect, this->load_error, nullptr, &this->platform_ui->mjr_context());
		}

		// render UIs
		if (this->ui0_enable) {
			mjui_render(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
		}
		if (this->ui1_enable) {
			mjui_render(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());
		}

		// finalize
		this->platform_ui->SwapBuffers();

		return;
	}

	// Update UI sections from last sync
	if (pending_.ui_update_simulation) {
		if (this->ui0_enable && this->ui0.sect[SECT_SIMULATION].state) {
			mjui0_update_section(this, SECT_SIMULATION);
		}
		pending_.ui_update_simulation = false;
	}

	if (this->ui0_enable && this->ui0.sect[SECT_WATCH].state) {
		mjui0_update_section(this, SECT_WATCH);
	}

	if (pending_.ui_update_physics) {
		if (this->ui0_enable && this->ui0.sect[SECT_PHYSICS].state) {
			mjui0_update_section(this, SECT_PHYSICS);
		}
		pending_.ui_update_physics = false;
	}

	if (is_passive_) {
		if (this->ui0_enable && this->ui0.sect[SECT_RENDERING].state &&
		    (cam_prev_.type != cam.type || cam_prev_.fixedcamid != cam.fixedcamid ||
		     cam_prev_.trackbodyid != cam.trackbodyid || opt_prev_.label != opt.label || opt_prev_.frame != opt.frame ||
		     IsDifferent(opt_prev_.flags, opt.flags))) {
			pending_.ui_update_rendering = true;
		}

		if (this->ui0_enable && this->ui0.sect[SECT_RENDERING].state &&
		    (IsDifferent(opt_prev_.geomgroup, opt.geomgroup) || IsDifferent(opt_prev_.sitegroup, opt.sitegroup) ||
		     IsDifferent(opt_prev_.jointgroup, opt.jointgroup) || IsDifferent(opt_prev_.tendongroup, opt.tendongroup) ||
		     IsDifferent(opt_prev_.actuatorgroup, opt.actuatorgroup) || IsDifferent(opt_prev_.flexgroup, opt.flexgroup) ||
		     IsDifferent(opt_prev_.flexgroup, opt.flexgroup) || IsDifferent(opt_prev_.skingroup, opt.skingroup))) {
			mjui0_update_section(this, SECT_GROUP);
		}

		opt_prev_ = opt;
		cam_prev_ = cam;
	}

	if (pending_.ui_update_rendering) {
		if (this->ui0_enable && this->ui0.sect[SECT_RENDERING].state) {
			mjui0_update_section(this, SECT_RENDERING);
		}
		pending_.ui_update_rendering = false;
	}

	if (pending_.ui_update_joint) {
		if (this->ui1_enable && this->ui1.sect[SECT_JOINT].state) {
			mjui_update(SECT_JOINT, -1, &this->ui1, &this->uistate, &this->platform_ui->mjr_context());
		}
		pending_.ui_update_joint = false;
	}

	if (pending_.ui_remake_ctrl) {
		if (this->ui1_enable && this->ui1.sect[SECT_CONTROL].state) {
			this->ui1.nsect = SECT_CONTROL;
			MakeControlSection(this);
			this->ui1.nsect = NSECT1;
			UiModify(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());
		}
		pending_.ui_remake_ctrl = false;
	}

	if (pending_.ui_update_ctrl) {
		if (this->ui1_enable && this->ui1.sect[SECT_CONTROL].state) {
			mjui_update(SECT_CONTROL, -1, &this->ui1, &this->uistate, &this->platform_ui->mjr_context());
		}
		pending_.ui_update_ctrl = false;
	}

	// Render scene
	mjr_render(rect, &this->scn, &this->platform_ui->mjr_context());

	// Show last loading error
	if (this->load_error[0]) {
		mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOM, rect, this->load_error, nullptr, &this->platform_ui->mjr_context());
	}

	// show pause/loading label
	if (!this->run || this->loadrequest) {
		char label[30] = { '\0' };
		if (this->loadrequest) {
			std::snprintf(label, sizeof(label), "LOADING...");
		} else if (this->scrub_index == 0) {
			std::snprintf(label, sizeof(label), "PAUSE");
		} else {
			std::snprintf(label, sizeof(label), "PAUSE (%d)", this->scrub_index);
		}
		mjr_overlay(mjFONT_BIG, mjGRID_TOP, smallrect, label, nullptr, &this->platform_ui->mjr_context());
	}

	// Get desired and actual percent-of-realtime
	float desired_real_time = MujocoEnv::percentRealTime[env_->settings_.real_time_index];
	float actual_real_time  = 100.f / env_->sim_state_.measured_slowdown;

	// If running, check for misalignment of more than 10%
	float realtime_offset = mju_abs(actual_real_time - desired_real_time);
	bool misaligned       = this->run && realtime_offset > 0.1f * desired_real_time;

	// Make realtime overlay label
	char rtlabel[30] = { '\0' };
	if (desired_real_time != 100.0f || misaligned) {
		// Print desired realtime
		int labelsize;
		if (desired_real_time != -1) {
			labelsize = std::snprintf(rtlabel, sizeof(rtlabel), "%g%%", desired_real_time);
		} else {
			labelsize  = 1;
			rtlabel[0] = '+';
		}

		std::snprintf(rtlabel + labelsize, sizeof(rtlabel) - util::as_unsigned(labelsize), " (%-4.1f%%)",
		              actual_real_time);
	}

	// show real-time overlay
	if (rtlabel[0]) {
		mjr_overlay(mjFONT_BIG, mjGRID_TOPLEFT, smallrect, rtlabel, nullptr, &this->platform_ui->mjr_context());
	}

	// Show UI 0
	if (this->ui0_enable) {
		mjui_render(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
	}

	// Show UI 1
	if (this->ui1_enable) {
		mjui_render(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());
	}

	// Show help
	if (this->help) {
		mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, help_title, help_content, &this->platform_ui->mjr_context());
	}

	// Show info
	if (this->info) {
		mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, this->info_title, this->info_content,
		            &this->platform_ui->mjr_context());
	}

	// Show profiler
	if (this->profiler) {
		ShowProfiler(this, rect);
	}

	// Show sensor
	if (this->sensor) {
		ShowSensor(this, smallrect);
	}

	// Take screenshot, save to file
	if (this->screenshot_request.exchange(false)) {
		const unsigned int h = util::as_unsigned(uistate.rect[0].height);
		const unsigned int w = util::as_unsigned(uistate.rect[0].width);
		std::unique_ptr<unsigned char[]> rgb(new unsigned char[3 * w * h]);
		if (!rgb) {
			mju_error("Could not allocate memory for screenshot");
		}
		mjr_readPixels(rgb.get(), nullptr, uistate.rect[0], &this->platform_ui->mjr_context());

		// Flip up-down
		for (uint16_t r = 0; r < h / 2; ++r) {
			unsigned char *top_row    = &rgb[3 * w * r];
			unsigned char *bottom_row = &rgb[3 * w * (h - r - 1)];
			std::swap_ranges(top_row, top_row + 3 * w, bottom_row);
		}

		const std::string path = GetSavePath("screenshot.png");
		if (!path.empty()) {
			if (lodepng::encode(path, rgb.get(), w, h, LCT_RGB)) {
				mju_error("Could not save screenshot");
			} else {
				std::printf("Saved screenshot to %s\n", path.c_str());
			}
		}
	}

	// Finalize
	this->platform_ui->SwapBuffers();
}

void Viewer::RenderLoop()
{
	// Set timer callback (milliseconds)
	mjcb_time = Timer;

	// Init abstract visualization
	mjv_defaultCamera(&this->cam);
	mjv_defaultOption(&this->opt);
	InitializeProfiler(this);
	InitializeSensor(this);

	// Make empty scene
	if (!is_passive_) {
		mjv_defaultScene(&this->scn);
		mjv_makeScene(nullptr, &this->scn, kMaxGeom);
	}

	if (!this->platform_ui->IsGPUAccelerated()) {
		this->scn.flags[mjRND_SHADOW]     = 0;
		this->scn.flags[mjRND_REFLECTION] = 0;
	}

	// Select default font
	int fontscale = ComputeFontScale(*this->platform_ui);
	this->font    = fontscale / 50 - 1;

	// make empty context
	this->platform_ui->RefreshMjrContext(nullptr, fontscale);

	// Init state and UIs
	std::memset(&this->uistate, 0, sizeof(mjuiState));
	std::memset(&this->ui0, 0, sizeof(mjUI));
	std::memset(&this->ui1, 0, sizeof(mjUI));

	auto [buf_width, buf_height] = this->platform_ui->GetFramebufferSize();
	this->uistate.nrect          = 1;
	this->uistate.rect[0].width  = buf_width;
	this->uistate.rect[0].height = buf_height;

	this->ui0.spacing   = mjui_themeSpacing(this->spacing);
	this->ui0.color     = mjui_themeColor(this->color);
	this->ui0.predicate = UiPredicate;
	this->ui0.rectid    = 1;
	this->ui0.auxid     = 0;

	this->ui1.spacing   = mjui_themeSpacing(this->spacing);
	this->ui1.color     = mjui_themeColor(this->color);
	this->ui1.predicate = UiPredicate;
	this->ui1.rectid    = 2;
	this->ui1.auxid     = 1;

	// Set GUI adapter callbacks
	this->uistate.userdata = this;
	this->platform_ui->SetEventCallback(UiEvent);
	this->platform_ui->SetLayoutCallback(UiLayout);

	// Populate UIs with standard sections, open some sections initially
	this->ui0.userdata = this;
	this->ui1.userdata = this;
	mjui_add(&this->ui0, defFile);
	mjui_add(&this->ui0, this->def_option);
	mjui_add(&this->ui0, this->def_simulation);
	this->ui0.sect[0].state = 1;
	this->ui0.sect[1].state = 1;
	this->ui0.sect[2].state = 1;
	mjui_add(&this->ui0, this->def_watch);
	UiModify(&this->ui0, &this->uistate, &this->platform_ui->mjr_context());
	UiModify(&this->ui1, &this->uistate, &this->platform_ui->mjr_context());

	// Set VSync to initial value
	this->platform_ui->SetVSync(this->vsync);

	frames_          = 0;
	last_fps_update_ = Clock::now();

	// Run event loop
	while (!this->platform_ui->ShouldCloseWindow() && !this->exit_request.load()) {
		{
			const MutexLock lock(this->mtx);

			// Load model (not on first pass, to show "loading" label)
			if (this->loadrequest == 1) {
				this->LoadOnRenderThread();
			} else if (this->loadrequest == 2) {
				this->loadrequest = 1;
			}

			// Poll and handle events
			this->platform_ui->PollEvents();

			// upload assets if requested
			bool upload_notify = false;
			if (hfield_upload_ != -1) {
				mjr_uploadHField(m_.get(), &platform_ui->mjr_context(), hfield_upload_);
				hfield_upload_ = -1;
				upload_notify  = true;
			}
			if (mesh_upload_ != -1) {
				mjr_uploadMesh(m_.get(), &platform_ui->mjr_context(), mesh_upload_);
				mesh_upload_  = -1;
				upload_notify = true;
			}
			if (texture_upload_ != -1) {
				mjr_uploadTexture(m_.get(), &platform_ui->mjr_context(), texture_upload_);
				texture_upload_ = -1;
				upload_notify   = true;
			}
			if (upload_notify) {
				cond_upload_.notify_all();
			}

			// Update scene, doing a full sync if the environment is not busy loading
			if (!this->is_passive_ && this->env_->getOperationalStatus() == 0) {
				Sync();
			} else {
				scnstate_.data.warning[mjWARN_VGEOMFULL].number +=
				    mjv_updateSceneFromState(&scnstate_, &this->opt, &this->pert, &this->cam, mjCAT_ALL, &this->scn);
			}
		} // MutexLock (unblocks simulation thread)

		// Render while simulation is running
		this->Render();

		// Update FPS stat, at most 5 times per second
		auto now        = Clock::now();
		double interval = Seconds(now - last_fps_update_).count();
		++frames_;
		if (interval > 0.2) {
			last_fps_update_ = now;
			fps_             = frames_ / interval;
			frames_          = 0;
		}
	}

	const MutexLock lock(this->mtx);
	mjv_freeScene(&this->scn);
	if (is_passive_) {
		mjv_freeSceneState(&scnstate_);
	}

	this->exit_request.store(2);
	env_->disconnectViewer(this);
}

void Viewer::AddToHistory()
{
	if (history_.empty()) {
		return;
	}

	// circular increment of cursor
	history_cursor_ = (history_cursor_ + 1) % nhistory_;

	// add state at cursor
	mjtNum *state = &history_[state_size_ * history_cursor_];
	mj_getState(m_.get(), d_.get(), state, mjSTATE_INTEGRATION);
}

void Viewer::UpdateHField(int hfieldid)
{
	MutexLock lock(this->mtx);
	if (!m_ || hfieldid < 0 || hfieldid >= m_->nhfield) {
		return;
	}
	hfield_upload_ = hfieldid;
	cond_upload_.wait(lock, [this]() { return hfield_upload_ == -1; });
}

void Viewer::UpdateMesh(int meshid)
{
	MutexLock lock(this->mtx);
	if (!m_ || meshid < 0 || meshid >= m_->nmesh) {
		return;
	}
	mesh_upload_ = meshid;
	cond_upload_.wait(lock, [this]() { return mesh_upload_ == -1; });
}

void Viewer::UpdateTexture(int texid)
{
	MutexLock lock(this->mtx);
	if (!m_ || texid < 0 || texid >= m_->ntex) {
		return;
	}
	texture_upload_ = texid;
	cond_upload_.wait(lock, [this]() { return texture_upload_ == -1; });
}

} // namespace mujoco_ros
