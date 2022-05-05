#include <mujoco_ros/mujoco_sim.h>
#include <mujoco_ros/plugin_utils.h>

#include <rosgraph_msgs/Clock.h>

namespace MujocoSim {

using namespace detail;

// REVEIW: Still needed?
int jointName2id(const std::string &joint_name)
{
	return mj_name2id(m_.get(), mjOBJ_JOINT, joint_name.c_str());
}

// REVIEW: Still needed?
bool isUp(void)
{
	return !settings_.exitrequest;
}

void init(std::string modelfile)
{
	nh_.reset(new ros::NodeHandle("~"));

	bool unpause;
	nh_->param<bool>("unpause", unpause, true);
	if (unpause)
		settings_.run = 1;

	// std::string modelfile;
	// nh_->getParam("modelfile", modelfile);

	// Print version, check compatibility
	ROS_INFO("MuJoCo Pro library version %.2lf\n", 0.01 * mj_version());
	if (mjVERSION_HEADER != mj_version()) {
		ROS_ERROR_NAMED("mujoco", "Headers and library have different versions");
		mju_error("Headers and library have different versions");
	}

	nh_->param<bool>("visualize", vis_, true);

	if (vis_) {
		initVisual();
	} else {
		ROS_DEBUG_NAMED("mujoco", "Will run in headless mode!");
	}

	pub_clock_ = nh_->advertise<rosgraph_msgs::Clock>("/clock", 10);

	// If not already set, set use_sim_time param manually
	if (!(nh_->hasParam("/use_sim_time")))
		nh_->setParam("/use_sim_time", true);

	if (!modelfile.empty()) {
		std::strcpy(filename_, modelfile.c_str());
		settings_.loadrequest = 2;
	}

	nh_->getParam("initial_joint_positions/joint_map", init_joint_pos_map_);

	for (auto const &[name, value] : init_joint_pos_map_) {
		int id = jointName2id(name);
		if (id == -1) {
			ROS_WARN_STREAM_NAMED("mujoco", "Joint with name '"
			                                    << name << "' could not be found. Initial joint position cannot be set!");
			continue;
		}

		ROS_DEBUG_STREAM_NAMED("mujoco", "Joint name '" << name << "' (mjID '" << id << "') setting to value " << value);

		setJointPosition(value, id);
	}

	setupCallbacks();
	plugin_util::loadRegisteredPlugins(*nh_);

	std::thread simthread(simulate);
	eventloop();

	ROS_DEBUG_NAMED("mujoco", "Event loop terminated");
	settings_.exitrequest = 1;
	simthread.join();

	if (vis_)
		uiClearCallback(window_);

	ROS_DEBUG_NAMED("mujoco", "Sim thread terminated");

	sim_mtx.unlock();
	render_mtx.unlock();

	free(ctrlnoise_);
	d_.reset();
	m_.reset();
	mjv_freeScene(&scn_);
	mjr_freeContext(&con_);
	ROS_DEBUG_NAMED("mujoco", "Cleanup done");
}

// Get current position, velocity, acceleration, and effort of a specific joint
std::array<double, 4> getJointData(const int &joint_id)
{
	return { d_->qpos[m_->jnt_qposadr[joint_id]], d_->qvel[m_->jnt_dofadr[joint_id]], d_->qacc[m_->jnt_dofadr[joint_id]],
		      d_->qfrc_applied[m_->jnt_dofadr[joint_id]] };
}

// REVIEW: Still needed?
double getBodyMass(const int &body_id)
{
	return m_->body_mass[body_id];
}

// REVIEW: Still needed?
bool isSimReady(void)
{
	// Is model loaded?
	if (!m_)
		return false;

	// Is data loaded?
	if (!d_)
		return false;

	// We expect at least one controllable joint
	if (m_->njnt < 1)
		return false;

	return true;
}

// REVIEW: Serve as ROS Callback?
void requestExternalShutdown(void)
{
	settings_.exitrequest = 1;
}

// REVIEW: Still needed?
std::array<double, 3> getGravity(void)
{
	return { m_->opt.gravity[0], m_->opt.gravity[1], m_->opt.gravity[2] };
}

// Review: Still needed?
void setJointEffort(const double &command, const int &joint_id)
{
	d_->qfrc_applied[m_->jnt_dofadr[joint_id]] = command;
}

// Review: Still needed?
void setJointPosition(const double &pos, const int &joint_id)
{
	d_->qpos[m_->jnt_qposadr[joint_id]]        = pos;
	d_->qvel[m_->jnt_dofadr[joint_id]]         = 0;
	d_->qfrc_applied[m_->jnt_dofadr[joint_id]] = 0;
}

namespace detail {

void publishSimTime(void)
{
	rosgraph_msgs::Clock ros_time;
	ros_time.clock.fromSec(d_->time);
	pub_clock_.publish(ros_time);
	last_time_ = d_->time;
}

void eventloop(void)
{
	while ((!settings_.exitrequest && !vis_) || (!settings_.exitrequest && !glfwWindowShouldClose(window_))) {
		// Critical operations
		sim_mtx.lock();

		if (settings_.loadrequest == 1) {
			loadModel();
		} else if (settings_.loadrequest > 1) {
			settings_.loadrequest = 1;
		}

		if (vis_) {
			// Handle events (via callbacks)
			glfwPollEvents();

			// Prepare to render
			prepare();
		}

		// Allow simulation thread to run
		sim_mtx.unlock();

		// render while sim is running
		if (vis_)
			render(window_);
	}
}

void simulate(void)
{
	int num_steps;
	nh_->param<int>("num_steps", num_steps, -1);

	ROS_DEBUG_STREAM_COND_NAMED(num_steps > 0, "mujoco", "Simulation will terminate after " << num_steps << " steps");

	// cpu-sim syncronization point
	double cpusync = 0;
	mjtNum simsync = 0;

	while (!settings_.exitrequest && num_steps != 0) {
		if (d_) {
			publishSimTime();
		}

		if (settings_.run && settings_.busywait) {
			std::this_thread::yield();
		} else {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		sim_mtx.lock();

		if (m_) {
			if (settings_.run) {
				// Count steps until termination
				if (num_steps > 0) {
					num_steps--;
					ROS_INFO_COND_NAMED(num_steps == 0, "mujoco", "running last sim step before termination!");
				}

				double tmstart = glfwGetTime();

				// Inject noise
				if (settings_.ctrlnoisestd) {
					// Convert rate and scale to discrete time given current timestep
					mjtNum rate  = mju_exp(-m_->opt.timestep / settings_.ctrlnoiserate);
					mjtNum scale = settings_.ctrlnoisestd * mju_sqrt(1 - rate * rate);

					for (int i = 0; i < m_->nu; i++) {
						// Update noise
						ctrlnoise_[i] = rate * ctrlnoise_[i] + scale * mju_standardNormal(nullptr);
						// Apply noise
						d_->ctrl[i] = ctrlnoise_[i];
					}
				}

				// Out-of-sync (for any reason)
				mjtNum offset = mju_abs((d_->time * settings_.slow_down - simsync) - (tmstart - cpusync));
				if (d_->time * settings_.slow_down < simsync || tmstart < cpusync || cpusync == 0 ||
				    offset > syncmisalign_ * settings_.slow_down || settings_.speed_changed) {
					// Re-sync

					// ROS_WARN_STREAM_NAMED("mujoco", "Out of sync by " << offset << ". Re-syncing...");

					cpusync                 = tmstart;
					simsync                 = d_->time * settings_.slow_down;
					settings_.speed_changed = false;

					// Clear old perturbations, apply new
					mju_zero(d_->xfrc_applied, 6 * m_->nbody);
					mjv_applyPerturbPose(m_.get(), d_.get(), &pert_, 0); // Move mocap bodies only
					mjv_applyPerturbForce(m_.get(), d_.get(), &pert_);

					// Run single step, let next iteration deal with timing
					mj_step(m_.get(), d_.get());
					publishSimTime();
				} else { // in-sync
					// Step while simtime lags behind cputime , and within safefactor
					while ((d_->time * settings_.slow_down - simsync) < (glfwGetTime() - cpusync) &&
					       (glfwGetTime() - tmstart) < refreshfactor_ / vmode_.refreshRate) {
						// clear old perturbations, apply new
						mju_zero(d_->xfrc_applied, 6 * m_->nbody);
						mjv_applyPerturbPose(m_.get(), d_.get(), &pert_, 0); // Move mocap bodies only
						mjv_applyPerturbForce(m_.get(), d_.get(), &pert_);

						// Run mj_step
						mjtNum prevtm = d_->time * settings_.slow_down;
						mj_step(m_.get(), d_.get());
						publishSimTime();

						// break on reset
						if (d_->time * settings_.slow_down < prevtm) {
							break;
						}
					}
				}
			} else { // Paused
				mjv_applyPerturbPose(m_.get(), d_.get(), &pert_, 1); // Move mocap and dynamic bodies

				// Run mj_forward, to update rendering and joint sliders
				mj_forward(m_.get(), d_.get());
			}
		}
		sim_mtx.unlock();
	}
	// Requests eventloop shutdown in case we ran out of simulation steps to use
	settings_.exitrequest = 1;
}

void initVisual()
{
	if (!glfwInit()) {
		ROS_ERROR_NAMED("mujoco", "Could not initialize GLFW");
		mju_error("Could not initialize GLFW");
	}
	mjcb_time = timer;

	// multisampling
	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_VISIBLE, 1);

	// get videomode and save
	vmode_ = *glfwGetVideoMode(glfwGetPrimaryMonitor());

	// create window
	window_ = glfwCreateWindow((2 * vmode_.width) / 3, (2 * vmode_.height) / 3, "Simulate", NULL, NULL);

	if (!window_) {
		glfwTerminate();
		ROS_ERROR_NAMED("mujoco", "Could not create window");
		mju_error("could not create window");
	}

	// save window position and size
	glfwGetWindowPos(window_, windowpos_, windowpos_ + 1);
	glfwGetWindowSize(window_, windowsize_, windowsize_ + 1);

	// make context current, set v-sync
	glfwMakeContextCurrent(window_);
	glfwSwapInterval(settings_.vsync);

	// init abstract visualization
	mjv_defaultCamera(&cam_);
	mjv_defaultOption(&vopt_);
	profilerInit();
	sensorInit();

	// make empty scene
	mjv_defaultScene(&scn_);
	mjv_makeScene(NULL, &scn_, maxgeom_);

	// select default font
	int fontscale  = uiFontScale(window_);
	settings_.font = fontscale / 50 - 1;

	// make empty context
	mjr_defaultContext(&con_);
	mjr_makeContext(NULL, &con_, fontscale);

	// set GLFW callbacks
	uiSetCallback(window_, &uistate_, uiEvent, uiLayout);
	glfwSetWindowRefreshCallback(window_, render);
	glfwSetDropCallback(window_, drop);

	// init state und uis
	std::memset(&uistate_, 0, sizeof(mjuiState));
	std::memset(&ui0_, 0, sizeof(mjUI));
	std::memset(&ui1_, 0, sizeof(mjUI));
	ui0_.spacing   = mjui_themeSpacing(settings_.spacing);
	ui0_.color     = mjui_themeColor(settings_.color);
	ui0_.predicate = uiPredicate;
	ui0_.rectid    = 1;
	ui0_.auxid     = 0;

	ui1_.spacing   = mjui_themeSpacing(settings_.spacing);
	ui1_.color     = mjui_themeColor(settings_.color);
	ui1_.predicate = uiPredicate;
	ui1_.rectid    = 2;
	ui1_.auxid     = 1;

	// populate uis with standard sections
	mjui_add(&ui0_, defFile);
	mjui_add(&ui0_, defOption);
	mjui_add(&ui0_, defSimulation);
	mjui_add(&ui0_, defWatch);
	uiModify(window_, &ui0_, &uistate_, &con_);
	uiModify(window_, &ui1_, &uistate_, &con_);
}

void loadModel(void)
{
	settings_.loadrequest = 0;
	ROS_DEBUG_NAMED("mujoco", "Loading model...");
	// Make sure filename is given
	if (!filename_) {
		return;
	}

	// Load and compile
	char error[500] = "";
	mjModel *mnew   = 0;
	if (mju::strlen_arr(filename_) > 4 && !std::strncmp(filename_ + mju::strlen_arr(filename_) - 4, ".mjb",
	                                                    mju::sizeof_arr(filename_) - mju::strlen_arr(filename_) + 4)) {
		mnew = mj_loadModel(filename_, NULL);
		if (!mnew) {
			mju::strcpy_arr(error, "could not load binary model");
		}
	} else {
		mnew = mj_loadXML(filename_, NULL, error, 500);
	}
	if (!mnew) {
		std::printf("%s\n", error);
		return;
	}

	// Compiler warning: print and pause
	if (error[0]) {
		// mj_forward() will print the warning message
		ROS_WARN_NAMED("mujoco", "Model compiled, but simulation warning (paused): \n %s\n\n", error);
		std::printf("Model compiled, but simulation warning (paused): \n %s\n\n", error);
		settings_.run = 0;
	}

	ROS_DEBUG_NAMED("mujoco", "replacing model and data ...");
	// Delete old model, assign new
	m_.reset(mnew);
	d_.reset(mj_makeData(m_.get()));
	mj_forward(m_.get(), d_.get());

	ROS_DEBUG_NAMED("mujoco", "resetting noise ...");
	// Allocate ctrlnoise
	free(ctrlnoise_);
	ctrlnoise_ = (mjtNum *)malloc(sizeof(mjtNum) * m_->nu);
	mju_zero(ctrlnoise_, m_->nu);

	ROS_DEBUG_NAMED("mujoco", "creating scene ...");
	// Re-create scene and context
	mjv_makeScene(m_.get(), &scn_, maxgeom_);
	if (vis_)
		mjr_makeContext(m_.get(), &con_, 50 * (settings_.font + 1));

	ROS_DEBUG_NAMED("mujoco", "clear perturb ...");
	// Clear perturbation state
	pert_.active     = 0;
	pert_.select     = 0;
	pert_.skinselect = -1;

	// Align and scale view unless reloading the same file
	if (mju::strcmp_arr(filename_, previous_filename_)) {
		alignScale();
		mju::strcpy_arr(previous_filename_, filename_);
	}

	ROS_DEBUG_NAMED("mujoco", "updating scene...");
	// Update scene
	mjv_updateScene(m_.get(), d_.get(), &vopt_, &pert_, &cam_, mjCAT_ALL, &scn_);

	if (vis_) {
		// Set window title to model name
		if (window_ && m_->names) {
			char title[200] = "Simulate : ";
			mju::strcat_arr(title, m_->names);
			glfwSetWindowTitle(window_, title);
		}

		// Set keyframe range and divisions
		ui0_.sect[SECT_SIMULATION].item[5].slider.range[0]  = 0;
		ui0_.sect[SECT_SIMULATION].item[5].slider.range[1]  = mjMAX(0, m_->nkey - 1);
		ui0_.sect[SECT_SIMULATION].item[5].slider.divisions = mjMAX(1, m_->nkey - 1);

		// Rebuild UI Sections
		makeSections();

		// Full UI update
		uiModify(window_, &ui0_, &uistate_, &con_);
		uiModify(window_, &ui1_, &uistate_, &con_);
	}

	ROS_DEBUG_NAMED("mujoco", "updating settings ...");
	updateSettings();
	ROS_DEBUG_NAMED("mujoco", "settings updated ...");
}

int uiPredicate(int category, void *userdata)
{
	switch (category) {
		case 2: // require model
			return (m_ != NULL);

		case 3: //
			return (m_ && m_->nkey);

		case 4:
			return (m_ && !settings_.run);

		default:
			return 1;
	}
}

void drop(GLFWwindow *window, int count, const char **paths)
{
	// Ignore file drop

	// make sure list is non-empty
	// if (count > 0) {
	// 	mju::strcpy_arr(filename_, paths[0]);
	// }
}

void render(GLFWwindow *window)
{
	render_mtx.lock();

	// get 3D rectangle and reduced for profiler
	mjrRect rect      = uistate_.rect[3];
	mjrRect smallrect = rect;

	if (settings_.profiler) {
		smallrect.width = rect.width - rect.width / 4;
	}

	// no model
	if (!m_) {
		// blank screen
		mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);

		// label
		if (settings_.loadrequest)
			mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect, "loading", NULL, &con_);

		//// We don't want this. A model should be loaded over services or during start
		// else
		// mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, "Drag-and-drop model file here", 0, &con_);

		// render uis
		if (settings_.ui0)
			mjui_render(&ui0_, &uistate_, &con_);
		if (settings_.ui1)
			mjui_render(&ui1_, &uistate_, &con_);

		// finalize
		glfwSwapBuffers(window);
	}

	// render scene
	mjr_render(rect, &scn_, &con_);

	// show pause/loading label
	if (!settings_.run || settings_.loadrequest)
		mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect, settings_.loadrequest ? "loading" : "pause", NULL, &con_);

	// show realtime label
	if (settings_.run && settings_.slow_down != 1) {
		std::string realtime_label = "1/" + std::to_string(settings_.slow_down) + "x";
		mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect, realtime_label.c_str(), NULL, &con_);
	}

	// show ui 0
	if (settings_.ui0)
		mjui_render(&ui0_, &uistate_, &con_);

	// show ui 1
	if (settings_.ui1)
		mjui_render(&ui1_, &uistate_, &con_);

	// show help
	if (settings_.help)
		mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, help_title_, help_content_, &con_);

	// show info
	if (settings_.info)
		mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, info_title_, info_content_, &con_);

	// show profiler
	if (settings_.profiler)
		profilerShow(rect);

	// show sensor
	if (settings_.sensor)
		sensorShow(smallrect);

	// finalize
	glfwSwapBuffers(window);
	render_mtx.unlock();
}

// Set window layout
void uiLayout(mjuiState *state)
{
	mjrRect *rect = state->rect;

	// set number of rectangles
	state->nrect = 4;

	// rect 0: entire framebuffer
	rect[0].left   = 0;
	rect[0].bottom = 0;
	glfwGetFramebufferSize(window_, &rect[0].width, &rect[0].height);

	// rect 1: UI 0
	rect[1].left   = 0;
	rect[1].width  = settings_.ui0 ? ui0_.width : 0;
	rect[1].bottom = 0;
	rect[1].height = rect[0].height;

	// rect 2: UI 1
	rect[2].width  = settings_.ui1 ? ui1_.width : 0;
	rect[2].left   = mjMAX(0, rect[0].width - rect[2].width);
	rect[2].bottom = 0;
	rect[2].height = rect[0].height;

	// rect 3: 3D plot (everything else is an overlay)
	rect[3].left   = rect[1].width;
	rect[3].width  = mjMAX(0, rect[0].width - rect[1].width - rect[2].width);
	rect[3].bottom = 0;
	rect[3].height = rect[0].height;
}

// Handle UI event
void uiEvent(mjuiState *state)
{
	int i;
	char err[200];

	// call UI 0 if event is directed to it
	if ((state->dragrect == ui0_.rectid) || (state->dragrect == 0 && state->mouserect == ui0_.rectid) ||
	    state->type == mjEVENT_KEY) {
		// process UI event
		mjuiItem *it = mjui_event(&ui0_, state, &con_);

		// file section
		if (it && it->sectionid == SECT_FILE) {
			switch (it->itemid) {
				case 0: // save xml
					if (!mj_saveLastXML("mjmodel.xml", m_.get(), err, 200))
						ROS_ERROR("Save XML error: %s", err);
					break;

				case 1: // Save mjb
					mj_saveModel(m_.get(), "mjmodel.mjb", NULL, 0);
					break;

				case 2: // print model
					mj_printModel(m_.get(), "MJMODEL.TXT");
					break;

				case 3: // print data
					mj_printData(m_.get(), d_.get(), "MJDATA.TXT");
					break;

				case 4: // Quit
					settings_.exitrequest = 1;
					break;
			}
		}

		// option section
		else if (it && it->sectionid == SECT_OPTION) {
			switch (it->itemid) {
				case 0: // Spacing
					ui0_.spacing = mjui_themeSpacing(settings_.spacing);
					ui1_.spacing = mjui_themeSpacing(settings_.spacing);
					break;

				case 1: // Color
					ui0_.color = mjui_themeColor(settings_.color);
					ui1_.color = mjui_themeColor(settings_.color);
					break;

				case 2: // Font
					mjr_changeFont(50 * (settings_.font + 1), &con_);
					break;

				case 9: // Full screen
					if (glfwGetWindowMonitor(window_)) {
						// restore window from saved data
						glfwSetWindowMonitor(window_, NULL, windowpos_[0], windowpos_[1], windowsize_[0], windowsize_[1], 0);
					} else { // currently windowed: switch to fullscreen
						// save window data
						glfwGetWindowPos(window_, windowpos_, windowpos_ + 1);
						glfwGetWindowSize(window_, windowsize_, windowsize_ + 1);

						// switch
						glfwSetWindowMonitor(window_, glfwGetPrimaryMonitor(), 0, 0, vmode_.width, vmode_.height,
						                     vmode_.refreshRate);
					}

					// reinstate vsync, just in case
					glfwSwapInterval(settings_.vsync);
					break;

				case 10: // Vertical sync
					glfwSwapInterval(settings_.vsync);
					break;
			}

			// modify UI
			uiModify(window_, &ui0_, state, &con_);
			uiModify(window_, &ui1_, state, &con_);
		}

		// simulation section
		else if (it && it->sectionid == SECT_SIMULATION) {
			switch (it->itemid) {
				case 1: // reset
					if (m_) {
						mj_resetData(m_.get(), d_.get());
						d_->time = last_time_;
						mj_forward(m_.get(), d_.get());
						profilerUpdate();
						sensorUpdate();
						updateSettings();
					}
					break;

				case 2: // Reload
					settings_.loadrequest = 1;
					break;

				case 3: // Align
					alignScale();
					updateSettings();
					break;

				case 4: // Copy Pose
					copyKey();
					break;

				case 5: // Adjust key
				case 6: // Reset to key
					i        = settings_.key;
					d_->time = m_->key_time[i];
					mju_copy(d_->qpos, m_->key_qpos + i * m_->nq, m_->nq);
					mju_copy(d_->qvel, m_->key_qvel + i * m_->nv, m_->nv);
					mju_copy(d_->act, m_->key_act + i * m_->na, m_->na);
					mju_copy(d_->mocap_pos, m_->key_mpos + i * 3 * m_->nmocap, 3 * m_->nmocap);
					mju_copy(d_->mocap_quat, m_->key_mquat + i * 4 * m_->nmocap, 4 * m_->nmocap);
					mj_forward(m_.get(), d_.get());
					profilerUpdate();
					sensorUpdate();
					updateSettings();
					break;

				case 7: // Set key
					i               = settings_.key;
					m_->key_time[i] = d_->time;
					mju_copy(m_->key_qpos + i * m_->nq, d_->qpos, m_->nq);
					mju_copy(m_->key_qvel + i * m_->nv, d_->qvel, m_->nv);
					mju_copy(m_->key_act + i * m_->na, d_->act, m_->na);
					mju_copy(m_->key_mpos + i * 3 * m_->nmocap, d_->mocap_pos, 3 * m_->nmocap);
					mju_copy(m_->key_mquat + i * 4 * m_->nmocap, d_->mocap_quat, 4 * m_->nmocap);
					break;
			}
		}

		// Physics section
		else if (it && it->sectionid == SECT_PHYSICS) {
			// Update disable flags in mjOption
			m_->opt.disableflags = 0;
			for (i = 0; i < mjNDISABLE; i++) {
				if (settings_.disable[i]) {
					m_->opt.disableflags |= (1 << i);
				}
			}

			// Update enable flags in mjOption
			m_->opt.enableflags = 0;
			for (i = 0; i < mjNENABLE; i++) {
				if (settings_.enable[i]) {
					m_->opt.enableflags |= (1 << i);
				}
			}
		}

		// Rendering section
		else if (it && it->sectionid == SECT_RENDERING) {
			// Set camera in mjvCamera
			if (settings_.camera == 0) {
				cam_.type = mjCAMERA_FREE;
			} else if (settings_.camera == 1) {
				if (pert_.select > 0) {
					cam_.type        = mjCAMERA_TRACKING;
					cam_.trackbodyid = pert_.select;
					cam_.fixedcamid  = -1;
				} else {
					cam_.type        = mjCAMERA_FREE;
					settings_.camera = 0;
					mjui_update(SECT_RENDERING, -1, &ui0_, &uistate_, &con_);
				}
			} else {
				cam_.type       = mjCAMERA_FIXED;
				cam_.fixedcamid = settings_.camera - 2;
			}
			// Print floating camera as MJCF element
			if (it->itemid == 3) {
				printCamera(scn_.camera);
			}
		}

		// Group section
		else if (it && it->sectionid == SECT_GROUP) {
			// Remake joint section if joint group changed
			if (it->name[0] == 'J' && it->name[1] == 'o') {
				ui1_.nsect = SECT_JOINT;
				makeJoint(ui1_.sect[SECT_JOINT].state);
				ui1_.nsect = NSECT1;
				uiModify(window_, &ui1_, state, &con_);
			}

			// Remake control section if actuator group changed
			if (it->name[0] == 'A' && it->name[1] == 'c') {
				ui1_.nsect = SECT_CONTROL;
				makeControl(ui1_.sect[SECT_CONTROL].state);
				ui1_.nsect = NSECT1;
				uiModify(window_, &ui1_, state, &con_);
			}
		}

		// Stop if UI processed event
		if (it != NULL || (state->type == mjEVENT_KEY && state->key == 0)) {
			return;
		}
	}

	// Call UI 1 if event is directed to it
	if ((state->dragrect == ui1_.rectid) || (state->dragrect == 0 && state->mouserect == ui1_.rectid) ||
	    state->type == mjEVENT_KEY) {
		// Process UI event
		mjuiItem *it = mjui_event(&ui1_, state, &con_);

		// control section
		if (it && it->sectionid == SECT_CONTROL) {
			// clear controls
			if (it->itemid == 0) {
				mju_zero(d_->ctrl, m_->nu);
				mjui_update(SECT_CONTROL, -1, &ui1_, &uistate_, &con_);
			}
		}

		// Stop if UI processed event
		if (it != NULL || (state->type == mjEVENT_KEY && state->key == 0)) {
			return;
		}
	}

	// Short not handled by UI
	if (state->type == mjEVENT_KEY && state->key != 0) {
		switch (state->key) {
			case ' ': // Mode
				if (m_) {
					settings_.run = 1 - settings_.run;
					pert_.active  = 0;
					mjui_update(-1, -1, &ui0_, state, &con_);
				}
				break;

			case mjKEY_RIGHT: // Step forward
				if (m_ && !settings_.run) {
					clearTimers();
					mj_step(m_.get(), d_.get());
					publishSimTime();
					profilerUpdate();
					sensorUpdate();
					updateSettings();
				}
				break;

			case mjKEY_LEFT: // Step back
				ROS_DEBUG_THROTTLE_NAMED(1, "mujoco",
				                         "Stepping backwards is disabled as rostime should never run backwards.");
				break;

			case mjKEY_DOWN: // Step forward 100
				if (m_ && !settings_.run) {
					clearTimers();
					for (i = 0; i < 100; i++) {
						mj_step(m_.get(), d_.get());
						publishSimTime();
					}
					profilerUpdate();
					sensorUpdate();
					updateSettings();
				}
				break;

			case mjKEY_UP: // Step backward 100
				ROS_DEBUG_THROTTLE_NAMED(1, "mujoco",
				                         "Stepping backwards is disabled as rostime should never run backwards.");
				break;

			case mjKEY_PAGE_UP: // Select parent body
				if (m_ && pert_.select > 0) {
					pert_.select     = m_->body_parentid[pert_.select];
					pert_.skinselect = -1;

					// Stop perturbation if world reached
					if (pert_.select <= 0) {
						pert_.active = 0;
					}
				}
				break;

			case '-': //  Slow down
				if (settings_.slow_down < max_slow_down_ && !state->shift) {
					settings_.slow_down *= 2;
					settings_.speed_changed = true;
				}
				break;

			case '=': // Speed up
				if (settings_.slow_down > 1 && !state->shift) {
					settings_.slow_down /= 2;
					settings_.speed_changed = true;
				}
				break;
		}

		return;
	}

	// 3D Scroll
	if (state->type == mjEVENT_SCROLL && state->mouserect == 3 && m_) {
		// Emulate vertical mouse motion = 5% of window height
		mjv_moveCamera(m_.get(), mjMOUSE_ZOOM, 0, -0.05 * state->sy, &scn_, &cam_);

		return;
	}

	// 3D press
	if (state->type == mjEVENT_PRESS && state->mouserect == 3 && m_) {
		// Set perturbation
		int newperturb = 0;
		if (state->control && pert_.select > 0) {
			// right: translate; left: rotate
			if (state->right) {
				newperturb = mjPERT_TRANSLATE;
			} else if (state->left) {
				newperturb = mjPERT_ROTATE;
			}

			// Perturbation onset: reset reference
			if (newperturb && !pert_.active) {
				mjv_initPerturb(m_.get(), d_.get(), &scn_, &pert_);
			}
		}
		pert_.active = newperturb;

		// Handle double-click
		if (state->doubleclick) {
			// Determine selection mode
			int selmode;
			if (state->button == mjBUTTON_LEFT) {
				selmode = 1;
			} else if (state->control) {
				selmode = 3;
			} else {
				selmode = 2;
			}

			// Find geom and 3D click point, get corresponding body
			mjrRect r = state->rect[3];
			mjtNum selpnt[3];
			int selgeom, selskin;
			int selbody = mjv_select(m_.get(), d_.get(), &vopt_, (mjtNum)r.width / (mjtNum)r.height,
			                         (mjtNum)(state->x - r.left) / (mjtNum)r.width,
			                         (mjtNum)(state->y - r.bottom) / (mjtNum)r.height, &scn_, selpnt, &selgeom, &selskin);

			// Set lookat point, start tracking is requested
			if (selmode == 2 || selmode == 3) {
				// Copy selpnt if anything clicked
				if (selbody >= 0) {
					mju_copy3(cam_.lookat, selpnt);
				}

				// Switch to tracking camera if dynamic body clicked
				if (selmode == 3 && selbody > 0) {
					// Mujoco camera
					cam_.type        = mjCAMERA_TRACKING;
					cam_.trackbodyid = selbody;
					cam_.fixedcamid  = -1;

					// UI camera
					settings_.camera = 1;
					mjui_update(SECT_RENDERING, -1, &ui0_, &uistate_, &con_);
				}
			}

			// Set body selection
			else {
				if (selbody >= 0) {
					// Record selection
					pert_.select     = selbody;
					pert_.skinselect = selskin;

					// Compute localpos
					mjtNum tmp[3];
					mju_sub3(tmp, selpnt, d_->xpos + 3 * pert_.select);
					mju_mulMatTVec(pert_.localpos, d_->xmat + 9 * pert_.select, tmp, 3, 3);
				} else {
					pert_.select     = 0;
					pert_.skinselect = 0;
				}
			}

			// Stop perturbation on select
			pert_.active = 0;
		}

		return;
	}

	// 3D release
	if (state->type == mjEVENT_RELEASE && state->dragrect == 3 && m_) {
		// Stop perturbation
		pert_.active = 0;

		return;
	}

	// 3D move
	if (state->type == mjEVENT_MOVE && state->dragrect == 3 && m_) {
		// Determine action base on mouse button
		mjtMouse action;
		if (state->right) {
			action = state->shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
		} else if (state->left) {
			action = state->shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
		} else {
			action = mjMOUSE_ZOOM;
		}

		// Move perturb or camera
		mjrRect r = state->rect[3];
		if (pert_.active) {
			mjv_movePerturb(m_.get(), d_.get(), action, state->dx / r.height, -state->dy / r.height, &scn_, &pert_);
		} else {
			mjv_moveCamera(m_.get(), action, state->dx / r.height, -state->dy / r.height, &scn_, &cam_);
		}

		return;
	}
}

// print current camera as MJCF specification
void printCamera(mjvGLCamera *camera)
{
	mjtNum cam_right[3];
	mjtNum cam_forward[3];
	mjtNum cam_up[3];
	mju_f2n(cam_forward, camera[0].forward, 3);
	mju_f2n(cam_up, camera[0].up, 3);
	mju_cross(cam_right, cam_forward, cam_up);
	std::printf("<camera pos=\"%.3f %.3f %.3f\" xyaxes=\"%.3f %.3f %.3f %.3f %.3f %.3f\"/>\n",
	            (camera[0].pos[0] + camera[1].pos[0]) / 2, (camera[0].pos[1] + camera[1].pos[1]) / 2,
	            (camera[0].pos[2] + camera[1].pos[2]) / 2, cam_right[0], cam_right[1], cam_right[2], camera[0].up[0],
	            camera[0].up[1], camera[0].up[2]);
}

// copy qpos to clipboard as key
void copyKey(void)
{
	char clipboard[5000] = "<key qpos='";
	char buf[200];

	// prepare string
	for (int i = 0; i < m_->nq; i++) {
		mju::sprintf_arr(buf, i == m_->nq - 1 ? "%g" : "%g ", d_->qpos[i]);
		mju::strcat_arr(clipboard, buf);
	}
	mju::strcat_arr(clipboard, "'/>");

	// copy to clipboard
	glfwSetClipboardString(window_, clipboard);
}

void sensorInit(void)
{
	// set figure to default
	mjv_defaultFigure(&figsensor_);
	figsensor_.figurergba[3] = 0.5f;

	// set flags
	figsensor_.flg_extend    = 1;
	figsensor_.flg_barplot   = 1;
	figsensor_.flg_symmetric = 1;

	// title
	mju::strcpy_arr(figsensor_.title, "Sensor data");

	// y-tick number format
	mju::strcpy_arr(figsensor_.yformat, "%0f");

	// grid size
	figsensor_.gridsize[0] = 2;
	figsensor_.gridsize[1] = 3;

	// minimum range
	figsensor_.range[0][0] = 0;
	figsensor_.range[0][0] = 0;
	figsensor_.range[1][0] = -1;
	figsensor_.range[1][1] = 1;
}

// Update sensor figure
void sensorUpdate(void)
{
	static const int maxline = 10;

	// clear linepnt
	for (int i = 0; i < maxline; i++) {
		figsensor_.linepnt[i] = 0;
	}

	// start with line 0
	int lineid = 0;

	// loop over sensors
	for (int n = 0; n < m_->nsensor; n++) {
		// go to next line if type is different
		if (n > 0 && m_->sensor_type[n] != m_->sensor_type[n - 1])
			lineid = mjMIN(lineid + 1, maxline - 1);

		// get info about this sensor
		mjtNum cutoff = (m_->sensor_cutoff[n] > 0 ? m_->sensor_cutoff[n] : 1);
		int adr       = m_->sensor_adr[n];
		int dim       = m_->sensor_dim[n];

		// data pointer in line
		int p = figsensor_.linepnt[lineid];

		// fill in data for this sensor
		for (int i = 0; i < dim; i++) {
			// check size
			if ((p + 2 * i) > mjMAXLINEPNT / 2)
				break;

			// x
			figsensor_.linedata[lineid][2 * p + 4 * i]     = (float)(adr + i);
			figsensor_.linedata[lineid][2 * p + 4 * i + 2] = (float)(adr + i);

			// y
			figsensor_.linedata[lineid][2 * p + 4 * i + 1] = 0;
			figsensor_.linedata[lineid][2 * p + 4 * i + 3] = (float)(d_->sensordata[adr + i] / cutoff);
		}

		// update linepnt
		figsensor_.linepnt[lineid] = mjMIN(mjMAXLINEPNT - 1, figsensor_.linepnt[lineid] + 2 * dim);
	}
}

// Show sensor figure
void sensorShow(mjrRect rect)
{
	// constant width with and without profiler
	int width = settings_.profiler ? rect.width / 3 : rect.width / 4;

	// render figure on the right
	mjrRect viewport = { rect.left + rect.width - width, rect.bottom, width, rect.height / 3 };
	mjr_figure(viewport, &figsensor_, &con_);
}

void profilerInit(void)
{
	int i, n;

	// set figures to default
	mjv_defaultFigure(&figconstraint_);
	mjv_defaultFigure(&figcost_);
	mjv_defaultFigure(&figtimer_);
	mjv_defaultFigure(&figsize_);

	// titles
	mju::strcpy_arr(figconstraint_.title, "Counts");
	mju::strcpy_arr(figcost_.title, "Convergence (log 10)");
	mju::strcpy_arr(figsize_.title, "Dimensions");
	mju::strcpy_arr(figtimer_.title, "CPU time (msec)");

	// x-labels
	mju::strcpy_arr(figconstraint_.xlabel, "Solver iteration");
	mju::strcpy_arr(figcost_.xlabel, "Solver iteration");
	mju::strcpy_arr(figsize_.xlabel, "Video Frame");
	mju::strcpy_arr(figtimer_.xlabel, "Video Frame");

	// y-tick number formats
	mju::strcpy_arr(figconstraint_.yformat, "%.0f");
	mju::strcpy_arr(figcost_.yformat, "%.1f");
	mju::strcpy_arr(figsize_.yformat, "%.0f");
	mju::strcpy_arr(figtimer_.yformat, "%.2f");

	// colors
	figconstraint_.figurergba[0] = 0.1f;
	figcost_.figurergba[2]       = 0.2f;
	figsize_.figurergba[0]       = 0.1f;
	figtimer_.figurergba[2]      = 0.2f;
	figconstraint_.figurergba[3] = 0.5f;
	figcost_.figurergba[3]       = 0.5f;
	figsize_.figurergba[3]       = 0.5f;
	figtimer_.figurergba[3]      = 0.5f;

	// legends
	mju::strcpy_arr(figconstraint_.linename[0], "total");
	mju::strcpy_arr(figconstraint_.linename[1], "active");
	mju::strcpy_arr(figconstraint_.linename[2], "changed");
	mju::strcpy_arr(figconstraint_.linename[3], "evals");
	mju::strcpy_arr(figconstraint_.linename[4], "updates");
	mju::strcpy_arr(figcost_.linename[0], "improvement");
	mju::strcpy_arr(figcost_.linename[1], "gradient");
	mju::strcpy_arr(figcost_.linename[2], "lineslope");
	mju::strcpy_arr(figsize_.linename[0], "dof");
	mju::strcpy_arr(figsize_.linename[1], "body");
	mju::strcpy_arr(figsize_.linename[2], "constraint");
	mju::strcpy_arr(figsize_.linename[3], "sqrt(nnz)");
	mju::strcpy_arr(figsize_.linename[4], "contact");
	mju::strcpy_arr(figsize_.linename[5], "iteration");
	mju::strcpy_arr(figtimer_.linename[0], "total");
	mju::strcpy_arr(figtimer_.linename[1], "collision");
	mju::strcpy_arr(figtimer_.linename[2], "prepare");
	mju::strcpy_arr(figtimer_.linename[3], "solve");
	mju::strcpy_arr(figtimer_.linename[4], "other");

	// grid sizes
	figconstraint_.gridsize[0] = 5;
	figconstraint_.gridsize[1] = 5;
	figcost_.gridsize[0]       = 5;
	figcost_.gridsize[1]       = 5;
	figsize_.gridsize[0]       = 3;
	figsize_.gridsize[1]       = 5;
	figtimer_.gridsize[0]      = 3;
	figtimer_.gridsize[1]      = 5;

	// minimum ranges
	figconstraint_.range[0][0] = 0;
	figconstraint_.range[0][1] = 20;
	figconstraint_.range[1][0] = 0;
	figconstraint_.range[1][1] = 80;
	figcost_.range[0][0]       = 0;
	figcost_.range[0][1]       = 20;
	figcost_.range[1][0]       = -15;
	figcost_.range[1][1]       = 5;
	figsize_.range[0][0]       = -200;
	figsize_.range[0][1]       = 0;
	figsize_.range[1][0]       = 0;
	figsize_.range[1][1]       = 100;
	figtimer_.range[0][0]      = -200;
	figtimer_.range[0][1]      = 0;
	figtimer_.range[1][0]      = 0;
	figtimer_.range[1][1]      = 0.4f;

	// init x axis on history figures (do not show yet)
	for (n = 0; n < 6; n++)
		for (i = 0; i < mjMAXLINEPNT; i++) {
			figtimer_.linedata[n][2 * i] = (float)-i;
			figsize_.linedata[n][2 * i]  = (float)-i;
		}
}

void profilerUpdate(void)
{
	int i, n;

	// update constraint figure
	figconstraint_.linepnt[0] = mjMIN(mjMIN(d_->solver_iter, mjNSOLVER), mjMAXLINEPNT);
	for (i = 1; i < 5; i++) {
		figconstraint_.linepnt[i] = figconstraint_.linepnt[0];
	}
	if (m_->opt.solver == mjSOL_PGS) {
		figconstraint_.linepnt[3] = 0;
		figconstraint_.linepnt[4] = 0;
	}
	if (m_->opt.solver == mjSOL_CG) {
		figconstraint_.linepnt[4] = 0;
	}
	for (i = 0; i < figconstraint_.linepnt[0]; i++) {
		// x
		figconstraint_.linedata[0][2 * i] = (float)i;
		figconstraint_.linedata[1][2 * i] = (float)i;
		figconstraint_.linedata[2][2 * i] = (float)i;
		figconstraint_.linedata[3][2 * i] = (float)i;
		figconstraint_.linedata[4][2 * i] = (float)i;

		// y
		figconstraint_.linedata[0][2 * i + 1] = (float)d_->nefc;
		figconstraint_.linedata[1][2 * i + 1] = (float)d_->solver[i].nactive;
		figconstraint_.linedata[2][2 * i + 1] = (float)d_->solver[i].nchange;
		figconstraint_.linedata[3][2 * i + 1] = (float)d_->solver[i].neval;
		figconstraint_.linedata[4][2 * i + 1] = (float)d_->solver[i].nupdate;
	}

	// update cost figure
	figcost_.linepnt[0] = mjMIN(mjMIN(d_->solver_iter, mjNSOLVER), mjMAXLINEPNT);
	for (i = 1; i < 3; i++) {
		figcost_.linepnt[i] = figcost_.linepnt[0];
	}
	if (m_->opt.solver == mjSOL_PGS) {
		figcost_.linepnt[1] = 0;
		figcost_.linepnt[2] = 0;
	}

	for (i = 0; i < figcost_.linepnt[0]; i++) {
		// x
		figcost_.linedata[0][2 * i] = (float)i;
		figcost_.linedata[1][2 * i] = (float)i;
		figcost_.linedata[2][2 * i] = (float)i;

		// y
		figcost_.linedata[0][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, d_->solver[i].improvement));
		figcost_.linedata[1][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, d_->solver[i].gradient));
		figcost_.linedata[2][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, d_->solver[i].lineslope));
	}

	// get timers: total, collision, prepare, solve, other
	mjtNum total = d_->timer[mjTIMER_STEP].duration;
	int number   = d_->timer[mjTIMER_STEP].number;
	if (!number) {
		total  = d_->timer[mjTIMER_FORWARD].duration;
		number = d_->timer[mjTIMER_FORWARD].number;
	}
	number         = mjMAX(1, number);
	float tdata[5] = { (float)(total / number), (float)(d_->timer[mjTIMER_POS_COLLISION].duration / number),
		                (float)(d_->timer[mjTIMER_POS_MAKE].duration / number) +
		                    (float)(d_->timer[mjTIMER_POS_PROJECT].duration / number),
		                (float)(d_->timer[mjTIMER_CONSTRAINT].duration / number), 0 };
	tdata[4]       = tdata[0] - tdata[1] - tdata[2] - tdata[3];

	// update figtimer
	int pnt = mjMIN(201, figtimer_.linepnt[0] + 1);
	for (n = 0; n < 5; n++) {
		// shift data
		for (i = pnt - 1; i > 0; i--) {
			figtimer_.linedata[n][2 * i + 1] = figtimer_.linedata[n][2 * i - 1];
		}

		// assign new
		figtimer_.linepnt[n]     = pnt;
		figtimer_.linedata[n][1] = tdata[n];
	}

	// get sizes: nv, nbody, nefc, sqrt(nnz), ncont, iter
	float sdata[6] = { (float)m_->nv,   (float)m_->nbody,      (float)d_->nefc, (float)mju_sqrt((mjtNum)d_->solver_nnz),
		                (float)d_->ncon, (float)d_->solver_iter };

	// update figsize
	pnt = mjMIN(201, figsize_.linepnt[0] + 1);
	for (n = 0; n < 6; n++) {
		// shift data
		for (i = pnt - 1; i > 0; i--) {
			figsize_.linedata[n][2 * i + 1] = figsize_.linedata[n][2 * i - 1];
		}

		// assign new
		figsize_.linepnt[n]     = pnt;
		figsize_.linedata[n][1] = sdata[n];
	}
}

void profilerShow(mjrRect rect)
{
	mjrRect viewport = { rect.left + rect.width - rect.width / 4, rect.bottom, rect.width / 4, rect.height / 4 };
	mjr_figure(viewport, &figtimer_, &con_);
	viewport.bottom += rect.height / 4;
	mjr_figure(viewport, &figsize_, &con_);
	viewport.bottom += rect.height / 4;
	mjr_figure(viewport, &figcost_, &con_);
	viewport.bottom += rect.height / 4;
	mjr_figure(viewport, &figconstraint_, &con_);
}

// Update UI 0 when MuJoCo structures change (except for joint sliders)
void updateSettings(void)
{
	int i;

	ROS_DEBUG_ONCE_NAMED("mujoco", "\tupdating physics");
	// physics flags
	for (i = 0; i < mjNDISABLE; i++) {
		settings_.disable[i] = ((m_->opt.disableflags & (i << i)) != 0);
	}
	for (i = 0; i < mjNENABLE; i++) {
		settings_.enable[i] = ((m_->opt.enableflags & (1 << 1)) != 0);
	}

	ROS_DEBUG_ONCE_NAMED("mujoco", "\tupdating cam");
	// camera
	if (cam_.type == mjCAMERA_FIXED) {
		settings_.camera = 2 + cam_.fixedcamid;
	} else if (cam_.type == mjCAMERA_TRACKING) {
		settings_.camera = 1;
	} else {
		settings_.camera = 0;
	}

	// update UI
	if (vis_)
		mjui_update(-1, -1, &ui0_, &uistate_, &con_);
}

// Physics section of UI
void makePhysics(int oldstate)
{
	int i;

	mjuiDef defPhysics[]     = { { mjITEM_SECTION, "Physics", oldstate, NULL, "AP" },
                            { mjITEM_SELECT, "Integrator", 2, &(m_->opt.integrator), "Euler\nRK4" },
                            { mjITEM_SELECT, "Collision", 2, &(m_->opt.collision), "All\nPair\nDynamic" },
                            { mjITEM_SELECT, "Cone", 2, &(m_->opt.cone), "Pyramidal\nElliptic" },
                            { mjITEM_SELECT, "Jacobian", 2, &(m_->opt.jacobian), "Dense\nSparse\nAuto" },
                            { mjITEM_SELECT, "Solver", 2, &(m_->opt.solver), "PGS\nCG\nNewton" },
                            { mjITEM_SEPARATOR, "Algorithmic Parameters", 1 },
                            { mjITEM_EDITNUM, "Timestep", 2, &(m_->opt.timestep), "1 0 1" },
                            { mjITEM_EDITINT, "Iterations", 2, &(m_->opt.iterations), "1 0 1000" },
                            { mjITEM_EDITNUM, "Tolerance", 2, &(m_->opt.tolerance), "1 0 1" },
                            { mjITEM_EDITINT, "Noslip Iter", 2, &(m_->opt.noslip_iterations), "1 0 1000" },
                            { mjITEM_EDITNUM, "Noslip Tol", 2, &(m_->opt.noslip_tolerance), "1 0 1" },
                            { mjITEM_EDITINT, "MPR Iter", 2, &(m_->opt.mpr_iterations), "1 0 1000" },
                            { mjITEM_EDITNUM, "MPR Tol", 2, &(m_->opt.mpr_tolerance), "1 0 1" },
                            { mjITEM_EDITNUM, "API Rate", 2, &(m_->opt.apirate), "1 0 1000" },
                            { mjITEM_SEPARATOR, "Physical Parameters", 1 },
                            { mjITEM_EDITNUM, "Gravity", 2, m_->opt.gravity, "3" },
                            { mjITEM_EDITNUM, "Wind", 2, m_->opt.wind, "3" },
                            { mjITEM_EDITNUM, "Magnetic", 2, m_->opt.magnetic, "3" },
                            { mjITEM_EDITNUM, "Density", 2, &(m_->opt.density), "1" },
                            { mjITEM_EDITNUM, "Viscosity", 2, &(m_->opt.viscosity), "1" },
                            { mjITEM_EDITNUM, "Imp Ratio", 2, &(m_->opt.impratio), "1" },
                            { mjITEM_SEPARATOR, "Disable Flags", 1 },
                            { mjITEM_END } };
	mjuiDef defEnableFlags[] = { { mjITEM_SEPARATOR, "Enable Flags", 1 }, { mjITEM_END } };
	mjuiDef defOverride[]    = { { mjITEM_SEPARATOR, "Contact Override", 1 },
                             { mjITEM_EDITNUM, "Margin", 2, &(m_->opt.o_margin), "1" },
                             { mjITEM_EDITNUM, "Sol Imp", 2, &(m_->opt.o_solimp), "5" },
                             { mjITEM_EDITNUM, "Sol Ref", 2, &(m_->opt.o_solref), "2" },
                             { mjITEM_END } };

	// add physics
	mjui_add(&ui0_, defPhysics);

	// add flags programmatically
	mjuiDef defFlag[] = { { mjITEM_CHECKINT, "", 2, NULL, "" }, { mjITEM_END } };
	for (i = 0; i < mjNDISABLE; i++) {
		mju::strcpy_arr(defFlag[0].name, mjDISABLESTRING[i]);
		defFlag[0].pdata = settings_.disable + i;
		mjui_add(&ui0_, defFlag);
	}
	mjui_add(&ui0_, defEnableFlags);
	for (i = 0; i < mjNENABLE; i++) {
		mju::strcpy_arr(defFlag[0].name, mjENABLESTRING[i]);
		defFlag[0].pdata = settings_.enable + i;
		mjui_add(&ui0_, defFlag);
	}

	// add contact override
	mjui_add(&ui0_, defOverride);
}

// Make rendering section of UI
void makeRendering(int oldstate)
{
	int i, j;

	mjuiDef defRendering[] = {
		{ mjITEM_SECTION, "Rendering", oldstate, NULL, "AR" },
		{ mjITEM_SELECT, "Camera", 2, &(settings_.camera), "Free\nTracking" },
		{ mjITEM_SELECT, "Label", 2, &(vopt_.label),
		  "None\nBody\nJoint\nGeom\nSite\nCamera\nLight\nTendon\nActuator\nConstraint\nSkin\nSelection\nSel Pnt\nForce" },
		{ mjITEM_SELECT, "Frame", 2, &(vopt_.frame), "None\nBody\nGeom\nSite\nCamera\nLight\nWorld" },
		{ mjITEM_BUTTON, "Print camera", 2, NULL, "" },
		{
		    mjITEM_SEPARATOR,
		    "Model Elements",
		    1,
		},
		{ mjITEM_END }
	};
	mjuiDef defOpenGL[] = { { mjITEM_SEPARATOR, "OpenGL Effects", 1 }, { mjITEM_END } };

	// add model cameras, up to UI limit
	for (i = 0; i < mjMIN(m_->ncam, mjMAXUIMULTI - 2); i++) {
		// prepare name
		char camname[mjMAXUITEXT] = "\n";
		if (m_->names[m_->name_camadr[i]]) {
			mju::strcat_arr(camname, m_->names + m_->name_camadr[i]);
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
	mjui_add(&ui0_, defRendering);

	// add flags programmatically
	mjuiDef defFlag[] = { { mjITEM_CHECKBYTE, "", 2, NULL, "" }, { mjITEM_END } };
	for (i = 0; i < mjNVISFLAG; i++) {
		// set name, remove "&"
		mju::strcpy_arr(defFlag[0].name, mjVISSTRING[i][0]);
		for (j = 0; j < strlen(mjVISSTRING[i][0]); j++) {
			if (mjVISSTRING[i][0][j] == '&') {
				mju_strncpy(defFlag[0].name + j, mjVISSTRING[i][0] + j + 1, mju::sizeof_arr(defFlag[0].name) - j);
				break;
			}
		}

		// set shortcut and data
		mju::sprintf_arr(defFlag[0].other, " %s", mjVISSTRING[i][2]);
		defFlag[0].pdata = vopt_.flags + i;
		mjui_add(&ui0_, defFlag);
	}
	mjui_add(&ui0_, defOpenGL);
	for (i = 0; i < mjNRNDFLAG; i++) {
		mju::strcpy_arr(defFlag[0].name, mjRNDSTRING[i][0]);
		mju::sprintf_arr(defFlag[0].other, " %s", mjRNDSTRING[i][2]);
		defFlag[0].pdata = scn_.flags + i;
		mjui_add(&ui0_, defFlag);
	}
}

// Make group section UI
void makeGroup(int oldstate)
{
	mjuiDef defGroup[] = { { mjITEM_SECTION, "Group enable", oldstate, NULL, "AG" },
		                    { mjITEM_SEPARATOR, "Geom groups", 1 },
		                    { mjITEM_CHECKBYTE, "Geom 0", 2, vopt_.geomgroup, " 0" },
		                    { mjITEM_CHECKBYTE, "Geom 1", 2, vopt_.geomgroup + 1, " 1" },
		                    { mjITEM_CHECKBYTE, "Geom 2", 2, vopt_.geomgroup + 2, " 2" },
		                    { mjITEM_CHECKBYTE, "Geom 3", 2, vopt_.geomgroup + 3, " 3" },
		                    { mjITEM_CHECKBYTE, "Geom 4", 2, vopt_.geomgroup + 4, " 4" },
		                    { mjITEM_CHECKBYTE, "Geom 5", 2, vopt_.geomgroup + 5, " 5" },
		                    { mjITEM_SEPARATOR, "Site groups", 1 },
		                    { mjITEM_CHECKBYTE, "Site 0", 2, vopt_.sitegroup, "S0" },
		                    { mjITEM_CHECKBYTE, "Site 1", 2, vopt_.sitegroup + 1, "S1" },
		                    { mjITEM_CHECKBYTE, "Site 2", 2, vopt_.sitegroup + 2, "S2" },
		                    { mjITEM_CHECKBYTE, "Site 3", 2, vopt_.sitegroup + 3, "S3" },
		                    { mjITEM_CHECKBYTE, "Site 4", 2, vopt_.sitegroup + 4, "S4" },
		                    { mjITEM_CHECKBYTE, "Site 5", 2, vopt_.sitegroup + 5, "S5" },
		                    { mjITEM_SEPARATOR, "Joint groups", 1 },
		                    { mjITEM_CHECKBYTE, "Joint 0", 2, vopt_.jointgroup, "" },
		                    { mjITEM_CHECKBYTE, "Joint 1", 2, vopt_.jointgroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Joint 2", 2, vopt_.jointgroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Joint 3", 2, vopt_.jointgroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Joint 4", 2, vopt_.jointgroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Joint 5", 2, vopt_.jointgroup + 5, "" },
		                    { mjITEM_SEPARATOR, "Tendon groups", 1 },
		                    { mjITEM_CHECKBYTE, "Tendon 0", 2, vopt_.tendongroup, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 1", 2, vopt_.tendongroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 2", 2, vopt_.tendongroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 3", 2, vopt_.tendongroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 4", 2, vopt_.tendongroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 5", 2, vopt_.tendongroup + 5, "" },
		                    { mjITEM_SEPARATOR, "Actuator groups", 1 },
		                    { mjITEM_CHECKBYTE, "Actuator 0", 2, vopt_.actuatorgroup, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 1", 2, vopt_.actuatorgroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 2", 2, vopt_.actuatorgroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 3", 2, vopt_.actuatorgroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 4", 2, vopt_.actuatorgroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 5", 2, vopt_.actuatorgroup + 5, "" },
		                    { mjITEM_END } };

	mjui_add(&ui0_, defGroup);
}

// Make joint section of UI
void makeJoint(int oldstate)
{
	int i;

	mjuiDef defJoint[] = {
		{ mjITEM_SECTION, "Joint", oldstate, NULL, "AJ" },
		{ mjITEM_END },
	};
	mjuiDef defSlider[] = { { mjITEM_SLIDERNUM, "", 2, NULL, "0 1" }, { mjITEM_END } };

	// add section
	mjui_add(&ui1_, defJoint);
	defSlider[0].state = 4;

	// add scalar joints, exit if UI limit reached
	int itemcnt = 0;
	for (i = 0; i < m_->njnt && itemcnt < mjMAXUIITEM; i++) {
		if ((m_->jnt_type[i] == mjJNT_HINGE || m_->jnt_type[i] == mjJNT_SLIDE)) {
			// skip if joint group is disabled
			if (!vopt_.jointgroup[mjMAX(0, mjMIN(mjNGROUP - 1, m_->jnt_group[i]))]) {
				continue;
			}

			// set data and name
			defSlider[0].pdata = d_->qpos + m_->jnt_qposadr[i];
			if (m_->names[m_->name_jntadr[i]]) {
				mju::strcpy_arr(defSlider[0].name, m_->names + m_->name_jntadr[i]);
			} else {
				mju::sprintf_arr(defSlider[0].name, "joint %d", i);
			}

			// set range
			if (m_->jnt_limited[i]) {
				mju::sprintf_arr(defSlider[0].other, "%.4g %.4g", m_->jnt_range[2 * i], m_->jnt_range[2 * i + 1]);
			} else if (m_->jnt_type[i] == mjJNT_SLIDE) {
				mju::strcpy_arr(defSlider[0].other, "-1 1");
			} else {
				mju::strcpy_arr(defSlider[0].other, "-3.1416 3.1416");
			}

			// add and count
			mjui_add(&ui1_, defSlider);
			itemcnt++;
		}
	}
}

// Make control section of UI
void makeControl(int oldstate)
{
	int i;

	mjuiDef defControl[] = { { mjITEM_SECTION, "Control", oldstate, NULL, "AC" },
		                      { mjITEM_BUTTON, "Clear all", 2 },
		                      { mjITEM_END } };
	mjuiDef defSlider[]  = { { mjITEM_SLIDERNUM, "", 2, NULL, "0 1" }, { mjITEM_END } };

	// Add section
	mjui_add(&ui1_, defControl);
	defSlider[0].state = 2;

	// Add controls, exit if UI limit reached (Clear button already added)
	int itemcnt = 1;
	for (i = 0; i < m_->nu && itemcnt < mjMAXUIITEM; i++) {
		// Skip if actuator group is disabled
		if (!vopt_.actuatorgroup[mjMAX(0, mjMIN(mjNGROUP - 1, m_->actuator_group[i]))]) {
			continue;
		}

		// set data and name
		defSlider[0].pdata = d_->ctrl + i;
		if (m_->names[m_->name_actuatoradr[i]]) {
			mju::strcpy_arr(defSlider[0].name, m_->names + m_->name_actuatoradr[i]);
		} else {
			mju::sprintf_arr(defSlider[0].name, "control %d", i);
		}

		// set range
		if (m_->actuator_ctrllimited[i]) {
			mju::sprintf_arr(defSlider[0].other, "%.4g %.4g", m_->actuator_ctrlrange[2 * i + 1]);
		} else {
			mju::strcpy_arr(defSlider[0].other, "-1 1");
		}

		// add and count
		mjui_add(&ui1_, defSlider);
		itemcnt++;
	}
}

// Make model-dependent UI sections
void makeSections(void)
{
	int i;

	// get section open-close state, UI 0
	int oldstate0[NSECT0];
	for (i = 0; i < NSECT0; i++) {
		oldstate0[i] = 0;
		if (ui0_.nsect > i) {
			oldstate0[i] = ui0_.sect[i].state;
		}
	}

	// get section open-close state, UI 1
	int oldstate1[NSECT1];
	for (i = 0; i < NSECT1; i++) {
		oldstate1[i] = 0;
		if (ui1_.nsect > i) {
			oldstate1[i] = ui1_.sect[i].state;
		}
	}

	// clear model-dependent sections of UI
	ui0_.nsect = SECT_PHYSICS;
	ui1_.nsect = 0;

	// make
	makePhysics(oldstate0[SECT_PHYSICS]);
	makeRendering(oldstate0[SECT_RENDERING]);
	makeGroup(oldstate0[SECT_GROUP]);
	makeJoint(oldstate1[SECT_JOINT]);
	makeControl(oldstate1[SECT_CONTROL]);
}

// Prepare to render
void prepare(void)
{
	// data for FPS calculation
	static double lastupdatetm = 0;

	// update interval, save update time
	double tmnow    = glfwGetTime();
	double interval = tmnow - lastupdatetm;
	interval        = mjMIN(1, mjMAX(0.0001, interval));
	lastupdatetm    = tmnow;

	// No model: nothing to do
	if (!m_) {
		return;
	}

	// Update scene
	mjv_updateScene(m_.get(), d_.get(), &vopt_, &pert_, &cam_, mjCAT_ALL, &scn_);

	// Update watch
	if (settings_.ui0 && ui0_.sect[SECT_WATCH].state) {
		watch();
		mjui_update(SECT_WATCH, -1, &ui0_, &uistate_, &con_);
	}

	// Update joint
	if (settings_.ui1 && ui1_.sect[SECT_JOINT].state) {
		mjui_update(SECT_JOINT, -1, &ui1_, &uistate_, &con_);
	}

	// Update info text
	if (settings_.info) {
		infotext(info_title_, info_content_, interval);
	}

	// Update control
	if (settings_.ui1 && ui1_.sect[SECT_CONTROL].state) {
		mjui_update(SECT_CONTROL, -1, &ui1_, &uistate_, &con_);
	}

	// Update profiler
	if (settings_.profiler && settings_.run) {
		profilerUpdate();
	}

	// Update sensor
	if (settings_.sensor && settings_.run) {
		sensorUpdate();
	}

	// clear timers once profiler info has been copied
	clearTimers();
}

// Sprintf forwarding, to avoid compiler warning in x-macro
void printField(char (&str)[mjMAXUINAME], void *ptr)
{
	mju::sprintf_arr(str, "%g", *(mjtNum *)ptr);
}

// Update watch
void watch(void)
{
	// clear
	ui0_.sect[SECT_WATCH].item[2].multi.nelem = 1;
	mju::strcpy_arr(ui0_.sect[SECT_WATCH].item[2].multi.name[0], "invalid field");

	// prepare symbols needed by xmacro
	MJDATA_POINTERS_PREAMBLE(m_);

// find specified field in mjData arrays, update value
#define X(TYPE, NAME, NR, NC)                                                                 \
	if (!mju::strcmp_arr(#NAME, settings_.field) && !mju::strcmp_arr(#TYPE, "mjtNum")) {       \
		if (settings_.index >= 0 && settings_.index < m_->NR * NC) {                            \
			printField(ui0_.sect[SECT_WATCH].item[2].multi.name[0], d_->NAME + settings_.index); \
		} else {                                                                                \
			mju::strcpy_arr(ui0_.sect[SECT_WATCH].item[2].multi.name[0], "invalid index");       \
		}                                                                                       \
		return;                                                                                 \
	}

	MJDATA_POINTERS
#undef X
}

// Prepare info text
void infotext(char (&title)[kBufSize], char (&content)[kBufSize], double interval)
{
	char tmp[20];

	// Compute solver error
	mjtNum solerr = 0;
	if (d_->solver_iter) {
		int ind = mjMIN(d_->solver_iter - 1, mjNSOLVER - 1);
		solerr  = mju_min(d_->solver[ind].improvement, d_->solver[ind].gradient);
		if (solerr == 0) {
			solerr = mju_max(d_->solver[ind].improvement, d_->solver[ind].gradient);
		}
	}
	solerr = mju_log10(mju_max(mjMINVAL, solerr));

	const std::string realtime_nominator = settings_.slow_down == 1 ? "" : "1/";
	mju::strcpy_arr(title, "Time\nSize\nCPU\nSolver\nFPS\nstack\nconbuf\nefcbuf");
	mju::sprintf_arr(content, "%-9.3f %s%d x\n%d  (%d con)\n%.3f\n%.1f  (%d it)\n%.0f\n%.3f\n%.3f\n%.3f", d_->time,
	                 realtime_nominator.c_str(), settings_.slow_down, d_->nefc, d_->ncon,
	                 settings_.run ? d_->timer[mjTIMER_STEP].duration / mjMAX(1, d_->timer[mjTIMER_STEP].number) :
	                                 d_->timer[mjTIMER_FORWARD].duration / mjMAX(1, d_->timer[mjTIMER_FORWARD].number),
	                 solerr, d_->solver_iter, 1 / interval, d_->maxuse_stack / (double)d_->nstack,
	                 d_->maxuse_con / (double)m_->nconmax, d_->maxuse_efc / (double)m_->njmax);

	// Add energy if enabled
	if (mjENABLED(mjENBL_ENERGY)) {
		mju::sprintf_arr(tmp, "\n%.3f", d_->energy[0] + d_->energy[1]);
		mju::strcat_arr(content, tmp);
		mju::strcat_arr(title, "\nEnergy");
	}

	// Add FwdInv if enabled
	if (mjENABLED(mjENBL_FWDINV)) {
		mju::sprintf_arr(tmp, "\n%.1f %.1f", mju_log10(mju_max(mjMINVAL, d_->solver_fwdinv[0])),
		                 mju_log10(mju_max(mjMINVAL, d_->solver_fwdinv[1])));
		mju::strcat_arr(content, tmp);
		mju::strcat_arr(title, "\nFwdInv");
	}
}

//---------------------------------- utility functions --------------------------------------
// Align and scale view
void alignScale(void)
{
	// autoscale
	cam_.lookat[0] = m_->stat.center[0];
	cam_.lookat[1] = m_->stat.center[1];
	cam_.lookat[2] = m_->stat.center[2];
	cam_.distance  = 1.5 * m_->stat.extent;

	// set to free camera
	cam_.type = mjCAMERA_FREE;
}

// Clear all timers
void clearTimers(void)
{
	for (int i = 0; i < mjNTIMER; i++) {
		d_->timer[i].duration = 0;
		d_->timer[i].number   = 0;
	}
}

// millisecond timer, for MuJoCo built-in profiler
mjtNum timer(void)
{
	return (mjtNum)(1000 * glfwGetTime());
}

void setupCallbacks()
{
	service_servers_.push_back(nh_->advertiseService("set_pause", setPauseCB));
	service_servers_.push_back(nh_->advertiseService("shutdown", shutdownCB));
}

// Service call callbacks
bool shutdownCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	requestExternalShutdown();
	return true;
}

bool setPauseCB(mujoco_ros_msgs::SetPause::Request &req, mujoco_ros_msgs::SetPause::Response &resp)
{
	ROS_DEBUG_STREAM("PauseCB called with: " << (bool)req.paused);
	settings_.run = !req.paused;
	return true;
}

} // end namespace detail

} // end namespace MujocoSim
