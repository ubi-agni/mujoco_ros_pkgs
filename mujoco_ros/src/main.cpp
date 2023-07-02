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

#include <mujoco_ros/glfw_adapter.h>
#include <mujoco_ros/viewer.h>
#include <mujoco_ros/mujoco_env.h>

#include <mujoco_ros/array_safety.h>
#include <mujoco/mujoco.h>

#include <boost/program_options.hpp>
#include <signal.h>
#include <thread>

namespace {

void sigint_handler(int sig)
{
	ros::shutdown();
}

namespace po  = boost::program_options;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign       = 0.1; // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
const int kErrorLength          = 1024; // load error string length

mjModel *m = nullptr;
mjData *d  = nullptr;

mjtNum *ctrlnoise = nullptr;

using Seconds = std::chrono::duration<double>;

} // anonymous namespace

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mujoco");
	ros::start();

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::NodeHandle nh = ros::NodeHandle("~");

	signal(SIGINT, sigint_handler);

	std::string admin_hash("");

	po::options_description options;
	options.add_options() // clang-format off
	  ("help,h", "Produce this help message")
	  ("admin-hash", po::value<std::string>(&admin_hash),"Set the admin hash for eval mode.");
	// clang-format on
	po::variables_map vm;

	try {
		po::store(po::parse_command_line(argc, argv, options), vm);
		po::notify(vm);

		if (vm.count("help")) {
			std::cout << "command line options:\n" << options;
			exit(0);
		}
	} catch (std::exception &e) {
		ROS_ERROR("Error parsing command line: %s", e.what());
		exit(-1);
	}

	bool vis = true;

	/*
	 * Model (file) passing: the model can be provided as file to parse or directly as string stored in the rosparam
	 * server. If both string and file are provided, the string takes precedence.
	 */

	std::string filename;
	nh.getParam("modelfile", filename);

	std::string xml_content_path;
	std::string xml_content;
	bool wait_for_xml = nh.param<bool>("wait_for_xml", false);

	ROS_INFO_COND(wait_for_xml, "Waiting for xml content to be available on rosparam server");

	while (wait_for_xml) {
		if (nh.searchParam("mujoco_xml", xml_content_path) || ros::param::search("mujoco_xml", xml_content_path)) {
			ROS_DEBUG_STREAM("Found mujoco_xml_content param under " << xml_content_path);

			nh.getParam(xml_content_path, xml_content);
			if (!xml_content.empty()) {
				ROS_INFO("Got xml content from ros param server");
				filename = "rosparam_content";
			}
			wait_for_xml = false;
		}
	}

	if (!filename.empty()) {
		ROS_INFO_STREAM("Using modelfile " << filename);
	} else {
		ROS_WARN("No modelfile was provided, launching empty simulation!");
	}

	std::printf("MuJoCo version %s\n", mj_versionString());
	if (mjVERSION_HEADER != mj_version()) {
		mju_error("Headers and library have different versions");
	}

	// TODO(dleins): Should MuJoCo Plugins be loaded?
	{
		mujoco_ros::MujocoEnv env;

		bool no_x;
		nh.param("no_x", no_x, false);

		if (!no_x) {
			nh.param("headless", no_x, false);
		}

		// const char *filename = nullptr;
		if (!filename.empty()) {
			mju::strcpy_arr(env.queued_filename_, filename.c_str());
			env.settings_.load_request = 2;
		}

		env.startPhysicsLoop();
		env.startEventLoop();

		if (!no_x) {
			ROS_INFO("Launching viewer");
			// Start simulation UI loop (blocking)
			// auto viewer = std::make_unique<mujoco_ros::Viewer>(std::make_unique<mujoco_ros::GlfwAdapter>(), &env,
			//    /* fully_managed = */ true);
			auto viewer = std::make_unique<mujoco_ros::Viewer>(
			    std::unique_ptr<mujoco_ros::PlatformUIAdapter>(env.gui_adapter_), &env, /* fully_managed = */ true);
			viewer->RenderLoop();
		} else {
			ROS_INFO("Running headless");
		}

		env.waitForPhysicsJoin();
		env.waitForEventsJoin();
	} // end of scope for env for proper destruction

	ROS_INFO("MuJoCo ROS Simulation Server node is terminating");

	spinner.stop();
	ros::shutdown();
	exit(0);
}
