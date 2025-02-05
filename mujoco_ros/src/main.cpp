/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022-2024, Bielefeld University
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

#include <csignal>
#include <thread>

#include <mujoco_ros/render_backend.hpp>
#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/logging.hpp>

#include <mujoco_ros/mujoco_env.hpp>

#include <mujoco_ros/array_safety.h>
#include <mujoco/mujoco.h>

#include <boost/program_options.hpp>

#if RENDER_BACKEND == GLFW_BACKEND
#include <mujoco_ros/glfw_adapter.h>
#include <mujoco_ros/viewer.hpp>
#endif

namespace {

std::shared_ptr<mujoco_ros::MujocoEnv> env;

void sigint_handler(int /*sig*/)
{
	std::printf("Registered C-c. Shutting down MuJoCo ROS Server ...\n");
	env->settings_.exit_request.store(1);
#if MJR_ROS_VERSION == ROS_2
	rclcpp::shutdown();
#endif
}

namespace po  = boost::program_options;
namespace mju = ::mujoco::sample_util;

using Seconds = std::chrono::duration<double>;

} // anonymous namespace

#if MJR_ROS_VERSION == ROS_2
static bool should_exit = false;
void async_spin(const rclcpp::executors::MultiThreadedExecutor::SharedPtr &executor)
{
	while (!should_exit) {
		executor->spin_some();
		std::this_thread::sleep_for(Seconds(0.01));
	}
}
#endif

int main(int argc, char **argv)
{
#if MJR_ROS_VERSION == ROS_1
	ros::init(argc, argv, "Mujoco");
	ros::start();

	ros::AsyncSpinner spinner(4);
	spinner.start();
#else // MJR_ROS_VERSION == ROS_2
	rclcpp::init(argc, argv);
#endif

	signal(SIGINT, sigint_handler);

	std::string admin_hash("");

	po::options_description options;
	options.add_options() // clang-format off
	  ("help,h", "Produce this help message")
	  ("admin-hash", po::value<std::string>(&admin_hash),"Set the admin hash for eval mode.");
	// clang-format on
	po::variables_map vm;

	try {
		po::store(po::command_line_parser(argc, argv).options(options).allow_unregistered().run(), vm);
		po::notify(vm);

		if (vm.count("help")) {
			std::cout << "command line options:\n" << options;
			exit(0);
		}
	} catch (std::exception &e) {
		MJR_ERROR("Error parsing command line: %s", e.what());
		exit(-1);
	}

	std::printf("MuJoCo version %s\n", mj_versionString());
	if (mjVERSION_HEADER != mj_version()) {
		MJR_ERROR_STREAM("Headers (" << mjVERSION_HEADER << ") and library (" << mj_versionString()
		                             << ") have different versions");
		mju_error("Headers and library have different versions");
	}

	// TODO(dleins): Should MuJoCo Plugins be loaded?

#if MJR_ROS_VERSION == ROS_1
	env = std::make_shared<mujoco_ros::MujocoEnv>(admin_hash);
#else // MJR_ROS_VERSION == ROS_2
	// prepare for spinning later
	rclcpp::executors::MultiThreadedExecutor::SharedPtr executor =
	    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
	env = std::make_shared<mujoco_ros::MujocoEnv>(executor, admin_hash);
	executor->add_node(env);
#endif

	env->StartPhysicsLoop();
	env->StartEventLoop();

#if RENDER_BACKEND == GLFW_BACKEND
	if (!env->settings_.headless) {
		MJR_INFO("Launching viewer");
		auto viewer = std::make_unique<mujoco_ros::Viewer>(
		    std::unique_ptr<mujoco_ros::PlatformUIAdapter>(env->gui_adapter_), env.get(), /* is_passive = */ false);
#if MJR_ROS_VERSION == ROS_1
		viewer->RenderLoop();
#else // MJR_ROS_VERSION == ROS_2
      // TODO: This is a workaround for running a blocking viewer and a blocking executor in ROS 2.
      // Running the render loop in a separate thread does not work. The otherway around does, but
      // I'm not sure this is ideal.
		std::thread executor_thread = std::thread(std::bind(&async_spin, std::ref(executor)));
		viewer->RenderLoop();
		MJR_INFO("Viewer terminated");
		MJR_DEBUG("Joining executor thread");
		should_exit = true;
		executor_thread.join();
		MJR_DEBUG("Joined executor thread");
#endif
	}
#else
	MJR_ERROR_COND(!env->settings_.headless, "GLFW backend not available. Cannot launch viewer");
#endif
	MJR_INFO_COND(env->settings_.headless, "Running headless");

#if MJR_ROS_VERSION == ROS_2 && RENDER_BACKEND != GLFW_BACKEND
	executor->spin();
#endif

	env->WaitForPhysicsJoin();
	env->WaitForEventsJoin();
	env.reset();

	MJR_INFO("MuJoCo ROS Simulation Server node is terminating");

#if MJR_ROS_VERSION == ROS_1
	spinner.stop();
	ros::shutdown();
#else // MJR_ROS_VERSION == ROS_2
	rclcpp::shutdown();
#endif
	exit(0);
}
