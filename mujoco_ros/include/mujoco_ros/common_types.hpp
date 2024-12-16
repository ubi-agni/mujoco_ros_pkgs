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

#pragma once

#include <mujoco_ros/ros_version.hpp>

#include <mujoco/mujoco.h>

#if MJR_ROS_VERSION == ROS_1

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#else // MJR_ROS_VERSION == ROS_2

#include <rclcpp/rclcpp.hpp>

#endif

namespace mujoco_ros {

using Clock = std::chrono::steady_clock;
static_assert(std::ratio_less_equal_v<Clock::period, std::milli>, "Clock must have millisecond precision or better");

using Seconds      = std::chrono::duration<double>;
using Milliseconds = std::chrono::duration<double, std::milli>;

namespace rendering {

using StreamType = enum StreamType_ : uint8_t {
	RGB       = 1,
	DEPTH     = 1 << 1,
	SEGMENTED = 1 << 1 << 1,

	// Combined types to be cast safe
	RGB_D   = 3,
	RGB_S   = 5,
	DEPTH_S = 6,
	RGB_D_S = 7
};

static constexpr StreamType kDEFAULT_CAM_STREAM_TYPE = StreamType::RGB;
static constexpr float kDEFAULT_CAM_PUB_FREQ         = 15.f;
static constexpr bool kDEFAULT_CAM_USE_SEGID         = false;
static constexpr int kDEFAULT_CAM_WIDTH              = 720;
static constexpr int kDEFAULT_CAM_HEIGHT             = 480;
static constexpr char kDEFAULT_CAM_RGB_TOPIC[]       = "rgb";
static constexpr char kDEFAULT_CAM_DEPTH_TOPIC[]     = "depth";
static constexpr char kDEFAULT_CAM_SEGMENT_TOPIC[]   = "segmented";

class OffscreenCamera;
using OffscreenCameraPtr = std::unique_ptr<OffscreenCamera>;

} // namespace rendering

// Struct holding all the data needed for offscreen rendering
struct OffscreenRenderContext;

/**
 * @def mjModelPtr
 * @brief std::shared_ptr to mjModel
 */
using mjModelPtr = std::shared_ptr<mjModel>;
/**
 * @def mjDataPtr
 * @brief std::shared_ptr to mjData
 */
using mjDataPtr = std::shared_ptr<mjData>;

// MujocoPlugin
class MujocoPlugin;

/**
 * @def MujocoPluginPtr
 * @brief std::unique_ptr to MujocoPlugin
 */
using MujocoPluginPtr = std::unique_ptr<MujocoPlugin>;

// MujocoEnvironment
class MujocoEnv;

/**
 * @def MujocoEnvPtr
 * @brief ptr to MujocoEnv
 */
using MujocoEnvPtr = MujocoEnv *;

// Viewer
class Viewer;

} // namespace mujoco_ros
