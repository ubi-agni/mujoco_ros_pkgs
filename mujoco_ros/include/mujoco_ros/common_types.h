#pragma once

#include <mujoco.h>

static constexpr int kBufSize = 1000;

namespace MujocoSim {

/**
 * @brief Helper enum to enable/disable features of the respective sim mode.
 *
 */
typedef enum simMode_
{
	SINGLE = 0,
	PARALLEL
} simMode;

/**
 * @def mjModelPtr
 * @brief boost::shared_ptr to mjModel
 */
typedef boost::shared_ptr<mjModel> mjModelPtr;
/**
 * @def mjDataPtr
 * @brief boost::shared_ptr to mjData
 */
typedef boost::shared_ptr<mjData> mjDataPtr;

// MujocoPlugin
class MujocoPlugin;

/**
 * @def MujocoPluginPtr
 * @brief boost::shared_ptr to MujocoPlugin
 */
typedef boost::shared_ptr<MujocoPlugin> MujocoPluginPtr;

// MujocoEnvironment
struct MujocoEnv;
struct MujocoEnvParallel;

/**
 * @def MujocoEnvPtr
 * @brief boost::shared_ptr to MujocoEnv
 */
typedef boost::shared_ptr<MujocoEnv> MujocoEnvPtr;
/**
 * @def MujocoEnvParallelPtr
 * @brief boost::shared_ptr to MujocoEnvParallel
 */
typedef boost::shared_ptr<MujocoEnvParallel> MujocoEnvParallelPtr;

} // namespace MujocoSim
