#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"

namespace mujoco_ros2{

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


} // namespace mujoco_ros2