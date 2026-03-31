#pragma once

#include "fuel_exploration/common/navigation_utils.hpp"

#include <cstddef>
#include <optional>

#include <nav_msgs/msg/path.hpp>

namespace fuel_exploration::controller {

struct ExplorationControllerState {
  std::optional<RobotPose> robot_pose;
  nav_msgs::msg::Path path;
  std::optional<std::size_t> active_target_index;
  mutable bool warned_transform_failure{false};
};

}  // namespace fuel_exploration::controller
