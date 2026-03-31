#pragma once

#include "fuel_exploration/common/navigation_utils.hpp"

#include <cstddef>
#include <optional>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

namespace fuel_exploration::controller {

struct PathTrackingConfig {
  double lookahead_distance{0.35};
  double waypoint_tolerance{0.15};
  double goal_tolerance{0.20};
  double slow_down_distance{0.60};
  double max_linear_speed{0.22};
  double max_angular_speed{1.20};
  double linear_kp{0.80};
  double angular_kp{1.80};
  double rotate_in_place_threshold{0.55};
};

struct PathTrackingResult {
  geometry_msgs::msg::Twist cmd_vel;
  std::optional<std::size_t> active_target_index;
  bool goal_reached{false};
};

std::size_t find_closest_pose_index(const nav_msgs::msg::Path &path,
                                    const RobotPose &robot_pose);

std::size_t choose_lookahead_index(const nav_msgs::msg::Path &path,
                                   std::size_t closest_index,
                                   double lookahead_distance);

PathTrackingResult compute_path_tracking_command(
    const nav_msgs::msg::Path &path,
    const RobotPose &robot_pose,
    const PathTrackingConfig &config);

}  // namespace fuel_exploration::controller
