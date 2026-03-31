#include "fuel_exploration/controller/path_tracking.hpp"

#include "fuel_exploration/common/navigation_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fuel_exploration::controller {

std::size_t find_closest_pose_index(const nav_msgs::msg::Path &path,
                                    const RobotPose &robot_pose) {
  if (path.poses.empty()) {
    return 0;
  }

  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t idx = 0; idx < path.poses.size(); ++idx) {
    const auto &pose = path.poses[idx].pose.position;
    const double distance =
        distance_xy(robot_pose.x, robot_pose.y, pose.x, pose.y);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = idx;
    }
  }
  return closest_index;
}

std::size_t choose_lookahead_index(const nav_msgs::msg::Path &path,
                                   const std::size_t closest_index,
                                   const double lookahead_distance) {
  if (path.poses.empty()) {
    return 0;
  }

  double accumulated_distance = 0.0;
  std::size_t target_index = closest_index;
  while (target_index + 1 < path.poses.size() &&
         accumulated_distance < lookahead_distance) {
    const auto &current = path.poses[target_index].pose.position;
    const auto &next = path.poses[target_index + 1].pose.position;
    accumulated_distance += distance_xy(current.x, current.y, next.x, next.y);
    ++target_index;
  }
  return target_index;
}

PathTrackingResult compute_path_tracking_command(
    const nav_msgs::msg::Path &path,
    const RobotPose &robot_pose,
    const PathTrackingConfig &config) {
  PathTrackingResult result;
  if (path.poses.empty()) {
    return result;
  }

  const std::size_t closest_index = find_closest_pose_index(path, robot_pose);
  const auto &goal_pose = path.poses.back().pose.position;
  const double goal_distance =
      distance_xy(robot_pose.x, robot_pose.y, goal_pose.x, goal_pose.y);
  if (goal_distance < config.goal_tolerance) {
    result.goal_reached = true;
    return result;
  }

  std::size_t target_index =
      choose_lookahead_index(path, closest_index, config.lookahead_distance);
  while (target_index + 1 < path.poses.size()) {
    const auto &target_pose = path.poses[target_index].pose.position;
    const double target_distance =
        distance_xy(robot_pose.x, robot_pose.y, target_pose.x, target_pose.y);
    if (target_distance > config.waypoint_tolerance) {
      break;
    }
    ++target_index;
  }
  result.active_target_index = target_index;

  const auto &target_pose = path.poses[target_index].pose.position;
  const double target_heading =
      std::atan2(target_pose.y - robot_pose.y, target_pose.x - robot_pose.x);
  const double heading_error =
      normalize_angle(target_heading - robot_pose.yaw);

  double linear_speed =
      std::min(config.max_linear_speed, config.linear_kp * goal_distance);
  if (goal_distance < config.slow_down_distance) {
    linear_speed *= goal_distance / std::max(0.05, config.slow_down_distance);
  }
  linear_speed *= std::max(0.0, std::cos(heading_error));
  if (std::abs(heading_error) > config.rotate_in_place_threshold) {
    linear_speed = 0.0;
  }

  const double angular_speed =
      clamp_abs(config.angular_kp * heading_error, config.max_angular_speed);

  result.cmd_vel.linear.x = std::max(0.0, linear_speed);
  result.cmd_vel.angular.z = angular_speed;
  return result;
}

}  // namespace fuel_exploration::controller
