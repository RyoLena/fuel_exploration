#pragma once

#include "fuel_exploration/common/navigation_utils.hpp"
#include "fuel_exploration/frontier/frontier_types.hpp"

#include <cmath>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

namespace fuel_exploration::frontier {

struct FrontierDetectorState {
  std::set<int> frontier_set;
  std::vector<int8_t> prev_map_data;
  std::optional<RobotPose> robot_pose;
  std::optional<ViewpointCandidate> active_goal;
  std::vector<VisitedViewpoint> visited_viewpoints;
  nav_msgs::msg::Path trajectory_path;
  bool warned_missing_pose{false};
  mutable bool warned_transform_failure{false};
  std::string last_status;
};

inline bool mark_goal_reached_if_needed(
    FrontierDetectorState &state, const RobotPose &planning_pose,
    const double goal_reached_tolerance_m,
    const double revisit_block_radius_m, const rclcpp::Logger &logger) {
  if (!state.active_goal.has_value()) {
    return false;
  }

  if (std::hypot(planning_pose.x - state.active_goal->x,
                 planning_pose.y - state.active_goal->y) >
      goal_reached_tolerance_m) {
    return false;
  }

  state.visited_viewpoints.push_back(
      VisitedViewpoint{state.active_goal->x, state.active_goal->y});
  RCLCPP_INFO(logger,
              "Reached goal viewpoint, blocking revisits within %.2f m. "
              "Visited viewpoints: %zu",
              revisit_block_radius_m, state.visited_viewpoints.size());
  state.active_goal.reset();
  return true;
}

inline void append_trajectory_pose(
    FrontierDetectorState &state, const std_msgs::msg::Header &header,
    const RobotPose &robot_pose, const double trajectory_record_distance_m,
    const rclcpp::Logger &logger) {
  if (!state.trajectory_path.poses.empty() &&
      state.trajectory_path.header.frame_id != header.frame_id) {
    RCLCPP_WARN(logger,
                "Trajectory frame changed from '%s' to '%s'; clearing accumulated "
                "trajectory.",
                state.trajectory_path.header.frame_id.c_str(),
                header.frame_id.c_str());
    state.trajectory_path.poses.clear();
  }

  state.trajectory_path.header = header;

  geometry_msgs::msg::PoseStamped trajectory_pose;
  trajectory_pose.header = header;
  trajectory_pose.pose.position.x = robot_pose.x;
  trajectory_pose.pose.position.y = robot_pose.y;
  trajectory_pose.pose.position.z = 0.0;
  trajectory_pose.pose.orientation = yaw_to_quaternion(robot_pose.yaw);

  if (state.trajectory_path.poses.empty()) {
    state.trajectory_path.poses.push_back(trajectory_pose);
    return;
  }

  const auto &last_position = state.trajectory_path.poses.back().pose.position;
  const double distance_since_last_sample =
      std::hypot(trajectory_pose.pose.position.x - last_position.x,
                 trajectory_pose.pose.position.y - last_position.y);
  if (distance_since_last_sample >= trajectory_record_distance_m) {
    state.trajectory_path.poses.push_back(trajectory_pose);
  } else {
    state.trajectory_path.poses.back().header = header;
    state.trajectory_path.poses.back().pose.orientation =
        trajectory_pose.pose.orientation;
  }
}

}  // namespace fuel_exploration::frontier
