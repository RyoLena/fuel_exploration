#pragma once

#include "fuel_exploration/controller/path_tracking.hpp"

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace fuel_exploration::controller {

struct ExplorationControllerConfig {
  std::string odom_topic{"/odom"};
  std::string path_topic{"/next_exploration_path"};
  std::string cmd_vel_topic{"/cmd_vel"};
  double control_rate_hz{10.0};
  bool path_frame_must_match{false};
  double transform_timeout_s{0.1};
  PathTrackingConfig tracking;
};

inline ExplorationControllerConfig declare_exploration_controller_config(
    rclcpp::Node &node) {
  ExplorationControllerConfig config;
  config.odom_topic =
      node.declare_parameter<std::string>("odom_topic", "/odom");
  config.path_topic = node.declare_parameter<std::string>(
      "path_topic", "/next_exploration_path");
  config.cmd_vel_topic =
      node.declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  config.control_rate_hz =
      node.declare_parameter<double>("control_rate_hz", 10.0);
  config.tracking.lookahead_distance =
      node.declare_parameter<double>("lookahead_distance", 0.35);
  config.tracking.waypoint_tolerance =
      node.declare_parameter<double>("waypoint_tolerance", 0.15);
  config.tracking.goal_tolerance =
      node.declare_parameter<double>("goal_tolerance", 0.20);
  config.tracking.slow_down_distance =
      node.declare_parameter<double>("slow_down_distance", 0.60);
  config.tracking.max_linear_speed =
      node.declare_parameter<double>("max_linear_speed", 0.22);
  config.tracking.max_angular_speed =
      node.declare_parameter<double>("max_angular_speed", 1.20);
  config.tracking.linear_kp =
      node.declare_parameter<double>("linear_kp", 0.80);
  config.tracking.angular_kp =
      node.declare_parameter<double>("angular_kp", 1.80);
  config.tracking.rotate_in_place_threshold =
      node.declare_parameter<double>("rotate_in_place_threshold", 0.55);
  config.path_frame_must_match =
      node.declare_parameter<bool>("path_frame_must_match", false);
  config.transform_timeout_s =
      node.declare_parameter<double>("transform_timeout_s", 0.1);
  return config;
}

}  // namespace fuel_exploration::controller
