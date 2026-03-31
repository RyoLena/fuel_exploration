#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace fuel_exploration::frontier {

struct FrontierDetectorConfig {
  int min_cluster_size{10};
  std::vector<double> sample_radii_m{0.35, 0.55, 0.8, 1.1, 1.5};
  int angular_samples{16};
  double max_view_range_m{3.5};
  double candidate_clearance_m{0.1};
  int min_viewpoint_coverage{1};
  int max_viewpoints{15};
  int occupied_threshold{50};
  bool block_unknown_in_los{true};
  std::string odom_topic{"/odom"};
  std::string goal_topic{"/next_exploration_goal"};
  std::string path_topic{"/next_exploration_path"};
  std::string trajectory_topic{"/exploration_trajectory"};
  double goal_distance_weight{1.0};
  double path_clearance_m{0.15};
  bool block_unknown_in_path{true};
  double robot_pose_timeout_s{0.1};
  double min_goal_distance_m{0.35};
  double viewpoint_rank_penalty{0.05};
  double goal_reached_tolerance_m{0.25};
  double revisit_block_radius_m{0.45};
  double trajectory_record_distance_m{0.05};
  int tsp_lookahead{4};
};

inline FrontierDetectorConfig declare_frontier_detector_config(
    rclcpp::Node &node) {
  FrontierDetectorConfig config;
  config.min_cluster_size = node.declare_parameter<int>("min_cluster_size", 10);
  config.sample_radii_m = node.declare_parameter<std::vector<double>>(
      "sample_radii_m", {0.35, 0.55, 0.8, 1.1, 1.5});
  config.angular_samples = std::max<int>(
      1, node.declare_parameter<int>("angular_samples", 16));
  config.max_view_range_m =
      node.declare_parameter<double>("max_view_range_m", 3.5);
  config.candidate_clearance_m =
      node.declare_parameter<double>("candidate_clearance_m", 0.1);
  config.min_viewpoint_coverage = std::max<int>(
      1, node.declare_parameter<int>("min_viewpoint_coverage", 1));
  config.max_viewpoints = std::max<int>(
      1, node.declare_parameter<int>("max_viewpoints", 15));
  config.occupied_threshold =
      node.declare_parameter<int>("occupied_threshold", 50);
  config.block_unknown_in_los =
      node.declare_parameter<bool>("block_unknown_in_los", true);
  config.odom_topic =
      node.declare_parameter<std::string>("odom_topic", "/odom");
  config.goal_topic = node.declare_parameter<std::string>(
      "goal_topic", "/next_exploration_goal");
  config.path_topic = node.declare_parameter<std::string>(
      "path_topic", "/next_exploration_path");
  config.trajectory_topic = node.declare_parameter<std::string>(
      "trajectory_topic", "/exploration_trajectory");
  config.goal_distance_weight =
      node.declare_parameter<double>("goal_distance_weight", 1.0);
  config.path_clearance_m =
      node.declare_parameter<double>("path_clearance_m", 0.15);
  config.block_unknown_in_path =
      node.declare_parameter<bool>("block_unknown_in_path", true);
  config.robot_pose_timeout_s =
      node.declare_parameter<double>("robot_pose_timeout_s", 0.1);
  config.min_goal_distance_m =
      node.declare_parameter<double>("min_goal_distance_m", 0.35);
  config.viewpoint_rank_penalty =
      node.declare_parameter<double>("viewpoint_rank_penalty", 0.05);
  config.goal_reached_tolerance_m =
      node.declare_parameter<double>("goal_reached_tolerance_m", 0.25);
  config.revisit_block_radius_m =
      node.declare_parameter<double>("revisit_block_radius_m", 0.45);
  config.trajectory_record_distance_m = std::max(
      0.0, node.declare_parameter<double>("trajectory_record_distance_m", 0.05));
  config.tsp_lookahead = node.declare_parameter<int>("tsp_lookahead", 4);
  return config;
}

}  // namespace fuel_exploration::frontier
