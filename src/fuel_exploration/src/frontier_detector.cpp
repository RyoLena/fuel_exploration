#include "fuel_exploration/common/navigation_utils.hpp"
#include "fuel_exploration/common/occupancy_grid_utils.hpp"
#include "fuel_exploration/common/robot_pose_transform.hpp"
#include "fuel_exploration/frontier/frontier_algorithms.hpp"
#include "fuel_exploration/frontier/frontier_detector_config.hpp"
#include "fuel_exploration/frontier/frontier_detector_state.hpp"
#include "fuel_exploration/frontier/frontier_goal_selection.hpp"
#include "fuel_exploration/frontier/frontier_map_processing.hpp"
#include "fuel_exploration/frontier/frontier_path_messages.hpp"
#include "fuel_exploration/frontier/frontier_types.hpp"
#include "fuel_exploration/frontier/frontier_visualization.hpp"

#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace {

using OccupancyGrid = fuel_exploration::OccupancyGrid;
using fuel_exploration::quaternion_to_yaw;
using fuel_exploration::RobotPose;
using fuel_exploration::transform_robot_pose;
using fuel_exploration::frontier::append_trajectory_pose;
using fuel_exploration::frontier::build_frontier_clusters;
using fuel_exploration::frontier::build_frontier_marker_array;
using fuel_exploration::frontier::build_goal_pose_message;
using fuel_exploration::frontier::build_planned_path_message;
using fuel_exploration::frontier::ClusterStatistics;
using fuel_exploration::frontier::declare_frontier_detector_config;
using fuel_exploration::frontier::FrontierDetectorConfig;
using fuel_exploration::frontier::FrontierDetectorState;
using fuel_exploration::frontier::FrontierCluster;
using fuel_exploration::frontier::mark_goal_reached_if_needed;
using fuel_exploration::frontier::select_next_goal;
using fuel_exploration::frontier::SelectedGoal;
using fuel_exploration::frontier::summarize_clusters;
using fuel_exploration::frontier::update_frontier_set;
using fuel_exploration::frontier::ViewpointCandidate;
using fuel_exploration::frontier::VisitedViewpoint;

} // namespace

class FrontierDetector : public rclcpp::Node {
public:
  FrontierDetector() : Node("frontier_detector") {
    config_ = declare_frontier_detector_config(*this);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10,
        std::bind(&FrontierDetector::map_callback, this,
                  std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        config_.odom_topic, 10,
        std::bind(&FrontierDetector::odom_callback, this, std::placeholders::_1));
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        config_.goal_topic, 10);
    path_pub_ =
        this->create_publisher<nav_msgs::msg::Path>(config_.path_topic, 10);
    trajectory_pub_ =
        this->create_publisher<nav_msgs::msg::Path>(config_.trajectory_topic, 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/exploration_status", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "frontiers", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void publish_status(const std::string &status,
                      const std::string &detail = "");
  std::optional<RobotPose> robot_pose_in_frame(
      const std::string &target_frame) const;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;

  FrontierDetectorConfig config_;
  FrontierDetectorState state_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

void FrontierDetector::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  state_.robot_pose = RobotPose{msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                quaternion_to_yaw(msg->pose.pose.orientation),
                                msg->header.frame_id};
}

void FrontierDetector::publish_status(const std::string &status,
                                      const std::string &detail) {
  std_msgs::msg::String status_msg;
  status_msg.data = detail.empty() ? status : status + " | " + detail;
  status_pub_->publish(status_msg);

  if (status != state_.last_status) {
    RCLCPP_INFO(this->get_logger(), "Exploration status -> %s",
                status_msg.data.c_str());
    state_.last_status = status;
  }
}

std::optional<RobotPose> FrontierDetector::robot_pose_in_frame(
    const std::string &target_frame) const {
  return transform_robot_pose(state_.robot_pose, target_frame, *tf_buffer_,
                              config_.robot_pose_timeout_s,
                              this->get_logger(),
                              state_.warned_transform_failure);
}

void FrontierDetector::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received map: %u x %u", msg->info.width,
              msg->info.height);

  const auto frontier_update =
      update_frontier_set(*msg, state_.prev_map_data, state_.frontier_set);
  if (frontier_update.full_rebuild) {
    if (frontier_update.map_resized) {
      RCLCPP_INFO(this->get_logger(),
                  "Map size changed; rebuilt frontier set from scratch. "
                  "Frontier cells: %zu",
                  state_.frontier_set.size());
    } else {
      RCLCPP_INFO(this->get_logger(), "Initial frontier cells: %zu",
                  state_.frontier_set.size());
    }
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Incremental update: %zu cells checked, %zu frontiers",
                frontier_update.checked_cell_count, state_.frontier_set.size());
  }

  state_.prev_map_data = msg->data;

  std::vector<FrontierCluster> clusters = build_frontier_clusters(
      state_.frontier_set, *msg, config_.min_cluster_size,
      config_.sample_radii_m, config_.angular_samples,
      config_.candidate_clearance_m, config_.max_view_range_m,
      config_.min_viewpoint_coverage, config_.max_viewpoints,
      config_.occupied_threshold, config_.block_unknown_in_los);
  const ClusterStatistics cluster_stats = summarize_clusters(clusters);

  RCLCPP_INFO(this->get_logger(),
              "Retained %zu frontier clusters, %zu without valid viewpoints, "
              "%zu viewpoints total",
              clusters.size(), cluster_stats.clusters_without_viewpoint,
              cluster_stats.total_viewpoints);

  std::optional<SelectedGoal> selected_goal;
  nav_msgs::msg::Path planned_path_msg;
  planned_path_msg.header = msg->header;
  const auto planning_pose = robot_pose_in_frame(msg->header.frame_id);
  if (planning_pose.has_value()) {
    append_trajectory_pose(state_, msg->header, *planning_pose,
                           config_.trajectory_record_distance_m,
                           this->get_logger());
  } else if (!state_.trajectory_path.poses.empty()) {
    state_.trajectory_path.header.stamp = msg->header.stamp;
  }
  std::string exploration_status = "idle";
  std::string status_detail = "clusters=" + std::to_string(clusters.size()) +
                              ", viewpoints=" +
                              std::to_string(cluster_stats.total_viewpoints) +
                              ", visited=" +
                              std::to_string(state_.visited_viewpoints.size());
  if (planning_pose.has_value()) {
    mark_goal_reached_if_needed(state_, *planning_pose,
                                config_.goal_reached_tolerance_m,
                                config_.revisit_block_radius_m,
                                this->get_logger());

    const auto goal_selection = select_next_goal(
        *planning_pose, clusters, *msg, config_.occupied_threshold,
        config_.block_unknown_in_path, config_.path_clearance_m,
        state_.visited_viewpoints, config_.revisit_block_radius_m,
        config_.tsp_lookahead, config_.min_goal_distance_m,
        config_.goal_distance_weight, config_.viewpoint_rank_penalty);
    if (goal_selection.revisit_filter_relaxed) {
      RCLCPP_INFO(this->get_logger(),
                  "All candidate viewpoints were blocked by the %.2f m revisit "
                  "radius; retrying goal selection with revisit blocking relaxed.",
                  config_.revisit_block_radius_m);
    }
    RCLCPP_INFO(this->get_logger(), "TSP order: %s",
                goal_selection.tsp_order_description.c_str());
    selected_goal = goal_selection.selected_goal;

    if (selected_goal.has_value()) {
      exploration_status = "exploring";
      status_detail +=
          ", goal_cluster=" + std::to_string(selected_goal->cluster_index) +
          ", goal_viewpoint=" + std::to_string(selected_goal->viewpoint_index);
      state_.active_goal = selected_goal->viewpoint;
      goal_pub_->publish(build_goal_pose_message(msg->header, *selected_goal));
      planned_path_msg =
          build_planned_path_message(msg->header, *msg, selected_goal->path_indices);
    } else if (cluster_stats.clusters_with_viewpoint > 0U) {
      exploration_status = "stalled";
      RCLCPP_INFO(this->get_logger(),
                  "No goal selected beyond %.2f m; remaining candidate "
                  "viewpoints are either already visited or unreachable.",
                  config_.min_goal_distance_m);
    }
  } else if (!state_.robot_pose.has_value() && !state_.warned_missing_pose) {
    exploration_status = "waiting_for_pose";
    RCLCPP_WARN(this->get_logger(),
                "No robot pose received on %s yet, so next-goal selection is "
                "disabled for now.",
                config_.odom_topic.c_str());
    state_.warned_missing_pose = true;
  } else if (state_.robot_pose.has_value()) {
    exploration_status = "waiting_for_tf";
  }
  path_pub_->publish(planned_path_msg);
  trajectory_pub_->publish(state_.trajectory_path);

  if (state_.frontier_set.empty()) {
    exploration_status = "finished";
    status_detail = "frontiers=0";
  } else if (clusters.empty() && cluster_stats.total_viewpoints == 0U &&
             exploration_status != "waiting_for_pose" &&
             exploration_status != "waiting_for_tf") {
    exploration_status = "no_clusters";
    status_detail = "frontiers=" + std::to_string(state_.frontier_set.size()) +
                    ", below_cluster_threshold";
  } else if (cluster_stats.clusters_with_viewpoint == 0U && !clusters.empty() &&
             exploration_status != "waiting_for_pose" &&
             exploration_status != "waiting_for_tf") {
    exploration_status = "no_viewpoints";
  }

  publish_status(exploration_status, status_detail);

  marker_pub_->publish(build_frontier_marker_array(
      *msg, state_.frontier_set, clusters, config_.max_viewpoints,
      selected_goal, planned_path_msg, state_.trajectory_path,
      state_.robot_pose.has_value()));
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
