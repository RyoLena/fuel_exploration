#include "fuel_exploration/frontier/frontier_visualization.hpp"

#include "fuel_exploration/common/occupancy_grid_utils.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>

namespace {

using fuel_exploration::grid_to_world;

visualization_msgs::msg::Marker make_delete_all_marker(
    const std_msgs::msg::Header &header) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

}  // namespace

namespace fuel_exploration::frontier {

visualization_msgs::msg::MarkerArray build_frontier_marker_array(
    const OccupancyGrid &map, const std::set<int> &frontier_set,
    const std::vector<FrontierCluster> &clusters, const int max_viewpoints,
    const std::optional<SelectedGoal> &selected_goal,
    const nav_msgs::msg::Path &planned_path,
    const nav_msgs::msg::Path &trajectory_path,
    const bool show_selected_path) {
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(make_delete_all_marker(map.header));

  const int width = static_cast<int>(map.info.width);

  if (!frontier_set.empty()) {
    visualization_msgs::msg::Marker frontier_cells_marker;
    frontier_cells_marker.header = map.header;
    frontier_cells_marker.ns = "frontier_cells";
    frontier_cells_marker.id = 0;
    frontier_cells_marker.type = visualization_msgs::msg::Marker::POINTS;
    frontier_cells_marker.action = visualization_msgs::msg::Marker::ADD;
    frontier_cells_marker.pose.orientation.w = 1.0;
    frontier_cells_marker.scale.x = std::max(0.03, map.info.resolution * 0.75);
    frontier_cells_marker.scale.y = std::max(0.03, map.info.resolution * 0.75);
    frontier_cells_marker.color.r = 1.0F;
    frontier_cells_marker.color.g = 0.45F;
    frontier_cells_marker.color.b = 0.15F;
    frontier_cells_marker.color.a = 0.85F;

    for (const int idx : frontier_set) {
      const int frontier_row = idx / width;
      const int frontier_col = idx % width;
      const auto world_point = grid_to_world(frontier_row, frontier_col, map);

      geometry_msgs::msg::Point point;
      point.x = world_point.first;
      point.y = world_point.second;
      point.z = 0.04;
      frontier_cells_marker.points.push_back(point);
    }

    marker_array.markers.push_back(frontier_cells_marker);
  }

  if (!trajectory_path.poses.empty()) {
    visualization_msgs::msg::Marker trajectory_marker;
    trajectory_marker.header = trajectory_path.header;
    trajectory_marker.ns = "exploration_trajectory";
    trajectory_marker.id = 0;
    trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker.pose.orientation.w = 1.0;
    trajectory_marker.scale.x = 0.08;
    trajectory_marker.color.r = 0.1F;
    trajectory_marker.color.g = 0.95F;
    trajectory_marker.color.b = 0.95F;
    trajectory_marker.color.a = 0.95F;

    for (const auto &pose : trajectory_path.poses) {
      geometry_msgs::msg::Point point;
      point.x = pose.pose.position.x;
      point.y = pose.pose.position.y;
      point.z = 0.08;
      trajectory_marker.points.push_back(point);
    }

    marker_array.markers.push_back(trajectory_marker);
  }

  const int viewpoint_id_stride = std::max(1, max_viewpoints);
  for (std::size_t cluster_idx = 0; cluster_idx < clusters.size(); ++cluster_idx) {
    const auto &cluster = clusters[cluster_idx];

    visualization_msgs::msg::Marker centroid_marker;
    centroid_marker.header = map.header;
    centroid_marker.ns = "centroids";
    centroid_marker.id = static_cast<int>(cluster_idx);
    centroid_marker.type = visualization_msgs::msg::Marker::SPHERE;
    centroid_marker.action = visualization_msgs::msg::Marker::ADD;
    centroid_marker.pose.orientation.w = 1.0;
    centroid_marker.pose.position.x = cluster.centroid_x;
    centroid_marker.pose.position.y = cluster.centroid_y;
    centroid_marker.pose.position.z = 0.1;
    centroid_marker.scale.x = 0.30;
    centroid_marker.scale.y = 0.30;
    centroid_marker.scale.z = 0.30;
    centroid_marker.color.r = 1.0F;
    centroid_marker.color.a = 1.0F;
    marker_array.markers.push_back(centroid_marker);

    for (std::size_t viewpoint_idx = 0; viewpoint_idx < cluster.viewpoints.size();
         ++viewpoint_idx) {
      const auto &viewpoint = cluster.viewpoints[viewpoint_idx];
      const bool is_best_viewpoint = viewpoint_idx == 0;
      const int marker_id = static_cast<int>(cluster_idx) * viewpoint_id_stride +
                            static_cast<int>(viewpoint_idx);

      visualization_msgs::msg::Marker viewpoint_marker;
      viewpoint_marker.header = map.header;
      viewpoint_marker.ns =
          is_best_viewpoint ? "best_viewpoints" : "candidate_viewpoints";
      viewpoint_marker.id = marker_id;
      viewpoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
      viewpoint_marker.action = visualization_msgs::msg::Marker::ADD;
      viewpoint_marker.pose.orientation.w = 1.0;
      viewpoint_marker.pose.position.x = viewpoint.x;
      viewpoint_marker.pose.position.y = viewpoint.y;
      viewpoint_marker.pose.position.z = 0.12;
      viewpoint_marker.scale.x = is_best_viewpoint ? 0.25 : 0.18;
      viewpoint_marker.scale.y = is_best_viewpoint ? 0.25 : 0.18;
      viewpoint_marker.scale.z = is_best_viewpoint ? 0.25 : 0.18;
      viewpoint_marker.color.r = is_best_viewpoint ? 0.1F : 0.2F;
      viewpoint_marker.color.g = is_best_viewpoint ? 0.9F : 0.6F;
      viewpoint_marker.color.b = is_best_viewpoint ? 0.2F : 1.0F;
      viewpoint_marker.color.a = is_best_viewpoint ? 1.0F : 0.65F;
      marker_array.markers.push_back(viewpoint_marker);

      if (!is_best_viewpoint) {
        continue;
      }

      visualization_msgs::msg::Marker heading_marker;
      heading_marker.header = map.header;
      heading_marker.ns = "best_viewpoint_heading";
      heading_marker.id = static_cast<int>(cluster_idx);
      heading_marker.type = visualization_msgs::msg::Marker::ARROW;
      heading_marker.action = visualization_msgs::msg::Marker::ADD;
      heading_marker.pose.position.x = viewpoint.x;
      heading_marker.pose.position.y = viewpoint.y;
      heading_marker.pose.position.z = 0.15;
      heading_marker.pose.orientation.z = std::sin(viewpoint.yaw / 2.0);
      heading_marker.pose.orientation.w = std::cos(viewpoint.yaw / 2.0);
      heading_marker.scale.x = 0.35;
      heading_marker.scale.y = 0.08;
      heading_marker.scale.z = 0.08;
      heading_marker.color.g = 0.9F;
      heading_marker.color.b = 0.1F;
      heading_marker.color.a = 1.0F;
      marker_array.markers.push_back(heading_marker);

      visualization_msgs::msg::Marker score_marker;
      score_marker.header = map.header;
      score_marker.ns = "viewpoint_scores";
      score_marker.id = static_cast<int>(cluster_idx);
      score_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      score_marker.action = visualization_msgs::msg::Marker::ADD;
      score_marker.pose.orientation.w = 1.0;
      score_marker.pose.position.x = viewpoint.x;
      score_marker.pose.position.y = viewpoint.y;
      score_marker.pose.position.z = 0.45;
      score_marker.scale.z = 0.18;
      score_marker.color.r = 1.0F;
      score_marker.color.g = 1.0F;
      score_marker.color.b = 1.0F;
      score_marker.color.a = 1.0F;
      score_marker.text = "score: " + std::to_string(viewpoint.score);
      marker_array.markers.push_back(score_marker);
    }
  }

  if (selected_goal.has_value()) {
    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header = map.header;
    goal_marker.ns = "selected_goal";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.pose.position.x = selected_goal->viewpoint.x;
    goal_marker.pose.position.y = selected_goal->viewpoint.y;
    goal_marker.pose.position.z = 0.05;
    goal_marker.scale.x = 0.40;
    goal_marker.scale.y = 0.40;
    goal_marker.scale.z = 0.10;
    goal_marker.color.r = 1.0F;
    goal_marker.color.g = 0.9F;
    goal_marker.color.a = 0.85F;
    marker_array.markers.push_back(goal_marker);

    visualization_msgs::msg::Marker goal_label;
    goal_label.header = map.header;
    goal_label.ns = "selected_goal_label";
    goal_label.id = 0;
    goal_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    goal_label.action = visualization_msgs::msg::Marker::ADD;
    goal_label.pose.orientation.w = 1.0;
    goal_label.pose.position.x = selected_goal->viewpoint.x;
    goal_label.pose.position.y = selected_goal->viewpoint.y;
    goal_label.pose.position.z = 0.70;
    goal_label.scale.z = 0.22;
    goal_label.color.r = 1.0F;
    goal_label.color.g = 0.95F;
    goal_label.color.b = 0.2F;
    goal_label.color.a = 1.0F;
    goal_label.text =
        "goal c" + std::to_string(selected_goal->cluster_index) + " v" +
        std::to_string(selected_goal->viewpoint_index) + " | u:" +
        std::to_string(selected_goal->utility).substr(0, 4);
    marker_array.markers.push_back(goal_label);

    if (show_selected_path) {
      visualization_msgs::msg::Marker path_marker;
      path_marker.header = map.header;
      path_marker.ns = "selected_goal_path";
      path_marker.id = 0;
      path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path_marker.action = visualization_msgs::msg::Marker::ADD;
      path_marker.pose.orientation.w = 1.0;
      for (const auto &pose : planned_path.poses) {
        geometry_msgs::msg::Point point;
        point.x = pose.pose.position.x;
        point.y = pose.pose.position.y;
        point.z = 0.18;
        path_marker.points.push_back(point);
      }
      path_marker.scale.x = 0.06;
      path_marker.scale.y = 0.0;
      path_marker.scale.z = 0.0;
      path_marker.color.r = 1.0F;
      path_marker.color.g = 0.8F;
      path_marker.color.b = 0.1F;
      path_marker.color.a = 0.95F;
      marker_array.markers.push_back(path_marker);
    }
  }

  return marker_array;
}

}  // namespace fuel_exploration::frontier
