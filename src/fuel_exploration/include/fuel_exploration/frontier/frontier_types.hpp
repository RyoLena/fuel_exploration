#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace fuel_exploration::frontier {

struct ViewpointCandidate {
  double x;
  double y;
  double yaw;
  int score;
};

struct FrontierCluster {
  std::vector<int> cells;
  double centroid_x;
  double centroid_y;
  std::vector<ViewpointCandidate> viewpoints;
};

struct SelectedGoal {
  ViewpointCandidate viewpoint;
  std::size_t cluster_index;
  std::size_t viewpoint_index;
  double utility;
  double distance;
  std::vector<int> path_indices;
};

struct PlannedPath {
  std::vector<int> indices;
  double length_m;
};

struct VisitedViewpoint {
  double x;
  double y;
};

struct DijkstraResult {
  std::vector<std::size_t> selected_viewpoint_indices;
  double total_cost;
};

struct ClusterStatistics {
  std::size_t clusters_without_viewpoint{0};
  std::size_t clusters_with_viewpoint{0};
  std::size_t total_viewpoints{0};
};

struct FrontierSetUpdateResult {
  bool full_rebuild{false};
  bool map_resized{false};
  std::size_t checked_cell_count{0};
};

struct GoalSelectionResult {
  std::optional<SelectedGoal> selected_goal;
  std::string tsp_order_description;
  bool revisit_filter_relaxed{false};
};

}  // namespace fuel_exploration::frontier
