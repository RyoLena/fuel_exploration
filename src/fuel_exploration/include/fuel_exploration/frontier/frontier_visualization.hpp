#pragma once

#include "fuel_exploration/common/occupancy_grid_utils.hpp"
#include "fuel_exploration/frontier/frontier_types.hpp"

#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <set>
#include <visualization_msgs/msg/marker_array.hpp>

namespace fuel_exploration::frontier {

visualization_msgs::msg::MarkerArray build_frontier_marker_array(
    const OccupancyGrid &map, const std::set<int> &frontier_set,
    const std::vector<FrontierCluster> &clusters, int max_viewpoints,
    const std::optional<SelectedGoal> &selected_goal,
    const nav_msgs::msg::Path &planned_path,
    const nav_msgs::msg::Path &trajectory_path, bool show_selected_path);

}  // namespace fuel_exploration::frontier
