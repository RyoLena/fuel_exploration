#pragma once

#include "fuel_exploration/common/occupancy_grid_utils.hpp"
#include "fuel_exploration/frontier/frontier_types.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>

#include <vector>

namespace fuel_exploration::frontier {

geometry_msgs::msg::PoseStamped build_goal_pose_message(
    const std_msgs::msg::Header &header,
    const SelectedGoal &selected_goal);

nav_msgs::msg::Path build_planned_path_message(
    const std_msgs::msg::Header &header,
    const OccupancyGrid &map,
    const std::vector<int> &path_indices);

}  // namespace fuel_exploration::frontier
