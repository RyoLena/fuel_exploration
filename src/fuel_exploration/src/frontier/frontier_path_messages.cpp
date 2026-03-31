#include "fuel_exploration/frontier/frontier_path_messages.hpp"

#include "fuel_exploration/common/navigation_utils.hpp"
#include "fuel_exploration/common/occupancy_grid_utils.hpp"

namespace fuel_exploration::frontier {

geometry_msgs::msg::PoseStamped build_goal_pose_message(
    const std_msgs::msg::Header &header,
    const SelectedGoal &selected_goal) {
  geometry_msgs::msg::PoseStamped goal_msg;
  goal_msg.header = header;
  goal_msg.pose.position.x = selected_goal.viewpoint.x;
  goal_msg.pose.position.y = selected_goal.viewpoint.y;
  goal_msg.pose.position.z = 0.0;
  goal_msg.pose.orientation = yaw_to_quaternion(selected_goal.viewpoint.yaw);
  return goal_msg;
}

nav_msgs::msg::Path build_planned_path_message(
    const std_msgs::msg::Header &header,
    const OccupancyGrid &map,
    const std::vector<int> &path_indices) {
  nav_msgs::msg::Path planned_path_msg;
  planned_path_msg.header = header;

  const int width = static_cast<int>(map.info.width);
  for (const int path_index : path_indices) {
    const int path_row = path_index / width;
    const int path_col = path_index % width;
    const auto world_point = grid_to_world(path_row, path_col, map);

    geometry_msgs::msg::PoseStamped path_pose;
    path_pose.header = header;
    path_pose.pose.position.x = world_point.first;
    path_pose.pose.position.y = world_point.second;
    path_pose.pose.position.z = 0.0;
    path_pose.pose.orientation.w = 1.0;
    planned_path_msg.poses.push_back(path_pose);
  }

  return planned_path_msg;
}

}  // namespace fuel_exploration::frontier
