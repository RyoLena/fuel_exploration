#pragma once

#include "fuel_exploration/common/navigation_utils.hpp"
#include "fuel_exploration/common/occupancy_grid_utils.hpp"
#include "fuel_exploration/frontier/frontier_types.hpp"

#include <vector>

namespace fuel_exploration::frontier {

GoalSelectionResult select_next_goal(
    const RobotPose &planning_pose,
    const std::vector<FrontierCluster> &clusters,
    const OccupancyGrid &map,
    int occupied_threshold,
    bool block_unknown_in_path,
    double path_clearance_m,
    const std::vector<VisitedViewpoint> &visited_viewpoints,
    double revisit_block_radius_m,
    int tsp_lookahead,
    double min_goal_distance_m,
    double goal_distance_weight,
    double viewpoint_rank_penalty);

}  // namespace fuel_exploration::frontier
